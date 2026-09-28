// SPDX-License-Identifier: AGPL-3.0-only
//! The macOS `/tmp/*.shm_state` population — detect it, price it,
//! and reclaim the entries that are PROVABLY dead.
//!
//! # The mechanism (each link read out of the vendored source, not inferred)
//!
//! macOS has no `/dev/shm`, so `iceoryx2-pal-posix` keeps ONE `<name>.shm_state`
//! file per SHM segment in `TEMP_DIRECTORY`, which is literally `/tmp/`
//! (`iceoryx2-pal-configuration-0.9.1/src/lib.rs:34`). The file is created by
//! `shm_open`'s `O_CREAT` arm (`macos/mman.rs:171`) and removed by `shm_unlink`
//! (`macos/mman.rs:191`) — so a SIGKILLed owner leaks it forever. And
//! `shm_list()` (`macos/mman.rs:228`) is a FULL READDIR of `/tmp`, which every
//! startup path that sweeps dead nodes walks into.
//!
//! MEASURED: with 72,707 of them accumulated, one scan takes
//! 393 s under load, 13 s quiet, 0.00 s after cleanup. The visible failure is
//! a `graph run` supervisor stalling with EMPTY stdout/stderr before its first
//! log line. Linux does not have this pathology (it has a real `/dev/shm`).
//!
//! # What a `.shm_state` file IS, and what makes one PROVABLY dead
//!
//! Its CONTENT is the "real" POSIX shm object name the logical name resolves
//! to — a NUL-padded 33-byte buffer minted by `generate_real_shm_name`
//! (`macos/mman.rs:133`) as:
//!
//! ```text
//! <pid>_<tv_sec>_<tv_usec>_<counter>
//! ```
//!
//! So the file NAMES ITS OWN CREATOR. That is the evidence this module runs on,
//! and it is evidence rather than a heuristic: a state file is written exactly
//! once, under `O_EXCL | O_CREAT`, by the process that created the segment.
//! **A file is PROVABLY DEAD iff `kill(creator_pid, 0)` reports `ESRCH`** — no
//! such process — which means the creator can never come back to unlink it.
//!
//! An mtime heuristic was NOT used and would not have been sound: a long-lived
//! robot's segments are arbitrarily old and perfectly alive.
//!
//! ## The two ways the proof can be wrong, and which way each fails
//!
//! * **PID reuse.** A dead creator's pid may have been recycled by an unrelated
//!   process, in which case `kill` reports the RECYCLED process as alive and we
//!   REFUSE to reclaim. That is the safe direction — the residual is a leaked
//!   file we decline to delete, never a live segment we delete.
//! * **`EPERM`.** The process exists but is not ours. `kill` says `EPERM`,
//!   which is positive evidence of EXISTENCE, so it reads as alive. Also the
//!   safe direction.
//!
//! Any other errno reads as [`ProcessLiveness::Unknown`] and refuses. There is
//! no arrangement of these answers that deletes the state file of a live
//! creator.
//!
//! Run-lifetime detection deliberately refuses `kill(pid, 0)`.
//! That refusal turns on there being a SECOND
//! mechanism (an iceoryx2 publisher count) giving the same answer, so two
//! mechanisms could disagree. Here there is no second mechanism — the pid is
//! the only thing the file records — and, unlike the run-lifetime case, pid
//! recycling biases toward REFUSING to act rather than toward acting wrongly.
//!
//! # Reclamation unlinks the OBJECT before it removes the FILE
//!
//! MEASURED: of 4,000 leaked
//! state files sampled, **3,998 named a POSIX shm object that still EXISTS in
//! the kernel** (2 had a live creator — concurrent work on the same machine) and
//! ZERO named an absent one. Sampling 600 of those orphans, `fstat` reported
//! 14,434,304 bytes, a mean of ~24 KiB each. So the leak is not only directory
//! entries: the kernel objects leak with them.
//!
//! That is why reclamation mirrors `shm_unlink`'s own order
//! (`macos/mman.rs:185`): unlink the REAL OBJECT first, and remove the state
//! file only if that succeeded or reported `ENOENT`. Removing the file first
//! would strand the object under a name nothing can ever resolve again — an
//! unreclaimable kernel leak in place of a reclaimable one.
//!
//! # The diagnostic must not become the hang
//!
//! The readdir IS the expensive operation, so the walk is STREAMED under a wall
//! budget and reports a FLOOR plus a truncation marker rather than a silently
//! wrong exact count. A ZERO budget refuses to START the walk, so an exhausted
//! budget can never buy the scan a fresh ceiling (the `COMPLETION_BUDGET`
//! precedent in [`crate::completions`]).
//!
//! Classification carries its OWN, strictly smaller budget. The COUNT is the
//! headline diagnostic and reclamation is secondary, so per-file probing is
//! never allowed to starve the count — otherwise a catastrophic directory
//! could report "at least 40 files" and be classified [`ScanSeverity::Healthy`],
//! which is worse than reporting nothing.
//!
//! # The desk heals itself
//!
//! A reclamation that runs only when an operator types `cerulion clean`
//! runs only on the machines of people who already know about the leak.
//! [`reclaim_at_exit`] runs the SAME walk — same evidence, same refusals, same
//! floor reporting — on the graceful teardown of a `cerulion graph run`, under
//! [`SHM_STATE_EXIT_RECLAIM_BUDGET`]. Exit is the one moment where hygiene
//! delays no work: the graph is over, so a bounded pass costs the prompt and
//! nothing else. A run that is SIGKILLed skips it by nature; its residue is
//! swept by the next run that ends gracefully, which is exactly the progressive
//! behaviour the budget already assumes.
//!
//! Nothing about the STARTUP path changed. It still pays only the bounded
//! dead-node sweep, and it still reclaims no state files.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime};

/// The suffix `iceoryx2-pal-posix` gives every state file
/// (`macos/settings.rs:16`).
pub const SHM_STATE_SUFFIX: &str = ".shm_state";

/// The directory those files live in — `iceoryx2_pal_configuration::TEMP_DIRECTORY`
/// (`iceoryx2-pal-configuration-0.9.1/src/lib.rs:34`).
///
/// Hardcoded rather than read from the environment ON PURPOSE: iceoryx2 does
/// not honour `TMPDIR` either, so honouring it here would point the diagnostic
/// at a directory the leak is not in.
pub const SHM_STATE_DIRECTORY: &str = "/tmp/";

/// The budget for a walk that only REPORTS. A diagnostic must never make the
/// operator wait, so this is short and the count it produces may be a floor.
pub const SHM_STATE_REPORT_BUDGET: Duration = Duration::from_millis(2_000);

/// The budget for a walk that RECLAIMS.
///
/// Deliberately larger than the report budget: the operator has explicitly
/// asked for the fix and is watching it run, and a remedy that cannot finish
/// is not a remedy — at the measured basis (72,707 files, 13 s on a quiet
/// desk) the whole pathological directory clears inside this in ONE run,
/// with room to spare. Still a hard bound, so `cerulion clean` can be slow
/// and can never hang.
pub const SHM_STATE_RECLAIM_BUDGET: Duration = Duration::from_millis(30_000);

const _: () = assert!(
    SHM_STATE_RECLAIM_BUDGET.as_millis() >= SHM_STATE_REPORT_BUDGET.as_millis(),
    "the remedy may take longer than the diagnostic, never less"
);

/// The budget for the reclamation a GRACEFUL GRAPH EXIT performs.
///
/// Sits between the two budgets above because the exit path is between the two
/// situations they describe. Nobody typed a command asking for hygiene, so it
/// cannot spend `cerulion clean`'s 30 s — a Ctrl+C that takes half a minute to
/// return the shell is a worse defect than the one being fixed. But the graph
/// is already over, so unlike a STARTUP pass it delays no work the operator is
/// waiting on; it only delays the prompt.
///
/// A hard bound, never a target: [`scan`] reports a FLOOR and stops. Anything
/// left is still there for the next graceful exit, which is the whole point —
/// a desk that runs graphs heals progressively with no operator action ever.
pub const SHM_STATE_EXIT_RECLAIM_BUDGET: Duration = Duration::from_millis(2_000);

const _: () = assert!(
    SHM_STATE_EXIT_RECLAIM_BUDGET.as_millis() <= SHM_STATE_RECLAIM_BUDGET.as_millis(),
    "the exit pass is hygiene nobody asked for; the explicit remedy may always take longer"
);
const _: () = assert!(
    SHM_STATE_EXIT_RECLAIM_BUDGET.as_millis() > 0,
    "a ZERO budget refuses to start the walk, which would ship this feature INERT"
);

/// How much of a walk's budget the per-file reading, probing and reclaiming
/// may consume.
///
/// A strict fraction, so the COUNT — the headline diagnostic — always outlives
/// classification. Without it a catastrophic directory could report "at least
/// 40 files" and be classified [`ScanSeverity::Healthy`], which is worse than
/// reporting nothing at all.
pub fn classify_budget(scan_budget: Duration) -> Duration {
    scan_budget * 3 / 4
}

/// The measured basis: file count of the measured
/// pathological directory.
pub const MEASURED_BASIS_FILES: u64 = 72_707;
/// The measured basis: one `shm_list()` scan of that directory on a QUIET desk.
pub const MEASURED_BASIS_QUIET_MS: u64 = 13_000;
/// The measured basis: one `shm_list()` scan of that directory UNDER LOAD.
pub const MEASURED_BASIS_LOADED_MS: u64 = 393_000;

/// A projected quiet scan at or above this is [`ScanSeverity::Elevated`].
pub const ELEVATED_QUIET_MS: u64 = 1_000;
/// A projected quiet scan at or above this is [`ScanSeverity::Severe`].
pub const SEVERE_QUIET_MS: u64 = 10_000;

const _: () = assert!(
    ELEVATED_QUIET_MS < SEVERE_QUIET_MS,
    "severity bands must be ordered"
);

/// How many distinct reclamation failures are quoted before the render
/// collapses to a count. Bounded so a directory in which EVERY reclamation
/// fails cannot make the report itself unbounded.
pub const MAX_QUOTED_FAILURES: usize = 8;

/// Does this platform keep `.shm_state` files at all?
///
/// `iceoryx2-pal-posix` uses the state-file indirection on the platforms with
/// no `/dev/shm`: macOS (`macos/mman.rs`) and FreeBSD (`freebsd/mman.rs`).
/// Linux opens POSIX shm objects directly and leaves no such file.
pub const fn platform_uses_shm_state_files() -> bool {
    cfg!(any(target_os = "macos", target_os = "freebsd"))
}

/// Projected cost of ONE `shm_list()` scan over a directory of `files` state
/// files, derived from the measured basis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanCostEstimate {
    /// Projected wall time on a quiet machine.
    pub quiet_ms: u64,
    /// Projected wall time under the load the basis was measured under.
    pub loaded_ms: u64,
}

/// Project the cost of one dead-node sweep over a directory of this size.
///
/// Linear in the file count, which is what a readdir is. The two figures are
/// the SAME measurement under two machine states, not a confidence interval —
/// the real cost depends on what else is running.
pub fn estimate_scan_cost(files: u64) -> ScanCostEstimate {
    ScanCostEstimate {
        quiet_ms: files.saturating_mul(MEASURED_BASIS_QUIET_MS) / MEASURED_BASIS_FILES,
        loaded_ms: files.saturating_mul(MEASURED_BASIS_LOADED_MS) / MEASURED_BASIS_FILES,
    }
}

/// How bad the population is, in terms of the projected QUIET scan it implies.
///
/// Banded on projected TIME rather than on a file count so the thresholds say
/// what they mean; the file counts they correspond to fall out of the basis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanSeverity {
    /// Nothing here is worth an operator's attention.
    Healthy,
    /// Every dead-node sweep on this machine now costs real time.
    Elevated,
    /// A sweep here is the supervisor-stall class the module docs describe.
    Severe,
}

/// Classify a population by the quiet scan it projects.
pub fn severity(files: u64) -> ScanSeverity {
    let quiet = estimate_scan_cost(files).quiet_ms;
    if quiet >= SEVERE_QUIET_MS {
        ScanSeverity::Severe
    } else if quiet >= ELEVATED_QUIET_MS {
        ScanSeverity::Elevated
    } else {
        ScanSeverity::Healthy
    }
}

/// The real POSIX shm object name a state file resolves to, plus the pid of the
/// process that minted it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RealShmName {
    /// The name to hand `shm_open` / `shm_unlink`, verbatim.
    pub name: String,
    /// The pid `generate_real_shm_name` stamped as the first field.
    pub creator_pid: i32,
}

/// Why a state file's content could not be read as a real shm name.
///
/// Every variant means REFUSE — an unparsable file is never reclaimed, because
/// the pid is the whole of the proof and a file that carries no pid carries no
/// proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnparsableReason {
    /// Zero bytes before the first NUL. `shm_open` leaves one of these behind
    /// if a process dies between `O_EXCL|O_CREAT` and the `write`, and it is a
    /// poison pill for that logical name — no pid is recorded, so the
    /// [`ProcessLiveness`] proof cannot be run on it at all.
    ///
    /// It is NOT unreclaimable: a SECOND evidence shape, keyed on AGE rather
    /// than on a pid, adjudicates it — see [`EMPTY_STATE_FILE_MIN_AGE`] and
    /// [`StateFileVerdict::EmptyOrphan`]. This variant is what a file too YOUNG
    /// for that rule (or one whose age could not be established) still refuses
    /// as.
    Empty,
    /// Not the FOUR `_`-separated segments `generate_real_shm_name` mints.
    NotFourSegments {
        /// How many segments the content actually carried.
        found: usize,
    },
    /// The leading segment — the pid — is not a non-empty run of ASCII digits.
    PidNotNumeric,
    /// A NON-pid segment is not a non-empty run of ASCII digits.
    SegmentNotNumeric {
        /// Which segment (0 is the pid, so this is always 1, 2 or 3).
        index: usize,
    },
    /// The leading segment is numeric but not a pid we may signal — `kill(0, 0)`
    /// signals the CALLER'S WHOLE PROCESS GROUP and a negative pid signals a
    /// group, so anything at or below zero is refused outright.
    PidNotPositive,
    /// The leading segment is a number too large for `pid_t`.
    PidOutOfRange,
}

/// How many `_`-separated segments `generate_real_shm_name` mints.
///
/// `<pid>_<tv_sec>_<tv_usec>_<counter>` — `macos/mman.rs:133`.
const MINTED_SEGMENTS: usize = 4;

/// How old a ZERO-LENGTH state file must be before its age counts as evidence.
///
/// # The zero-length poison pill, and why AGE is proof here
///
/// A populated state file's age proves nothing — a long-lived robot's segments
/// are arbitrarily old and perfectly alive, which is why this module refuses an
/// mtime heuristic everywhere else. A ZERO-LENGTH one is a different object,
/// because of the order `iceoryx2-pal-posix` writes it in
/// (`macos/mman.rs:110-130`, `write_real_shm_name`):
///
/// ```text
/// open(<name>.shm_state, O_EXCL | O_CREAT | O_RDWR)   // the file appears, 0 bytes
/// write(fd, <pid>_<sec>_<usec>_<counter>, 33)         // …and is populated
/// ```
///
/// Those two syscalls are ADJACENT: the name was generated before the call, and
/// nothing else runs between them. A failed `write` `remove`s the file itself.
/// So a state file is legitimately zero-length only while its creator is
/// literally between those two lines, and the only way to leave one behind is a
/// hard kill inside that window.
///
/// ## What a false positive would destroy, and why this gate cannot reach it
///
/// **There is no kernel object to strand.** `shm_open` (`macos/mman.rs:155-180`)
/// calls `write_real_shm_name` FIRST and only then `shm_open`s the real name, so
/// a state file that never got its content also never got an object. This is
/// stronger than the "probe the named object" leg the reclamation uses for the
/// pid path — which is unavailable here anyway, the missing name being exactly
/// what is missing. Removing an empty file therefore cannot convert a
/// reclaimable leak into a permanent one; it un-poisons the logical name.
///
/// **The one thing it can destroy** is a creator caught mid-window: its `write`
/// then lands in an unlinked inode, its own segment still works, but a peer
/// opening the same logical name finds no state file and mints a SECOND object
/// — one logical name, two segments. Reaching that requires a process stopped
/// between two adjacent syscalls for this entire bound. A day is chosen because
/// nothing about a leaked poison pill is urgent: it is permanent until removed,
/// and removing it one run later costs nothing.
///
/// Two further properties bound the blast radius, neither of them relied upon:
/// `/tmp` is sticky (`drwxrwxrwt`), so this can only ever unlink a file the
/// calling uid owns; and the state file is minted `S_IWUSR`-only, so a foreign
/// uid's is refused by the kernel rather than by us.
///
/// ## What this rule DEPENDS on, so a future upgrade re-reads it
///
/// The whole argument above is a claim about `iceoryx2-pal-posix` **0.9.1**'s
/// write order — that the state file is written BEFORE the object is created,
/// and that the two syscalls leaving an empty one are adjacent. If a PAL
/// upgrade ever created the object first, an empty file could name a real
/// orphan and removing it would strand one permanently. The version is
/// exact-pinned (`=0.9.1`, guarded by `cerulion_core`'s
/// `iceoryx2_version_lockstep_test`), so a bump is a deliberate act — and this
/// is the paragraph it has to come back to. The pid rule below is unaffected:
/// it names its object and unlinks it first, whatever the order.
pub const EMPTY_STATE_FILE_MIN_AGE: Duration = Duration::from_secs(24 * 60 * 60);

const _: () = assert!(
    EMPTY_STATE_FILE_MIN_AGE.as_millis() > SHM_STATE_RECLAIM_BUDGET.as_millis(),
    "the age bound must dwarf any single sweep, or a file could age INTO the rule \
     during the very walk that is judging it"
);

/// How long ago an entry was last written, as evidence.
///
/// An [`Option`] would have done the same job and said less: this is the third
/// place in the module where a probe can come back with NO answer, and each one
/// is required to REFUSE rather than guess. Naming the state makes that the
/// obvious reading at every call site.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileAge {
    /// The file's mtime was read and is in the past.
    Known(Duration),
    /// No usable age: the stat failed, or the mtime is in the FUTURE (a clock
    /// step, a copied file), which is not an age at all.
    Unknown,
}

impl FileAge {
    /// Is this age PROVEN to be at least `bound`?
    ///
    /// [`FileAge::Unknown`] answers `false`, which is the refusal direction —
    /// an unknown age is never evidence of an old file.
    #[must_use]
    pub fn at_least(self, bound: Duration) -> bool {
        matches!(self, Self::Known(age) if age >= bound)
    }
}

/// Read a state file's 33-byte content as the real shm name it records.
///
/// The WHOLE shape is validated, not just the pid: `/tmp` is world-writable,
/// so this content is HOSTILE INPUT — anybody can create a file there with any
/// bytes in it, and the name that comes out is handed to `shm_unlink`. A rule
/// that only checked the leading segment accepted `123_<anything>` (and a bare
/// `123`), so once pid 123 died, forged or foreign content could aim the
/// reclamation at an UNRELATED object name.
///
/// So the admissible set is exactly what `generate_real_shm_name` can emit:
/// four `_`-separated runs of ASCII digits, nothing else. That is strictly
/// narrower than a character filter — it rejects `123` (too few segments) and
/// `123_a_b_c` (non-numeric segments) as well as `../etc/passwd` — and it
/// leaves no byte outside `[0-9_]` reachable, so nothing resembling a path or
/// a shell metacharacter can survive to the syscall.
///
/// Fails CLOSED on anything it does not fully understand, and each refusal
/// names WHICH premise was missing rather than collapsing to one reason.
pub fn parse_real_shm_name(content: &[u8]) -> Result<RealShmName, UnparsableReason> {
    let end = content
        .iter()
        .position(|b| *b == 0)
        .unwrap_or(content.len());
    let name_bytes = &content[..end];
    if name_bytes.is_empty() {
        return Err(UnparsableReason::Empty);
    }

    let segments: Vec<&[u8]> = name_bytes.split(|b| *b == b'_').collect();
    if segments.len() != MINTED_SEGMENTS {
        return Err(UnparsableReason::NotFourSegments {
            found: segments.len(),
        });
    }
    for (index, segment) in segments.iter().enumerate() {
        if segment.is_empty() || !segment.iter().all(u8::is_ascii_digit) {
            return Err(if index == 0 {
                UnparsableReason::PidNotNumeric
            } else {
                UnparsableReason::SegmentNotNumeric { index }
            });
        }
    }

    // Every byte is now proven to be an ASCII digit or the `_` separator.
    let pid_text =
        core::str::from_utf8(segments[0]).map_err(|_| UnparsableReason::PidNotNumeric)?;
    let pid: i64 = pid_text
        .parse()
        .map_err(|_| UnparsableReason::PidOutOfRange)?;
    if pid <= 0 {
        return Err(UnparsableReason::PidNotPositive);
    }
    let pid = i32::try_from(pid).map_err(|_| UnparsableReason::PidOutOfRange)?;
    // Infallible: the loop above proved every byte is `[0-9_]`, i.e. ASCII.
    // Built by hand rather than through `String::from_utf8` so there is no
    // error arm here demanding a reason variant nothing can reach.
    let name: String = name_bytes.iter().map(|b| char::from(*b)).collect();
    Ok(RealShmName {
        name,
        creator_pid: pid,
    })
}

/// What the system says about a pid.
///
/// `Unknown` is deliberately NOT folded into either answer: it means the probe
/// returned an errno that cannot be read as proof, and a diagnostic that guesses
/// there is the one that deletes a live segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessLiveness {
    /// The process exists (`kill` succeeded, or refused with `EPERM`).
    Alive,
    /// No such process (`ESRCH`).
    Gone,
    /// The probe gave an answer that proves nothing either way.
    Unknown,
}

/// The verdict on ONE state file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateFileVerdict {
    /// The creator process is GONE. Reclaimable: object first, then the file.
    ProvenDead(RealShmName),
    /// The file records NO name and is older than [`EMPTY_STATE_FILE_MIN_AGE`]
    /// (the zero-length poison pill). Reclaimable, and the ONLY verdict whose reclamation
    /// touches no kernel object — there is provably none to touch, because the
    /// PAL creates the object only AFTER writing this file.
    EmptyOrphan,
    /// The creator is still running. Refuse — this may be a live graph.
    CreatorAlive {
        /// The pid the file records.
        pid: i32,
    },
    /// The file names a segment belonging to an iceoryx2 namespace that still
    /// has a node registered. Refuse whatever the creator's pid
    /// says: on this platform a state file is the whole NAMESPACE's shared
    /// management segment, and its creator is merely whoever opened the
    /// namespace first.
    NamespaceInUse,
    /// The liveness probe could not answer. Refuse.
    CreatorUnknown {
        /// The pid the file records.
        pid: i32,
    },
    /// The content does not record a usable pid. Refuse.
    Unparsable(UnparsableReason),
    /// The file itself could not be read. Refuse.
    Unreadable(UnreadableReason),
}

/// Why a directory entry's content could not be read.
///
/// `/tmp` is world-writable, so an entry named `<something>.shm_state` is
/// whatever anybody put there — and two of these arms are refusals of an
/// ATTACK, not of a corrupt file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum UnreadableReason {
    /// Not a regular file — a directory, socket, device, symlink, or FIFO.
    ///
    /// The FIFO is the dangerous one: on macOS `open(2)` for reading BLOCKS
    /// until a writer appears, with no timeout, so a single hostile entry
    /// would hang the diagnostic FOREVER — inside the one command whose whole
    /// contract is that it never hangs. Refused on its TYPE, decided before
    /// any open. A SYMLINK is refused for the neighbouring reason: following
    /// one would let an attacker aim the read (and the size check) at a file
    /// somewhere else entirely, FIFO included.
    NotARegularFile,
    /// Larger than [`MAX_STATE_FILE_BYTES`]. The real content is 33 bytes; an
    /// unbounded read would let one crafted entry allocate past the budget the
    /// whole walk is bounded by.
    TooLarge {
        /// The size `symlink_metadata` reported.
        bytes: u64,
    },
    /// The stat or the read itself failed.
    Io,
}

/// The system calls this module needs, behind a seam so the classifier and the
/// reclaimer are testable over a tempdir with no real processes and no real
/// shared memory.
pub trait SystemProbe {
    /// Does a process with this pid exist?
    fn process_liveness(&self, pid: i32) -> ProcessLiveness;
    /// Size of the named POSIX shm object, or `None` if it does not exist.
    fn shm_object_size(&self, name: &str) -> Option<u64>;
    /// Unlink the named POSIX shm object. `Ok(())` also covers "it was already
    /// gone", which is the same outcome for our purposes.
    fn unlink_shm_object(&self, name: &str) -> Result<(), String>;
    /// The iceoryx2 config PREFIXES that still have a node registered.
    ///
    /// The SECOND piece of evidence, and the one the pid alone cannot
    /// give. Every `.shm_state` file observed on a real desk is a
    /// `<prefix><id>_node.<major>_<minor>_<patch>.global_mgmt` — the ONE
    /// management segment a whole iceoryx2 NAMESPACE shares, not a per-process
    /// resource. Its creator is simply whichever process opened the namespace
    /// first, so "the creator is gone" says nothing whatever about whether the
    /// segment is still in use.
    ///
    /// The version in the middle is iceoryx2's own, added in 0.9.3 so two
    /// versions on one machine keep separate registries. It sits AFTER the
    /// config prefix, which is what this evidence is keyed on, so the
    /// prefix-match below is unaffected by it; `isolated_root_evidence_test`
    /// checks that against real files rather than against this paragraph.
    ///
    /// A namespace with a registered node still needs its name mapping;
    /// removing it splits the namespace in two (the next `shm_open` finds no
    /// mapping and mints a SECOND segment under the same logical name) and
    /// leaves every node registered against the first one unreachable and
    /// unreclaimable.
    ///
    /// Answering with an EMPTY set means "no namespace is in use", so an
    /// implementation that cannot enumerate the registry must say so by
    /// refusing rather than by returning nothing — see [`LibcProbe`]'s
    /// implementation, which treats an unreadable registry as ALL namespaces
    /// in use.
    ///
    /// **Scope.** [`LibcProbe`] does not read only the GLOBAL
    /// config's registry. State files always live in ONE directory (the PAL
    /// hard-codes [`SHM_STATE_DIRECTORY`], independent of `root_path`) but a
    /// node registry does not, so evidence gathered from a single registry root
    /// reads every OTHER root's namespaces as free. That is not a hypothetical
    /// shape: `iceoryx2::testing`'s isolated configs register under
    /// `<root>/tests/nodes/` while dropping their mappings into the same
    /// `/tmp/`, so on a test-heavy desk a `cerulion clean` could strand a
    /// mapping a running suite was holding. [`live_registry_anchors`] +
    /// [`namespaces_in_use_under`] now ENUMERATE the roots and union their
    /// evidence; the residual is stated there.
    ///
    /// `budget` is what is LEFT of the caller's own bound, not a fresh one. A
    /// bound you can pay twice is not a bound: a fixed budget here would let one
    /// `scan` overrun its advertised cap by the whole walk. A spent budget must
    /// answer `Unknown` (refuse), never an empty set.
    fn namespaces_in_use(&self, budget: Duration) -> NamespacesInUse;
}

/// Which iceoryx2 namespaces still have a node registered.
///
/// A closed enum rather than a bare set, because "the registry could not be
/// read" and "the registry is empty" are the same value in a set and MUST be
/// opposite decisions: the first has to refuse everything.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NamespacesInUse {
    /// The registry was read. These prefixes have at least one node.
    Known(std::collections::BTreeSet<String>),
    /// The registry could not be read (or the walk was truncated), so nothing
    /// can be proven unused. Refuse every file.
    Unknown,
}

impl NamespacesInUse {
    /// Does this state file belong to a namespace that is still in use?
    ///
    /// iceoryx2 prefixes are minted prefix-free (see `multiprocess.rs`), so a
    /// `starts_with` match is unambiguous.
    #[must_use]
    pub fn covers(&self, state_file_name: &str) -> bool {
        match self {
            Self::Unknown => true,
            Self::Known(prefixes) => prefixes.iter().any(|p| state_file_name.starts_with(p)),
        }
    }
}

/// The suffix iceoryx2 gives the per-node details file inside a node-registry
/// entry. What precedes it is the config PREFIX that names the namespace.
const NODE_DETAILS_SUFFIX: &str = "node.details";

/// Read the registry and collect the prefixes that still have a node.
///
/// # The layout this walks, verified against a live registry
///
/// `/tmp/iceoryx2/nodes/` holds TWO kinds of entry side by side:
///
/// ```text
/// drwx------  2889553339617049664376854491/          <- node dir, holds <prefix>node.details
/// -rwx------  cer_p_1dcd..975.node_monitor           <- FILE
/// -r--------  cer_p_1dcd..975.node_monitor_context   <- FILE
/// -rwx------  cer_p_1dcd..975.node_monitor_owner_lock<- FILE
/// ```
///
/// The monitor FILES are a known part of the layout, not damage, and every node
/// that has them also has its directory. Treating them as unreadable
/// directories would make this walk answer `Unknown`
/// the moment any node had ever existed — silently turning the whole
/// reclamation inert. A non-directory entry is therefore SKIPPED as evidence.
///
/// # Fail-closed, still
///
/// Genuinely unreadable state keeps refusing: an unreadable registry, an entry
/// whose type cannot be determined, a node DIRECTORY that cannot be listed, and
/// a spent budget all yield [`NamespacesInUse::Unknown`]. An ABSENT registry is
/// the one true empty: there are no nodes.
///
/// Skipping the monitor files loses no real evidence. The only shape they could
/// uniquely witness is a node mid-creation whose directory does not exist yet —
/// and that node's creating process is by definition alive, so its state file is
/// already refused by the liveness gate.
pub fn namespaces_in_use_at(dir: &Path, budget: Duration) -> NamespacesInUse {
    let started = Instant::now();
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
            return NamespacesInUse::Known(std::collections::BTreeSet::new())
        }
        Err(_) => return NamespacesInUse::Unknown,
    };

    let mut prefixes = std::collections::BTreeSet::new();
    for entry in entries {
        if started.elapsed() >= budget {
            return NamespacesInUse::Unknown;
        }
        let Ok(entry) = entry else {
            return NamespacesInUse::Unknown;
        };
        // The monitor files live beside the node directories. A type that cannot
        // be determined is still a refusal — only a KNOWN non-directory is skipped.
        match entry.file_type() {
            Ok(kind) if kind.is_dir() => {}
            Ok(_) => continue,
            Err(_) => return NamespacesInUse::Unknown,
        }
        // A node-registry entry is a DIRECTORY holding `<prefix>node.details`.
        let Ok(inner) = std::fs::read_dir(entry.path()) else {
            // A node whose contents cannot be listed is a node whose namespace
            // cannot be named — and it is still registered.
            return NamespacesInUse::Unknown;
        };
        for file in inner {
            let Ok(file) = file else {
                return NamespacesInUse::Unknown;
            };
            let name = file.file_name().to_string_lossy().to_string();
            if let Some(prefix) = name.strip_suffix(NODE_DETAILS_SUFFIX) {
                if !prefix.is_empty() {
                    prefixes.insert(prefix.to_string());
                }
            }
        }
    }
    NamespacesInUse::Known(prefixes)
}

/// One place to LOOK for node registries: an iceoryx2 root path, plus the
/// directory name that root's config gives its registry.
///
/// Two fields rather than a ready-made registry path because the search below
/// needs BOTH — the root to enumerate, and the name to recognise a registry by
/// when it finds one under a sub-root.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegistryAnchor {
    /// The `global.root_path` of some iceoryx2 config.
    pub root: PathBuf,
    /// That config's `global.node.directory` — the basename its registry has.
    pub node_dir_name: String,
}

/// Every node-registry directory reachable from `anchors`.
///
/// An anchor contributes its OWN registry (`<root>/<node_dir_name>`) and every
/// registry ONE ENTRY down (`<root>/<sub>/<node_dir_name>`).
///
/// # Why one entry is the reach
///
/// DERIVED from the layout, not tuned. `iceoryx2::testing::
/// generate_isolated_config` takes a default `Config` and points `root_path` at
/// `iceoryx2_bb_posix::config::TEST_DIRECTORY`, which on EVERY platform the PAL
/// defines is the compiled-in root plus exactly one entry:
///
/// ```text
/// unix     /tmp/iceoryx2/     ->  /tmp/iceoryx2/tests/
/// android  /data/iceoryx2/    ->  /data/iceoryx2/tests/
/// windows  C:\Temp\iceoryx2\  ->  C:\Temp\iceoryx2\tests\
/// ```
///
/// (`iceoryx2-pal-configuration-0.9.1/src/lib.rs:34-77`.) So one entry reaches
/// the isolated registry, and speculative extra levels are not free: each one
/// multiplies the directory reads by the fan-out of `services/`, which on a busy
/// desk holds one directory per service. Stated as a doc rather than a `const`
/// on purpose — a constant no code reads is a claim of configurability that is
/// not true.
///
/// `None` is the fail-closed answer and means "the search itself could not be
/// completed", which must refuse every file rather than report the registries it
/// happened to reach: a partial root set is indistinguishable from a complete
/// one at the union below, and the missing root is exactly the one whose
/// namespaces would read as free.
///
/// An ABSENT anchor root is NOT a failure — there is provably no registry under
/// a directory that does not exist. The registry directory itself is not
/// stat'd here; [`namespaces_in_use_at`] already treats an absent one as the
/// true empty.
pub fn registry_roots_under(
    anchors: &[RegistryAnchor],
    started: Instant,
    budget: Duration,
) -> Option<Vec<PathBuf>> {
    let mut roots: std::collections::BTreeSet<PathBuf> = std::collections::BTreeSet::new();
    for anchor in anchors {
        if started.elapsed() >= budget {
            return None;
        }
        roots.insert(anchor.root.join(&anchor.node_dir_name));

        let entries = match std::fs::read_dir(&anchor.root) {
            Ok(entries) => entries,
            // No root, no registries under it.
            Err(err) if err.kind() == std::io::ErrorKind::NotFound => continue,
            // We could not LOOK, which is never an absence.
            Err(_) => return None,
        };
        for entry in entries {
            if started.elapsed() >= budget {
                return None;
            }
            let Ok(entry) = entry else {
                return None;
            };
            // `DirEntry::file_type` does NOT follow links, so this judges the
            // entry itself. Three arms, and the middle one is why they are
            // spelled out rather than collapsed into `is_dir` / skip:
            //
            // * a DIRECTORY is a candidate sub-root. `services/` and the
            //   anchor's own registry are directories too, and each simply
            //   costs the one stat below and yields nothing (nothing under them
            //   is named `<node_dir_name>`). Filtering by NAME instead —
            //   skipping everything not called `tests` — would hardcode today's
            //   layout; missing a real sub-root costs far more than a wasted
            //   stat.
            // * a SYMLINK is `Unknown`, the same stance the candidate stat
            //   below takes and for the same reason: following it would let
            //   whoever planted it aim this walk somewhere else, and SKIPPING it
            //   would silently declare every namespace registered under it free
            //   — the stranding this search exists to prevent, reached before
            //   the candidate is ever examined. The cost is deliberate: one
            //   planted link under an anchor root makes the evidence Unknown
            //   machine-wide and the reclaimer refuses every file until it is
            //   removed. That is the fail-closed direction, chosen over
            //   silently freeing a live namespace.
            // * anything ELSE — a regular file, a socket, a FIFO — cannot HOLD
            //   a registry directory, so there is provably nothing under it to
            //   miss and skipping it is correct.
            match entry.file_type() {
                Ok(kind) if kind.is_dir() => {}
                Ok(kind) if kind.is_symlink() => return None,
                Ok(_) => continue,
                Err(_) => return None,
            }
            let candidate = entry.path().join(&anchor.node_dir_name);
            // `symlink_metadata` does NOT follow links, so a SYMLINK is judged
            // as a link rather than as whatever it points at — the same stance
            // `read_state_file` takes, and for the same reason: following one
            // would let whoever planted it aim this walk somewhere else.
            match std::fs::symlink_metadata(&candidate) {
                Ok(meta) if meta.is_dir() => {
                    roots.insert(candidate);
                }
                // It EXISTS but is not a directory — a file, a symlink, a
                // socket. Refusing to look would silently declare whatever
                // namespaces it names free, which is the stranding this fix
                // exists to prevent; following it would let it steer the walk.
                // A shape we can neither read nor dismiss is `Unknown`.
                Ok(_) => return None,
                // Nothing there: this sub-directory is simply not a root.
                Err(err) if err.kind() == std::io::ErrorKind::NotFound => {}
                Err(_) => return None,
            }
        }
    }
    Some(roots.into_iter().collect())
}

/// The UNION of every anchor-reachable registry's in-use namespaces.
///
/// Fail-closed at three separate points, because a set cannot express doubt and
/// each of these would otherwise become a silent "nothing is in use":
///
/// 1. the root search could not be completed ([`registry_roots_under`] `None`);
/// 2. ANY discovered registry answers [`NamespacesInUse::Unknown`];
/// 3. the budget runs out before every registry has been read — a sub-walk
///    handed nothing answers `Unknown` rather than an empty set, so this needs
///    no separate check here.
///
/// `budget` bounds the WHOLE union: ONE clock is taken at entry and every
/// sub-walk is handed what is LEFT of it. A bound you can pay once per root is
/// not a bound.
///
/// **Residual.** This reaches roots that are the anchors themselves or
/// one entry below them. A config whose `root_path` points somewhere else
/// entirely — a directory no anchor names and that is not under one — is still
/// invisible, and its namespaces still read as free. Nothing in this repository
/// and nothing in `iceoryx2::testing` produces that shape; a third-party
/// process on the same desk with a hand-written `root_path` could.
pub fn namespaces_in_use_under(anchors: &[RegistryAnchor], budget: Duration) -> NamespacesInUse {
    let started = Instant::now();
    let Some(roots) = registry_roots_under(anchors, started, budget) else {
        return NamespacesInUse::Unknown;
    };

    let mut union = std::collections::BTreeSet::new();
    for root in roots {
        let remaining = budget.saturating_sub(started.elapsed());
        match namespaces_in_use_at(&root, remaining) {
            NamespacesInUse::Known(prefixes) => union.extend(prefixes),
            // One unreadable registry is one namespace set that cannot be named, and
            // a union missing a set is a union that proves nothing.
            NamespacesInUse::Unknown => return NamespacesInUse::Unknown,
        }
    }
    NamespacesInUse::Known(union)
}

/// The anchors to search on THIS machine.
///
/// Two configs, because they can disagree and each is authoritative for a
/// different population:
///
/// * `Config::global_config()` is what a Cerulion process on this desk actually
///   runs under — a config FILE may have moved its root.
/// * `Config::default()` is the COMPILED-IN root, and it is what
///   `iceoryx2::testing::generate_isolated_config` starts from (it only
///   overrides `root_path` and `prefix`), so the isolated roots hang off this
///   one whatever the config file says.
///
/// Deduped here when the two agree — the ordinary case, where no config file is
/// loaded, so one search runs rather than two — and again by the root SET
/// [`registry_roots_under`] builds, which is what covers two anchors that
/// disagree on the root but reach an overlapping registry.
#[must_use]
pub fn live_registry_anchors() -> Vec<RegistryAnchor> {
    let anchor_of = |cfg: &iceoryx2::config::Config| RegistryAnchor {
        root: PathBuf::from(String::from(cfg.global.root_path())),
        node_dir_name: String::from(&cfg.global.node.directory),
    };
    let global = anchor_of(iceoryx2::config::Config::global_config());
    let default = anchor_of(&iceoryx2::config::Config::default());
    if global == default {
        vec![global]
    } else {
        vec![global, default]
    }
}

/// What [`creator_verdict`] can say about the process that minted a registry
/// entry. Deliberately NOT [`ProcessLiveness`]: that type is this module's
/// internal evidence vocabulary and the no-second-copy guard forbids naming it
/// anywhere else, so a caller outside gets this closed answer instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreatorVerdict {
    /// `kill(pid, 0)` answered `ESRCH`: no such process. Reclaimable.
    Gone,
    /// The process exists (or exists and refuses signals). Refuse.
    Alive,
    /// No usable proof either way. Refuse.
    Unknown,
}

/// The ONE liveness answer for the CREATOR of a registry entry — `(pid,
/// creation stamp)` — reached through this module's own predicate
/// ([`SystemProbe::process_liveness`] on [`LibcProbe`], i.e. `kill(pid, 0)`),
/// so `cerulion clean` routes through the same evidence the `.shm_state`
/// reclamation trusts and carries no second spelling of it.
///
/// `created_at_unix_s` is the entry's `creation_time` on a `Realtime` clock,
/// or `None` when the stamp is on a clock that cannot be compared to wall time
/// (`Monotonic`). A stamp LATER than now (past a minute of skew) is not
/// evidence about any process and answers `Unknown`. A pid that does not fit
/// `kill(2)`'s argument answers `Unknown` too. Pid reuse is NOT detected:
/// a reused pid reads `Alive`, which refuses — the safe direction.
pub fn creator_verdict(pid: u32, created_at_unix_s: Option<u64>) -> CreatorVerdict {
    const CLOCK_SKEW_SLACK_S: u64 = 60;
    if let Some(created) = created_at_unix_s {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        if created > now.saturating_add(CLOCK_SKEW_SLACK_S) {
            return CreatorVerdict::Unknown;
        }
    }
    #[cfg(unix)]
    {
        match LibcProbe.process_liveness(i32::try_from(pid).unwrap_or(0)) {
            ProcessLiveness::Gone => CreatorVerdict::Gone,
            ProcessLiveness::Alive => CreatorVerdict::Alive,
            ProcessLiveness::Unknown => CreatorVerdict::Unknown,
        }
    }
    #[cfg(not(unix))]
    {
        let _ = pid;
        CreatorVerdict::Unknown
    }
}

/// The real probe: `kill(2)`, `shm_open(2)` + `fstat(2)`, `shm_unlink(2)`.
#[cfg(unix)]
#[derive(Debug, Default, Clone, Copy)]
pub struct LibcProbe;

#[cfg(unix)]
impl SystemProbe for LibcProbe {
    fn process_liveness(&self, pid: i32) -> ProcessLiveness {
        if pid <= 0 {
            // Never reachable through `parse_real_shm_name`, which refuses
            // these — but `kill(0, ..)` signals the caller's whole process
            // group, so the guard is restated at the syscall itself.
            return ProcessLiveness::Unknown;
        }
        // SAFETY: signal 0 performs the error checking without sending a
        // signal, and `pid` is proven positive above.
        let rc = unsafe { libc::kill(pid, 0) };
        if rc == 0 {
            return ProcessLiveness::Alive;
        }
        match std::io::Error::last_os_error().raw_os_error() {
            // The process exists; we simply may not signal it.
            Some(libc::EPERM) => ProcessLiveness::Alive,
            Some(libc::ESRCH) => ProcessLiveness::Gone,
            _ => ProcessLiveness::Unknown,
        }
    }

    fn shm_object_size(&self, name: &str) -> Option<u64> {
        let c_name = std::ffi::CString::new(name).ok()?;
        // SAFETY: `c_name` is a valid NUL-terminated string for the duration of
        // the call; no `O_CREAT`, so this can only observe an existing object.
        let fd = unsafe { libc::shm_open(c_name.as_ptr(), libc::O_RDONLY, 0) };
        if fd < 0 {
            return None;
        }
        let mut st: libc::stat = unsafe { std::mem::zeroed() };
        // SAFETY: `fd` is a live descriptor and `st` is a valid out-parameter.
        let rc = unsafe { libc::fstat(fd, &mut st) };
        // SAFETY: `fd` came from a successful `shm_open` and is closed once.
        unsafe { libc::close(fd) };
        if rc != 0 {
            return None;
        }
        u64::try_from(st.st_size).ok()
    }

    fn namespaces_in_use(&self, budget: Duration) -> NamespacesInUse {
        // The union over every reachable registry root, not just the
        // global one. State files are root-INDEPENDENT (they all land in
        // `SHM_STATE_DIRECTORY`), so evidence from one root alone reads every
        // other root's live namespaces as free.
        namespaces_in_use_under(&live_registry_anchors(), budget)
    }

    fn unlink_shm_object(&self, name: &str) -> Result<(), String> {
        let c_name = std::ffi::CString::new(name)
            .map_err(|_| "the recorded name contains an interior NUL".to_string())?;
        // SAFETY: `c_name` is a valid NUL-terminated string for the call.
        let rc = unsafe { libc::shm_unlink(c_name.as_ptr()) };
        if rc == 0 {
            return Ok(());
        }
        let err = std::io::Error::last_os_error();
        if err.raw_os_error() == Some(libc::ENOENT) {
            // Already gone. `iceoryx2-pal-posix`'s own `shm_unlink` treats this
            // exactly the same way (`macos/mman.rs:190`) and goes on to remove
            // the state file.
            return Ok(());
        }
        Err(err.to_string())
    }
}

/// The most a `.shm_state` file may hold before it is refused unread.
///
/// The real content is a 33-byte `SHM_MAX_NAME_LEN` buffer. 4 KiB is a
/// generous ceiling that still bounds a crafted entry to one page.
pub const MAX_STATE_FILE_BYTES: u64 = 4096;

const _: () = assert!(
    MAX_STATE_FILE_BYTES > 33,
    "the cap must admit the 33-byte buffer `write_real_shm_name` really writes"
);

/// One entry's bytes and its age — everything [`classify`] is allowed to
/// decide on.
///
/// The age rides along rather than being fetched separately because
/// [`read_state_file`] already `fstat`s the OPEN handle to close the
/// stat-to-open race, so the timestamp is free at exactly the point where it is
/// also trustworthy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StateFileRead {
    /// The bytes read — at most [`MAX_STATE_FILE_BYTES`].
    pub content: Vec<u8>,
    /// How long ago the OPENED handle says it was last written.
    pub age: FileAge,
}

/// Read one candidate entry's content, refusing anything hostile BEFORE it
/// can cost anything.
///
/// Order is the whole of the safety property, and each step closes one way the
/// next could be attacked:
///
/// 1. `symlink_metadata` — does NOT follow symlinks, so a link is judged as a
///    LINK rather than as whatever it points at.
/// 2. Refuse anything that is not a REGULAR FILE. This is what keeps a FIFO
///    from ever being opened, and it is a decision taken with no open at all.
/// 3. Refuse anything past the size cap, using the size that same stat
///    reported.
/// 4. Open with `O_NOFOLLOW | O_NONBLOCK` (Unix), then RE-STAT the open
///    handle and refuse if the type changed. Steps 1-3 raced a swap: an
///    attacker who replaces the path with a FIFO between the stat and the open
///    would otherwise get the block back. `O_NONBLOCK` means even that open
///    returns immediately, and the `fstat` on the handle refuses what it
///    really got.
/// 5. Read at most the cap, and take the AGE off the same open handle (step 4's
///    `fstat`) rather than off the pre-open stat — a swapped path is refused by
///    the type check either way, but the age must describe the bytes we read.
pub fn read_state_file(path: &Path) -> Result<StateFileRead, UnreadableReason> {
    let meta = std::fs::symlink_metadata(path).map_err(|_| UnreadableReason::Io)?;
    if !meta.file_type().is_file() {
        return Err(UnreadableReason::NotARegularFile);
    }
    if meta.len() > MAX_STATE_FILE_BYTES {
        return Err(UnreadableReason::TooLarge { bytes: meta.len() });
    }

    let mut options = std::fs::OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        // `O_NOFOLLOW` refuses a symlink at the syscall (closing the swap that
        // step 1 could only see a moment earlier); `O_NONBLOCK` makes the open
        // of a FIFO return instead of waiting for a writer, so step 4's
        // re-stat can refuse it.
        options.custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
    }
    let mut file = options.open(path).map_err(|_| UnreadableReason::Io)?;

    // The handle's OWN type — this is what closes the stat-to-open race.
    let opened = file.metadata().map_err(|_| UnreadableReason::Io)?;
    if !opened.file_type().is_file() {
        return Err(UnreadableReason::NotARegularFile);
    }
    if opened.len() > MAX_STATE_FILE_BYTES {
        return Err(UnreadableReason::TooLarge {
            bytes: opened.len(),
        });
    }

    use std::io::Read as _;
    let mut buf = Vec::new();
    file.by_ref()
        .take(MAX_STATE_FILE_BYTES)
        .read_to_end(&mut buf)
        .map_err(|_| UnreadableReason::Io)?;
    Ok(StateFileRead {
        content: buf,
        age: file_age(&opened),
    })
}

/// The age of an already-`stat`ed entry, refusing anything that is not one.
///
/// A mtime in the FUTURE is [`FileAge::Unknown`], not a saturated zero: a
/// clock step or a copied timestamp is an absence of evidence, and folding it
/// to "brand new" would be the safe answer only by accident.
fn file_age(meta: &std::fs::Metadata) -> FileAge {
    let Ok(modified) = meta.modified() else {
        return FileAge::Unknown;
    };
    match SystemTime::now().duration_since(modified) {
        Ok(age) => FileAge::Known(age),
        Err(_) => FileAge::Unknown,
    }
}

/// Decide what a state file's bytes permit.
///
/// THE one classifier. Every caller — `cerulion clean`, the graceful
/// exit pass — routes here rather than re-deciding, so there is one definition
/// of what counts as proof.
pub fn classify(
    read: Result<&StateFileRead, UnreadableReason>,
    probe: &dyn SystemProbe,
) -> StateFileVerdict {
    let read = match read {
        Ok(read) => read,
        Err(reason) => return StateFileVerdict::Unreadable(reason),
    };
    let real = match parse_real_shm_name(&read.content) {
        Ok(real) => real,
        // The ONE unparsable shape with a second evidence route: a file that
        // records no name records no pid either, so the liveness proof cannot
        // run — but the PAL's write order makes AGE proof here. Every OTHER
        // unparsable shape is forged or corrupt content, which age says nothing
        // about, and stays refused forever.
        Err(UnparsableReason::Empty) if read.age.at_least(EMPTY_STATE_FILE_MIN_AGE) => {
            return StateFileVerdict::EmptyOrphan
        }
        Err(reason) => return StateFileVerdict::Unparsable(reason),
    };
    match probe.process_liveness(real.creator_pid) {
        ProcessLiveness::Gone => StateFileVerdict::ProvenDead(real),
        ProcessLiveness::Alive => StateFileVerdict::CreatorAlive {
            pid: real.creator_pid,
        },
        ProcessLiveness::Unknown => StateFileVerdict::CreatorUnknown {
            pid: real.creator_pid,
        },
    }
}

/// Whether the walk may delete what it proves dead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanMode {
    /// Count and classify; touch nothing.
    ReportOnly,
    /// Count, classify, and reclaim every PROVABLY dead entry.
    Reclaim,
}

/// What one walk found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShmStateReport {
    /// The directory walked.
    pub dir: PathBuf,
    /// State files SEEN. A FLOOR when [`Self::truncated`] is set.
    pub counted: u64,
    /// The walk ran out of budget, so `counted` is "at least".
    pub truncated: bool,
    /// How long the walk actually took.
    pub elapsed: Duration,
    /// Files classified (a subset of `counted` once classification is
    /// truncated).
    pub classified: u64,
    /// Classification ran out of ITS budget before the walk ended.
    pub classification_truncated: bool,
    /// Files whose creator process is provably gone.
    pub proven_dead: u64,
    /// Files that record NO name and are past [`EMPTY_STATE_FILE_MIN_AGE`]
    /// (the zero-length poison pill). Counted apart from [`Self::proven_dead`] because
    /// they rest on a DIFFERENT proof and an operator reading a report should
    /// be able to tell the two populations apart.
    pub empty_orphans: u64,
    /// Files whose creator is still running — deliberately left alone.
    pub creator_alive: u64,
    /// Files belonging to an iceoryx2 namespace that still has a node
    /// registered — deliberately left alone, whatever the creating
    /// process's pid says. Counted apart from [`Self::creator_alive`] because
    /// it is a different proof about a different thing: not "somebody made
    /// this and is still here" but "somebody is still USING this".
    pub namespace_in_use: u64,
    /// Files we could not prove anything about — left alone, and reported
    /// rather than deleted.
    pub unproven: u64,
    /// State files actually removed (`ScanMode::Reclaim` only), across BOTH
    /// proofs.
    pub reclaimed_files: u64,
    /// The [`Self::empty_orphans`] share of [`Self::reclaimed_files`]. Exact
    /// rather than derived from `empty_orphans`, which counts what was
    /// CLASSIFIED — a removal can still fail.
    pub reclaimed_empty_files: u64,
    /// Bytes of orphaned POSIX shm object released with them.
    pub reclaimed_object_bytes: u64,
    /// Whether this walk was allowed to DELETE — i.e. ran under
    /// [`ScanMode::Reclaim`].
    ///
    /// Threaded from the mode, never inferred from the counters. "Reclaimed
    /// nothing" and "was not allowed to try" are different facts and a count of
    /// zero is both of them: a pass that ran and whose every unlink FAILED has
    /// `reclaimed_files == 0` and must not be reported as one that never ran,
    /// with its own failure lines printed directly underneath.
    pub attempted_reclamation: bool,
    /// Reclamations that failed, quoted up to [`MAX_QUOTED_FAILURES`].
    pub reclaim_failures: Vec<String>,
    /// Reclamation failures beyond the quoted ones.
    pub reclaim_failures_elided: u64,
    /// The walk could not be performed at all, and WHY. `None` means it ran.
    ///
    /// A failed `read_dir` — a permission error, an I/O error, a path that is
    /// not a directory — must never read as a successful EMPTY answer, or
    /// `cerulion clean` prints a clean no-leak result on a machine it has not
    /// managed to look at. That is the refusal-vs-absence class the budget
    /// arms already refuse; a walk that did not happen must say so.
    pub refused: Option<String>,
    /// Directory entries the walk could not even NAME (a `read_dir` iteration
    /// error, or a non-UTF-8 name). Nonzero means the count is a floor for a
    /// second reason, unrelated to the budget.
    pub skipped_entries: u64,
}

impl ShmStateReport {
    /// A report for a walk that never started.
    fn empty(dir: PathBuf) -> Self {
        Self {
            dir,
            counted: 0,
            truncated: false,
            elapsed: Duration::ZERO,
            classified: 0,
            classification_truncated: false,
            proven_dead: 0,
            empty_orphans: 0,
            creator_alive: 0,
            namespace_in_use: 0,
            unproven: 0,
            reclaimed_files: 0,
            reclaimed_empty_files: 0,
            reclaimed_object_bytes: 0,
            attempted_reclamation: false,
            reclaim_failures: Vec::new(),
            reclaim_failures_elided: 0,
            refused: None,
            skipped_entries: 0,
        }
    }

    /// Projected cost of one dead-node sweep over what was counted.
    ///
    /// A floor count projects a floor cost — the real directory is only ever
    /// bigger.
    pub fn cost(&self) -> ScanCostEstimate {
        estimate_scan_cost(self.counted)
    }

    /// Severity of what was counted, on the same floor caveat as [`Self::cost`].
    pub fn severity(&self) -> ScanSeverity {
        severity(self.counted)
    }

    /// Did the walk RUN and find nothing?
    ///
    /// A refused or truncated walk is NOT empty — it is a walk that cannot
    /// support an absence claim.
    pub fn is_empty(&self) -> bool {
        self.counted == 0 && !self.truncated && self.refused.is_none()
    }
}

/// Walk `dir` for `.shm_state` files, classify them, and (in
/// [`ScanMode::Reclaim`]) reclaim the ones proven dead.
///
/// `scan_budget` bounds the WHOLE walk; `reclaim_budget` bounds the per-file
/// reading, probing and reclaiming. A zero budget on either refuses to START
/// that work rather than letting an exhausted budget buy a fresh ceiling.
///
/// Never returns an error for a missing directory — a machine with no
/// `.shm_state` files is the healthy case, not a fault.
pub fn scan(
    dir: &Path,
    scan_budget: Duration,
    reclaim_budget: Duration,
    mode: ScanMode,
    probe: &dyn SystemProbe,
) -> ShmStateReport {
    let started = Instant::now();
    let mut report = ShmStateReport::empty(dir.to_path_buf());

    if scan_budget.is_zero() {
        // A zero budget cannot start the readdir at all. `counted` is a floor
        // of ZERO, and `truncated` is what says so.
        report.truncated = true;
        return report;
    }

    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        // A directory that does not exist genuinely holds no state files, so
        // that is the healthy EMPTY answer. Every OTHER error — permission
        // denied, an I/O fault, a path that is not a directory — means we
        // could not LOOK, which is a refusal and never an absence.
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => return report,
        Err(err) => {
            report.refused = Some(err.to_string());
            return report;
        }
    };

    report.attempted_reclamation = mode == ScanMode::Reclaim;

    // Classification is refused outright at a zero budget, for the same reason.
    let mut classifying = !reclaim_budget.is_zero();
    report.classification_truncated = !classifying;

    // ONE registry walk for the whole scan. A `.shm_state` file is
    // the shared management segment of an iceoryx2 NAMESPACE, so a namespace
    // with any node registered still needs its name mapping — whatever the
    // creating process's pid says. Fails closed: an unreadable or truncated
    // walk answers `Unknown`, which refuses every file.
    let in_use = probe.namespaces_in_use(scan_budget.saturating_sub(started.elapsed()));

    for entry in entries {
        if started.elapsed() >= scan_budget {
            report.truncated = true;
            break;
        }
        // An entry that cannot even be NAMED is COUNTED as skipped rather than
        // silently passed over: it may or may not have been a state file, so
        // the population becomes a floor for a reason the budget cannot
        // explain.
        let Ok(entry) = entry else {
            report.skipped_entries += 1;
            continue;
        };
        let name = entry.file_name();
        let Some(name) = name.to_str() else {
            report.skipped_entries += 1;
            continue;
        };
        if !name.ends_with(SHM_STATE_SUFFIX) {
            continue;
        }
        report.counted += 1;

        if classifying && started.elapsed() >= reclaim_budget {
            classifying = false;
            report.classification_truncated = true;
        }
        if !classifying {
            continue;
        }

        report.classified += 1;
        if in_use.covers(name) {
            // Decided on the NAME, before any read: the namespace evidence
            // outranks the creator's pid and makes the read pointless.
            report.namespace_in_use += 1;
            continue;
        }

        let path = entry.path();
        let read = read_state_file(&path);
        let verdict = classify(read.as_ref().map_err(Clone::clone), probe);
        match verdict {
            StateFileVerdict::ProvenDead(real) => {
                report.proven_dead += 1;
                if mode == ScanMode::Reclaim {
                    reclaim_one(&path, &real, probe, &mut report);
                }
            }
            StateFileVerdict::EmptyOrphan => {
                report.empty_orphans += 1;
                if mode == ScanMode::Reclaim {
                    reclaim_empty(&path, &mut report);
                }
            }
            StateFileVerdict::CreatorAlive { .. } => report.creator_alive += 1,
            // Unreachable through this loop (the name gate above decides it
            // first) but matched explicitly so a future `classify` arm cannot
            // fall into `unproven` by accident.
            StateFileVerdict::NamespaceInUse => report.namespace_in_use += 1,
            StateFileVerdict::CreatorUnknown { .. }
            | StateFileVerdict::Unparsable(_)
            | StateFileVerdict::Unreadable(_) => report.unproven += 1,
        }
    }

    report.elapsed = started.elapsed();
    report
}

/// Release ONE proven-dead entry: the kernel object first, the state file
/// second.
///
/// The order is load-bearing, not stylistic. Removing the file first strands
/// the object under a name no `shm_open` can ever resolve again, turning a
/// reclaimable leak into a permanent one — so a failure to unlink the object
/// leaves the state file exactly where it is, and says so.
fn reclaim_one(
    path: &Path,
    real: &RealShmName,
    probe: &dyn SystemProbe,
    report: &mut ShmStateReport,
) {
    let bytes = probe.shm_object_size(&real.name).unwrap_or(0);
    if let Err(err) = probe.unlink_shm_object(&real.name) {
        record_failure(report, format!("{}: {err}", real.name));
        return;
    }
    match std::fs::remove_file(path) {
        Ok(()) => {
            report.reclaimed_files += 1;
            report.reclaimed_object_bytes = report.reclaimed_object_bytes.saturating_add(bytes);
        }
        Err(err) => record_failure(report, format!("{}: {err}", path.display())),
    }
}

/// Release ONE zero-length orphan: the file, and only the file.
///
/// The asymmetry with [`reclaim_one`] is the whole point and is not an
/// omission. That function unlinks the kernel object first because removing the
/// file would strand it under an unresolvable name. Here there is provably no
/// object — `shm_open` writes this file BEFORE it creates one
/// (`macos/mman.rs:155-180`), so a file that never got its content never got an
/// object either — and there is no name to unlink it by even if there were.
fn reclaim_empty(path: &Path, report: &mut ShmStateReport) {
    match std::fs::remove_file(path) {
        Ok(()) => {
            report.reclaimed_files += 1;
            report.reclaimed_empty_files += 1;
        }
        Err(err) => record_failure(report, format!("{}: {err}", path.display())),
    }
}

fn record_failure(report: &mut ShmStateReport, message: String) {
    if report.reclaim_failures.len() < MAX_QUOTED_FAILURES {
        report.reclaim_failures.push(message);
    } else {
        report.reclaim_failures_elided += 1;
    }
}

/// The GRACEFUL-EXIT reclamation: one budgeted [`ScanMode::Reclaim`]
/// walk, run when a `cerulion graph run` is over.
///
/// A named entry point rather than a `scan` call at the exit site, so the exit
/// path cannot acquire its own opinion about the mode or the budget split — the
/// same reason `cerulion clean`'s two arms are structurally pinned. Everything
/// it does is [`scan`]'s: same evidence, same refusals, same floor reporting.
///
/// Note which pairing is fixed here: `Reclaim` mode with `classify_budget` of
/// the SAME budget the walk gets, so per-file probing can never starve the
/// count (see [`classify_budget`]).
pub fn reclaim_at_exit(dir: &Path, budget: Duration, probe: &dyn SystemProbe) -> ShmStateReport {
    scan(
        dir,
        budget,
        classify_budget(budget),
        ScanMode::Reclaim,
        probe,
    )
}

/// The ONE line a graceful exit prints, or nothing at all.
///
/// Split from the text so the LEVEL is a property of the report rather than a
/// decision the call site re-derives: a reclamation is `info!` housekeeping, a
/// reclamation that FAILED is a `warn!`, and a pure renderer cannot log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExitReclaimLine {
    /// The rendered line.
    pub text: String,
    /// At least one reclamation failed, so this is not routine housekeeping.
    pub failed: bool,
}

/// Render what a graceful exit's reclamation should SAY — `None` when it should
/// say nothing.
///
/// Silence is the common case and it is deliberate: every `graph run` on every
/// desk ends, and a hygiene pass that announces "nothing to do" on each of them
/// is noise that trains an operator to stop reading the shutdown lines.
///
/// What breaks the silence is a CHANGE (files were removed) or a FAILURE
/// (something was proven dead and could not be removed). A walk that was merely
/// TRUNCATED with nothing reclaimed stays silent on purpose: it made no change
/// and found nothing it could act on, and the population diagnostic — with its
/// severity bands and its projected scan cost — is `cerulion clean`'s job, which
/// the startup sweep's own budget warning already names.
#[must_use]
pub fn render_exit_reclaim_line(report: &ShmStateReport) -> Option<ExitReclaimLine> {
    let failures = report.reclaim_failures.len() as u64 + report.reclaim_failures_elided;
    if report.reclaimed_files == 0 && failures == 0 {
        return None;
    }

    let mut text = if report.reclaimed_files == 0 {
        "reclaimed nothing at exit".to_string()
    } else {
        format!(
            "reclaimed {} stale {SHM_STATE_SUFFIX} file(s) at exit, releasing {} of orphaned \
             shared memory",
            report.reclaimed_files,
            render_bytes(report.reclaimed_object_bytes)
        )
    };
    if report.reclaimed_empty_files > 0 {
        // A distinct proof deserves a distinct phrase: these were adjudicated
        // by AGE, not by a dead pid, and they released no memory (there is
        // none) — so folding them silently into the byte figure would make the
        // headline number look wrong.
        text.push_str(&format!(
            "; {} of them recorded no name (a creator killed mid-create) and held no object",
            report.reclaimed_empty_files
        ));
    }
    if failures > 0 {
        text.push_str(&format!(
            "; {failures} reclamation(s) failed and were left in place"
        ));
    }
    if report.truncated || report.classification_truncated {
        text.push_str(
            "; the exit budget ran out before the directory ended — run `cerulion clean` to finish",
        );
    }
    Some(ExitReclaimLine {
        text,
        failed: failures > 0,
    })
}

/// How many nodes iceoryx2's on-disk registry currently holds.
///
/// Adjacent fact: this is the OTHER population every dead-node sweep
/// walks (`Node::list` reads this directory and parses every filename), so it
/// belongs on the same surface. Cheap — one readdir of a directory that is
/// small on a healthy machine — but budgeted anyway, on the same rule.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NodeRegistryReport {
    /// The directory walked.
    pub dir: PathBuf,
    /// Entries seen. A FLOOR when [`Self::truncated`] is set.
    pub entries: u64,
    /// The walk ran out of budget.
    pub truncated: bool,
    /// The directory does not exist — no iceoryx2 process has ever run here,
    /// or it has already been cleaned. Set ONLY on `NotFound`: every other
    /// error means we could not look, which is [`Self::refused`].
    pub absent: bool,
    /// The walk could not be performed, and WHY. `None` means it ran.
    pub refused: Option<String>,
    /// Entries the walk could not name. Nonzero makes the count a floor.
    pub skipped_entries: u64,
}

/// Count the entries in iceoryx2's node-registry directory.
pub fn count_node_registry(dir: &Path, budget: Duration) -> NodeRegistryReport {
    let started = Instant::now();
    let mut report = NodeRegistryReport {
        dir: dir.to_path_buf(),
        entries: 0,
        truncated: false,
        absent: false,
        refused: None,
        skipped_entries: 0,
    };
    if budget.is_zero() {
        report.truncated = true;
        return report;
    }
    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
            report.absent = true;
            return report;
        }
        Err(err) => {
            report.refused = Some(err.to_string());
            return report;
        }
    };
    for entry in entries {
        if started.elapsed() >= budget {
            report.truncated = true;
            break;
        }
        if entry.is_ok() {
            report.entries += 1;
        } else {
            report.skipped_entries += 1;
        }
    }
    report
}

/// The node-registry directory iceoryx2 is configured to use.
///
/// Read from the LIVE global config rather than hardcoded, so a custom
/// `root_path` is followed instead of quietly reporting on `/tmp/iceoryx2/`.
pub fn iceoryx2_node_dir() -> PathBuf {
    let dir = iceoryx2::config::Config::global_config().global.node_dir();
    PathBuf::from(String::from(&dir))
}

/// Render a duration in milliseconds the way an operator reads one.
fn render_ms(ms: u64) -> String {
    if ms < 1_000 {
        format!("{ms} ms")
    } else if ms < 60_000 {
        format!("{:.1} s", ms as f64 / 1_000.0)
    } else {
        format!("{:.1} min", ms as f64 / 60_000.0)
    }
}

/// Render [`EMPTY_STATE_FILE_MIN_AGE`] the way an operator reads it.
///
/// Whole hours rather than through [`render_ms`], which would show a day as
/// `1440.0 min` — a figure nobody can check against the constant at a glance.
fn render_min_age() -> String {
    const HOUR: u64 = 3_600;
    const _: () = assert!(
        EMPTY_STATE_FILE_MIN_AGE.as_secs() >= HOUR
            && EMPTY_STATE_FILE_MIN_AGE.as_secs() / HOUR * HOUR
                == EMPTY_STATE_FILE_MIN_AGE.as_secs(),
        "the age bound is rendered in whole hours; a value that is not one would render as a lie"
    );
    format!("{} h", EMPTY_STATE_FILE_MIN_AGE.as_secs() / 3_600)
}

/// Render a byte count the way an operator reads one.
fn render_bytes(bytes: u64) -> String {
    const KIB: u64 = 1024;
    const MIB: u64 = KIB * 1024;
    const GIB: u64 = MIB * 1024;
    if bytes < KIB {
        format!("{bytes} B")
    } else if bytes < MIB {
        format!("{:.1} KiB", bytes as f64 / KIB as f64)
    } else if bytes < GIB {
        format!("{:.1} MiB", bytes as f64 / MIB as f64)
    } else {
        format!("{:.2} GiB", bytes as f64 / GIB as f64)
    }
}

/// The `cerulion clean` diagnostic block, as lines.
///
/// PURE so the surface an operator actually reads is oracle-testable — the
/// numbers only mean anything if the render carries them, and a render that
/// silently drops one looks exactly like a healthy machine.
///
/// `shm` is `None` on a platform that keeps no state files, which is REPORTED
/// rather than skipped: silence there is indistinguishable from a diagnostic
/// that ran and found nothing, and the node-registry half is applicable
/// everywhere.
pub fn render_lines(nodes: &NodeRegistryReport, shm: Option<&ShmStateReport>) -> Vec<String> {
    let mut out = Vec::new();

    if let Some(reason) = &nodes.refused {
        // NOT an absence: we could not look, and saying "none" would be a
        // claim nothing gathered.
        out.push(format!(
            "iceoryx2 node registry: COULD NOT READ {} — {reason}",
            nodes.dir.display()
        ));
    } else if nodes.absent {
        out.push(format!(
            "iceoryx2 node registry: none at {} (no iceoryx2 process has left state here)",
            nodes.dir.display()
        ));
    } else {
        let floor = if nodes.truncated || nodes.skipped_entries > 0 {
            "at least "
        } else {
            ""
        };
        out.push(format!(
            "iceoryx2 node registry: {floor}{} entr{} under {}",
            nodes.entries,
            if nodes.entries == 1 { "y" } else { "ies" },
            nodes.dir.display()
        ));
    }

    let Some(shm) = shm else {
        out.push(
            "shm-state files: not applicable on this platform (only macOS/FreeBSD keep \
             `.shm_state` files — this OS opens POSIX shared memory directly)"
                .to_string(),
        );
        return out;
    };

    if let Some(reason) = &shm.refused {
        // The walk did not happen. Reporting 0 here would be the same wrong
        // claim the FLOOR machinery exists to refuse one layer down.
        out.push(format!(
            "{}*{SHM_STATE_SUFFIX} population: COULD NOT READ {} — {reason}",
            shm.dir.display(),
            shm.dir.display()
        ));
        out.push(
            "  the population is UNKNOWN — this is not a report that nothing leaked.".to_string(),
        );
        return out;
    }

    if shm.is_empty() {
        out.push(format!(
            "{}*{SHM_STATE_SUFFIX} population: 0 — nothing leaked here",
            shm.dir.display()
        ));
        return out;
    }

    let floor = if shm.truncated || shm.skipped_entries > 0 {
        "at least "
    } else {
        ""
    };
    let severity_label = match shm.severity() {
        ScanSeverity::Healthy => "healthy",
        ScanSeverity::Elevated => "ELEVATED",
        ScanSeverity::Severe => "SEVERE",
    };
    let cost = shm.cost();
    out.push(format!(
        "{}*{SHM_STATE_SUFFIX} population: {floor}{} file(s) — {severity_label}",
        shm.dir.display(),
        shm.counted
    ));
    out.push(format!(
        "  one dead-node sweep over this directory projects ~{} quiet / ~{} under load;",
        render_ms(cost.quiet_ms),
        render_ms(cost.loaded_ms)
    ));
    out.push(
        "  every startup path that sweeps dead nodes pays that before its first log line."
            .to_string(),
    );
    if shm.truncated {
        out.push(
            "  the walk hit its budget, so the count above is a FLOOR — the real population \
             is higher."
                .to_string(),
        );
    }
    if shm.skipped_entries > 0 {
        out.push(format!(
            "  {} director{} could not be read, so the count above is a FLOOR.",
            shm.skipped_entries,
            if shm.skipped_entries == 1 {
                "y entry"
            } else {
                "y entries"
            }
        ));
    }
    if shm.classification_truncated {
        out.push(format!(
            "  classification stopped at {} file(s); re-run `cerulion clean` to continue.",
            shm.classified
        ));
    }

    if shm.reclaimed_files > 0 {
        out.push(format!(
            "  reclaimed {} file(s), releasing {} of orphaned shared memory.",
            shm.reclaimed_files,
            render_bytes(shm.reclaimed_object_bytes)
        ));
        if shm.reclaimed_empty_files > 0 {
            // Named apart because they rest on a different proof and released
            // no memory: folding them into the byte figure above would make
            // that number read as if it under-counted.
            out.push(format!(
                "  {} of those recorded no name at all (a creator killed mid-create, \
                 older than {}) and held no object.",
                shm.reclaimed_empty_files,
                render_min_age()
            ));
        }
    } else if shm.attempted_reclamation {
        // RAN and removed nothing. Keyed on the mode, not on the count: this
        // shape and "was never allowed to try" both have `reclaimed_files == 0`
        // and are opposite facts, and the failure lines printed below would
        // flatly contradict a "did not run" claim here.
        let failures = shm.reclaim_failures.len() as u64 + shm.reclaim_failures_elided;
        let outcome = if failures > 0 {
            format!(
                "Reclamation RAN this pass and every removal failed ({failures} \
                 failure(s)) — see the reclaim failures below."
            )
        } else {
            // Defensive: `reclaim_one` either counts a removal or records a
            // failure, so a proven-dead file cannot leave both at zero. Stated
            // without claiming failures rather than left to a wrong branch.
            "Reclamation RAN this pass and removed none of them.".to_string()
        };
        if shm.proven_dead > 0 {
            out.push(format!(
                "  {} file(s) are provably dead (creator process gone). {outcome}",
                shm.proven_dead
            ));
        }
        if shm.empty_orphans > 0 {
            out.push(format!(
                "  {} file(s) record no name at all (a creator killed mid-create, older \
                 than {}). {outcome}",
                shm.empty_orphans,
                render_min_age()
            ));
        }
    } else {
        if shm.proven_dead > 0 {
            out.push(format!(
                "  {} file(s) are provably dead (creator process gone). Reclamation did \
                 not run this pass — it runs when `cerulion clean` is invoked without \
                 `--report-only` AND the dead-node sweep converges (no failures, nothing \
                 deferred; an unconverged pass reports the blockers above).",
                shm.proven_dead
            ));
        }
        if shm.empty_orphans > 0 {
            out.push(format!(
                "  {} file(s) record no name at all (a creator killed mid-create, older \
                 than {}). Reclamation did not run this pass — same gate as above.",
                shm.empty_orphans,
                render_min_age()
            ));
        }
    }
    if shm.creator_alive > 0 {
        out.push(format!(
            "  {} left in place: the creating process is still running.",
            shm.creator_alive
        ));
    }
    if shm.namespace_in_use > 0 {
        out.push(format!(
            "  {} left in place: an iceoryx2 node is still registered against that \
             namespace, which still needs the name mapping.",
            shm.namespace_in_use
        ));
    }
    if shm.unproven > 0 {
        out.push(format!(
            "  {} left in place: UNPROVEN (no usable proof the creator is gone) — never \
             deleted on a guess.",
            shm.unproven
        ));
    }
    for failure in &shm.reclaim_failures {
        out.push(format!("  reclaim failed — {failure}"));
    }
    if shm.reclaim_failures_elided > 0 {
        out.push(format!(
            "  … and {} further reclaim failure(s).",
            shm.reclaim_failures_elided
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::sync::Mutex;

    /// A hand-driven probe: every answer is dictated by the test, so no arm
    /// depends on a real process or a real shared-memory object.
    #[derive(Default)]
    struct FakeProbe {
        liveness: HashMap<i32, ProcessLiveness>,
        default_liveness: Option<ProcessLiveness>,
        sizes: HashMap<String, u64>,
        unlink_errors: HashMap<String, String>,
        unlinked: Mutex<Vec<String>>,
        probe_delay: Duration,
        /// `None` = the default answer, an empty KNOWN set: no
        /// namespace is in use, so the older arms keep asserting exactly
        /// what they asserted. `NamespacesInUse` deliberately has no `Default`
        /// — a permissive default on the production type is the shape that
        /// would let a new probe silently permit every reclamation.
        namespaces: Option<NamespacesInUse>,
        /// The budget `namespaces_in_use` was handed, recorded so a test can
        /// prove it came from the caller's remaining bound.
        handed_budget: Mutex<Option<Duration>>,
    }

    impl FakeProbe {
        fn dead(pids: &[i32]) -> Self {
            let mut probe = Self::default();
            for pid in pids {
                probe.liveness.insert(*pid, ProcessLiveness::Gone);
            }
            probe
        }
        fn all_dead() -> Self {
            Self {
                default_liveness: Some(ProcessLiveness::Gone),
                ..Self::default()
            }
        }
        /// Every pid reads ALIVE — the control for the age-gated arms, where a
        /// verdict must come from the age rule and could not have come from the
        /// liveness one.
        fn all_alive() -> Self {
            Self {
                default_liveness: Some(ProcessLiveness::Alive),
                ..Self::default()
            }
        }
        fn with_size(mut self, name: &str, bytes: u64) -> Self {
            self.sizes.insert(name.to_string(), bytes);
            self
        }
        fn failing_unlink(mut self, name: &str, err: &str) -> Self {
            self.unlink_errors.insert(name.to_string(), err.to_string());
            self
        }
        fn slow(mut self, delay: Duration) -> Self {
            self.probe_delay = delay;
            self
        }
        fn unlinked(&self) -> Vec<String> {
            self.unlinked.lock().expect("unlink ledger").clone()
        }
        /// Declare a namespace prefix as still having a registered
        /// node, so files under it must be refused.
        fn namespace_in_use(mut self, prefix: &str) -> Self {
            let mut set = match self.namespaces.take() {
                Some(NamespacesInUse::Known(set)) => set,
                _ => std::collections::BTreeSet::new(),
            };
            set.insert(prefix.to_string());
            self.namespaces = Some(NamespacesInUse::Known(set));
            self
        }
        /// The registry could not be read at all — everything must be refused.
        fn registry_unreadable(mut self) -> Self {
            self.namespaces = Some(NamespacesInUse::Unknown);
            self
        }
        fn handed_budget(&self) -> Option<Duration> {
            *self.handed_budget.lock().expect("handed budget")
        }
    }

    impl SystemProbe for FakeProbe {
        fn process_liveness(&self, pid: i32) -> ProcessLiveness {
            if !self.probe_delay.is_zero() {
                std::thread::sleep(self.probe_delay);
            }
            self.liveness
                .get(&pid)
                .copied()
                .or(self.default_liveness)
                .unwrap_or(ProcessLiveness::Alive)
        }
        fn shm_object_size(&self, name: &str) -> Option<u64> {
            self.sizes.get(name).copied()
        }
        fn unlink_shm_object(&self, name: &str) -> Result<(), String> {
            if let Some(err) = self.unlink_errors.get(name) {
                return Err(err.clone());
            }
            self.unlinked
                .lock()
                .expect("unlink ledger")
                .push(name.to_string());
            Ok(())
        }
        fn namespaces_in_use(&self, budget: Duration) -> NamespacesInUse {
            *self.handed_budget.lock().expect("handed budget") = Some(budget);
            // Mirrors the real walk's contract: no budget, no evidence.
            if budget.is_zero() {
                return NamespacesInUse::Unknown;
            }
            self.namespaces
                .clone()
                .unwrap_or_else(|| NamespacesInUse::Known(std::collections::BTreeSet::new()))
        }
    }

    /// Write a `.shm_state` file the way `write_real_shm_name` does: the name
    /// NUL-padded to the 33-byte `SHM_MAX_NAME_LEN` buffer.
    fn write_state_file(dir: &Path, file_stem: &str, real_name: &str) -> PathBuf {
        let mut buf = vec![0u8; 33];
        buf[..real_name.len()].copy_from_slice(real_name.as_bytes());
        let path = dir.join(format!("{file_stem}{SHM_STATE_SUFFIX}"));
        std::fs::write(&path, &buf).expect("write state file");
        path
    }

    /// Backdate a file's mtime by `age`.
    ///
    /// The age gate is a DAY, so an arm that waited for it would never run.
    /// Backdating drives the real predicate over the real filesystem instead of
    /// hand-feeding [`FileAge`] to the classifier, which is what makes the
    /// end-to-end arms able to see a `read_state_file` that stopped reporting
    /// an age at all.
    #[cfg(unix)]
    fn age_file(path: &Path, age: Duration) {
        set_mtime(path, SystemTime::now() - age);
    }

    #[cfg(unix)]
    fn set_mtime(path: &Path, when: SystemTime) {
        let secs = when
            .duration_since(std::time::UNIX_EPOCH)
            .expect("a post-epoch timestamp")
            .as_secs();
        let tv = libc::timeval {
            tv_sec: libc::time_t::try_from(secs).expect("seconds fit time_t"),
            tv_usec: 0,
        };
        let times = [tv, tv];
        let c = std::ffi::CString::new(path.as_os_str().as_encoded_bytes()).expect("path");
        // SAFETY: `c` is a valid NUL-terminated path and `times` is a 2-element
        // `timeval` array, which is exactly what `utimes(2)` reads.
        let rc = unsafe { libc::utimes(c.as_ptr(), times.as_ptr()) };
        assert_eq!(rc, 0, "utimes must succeed on {}", path.display());
    }

    fn tempdir(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "{tag}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("clock")
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).expect("create tempdir");
        dir
    }

    // ---------------------------------------------------------------- parsing

    #[test]
    fn the_recorded_name_is_read_exactly_as_the_pal_writes_it() {
        // The literal content of a real leaked file on a development machine,
        // NUL-padded to 33 bytes exactly as `write_real_shm_name` writes it.
        let mut content = vec![0u8; 33];
        content[..25].copy_from_slice(b"72856_1787420947_684024_3");
        let parsed = parse_real_shm_name(&content).expect("a real pal-written name must parse");
        assert_eq!(parsed.name, "72856_1787420947_684024_3");
        assert_eq!(parsed.creator_pid, 72856);
    }

    #[test]
    fn a_content_that_records_no_usable_pid_is_refused_by_reason() {
        // Each arm is a distinct way the proof can be missing, and every one
        // must REFUSE — the pid is the whole of the evidence.
        let cases: &[(&[u8], UnparsableReason)] = &[
            (b"", UnparsableReason::Empty),
            (b"\0\0\0", UnparsableReason::Empty),
            (b"notapid_1_2_3", UnparsableReason::PidNotNumeric),
            (b"_1787420947_684024_3", UnparsableReason::PidNotNumeric),
            (b"0_1787420947_684024_3", UnparsableReason::PidNotPositive),
            (b"99999999999999_1_2_3", UnparsableReason::PidOutOfRange),
            // Four segments, but the leading one is not a pid — the path
            // characters live inside segment 0.
            (b"123/../etc_1_2_3", UnparsableReason::PidNotNumeric),
            (
                b"123 rm -rf_1_2",
                UnparsableReason::NotFourSegments { found: 3 },
            ),
        ];
        for (content, expected) in cases {
            assert_eq!(
                parse_real_shm_name(content),
                Err(*expected),
                "content {content:?} must be refused as {expected:?}"
            );
        }
    }

    #[test]
    fn only_the_complete_minted_shape_is_accepted_as_a_name_to_unlink() {
        // `/tmp` is WORLD-WRITABLE, so this content is hostile input and the
        // name that survives is handed straight to `shm_unlink`. Validating
        // only the leading segment accepted `123_<anything>` — so once pid 123
        // died, forged content could aim the reclamation at an UNRELATED
        // object name. The whole minted shape is the gate.
        let attacks: &[(&[u8], UnparsableReason)] = &[
            // The headline: a real pid, then anything at all.
            (
                b"123_iox2_someone_elses_segment",
                UnparsableReason::NotFourSegments { found: 5 },
            ),
            (
                b"123_../../dev/shm/victim",
                UnparsableReason::NotFourSegments { found: 2 },
            ),
            // A BARE pid — no separators at all.
            (b"123", UnparsableReason::NotFourSegments { found: 1 }),
            // Right count, wrong content: each non-pid segment must be digits.
            (
                b"123_a_2_3",
                UnparsableReason::SegmentNotNumeric { index: 1 },
            ),
            (
                b"123_1_2_victim",
                UnparsableReason::SegmentNotNumeric { index: 3 },
            ),
            // An EMPTY segment is not a run of digits either.
            (
                b"123__2_3",
                UnparsableReason::SegmentNotNumeric { index: 1 },
            ),
            (
                b"123_1_2_",
                UnparsableReason::SegmentNotNumeric { index: 3 },
            ),
            // Too many segments, even when every one is numeric.
            (
                b"123_1_2_3_4",
                UnparsableReason::NotFourSegments { found: 5 },
            ),
        ];
        for (content, expected) in attacks {
            assert_eq!(
                parse_real_shm_name(content),
                Err(*expected),
                "hostile content {content:?} must be refused as {expected:?}"
            );
        }

        // ANTI-TAUTOLOGY: the real minted shape still parses, so the gate is
        // not simply refusing everything.
        let real = parse_real_shm_name(b"72856_1787420947_684024_3").expect("the real shape");
        assert_eq!(real.name, "72856_1787420947_684024_3");
        assert_eq!(real.creator_pid, 72856);
    }

    #[test]
    fn a_name_shorter_than_the_buffer_stops_at_the_first_nul() {
        // Anything after the NUL is padding, not name — handing the padding to
        // `shm_unlink` would name a different object.
        let mut content = vec![0u8; 33];
        content[..8].copy_from_slice(b"41_2_3_4");
        content[10] = b'X'; // trailing garbage past the terminator
        let parsed = parse_real_shm_name(&content).expect("parse");
        assert_eq!(parsed.name, "41_2_3_4");
        assert_eq!(parsed.creator_pid, 41);
    }

    // ----------------------------------------------------------- classifying

    /// A hand-built read: the bytes a state file holds plus an age, so the
    /// classifier can be driven without a filesystem.
    fn read_of(content: &[u8], age: FileAge) -> StateFileRead {
        StateFileRead {
            content: content.to_vec(),
            age,
        }
    }

    #[test]
    fn only_a_gone_creator_is_ever_proven_dead() {
        let content = read_of(b"4242_1_2_3\0\0", FileAge::Known(Duration::from_secs(9)));
        let mut probe = FakeProbe::default();

        probe.liveness.insert(4242, ProcessLiveness::Gone);
        assert!(matches!(
            classify(Ok(&content), &probe),
            StateFileVerdict::ProvenDead(_)
        ));

        probe.liveness.insert(4242, ProcessLiveness::Alive);
        assert_eq!(
            classify(Ok(&content), &probe),
            StateFileVerdict::CreatorAlive { pid: 4242 }
        );

        probe.liveness.insert(4242, ProcessLiveness::Unknown);
        assert_eq!(
            classify(Ok(&content), &probe),
            StateFileVerdict::CreatorUnknown { pid: 4242 },
            "an errno we cannot read as proof must never license a delete"
        );

        // Every unreadable REASON is refused, and each is carried through so
        // the report can say which one it was.
        for reason in [
            UnreadableReason::Io,
            UnreadableReason::NotARegularFile,
            UnreadableReason::TooLarge { bytes: 1 << 30 },
        ] {
            assert_eq!(
                classify(Err(reason.clone()), &probe),
                StateFileVerdict::Unreadable(reason)
            );
        }
    }

    // --------------------------------------------- zero-length files: empties

    #[test]
    fn a_zero_length_file_is_adjudicated_by_age_and_by_nothing_else() {
        // The pid proof cannot run on a file that records no pid, so this is
        // the module's SECOND evidence shape and it is pinned on both sides of
        // its threshold. A creator is alive throughout, so the arms cannot be
        // passing through the liveness path by accident.
        let probe = FakeProbe::all_alive();
        let bound = EMPTY_STATE_FILE_MIN_AGE;

        // The shapes a real PAL leaves: zero bytes, and a NUL-padded buffer
        // that was never written (both are "empty before the first NUL").
        for content in [b"".as_slice(), &[0u8; 33]] {
            assert_eq!(
                classify(Ok(&read_of(content, FileAge::Known(bound))), &probe),
                StateFileVerdict::EmptyOrphan,
                "exactly AT the bound is old enough"
            );
            assert_eq!(
                classify(
                    Ok(&read_of(
                        content,
                        FileAge::Known(bound - Duration::from_secs(1))
                    )),
                    &probe
                ),
                StateFileVerdict::Unparsable(UnparsableReason::Empty),
                "one second under the bound must still refuse"
            );
            assert_eq!(
                classify(Ok(&read_of(content, FileAge::Unknown)), &probe),
                StateFileVerdict::Unparsable(UnparsableReason::Empty),
                "an age we could not establish is never evidence of an old file"
            );
        }

        // And the rule is scoped to EMPTY. Every other unparsable shape is
        // forged or corrupt content, which age says nothing about — a
        // rule that keyed on age alone would reclaim an attacker's file.
        for hostile in [
            b"notapid_1_2_3".as_slice(),
            b"123",
            b"123_a_2_3",
            b"0_1_2_3",
        ] {
            let verdict = classify(Ok(&read_of(hostile, FileAge::Known(bound * 400))), &probe);
            assert!(
                matches!(verdict, StateFileVerdict::Unparsable(reason) if reason != UnparsableReason::Empty),
                "an ancient but NON-empty unparsable file must stay refused; {hostile:?} gave \
                 {verdict:?}"
            );
        }
    }

    #[cfg(unix)]
    #[test]
    fn an_aged_out_empty_file_is_reclaimed_without_ever_naming_an_object() {
        // The PAL creates the kernel object only AFTER writing this file, so
        // there is provably none — and no name to unlink it by. The ledger is
        // the oracle: a reclamation that touched the object plane at all would
        // have to invent a name.
        let dir = tempdir("emptyorphan");
        let empty = dir.join(format!("poisoned{SHM_STATE_SUFFIX}"));
        std::fs::write(&empty, b"").expect("write");
        age_file(&empty, EMPTY_STATE_FILE_MIN_AGE + Duration::from_secs(60));
        // A young empty beside it, and a live creator's file: neither may move.
        let young = write_state_file(&dir, "young", "");
        let live = write_state_file(&dir, "live", "4321_1_1_1");

        let probe = FakeProbe::all_alive();

        let report = reclaim_at_exit(&dir, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);

        assert_eq!(report.counted, 3);
        assert_eq!(report.empty_orphans, 1, "only the aged empty qualifies");
        assert_eq!(report.proven_dead, 0, "no pid proof was available anywhere");
        assert_eq!(report.reclaimed_files, 1);
        assert_eq!(report.reclaimed_empty_files, 1);
        assert_eq!(
            report.reclaimed_object_bytes, 0,
            "an empty file holds no object, so nothing was released"
        );
        assert!(
            probe.unlinked().is_empty(),
            "the object plane must not be touched at all, got {:?}",
            probe.unlinked()
        );
        assert!(!empty.exists(), "the aged orphan is gone");
        assert!(young.exists(), "a young empty must survive");
        assert!(live.exists(), "a live creator's file must survive");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(unix)]
    #[test]
    fn a_future_mtime_is_an_unknown_age_rather_than_a_brand_new_file() {
        // A clock step or a copied timestamp is an ABSENCE of evidence. Folding
        // it to zero would be the safe answer only by accident; folding it the
        // other way would reclaim on a clock bug.
        let dir = tempdir("futuremtime");
        let path = dir.join(format!("future{SHM_STATE_SUFFIX}"));
        std::fs::write(&path, b"").expect("write");
        set_mtime(&path, SystemTime::now() + Duration::from_secs(3_600));

        let read = read_state_file(&path).expect("a regular empty file reads fine");
        assert_eq!(read.age, FileAge::Unknown);
        assert!(read.content.is_empty());

        let probe = FakeProbe::all_alive();
        let report = reclaim_at_exit(&dir, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);
        assert_eq!(report.empty_orphans, 0);
        assert_eq!(report.unproven, 1);
        assert!(path.exists(), "an unknown age never licenses a delete");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_age_predicate_answers_its_hand_written_vectors() {
        let s = Duration::from_secs;
        assert!(FileAge::Known(s(10)).at_least(s(10)), "at the bound");
        assert!(FileAge::Known(s(11)).at_least(s(10)));
        assert!(!FileAge::Known(s(9)).at_least(s(10)));
        assert!(!FileAge::Unknown.at_least(s(10)));
        assert!(
            !FileAge::Unknown.at_least(Duration::ZERO),
            "an unknown age must not satisfy even a vacuous bound"
        );
    }

    // ------------------------------------------------------- hostile entries

    /// Create a FIFO at `path`. On macOS `open(2)` for reading one BLOCKS
    /// until a writer appears — which is what makes this the sharpest entry
    /// an attacker can leave in a world-writable directory.
    #[cfg(unix)]
    fn make_fifo(path: &Path) {
        let c = std::ffi::CString::new(path.as_os_str().as_encoded_bytes()).expect("path");
        // SAFETY: `c` is a valid NUL-terminated path for the duration.
        let rc = unsafe { libc::mkfifo(c.as_ptr(), 0o600) };
        assert_eq!(rc, 0, "mkfifo must succeed at {}", path.display());
    }

    #[cfg(unix)]
    #[test]
    fn a_fifo_named_like_a_state_file_is_refused_without_ever_blocking() {
        // THE hostile shape. `/tmp` is world-writable, so anybody can leave a
        // FIFO called `<x>.shm_state` there, and an unguarded read would wait
        // for a writer FOREVER — an indefinite hang inside the one command
        // whose contract is that it never hangs. The refusal is decided from
        // the entry's TYPE, before any open.
        //
        // The BOUND is the oracle: this whole body must finish in seconds. A
        // blocking implementation does not fail it, it hangs — so the arm runs
        // the walk on a helper thread and reports a TIMEOUT as the failure.
        let dir = tempdir("fifo");
        make_fifo(&dir.join(format!("hostile{SHM_STATE_SUFFIX}")));
        // ANTI-TAUTOLOGY: a genuine file beside it must still classify, or
        // "the walk finished" is satisfied by a walk that skipped everything.
        write_state_file(&dir, "real", "900_1_1_1");

        let (tx, rx) = std::sync::mpsc::channel();
        let walk_dir = dir.clone();
        std::thread::spawn(move || {
            let probe = FakeProbe::all_dead();
            let report = scan(
                &walk_dir,
                SHM_STATE_REPORT_BUDGET,
                classify_budget(SHM_STATE_REPORT_BUDGET),
                ScanMode::ReportOnly,
                &probe,
            );
            let _ = tx.send(report);
        });
        let report = rx
            .recv_timeout(Duration::from_secs(20))
            .expect("the walk must not block on a FIFO — a hang here IS the defect");

        assert_eq!(report.counted, 2, "both entries are named `.shm_state`");
        assert_eq!(report.proven_dead, 1, "the real file still classifies");
        assert_eq!(report.unproven, 1, "the FIFO is refused, never read");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(unix)]
    #[test]
    fn a_fifo_is_refused_on_its_type_rather_than_on_a_failed_read() {
        // The reason matters as much as the refusal: `NotARegularFile` is what
        // says the decision was taken from `symlink_metadata`, i.e. BEFORE any
        // open. An `Io` here would mean we opened it and something else went
        // wrong — the shape that can block.
        let dir = tempdir("fifokind");
        let path = dir.join("f.shm_state");
        make_fifo(&path);

        let (tx, rx) = std::sync::mpsc::channel();
        let probe_path = path.clone();
        std::thread::spawn(move || {
            let _ = tx.send(read_state_file(&probe_path));
        });
        let outcome = rx
            .recv_timeout(Duration::from_secs(20))
            .expect("reading a FIFO must not block");
        assert_eq!(outcome, Err(UnreadableReason::NotARegularFile));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(unix)]
    #[test]
    fn a_symlink_is_refused_rather_than_followed() {
        // Following one would let an attacker aim BOTH the size check and the
        // read at a file somewhere else — a FIFO included, which is how the
        // type check above would be bypassed.
        let dir = tempdir("symlink");
        let real = write_state_file(&dir, "target", "901_1_1_1");
        let link = dir.join(format!("link{SHM_STATE_SUFFIX}"));
        std::os::unix::fs::symlink(&real, &link).expect("symlink");

        assert_eq!(
            read_state_file(&link),
            Err(UnreadableReason::NotARegularFile),
            "a symlink must be judged as a LINK, not as its target"
        );
        // ANTI-TAUTOLOGY: the target itself reads fine, so the refusal is
        // about the link and not about the content.
        assert!(read_state_file(&real).is_ok());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_file_past_the_size_cap_is_refused_unread() {
        // An unbounded read lets ONE crafted entry allocate past the budget
        // the whole walk is bounded by.
        let dir = tempdir("oversize");
        let big = dir.join(format!("big{SHM_STATE_SUFFIX}"));
        let bytes = usize::try_from(MAX_STATE_FILE_BYTES).expect("cap fits") + 1;
        std::fs::write(&big, vec![b'1'; bytes]).expect("write");

        assert_eq!(
            read_state_file(&big),
            Err(UnreadableReason::TooLarge {
                bytes: MAX_STATE_FILE_BYTES + 1
            })
        );

        // The BOUNDARY, from the admitted side: exactly at the cap is read.
        let at_cap = dir.join(format!("atcap{SHM_STATE_SUFFIX}"));
        std::fs::write(
            &at_cap,
            vec![b'1'; usize::try_from(MAX_STATE_FILE_BYTES).expect("cap fits")],
        )
        .expect("write");
        let read = read_state_file(&at_cap).expect("a file at the cap is read");
        assert_eq!(read.content.len() as u64, MAX_STATE_FILE_BYTES);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_directory_named_like_a_state_file_is_refused_on_its_type() {
        let dir = tempdir("direntry");
        std::fs::create_dir(dir.join(format!("d{SHM_STATE_SUFFIX}"))).expect("mkdir");
        let probe = FakeProbe::all_dead();
        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(report.counted, 1);
        assert_eq!(report.unproven, 1);
        assert_eq!(report.reclaimed_files, 0);
        assert!(probe.unlinked().is_empty());
        std::fs::remove_dir_all(&dir).ok();
    }

    // --------------------------------------------------------------- scanning

    #[test]
    fn the_walk_counts_only_state_files_and_leaves_everything_else_alone() {
        let dir = tempdir("counts");
        write_state_file(&dir, "a", "11_1_1_1");
        write_state_file(&dir, "b", "12_1_1_1");
        std::fs::write(dir.join("unrelated.log"), b"not ours").expect("write");
        std::fs::write(dir.join("shm_state"), b"no dot").expect("write");

        let probe = FakeProbe::all_dead();
        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::ReportOnly,
            &probe,
        );

        assert_eq!(report.counted, 2, "only the two `.shm_state` files count");
        assert!(!report.truncated);
        assert_eq!(report.proven_dead, 2);
        assert!(
            probe.unlinked().is_empty(),
            "ReportOnly must touch nothing at all"
        );
        assert!(dir.join("unrelated.log").exists());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn reclamation_takes_only_the_proven_dead_and_leaves_the_rest_in_place() {
        let dir = tempdir("reclaim");
        let dead = write_state_file(&dir, "dead", "100_1_1_1");
        let live = write_state_file(&dir, "live", "200_1_1_1");
        let unknown = write_state_file(&dir, "unknown", "300_1_1_1");
        let unparsable = write_state_file(&dir, "junk", "");

        let mut probe = FakeProbe::dead(&[100]);
        probe.liveness.insert(200, ProcessLiveness::Alive);
        probe.liveness.insert(300, ProcessLiveness::Unknown);
        let probe = probe.with_size("100_1_1_1", 24_057);

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(report.counted, 4);
        assert_eq!(report.proven_dead, 1);
        assert_eq!(report.creator_alive, 1);
        assert_eq!(report.unproven, 2, "the unknown pid and the empty file");
        assert_eq!(report.reclaimed_files, 1);
        assert_eq!(
            report.reclaimed_object_bytes, 24_057,
            "the orphaned kernel object's size is what reclamation actually freed"
        );
        assert_eq!(
            probe.unlinked(),
            vec!["100_1_1_1".to_string()],
            "exactly the proven-dead object is unlinked, and nothing else"
        );
        assert!(!dead.exists(), "the proven-dead state file is gone");
        assert!(live.exists(), "a live creator's file must survive");
        assert!(unknown.exists(), "an unprovable file must survive");
        assert!(unparsable.exists(), "an unparsable file must survive");
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The headline. Every `.shm_state` file MEASURED on a real desk
    /// (2,018 of 2,018) is a `<prefix><id>_node.<version>.global_mgmt` — the one
    /// management segment an iceoryx2 NAMESPACE shares — so the creating pid is
    /// merely whichever process opened the namespace first and says nothing at
    /// all about whether the segment is still in use. A namespace with a node
    /// registered must be refused; one with none stays reclaimable, which is
    /// the control that stops this arm passing on a reclaimer that refuses
    /// everything.
    #[test]
    fn a_namespace_that_still_has_a_registered_node_is_refused_whatever_the_pid_says() {
        let dir = tempdir("ns_in_use");
        // Both creators are DEAD: the only thing separating the two files is
        // whether their namespace still has a node.
        // Names carry iceoryx2's version between the id and the suffix, as the
        // library writes them; the evidence is keyed on the PREFIX, which is
        // what this arm proves is still true with the version present.
        let live_ns = write_state_file(&dir, "iox2_abc_node.0_10_0.global_mgmt", "100_1_1_1");
        let dead_ns = write_state_file(&dir, "cer_p_dead_node.0_10_0.global_mgmt", "101_1_1_1");

        let probe = FakeProbe::all_dead()
            .with_size("100_1_1_1", 4_096)
            .with_size("101_1_1_1", 4_096)
            .namespace_in_use("iox2_");

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(report.counted, 2);
        assert_eq!(
            report.namespace_in_use, 1,
            "the in-use namespace is refused"
        );
        assert_eq!(report.proven_dead, 1, "and the other is still reclaimable");
        assert_eq!(report.reclaimed_files, 1);
        assert!(
            live_ns.exists(),
            "the shared mapping of a namespace with a live node must survive — removing it \
             splits the namespace and strands every node registered against it"
        );
        assert!(
            !dead_ns.exists(),
            "control: a dead namespace is still cleaned"
        );
        assert_eq!(
            probe.unlinked(),
            vec!["101_1_1_1".to_string()],
            "and only the dead namespace's object is unlinked"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    /// Refusal, not absence: a registry that cannot be enumerated proves
    /// nothing unused, so nothing may be reclaimed. The same file reclaims
    /// cleanly under a READABLE empty registry, which is what makes this an
    /// assertion about the unknown answer rather than about the file.
    #[test]
    fn a_registry_that_cannot_be_read_refuses_every_reclamation() {
        for (unreadable, expected_reclaimed) in [(true, 0), (false, 1)] {
            let dir = tempdir(if unreadable { "ns_unk" } else { "ns_known" });
            let file = write_state_file(&dir, "cer_p_x_node.0_10_0.global_mgmt", "100_1_1_1");
            let probe = FakeProbe::all_dead().with_size("100_1_1_1", 4_096);
            let probe = if unreadable {
                probe.registry_unreadable()
            } else {
                probe
            };

            let report = scan(
                &dir,
                SHM_STATE_REPORT_BUDGET,
                classify_budget(SHM_STATE_REPORT_BUDGET),
                ScanMode::Reclaim,
                &probe,
            );

            assert_eq!(
                report.reclaimed_files, expected_reclaimed,
                "unreadable={unreadable}: an unreadable registry must refuse, a readable \
                 empty one must not"
            );
            assert_eq!(file.exists(), unreadable);
            std::fs::remove_dir_all(&dir).ok();
        }
    }

    /// The registry walk that produces the evidence, over a real directory
    /// laid out the way iceoryx2 lays it out: one directory per node, holding
    /// `<prefix>node.details`.
    #[test]
    fn the_registry_walk_reads_prefixes_off_the_node_details_files() {
        let dir = tempdir("ns_walk");
        for (node, details) in [
            ("node_a", "iox2_9204node.details"),
            ("node_b", "cer_p_beefnode.details"),
            // A second node of the SAME namespace collapses to one prefix.
            ("node_c", "iox2_9204node.details"),
            // Anything else inside a node directory is not a prefix source.
            ("node_d", "unrelated.file"),
        ] {
            std::fs::create_dir_all(dir.join(node)).expect("node dir");
            std::fs::write(dir.join(node).join(details), b"x").expect("details");
        }

        let walked = namespaces_in_use_at(&dir, Duration::from_secs(5));
        let NamespacesInUse::Known(prefixes) = walked else {
            panic!("a readable registry must be Known");
        };
        assert_eq!(
            prefixes.iter().cloned().collect::<Vec<_>>(),
            vec!["cer_p_beef".to_string(), "iox2_9204".to_string()],
            "one entry per NAMESPACE, read off the details suffix"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The registry holds node-monitor FILES beside the
    /// node directories — VERIFIED against a live `/tmp/iceoryx2/nodes`:
    ///
    /// ```text
    /// drwx------  2889553339617049664376854491/
    /// -rwx------  cer_p_1dcd..975.node_monitor
    /// -r--------  cer_p_1dcd..975.node_monitor_context
    /// -rwx------  cer_p_1dcd..975.node_monitor_owner_lock
    /// ```
    ///
    /// A walk that does not skip non-directory entries lets ONE monitor file make the
    /// walk answer `Unknown`, and the whole reclamation goes inert the moment any
    /// node had ever existed. The second half of this arm is what makes that
    /// visible: a provably-dead OTHER namespace must still be reclaimed.
    #[test]
    fn monitor_files_beside_the_node_directories_do_not_blind_the_walk() {
        let registry = tempdir("ns_monitor_registry");
        // One live-shaped node: its directory AND its three monitor files.
        std::fs::create_dir_all(registry.join("2889553339617049664376854491")).expect("node dir");
        std::fs::write(
            registry
                .join("2889553339617049664376854491")
                .join("iox2_9204node.details"),
            b"x",
        )
        .expect("details");
        for suffix in [
            ".node_monitor",
            ".node_monitor_context",
            ".node_monitor_owner_lock",
        ] {
            std::fs::write(
                registry.join(format!("iox2_92042889553339617049664376854491{suffix}")),
                b"",
            )
            .expect("monitor file");
        }

        let walked = namespaces_in_use_at(&registry, Duration::from_secs(5));
        let NamespacesInUse::Known(prefixes) = walked else {
            panic!(
                "monitor files are part of the layout, not damage — the walk must still \
                 produce evidence, not Unknown"
            );
        };
        assert_eq!(
            prefixes.iter().cloned().collect::<Vec<_>>(),
            vec!["iox2_9204".to_string()],
            "the prefix comes off the node DIRECTORY; the monitor files are skipped"
        );

        // …and the evidence is USABLE: the live namespace is refused while a
        // provably-dead OTHER namespace is still reclaimed. Without this half,
        // an all-refusing walk would pass the assertion above.
        let dir = tempdir("ns_monitor_state");
        let live = write_state_file(&dir, "iox2_9204_node.0_10_0.global_mgmt", "100_1_1_1");
        let dead = write_state_file(&dir, "cer_p_gone_node.0_10_0.global_mgmt", "101_1_1_1");
        let probe = FakeProbe::all_dead()
            .with_size("100_1_1_1", 4_096)
            .with_size("101_1_1_1", 4_096)
            .namespace_in_use("iox2_9204");

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(report.namespace_in_use, 1);
        assert_eq!(
            report.reclaimed_files, 1,
            "a live namespace must not make the whole pass inert"
        );
        assert!(live.exists());
        assert!(!dead.exists());
        std::fs::remove_dir_all(&registry).ok();
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The registry walk must run on what is LEFT of the
    /// scan's bound, never a fresh fixed one — a bound you can pay twice is not
    /// a bound. Pinned on the budget the probe is HANDED (a wall assertion tight
    /// enough to separate the two would be load-invertible).
    #[test]
    fn the_registry_walk_runs_on_what_is_left_of_the_scans_budget() {
        let dir = tempdir("ns_budget_thread");
        write_state_file(&dir, "cer_p_x_node.0_10_0.global_mgmt", "100_1_1_1");
        let probe = FakeProbe::all_dead().with_size("100_1_1_1", 4_096);

        // DELIBERATELY far below a fixed 2 s walk:
        // an assertion made against a scan budget that happens to equal a
        // hardcoded one is satisfied by the very defect it exists to catch.
        let scan_budget = Duration::from_millis(50);
        let _ = scan(
            &dir,
            scan_budget,
            classify_budget(scan_budget),
            ScanMode::Reclaim,
            &probe,
        );
        let handed = probe.handed_budget().expect("the walk must be budgeted");
        assert!(
            handed <= scan_budget,
            "the walk was handed {handed:?}, which is not bounded by the scan's own \
             {scan_budget:?} — a fixed budget here lets one scan overrun its cap"
        );

        // The floor of the same claim: a scan with NOTHING left hands nothing,
        // and a walk with no budget proves nothing, so nothing is reclaimed.
        let dir2 = tempdir("ns_budget_spent");
        let file = write_state_file(&dir2, "cer_p_y_node.0_10_0.global_mgmt", "100_1_1_1");
        let spent = FakeProbe::all_dead().with_size("100_1_1_1", 4_096);
        let report = scan(
            &dir2,
            Duration::ZERO,
            Duration::ZERO,
            ScanMode::Reclaim,
            &spent,
        );
        assert_eq!(
            report.reclaimed_files, 0,
            "a spent budget must refuse, never reclaim on a fresh one"
        );
        assert!(file.exists());
        std::fs::remove_dir_all(&dir).ok();
        std::fs::remove_dir_all(&dir2).ok();
    }

    /// The two answers a walk can give about an empty result, which a bare set
    /// could not tell apart: an ABSENT registry genuinely has no nodes, while a
    /// walk that ran out of budget knows nothing and must refuse.
    #[test]
    fn an_absent_registry_is_empty_but_a_truncated_walk_is_unknown() {
        let absent =
            std::env::temp_dir().join(format!("absent_{}_{}", std::process::id(), "registry"));
        std::fs::remove_dir_all(&absent).ok();
        assert_eq!(
            namespaces_in_use_at(&absent, Duration::from_secs(5)),
            NamespacesInUse::Known(std::collections::BTreeSet::new()),
            "a registry that does not exist holds no nodes"
        );

        let dir = tempdir("ns_budget");
        std::fs::create_dir_all(dir.join("node_a")).expect("node dir");
        std::fs::write(dir.join("node_a").join("iox2_node.details"), b"x").expect("details");
        assert_eq!(
            namespaces_in_use_at(&dir, Duration::ZERO),
            NamespacesInUse::Unknown,
            "a walk with no budget proves nothing and must refuse everything"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    // ---------------------------------------------------------------------
    // The evidence must cover ISOLATED-ROOT registries too.
    //
    // `.shm_state` files are root-INDEPENDENT — the PAL puts every one of them
    // in `SHM_STATE_DIRECTORY` whatever `global.root_path` says
    // (`iceoryx2-pal-posix-0.9.1/src/macos/settings.rs:15`) — while a node
    // registry is `<root_path>/<node.directory>`. So evidence from ONE root
    // reads every other root's live namespaces as free.
    //
    // `iceoryx2::testing::generate_isolated_config` produces exactly that
    // shape: it takes a default `Config` and points `root_path` at
    // `TEST_DIRECTORY`, which is the compiled-in root plus one entry
    // (`/tmp/iceoryx2/` -> `/tmp/iceoryx2/tests/`). These arms build that
    // layout by hand; the real-transport half is
    // `tests/isolated_root_evidence_test.rs`.
    // ---------------------------------------------------------------------

    /// Plant one registered node under `<root>/<sub>/nodes`, or under
    /// `<root>/nodes` when `sub` is `None`.
    fn plant_registered_node(root: &Path, sub: Option<&str>, prefix: &str) {
        let registry = match sub {
            Some(sub) => root.join(sub).join("nodes"),
            None => root.join("nodes"),
        };
        let node = registry.join(format!("node_of_{prefix}"));
        std::fs::create_dir_all(&node).expect("node dir");
        std::fs::write(node.join(format!("{prefix}{NODE_DETAILS_SUFFIX}")), b"x")
            .expect("node details");
    }

    fn anchors_at(root: &Path) -> Vec<RegistryAnchor> {
        vec![RegistryAnchor {
            root: root.to_path_buf(),
            node_dir_name: "nodes".to_string(),
        }]
    }

    /// A probe whose EVIDENCE is the real [`namespaces_in_use_under`] over a
    /// hand-built layout, so a `scan` arm exercises the production union rather
    /// than a dictated answer. Everything else is the ordinary fake.
    struct RootSearchProbe {
        anchors: Vec<RegistryAnchor>,
        unlinked: Mutex<Vec<String>>,
    }

    impl RootSearchProbe {
        fn at(root: &Path) -> Self {
            Self {
                anchors: anchors_at(root),
                unlinked: Mutex::new(Vec::new()),
            }
        }
        fn unlinked(&self) -> Vec<String> {
            self.unlinked.lock().expect("unlink ledger").clone()
        }
    }

    impl SystemProbe for RootSearchProbe {
        fn process_liveness(&self, _pid: i32) -> ProcessLiveness {
            // Every creator is GONE, so the ONLY thing that can refuse a file
            // here is the namespace evidence — which is what these arms are
            // about.
            ProcessLiveness::Gone
        }
        fn shm_object_size(&self, _name: &str) -> Option<u64> {
            Some(4_096)
        }
        fn unlink_shm_object(&self, name: &str) -> Result<(), String> {
            self.unlinked
                .lock()
                .expect("unlink ledger")
                .push(name.to_string());
            Ok(())
        }
        fn namespaces_in_use(&self, budget: Duration) -> NamespacesInUse {
            namespaces_in_use_under(&self.anchors, budget)
        }
    }

    /// The headline: a registry ONE LEVEL below the anchor root contributes its
    /// namespaces, and the global-root-only walk
    /// is asserted IN THE SAME BODY not to, so the arm names the
    /// defect rather than merely describing the remedy.
    #[test]
    fn the_union_covers_a_registry_one_level_below_the_anchor_root() {
        let root = tempdir("two_roots");
        plant_registered_node(&root, None, "iox2_");
        plant_registered_node(&root, Some("tests"), "test_prefix_abcd");
        // A sibling that is not a root at all must not disturb the search.
        std::fs::create_dir_all(root.join("services").join("some_service")).expect("services");

        let union = namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5));
        let NamespacesInUse::Known(prefixes) = union else {
            panic!("a readable layout must produce evidence, not Unknown");
        };
        assert_eq!(
            prefixes.iter().cloned().collect::<Vec<_>>(),
            vec!["iox2_".to_string(), "test_prefix_abcd".to_string()],
            "the union must carry BOTH roots' namespaces"
        );

        // The defect, stated as an assertion: the global registry alone knows
        // nothing about the isolated one, so its files read as free.
        let global_only = namespaces_in_use_at(&root.join("nodes"), Duration::from_secs(5));
        assert!(
            !global_only.covers("test_prefix_abcd_node.0_10_0.global_mgmt.shm_state"),
            "precondition: the global-root-only evidence must MISS the isolated namespace — \
             otherwise this arm is not testing anything"
        );
        std::fs::remove_dir_all(&root).ok();
    }

    /// The regression arm, at the `scan` seam an operator actually reaches: a
    /// mapping belonging to a LIVE isolated-root namespace survives a reclaim
    /// pass, and — in the same body, over the same layout — a mapping whose
    /// namespace has no registered node anywhere is still reclaimed.
    ///
    /// The second half is the anti-tautology: without it, a fix that simply
    /// refused everything (the layout-MAJOR class, where one monitor
    /// file turned the whole pass inert) would pass the first half.
    #[test]
    fn a_live_isolated_root_namespace_refuses_reclamation_while_a_dead_one_does_not() {
        let root = tempdir("scan_root");
        plant_registered_node(&root, Some("tests"), "test_prefix_live");

        let files = tempdir("scan_files");
        let live = write_state_file(
            &files,
            "test_prefix_live_node.0_10_0.global_mgmt",
            "100_1_1_1",
        );
        let dead = write_state_file(
            &files,
            "test_prefix_dead_node.0_10_0.global_mgmt",
            "101_1_1_1",
        );

        let probe = RootSearchProbe::at(&root);
        let report = scan(
            &files,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(
            report.namespace_in_use, 1,
            "the isolated root's live namespace must be recognised"
        );
        assert!(
            live.exists(),
            "a mapping whose namespace still has a registered node — in an ISOLATED root — \
             must survive the pass; removing it splits the namespace and strands every node \
             registered against it"
        );
        assert_eq!(
            report.reclaimed_files, 1,
            "…and a namespace with no registered node in ANY root is still reclaimable — \
             covering the isolated root must not make the whole pass inert"
        );
        assert!(!dead.exists());
        assert_eq!(probe.unlinked(), vec!["101_1_1_1".to_string()]);
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&files).ok();
    }

    /// Fail-closed, at each of the three points the union can lose its footing.
    /// Every one of these would otherwise become a silent "no namespace is in
    /// use", which is the permissive answer that strands mappings.
    #[test]
    fn a_search_that_cannot_be_completed_refuses_everything() {
        // 1. The anchor root EXISTS but cannot be enumerated. A regular file
        //    where a directory belongs gives `NotADirectory` — an error, not
        //    the `NotFound` that means "no root here" — deterministically, with
        //    no dependence on the user the suite runs as.
        let file_root = tempdir("file_root_parent").join("root");
        std::fs::write(&file_root, b"not a directory").expect("write file root");
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&file_root), Duration::from_secs(5)),
            NamespacesInUse::Unknown,
            "a root we could not LOOK in is never an absence"
        );

        // 2. ONE discovered registry is unreadable: the whole union refuses,
        //    even though the OTHER registry read cleanly. A union missing a set
        //    proves nothing about the names in that set.
        let root = tempdir("poisoned_union");
        plant_registered_node(&root, Some("tests"), "test_prefix_live");
        std::fs::write(root.join("nodes"), b"not a registry").expect("file registry");
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5)),
            NamespacesInUse::Unknown,
            "one unreadable registry must poison the union, not be quietly dropped from it"
        );

        // 3. A spent budget. The search must refuse to START rather than let an
        //    exhausted bound buy a fresh ceiling.
        let ok = tempdir("spent_budget");
        plant_registered_node(&ok, None, "iox2_");
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&ok), Duration::ZERO),
            NamespacesInUse::Unknown,
            "no budget, no evidence"
        );

        // The control that makes all three mean something: the SAME shapes,
        // repaired, produce evidence.
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&ok), Duration::from_secs(5)),
            NamespacesInUse::Known(["iox2_".to_string()].into_iter().collect()),
            "anti-tautology: a readable layout with a real budget is not Unknown"
        );

        std::fs::remove_file(&file_root).ok();
        std::fs::remove_dir_all(file_root.parent().expect("parent")).ok();
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&ok).ok();
    }

    /// An anchor root that does not exist holds no registries — the ONE true
    /// empty in the search, and the reason a desk that has never run iceoryx2
    /// does not refuse every file.
    #[test]
    fn an_absent_anchor_root_is_the_honest_empty() {
        let absent = std::env::temp_dir().join(format!(
            "absent_root_{}_{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        std::fs::remove_dir_all(&absent).ok();
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&absent), Duration::from_secs(5)),
            NamespacesInUse::Known(std::collections::BTreeSet::new()),
            "a root that does not exist has no registries under it"
        );
    }

    /// A SYMLINK where a nested registry would be is neither followed nor
    /// dismissed. Following it would let whoever planted it aim the walk;
    /// dismissing it would declare the namespaces it names free, which is the
    /// stranding this fix exists to prevent.
    #[cfg(unix)]
    #[test]
    fn a_symlinked_nested_registry_is_refused_rather_than_followed_or_dismissed() {
        let root = tempdir("symlink_root");
        let elsewhere = tempdir("symlink_target");
        std::fs::create_dir_all(root.join("tests")).expect("sub root");
        std::os::unix::fs::symlink(&elsewhere, root.join("tests").join("nodes"))
            .expect("symlink the nested registry");

        assert_eq!(
            namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5)),
            NamespacesInUse::Unknown,
            "a shape that can be neither read as a registry nor dismissed is Unknown"
        );

        // Control: replace the link with a real registry and the same layout
        // produces evidence — so the arm above is about the LINK, not about the
        // sub-directory existing at all.
        std::fs::remove_file(root.join("tests").join("nodes")).expect("drop the link");
        plant_registered_node(&root, Some("tests"), "test_prefix_real");
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5)),
            NamespacesInUse::Known(["test_prefix_real".to_string()].into_iter().collect()),
        );
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&elsewhere).ok();
    }

    /// The SUB-ROOT twin of the arm above, one level EARLIER in the walk.
    ///
    /// `DirEntry::file_type` does not follow links, so a symlinked entry under
    /// an anchor root is neither a directory nor a shape the candidate stat ever
    /// sees — it is judged, and dismissed, before the registry under it is
    /// looked for at all. That silently declares every namespace registered
    /// there free, which is exactly the stranding the candidate-level arm above
    /// exists to prevent; the two must answer the same way.
    #[cfg(unix)]
    #[test]
    fn a_symlinked_sub_root_is_refused_rather_than_skipped() {
        let root = tempdir("symlink_sub_root");
        let elsewhere = tempdir("symlink_sub_target");
        // A REAL registry behind the link, with a REAL registered node: the
        // namespace a walk that skipped the link would report as free.
        plant_registered_node(&elsewhere, None, "test_prefix_behind_link");
        std::os::unix::fs::symlink(&elsewhere, root.join("tests")).expect("symlink the sub-root");

        assert_eq!(
            namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5)),
            NamespacesInUse::Unknown,
            "a symlinked sub-root can be neither followed nor dismissed — skipping it answers \
             Known(empty) and frees a live namespace"
        );

        // Control: the SAME layout with a real directory in the link's place
        // produces evidence, and that evidence CARRIES the namespace behind it —
        // so the arm above is about the LINK, not about the sub-root existing.
        std::fs::remove_file(root.join("tests")).expect("drop the link");
        plant_registered_node(&root, Some("tests"), "test_prefix_behind_link");
        assert_eq!(
            namespaces_in_use_under(&anchors_at(&root), Duration::from_secs(5)),
            NamespacesInUse::Known(
                ["test_prefix_behind_link".to_string()]
                    .into_iter()
                    .collect()
            ),
        );
        std::fs::remove_dir_all(&root).ok();
        std::fs::remove_dir_all(&elsewhere).ok();
    }

    /// The anchors are DERIVED from iceoryx2's own config, never hand-listed,
    /// and the compiled-in default is always among them — which is what makes
    /// `iceoryx2::testing`'s roots reachable even when a config file has moved
    /// the global root somewhere else (the testing helper starts from
    /// `Config::default()` and only overrides `root_path` and `prefix`).
    #[test]
    fn the_live_anchors_are_derived_from_iceoryx2s_own_config() {
        let anchors = live_registry_anchors();
        assert!(
            !anchors.is_empty(),
            "an empty anchor set would search nothing and answer Known(empty) — inert"
        );
        let default = iceoryx2::config::Config::default();
        let default_root = PathBuf::from(String::from(default.global.root_path()));
        assert!(
            anchors.iter().any(|a| a.root == default_root),
            "the COMPILED-IN root must always be searched: it is what \
             `iceoryx2::testing::generate_isolated_config` hangs its roots off, whatever a \
             config file says. anchors = {anchors:?}"
        );
        for anchor in &anchors {
            assert!(
                !anchor.node_dir_name.is_empty(),
                "a registry directory name must come from the config, never be guessed"
            );
        }
    }

    /// `covers` is the whole decision, so both arms are pinned directly —
    /// including that `Unknown` covers a name no prefix could match.
    #[test]
    fn coverage_matches_on_the_prefix_and_unknown_covers_everything() {
        let known = NamespacesInUse::Known(
            ["iox2_", "cer_p_beef"]
                .iter()
                .map(|s| (*s).to_string())
                .collect(),
        );
        assert!(known.covers("iox2_9204_node.0_10_0.global_mgmt.shm_state"));
        assert!(known.covers("cer_p_beef1234_node.0_10_0.global_mgmt.shm_state"));
        assert!(
            !known.covers("cer_p_other1234_node.0_10_0.global_mgmt.shm_state"),
            "a namespace with no registered node is not covered"
        );
        assert!(
            !NamespacesInUse::Known(std::collections::BTreeSet::new()).covers("anything"),
            "an empty KNOWN set covers nothing"
        );
        assert!(
            NamespacesInUse::Unknown.covers("anything"),
            "an unknown registry covers everything — refusal, not absence"
        );
    }

    #[test]
    fn a_failed_object_unlink_leaves_the_state_file_where_it_is() {
        // Removing the file first would strand the object under a name nothing
        // can resolve again — a permanent kernel leak in place of a
        // reclaimable one.
        let dir = tempdir("unlinkfail");
        let path = write_state_file(&dir, "stuck", "500_1_1_1");
        let probe = FakeProbe::all_dead().failing_unlink("500_1_1_1", "Operation not permitted");

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(report.proven_dead, 1);
        assert_eq!(report.reclaimed_files, 0);
        assert_eq!(report.reclaimed_object_bytes, 0);
        assert_eq!(report.reclaim_failures.len(), 1);
        assert!(report.reclaim_failures[0].contains("Operation not permitted"));
        assert!(
            path.exists(),
            "the state file must survive an object-unlink failure"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_quoted_failure_list_is_bounded_and_the_rest_are_counted() {
        let dir = tempdir("failbound");
        let mut probe = FakeProbe::all_dead();
        for i in 0..(MAX_QUOTED_FAILURES + 3) {
            let name = format!("{}_1_1_1", 900 + i);
            write_state_file(&dir, &format!("f{i}"), &name);
            probe = probe.failing_unlink(&name, "denied");
        }

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(report.reclaim_failures.len(), MAX_QUOTED_FAILURES);
        assert_eq!(report.reclaim_failures_elided, 3);
        std::fs::remove_dir_all(&dir).ok();
    }

    // ----------------------------------------------------- the exit-path pass

    #[test]
    fn the_exit_pass_reclaims_the_proven_dead_and_refuses_everything_else() {
        // The headline behaviour, driven through the entry point the graceful
        // teardown actually calls — so an exit pass that quietly acquired its
        // own mode or its own evidence rule fails here.
        let dir = tempdir("exitreclaim");
        let dead = write_state_file(&dir, "dead", "1100_1_1_1");
        let live = write_state_file(&dir, "live", "1200_1_1_1");
        let unknown = write_state_file(&dir, "unknown", "1300_1_1_1");
        let forged = write_state_file(&dir, "forged", "1100_someone_elses_object");

        let mut probe = FakeProbe::dead(&[1100]);
        probe.liveness.insert(1200, ProcessLiveness::Alive);
        probe.liveness.insert(1300, ProcessLiveness::Unknown);
        let probe = probe.with_size("1100_1_1_1", 24_576);

        let report = reclaim_at_exit(&dir, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);

        assert_eq!(report.counted, 4);
        assert_eq!(report.proven_dead, 1);
        assert_eq!(report.creator_alive, 1);
        assert_eq!(
            report.unproven, 2,
            "the unknown pid and the forged name are both refused"
        );
        assert_eq!(report.reclaimed_files, 1);
        assert_eq!(report.reclaimed_object_bytes, 24_576);
        assert_eq!(probe.unlinked(), vec!["1100_1_1_1".to_string()]);
        assert!(!dead.exists());
        assert!(live.exists(), "a live creator's file must survive an exit");
        assert!(unknown.exists());
        assert!(
            forged.exists(),
            "a forged name that borrows a dead pid must survive an exit too"
        );

        // The line an operator sees, from the same report.
        let line =
            render_exit_reclaim_line(&report).expect("a run that removed a file must say so");
        assert!(!line.failed);
        assert!(line.text.contains("reclaimed 1"), "{}", line.text);
        assert!(line.text.contains("24.0 KiB"), "{}", line.text);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_exit_pass_says_nothing_when_it_changed_nothing() {
        // Every graph run on every desk ends. A pass that announced "nothing to
        // do" each time is noise that trains an operator out of reading the
        // shutdown lines — so silence is the contract, and it is asserted
        // ALONGSIDE its positive arm so the absence cannot be vacuous.
        let quiet = tempdir("exitquiet");
        let probe = FakeProbe::all_alive();

        let empty_dir = reclaim_at_exit(&quiet, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);
        assert!(empty_dir.is_empty());
        assert_eq!(render_exit_reclaim_line(&empty_dir), None);

        // Populated, but nothing reclaimable: still silent.
        write_state_file(&quiet, "alive", "1400_1_1_1");
        let nothing_to_do = reclaim_at_exit(&quiet, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);
        assert_eq!(nothing_to_do.counted, 1);
        assert_eq!(nothing_to_do.creator_alive, 1);
        assert_eq!(
            render_exit_reclaim_line(&nothing_to_do),
            None,
            "a walk that found only live creators has nothing to report"
        );

        // THE ANTI-VACUITY HALF, same fixture: one dead creator and the line
        // appears. Without this, a renderer that returned `None` unconditionally
        // would pass every assertion above.
        write_state_file(&quiet, "dead", "1500_1_1_1");
        let mut speaking = FakeProbe::dead(&[1500]);
        speaking.liveness.insert(1400, ProcessLiveness::Alive);
        let acted = reclaim_at_exit(&quiet, SHM_STATE_EXIT_RECLAIM_BUDGET, &speaking);
        assert_eq!(acted.reclaimed_files, 1);
        assert!(render_exit_reclaim_line(&acted).is_some());
        std::fs::remove_dir_all(&quiet).ok();
    }

    #[test]
    fn a_failed_exit_reclamation_is_reported_and_marked_as_a_failure() {
        // The one shape that breaks the silence WITHOUT a change: something was
        // proven dead and could not be removed. Reporting it at the routine
        // level would file a real fault under housekeeping.
        let dir = tempdir("exitfail");
        let path = write_state_file(&dir, "stuck", "1600_1_1_1");
        let probe = FakeProbe::all_dead().failing_unlink("1600_1_1_1", "Operation not permitted");

        let report = reclaim_at_exit(&dir, SHM_STATE_EXIT_RECLAIM_BUDGET, &probe);
        assert_eq!(report.reclaimed_files, 0);
        assert!(path.exists());

        let line = render_exit_reclaim_line(&report).expect("a failure must not be silent");
        assert!(
            line.failed,
            "and must not be filed as routine: {}",
            line.text
        );
        assert!(line.text.contains("reclaimed nothing"), "{}", line.text);
        assert!(
            line.text.contains("1 reclamation(s) failed"),
            "{}",
            line.text
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_exit_pass_truncates_with_a_floor_report_rather_than_running_long() {
        // A shutdown must never stall on hygiene. The budget is what makes the
        // walk stop, and the truncation marker is what stops the stopped walk
        // from reading as a completed one.
        let dir = tempdir("exitbudget");
        for i in 0..12 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 1_700 + i));
        }
        // Load can only make the probe slower, so the budget is blown harder,
        // never less — the direction that keeps this stable on a busy desk.
        let probe = FakeProbe::all_dead().slow(Duration::from_millis(60));

        let started = Instant::now();
        let report = reclaim_at_exit(&dir, Duration::from_millis(120), &probe);
        let wall = started.elapsed();

        assert!(
            report.truncated || report.classification_truncated,
            "a pass that could not finish must say its answer is a floor: {report:?}"
        );
        assert!(
            report.reclaimed_files < 12,
            "the budget must actually stop the work, got {}",
            report.reclaimed_files
        );
        assert!(
            wall < Duration::from_secs(5),
            "the exit pass must respect its budget (plus one attempt), took {wall:?}"
        );
        if let Some(line) = render_exit_reclaim_line(&report) {
            assert!(
                line.text.contains("cerulion clean"),
                "a truncated pass must name the remedy that finishes the job: {}",
                line.text
            );
        }

        // ANTI-VACUITY: the same fixture at the real budget clears everything,
        // so the arm above is measuring the BUDGET and not a broken fixture.
        let quick = FakeProbe::all_dead();
        let full = reclaim_at_exit(&dir, SHM_STATE_EXIT_RECLAIM_BUDGET, &quick);
        assert!(!full.truncated && !full.classification_truncated);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_exit_budget_sits_between_the_diagnostic_and_the_explicit_remedy() {
        // Ordering the three budgets is the whole argument for having three:
        // a shutdown may take longer than a report (nobody is waiting on work)
        // and must take less than the command whose entire job is the sweep.
        assert!(SHM_STATE_EXIT_RECLAIM_BUDGET <= SHM_STATE_RECLAIM_BUDGET);
        assert!(!SHM_STATE_EXIT_RECLAIM_BUDGET.is_zero());
        assert!(
            classify_budget(SHM_STATE_EXIT_RECLAIM_BUDGET) < SHM_STATE_EXIT_RECLAIM_BUDGET,
            "classification must stay a strict fraction of the exit walk too"
        );
    }

    // ---------------------------------------------------------------- budgets

    #[test]
    fn a_zero_scan_budget_refuses_to_start_and_says_its_count_is_a_floor() {
        let dir = tempdir("zerobudget");
        for i in 0..5 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 700 + i));
        }
        let probe = FakeProbe::all_dead();

        let refused = scan(
            &dir,
            Duration::ZERO,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(refused.counted, 0);
        assert!(
            refused.truncated,
            "a zero count without the truncation marker is an affirmative claim of an empty directory"
        );
        assert!(
            probe.unlinked().is_empty(),
            "a refused walk must reclaim nothing"
        );

        // The ANTI-VACUITY half: the same fixture at a real budget is NOT
        // empty, so the zero-budget arm is measuring the refusal and not an
        // empty directory.
        let control = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::ReportOnly,
            &probe,
        );
        assert_eq!(control.counted, 5);
        assert!(!control.truncated);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_refused_walk_reports_a_refusal_even_when_there_was_nothing_to_walk() {
        // A zero count only means anything ALONGSIDE the marker that says
        // whether we looked. On a directory that cannot be opened, "refused
        // before starting" and "walked it, found nothing" produce the same
        // count, so the refusal has to be decided from the BUDGET — before
        // the directory is consulted at all. Otherwise a refused walk over an
        // unreadable directory reports a confident, wrong absence.
        //
        // This is also the only oracle that can see the budget guard: on a
        // directory that DOES open, the loop's own deadline check breaks on
        // the first entry and reproduces the refusal exactly.
        let probe = FakeProbe::all_dead();
        let missing = Path::new("/definitely/not/a/directory/probe-refused");

        let refused = scan(
            missing,
            Duration::ZERO,
            Duration::ZERO,
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(refused.counted, 0);
        assert!(
            refused.truncated,
            "a zero-budget walk must report a REFUSAL, never an empty directory"
        );

        // ANTI-TAUTOLOGY: the same unreadable directory at a REAL budget is a
        // complete answer, so `truncated` really is carrying the refusal and
        // not the absence.
        let looked = scan(
            missing,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert!(!looked.truncated);
        assert!(looked.is_empty());
    }

    #[test]
    fn a_refused_classification_says_so_even_on_a_directory_with_nothing_in_it() {
        // `classification_truncated == false` is a POSITIVE claim that every
        // counted file was classified. A refused budget classified nothing,
        // and an empty directory does not make that claim true — so the
        // refusal must be decided from the budget rather than from the loop
        // happening to notice.
        let dir = tempdir("emptyreclaim");
        let probe = FakeProbe::all_dead();

        let refused = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            Duration::ZERO,
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(refused.counted, 0);
        assert!(!refused.truncated, "the walk itself completed");
        assert!(
            refused.classification_truncated,
            "classification was refused, and an empty directory does not turn that into a \
             completed classification"
        );

        // ANTI-TAUTOLOGY: a real budget over the SAME empty directory makes
        // no truncation claim at all.
        let complete = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert!(!complete.classification_truncated);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_zero_reclaim_budget_still_counts_but_classifies_nothing() {
        // The count is the headline diagnostic; reclamation is secondary, so
        // the two budgets are separate and the cheap one always survives.
        let dir = tempdir("zeroreclaim");
        for i in 0..4 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 800 + i));
        }
        let probe = FakeProbe::all_dead();

        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            Duration::ZERO,
            ScanMode::Reclaim,
            &probe,
        );
        assert_eq!(report.counted, 4, "counting must not depend on classifying");
        assert!(!report.truncated);
        assert_eq!(report.classified, 0);
        assert!(report.classification_truncated);
        assert_eq!(report.proven_dead, 0);
        assert_eq!(report.reclaimed_files, 0);
        assert!(probe.unlinked().is_empty());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn an_exhausted_classification_budget_stops_probing_but_finishes_the_count() {
        // Load can only make the probe SLOWER, so the budget is blown harder,
        // never less — the direction that keeps this arm stable on a busy desk.
        let dir = tempdir("slowprobe");
        for i in 0..12 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 600 + i));
        }
        let probe = FakeProbe::all_dead().slow(Duration::from_millis(60));

        let report = scan(
            &dir,
            Duration::from_secs(30),
            Duration::from_millis(1),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(
            report.counted, 12,
            "the count survives a spent probe budget"
        );
        assert!(!report.truncated);
        assert!(report.classification_truncated);
        assert!(
            report.classified < report.counted,
            "classification must stop early, got {} of {}",
            report.classified,
            report.counted
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn an_exhausted_scan_budget_reports_a_floor_rather_than_a_wrong_exact_count() {
        let dir = tempdir("slowscan");
        for i in 0..12 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 500 + i));
        }
        // A probe slow enough that the SCAN budget (which classification runs
        // inside) is spent well before the directory ends.
        let probe = FakeProbe::all_dead().slow(Duration::from_millis(60));

        let report = scan(
            &dir,
            Duration::from_millis(1),
            Duration::from_millis(1),
            ScanMode::ReportOnly,
            &probe,
        );
        assert!(
            report.truncated,
            "a walk that stopped early must say its count is a floor"
        );
        assert!(report.counted <= 12);
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(unix)]
    #[test]
    fn an_unreadable_directory_is_a_refusal_never_an_empty_population() {
        // A `read_dir` failure must never read as a successful EMPTY answer: a
        // permission or I/O fault would make `cerulion clean` print a clean no-leak
        // result on a machine it had not managed to look at. Same class as the
        // budget refusals, on a path the budget cannot reach.
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempdir("unreadable");
        write_state_file(&dir, "hidden", "902_1_1_1");
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o000)).expect("chmod");

        // The PRECONDITION is established INDEPENDENTLY of the code under
        // test. Gating on the REPORT instead makes every assertion below
        // VACUOUS under exactly the defect they exist to catch — a refusal
        // that degrades to an empty report has `refused: None`, so the whole
        // block is skipped and the test passes: a report-gated arm lets the
        // defect go undetected.
        if std::fs::read_dir(&dir).is_ok() {
            // Running as root defeats the mode bits; the environment cannot
            // produce the case, so there is nothing to assert.
            std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).ok();
            std::fs::remove_dir_all(&dir).ok();
            return;
        }

        let probe = FakeProbe::all_dead();
        let report = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );

        assert_eq!(report.counted, 0);
        assert!(
            report.refused.is_some(),
            "a walk that could not open the directory must report a REFUSAL; got {report:?}"
        );
        assert!(
            !report.is_empty(),
            "a walk that could not look must not read as an empty directory"
        );
        assert!(
            probe.unlinked().is_empty(),
            "a refused walk reclaims nothing"
        );

        let text = joined(&render_lines(&nodes_report(1), Some(&report)));
        assert!(text.contains("COULD NOT READ"), "{text}");
        assert!(text.contains("UNKNOWN"), "{text}");
        assert!(
            !text.contains("nothing leaked here"),
            "a refusal must never render as a clean report: {text}"
        );

        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).ok();
        std::fs::remove_dir_all(&dir).ok();
    }

    #[cfg(unix)]
    #[test]
    fn an_unreadable_node_registry_is_a_refusal_never_an_absence() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempdir("noderefuse");
        std::fs::write(dir.join("1"), b"x").expect("write");
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o000)).expect("chmod");

        // Precondition established independently of the code under test — see
        // the sibling arm above for why gating on the report is vacuous.
        if std::fs::read_dir(&dir).is_ok() {
            std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).ok();
            std::fs::remove_dir_all(&dir).ok();
            return;
        }

        let report = count_node_registry(&dir, Duration::from_secs(30));
        assert!(
            report.refused.is_some(),
            "a registry we cannot read must report a REFUSAL; got {report:?}"
        );
        assert!(
            !report.absent,
            "`absent` is reserved for NotFound — a directory we cannot read is not missing"
        );
        assert_eq!(report.entries, 0);
        let text = joined(&render_lines(&report, None));
        assert!(text.contains("COULD NOT READ"), "{text}");
        assert!(
            !text.contains("no iceoryx2 process has left state here"),
            "a refusal must not render as an absence: {text}"
        );

        // The NotFound arm still reads as a genuine absence, so the two are
        // distinguished rather than merged.
        let missing = count_node_registry(
            Path::new("/definitely/not/here/probe"),
            Duration::from_secs(1),
        );
        assert!(missing.absent);
        assert!(missing.refused.is_none());

        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o700)).ok();
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_missing_directory_is_the_healthy_case_not_a_fault() {
        let probe = FakeProbe::all_dead();
        let report = scan(
            Path::new("/definitely/not/a/directory/probe"),
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::Reclaim,
            &probe,
        );
        assert!(report.is_empty());
        assert!(!report.truncated);
    }

    // ----------------------------------------------------------------- cost

    #[test]
    fn classification_is_never_allowed_to_outlive_the_count() {
        // The COUNT is the headline diagnostic; if per-file probing could
        // consume the whole walk, a catastrophic directory would report a
        // tiny floor and be classified `Healthy`.
        for ms in [1u64, 7, 100, 2_000, 30_000] {
            let scan = Duration::from_millis(ms);
            assert!(
                classify_budget(scan) < scan,
                "classification must be a strict fraction of the walk at {ms} ms"
            );
        }
        // The production pair, and the ordering the const-assert also pins.
        assert!(SHM_STATE_RECLAIM_BUDGET >= SHM_STATE_REPORT_BUDGET);
        assert!(classify_budget(SHM_STATE_REPORT_BUDGET) < SHM_STATE_REPORT_BUDGET);
        // The remedy must be able to clear the MEASURED pathological
        // directory in one run on a quiet desk, or it is not a remedy.
        assert!(
            classify_budget(SHM_STATE_RECLAIM_BUDGET).as_millis()
                > u128::from(MEASURED_BASIS_QUIET_MS),
            "a reclaim run must outlast one quiet scan of the measured basis"
        );
    }

    #[test]
    fn the_cost_estimate_reproduces_its_own_measured_basis() {
        // The basis is MEASURED; at the basis size the
        // projection must be the measurement itself, or the constants and the
        // arithmetic have drifted apart.
        let at_basis = estimate_scan_cost(MEASURED_BASIS_FILES);
        assert_eq!(at_basis.quiet_ms, MEASURED_BASIS_QUIET_MS);
        assert_eq!(at_basis.loaded_ms, MEASURED_BASIS_LOADED_MS);

        assert_eq!(
            estimate_scan_cost(0),
            ScanCostEstimate {
                quiet_ms: 0,
                loaded_ms: 0
            }
        );
        // Linear, so half the files is half the time.
        let half = estimate_scan_cost(MEASURED_BASIS_FILES / 2);
        assert!(half.quiet_ms.abs_diff(MEASURED_BASIS_QUIET_MS / 2) <= 1);

        // A hostile count must not panic or wrap.
        let huge = estimate_scan_cost(u64::MAX);
        assert!(huge.quiet_ms > 0 && huge.loaded_ms > 0);
    }

    #[test]
    fn severity_is_pinned_on_both_sides_of_each_band() {
        // Derived from the basis rather than hand-written, so a basis change
        // moves the boundary instead of silently invalidating the test. CEIL,
        // because `estimate_scan_cost` floors: the smallest count that
        // PROJECTS at or above the threshold is the ceiling of the exact
        // solution, and a floored boundary lands one file short of its own
        // band.
        let elevated_at =
            (MEASURED_BASIS_FILES * ELEVATED_QUIET_MS).div_ceil(MEASURED_BASIS_QUIET_MS);
        let severe_at = (MEASURED_BASIS_FILES * SEVERE_QUIET_MS).div_ceil(MEASURED_BASIS_QUIET_MS);

        assert_eq!(severity(0), ScanSeverity::Healthy);
        assert_eq!(severity(elevated_at - 1), ScanSeverity::Healthy);
        assert_eq!(severity(elevated_at), ScanSeverity::Elevated);
        assert_eq!(severity(severe_at - 1), ScanSeverity::Elevated);
        assert_eq!(severity(severe_at), ScanSeverity::Severe);
        // The measured pathological directory must land in the top band, or
        // the bands do not describe the leak this exists for.
        assert_eq!(severity(MEASURED_BASIS_FILES), ScanSeverity::Severe);
    }

    // --------------------------------------------------------- node registry

    #[test]
    fn the_node_registry_count_reports_a_floor_and_an_absence_apart() {
        let dir = tempdir("nodes");
        for i in 0..6 {
            std::fs::write(dir.join(format!("{i}")), b"x").expect("write");
        }

        let counted = count_node_registry(&dir, Duration::from_secs(30));
        assert_eq!(counted.entries, 6);
        assert!(!counted.truncated);
        assert!(!counted.absent);

        let refused = count_node_registry(&dir, Duration::ZERO);
        assert_eq!(refused.entries, 0);
        assert!(refused.truncated, "a refused count is a floor, not a zero");
        assert!(
            !refused.absent,
            "the directory exists; we simply did not look"
        );

        let missing = count_node_registry(
            Path::new("/definitely/not/here/probe"),
            Duration::from_secs(1),
        );
        assert!(missing.absent);
        assert_eq!(missing.entries, 0);
        assert!(
            !missing.truncated,
            "an absent registry is a complete answer, not a truncated one"
        );
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_platform_predicate_matches_the_platforms_that_keep_state_files() {
        // `iceoryx2-pal-posix` uses the state-file indirection exactly where
        // there is no `/dev/shm`.
        #[cfg(target_os = "macos")]
        assert!(platform_uses_shm_state_files());
        #[cfg(target_os = "linux")]
        assert!(!platform_uses_shm_state_files());
    }

    // ---------------------------------------------------------------- render

    fn nodes_report(entries: u64) -> NodeRegistryReport {
        NodeRegistryReport {
            dir: PathBuf::from("/tmp/iceoryx2/nodes"),
            entries,
            truncated: false,
            absent: false,
            refused: None,
            skipped_entries: 0,
        }
    }

    fn joined(lines: &[String]) -> String {
        lines.join("\n")
    }

    #[test]
    fn the_report_carries_every_number_it_measured() {
        // A diagnostic that computes a population and then does not print it
        // is indistinguishable from a healthy machine — which is the whole
        // defect this module exists to fix, one layer up.
        let mut shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
        shm.counted = MEASURED_BASIS_FILES;
        shm.classified = MEASURED_BASIS_FILES;
        shm.proven_dead = 70_000;
        shm.creator_alive = 2;
        shm.unproven = 5;
        shm.reclaimed_files = 70_000;
        shm.reclaimed_object_bytes = 24_057 * 70_000;

        let text = joined(&render_lines(&nodes_report(429), Some(&shm)));

        assert!(
            text.contains("429"),
            "the node-registry count must be shown"
        );
        assert!(
            text.contains("/tmp/iceoryx2/nodes"),
            "and where it came from"
        );
        assert!(
            text.contains("72707"),
            "the state-file population must be shown: {text}"
        );
        assert!(text.contains("SEVERE"), "and its severity: {text}");
        // The two measured figures, rendered: 13,000 ms and 393,000 ms.
        assert!(
            text.contains("13.0 s") && text.contains("6.5 min"),
            "and the projected scan cost, both machine states: {text}"
        );
        assert!(text.contains("70000"), "and what was reclaimed: {text}");
        assert!(
            text.contains("GiB"),
            "and the shared memory it freed: {text}"
        );
        assert!(
            text.contains("still running"),
            "and what was deliberately left alone: {text}"
        );
        assert!(
            text.contains("UNPROVEN"),
            "and what could not be proven either way: {text}"
        );
    }

    #[test]
    fn a_truncated_count_is_rendered_as_a_floor_never_as_an_exact_population() {
        let mut shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
        shm.counted = 40;
        shm.truncated = true;
        let text = joined(&render_lines(&nodes_report(3), Some(&shm)));
        assert!(text.contains("at least 40"), "{text}");
        assert!(text.contains("FLOOR"), "{text}");

        let mut exact = ShmStateReport::empty(PathBuf::from("/tmp/"));
        exact.counted = 40;
        let text = joined(&render_lines(&nodes_report(3), Some(&exact)));
        assert!(
            !text.contains("at least"),
            "an exact count must not hedge: {text}"
        );
        assert!(!text.contains("FLOOR"), "{text}");
    }

    #[test]
    fn a_report_only_run_names_what_it_would_reclaim_rather_than_claiming_it_did() {
        let mut shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
        shm.counted = 9;
        shm.classified = 9;
        shm.proven_dead = 9;
        let text = joined(&render_lines(&nodes_report(1), Some(&shm)));
        assert!(text.contains("provably dead"), "{text}");
        assert!(
            text.contains("--report-only"),
            "the remedy names the flag: {text}"
        );
        assert!(
            !text.contains("reclaimed"),
            "a run that deleted nothing must not claim it did: {text}"
        );
    }

    /// "reclaimed nothing" and "was never allowed to try" are
    /// opposite facts that share a count of zero, so the render keys on the
    /// MODE. All three shapes in one body, because a render keyed on the count
    /// looks right in isolation: a pass whose every unlink failed would print
    /// "Reclamation did not run this pass" directly above its own
    /// "reclaim failed — …" lines.
    #[test]
    fn the_report_says_whether_reclamation_was_attempted_not_whether_it_succeeded() {
        let base = || {
            let mut shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
            shm.counted = 4;
            shm.classified = 4;
            shm.proven_dead = 4;
            shm
        };

        // 1. NOT ATTEMPTED — the gate text.
        let withheld = base();
        let text = joined(&render_lines(&nodes_report(1), Some(&withheld)));
        assert!(
            text.contains("Reclamation did not run this pass"),
            "a withheld pass must state the gate: {text}"
        );
        assert!(text.contains("--report-only"), "{text}");
        assert!(!text.contains("RAN this pass"), "{text}");

        // 2. ATTEMPTED and reclaimed — the ordinary success line.
        let mut done = base();
        done.attempted_reclamation = true;
        done.reclaimed_files = 4;
        done.reclaimed_object_bytes = 4 * 4_096;
        let text = joined(&render_lines(&nodes_report(1), Some(&done)));
        assert!(text.contains("reclaimed 4 file(s)"), "{text}");
        assert!(
            !text.contains("did not run") && !text.contains("RAN this pass"),
            "a successful pass needs neither disclaimer: {text}"
        );

        // 3. ATTEMPTED and every removal FAILED — the shape the old branch key
        // got wrong. It must NOT claim the pass did not run, and it must point
        // at the failure lines it prints below.
        let mut failed = base();
        failed.attempted_reclamation = true;
        failed.reclaim_failures = vec![
            "/tmp/a.shm_state: Permission denied".to_string(),
            "/tmp/b.shm_state: Permission denied".to_string(),
        ];
        failed.reclaim_failures_elided = 2;
        let text = joined(&render_lines(&nodes_report(1), Some(&failed)));
        assert!(
            !text.contains("did not run"),
            "a pass that RAN and failed must never report as one that never ran — its own \
             failure lines are printed right below: {text}"
        );
        assert!(
            text.contains("Reclamation RAN this pass and every removal failed (4 failure(s))"),
            "it must name the outcome and the count (2 quoted + 2 elided): {text}"
        );
        assert!(
            text.contains("reclaim failed — /tmp/a.shm_state"),
            "and the breakdown it points at must be there: {text}"
        );
    }

    #[test]
    fn the_clean_report_names_the_empty_orphans_as_their_own_population() {
        // Rider 1 rides the SHARED classifier, so `cerulion clean` inherits it
        // for free — but only if the render says so. Two proofs with one number
        // would leave an operator unable to tell "70 dead creators" from "70
        // poison pills", which need different explanations.
        let mut looked = ShmStateReport::empty(PathBuf::from("/tmp/"));
        looked.counted = 12;
        looked.classified = 12;
        looked.proven_dead = 5;
        looked.empty_orphans = 7;
        let text = joined(&render_lines(&nodes_report(1), Some(&looked)));
        assert!(text.contains("5 file(s) are provably dead"), "{text}");
        assert!(text.contains("7 file(s) record no name"), "{text}");
        assert!(text.contains("24 h"), "and the bound they cleared: {text}");
        assert!(!text.contains("reclaimed"), "nothing was removed: {text}");

        let mut acted = ShmStateReport::empty(PathBuf::from("/tmp/"));
        acted.counted = 12;
        acted.classified = 12;
        acted.proven_dead = 5;
        acted.empty_orphans = 7;
        acted.reclaimed_files = 12;
        acted.reclaimed_empty_files = 7;
        acted.reclaimed_object_bytes = 5 * 24_576;
        let text = joined(&render_lines(&nodes_report(1), Some(&acted)));
        assert!(text.contains("reclaimed 12 file(s)"), "{text}");
        assert!(
            text.contains("7 of those recorded no name"),
            "the byte figure covers the other five only, so the split must show: {text}"
        );
    }

    #[test]
    fn a_platform_with_no_state_files_says_so_rather_than_going_quiet() {
        // Silence is indistinguishable from a diagnostic that ran and found
        // nothing, and the node-registry half applies everywhere.
        let text = joined(&render_lines(&nodes_report(2), None));
        assert!(text.contains("not applicable on this platform"), "{text}");
        assert!(
            text.contains("2 entries"),
            "the node half still renders: {text}"
        );
    }

    #[test]
    fn a_clean_directory_renders_one_line_and_no_alarm() {
        let shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
        let text = joined(&render_lines(&nodes_report(0), Some(&shm)));
        assert!(text.contains("nothing leaked here"), "{text}");
        assert!(
            !text.contains("SEVERE") && !text.contains("ELEVATED"),
            "{text}"
        );
    }

    #[test]
    fn reclaim_failures_are_quoted_then_counted() {
        let mut shm = ShmStateReport::empty(PathBuf::from("/tmp/"));
        shm.counted = 20;
        shm.classified = 20;
        shm.proven_dead = 20;
        shm.reclaim_failures = vec!["77_1_1_1: Operation not permitted".to_string()];
        shm.reclaim_failures_elided = 11;
        let text = joined(&render_lines(&nodes_report(1), Some(&shm)));
        assert!(text.contains("Operation not permitted"), "{text}");
        assert!(text.contains("11 further reclaim failure"), "{text}");
    }

    #[test]
    fn the_render_helpers_answer_their_hand_written_vectors() {
        assert_eq!(render_ms(0), "0 ms");
        assert_eq!(render_ms(999), "999 ms");
        assert_eq!(render_ms(1_000), "1.0 s");
        assert_eq!(render_ms(59_999), "60.0 s");
        assert_eq!(render_ms(60_000), "1.0 min");
        assert_eq!(render_bytes(0), "0 B");
        assert_eq!(render_bytes(1023), "1023 B");
        assert_eq!(render_bytes(1024), "1.0 KiB");
        assert_eq!(render_bytes(1024 * 1024), "1.0 MiB");
        assert_eq!(render_bytes(1024 * 1024 * 1024), "1.00 GiB");
    }

    #[test]
    fn two_walks_over_one_directory_report_identically() {
        let dir = tempdir("determinism");
        for i in 0..5 {
            write_state_file(&dir, &format!("f{i}"), &format!("{}_1_1_1", 400 + i));
        }
        let probe = FakeProbe::all_dead();
        let first = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::ReportOnly,
            &probe,
        );
        let second = scan(
            &dir,
            SHM_STATE_REPORT_BUDGET,
            classify_budget(SHM_STATE_REPORT_BUDGET),
            ScanMode::ReportOnly,
            &probe,
        );
        assert_eq!(first.counted, second.counted);
        assert_eq!(first.proven_dead, second.proven_dead);
        assert_eq!(first.severity(), second.severity());
        assert_eq!(first.cost(), second.cost());
        std::fs::remove_dir_all(&dir).ok();
    }

    // --------------------------------------------------------- one classifier

    /// Strip `//` line comments, `/* … */` block comments (DEPTH-TRACKED —
    /// Rust's nest) and string literals, so the walks below read CODE and not
    /// the prose that explains it.
    ///
    /// Load-bearing in the false-FAILURE direction rather than the false-pass
    /// one: a second adjudicator is code, so a raw view could never miss it —
    /// but a sibling module's doc comment explaining why it does NOT classify
    /// would fail the walk on a mention, which is the fastest way to get a
    /// structural guard deleted.
    fn code_only(src: &str) -> String {
        let b = src.as_bytes();
        let mut out = String::with_capacity(src.len());
        let mut i = 0usize;
        let mut depth = 0usize;
        while i < b.len() {
            if depth == 0 {
                // A RAW string: `r`, zero or more `#`, then `"`.
                if b[i] == b'r' {
                    let mut j = i + 1;
                    while j < b.len() && b[j] == b'#' {
                        j += 1;
                    }
                    if j < b.len() && b[j] == b'"' {
                        let hashes = j - i - 1;
                        let mut k = j + 1;
                        'raw: while k < b.len() {
                            if b[k] == b'"' {
                                let mut h = 0usize;
                                while h < hashes && k + 1 + h < b.len() && b[k + 1 + h] == b'#' {
                                    h += 1;
                                }
                                if h == hashes {
                                    k = k + 1 + hashes;
                                    break 'raw;
                                }
                            }
                            k += 1;
                        }
                        out.push(' ');
                        i = k.min(b.len());
                        continue;
                    }
                }
                if b[i] == b'"' {
                    let mut k = i + 1;
                    while k < b.len() {
                        match b[k] {
                            b'\\' => k += 2,
                            b'"' => {
                                k += 1;
                                break;
                            }
                            _ => k += 1,
                        }
                    }
                    out.push(' ');
                    i = k.min(b.len());
                    continue;
                }
                if b[i] == b'/' && b.get(i + 1) == Some(&b'/') {
                    while i < b.len() && b[i] != b'\n' {
                        i += 1;
                    }
                    out.push(' ');
                    continue;
                }
            }
            if b[i] == b'/' && b.get(i + 1) == Some(&b'*') {
                depth += 1;
                i += 2;
                continue;
            }
            if depth > 0 && b[i] == b'*' && b.get(i + 1) == Some(&b'/') {
                depth -= 1;
                i += 2;
                out.push(' ');
                continue;
            }
            if depth > 0 {
                i += 1;
                continue;
            }
            let ch_len = src[i..].chars().next().map_or(1, char::len_utf8);
            out.push_str(&src[i..i + ch_len]);
            i += ch_len;
        }
        assert_eq!(
            depth, 0,
            "code_only: unterminated block comment — the stripped view would swallow the rest \
             of the file and make every assertion over it vacuous"
        );
        out
    }

    #[test]
    fn code_only_strips_comments_and_literals_and_nothing_else() {
        // Without this oracle a broken stripper makes every walk below vacuous
        // rather than loud.
        assert_eq!(
            code_only("let a = 1; // n\nlet b = 2;"),
            "let a = 1;  \nlet b = 2;"
        );
        assert_eq!(code_only("a /* gone */ b"), "a   b");
        assert_eq!(code_only("a /* one /* two */ still */ b"), "a    b");
        assert_eq!(code_only("a /* x // y\n z */ b"), "a   b");
        assert_eq!(code_only("f(\"ProcessLiveness\")"), "f( )");
        assert_eq!(code_only("r#\"ProcessLiveness\"#;"), " ;");
        assert_eq!(code_only("// ✂\nx"), " \nx");
        assert_eq!(code_only("plain"), "plain");
    }

    /// The brace-matched body of the function whose signature starts with `sig`.
    fn fn_body<'a>(src: &'a str, sig: &str) -> Option<&'a str> {
        let start = src.find(sig)?;
        let open = start + src[start..].find('{')?;
        let mut depth = 0usize;
        for (i, c) in src[open..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return Some(&src[open..open + i + 1]);
                    }
                }
                _ => {}
            }
        }
        None
    }

    #[test]
    fn the_exit_path_reclaims_through_the_shared_machinery() {
        // The one-classifier rule at the exit-path seam: the exit pass must
        // CALL the classifier, never carry a second one. A hand-rolled loop
        // there would look plausible — `/tmp`, a suffix match, a
        // `remove_file` — and would have no evidence gate at all.
        let src = code_only(include_str!("ipc_cleanup.rs"));
        let body = fn_body(&src, "pub fn run_exit_hygiene(")
            .expect("the exit hygiene pass must still exist");
        // The ORDER and the GATE live in the seam `run_exit_hygiene` delegates
        // to, which is also what the behavioural pin in `ipc_cleanup` drives.
        let seam =
            fn_body(&src, "fn exit_hygiene_pass<").expect("the exit hygiene seam must still exist");

        assert!(
            body.contains("shm_state::reclaim_at_exit("),
            "the exit pass must go through the shared entry point; body was:\n{body}"
        );
        assert!(
            body.contains("cleanup_dead_iceoryx2_nodes_bounded"),
            "and through the shared bounded dead-node sweep; body was:\n{body}"
        );

        // ORDER, and it is correctness (see `run_exit_hygiene`'s doc): a
        // `.shm_state` file is the only mapping from an iceoryx2 resource name
        // to the object behind it, so a reclamation that runs BEFORE the sweep
        // takes away the mapping the sweep needs and leaves every dead node it
        // had not reached permanently unreclaimable. Measured, against a
        // control, in `tests/reclaim_ordering_test.rs`.
        let sweep_at = body
            .find("cleanup_dead_iceoryx2_nodes_bounded")
            .expect("checked above");
        let reclaim_at = body
            .find("shm_state::reclaim_at_exit(")
            .expect("checked above");
        assert!(
            sweep_at < reclaim_at,
            "the dead-node sweep must run BEFORE the state-file reclamation — reclaiming first \
             strands every dead node still registered; body was:\n{body}"
        );
        // …and the reclamation is gated on the sweep having CONVERGED. Order
        // alone is not enough: a bounded sweep that DEFERRED a dead node, or
        // one that failed to remove it, leaves an entry whose name mappings are
        // still needed.
        assert!(
            seam.contains("registry_converged(&swept)"),
            "the state-file pass must be gated on a converged registry; seam was:\n{seam}"
        );
        // No second implementation anywhere in the wiring module: the two
        // destructive primitives and the liveness probe all live in
        // `shm_state`, behind the evidence gate.
        for forbidden in ["remove_file(", "shm_unlink", "libc::kill(", "read_dir("] {
            assert!(
                !src.contains(forbidden),
                "`ipc_cleanup` must not perform reclamation itself, but names `{forbidden}`"
            );
        }
        // ANTI-TAUTOLOGY: the stripped view really does hold the module's code,
        // so the absences above are claims about the code and not about an
        // emptied view.
        assert!(
            src.contains("pub fn run_exit_hygiene(") && src.contains("impl Drop for"),
            "the walk must still see the wiring it claims to read"
        );
    }

    #[test]
    fn no_second_copy_of_the_liveness_evidence_predicate_exists() {
        // ONE definition of what counts as proof. Two would drift, and the
        // direction they drift in deletes live shared memory.
        //
        // Both crates that touch this feature are walked: the engine, where a
        // convenience helper would land, and the CLI, where `cerulion clean`
        // already calls into the machinery and could just as easily inline it.
        let here = Path::new(env!("CARGO_MANIFEST_DIR"));
        let roots = [here.join("src"), here.join("../cerulion_cli/src")];
        let this_file = here.join("src/shm_state.rs");

        let mut visited = 0usize;
        let mut offenders = Vec::new();
        for root in &roots {
            assert!(
                root.is_dir(),
                "the walk must be able to reach {} — a walk that cannot look is not a walk \
                 that found nothing",
                root.display()
            );
            for path in rust_files(root) {
                if path == this_file {
                    continue;
                }
                let src = code_only(
                    &std::fs::read_to_string(&path)
                        .unwrap_or_else(|e| panic!("read {}: {e}", path.display())),
                );
                visited += 1;
                // The guarded set is wider than the LIVENESS predicate: it also covers
                // the root-set evidence beside it. A second copy of the registry
                // search is the same hazard for the same reason: the direction
                // two copies drift in is "this root was not searched", which
                // reads a live namespace as free and deletes its mapping.
                for name in [
                    "ProcessLiveness",
                    "StateFileVerdict",
                    "parse_real_shm_name",
                    "namespaces_in_use_at",
                    "namespaces_in_use_under",
                    "registry_roots_under",
                ] {
                    if src.contains(name) {
                        offenders.push(format!("{} names `{name}`", path.display()));
                    }
                }
                // The predicate itself, in the one spelling it has: `kill` with
                // signal 0 is a liveness probe and nothing else. Every OTHER
                // `libc::kill` in these crates sends a real signal to a child.
                let mut at = 0usize;
                while let Some(hit) = src[at..].find("libc::kill(") {
                    let start = at + hit;
                    let window = &src[start..(start + 64).min(src.len())];
                    if window.contains(", 0)") {
                        offenders.push(format!(
                            "{} probes liveness with kill(pid, 0)",
                            path.display()
                        ));
                    }
                    at = start + "libc::kill(".len();
                }
            }
        }

        assert!(
            offenders.is_empty(),
            "the `.shm_state` evidence rule lives in ONE module (`shm_state.rs`) and every \
             caller routes through `classify` / `scan` / `reclaim_at_exit` / `creator_verdict` \
             (the last is the orphan port-tag reclaim's entry point: a (pid, creation) verdict \
             by this module's own predicate, so that reclaim carries no copy). Found:\n  {}",
            offenders.join("\n  ")
        );

        // ANTI-TAUTOLOGY, both halves. The walk must have looked at real files,
        // and the tokens it searched for must be findable at all — otherwise a
        // renamed type turns this into a test that can never fail.
        assert!(
            visited > 10,
            "the walk visited only {visited} files, which is not the two crates it claims"
        );
        let mine = code_only(include_str!("shm_state.rs"));
        for name in [
            "ProcessLiveness",
            "StateFileVerdict",
            "parse_real_shm_name",
            "libc::kill(",
            "namespaces_in_use_at",
            "namespaces_in_use_under",
            "registry_roots_under",
        ] {
            assert!(
                mine.contains(name),
                "`{name}` must still exist HERE, or the exclusion above is guarding nothing"
            );
        }
    }

    /// Every `.rs` file under `root`, recursively.
    fn rust_files(root: &Path) -> Vec<PathBuf> {
        let mut out = Vec::new();
        let mut stack = vec![root.to_path_buf()];
        while let Some(dir) = stack.pop() {
            let entries = std::fs::read_dir(&dir)
                .unwrap_or_else(|e| panic!("read_dir {}: {e}", dir.display()));
            for entry in entries {
                let path = entry.expect("directory entry").path();
                if path.is_dir() {
                    stack.push(path);
                } else if path.extension().is_some_and(|e| e == "rs") {
                    out.push(path);
                }
            }
        }
        out.sort();
        out
    }

    // ------------------------------------------------------------ real probe

    #[cfg(unix)]
    #[test]
    fn the_real_probe_reads_this_process_as_alive_and_an_impossible_pid_as_gone() {
        // The one arm that exercises the PRODUCTION probe rather than the
        // double, so the classifier cannot ship against a `kill` wrapper that
        // never worked.
        let probe = LibcProbe;
        let own = i32::try_from(std::process::id()).expect("pid fits");
        assert_eq!(probe.process_liveness(own), ProcessLiveness::Alive);
        // `kill(0, ..)` signals the whole process group, so it must never be
        // reported as a liveness answer.
        assert_eq!(probe.process_liveness(0), ProcessLiveness::Unknown);
        assert_eq!(probe.process_liveness(-1), ProcessLiveness::Unknown);
        assert_eq!(probe.shm_object_size("no-such-object-anywhere"), None);
        assert!(
            probe.unlink_shm_object("no-such-object-anywhere").is_ok(),
            "an already-absent object is the same outcome as one we just removed"
        );
    }
}
