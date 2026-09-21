// SPDX-License-Identifier: AGPL-3.0-only
//! Reclaim the SHM a SIGKILLed run left behind, at the next
//! `graph run`.
//!
//! # The leak this closes
//!
//! Three mechanisms exist today to remove a run's shared memory, and none of
//! them survives a SIGKILL:
//!
//! * the ring owner's `Drop` (`shm_ring::ShmRingOwner`) — a controlled exit only;
//! * `graph_cmd`'s `WorkerRingSweeper` — worker TRACE tags only, and armed only
//!   under `--record`;
//! * `state_ring::unlink_stale_state_rings` — SAME-TAG only, at arm time, which
//!   under the per-run derived tag `cer_run_{run_id:032x}` is a collision guard
//!   and never a cross-run reclaimer.
//!
//! So `kill -9` on a supervisor or a monolith leaves its `cer_st_*` state rings
//! (64.06 MiB apparent per rank), its `cer_sta_*` arm word, its trace rings
//! (40.06 MiB per rank once Flashback mints them on every run) and its run
//! directory standing until reboot. On an Orin NX-8 whose `/dev/shm` is ~3.7 GiB
//! that is a handful of crashes from exhaustion.
//!
//! # Why `run.json` IS the ledger
//!
//! Ring names are FNV hashes (`/cer_rg_<hash>`, `/cer_sta_<hash>`), so listing
//! `/dev/shm` yields names nothing can attribute to a run — a sweeper cannot
//! decide from the filesystem alone whether `/cer_rg_9f2c…` belongs to a dead
//! run or to the one that is running right now. `run.json` already exists on
//! EVERY Unix run, already survives SIGKILL, already carries `run_id`, and gains
//! the ring TAGS through `run_dir::declare_run_rings`. Everything else is
//! derivable from those two facts, so no second ledger file is introduced.
//!
//! # Scope
//!
//! This sweeps the FOUR families a run's ledger can name: trace rings, the
//! departure ring, per-rank state rings, and the arm word. It deliberately does
//! NOT sweep `/cer_bar_*` (barrier), `/cer_db_*` (doorbell) or wedge
//! pages — those are page-class objects with the same hashed-name problem, and
//! folding them in means teaching this module the multi-process lifecycle. They
//! are outside this sweep, and the module's "covers SIGKILL leaks" claim is
//! scoped to what it actually walks.
//!
//! # Safety posture
//!
//! Reclaiming is authorised by ONE thing: every lock in the directory was
//! ACQUIRABLE (see [`crate::run_lock`]). Everything else fails toward leaving
//! the directory alone — no ledger, an unparseable ledger, an absent lock, an
//! unreadable lock, or a name a LIVE run also claims. The cost of a wrong
//! reclaim is a live run's rings going unattachable; the cost of a wrong skip is
//! a bounded, warned leak that the next run retries.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use crate::run_dir::{RingDecl, RUN_MANIFEST_FILE};
use crate::run_lock::{probe_lock, LockState};

/// A run's SHM claim, as its `run.json` records it.
///
/// Deliberately tolerant: every field is optional, because this parses a
/// document written by a possibly-OLDER build. A ledger missing `rings` is a run
/// that created none (or died before declaring them), not a corrupt one.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RunLedger {
    /// The run's identity, from `run.json`'s `"0x…"` hex string.
    pub run_id: Option<u128>,
    /// Declared trace + departure ring TAGS (not resolved SHM names).
    pub rings: Vec<RingDecl>,
    /// The checkpoint arm tag this run named, and whether an operator chose it.
    pub state_arm_tag: Option<String>,
    /// `true` when the arm tag came from `CERULION_STATE_ARM_TAG`.
    ///
    /// This is the whole reason the source is recorded rather than re-derived: a
    /// DERIVED tag is `cer_run_{run_id:032x}` and therefore unique to one run, so
    /// sweeping it can never touch anybody else. An EXPLICIT tag is a string an
    /// operator typed and may have typed for TWO runs, so it is only sweepable
    /// after checking every live run's ledger.
    pub state_arm_explicit: bool,
}

/// PURE: read a run's SHM claim out of `run.json` bytes.
///
/// `None` when the bytes are not a JSON object — the fail-closed arm, because a
/// directory whose ledger cannot be read is one whose names cannot be
/// attributed, and an unattributable name must never be unlinked.
///
/// A PRESENT but oddly-typed field degrades to absent rather than failing the
/// whole parse: a `rings` entry missing its `tag` contributes nothing, and the
/// remaining entries are still reclaimable. That direction is safe (fewer names
/// swept = a smaller leak), and the alternative — refusing the document — would
/// let one malformed entry pin a whole run's rings forever.
#[must_use]
pub fn parse_run_ledger(bytes: &[u8]) -> Option<RunLedger> {
    let doc: serde_json::Value = serde_json::from_slice(bytes).ok()?;
    let obj = doc.as_object()?;
    let run_id = obj
        .get("run_id")
        .and_then(serde_json::Value::as_str)
        .and_then(|s| u128::from_str_radix(s.trim_start_matches("0x"), 16).ok());
    let rings = obj
        .get("rings")
        .and_then(serde_json::Value::as_array)
        .map(|a| {
            a.iter()
                .filter_map(|r| {
                    let tag = r.get("tag")?.as_str()?.to_string();
                    // A ring whose rank is missing or out of range still has a
                    // NAME, which is the only thing a sweep needs; the rank is
                    // carried for the log line.
                    let rank = r
                        .get("rank")
                        .and_then(serde_json::Value::as_u64)
                        .and_then(|v| u32::try_from(v).ok())
                        .unwrap_or(0);
                    Some(RingDecl { tag, rank })
                })
                .collect()
        })
        .unwrap_or_default();
    // The ARM word's tag comes from the ONE ledger vocabulary
    // (`run_dir::run_manifest_shm` → `ShmDecl`), never from a second spelling of
    // the same fact. The `class` is what selects it — `ShmClass::Arm` — so a
    // future writer adding a class this sweeper does not handle is IGNORED here
    // rather than mis-read as an arm tag.
    let arm_decl = crate::run_dir::run_manifest_shm(bytes)
        .into_iter()
        .find(|d| d.class == crate::run_dir::ShmClass::Arm);
    let declared_tag = arm_decl.as_ref().map(|d| d.tag.clone());
    let state_arm_explicit = arm_decl
        .as_ref()
        .is_some_and(|d| d.source == crate::run_dir::TagSource::Explicit);
    // AN ABSENT CLASS IS *UNDECLARED*, NEVER "THIS RUN MINTED NONE".
    //
    // This is the rule that decides whether the sweeper works on exactly the
    // runs it exists for. A ledger is written by a process that then CRASHED, so
    // "the arm block is missing" is at least as likely to mean "it died before
    // recording it" as "it never armed a plane" — and treating absence as
    // nothing-to-sweep skips precisely the objects a partially-written ledger
    // failed to name.
    //
    // The fallback is NAME-PATTERN discovery BOUNDED BY THIS RUN'S OWN IDENTITY:
    // `state_arm_tag_for_run(run_id)` embeds the 128-bit run id, so the names it
    // derives can belong to no other run and sweeping them is safe even if this
    // run in fact armed nothing (an absent name is a silent ENOENT). What it
    // cannot recover is an EXPLICIT `CERULION_STATE_ARM_TAG` nobody wrote down —
    // that one leaks, and leaking is the direction this module always fails in.
    //
    // Derived by construction, so it is never subject to the live-collision
    // refusal: a name carrying this run's id cannot be a live run's claim.
    let state_arm_tag =
        declared_tag.or_else(|| run_id.map(cerulion_core::state_arm::state_arm_tag_for_run));
    Some(RunLedger {
        run_id,
        rings,
        state_arm_tag,
        // A DERIVED fallback is derived, whatever the missing entry would have
        // said. Marking it explicit would subject a name that provably belongs
        // to this run alone to a collision check it can never need.
        state_arm_explicit: declared_tag_was_explicit(arm_decl.is_some(), state_arm_explicit),
    })
}

/// `true` only when the ledger REALLY declared an explicit tag.
///
/// Split out so the "a derived fallback is never explicit" rule is one named
/// place rather than a boolean expression a later edit can invert by accident.
fn declared_tag_was_explicit(had_entry: bool, source_said_explicit: bool) -> bool {
    had_entry && source_said_explicit
}

/// Why one name in a dead run's ledger was left standing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefuseReason {
    /// A LIVE run's ledger claims the same name. Only reachable through an
    /// explicit `CERULION_STATE_ARM_TAG` shared by two runs — a derived tag is
    /// unique per run by construction.
    ClaimedByLiveRun,
}

impl RefuseReason {
    /// The operator-facing phrase for a log line.
    #[must_use]
    pub fn why(self) -> &'static str {
        match self {
            RefuseReason::ClaimedByLiveRun => {
                "a LIVE run's ledger claims the same CERULION_STATE_ARM_TAG"
            }
        }
    }
}

/// PURE: what reclaiming one dead run's ledger would touch.
///
/// Split from the executor because this is the POLICY half — which names are in
/// scope, which are refused, and whether the directory itself may go — and it is
/// the half that must never be wrong. The executor is mechanism: `shm_unlink`
/// and `remove_dir_all`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ReclaimPlan {
    /// Fully-resolved POSIX SHM object names to unlink (trace rings, the
    /// departure ring, the arm word).
    pub unlink_names: Vec<String>,
    /// Arm tags whose per-rank state rings are swept by a rank SCAN.
    ///
    /// A tag rather than a name list because the rank space is not knowable from
    /// the ledger: `state_ring::unlink_stale_state_rings` walks it with the
    /// crate's own gap tolerance, and duplicating that walk here would be a
    /// second copy of a rule that already has one home.
    pub state_tags: Vec<String>,
    /// Names/tags deliberately left standing, with the reason.
    pub refused: Vec<(String, RefuseReason)>,
}

/// PURE: plan the reclaim of `ledger`, given the tags LIVE runs currently claim.
///
/// `live_state_tags` holds the arm tags of every directory that probed
/// [`LockState::Held`] in the same walk. It exists for one shape: two runs
/// started with the SAME `CERULION_STATE_ARM_TAG`, one of them since killed.
/// Sweeping the dead one's tag would unlink the LIVE one's state rings and arm
/// word, because they are literally the same names.
#[must_use]
pub fn plan_reclaim(ledger: &RunLedger, live_state_tags: &BTreeSet<String>) -> ReclaimPlan {
    let mut plan = ReclaimPlan::default();
    // Trace + departure rings. Their tags carry the supervisor pid and the run's
    // graph name, so two LIVE runs cannot collide on one; no live check is
    // needed and none is performed (a check keyed on a set that can never
    // contain them would be inert code pretending to be a guard).
    for ring in &ledger.rings {
        plan.unlink_names
            .push(cerulion_core::shm_ring::ring_shm_name(&ring.tag));
    }
    if let Some(tag) = &ledger.state_arm_tag {
        if ledger.state_arm_explicit && live_state_tags.contains(tag) {
            plan.refused
                .push((tag.clone(), RefuseReason::ClaimedByLiveRun));
        } else {
            plan.state_tags.push(tag.clone());
            plan.unlink_names
                .push(cerulion_core::state_arm::state_arm_shm_name(tag));
        }
    }
    plan
}

/// What one sweep did, for the caller's log line and for tests.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SweepReport {
    /// Run directories reclaimed.
    pub runs_reclaimed: usize,
    /// SHM object names successfully unlinked.
    pub names_unlinked: usize,
    /// Apparent bytes those names accounted for — the figure the warn quotes.
    pub bytes_reclaimed: u64,
    /// Directories left alone because a lock was HELD (a live run).
    pub live_skipped: usize,
    /// Directories left alone for an UNKNOWN reason: no ledger, an unparseable
    /// ledger, or a lock we could not read. Reported separately from
    /// `live_skipped` because it is the only arm an operator might need to act
    /// on.
    pub unknown_skipped: usize,
    /// Names left standing because a live run claims them.
    pub names_refused: usize,
    /// Dead runs where at least one name SURVIVED the reclaim — a refusal, or an
    /// `shm_unlink` that failed. Their directories are KEPT (the ledger a later
    /// sweep retries from) and they are deliberately NOT counted in
    /// `runs_reclaimed`, because they were not.
    pub runs_partially_reclaimed: usize,
    /// Directories that classified DEAD in pass 1 and were something else by the
    /// time pass 2 reached them — the mid-sweep attach the re-probe exists for.
    ///
    /// Reported rather than folded into the skip counters alone, because it is
    /// the only evidence the re-check ever DOES anything: a sweep that stopped
    /// re-probing would leave this permanently zero while every other number in
    /// the report stayed plausible.
    pub candidates_reclassified: usize,
}

impl SweepReport {
    /// Did this sweep actually reclaim anything? The warn is emitted only when
    /// it did — a clean machine's every `graph run` must not narrate a no-op.
    #[must_use]
    pub fn reclaimed_anything(&self) -> bool {
        self.runs_reclaimed > 0 || self.names_unlinked > 0
    }
}

/// One trace ring's apparent size: the 64 KiB-class header + manifest plus
/// `2^20 × 40 B` of records. Quoted in the warn so the operator sees what was
/// actually recovered rather than a count of hashes.
///
/// This is the APPARENT size — a ring's RESIDENT footprint is
/// `header + 40 B × min(pushed, capacity)` because tmpfs pages are demand-faulted
/// — so the figure is an upper bound, and the warn says "up to". Reporting the
/// resident figure instead would need `/proc/<pid>/smaps` of a process that no
/// longer exists.
const TRACE_RING_APPARENT_BYTES: u64 = 65_600 + (1 << 20) * 40;

/// One state ring's apparent size: `2^17 × 512 B` of records plus the same
/// header class.
const STATE_RING_APPARENT_BYTES: u64 = 65_600 + (1 << 17) * 512;

/// The arm word is one page-class object.
const ARM_WORD_APPARENT_BYTES: u64 = 4096;

/// Reclaim every DEAD run's SHM under `root`.
///
/// Bounded by construction: one `read_dir`, then per directory one lock-file
/// listing, one `flock` try per lock, one `run.json` read, and one `shm_unlink`
/// per declared name plus the state-ring rank scan. Nothing blocks — every lock
/// probe is `LOCK_NB` — so this cannot delay a `graph run`'s start behind
/// another run's lifetime.
///
/// # Never fails a run
///
/// Returns a report, never an error. A sweep is housekeeping; a machine whose
/// `~/.cerulion/runs` cannot be walked must still be able to run a graph. Every
/// I/O failure degrades to "leave that directory alone".
///
/// # Self-exclusion is structural
///
/// The caller holds its OWN `run.lock` before calling this, so its directory
/// probes [`LockState::Held`] through the kernel and is skipped as a live run —
/// there is no path comparison to get wrong.
pub fn sweep_stale_runs(root: &Path) -> SweepReport {
    sweep_stale_runs_with_hook(root, &mut || {})
}

/// [`sweep_stale_runs`] with a seam that runs ONCE between the classification
/// pass and the reclaim pass.
///
/// The window between the two passes is real but microseconds wide, so a test
/// that tried to RACE into it would be timing-dependent in the one direction
/// that matters — a flake there reads as "the re-probe works" rather than as a
/// failure. The seam removes the scheduling luck while leaving the mechanism
/// exactly as production runs it: the hook does nothing a concurrent process
/// could not do, and pass 2's per-directory re-probe is the same code either
/// way. (`shm_ring`'s `drain_with_pre_commit_hook_for_test` is the precedent.)
///
/// Not `#[cfg(test)]`: the test that needs it is an INTEGRATION test, which
/// compiles against this crate as an ordinary dependency.
#[doc(hidden)]
pub fn sweep_stale_runs_with_hook(root: &Path, between_passes: &mut dyn FnMut()) -> SweepReport {
    let mut report = SweepReport::default();
    let Ok(entries) = std::fs::read_dir(root) else {
        // No run root yet (the common case on a fresh machine) or one this sweep
        // cannot read. Nothing to say.
        return report;
    };
    // PASS 1: classify every directory, and collect the arm tags LIVE runs
    // claim. It has to be a separate pass — a dead run sharing an explicit tag
    // with a live one may be walked FIRST, and a single-pass sweep would unlink
    // the live run's rings before ever seeing it.
    let mut candidates: Vec<(PathBuf, RunLedger)> = Vec::new();
    let mut live_state_tags: BTreeSet<String> = BTreeSet::new();
    for entry in entries.flatten() {
        let dir = entry.path();
        if !dir.is_dir() {
            continue;
        }
        let ledger = std::fs::read(dir.join(RUN_MANIFEST_FILE))
            .ok()
            .and_then(|b| parse_run_ledger(&b));
        match classify_dir(&dir) {
            DirVerdict::Live => {
                report.live_skipped += 1;
                if let Some(tag) = ledger.and_then(|l| l.state_arm_tag) {
                    live_state_tags.insert(tag);
                }
            }
            DirVerdict::Unknown => {
                report.unknown_skipped += 1;
                // A directory this sweep cannot classify may still be LIVE, so its tag
                // joins the protected set too: refusing to sweep a name an
                // unknown run might hold is the same fail-toward-not-sweeping
                // rule one level up.
                if let Some(tag) = ledger.and_then(|l| l.state_arm_tag) {
                    live_state_tags.insert(tag);
                }
            }
            DirVerdict::Dead => match ledger {
                // A dead run with NO readable ledger names nothing this sweep
                // could attribute. The directory is left standing rather than
                // removed: removing it would destroy the only evidence that a
                // leak happened, and it costs bytes, not megabytes.
                None => report.unknown_skipped += 1,
                Some(l) => candidates.push((dir, l)),
            },
        }
    }
    between_passes();
    // PASS 2: execute, now that every live claim is known.
    for (dir, ledger) in candidates {
        // RE-PROBE IMMEDIATELY BEFORE RECLAIMING THIS DIRECTORY.
        //
        // Pass 1's verdict is a SNAPSHOT, and everything between the two passes
        // is time an attacher can arrive in: `bag record --run` resolves a run
        // and flocks `attach-<pid>.lock`, and a run directory it picked up in
        // that window would be classified dead HERE and reclaimed out from under
        // a process that is actively reading it. The window is not
        // instantaneous either — pass 1 walks every directory, and pass 2 does
        // an `shm_unlink` per name plus a state-ring rank scan for each earlier
        // candidate before reaching this one.
        //
        // A re-probe cannot make the decision atomic — nothing can, without a
        // lock over the whole root, which would serialise every starting run
        // behind every sweep — but it does not need to. It SHRINKS the window
        // from "the whole sweep" to "one classification", and it fails in the
        // safe direction: the only thing that can appear inside the remaining
        // window is a NEW lock, which the next sweep sees.
        //
        // Live AND Unknown are both skipped, i.e. only a still-`Dead` verdict
        // proceeds — the same fail-toward-not-sweeping rule pass 1 applies, and
        // the reason `matches!(.., Dead)` is written rather than `!= Live`.
        let verdict = classify_dir(&dir);
        if !matches!(verdict, DirVerdict::Dead) {
            tracing::debug!(
                run_dir = %dir.display(),
                "a run directory changed state between classification and \
                 reclaim — leaving it alone (a mid-sweep attach is the shape this \
                 re-check exists for)"
            );
            match verdict {
                DirVerdict::Live => report.live_skipped += 1,
                _ => report.unknown_skipped += 1,
            }
            // The pass 1 tallies already counted this directory as a candidate
            // rather than a skip, so correct that here — a report must describe
            // what the sweep DID, not what it planned to do.
            report.candidates_reclassified += 1;
            continue;
        }
        let plan = plan_reclaim(&ledger, &live_state_tags);
        report.names_refused += plan.refused.len();
        for (name, reason) in &plan.refused {
            tracing::warn!(
                run_dir = %dir.display(),
                name = %name,
                reason = reason.why(),
                "left a dead run's checkpoint plane standing — it shares an \
                 explicit CERULION_STATE_ARM_TAG with a run that is still alive"
            );
        }
        let mut names = 0usize;
        let mut bytes = 0u64;
        // NAMES THAT SURVIVED THIS RECLAIM, and why. The directory is the LEDGER
        // a later sweep retries from, so it may only go once every name it
        // accounts for is really gone — see the removal decision below.
        let mut survivors: Vec<String> = Vec::new();
        for name in &plan.unlink_names {
            match unlink_shm_name(name) {
                UnlinkOutcome::Removed => {
                    names += 1;
                    // An arm word and a ring are wildly different sizes and the
                    // ledger says which is which by construction: the arm word is
                    // the ONLY `/cer_sta_` name a plan can carry.
                    bytes += if name.starts_with("/cer_sta_") {
                        ARM_WORD_APPARENT_BYTES
                    } else {
                        TRACE_RING_APPARENT_BYTES
                    };
                }
                // Nothing was there. Not a survivor: an already-gone name is the
                // ordinary case (the owner's `Drop` ran, or an earlier sweep got
                // there), and treating it as one would pin every clean run's
                // directory forever.
                UnlinkOutcome::Absent => {}
                UnlinkOutcome::Failed(why) => {
                    tracing::warn!(
                        run_dir = %dir.display(),
                        name = %name,
                        error = %why,
                        "could not unlink a dead run's shared memory"
                    );
                    survivors.push(name.clone());
                }
            }
        }
        for tag in &plan.state_tags {
            let sweep = cerulion_core::state_ring::unlink_stale_state_rings(tag);
            names += sweep.removed.len();
            bytes += sweep.removed.len() as u64 * STATE_RING_APPARENT_BYTES;
            for (rank, why) in &sweep.refused {
                tracing::warn!(
                    run_dir = %dir.display(),
                    tag = %tag,
                    rank = rank,
                    error = %why,
                    "could not unlink a dead run's state ring"
                );
                survivors.push(format!("{tag} rank {rank}"));
            }
        }
        // A REFUSAL IS ALSO A SURVIVOR. `plan.refused` names what policy declined
        // to touch (a live run's shared explicit tag); `survivors` names what the
        // filesystem declined to remove. Both mean a name in this ledger is still
        // standing, and both must keep the ledger — which is why `remove_dir` is
        // decided HERE, from the outcomes, rather than by the plan alone.
        let refused_names = plan.refused.iter().map(|(n, _)| n.clone());
        survivors.extend(refused_names);
        // THE DIRECTORY GOES ONLY IF NOTHING SURVIVED.
        //
        // `run.json` is the only thing that can attribute a hashed
        // `/cer_rg_…` name to a run, so removing it while a name it declares is
        // still there STRANDS that name permanently: no later sweep can know
        // whose it was, and `/dev/shm` cannot say. The leak stops being bounded
        // and stops being retryable in the same instant.
        //
        // The run is likewise NOT counted reclaimed, because it was not: a
        // report claiming a reclaim over a run whose memory is still resident is
        // the silent-success class this whole module is shaped against.
        if survivors.is_empty() {
            if let Err(e) = std::fs::remove_dir_all(&dir) {
                if e.kind() != std::io::ErrorKind::NotFound {
                    tracing::debug!(
                        run_dir = %dir.display(),
                        error = %e,
                        "reclaimed a dead run's shared memory but could not remove \
                         its directory"
                    );
                }
            }
            report.runs_reclaimed += 1;
        } else {
            report.runs_partially_reclaimed += 1;
            tracing::warn!(
                run_dir = %dir.display(),
                survivors = survivors.len(),
                names = %survivors.join(", "),
                "could not fully reclaim a dead run — its directory is KEPT as the \
                 ledger a later `graph run` retries from, because removing it would leave \
                 these names unattributable forever"
            );
        }
        report.names_unlinked += names;
        report.bytes_reclaimed += bytes;
        // Only when something was ACTUALLY removed. A dead run whose ledger
        // named objects that were all already gone (the ordinary stale-directory
        // case) reclaims a directory and no memory, and announcing "reclaimed
        // the shared memory of a run that did not shut down" over zero bytes is
        // a claim about a leak that did not happen.
        if names > 0 {
            tracing::warn!(
                run_dir = %dir.display(),
                run_id = %ledger
                    .run_id
                    .map_or_else(|| "unknown".to_string(), |id| format!("0x{id:032x}")),
                names = names,
                bytes = bytes,
                "reclaimed the shared memory of a run that did not shut down — its \
                 process was killed, so nothing unlinked these names at exit"
            );
        }
    }
    report
}

/// The three ways a run directory can be classified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DirVerdict {
    /// At least one lock is HELD — somebody is running.
    Live,
    /// Every lock present was ACQUIRABLE, and there was at least one.
    Dead,
    /// No lock at all, or one we could not read.
    Unknown,
}

/// Classify one run directory from its lock files alone.
///
/// # Why EVERY lock must be free, not just `run.lock`
///
/// A supervisor SIGKILL does not necessarily take its workers with it — nothing
/// in this repo installs `PDEATHSIG`, and whether workers self-terminate on the
/// barrier/wedge path is unverified. So a directory can hold a FREE `run.lock`
/// beside a HELD `worker-3.lock`, meaning the supervisor is gone but rank 3 is
/// still stepping and still writing its rings. Sweeping on `run.lock` alone
/// would unlink a ring under a live writer.
///
/// The same applies to `attach-<pid>.lock`: a `bag record --run` attacher
/// outlives its run by design (it finalises a bag), and is still reading the
/// directory.
fn classify_dir(dir: &Path) -> DirVerdict {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return DirVerdict::Unknown;
    };
    let mut saw_lock = false;
    let mut all_free = true;
    for entry in entries.flatten() {
        let name = entry.file_name();
        let Some(name) = name.to_str() else { continue };
        if !crate::run_lock::is_lock_file(name) {
            continue;
        }
        saw_lock = true;
        match probe_lock(&entry.path()) {
            LockState::Held => return DirVerdict::Live,
            s if s.proves_dead() => {}
            // Absent (raced away between listing and probing) or Unreadable:
            // one unknown lock makes the whole directory unknown.
            _ => all_free = false,
        }
    }
    if !saw_lock {
        // A directory written before this module existed, or one killed inside
        // the `mkdir`→flock window. Its run may be live and its names cannot be
        // proven orphaned, so it is never swept. The `run.lock`-before-`run.json`
        // ordering means the second case carries no ledger anyway.
        return DirVerdict::Unknown;
    }
    if all_free {
        DirVerdict::Dead
    } else {
        DirVerdict::Unknown
    }
}

/// What one `shm_unlink` did — THREE outcomes, not two.
///
/// The old `bool` collapsed the two that matter most: an ENOENT (nothing was
/// there) and an EPERM/EACCES (something IS there and we could not remove it)
/// both read `false`, so a name left standing was indistinguishable from a name
/// that never existed — and the caller removed the ledger in both cases,
/// stranding the first kind forever.
#[derive(Debug, Clone, PartialEq, Eq)]
enum UnlinkOutcome {
    /// The name was removed.
    Removed,
    /// Nothing was there. The ORDINARY case (the owner's `Drop` already ran, or
    /// a previous sweep got there) and deliberately silent — a sweep of an
    /// already-clean run must not narrate housekeeping on every `graph run`.
    Absent,
    /// The name EXISTS and could not be unlinked (a foreign owner, a different
    /// uid). The object is still occupying memory and still belongs to this
    /// ledger, so its run directory has to stay.
    Failed(String),
}

// Test-only fault seam: when armed with a name, that name reports
// `UnlinkOutcome::Failed`.
//
// A REAL `shm_unlink` failure needs an object this uid cannot remove (EPERM on a
// foreign owner), which a test cannot arrange without root — and it is the one
// outcome the survivor rule exists for, so leaving it undriven would mean the
// rule ships pinned only by the refusal half. THREAD-SCOPED for the same reason
// `run_lock`'s flock seam is: libtest runs tests in parallel, and a
// process-wide arm makes concurrent siblings fail an unlink they never asked to.
#[cfg(test)]
thread_local! {
    static FAULT_UNLINK: std::cell::RefCell<Option<String>> =
        const { std::cell::RefCell::new(None) };
}

/// Arm [`FAULT_UNLINK`] for the lifetime of the returned guard.
#[cfg(test)]
fn fault_inject_unlink(shm_name: &str) -> impl Drop {
    struct Guard;
    impl Drop for Guard {
        fn drop(&mut self) {
            FAULT_UNLINK.with(|f| *f.borrow_mut() = None);
        }
    }
    FAULT_UNLINK.with(|f| *f.borrow_mut() = Some(shm_name.to_string()));
    Guard
}

/// `shm_unlink` one name.
fn unlink_shm_name(name: &str) -> UnlinkOutcome {
    #[cfg(test)]
    if FAULT_UNLINK.with(|f| f.borrow().as_deref() == Some(name)) {
        return UnlinkOutcome::Failed("injected: operation not permitted".to_string());
    }
    #[cfg(unix)]
    {
        let Ok(c_name) = std::ffi::CString::new(name) else {
            // A name with an interior NUL cannot have been created by this
            // crate, so nothing of ours is standing under it.
            return UnlinkOutcome::Absent;
        };
        // SAFETY: FFI unlink of a NUL-terminated name derived from this run's
        // own ledger; `shm_unlink` has no precondition a caller can violate, and
        // it removes only the NAME (a live mapping of the object survives).
        let rc = unsafe { libc::shm_unlink(c_name.as_ptr()) };
        if rc == 0 {
            return UnlinkOutcome::Removed;
        }
        let err = std::io::Error::last_os_error();
        if err.kind() == std::io::ErrorKind::NotFound {
            UnlinkOutcome::Absent
        } else {
            UnlinkOutcome::Failed(err.to_string())
        }
    }
    #[cfg(not(unix))]
    {
        let _ = name;
        UnlinkOutcome::Absent
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ledger_bytes(v: serde_json::Value) -> Vec<u8> {
        serde_json::to_vec(&v).expect("json")
    }

    /// A full ledger round-trips into every field. Hand oracle — the values are
    /// written here and asserted here, never read back out of the parser's own
    /// output.
    #[test]
    fn a_full_ledger_parses_into_every_field() {
        let bytes = ledger_bytes(serde_json::json!({
            "version": 1,
            "run_id": "0x0000000000000000000000000000002a",
            "rings": [
                { "tag": "cer_rec_demo_991_r0", "rank": 0 },
                { "tag": "cer_rec_demo_991_r1", "rank": 1 },
                { "tag": "cer_rec_demo_991_dep", "rank": 4_294_967_295u32 },
            ],
            "shm": [
                { "tag": "cer_run_0000002a", "class": "arm", "rank": null, "source": "derived" },
            ],
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        assert_eq!(l.run_id, Some(0x2a));
        assert_eq!(
            l.rings,
            vec![
                RingDecl {
                    tag: "cer_rec_demo_991_r0".to_string(),
                    rank: 0
                },
                RingDecl {
                    tag: "cer_rec_demo_991_r1".to_string(),
                    rank: 1
                },
                RingDecl {
                    tag: "cer_rec_demo_991_dep".to_string(),
                    rank: u32::MAX
                },
            ]
        );
        assert_eq!(l.state_arm_tag.as_deref(), Some("cer_run_0000002a"));
        assert!(!l.state_arm_explicit, "`derived` is not explicit");
    }

    /// An EXPLICIT source must be recognised — it is the only thing that turns
    /// on the live-collision check, so reading it as derived would silently
    /// re-open the shared-tag hazard.
    #[test]
    fn an_explicit_arm_source_is_recognised_and_nothing_else_is() {
        for (source, want) in [
            ("explicit", true),
            ("derived", false),
            ("Explicit", false),
            ("", false),
        ] {
            let bytes = ledger_bytes(serde_json::json!({
                "shm": [
                    { "tag": "hand_picked", "class": "arm", "rank": null, "source": source },
                ],
            }));
            let l = parse_run_ledger(&bytes).expect("parses");
            assert_eq!(
                l.state_arm_explicit, want,
                "source `{source}` must read explicit={want}"
            );
        }
        // An entry with NO source is DROPPED by `run_manifest_shm` — that
        // reader refuses anything it cannot fully understand, because this
        // vector feeds a sweeper that UNLINKS. So the ledger declares no arm at
        // all and the run falls through to the derived fallback, which is
        // strictly safer than guessing a source for it.
        let bytes = ledger_bytes(serde_json::json!({
            "run_id": "0x000000000000000000000000000000ff",
            "shm": [{ "tag": "t", "class": "arm", "rank": null }],
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        assert!(!l.state_arm_explicit);
        assert_eq!(
            l.state_arm_tag.as_deref(),
            Some(cerulion_core::state_arm::state_arm_tag_for_run(0xff).as_str()),
            "a source-less entry is not readable, so it declares nothing"
        );
    }

    /// An older `run.json` parses to an EMPTY claim rather than failing:
    /// its directory is then a dead run with nothing to unlink, and its lock
    /// files are absent so it is never reached anyway.
    #[test]
    fn a_ledger_with_no_shm_keys_parses_empty() {
        let bytes = ledger_bytes(serde_json::json!({
            "version": 1,
            "run_id": "0x00000000000000000000000000000001",
            "graph_name": "demo",
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        assert_eq!(l.run_id, Some(1));
        assert!(l.rings.is_empty());
        // NOT `None`: an absent class is UNDECLARED, so the checkpoint plane's
        // names fall back to derivation from this run's own id — see
        // `an_undeclared_state_arm_falls_back_to_this_runs_own_derived_names`.
        assert_eq!(
            l.state_arm_tag.as_deref(),
            Some(cerulion_core::state_arm::state_arm_tag_for_run(1).as_str())
        );
        assert!(!l.state_arm_explicit);
    }

    /// THE UNDECLARED-CLASS RULE: a ledger that names no checkpoint plane is a
    /// ledger whose writer may simply have crashed before recording one, so the
    /// sweeper falls back to the names this run's own identity DERIVES rather
    /// than concluding there is nothing to sweep.
    ///
    /// Bounded by the run id, so the fallback can never reach another run's
    /// objects — which is what makes it safe to sweep names this run may never
    /// have created (an absent name is a silent ENOENT).
    #[test]
    fn an_undeclared_state_arm_falls_back_to_this_runs_own_derived_names() {
        let bytes = ledger_bytes(serde_json::json!({
            "run_id": "0x000000000000000000000000000000ff",
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        let derived = cerulion_core::state_arm::state_arm_tag_for_run(0xff);
        assert_eq!(l.state_arm_tag.as_deref(), Some(derived.as_str()));
        assert!(
            !l.state_arm_explicit,
            "a DERIVED fallback is never explicit — marking it so would subject a \
             name that provably belongs to this run alone to a collision check it \
             can never need"
        );

        // …and it really reaches the plan, so the fallback is not a field nobody
        // reads.
        let plan = plan_reclaim(&l, &BTreeSet::new());
        assert_eq!(plan.state_tags, vec![derived.clone()]);
        assert_eq!(
            plan.unlink_names,
            vec![cerulion_core::state_arm::state_arm_shm_name(&derived)]
        );

        // With NO run id there is nothing to derive FROM, and inventing a name
        // would be exactly the unattributable unlink this module forbids.
        let idless = parse_run_ledger(&ledger_bytes(serde_json::json!({}))).expect("parses");
        assert_eq!(idless.state_arm_tag, None);
    }

    /// A DECLARED tag still wins over the fallback — otherwise an operator's
    /// explicit `CERULION_STATE_ARM_TAG` would be silently replaced by a derived
    /// name and the real objects would leak.
    #[test]
    fn a_declared_tag_is_used_in_preference_to_the_derived_fallback() {
        let bytes = ledger_bytes(serde_json::json!({
            "run_id": "0x000000000000000000000000000000ff",
            "shm": [
                { "tag": "hand_picked", "class": "arm", "rank": null, "source": "explicit" },
            ],
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        assert_eq!(l.state_arm_tag.as_deref(), Some("hand_picked"));
        assert!(l.state_arm_explicit);
    }

    /// Fail-CLOSED: bytes that are not a JSON OBJECT yield `None`, and the
    /// sweeper turns `None` into "leave the directory alone".
    #[test]
    fn an_unparseable_ledger_is_none_not_an_empty_claim() {
        assert_eq!(parse_run_ledger(b"not json"), None);
        assert_eq!(parse_run_ledger(b""), None);
        assert_eq!(parse_run_ledger(b"[1,2,3]"), None, "an array is not a run");
        assert_eq!(parse_run_ledger(b"null"), None);
    }

    /// A malformed ENTRY degrades to absent while its siblings survive — one bad
    /// record must not pin a whole run's rings.
    #[test]
    fn a_malformed_ring_entry_drops_without_taking_its_siblings() {
        let bytes = ledger_bytes(serde_json::json!({
            "rings": [
                { "rank": 0 },
                { "tag": "good_r1", "rank": 1 },
                { "tag": 7, "rank": 2 },
                { "tag": "good_r3" },
            ],
        }));
        let l = parse_run_ledger(&bytes).expect("parses");
        assert_eq!(
            l.rings,
            vec![
                RingDecl {
                    tag: "good_r1".to_string(),
                    rank: 1
                },
                // A rank-less entry keeps its name and defaults the rank — the
                // name is all a sweep needs.
                RingDecl {
                    tag: "good_r3".to_string(),
                    rank: 0
                },
            ]
        );
    }

    /// The headline plan: every declared ring resolves to its hashed SHM name,
    /// the arm word joins them, and the state tag is carried for the rank scan.
    ///
    /// The expected names are computed with the SAME public derivations the
    /// producers use, because a hand-typed hash would pin this test to today's
    /// FNV recipe rather than to the property under test (that the plan uses the
    /// one recipe in the system).
    #[test]
    fn a_derived_tag_plan_covers_rings_state_and_the_arm_word() {
        let ledger = RunLedger {
            run_id: Some(0x2a),
            rings: vec![
                RingDecl {
                    tag: "cer_rec_demo_991_r0".to_string(),
                    rank: 0,
                },
                RingDecl {
                    tag: "cer_rec_demo_991_dep".to_string(),
                    rank: u32::MAX,
                },
            ],
            state_arm_tag: Some("cer_run_0000002a".to_string()),
            state_arm_explicit: false,
        };
        let plan = plan_reclaim(&ledger, &BTreeSet::new());
        assert_eq!(
            plan.unlink_names,
            vec![
                cerulion_core::shm_ring::ring_shm_name("cer_rec_demo_991_r0"),
                cerulion_core::shm_ring::ring_shm_name("cer_rec_demo_991_dep"),
                cerulion_core::state_arm::state_arm_shm_name("cer_run_0000002a"),
            ]
        );
        assert_eq!(plan.state_tags, vec!["cer_run_0000002a".to_string()]);
        assert!(plan.refused.is_empty());
        assert!(
            plan.refused.is_empty(),
            "nothing refused ⇒ nothing keeps the ledger"
        );
    }

    /// THE safety oracle: a dead run whose EXPLICIT arm tag is also claimed by a
    /// LIVE run keeps its checkpoint plane, and its directory survives so the
    /// next run can retry.
    ///
    /// Its trace rings are still reclaimed — they are named from the supervisor
    /// pid and cannot be shared — so the refusal is scoped to the names that can
    /// actually collide rather than pinning everything.
    #[test]
    fn an_explicit_tag_a_live_run_claims_is_refused_but_its_rings_are_not() {
        let ledger = RunLedger {
            run_id: Some(7),
            rings: vec![RingDecl {
                tag: "cer_rec_demo_991_r0".to_string(),
                rank: 0,
            }],
            state_arm_tag: Some("shared_by_hand".to_string()),
            state_arm_explicit: true,
        };
        let live: BTreeSet<String> = ["shared_by_hand".to_string()].into_iter().collect();
        let plan = plan_reclaim(&ledger, &live);
        assert_eq!(
            plan.unlink_names,
            vec![cerulion_core::shm_ring::ring_shm_name(
                "cer_rec_demo_991_r0"
            )],
            "the arm word must NOT be in the unlink set"
        );
        assert!(plan.state_tags.is_empty(), "no rank scan under a live tag");
        assert_eq!(
            plan.refused,
            vec![("shared_by_hand".to_string(), RefuseReason::ClaimedByLiveRun)]
        );
        assert!(
            !plan.refused.is_empty(),
            "a refusal is what keeps the ledger — the executor reads it, so an \
             empty refusal list here means the directory goes"
        );
    }

    /// The collision check is scoped to EXPLICIT tags. A derived tag is
    /// `cer_run_{run_id}` and unique per run, so a set membership can only be a
    /// false positive — and treating it as a refusal would make a dead run's
    /// state plane permanently unsweepable.
    #[test]
    fn a_derived_tag_is_swept_even_if_the_same_string_appears_live() {
        let ledger = RunLedger {
            run_id: Some(0x2a),
            rings: Vec::new(),
            state_arm_tag: Some("cer_run_0000002a".to_string()),
            state_arm_explicit: false,
        };
        let live: BTreeSet<String> = ["cer_run_0000002a".to_string()].into_iter().collect();
        let plan = plan_reclaim(&ledger, &live);
        assert_eq!(plan.state_tags, vec!["cer_run_0000002a".to_string()]);
        assert!(plan.refused.is_empty());
        assert!(plan.refused.is_empty());
    }

    /// An EXPLICIT tag NO live run claims is swept normally — the anti-tautology
    /// half of the collision test (without it, refusing every explicit tag would
    /// pass).
    #[test]
    fn an_explicit_tag_no_live_run_claims_is_swept() {
        let ledger = RunLedger {
            state_arm_tag: Some("hand_picked".to_string()),
            state_arm_explicit: true,
            ..RunLedger::default()
        };
        let plan = plan_reclaim(&ledger, &BTreeSet::new());
        assert_eq!(plan.state_tags, vec!["hand_picked".to_string()]);
        assert!(plan.refused.is_empty());
        assert!(plan.refused.is_empty());
    }

    /// A ledger claiming nothing plans nothing — and still authorises removing
    /// the directory, which is what reclaims a SIGKILLed run that never got as
    /// far as creating a ring.
    #[test]
    fn an_empty_ledger_plans_nothing_but_still_clears_the_directory() {
        let plan = plan_reclaim(&RunLedger::default(), &BTreeSet::new());
        assert!(plan.unlink_names.is_empty());
        assert!(plan.state_tags.is_empty());
        assert!(plan.refused.is_empty());
        assert!(plan.refused.is_empty());
    }

    #[test]
    fn the_apparent_size_constants_match_the_shipped_ring_geometry() {
        // 40.06 MiB and 64.06 MiB — the figures the design's cost table quotes,
        // asserted so a ring-geometry change makes the warn's arithmetic wrong
        // LOUDLY rather than silently.
        assert_eq!(TRACE_RING_APPARENT_BYTES, 42_008_640);
        assert_eq!(STATE_RING_APPARENT_BYTES, 67_174_464);
    }

    /// THE SURVIVOR RULE: a name that could not be unlinked KEEPS its ledger,
    /// and the run is not counted reclaimed.
    ///
    /// `run.json` is the only thing that can attribute a hashed `/cer_rg_…` name
    /// to a run, so removing it while a name it declares is still standing
    /// STRANDS that name permanently — no later sweep can know whose it was and
    /// `/dev/shm` cannot say. The leak stops being bounded and stops being
    /// retryable in the same instant.
    ///
    /// Driven through a fault seam because a real EPERM needs an object this uid
    /// cannot remove, which a test cannot arrange without root.
    ///
    /// No `#[serial]`: the fault seam is THREAD-scoped and the fixture's temp
    /// root and PID-keyed SHM name are unique to this arm, so there is nothing
    /// process-global left to serialise against.
    #[test]
    fn a_name_that_could_not_be_unlinked_keeps_its_ledger_for_the_next_sweep() {
        let root = tempfile::tempdir().expect("root");
        let tag = format!("survivor_{}", std::process::id());
        let name = cerulion_core::shm_ring::ring_shm_name(&tag);
        // A REAL SHM object, so "it survived" is a fact about the filesystem
        // rather than about the fault seam's return value.
        let c = std::ffi::CString::new(name.clone()).expect("nul-free");
        // SAFETY: FFI create of a NUL-terminated name this test owns.
        let fd = unsafe {
            libc::shm_open(
                c.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600 as libc::c_uint,
            )
        };
        assert!(
            fd >= 0,
            "create the fixture: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: closing the descriptor this call just opened.
        unsafe { libc::close(fd) };

        let dir = root.path().join("survivor-0001");
        std::fs::create_dir_all(&dir).expect("dir");
        std::fs::write(
            dir.join(RUN_MANIFEST_FILE),
            serde_json::to_vec(&serde_json::json!({
                "run_id": "0x0000000000000000000000000000007b",
                "rings": [{ "tag": tag, "rank": 0 }],
            }))
            .expect("json"),
        )
        .expect("run.json");
        drop(
            crate::run_lock::RunLock::acquire(&dir.join(crate::run_lock::RUN_LOCK_FILE))
                .expect("free lock"),
        );

        {
            // Armed with the RESOLVED SHM name, not the tag: `ring_shm_name`
            // HASHES the tag, so the tag is not a substring of the name and a
            // tag-keyed arm would silently never fire (measured — the first
            // version of this test reported a clean reclaim).
            let _fault = fault_inject_unlink(&name);
            let report = sweep_stale_runs(root.path());
            assert_eq!(
                report.runs_reclaimed, 0,
                "a run whose memory is still resident was NOT reclaimed, and the \
                 report must not say it was"
            );
            assert_eq!(report.runs_partially_reclaimed, 1);
            assert_eq!(report.names_unlinked, 0);
            assert_eq!(report.bytes_reclaimed, 0);
            assert!(
                dir.exists(),
                "the ledger must survive — it is the only thing that can attribute \
                 this hashed name to a run"
            );
            assert!(
                dir.join(RUN_MANIFEST_FILE).exists(),
                "…and specifically the manifest, not merely the directory"
            );
            assert!(cerulion_core::shm_ring::shm_object_exists(&name));
        }

        // The next sweep RETRIES from that ledger and finishes the job — the
        // anti-tautology half, without which "keep the directory" is satisfied
        // by a sweeper that never reclaims anything.
        let second = sweep_stale_runs(root.path());
        assert_eq!(second.runs_reclaimed, 1);
        assert_eq!(second.runs_partially_reclaimed, 0);
        assert_eq!(second.names_unlinked, 1);
        assert!(!dir.exists());
        assert!(!cerulion_core::shm_ring::shm_object_exists(&name));
    }

    #[test]
    fn a_report_that_touched_nothing_is_silent() {
        assert!(!SweepReport::default().reclaimed_anything());
        assert!(SweepReport {
            runs_reclaimed: 1,
            ..SweepReport::default()
        }
        .reclaimed_anything());
        assert!(SweepReport {
            names_unlinked: 1,
            ..SweepReport::default()
        }
        .reclaimed_anything());
        // Skips alone are NOT a reclaim — a machine with one live run must not
        // warn on every subsequent `graph run`.
        assert!(!SweepReport {
            live_skipped: 3,
            unknown_skipped: 2,
            ..SweepReport::default()
        }
        .reclaimed_anything());
    }
}
