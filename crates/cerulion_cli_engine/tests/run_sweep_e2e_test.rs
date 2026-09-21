// SPDX-License-Identifier: AGPL-3.0-only
//! The run-directory sweeper against REAL POSIX SHM.
//!
//! The pure halves (`run_sweep::plan_reclaim`, `run_lock::probe_lock`) are
//! oracle-tested in their own modules. What they cannot see is the thing this
//! feature actually does: `shm_unlink` a name and `remove_dir_all` a directory,
//! decided by a `flock` a live process is holding. Every arm here creates REAL
//! SHM objects under the names a ledger declares and asks the ONE question that
//! matters afterwards — **does that name still exist?** — through
//! `cerulion_core::shm_ring::shm_object_exists`.
//!
//! # Why raw SHM objects rather than real rings
//!
//! The sweeper's entire job is to remove NAMES. A real `ShmRingOwner` would
//! reserve 40 MiB (a trace ring) or 64 MiB (a state ring) per object to prove
//! nothing extra: `shm_unlink` neither reads a header nor cares what the object
//! holds, and a real owner would additionally unlink on its OWN `Drop`, which is
//! precisely the mechanism a SIGKILL bypasses and this feature replaces. Raw
//! `shm_open(O_CREAT | O_EXCL)` objects are the FAITHFUL fixture: they model an
//! orphan whose owner is gone, which is the only shape a sweeper ever meets.
//!
//! # Parallel-safe
//!
//! Every arm mints its own temp root and its own PID+counter-unique tags, so no
//! two arms can name the same SHM object or the same directory. The ONE arm that
//! drives the production entry point (`start_run_descriptor`, which resolves its
//! root from `CERULION_HOME`) mutates process env and is `#[serial]`.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};

use cerulion_cli_engine::run_lock::{attach_lock_file, worker_lock_file, RunLock, RUN_LOCK_FILE};
use cerulion_cli_engine::run_sweep::{sweep_stale_runs, SweepReport};
use cerulion_core::shm_ring::{ring_shm_name, shm_object_exists};
use cerulion_core::state_arm::state_arm_shm_name;
use cerulion_core::state_ring::state_ring_shm_name;

/// One trace ring's apparent size, as the sweeper's warn accounts for it.
const TRACE_RING_BYTES: u64 = 65_600 + (1 << 20) * 40;
/// One state ring's apparent size.
const STATE_RING_BYTES: u64 = 65_600 + (1 << 17) * 512;
/// The arm word's page-class size.
const ARM_WORD_BYTES: u64 = 4096;

/// Per-test uniqueness: PID plus a process-local counter, so two arms running
/// concurrently in one binary can never collide on an SHM name.
fn unique(tag: &str) -> String {
    static N: AtomicU32 = AtomicU32::new(0);
    format!(
        "{tag}_{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

/// A raw SHM object that exists until this guard drops (or a sweep removes it).
///
/// `Drop` unlinks best-effort so a FAILING arm does not leak a name into
/// `/dev/shm` for the rest of the machine's uptime; a name a sweep already took
/// is an ordinary ENOENT here.
struct ShmName(String);

impl ShmName {
    fn create(name: &str) -> Self {
        let c = std::ffi::CString::new(name).expect("shm name is NUL-free");
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
            "could not create the SHM fixture `{name}`: {}",
            std::io::Error::last_os_error()
        );
        // SAFETY: sizing + closing the descriptor this call just opened.
        unsafe {
            libc::ftruncate(fd, 4096);
            libc::close(fd);
        }
        assert!(shm_object_exists(name), "`{name}` must exist once created");
        Self(name.to_string())
    }

    fn name(&self) -> &str {
        &self.0
    }
}

impl Drop for ShmName {
    fn drop(&mut self) {
        if let Ok(c) = std::ffi::CString::new(self.0.as_str()) {
            // SAFETY: best-effort FFI unlink of a valid NUL-terminated name.
            unsafe { libc::shm_unlink(c.as_ptr()) };
        }
    }
}

/// Build a run directory holding `run.json`.
fn make_run_dir(root: &Path, dir_name: &str, manifest: serde_json::Value) -> PathBuf {
    let dir = root.join(dir_name);
    std::fs::create_dir_all(&dir).expect("run dir");
    std::fs::write(
        dir.join("run.json"),
        serde_json::to_vec_pretty(&manifest).expect("json"),
    )
    .expect("run.json");
    dir
}

/// A ledger naming `rings` and a checkpoint ARM entry.
///
/// The `shm` array is the ONE ledger vocabulary (`ShmDecl`), which this
/// sweeper reads through `run_dir::run_manifest_shm` — the fixture writes the
/// shape the production writer writes, not a second spelling of it.
fn manifest(
    run_id: u128,
    ring_tags: &[(&str, u32)],
    arm_tag: &str,
    explicit: bool,
) -> serde_json::Value {
    let mut shm: Vec<serde_json::Value> = ring_tags
        .iter()
        .map(|(t, r)| {
            serde_json::json!({
                "tag": t,
                "class": if *r == u32::MAX { "departure" } else { "trace" },
                "rank": r,
                "source": "derived",
            })
        })
        .collect();
    shm.push(serde_json::json!({
        "tag": arm_tag,
        "class": "arm",
        "rank": serde_json::Value::Null,
        "source": if explicit { "explicit" } else { "derived" },
    }));
    serde_json::json!({
        "version": 1,
        "run_id": format!("0x{run_id:032x}"),
        "graph_name": "sweep_fixture",
        "rings": ring_tags
            .iter()
            .map(|(t, r)| serde_json::json!({ "tag": t, "rank": r }))
            .collect::<Vec<_>>(),
        "shm": shm,
    })
}

/// Create the lock file WITHOUT holding it — the shape a dead run leaves.
fn free_lock(dir: &Path, name: &str) {
    drop(RunLock::acquire(&dir.join(name)).expect("acquire then release"));
}

/// THE headline: a dead run's every declared name is reclaimed, its directory
/// goes, and the report accounts for the bytes.
///
/// The oracle is the SHM OBJECT, never the return value — a sweeper that
/// reported perfect numbers while unlinking nothing is exactly the failure a
/// report-only assertion cannot see.
#[test]
fn a_dead_runs_declared_shm_is_reclaimed_and_its_directory_removed() {
    let root = tempfile::tempdir().expect("root");
    let ring_a = unique("r0");
    let ring_dep = unique("dep");
    let arm = unique("arm");

    // The four families the ledger can name: two trace-class rings, TWO ranks of
    // state ring (the rank scan is what finds them), and the arm word.
    let shm_ring_a = ShmName::create(&ring_shm_name(&ring_a));
    let shm_ring_dep = ShmName::create(&ring_shm_name(&ring_dep));
    let shm_state_0 = ShmName::create(&state_ring_shm_name(&arm, 0).expect("rank 0 name"));
    let shm_state_1 = ShmName::create(&state_ring_shm_name(&arm, 1).expect("rank 1 name"));
    let shm_arm = ShmName::create(&state_arm_shm_name(&arm));

    let dir = make_run_dir(
        root.path(),
        "dead-0001",
        manifest(0x2a, &[(&ring_a, 0), (&ring_dep, u32::MAX)], &arm, false),
    );
    // A dead run: the file survives its holder, the LOCK does not.
    free_lock(&dir, RUN_LOCK_FILE);
    free_lock(&dir, &worker_lock_file(0));

    let report = sweep_stale_runs(root.path());

    for shm in [
        &shm_ring_a,
        &shm_ring_dep,
        &shm_state_0,
        &shm_state_1,
        &shm_arm,
    ] {
        assert!(
            !shm_object_exists(shm.name()),
            "`{}` must be unlinked by the sweep",
            shm.name()
        );
    }
    assert!(!dir.exists(), "the run directory must be removed");
    assert_eq!(
        report,
        SweepReport {
            runs_reclaimed: 1,
            // 2 trace-class rings + the arm word + 2 state-ring ranks.
            names_unlinked: 5,
            bytes_reclaimed: 2 * TRACE_RING_BYTES + ARM_WORD_BYTES + 2 * STATE_RING_BYTES,
            live_skipped: 0,
            unknown_skipped: 0,
            names_refused: 0,
            runs_partially_reclaimed: 0,
            candidates_reclassified: 0,
        },
        "the report must account for exactly what was removed"
    );
    assert!(report.reclaimed_anything());
}

/// THE safety property: a LIVE run's names survive another run's sweep.
///
/// "Live" is proved by a HELD `run.lock` and nothing else — no pid, no registry
/// record — which is the whole point of the flock oracle.
#[test]
fn a_live_runs_shm_survives_another_runs_sweep() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("live");
    let arm = unique("livearm");
    let shm_ring = ShmName::create(&ring_shm_name(&ring));
    let shm_arm = ShmName::create(&state_arm_shm_name(&arm));

    let dir = make_run_dir(
        root.path(),
        "live-0001",
        manifest(7, &[(&ring, 0)], &arm, false),
    );
    let _held = RunLock::acquire(&dir.join(RUN_LOCK_FILE)).expect("hold the run lock");

    let report = sweep_stale_runs(root.path());

    assert!(
        shm_object_exists(shm_ring.name()),
        "a live run's trace ring must survive"
    );
    assert!(
        shm_object_exists(shm_arm.name()),
        "a live run's arm word must survive"
    );
    assert!(dir.exists(), "a live run's directory must survive");
    assert_eq!(report.live_skipped, 1);
    assert_eq!(report.runs_reclaimed, 0);
    assert!(
        !report.reclaimed_anything(),
        "a sweep that touched nothing must not warn"
    );
}

/// A SIGKILLed supervisor does not necessarily take its workers with it, and a
/// rank that is still stepping is still writing its rings.
///
/// This is the arm `run.lock` alone cannot satisfy: the supervisor's lock is
/// FREE (it really did die) while rank 1's is HELD, and the run must be left
/// entirely alone.
#[test]
fn a_dead_supervisor_with_a_live_worker_is_not_swept() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("orphan");
    let shm_ring = ShmName::create(&ring_shm_name(&ring));

    let dir = make_run_dir(
        root.path(),
        "orphan-0001",
        manifest(9, &[(&ring, 1)], &unique("orphanarm"), false),
    );
    free_lock(&dir, RUN_LOCK_FILE);
    free_lock(&dir, &worker_lock_file(0));
    let _rank1 = RunLock::acquire(&dir.join(worker_lock_file(1))).expect("rank 1 still alive");

    let report = sweep_stale_runs(root.path());

    assert!(
        shm_object_exists(shm_ring.name()),
        "a ring a LIVE rank is still writing must not be unlinked because its \
         supervisor died"
    );
    assert!(dir.exists());
    assert_eq!(report.live_skipped, 1);
    assert_eq!(report.runs_reclaimed, 0);
}

/// An attacher OUTLIVES its run by design — it finalizes a bag after the run
/// ends — so a held `attach-<pid>.lock` on an otherwise-dead run keeps the whole
/// directory standing until that recorder is gone.
#[test]
fn an_attacher_arriving_between_the_two_passes_is_not_swept_out_from_under() {
    let root = tempfile::tempdir().expect("root");
    // TWO dead runs: one is genuinely reclaimable throughout, the other gains an
    // attacher AFTER pass 1 has already classified it dead. Without the second,
    // "nothing was reclaimed" would satisfy the survival assertion; without the
    // first, the arm could not tell a working re-probe from a sweep that had
    // stopped reclaiming anything at all.
    let arm_free = unique("tocfree");
    let arm_attached = unique("tocattached");
    let shm_free = ShmName::create(&state_arm_shm_name(&arm_free));
    let shm_attached = ShmName::create(&state_arm_shm_name(&arm_attached));

    let dir_free = make_run_dir(
        root.path(),
        "toctou-0001",
        manifest(71, &[], &arm_free, false),
    );
    let dir_attached = make_run_dir(
        root.path(),
        "toctou-0002",
        manifest(72, &[], &arm_attached, false),
    );
    free_lock(&dir_free, RUN_LOCK_FILE);
    free_lock(&dir_attached, RUN_LOCK_FILE);

    // THE WINDOW, driven rather than raced. `bag record --run` resolves a run
    // and flocks `attach-<pid>.lock`; landing that inside the sweep is what the
    // per-directory re-probe exists for, and the hook lands it at exactly the
    // instant pass 1's verdicts have been taken and none has been acted on.
    let mut attach = None;
    let report =
        cerulion_cli_engine::run_sweep::sweep_stale_runs_with_hook(root.path(), &mut || {
            attach = Some(
                RunLock::acquire(&dir_attached.join(attach_lock_file(std::process::id())))
                    .expect("the attacher takes its lock mid-sweep"),
            );
        });
    let attach = attach.expect("the hook must have run");

    assert!(
        !shm_object_exists(shm_free.name()),
        "the still-dead run must be reclaimed — otherwise this arm passes for a \
         sweep that reclaims nothing"
    );
    assert!(!dir_free.exists());
    assert!(
        shm_object_exists(shm_attached.name()),
        "a directory an attacher locked AFTER classification must not be \
         reclaimed — pass 1's verdict is a snapshot, and acting on it without \
         re-checking deletes a run out from under a live reader"
    );
    assert!(
        dir_attached.exists(),
        "…and its ledger must survive with it"
    );
    assert_eq!(report.runs_reclaimed, 1);
    assert_eq!(
        report.candidates_reclassified, 1,
        "the re-probe must REPORT that it changed its mind — the only evidence \
         it runs at all"
    );
    assert_eq!(report.live_skipped, 1);

    // Once the attacher is gone the next sweep takes it, so the skip is a
    // deferral rather than a permanent exemption.
    drop(attach);
    let second = sweep_stale_runs(root.path());
    assert!(!shm_object_exists(shm_attached.name()));
    assert!(!dir_attached.exists());
    assert_eq!(second.runs_reclaimed, 1);
    assert_eq!(second.candidates_reclassified, 0);
}

#[test]
fn a_held_attach_lock_keeps_a_dead_runs_directory_and_names() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("attached");
    let shm_ring = ShmName::create(&ring_shm_name(&ring));

    let dir = make_run_dir(
        root.path(),
        "attached-0001",
        manifest(11, &[(&ring, 0)], &unique("attacharm"), false),
    );
    free_lock(&dir, RUN_LOCK_FILE);
    let attach = RunLock::acquire(&dir.join(attach_lock_file(std::process::id())))
        .expect("hold the attach lock");

    let report = sweep_stale_runs(root.path());
    assert!(
        shm_object_exists(shm_ring.name()),
        "a ring a live attacher may still be reading must survive"
    );
    assert!(
        dir.exists(),
        "the directory must survive while a recorder is still reading it"
    );
    assert_eq!(report.live_skipped, 1);

    // …and once the attacher is gone, a later run reclaims it. Without this the
    // arm above would also pass a sweeper that never reclaims anything.
    drop(attach);
    let second = sweep_stale_runs(root.path());
    assert!(!shm_object_exists(shm_ring.name()));
    assert!(!dir.exists());
    assert_eq!(second.runs_reclaimed, 1);
}

/// The START RACE, in the shape the ordering rule makes reachable: a directory
/// that exists with NO ledger yet.
///
/// `run.lock` is flocked BEFORE `run.json` is rendered, so this window carries
/// nothing a sweeper could attribute — and it is skipped as UNKNOWN rather than
/// reclaimed, which is what keeps a starting run's names safe.
#[test]
fn a_directory_with_no_ledger_is_left_alone() {
    let root = tempfile::tempdir().expect("root");
    let starting = root.path().join("starting-0001");
    std::fs::create_dir_all(&starting).expect("mkdir");

    let report = sweep_stale_runs(root.path());

    assert!(
        starting.exists(),
        "a directory in the mkdir->ledger window must not be removed"
    );
    assert_eq!(report.runs_reclaimed, 0);
    assert_eq!(report.unknown_skipped, 1);
}

/// The OTHER half of the start race, and the one that would matter if the
/// ordering rule were ever reversed: a ledger with NO lock beside it.
///
/// An absent lock is UNKNOWN, never dead. If production rendered `run.json`
/// before taking its lock, a live run would pass through exactly this shape —
/// so this rule is what makes the reversal a leak rather than a catastrophe.
/// (The ordering itself is pinned structurally by
/// `the_run_lock_is_acquired_before_the_manifest_is_written`.)
#[test]
fn a_ledger_with_no_lock_file_is_unknown_never_dead() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("nolock");
    let shm_ring = ShmName::create(&ring_shm_name(&ring));

    let dir = make_run_dir(
        root.path(),
        "nolock-0001",
        manifest(13, &[(&ring, 0)], &unique("nolockarm"), false),
    );
    // Deliberately NO lock file of any kind.

    let report = sweep_stale_runs(root.path());

    assert!(
        shm_object_exists(shm_ring.name()),
        "a ledger with no lock beside it proves nothing about its run"
    );
    assert!(dir.exists());
    assert_eq!(report.runs_reclaimed, 0);
    assert_eq!(report.unknown_skipped, 1);
}

/// An unparseable ledger is fail-CLOSED: its names cannot be attributed, so
/// nothing is unlinked and the directory stands.
#[test]
fn an_unparseable_ledger_is_left_alone() {
    let root = tempfile::tempdir().expect("root");
    let dir = root.path().join("corrupt-0001");
    std::fs::create_dir_all(&dir).expect("mkdir");
    std::fs::write(dir.join("run.json"), b"{ this is not json").expect("write");
    free_lock(&dir, RUN_LOCK_FILE);

    let report = sweep_stale_runs(root.path());

    assert!(dir.exists(), "an unreadable ledger must not be reclaimed");
    assert_eq!(report.runs_reclaimed, 0);
    assert_eq!(report.unknown_skipped, 1);
}

/// The EXPLICIT-TAG hazard: two runs started with one hand-picked
/// `CERULION_STATE_ARM_TAG`, one of them since killed.
///
/// Sweeping the dead one's tag would unlink the LIVE one's arm word and state
/// rings — they are literally the same names. The refusal is SCOPED: the dead
/// run's own trace rings are still reclaimed (their tags carry a supervisor pid
/// and cannot be shared), and its directory SURVIVES so a later run can retry.
#[test]
fn an_explicit_tag_a_live_run_also_claims_is_refused_but_its_rings_are_not() {
    let root = tempfile::tempdir().expect("root");
    let shared_arm = unique("shared");
    let dead_ring = unique("deadring");

    let shm_arm = ShmName::create(&state_arm_shm_name(&shared_arm));
    let shm_state = ShmName::create(&state_ring_shm_name(&shared_arm, 0).expect("rank 0"));
    let shm_dead_ring = ShmName::create(&ring_shm_name(&dead_ring));

    let dead = make_run_dir(
        root.path(),
        "sharedead-0001",
        manifest(21, &[(&dead_ring, 0)], &shared_arm, true),
    );
    free_lock(&dead, RUN_LOCK_FILE);

    let live = make_run_dir(
        root.path(),
        "sharelive-0002",
        manifest(22, &[], &shared_arm, true),
    );
    let _held = RunLock::acquire(&live.join(RUN_LOCK_FILE)).expect("hold");

    let report = sweep_stale_runs(root.path());

    assert!(
        shm_object_exists(shm_arm.name()),
        "the LIVE run's arm word must survive its dead twin's sweep"
    );
    assert!(
        shm_object_exists(shm_state.name()),
        "the LIVE run's state ring must survive"
    );
    assert!(
        !shm_object_exists(shm_dead_ring.name()),
        "the dead run's OWN trace ring is not shareable and must still be reclaimed"
    );
    assert!(
        dead.exists(),
        "a refusal must keep the ledger, or the refused name becomes unattributable"
    );
    assert!(live.exists());
    assert_eq!(report.names_refused, 1);
    assert_eq!(report.names_unlinked, 1);
    assert_eq!(report.live_skipped, 1);
    // A REFUSAL is a survivor too, so this run is PARTIALLY reclaimed and must
    // not be counted as a clean one — its checkpoint plane is still resident.
    assert_eq!(report.runs_partially_reclaimed, 1);
    assert_eq!(report.runs_reclaimed, 0);
}

/// The same explicit tag with NO live claimant IS swept — the anti-tautology
/// half of the refusal arm, without which "refuse every explicit tag" passes.
#[test]
fn an_explicit_tag_no_live_run_claims_is_swept() {
    let root = tempfile::tempdir().expect("root");
    let arm = unique("lonelyexplicit");
    let shm_arm = ShmName::create(&state_arm_shm_name(&arm));
    let shm_state = ShmName::create(&state_ring_shm_name(&arm, 0).expect("rank 0"));

    let dir = make_run_dir(root.path(), "lonely-0001", manifest(31, &[], &arm, true));
    free_lock(&dir, RUN_LOCK_FILE);

    let report = sweep_stale_runs(root.path());

    assert!(!shm_object_exists(shm_arm.name()));
    assert!(!shm_object_exists(shm_state.name()));
    assert!(!dir.exists());
    assert_eq!(report.names_refused, 0);
    assert_eq!(report.names_unlinked, 2);
}

/// Idempotency: a second sweep over an already-reclaimed root is a silent no-op.
///
/// A crash mid-sweep leaves a partial reclaim, and the next run retries — so
/// re-running must be free and must not warn about work it did not do.
#[test]
fn a_second_sweep_is_a_silent_no_op() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("idem");
    let _shm = ShmName::create(&ring_shm_name(&ring));
    let dir = make_run_dir(
        root.path(),
        "idem-0001",
        manifest(41, &[(&ring, 0)], &unique("idemarm"), false),
    );
    free_lock(&dir, RUN_LOCK_FILE);

    let first = sweep_stale_runs(root.path());
    assert!(first.reclaimed_anything());

    let second = sweep_stale_runs(root.path());
    assert_eq!(second, SweepReport::default());
    assert!(
        !second.reclaimed_anything(),
        "an already-clean root must produce no warn"
    );
}

/// A root that does not exist yet — the ordinary case on a fresh machine — is
/// answered silently rather than by a failure.
#[test]
fn a_missing_root_is_a_silent_no_op() {
    let root = tempfile::tempdir().expect("root");
    let report = sweep_stale_runs(&root.path().join("never-created"));
    assert_eq!(report, SweepReport::default());
}

/// A dead run's declared name that is ALREADY GONE (its owner's `Drop` ran, or a
/// previous sweep got there) is not counted as reclaimed — otherwise every clean
/// exit's leftover directory would warn about megabytes it did not recover.
#[test]
fn names_that_were_already_gone_are_not_counted_as_reclaimed() {
    let root = tempfile::tempdir().expect("root");
    let ring = unique("gone");
    // NOTE: no `ShmName::create` — the ledger names an object that never existed.
    let dir = make_run_dir(
        root.path(),
        "gone-0001",
        manifest(51, &[(&ring, 0)], &unique("gonearm"), false),
    );
    free_lock(&dir, RUN_LOCK_FILE);

    let report = sweep_stale_runs(root.path());

    assert_eq!(report.names_unlinked, 0, "nothing was there to unlink");
    assert_eq!(
        report.bytes_reclaimed, 0,
        "and therefore no bytes recovered"
    );
    assert_eq!(
        report.runs_reclaimed, 1,
        "the stale DIRECTORY is still reclaimed — that is a run-directory leak too"
    );
    assert!(!dir.exists());
}

/// STRUCTURAL: the Unix-only sweeper is never named from an ungated place.
///
/// `run_sweep` resolves POSIX SHM names, so it depends on
/// `cerulion_core::{shm_ring, state_arm, state_ring}` — each of which is itself
/// `#[cfg(unix)]` because POSIX shared memory is what they are. Declaring the
/// module unconditionally, or calling it from an ungated block, is a non-Unix
/// BUILD FAILURE rather than a feature that degrades.
///
/// This box builds only the Unix arm and the iceoryx2 tree does not cross-build
/// to a non-Unix target, so the claim is pinned by SOURCE SYMMETRY rather than
/// by a second compile — the `cerulion_cli/src/cfg_symmetry_tests.rs` precedent
/// for exactly this situation. What it can prove is what actually broke: the
/// declaration carries the gate, and every reference sits under one.
#[test]
fn the_unix_only_sweeper_is_declared_and_called_under_a_cfg_gate() {
    let lib = std::fs::read_to_string("src/lib.rs").expect("lib.rs");
    let decl = lib
        .find("pub mod run_sweep;")
        .expect("the module must still be declared");
    // The gate must be the attribute IMMEDIATELY above the declaration, not
    // merely present somewhere in a 200-line file.
    let preceding = lib[..decl].trim_end();
    assert!(
        preceding.ends_with("#[cfg(unix)]"),
        "`pub mod run_sweep;` must be gated `#[cfg(unix)]` — it names \
         cerulion_core's POSIX-SHM modules, which are themselves Unix-only, so \
         an ungated declaration does not compile off Unix. Found instead: {:?}",
        &preceding[preceding.len().saturating_sub(60)..]
    );
    // Anti-tautology: its SIBLING is deliberately NOT gated (it compiles
    // everywhere, reporting UNKNOWN off Unix), so this assertion is about
    // run_sweep specifically rather than about the file having a `cfg` in it.
    let sibling = lib
        .find("pub mod run_lock;")
        .expect("the sibling must still be declared");
    assert!(
        !lib[..sibling].trim_end().ends_with("#[cfg(unix)]"),
        "`run_lock` must stay UNGATED — gating it would make this test pass for \
         the wrong reason and would hide a real portability question"
    );

    // Every reference outside the module itself must sit under a gate.
    let run_dir = std::fs::read_to_string("src/run_dir.rs").expect("run_dir.rs");
    for (i, _) in run_dir.match_indices("crate::run_sweep::") {
        let before = &run_dir[..i];
        assert!(
            before.contains("#[cfg(unix)]"),
            "a `crate::run_sweep::` reference in run_dir.rs must sit under a \
             `#[cfg(unix)]` block"
        );
        // …and specifically under the NEAREST enclosing gate, not one that
        // happens to appear earlier in the file: the last `#[cfg(...)]` before
        // this reference must be the Unix one.
        let last_cfg = before.rfind("#[cfg(").expect("checked above");
        let attr_end = before[last_cfg..]
            .find(']')
            .map(|e| last_cfg + e + 1)
            .expect("a cfg attribute is bracketed");
        assert_eq!(
            &before[last_cfg..attr_end],
            "#[cfg(unix)]",
            "the gate nearest a `crate::run_sweep::` reference must be \
             `#[cfg(unix)]`, or the reference is compiled on a target where the \
             module does not exist"
        );
    }
}

/// STRUCTURAL: `run.lock` is acquired BEFORE the manifest is written.
///
/// Behaviourally invisible — a sweeper racing that exact window is a
/// microsecond-scale interleaving no test can schedule — while being the whole
/// safety argument: at the instant a ledger becomes visible its lock is already
/// held, so a sweeper either sees no ledger (and skips) or sees a held lock (and
/// skips). Reversing the two lines re-opens a window in which a LIVE run is
/// indistinguishable from a dead one.
#[test]
fn the_run_lock_is_acquired_before_the_manifest_is_written() {
    let src = std::fs::read_to_string("src/run_dir.rs").expect("run_dir.rs");
    let body = fn_body(&src, "pub fn start_run_descriptor(")
        .expect("start_run_descriptor must still exist");
    let acquire = body
        .find("RunLock::acquire(")
        .expect("start_run_descriptor must take the run lock");
    let write = body
        .find("write_run_artifacts(")
        .expect("start_run_descriptor must write its artifacts");
    assert!(
        acquire < write,
        "the run lock must be acquired BEFORE `run.json` is rendered — reversing \
         them lets a sweeper observe a ledger whose lock is not yet held, i.e. a \
         live run that looks exactly like a dead one"
    );
    // The sweep itself must run AFTER the lock, or the starting run could reclaim
    // its own directory (self-exclusion is a kernel property of the held lock,
    // not a path comparison).
    let sweep = body
        .find("sweep_stale_runs(")
        .expect("start_run_descriptor must sweep");
    assert!(
        acquire < sweep,
        "the sweep must run with this run's own lock already held"
    );
}

/// The WORKER's lock is taken before its READY sentinel, for the same reason.
#[test]
fn a_worker_locks_before_it_signals_ready() {
    let src = std::fs::read_to_string("src/graph_cmd.rs").expect("graph_cmd.rs");
    let body = fn_body(&src, "fn graph_run_worker(").expect("graph_run_worker must still exist");
    let acquire = body
        .find("take_worker_lock(")
        .expect("the worker must take its own lock");
    // The sentinel is published ATOMICALLY (tmp sibling + rename — the
    // torn-READY fix), so the point at which it becomes VISIBLE to the
    // supervisor is the rename into `plan.ready_path`; the ordering pin anchors
    // there rather than on the tmp write.
    let ready = body
        .find("std::fs::rename(&ready_tmp, &plan.ready_path)")
        .expect("the worker must publish its READY sentinel (atomic tmp write + rename)");
    assert!(
        acquire < ready,
        "a worker must hold its own lock BEFORE READY — after READY its rings are \
         being written, and a sweeper that could see a READY worker without its \
         lock is free to unlink a ring under a live writer"
    );
    // The outcome must be BOUND, not discarded: `take_worker_lock` returns the
    // held lock, and a call whose result is dropped releases it immediately —
    // which reads exactly like holding it while protecting nothing.
    let bound = body[..acquire].trim_end();
    assert!(
        bound.ends_with('='),
        "the worker lock must be BOUND for the worker's lifetime — a discarded \
         result drops the lock at the end of the statement. Found: {:?}",
        &bound[bound.len().saturating_sub(60)..]
    );
    // …and the refusal must PROPAGATE. `take_worker_lock` returns a `Result`
    // whose Err arm is the whole fix; a `.ok()` or an `unwrap_or`
    // there would restore run-on-unprotected while keeping every ordering
    // assertion above green.
    let stmt_end = body[acquire..]
        .find(";}")
        .or_else(|| body[acquire..].find(";\n"))
        .map(|e| acquire + e)
        .expect("the call is a statement");
    assert!(
        body[acquire..stmt_end].contains(")?"),
        "a failed worker lock must PROPAGATE (`?`) — swallowing it runs the rank \
         unprotected, which is the one state the sweeper's liveness model cannot \
         see. Found: {:?}",
        &body[acquire..stmt_end]
    );
}

/// Brace-matched body of the function whose signature starts at `needle`.
///
/// Not a `contains` over the whole file: both ordering claims above are about
/// ONE function, and a file-wide search would be satisfied by the two tokens
/// appearing anywhere in 20k lines.
fn fn_body(src: &str, needle: &str) -> Option<String> {
    let start = src.find(needle)?;
    let open = src[start..].find('{')? + start;
    let mut depth = 0usize;
    for (i, c) in src[open..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(src[open..open + i].to_string());
                }
            }
            _ => {}
        }
    }
    None
}

/// The PRODUCTION entry point cannot sweep the run that is starting.
///
/// Self-exclusion is a KERNEL property — the starting run holds its own
/// `run.lock` before it sweeps, and `flock` contends across independent open
/// file descriptions even within one process — rather than a path comparison
/// somebody has to remember to write. Driven through `start_run_descriptor`
/// twice so it is the real seam, not a re-implementation of it.
///
/// `#[serial]`: `CERULION_HOME` is process-global.
#[test]
#[serial_test::serial]
fn a_starting_run_sweeps_a_dead_one_and_never_itself() {
    let home = tempfile::tempdir().expect("home");
    let _guard = HomeGuard::set(home.path());

    let root = home.path().join("runs");
    std::fs::create_dir_all(&root).expect("runs root");

    // A DEAD run's leftovers, with a real SHM name to reclaim.
    let dead_ring = unique("prod_dead");
    let shm_dead = ShmName::create(&ring_shm_name(&dead_ring));
    let dead = make_run_dir(
        &root,
        "dead-run-0001",
        manifest(61, &[(&dead_ring, 0)], &unique("prod_deadarm"), false),
    );
    free_lock(&dead, RUN_LOCK_FILE);

    // Descriptor A: a LIVE run, still holding its lock.
    let a = start_descriptor("alpha");
    assert!(a.path().exists(), "run A's directory must exist");

    // Descriptor B: a second run starting. Its `start_run_descriptor` sweeps.
    let b = start_descriptor("beta");

    assert!(
        !shm_dead.name().is_empty() && !shm_object_exists(shm_dead.name()),
        "the DEAD run's shared memory must be reclaimed by the starting run"
    );
    assert!(!dead.exists(), "the dead run's directory must be removed");
    assert!(
        a.path().exists(),
        "a LIVE run's directory must survive another run's sweep"
    );
    assert!(
        b.path().exists(),
        "the sweeping run must not remove its OWN directory"
    );
    drop(a);
    drop(b);
}

fn start_descriptor(name: &str) -> cerulion_cli_engine::run_dir::RunDescriptor {
    cerulion_cli_engine::run_dir::start_run_descriptor(
        cerulion_cli_engine::run_dir::RunDescriptorSpec {
            run_id: cerulion_cli_engine::run_dir::mint_run_id(),
            graph_name: name,
            run_started_at_ns: 1_753_000_000_000_000_000,
            network: cerulion_cli_engine::run_dir::NetworkPostureLabel::Off,
            partition: cerulion_cli_engine::run_dir::PartitionProvenance::Declared,
            process_groups: false,
            shm: Vec::new(),
            gating: cerulion_cli_engine::run_dir::GatingClock::Wall,
            graph_yaml: "nodes: {}\n".to_string(),
            env_json: b"{}".to_vec(),
            recorder_json: b"{}".to_vec(),
        },
    )
    .expect("descriptor")
}

/// Restores `CERULION_HOME` on drop, so a panicking arm cannot leak it into the
/// rest of the binary.
struct HomeGuard(Option<String>);

impl HomeGuard {
    fn set(dir: &Path) -> Self {
        let prev = std::env::var("CERULION_HOME").ok();
        std::env::set_var("CERULION_HOME", dir);
        HomeGuard(prev)
    }
}

impl Drop for HomeGuard {
    fn drop(&mut self) {
        match self.0.take() {
            Some(v) => std::env::set_var("CERULION_HOME", v),
            None => std::env::remove_var("CERULION_HOME"),
        }
    }
}
