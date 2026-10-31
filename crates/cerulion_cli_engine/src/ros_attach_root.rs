// SPDX-License-Identifier: AGPL-3.0-only
//! The workspace root `cerulion ros2 attach` runs against, resolved and owned
//! in one place.
//!
//! The attach root's logic lives here (the engine crate owns command logic;
//! `cerulion_cli` is a thin clap binary) so the no-workspace case is
//! unit-testable on the engine's hermetic seams: workspace discovery, the
//! `--dry-run` gate that tolerates a missing workspace, the exclusive temp
//! root and its cleanup guard. The binary calls [`AttachRoot::resolve`] with
//! its cwd and the `--dry-run` flag, holds the guard for the rest of the verb,
//! and calls the engine with the resolved root.
//!
//! Two shapes exist:
//!
//! * [`AttachRoot::Workspace`] - a discovered workspace; nothing is created,
//!   nothing is cleaned up (the workspace's own files must never be touched).
//! * [`AttachRoot::ExclusiveTemp`] - a `--dry-run` with no workspace:
//!   an EMPTY, EXCLUSIVELY created temp dir, removed again when the guard
//!   drops. See [`AttachRoot::resolve`].

use std::path::{Path, PathBuf};

use crate::error::{CliError, CliResult};
use crate::workspace::CerulionWorkspace;

/// How many `AlreadyExists` collisions the exclusive-root probe tolerates
/// before refusing. A collision means a stale dir from a killed earlier run
/// (or one pre-created by hand) sits at a PID-derived name; the probe retries
/// with `-2`, `-3`, ... so a hostile or busy temp dir cannot make the verb
/// fail, but the sequence is BOUNDED: an unbounded loop on a temp dir that
/// always answers `AlreadyExists` would hang the verb instead of refusing it.
const MAX_ROOT_COLLISIONS: u32 = 64;

/// How many candidate names the exclusive-root probe reaches: the plain
/// PID-derived name plus one retry per tolerated collision. DERIVED from
/// [`MAX_ROOT_COLLISIONS`] rather than written out, because the refusal text
/// quotes this count and a hand-written one disagrees with the loop the first
/// time the bound moves.
const ROOT_CANDIDATES_PROBED: u32 = MAX_ROOT_COLLISIONS + 1;

/// The two remedies every workspace-less dry-run refusal names. A refusal that
/// states only that the root could not be created leaves the operator with no
/// next step, so both refusal arms interpolate this sentence and the `ros2
/// attach` row in `docs/user-api.md` quotes it (a doc test reads it from here).
pub const DRY_RUN_ROOT_REMEDIES: &str =
    "Point TMPDIR at a writable location, or run the dry-run from inside a workspace.";

/// The root `cerulion ros2 attach` will run against.
///
/// Construct via [`AttachRoot::resolve`] and hold the guard it returns for the
/// rest of the verb, so an [`AttachRoot::ExclusiveTemp`] root never outlives
/// the run that created it.
#[derive(Debug)]
pub enum AttachRoot {
    /// A discovered workspace. Nothing created, nothing removed.
    Workspace(PathBuf),
    /// The workspace-less `--dry-run` root: exclusively created, empty,
    /// removed on drop-guard exit.
    ExclusiveTemp(PathBuf),
}

impl AttachRoot {
    /// The root path handed to the engine's attach flow.
    pub fn path(&self) -> &Path {
        match self {
            AttachRoot::Workspace(root) | AttachRoot::ExclusiveTemp(root) => root,
        }
    }

    /// Resolve the attach root by discovering the workspace from `start_dir`,
    /// WITH its cleanup guard: `Some` for [`AttachRoot::ExclusiveTemp`] only
    /// (a workspace root is never cleaned up). The guard is handed out here and
    /// nowhere else, and [`ExclusiveRootGuard`] is constructible only inside
    /// this module, so one temporary root can never have two guards; a second
    /// guard's drop would warn about a root the first already removed.
    ///
    /// Discovery from `start_dir`:
    /// the discovered workspace when one exists (a `--dry-run` inside one
    /// still reads its `.msg` store); otherwise, for a `--dry-run` ONLY, an
    /// exclusively created empty temp dir under [`std::env::temp_dir`].
    /// Without `--dry-run`, a missing workspace is the discovery's
    /// `WorkspaceNotFound` error unchanged (write and run require a real
    /// workspace). Any other discovery error (a `Cargo.toml` that could not be
    /// read while walking) is returned whatever the flag.
    ///
    /// Created EMPTY and EXCLUSIVELY (`create_dir`, which fails on
    /// `AlreadyExists`): a merely predictable PID-named path is only probably
    /// absent, and a workspace-less dry-run that read a pre-populated
    /// `schemas/` store left at that path would classify discovered types
    /// against schemas the user never supplied: a report integrity gap, not a
    /// crash. The exclusively created root carries no `schemas/` and no
    /// `nodes/dds_bridge`, so type resolution stays on the built-in corpus and
    /// the vendored-bridge probe fails open.
    pub fn resolve(
        start_dir: &Path,
        dry_run: bool,
    ) -> CliResult<(Self, Option<ExclusiveRootGuard>)> {
        resolve_in(start_dir, dry_run, &std::env::temp_dir())
    }
}

/// [`AttachRoot::resolve`] over an INJECTED temp base: the production form
/// reads `std::env::temp_dir()`, the tests pin their own tempdir.
fn resolve_in(
    start_dir: &Path,
    dry_run: bool,
    temp_base: &Path,
) -> CliResult<(AttachRoot, Option<ExclusiveRootGuard>)> {
    match CerulionWorkspace::discover(start_dir) {
        Ok(ws) => Ok((AttachRoot::Workspace(ws.root), None)),
        Err(CliError::WorkspaceNotFound { .. }) if dry_run => {
            let root = exclusive_dry_run_root_in(temp_base)?;
            Ok((
                AttachRoot::ExclusiveTemp(root.clone()),
                Some(ExclusiveRootGuard(Some(root))),
            ))
        }
        Err(err) => Err(err),
    }
}

/// Root handed to the engine when `--dry-run` has no workspace.
///
/// A name collision (a stale dir from a killed earlier run, or one
/// pre-created by hand) retries with `-2`, `-3`, ... so the probe reaches
/// [`ROOT_CANDIDATES_PROBED`] names in all (the plain name plus
/// [`MAX_ROOT_COLLISIONS`] retries), and the refusal states that count; the
/// loop is BOUNDED so a temp dir that answers `AlreadyExists` to every
/// candidate refuses loudly instead of hanging the verb. Any OTHER failure to
/// create the root is an error (never a silent builtins-only degradation,
/// which would make the report untrustworthy without telling the operator
/// why). The refusal names the two remedies that
/// keep the dry-run usable on a host with an unwritable temp dir.
///
/// The base directory is INJECTED: the production form passes
/// `std::env::temp_dir()`, the tests pin their own tempdir so parallel test
/// runs (and their PID-derived candidate names) never share a directory.
fn exclusive_dry_run_root_in(base: &Path) -> CliResult<PathBuf> {
    let mut candidate = base.join(format!("cerulion-attach-dry-run-{}", std::process::id()));
    for attempt in 0..=MAX_ROOT_COLLISIONS {
        match std::fs::create_dir(&candidate) {
            Ok(()) => return Ok(candidate),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
                if attempt == MAX_ROOT_COLLISIONS {
                    return Err(CliError::Validation(format!(
                        "ros2 attach: every candidate temporary root under {} was already \
                         taken ({ROOT_CANDIDATES_PROBED} names probed, all derived from \
                         pid {}); the dry-run needs one writable directory outside a \
                         workspace \
                         (created empty, removed again at exit). {DRY_RUN_ROOT_REMEDIES}",
                        base.display(),
                        std::process::id()
                    )));
                }
                candidate = base.join(format!(
                    "cerulion-attach-dry-run-{}-{}",
                    std::process::id(),
                    attempt + 2
                ));
            }
            // Any other failure is loud, never a silent builtins-only
            // degradation: an operator who cannot see why the report shows
            // no workspace schemas cannot trust its resolvability verdicts.
            // The refusal names the two remedies that keep the dry-run
            // usable on a host with an unwritable temp dir. ASCII
            // punctuation only: this string is shipped text, and the
            // public-surface dash gate scans string literals.
            Err(e) => {
                return Err(CliError::Validation(format!(
                    "ros2 attach: could not create a temporary root for the workspace-less \
                     dry-run at {}: {e}; the dry-run needs one writable directory outside a \
                     workspace (created empty, removed again at exit). \
                     {DRY_RUN_ROOT_REMEDIES}",
                    candidate.display()
                )));
            }
        }
    }
    unreachable!("the loop returns or errors on every arm")
}

/// RAII guard for the workspace-less dry-run's temporary root: the guard's
/// `Drop` runs on EVERY exit path out of the `ros2 attach` dispatch - the Ok
/// returns, the `?` error returns (a failed DDS discovery leaves through
/// exactly such a `?`), and a panic unwind - so the exclusively created root
/// never outlives the verb that created it. Handed out by
/// [`AttachRoot::resolve`] and constructible nowhere else, so one root has one
/// guard; `None` on every arm that had a workspace (there is nothing to clean
/// up, and the workspace's own files must not be touched).
///
/// `Debug` is derived because `resolve` returns the guard beside the root, so
/// the pair lands in the `expect`/`expect_err` message of every arm that
/// resolves one.
#[derive(Debug)]
pub struct ExclusiveRootGuard(Option<PathBuf>);

impl Drop for ExclusiveRootGuard {
    fn drop(&mut self) {
        if let Some(root) = self.0.take() {
            exclusive_root_cleanup(&root);
        }
    }
}

/// Remove the workspace-less dry-run's temporary root. A leftover dir is the
/// predictable-path bug reborn: the next process could draw the same PID and
/// read a directory that is not provably empty. Best effort, loud on
/// failure, never fails the run (the report has already been printed).
/// Deliberately `remove_dir`, NOT `remove_dir_all`: the dir is provably
/// empty (a dry-run writes nothing), so a non-empty failure surfaces loudly
/// instead of recursively deleting content this process did not create.
fn exclusive_root_cleanup(root: &Path) {
    if let Err(e) = std::fs::remove_dir(root) {
        // Recoverable, so warn (never fail the run: the report has already
        // been printed). Library code never prints - this is the tracing
        // shape of the binary's `note:` advisory.
        tracing::warn!(
            path = %root.display(),
            error = %e,
            "could not remove the temporary dry-run root"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Cold-path allocations only: root resolution is verb setup, never the
    // publish/receive hot path.

    /// A minimal on-disk workspace: the two markers `discover` requires.
    fn workspace_at(root: &Path) {
        std::fs::create_dir_all(root.join("graphs")).unwrap();
        std::fs::write(root.join("Cargo.toml"), "[workspace]\nmembers = []\n").unwrap();
    }

    #[test]
    fn resolve_uses_the_workspace_when_found() {
        let ws = tempfile::tempdir().unwrap();
        workspace_at(ws.path());
        let start = ws.path().join("graphs");
        let temp_base = tempfile::tempdir().unwrap();
        for dry_run in [false, true] {
            let (resolved, guard) =
                resolve_in(&start, dry_run, temp_base.path()).expect("workspace arm");
            assert!(
                matches!(resolved, AttachRoot::Workspace(ref p) if p == ws.path()),
                "dry_run={dry_run}: a found workspace is the root; got {resolved:?}"
            );
            // A workspace root is NEVER cleaned up: `resolve` hands out no
            // guard, so nothing can remove the workspace's own files.
            assert!(guard.is_none());
        }
        // Nothing was created under the temp base on the workspace arm.
        assert_eq!(std::fs::read_dir(temp_base.path()).unwrap().count(), 0);
    }

    #[test]
    fn resolve_without_a_workspace_refuses_unless_dry_run() {
        let outside = tempfile::tempdir().unwrap();
        let temp_base = tempfile::tempdir().unwrap();
        let err = resolve_in(outside.path(), false, temp_base.path())
            .expect_err("write and run require a workspace");
        assert!(
            matches!(err, CliError::WorkspaceNotFound { .. }),
            "the discovery error passes through unchanged; got {err:?}"
        );
        assert_eq!(std::fs::read_dir(temp_base.path()).unwrap().count(), 0);
    }

    #[test]
    fn resolve_without_a_workspace_on_dry_run_creates_an_exclusive_root() {
        let outside = tempfile::tempdir().unwrap();
        let temp_base = tempfile::tempdir().unwrap();
        let (resolved, guard) =
            resolve_in(outside.path(), true, temp_base.path()).expect("dry-run arm");
        let root = match &resolved {
            AttachRoot::ExclusiveTemp(root) => root.clone(),
            other => panic!("expected the exclusive temp root; got {other:?}"),
        };
        assert_eq!(root.parent(), Some(temp_base.path()));
        assert!(std::fs::read_dir(&root).unwrap().next().is_none());
        // The guard comes from `resolve` and from nowhere else, so this root
        // has exactly one owner: it stands while that guard lives and is gone
        // once it drops.
        let guard = guard.expect("the exclusive-temp arm hands out its guard");
        assert!(root.is_dir(), "the root stands while its guard lives");
        drop(guard);
        assert!(!root.exists(), "the guard removes the root it was handed");
    }

    #[test]
    fn exclusive_root_is_created_empty_and_exclusively() {
        let base = tempfile::tempdir().unwrap();
        let root = exclusive_dry_run_root_in(base.path()).expect("temp root created");
        // Exclusive: re-creating at the same path must fail with AlreadyExists,
        // proving the dir exists AND nothing else could have pre-created it.
        let again = std::fs::create_dir(&root);
        assert!(again.is_err(), "the root must exist exclusively");
        // Empty: no schemas/, no graphs/, nothing.
        assert!(
            std::fs::read_dir(&root).expect("listable").next().is_none(),
            "the exclusive root is created empty"
        );
        std::fs::remove_dir(&root).expect("cleanup in the passing arm");
    }

    #[test]
    fn exclusive_root_guard_removes_the_dir_on_drop() {
        let base = tempfile::tempdir().unwrap();
        let root = exclusive_dry_run_root_in(base.path()).expect("temp root created");
        {
            let _guard = ExclusiveRootGuard(Some(root.clone()));
            assert!(root.is_dir(), "the root survives while the guard lives");
        }
        assert!(!root.exists(), "the guard's drop removes the root");
    }

    #[test]
    fn a_hostile_collision_run_refuses_after_the_bound() {
        // Occupy every candidate name the probe can reach from THIS pid: the
        // plain name and the -2..-(MAX+1) suffixed names. The probe must
        // refuse after MAX_ROOT_COLLISIONS attempts, never hang. The base is
        // this test's own tempdir, so a parallel sibling test probing the
        // same pid-derived names under the machine's temp dir cannot race it.
        let base = tempfile::tempdir().unwrap();
        let mut occupied = Vec::new();
        let plain = base
            .path()
            .join(format!("cerulion-attach-dry-run-{}", std::process::id()));
        std::fs::create_dir(&plain).expect("occupy the plain candidate");
        occupied.push(plain);
        for n in 2..=(MAX_ROOT_COLLISIONS + 1) {
            let name = base.path().join(format!(
                "cerulion-attach-dry-run-{}-{n}",
                std::process::id()
            ));
            std::fs::create_dir(&name).expect("occupy a suffixed candidate");
            occupied.push(name);
        }
        let err = exclusive_dry_run_root_in(base.path()).expect_err("the bounded probe refuses");
        assert!(
            err.to_string().contains("already taken"),
            "the refusal names the collision bound; got: {err}"
        );
        // The count the refusal states is derived HERE from the occupied set,
        // never copied from the constant. Occupying exactly this set forces
        // the refusal, so the probe reaches no name beyond it; freeing the LAST
        // of them lets the probe land there, so it reaches every one. The two
        // together pin the count to the set's size.
        let probed = format!("{} names probed", occupied.len());
        assert!(
            err.to_string().contains(&probed),
            "the refusal must state the names it probed ({probed}); got: {err}"
        );
        let last = occupied.pop().expect("the last occupied name");
        std::fs::remove_dir(&last).expect("free the last candidate name");
        let landed =
            exclusive_dry_run_root_in(base.path()).expect("the last candidate name is reachable");
        assert_eq!(landed, last, "the probe reaches the last name it counts");
        std::fs::remove_dir(&landed).expect("test cleanup");
        for path in occupied {
            std::fs::remove_dir(&path).expect("test cleanup");
        }
    }

    #[test]
    fn an_uncreatable_root_refuses_and_names_the_remedies() {
        // A base that is a regular FILE: `create_dir` under it fails with an
        // error other than `AlreadyExists` for every user, root included (a
        // read-only directory does not stop a process that bypasses
        // permission checks). That is the refusal arm, not the retry arm.
        let dir = tempfile::tempdir().unwrap();
        let base = dir.path().join("not-a-directory");
        std::fs::write(&base, b"").unwrap();
        let err = exclusive_dry_run_root_in(&base).expect_err("an uncreatable root refuses");
        let msg = err.to_string();
        assert!(
            matches!(err, CliError::Validation(_)),
            "the refusal is a validation error; got {err:?}"
        );
        assert!(
            msg.contains("could not create a temporary root")
                && msg.contains("Point TMPDIR at a writable location")
                && msg.contains("run the dry-run from inside a workspace"),
            "the refusal names its cause and both remedies; got: {msg}"
        );
        // The refusal left nothing behind: the base is still the plain file.
        assert!(base.is_file());
    }
}
