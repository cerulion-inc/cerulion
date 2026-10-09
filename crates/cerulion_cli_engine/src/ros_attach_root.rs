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

/// The root `cerulion ros2 attach` will run against.
///
/// Construct via [`AttachRoot::resolve`]; drop the value (or the guard it
/// hands out) at the end of the verb so an [`AttachRoot::ExclusiveTemp`]
/// root never outlives the run that created it.
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

    /// The RAII cleanup guard for this root: `Some` only for
    /// [`AttachRoot::ExclusiveTemp`] (a workspace root is never cleaned up).
    pub fn guard(&self) -> Option<ExclusiveRootGuard> {
        match self {
            AttachRoot::Workspace(_) => None,
            AttachRoot::ExclusiveTemp(root) => Some(ExclusiveRootGuard(Some(root.clone()))),
        }
    }

    /// Resolve the attach root by discovering the workspace from `start_dir`:
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
    pub fn resolve(start_dir: &Path, dry_run: bool) -> CliResult<Self> {
        resolve_in(start_dir, dry_run, &std::env::temp_dir())
    }
}

/// [`AttachRoot::resolve`] over an INJECTED temp base: the production form
/// reads `std::env::temp_dir()`, the tests pin their own tempdir.
fn resolve_in(start_dir: &Path, dry_run: bool, temp_base: &Path) -> CliResult<AttachRoot> {
    match CerulionWorkspace::discover(start_dir) {
        Ok(ws) => Ok(AttachRoot::Workspace(ws.root)),
        Err(CliError::WorkspaceNotFound { .. }) if dry_run => Ok(AttachRoot::ExclusiveTemp(
            exclusive_dry_run_root_in(temp_base)?,
        )),
        Err(err) => Err(err),
    }
}

/// Root handed to the engine when `--dry-run` has no workspace.
///
/// A name collision (a stale dir from a killed earlier run, or one
/// pre-created by hand) retries with `-2`, `-3`, ... up to
/// [`MAX_ROOT_COLLISIONS`] attempts; the loop is BOUNDED so a temp dir that
/// answers `AlreadyExists` to every candidate refuses loudly instead of
/// hanging the verb. Any OTHER failure to create the root is an error (never a
/// silent builtins-only degradation, which would make the report untrustworthy
/// without telling the operator why). The refusal names the two remedies that
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
                         taken ({MAX_ROOT_COLLISIONS} attempts, names derived from pid {}); \
                         the dry-run needs one writable directory outside a workspace \
                         (created empty, removed again at exit). Point TMPDIR at a \
                         writable location, or run the dry-run from inside a workspace.",
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
                     workspace (created empty, removed again at exit). Point TMPDIR at a \
                     writable location, or run the dry-run from inside a workspace.",
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
/// never outlives the verb that created it. Constructed from
/// [`AttachRoot::guard`]; `None` on every arm that had a workspace (there is
/// nothing to clean up, and the workspace's own files must not be touched).
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
            let resolved = resolve_in(&start, dry_run, temp_base.path()).expect("workspace arm");
            assert!(
                matches!(resolved, AttachRoot::Workspace(ref p) if p == ws.path()),
                "dry_run={dry_run}: a found workspace is the root; got {resolved:?}"
            );
            // A workspace root is NEVER cleaned up: the guard is None.
            assert!(resolved.guard().is_none());
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
        let resolved = resolve_in(outside.path(), true, temp_base.path()).expect("dry-run arm");
        let root = match &resolved {
            AttachRoot::ExclusiveTemp(root) => root.clone(),
            other => panic!("expected the exclusive temp root; got {other:?}"),
        };
        assert_eq!(root.parent(), Some(temp_base.path()));
        assert!(std::fs::read_dir(&root).unwrap().next().is_none());
        drop(resolved.guard());
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
        for path in occupied {
            std::fs::remove_dir(&path).expect("test cleanup");
        }
        assert!(
            err.to_string().contains("already taken"),
            "the refusal names the collision bound; got: {err}"
        );
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
