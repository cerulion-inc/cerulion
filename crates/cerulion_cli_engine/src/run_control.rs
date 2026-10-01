// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph pause <run>` and `cerulion graph resume <run>`: hold a live run
//! at a step boundary and stop its clock, then let it go on.
//!
//! # What the verb does
//!
//! It finds the run in the live-run registry, opens the run's pause page
//! ([`cerulion_core::pause_page`]), flips it, and records the new state in the
//! run's `run.json` as `"paused": true` or `"paused": false`. Everything else is
//! the run's own doing: the live loop of every process of the run (the monolith, or
//! each worker of a multi-process run) sees the page and holds at its next step
//! boundary, and the run clock reads the page too, so no timer skips a tick and no
//! timestamp jumps.
//!
//! # Who can be paused
//!
//! A run whose pause page exists: a `graph run` of this build on a Unix host, on the
//! real clock (the default), whose graph has no ROS 2 entries. A run that cannot be
//! paused says so rather than pretending: one started by an older build has no page,
//! `--time-source virtual` has no wall clock to stop, and a ROS 2 entry is a separate
//! process that does not read the page, so a pause could not hold the whole run.
//!
//! # Idempotent, and self-healing
//!
//! Pausing a paused run, or resuming a live one, changes nothing and succeeds. Every
//! call also re-writes `run.json` from the PAGE, which is the truth, so a verb killed
//! between flipping the page and writing the manifest is repaired by running it
//! again.
//!
//! # Serialised
//!
//! Two control commands on one run (an operator and a script, two terminals) take
//! turns. Each one flips the page, reads it back and mirrors it into `run.json` while
//! holding a lock on the run directory ([`crate::run_dir::transition_run_paused`]), so
//! the manifest always ends up showing the state the page ended in, and the page's
//! resume never races another resume.
//!
//! # Trust
//!
//! The registry names a run's directory, and any local process can publish a record.
//! The verb writes `run.json` only in a directory named for the run, owned by the
//! invoking user, whose manifest (when readable) names the run, and flips nothing
//! otherwise.

use std::path::Path;

use cerulion_core::pause_page::{pause_tag_for_run, MappedPausePage, PauseTransition};
use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{gather_current_runs, RunRecord, RunState};

use crate::bag_cmd::{select_run, RunSelection, RunTarget};
use crate::error::CliError;

/// The exit code of `graph pause` and `graph resume` when no live run matches:
/// the run is not running (it has ended, or never was).
pub const EXIT_NOT_RUNNING: u8 = 4;

/// Which way the verb moves the run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunControlOp {
    /// Hold the run and stop its clock.
    Pause,
    /// Let the run go on from where it stopped.
    Resume,
}

impl RunControlOp {
    /// The verb, as the operator typed it.
    #[must_use]
    pub fn verb(self) -> &'static str {
        match self {
            RunControlOp::Pause => "pause",
            RunControlOp::Resume => "resume",
        }
    }
}

/// What a successful verb did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunControlReport {
    /// The run that was addressed.
    pub run_id: u128,
    /// Its graph name.
    pub graph_name: String,
    /// The state the run is in now.
    pub paused: bool,
    /// Whether this call moved the run; `false` means it was already there.
    pub changed: bool,
    /// Why `run.json` could not be updated, when it could not. The pause itself
    /// stands: the page is the truth and the manifest is its mirror.
    pub manifest_warning: Option<String>,
}

impl RunControlReport {
    /// The one line the verb prints.
    #[must_use]
    pub fn render(&self) -> String {
        let state = if self.paused { "paused" } else { "running" };
        let how = if self.changed { "now" } else { "already" };
        format!(
            "run 0x{:032x} ({}) is {how} {state}",
            self.run_id, self.graph_name
        )
    }
}

impl RunControlReport {
    /// The warning for the operator when `run.json` could not be updated, else `None`.
    #[must_use]
    pub fn warning_line(&self) -> Option<String> {
        let warning = self.manifest_warning.as_ref()?;
        let state = if self.paused { "paused" } else { "running" };
        Some(format!(
            "Warning: the run is {state}, but its run.json could not be updated ({warning}), so \
             the viz badge and other readers of run.json may show the old state. Run the same \
             command again to repair it."
        ))
    }
}

/// Why the verb could not do what it was asked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RunControlError {
    /// No live run matches. Exit [`EXIT_NOT_RUNNING`].
    NotRunning(String),
    /// More than one run matches, and the verb will not guess.
    Ambiguous(String),
    /// The run is live but exposes no pause page: it was started by a build that
    /// predates the verb, it runs on virtual time, or its owner could not create
    /// the page.
    NotPausable(String),
    /// The registry or the page could not be read, the live-run picture could not be
    /// established, or the run's directory is not the one the registry claimed.
    Failed(String),
}

impl RunControlError {
    /// The exit code the verb returns for this error.
    #[must_use]
    pub fn exit_code(&self) -> u8 {
        match self {
            RunControlError::NotRunning(_) => EXIT_NOT_RUNNING,
            _ => 1,
        }
    }
}

impl std::fmt::Display for RunControlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunControlError::NotRunning(m)
            | RunControlError::Ambiguous(m)
            | RunControlError::NotPausable(m)
            | RunControlError::Failed(m) => f.write_str(m),
        }
    }
}

impl From<RunControlError> for CliError {
    fn from(e: RunControlError) -> Self {
        CliError::Validation(e.to_string())
    }
}

/// Pause or resume the live run `target` names (a run id, with or without `0x`, or a
/// graph name that matches exactly one live run).
///
/// # Errors
///
/// See [`RunControlError`].
pub fn control_run(target: &str, op: RunControlOp) -> Result<RunControlReport, RunControlError> {
    let gather = gather_current_runs()
        .map_err(|e| RunControlError::Failed(format!("could not gather live runs: {e}")))?;
    let record = pick_run(&gather.records, target, op, gather.completeness)?;
    apply(&record, op)
}

/// Whether `target` spells `record`'s run id (with or without `0x`), as opposed to its
/// graph name.
fn target_is_run_id(record: &RunRecord, target: &str) -> bool {
    let t = target.trim();
    let id = t
        .strip_prefix("0x")
        .or_else(|| t.strip_prefix("0X"))
        .unwrap_or(t);
    format!("{:032x}", record.run_id).eq_ignore_ascii_case(id)
}

/// PURE: the run `target` names among `records`, or why none can be addressed.
///
/// `completeness` is what the registry gather established. A gather that did not hear
/// every live writer is not evidence of absence, and a graph name it resolves to one
/// run may have a second, unheard, run behind it. So an unsettled gather never turns
/// "not found" into "not running" (exit 4), and never lets a NAME stand for one run;
/// a run id that matched exactly can stand, because no unheard run can share it.
fn pick_run(
    records: &[RunRecord],
    target: &str,
    op: RunControlOp,
    completeness: GatherCompleteness,
) -> Result<RunRecord, RunControlError> {
    let verb = op.verb();
    let unsettled = match completeness {
        GatherCompleteness::Settled => None,
        GatherCompleteness::Incomplete {
            live_writers,
            writers_heard,
        } => Some(format!(
            "the live-run registry did not settle ({writers_heard} of {live_writers} live \
             writers answered), so `{target}` cannot be confirmed. Try again, or name the run \
             by its run id"
        )),
    };
    match select_run(records, &RunTarget::Named(target.to_string())) {
        RunSelection::Attach(record) => {
            if record.state == RunState::Ending {
                return Err(RunControlError::NotRunning(format!(
                    "run 0x{:032x} ({}) is shutting down, so it cannot be {}d",
                    record.run_id, record.graph_name, verb
                )));
            }
            if let Some(why) = unsettled {
                if !target_is_run_id(&record, target) {
                    return Err(RunControlError::Failed(format!(
                        "{why}: another live run may share the name `{target}`"
                    )));
                }
            }
            Ok(*record)
        }
        RunSelection::Ambiguous(live) => Err(RunControlError::Ambiguous(format!(
            "`{target}` names more than one live run, and `graph {verb}` will not guess which \
             to {verb}:\n  {}\nName one by its run id (the first column).",
            live.join("\n  ")
        ))),
        RunSelection::NoSuchRun { .. } | RunSelection::NoLiveRun if unsettled.is_some() => {
            Err(RunControlError::Failed(unsettled.unwrap_or_default()))
        }
        RunSelection::NoSuchRun { requested, live } => {
            Err(RunControlError::NotRunning(if live.is_empty() {
                format!("`{requested}` is not running: no run is live on this machine")
            } else {
                format!(
                    "`{requested}` is not running. These runs are live:\n  {}",
                    live.join("\n  ")
                )
            }))
        }
        RunSelection::NoLiveRun => Err(RunControlError::NotRunning(format!(
            "`{target}` is not running: no run is live on this machine"
        ))),
    }
}

/// Flip the run's page and mirror the result into its manifest, as one step under the
/// run directory lock.
fn apply(record: &RunRecord, op: RunControlOp) -> Result<RunControlReport, RunControlError> {
    let tag = pause_tag_for_run(record.run_id);
    let page = MappedPausePage::open_unowned(&tag).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            RunControlError::NotPausable(format!(
                "run 0x{:032x} ({}) cannot be {}d: it has no pause page. A run started by a \
                 build that predates `graph pause`, a run on virtual time (`--time-source \
                 virtual`), a run whose graph has ROS 2 entries, and a run whose page could \
                 not be created all lack one; start the run again with this build to pause it \
                 where it can be",
                record.run_id,
                record.graph_name,
                op.verb()
            ))
        } else {
            RunControlError::Failed(format!(
                "could not open run 0x{:032x}'s pause page: {e}",
                record.run_id
            ))
        }
    })?;
    // The PAGE is the truth and the manifest its mirror, so the mirror is written from
    // the page on every call, including a call that changed nothing: that is what
    // repairs a verb killed between the two writes. The flip, the read-back and the
    // mirror are one step under the run directory lock, so a second command cannot
    // land its manifest write between this command's flip and its own.
    let (transition, mirrored) =
        crate::run_dir::transition_run_paused(Path::new(&record.run_dir), record.run_id, || {
            let transition = match op {
                RunControlOp::Pause => page.pause(),
                RunControlOp::Resume => page.resume(),
            };
            (transition, page.is_paused())
        })
        .map_err(|e| RunControlError::Failed(e.to_string()))?;
    Ok(RunControlReport {
        run_id: record.run_id,
        graph_name: record.graph_name.clone(),
        paused: page.is_paused(),
        changed: transition == PauseTransition::Changed,
        manifest_warning: mirrored.err().map(|e| e.to_string()),
    })
}

/// Run the verb and print its one line to `out`, for the CLI. When the run changed state
/// but `run.json` could not be updated, a warning line goes to `warn` as well: the verb
/// succeeded, and the operator is told which view may now be stale. The error carries
/// the exit code.
///
/// # Errors
///
/// See [`RunControlError`].
pub fn run_control_verb(
    target: &str,
    op: RunControlOp,
    out: &mut dyn std::io::Write,
    warn: &mut dyn std::io::Write,
) -> Result<(), RunControlError> {
    let report = control_run(target, op)?;
    writeln!(out, "{}", report.render())
        .map_err(|e| RunControlError::Failed(format!("could not write the result: {e}")))?;
    if let Some(line) = report.warning_line() {
        // Best effort: a closed stderr must not turn a success into a failure.
        let _ = writeln!(warn, "{line}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SETTLED: GatherCompleteness = GatherCompleteness::Settled;
    const UNSETTLED: GatherCompleteness = GatherCompleteness::Incomplete {
        live_writers: 2,
        writers_heard: 1,
    };

    fn rec(run_id: u128, graph: &str, state: RunState) -> RunRecord {
        RunRecord {
            run_id,
            supervisor_pid: 100 + run_id as u32,
            run_started_at_ns: 0,
            state,
            graph_name: graph.to_string(),
            run_dir: format!("/nonexistent/{run_id}"),
        }
    }

    /// A run directory under a fresh run directory root, holding a `run.json` that
    /// names `run_id` and one key the verb does not own.
    fn run_dir_under_root(run_id: u128) -> (tempfile::TempDir, std::path::PathBuf) {
        let root = tempfile::tempdir().expect("root");
        let dir = root.path().join(format!("nav-{run_id:032x}"));
        std::fs::create_dir(&dir).expect("run dir");
        std::fs::write(
            dir.join(crate::run_dir::RUN_MANIFEST_FILE),
            format!(r#"{{"version":1,"run_id":"0x{run_id:032x}","keep":"me"}}"#),
        )
        .expect("seed run.json");
        (root, dir)
    }

    fn manifest(dir: &Path) -> serde_json::Value {
        serde_json::from_slice(
            &std::fs::read(dir.join(crate::run_dir::RUN_MANIFEST_FILE)).expect("read"),
        )
        .expect("json")
    }

    #[test]
    fn a_run_id_with_or_without_the_prefix_picks_one_live_run() {
        let runs = [
            rec(0x2a, "nav", RunState::Live),
            rec(0x2b, "arm", RunState::Live),
        ];
        for target in [
            "0x0000000000000000000000000000002a",
            "0000000000000000000000000000002a",
            "0X0000000000000000000000000000002A",
            "nav",
        ] {
            let got = pick_run(&runs, target, RunControlOp::Pause, SETTLED).expect(target);
            assert_eq!(got.run_id, 0x2a, "{target}");
        }
    }

    #[test]
    fn no_match_is_not_running_with_exit_four_and_lists_what_is_live() {
        let runs = [rec(0x2a, "nav", RunState::Live)];
        let err = pick_run(&runs, "ghost", RunControlOp::Pause, SETTLED).expect_err("no such run");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        let text = err.to_string();
        assert!(text.contains("`ghost` is not running"), "{text}");
        assert!(text.contains("nav"), "the live runs are named: {text}");
    }

    #[test]
    fn nothing_live_is_also_not_running_with_exit_four() {
        let err = pick_run(&[], "nav", RunControlOp::Resume, SETTLED).expect_err("nothing live");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        assert!(err.to_string().contains("no run is live"), "{err}");
    }

    #[test]
    fn two_runs_of_one_graph_are_refused_not_guessed_and_exit_one() {
        let runs = [rec(1, "nav", RunState::Live), rec(2, "nav", RunState::Live)];
        let err = pick_run(&runs, "nav", RunControlOp::Pause, SETTLED).expect_err("ambiguous");
        assert_eq!(err.exit_code(), 1);
        let text = err.to_string();
        assert!(text.contains("will not guess"), "{text}");
        assert!(
            text.contains(&format!("0x{:032x}", 1)) && text.contains(&format!("0x{:032x}", 2)),
            "both candidates are named so the retry is a copy-paste: {text}"
        );
    }

    #[test]
    fn a_run_that_is_shutting_down_is_not_running() {
        let runs = [rec(7, "nav", RunState::Ending)];
        let err = pick_run(&runs, "nav", RunControlOp::Pause, SETTLED).expect_err("ending");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        assert!(err.to_string().contains("shutting down"), "{err}");
    }

    /// A gather that did not hear every live writer is not evidence of absence: a run
    /// it did not hear is reported as an unsettled picture (exit 1), never as "not
    /// running" (exit 4), whichever way the run is named.
    #[test]
    fn an_unsettled_gather_never_reports_a_run_as_not_running() {
        let runs = [rec(0x2a, "nav", RunState::Live)];
        for target in ["ghost", "0x00000000000000000000000000000063"] {
            let err =
                pick_run(&runs, target, RunControlOp::Pause, UNSETTLED).expect_err("unsettled");
            assert!(matches!(err, RunControlError::Failed(_)), "{err:?}");
            assert_eq!(err.exit_code(), 1, "{target}");
            assert!(err.to_string().contains("did not settle"), "{err}");
        }
        let err = pick_run(&[], "nav", RunControlOp::Pause, UNSETTLED).expect_err("nothing heard");
        assert!(matches!(err, RunControlError::Failed(_)), "{err:?}");
    }

    /// A graph name that resolves to one HEARD run may have an unheard twin, so an
    /// unsettled gather refuses the name; the run id of the same run still works,
    /// because no unheard run can share it.
    #[test]
    fn an_unsettled_gather_refuses_a_name_but_accepts_an_exact_run_id() {
        let runs = [rec(0x2a, "nav", RunState::Live)];
        let err = pick_run(&runs, "nav", RunControlOp::Pause, UNSETTLED).expect_err("name");
        assert!(matches!(err, RunControlError::Failed(_)), "{err:?}");
        let text = err.to_string();
        assert!(
            text.contains("another live run may share the name") && text.contains("run id"),
            "{text}"
        );
        let got = pick_run(
            &runs,
            "0x0000000000000000000000000000002a",
            RunControlOp::Pause,
            UNSETTLED,
        )
        .expect("an exact run id stands");
        assert_eq!(got.run_id, 0x2a);
    }

    #[test]
    fn a_run_with_no_pause_page_is_not_pausable_and_says_what_to_do() {
        // A run id nothing created a page for: the open fails NotFound.
        let record = rec(0x7fff_dead_beef_0001, "nav", RunState::Live);
        let err = apply(&record, RunControlOp::Pause).expect_err("no page");
        assert!(matches!(err, RunControlError::NotPausable(_)), "{err:?}");
        assert_eq!(err.exit_code(), 1);
        let text = err.to_string();
        assert!(text.contains("no pause page"), "{text}");
        assert!(text.contains("ROS 2"), "{text}");
        assert!(text.contains("start the run again"), "{text}");
    }

    #[test]
    fn pausing_flips_the_page_and_the_manifest_and_a_repeat_changes_nothing() {
        let run_id: u128 =
            0x5eed_0000_0000_0000_0000_0000_0000_0000 | u128::from(std::process::id());
        let owner = MappedPausePage::create_owned(&pause_tag_for_run(run_id)).expect("page");
        let (_root, dir) = run_dir_under_root(run_id);
        let mut record = rec(run_id, "nav", RunState::Live);
        record.run_dir = dir.display().to_string();

        let first = apply(&record, RunControlOp::Pause).expect("pause");
        assert!(first.paused && first.changed, "{first:?}");
        assert!(owner.is_paused(), "the owner's page must show the pause");
        let doc = manifest(&dir);
        assert_eq!(doc["paused"], true);
        assert_eq!(
            doc["keep"], "me",
            "every key the verb does not own survives"
        );

        let again = apply(&record, RunControlOp::Pause).expect("pause again");
        assert!(again.paused && !again.changed, "{again:?}");
        assert!(
            again.render().contains("already paused"),
            "{}",
            again.render()
        );

        let resumed = apply(&record, RunControlOp::Resume).expect("resume");
        assert!(!resumed.paused && resumed.changed, "{resumed:?}");
        assert!(!owner.is_paused());
        assert_eq!(
            manifest(&dir)["paused"],
            false,
            "resume writes false, not an absent key"
        );
        let again = apply(&record, RunControlOp::Resume).expect("resume again");
        assert!(!again.changed && again.render().contains("already running"));
    }

    #[test]
    fn a_manifest_that_cannot_be_written_is_a_warning_and_the_pause_stands() {
        let run_id: u128 =
            0x5eee_0000_0000_0000_0000_0000_0000_0000 | u128::from(std::process::id());
        let owner = MappedPausePage::create_owned(&pause_tag_for_run(run_id)).expect("page");
        let (_root, dir) = run_dir_under_root(run_id);
        std::fs::remove_file(dir.join(crate::run_dir::RUN_MANIFEST_FILE)).expect("remove");
        let mut record = rec(run_id, "nav", RunState::Live);
        record.run_dir = dir.display().to_string();
        let report = apply(&record, RunControlOp::Pause).expect("the pause itself succeeds");
        assert!(report.paused && owner.is_paused());
        assert!(
            report.manifest_warning.is_some(),
            "the lost mirror is reported, not swallowed"
        );
        let line = report.warning_line().expect("the operator is told");
        assert!(
            line.starts_with("Warning:") && line.contains("paused") && line.contains("again"),
            "{line}"
        );
    }

    /// The registry is shared memory any local process can publish into, so a record's
    /// directory is a claim. A directory not named for the run, or whose manifest names
    /// another run, is refused BEFORE the page moves and before any file is written.
    #[test]
    fn a_run_directory_the_registry_claims_but_is_not_this_runs_is_refused_untouched() {
        let run_id: u128 =
            0x5eef_0000_0000_0000_0000_0000_0000_0000 | u128::from(std::process::id());
        let owner = MappedPausePage::create_owned(&pause_tag_for_run(run_id)).expect("page");
        let mut record = rec(run_id, "nav", RunState::Live);

        // A directory that is not named for this run (another run's, say).
        let (_a, other) = run_dir_under_root(run_id ^ 1);
        record.run_dir = other.display().to_string();
        let err = apply(&record, RunControlOp::Pause).expect_err("not named for the run");
        assert!(
            err.to_string().contains("does not carry the run id"),
            "{err}"
        );
        assert!(!owner.is_paused(), "a refused verb must not flip the page");
        assert!(manifest(&other).get("paused").is_none());

        // Named for the run, but its manifest belongs to another run.
        let (_b, dir) = run_dir_under_root(run_id);
        std::fs::write(
            dir.join(crate::run_dir::RUN_MANIFEST_FILE),
            format!(r#"{{"version":1,"run_id":"0x{:032x}"}}"#, run_id ^ 1),
        )
        .expect("rewrite run.json");
        record.run_dir = dir.display().to_string();
        let err = apply(&record, RunControlOp::Pause).expect_err("wrong run");
        assert!(err.to_string().contains("different run"), "{err}");
        assert!(!owner.is_paused());
        assert!(manifest(&dir).get("paused").is_none());

        // A path that is not there at all.
        record.run_dir = "/nonexistent/run".to_string();
        let err = apply(&record, RunControlOp::Pause).expect_err("missing");
        assert!(matches!(err, RunControlError::Failed(_)), "{err:?}");
        assert!(!owner.is_paused());
    }

    /// Overlapping pauses and resumes: whatever order the commands land in, the manifest
    /// ends up showing the state the page ended in, and the page is never left holding a
    /// resume that two commands interleaved.
    #[test]
    fn overlapping_commands_leave_the_manifest_showing_the_state_the_page_ended_in() {
        let run_id: u128 =
            0x5ef0_0000_0000_0000_0000_0000_0000_0000 | u128::from(std::process::id());
        let owner = MappedPausePage::create_owned(&pause_tag_for_run(run_id)).expect("page");
        let (_root, dir) = run_dir_under_root(run_id);
        let mut record = rec(run_id, "nav", RunState::Live);
        record.run_dir = dir.display().to_string();
        for round in 0..20 {
            std::thread::scope(|s| {
                for i in 0..8 {
                    let op = if (round + i) % 2 == 0 {
                        RunControlOp::Pause
                    } else {
                        RunControlOp::Resume
                    };
                    let record = &record;
                    s.spawn(move || {
                        apply(record, op).expect("apply");
                    });
                }
            });
            assert_eq!(
                manifest(&dir)["paused"],
                owner.is_paused(),
                "round {round}: the manifest must mirror the page"
            );
        }
    }

    #[test]
    fn the_rendered_line_says_which_run_and_what_state() {
        let report = RunControlReport {
            run_id: 0x2a,
            graph_name: "nav".to_string(),
            paused: true,
            changed: true,
            manifest_warning: None,
        };
        assert_eq!(
            report.render(),
            "run 0x0000000000000000000000000000002a (nav) is now paused"
        );
        assert_eq!(report.warning_line(), None);
    }
}
