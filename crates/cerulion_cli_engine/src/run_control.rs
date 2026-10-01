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
//! real clock (the default). A run that cannot be paused says so rather than
//! pretending: one started by an older build has no page, and `--time-source virtual`
//! has no wall clock to stop.
//!
//! # Idempotent, and self-healing
//!
//! Pausing a paused run, or resuming a live one, changes nothing and succeeds. Every
//! call also re-writes `run.json` from the PAGE, which is the truth, so a verb killed
//! between flipping the page and writing the manifest is repaired by running it
//! again.

use std::path::Path;

use cerulion_core::pause_page::{pause_tag_for_run, MappedPausePage, PauseTransition};
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
    /// The registry or the page could not be read.
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
    let record = pick_run(&gather.records, target, op)?;
    apply(&record, op)
}

/// PURE: the run `target` names among `records`, or why none can be addressed.
fn pick_run(
    records: &[RunRecord],
    target: &str,
    op: RunControlOp,
) -> Result<RunRecord, RunControlError> {
    let verb = op.verb();
    match select_run(records, &RunTarget::Named(target.to_string())) {
        RunSelection::Attach(record) => {
            if record.state == RunState::Ending {
                return Err(RunControlError::NotRunning(format!(
                    "run 0x{:032x} ({}) is shutting down, so it cannot be {}d",
                    record.run_id, record.graph_name, verb
                )));
            }
            Ok(*record)
        }
        RunSelection::Ambiguous(live) => Err(RunControlError::Ambiguous(format!(
            "`{target}` names more than one live run, and `graph {verb}` will not guess which \
             to {verb}:\n  {}\nName one by its run id (the first column).",
            live.join("\n  ")
        ))),
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

/// Flip the run's page and mirror the result into its manifest.
fn apply(record: &RunRecord, op: RunControlOp) -> Result<RunControlReport, RunControlError> {
    let tag = pause_tag_for_run(record.run_id);
    let page = MappedPausePage::open_unowned(&tag).map_err(|e| {
        if e.kind() == std::io::ErrorKind::NotFound {
            RunControlError::NotPausable(format!(
                "run 0x{:032x} ({}) cannot be {}d: it has no pause page. A run started by a \
                 build that predates `graph pause`, a run on virtual time (`--time-source \
                 virtual`), and a run whose page could not be created all lack one; start the \
                 run again with this build to pause it",
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
    let transition = match op {
        RunControlOp::Pause => page.pause(),
        RunControlOp::Resume => page.resume(),
    };
    let paused = page.is_paused();
    // The PAGE is the truth and the manifest its mirror, so the mirror is written from
    // the page on every call, including a call that changed nothing: that is what
    // repairs a verb killed between the two writes.
    let manifest_warning = crate::run_dir::declare_run_paused(Path::new(&record.run_dir), paused)
        .err()
        .map(|e| e.to_string());
    Ok(RunControlReport {
        run_id: record.run_id,
        graph_name: record.graph_name.clone(),
        paused,
        changed: transition == PauseTransition::Changed,
        manifest_warning,
    })
}

/// Run the verb and print its one line, for the CLI. The error carries the exit code.
///
/// # Errors
///
/// See [`RunControlError`].
pub fn run_control_verb(
    target: &str,
    op: RunControlOp,
    out: &mut dyn std::io::Write,
) -> Result<(), RunControlError> {
    let report = control_run(target, op)?;
    writeln!(out, "{}", report.render())
        .map_err(|e| RunControlError::Failed(format!("could not write the result: {e}")))?;
    if let Some(warning) = &report.manifest_warning {
        tracing::warn!(
            run_id = format_args!("0x{:032x}", report.run_id),
            state = if report.paused { "paused" } else { "running" },
            error = %warning,
            "the run changed state but its run.json could not be updated"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
            let got = pick_run(&runs, target, RunControlOp::Pause).expect(target);
            assert_eq!(got.run_id, 0x2a, "{target}");
        }
    }

    #[test]
    fn no_match_is_not_running_with_exit_four_and_lists_what_is_live() {
        let runs = [rec(0x2a, "nav", RunState::Live)];
        let err = pick_run(&runs, "ghost", RunControlOp::Pause).expect_err("no such run");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        let text = err.to_string();
        assert!(text.contains("`ghost` is not running"), "{text}");
        assert!(text.contains("nav"), "the live runs are named: {text}");
    }

    #[test]
    fn nothing_live_is_also_not_running_with_exit_four() {
        let err = pick_run(&[], "nav", RunControlOp::Resume).expect_err("nothing live");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        assert!(err.to_string().contains("no run is live"), "{err}");
    }

    #[test]
    fn two_runs_of_one_graph_are_refused_not_guessed_and_exit_one() {
        let runs = [rec(1, "nav", RunState::Live), rec(2, "nav", RunState::Live)];
        let err = pick_run(&runs, "nav", RunControlOp::Pause).expect_err("ambiguous");
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
        let err = pick_run(&runs, "nav", RunControlOp::Pause).expect_err("ending");
        assert_eq!(err.exit_code(), EXIT_NOT_RUNNING);
        assert!(err.to_string().contains("shutting down"), "{err}");
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
        assert!(text.contains("start the run again"), "{text}");
    }

    #[test]
    fn pausing_flips_the_page_and_the_manifest_and_a_repeat_changes_nothing() {
        let run_id: u128 =
            0x5eed_0000_0000_0000_0000_0000_0000_0000 | u128::from(std::process::id());
        let owner = MappedPausePage::create_owned(&pause_tag_for_run(run_id)).expect("page");
        let dir = tempfile::tempdir().expect("tempdir");
        std::fs::write(
            dir.path().join(crate::run_dir::RUN_MANIFEST_FILE),
            br#"{"version":1,"run_id":"0x1","keep":"me"}"#,
        )
        .expect("seed run.json");
        let mut record = rec(run_id, "nav", RunState::Live);
        record.run_dir = dir.path().display().to_string();

        let first = apply(&record, RunControlOp::Pause).expect("pause");
        assert!(first.paused && first.changed, "{first:?}");
        assert!(owner.is_paused(), "the owner's page must show the pause");
        let manifest: serde_json::Value = serde_json::from_slice(
            &std::fs::read(dir.path().join(crate::run_dir::RUN_MANIFEST_FILE)).expect("read"),
        )
        .expect("json");
        assert_eq!(manifest["paused"], true);
        assert_eq!(
            manifest["keep"], "me",
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
        let manifest: serde_json::Value = serde_json::from_slice(
            &std::fs::read(dir.path().join(crate::run_dir::RUN_MANIFEST_FILE)).expect("read"),
        )
        .expect("json");
        assert_eq!(
            manifest["paused"], false,
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
        let record = rec(run_id, "nav", RunState::Live); // run_dir does not exist
        let report = apply(&record, RunControlOp::Pause).expect("the pause itself succeeds");
        assert!(report.paused && owner.is_paused());
        assert!(
            report.manifest_warning.is_some(),
            "the lost mirror is reported, not swallowed"
        );
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
    }
}
