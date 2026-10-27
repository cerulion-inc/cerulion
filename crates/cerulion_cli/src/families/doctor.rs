// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion doctor`: read-only checks of this host, each with a verdict and
//! the exact fix.
//!
//! A verb family (see [`super`]): parsing, dispatch and output only. The
//! checks live in `cerulion_cli_engine::doctor`.

use std::process::ExitCode;
use std::time::Duration;

use clap::builder::PossibleValuesParser;
use clap::{ArgMatches, Args, Command, FromArgMatches, Subcommand};

use super::VerbFamily;
use crate::cli::VerbLogClass;

// The family's one verb, declared as a one-variant `Subcommand` whose
// arguments are a separate `Args` struct, so `run` parses the family's own
// sub-matches straight into it.
#[derive(Subcommand)]
enum DoctorCmd {
    /// Check this host for problems that slow or stop Cerulion, with the fix for each.
    ///
    /// Every check reads one fact about this machine and reports pass, info,
    /// warn, fail or skip; each warn and fail says why it matters and the exact
    /// command or edit that fixes it. No check needs root, and none writes a
    /// file or changes a sysctl or a network setting; the open-file check
    /// raises this process's own limit the way `graph run` does. Exit 0 when
    /// no check failed (warnings included), 1 when any check failed, 2 for a
    /// usage error.
    Doctor(DoctorArgs),
}

#[derive(Args)]
struct DoctorArgs {
    /// Print the report as one JSON object on stdout instead of the table.
    #[arg(long)]
    json: bool,
    /// Skip the checks that use the network; no check in this release does.
    /// `CERULION_NETWORK=off` does the same.
    #[arg(long)]
    local: bool,
    /// Run only this check (repeatable). Default: every check.
    #[arg(
        long = "check",
        value_name = "ID",
        value_parser = PossibleValuesParser::new(cerulion_cli_engine::doctor::check_ids())
    )]
    checks: Vec<String>,
    /// How long to wait for the checks, in milliseconds. A check that has not
    /// answered by then is reported as a warning. Default: twice the bounded
    /// scan of the stale-node check.
    #[arg(
        long = "timeout-ms",
        value_name = "MS",
        value_parser = clap::value_parser!(u64).range(1..)
    )]
    timeout_ms: Option<u64>,
}

/// The `doctor` verb family.
pub(crate) struct Doctor;

impl VerbFamily for Doctor {
    fn name(&self) -> &'static str {
        "doctor"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("clean")
    }

    fn command(&self) -> Command {
        super::derived_verb::<DoctorCmd>()
    }

    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    /// The verb diagnoses a machine that has never signed in, among others,
    /// and reaches no account, so the login gate does not run before it.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        false
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let args = DoctorArgs::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        run(&args)
    }
}

fn run(args: &DoctorArgs) -> ExitCode {
    use cerulion_cli_engine::doctor;

    let selected = match doctor::select(&args.checks) {
        Ok(selected) => selected,
        Err(unknown) => {
            eprintln!("Error: {unknown}");
            return ExitCode::from(2);
        }
    };
    let cwd = match std::env::current_dir() {
        Ok(cwd) => cwd,
        Err(e) => {
            eprintln!("Error: cannot read the current directory: {e}");
            return ExitCode::FAILURE;
        }
    };
    let workspace_root = cerulion_cli_engine::workspace::CerulionWorkspace::discover(&cwd)
        .ok()
        .map(|ws| ws.root);
    let ctx = doctor::Context {
        cwd,
        workspace_root,
    };
    let timeout = args
        .timeout_ms
        .map_or(doctor::DEFAULT_TIMEOUT, Duration::from_millis);
    let report = doctor::run(&selected, &ctx, doctor::network_gate(args.local), timeout);
    if args.json {
        println!(
            "{}",
            serde_json::to_string_pretty(&report.to_json())
                .unwrap_or_else(|_| report.to_json().to_string())
        );
    } else {
        print!("{}", report.render_human());
    }
    ExitCode::from(report.exit_code())
}
