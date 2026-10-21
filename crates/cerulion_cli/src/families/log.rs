// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion log`: the log records of every node in the running local graph.
//!
//! A verb family (see [`super`]): this module is the whole of the verb, from
//! its clap definition to its dispatch. The command logic lives in
//! `cerulion_cli_engine::log_cmd`.

use std::process::ExitCode;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};
use clap_complete::engine::ArgValueCandidates;
use clap_complete::CompletionCandidate;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::log_cmd::{self, LogLevel};

use super::VerbFamily;
use crate::cli::VerbLogClass;

// The family's top-level verb, declared as a one-variant `Subcommand` so the
// derive generates the same `log` command the `Commands` enum would.
#[derive(Subcommand)]
enum LogCmd {
    /// Node logs
    Log {
        #[command(subcommand)]
        action: LogAction,
    },
}

#[derive(Subcommand)]
enum LogAction {
    /// Print the log records of the running local graph, one line each
    ///
    /// Reads `/rosout`, the host's one log topic, as `rcl_interfaces/Log` records.
    /// Each line is
    /// `<seconds>.<nanoseconds> <LEVEL> <node> <target> <file>:<line> <message>`.
    Echo {
        /// Show only the records of the node with this logger name (repeat
        /// the flag for several nodes). For a ROS 2 node it is the node name.
        #[arg(long = "node", value_name = "NAME")]
        node: Vec<String>,
        /// Show records at this level or above: debug, info, warn or error
        #[arg(long, value_name = "LEVEL", add = ArgValueCandidates::new(level_names))]
        level: Option<String>,
    },
}

/// The `--level` candidates: the accepted level names, lowest first.
fn level_names() -> Vec<CompletionCandidate> {
    LogLevel::ALL
        .iter()
        .map(|level| CompletionCandidate::new(level.name()))
        .collect()
}

/// The `log` verb family.
pub(crate) struct Log;

impl VerbFamily for Log {
    fn name(&self) -> &'static str {
        "log"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("schema")
    }

    fn command(&self) -> Command {
        super::derived_verb::<LogCmd>()
    }

    /// `log echo` streams records to stdout the way `topic echo` streams
    /// frames, so like every topic verb it defaults quiet: the framework's
    /// own lifecycle lines must not interleave with the records.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = LogAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        super::exit_code_of(run(action))
    }
}

fn run(action: LogAction) -> CliResult<()> {
    match action {
        LogAction::Echo { node, level } => {
            // The filters are checked before anything is opened, so a bad
            // level is refused at once, with no topic attach and no
            // interrupt handler installed.
            let filter = log_cmd::resolve_filter(&node, level.as_deref())?;
            let running = crate::setup_ctrlc_handler()?;
            log_cmd::log_echo(
                &filter,
                running,
                &mut std::io::stdout(),
                &mut std::io::stderr(),
            )
        }
    }
}
