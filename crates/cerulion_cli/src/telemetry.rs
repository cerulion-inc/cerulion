// SPDX-License-Identifier: AGPL-3.0-only
//! Usage telemetry for the `cerulion` CLI: the `cerulion telemetry` verb, the
//! one-time first-run notice and the `cli_command_run` event. The public
//! disclosure is `docs/telemetry.md`.
//!
//! Nothing is sent unless [`Client::from_env`] returns a client, which needs
//! a key and consent. The run that prints the notice sends nothing, so the
//! user reads it before the first event leaves the machine.

use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use cerulion_cli_engine::auth;
use cerulion_cli_engine::error::{CliError, CliResult};
use cerulion_telemetry::consent::{self, Source};
use cerulion_telemetry::{guard, Client, Common, EventSpec, Props, DEFAULT_SHUTDOWN_BUDGET};

use crate::cli::TelemetryAction;

/// One CLI invocation: which verb, how it ended, roughly how long it took.
pub const CLI_COMMAND_RUN: EventSpec = EventSpec {
    name: "cli_command_run",
    allowlist: &["verb", "subverb", "exit_code", "duration_bucket"],
};

/// Printed to stderr once per machine, on the first run that could send.
pub const NOTICE: &str = "\
Cerulion sends anonymous usage events from this CLI: the verb you ran, its \
exit code and a rough duration. Never arguments, paths, topic names or data.
Turn it off with `cerulion telemetry off` or DO_NOT_TRACK=1. Details: \
https://github.com/cerulion-inc/cerulion/blob/main/docs/telemetry.md";

/// Verbs that record no `cli_command_run`: the consent verb itself (an
/// opt-out must not be counted), the completion hook a shell runs on every
/// start, and the recorder daemon, which runs as its own long-lived process.
const UNRECORDED_VERBS: &[&str] = &["telemetry", "completions", "bagd"];

/// Subprocesses a recorded parent spawns; the parent's event covers them.
const UNRECORDED_SUBVERBS: &[&str] = &["run-worker", "run-gateway"];

/// The fixed properties every CLI event carries.
pub fn common() -> Common {
    Common {
        surface: "cli".into(),
        env: if cfg!(debug_assertions) {
            "dev"
        } else {
            "prod"
        }
        .into(),
        app_version: env!("CARGO_PKG_VERSION").into(),
        channel: None,
    }
}

/// A coarse duration: enough to tell a quick query from a long run, never a
/// timing precise enough to fingerprint anything.
pub fn duration_bucket(elapsed: Duration) -> &'static str {
    match elapsed.as_secs() {
        0 => "lt_1s",
        1..=9 => "1s_10s",
        10..=59 => "10s_1m",
        60..=599 => "1m_10m",
        _ => "gte_10m",
    }
}

/// The numeric process exit code, or `-1` if it is outside `0..=255`.
pub fn exit_code_number(code: ExitCode) -> i64 {
    (0..=u8::MAX)
        .find(|n| ExitCode::from(*n) == code)
        .map_or(-1, i64::from)
}

/// `cli_command_run` properties. `verb` and `subverb` are clap subcommand
/// names, fixed strings from the command definition, never user input.
pub fn command_run_props(
    verb: &str,
    subverb: Option<&str>,
    code: ExitCode,
    elapsed: Duration,
) -> Props {
    let mut props: Props = vec![("verb".into(), verb.into())];
    if let Some(subverb) = subverb {
        props.push(("subverb".into(), subverb.into()));
    }
    props.push(("exit_code".into(), exit_code_number(code).into()));
    props.push(("duration_bucket".into(), duration_bucket(elapsed).into()));
    props
}

/// Whether an invocation of `verb subverb` records a `cli_command_run`.
pub fn is_recorded(verb: &str, subverb: Option<&str>) -> bool {
    !UNRECORDED_VERBS.contains(&verb) && !subverb.is_some_and(|s| UNRECORDED_SUBVERBS.contains(&s))
}

/// A started invocation, finished by [`CommandRun::finish`].
pub struct CommandRun {
    client: Client,
    verb: String,
    subverb: Option<String>,
    started: Instant,
}

impl CommandRun {
    /// Begin recording, or `None` when nothing may be sent: no key or no
    /// consent, an unrecorded verb, or this run claimed the first-run notice.
    pub fn start(matches: &clap::ArgMatches) -> Option<CommandRun> {
        let (verb, sub) = matches.subcommand()?;
        let subverb = sub.subcommand_name();
        if !is_recorded(verb, subverb) {
            return None;
        }
        let client = Client::from_env(common())?;
        match consent::claim_notice() {
            Ok(false) => {}
            Ok(true) => {
                eprintln!("{NOTICE}\n");
                return None;
            }
            // Without a readable consent file the notice cannot be tracked,
            // so it cannot be known to have been shown: send nothing.
            Err(_) => return None,
        }
        Some(CommandRun {
            client,
            verb: verb.to_owned(),
            subverb: subverb.map(str::to_owned),
            started: Instant::now(),
        })
    }

    /// Record the outcome and flush within [`DEFAULT_SHUTDOWN_BUDGET`]. The
    /// identity is read here, after the command ran, so a first `login`
    /// is attributed to the account it just signed in.
    pub fn finish(mut self, code: ExitCode) {
        let props = command_run_props(
            &self.verb,
            self.subverb.as_deref(),
            code,
            self.started.elapsed(),
        );
        match signed_in_account() {
            Some(sub) => self.client.capture(CLI_COMMAND_RUN, &sub, props),
            None => {
                if let Ok(Some(anon_id)) = consent::anon_id() {
                    self.client
                        .capture_anonymous(CLI_COMMAND_RUN, &anon_id, props);
                }
            }
        }
        self.client.shutdown(DEFAULT_SHUTDOWN_BUDGET);
    }
}

/// The signed-in account id, if it has the shape of one.
fn signed_in_account() -> Option<String> {
    let loaded = auth::load();
    let id = &loaded.state()?.account_id;
    guard::check_sub(id).ok().map(|()| id.clone())
}

/// Human wording for the rule that decided the status.
fn source_label(source: Source) -> &'static str {
    match source {
        Source::NotCompiled => "not compiled into this build",
        Source::DoNotTrack => "set by DO_NOT_TRACK",
        Source::EnvVar => "set by CERULION_TELEMETRY",
        Source::File => "set by `cerulion telemetry on|off`",
        Source::Default => "default",
    }
}

/// `cerulion telemetry status|on|off`.
pub fn run_verb(action: TelemetryAction, out: &mut impl Write) -> CliResult<()> {
    let io = |e: std::io::Error| CliError::Io(e);
    if !matches!(action, TelemetryAction::Status) {
        let enabled = matches!(action, TelemetryAction::On);
        if consent::status().source == Source::NotCompiled {
            writeln!(
                out,
                "telemetry is not compiled into this build; it never sends anything"
            )
            .map_err(io)?;
            return Ok(());
        }
        consent::set_enabled(enabled).map_err(|e| {
            CliError::Validation(format!("could not update the telemetry consent file: {e}"))
        })?;
    }
    let status = consent::status();
    let state = if status.enabled { "on" } else { "off" };
    writeln!(out, "telemetry: {state} ({})", source_label(status.source)).map_err(io)?;
    if matches!(status.source, Source::DoNotTrack | Source::EnvVar) {
        writeln!(
            out,
            "the environment variable overrides the consent file until it is unset"
        )
        .map_err(io)?;
    }
    #[cfg(feature = "telemetry")]
    {
        if let Ok(path) = consent::file_path() {
            writeln!(out, "consent file: {}", path.display()).map_err(io)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn durations_fall_into_coarse_buckets() {
        let b = |s| duration_bucket(Duration::from_secs(s));
        assert_eq!(duration_bucket(Duration::from_millis(999)), "lt_1s");
        assert_eq!(b(1), "1s_10s");
        assert_eq!(b(59), "10s_1m");
        assert_eq!(b(60), "1m_10m");
        assert_eq!(b(600), "gte_10m");
        assert_eq!(b(u64::MAX), "gte_10m");
    }

    #[test]
    fn exit_codes_map_to_their_numbers() {
        assert_eq!(exit_code_number(ExitCode::SUCCESS), 0);
        assert_eq!(exit_code_number(ExitCode::FAILURE), 1);
        assert_eq!(exit_code_number(ExitCode::from(7)), 7);
        assert_eq!(exit_code_number(ExitCode::from(255)), 255);
    }

    #[test]
    fn command_run_props_stay_inside_the_allowlist_and_pass_the_guard() {
        for subverb in [None, Some("run")] {
            let props = command_run_props("graph", subverb, ExitCode::FAILURE, Duration::ZERO);
            let (kept, dropped) = guard::filter(props.clone(), CLI_COMMAND_RUN.allowlist);
            assert!(dropped.is_empty(), "{dropped:?}");
            assert_eq!(kept, props);
        }
        guard::check_event_name(CLI_COMMAND_RUN.name).expect("event name");
        let c = common();
        for value in [&c.surface, &c.env, &c.app_version] {
            guard::check_str(value).expect(value);
        }
    }

    #[test]
    fn consent_completion_daemon_and_subprocess_verbs_are_unrecorded() {
        assert!(!is_recorded("telemetry", Some("off")));
        assert!(!is_recorded("completions", None));
        assert!(!is_recorded("bagd", None));
        assert!(!is_recorded("graph", Some("run-worker")));
        assert!(!is_recorded("graph", Some("run-gateway")));
        assert!(is_recorded("graph", Some("run")));
        assert!(is_recorded("login", None));
    }

    #[test]
    fn every_unrecorded_name_is_a_real_subcommand() {
        let cli = <crate::cli::Cli as clap::CommandFactory>::command();
        let names: Vec<&str> = cli.get_subcommands().map(|c| c.get_name()).collect();
        for verb in UNRECORDED_VERBS {
            // `bagd` exists on every platform (a stub off Unix).
            assert!(names.contains(verb), "{verb}");
        }
        let graph = cli.find_subcommand("graph").expect("graph");
        for sub in UNRECORDED_SUBVERBS {
            assert!(graph.find_subcommand(sub).is_some(), "{sub}");
        }
    }
}
