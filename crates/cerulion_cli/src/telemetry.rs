// SPDX-License-Identifier: AGPL-3.0-only
//! Usage telemetry for the `cerulion` CLI: the `cerulion telemetry` verb, the
//! one-time first-run notice and the `cli_command_run` event. The public
//! disclosure is `docs/telemetry.md`.
//!
//! Nothing is sent unless [`Client::from_env_or_key`] returns a client, which
//! needs a key (`POSTHOG_API_KEY`, else the `CERULION_POSTHOG_KEY` a release
//! build was compiled with) and consent. The run that prints the notice sends nothing, so the
//! user reads it before the first event leaves the machine.

use std::io::Write;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use cerulion_cli_engine::auth;
use cerulion_cli_engine::error::{CliError, CliResult};
use cerulion_cli_engine::login_cmd::LoginOutcome;
use cerulion_telemetry::consent::{self, Source};
use cerulion_telemetry::{guard, Client, Common, EventSpec, Props, DEFAULT_SHUTDOWN_BUDGET};

use crate::cli::TelemetryAction;

/// One CLI invocation: which verb, how it ended, roughly how long it took.
pub const CLI_COMMAND_RUN: EventSpec = EventSpec {
    name: "cli_command_run",
    allowlist: &[
        "verb",
        "subverb",
        "exit_code",
        "duration_bucket",
        "install_method",
    ],
};

/// A completed device login. The account is the event's `distinct_id`.
pub const CLI_LOGIN_COMPLETED: EventSpec = EventSpec {
    name: "cli_login_completed",
    allowlist: &["is_account_switch"],
};

/// The PostHog project key a release build carries; `None` in a source build.
/// Read by the compiler, so the key never passes through build-script output.
const BAKED_KEY: Option<&str> = option_env!("CERULION_POSTHOG_KEY");

/// Whether this process may send: set once [`CommandRun::start`] has
/// decided to record, so a login inside the run that printed the notice, or
/// inside an unrecorded verb, sends nothing either.
static SENDING: AtomicBool = AtomicBool::new(false);

/// Set when [`CommandRun::start`] could have sent but printed the notice
/// instead: a first login inside this run leaves its anonymous id pending.
static NOTICE_RUN: AtomicBool = AtomicBool::new(false);

/// Set when [`login_anon_id`] found an unmerged anonymous id it could not
/// carry because this run sends nothing.
static UNCARRIED: AtomicBool = AtomicBool::new(false);

/// The process's one client, installed by [`CommandRun::start`] and shut down
/// once by [`CommandRun::finish`], so every event of an invocation shares
/// one [`DEFAULT_SHUTDOWN_BUDGET`].
static CLIENT: Mutex<Option<Client>> = Mutex::new(None);

/// Next to the consent file: an anonymous id that a signed-in account has
/// not been merged with yet. Holds no data; its presence is the flag.
#[cfg(feature = "telemetry")]
const PENDING_ALIAS_FILE: &str = "telemetry_alias_pending";

/// Printed to stderr once per machine, on the first run that could send.
pub const NOTICE: &str = "\
Cerulion sends usage events from this CLI: the verb you ran, its exit code, \
a rough duration and how it was installed, under your Cerulion account id \
once you sign in and a random anonymous id before that. Never arguments, \
paths, topic names or data.
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
    install_method: Option<&str>,
) -> Props {
    let mut props: Props = vec![("verb".into(), verb.into())];
    if let Some(subverb) = subverb {
        props.push(("subverb".into(), subverb.into()));
    }
    props.push(("exit_code".into(), exit_code_number(code).into()));
    props.push(("duration_bucket".into(), duration_bucket(elapsed).into()));
    if let Some(method) = install_method {
        props.push(("install_method".into(), method.into()));
    }
    props
}

/// Whether an invocation of `verb subverb` records a `cli_command_run`.
pub fn is_recorded(verb: &str, subverb: Option<&str>) -> bool {
    !UNRECORDED_VERBS.contains(&verb) && !subverb.is_some_and(|s| UNRECORDED_SUBVERBS.contains(&s))
}

/// A started invocation, finished by [`CommandRun::finish`].
pub struct CommandRun {
    verb: String,
    subverb: Option<String>,
    started: Instant,
}

impl CommandRun {
    /// Begin recording, or `None` when nothing may be sent: no key or no
    /// consent, an unrecorded verb, or this run printed the first-run notice.
    pub fn start(matches: &clap::ArgMatches) -> Option<CommandRun> {
        let (verb, sub) = matches.subcommand()?;
        let subverb = sub.subcommand_name();
        if !is_recorded(verb, subverb) {
            return None;
        }
        let client = Client::from_env_or_key(BAKED_KEY, common())?;
        // Printed under the consent lock before it is recorded: concurrent
        // first runs print it once, and a run killed in between prints it
        // again next time. The run that prints it sends nothing.
        match consent::show_notice_once(|| eprintln!("{NOTICE}\n")) {
            Ok(false) => {}
            Ok(true) => {
                NOTICE_RUN.store(true, Ordering::Relaxed);
                return None;
            }
            // Without a readable consent file the notice cannot be tracked,
            // so it cannot be known to have been shown: send nothing.
            Err(_) => return None,
        }
        merge_pending_alias(&client);
        *CLIENT.lock().unwrap_or_else(PoisonError::into_inner) = Some(client);
        SENDING.store(true, Ordering::Relaxed);
        Some(CommandRun {
            verb: verb.to_owned(),
            subverb: subverb.map(str::to_owned),
            started: Instant::now(),
        })
    }

    /// Record the outcome and flush within [`DEFAULT_SHUTDOWN_BUDGET`]. The
    /// identity is read here, after the command ran, so a first `login`
    /// is attributed to the account it just signed in.
    pub fn finish(self, code: ExitCode) {
        let props = command_run_props(
            &self.verb,
            self.subverb.as_deref(),
            code,
            self.started.elapsed(),
            crate::install_marker::current_method(),
        );
        emit(CLI_COMMAND_RUN, props);
        SENDING.store(false, Ordering::Relaxed);
        let client = CLIENT.lock().unwrap_or_else(PoisonError::into_inner).take();
        if let Some(mut client) = client {
            client.shutdown(DEFAULT_SHUTDOWN_BUDGET);
        }
    }
}

/// Queue one event on this invocation's client, attributed to the signed-in
/// account or else the anonymous id. A no-op when this process sends
/// nothing. Consent is read again at every event, so a `cerulion telemetry
/// off` from another terminal while a long command runs stops its events.
pub fn emit(spec: EventSpec, props: Props) {
    if !SENDING.load(Ordering::Relaxed) || !consent::status().enabled {
        return;
    }
    let guard = CLIENT.lock().unwrap_or_else(PoisonError::into_inner);
    let Some(client) = guard.as_ref() else {
        return;
    };
    match auth::load().state().and_then(|s| hosted_sub(&s.account_id)) {
        Some(sub) => client.capture(spec, &sub, props),
        None => {
            if let Ok(Some(anon_id)) = consent::anon_id() {
                client.capture_anonymous(spec, &anon_id, props);
            }
        }
    }
}

#[cfg(feature = "telemetry")]
fn pending_alias_path() -> Option<std::path::PathBuf> {
    Some(
        consent::file_path()
            .ok()?
            .with_file_name(PENDING_ALIAS_FILE),
    )
}

/// Merge an anonymous id left pending by a first login that ran while the
/// notice was printed, now that this run may send. The marker is removed
/// once read, unless the anonymous id cannot be read: without a hosted
/// account there is nothing to merge.
fn merge_pending_alias(client: &Client) {
    #[cfg(feature = "telemetry")]
    {
        let Some(path) = pending_alias_path().filter(|p| p.exists()) else {
            return;
        };
        // Consent is read again: an opt-out since the client was built keeps
        // the marker, so nothing is sent and nothing owed is forgotten.
        if !consent::status().enabled {
            return;
        }
        let sub = auth::load().state().and_then(|s| hosted_sub(&s.account_id));
        match (sub, consent::anon_id()) {
            (_, Err(_)) => return,
            (Some(sub), Ok(Some(anon_id))) => client.alias(&sub, &anon_id),
            _ => {}
        }
        let _ = std::fs::remove_file(path);
    }
    #[cfg(not(feature = "telemetry"))]
    let _ = client;
}

/// The signed-in account id, if it is a hosted account id (a lowercase UUID,
/// the account service's user id). Any other shape, such as the base64url id
/// a self-hosted account service issues, is never sent as a person id: the
/// event falls back to the anonymous id.
fn hosted_sub(account_id: &str) -> Option<String> {
    guard::check_sub(account_id)
        .ok()
        .map(|()| account_id.to_owned())
}

/// The anonymous id to carry into a device login, so the account service can
/// merge this machine's anonymous events into the account that signs in.
/// `None` when this process sends nothing, and when `auth.json` records a
/// completed login: the id has then been merged into THAT account, and carrying it
/// into a login as someone else would merge the two people.
pub fn login_anon_id() -> Option<String> {
    if auth::load().state().is_some_and(|s| s.logged_in_ever) {
        return None;
    }
    if SENDING.load(Ordering::Relaxed) {
        // Consent is read again: `cerulion telemetry off` in another terminal
        // since this run started must keep the id out of the login.
        if !consent::status().enabled {
            return None;
        }
        return consent::anon_id().ok().flatten();
    }
    // Only the notice run defers a merge; any other non-sending run mints no
    // id and writes no consent file.
    if NOTICE_RUN.load(Ordering::Relaxed)
        && consent::status().enabled
        && consent::anon_id().ok().flatten().is_some()
    {
        UNCARRIED.store(true, Ordering::Relaxed);
    }
    None
}

/// After a successful login: on an account switch, replace the anonymous id
/// so it is never attributed to the previous account again; then, when this
/// process sends, record `cli_login_completed`. `carried` is the id
/// [`login_anon_id`] put in the device-start body. It is also merged into the
/// account from here, which covers an account service that ignores the
/// field; a service that already merged it makes this a repeat of the same
/// merge.
pub fn login_completed(outcome: &LoginOutcome, carried: Option<&str>) {
    #[cfg(feature = "telemetry")]
    {
        if outcome.switched_account && consent::file_path().is_ok_and(|p| p.exists()) {
            if let Some(path) = pending_alias_path() {
                let _ = std::fs::remove_file(path);
            }
            // An id that cannot be rotated must not keep sending: stop this
            // run's events rather than attribute them to the old account.
            if consent::rotate_anon_id().is_err() {
                SENDING.store(false, Ordering::Relaxed);
            }
        }
        // The run that printed the notice sends nothing, so the merge waits
        // for the next run that may send (see `merge_pending_alias`).
        if UNCARRIED.load(Ordering::Relaxed) && !outcome.switched_account {
            if let Some(path) = pending_alias_path() {
                let _ = std::fs::write(path, b"");
            }
        }
    }
    if !SENDING.load(Ordering::Relaxed) || !consent::status().enabled {
        return;
    }
    let Some(sub) = hosted_sub(&outcome.state.account_id) else {
        return;
    };
    let guard = CLIENT.lock().unwrap_or_else(PoisonError::into_inner);
    let Some(client) = guard.as_ref() else {
        return;
    };
    if let Some(anon_id) = carried {
        client.alias(&sub, anon_id);
    }
    client.capture(
        CLI_LOGIN_COMPLETED,
        &sub,
        login_props(outcome.switched_account),
    );
}

/// `cli_login_completed` properties.
pub fn login_props(is_account_switch: bool) -> Props {
    vec![("is_account_switch".into(), is_account_switch.into())]
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
        for (subverb, method) in [(None, None), (Some("run"), Some("install.sh"))] {
            let props =
                command_run_props("graph", subverb, ExitCode::FAILURE, Duration::ZERO, method);
            let (kept, dropped) = guard::filter(props.clone(), CLI_COMMAND_RUN.allowlist);
            assert!(dropped.is_empty(), "{dropped:?}");
            assert_eq!(kept, props);
        }
        guard::check_event_name(CLI_COMMAND_RUN.name).expect("event name");
        guard::check_event_name(CLI_LOGIN_COMPLETED.name).expect("event name");
        for switch in [false, true] {
            let (_, dropped) = guard::filter(login_props(switch), CLI_LOGIN_COMPLETED.allowlist);
            assert!(dropped.is_empty(), "{dropped:?}");
        }
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
