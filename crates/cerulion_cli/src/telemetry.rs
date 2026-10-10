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
/// not been merged with yet. Holds no data; its presence is the flag. The
/// CLI itself has sent nothing by the time the notice run ends, so what this
/// merge joins to the account are the events another surface on the
/// machine, such as the vizd daemon, sent under the same consent file's id.
#[cfg(feature = "telemetry")]
const PENDING_ALIAS_FILE: &str = "telemetry_alias_pending";
/// Beside `telemetry.json`: the account the anonymous id was last used for.
/// A signed-in account that differs from it has the anonymous id rotated
/// before anything more is sent, so one id is never attributed to two
/// accounts, even when a login saved the new account and then failed. An
/// empty record is a claim: a login carried the id and no account is
/// recorded for it, because that login is still running, failed, or could
/// not write the account; or a rotation the id was owed failed. Either way
/// no later login carries it, and every run with an account rotates it
/// first.
#[cfg(feature = "telemetry")]
const ANON_ACCOUNT_FILE: &str = "telemetry_anon_account";

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
        // A closed stderr must not stop the command: the notice is then never
        // saved as shown, so a later run shows it, and this run sends nothing.
        match consent::notice_shown() {
            Ok(true) => {}
            // Written outside the consent lock, so a stderr nobody drains
            // holds up only this run. Two first runs at once may both show
            // it; neither sends. It is saved as shown only once written.
            Ok(false) | Err(cerulion_telemetry::Error::Json(_)) => {
                if writeln!(std::io::stderr(), "{NOTICE}\n").is_ok() {
                    let _ = consent::mark_notice_shown();
                }
                NOTICE_RUN.store(true, Ordering::Relaxed);
                return None;
            }
            // Without a readable consent file the notice cannot be tracked,
            // so it cannot be known to have been shown: send nothing.
            Err(_) => return None,
        }
        if !settle_anon_account() {
            return None;
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
/// So is the account: a login in this run, or in another terminal, may have
/// saved another account, and the anonymous id is settled for it before the
/// event picks its identity. An id that cannot be settled stops this run's
/// events rather than send them under the previous account's id.
pub fn emit(spec: EventSpec, props: Props) {
    if !SENDING.load(Ordering::Relaxed) || !consent::status().enabled {
        return;
    }
    if !settle_anon_account() {
        SENDING.store(false, Ordering::Relaxed);
        return;
    }
    let guard = CLIENT.lock().unwrap_or_else(PoisonError::into_inner);
    let Some(client) = guard.as_ref() else {
        return;
    };
    match auth::load().state().and_then(|s| hosted_sub(&s.account_id)) {
        Some(sub) => {
            consent::while_enabled(|| client.capture(spec, &sub, props));
        }
        None => {
            if let Ok(Some(anon_id)) = consent::anon_id() {
                consent::while_enabled(|| client.capture_anonymous(spec, &anon_id, props));
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

#[cfg(feature = "telemetry")]
fn anon_account_path() -> Option<std::path::PathBuf> {
    Some(consent::file_path().ok()?.with_file_name(ANON_ACCOUNT_FILE))
}

/// What `telemetry_anon_account` says about the anonymous id.
#[cfg(feature = "telemetry")]
enum AnonAccount {
    /// No record: no account has used the id.
    Unclaimed,
    /// The account the id was last used for.
    Bound(String),
    /// A record exists but cannot be read, or there is no home to read it
    /// from: the id's account is unknown, so it is treated as another
    /// account's, never carried into a login and rotated before use.
    Unknown,
}

#[cfg(feature = "telemetry")]
fn anon_account() -> AnonAccount {
    let Some(path) = anon_account_path() else {
        return AnonAccount::Unknown;
    };
    match std::fs::read_to_string(path) {
        Ok(account) => AnonAccount::Bound(account),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => AnonAccount::Unclaimed,
        Err(_) => AnonAccount::Unknown,
    }
}

/// Record `account` as the anonymous id's account; `false` when the record
/// could not be written. The old record then stays, so a later run rotates
/// the id again rather than reuse it, and the caller must not send under an
/// id whose account is not on disk.
#[cfg(feature = "telemetry")]
fn bind_anon_account(account: &str) -> bool {
    let Some(path) = anon_account_path() else {
        return false;
    };
    match std::fs::write(path, account) {
        Ok(()) => true,
        Err(e) => {
            tracing::warn!(error = %e, "telemetry anonymous id account record not written");
            false
        }
    }
}

/// Claim the anonymous id for the login about to carry it: create its
/// account record, empty, before the id leaves the machine. The creation is
/// exclusive, so of concurrent first logins exactly one carries the id, and
/// a later login never finds the id unclaimed, even once `auth.json` is
/// gone and the account was never written over the claim. The claim is
/// never released: the login asks for the id right before its device-start
/// request, and a login that fails after that request may still have been
/// joined to its account by the account service, so the id is never carried
/// again and the next run with an account rotates it.
/// `false` when another login holds the claim or it cannot be made: the id
/// then stays on the machine.
#[cfg(feature = "telemetry")]
fn claim_anon_id() -> bool {
    let Some(path) = anon_account_path() else {
        return false;
    };
    match std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
    {
        Ok(_) => true,
        Err(e) => {
            if e.kind() != std::io::ErrorKind::AlreadyExists {
                tracing::warn!(error = %e, "telemetry anonymous id not claimed for the login");
            }
            false
        }
    }
}

/// Remove the pending-alias marker; a marker that is already gone is fine.
#[cfg(feature = "telemetry")]
fn remove_pending_alias(path: &std::path::Path) {
    if let Err(e) = std::fs::remove_file(path) {
        if e.kind() != std::io::ErrorKind::NotFound {
            tracing::warn!(error = %e, "telemetry alias marker not removed");
        }
    }
}

/// Replace the anonymous id, dropping any merge left pending for the old
/// one. `true` when there was no consent file, so no id, to replace.
#[cfg(feature = "telemetry")]
fn rotate_existing_anon_id() -> bool {
    if !consent::file_path().is_ok_and(|p| p.exists()) {
        return true;
    }
    if let Some(path) = pending_alias_path() {
        remove_pending_alias(&path);
    }
    consent::rotate_anon_id().is_ok()
}

/// Rotate the anonymous id when the signed-in account is not the one it was
/// last used for; with no record yet, the signed-in account claims it.
/// `false` while that rotation, or the record of the account, fails: the run
/// must then send nothing, or the old account's id would be attributed to
/// the new one, or an id sent under this account could later be carried into
/// another.
fn settle_anon_account() -> bool {
    #[cfg(feature = "telemetry")]
    {
        let Some(account) = auth::load().state().map(|s| s.account_id.clone()) else {
            return true;
        };
        match anon_account() {
            AnonAccount::Bound(bound) if bound == account => return true,
            AnonAccount::Unclaimed => {}
            AnonAccount::Bound(_) | AnonAccount::Unknown => {
                if !rotate_existing_anon_id() {
                    return false;
                }
            }
        }
        bind_anon_account(&account)
    }
    #[cfg(not(feature = "telemetry"))]
    true
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
        let sub = auth::load().state().and_then(|s| hosted_sub(&s.account_id));
        let Ok(anon_id) = consent::anon_id() else {
            return;
        };
        // Settled under the consent lock: of concurrent runs, only the first
        // to take it still finds the marker, and an opt-out keeps the marker
        // so nothing owed is forgotten. A crash before the removal leaves
        // the marker for the next run.
        consent::while_enabled(|| {
            if !path.exists() {
                return;
            }
            if let (Some(sub), Some(anon_id)) = (sub, anon_id) {
                client.alias(&sub, &anon_id);
            }
            remove_pending_alias(&path);
        });
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
/// `None` when this process sends nothing, and when the id was already used
/// for an account: `auth.json` records a completed or a signed-out login, or
/// `telemetry_anon_account` names the account the id was merged into after
/// `auth.json` was removed. The id has then been merged into THAT account,
/// and carrying it into a login as someone else would merge the two people.
/// The id is claimed (see [`claim_anon_id`]) before it is returned, so this
/// is asked right before the device-start request, and only when a login
/// runs.
pub fn login_anon_id() -> Option<String> {
    // Only a machine that never signed in carries its id. A signed-out
    // record still names the account that was here, and a corrupt one may:
    // the next sign-in can be someone else, so neither counts as never.
    match auth::load() {
        auth::LoadedAuth::Absent => {}
        auth::LoadedAuth::Present(state) if !state.logged_in_ever => {}
        auth::LoadedAuth::Present(_)
        | auth::LoadedAuth::SignedOut { .. }
        | auth::LoadedAuth::Corrupt(_) => return None,
    }
    #[cfg(feature = "telemetry")]
    {
        if !matches!(anon_account(), AnonAccount::Unclaimed) {
            return None;
        }
    }
    if SENDING.load(Ordering::Relaxed) {
        // Consent is read again: `cerulion telemetry off` in another terminal
        // since this run started must keep the id out of the login.
        if !consent::status().enabled {
            return None;
        }
        // The file is user-editable: only a well-formed id leaves the machine.
        let anon_id = consent::anon_id()
            .ok()
            .flatten()
            .filter(|id| guard::check_anon_id(id).is_ok())?;
        #[cfg(feature = "telemetry")]
        {
            if !claim_anon_id() {
                return None;
            }
        }
        return Some(anon_id);
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

/// After a successful login: on an account switch, or when the anonymous id
/// was last used for another account, replace it so it is never attributed
/// to the previous account again, and record the new account as its
/// account; then, when this process sends, record `cli_login_completed`.
/// The id the device-start body carried (`outcome.telemetry_anon_id`, put
/// there by [`login_anon_id`]) is also merged into the account from here,
/// which covers an account service that ignores the field; a service that
/// already merged it makes this a repeat of the same merge.
pub fn login_completed(outcome: &LoginOutcome) {
    #[cfg(feature = "telemetry")]
    let carried = settle_login(outcome);
    #[cfg(not(feature = "telemetry"))]
    let carried = outcome.telemetry_anon_id.as_deref();
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
    consent::while_enabled(|| {
        if let Some(anon_id) = carried {
            client.alias(&sub, anon_id);
        }
        client.capture(
            CLI_LOGIN_COMPLETED,
            &sub,
            login_props(outcome.switched_account),
        );
    });
}

/// The on-disk part of [`login_completed`]: rotate the id when it is owed,
/// record the account, or mark the merge the notice run deferred. Returns
/// the carried id that is still to be merged into the account from here:
/// `None` when none was carried, or when another login bound the id to its
/// account while this one was in flight. That login rotated the id for its
/// account; the id this login carried is merged by the device-start request
/// alone, never a second time from here.
#[cfg(feature = "telemetry")]
fn settle_login(outcome: &LoginOutcome) -> Option<&str> {
    let mut carried = outcome.telemetry_anon_id.as_deref();
    // An opted-out login leaves the id, its account and the pending alias
    // as they were: the next run that may send finds the account changed
    // and rotates the id then, before anything is sent.
    if !consent::status().enabled {
        return None;
    }
    // The notice run defers the merge of an uncarried id, but only a hosted
    // account is ever merged: for any other account there is nothing owed,
    // and the id stays the first hosted login's to carry.
    let deferred =
        UNCARRIED.load(Ordering::Relaxed) && hosted_sub(&outcome.state.account_id).is_some();
    if consent::file_path().is_ok_and(|p| p.exists()) {
        let account = &outcome.state.account_id;
        // An id this login carried was claimed by [`login_anon_id`]: the
        // empty record is this login's own, not another account's. A login
        // that found this claim and could not rotate the id left it empty
        // too, and sent nothing under the id, so it is still no account's.
        let claimed_here = carried.is_some();
        let anothers = match anon_account() {
            AnonAccount::Unclaimed => false,
            AnonAccount::Bound(bound) if bound.is_empty() && claimed_here => false,
            AnonAccount::Bound(bound) => bound != *account,
            AnonAccount::Unknown => true,
        };
        if anothers {
            carried = None;
        }
        let owed = outcome.switched_account || anothers;
        // The account is recorded for the id only when the id is in play:
        // this run sends, carried it into the login, or left its merge
        // pending. A run that sends nothing (no key) records nothing, so
        // the id stays the first sending login's to carry.
        let in_play = SENDING.load(Ordering::Relaxed) || claimed_here || deferred;
        // An id that cannot be rotated must not keep sending: stop this
        // run's events rather than attribute them to the old account, and
        // record it as no account's. No account id is empty, so every later
        // run sees a mismatch and retries the rotation before it sends,
        // even where no account was recorded before.
        if owed && !rotate_existing_anon_id() {
            SENDING.store(false, Ordering::Relaxed);
            bind_anon_account("");
        } else if in_play && !bind_anon_account(account) {
            // The account is not on disk for this id: nothing more leaves
            // under it. A carried id keeps its claim, so no later login
            // carries it; the next run with an account settles it before
            // it sends.
            SENDING.store(false, Ordering::Relaxed);
        }
    }
    // The run that printed the notice sends nothing, so the merge waits
    // for the next run that may send (see `merge_pending_alias`).
    if deferred && !outcome.switched_account {
        if let Some(path) = pending_alias_path() {
            if let Err(e) = std::fs::write(path, b"") {
                tracing::warn!(error = %e, "telemetry alias marker not written");
            }
        }
    }
    carried
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
        // The decision is the input; whether anything leaves depends on a
        // key. Say so, or "on (default)" reads as "data is being sent" on a
        // source build that sends nothing.
        if status.enabled && !key_present() {
            writeln!(out, "no telemetry key in this build: nothing is sent").map_err(io)?;
        }
        if let Ok(path) = consent::file_path() {
            writeln!(out, "consent file: {}", path.display()).map_err(io)?;
        }
    }
    Ok(())
}

/// Whether [`Client::from_env_or_key`] has a key to send with:
/// `POSTHOG_API_KEY` or the baked key, set and not blank, the same rule the
/// client applies.
#[cfg(feature = "telemetry")]
fn key_present() -> bool {
    let present = |k: &str| !k.trim().is_empty();
    match std::env::var("POSTHOG_API_KEY") {
        Ok(key) if present(&key) => true,
        Err(std::env::VarError::NotUnicode(_)) => false,
        _ => BAKED_KEY.is_some_and(present),
    }
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
