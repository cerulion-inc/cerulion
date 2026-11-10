// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-remoted` — the robot-level remote-plane daemon binary.
//!
//! Thin clap wrapper: resolve the config (state-root path conventions, the relay
//! seam, the `--network off` kill-switch), then run the always-on accept loop
//! until Ctrl-C or SIGTERM, and stop gracefully within the hard-exit deadline
//! (`cerulion_remoted::shutdown`). All logic lives in the `cerulion_remoted`
//! library.
//!
//! Exit status: 0 after a graceful stop, also one that starts because the
//! accept loop stopped on its own, and when the network kill switch refuses to
//! serve; 1 on a configuration or serving error, or when the stop signals
//! cannot be installed; 2 on a command-line usage error; 3 when a participant
//! registered on the shutdown stage failed, panicked or was cancelled (the
//! teardown after it still ran; this binary registers none of its own); 4 when
//! the hard-exit deadline or a second stop signal ends a stop that has not
//! finished, as the first stop signal does after the accept loop stopped on
//! its own.
//!
//! Log lines go to stderr through the bounded queue of
//! [`cerulion_remoted::log_writer`]: a full stderr drops and counts them and
//! holds no thread that logs. Each exit waits at most [`EXIT_LOG_GRACE`] for
//! that queue.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use clap::Parser;
use tokio::sync::oneshot;

use cerulion_link::RelayConfig;
use cerulion_remoted::log_writer::{self, Limits, LogWriter};
use cerulion_remoted::shutdown::ShutdownStageError;
use cerulion_remoted::{resolve_network_disabled_loud, run, RemotedConfig, RemotedError};

/// The robot-level daemon that owns the ONE iroh endpoint and multiplexes the
/// wire + ops ALPNs, gating every accept through a deny-by-default authorizer.
#[derive(Parser, Debug)]
#[command(name = "cerulion-remoted", version, about, long_about = None)]
struct Cli {
    /// On-robot state root (default: cerud's `/var/lib/cerulion`). remoted's
    /// files live under `<root>/remoted/`.
    ///
    /// PREFER setting this via the CERULION_STATE_ROOT env: the LAN gateway (a
    /// SEPARATE process that publishes the mDNS beacon) resolves the state root
    /// from that env / the default to find remoted's beacon-facts file — it
    /// cannot see this flag. A non-default `--state-root` passed WITHOUT the
    /// matching env means the gateway won't find the facts, so the mDNS TXT
    /// enrichment (eid/iroh_port/claimable) is silently omitted (the beacon
    /// still advertises; direct-dial-by-key just falls back to discovery).
    #[arg(long, env = "CERULION_STATE_ROOT")]
    state_root: Option<PathBuf>,

    /// Internal registration writer used by the automatic bootstrap worker.
    #[arg(long, hide = true, conflicts_with_all = ["key_file", "store_file", "store_mac_key_file", "index_file"])]
    provision_bundle: Option<PathBuf>,

    /// Override the device-key file (32 raw bytes; public half = EndpointId).
    #[arg(long)]
    key_file: Option<PathBuf>,

    /// Override the trust-store file.
    #[arg(long)]
    store_file: Option<PathBuf>,

    /// Override the trust-store MAC-key file (skeleton seam — production reads
    /// this from firmware secure storage).
    #[arg(long)]
    store_mac_key_file: Option<PathBuf>,

    /// Override the device→account side-map file.
    #[arg(long)]
    index_file: Option<PathBuf>,

    // NOTE: there is deliberately NO `--beacon-facts-file` override, unlike the
    // four file flags above. Those are remoted-PRIVATE files (only remoted reads
    // them), so a per-file override is harmless. `beacon_facts_file` is a
    // CROSS-PROCESS HANDOFF file — the SEPARATE LAN gateway process also reads it,
    // resolving its path from CERULION_STATE_ROOT / the default (it cannot see any
    // remoted CLI flag). A standalone `--beacon-facts-file` only remoted knew
    // about would make remoted write to /custom while the gateway reads the
    // default, silently omitting the mDNS TXT enrichment with no error either
    // side. The ONLY correct redirect for this file is `--state-root` /
    // `CERULION_STATE_ROOT`, which BOTH processes honor — so `beacon_facts_file`
    // stays derived from the state root in `RemotedConfig::from_state_root` and is
    // never independently overridable.
    #[command(flatten)]
    rcp: cerulion_remoted::rcp::RcpFlags,

    /// Self-hosted relay URL (prod). Default: n0 public relays.
    #[arg(long, env = "CERULION_RELAY_URL")]
    relay_url: Option<String>,

    /// Disable all relays (LAN / direct-dial only) — for tests / air-gap.
    #[arg(long)]
    relay_disabled: bool,

    /// Kill-switch: pass `off` to refuse to serve (R8 parity with the gateway).
    /// `CERULION_NETWORK=off` also engages it (either mechanism can trip it).
    #[arg(long)]
    network: Option<String>,
}

impl Cli {
    fn into_config(self) -> Result<RemotedConfig, RemotedError> {
        // The kill-switch is the OR of the flag and the env (either trips it).
        // The loud variant WARNS on an unrecognized value (a mistyped `--network
        // of` must never be a silently-ineffective kill switch) while keeping the
        // safe network-ON default.
        let network_disabled = resolve_network_disabled_loud(
            self.network.as_deref(),
            std::env::var("CERULION_NETWORK").ok().as_deref(),
        );
        let relay = resolve_relay(self.relay_url.as_deref(), self.relay_disabled)?;
        let state_root = self
            .state_root
            .unwrap_or_else(|| PathBuf::from(cerud::constants::DEFAULT_STATE_ROOT));
        let mut config = RemotedConfig::from_state_root(&state_root, relay, network_disabled);
        if let Some(p) = self.key_file {
            config.key_file = p;
        }
        if let Some(p) = self.store_file {
            config.store_file = p;
        }
        if let Some(p) = self.store_mac_key_file {
            config.store_mac_key_file = p;
        }
        if let Some(p) = self.index_file {
            config.index_file = p;
        }
        self.rcp.apply(&mut config.rcp);
        // No `beacon_facts_file` override here on purpose — it is a cross-process
        // handoff file redirected only via --state-root/CERULION_STATE_ROOT (see
        // the field-absence note on the `Cli` struct); it stays derived from the
        // resolved state root by `RemotedConfig::from_state_root`.
        Ok(config)
    }
}

/// Resolve the relay posture: `--relay-disabled` wins; else a `--relay-url`
/// (or its `CERULION_RELAY_URL` env, already merged by clap) becomes a
/// self-hosted `Custom` relay; else the n0 public relays.
///
/// DELEGATES to the ONE shared `cerulion_link::RelayConfig::resolve` precedence
/// helper (also linked by the desk client) so the parse/precedence logic is
/// single-sourced. The env is passed as the `env_url` slot because clap's
/// `env = "CERULION_RELAY_URL"` on `--relay-url` already merged it (never
/// double-read); `remoted` has no config file, so `config_url` is `None`.
fn resolve_relay(
    relay_url: Option<&str>,
    relay_disabled: bool,
) -> Result<RelayConfig, RemotedError> {
    RelayConfig::resolve(relay_disabled, relay_url, None)
        .map_err(|e| RemotedError::Config(e.to_string()))
}

/// Install the subscriber: fmt over stderr, through the log writer's queue
/// ([`log_writer::start`]). When the writer thread cannot start, one error
/// line is written straight to stderr and the daemon exits 1, as it does when
/// the stop thread cannot start.
fn init_tracing() -> Result<LogWriter, ExitCode> {
    use tracing_subscriber::EnvFilter;
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    // Log to STDERR (a daemon's stdout stays clean); mirrors the workspace's
    // stdout↔stderr discipline (MEMORY: fmt::layer defaults to stdout).
    // A failed write is dropped: reporting it goes through `eprintln!`, which
    // panics when stderr cannot be written.
    match log_writer::start(queue_limits()) {
        Ok(log) => {
            tracing_subscriber::fmt()
                .with_env_filter(filter)
                .with_writer(log.clone())
                .log_internal_errors(false)
                .init();
            Ok(log)
        }
        Err(e) => {
            tracing_subscriber::fmt()
                .with_env_filter(filter)
                .with_writer(std::io::stderr)
                .log_internal_errors(false)
                .init();
            tracing::error!(error = %e, "cerulion_remoted: the log writer thread could not start");
            Err(ExitCode::FAILURE)
        }
    }
}

/// The log queue's bounds: [`Limits::DEFAULT`], or, in a build with the test
/// seam, the positive line bound its override variable holds.
fn queue_limits() -> Limits {
    #[cfg(feature = "test-seam")]
    {
        let lines = std::env::var(log_writer::QUEUE_LINES_FOR_TEST_ENV)
            .ok()
            .and_then(|value| value.trim().parse::<usize>().ok())
            .filter(|lines| *lines > 0);
        if let Some(lines) = lines {
            return Limits {
                lines,
                ..Limits::DEFAULT
            };
        }
    }
    Limits::DEFAULT
}

fn main() -> ExitCode {
    let log = match init_tracing() {
        Ok(log) => log,
        Err(status) => return status,
    };
    let status = match tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
    {
        // The runtime is dropped before the drain below, so the lines its
        // tasks write as they are dropped are queued by then.
        Ok(runtime) => runtime.block_on(serve(log.clone())),
        Err(e) => {
            tracing::error!(error = %e, "cerulion_remoted: the runtime could not start");
            ExitCode::FAILURE
        }
    };
    let _ = log.drain(exit_log_grace());
    status
}

/// Resolve the configuration and serve until a stop.
async fn serve(log: LogWriter) -> ExitCode {
    // Apply `IOX2_LOG_LEVEL` to THIS binary's `iceoryx2-log` static
    // (a per-linked-copy static — see `init_iceoryx_log_level`), before the SHM
    // tap path can touch iceoryx2.
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
    let mut cli = Cli::parse();
    if let Some(registration_bundle) = cli.provision_bundle.take() {
        let state_root = cli
            .state_root
            .take()
            .unwrap_or_else(|| PathBuf::from(cerud::constants::DEFAULT_STATE_ROOT));
        return provision_registered_robot(state_root, registration_bundle);
    }
    let config = match cli.into_config() {
        Ok(c) => c,
        Err(e) => {
            tracing::error!(error = %e, "cerulion_remoted: configuration error");
            return ExitCode::FAILURE;
        }
    };
    let deadline = cerulion_remoted::shutdown::hard_exit_from_env();
    let (requested, serving_ended) = match start_stop_thread(deadline, log) {
        Ok(handles) => handles,
        Err(e) => {
            tracing::error!(error = %e, "cerulion_remoted: the stop signals could not be installed");
            return ExitCode::FAILURE;
        }
    };
    // Resolves on the first stop signal, which the stop thread sends after it
    // armed the deadline, or when the stop thread is gone. A full stderr can
    // hold this line, not the deadline. Serving drops this future when its
    // accept loop stops or when it never starts; `ended` then tells the stop
    // thread, which arms the deadline unless a stop signal already did.
    let deadline_ms = u64::try_from(deadline.as_millis()).unwrap_or(u64::MAX);
    let ended = SendOnDrop(Some(serving_ended));
    let served = run(config, async move {
        let _ended = ended;
        if let Ok(signal) = requested.await {
            tracing::info!(
                signal,
                deadline_ms,
                "cerulion_remoted: stop signal received; stopping gracefully"
            );
        }
    })
    .await;
    match &served {
        Ok(()) => {}
        Err(RemotedError::Shutdown(error)) => log_shutdown_failures(error),
        Err(e) => tracing::error!(error = %e, "cerulion_remoted: failed"),
    }
    ExitCode::from(exit_status(&served))
}

/// Sends on its channel when dropped.
struct SendOnDrop(Option<oneshot::Sender<()>>);

impl Drop for SendOnDrop {
    fn drop(&mut self) {
        if let Some(sender) = self.0.take() {
            let _ = sender.send(());
        }
    }
}

/// The exit status when a participant of the shutdown stage did not complete.
const EXIT_SHUTDOWN_INCOMPLETE: u8 = 3;

/// The exit status of serving that returned `served`.
fn exit_status(served: &Result<(), RemotedError>) -> u8 {
    match served {
        Ok(()) => 0,
        Err(RemotedError::Shutdown(_)) => EXIT_SHUTDOWN_INCOMPLETE,
        Err(_) => 1,
    }
}

/// One error line per participant of the shutdown stage that did not complete.
fn log_shutdown_failures(error: &ShutdownStageError) {
    for failure in &error.failures {
        tracing::error!(
            participant = failure.participant,
            kind = %failure.kind,
            "cerulion_remoted: a shutdown participant did not complete"
        );
    }
}

/// The exit status when the hard-exit deadline or a second stop signal ends a
/// stop that has not finished.
const EXIT_STOP_UNFINISHED: i32 = 4;

/// The installed SIGINT and SIGTERM handlers (Ctrl-C alone off unix).
struct StopSignals {
    #[cfg(unix)]
    interrupt: tokio::signal::unix::Signal,
    #[cfg(unix)]
    terminate: tokio::signal::unix::Signal,
}

impl StopSignals {
    /// Install the handlers; call within a runtime.
    fn install() -> std::io::Result<Self> {
        #[cfg(unix)]
        {
            use tokio::signal::unix::{signal, SignalKind};
            Ok(Self {
                interrupt: signal(SignalKind::interrupt())?,
                terminate: signal(SignalKind::terminate())?,
            })
        }
        #[cfg(not(unix))]
        {
            Ok(Self {})
        }
    }

    /// The name of the next stop signal, or `None` once the signal driver is
    /// gone.
    async fn next(&mut self) -> Option<&'static str> {
        #[cfg(unix)]
        {
            tokio::select! {
                Some(()) = self.interrupt.recv() => Some("SIGINT"),
                Some(()) = self.terminate.recv() => Some("SIGTERM"),
                else => None,
            }
        }
        #[cfg(not(unix))]
        {
            tokio::signal::ctrl_c().await.ok().map(|()| "SIGINT")
        }
    }
}

/// Start the stop thread, which reads the stop signals on a runtime of its own,
/// so they still answer while the serving runtime tears down, and wait until it
/// has installed them. Returns the receiver the first stop signal resolves,
/// with the signal's name, and the sender that tells the thread serving has
/// ended.
fn start_stop_thread(
    deadline: Duration,
    log: LogWriter,
) -> std::io::Result<(oneshot::Receiver<&'static str>, oneshot::Sender<()>)> {
    let (request, requested) = oneshot::channel();
    let (serving_ended, ended) = oneshot::channel();
    let (ready, installed) = std::sync::mpsc::channel();
    let _detached = std::thread::Builder::new()
        .name("remoted-stop".into())
        .spawn(move || {
            let setup = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .and_then(|runtime| {
                    let signals = runtime.block_on(async { StopSignals::install() })?;
                    Ok((runtime, signals))
                });
            match setup {
                Ok((runtime, signals)) => {
                    let _ = ready.send(Ok(()));
                    runtime.block_on(stop_sequence(signals, deadline, request, ended, log));
                }
                Err(e) => {
                    let _ = ready.send(Err(e));
                }
            }
        })?;
    // Returns once the thread has sent its setup result or ended.
    installed
        .recv()
        .unwrap_or_else(|_| Err(std::io::Error::other("the stop thread ended during setup")))?;
    Ok((requested, serving_ended))
}

/// On the first stop signal, arm the deadline, then send serving the signal's
/// name, which serving logs. Without one, arm it when serving drops its
/// shutdown future. From then a stop signal ends the process, after at most
/// [`exit_log_grace`] for the log queue, and the deadline ends it if it is
/// still running. Its two lines are queued for the log writer, so a full
/// stderr can delay a line, never the stop request or the exit.
async fn stop_sequence(
    mut signals: StopSignals,
    deadline: Duration,
    request: oneshot::Sender<&'static str>,
    ended: oneshot::Receiver<()>,
    log: LogWriter,
) {
    let deadline_ms = u64::try_from(deadline.as_millis()).unwrap_or(u64::MAX);
    let expired = tokio::select! {
        // `ended` first: a signal ready at the same time stays queued for the select below.
        biased;
        _ = ended => tokio::time::sleep(deadline),
        first = signals.next() => {
            // `next` yields None only once both signal streams end, which
            // the signal driver of this runtime never does while it runs.
            let Some(first) = first else { return };
            let expired = tokio::time::sleep(deadline);
            let _ = request.send(first);
            expired
        }
    };
    tokio::pin!(expired);
    tokio::select! {
        Some(second) = signals.next() => tracing::warn!(
            signal = second,
            status = EXIT_STOP_UNFINISHED,
            "cerulion_remoted: a second stop signal during the stop; exiting now"
        ),
        () = &mut expired => tracing::error!(
            deadline_ms,
            status = EXIT_STOP_UNFINISHED,
            "cerulion_remoted: the stop did not finish within the hard-exit deadline; exiting now"
        ),
    }
    // A blocking wait on this thread's own runtime, which has nothing else
    // left to run.
    let _ = log.drain(exit_log_grace());
    // `_exit` ends the process at once: it runs no atexit handler, flushes no
    // stdio buffer and runs no destructor, so no exit-time cleanup can block it.
    // SAFETY: `_exit` takes a status and terminates the process; it touches no
    // memory of this program.
    unsafe { libc::_exit(EXIT_STOP_UNFINISHED) }
}

/// How long an exit waits for the log writer to write the lines still queued:
/// 100 ms, after the serving runtime is dropped on the return from `main`, and
/// before `_exit` in the stop thread ([`LogWriter::drain`]). The exit goes on
/// as soon as no line is queued and none is being written. Lines still queued
/// at four fifths of it are discarded, and one error line names how many; a
/// full stderr delays the exit by no more than this. Not tunable: the
/// hard-exit deadline is the operator's knob, and this only gives a free
/// stderr time for the last lines. Only a build with the test seam reads an
/// override, from `CERULION_REMOTED_EXIT_LOG_GRACE_MS_FOR_TEST`.
const EXIT_LOG_GRACE: Duration = Duration::from_millis(100);

/// The exit's wait for the log queue: [`EXIT_LOG_GRACE`], or, in a build with
/// the test seam, the positive whole number of milliseconds its override
/// variable holds.
fn exit_log_grace() -> Duration {
    #[cfg(feature = "test-seam")]
    {
        let ms = std::env::var(cerulion_remoted::shutdown::EXIT_LOG_GRACE_FOR_TEST_ENV)
            .ok()
            .and_then(|value| value.trim().parse::<u64>().ok())
            .filter(|ms| *ms > 0);
        if let Some(ms) = ms {
            return Duration::from_millis(ms);
        }
    }
    EXIT_LOG_GRACE
}

fn provision_registered_robot(state_root: PathBuf, registration_bundle: PathBuf) -> ExitCode {
    use cerulion_remoted::provision::{provision, ProvisionConfig};
    use std::io::Write;

    let config = ProvisionConfig {
        state_root,
        registration_bundle,
    };
    let now_ns = cerulion_remoted::RemotedClock::wall().now_ns();
    match provision(&config, now_ns) {
        Ok(registered) => {
            let output = serde_json::json!({
                "version": 1,
                "robot_id": hex::encode(registered.robot_id.0),
                "endpoint_id": hex::encode(registered.endpoint_id.0),
                "owner_account": hex::encode(registered.owner_account.0),
            });
            let mut stdout = std::io::stdout().lock();
            if serde_json::to_writer(&mut stdout, &output).is_err()
                || stdout.write_all(b"\n").is_err()
                || stdout.flush().is_err()
            {
                tracing::error!(
                    "registered robot state is durable, but the result could not be written"
                );
                return ExitCode::FAILURE;
            }
            ExitCode::SUCCESS
        }
        Err(error) => {
            tracing::error!(error = %error, "automatic robot provisioning failed");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn automatic_writer_argument_is_hidden_and_parses_as_a_path() {
        let help = Cli::command().render_long_help().to_string();
        assert!(!help.contains("--provision-bundle"));
        let cli = Cli::try_parse_from([
            "cerulion-remoted",
            "--state-root",
            "/state",
            "--provision-bundle",
            "/state/registration.json",
        ])
        .unwrap();
        assert_eq!(
            cli.provision_bundle,
            Some(PathBuf::from("/state/registration.json"))
        );
    }

    #[test]
    fn dropping_the_shutdown_future_tells_the_stop_thread_serving_ended() {
        let (sender, mut ended) = oneshot::channel();
        let guard = SendOnDrop(Some(sender));
        assert!(ended.try_recv().is_err(), "nothing is sent while it lives");
        drop(guard);
        assert_eq!(ended.try_recv(), Ok(()));
    }

    #[test]
    fn an_incomplete_shutdown_stage_exits_3_and_other_errors_exit_1() {
        use cerulion_remoted::shutdown::{ShutdownFailure, ShutdownFailureKind};
        let incomplete = ShutdownStageError {
            failures: vec![ShutdownFailure {
                participant: "a",
                kind: ShutdownFailureKind::Panicked,
            }],
        };
        assert_eq!(exit_status(&Ok(())), 0);
        assert_eq!(exit_status(&Err(RemotedError::Shutdown(incomplete))), 3);
        assert_eq!(exit_status(&Err(RemotedError::Config("x".into()))), 1);
    }

    #[test]
    fn automatic_writer_cannot_silently_ignore_individual_state_overrides() {
        for argument in [
            "--key-file",
            "--store-file",
            "--store-mac-key-file",
            "--index-file",
        ] {
            assert!(Cli::try_parse_from([
                "cerulion-remoted",
                "--provision-bundle",
                "/state/registration.json",
                argument,
                "/different-state",
            ])
            .is_err());
        }
    }
}
