// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-connectd` — the desk-side iroh sibling binary.
//!
//! Two subcommands, ONE iroh-linking sibling (the `cerulion` CLI stays iroh-free
//! and SPAWNS this, exactly as it spawns the gateway / `vizd`):
//!
//! - `cerulion-connectd connect …` — dial a robot's `cerulion/wire/1` ALPN, demand
//!   topics, and re-inject each into desk-local SHM ("remote = local"). The catalog
//!   prints to STDOUT; structured logs go to STDERR.
//! - `cerulion-connectd pair …` — run the CPace **code-pairing** ceremony over the
//!   robot's `cerulion/ops/1` ALPN so the robot trusts this desk's device key.
//!   Deterministic STDOUT state lines (`pairing:` / `paired:`); a distinct exit
//!   code per outcome (see [`cerulion_connectd::pair`]).
//!
//! Both subcommands are spawned by `cerulion connect` / `cerulion pair`; users run
//! those verbs, not this binary directly. All logic lives in the library.

use std::io::{IsTerminal, Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;

use clap::{Parser, Subcommand};

use cerulion_connectd::config::{
    parse_account, parse_addrs, parse_eid, resolve_desk_seed, resolve_or_create_desk_seed,
    ConnectConfig, DemandSet, DEFAULT_ROBOT_TIMEOUT,
};
use cerulion_connectd::pair::{run_pair, PairConfig, EXIT_INTERRUPTED, EXIT_USAGE};
use cerulion_connectd::resolve_epoch_dir_from_env;
use cerulion_connectd::sanitize_peer_text;
use cerulion_connectd::worker::run_connect;
use cerulion_connectd::ConnectError;
// The operator-loudness class of an epoch-push outcome — three
// levels, so the healthy "this desk holds nothing" state is never rendered as a failure.
use cerulion_connectd::EpochPushSeverity;
use cerulion_core::transport::cerulion_q::CatalogReply;
use cerulion_core::{TransportConfig, TransportManager};
use cerulion_link::{RelayConfig, RelayUrl};

/// The desk-side iroh sibling `cerulion connect` / `cerulion pair` spawn.
//
// `about` is spelled out here. A bare `about` takes the crate's manifest
// description, which is written for crates.io, not for a terminal.
#[derive(Parser, Debug)]
#[command(
    name = "cerulion-connectd",
    version,
    about = "The helper that `cerulion connect` and `cerulion pair` run to reach a remote robot. Use those verbs; you do not normally run this binary yourself.",
    long_about = None
)]
struct Cli {
    #[command(subcommand)]
    command: ConnectdCommand,
}

#[derive(Subcommand, Debug)]
enum ConnectdCommand {
    /// Dial a robot and mirror its topics into this machine's shared memory, so
    /// `cerulion topic echo` and `cerulion viz` see them as local topics.
    Connect(ConnectCli),
    /// Pair this desk with a robot.
    ///
    /// Runs the code-pairing ceremony, so the robot adds this desk's device key
    /// to its access list. Afterwards `cerulion connect <robot>` is admitted
    /// with no flags.
    Pair(PairCli),
}

/// `cerulion-connectd connect`: the desk-side re-inject client flags.
#[derive(Parser, Debug)]
struct ConnectCli {
    /// The robot's iroh endpoint id (64-char hex: the public half of its
    /// 32-byte device key). Obtain it from `cerulion pair`, the robot's mDNS
    /// TXT `eid=` record, or the robot operator.
    #[arg(long, value_name = "HEX")]
    eid: String,

    /// The name THIS DESK knows the robot by (the `~/.cerulion/robots.toml`
    /// name the user dialed, or a reverse lookup of `--eid` in that file).
    /// `cerulion connect` fills it in; it selects the desk's cached revocation
    /// record for the robot (`<name>.epoch`). When omitted, this desk has no
    /// verified name for the robot, so NO revocation record is pushed (the
    /// robot's own reported identity is never used to select a cache file).
    #[arg(long = "robot-name", value_name = "NAME")]
    robot_name: Option<String>,

    /// A direct `ip:port` socket address for the robot (LAN direct-dial).
    /// Repeatable. When omitted, the eid is resolved via relay/discovery.
    #[arg(long = "addr", value_name = "IP:PORT")]
    addrs: Vec<String>,

    /// A topic to demand + re-inject, e.g. `/utlidar/cloud`. Repeatable. With no
    /// `--topic` and no `--all`, the catalog is fetched + printed and NOTHING is
    /// demanded (the discoverable default).
    #[arg(long = "topic", value_name = "TOPIC")]
    topics: Vec<String>,

    /// Demand + re-inject EVERY topic the robot's catalog lists.
    #[arg(long)]
    all: bool,

    /// The desk device key file (32 raw bytes: an ed25519 seed whose public
    /// half is the desk identity the robot pairs with). When omitted, an
    /// EPHEMERAL key is used, which an unpaired robot REFUSES: pair the desk
    /// first with `cerulion pair`.
    #[arg(long, value_name = "PATH")]
    key_file: Option<PathBuf>,

    /// The desk schema store directory to materialize fetched `.msg`/YAML schemas
    /// into (so `topic echo` / `viz` can decode a never-seen type). When omitted,
    /// schema materialization is skipped.
    #[arg(long, value_name = "DIR")]
    schemas_dir: Option<PathBuf>,

    /// Self-hosted relay URL. Default: n0 public relays.
    #[arg(long, env = "CERULION_RELAY_URL")]
    relay_url: Option<String>,

    /// Disable all relays (LAN or direct-dial only), for tests and air-gapped
    /// networks.
    #[arg(long)]
    relay_disabled: bool,

    /// Kill-switch: pass `off` to refuse to connect. `CERULION_NETWORK=off`
    /// also engages it.
    #[arg(long)]
    network: Option<String>,
}

/// `cerulion-connectd pair`: the desk-side code-pairing ceremony flags.
///
/// # Exit codes (the machine signal a Studio driver branches on)
///
/// - `0` paired, `1` usage or config error, `2` robot refused (no armed code,
///   denied, or the window expired), `3` unreachable (dial failure or timeout),
///   `4` wrong pairing code or attempts exhausted, `130` interrupted (Ctrl-C or
///   SIGTERM during the ceremony; no pairing occurred).
#[derive(Parser)]
struct PairCli {
    /// The robot's iroh endpoint id (64-char hex: the public half of its
    /// 32-byte device key). `cerulion pair` resolves it from the robot name you
    /// typed (mDNS `eid=`, else `robots.toml`) or takes it from `--eid`.
    #[arg(long, value_name = "HEX")]
    eid: String,

    /// A direct `ip:port` for the robot (LAN direct-dial). Repeatable.
    #[arg(long = "addr", value_name = "IP:PORT")]
    addrs: Vec<String>,

    /// The desk device key file (32 raw bytes: an ed25519 seed). REQUIRED for
    /// pairing (an ephemeral key the robot added to its access list would
    /// vanish on exit). `cerulion pair` creates and passes
    /// `~/.cerulion/desk.key`.
    #[arg(long, value_name = "PATH")]
    key_file: Option<PathBuf>,

    /// The short pairing code the robot owner armed. PREFER omitting this: the
    /// code is then read from stdin (a prompt on a terminal, or a piped line),
    /// which keeps it OFF the process arguments. A code passed via `--code` is
    /// visible in process listings (`ps`) for the ceremony window.
    #[arg(long, value_name = "CODE")]
    code: Option<String>,

    /// The account (64-char hex) this pairing is FOR. When omitted, a self-account
    /// is derived from the desk device key (a desk with a keypair and no
    /// Cerulion account).
    #[arg(long, value_name = "HEX")]
    account: Option<String>,

    /// The human label the robot stores on its access-list row (so the owner
    /// recognizes this desk). `cerulion pair` defaults it to the desk hostname.
    #[arg(long, value_name = "NAME")]
    label: Option<String>,

    /// A friendly robot name for the STDOUT state lines (the name the user typed).
    /// When omitted, the eid hex is shown.
    #[arg(long = "robot-name", value_name = "NAME")]
    robot_name: Option<String>,

    /// Self-hosted relay URL. Default: n0 public relays.
    #[arg(long, env = "CERULION_RELAY_URL")]
    relay_url: Option<String>,

    /// Disable all relays (LAN or direct-dial only), for tests and air-gapped
    /// networks.
    #[arg(long)]
    relay_disabled: bool,
}

impl std::fmt::Debug for PairCli {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("PairCli { code: [REDACTED] }")
    }
}

impl ConnectCli {
    fn into_config(self) -> Result<ConnectConfig, ConnectError> {
        let robot_eid = parse_eid(&self.eid)?;
        let direct_addrs = parse_addrs(&self.addrs)?;
        let desk_seed = resolve_desk_seed(self.key_file.as_deref())?;
        let relay = resolve_relay(self.relay_url.as_deref(), self.relay_disabled)?;
        // The desk's revocation-epoch cache directory, resolved through the
        // SHARED substrate `cerulion-netd`'s WAN plane calls — ONE env name
        // (`CERULION_EPOCH_DIR`), ONE fallback (the `epochs/` dir NEXT TO the desk key
        // file). Two resolvers would mean a relocated cache is visible to exactly one
        // desk path, silently.
        //
        // The push itself is UNCONDITIONAL (by design): there is no flag to
        // withhold a revocation, and an ephemeral key simply has no cache to read.
        let epoch_dir = resolve_epoch_dir_from_env(self.key_file.as_deref());
        Ok(ConnectConfig {
            robot_eid,
            direct_addrs,
            demand: DemandSet::from_flags(self.topics, self.all),
            desk_seed,
            relay,
            schemas_dir: self.schemas_dir,
            robot_timeout: DEFAULT_ROBOT_TIMEOUT,
            epoch_dir,
            // The DESK's name for this robot, never the
            // robot's own reported identity — the dialed peer must not get to choose
            // which cached epoch artifact it is handed.
            robot_name: self
                .robot_name
                .map(|n| n.trim().to_string())
                .filter(|n| !n.is_empty()),
        })
    }
}

/// Resolve the relay posture: `--relay-disabled` wins; else `--relay-url` becomes
/// a self-hosted `Custom` relay; else the n0 public relays.
fn resolve_relay(
    relay_url: Option<&str>,
    relay_disabled: bool,
) -> Result<RelayConfig, ConnectError> {
    if relay_disabled {
        return Ok(RelayConfig::Disabled);
    }
    match relay_url {
        None => Ok(RelayConfig::N0Default),
        Some(u) => {
            let url: RelayUrl = u.parse().map_err(|e| {
                ConnectError::DeskKey(format!("--relay-url '{u}' is not a valid relay URL: {e}"))
            })?;
            Ok(RelayConfig::Custom(vec![url]))
        }
    }
}

/// Whether the `off` kill-switch is engaged (either the flag or the env). Warns
/// on an unrecognized non-`off` value (a mistyped `--network of` must never be a
/// silently-ineffective switch).
fn network_off(flag: Option<&str>, env: Option<&str>) -> bool {
    for (src, val) in [("--network", flag), ("CERULION_NETWORK", env)] {
        match val {
            Some("off") => return true,
            Some(other) if !other.is_empty() => {
                tracing::warn!(
                    source = src,
                    value = other,
                    "cerulion connect: unrecognized network value (only `off` is a kill-switch); \
                     ignoring — connecting"
                );
            }
            _ => {}
        }
    }
    false
}

fn init_tracing() {
    use tracing_subscriber::EnvFilter;
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    // Log to STDERR so STDOUT stays clean for the user-facing catalog / state
    // lines; mirrors the workspace's stdout↔stderr discipline.
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();
}

/// Build the robot's catalog listing, one rendered line per element.
///
/// EVERY peer-chosen string here — the robot's self-reported name AND each topic name —
/// is rendered through the shared sanitizer. This is the most exposed surface of the
/// three: it writes straight to an interactive STDOUT, where a raw ESC sequence in a
/// topic name would repaint the screen and a raw newline would forge extra catalog rows.
/// Same policy and bound as `ConnectSummary::robot_display`.
///
/// PURE (returns the lines instead of printing them) so the sanitization is
/// oracle-testable — see `tests::catalog_lines_neuter_and_bound_every_peer_string`.
/// [`print_catalog`] is the thin I/O half.
fn catalog_lines(catalog: &CatalogReply) -> Vec<String> {
    let mut lines = Vec::with_capacity(catalog.entries.len() + 2);
    lines.push(format!("robot: {}", sanitize_peer_text(&catalog.robot)));
    lines.push(format!("TOPICS ({}):", catalog.entries.len()));
    for entry in &catalog.entries {
        let hash = match entry.schema_hash {
            Some(h) => format!("{h:#018x}"),
            None => "—".to_string(),
        };
        lines.push(format!(
            "  {}  schema_hash={hash}",
            sanitize_peer_text(&entry.topic)
        ));
    }
    lines
}

/// Print the robot's catalog to STDOUT (user-facing). Rendering lives in the pure
/// [`catalog_lines`]; this is the I/O half.
fn print_catalog(catalog: &CatalogReply) {
    for line in catalog_lines(catalog) {
        println!("{line}");
    }
}

/// The desk transport: a `network:None` `TransportManager` (opens NO zenoh
/// session; re-injection is local SHM only).
fn desk_transport() -> Result<Arc<TransportManager>, ConnectError> {
    TransportManager::init(TransportConfig::default()).map_err(|e| ConnectError::Ingress {
        topic: "<desk transport>".to_string(),
        reason: e.to_string(),
    })
}

/// Ctrl-C OR stdin EOF resolves the shutdown (so a parent that closes our stdin —
/// the Studio subprocess teardown — stops us cleanly too).
async fn shutdown_signal() {
    let ctrl_c = async {
        let _ = tokio::signal::ctrl_c().await;
        tracing::info!("cerulion connect: Ctrl-C received");
    };
    // Only watch stdin-EOF when stdin is NOT a terminal (a piped parent); on an
    // interactive TTY, reading stdin would swallow keystrokes.
    let stdin_eof = async {
        if std::io::stdin().is_terminal() {
            std::future::pending::<()>().await;
        } else {
            let _ = tokio::task::spawn_blocking(|| {
                let mut buf = [0u8; 64];
                // Read until EOF (0) or error; the parent closing our stdin ends it.
                while let Ok(n) = std::io::stdin().read(&mut buf) {
                    if n == 0 {
                        break;
                    }
                }
            })
            .await;
            tracing::info!("cerulion connect: stdin closed (parent went away)");
        }
    };
    tokio::select! {
        _ = ctrl_c => {}
        _ = stdin_eof => {}
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    // Apply `IOX2_LOG_LEVEL` to THIS binary's `iceoryx2-log` static
    // (a per-linked-copy static — see `init_iceoryx_log_level`), before the
    // re-inject path can touch iceoryx2.
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
    let cli = Cli::parse();
    match cli.command {
        ConnectdCommand::Connect(c) => run_connect_command(c).await,
        ConnectdCommand::Pair(p) => run_pair_command(p).await,
    }
}

/// The `connect` subcommand: dial the wire plane + re-inject until Ctrl-C / the
/// robot disconnects. Exit SUCCESS/FAILURE (the re-inject client has no richer
/// outcome contract).
async fn run_connect_command(cli: ConnectCli) -> ExitCode {
    if network_off(
        cli.network.as_deref(),
        std::env::var("CERULION_NETWORK").ok().as_deref(),
    ) {
        tracing::warn!(
            "cerulion connect: network kill-switch engaged (--network off / CERULION_NETWORK=off) \
             — refusing to connect; exiting cleanly"
        );
        return ExitCode::SUCCESS;
    }

    let config = match cli.into_config() {
        Ok(c) => c,
        Err(e) => {
            tracing::error!(error = %e, "cerulion connect: configuration error");
            return ExitCode::FAILURE;
        }
    };

    let manager = match desk_transport() {
        Ok(m) => m,
        Err(e) => {
            tracing::error!(error = %e, "cerulion connect: could not init desk transport");
            return ExitCode::FAILURE;
        }
    };

    match run_connect(config, manager, print_catalog, shutdown_signal()).await {
        Ok(summary) => {
            // SURFACE the session's revocation-epoch outcome (Principle #3 —
            // the observable exists so an operator learns a revocation did not land
            // WITHOUT grepping the dial's logs).
            //
            // THREE levels, not two: a desk that holds NOTHING for this
            // robot — never synced, or a guest desk (the account service's access
            // endpoint is owner-only) — has no revocation it failed to carry, so warning
            // about it on EVERY dial would be a permanent false alarm that trains
            // operators to ignore the real one. `warn` is reserved for a revocation this
            // desk was (or may have been) holding that did NOT reach the robot.
            //
            // The robot name is peer-controlled text, so it is rendered through
            // `robot_display()` (sanitized + bounded) — never raw.
            match summary.epoch_push.severity() {
                EpochPushSeverity::Current => tracing::info!(
                    robot = %summary.robot_display(),
                    outcome = %summary.epoch_push,
                    "cerulion connect: revocation epoch"
                ),
                EpochPushSeverity::NothingToCarry => tracing::debug!(
                    robot = %summary.robot_display(),
                    outcome = %summary.epoch_push,
                    "cerulion connect: revocation epoch — nothing to carry"
                ),
                EpochPushSeverity::NotCarried => tracing::warn!(
                    robot = %summary.robot_display(),
                    outcome = %summary.epoch_push,
                    "cerulion connect: revocation epoch NOT confirmed on the robot"
                ),
            }
            // `row.topic` is peer-chosen too: under `--all` these are exactly the names
            // the robot listed in its catalog (`resolve_demand_topics` clones them
            // verbatim), so the teardown summary gets the same sanitizer as every other
            // topic render.
            for row in &summary.per_topic {
                tracing::info!(
                    topic = %sanitize_peer_text(&row.topic),
                    reinjected = row.reinjected,
                    rejected = row.rejected,
                    reinject_failed = row.reinject_failed,
                    "cerulion connect: topic teardown"
                );
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            // Peer text inside a `ConnectError` is neutered + bounded WHERE IT ENTERS —
            // `worker::classify_catalog_reply` (the refusal `reason`, the
            // `Error { message }` behind `Control`, the `{:?}` blob behind `Protocol`),
            // `worker::classify_demand_reply` (the `Protocol` detail), and
            // `schema::{validate_qualified,dest_path,materialize_one}` (the
            // robot-controlled `qualified` name behind `Schema`). That is the primary
            // defense and where new peer-text sources must keep sanitizing.
            //
            // These two arms are nevertheless the ONE surface that renders EVERY
            // `ConnectError` variant, including variants no classifier covers today and
            // variants a future change adds — so they sanitize AGAIN as cheap
            // defense-in-depth. (The `pair` verb has its OWN such surface below;
            // this comment scopes `connect` only.)
            // That is SAFE and DELIBERATE: `sanitize_peer_text` is
            // IDEMPOTENT (`f(f(s)) == f(s)`; a second application truncates at the same char
            // boundary and re-appends the identical marker, and none of the characters
            // it emits — `U+FFFD`, `…(truncated)` — fall in the neutered C0/DEL/C1
            // classes). That property is pinned by
            // `cerulion_wireclient::epoch::tests::peer_text_sanitizer_is_idempotent`, so
            // it cannot silently rot underneath this layering.
            //
            // Cost note: the bound applies to the WHOLE rendered error here, so a
            // (hypothetical) >512-char structural message would be clipped with an
            // explicit marker rather than silently — the price of "no raw peer
            // byte can reach an operator's terminal through this arm".
            match &e {
                ConnectError::Refused(reason) => tracing::error!(
                    reason = %sanitize_peer_text(reason),
                    "cerulion connect: the robot refused the wire plane — pair the desk with the \
                     robot first (`cerulion pair <robot>`)"
                ),
                other => tracing::error!(
                    error = %sanitize_peer_text(&other.to_string()),
                    "cerulion connect: failed"
                ),
            }
            ExitCode::FAILURE
        }
    }
}

/// The `pair` subcommand: run the CPace code-pairing ceremony. Maps the ceremony
/// outcome to the documented 0–4 exit-code contract.
async fn run_pair_command(cli: PairCli) -> ExitCode {
    // Resolve every input up front; any config error is exit USAGE.
    let robot_eid = match parse_eid(&cli.eid) {
        Ok(e) => e,
        Err(e) => return usage_error(e),
    };
    let direct_addrs = match parse_addrs(&cli.addrs) {
        Ok(a) => a,
        Err(e) => return usage_error(e),
    };
    // Pairing REQUIRES a persistent desk key: an ephemeral key the robot
    // access-lists would vanish the moment this process exits.
    let Some(key_file) = cli.key_file.as_deref() else {
        tracing::error!(
            "cerulion pair: --key-file is required (a persistent desk device key). Run \
             `cerulion pair <robot>`, which creates + passes ~/.cerulion/desk.key."
        );
        return ExitCode::from(EXIT_USAGE);
    };
    // Create-if-absent: the persistent desk key (0600) is written here if the
    // path does not exist, and NEVER overwritten if it does.
    let desk_seed = match resolve_or_create_desk_seed(key_file) {
        Ok(s) => s,
        Err(e) => return usage_error(e),
    };
    let account = match parse_account(cli.account.as_deref()) {
        Ok(a) => a,
        Err(e) => return usage_error(e),
    };
    let relay = match resolve_relay(cli.relay_url.as_deref(), cli.relay_disabled) {
        Ok(r) => r,
        Err(e) => return usage_error(e),
    };
    let label = cli.label.unwrap_or_else(default_pair_label);
    let robot_display = cli
        .robot_name
        .unwrap_or_else(|| hex::encode(robot_eid.as_bytes()));
    let cli_code = cli.code;

    // The code read + the ceremony, BOTH cancellable by a Ctrl-C. The (blocking)
    // stdin code read runs on a blocking task so a Ctrl-C AT the prompt is caught
    // by the select below (a deliberate EXIT_INTERRUPTED) instead of killing us by
    // default disposition — a keystroke must never surface as a
    // synthesized clean "paired" 0 in the parent (`exit_code_of` would otherwise
    // see signal death and `spawn_and_pair` would falsely pin robots.toml).
    let ceremony = async move {
        let code = match tokio::task::spawn_blocking(move || resolve_pair_code(cli_code)).await {
            Ok(Ok(c)) => c,
            Ok(Err(msg)) => {
                tracing::error!("cerulion pair: {msg}");
                return ExitCode::from(EXIT_USAGE);
            }
            Err(e) => {
                tracing::error!("cerulion pair: reading the pairing code failed: {e}");
                return ExitCode::from(EXIT_USAGE);
            }
        };
        let config = PairConfig {
            robot_eid,
            direct_addrs,
            desk_seed,
            relay,
            code,
            account,
            label,
            robot_display,
            timeout: DEFAULT_ROBOT_TIMEOUT,
        };
        match run_pair(config).await {
            Ok(_outcome) => ExitCode::SUCCESS,
            Err(e) => {
                // Every `PairError` variant renders HERE, and the
                // Refused / CodeMismatch ones carry text the ROBOT chose — at the moment
                // the desk trusts the robot LEAST. Peer text is neutered + bounded where
                // it ENTERS (`pair::classify_call_error` / `classify_transport_error` /
                // `parse_finish_reply` / `decode_hex_field` / `classify_finish_error`);
                // this arm sanitizes AGAIN as cheap defense-in-depth, exactly like the
                // `connect` arms above and under the same idempotence + cost notes.
                tracing::error!(
                    error = %sanitize_peer_text(&e.to_string()),
                    "cerulion pair: pairing failed"
                );
                ExitCode::from(e.exit_code())
            }
        }
    };

    tokio::select! {
        code = ceremony => code,
        _ = pair_interrupt() => {
            tracing::warn!("cerulion pair: interrupted (Ctrl-C) before completion — NOT paired");
            ExitCode::from(EXIT_INTERRUPTED)
        }
    }
}

/// An interrupt watcher for the pairing ceremony — Ctrl-C (SIGINT) AND, on Unix,
/// SIGTERM. Unlike `connect`, stdin is the pairing-CODE channel here, so stdin-EOF
/// is NOT a shutdown — only a termination signal interrupts. Registering this
/// future (via the `select!`) installs the signal handlers, so a signal AT the
/// code prompt is caught (a deliberate `EXIT_INTERRUPTED`) rather than
/// default-killing the process by disposition.
async fn pair_interrupt() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        match signal(SignalKind::terminate()) {
            Ok(mut term) => {
                tokio::select! {
                    _ = tokio::signal::ctrl_c() => tracing::info!("cerulion pair: Ctrl-C received"),
                    _ = term.recv() => tracing::info!("cerulion pair: SIGTERM received"),
                }
            }
            Err(e) => {
                // SIGTERM couldn't be registered — fall back to Ctrl-C only.
                tracing::debug!(error = %e, "cerulion pair: could not watch SIGTERM; Ctrl-C only");
                let _ = tokio::signal::ctrl_c().await;
                tracing::info!("cerulion pair: Ctrl-C received");
            }
        }
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
        tracing::info!("cerulion pair: Ctrl-C received");
    }
}

/// Log a configuration error + return the USAGE exit code.
fn usage_error(e: ConnectError) -> ExitCode {
    tracing::error!(error = %e, "cerulion pair: configuration error");
    ExitCode::from(EXIT_USAGE)
}

/// The default access-list label the robot stores for this desk when `--label` is
/// omitted: the desk hostname (or the `CERULION_ROBOT_IDENTITY` override), via the
/// SAME platform identity resolver the `cerulion pair` CLI uses
/// (`cli_engine::pair_cmd::desk_label_default` → this helper), so a direct
/// `cerulion-connectd pair` invocation and the CLI produce ONE consistent label
/// (never a divergent hardcoded literal).
fn default_pair_label() -> String {
    cerulion_core::graph::robot_identity_from_env()
}

/// Resolve the pairing code: `--code` if given (non-empty), else one line from
/// stdin (a TTY prompt on STDERR for a human; a piped line for a Studio driver).
/// An empty code or a closed stdin is a LOUD error (never a silent blank code).
fn resolve_pair_code(cli_code: Option<String>) -> Result<String, String> {
    if let Some(code) = cli_code {
        let c = code.trim().to_string();
        if c.is_empty() {
            return Err("--code was empty; provide the pairing code from the robot".to_string());
        }
        return Ok(c);
    }
    if std::io::stdin().is_terminal() {
        eprint!("Enter the pairing code from the robot: ");
        let _ = std::io::stderr().flush();
    }
    let mut line = String::new();
    let n = std::io::stdin()
        .read_line(&mut line)
        .map_err(|e| format!("reading the pairing code from stdin failed: {e}"))?;
    if n == 0 {
        return Err(
            "no pairing code provided (stdin closed) — pass --code or type the code from the robot"
                .to_string(),
        );
    }
    let c = line.trim().to_string();
    if c.is_empty() {
        return Err("the pairing code was empty".to_string());
    }
    Ok(c)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{CatalogEntry, CatalogProvenance};

    #[test]
    fn pair_cli_debug_redacts_a_parsed_pairing_code() {
        let eid = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
        let code = "731-246-private-pairing-code";
        let with_code = PairCli::try_parse_from(["pair", "--eid", eid, "--code", code]).unwrap();
        let without_code = PairCli::try_parse_from(["pair", "--eid", eid]).unwrap();
        assert_eq!(with_code.code.as_deref(), Some(code));
        assert_eq!(without_code.code, None);
        for config in [with_code, without_code] {
            for output in [format!("{config:?}"), format!("{config:#?}")] {
                assert_eq!(output, "PairCli { code: [REDACTED] }");
                assert!(!output.contains(code));
            }
        }
    }

    /// The `--label` fallback resolves the desk identity (hostname / override) and
    /// is NEVER blank — the robot always stores a non-empty label for this desk.
    #[test]
    fn default_pair_label_is_never_blank() {
        assert!(
            !default_pair_label().is_empty(),
            "the desk pairing label must never be empty"
        );
    }

    /// The catalog STDOUT path neuters + bounds EVERY peer-chosen
    /// string it renders.
    ///
    /// This is the one `main.rs` render surface that writes to an INTERACTIVE stdout
    /// (the two teardown/error surfaces go through `tracing`), and nothing else
    /// pins it: `cargo test -p cerulion_connectd --lib` cannot reach a bin's render
    /// sites, and the bin's other test covers only `default_pair_label`. Without this
    /// test a dropped `sanitize_peer_text` here would regress silently.
    ///
    /// HAND ORACLES (never a self-compare): a screen-clearing CSI escape, a CR line
    /// overwrite and a NEWLINE that would forge extra catalog rows are all `U+FFFD`;
    /// a megabyte topic name is bounded with the explicit marker. Dropping
    /// either `sanitize_peer_text` call in `catalog_lines` fails this test.
    #[test]
    fn catalog_lines_neuter_and_bound_every_peer_string() {
        let entry = |topic: &str| CatalogEntry {
            topic: topic.to_string(),
            schema_hash: Some(0xdead_beef),
            schema_name: None,
            provenance: CatalogProvenance::Runtime,
            producer_count: None,
            // This test is about SANITIZING peer strings; liveness carries no
            // peer-controlled text, so UNKNOWN keeps the fixture on-topic.
            liveness: None,
        };
        let catalog = CatalogReply {
            version: 1,
            // A hostile robot NAME: CSI screen-clear + CR + newline.
            robot: "go\u{1b}[2J2\rx\ny".to_string(),
            entries: vec![
                entry("/scan"),
                // A hostile TOPIC name (the row that would forge extra catalog rows).
                entry("/a\u{1b}[2Jb\rc\nd"),
                // A megabyte topic name.
                entry(&"z".repeat(1_000_000)),
            ],
            error: None,
        };

        let lines = catalog_lines(&catalog);
        assert_eq!(lines.len(), 5, "header + count + one row per entry");
        assert_eq!(lines[0], "robot: go\u{fffd}[2J2\u{fffd}x\u{fffd}y");
        assert_eq!(lines[1], "TOPICS (3):");
        assert_eq!(lines[2], "  /scan  schema_hash=0x00000000deadbeef");
        assert_eq!(
            lines[3],
            "  /a\u{fffd}[2Jb\u{fffd}c\u{fffd}d  schema_hash=0x00000000deadbeef"
        );
        // The flood row is bounded with the explicit marker, so it cannot dump a
        // megabyte into the operator's terminal.
        assert!(lines[4].contains("…(truncated)"));
        assert!(
            lines[4].chars().count() < 600,
            "a megabyte topic name is bounded: {} chars",
            lines[4].chars().count()
        );
        // NO line may carry a raw control character — the property the whole render
        // path exists to guarantee (a leading-two-space indent is the only whitespace).
        for line in &lines {
            assert!(
                !line.chars().any(|c| c.is_control()),
                "no rendered catalog line may carry a raw control character: {line:?}"
            );
        }
    }
}
