// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-rcpd --manifest rcp.yaml [--stdio | --http [ADDR]]`, the receipt
//! and recording checks, and `cerulion-rcpd jev on | off | status`.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use cerulion_rcpd::eye::host::{EyeHost, EyeSettings};
use cerulion_rcpd::eye::verify::{verify_bag, EyeTopics};
use cerulion_rcpd::eye::{switch, JEV_KEY_FILE_ENV};
use cerulion_rcpd::http::{BearerToken, HttpServer};
use cerulion_rcpd::journal::Journal;
use cerulion_rcpd::manifest::RcpManifest;
use cerulion_rcpd::observer::GraphSource;
use cerulion_rcpd::server::RcpServer;
use cerulion_rcpd::tools::{ReceiptLock, ToolHost};
use cerulion_rcpd::{RcpdError, DEFAULT_HTTP_ADDR, TOKEN_ENV};
use clap::Parser;
use rmcp::ServiceExt;

#[derive(Parser, Debug)]
#[command(
    name = "cerulion-rcpd",
    version,
    about,
    args_conflicts_with_subcommands = true,
    subcommand_negates_reqs = true
)]
struct Args {
    #[command(subcommand)]
    command: Option<Command>,
    /// The `rcp.yaml` manifest.
    #[arg(long, required_unless_present_any = ["verify_receipts", "verify_bag"])]
    manifest: Option<PathBuf>,
    /// Serve MCP over stdin/stdout (the default).
    #[arg(long, conflicts_with = "http")]
    stdio: bool,
    /// Serve MCP over streamable HTTP. Needs CERULION_RCP_TOKEN.
    #[arg(long, value_name = "ADDR", num_args = 0..=1, default_missing_value = DEFAULT_HTTP_ADDR)]
    http: Option<SocketAddr>,
    /// The hash-chained receipt log. Defaults to
    /// `.cerulion/rcp/receipts.jsonl` beside the manifest.
    #[arg(long, value_name = "PATH")]
    receipts: Option<PathBuf>,
    /// Check a receipt log's hash chain (rotated files included), print one
    /// line to stderr and exit: 0 when every link holds, 1 when the log is
    /// missing, empty or broken.
    #[arg(long, value_name = "PATH", conflicts_with_all = ["manifest", "stdio", "http"])]
    verify_receipts: Option<PathBuf>,
    /// Join a recording's Jev answers to the receipt log (`--receipts`, or
    /// the manifest's default), print one line to stderr and exit: 0 when
    /// every answer has its receipt and every check holds, 1 otherwise.
    #[arg(long, value_name = "BAG", conflicts_with_all = ["stdio", "http", "verify_receipts"])]
    verify_bag: Option<PathBuf>,
}

#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Turn the Jev eye on or off for this robot, or show its status.
    Jev {
        #[command(subcommand)]
        action: JevAction,
        /// The `rcp.yaml` manifest.
        #[arg(long, global = true, default_value = "rcp.yaml")]
        manifest: PathBuf,
        /// The receipt log, whose directory holds the switch and status.
        #[arg(long, global = true, value_name = "PATH")]
        receipts: Option<PathBuf>,
    },
}

#[derive(clap::Subcommand, Debug, Clone, Copy)]
enum JevAction {
    /// Turn Jev on: frames leave the robot when the gate asks.
    On,
    /// Turn Jev off: the running daemon stops sending with the next ask.
    Off,
    /// Print on or off, route, model, today's asks and looks, and the last
    /// outcome.
    Status,
}

#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    let args = Args::parse();
    if let Some(Command::Jev {
        action,
        manifest,
        receipts,
    }) = &args.command
    {
        return jev(*action, manifest, receipts.as_deref());
    }
    if let Some(path) = args.verify_receipts.as_deref() {
        return verify_receipts(path);
    }
    if let Some(bag) = args.verify_bag.as_deref() {
        return verify_eye_bag(bag, args.manifest.as_deref(), args.receipts.as_deref());
    }
    match run(args).await {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "cerulion-rcpd stopped");
            ExitCode::FAILURE
        }
    }
}

/// `--verify-receipts`: the chain check, reported on stderr (stdout stays
/// reserved for the MCP stream in every mode) whatever `RUST_LOG` says.
#[allow(
    clippy::print_stderr,
    reason = "a one-shot report the operator asked for, not a log line"
)]
fn verify_receipts(path: &std::path::Path) -> ExitCode {
    match cerulion_rcpd::audit::verify_log(path) {
        Ok(report) => {
            let start = if report.from_genesis {
                String::from("from genesis")
            } else {
                format!(
                    "from seq {} (older rotated files are gone)",
                    report.first_seq
                )
            };
            eprintln!(
                "receipt chain intact: {} entries in {} file(s), {start}; last hash {}",
                report.entries,
                report.files.len(),
                report.last_hash
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("receipt chain BROKEN: {e}");
            ExitCode::FAILURE
        }
    }
}

/// The receipt log path: `--receipts`, else `.cerulion/rcp/receipts.jsonl`
/// beside the manifest.
fn receipts_path(manifest: &std::path::Path, receipts: Option<&std::path::Path>) -> PathBuf {
    receipts.map_or_else(
        || {
            manifest
                .parent()
                .unwrap_or_else(|| std::path::Path::new(""))
                .join(".cerulion/rcp/receipts.jsonl")
        },
        std::path::Path::to_path_buf,
    )
}

/// The directory beside the receipt log: switch, notice marker, status and
/// the default key file.
fn state_dir(receipts: &std::path::Path) -> PathBuf {
    receipts
        .parent()
        .filter(|d| !d.as_os_str().is_empty())
        .map_or_else(|| PathBuf::from("."), std::path::Path::to_path_buf)
}

fn eye_settings(
    manifest: &std::path::Path,
    loaded: &cerulion_rcpd::manifest::LoadedManifest,
    receipts: &std::path::Path,
) -> Result<EyeSettings, RcpdError> {
    EyeSettings::resolve(
        &loaded.eyes,
        manifest
            .parent()
            .unwrap_or_else(|| std::path::Path::new("")),
        &state_dir(receipts),
        std::env::var(JEV_KEY_FILE_ENV).ok().as_deref(),
    )
    .map_err(|reason| RcpdError::ManifestParse {
        path: manifest.to_path_buf(),
        reason,
    })
}

/// `--verify-bag`: the join report on stderr.
#[allow(
    clippy::print_stderr,
    reason = "a one-shot report the operator asked for, not a log line"
)]
fn verify_eye_bag(
    bag: &std::path::Path,
    manifest: Option<&std::path::Path>,
    receipts: Option<&std::path::Path>,
) -> ExitCode {
    let loaded = match manifest.map(RcpManifest::load_full).transpose() {
        Ok(loaded) => loaded,
        Err(e) => {
            eprintln!("verify-bag: {e}");
            return ExitCode::FAILURE;
        }
    };
    let receipts = match (receipts, manifest) {
        (Some(path), _) => path.to_path_buf(),
        (None, Some(manifest)) => receipts_path(manifest, None),
        (None, None) => {
            eprintln!(
                "verify-bag: name the receipt log with --receipts, or the manifest with --manifest"
            );
            return ExitCode::FAILURE;
        }
    };
    let topics = match loaded.as_ref().and_then(|l| l.eyes.jev.as_ref()) {
        Some(jev) => EyeTopics {
            ask: jev.ask_topic.clone(),
            answer: jev.answer_topic.clone(),
            frame: Some(jev.frame_topic.clone()),
        },
        None => EyeTopics::default(),
    };
    match verify_bag(bag, &receipts, &topics) {
        Ok(report) if report.problems.is_empty() => {
            eprintln!(
                "eye answers joined: {} of {} answers have their receipt; {} asks ({} without an \
                 answer in the recording), {} late receipts, {} requests recomputed from the \
                 recorded frame",
                report.joined,
                report.answers,
                report.asks,
                report.unanswered,
                report.late,
                report.requests_checked
            );
            ExitCode::SUCCESS
        }
        Ok(report) => {
            for problem in &report.problems {
                eprintln!("eye answers BROKEN: {problem}");
            }
            ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("verify-bag: {e}");
            ExitCode::FAILURE
        }
    }
}

/// `jev on | off | status`.
#[allow(
    clippy::print_stdout,
    clippy::print_stderr,
    reason = "a one-shot command the owner ran, not a log line"
)]
fn jev(
    action: JevAction,
    manifest: &std::path::Path,
    receipts: Option<&std::path::Path>,
) -> ExitCode {
    let loaded = match RcpManifest::load_full(manifest) {
        Ok(loaded) => loaded,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    let receipts = receipts_path(manifest, receipts);
    let settings = match eye_settings(manifest, &loaded, &receipts) {
        Ok(settings) => settings,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };
    let host = settings
        .endpoint
        .as_ref()
        .and_then(|u| u.host_str().map(str::to_owned))
        .unwrap_or_default();
    match action {
        JevAction::On | JevAction::Off => {
            let on = matches!(action, JevAction::On);
            if let Err(e) = switch::write_switch(&settings.state_dir, on) {
                eprintln!(
                    "cannot write the switch in {}: {e}",
                    settings.state_dir.display()
                );
                return ExitCode::FAILURE;
            }
            if on {
                println!("Jev is on. {}", switch::notice_line(&host));
            } else {
                println!("Jev is off; a running cerulion-rcpd stops sending with its next ask.");
            }
            ExitCode::SUCCESS
        }
        JevAction::Status => {
            if settings.jev.is_none() {
                println!("jev: off (the manifest has no eyes.jev section)");
                return ExitCode::SUCCESS;
            }
            let now = settings.status();
            let counted = switch::read_status(&settings.state_dir).filter(|s| s.day == now.day);
            println!("jev: {}", if now.on { "on" } else { "off" });
            println!("switch: {} (notice {})", now.switch, now.notice);
            println!("credential: {}", now.credential);
            println!("route: {}", now.route);
            println!("model: {}", now.model);
            let (asks, looks, limit, last) = counted.map_or((0, 0, false, None), |s| {
                (
                    s.asks_today,
                    s.looks_today,
                    s.limit_reached_today,
                    s.last_outcome,
                )
            });
            println!(
                "today ({} UTC): {asks} asks, {looks} looks, limit reached: {}",
                now.day,
                if limit { "yes" } else { "no" }
            );
            println!("last outcome: {}", last.as_deref().unwrap_or("none"));
            ExitCode::SUCCESS
        }
    }
}

fn init_tracing() {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .init();
}

async fn run(args: Args) -> Result<(), RcpdError> {
    let token = match args.http {
        Some(_) => Some(BearerToken::from_env_value(std::env::var(TOKEN_ENV).ok())?),
        None => None,
    };
    let manifest = args.manifest.unwrap_or_default();
    let loaded = RcpManifest::load_full(&manifest)?;
    let receipts = receipts_path(&manifest, args.receipts.as_deref());
    let eye_settings = eye_settings(&manifest, &loaded, &receipts)?;
    let manager = cerulion_core::TransportManager::init_default()?;
    // The receipt lock first: a second daemon on the same log stops here,
    // before it opens anything on the transport.
    let lock = ReceiptLock::acquire(&receipts)?;
    let latch = Arc::new(AtomicBool::new(false));
    let journal = Journal::spawn(
        manager.clone(),
        loaded.journal.clone(),
        loaded.state.clone(),
        latch.clone(),
    )?;
    let host = Arc::new(
        ToolHost::open(loaded.tools, journal, latch, &receipts, lock)?
            .with_estop_clear(loaded.limits.estop_clear),
    );
    tracing::info!(
        receipts = %receipts.display(),
        calls = %loaded.journal.calls_topic,
        results = %loaded.journal.results_topic,
        "tools journaled"
    );
    // The eye's ports open only under `eyes.jev`. An eye that cannot start
    // is logged and the daemon serves without it.
    let _eye = match &eye_settings.jev {
        Some(_) => match EyeHost::spawn(
            manager.clone(),
            eye_settings,
            host.receipt_sink(),
            tokio::runtime::Handle::current(),
        ) {
            Ok(eye) => Some(eye),
            Err(e) => {
                tracing::error!(error = %e, "the Jev eye did not start; serving without it");
                None
            }
        },
        None => None,
    };
    let source = GraphSource::new(manager, loaded.identity);
    let feed = source.feed();
    let _observer = source.spawn()?;
    let state = loaded.state;
    let make_server = move || RcpServer::with_tools(feed.clone(), host.clone(), state.clone());
    match (args.http, token) {
        (Some(addr), Some(token)) => serve_http(addr, token, make_server).await,
        _ => serve_stdio(make_server()).await,
    }
}

async fn serve_stdio(server: RcpServer) -> Result<(), RcpdError> {
    tracing::info!("serving MCP on stdio");
    let session = server
        .serve(rmcp::transport::stdio())
        .await
        .map_err(|e| RcpdError::Session {
            reason: e.to_string(),
        })?;
    session.waiting().await.map_err(|e| RcpdError::Session {
        reason: e.to_string(),
    })?;
    Ok(())
}

async fn serve_http(
    addr: SocketAddr,
    token: BearerToken,
    make_server: impl Fn() -> RcpServer + Send + Sync + 'static,
) -> Result<(), RcpdError> {
    let server = HttpServer::bind_serving(addr, token, make_server).await?;
    let bound = server
        .local_addr()
        .map_err(|source| RcpdError::Bind { addr, source })?;
    tracing::info!(addr = %bound, "serving MCP on streamable HTTP");
    server
        .serve(async {
            if let Err(e) = tokio::signal::ctrl_c().await {
                tracing::error!(error = %e, "cannot listen for ctrl-c; shutting down");
            }
        })
        .await;
    Ok(())
}
