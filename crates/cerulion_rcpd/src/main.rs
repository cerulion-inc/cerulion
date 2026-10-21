// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-rcpd --manifest rcp.yaml [--stdio | --http [ADDR]]`.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

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
#[command(name = "cerulion-rcpd", version, about)]
struct Args {
    /// The `rcp.yaml` manifest.
    #[arg(long, required_unless_present = "verify_receipts")]
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
}

#[tokio::main]
async fn main() -> ExitCode {
    init_tracing();
    let args = Args::parse();
    if let Some(path) = args.verify_receipts.as_deref() {
        return verify_receipts(path);
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
    let receipts = args.receipts.unwrap_or_else(|| {
        manifest
            .parent()
            .unwrap_or_else(|| std::path::Path::new(""))
            .join(".cerulion/rcp/receipts.jsonl")
    });
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
    let host = Arc::new(ToolHost::open(
        loaded.tools,
        journal,
        latch,
        &receipts,
        lock,
    )?);
    tracing::info!(
        receipts = %receipts.display(),
        calls = %loaded.journal.calls_topic,
        results = %loaded.journal.results_topic,
        "tools journaled"
    );
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
