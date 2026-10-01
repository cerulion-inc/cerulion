//! Tokio Unix-domain listener for the workspace daemon.

use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
#[cfg(test)]
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;
use std::sync::Mutex as StdMutex;
#[cfg(test)]
use std::sync::OnceLock;

use tokio::io::{AsyncBufRead, AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{UnixListener, UnixStream};
use tokio::sync::watch;
use tokio::task::JoinHandle;
use tokio::time::{sleep, Duration};

use crate::hygiene::{self, SocketGuard};
use crate::protocol;
use crate::WsdError;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

/// Longest request line the daemon reads. A longer line is answered with a
/// `bad_request` error (`id: null`) and the connection is closed; the daemon
/// never buffers past the bound. Public so a client can size its requests.
pub const MAX_REQUEST_LINE_BYTES: usize = 1024 * 1024;

/// Env knob: how long `shutdown` waits for in-flight requests before it
/// aborts them (ms). Mirrors `CERULION_NETD_HARD_EXIT_MS`; a request blocked
/// on a foreign workspace lock must not turn SIGTERM into a zombie that keeps
/// the socket singleton.
pub const HARD_EXIT_ENV: &str = "CERULION_WSD_HARD_EXIT_MS";
const DEFAULT_HARD_EXIT_MS: u64 = 5000;

/// How many reply lines a request may have queued ahead of the socket writer.
const REPLY_LINES_IN_FLIGHT: usize = 256;

fn hard_exit_deadline() -> Duration {
    match std::env::var(HARD_EXIT_ENV) {
        Ok(value) => match value.trim().parse::<u64>() {
            Ok(ms) => Duration::from_millis(ms),
            Err(_) => {
                tracing::warn!(
                    env = HARD_EXIT_ENV,
                    value = %value,
                    default_ms = DEFAULT_HARD_EXIT_MS,
                    "unparseable shutdown deadline; using the default"
                );
                Duration::from_millis(DEFAULT_HARD_EXIT_MS)
            }
        },
        Err(_) => Duration::from_millis(DEFAULT_HARD_EXIT_MS),
    }
}

struct ConnectionTaskRegistry {
    accepting: bool,
    tasks: Vec<JoinHandle<()>>,
}

type ConnectionTasks = Arc<StdMutex<ConnectionTaskRegistry>>;

#[cfg(test)]
static BLOCKING_REQUEST_STARTED: OnceLock<StdMutex<Option<Sender<()>>>> = OnceLock::new();
#[cfg(test)]
static BLOCKING_REQUEST_FINISHED: OnceLock<StdMutex<Option<Sender<()>>>> = OnceLock::new();

#[cfg(test)]
fn notify_blocking_request_started() {
    if let Some(sender) = BLOCKING_REQUEST_STARTED
        .get()
        .and_then(|hook| hook.lock().expect("blocking request hook poisoned").take())
    {
        let _ = sender.send(());
    }
}

#[cfg(test)]
fn notify_blocking_request_finished() {
    if let Some(sender) = BLOCKING_REQUEST_FINISHED
        .get()
        .and_then(|hook| hook.lock().expect("blocking request hook poisoned").take())
    {
        let _ = sender.send(());
    }
}

#[derive(Debug, Clone)]
pub struct WsdConfig {
    pub socket_path: PathBuf,
    /// The program `graph.validate` runs to read a node library's info in a
    /// CHILD process (`<program> --inspect-node <cdylib>`): `None` means this
    /// daemon's own binary. A library that aborts on load then kills the child,
    /// not the daemon — see `cerulion_cli_engine::node_inspector`.
    pub inspector_program: Option<PathBuf>,
    /// How long one inspection may take before the child is killed.
    pub inspector_timeout: Duration,
}

impl Default for WsdConfig {
    fn default() -> Self {
        Self {
            socket_path: hygiene::default_socket_path(),
            inspector_program: None,
            inspector_timeout: cerulion_cli_engine::node_inspector::DEFAULT_INSPECT_TIMEOUT,
        }
    }
}

/// The argument the daemon's own binary answers with a library's info JSON.
pub const INSPECT_NODE_FLAG: &str = "--inspect-node";

type Inspector = Arc<dyn cerulion_cli_engine::node_inspector::NodeInspector>;

fn build_inspector(config: &WsdConfig) -> Result<Inspector, WsdError> {
    let program = match &config.inspector_program {
        Some(program) => program.clone(),
        None => std::env::current_exe()?,
    };
    Ok(Arc::new(
        cerulion_cli_engine::node_inspector::SubprocessInspector::new(
            program,
            vec![std::ffi::OsString::from(INSPECT_NODE_FLAG)],
            config.inspector_timeout,
        ),
    ))
}

pub struct RunningWsd {
    socket_path: PathBuf,
    shutdown: watch::Sender<bool>,
    accept_handle: Option<JoinHandle<()>>,
    connection_tasks: ConnectionTasks,
    guard: Option<Arc<SocketGuard>>,
}

impl RunningWsd {
    pub fn socket_path(&self) -> &std::path::Path {
        &self.socket_path
    }

    /// Stop accepting, wait for in-flight requests up to the
    /// [`HARD_EXIT_ENV`] deadline (default 5 s), abort whatever is still
    /// running, then release the socket + pidfile singleton.
    pub async fn shutdown(&mut self) {
        let _ = self.shutdown.send(true);
        if let Some(handle) = self.accept_handle.take() {
            let _ = handle.await;
        }
        let deadline = hard_exit_deadline();
        if tokio::time::timeout(deadline, drain_connection_tasks(&self.connection_tasks))
            .await
            .is_err()
        {
            tracing::warn!(
                deadline_ms = deadline.as_millis() as u64,
                "in-flight requests did not finish before the shutdown deadline; aborting them"
            );
            abort_connection_tasks(&self.connection_tasks);
        }
        self.guard.take();
    }
}

impl Drop for RunningWsd {
    fn drop(&mut self) {
        let _ = self.shutdown.send(true);
        if let Some(handle) = self.accept_handle.take() {
            handle.abort();
        }
        abort_connection_tasks(&self.connection_tasks);
        drop(self.guard.take());
    }
}

pub async fn start(config: WsdConfig) -> Result<RunningWsd, WsdError> {
    let inspector = build_inspector(&config)?;
    let (listener, guard) = hygiene::acquire_socket(config.socket_path.clone())?;
    let guard = Arc::new(guard);
    let (shutdown, shutdown_rx) = watch::channel(false);
    let connection_tasks = Arc::new(StdMutex::new(ConnectionTaskRegistry {
        accepting: true,
        tasks: Vec::new(),
    }));
    let accept_handle = tokio::spawn(accept_loop(
        listener,
        shutdown_rx,
        Arc::clone(&connection_tasks),
        Arc::clone(&guard),
        inspector,
    ));
    Ok(RunningWsd {
        socket_path: config.socket_path,
        shutdown,
        accept_handle: Some(accept_handle),
        connection_tasks,
        guard: Some(guard),
    })
}

async fn accept_loop(
    listener: UnixListener,
    mut shutdown: watch::Receiver<bool>,
    connection_tasks: ConnectionTasks,
    socket_guard: Arc<SocketGuard>,
    inspector: Inspector,
) {
    let mut accept_failures = FailureRegimeLatch::new();
    loop {
        tokio::select! {
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    break;
                }
            }
            accepted = listener.accept() => match accepted {
                Ok((stream, _)) => {
                    if *shutdown.borrow() {
                        break;
                    }
                    let task = tokio::spawn(handle_connection(
                        stream,
                        shutdown.clone(),
                        Arc::clone(&socket_guard),
                        Arc::clone(&inspector),
                    ));
                    let mut registry = connection_tasks
                        .lock()
                        .expect("connection task registry poisoned");
                    // Reap finished handlers so a standing daemon's registry
                    // stays bounded by its LIVE connections (netd does the same).
                    registry.tasks.retain(|task| !task.is_finished());
                    if registry.accepting {
                        registry.tasks.push(task);
                    } else {
                        task.abort();
                        break;
                    }
                }
                Err(error) => {
                    // NEVER leave the loop on an accept error: a daemon that
                    // stops accepting while it keeps the socket + pidfile flock
                    // is a zombie every replacement refuses to start beside.
                    // Log through the flood latch and retry after a pause, the
                    // netd behaviour; shutdown is the only exit.
                    match accept_failures.on_failure() {
                        RegimeDecision::Loud => {
                            tracing::warn!(error = %error, "workspace daemon accept failed");
                        }
                        RegimeDecision::StillFailing { total, suppressed } => {
                            tracing::warn!(
                                error = %error,
                                total_failures = total,
                                suppressed,
                                "workspace daemon accept still failing"
                            );
                        }
                        RegimeDecision::Suppressed { suppressed } => {
                            tracing::debug!(
                                error = %error,
                                suppressed,
                                "workspace daemon accept failure repeated"
                            );
                        }
                    }
                    tokio::select! {
                        _ = sleep(Duration::from_millis(50)) => {}
                        changed = shutdown.changed() => {
                            if changed.is_err() || *shutdown.borrow() {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

async fn drain_connection_tasks(connection_tasks: &ConnectionTasks) {
    let tasks = {
        let mut registry = connection_tasks
            .lock()
            .expect("connection task registry poisoned");
        registry.accepting = false;
        std::mem::take(&mut registry.tasks)
    };
    for task in tasks {
        let _ = task.await;
    }
}

fn abort_connection_tasks(connection_tasks: &ConnectionTasks) {
    let mut registry = connection_tasks
        .lock()
        .expect("connection task registry poisoned");
    registry.accepting = false;
    for task in &registry.tasks {
        task.abort();
    }
    registry.tasks.clear();
}

async fn handle_connection(
    stream: UnixStream,
    mut shutdown: watch::Receiver<bool>,
    socket_guard: Arc<SocketGuard>,
    inspector: Inspector,
) {
    let (reader, mut writer) = stream.into_split();
    if writer
        .write_all(format!("{}\n", protocol::hello_line()).as_bytes())
        .await
        .is_err()
    {
        return;
    }
    let mut reader = BufReader::new(reader);
    // Bytes a client sent behind a running `node.build`: set aside so the
    // read half can keep watching for a hangup, and answered afterwards.
    let mut pending: Vec<u8> = Vec::new();
    loop {
        if *shutdown.borrow() {
            break;
        }
        let line = tokio::select! {
            result = read_bounded_line(&mut reader, &mut pending) => match result {
                Ok(Some(line)) => line,
                Ok(None) => break,
                Err(error) => {
                    // An over-long or non-UTF-8 line is a protocol error, so the
                    // client gets a structured answer (no id to correlate) before
                    // the connection closes; the daemon never buffers past the
                    // bound.
                    tracing::warn!(error = %error, "workspace daemon request line rejected");
                    let rejection = protocol::Response::failure(None, "bad_request", error.to_string());
                    if let Ok(encoded) = serde_json::to_string(&rejection) {
                        let _ = writer.write_all(format!("{encoded}\n").as_bytes()).await;
                    }
                    break;
                }
            },
            changed = shutdown.changed() => {
                if changed.is_err() || *shutdown.borrow() {
                    break;
                }
                continue;
            }
        };
        let mut watching_for_eof = protocol::cancels_on_hangup(&line);
        let request_guard = Arc::clone(&socket_guard);
        let request_inspector = Arc::clone(&inspector);
        // The handler runs on a blocking thread and hands each line it
        // produces over a bounded channel (a client that reads slowly slows a
        // build down instead of growing memory). `cancelled` is how a closed
        // connection reaches a build in progress.
        let (lines, mut produced) = tokio::sync::mpsc::channel::<String>(REPLY_LINES_IN_FLIGHT);
        let cancelled = Arc::new(AtomicBool::new(false));
        let handler_cancelled = Arc::clone(&cancelled);
        // A connection task aborted mid-request (shutdown) stops its build too.
        let _cancel_on_drop = CancelOnDrop(Arc::clone(&cancelled));
        let handler = tokio::task::spawn_blocking(move || {
            {
                let _socket_guard = request_guard;
                #[cfg(test)]
                notify_blocking_request_started();
                protocol::handle_line_streaming(
                    &line,
                    request_inspector.as_ref(),
                    &mut |value| match serde_json::to_string(&value) {
                        Ok(encoded) => {
                            if lines.blocking_send(encoded).is_err() {
                                handler_cancelled.store(true, Ordering::Release);
                            }
                        }
                        Err(error) => {
                            tracing::error!(error = %error, "failed to encode workspace daemon response");
                            handler_cancelled.store(true, Ordering::Release);
                        }
                    },
                    &handler_cancelled,
                );
            }
            #[cfg(test)]
            notify_blocking_request_finished();
        });
        let mut client_gone = false;
        loop {
            tokio::select! {
                encoded = produced.recv() => match encoded {
                    Some(encoded) => {
                        if !client_gone
                            && writer
                                .write_all(format!("{encoded}\n").as_bytes())
                                .await
                                .is_err()
                        {
                            client_gone = true;
                            cancelled.store(true, Ordering::Release);
                        }
                    }
                    None => break,
                },
                // `node.build` only: the client keeps its connection open
                // until `done`, so an end of file here is it hanging up (or
                // half-closing), which cancels the build. Every other verb
                // keeps answering a client that has closed its write side.
                // Bytes (a pipelined next request) are set aside, not lost,
                // and watching goes on behind them.
                ready = reader.fill_buf(), if watching_for_eof => match ready {
                    Ok(bytes) if !bytes.is_empty() => {
                        let taken = bytes.len();
                        if queue_overflows(&pending, bytes) {
                            watching_for_eof = false;
                            client_gone = true;
                            cancelled.store(true, Ordering::Release);
                        } else {
                            pending.extend_from_slice(bytes);
                            reader.consume(taken);
                        }
                    }
                    _ => {
                        watching_for_eof = false;
                        client_gone = true;
                        cancelled.store(true, Ordering::Release);
                    }
                },
            }
        }
        if let Err(error) = handler.await {
            tracing::error!(error = %error, "workspace daemon request task failed");
            break;
        }
        if client_gone {
            break;
        }
    }
}

/// Sets the flag when dropped: the request is over, or its task was aborted.
struct CancelOnDrop(Arc<AtomicBool>);

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Release);
    }
}

/// Requests a client may queue behind a running `node.build`, in bytes. Each
/// line is still held to [`MAX_REQUEST_LINE_BYTES`]; this bounds how many such
/// lines the daemon keeps in memory while it waits for the build to end.
const MAX_QUEUED_BYTES: usize = 8 * MAX_REQUEST_LINE_BYTES;

/// Would taking `incoming` into `pending` queue too much: more than
/// [`MAX_QUEUED_BYTES`] in all, or any one line (counting the unfinished one
/// `pending` ends with) past the line limit?
fn queue_overflows(pending: &[u8], incoming: &[u8]) -> bool {
    if pending.len() + incoming.len() > MAX_QUEUED_BYTES {
        return true;
    }
    let mut line_len = pending.len()
        - pending
            .iter()
            .rposition(|byte| *byte == b'\n')
            .map_or(0, |at| at + 1);
    for byte in incoming {
        if *byte == b'\n' {
            line_len = 0;
        } else {
            line_len += 1;
            if line_len > MAX_REQUEST_LINE_BYTES {
                return true;
            }
        }
    }
    false
}

/// Read one request line. `pending` holds bytes already taken off the socket
/// but not yet consumed as a request (a client that pipelined a request behind
/// a `node.build`), and keeps whatever follows the returned line.
async fn read_bounded_line<R>(reader: &mut R, pending: &mut Vec<u8>) -> io::Result<Option<String>>
where
    R: AsyncBufRead + Unpin,
{
    let too_long = || {
        io::Error::new(
            io::ErrorKind::InvalidData,
            "request line exceeds maximum length",
        )
    };
    let invalid = |error| io::Error::new(io::ErrorKind::InvalidData, error);
    loop {
        if let Some(newline) = pending.iter().position(|byte| *byte == b'\n') {
            if newline > MAX_REQUEST_LINE_BYTES {
                return Err(too_long());
            }
            let rest = pending.split_off(newline + 1);
            let mut line = std::mem::replace(pending, rest);
            line.pop();
            return String::from_utf8(line).map(Some).map_err(invalid);
        }
        if pending.len() > MAX_REQUEST_LINE_BYTES {
            return Err(too_long());
        }
        let buffer = reader.fill_buf().await?;
        if buffer.is_empty() {
            return if pending.is_empty() {
                Ok(None)
            } else {
                String::from_utf8(std::mem::take(pending))
                    .map(Some)
                    .map_err(invalid)
            };
        }
        pending.extend_from_slice(buffer);
        let consumed = buffer.len();
        reader.consume(consumed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_cli_engine::node_cmd;
    use cerulion_cli_engine::workspace;
    use cerulion_cli_engine::workspace_lock::WorkspaceLock;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
    use tokio::net::UnixStream;

    #[test]
    fn queued_requests_are_limited_per_line_and_in_total() {
        let line = |len: usize| {
            let mut bytes = vec![b'x'; len];
            bytes.push(b'\n');
            bytes
        };
        // Several valid lines that together pass one line's limit are fine.
        let mut pending = Vec::new();
        for _ in 0..4 {
            let incoming = line(MAX_REQUEST_LINE_BYTES / 2);
            assert!(!queue_overflows(&pending, &incoming));
            pending.extend_from_slice(&incoming);
        }
        assert!(pending.len() > MAX_REQUEST_LINE_BYTES);
        // One unfinished line past the limit is not.
        let long = vec![b'x'; MAX_REQUEST_LINE_BYTES + 1];
        assert!(queue_overflows(&[], &long));
        let half = vec![b'x'; MAX_REQUEST_LINE_BYTES / 2 + 1];
        assert!(queue_overflows(&half, &half));
        // A line that crosses the boundary between what is held and what
        // arrives, with a newline in the new bytes, is still one line.
        let mut ends = half.clone();
        ends.push(b'\n');
        assert!(queue_overflows(&half, &ends));
        // A finished line behind a long unfinished one does not reset it.
        assert!(!queue_overflows(&line(10), &half));
        // Nor is the total unbounded.
        let full = vec![b'\n'; MAX_QUEUED_BYTES];
        assert!(queue_overflows(&full, b"\n"));
    }

    #[tokio::test]
    async fn shutdown_during_in_flight_mutation_keeps_singleton_until_it_commits() {
        let base =
            std::env::temp_dir().join(format!("cerulion-wsd-inflight-{}", std::process::id()));
        let parent = (0..100)
            .map(|suffix| base.with_extension(suffix.to_string()))
            .find(|path| std::fs::create_dir(path).is_ok())
            .expect("could not create a unique temporary directory");
        let root = workspace::workspace_create(&parent, "workspace")
            .unwrap()
            .root;
        node_cmd::node_create(
            &root.join("nodes"),
            &root.join("Cargo.toml"),
            "camera",
            None,
        )
        .unwrap();
        let socket = parent.join("wsd.sock");
        let running = start(WsdConfig {
            socket_path: socket.clone(),
            ..WsdConfig::default()
        })
        .await
        .unwrap();

        let workspace_lock = WorkspaceLock::acquire(&root).unwrap();
        let (started_tx, started_rx) = mpsc::channel();
        let (finished_tx, finished_rx) = mpsc::channel();
        BLOCKING_REQUEST_STARTED
            .get_or_init(|| StdMutex::new(None))
            .lock()
            .unwrap()
            .replace(started_tx);
        BLOCKING_REQUEST_FINISHED
            .get_or_init(|| StdMutex::new(None))
            .lock()
            .unwrap()
            .replace(finished_tx);

        let stream = UnixStream::connect(&socket).await.unwrap();
        let (reader, mut writer) = stream.into_split();
        let mut reader = BufReader::new(reader);
        let mut hello = String::new();
        reader.read_line(&mut hello).await.unwrap();
        writer
            .write_all(
                format!(
                    "{{\"id\":1,\"verb\":\"node.modify\",\"root\":\"{}\",\"node_type\":\"camera\",\"op\":{{\"op\":\"add_port\",\"port_name\":\"image\",\"is_output\":true}}}}\n",
                    root.display()
                )
                .as_bytes(),
            )
            .await
            .unwrap();
        tokio::task::spawn_blocking(move || started_rx.recv().unwrap())
            .await
            .unwrap();

        drop(running);
        assert!(hygiene::acquire_socket(socket.clone()).is_err());

        drop(workspace_lock);
        tokio::task::spawn_blocking(move || finished_rx.recv().unwrap())
            .await
            .unwrap();
        let (_listener, _replacement_guard) = hygiene::acquire_socket(socket).unwrap();
        let _ = std::fs::remove_dir_all(parent);
    }
}
