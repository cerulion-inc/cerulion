//! Tokio Unix-domain listener for the workspace daemon.

use std::io;
use std::path::PathBuf;
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
    loop {
        if *shutdown.borrow() {
            break;
        }
        let line = tokio::select! {
            result = read_bounded_line(&mut reader) => match result {
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
        let request_guard = Arc::clone(&socket_guard);
        let request_inspector = Arc::clone(&inspector);
        let response = match tokio::task::spawn_blocking(move || {
            let response = {
                let _socket_guard = request_guard;
                #[cfg(test)]
                notify_blocking_request_started();
                protocol::handle_line(&line, request_inspector.as_ref())
            };
            #[cfg(test)]
            notify_blocking_request_finished();
            response
        })
        .await
        {
            Ok(response) => response,
            Err(error) => {
                tracing::error!(error = %error, "workspace daemon request task failed");
                break;
            }
        };
        let encoded = match serde_json::to_string(&response) {
            Ok(encoded) => encoded,
            Err(error) => {
                tracing::error!(error = %error, "failed to encode workspace daemon response");
                break;
            }
        };
        if writer
            .write_all(format!("{encoded}\n").as_bytes())
            .await
            .is_err()
        {
            break;
        }
    }
}

async fn read_bounded_line<R>(reader: &mut R) -> io::Result<Option<String>>
where
    R: AsyncBufRead + Unpin,
{
    let mut line = Vec::new();
    loop {
        let buffer = reader.fill_buf().await?;
        if buffer.is_empty() {
            return if line.is_empty() {
                Ok(None)
            } else {
                String::from_utf8(line)
                    .map(Some)
                    .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))
            };
        }
        if let Some(newline) = buffer.iter().position(|byte| *byte == b'\n') {
            if line.len() + newline > MAX_REQUEST_LINE_BYTES {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "request line exceeds maximum length",
                ));
            }
            line.extend_from_slice(&buffer[..newline]);
            reader.consume(newline + 1);
            return String::from_utf8(line)
                .map(Some)
                .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error));
        }
        if line.len() + buffer.len() > MAX_REQUEST_LINE_BYTES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "request line exceeds maximum length",
            ));
        }
        line.extend_from_slice(buffer);
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
