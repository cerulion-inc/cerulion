// SPDX-License-Identifier: AGPL-3.0-only
//! The real `cerulion-connectd connect`, and the real `cerulion connect` that
//! spawns it, stopped while a session streams, with stdin a pipe this test
//! holds open.
//!
//! A test robot on loopback lists one topic in its catalog, accepts the
//! demand for it and opens the topic's data stream, then sends no frame, so
//! the session streams until it is stopped.
//!
//! - SIGINT to `cerulion-connectd connect` ends the session with exit 0
//!   within [`EXIT_CEILING`] of the signal, having logged that it caught
//!   Ctrl-C and tore the session down.
//! - Closing stdin ends the session with exit 0 within [`EXIT_CEILING`] of
//!   the close, having logged that stdin closed.
//! - SIGINT and SIGTERM sent to `cerulion connect` alone (it forwards SIGINT
//!   to the helper) each end the run with exit 0 within [`EXIT_CEILING`],
//!   well inside the 5 s grace after which `cerulion` kills the helper (exit
//!   137).
//!
//! In every run stdout is exactly the robot's catalog. A signal is sent once
//! stderr carries the line that says how to stop the session, which the
//! helper logs after its Ctrl-C handler and stdin watcher are in place. Each
//! run has its own temporary home and working directory.
//!
//! Prerequisite for the `cerulion connect` arms: `cargo build -p cerulion_cli`
//! in the same target and profile, which puts `cerulion` beside
//! `cerulion-connectd`.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Read};
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, ExitStatus, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_connectd::protocol::{StreamPreamble, WireRequest, WireResponse};
use cerulion_core::transport::cerulion_q::{
    CatalogEntry, CatalogProvenance, CatalogReply, SchemaReply, CATALOG_WIRE_VERSION,
};
use cerulion_link::{
    accept_frame_stream, accept_one, alpn, build_endpoint, open_uni_frame_stream, read_frame,
    write_frame, Endpoint, EndpointConfig, RelayConfig, DEFAULT_MAX_FRAME_LEN,
};

/// The name the test robot gives itself in its catalog.
const ROBOT: &str = "connect-interrupt-test-robot";

/// The one topic the test robot lists and every run demands.
const TOPIC: &str = "/connect_interrupt/imu";

/// The stderr line the helper logs once its Ctrl-C handler and stdin watcher
/// are in place.
const READY: &str = "cerulion connect: re-injecting into desk-local SHM";

/// The stderr line of a helper that caught SIGINT.
const CTRL_C: &str = "cerulion connect: Ctrl-C received";

/// The stderr line of a helper whose stdin reached its end.
const STDIN_CLOSED: &str = "cerulion connect: stdin closed (parent went away)";

/// The stderr line of a helper that stops its session on either of the two.
const TEARDOWN: &str = "cerulion connect: shutdown signalled; tearing down";

/// How long a run may take to log [`READY`].
const READY_LIMIT: Duration = Duration::from_secs(30);

/// The most the time from the stop to the reaped exit may be.
const EXIT_CEILING: Duration = Duration::from_secs(2);

/// How long the test waits for the exit before it kills the run: past the
/// 5 s grace, so a helper that never exits shows under `cerulion connect` as
/// the 137 it becomes.
const WAIT_LIMIT: Duration = Duration::from_secs(15);

/// How long the test reads the pipes after the run is reaped.
const DRAIN_LIMIT: Duration = Duration::from_secs(5);

/// The stdout of a run: the test robot's catalog as the helper prints it,
/// with no hash for a topic whose catalog entry has none.
fn catalog_stdout() -> String {
    format!("robot: {ROBOT}\nTOPICS (1):\n  {TOPIC}  schema_hash=\u{2014}")
}

/// The test robot's catalog: [`TOPIC`] with no schema hash, so the helper
/// opens no shared-memory publisher until a frame arrives, and none does.
fn catalog() -> CatalogReply {
    CatalogReply {
        wire_header_version: cerulion_core::wire::WIRE_HEADER_VERSION,
        version: CATALOG_WIRE_VERSION,
        robot: ROBOT.to_string(),
        entries: vec![CatalogEntry {
            egress_qos: None,
            topic: TOPIC.to_string(),
            schema_hash: None,
            schema_name: None,
            provenance: CatalogProvenance::Runtime,
            producer_count: None,
            liveness: None,
        }],
        error: None,
    }
}

/// Serve one desk session on `endpoint`: the catalog, and for each demand a
/// data stream opened with its preamble before the demand is accepted, as a
/// robot does, that then carries nothing. Every other request is answered
/// with an error. Returns once the desk closes the connection.
async fn serve(endpoint: Endpoint) {
    let Ok(Some(accepted)) = accept_one(&endpoint).await else {
        return;
    };
    let connection = accepted.connection;
    let Ok((mut send, mut recv)) = accept_frame_stream(&connection).await else {
        return;
    };
    let mut data_streams = Vec::new();
    while let Ok(bytes) = read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN).await {
        let Ok(request) = serde_json::from_slice::<WireRequest>(&bytes) else {
            break;
        };
        let reply = match request {
            WireRequest::Catalog => WireResponse::Catalog(catalog()),
            WireRequest::Demand { topic, .. } => {
                let Ok(mut stream) = open_uni_frame_stream(&connection).await else {
                    break;
                };
                let preamble = serde_json::to_vec(&StreamPreamble {
                    topic: topic.clone(),
                })
                .unwrap();
                if write_frame(&mut stream, &preamble).await.is_err() {
                    break;
                }
                data_streams.push(stream);
                WireResponse::DemandAccepted { topic }
            }
            WireRequest::Schema { topic } => {
                WireResponse::Schema(SchemaReply::not_found(ROBOT, &topic, "no schema"))
            }
            WireRequest::Undemand { topic } => WireResponse::Undemanded {
                topic,
                was_demanded: false,
            },
            WireRequest::Status | WireRequest::SyncEpoch { .. } => WireResponse::Error {
                topic: None,
                message: "this test robot does not serve that request".to_string(),
            },
        };
        let reply = serde_json::to_vec(&reply).unwrap();
        if write_frame(&mut send, &reply).await.is_err() {
            break;
        }
    }
    let _ = connection.closed().await;
    drop(data_streams);
}

/// The test robot: its endpoint id, its loopback address, and the runtime
/// it serves on, which ends it when dropped.
struct TestRobot {
    eid: String,
    addr: String,
    _runtime: tokio::runtime::Runtime,
}

impl TestRobot {
    /// Bind the robot with the key `[seed; 32]` and start serving.
    fn start(seed: u8) -> Self {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let endpoint = runtime
            .block_on(build_endpoint(
                EndpointConfig::new([seed; 32])
                    .with_relay(RelayConfig::Disabled)
                    .with_alpns(vec![alpn::WIRE.to_vec()]),
            ))
            .expect("bind the test robot");
        let port = endpoint
            .bound_sockets()
            .into_iter()
            .find(SocketAddr::is_ipv4)
            .map(|socket| socket.port())
            .expect("the test robot's IPv4 port");
        let eid = hex::encode(endpoint.id().as_bytes());
        runtime.spawn(serve(endpoint));
        TestRobot {
            eid,
            addr: format!("127.0.0.1:{port}"),
            _runtime: runtime,
        }
    }
}

/// The `cerulion` built beside the helper this test runs.
fn cerulion() -> PathBuf {
    let bin = Path::new(env!("CARGO_BIN_EXE_cerulion-connectd")).with_file_name("cerulion");
    assert!(
        bin.is_file(),
        "{} is missing: build it first with `cargo build -p cerulion_cli`",
        bin.display()
    );
    bin
}

/// A child that is killed and reaped when dropped, so a failed assertion
/// leaves no process behind.
struct KillOnDrop(Child);

impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Read `pipe` to its end on a thread, line by line, into the returned
/// channel.
fn lines_of(pipe: impl Read + Send + 'static) -> mpsc::Receiver<String> {
    let (tx, lines) = mpsc::channel();
    std::thread::spawn(move || {
        for line in BufReader::new(pipe).lines() {
            let Ok(line) = line else { break };
            if tx.send(line).is_err() {
                break;
            }
        }
    });
    lines
}

/// Append to `log` the lines of `lines` until its pipe ends or `limit`
/// passes.
fn drain(lines: &mpsc::Receiver<String>, log: &mut Vec<String>, limit: Duration) {
    let until = Instant::now() + limit;
    while let Ok(line) = lines.recv_timeout(until.saturating_duration_since(Instant::now())) {
        log.push(line);
    }
}

/// What one run left: its exit status (`None` when it had not exited
/// [`WAIT_LIMIT`] after the moment measured from), the time from that moment
/// to the reaped exit, its stdout and its stderr.
struct Ended {
    status: Option<ExitStatus>,
    elapsed: Duration,
    stdout: String,
    stderr: String,
}

/// One run demanding [`TOPIC`] from a test robot: stdin a pipe the test
/// holds, stdout and stderr read line by line on threads.
struct Run {
    child: KillOnDrop,
    stdin: Option<ChildStdin>,
    stdout: mpsc::Receiver<String>,
    stderr: mpsc::Receiver<String>,
    log: Vec<String>,
    _dir: tempfile::TempDir,
}

impl Run {
    /// `cerulion-connectd connect`.
    fn helper(robot: &TestRobot) -> Self {
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-connectd"));
        command.arg("connect");
        Self::start(command, robot)
    }

    /// `cerulion connect`, which spawns the helper this test runs.
    fn cli(robot: &TestRobot) -> Self {
        let mut command = Command::new(cerulion());
        command
            .arg("connect")
            .env(
                "CERULION_CONNECTD_BIN",
                env!("CARGO_BIN_EXE_cerulion-connectd"),
            )
            .env("CERULION_LOGIN_GATE", "off")
            .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
            .env("DO_NOT_TRACK", "1")
            .env_remove("POSTHOG_API_KEY")
            .env_remove("CERULION_NETD_NETWORK");
        Self::start(command, robot)
    }

    fn start(mut command: Command, robot: &TestRobot) -> Self {
        let dir = tempfile::tempdir().unwrap();
        command
            .args(["--eid", &robot.eid, "--addr", &robot.addr])
            .args(["--topic", TOPIC, "--relay-disabled"])
            .current_dir(dir.path())
            .env("CERULION_HOME", dir.path().join("home"))
            .env("NO_COLOR", "1")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_RELAY_URL")
            .env_remove("RUST_LOG")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = KillOnDrop(command.spawn().expect("spawn the connect run"));
        let stdin = child.0.stdin.take();
        let stdout = lines_of(child.0.stdout.take().expect("the stdout pipe"));
        let stderr = lines_of(child.0.stderr.take().expect("the stderr pipe"));
        Run {
            child,
            stdin,
            stdout,
            stderr,
            log: Vec::new(),
            _dir: dir,
        }
    }

    /// Read stderr until a line contains `needle`.
    fn wait_for_line(&mut self, needle: &str) {
        let until = Instant::now() + READY_LIMIT;
        while !self.log.iter().any(|line| line.contains(needle)) {
            let left = until.saturating_duration_since(Instant::now());
            match self.stderr.recv_timeout(left) {
                Ok(line) => self.log.push(line),
                Err(_) => panic!(
                    "no `{needle}` line within {READY_LIMIT:?}; stderr:\n{}",
                    self.log.join("\n")
                ),
            }
        }
    }

    /// Send `signal` (a `kill -s` name) and return the moment it was sent.
    fn signal(&self, signal: &str) -> Instant {
        let sent = Instant::now();
        let kill = Command::new("kill")
            .args(["-s", signal, &self.child.0.id().to_string()])
            .status()
            .expect("run kill");
        assert!(kill.success(), "kill -s {signal} failed: {kill:?}");
        sent
    }

    /// Wait for the exit, measured from `since`, then reap the run and read
    /// the rest of both pipes. Stdin stays open until the run is reaped,
    /// unless the test closed it.
    fn finish(mut self, since: Instant) -> Ended {
        let until = since + WAIT_LIMIT;
        let status = loop {
            if let Some(status) = self.child.0.try_wait().expect("poll the connect run") {
                break Some(status);
            }
            if Instant::now() >= until {
                break None;
            }
            std::thread::sleep(Duration::from_millis(10));
        };
        let elapsed = since.elapsed();
        drop(self.child);
        drop(self.stdin);
        drain(&self.stderr, &mut self.log, DRAIN_LIMIT);
        let mut stdout = Vec::new();
        drain(&self.stdout, &mut stdout, DRAIN_LIMIT);
        Ended {
            status,
            elapsed,
            stdout: stdout.join("\n"),
            stderr: self.log.join("\n"),
        }
    }
}

/// The run exited 0 within [`EXIT_CEILING`] of the moment measured from,
/// its helper having logged `caught` and not `not_caught`, and torn the
/// session down, with exactly the catalog on stdout.
fn assert_stopped(what: &str, run: &Ended, caught: &str, not_caught: &str) {
    let stderr = &run.stderr;
    let Some(status) = run.status else {
        panic!("{what}: still running {WAIT_LIMIT:?} later; stderr:\n{stderr}");
    };
    assert_eq!(
        status.code(),
        Some(0),
        "{what}: {status:?} after {:?}; stderr:\n{stderr}",
        run.elapsed
    );
    assert!(
        run.elapsed <= EXIT_CEILING,
        "{what}: the exit took {:?}, over {EXIT_CEILING:?}; stderr:\n{stderr}",
        run.elapsed
    );
    assert!(stderr.contains(caught), "{what}: stderr:\n{stderr}");
    assert!(!stderr.contains(not_caught), "{what}: stderr:\n{stderr}");
    assert!(stderr.contains(TEARDOWN), "{what}: stderr:\n{stderr}");
    assert_eq!(run.stdout, catalog_stdout(), "{what}: stderr:\n{stderr}");
}

#[test]
fn sigint_to_the_helper_ends_the_session_with_exit_0_promptly_with_stdin_open() {
    let robot = TestRobot::start(0x61);
    let mut run = Run::helper(&robot);
    run.wait_for_line(READY);
    let sent = run.signal("INT");
    let ended = run.finish(sent);
    assert_stopped(
        "SIGINT to cerulion-connectd connect",
        &ended,
        CTRL_C,
        STDIN_CLOSED,
    );
}

#[test]
fn closing_stdin_ends_the_session_with_exit_0_promptly() {
    let robot = TestRobot::start(0x62);
    let mut run = Run::helper(&robot);
    run.wait_for_line(READY);
    let closed = Instant::now();
    drop(run.stdin.take());
    let ended = run.finish(closed);
    assert_stopped("a closed stdin", &ended, STDIN_CLOSED, CTRL_C);
}

/// `signal` to `cerulion connect` alone, once the helper streams, ends the
/// run with exit 0 within [`EXIT_CEILING`], the helper having caught the
/// forwarded SIGINT.
fn assert_cli_stopped_by(signal: &str, seed: u8) {
    let robot = TestRobot::start(seed);
    let mut run = Run::cli(&robot);
    run.wait_for_line(READY);
    let sent = run.signal(signal);
    let ended = run.finish(sent);
    let what = format!("SIG{signal} to cerulion connect");
    assert_stopped(&what, &ended, CTRL_C, STDIN_CLOSED);
}

#[test]
fn sigint_to_cerulion_connect_ends_the_run_with_exit_0_promptly_with_stdin_open() {
    assert_cli_stopped_by("INT", 0x63);
}

#[test]
fn sigterm_to_cerulion_connect_ends_the_run_with_exit_0_promptly_with_stdin_open() {
    assert_cli_stopped_by("TERM", 0x64);
}
