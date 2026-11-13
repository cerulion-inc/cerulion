// SPDX-License-Identifier: AGPL-3.0-only
//! The real `cerulion-connectd pair`, interrupted while it waits, with stdin a
//! pipe this test holds open.
//!
//! - At the pairing-code read (nothing written, so the read stays parked):
//!   SIGINT and SIGTERM each end the run with exit 130 (the documented
//!   `interrupted` code) within [`EXIT_CEILING`] of the signal, log which
//!   signal they caught and print nothing on stdout (no ceremony began).
//! - In the ceremony: a code written on the open pipe reaches a test robot
//!   that accepts the ops stream and never answers, so the ceremony waits on
//!   its blocking read; SIGINT ends the run with exit 130 within
//!   [`EXIT_CEILING`], and stdout carries exactly the `pairing:` line.
//! - A stdin closed with no code is the usage error (exit 1), with nothing on
//!   stdout.
//!
//! A signal is sent once the run is observably waiting: at the code read, once
//! stderr carries the desk key line, which the run logs after its signal
//! handlers are in place; in the ceremony, once the test robot has accepted
//! the stream the run opened. Each run has its own temporary home and key file.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{SocketAddr, UdpSocket};
use std::process::{Child, ChildStdin, Command, ExitStatus, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_link::{
    accept_frame_stream, accept_one, alpn, build_endpoint, EndpointConfig, RelayConfig,
};

/// A well-formed endpoint id no robot in this test owns.
const EID: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";

/// The stderr line the run logs once its signal handlers are in place.
const READY: &str = "cerulion pair: created a new desk device key (0600)";

/// The stderr lines of a run that caught SIGINT and stopped.
const CTRL_C: &str = "cerulion pair: Ctrl-C received";
const INTERRUPTED: &str = "cerulion pair: interrupted (Ctrl-C) before completion";

/// How long the run may take to reach the point a test waits for.
const READY_LIMIT: Duration = Duration::from_secs(30);

/// The most the time from the signal to the reaped exit may be.
const EXIT_CEILING: Duration = Duration::from_secs(2);

/// How long the test waits for the exit before it kills the run.
const WAIT_LIMIT: Duration = Duration::from_secs(10);

/// How long the test reads the pipes after the run is reaped.
const DRAIN_LIMIT: Duration = Duration::from_secs(5);

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

/// One `cerulion-connectd pair` run with no `--code`: stdin a pipe the test
/// holds, stdout and stderr read line by line on threads.
struct PairRun {
    child: KillOnDrop,
    stdin: Option<ChildStdin>,
    stdout: mpsc::Receiver<String>,
    stderr: mpsc::Receiver<String>,
    log: Vec<String>,
    _dir: tempfile::TempDir,
}

impl PairRun {
    /// Start `pair` toward `eid` at the direct address `addr`.
    fn start(eid: &str, addr: &str) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-connectd"));
        command
            .args(["pair", "--eid", eid, "--relay-disabled"])
            .args(["--addr", addr])
            .arg("--key-file")
            .arg(dir.path().join("desk.key"))
            .args(["--label", "pair-interrupt-test"])
            .env("CERULION_HOME", dir.path().join("home"))
            .env("NO_COLOR", "1")
            .env_remove("CERULION_RELAY_URL")
            .env_remove("RUST_LOG")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = KillOnDrop(command.spawn().expect("spawn cerulion-connectd pair"));
        let stdin = child.0.stdin.take();
        let stdout = lines_of(child.0.stdout.take().expect("the stdout pipe"));
        let stderr = lines_of(child.0.stderr.take().expect("the stderr pipe"));
        PairRun {
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
    /// the rest of both pipes. Stdin stays open until the run is reaped.
    fn finish(mut self, since: Instant) -> Ended {
        let until = since + WAIT_LIMIT;
        let status = loop {
            if let Some(status) = self.child.0.try_wait().expect("poll the pair run") {
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

/// The run exited with `code` within [`EXIT_CEILING`] of the moment measured
/// from.
fn assert_exit(what: &str, run: &Ended, code: i32) {
    let stderr = &run.stderr;
    let Some(status) = run.status else {
        panic!("{what}: still running {WAIT_LIMIT:?} later, with stdin open; stderr:\n{stderr}");
    };
    assert_eq!(
        status.code(),
        Some(code),
        "{what}: {status:?} after {:?}; stderr:\n{stderr}",
        run.elapsed
    );
    assert!(
        run.elapsed <= EXIT_CEILING,
        "{what}: the exit took {:?}, over {EXIT_CEILING:?}; stderr:\n{stderr}",
        run.elapsed
    );
}

/// A robot address that takes datagrams and answers none, held by the caller.
fn silent_address() -> (UdpSocket, String) {
    let socket = UdpSocket::bind("127.0.0.1:0").expect("bind the stand-in robot socket");
    let addr = socket.local_addr().unwrap().to_string();
    (socket, addr)
}

/// `signal` at the code read ends the run with exit 130 within
/// [`EXIT_CEILING`], having logged `caught`, with nothing on stdout.
fn assert_interrupted_at_the_code_read(signal: &str, caught: &str) {
    let (_robot, addr) = silent_address();
    let mut run = PairRun::start(EID, &addr);
    run.wait_for_line(READY);
    let sent = run.signal(signal);
    let ended = run.finish(sent);
    let what = format!("SIG{signal} at the code read");
    assert_exit(&what, &ended, 130);
    assert!(ended.stderr.contains(caught), "{what}: {}", ended.stderr);
    assert!(
        ended.stderr.contains(INTERRUPTED),
        "{what}: {}",
        ended.stderr
    );
    assert_eq!(ended.stdout, "", "{what}: {}", ended.stderr);
}

#[test]
fn sigint_at_the_code_read_exits_130_promptly_with_stdin_open() {
    assert_interrupted_at_the_code_read("INT", CTRL_C);
}

#[test]
fn sigterm_at_the_code_read_exits_130_promptly_with_stdin_open() {
    assert_interrupted_at_the_code_read("TERM", "cerulion pair: SIGTERM received");
}

#[test]
fn sigint_in_the_ceremony_with_a_silent_robot_exits_130_promptly_with_stdin_open() {
    let rt = tokio::runtime::Runtime::new().unwrap();
    let robot = rt
        .block_on(build_endpoint(
            EndpointConfig::new([0x5a; 32])
                .with_relay(RelayConfig::Disabled)
                .with_alpns(vec![alpn::OPS.to_vec()]),
        ))
        .expect("bind the test robot");
    let port = robot
        .bound_sockets()
        .into_iter()
        .find(SocketAddr::is_ipv4)
        .map(|socket| socket.port())
        .expect("the test robot's IPv4 port");
    let eid = hex::encode(robot.id().as_bytes());

    let mut run = PairRun::start(&eid, &format!("127.0.0.1:{port}"));
    run.wait_for_line(READY);
    let mut stdin = run.stdin.take().expect("the stdin pipe");
    stdin.write_all(b"12345678\n").unwrap();
    stdin.flush().unwrap();
    run.stdin = Some(stdin);
    // The run dials only once it has read the code, and it writes its ops
    // hello before its first read, so an accepted stream means it waits on
    // the robot's reply; the robot never sends one.
    let (connection, streams) = rt.block_on(async {
        let accepted = tokio::time::timeout(READY_LIMIT, accept_one(&robot))
            .await
            .expect("the run dials the robot")
            .expect("accept the run's connection")
            .expect("the robot endpoint is open");
        let opened = accept_frame_stream(&accepted.connection);
        let streams = tokio::time::timeout(READY_LIMIT, opened)
            .await
            .expect("the run opens its ops stream")
            .expect("accept the run's ops stream");
        (accepted.connection, streams)
    });
    let sent = run.signal("INT");
    let ended = run.finish(sent);
    drop(streams);
    drop(connection);
    rt.block_on(robot.close());

    let what = "SIGINT in the ceremony";
    assert_exit(what, &ended, 130);
    assert!(ended.stderr.contains(CTRL_C), "{what}: {}", ended.stderr);
    assert!(
        ended.stderr.contains(INTERRUPTED),
        "{what}: {}",
        ended.stderr
    );
    assert_eq!(
        ended.stdout,
        format!("pairing: robot={eid} eid={eid}"),
        "{what}: {}",
        ended.stderr
    );
}

#[test]
fn a_stdin_closed_with_no_code_is_a_usage_error() {
    let (_robot, addr) = silent_address();
    let mut run = PairRun::start(EID, &addr);
    drop(run.stdin.take());
    let ended = run.finish(Instant::now());
    let what = "a closed stdin";
    let Some(status) = ended.status else {
        panic!(
            "{what}: still running {WAIT_LIMIT:?} later; {}",
            ended.stderr
        );
    };
    assert_eq!(status.code(), Some(1), "{what}: {}", ended.stderr);
    assert!(
        ended
            .stderr
            .contains("cerulion pair: no pairing code provided (stdin closed)"),
        "{what}: {}",
        ended.stderr
    );
    assert!(
        !ended.stderr.contains(INTERRUPTED),
        "{what}: {}",
        ended.stderr
    );
    assert_eq!(ended.stdout, "", "{what}: {}", ended.stderr);
}
