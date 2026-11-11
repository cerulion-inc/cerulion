// SPDX-License-Identifier: AGPL-3.0-only
//! The stop of the real `cerulion-remoted` binary (`CARGO_BIN_EXE_cerulion-remoted`):
//!
//! - SIGTERM runs the graceful stop and exits 0, as Ctrl-C (SIGINT) does;
//! - a stop whose shutdown stage never finishes is ended by the hard-exit
//!   deadline with status 4, no sooner than the deadline;
//! - a second stop signal during the stop exits with status 4 long before the
//!   deadline, after its warn line;
//! - a participant that fails exits 3, after the rest of the teardown ran;
//! - with no signal, a stop the endpoint's own close started is still ended by
//!   the deadline with status 4;
//! - after the endpoint's own close, the first stop signal counts as a second
//!   one and exits with status 4, after its warn line;
//! - with stderr full and a stage that never finishes, the deadline still
//!   ends the stop with status 4, within a bound of the deadline plus 5 s;
//! - with stderr full, a graceful stop exits 0 long before the deadline: no
//!   log line holds it, and the exit waits at most its grace for the log
//!   queue;
//! - with stderr full, a graceful stop waits for its queued lines: it still
//!   runs a second after SIGTERM, and once the pipe drains it exits 0 with the
//!   teardown's line written.
//!
//! The stage's participant is the test seam's probe, which logs whether the
//! rcpd e-stop sender was already stopped when the stage ran it. A test that
//! asserts a line the exit waits for sets the exit's wait for the log queue
//! to 5 s through the test seam (the exit goes on once the line is written);
//! the other full-stderr tests keep the 100 ms a build without the test seam
//! waits.
//!
//! Each daemon serves its own temp state root and device key, relays disabled;
//! no peer connects, so the lazy wire plane never starts iceoryx2. The child
//! inherits the test runner's environment, except the variables
//! `Daemon::start_with` removes, and gets the ones each test sets.
#![cfg(unix)]

mod common;

use std::io::{PipeReader, PipeWriter, Read};
use std::path::Path;
use std::process::{Command, ExitStatus, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::log_writer::QUEUE_LINES_FOR_TEST_ENV as QUEUE_ENV;
use cerulion_remoted::shutdown::{
    CLOSE_ENDPOINT_FOR_TEST_ENV as CLOSE_ENV, EXIT_LOG_GRACE_FOR_TEST_ENV as GRACE_ENV,
    PROBE_FOR_TEST_ENV as PROBE_ENV,
};
use common::stderr_pipe::{fill, pump, read_matching, KillOnDrop};

/// The stop deadline variable, as docs/user-api.md names it.
const HARD_EXIT_ENV: &str = "CERULION_REMOTED_HARD_EXIT_MS";
/// The exit's wait for its line in the tests that assert that line.
const GRACE_MS: &str = "5000";
const MAC_KEY: &[u8] = b"stop-signal-mac-key";
const CHASSIS: &[u8] = b"stop-signal-chassis-secret-not-serial-derived";
const T_NOW: u64 = 1_000_000_000_000;
/// How long a daemon may take to log that it listens.
const START_BOUND: Duration = Duration::from_secs(60);
/// How long a stop may take before the test fails.
const STOP_BOUND: Duration = Duration::from_secs(20);
const LISTENING: &str = "cerulion_remoted: listening";
const STOP_RECEIVED: &str = "cerulion_remoted: stop signal received";
const ACCEPT_LOOP_STOPPED: &str = "shutdown signalled; stopping accept loop";
const PROBE_RAN: &str = "the shutdown probe runs";
const RCPD_NOT_STOPPED: &str = "rcpd_stopped=Some(false)";
const TEARDOWN_RAN: &str = "the rcpd e-stop sender and the epoch poller are stopped";
const ENDPOINT_CLOSED: &str = "endpoint closed; stopping accept loop";

/// A provisioned, claimed state root whose trust store is bound to `secret`'s
/// public key, the endpoint id the daemon binds.
fn write_state(root: &Path, secret: [u8; 32]) {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::write(remoted.join("device_key"), secret).unwrap();
    std::fs::write(remoted.join("trust_store.mac_key"), MAC_KEY).unwrap();
    let transport_key = ed25519_dalek::SigningKey::from_bytes(&secret)
        .verifying_key()
        .to_bytes();
    let roots = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey(transport_key),
        roots,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(AccountId([10; 32]), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();
}

/// A running daemon and the stderr lines read from it; killed and reaped on drop.
struct Daemon {
    child: KillOnDrop,
    lines: mpsc::Receiver<String>,
    log: Vec<String>,
    /// While held, the stderr reader reads nothing past the listening line.
    hold: Option<mpsc::Sender<()>>,
    _root: tempfile::TempDir,
}

impl Daemon {
    /// Start a daemon on the device key `[seed; 32]` with `env`, and wait until
    /// it listens.
    fn start(seed: u8, env: &[(&str, &str)]) -> Daemon {
        Daemon::start_with(seed, env, None)
    }

    /// [`Daemon::start`] with stderr on a pipe the test reads no further than
    /// the listening line, and a second write end of that pipe.
    fn start_holding(seed: u8, env: &[(&str, &str)]) -> (Daemon, PipeWriter) {
        let (reader, writer) = std::io::pipe().unwrap();
        let filler = writer.try_clone().unwrap();
        (
            Daemon::start_with(seed, env, Some((reader, writer))),
            filler,
        )
    }

    fn start_with(
        seed: u8,
        env: &[(&str, &str)],
        pipe: Option<(PipeReader, PipeWriter)>,
    ) -> Daemon {
        let root = tempfile::tempdir().unwrap();
        write_state(root.path(), [seed; 32]);
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-remoted"));
        command
            .arg("--state-root")
            .arg(root.path())
            .arg("--relay-disabled")
            .env("NO_COLOR", "1")
            .env("RUST_LOG", "info")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_STATE_ROOT")
            .env_remove("CERULION_RELAY_URL")
            .env_remove(HARD_EXIT_ENV)
            .env_remove(PROBE_ENV)
            .env_remove(CLOSE_ENV)
            .env_remove(GRACE_ENV)
            .env_remove(QUEUE_ENV)
            .envs(env.iter().copied())
            .stdin(Stdio::null())
            .stdout(Stdio::null());
        let (stderr, hold): (Box<dyn Read + Send>, _) = match pipe {
            Some((reader, writer)) => {
                command.stderr(writer);
                let (hold, held) = mpsc::channel();
                (Box::new(reader), Some((hold, held)))
            }
            None => {
                command.stderr(Stdio::piped());
                (Box::new(std::io::empty()), None)
            }
        };
        let mut child = KillOnDrop(command.spawn().expect("spawn cerulion-remoted"));
        // The command's copy of a pipe's write end closes here.
        drop(command);
        let stderr: Box<dyn Read + Send> = match child.0.stderr.take() {
            Some(piped) => Box::new(piped),
            None => stderr,
        };
        let (sender, lines) = mpsc::channel();
        let (hold, held) = hold.unzip();
        std::thread::spawn(move || pump(stderr, sender, held, LISTENING));
        let mut daemon = Daemon {
            child,
            lines,
            log: Vec::new(),
            hold,
            _root: root,
        };
        daemon.wait_for_line(LISTENING, START_BOUND);
        daemon
    }

    /// The first stderr line containing `needle`, read within `bound`.
    fn wait_for_line(&mut self, needle: &str, bound: Duration) -> String {
        read_matching(&self.lines, &mut self.log, 0, bound, |l| l.contains(needle))
    }

    /// Send `signal`; returns the instant just before it was sent.
    fn signal(&self, signal: libc::c_int) -> Instant {
        let pid = libc::pid_t::try_from(self.child.0.id()).unwrap();
        let sent_at = Instant::now();
        // SAFETY: kill(2) with a pid this test spawned and a valid signal number;
        // no memory is shared with the call.
        let sent = unsafe { libc::kill(pid, signal) };
        assert_eq!(sent, 0, "kill: {}", std::io::Error::last_os_error());
        sent_at
    }

    /// The exit status within `bound` of `since`, and the time from `since` to
    /// the exit; then the rest of stderr is read into the log.
    fn wait_exit(&mut self, since: Instant, bound: Duration) -> (ExitStatus, Duration) {
        let status = loop {
            if let Some(status) = self.child.0.try_wait().unwrap() {
                break status;
            }
            assert!(
                since.elapsed() < bound,
                "still running {bound:?} after the signal; log:\n{}",
                self.dump()
            );
            std::thread::sleep(Duration::from_millis(20));
        };
        let took = since.elapsed();
        self.hold = None;
        // The pipe closes with the process; the reader thread then ends.
        while let Ok(line) = self.lines.recv_timeout(Duration::from_secs(5)) {
            self.log.push(line);
        }
        (status, took)
    }

    fn dump(&self) -> String {
        self.log.join("\n")
    }

    /// The log line containing every one of `needles`.
    fn line_with(&self, needles: &[&str]) -> Option<&String> {
        self.line_index(needles).map(|i| &self.log[i])
    }

    /// The index of the first log line containing every one of `needles`.
    fn line_index(&self, needles: &[&str]) -> Option<usize> {
        self.log
            .iter()
            .position(|line| needles.iter().all(|n| line.contains(n)))
    }
}

/// A stop by `signal`: exit 0 with the accept loop stopped, and the stop line
/// names `name`.
fn assert_graceful_stop(seed: u8, signal: libc::c_int, name: &str) {
    let mut daemon = Daemon::start(seed, &[]);
    let sent = daemon.signal(signal);
    let (status, _) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(0),
        "{name} must stop the daemon with exit 0, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[ACCEPT_LOOP_STOPPED]).is_some(),
        "the graceful path stopped the accept loop; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[STOP_RECEIVED, name]).is_some(),
        "one stop line names {name}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[TEARDOWN_RAN]).is_some(),
        "the teardown after the stage ran; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn sigterm_runs_the_graceful_stop_and_exits_0() {
    assert_graceful_stop(0x51, libc::SIGTERM, "SIGTERM");
}

#[test]
fn ctrl_c_still_runs_the_graceful_stop_and_exits_0() {
    assert_graceful_stop(0x52, libc::SIGINT, "SIGINT");
}

#[test]
fn the_hard_exit_deadline_ends_a_stop_that_cannot_finish_with_4() {
    let deadline = Duration::from_millis(3000);
    let mut daemon = Daemon::start(
        0x53,
        &[
            (HARD_EXIT_ENV, "3000"),
            (PROBE_ENV, "stall"),
            (GRACE_ENV, GRACE_MS),
        ],
    );
    let sent = daemon.signal(libc::SIGTERM);
    let (status, took) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(4),
        "the deadline ends the stalled stop with 4, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        took >= deadline,
        "exited after {took:?}, before the deadline"
    );
    assert!(
        daemon.line_with(&[PROBE_RAN, RCPD_NOT_STOPPED]).is_some(),
        "the stage ran, before the rcpd e-stop sender was stopped; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&[
                "did not finish within the hard-exit deadline",
                "deadline_ms=3000 "
            ])
            .is_some(),
        "one error line names the deadline; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn a_second_stop_signal_during_the_stop_exits_with_4() {
    let mut daemon = Daemon::start(
        0x54,
        &[
            (HARD_EXIT_ENV, "600000"),
            (PROBE_ENV, "stall"),
            (GRACE_ENV, GRACE_MS),
        ],
    );
    daemon.signal(libc::SIGTERM);
    daemon.wait_for_line(PROBE_RAN, STOP_BOUND);
    let sent = daemon.signal(libc::SIGINT);
    let (status, _) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(4),
        "a second SIGINT exits with 4 long before the deadline, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&["a second stop signal during the stop", "SIGINT"])
            .is_some(),
        "one warn line names the second signal; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn without_a_signal_the_deadline_ends_a_stop_the_endpoint_close_started_with_4() {
    // Before the process exists, so before the deadline can arm.
    let t0 = Instant::now();
    let mut daemon = Daemon::start(
        0x57,
        &[
            (HARD_EXIT_ENV, "3000"),
            (PROBE_ENV, "stall"),
            (CLOSE_ENV, "1"),
            (GRACE_ENV, GRACE_MS),
        ],
    );
    // From the listening line: the 3 s deadline and at most the exit's 5 s
    // wait for its line.
    let started = Instant::now();
    let (status, took) = daemon.wait_exit(started, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(4),
        "the deadline ends the stalled stop with 4, got {status:?}; log:\n{}",
        daemon.dump()
    );
    let since_spawn = took + (started - t0);
    assert!(
        since_spawn >= Duration::from_millis(3000),
        "exited {since_spawn:?} after the spawn, before the 3000 ms hard-exit deadline"
    );
    assert!(
        daemon.line_with(&[ENDPOINT_CLOSED]).is_some(),
        "the endpoint's own close stopped the accept loop; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[PROBE_RAN]).is_some(),
        "the stage ran; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&[
                "did not finish within the hard-exit deadline",
                "deadline_ms=3000 "
            ])
            .is_some(),
        "the deadline, not a signal, ended the stop; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[STOP_RECEIVED]).is_none(),
        "no stop signal was sent; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn after_the_endpoint_closed_on_its_own_the_first_stop_signal_exits_4() {
    let mut daemon = Daemon::start(
        0x58,
        &[
            (HARD_EXIT_ENV, "600000"),
            (PROBE_ENV, "stall"),
            (CLOSE_ENV, "1"),
            (GRACE_ENV, GRACE_MS),
        ],
    );
    daemon.wait_for_line(PROBE_RAN, STOP_BOUND);
    let sent = daemon.signal(libc::SIGTERM);
    let (status, _) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(4),
        "the first SIGTERM after the endpoint's own close exits with 4, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&["a second stop signal during the stop", "signal=\"SIGTERM\""])
            .is_some(),
        "the warn line of a second signal names the SIGTERM; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[STOP_RECEIVED]).is_none(),
        "no stop request reached serving; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&["did not finish within the hard-exit deadline"])
            .is_none(),
        "the signal, not the deadline, ended the stop; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn a_participant_that_fails_exits_3_after_the_teardown() {
    let mut daemon = Daemon::start(0x55, &[(PROBE_ENV, "fail")]);
    let sent = daemon.signal(libc::SIGTERM);
    let (status, _) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(3),
        "a failed participant exits 3, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[PROBE_RAN, RCPD_NOT_STOPPED]).is_some(),
        "the stage ran, before the rcpd e-stop sender was stopped; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon
            .line_with(&[
                "a shutdown participant did not complete",
                "shutdown-probe",
                "failed: the shutdown probe failed"
            ])
            .is_some(),
        "one error line names the participant and how it failed; log:\n{}",
        daemon.dump()
    );
    let probe = daemon.line_index(&[PROBE_RAN]);
    let teardown = daemon.line_index(&[TEARDOWN_RAN]);
    assert!(
        matches!((probe, teardown), (Some(p), Some(t)) if t > p),
        "the teardown ran after the failed stage; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn the_hard_exit_does_not_wait_for_a_full_stderr() {
    let deadline = Duration::from_millis(700);
    // The exit's wait for its line, then process exit and the 20 ms poll on a
    // loaded host.
    let bound = deadline + Duration::from_millis(100) + Duration::from_secs(5);
    let (mut daemon, filler) =
        Daemon::start_holding(0x56, &[(HARD_EXIT_ENV, "700"), (PROBE_ENV, "stall")]);
    fill(&filler);
    drop(filler);
    let sent = daemon.signal(libc::SIGTERM);
    let (status, took) = daemon.wait_exit(sent, bound);
    assert_eq!(
        status.code(),
        Some(4),
        "the deadline ends the stop with 4 while stderr is full, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        took >= deadline,
        "exited after {took:?}, before the deadline"
    );
    assert!(
        daemon
            .line_with(&["did not finish within the hard-exit deadline"])
            .is_none(),
        "the deadline's line reached stderr, so stderr was not full at the exit; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn a_graceful_stop_exits_0_while_stderr_is_full() {
    // Far past the bound, so a stop held by a log line ends at the deadline
    // with 4 and fails here.
    let (mut daemon, filler) = Daemon::start_holding(0x59, &[(HARD_EXIT_ENV, "600000")]);
    fill(&filler);
    drop(filler);
    let sent = daemon.signal(libc::SIGTERM);
    let (status, took) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(0),
        "the graceful stop exits 0 while stderr is full, got {status:?} after {took:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        took < Duration::from_secs(5),
        "the graceful stop took {took:?} while stderr was full"
    );
}

#[test]
fn a_graceful_stop_waits_for_its_queued_lines_while_stderr_is_full() {
    let (mut daemon, filler) =
        Daemon::start_holding(0x5a, &[(HARD_EXIT_ENV, "600000"), (GRACE_ENV, GRACE_MS)]);
    fill(&filler);
    drop(filler);
    let sent = daemon.signal(libc::SIGTERM);
    // The stop itself takes milliseconds; the exit then waits up to four
    // fifths of its 5 s grace for the lines queued behind the full pipe.
    std::thread::sleep(Duration::from_secs(1));
    assert!(
        daemon.child.0.try_wait().unwrap().is_none(),
        "exited with lines still queued behind a full stderr; log:\n{}",
        daemon.dump()
    );
    daemon.hold = None;
    let (status, _) = daemon.wait_exit(sent, STOP_BOUND);
    assert_eq!(
        status.code(),
        Some(0),
        "the graceful stop exits 0 once its lines are written, got {status:?}; log:\n{}",
        daemon.dump()
    );
    assert!(
        daemon.line_with(&[TEARDOWN_RAN]).is_some(),
        "the teardown's line, queued behind the full pipe, was written; log:\n{}",
        daemon.dump()
    );
}
