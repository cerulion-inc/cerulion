// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion pair` over the real binary, interrupted at the pairing-code read
//! of the real `cerulion-connectd pair` it spawns, with stdin a pipe this test
//! holds open and writes nothing to. SIGINT and SIGTERM sent to `cerulion`
//! alone (it forwards SIGINT to the helper) each end the run with exit 130
//! (the documented `interrupted` code) within [`EXIT_CEILING`] of the signal,
//! well inside the 5 s grace after which `cerulion` kills the helper (exit
//! 137), with nothing on stdout (no ceremony began).
//!
//! The signal is sent once stderr carries the helper's desk key line, which
//! it logs after its signal handlers are in place. Each run has its own
//! signed-in home and key file, and no code is ever written, so nothing is
//! dialed.
//!
//! Prerequisite: `cargo build -p cerulion_connectd` in the same target and
//! profile, which puts `cerulion-connectd` beside `cerulion`.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Read};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// A well-formed endpoint id no robot in this test owns.
const EID: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";

/// The stderr line the helper logs once its signal handlers are in place.
const READY: &str = "cerulion pair: created a new desk device key (0600)";

/// How long the run may take to log [`READY`].
const READY_LIMIT: Duration = Duration::from_secs(30);

/// The most the time from the signal to the reaped exit may be.
const EXIT_CEILING: Duration = Duration::from_secs(2);

/// How long the test waits for the exit before it kills the run: past the
/// 5 s grace, so a helper that never exits shows as the 137 it becomes.
const WAIT_LIMIT: Duration = Duration::from_secs(15);

/// The helper `cerulion pair` spawns: the one built beside this `cerulion`.
fn connectd() -> PathBuf {
    let bin = Path::new(env!("CARGO_BIN_EXE_cerulion")).with_file_name("cerulion-connectd");
    assert!(
        bin.is_file(),
        "{} is missing: build it first with `cargo build -p cerulion_connectd`",
        bin.display()
    );
    bin
}

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

/// What one interrupted run left: its exit status (`None` when it had not
/// exited [`WAIT_LIMIT`] after the signal), the time from the signal to the
/// reaped exit, its stdout and its stderr.
struct Interrupted {
    status: Option<ExitStatus>,
    elapsed: Duration,
    stdout: String,
    stderr: String,
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

/// Poll `child` until it exits or `limit` passes.
fn wait_bounded(child: &mut Child, limit: Duration) -> Option<ExitStatus> {
    let until = Instant::now() + limit;
    loop {
        if let Some(status) = child.try_wait().expect("poll the pair run") {
            return Some(status);
        }
        if Instant::now() >= until {
            return None;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Start `cerulion pair` with no `--code`, wait for [`READY`], send `signal`
/// (a `kill -s` name) to `cerulion` and wait for its exit, with stdin held
/// open throughout.
fn interrupt_at_the_code_read(signal: &str) -> Interrupted {
    let dir = tempfile::tempdir().unwrap();
    let home = dir.path().join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "pair-interrupt-cli-test").unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    command
        .args(["pair", "--eid", EID, "--relay-disabled"])
        .args(["--addr", "127.0.0.1:9"])
        .arg("--key-file")
        .arg(dir.path().join("desk.key"))
        .args(["--label", "pair-interrupt-cli-test"])
        .current_dir(dir.path())
        .env("CERULION_HOME", &home)
        .env("CERULION_CONNECTD_BIN", connectd())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env("DO_NOT_TRACK", "1")
        .env("NO_COLOR", "1")
        .env_remove("POSTHOG_API_KEY")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_RELAY_URL")
        .env_remove("RUST_LOG")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut run = KillOnDrop(command.spawn().expect("spawn cerulion pair"));
    // Held until the run is reaped: the open pipe keeps the code read parked.
    let stdin = run.0.stdin.take().expect("the stdin pipe");
    let stdout = lines_of(run.0.stdout.take().expect("the stdout pipe"));
    let lines = lines_of(run.0.stderr.take().expect("the stderr pipe"));

    let mut log: Vec<String> = Vec::new();
    let ready_by = Instant::now() + READY_LIMIT;
    while !log.iter().any(|line| line.contains(READY)) {
        match lines.recv_timeout(ready_by.saturating_duration_since(Instant::now())) {
            Ok(line) => log.push(line),
            Err(_) => panic!(
                "no `{READY}` line within {READY_LIMIT:?}; stderr:\n{}",
                log.join("\n")
            ),
        }
    }

    let sent = Instant::now();
    let kill = Command::new("kill")
        .args(["-s", signal, &run.0.id().to_string()])
        .status()
        .expect("run kill");
    assert!(kill.success(), "kill -s {signal} failed: {kill:?}");
    let status = wait_bounded(&mut run.0, WAIT_LIMIT);
    let elapsed = sent.elapsed();
    drop(run);
    drop(stdin);
    drain(&lines, &mut log, DRAIN_LIMIT);
    let mut out = Vec::new();
    drain(&stdout, &mut out, DRAIN_LIMIT);
    Interrupted {
        status,
        elapsed,
        stdout: out.join("\n"),
        stderr: log.join("\n"),
    }
}

/// `signal` to `cerulion` at the helper's code read ends the run with exit
/// 130 within [`EXIT_CEILING`], the helper having caught the forwarded
/// SIGINT, with nothing on stdout.
fn assert_interrupted(signal: &str) {
    let run = interrupt_at_the_code_read(signal);
    let stderr = &run.stderr;
    let Some(status) = run.status else {
        panic!(
            "SIG{signal}: still running {WAIT_LIMIT:?} after the signal, with stdin open; \
             stderr:\n{stderr}"
        );
    };
    assert_eq!(
        status.code(),
        Some(130),
        "SIG{signal}: {status:?} after {:?}; stderr:\n{stderr}",
        run.elapsed
    );
    assert!(
        run.elapsed <= EXIT_CEILING,
        "SIG{signal}: the exit took {:?}, over {EXIT_CEILING:?}; stderr:\n{stderr}",
        run.elapsed
    );
    assert!(
        stderr.contains("cerulion pair: Ctrl-C received"),
        "SIG{signal}: stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("cerulion pair: interrupted (Ctrl-C) before completion"),
        "SIG{signal}: stderr:\n{stderr}"
    );
    assert_eq!(run.stdout, "", "SIG{signal}: stderr:\n{stderr}");
}

#[test]
fn sigint_at_the_code_read_exits_130_promptly_with_stdin_open() {
    assert_interrupted("INT");
}

#[test]
fn sigterm_at_the_code_read_exits_130_promptly_with_stdin_open() {
    assert_interrupted("TERM");
}
