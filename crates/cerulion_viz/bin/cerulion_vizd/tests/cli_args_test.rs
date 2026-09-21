// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-vizd` argument handling over the REAL binary.
//!
//! Previously the daemon ignored ALL argv and started serving unconditionally, so
//! `cerulion-vizd --help` wedged the terminal on a long-lived daemon. These pins
//! spawn the actual binary (`CARGO_BIN_EXE_cerulion-vizd`) with `--help`/`-h` and
//! an unknown flag and assert the exit code + output WITHOUT ever starting the
//! daemon (a bounded wait — a regression that starts the daemon would hang, caught
//! by the wait timeout).

use std::io::Read;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// Run `cerulion-vizd <args>` with a hard timeout, returning `(exit_code, stdout,
/// stderr)`. A `--help`/`-h` (or unknown-arg) run exits immediately; a REGRESSION
/// that starts the daemon would block forever, so we SIGKILL past the bound and
/// report a sentinel exit code that fails the assert loudly.
fn run_vizd(args: &[&str]) -> (i32, String, String) {
    let bin = env!("CARGO_BIN_EXE_cerulion-vizd");
    let mut child = Command::new(bin)
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn cerulion-vizd");

    let deadline = Instant::now() + Duration::from_secs(15);
    let status = loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if Instant::now() >= deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!(
                    "`cerulion-vizd {args:?}` did not exit within 15s — it likely started the \
                     daemon instead of handling the argument (an ignore-all-args regression)"
                );
            }
            None => std::thread::sleep(Duration::from_millis(25)),
        }
    };

    let mut stdout = String::new();
    let mut stderr = String::new();
    if let Some(mut o) = child.stdout.take() {
        let _ = o.read_to_string(&mut stdout);
    }
    if let Some(mut e) = child.stderr.take() {
        let _ = e.read_to_string(&mut stderr);
    }
    (status.code().unwrap_or(-1), stdout, stderr)
}

#[test]
fn help_flag_prints_usage_and_exits_zero() {
    let (code, stdout, _stderr) = run_vizd(&["--help"]);
    assert_eq!(code, 0, "--help exits 0. stdout:\n{stdout}");
    assert!(
        stdout.contains("USAGE") && stdout.contains("cerulion-vizd"),
        "--help prints usage to stdout:\n{stdout}"
    );
    // The usage documents the new remote-arm env vars (the wired locators).
    assert!(
        stdout.contains("CERULION_VIZD_CONNECT") && stdout.contains("CERULION_VIZD_LISTEN"),
        "--help documents the locator env vars:\n{stdout}"
    );
}

#[test]
fn short_help_flag_also_exits_zero_with_usage() {
    let (code, stdout, _stderr) = run_vizd(&["-h"]);
    assert_eq!(code, 0, "-h exits 0. stdout:\n{stdout}");
    assert!(stdout.contains("USAGE"), "-h prints usage:\n{stdout}");
}

#[test]
fn an_unknown_arg_exits_nonzero_with_usage_on_stderr() {
    let (code, _stdout, stderr) = run_vizd(&["--bogus-flag"]);
    assert_ne!(code, 0, "an unknown arg exits nonzero. stderr:\n{stderr}");
    assert!(
        stderr.contains("unrecognized argument") && stderr.contains("--bogus-flag"),
        "an unknown arg names the offender + prints usage to stderr:\n{stderr}"
    );
}
