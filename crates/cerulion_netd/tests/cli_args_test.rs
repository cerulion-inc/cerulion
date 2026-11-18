// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd` with an argument that is not valid UTF-8, over the REAL
//! binary (`CARGO_BIN_EXE_cerulion-netd`): the refusal line naming the
//! argument's position and bytes, a blank line and the usage on stderr, exit 2,
//! nothing on stdout and no panic.
//!
//! The argument-to-action mapping is oracle-tested in `src/main.rs`; this pins
//! the output contract. The refusal returns before tracing and transport init,
//! so this needs no socket, lock, SHM root or env, and is parallel-safe.

use std::ffi::OsString;
use std::os::unix::ffi::OsStringExt as _;
use std::process::Command;

/// `cerulion-netd <arg>` exits 2 with `line`, a blank line and the usage on
/// stderr, and nothing on stdout.
fn assert_refused_with_usage(arg: &[u8], line: &str) {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .arg(OsString::from_vec(arg.to_vec()))
        .output()
        .expect("spawn cerulion-netd");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
    assert_eq!(out.status.code(), Some(2), "{stderr}");
    let lines: Vec<&str> = stderr.lines().take(6).collect();
    assert_eq!(
        lines,
        [
            line,
            "",
            "cerulion-netd: the network gateway daemon, one per computer.",
            "",
            "USAGE:",
            "    cerulion-netd [--help] [--version]",
        ],
        "the refusal line, a blank line, then the usage:\n{stderr}"
    );
    assert!(out.stdout.is_empty(), "nothing on stdout: {out:?}");
}

#[test]
fn an_argument_that_is_not_utf8_is_refused_with_its_position_and_the_usage() {
    assert_refused_with_usage(
        b"\xFFrobot",
        "cerulion-netd: argument 1 is not valid UTF-8: \"\\xFFrobot\"",
    );
}

/// A flag name that is not valid UTF-8 gets the same refusal, not the line for
/// an unrecognized argument.
#[test]
fn a_flag_name_that_is_not_utf8_is_refused_with_its_position_and_the_usage() {
    assert_refused_with_usage(
        b"--v\xE9rsion",
        "cerulion-netd: argument 1 is not valid UTF-8: \"--v\\xE9rsion\"",
    );
}
