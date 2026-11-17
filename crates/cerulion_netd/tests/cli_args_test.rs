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

#[test]
fn an_argument_that_is_not_utf8_is_refused_with_its_position_and_the_usage() {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .arg(OsString::from_vec(b"\xFFrobot".to_vec()))
        .output()
        .expect("spawn cerulion-netd");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
    assert_eq!(out.status.code(), Some(2), "{stderr}");
    let (line, usage) = stderr
        .split_once("\n\n")
        .expect("the refusal line, a blank line, then the usage");
    assert_eq!(
        line,
        "cerulion-netd: argument 1 is not valid UTF-8: \"\\xFFrobot\""
    );
    assert!(
        usage.starts_with("cerulion-netd: the network gateway daemon") && usage.contains("USAGE:"),
        "the usage follows the blank line:\n{stderr}"
    );
    assert!(out.stdout.is_empty(), "nothing on stdout: {out:?}");
}
