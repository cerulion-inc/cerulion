// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-wsd` with an argument that is not valid UTF-8, over the REAL
//! binary (`CARGO_BIN_EXE_cerulion-wsd`). Where no path is taken it is refused:
//! the line naming its position and bytes, a blank line and the usage on
//! stderr, exit 2. After `--inspect-node` it is a path: the child tries that
//! library and explains the failed load, exit 1. Neither panics. Both exit
//! before a socket is bound, so this is parallel-safe.

#![cfg(unix)]

use std::ffi::OsString;
use std::os::unix::ffi::{OsStrExt as _, OsStringExt as _};
use std::process::{Command, Output, Stdio};

fn run_wsd(args: &[OsString]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion-wsd"))
        .args(args)
        .env_remove("RUST_LOG")
        .stdin(Stdio::null())
        .output()
        .expect("spawn cerulion-wsd")
}

/// `cerulion-wsd <args>` exits 2 with `line`, a blank line and the usage on
/// stderr, and nothing on stdout.
fn assert_refused_with_usage(args: &[OsString], line: &str) {
    let out = run_wsd(args);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
    assert_eq!(out.status.code(), Some(2), "{stderr}");
    let lines: Vec<&str> = stderr.lines().take(6).collect();
    assert_eq!(
        lines,
        [
            line,
            "",
            "cerulion-wsd: the local workspace daemon.",
            "",
            "USAGE:",
            "    cerulion-wsd [--socket <path>] [--help]",
        ],
        "the refusal line, a blank line, then the usage:\n{stderr}"
    );
    assert!(out.stdout.is_empty(), "nothing on stdout: {out:?}");
}

#[test]
fn an_argument_that_is_not_utf8_where_no_path_is_taken_is_refused_with_the_usage() {
    assert_refused_with_usage(
        &[OsString::from_vec(b"\xFFrobot".to_vec())],
        "cerulion-wsd: argument 1 is not valid UTF-8: \"\\xFFrobot\"",
    );
}

/// A flag name that is not valid UTF-8, first and after a socket path, gets
/// the same refusal, not the line for an unrecognized or unexpected argument.
#[test]
fn a_flag_name_that_is_not_utf8_is_refused_with_its_position_and_the_usage() {
    assert_refused_with_usage(
        &[
            OsString::from_vec(b"--s\xF6cket".to_vec()),
            OsString::from("/tmp/x.sock"),
        ],
        "cerulion-wsd: argument 1 is not valid UTF-8: \"--s\\xF6cket\"",
    );
    assert_refused_with_usage(
        &[
            OsString::from("--socket"),
            OsString::from("/tmp/x.sock"),
            OsString::from_vec(b"--h\xE9lp".to_vec()),
        ],
        "cerulion-wsd: argument 3 is not valid UTF-8: \"--h\\xE9lp\"",
    );
}

#[test]
fn a_library_path_that_is_not_utf8_reaches_the_inspection() {
    let mut name = format!("cerulion-wsd-args-{}-", std::process::id()).into_bytes();
    name.extend_from_slice(b"caf\xE9.so");
    let library = std::env::temp_dir().join(std::ffi::OsStr::from_bytes(&name));
    assert!(!library.exists(), "the library must not exist: {library:?}");
    let out = run_wsd(&[OsString::from("--inspect-node"), library.clone().into()]);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
    assert!(!stderr.contains("not valid UTF-8"), "{stderr}");
    assert_eq!(out.status.code(), Some(1), "{stderr}");
    assert!(
        stderr.contains(&format!(
            "cerulion-wsd --inspect-node {}: ",
            library.display()
        )),
        "the failed load names the library it tried:\n{stderr}"
    );
    assert!(
        out.stdout.is_empty(),
        "the document channel stays empty: {out:?}"
    );
}
