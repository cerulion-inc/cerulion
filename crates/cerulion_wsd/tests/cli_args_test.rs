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

#[test]
fn an_argument_that_is_not_utf8_where_no_path_is_taken_is_refused_with_the_usage() {
    let out = run_wsd(&[OsString::from_vec(b"\xFFrobot".to_vec())]);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
    assert_eq!(out.status.code(), Some(2), "{stderr}");
    let (line, usage) = stderr
        .split_once("\n\n")
        .expect("the refusal line, a blank line, then the usage");
    assert_eq!(
        line,
        "cerulion-wsd: argument 1 is not valid UTF-8: \"\\xFFrobot\""
    );
    assert!(
        usage.starts_with("cerulion-wsd: the local workspace daemon.") && usage.contains("USAGE:"),
        "the usage follows the blank line:\n{stderr}"
    );
    assert!(out.stdout.is_empty(), "nothing on stdout: {out:?}");
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
