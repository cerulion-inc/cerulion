// SPDX-License-Identifier: AGPL-3.0-only
//! A command-line argument that is not valid UTF-8, over the REAL binary
//! (`CARGO_BIN_EXE_cerulion`).
//!
//! Pinned: such an argument in a text field is refused with exit 2 and one
//! `Error:` line naming its position and its bytes, then the remedy line, and
//! nothing panics; in a path field it is taken, and the verb runs on that path.
//! `cerulion ros2 run` forwarding such an argument byte for byte is pinned in
//! `ros2_run_e2e_test.rs`.
//!
//! Every subprocess is self-terminating under an isolated `HOME` and a temp
//! cwd: the refusal returns before logging, the network and the login gate,
//! and the path arm reads one file that does not exist. No `#[serial]`.

#![cfg(unix)]

use std::ffi::OsString;
use std::os::unix::ffi::OsStringExt as _;
use std::path::Path;
use std::process::{Command, Output};

/// The remedy line `not_utf8_args::REMEDY` prints. A duplicate literal on
/// purpose: `cerulion_cli` has no lib target, so the duplicate is the pin.
const REMEDY: &str =
    "Only a path may hold bytes that are not UTF-8: pass every other argument as valid UTF-8 text.";

fn raw(bytes: &[u8]) -> OsString {
    OsString::from_vec(bytes.to_vec())
}

fn run(args: &[OsString], root: &Path) -> Output {
    let home = root.join("home");
    std::fs::create_dir_all(&home).expect("home");
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .env("HOME", &home)
        // An unparseable ambient level would print a fallback line and break
        // the exact stderr oracle for a reason that is not this file's.
        .env("IOX2_LOG_LEVEL", "error")
        .env("CERULION_LOGIN_GATE", "off")
        .env("CERULION_NETWORK", "off")
        .env_remove("COMPLETE")
        .env_remove("RUST_LOG")
        .current_dir(root)
        .output()
        .expect("run cerulion")
}

fn assert_no_panic(out: &Output) {
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!stderr.contains("panicked at"), "a panic:\n{stderr}");
}

/// The headline: one argument that is not valid UTF-8, in a text field (a
/// topic name), is refused by its position with exit 2, not a panic (101).
#[test]
fn an_argument_that_is_not_utf8_in_a_text_field_is_refused_by_position() {
    let root = tempfile::tempdir().expect("tempdir");
    let out = run(
        &[raw(b"topic"), raw(b"echo"), raw(b"/caf\xE9")],
        root.path(),
    );
    assert_no_panic(&out);
    assert_eq!(out.status.code(), Some(2), "{out:?}");
    assert_eq!(
        String::from_utf8_lossy(&out.stderr),
        format!("Error: argument 3 is not valid UTF-8: \"/caf\\xE9\"\n{REMEDY}\n")
    );
    assert!(out.stdout.is_empty(), "{out:?}");
}

/// clap does not say which argument it refused, so every argument that is not
/// valid UTF-8 is named, in order, the path clap accepted (argument 3) among
/// them.
#[test]
fn every_argument_that_is_not_utf8_is_named_when_one_is_refused() {
    let root = tempfile::tempdir().expect("tempdir");
    let out = run(
        &[
            raw(b"bag"),
            raw(b"play"),
            raw(b"caf\xE9.mcap"),
            raw(b"--topics"),
            raw(b"\xFF"),
        ],
        root.path(),
    );
    assert_no_panic(&out);
    assert_eq!(out.status.code(), Some(2), "{out:?}");
    assert_eq!(
        String::from_utf8_lossy(&out.stderr),
        format!(
            "Error: argument 3 is not valid UTF-8: \"caf\\xE9.mcap\"\n\
             Error: argument 5 is not valid UTF-8: \"\\xFF\"\n{REMEDY}\n"
        )
    );
}

/// A path field takes the argument as given: `bag info` runs, on that path,
/// and fails only because the file is not there.
#[test]
fn an_argument_that_is_not_utf8_in_a_path_field_reaches_the_verb() {
    let root = tempfile::tempdir().expect("tempdir");
    let out = run(
        &[raw(b"bag"), raw(b"info"), raw(b"caf\xE9.mcap")],
        root.path(),
    );
    assert_no_panic(&out);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "{stderr}");
    assert!(!stderr.contains("not valid UTF-8"), "{stderr}");
    assert!(
        stderr.contains("caf\u{FFFD}.mcap"),
        "the verb names the path it tried:\n{stderr}"
    );
}
