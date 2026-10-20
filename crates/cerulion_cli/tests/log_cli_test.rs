// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion log echo` over the REAL binary (`CARGO_BIN_EXE_cerulion`): the
//! help pages and every refusal a user can meet before a record is read, with
//! the refusal text pinned byte for byte.
//!
//! The filters are checked before anything is opened, so an unknown level is
//! refused with no topic attach and no interrupt handler, whatever the host is
//! running. A node id can only be checked against a running graph, and with
//! none the verb says so and names the command that starts one; so does a
//! bare `log echo` when nothing publishes the log topic. Those two arms read
//! the live-run registry and the topic list, so each child gets its own
//! shared-memory namespace: a `config/iceoryx2.toml` in its working directory
//! (the first place iceoryx2 looks) with a private root and prefix, which a
//! graph running on the host, or another test's child, cannot reach. The arms
//! therefore run in parallel and need no quiet host.
//!
//! Each case is one subprocess under an isolated `HOME`, a working directory
//! with no workspace, the network kill switch and telemetry off.

#![cfg(unix)]

use std::path::Path;
use std::process::{Command, Output};

fn cerulion(args: &[&str], home: &Path, cwd: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .env("HOME", home)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_LOGIN_GATE", "off")
        .env("DO_NOT_TRACK", "1")
        .env("NO_COLOR", "1")
        .env_remove("RUST_LOG")
        .env_remove("COMPLETE")
        .current_dir(cwd)
        .args(args)
        .output()
        .expect("run cerulion")
}

struct Sandbox {
    _tmp: tempfile::TempDir,
    home: std::path::PathBuf,
    cwd: std::path::PathBuf,
}

/// A fresh `HOME` and working directory, plus the private iceoryx2 namespace
/// the child reads from `cwd/config/iceoryx2.toml`. Under `/tmp` so the
/// namespace's socket paths stay short.
fn sandbox() -> Sandbox {
    let tmp = tempfile::tempdir_in("/tmp").expect("tempdir");
    let home = tmp.path().join("home");
    let cwd = tmp.path().join("cwd");
    std::fs::create_dir_all(home.join(".cerulion")).expect("home");
    std::fs::create_dir_all(cwd.join("config")).expect("cwd");
    let root = tmp.path().join("shm");
    let prefix = tmp
        .path()
        .file_name()
        .expect("tempdir name")
        .to_str()
        .expect("utf-8 tempdir name")
        .replace('.', "_");
    std::fs::write(
        cwd.join("config").join("iceoryx2.toml"),
        format!(
            "[global]\nroot-path = \"{}\"\nprefix = \"log_{prefix}_\"\n",
            root.display()
        ),
    )
    .expect("iceoryx2 config");
    Sandbox {
        _tmp: tmp,
        home,
        cwd,
    }
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

#[test]
fn log_help_lists_echo_as_its_one_verb() {
    let sb = sandbox();
    let out = cerulion(&["log", "--help"], &sb.home, &sb.cwd);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", text(&out.stderr));
    let stdout = text(&out.stdout);
    assert!(
        stdout.contains("Usage: cerulion log [OPTIONS] <COMMAND>"),
        "{stdout}"
    );
    assert!(
        stdout.contains("  echo  Print the log records of the running local graph, one line each"),
        "{stdout}"
    );
}

#[test]
fn log_echo_help_names_both_filters_and_the_line_format() {
    let sb = sandbox();
    let out = cerulion(&["log", "echo", "--help"], &sb.home, &sb.cwd);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", text(&out.stderr));
    let stdout = text(&out.stdout);
    assert!(stdout.contains("--node <ID>"), "{stdout}");
    assert!(stdout.contains("--level <LEVEL>"), "{stdout}");
    assert!(
        stdout.contains("debug, info, warn or error"),
        "the accepted levels are in the help: {stdout}"
    );
    assert!(
        stdout.contains("<seconds>.<nanoseconds> <LEVEL> <node> <target> <file>:<line> <message>"),
        "the line format is in the help: {stdout}"
    );
}

#[test]
fn an_unknown_level_is_refused_before_anything_is_opened() {
    let sb = sandbox();
    let out = cerulion(&["log", "echo", "--level", "warning"], &sb.home, &sb.cwd);
    assert_eq!(out.status.code(), Some(1), "stdout: {}", text(&out.stdout));
    assert_eq!(
        text(&out.stderr),
        "Error: unknown log level 'warning'; accepted levels: debug, info, warn, error\n"
    );
    assert_eq!(text(&out.stdout), "");
}

#[test]
fn an_unknown_level_is_refused_even_when_a_node_filter_is_given_too() {
    // The level parses first, so the refusal does not depend on whether a
    // graph is running: the same text on every host.
    let sb = sandbox();
    let out = cerulion(
        &["log", "echo", "--node", "camera", "--level", "TRACE"],
        &sb.home,
        &sb.cwd,
    );
    assert_eq!(out.status.code(), Some(1));
    assert_eq!(
        text(&out.stderr),
        "Error: unknown log level 'TRACE'; accepted levels: debug, info, warn, error\n"
    );
}

#[test]
fn a_node_filter_with_no_running_graph_is_refused_naming_the_start_command() {
    let sb = sandbox();
    let out = cerulion(&["log", "echo", "--node", "camera"], &sb.home, &sb.cwd);
    assert_eq!(out.status.code(), Some(1), "stdout: {}", text(&out.stdout));
    let stderr = text(&out.stderr);
    assert!(
        stderr.ends_with(
            "Error: --node camera cannot be checked: no graph is running on this host; start \
             one with `cerulion graph run <graph>`\n"
        ),
        "{stderr}"
    );
    assert_eq!(text(&out.stdout), "");
}

#[test]
fn echo_with_nothing_publishing_the_log_topic_is_refused_naming_the_start_command() {
    let sb = sandbox();
    let out = cerulion(&["log", "echo"], &sb.home, &sb.cwd);
    assert_eq!(out.status.code(), Some(1), "stdout: {}", text(&out.stdout));
    let stderr = text(&out.stderr);
    assert!(
        stderr.ends_with(
            "Error: no running graph publishes /log on this host; start one with `cerulion \
             graph run <graph>`\n"
        ),
        "{stderr}"
    );
    assert_eq!(text(&out.stdout), "");
}
