// SPDX-License-Identifier: AGPL-3.0-only
//! The optional-system-dep notice REACHES THE USER, over the real
//! `cerulion` binary.
//!
//! `cerulion_cli_engine`'s `node_build_system_deps_test.rs` proves the ENGINE
//! hands the notice to its sink — but the sink there is a test closure. The
//! only thing that puts the report in front of a human is one closure in
//! `main.rs` (`&mut |notice| eprint!("{notice}")`), and replacing it with
//! `&mut |_| {}` left the entire suite green while the feature became
//! user-invisible. This file is that missing link: spawn the binary, read its
//! stderr.
//!
//! Hermetic and fast: the scaffolded node crate has NO dependencies, and its
//! declared pkg-config module cannot resolve on any machine, so the verdict is
//! identical on a developer Mac, a GStreamer-laden Orin and a bare CI runner.
//!
//! The progress line printed while cargo runs has the same single point of
//! failure (a second closure in `main.rs`), so it is pinned here too.
//!
//! The same harness carries the sibling of that class: the missing-toolchain
//! message (`cargo` not on `PATH`) is a pure mapping in the engine, and only
//! a spawned binary can show the spawn error still reaches it.

use std::path::{Path, PathBuf};
use std::process::Command;

/// A pkg-config module name no machine can resolve, so the rendered notice is
/// machine-independent.
const ABSENT_MODULE: &str = "cerulion-cli-definitely-absent-module";

const FEATURE: &str = "probe";

/// Scaffold the minimum a `cerulion node build` needs: a workspace root with a
/// `nodes/<type>/` member. The crate deliberately does NOT compile — the point
/// is that the notice survives the cargo failure, which is the exact run where
/// losing it hurts (a build that failed BECAUSE of an injected feature).
fn scaffold(dir: &Path) -> PathBuf {
    let root = dir.to_path_buf();
    let node = root.join("nodes").join("probe_node");
    std::fs::create_dir_all(node.join("src")).expect("create the node crate dirs");
    // `CerulionWorkspace::discover` requires a `graphs/` directory beside the
    // `[workspace]` manifest — without it the CLI refuses before reaching
    // `node build` at all.
    std::fs::create_dir_all(root.join("graphs")).expect("create graphs/");
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nmembers = [\"nodes/*\"]\nresolver = \"2\"\n",
    )
    .expect("write the workspace manifest");
    std::fs::write(
        node.join("Cargo.toml"),
        format!(
            "[package]\nname = \"probe_node\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [features]\n{FEATURE} = []\ndefault = []\n\n\
             [package.metadata.cerulion.optional-system-deps.{FEATURE}]\n\
             pkg-config = [\"{ABSENT_MODULE}\"]\n\
             summary = \"the CLI notice probe\"\n\
             without-it = \"the probe capability is unavailable\"\n\
             install.macos = \"brew install probe\"\n\
             install.debian = \"sudo apt install probe\"\n\
             install.linux = \"install probe\"\n"
        ),
    )
    .expect("write the node manifest");
    std::fs::write(
        node.join("src").join("lib.rs"),
        "this is not valid rust and will not compile\n",
    )
    .expect("write the node source");
    root
}

/// THE pin: `cerulion node build` PRINTS the system-dep report to stderr.
///
/// Deleting the print in `main.rs` — or reordering it so the notice is
/// printed only after a SUCCESSFUL build — fails here.
#[test]
fn node_build_prints_the_system_dep_notice_to_stderr() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        .output()
        .expect("run the cerulion binary");

    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "the stub crate cannot compile, so the build must fail — \
         stdout: {}\nstderr: {stderr}",
        String::from_utf8_lossy(&out.stdout)
    );

    // The notice, on the failure path. Which BRANCH renders (library-missing
    // vs pkg-config-tool-missing) depends on the machine, so assert what both must
    // carry: the node, the feature's fate, and an actionable command.
    assert!(
        stderr.contains("probe_node"),
        "the notice must name the node: {stderr}"
    );
    assert!(
        stderr.contains(&format!("Building WITHOUT `--features {FEATURE}`")),
        "the notice must state the feature is OFF: {stderr}"
    );
    assert!(
        stderr.contains("probe"),
        "the notice must carry the install command: {stderr}"
    );
    assert!(
        stderr.contains("the probe capability is unavailable"),
        "the notice must state the run-time consequence: {stderr}"
    );
}

/// The anti-tautology control: a node declaring NOTHING prints no notice, so
/// the assertions above track the declaration and not some banner the CLI
/// emits on every build.
#[test]
fn a_node_with_no_declaration_prints_no_system_dep_notice() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());
    // Strip the metadata block (and the now-pointless feature) back out.
    std::fs::write(
        ws.join("nodes").join("probe_node").join("Cargo.toml"),
        "[package]\nname = \"probe_node\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
    )
    .expect("rewrite the node manifest");

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        .output()
        .expect("run the cerulion binary");

    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "the stub crate still cannot compile");
    assert!(
        !stderr.contains("system dependency"),
        "a node with no declaration must say nothing about system deps: {stderr}"
    );
    assert!(
        !stderr.contains("Building WITHOUT"),
        "…and must not claim a feature was skipped: {stderr}"
    );
}

/// THE wiring pin for the missing-toolchain message: with no `cargo` on the
/// child's PATH the user must be told to install Rust, not handed "No such
/// file or directory (os error 2)". The engine's unit tests pin the pure
/// mapping; a refactor routing the spawn error through `CliError::Io` keeps
/// every one of them green while the user is back to the raw text. Hermetic:
/// the child's PATH is an EMPTY DIRECTORY (not unset — libc replaces an
/// absent PATH with a default search list), so the spawn fails at exec time
/// in milliseconds on any machine; a child-only `env` avoids any `set_var` race
/// with the parallel tests in this binary.
#[test]
fn node_build_without_cargo_on_path_names_the_rust_toolchain() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());
    let empty = tmp.path().join("empty-path");
    std::fs::create_dir_all(&empty).expect("create the empty PATH dir");

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        .env("PATH", &empty)
        .output()
        .expect("run the cerulion binary");

    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "stderr: {stderr}");
    assert!(
        stderr.contains("Build failed for 'probe_node': `cargo` was not found on PATH"),
        "the wrapper and the toolchain text must both reach stderr: {stderr}"
    );
    assert!(
        stderr.contains("https://rustup.rs"),
        "the remedy must be named: {stderr}"
    );
    assert!(
        !stderr.contains("os error 2"),
        "the raw io::Error text must not leak through: {stderr}"
    );
}

/// THE progress pin: `cerulion node build` says it is building BEFORE cargo
/// runs, on stderr.
///
/// Cargo's output is captured, so this line is all a user sees until the
/// build ends; without it a first build (which compiles the runtime too) is
/// minutes of blank terminal. The crate does not compile, which is what makes
/// this an ORDER pin: a line printed after cargo returns is skipped by the
/// early return on failure, so it would be missing here.
#[test]
fn node_build_prints_a_progress_line_before_cargo_runs() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        .output()
        .expect("run the cerulion binary");

    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "the stub crate cannot compile");
    assert!(
        stderr.contains("Building 'probe_node'."),
        "the progress line must name the node: {stderr}"
    );
    assert!(
        stderr.contains("first build of a workspace also compiles the Cerulion runtime"),
        "…and say why a first build is slow: {stderr}"
    );
    assert!(
        !stdout.contains("Building"),
        "stdout carries the result alone: {stdout}"
    );
}

/// The success path: the progress line still goes to stderr, and stdout is
/// exactly the result line, so a script reading stdout sees what it always saw.
#[test]
fn a_successful_node_build_keeps_stdout_to_the_result_line() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());
    let node = ws.join("nodes").join("probe_node");
    // A crate that compiles, with nothing declared and no dependencies.
    std::fs::write(
        node.join("Cargo.toml"),
        "[package]\nname = \"probe_node\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
    )
    .expect("rewrite the node manifest");
    std::fs::write(node.join("src").join("lib.rs"), "pub fn ok() {}\n")
        .expect("rewrite the node source");

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        // Keep the build inside the tempdir whatever the caller exported.
        .env("CARGO_TARGET_DIR", ws.join("target"))
        .output()
        .expect("run the cerulion binary");

    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "stdout: {stdout}\nstderr: {stderr}");
    assert_eq!(stdout, "Built 'probe_node'\n");
    assert!(
        stderr.contains("Building 'probe_node'."),
        "the progress line must be printed on the success path too: {stderr}"
    );
}
