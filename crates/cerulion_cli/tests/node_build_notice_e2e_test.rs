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

#[cfg(unix)]
mod fresh_executable;

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

/// Off a terminal, a machine with no Rust at all is told the one command that
/// installs it, is never asked, and gets the same error and exit code as a
/// build without the check. `Command::output` gives the child a null stdin, which is not a
/// terminal, and empty Cargo and Rustup homes leave no cargo or rustup to find
/// anywhere, so the official installer is the command named.
#[test]
fn node_build_off_a_terminal_names_the_rust_install_command_and_never_asks() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());
    let empty = tmp.path().join("empty-path");
    let cargo_home = tmp.path().join("cargo-home");
    let rustup_home = tmp.path().join("rustup-home");
    for dir in [&empty, &cargo_home, &rustup_home] {
        std::fs::create_dir_all(dir).expect("create an empty dir");
    }

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "probe_node"])
        .current_dir(&ws)
        .env("PATH", &empty)
        .env("CARGO_HOME", &cargo_home)
        .env("RUSTUP_HOME", &rustup_home)
        .env("RUST_LOG", "warn")
        .env_remove("RUSTC")
        .env_remove("CARGO_BUILD_RUSTC")
        .output()
        .expect("run the cerulion binary");

    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "stderr: {stderr}");
    let needed = format!(
        "{}\nTo install it, run: {}\n",
        cerulion_cli_engine::rust_setup::RUST_NEEDED,
        cerulion_cli_engine::rust_setup::RUSTUP_INIT_COMMAND
    );
    assert!(
        stderr.starts_with(&needed),
        "the install command comes first: {stderr}"
    );
    assert!(
        stderr.contains("Build failed for 'probe_node': `cargo` was not found on PATH"),
        "the existing error still follows: {stderr}"
    );
    assert!(
        !stderr.contains("[Y/n]"),
        "never asks off a terminal: {stderr}"
    );
    assert_eq!(
        std::fs::read_dir(&cargo_home)
            .expect("read the Cargo home")
            .count(),
        0,
        "nothing was installed"
    );
}

/// Write an executable stub. Stubs use shell builtins only, so they run on a
/// PATH that holds nothing else.
#[cfg(unix)]
fn stub(path: &Path, body: &str) {
    std::fs::create_dir_all(path.parent().expect("a stub has a parent")).expect("stub dir");
    fresh_executable::install_script(path, &format!("#!/bin/sh\n{body}\n"));
}

/// One hand-built machine for [`node_build_off_a_terminal_names_what_each_machine_needs`].
#[cfg(unix)]
struct Machine {
    name: &'static str,
    /// Stubs on PATH, by program name.
    on_path: &'static [(&'static str, &'static str)],
    /// Stubs in Cargo's home `bin` directory.
    in_cargo_home: &'static [(&'static str, &'static str)],
    /// An env var naming a compiler stub, and that stub's body.
    compiler_env: Option<(&'static str, &'static str)>,
    /// Whether the workspace carries a `rust-toolchain.toml`.
    toolchain_file: bool,
    /// The stderr prefix, from Cargo's home `bin` and the workspace root;
    /// `None` when no Rust line may appear.
    expected: Option<fn(&Path, &Path) -> String>,
}

/// The real probe, off a terminal, against hand-built machines: which `cargo`
/// is found, which compiler it asks (`RUSTC`, then `CARGO_BUILD_RUSTC`),
/// whether rustup or the helper is there, whether a toolchain file selects the
/// compiler, and what is printed for each. Every arm then reaches the build's
/// own failure with exit code 1, and none of them asks. The old compiler
/// answers only when rustup's automatic install is off, so an arm that expects
/// its release also proves the probe turned it off.
#[cfg(unix)]
#[test]
fn node_build_off_a_terminal_names_what_each_machine_needs() {
    use cerulion_cli_engine::rust_setup::{export_path_line, RUST_NEEDED, SELECT_STABLE};
    const FAILS: &str = "exit 1";
    const OLD: &str = "[ \"$RUSTUP_AUTO_INSTALL\" = 0 ] || exit 1\nprintf 'release: 1.80.0\\n'";
    const READY: &str = "printf 'release: 1.98.1\\n'";
    const NO_DEFAULT: &str = "[ \"$*\" = default ] || exit 99\nexit 1";
    const HAS_DEFAULT: &str = "[ \"$*\" = default ] || exit 99\nexit 0";
    fn upgrade(_: &Path, _: &Path) -> String {
        format!(
            "{RUST_NEEDED} The rustc Cargo uses reports 1.80.0.\n\
             rustup does not manage that Rust: upgrade it with the tool that installed it, or \
             install rustup from https://rustup.rs.\n"
        )
    }
    fn rustup_on_path(_: &Path, _: &Path) -> String {
        format!(
            "{RUST_NEEDED}\nTo install it, run: rustup toolchain install stable --profile \
             minimal\n{SELECT_STABLE}\n"
        )
    }
    let machines = [
        Machine {
            name: "cargo only in Cargo's home",
            on_path: &[],
            in_cargo_home: &[("cargo", FAILS)],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(|home_bin, _| {
                format!(
                    "{RUST_NEEDED} Cargo is installed in {}, which is not on your PATH.\n\
                     Add it to your PATH, for example in your shell startup file: {}\n",
                    home_bin.display(),
                    export_path_line(home_bin)
                )
            }),
        },
        Machine {
            name: "an older Rust without rustup",
            on_path: &[("cargo", FAILS), ("rustc", OLD)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(upgrade),
        },
        Machine {
            name: "RUSTC names the compiler Cargo uses",
            on_path: &[("cargo", FAILS), ("rustc", READY)],
            in_cargo_home: &[],
            compiler_env: Some(("RUSTC", OLD)),
            toolchain_file: false,
            expected: Some(upgrade),
        },
        Machine {
            name: "CARGO_BUILD_RUSTC names the compiler Cargo uses",
            on_path: &[("cargo", FAILS), ("rustc", READY)],
            in_cargo_home: &[],
            compiler_env: Some(("CARGO_BUILD_RUSTC", OLD)),
            toolchain_file: false,
            expected: Some(upgrade),
        },
        Machine {
            name: "rustup only in Cargo's home",
            on_path: &[("cargo", FAILS), ("rustc", OLD)],
            in_cargo_home: &[("rustup", NO_DEFAULT)],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(|home_bin, _| {
                format!(
                    "{RUST_NEEDED}\nTo install it, run: {} toolchain install stable --profile \
                     minimal\n{SELECT_STABLE}\n",
                    home_bin.join("rustup").display()
                )
            }),
        },
        Machine {
            name: "a toolchain file selects an older Rust",
            on_path: &[("cargo", FAILS), ("rustc", OLD), ("rustup", HAS_DEFAULT)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: true,
            expected: Some(|_, ws| {
                format!(
                    "{RUST_NEEDED} The toolchain file {} selects Rust 1.80.0; make it name \
                     `stable`, or remove it.\n",
                    ws.join("rust-toolchain.toml").display()
                )
            }),
        },
        Machine {
            name: "the helper on PATH",
            on_path: &[("cerulion-install-rust", FAILS)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(|_, _| {
                format!("{RUST_NEEDED}\nTo install it, run: cerulion-install-rust\n")
            }),
        },
        Machine {
            name: "rustup with no default",
            on_path: &[("cargo", FAILS), ("rustc", FAILS), ("rustup", NO_DEFAULT)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(rustup_on_path),
        },
        Machine {
            name: "an older Rust under rustup",
            on_path: &[("cargo", FAILS), ("rustc", OLD), ("rustup", HAS_DEFAULT)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: Some(rustup_on_path),
        },
        Machine {
            name: "an unreadable compiler with a rustup default",
            on_path: &[("cargo", FAILS), ("rustc", FAILS), ("rustup", HAS_DEFAULT)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: None,
        },
        Machine {
            name: "a ready Rust",
            on_path: &[("cargo", FAILS), ("rustc", READY)],
            in_cargo_home: &[],
            compiler_env: None,
            toolchain_file: false,
            expected: None,
        },
    ];
    for machine in machines {
        let name = machine.name;
        let tmp = tempfile::tempdir().expect("tempdir");
        let ws = scaffold(tmp.path());
        if machine.toolchain_file {
            std::fs::write(
                ws.join("rust-toolchain.toml"),
                "[toolchain]\nchannel = \"1.80.0\"\n",
            )
            .expect("write a toolchain file");
        }
        let path_dir = tmp.path().join("path");
        let home_bin = tmp.path().join("cargo-home").join("bin");
        let rustup_home = tmp.path().join("rustup-home");
        std::fs::create_dir_all(&path_dir).expect("PATH dir");
        std::fs::create_dir_all(&rustup_home).expect("rustup home");
        for (program, body) in machine.on_path {
            stub(&path_dir.join(program), body);
        }
        for (program, body) in machine.in_cargo_home {
            stub(&home_bin.join(program), body);
        }
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        command
            .args(["node", "build", "probe_node"])
            .current_dir(&ws)
            .env("PATH", &path_dir)
            .env("CARGO_HOME", home_bin.parent().expect("Cargo home"))
            .env("RUSTUP_HOME", &rustup_home)
            .env("RUST_LOG", "warn")
            .env_remove("RUSTC")
            .env_remove("CARGO_BUILD_RUSTC")
            .env_remove("RUSTUP_TOOLCHAIN")
            .env_remove("RUSTUP_AUTO_INSTALL");
        if let Some((var, body)) = machine.compiler_env {
            let selected = tmp.path().join("selected").join("rustc");
            stub(&selected, body);
            command.env(var, &selected);
        }
        let out = command.output().expect("run the cerulion binary");
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert_eq!(out.status.code(), Some(1), "{name}: {stderr}");
        assert!(
            !stderr.contains("[Y/n]"),
            "{name}: never asks off a terminal: {stderr}"
        );
        match machine.expected {
            Some(expected) => {
                // The CLI names the workspace by its canonical path.
                let expected =
                    expected(&home_bin, &ws.canonicalize().expect("canonical workspace"));
                assert!(
                    stderr.starts_with(&expected),
                    "{name}: expected\n{expected}\ngot\n{stderr}"
                );
            }
            None => assert!(
                !stderr.contains(RUST_NEEDED),
                "{name}: no Rust line expected: {stderr}"
            ),
        }
    }
}

/// A node that does not exist is refused before Rust is checked, so a typo
/// never meets an install question or command.
#[test]
fn node_build_of_an_unknown_node_is_refused_before_the_rust_check() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = scaffold(tmp.path());
    let empty = tmp.path().join("empty-path");
    let cargo_home = tmp.path().join("cargo-home");
    for dir in [&empty, &cargo_home] {
        std::fs::create_dir_all(dir).expect("create an empty dir");
    }
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "build", "no_such_node"])
        .current_dir(&ws)
        .env("PATH", &empty)
        .env("CARGO_HOME", &cargo_home)
        .env("RUST_LOG", "warn")
        .output()
        .expect("run the cerulion binary");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.code(), Some(1), "stderr: {stderr}");
    assert!(
        stderr.contains("no_such_node"),
        "the refusal names the node: {stderr}"
    );
    assert!(
        !stderr.contains(cerulion_cli_engine::rust_setup::RUST_NEEDED),
        "no Rust check for a node that does not exist: {stderr}"
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
