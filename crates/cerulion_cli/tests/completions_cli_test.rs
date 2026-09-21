// SPDX-License-Identifier: AGPL-3.0-only
//! The "prints nothing but candidates" guarantee, over
//! the REAL binary (`CARGO_BIN_EXE_cerulion`).
//!
//! A completion runs on a keystroke and its stdout is consumed by the shell,
//! so ANY other byte on either stream is garbage in the user's terminal — or,
//! for `cerulion completions <shell> > file`, garbage inside a file that gets
//! `source`d. `quiet_iceoryx2()` exists to close the one stderr path that
//! remained (an unparseable `IOX2_LOG_LEVEL` makes
//! `init_iceoryx_log_level_from_env` emit an `eprintln!`), and until now it was
//! reachable from no test at all.
//!
//! This cannot be an in-process test: the emission goes to the process's own
//! stderr through iceoryx2's built-in console logger, which no `tracing`
//! subscriber sees and libtest cannot capture. Same reasoning as
//! `cerulion_core`'s `iox2_log_level_test.rs`.
//!
//! Every subprocess here is a plain, self-terminating call under an ISOLATED
//! `HOME` and a temp cwd, so there is no `#[serial]` and no shared state.

use std::path::Path;
use std::process::{Command, Output};

/// The hostile environment a completion must survive silently.
///
/// - `IOX2_LOG_LEVEL=trace` — the loudest iceoryx2 level, so any internal
///   diagnostic on the SHM scan would print. `quiet_iceoryx2` overrides it.
/// - `RUST_LOG=trace` — would turn on every `tracing` target in the engine if
///   a subscriber were ever installed on this path.
/// - `CERULION_NETD_SOCKET` — points at nothing, so a stray netd client would
///   fail loudly rather than silently finding a live daemon.
fn hostile(cmd: &mut Command, home: &Path, cwd: &Path) {
    cmd.env("HOME", home)
        .env("IOX2_LOG_LEVEL", "trace")
        .env("RUST_LOG", "trace")
        .env("CERULION_NETD_SOCKET", home.join("no-such.sock"))
        .env_remove("CERULION_PEERS")
        .current_dir(cwd);
}

/// The record separator the generated bash function sets (`local IFS=$'\013'`,
/// forwarded as `_CLAP_IFS`). Passing it makes these tests drive the EXACT
/// protocol rather than the `\n` default that applies when it is unset.
const BASH_IFS: char = '\u{0b}';

/// Drive the completion protocol exactly as the generated shell script does.
fn complete(words: &[&str], home: &Path, cwd: &Path) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    hostile(&mut cmd, home, cwd);
    cmd.env("COMPLETE", "bash")
        .env("_CLAP_IFS", BASH_IFS.to_string())
        .env("_CLAP_COMPLETE_INDEX", (words.len() - 1).to_string())
        .arg("--")
        .arg("cerulion")
        .args(&words[1..])
        .output()
        .expect("run completion")
}

fn scaffold(root: &Path) {
    std::fs::create_dir_all(root.join("home").join(".cerulion")).expect("home");
    std::fs::create_dir_all(root.join("graphs")).expect("graphs");
    std::fs::write(root.join("Cargo.toml"), "[workspace]\nmembers = []\n").expect("manifest");
    std::fs::write(root.join("graphs").join("perception.yaml"), "nodes: {}\n").expect("graph");
}

#[test]
fn a_completion_prints_candidates_on_stdout_and_nothing_on_stderr() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    scaffold(root);
    let home = root.join("home");

    // Four shapes, chosen to cover every completer class: the pure clap tree,
    // a filesystem source, the peer-cache/robots source, and the SHM source
    // (the only one that touches iceoryx2 and so the only one that could
    // possibly emit an iceoryx2 diagnostic).
    for words in [
        vec!["cerulion", ""],
        vec!["cerulion", "graph", "run", ""],
        vec!["cerulion", "viz", "--robot", ""],
        vec!["cerulion", "topic", "hz", ""],
    ] {
        let out = complete(&words, &home, root);
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(
            out.stderr.is_empty(),
            "`{}` wrote {} bytes to stderr, which land in the user's shell:\n{stderr}",
            words.join(" "),
            out.stderr.len()
        );
        assert!(out.status.success(), "`{}` exited nonzero", words.join(" "));

        // stdout must be candidates and nothing else. bash's wire is
        // `\013`-separated VALUES with no help, so every field is a bare
        // candidate — a log line would show up as a field carrying spaces or
        // a newline.
        let stdout = String::from_utf8_lossy(&out.stdout);
        for field in stdout.split(BASH_IFS).filter(|f| !f.is_empty()) {
            assert!(
                !field.contains(char::is_whitespace),
                "`{}` emitted a non-candidate field on stdout: {field:?}\nfull: {stdout:?}",
                words.join(" ")
            );
        }
        // The clap tree always answers, so an empty stdout here would mean the
        // whole protocol broke and every assertion above went vacuous.
        if words.len() == 2 {
            assert!(
                stdout.split(BASH_IFS).any(|f| f == "graph"),
                "the completion protocol produced nothing recognisable: {stdout:?}"
            );
        }
    }
}

#[test]
fn an_unparseable_iox2_log_level_is_still_silent_on_a_completion() {
    // The specific path `quiet_iceoryx2` closes: `init_iceoryx_log_level_from_env`
    // emits one `eprintln!` naming the offending value. A normal verb is
    // ENTITLED to that complaint; a completion is not, because its stderr is
    // the user's prompt.
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    scaffold(root);
    let home = root.join("home");

    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    hostile(&mut cmd, &home, root);
    let out = cmd
        .env("IOX2_LOG_LEVEL", "notalevel")
        .env("COMPLETE", "bash")
        .env("_CLAP_COMPLETE_INDEX", "3")
        .args(["--", "cerulion", "topic", "hz", ""])
        .output()
        .expect("run completion");

    assert!(
        out.stderr.is_empty(),
        "an unparseable IOX2_LOG_LEVEL leaked to a completion's stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    // ANTI-TAUTOLOGY. Without this the assertion above is satisfied by a
    // binary that never complains about anything — the apparatus has to be
    // shown to fire on the path that is allowed to complain. `schema list`
    // is a plain one-shot verb that initialises the same logger.
    let mut normal = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    hostile(&mut normal, &home, root);
    let out = normal
        .env("IOX2_LOG_LEVEL", "notalevel")
        .env_remove("COMPLETE")
        .arg("schema")
        .arg("list")
        .output()
        .expect("run schema list");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("notalevel"),
        "the apparatus never fires: a NORMAL verb must still name the bad \
         IOX2_LOG_LEVEL, else the silence asserted above proves nothing.\n\
         stderr was: {stderr}"
    );
}

#[test]
fn the_completions_verb_writes_only_the_script_when_redirected() {
    // `cerulion completions zsh > _cerulion` must produce a file that is
    // exactly the shell script — the install hint goes to stderr, and only
    // when stdout is a terminal (a captured `Output` never is).
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    scaffold(root);
    let home = root.join("home");

    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    hostile(&mut cmd, &home, root);
    let out = cmd
        .env_remove("COMPLETE")
        .args(["completions", "zsh"])
        .output()
        .expect("run completions");

    assert!(out.status.success());
    assert!(
        out.stderr.is_empty(),
        "the hint must not reach a redirected stdout's sibling stream: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let script = String::from_utf8(out.stdout).expect("script is UTF-8");
    assert!(
        script.starts_with("#compdef cerulion"),
        "not a zsh script:\n{script}"
    );
    assert!(
        script.contains("COMPLETE=\"zsh\""),
        "missing the protocol var:\n{script}"
    );
}
