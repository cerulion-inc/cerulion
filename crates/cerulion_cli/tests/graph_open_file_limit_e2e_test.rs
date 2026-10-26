// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run` and `cerulion graph profile` refuse a graph estimated
//! to need more open files than the process may hold, before they touch the
//! workspace, and `CERULION_OPEN_FILE_CHECK=0` turns the refusal into a
//! warning.
//!
//! REAL-BINARY: each arm starts `cerulion` with its open-file limit set to
//! 1024 soft and 1024 hard before `exec` (a hard limit the raise cannot pass),
//! on a workspace whose one graph has 40 source to sink chains: 80 nodes, 40
//! outputs and 40 inputs, estimated at 40 + 40 x 14 + 40 x 22 = 1480 files.
//! The node types have no libraries, so a run the check lets through fails
//! later, at the library load, with a different error. The refusing arms
//! snapshot every file under the workspace before and after the run and
//! require the two to be equal.
#![cfg(unix)]

use std::collections::BTreeMap;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};
use std::time::{Duration, Instant};

const GRAPH: &str = "wide";
const LIMIT: libc::rlim_t = 1024;
const REFUSAL: &str = "graph 'wide': this graph has 80 nodes with 40 output and 40 input \
                       ports, and starting it is estimated to open about 1480 files in this \
                       process";
const ESCAPE: &str = "To start it anyway, set CERULION_OPEN_FILE_CHECK=0.";
const OVERRIDE_WARNING: &str =
    "CERULION_OPEN_FILE_CHECK=0: starting a graph the open-file check refuses";

/// A workspace whose `graphs/wide.yaml` holds 40 `src{i}` -> `sink{i}` chains.
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let mut yaml = format!("prefix: fdl{}\nnodes:\n", std::process::id());
    for i in 0..40 {
        for line in [
            format!("  - id: src{i}"),
            "    type: src".to_string(),
            "    outputs:".to_string(),
            "      - name: out".to_string(),
            "        schema: geometry_msgs/Vector3".to_string(),
            format!("  - id: sink{i}"),
            "    type: sink".to_string(),
            "    inputs:".to_string(),
            "      - name: in".to_string(),
            format!("        source: src{i}/out"),
        ] {
            yaml.push_str(&line);
            yaml.push('\n');
        }
    }
    std::fs::write(root.join("graphs").join(format!("{GRAPH}.yaml")), yaml).unwrap();
}

/// Every file under `root` with its bytes.
fn snapshot(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
    fn walk(dir: &Path, root: &Path, out: &mut BTreeMap<PathBuf, Vec<u8>>) {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                walk(&path, root, out);
            } else {
                let bytes = std::fs::read(&path).unwrap();
                out.insert(path.strip_prefix(root).unwrap().to_path_buf(), bytes);
            }
        }
    }
    let mut out = BTreeMap::new();
    walk(root, root, &mut out);
    out
}

/// Runs `cerulion <args>` in `root` at a 1024 soft and hard open-file limit,
/// with `extra_env` set, and returns its output. Kills it after 60 s.
fn run_at_1024(root: &Path, args: &[&str], extra_env: &[(&str, &str)]) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env_remove("CERULION_OPEN_FILE_CHECK")
        .env_remove("RUST_LOG")
        .env("CERULION_NETWORK", "off")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for (key, value) in extra_env {
        cmd.env(key, value);
    }
    let rl = libc::rlimit {
        rlim_cur: LIMIT,
        rlim_max: LIMIT,
    };
    // SAFETY: the closure runs in the forked child before `exec` and calls
    // only `setrlimit`, which is async-signal-safe, on a copied local.
    unsafe {
        cmd.pre_exec(move || {
            if libc::setrlimit(libc::RLIMIT_NOFILE, &rl) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let mut child = cmd.spawn().expect("spawn cerulion");
    let deadline = Instant::now() + Duration::from_secs(60);
    while child.try_wait().expect("try_wait").is_none() {
        if Instant::now() > deadline {
            let _ = child.kill();
            let out = child.wait_with_output().expect("collect the killed child");
            panic!(
                "cerulion {args:?} did not exit within 60 s\nstderr:\n{}",
                String::from_utf8_lossy(&out.stderr)
            );
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    child.wait_with_output().expect("collect cerulion output")
}

fn assert_refused_without_touching_the_workspace(args: &[&str]) {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    build_workspace(root);
    let before = snapshot(root);
    let out = run_at_1024(root, args, &[]);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        !out.status.success(),
        "cerulion {args:?} must fail, got {}\nstderr:\n{stderr}",
        out.status
    );
    assert!(stderr.contains(REFUSAL), "no refusal in stderr:\n{stderr}");
    assert!(
        stderr.contains("RLIMIT_NOFILE) is 1024 with a hard limit of 1024"),
        "{stderr}"
    );
    assert!(stderr.contains(ESCAPE), "{stderr}");
    assert_eq!(
        snapshot(root),
        before,
        "the refusal must leave every workspace file as it was"
    );
}

#[test]
fn graph_run_refuses_a_graph_above_the_limit_before_touching_the_workspace() {
    assert_refused_without_touching_the_workspace(&[
        "graph",
        "run",
        GRAPH,
        "--single-process",
        "--no-validate",
    ]);
}

#[test]
fn graph_profile_refuses_a_graph_above_the_limit_before_touching_the_workspace() {
    assert_refused_without_touching_the_workspace(&["graph", "profile", GRAPH]);
}

#[test]
fn the_override_warns_and_lets_graph_run_continue_past_the_check() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    build_workspace(root);
    let out = run_at_1024(
        root,
        &["graph", "run", GRAPH, "--single-process", "--no-validate"],
        &[("CERULION_OPEN_FILE_CHECK", "0")],
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains(OVERRIDE_WARNING),
        "no override warning in stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("estimated to open about 1480 files"),
        "the warning carries the estimate:\n{stderr}"
    );
    assert!(
        !stderr.contains(ESCAPE),
        "the refusal must not be raised under the override:\n{stderr}"
    );
    assert!(
        !out.status.success(),
        "the graph's node types have no libraries, so the run fails past the check"
    );
}
