// SPDX-License-Identifier: AGPL-3.0-only
//! `cli-node-run` gap: `cerulion node run <type>` (`cerulion_cli/
//! src/main.rs`'s `NodeAction::Run` arm) builds a TEMPORARY single-node graph
//! (`graphs/__temp_<node_type>.yaml`) wrapping the proven `graph_cmd::
//! graph_run` path, then deletes the temp graph file when `graph_run`
//! returns. Before this file the subcommand was tested ONLY at the clap
//! arg-parsing layer (`main.rs`'s `#[cfg(test)]` module — `node_run_release_
//! flag_parses` etc.); nothing drove it end-to-end.
//!
//! Spawns the REAL `cerulion` binary as `cerulion node run ticker --prefix
//! <prefix>` against a hand-built tempdir workspace whose node cdylib is a
//! COPY of the prebuilt `test_node_macro_period_cdylib` fixture
//! (`#[cerulion_node(period_ms = 50)]`, one `cmd: Vector3` output — no
//! in-test cargo build), cribbed from `graph_record_e2e_test.rs`'s
//! `build_workspace`/`ChildGuard`/`wait_bounded` patterns. Proves:
//!
//! 1. The temp-graph wrapping actually WIRES UP and STARTS the node — a
//!    second `cerulion topic echo <topic>` subprocess against the topic the
//!    temp graph publishes to observes a real delivered frame (not just
//!    "the process launched and didn't crash").
//! 2. SIGINT (the production Ctrl-C) drains it to a clean (exit 0) stop.
//! 3. The temp graph file (`graphs/__temp_ticker.yaml`) is gone afterward —
//!    `main.rs`'s "Clean up temporary graph" `remove_file` call actually ran
//!    on the happy exit path (never previously observed by any test).
//!
//! Needs `cargo build -p test_node_macro_period_cdylib` first (the repo's
//! fixture pattern — this test panics with that instruction if the artifact
//! is missing).
//!
//! Runs on the GLOBAL iceoryx2 namespace (no isolation seam at this layer,
//! same as `graph_record_e2e_test.rs`); `#[serial]` + a unique per-run
//! prefix keep it safe.

#![cfg(unix)]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// SIGKILL + reap on drop so a panicking test never leaks a child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

fn send_signal(pid: u32, sig: libc::c_int) {
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

/// The platform cdylib filename for a crate/node name.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// The prebuilt fixture cdylib (`test_node_macro_period_cdylib` — a
/// `#[cerulion_node(period_ms = 50)]` node with one `cmd: Vector3` output).
/// PANICS with the build instruction if missing (the repo's fixture pattern).
fn fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib")
}

/// Hand-build a minimal workspace in `root` for `cerulion node run ticker`:
/// a `[workspace]` Cargo.toml, an EMPTY `graphs/` dir (workspace discovery
/// requires it to exist — `CerulionWorkspace::discover` checks
/// `graphs_dir.is_dir()` before matching a `Cargo.toml` — and `node run`
/// builds its OWN temp graph inside it, so no graph YAML is pre-seeded
/// here), `nodes/ticker/src/lib.rs` (a copy of the fixture source —
/// `node_cmd::node_info` reads ONLY this file for port/policy metadata),
/// and `target/debug/libticker.*` (a copy of the PREBUILT fixture cdylib —
/// no in-test cargo build).
fn build_workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures/test_node_macro_period_cdylib/src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        fixture_cdylib(),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy fixture cdylib");
}

/// Spawn `cerulion node run ticker --prefix <prefix>` in `root`, with
/// stdout+stderr redirected to FILES (readable while the child runs — no
/// pipe deadlock).
fn spawn_node_run(root: &Path, prefix: &str) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["node", "run", "ticker", "--prefix", prefix])
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target) + info-level lifecycle logs
        // (the "starting graph (live)" readiness line lives in
        // `cerulion_cli_engine::graph_cmd`).
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — `node run` rides the permissive network default;
        // the kill-switch env keeps CI runs LOCAL-ONLY (no scouting/gateway).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion node run");
    (ChildGuard(child), stdout_path, stderr_path)
}

/// Spawn `cerulion topic echo <topic>` in `root`, stdout to a file.
/// `topic echo`/`topic hz` never call `discover_workspace()` (`main.rs`'s
/// `Commands::Topic` arm is a bare top-level match, unlike `Commands::Node`/
/// `Commands::Graph`) — `root` here is just a convenient cwd, not a
/// functional requirement.
///
/// LOCAL-ONLY: the sibling `graph run`/`node run` child already carries
/// `CERULION_NETWORK=off`, but the `topic` verb child inherited nothing. `topic
/// echo`'s `ensure_topic_available` routes a topic that is NOT (yet) locally listed
/// to the netd demand rung on the automagic scouting-ON path, which
/// `connect_or_spawn()`s the WELL-KNOWN `cerulion-netd` socket — so in the race
/// window before the publisher's service lands in the local registry this test
/// could SPAWN a machine-wide network daemon on the developer's desk and burn a LAN
/// gather inside its bounded wait. The kill switch is consulted FIRST in
/// `ensure_topic_available`, and this test only ever echoes a LOCAL topic, so the
/// assertions are unchanged — the race now fails fast and loud instead.
fn spawn_topic_echo(root: &Path, topic: &str) -> (ChildGuard, PathBuf) {
    let stdout_path = root.join("echo.stdout");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", "echo", topic])
        .current_dir(root)
        .env("CERULION_NETWORK", "off")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn cerulion topic echo");
    (ChildGuard(child), stdout_path)
}

/// Bounded poll for `needle` in the (growing) log file at `path`; returns the
/// full content once seen, panics at the deadline (the caller's `ChildGuard`
/// reaps on the panic unwind).
fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let content = std::fs::read_to_string(path).unwrap_or_default();
        if content.contains(needle) {
            return content;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

/// THE e2e pin: `node run` wires + starts the node — a live `topic echo`
/// subprocess against the derived topic sees delivered frames — SIGINT
/// drains it to a clean exit, and the temp graph file is gone afterward.
#[test]
#[serial]
fn node_run_e2e_wires_temp_graph_publishes_and_cleans_up_on_sigint() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path());
    let prefix = "noderune2e";
    // `node run`'s temp graph stages the node under its own type as the
    // node ID (`id.as_deref()` is `None` when `-i` isn't passed), one
    // `cmd: Vector3` output, matching `build_node_def`'s derivation.
    let topic = format!("/{prefix}/ticker/cmd");

    let (mut guard, _stdout_path, stderr_path) = spawn_node_run(tmp.path(), prefix);

    // Readiness: `graph_cmd::graph_run`'s live path logs this line AFTER the
    // runtime (and hence the topic's publisher service) is built, right
    // before entering the live loop. It is a `tracing::info!`, which goes to
    // STDERR (init_logging writes there so a command's stdout stays clean data).
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );

    // Prove the node is ACTUALLY firing, not just that the process launched:
    // a second `cerulion topic echo` subprocess against the derived topic
    // must observe at least one delivered (50ms-period) frame.
    let (mut echo_guard, echo_stdout) = spawn_topic_echo(tmp.path(), &topic);
    std::thread::sleep(Duration::from_millis(1000));
    send_signal(echo_guard.0.id(), libc::SIGINT);
    let echo_status = wait_bounded(&mut echo_guard.0, Duration::from_secs(10))
        .expect("topic echo did not exit after SIGINT");
    assert!(
        echo_status.success(),
        "topic echo must exit 0 on SIGINT, got {echo_status:?}"
    );
    let echo_log = read_file(&echo_stdout);
    assert!(
        echo_log.contains("seq=") && echo_log.contains("schema=0x"),
        "topic echo on '{topic}' must have displayed at least one delivered \
         frame from the node-run-started ticker — the real proof the temp \
         graph actually wired up and fired; echo stdout was:\n{echo_log}"
    );

    // The production Ctrl-C: SIGINT to the `node run` child.
    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("node run did not exit after SIGINT");
    assert!(status.success(), "Ctrl-C must exit 0, got {status:?}");

    // The "Clean up temporary graph" `remove_file` in `main.rs`'s
    // `NodeAction::Run` arm must have actually run.
    let temp_graph = tmp.path().join("graphs/__temp_ticker.yaml");
    assert!(
        !temp_graph.exists(),
        "the temporary graph file must be deleted after `node run` exits, found: {}",
        temp_graph.display()
    );
}
