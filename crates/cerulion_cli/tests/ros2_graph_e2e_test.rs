// SPDX-License-Identifier: AGPL-3.0-only
//! `ros2:` graph entries over the REAL binary: `cerulion graph run` spawns
//! each entry on the staged `rmw_cerulion` env, supervises it, tears it down
//! gracefully on shutdown, and applies `--peer-loss` to its death — with NO
//! ROS 2 installed.
//!
//! A fixture `ros2` shell script sits FIRST on the child's `PATH` (the
//! spawner resolves `ros2` through `PATH`, so no product seam substitutes it
//! — the same zero-surface trick `ros2_run_e2e_test.rs` uses); a fixture lib
//! dir carries an empty `librmw_cerulion.so`; `HOME` points at a tempdir so
//! the ament-prefix staging never touches the real `~/.cerulion`. The fixture
//! writes the env + argv it saw to a per-entry report file, then idles until
//! SIGINT (which it RECORDS before exiting — so a report carrying `sigint`
//! proves the graceful fan-out reached it, not the SIGKILL backstop) — or
//! exits with `CER_TEST_EXIT` to model a crash.
//!
//! Needs `cargo build -p test_node_macro_period_cdylib` first (the native
//! half of the mixed graph). `#[serial]`: the real binary joins the default
//! iceoryx2 namespace; unique prefixes per test.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";

const FAKE_ROS2: &str = r#"#!/bin/sh
name="$3"
if [ "$1" = "launch" ]; then name=$(basename "$2"); fi
f="$CER_TEST_REPORT_DIR/$name.txt"
{
  echo "argv=$*"
  echo "rmw=$RMW_IMPLEMENTATION"
  echo "ament=$AMENT_PREFIX_PATH"
  echo "pid=$$"
} > "$f"
if [ -n "$CER_TEST_EXIT" ]; then exit "$CER_TEST_EXIT"; fi
trap 'echo sigint >> "$f"; exit 0' INT
trap 'echo sigterm >> "$f"; exit 0' TERM
while :; do sleep 0.1; done
"#;

/// The two `ros2:` entries every mixed-graph arm declares: a `ros2 run` form
/// with a params file + inline param, and a `ros2 launch` form.
const TWO_ENTRIES: &str = "\
- id: move_group
  ros2:
    package: moveit_ros_move_group
    executable: move_group
    params_file: config/params.yaml
    params:
      rate: 10
- id: bringup
  ros2:
    launch: launch/robot.launch.py
    args: [\"use_rviz:=false\"]
";

/// A single `ros2 run`-form entry (for the death arms).
const ONE_ENTRY: &str = "\
- id: move_group
  ros2:
    package: moveit_ros_move_group
    executable: move_group
";

fn write_executable(path: &Path, body: &str) {
    use std::os::unix::fs::PermissionsExt;
    std::fs::write(path, body).expect("write script");
    let mut perms = std::fs::metadata(path).expect("meta").permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(path, perms).expect("chmod");
}

/// A workspace with ONE native period node (`ticker`, the prebuilt fixture)
/// plus `entries` appended to `nodes:`, the launch/params fixture files the
/// entries name, the fake `ros2` on its own PATH dir, the rmw fixture lib dir,
/// a HOME for staging and a report dir.
struct Sandbox {
    root: tempfile::TempDir,
}

/// The default native half: ONE period node (`ticker`, the prebuilt fixture).
const TICKER: &str = "\
- id: ticker
  type: ticker
  outputs:
  - name: cmd
    schema: geometry_msgs/Vector3
";

impl Sandbox {
    /// The default shape: `ticker` first, then `entries` appended.
    fn new(prefix: &str, entries: &str) -> Self {
        Self::with_nodes(prefix, &format!("{TICKER}{entries}"))
    }

    /// A workspace whose `nodes:` block is `nodes_yaml` VERBATIM — for the
    /// arms that pin authored ORDER (an entry interleaved between natives).
    fn with_nodes(prefix: &str, nodes_yaml: &str) -> Self {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        for d in [
            "graphs",
            "nodes/ticker/src",
            "target/debug",
            "bin",
            "libs",
            "home",
            "reports",
            "config",
            "launch",
        ] {
            std::fs::create_dir_all(root.join(d)).expect("mkdir");
        }
        std::fs::write(
            root.join("Cargo.toml"),
            "[workspace]\nresolver = \"2\"\nmembers = []\n",
        )
        .expect("write Cargo.toml");
        std::fs::write(
            root.join("graphs/demo.yaml"),
            format!("prefix: {prefix}\nnodes:\n{nodes_yaml}"),
        )
        .expect("write graph");
        let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("repo root")
            .join("test_fixtures")
            .join(PERIOD_FIXTURE)
            .join("src/lib.rs");
        std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy src");
        std::fs::copy(
            fixture_cdylib(PERIOD_FIXTURE),
            root.join("target/debug").join(dylib_file("ticker")),
        )
        .expect("copy fixture cdylib");
        write_executable(&root.join("bin/ros2"), FAKE_ROS2);
        std::fs::write(root.join("libs/librmw_cerulion.so"), b"fixture").expect("write rmw");
        std::fs::write(root.join("config/params.yaml"), b"{}").expect("write params");
        std::fs::write(root.join("launch/robot.launch.py"), b"# fixture").expect("write launch");
        Sandbox { root: tmp }
    }

    fn root(&self) -> &Path {
        self.root.path()
    }

    /// `cerulion <args>` in the workspace with the sandbox env staged onto
    /// the CHILD only (parallel-safe; the file is `#[serial]` for iceoryx2).
    fn cerulion(&self, args: &[&str]) -> Command {
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        // The sandbox `bin` FIRST (so `ros2` resolves to the fixture), then the
        // system dirs the fixture script itself needs (`basename`, `sleep`).
        // No real `ros2` lives in those.
        let path = format!("{}:/usr/bin:/bin", self.root().join("bin").display());
        cmd.args(args)
            .current_dir(self.root())
            .env("PATH", path)
            .env("CERULION_LIB_DIR", self.root().join("libs"))
            .env("HOME", self.root().join("home"))
            .env("CER_TEST_REPORT_DIR", self.root().join("reports"))
            .env("CERULION_NETWORK", "off")
            .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
            .env_remove("CARGO_TARGET_DIR")
            .env_remove("CERULION_ROS2_PRELOAD")
            // The login gate is on by default; this file is not about the gate.
            .env("CERULION_LOGIN_GATE", "off")
            .env_remove("CER_TEST_EXIT")
            .stdin(Stdio::null());
        cmd
    }

    /// Spawn a blocking `graph run demo --no-validate <extra>` with stdio to
    /// files; returns the guard + the stderr path.
    fn spawn_run(&self, extra: &[&str]) -> (ChildGuard, PathBuf) {
        let stderr_path = self.root().join("run.stderr");
        let mut cmd = self.cerulion(&["graph", "run", "demo", "--no-validate"]);
        cmd.args(extra)
            .stdout(Stdio::from(
                std::fs::File::create(self.root().join("run.stdout")).expect("stdout"),
            ))
            .stderr(Stdio::from(
                std::fs::File::create(&stderr_path).expect("stderr"),
            ));
        // The ros2 entries are CHILD processes of this run, so teardown must
        // reach the group — an orphaned entry outlives the whole binary.
        let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
        (guard, stderr_path)
    }

    fn report_path(&self, name: &str) -> PathBuf {
        self.root().join("reports").join(format!("{name}.txt"))
    }

    /// Bounded wait for an entry's report file (the fixture writes it as its
    /// FIRST act, so its presence = the child was spawned + saw its env).
    fn wait_for_report(&self, name: &str, timeout: Duration) -> String {
        let path = self.report_path(name);
        let start = Instant::now();
        loop {
            let text = read_file(&path);
            if text.contains("pid=") {
                return text;
            }
            assert!(
                start.elapsed() < timeout,
                "report for `{name}` not written within {timeout:?}; run stderr:\n{}",
                read_file(&self.root().join("run.stderr"))
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }
}

fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{}",
            read_file(path)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn line<'a>(report: &'a str, key: &str) -> &'a str {
    let prefix = format!("{key}=");
    report
        .lines()
        .find(|l| l.starts_with(&prefix))
        .map(|l| &l[prefix.len()..])
        .unwrap_or_else(|| panic!("report has no `{key}=` line:\n{report}"))
}

/// The node ids of a rendered graph.yaml, in document order.
fn node_order(yaml: &str) -> Vec<String> {
    cerulion_core::graph::parse_graph_raw(yaml)
        .expect("the embedded graph.yaml must parse")
        .nodes
        .iter()
        .map(|n| n.id.clone())
        .collect()
}

fn pid_of(report: &str) -> u32 {
    line(report, "pid").trim().parse().expect("pid line")
}

/// `kill(pid, 0)` — true while the pid still names a live (or zombie) process.
fn pid_alive(pid: u32) -> bool {
    // SAFETY: signal 0 delivers nothing; it only probes pid existence.
    unsafe { libc::kill(pid as libc::pid_t, 0) == 0 }
}

/// HEADLINE (single-process arm): both entries spawn on the staged transport
/// env (`RMW_IMPLEMENTATION=rmw_cerulion`, an ament prefix staged under the
/// sandbox HOME), receive the argv the graph declared (`run pkg exe
/// --ros-args --params-file <abs> -p rate:=10` / `launch <abs file> arg`),
/// and on Ctrl-C the run exits 0 having sent each child a GRACEFUL SIGINT
/// (the report records it) — no orphan survives.
#[test]
#[serial]
fn graph_run_spawns_entries_on_rmw_cerulion_and_reaps_them_on_sigint() {
    let sb = Sandbox::new("r2a", TWO_ENTRIES);
    let (mut child, stderr) = sb.spawn_run(&["--single-process"]);

    let mg = sb.wait_for_report("move_group", Duration::from_secs(30));
    let bu = sb.wait_for_report("robot.launch.py", Duration::from_secs(30));
    for report in [&mg, &bu] {
        assert_eq!(line(report, "rmw"), "rmw_cerulion", "{report}");
        assert!(
            Path::new(line(report, "ament")).starts_with(sb.root().join("home/.cerulion/ros2")),
            "ament prefix must be staged under the sandbox HOME: {report}"
        );
    }
    let mg_argv = line(&mg, "argv");
    assert!(
        mg_argv.starts_with("run moveit_ros_move_group move_group --ros-args --params-file "),
        "{mg_argv}"
    );
    assert!(
        mg_argv.contains("config/params.yaml -p rate:=10"),
        "{mg_argv}"
    );
    let bu_argv = line(&bu, "argv");
    assert!(bu_argv.starts_with("launch "), "{bu_argv}");
    assert!(
        bu_argv.ends_with("launch/robot.launch.py use_rviz:=false"),
        "{bu_argv}"
    );
    let (mg_pid, bu_pid) = (pid_of(&mg), pid_of(&bu));
    assert!(
        pid_alive(mg_pid) && pid_alive(bu_pid),
        "children must be running"
    );

    wait_for_log_line(&stderr, "starting graph (live)", Duration::from_secs(30));
    send_signal(child.id(), libc::SIGINT);
    let status = child
        .wait_bounded(Duration::from_secs(40))
        .expect("run must exit");
    assert_eq!(status.code(), Some(0), "stderr:\n{}", read_file(&stderr));

    for (name, pid) in [("move_group", mg_pid), ("robot.launch.py", bu_pid)] {
        let report = read_file(&sb.report_path(name));
        assert!(
            report.contains("sigint"),
            "`{name}` must have received the graceful SIGINT fan-out:\n{report}"
        );
        assert!(
            !pid_alive(pid),
            "`{name}` (pid {pid}) must be reaped — no orphan"
        );
    }
    assert!(
        read_file(&stderr).contains("ros2 entries stopped"),
        "stderr:\n{}",
        read_file(&stderr)
    );
}

/// `--peer-loss fail`: an entry that DIES mid-run (non-zero exit) stops the
/// run, which exits non-zero naming the entry.
#[test]
#[serial]
fn peer_loss_fail_stops_the_run_when_an_entry_dies() {
    let sb = Sandbox::new("r2b", ONE_ENTRY);
    let mut cmd = sb.cerulion(&[
        "graph",
        "run",
        "demo",
        "--no-validate",
        "--single-process",
        "--peer-loss",
        "fail",
    ]);
    let stderr_path = sb.root().join("run.stderr");
    cmd.env("CER_TEST_EXIT", "3")
        .stdout(Stdio::null())
        .stderr(Stdio::from(
            std::fs::File::create(&stderr_path).expect("stderr"),
        ));
    let mut child = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn");
    let status = child
        .wait_bounded(Duration::from_secs(60))
        .expect("run must exit");
    let stderr = read_file(&stderr_path);
    assert_ne!(
        status.code(),
        Some(0),
        "a dead entry under --peer-loss fail must fail the run:\n{stderr}"
    );
    assert!(stderr.contains("died mid-run"), "{stderr}");
    assert!(stderr.contains("'move_group'"), "{stderr}");
    assert!(stderr.contains("--peer-loss fail"), "{stderr}");
}

/// Default `--peer-loss continue`: a dead entry is a loud DEGRADED warn and
/// the graph keeps running; Ctrl-C still exits 0.
#[test]
#[serial]
fn peer_loss_continue_keeps_the_graph_running_after_an_entry_dies() {
    let sb = Sandbox::new("r2c", ONE_ENTRY);
    let stderr_path = sb.root().join("run.stderr");
    let mut cmd = sb.cerulion(&["graph", "run", "demo", "--no-validate", "--single-process"]);
    cmd.env("CER_TEST_EXIT", "3")
        .stdout(Stdio::null())
        .stderr(Stdio::from(
            std::fs::File::create(&stderr_path).expect("stderr"),
        ));
    let mut child = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn");

    wait_for_log_line(&stderr_path, "DEGRADED", Duration::from_secs(30));
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(30),
    );
    std::thread::sleep(Duration::from_millis(500));
    assert!(
        child.try_wait_noting().expect("try_wait").is_none(),
        "the graph must keep running after a degraded entry death:\n{}",
        read_file(&stderr_path)
    );
    send_signal(child.id(), libc::SIGINT);
    let status = child
        .wait_bounded(Duration::from_secs(40))
        .expect("run must exit");
    assert_eq!(status.code(), Some(0), "{}", read_file(&stderr_path));
}

/// The deployment precondition: a missing `librmw_cerulion.so` refuses the run
/// LOUDLY before any entry is spawned (no report is ever written).
#[test]
#[serial]
fn missing_rmw_lib_refuses_the_run_before_spawning() {
    let sb = Sandbox::new("r2d", ONE_ENTRY);
    let rmw_lib = sb.root().join("libs/librmw_cerulion.so");
    std::fs::remove_file(&rmw_lib).expect("remove rmw");
    let out = sb
        .cerulion(&["graph", "run", "demo", "--no-validate", "--single-process"])
        .output()
        .expect("run");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_ne!(out.status.code(), Some(0), "{stderr}");
    assert!(stderr.contains("librmw_cerulion.so"), "{stderr}");
    // The graph path stages its ros2 children through the same seam as the
    // verb, so it prints the same host-keyed remedy and never a cargo command.
    let expected =
        cerulion_cli_engine::ros2_cmd::missing_rmw_lib_message(std::env::consts::OS, &rmw_lib);
    assert!(stderr.contains(&expected), "{stderr}");
    assert!(!stderr.contains("cargo build"), "{stderr}");
    assert!(
        !sb.report_path("move_group").exists(),
        "no entry may be spawned when the rmw lib is missing"
    );
}

/// `graph validate` reports each entry (and passes); `graph levels` lists them
/// apart from the DAG as spawned, unscheduled processes.
#[test]
#[serial]
fn validate_and_levels_accept_the_mixed_graph() {
    let sb = Sandbox::new("r2e", TWO_ENTRIES);
    let out = sb
        .cerulion(&["graph", "validate", "demo"])
        .output()
        .expect("validate");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(
        out.status.code(),
        Some(0),
        "stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(
        stdout.contains("ros2 entry 'move_group'") && stdout.contains("ros2 entry 'bringup'"),
        "stdout:\n{stdout}"
    );

    let out = sb
        .cerulion(&["graph", "levels", "demo"])
        .output()
        .expect("levels");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(
        out.status.code(),
        Some(0),
        "stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(stdout.contains("ros2 processes"), "stdout:\n{stdout}");
    assert!(
        stdout.contains("move_group [ros2 run moveit_ros_move_group move_group"),
        "stdout:\n{stdout}"
    );
    assert!(
        stdout.contains("nodes: 1 "),
        "the DAG count excludes ros2 entries:\n{stdout}"
    );
}

/// The multi-process DEFAULT (no `--single-process`; the non-TTY floor derives
/// the partition in memory): the supervisor arm spawns + reaps the entries
/// exactly like the monolith arms.
#[test]
#[serial]
fn multi_process_default_spawns_and_reaps_entries() {
    let sb = Sandbox::new("r2f", ONE_ENTRY);
    let (mut child, stderr) = sb.spawn_run(&[]);
    let mg = sb.wait_for_report("move_group", Duration::from_secs(30));
    assert_eq!(line(&mg, "rmw"), "rmw_cerulion");
    let mg_pid = pid_of(&mg);
    // The supervisor's own readiness breadcrumb (workers spawned, GO issued).
    wait_for_log_line(&stderr, "GO", Duration::from_secs(60));
    send_signal(child.id(), libc::SIGINT);
    let status = child
        .wait_bounded(Duration::from_secs(60))
        .expect("run must exit");
    assert_eq!(status.code(), Some(0), "stderr:\n{}", read_file(&stderr));
    let report = read_file(&sb.report_path("move_group"));
    assert!(
        report.contains("sigint"),
        "graceful fan-out under the supervisor:\n{report}"
    );
    assert!(!pid_alive(mg_pid), "no orphan under the supervisor arm");
}

/// Needs a ROS 2 Jazzy container: the same graph against a REAL ROS 2 install (rclcpp `demo_nodes_cpp
/// talker` under `rmw_cerulion`). Not runnable on CI (no ROS distro, no
/// built `librmw_cerulion.so`). Recipe, inside the `ros2-bench` Jazzy
/// container with the repo bind-mounted at `/work`:
///
/// ```text
/// source /opt/ros/jazzy/setup.bash
/// cargo build --release -p rmw_cerulion -p cerulion_cli -p test_node_macro_period_cdylib
/// CERULION_LIB_DIR=/work/target/release \
///   cargo test -p cerulion_cli --test ros2_graph_e2e_test -- --ignored real_ros2 --nocapture
/// ```
///
/// Asserts: the talker's `/chatter` appears in `cerulion topic list` (its
/// frames ride the shared SHM transport), and Ctrl-C reaps it.
#[test]
#[ignore]
#[serial]
fn real_ros2_talker_publishes_over_rmw_cerulion() {
    let sb = Sandbox::new(
        "r2real",
        "- id: talker\n  ros2:\n    package: demo_nodes_cpp\n    executable: talker\n",
    );
    let lib_dir = std::env::var("CERULION_LIB_DIR").expect("CERULION_LIB_DIR (box recipe)");
    let real_path = std::env::var("PATH").expect("PATH");
    let stderr_path = sb.root().join("run.stderr");
    let mut cmd = sb.cerulion(&["graph", "run", "demo", "--no-validate", "--single-process"]);
    cmd.env("PATH", real_path)
        .env("CERULION_LIB_DIR", lib_dir)
        .env_remove("HOME")
        .stdout(Stdio::null())
        .stderr(Stdio::from(
            std::fs::File::create(&stderr_path).expect("stderr"),
        ));
    let mut child = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn");
    wait_for_log_line(&stderr_path, "ros2 entry spawned", Duration::from_secs(60));
    std::thread::sleep(Duration::from_secs(5));
    let list = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", "list", "--no-network"])
        .current_dir(sb.root())
        .output()
        .expect("topic list");
    let stdout = String::from_utf8_lossy(&list.stdout);
    assert!(
        stdout.contains("chatter"),
        "talker's /chatter must be on the shared transport:\n{stdout}"
    );
    send_signal(child.id(), libc::SIGINT);
    let status = child
        .wait_bounded(Duration::from_secs(60))
        .expect("run must exit");
    assert_eq!(status.code(), Some(0), "{}", read_file(&stderr_path));
}

/// Decision (bags EMBED + skip): a mixed-graph `--record` bag carries
/// the `ros2:` entries VERBATIM in its embedded graph, `bag info` renders
/// them as their own section, and `bag play --resim --verify` re-executes
/// the NATIVE half to exit 0 — naming the skipped entries in the warn AND
/// the `--report` JSON, never respawning ROS 2 (the fake `ros2` would write
/// a fresh report if it were spawned; the structural half of that pin is the
/// source walk in `cerulion_cli_engine/tests/ros2_resim_no_respawn_test.rs`).
///
/// Removing the preflight's take-and-skip
/// arm makes this resim REFUSE the bag (the embedded entry has no factory),
/// failing the exit-0 assert.
#[test]
#[serial]
fn recorded_mixed_graph_resims_exit_0_and_names_the_skipped_entries() {
    // INTERLEAVED on purpose (authored order is preserved in
    // both embeds): native `ticker`, then the ros2 entry, then native `ticker2`.
    let sb = Sandbox::with_nodes(
        "r2g",
        &format!(
            "{TICKER}{ONE_ENTRY}- id: ticker2\n  type: ticker\n  outputs:\n  - name: cmd\n    \
             schema: geometry_msgs/Vector3\n"
        ),
    );
    // `--record` conflicts with `--no-validate` (the recording IS the golden,
    // so it must be validated) — spawn without the helper's `--no-validate`.
    let stderr = sb.root().join("run.stderr");
    let mut cmd = sb.cerulion(&[
        "graph",
        "run",
        "demo",
        "--single-process",
        "--record=recordings",
    ]);
    cmd.stdout(Stdio::from(
        std::fs::File::create(sb.root().join("run.stdout")).expect("stdout"),
    ))
    .stderr(Stdio::from(std::fs::File::create(&stderr).expect("stderr")));
    let mut child = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn record run");
    sb.wait_for_report("move_group", Duration::from_secs(30));
    let bag = {
        let recordings = sb.root().join("recordings");
        let start = Instant::now();
        loop {
            if let Some(b) = mp_support::wait_for_bag(&recordings, Duration::from_secs(5)) {
                break b;
            }
            assert!(
                start.elapsed() < Duration::from_secs(60),
                "no bag appeared; stderr:\n{}",
                read_file(&stderr)
            );
        }
    };
    // SEAM 1 — the run directory's graph.yaml (read MID-RUN: the directory is
    // removed when the run ends): authored order, interleaving included.
    let run_dir_order = {
        let runs = sb.root().join("home/.cerulion/runs");
        let start = Instant::now();
        loop {
            let found = std::fs::read_dir(&runs)
                .ok()
                .into_iter()
                .flatten()
                .flatten()
                .map(|e| e.path().join("graph.yaml"))
                .find(|p| p.exists());
            if let Some(path) = found {
                break node_order(&read_file(&path));
            }
            assert!(
                start.elapsed() < Duration::from_secs(30),
                "no run directory graph.yaml appeared under {}",
                runs.display()
            );
            std::thread::sleep(Duration::from_millis(100));
        }
    };
    assert_eq!(
        run_dir_order,
        ["ticker", "move_group", "ticker2"],
        "the run directory's graph.yaml must carry the authored node order"
    );
    // Let the recording capture some steps, then stop cleanly.
    std::thread::sleep(Duration::from_secs(2));
    send_signal(child.id(), libc::SIGINT);
    let status = child
        .wait_bounded(Duration::from_secs(60))
        .expect("record run exits");
    assert_eq!(status.code(), Some(0), "stderr:\n{}", read_file(&stderr));

    // The fake ros2's report is consumed so a RESPAWN during resim would be
    // observable as the file reappearing.
    std::fs::remove_file(sb.report_path("move_group")).expect("consume the spawn report");

    // SEAM 2 — the bag's embedded graph.yaml: the same authored order.
    let bag_order = {
        let reader = cerulion_bag::BagReader::open(&bag).expect("open the recorded bag");
        let att = reader
            .attachment("graph.yaml")
            .expect("attachment index readable")
            .expect("the bag must embed graph.yaml");
        node_order(std::str::from_utf8(&att.data).expect("utf-8 graph.yaml"))
    };
    assert_eq!(
        bag_order,
        ["ticker", "move_group", "ticker2"],
        "the bag's embedded graph.yaml must carry the authored node order"
    );

    // `bag info` renders the entries as their own section.
    let info = sb
        .cerulion(&["bag", "info", &bag.display().to_string()])
        .output()
        .expect("bag info");
    let info_out = String::from_utf8_lossy(&info.stdout);
    assert_eq!(info.status.code(), Some(0), "{info_out}");
    assert!(
        info_out.contains("ros2 entries"),
        "bag info must render the section:\n{info_out}"
    );
    assert!(info_out.contains("move_group"), "{info_out}");

    // Resim with verify + report: exit 0, the warn names the entry, the
    // report JSON carries it, and the fake ros2 was NEVER respawned.
    let report_json = sb.root().join("resim_report.json");
    let out = sb
        .cerulion(&[
            "bag",
            "play",
            &bag.display().to_string(),
            "--resim",
            "all",
            "--verify",
            "--report",
            &report_json.display().to_string(),
        ])
        .output()
        .expect("resim");
    let resim_stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(
        out.status.code(),
        Some(0),
        "a mixed-graph bag must resim to exit 0 (the skip arm):\n{resim_stderr}"
    );
    assert!(
        resim_stderr.contains("not re-executed") && resim_stderr.contains("move_group"),
        "the skip warn must name the entry:\n{resim_stderr}"
    );
    let report = std::fs::read_to_string(&report_json).expect("report JSON written");
    let parsed: serde_json::Value = serde_json::from_str(&report).expect("report parses");
    assert_eq!(
        parsed["ros2_entries_skipped"],
        serde_json::json!(["move_group"]),
        "report:\n{report}"
    );
    assert!(
        !sb.report_path("move_group").exists(),
        "the resim must NEVER respawn a ros2 entry (the fake would have written its report)"
    );
}
