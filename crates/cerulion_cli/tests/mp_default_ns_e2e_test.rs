// SPDX-License-Identifier: AGPL-3.0-only
//! Default-namespace acceptance: the multi-process DATA PLANE lives on the DEFAULT
//! iceoryx2 namespace.
//!
//! Background: a `process_groups:` deployment used to mint a run-scoped
//! `cer_d_{fnv:016x}` SHM prefix (`mint_deployment_ix_config`), which
//! quarantined the whole data plane — `topic echo/hz/list/info`, cross-graph
//! absolute topics, and external publishers could never see a default
//! `graph run` deployment. The data plane was moved to the machine's resolved
//! global config (`Config::global_config()` — the DEFAULT namespace, prefix
//! `iox2_` on a config-file-free box); only infra stays run-scoped (barrier `cerdep_*`, trace
//! rings `cer_rec_*`/`cer_rg_*`, doorbells `/cer_db_<user>_*`, and the
//! supervisor's `cer_p_*` PLANNING namespace).
//!
//! Two subprocess-level pins over the REAL binary:
//!
//! * **TEST A — default-mp visibility** (the headline): a REAL
//!   2-group multi-process run (GO breadcrumb + >= 2 `run-worker` children
//!   asserted — never `--single-process`) is visible to separately-spawned
//!   `cerulion topic list` and delivers >= 1 frame to `cerulion topic echo`,
//!   both of which attach via `TransportManager::init` = the DEFAULT
//!   namespace. HAND ORACLE: the OLD behavior — earlier this exact shape
//!   saw NOTHING (`topic list` empty of the graph's topics, `topic echo`
//!   zero frames; `topic_introspect_cli_e2e_test.rs` had to force
//!   `--single-process` for precisely this reason) — so a frame received IS
//!   the proof the mp data plane is default-ns, not a self-compare. This
//!   test doubles as the hardware acceptance harness shape (the humanoid+driver
//!   graphs run the same run-then-introspect-externally recipe on a robot computer).
//!
//! * **TEST B — concurrent-same-graph loud collision** (the decision's accepted
//!   consequence): with run A live, a second run B of the SAME graph (same
//!   workspace, same prefix) exits NONZERO carrying the single-writer
//!   collision signature, and A SURVIVES (still alive, still delivering
//!   frames, then SIGINT -> clean exit 0 — the incumbent-survival shape of
//!   `cross_graph_collision_iox2_test.rs`). MECHANISM (read, not guessed):
//!   B's PLANNING build is on B's own `cer_p_{hex}` namespace (nonce =
//!   pid+timestamp, distinct from A's) so it does NOT collide; B's first
//!   spawned worker (producer-owning group p0 first, `spawn_order`) builds
//!   on the DEFAULT namespace and its `ticker/cmd` publisher hits the
//!   active-publisher single-writer pre-check
//!   (`cerulion_core/src/transport/mod.rs` ~:1640, "the topic already has N
//!   attached publisher(s) — graph topics are single-writer ... another
//!   graph (or another instance of this one)") -> the worker's build errors
//!   BEFORE it writes its READY sentinel (`graph_cmd.rs` step (8)) -> the
//!   worker process exits nonzero with `Error: ...` on the stderr it
//!   INHERITS from B's supervisor (workers are spawned with no stdio
//!   overrides) -> the supervisor's `wait_for_ready_or_exit`
//!   (`graph_cmd.rs` ~:3186) sees `ChildPoll::Exited` and returns the LOUD
//!   "exited (...) before signaling READY" error -> B's `graph run` exits
//!   nonzero. NOTE: `--peer-loss` is IRRELEVANT here — that policy governs
//!   the live JOIN loop only; a pre-READY worker death aborts the spawn
//!   phase unconditionally, so no flag is needed to make the collision
//!   terminal for B. HAND ORACLE: the collision error text is the
//!   independently-authored transport contract, and the earlier behavior
//!   (disjoint run-scoped namespaces) would have let B run SILENTLY
//!   side-by-side — B exiting nonzero on the single-writer check is exactly
//!   the designed protection the decision accepted.
//!
//! Assertion choice (TEST A): `topic echo` over `topic hz` — echo prints one
//! `seq=.. schema=0x..` line PER DELIVERED FRAME immediately on delivery, so
//! ">= 1 frame" is a direct substring assert with no report-interval
//! dependency; `hz` would also prove delivery but adds its 1s
//! `HZ_REPORT_INTERVAL` latency + rate parsing for no extra pin strength.
//! Neither verb has a bounded-run flag (confirmed in
//! `topic_introspect_cli_e2e_test.rs`), so echo runs for a bounded window and
//! is SIGINT'd — the same crib.
//!
//! Harness: the shared `mp_support` module (tempdir workspace, PREBUILT
//! fixture cdylibs — no in-test cargo build; the 3-node `ticker`(period 50ms)
//! -> `relay`(data-trigger) -> `sink`(data-trigger) chain split
//! `p0:[ticker,relay]` / `p1:[sink]`), `ChildGuard` process-GROUP teardown + orphan verdict, bounded
//! waits everywhere (no fixed sleep where a breadcrumb exists; the only
//! wall-clock windows are the unbounded-verb echo captures). Prerequisites
//! (the helpers PANIC with the instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]` (NOT linux-only): the mp supervisor is real
//! on macOS. `#[serial]`: the mp data plane IS the global default iceoryx2
//! namespace (unique per-test prefixes keep topic names apart; TEST B shares
//! one prefix between its two runs DELIBERATELY).

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{build_mp_workspace, read_file, send_signal, ChildGuard};

/// The supervisor's deployment-live breadcrumb (the mp-path marker — the same
/// pin `mp_record_e2e_test.rs` / `mp_auto_partition_e2e_test.rs` use).
const GO_MARKER: &str = "GO signaled; deployment live";

/// Spawn `cerulion graph run mpdemo --no-validate` (NO `--record`, NO
/// `--single-process` — this file needs a REAL default multi-process run) in
/// `root`, stdout + stderr redirected to `label`-scoped files so TEST B's two
/// concurrent runs don't clobber each other's logs. stdin is `/dev/null`
/// (belt-and-suspenders; the graph HAS `process_groups:`, so no auto-partition
/// consent flow runs).
fn spawn_graph_run_mp(root: &Path, label: &str) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join(format!("{label}.stdout"));
    let stderr_path = root.join(format!("{label}.stderr"));
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", "mpdemo", "--no-validate"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    let guard =
        ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run (multi-process)");
    (guard, stdout_path, stderr_path)
}

/// Bounded poll until `path`'s contents contain `needle` (crib of
/// `mp_auto_partition_e2e_test.rs::wait_for_log`).
fn wait_for_log(path: &Path, needle: &str, deadline: Duration) -> bool {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return true;
        }
        if start.elapsed() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// stdout + stderr merged (tracing's writer choice must not decide a pin).
fn merged_log(stdout_path: &Path, stderr_path: &Path) -> String {
    format!("{}\n{}", read_file(stdout_path), read_file(stderr_path))
}

/// The `run-worker` children of a supervisor pid (crib of
/// `mp_auto_partition_e2e_test.rs::worker_pids`).
fn worker_pids(supervisor_pid: u32) -> Vec<u32> {
    let out = Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", "run-worker"])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Bounded poll until the supervisor has >= `n` `run-worker` children.
fn wait_for_workers(supervisor_pid: u32, n: usize, deadline: Duration) -> Vec<u32> {
    let start = Instant::now();
    loop {
        let pids = worker_pids(supervisor_pid);
        if pids.len() >= n || start.elapsed() >= deadline {
            return pids;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Run `cerulion topic list` as a separate process (it attaches via the
/// DEFAULT iceoryx2 namespace — `Config::global_config()`), bounded, and
/// return its stdout. `topic list` exits on its own; the bound only guards a
/// hang from wedging CI.
fn run_topic_list(root: &Path) -> String {
    let stdout_path = root.join("topic_list.stdout");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        // Hermetic — `--no-network` skips the automagic remote-discovery
        // scouting query; this test observes only LOCAL shared-memory topics.
        .args(["topic", "list", "--no-network"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn cerulion topic list");
    // A one-shot CLI child is still a child: guarding it keeps every reap in
    // the type, which is what lets the free `wait_bounded` be private.
    let mut child = ChildGuard::single_process(child);
    let status = child
        .wait_bounded(Duration::from_secs(30))
        .expect("cerulion topic list did not exit");
    assert!(
        status.success(),
        "cerulion topic list must exit 0: {status:?}"
    );
    read_file(&stdout_path)
}

/// Spawn `cerulion topic echo <topic>` bounded by wall-clock `run_for` (the
/// verb has no `--count`/`--duration` flag), then SIGINT it and return its
/// captured stdout (crib of
/// `topic_introspect_cli_e2e_test.rs::run_topic_verb_bounded`).
///
/// LOCAL-ONLY (`CERULION_NETWORK=off`, matching the supervisor child):
/// `topic echo`'s `ensure_topic_available` routes a NOT-yet-locally-listed topic to
/// the netd demand rung on the automagic scouting-ON path, which
/// `connect_or_spawn()`s the WELL-KNOWN `cerulion-netd` socket — so in the race
/// window before the worker's publisher lands in the local registry this test could
/// SPAWN a machine-wide network daemon on the developer's desk. The kill switch is
/// consulted FIRST there, and this test only echoes a LOCAL topic, so the
/// assertions are unchanged.
fn run_topic_echo_bounded(root: &Path, topic: &str, run_for: Duration) -> String {
    let stdout_path = root.join("echo.stdout");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", "echo", topic])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn cerulion topic echo");
    std::thread::sleep(run_for);
    send_signal(child.id(), libc::SIGINT);
    // A one-shot CLI child is still a child: guarding it keeps every reap in
    // the type, which is what lets the free `wait_bounded` be private.
    let mut child = ChildGuard::single_process(child);
    let status = child
        .wait_bounded(Duration::from_secs(10))
        .expect("cerulion topic echo did not exit after SIGINT");
    assert!(
        status.success(),
        "cerulion topic echo must exit 0 on SIGINT, got {status:?}"
    );
    read_file(&stdout_path)
}

/// TEST A — the headline: a DEFAULT multi-process run's topics are
/// visible to (and deliver frames to) default-namespace introspection spawned
/// as separate processes.
///
/// Hand oracle: earlier the data plane was on a run-scoped `cer_d_{hex}`
/// namespace, so this EXACT shape (real mp run, no `--single-process`,
/// external `topic list`/`topic echo`) listed nothing and echoed ZERO frames.
/// Every assert below fails on that old behavior.
#[test]
#[serial]
fn mp_default_run_topics_visible_to_default_ns_introspection() {
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "mpdnsa";
    build_mp_workspace(tmp.path(), prefix);
    let (mut guard, stdout_path, stderr_path) = spawn_graph_run_mp(tmp.path(), "run_a");
    let sup_pid = guard.id();

    // The deployment goes LIVE (planning build + 2 READY-gated worker spawns
    // precede GO — generous bound for slow CI VMs). The GO breadcrumb only
    // prints on the mp supervisor path, so it doubles as the "really
    // multi-process" pin.
    // The GO breadcrumb is a `tracing::info!`, which now goes to STDERR
    // (init_logging writes there so a command's stdout stays clean data) —
    // poll stderr as the primary, keeping a short stdout fallback for
    // writer-choice tolerance.
    assert!(
        wait_for_log(&stderr_path, GO_MARKER, Duration::from_secs(90))
            || wait_for_log(&stdout_path, GO_MARKER, Duration::from_secs(1)),
        "the GO breadcrumb proves the supervisor deployment went live; log:\n{}",
        merged_log(&stdout_path, &stderr_path)
    );
    // Belt-and-suspenders "really mp": the supervisor has 2 live `run-worker`
    // children (p0 + p1).
    let pids = wait_for_workers(sup_pid, 2, Duration::from_secs(30));
    assert!(
        pids.len() >= 2,
        "a 2-group deployment must have >= 2 run-worker children, saw {pids:?}"
    );

    // (1) `topic list` from a SEPARATE process (default-namespace attach) sees
    // every graph-owned topic. Before the move: none of these appeared.
    let list_out = run_topic_list(tmp.path());
    for topic in [
        format!("/{prefix}/ticker/cmd"),
        format!("/{prefix}/relay/cmd"),
        format!("/{prefix}/sink/cmd"),
    ] {
        assert!(
            list_out.contains(&topic),
            "`cerulion topic list` must show the mp run's topic {topic}; stdout was:\n{list_out}"
        );
    }

    // (2) `topic echo` on the CROSS-GROUP edge (`relay/cmd` — published by
    // p0's relay, consumed by p1's sink) receives >= 1 frame: the external
    // subscriber attaches on the default namespace and real mp traffic
    // reaches it. ~30 ticks (50ms period) of margin above the 1-frame bar.
    let topic = format!("/{prefix}/relay/cmd");
    let echo_log = run_topic_echo_bounded(tmp.path(), &topic, Duration::from_millis(1500));
    assert!(
        echo_log.contains("seq=") && echo_log.contains("schema=0x"),
        "`cerulion topic echo {topic}` must display at least one delivered frame \
         (the `seq=.. ts=..ns schema=0x.. size=..` line) — receiving a frame IS \
         the proof the mp data plane is on the default namespace; stdout was:\n{echo_log}"
    );

    // (3) Clean shutdown: directed SIGINT to the supervisor (the production
    // Ctrl-C; the supervisor fans it out to the workers) -> exit 0.
    send_signal(sup_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "mp run must exit 0 on Ctrl-C, got {status:?}\nlog:\n{}",
        merged_log(&stdout_path, &stderr_path)
    );
}

/// TEST B — the decision's accepted consequence: concurrent runs of the same
/// graph share the default namespace and the second collides LOUDLY on the
/// single-writer publisher checks, while the incumbent survives.
///
/// Mechanism (verified in source, see the module doc): B's first worker dies
/// at `ticker/cmd` publisher creation on the single-writer active-publisher
/// pre-check (`transport/mod.rs` ~:1640) BEFORE writing READY; the
/// supervisor's `wait_for_ready_or_exit` (`graph_cmd.rs` ~:3186) turns the
/// pre-READY death into a loud spawn-phase abort UNCONDITIONALLY — no
/// `--peer-loss` involvement (that policy is live-JOIN-loop only), so B needs
/// no extra flag to exit nonzero.
///
/// Hand oracles: the collision substrings are the independently-authored
/// transport error contract ("single-writer" + "another graph (or another
/// instance of this one)"); earlier B would have run SILENTLY on its own
/// run-scoped namespace (no collision, two live duplicates) — nonzero-B +
/// surviving-A is the designed default-namespace behavior.
#[test]
#[serial]
fn concurrent_same_graph_second_run_refused_loudly_incumbent_survives() {
    let tmp = tempfile::tempdir().unwrap();
    let prefix = "mpdnsb";
    build_mp_workspace(tmp.path(), prefix);

    // Run A: up + live.
    let (mut guard_a, stdout_a, stderr_a) = spawn_graph_run_mp(tmp.path(), "run_a");
    // GO is a `tracing::info!` → STDERR (see TEST A) — poll stderr primary.
    assert!(
        wait_for_log(&stderr_a, GO_MARKER, Duration::from_secs(90))
            || wait_for_log(&stdout_a, GO_MARKER, Duration::from_secs(1)),
        "run A must go live before B is spawned; log:\n{}",
        merged_log(&stdout_a, &stderr_a)
    );

    // Run B: SAME graph, SAME workspace, SAME prefix -> same topic names on
    // the shared default namespace. Bounded window covers B's planning build
    // + first worker spawn + the 20ms-poll death detection.
    let (mut guard_b, stdout_b, stderr_b) = spawn_graph_run_mp(tmp.path(), "run_b");
    let status_b = guard_b.wait_bounded(Duration::from_secs(120)).expect(
        "run B must exit (refused by the single-writer collision) within the bounded window",
    );
    assert!(
        !status_b.success(),
        "a concurrent second run of the same graph must exit NONZERO, got {status_b:?}\nlog:\n{}",
        merged_log(&stdout_b, &stderr_b)
    );
    let log_b = merged_log(&stdout_b, &stderr_b);
    // The single-writer collision signature (stable core phrases from
    // `transport/mod.rs` ~:1643, surfaced on B's stderr by the worker process
    // that inherits it).
    assert!(
        log_b.contains("single-writer"),
        "B's output must carry the single-writer collision contract; log:\n{log_b}"
    );
    assert!(
        log_b.contains("another graph (or another instance of this one)"),
        "B's output must name the likely cause (another instance); log:\n{log_b}"
    );
    // The propagation-path pin: the supervisor abort is the pre-READY
    // spawn-phase arm, not a live-phase peer-loss verdict.
    assert!(
        log_b.contains("before signaling READY"),
        "B's supervisor must abort via the pre-READY worker-death arm; log:\n{log_b}"
    );

    // Incumbent survival (the `cross_graph_collision_iox2_test.rs` shape):
    // A is still alive after B's refusal...
    assert!(
        guard_a
            .try_wait_noting()
            .expect("try_wait on run A")
            .is_none(),
        "run A must SURVIVE run B's refusal (incumbent survival)"
    );
    // ...and still DELIVERING (a bounded echo on A's ticker output still
    // yields frames — the refusal cost A nothing).
    let topic = format!("/{prefix}/ticker/cmd");
    let echo_log = run_topic_echo_bounded(tmp.path(), &topic, Duration::from_millis(1500));
    assert!(
        echo_log.contains("seq="),
        "run A must still deliver frames after B's refusal; echo stdout was:\n{echo_log}"
    );

    // Clean shutdown of the incumbent.
    send_signal(guard_a.id(), libc::SIGINT);
    let status_a = guard_a
        .wait_bounded(Duration::from_secs(90))
        .expect("run A did not exit after SIGINT");
    assert!(
        status_a.success(),
        "run A must exit 0 on Ctrl-C after surviving B, got {status_a:?}\nlog:\n{}",
        merged_log(&stdout_a, &stderr_a)
    );
}
