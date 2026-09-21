// SPDX-License-Identifier: AGPL-3.0-only
//! The wedge alarm over the REAL binary, both directions.
//!
//! The supervisor half is unreachable from `cerulion_cli_engine`'s own arms:
//! they drive `WedgeObserver` in-process against a fake worker, so nothing there
//! can see whether a REAL `graph run-worker` opens the page its plan names, binds
//! its nodes to the slots the SUPERVISOR chose, and marks them on the fire path.
//! That meeting — supervisor and worker agreeing on `(tag, rank, slot)` across a
//! process boundary, under a plan file neither hand-wrote — is what these two
//! arms pin.
//!
//! 1. **[`a_healthy_multi_process_run_raises_no_wedge_alarm_and_leaves_no_pages`]**
//!    — the ANTI-TAUTOLOGY control for the whole feature AND the drain-gate pin.
//!    A healthy 2-node chain runs, is SIGINT'd, and exits 0 with not one alarm
//!    line — *during the run and through the drain*. The drain half is the one
//!    that bites: on Ctrl-C every worker leaves `run_live` BY DESIGN, so its step
//!    word freezes for the whole drain window against a 5 s threshold, and an
//!    ungated observation reports a wedge on every clean shutdown. The arm-time
//!    `info!` is asserted TOO, because without it "zero WARN lines" is equally
//!    satisfied by an alarm that was never armed.
//!
//! 2. **[`a_worker_whose_tick_never_returns_is_reported_per_node`]** — the
//!    POSITIVE arm, and the only place the cross-process meeting is proven. A
//!    node parks forever inside its first tick; the supervisor must name it.
//!
//! The hung tick is a `Condvar` park, never a `sleep`: a sleeping tick would make
//! the arm a race against a wall (the timer-coalescing class: a nominal 150 ms charged as
//! 1100–1696 ms under macOS background QoS) and would eventually RETURN, clearing
//! the very regime under test. A parked one is still parked however slow the machine
//! is, so every wait here is a generous liveness ceiling in seconds that load can
//! delay but never invert.
//!
//! Prerequisites (the repo's fixture pattern — PANICS with the instruction if
//! missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib -p test_node_hang_cdylib`
//!
//! ```bash
//! cargo test -p cerulion_cli --test wedge_alarm_e2e_test -- --test-threads=1
//! ```
//!
//! `#[cfg(unix)]` + `#[serial]`: the multi-process supervisor is Unix-wide
//! and these runs share the default iceoryx2 namespace.

#![cfg(unix)]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// A generous liveness ceiling for the POSITIVE arm's alarm. The threshold it is
/// waiting on is the 5 s floor; nothing is timed against this number, it exists
/// so a broken harness FAILS instead of hanging CI to its job timeout.
const ALARM_CEILING: Duration = Duration::from_secs(40);

/// How long the healthy run is left alive before the SIGINT. Comfortably past
/// the 5 s floor, so an ungated observer really would have accumulated a wedge's
/// worth of dwell had anything been frozen.
const HEALTHY_RUN: Duration = Duration::from_secs(9);

/// Every alarm line the supervisor can emit. The negative arm forbids ALL of
/// them: naming only the per-node head would leave the rank arms free to fire on
/// every clean shutdown.
const ALARM_MARKERS: &[&str] = &[
    "ENTERED A TICK AND HAS NOT RETURNED",
    "STILL inside the same tick",
    "STEP LOOP has not advanced",
    "STILL not advancing",
    "NEVER WROTE ITS WEDGE PAGE",
];

/// SIGKILL + reap on drop so a panicking test never leaks the supervisor.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// The per-deployment plan directory's name prefix — `cerulion_mp_<graph>_`
/// under the system temp dir, the supervisor's own construction. Every worker
/// carries it in its argv (`graph run-worker --plan <dir>/plan_<group>.json`),
/// so it is BOTH this file's pgrep needle and its plan-scan key. `<graph>` is
/// made unique per TEST PROCESS by [`graph_name`], which is what scopes both.
fn plan_dir_prefix(graph: &str) -> String {
    format!("cerulion_mp_{graph}_")
}

/// This file's graph names carry the test process's pid, so every name derived
/// from them — the plan directory, the pgrep needle, the SHM objects — is scoped
/// to THIS run and cannot reach a concurrent session's.
fn graph_name(stem: &str) -> String {
    format!("{stem}{}", std::process::id())
}

/// Remove any plan directory left behind by an earlier run of THIS test process
/// (a pid reuse after a SIGKILLed run), so [`wedge_tag_from_plan`] can require
/// exactly one and a stale tag can never stand in for a missing binding.
fn sweep_stale_plan_dirs(graph: &str) {
    let prefix = plan_dir_prefix(graph);
    let Ok(dirs) = std::fs::read_dir(std::env::temp_dir()) else {
        return;
    };
    for e in dirs.flatten() {
        if e.file_name().to_string_lossy().starts_with(&prefix) {
            let _ = std::fs::remove_dir_all(e.path());
        }
    }
}

/// A worker parked inside a hung tick cannot be SIGKILLed by the supervisor's
/// own `Drop` if the supervisor itself was killed, so the positive arm reaps by
/// NAME — and the name must be THIS DEPLOYMENT's, never a bare `run-worker`.
///
/// A bare `pgrep -f run-worker` + SIGKILL would sweep every
/// multi-process Cerulion run ON THE MACHINE: another session's live robot
/// graph, a sibling test binary's supervisor, another user's workers. The
/// class is real; the siblings in this
/// directory (`mp_record_e2e`, `mp_auto_partition_e2e`, `mp_supervisor_box`)
/// scope their worker pgreps to `-P <supervisor>`. That form is
/// unavailable HERE for the reason the sweeper exists at all: the supervisor may
/// already be dead, and an orphaned worker reparents to init, so ancestry is
/// gone by the time this runs.
///
/// The needle is therefore the deployment PLAN DIRECTORY, which every worker
/// carries in its argv and whose `<graph>` half is unique to this test process.
/// Each matched pid is re-read through `ps` and killed only if its command line
/// really carries both that needle and `run-worker` — pgrep matches a PATTERN,
/// and a SIGKILL wants the answer confirmed against the process itself.
struct WorkerSweeper {
    needle: String,
}
impl WorkerSweeper {
    fn for_graph(graph: &str) -> Self {
        Self {
            needle: plan_dir_prefix(graph),
        }
    }
}
impl Drop for WorkerSweeper {
    fn drop(&mut self) {
        let Ok(out) = Command::new("pgrep").args(["-f", &self.needle]).output() else {
            return;
        };
        for pid in String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<i32>().ok())
        {
            let Ok(ps) = Command::new("ps")
                .args(["-p", &pid.to_string(), "-o", "command="])
                .output()
            else {
                continue;
            };
            let cmdline = String::from_utf8_lossy(&ps.stdout);
            if !cmdline.contains(&self.needle) || !cmdline.contains("run-worker") {
                continue;
            }
            // SAFETY: kill(2) on a pid this sweep just read AND confirmed is one
            // of THIS deployment's workers; no memory is touched. ESRCH after a
            // clean exit is the expected no-op.
            unsafe {
                libc::kill(pid, libc::SIGKILL);
            }
        }
    }
}

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

fn send_sigint(pid: u32) {
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, libc::SIGINT);
    }
}

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// Scaffold a workspace whose graph is `graph` and whose nodes are
/// `(node_type, fixture_crate)` pairs, wired as a chain: node `i+1` consumes
/// node `i`'s `cmd`. UNPARTITIONED, so the multi-process default derives
/// process-per-node and the run is genuinely multi-process.
fn build_workspace(root: &Path, graph: &str, prefix: &str, chain: &[(&str, &str)]) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    for (node_type, fixture) in chain {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            cerulion_core::testing::find_fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    let mut yaml = format!("name: {graph}\nprefix: {prefix}\nnodes:\n");
    for (i, (node_type, _)) in chain.iter().enumerate() {
        yaml.push_str(&format!("- id: {node_type}\n  type: {node_type}\n"));
        if i == 0 {
            yaml.push_str("  inputs: []\n");
        } else {
            let upstream = chain[i - 1].0;
            yaml.push_str(&format!(
                "  inputs:\n  - name: trigger_in\n    source: {upstream}/cmd\n"
            ));
        }
        yaml.push_str("  outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n");
    }
    std::fs::write(root.join(format!("graphs/{graph}.yaml")), yaml).unwrap();
}

/// Spawn `cerulion graph run <graph>` with stdin `/dev/null` (the no-TTY consent
/// floor — an unpartitioned graph derives its partition IN MEMORY and runs
/// multi-process). Child stdout + stderr go to files readable while it runs.
fn spawn_graph_run(
    root: &Path,
    graph: &str,
    env: &[(&str, &str)],
) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    for (k, v) in env {
        cmd.env(k, v);
    }
    let child = cmd
        .args(["graph", "run", graph, "--no-validate"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic: no scouting session, no gateway (the permissive network
        // default would otherwise open the LAN in CI).
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    (ChildGuard(child), stdout_path, stderr_path)
}

/// Strip ANSI SGR sequences.
///
/// LOAD-BEARING, not hygiene (a lesson learned before, and re-learned here by watching
/// this arm fail against a log whose own text showed the right value): `tracing`
/// colours a field's `=` as well as its key, so the raw bytes of a rendered field
/// are `<esc>[3mnode_id<esc>[0m<esc>[2m=<esc>[0mhang` — `node_id=hang` is NOT a
/// substring of it, and every `key=value` assertion would be silently
/// unsatisfiable.
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\u{1b}' {
            out.push(c);
            continue;
        }
        // `ESC [ … <final>` — consume through the first byte in `@`..=`~`.
        if chars.next() == Some('[') {
            for f in chars.by_ref() {
                if ('@'..='~').contains(&f) {
                    break;
                }
            }
        }
    }
    out
}

fn merged_log(stdout_path: &Path, stderr_path: &Path) -> String {
    strip_ansi(&format!(
        "{}\n{}",
        read_file(stdout_path),
        read_file(stderr_path)
    ))
}

/// The wedge-page TAG this run stamped into its worker plans, read off a plan
/// file while the run is live.
///
/// The tag is a per-deployment nonce, so it is the only way a test can name the
/// SHM objects afterwards — and reading it out of the plan is itself evidence
/// the supervisor stamped a binding rather than merely creating pages.
/// Read it only ONCE THE RUN IS LIVE, never from spawn.
///
/// The plan directory is created early but the plan FILES are written after the
/// auto-partition pre-flight, the node loads and the alarm arming — and how long that
/// takes is a property of the machine, not of this feature (MEASURED: it raced a
/// 30 s poll started at spawn, on a desk carrying another run's stale SHM). The
/// files then live for the whole run, so a caller that has already proved the
/// supervisor reached GO needs no poll at all; the short one here only absorbs
/// filesystem latency.
///
/// Returns the SCAN REPORT on failure rather than a bare `None`: a silent
/// absence here is indistinguishable from a supervisor that stamped no binding,
/// which is exactly the regression the SHM-hygiene assertion exists to catch.
///
/// SCOPED TO THE CURRENT DEPLOYMENT, two ways: the
/// prefix carries this test process's pid via [`graph_name`], and the caller
/// sweeps stale directories before spawning, so exactly ONE plan dir may match —
/// asserted rather than assumed. Taking the FIRST tag any matching directory
/// offered would let a plan left by a killed prior run supply a tag for a
/// supervisor that stamped none, and the SHM-hygiene assertion would then pass
/// against a name this run never created.
fn wedge_tag_from_plan(graph: &str, deadline: Duration) -> Result<String, String> {
    let prefix = plan_dir_prefix(graph);
    let start = Instant::now();
    let mut seen: Vec<String> = Vec::new();
    loop {
        seen.clear();
        let mut dir_paths: Vec<std::path::PathBuf> = Vec::new();
        match std::fs::read_dir(std::env::temp_dir()) {
            Ok(dirs) => {
                for e in dirs.flatten() {
                    let name = e.file_name().to_string_lossy().into_owned();
                    if !name.starts_with(&prefix) {
                        continue;
                    }
                    seen.push(name);
                    dir_paths.push(e.path());
                }
            }
            Err(e) => return Err(format!("cannot read {:?}: {e}", std::env::temp_dir())),
        }
        // A SECOND directory under this run's own prefix is a harness fault, not
        // a slow supervisor: the prefix is pid-scoped and stale ones were swept
        // before the spawn. Reading either one would be a guess.
        if dir_paths.len() > 1 {
            return Err(format!(
                "{} plan directories match `{prefix}` ({seen:?}) — this run creates \
                 exactly one, so a tag read from any of them would not be provably \
                 this deployment's",
                dir_paths.len()
            ));
        }
        for dir in &dir_paths {
            let Ok(plans) = std::fs::read_dir(dir) else {
                continue;
            };
            for p in plans.flatten() {
                if !p.file_name().to_string_lossy().starts_with("plan_") {
                    continue;
                }
                let Ok(text) = std::fs::read_to_string(p.path()) else {
                    continue;
                };
                let Ok(v) = serde_json::from_str::<serde_json::Value>(&text) else {
                    continue;
                };
                if let Some(tag) = v
                    .get("wedge_page")
                    .and_then(|w| w.get("tag"))
                    .and_then(|t| t.as_str())
                {
                    return Ok(tag.to_string());
                }
            }
        }
        if start.elapsed() >= deadline {
            return Err(format!(
                "no plan under {:?} matching `{prefix}` carried a `wedge_page` binding \
                 within {deadline:?} (plan dirs seen: {seen:?})",
                std::env::temp_dir()
            ));
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// **THE ANTI-TAUTOLOGY CONTROL for the whole feature, and the drain-gate pin.**
///
/// Every other assertion about the alarm says "it fires when X". This one says
/// it fires ONLY then — which is the property an always-on diagnostic lives or
/// dies by, and the one a clean shutdown breaks by default: the drain window is
/// 20 s against a 5 s threshold, and during it every worker has left `run_live`
/// on purpose, so its step word is frozen and its nodes may be mid-tick.
#[test]
#[serial]
fn a_healthy_multi_process_run_raises_no_wedge_alarm_and_leaves_no_pages() {
    let graph = graph_name("wedgeok");
    sweep_stale_plan_dirs(&graph);
    let _sweeper = WorkerSweeper::for_graph(&graph);
    let dir = tempfile::tempdir().expect("tempdir");
    let root = dir.path();
    let prefix = format!("wdgok{}", std::process::id());
    build_workspace(
        root,
        &graph,
        &prefix,
        &[
            ("ticker", "test_node_macro_period_cdylib"),
            ("sink", "test_node_macro_data_trigger_cdylib"),
        ],
    );

    let home = root.join("home");
    std::fs::create_dir_all(&home).unwrap();
    let (mut guard, out, err) =
        spawn_graph_run(root, &graph, &[("CERULION_HOME", &home.to_string_lossy())]);
    let pid = guard.0.id();

    std::thread::sleep(HEALTHY_RUN);
    // The run must still be alive: a supervisor that died early would make every
    // "no alarm" assertion below vacuous.
    assert!(
        guard.0.try_wait().expect("try_wait").is_none(),
        "the healthy run exited before it was signalled — log:\n{}",
        merged_log(&out, &err)
    );

    // Read the tag now, NOT at spawn: the plan files appear only once the
    // supervisor has armed the alarm, and the run above is already proven past
    // that point (it is live and its GO breadcrumb is asserted below). The plan
    // directory is removed at exit, so this is the last chance to name the SHM
    // objects the hygiene check needs.
    //
    // The deadline is a GENEROUS LIVENESS CEILING, not a wall anything is timed
    // against: the run is alive throughout (asserted above) and stays alive until
    // the SIGINT below, so a longer poll is a LONGER healthy window — strictly
    // more of what this arm asserts, never less, and on a healthy desk it returns
    // in milliseconds and costs nothing.
    //
    // Raised from 10 s as SLACK for the ambient class this helper's own docs
    // already name — and stated at its measured strength rather than as a fix: on
    // a LOADED desk (an orphaned recorder at 25 % of a core, 154 stale iceoryx2
    // service files) no plan directory appeared within 45 s either, and the
    // PRE-CHANGE arm was RUN on that same desk and failed identically
    // (`plan dirs seen: []`). So the flake is ambient and pre-existing, not this
    // ceiling's; both arms pass on an idle desk with the plan directory observed
    // appearing ~2 s into the run.
    let tag = wedge_tag_from_plan(&graph, Duration::from_secs(45));

    send_sigint(pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(60))
        .unwrap_or_else(|| panic!("no clean exit — log:\n{}", merged_log(&out, &err)));
    let log = merged_log(&out, &err);
    assert_eq!(
        status.code(),
        Some(0),
        "a SIGINT'd healthy run must exit 0 — log:\n{log}"
    );

    // THE POSITIVE CONTROL: without it, "zero alarm lines" is satisfied by a
    // build in which the alarm was never armed at all.
    assert!(
        log.contains("wedge alarm armed"),
        "the supervisor must ARM the alarm, or the absence assertions below \
         prove nothing — log:\n{log}"
    );
    assert!(
        log.contains("GO"),
        "the run must really have reached the multi-process GO — log:\n{log}"
    );

    for marker in ALARM_MARKERS {
        assert!(
            !log.contains(marker),
            "a HEALTHY run raised `{marker}` — the alarm must be silent during the \
             run AND through the drain, where every worker has left `run_live` by \
             design and its step word is frozen for the whole window. Log:\n{log}"
        );
    }

    // SHM HYGIENE: the pages are owned by the supervisor, so a clean exit must
    // unlink every name. `open_unowned` is STRICT open-existing, so a surviving
    // name is exactly what this catches.
    let tag = tag.unwrap_or_else(|why| {
        panic!(
            "no worker plan carried a `wedge_page` binding — the supervisor must \
             stamp one per rank, and without the tag this arm cannot check SHM \
             hygiene. {why}"
        )
    });
    for rank in 0..2u32 {
        let err = cerulion_core::wedge_page::MappedWedgePage::open_unowned(&tag, rank)
            .expect_err("a wedge page name must not survive the run that created it");
        assert_eq!(
            err.kind(),
            std::io::ErrorKind::NotFound,
            "rank {rank}'s page name is still resolvable after the run: {err}"
        );
    }
}

/// **THE POSITIVE ARM** — a tick that never returns is reported, per node, by
/// name.
///
/// This is the only test in the repo where the supervisor and a REAL worker meet
/// on `(tag, rank, slot)` across a process boundary: the worker opens the page
/// its PLAN FILE names, binds its nodes to the slots the supervisor chose, marks
/// them from inside `evaluate_node`, and the supervisor reads the frozen pair
/// back out and names the node. A slot-order disagreement between the two sides
/// does not crash — it names the WRONG node — so a single-node graph is
/// deliberately paired with an assertion on the node's NAME.
///
/// ONE node on purpose: a hung worker stops arriving at the level barrier, which
/// poisons a sibling within `BARRIER_BOUNDARY_TIMEOUT` and turns the arm into a
/// test of the peer-loss policy instead. With a single rank the barrier is
/// vacuous and the only thing that can stop the run is the wedge itself.
#[test]
#[serial]
fn a_worker_whose_tick_never_returns_is_reported_per_node() {
    let graph = graph_name("wedgehang");
    sweep_stale_plan_dirs(&graph);
    let _sweeper = WorkerSweeper::for_graph(&graph);
    let dir = tempfile::tempdir().expect("tempdir");
    let root = dir.path();
    let prefix = format!("wdghang{}", std::process::id());
    build_workspace(root, &graph, &prefix, &[("hang", "test_node_hang_cdylib")]);

    let home = root.join("home");
    std::fs::create_dir_all(&home).unwrap();
    let (mut guard, out, err) = spawn_graph_run(
        root,
        &graph,
        &[
            ("CERULION_HOME", &home.to_string_lossy()),
            // The fixture's env switch: its FIRST tick parks forever.
            ("CER_HANG_MODE", "1"),
            // Shrink the drain window so the teardown below is quick — a parked
            // worker cannot answer SIGINT, so it is always the drain deadline's
            // SIGKILL that reaps it.
            ("CERULION_MP_DRAIN_MS", "2000"),
        ],
    );
    let pid = guard.0.id();

    let start = Instant::now();
    let log = loop {
        let log = merged_log(&out, &err);
        if log.contains("ENTERED A TICK AND HAS NOT RETURNED") {
            break log;
        }
        assert!(
            guard.0.try_wait().expect("try_wait").is_none(),
            "the run exited before the alarm fired — log:\n{log}"
        );
        assert!(
            start.elapsed() < ALARM_CEILING,
            "no wedge alarm within {ALARM_CEILING:?} — the supervisor and the worker \
             must meet on the page the plan names. Log:\n{log}"
        );
        std::thread::sleep(Duration::from_millis(100));
    };

    // It NAMES the node, at WARN, with the provenance an operator acts on.
    //
    // A slot-order disagreement between supervisor and worker does not crash —
    // it reports the WRONG id — so the id is the assertion, not the fact that
    // some line appeared. Nothing declared a `tick_within_ms`, so the rung is the
    // node's own `period_ms = 50`; 50 x 4 = 200 ms sits under the 5 s floor, and
    // `threshold_floored=true` beside `threshold_source="declared period_ms"` is
    // what tells the operator their declaration was not IGNORED, merely too tight
    // to alarm on. Whole-token `key=value` matching is the rule: a
    // substring would let `threshold_ms=5000` match `threshold_ms=50000`.
    let head = log
        .lines()
        .find(|l| l.contains("ENTERED A TICK AND HAS NOT RETURNED"))
        .expect("the head line");
    let field = |k: &str, v: &str| {
        let want = format!("{k}={v}");
        head.split_whitespace().any(|t| t == want)
    };
    assert!(
        head.split_whitespace().any(|t| t == "WARN"),
        "the head must be LOUD — line was:\n{head}"
    );
    assert!(
        field("node_id", "hang"),
        "the alarm must NAME the wedged node — line was:\n{head}"
    );
    assert!(
        head.contains("threshold_source=\"declared period_ms\""),
        "the head must name the rung it derived from — line was:\n{head}"
    );
    assert!(
        field("threshold_floored", "true"),
        "50 ms x4 = 200 ms is below the 5 s floor, so the floor raised it and the \
         operator must be told — line was:\n{head}"
    );
    assert!(
        field("threshold_ms", "5000"),
        "…and the number in force is the floor's — line was:\n{head}"
    );
    // The RANK arm must stay silent: a hung tick freezes the step thread too, so
    // both halves reach their thresholds together, and the rank line's own text
    // claims no node is inside a tick.
    assert!(
        !log.contains("STEP LOOP has not advanced"),
        "the rank arm must be SUPPRESSED while a node of it is wedged — its own \
         text would be false and would point at the wrong fault. Log:\n{log}"
    );

    // Teardown: the parked worker cannot answer a SIGINT, so the supervisor's
    // drain deadline SIGKILLs it. Bounded either way; the sweeper is the backstop.
    send_sigint(pid);
    let _ = wait_bounded(&mut guard.0, Duration::from_secs(30));
}
