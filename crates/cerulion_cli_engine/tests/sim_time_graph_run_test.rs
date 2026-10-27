// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run` on sim time, at the workspace seam: the real
//! [`graph_run`] over a hand-built tempdir workspace whose node libraries are
//! copies of the prebuilt macro fixtures, driven by a `/clock` publisher on
//! the run's own transport, with the ticker's published frames as the oracle.
//!
//! Every expected value is hand-written: the refusal texts a user reads, and
//! the sim stamps a 50 ms period node must carry when `/clock` steps
//! `1.000 s, 1.050 s, 1.100 s, 1.150 s`, then jumps to `1.400 s` across five
//! deadlines and steps on to `1.450 s`.
//!
//! Arms:
//! 1. `sim_time: true` + `--time-source virtual` is refused before any file
//!    is written: the graph file is byte-identical and no run directory exists.
//! 2. `sim_time: true` + `--record` is refused with the sim-time text.
//! 3. `sim_time: true` + `process_groups:` is refused unless
//!    `--single-process`, and the graph file is untouched.
//! 4. A `sim_time: true` graph whose own node publishes `/clock` is refused
//!    at load, by the run as by `graph validate`.
//! 5. A `sim_time: true` graph with an external-policy node is refused AT
//!    LAUNCH, before the run waits for a clock: no clock is ever published
//!    here, so a refusal placed after the wait would never return.
//! 6. The fire oracle: a period node fires once per forward `/clock` sample,
//!    each frame stamped with the sim time of its deadline, a sample that
//!    jumps across several deadlines delivers ONE frame, and the run ends
//!    cleanly when `running` clears.
//! 7. A graph whose node library is missing, run with `--no-validate` beside
//!    a `ros2:` entry, is refused by the library's name BEFORE the entry
//!    starts, on sim time and on the real clock alike: the entry's own
//!    preflight (its launch file is missing too) never gets to speak.
//!
//! Needs `cargo build -p test_node_macro_period_cdylib
//! -p test_node_macro_external_cdylib` first. `#[serial]`: arms 5 and 6 load a
//! node library, arm 6 owns the process-global transport singleton, and every
//! arm redirects `CERULION_HOME`.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cerulion_cli_engine::graph_cmd::{
    graph_run, CpuDmaLockMode, MonitorWaitMode, RecordCpu, RecordEnvMode, TimeSource,
};
use cerulion_cli_engine::sim_time_run::{
    SIM_TIME_EXTERNAL_NODES_REJECTION, TWO_CLOCK_SOURCES_REJECTION,
};
use cerulion_core::graph::sim_time::SIM_CLOCK_TOPIC;
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::TransportManager;
use native_ros2_messages::rosgraph_msgs::Clock;
use serial_test::serial;

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";
const EXTERNAL_FIXTURE: &str = "test_node_macro_external_cdylib";

/// The platform cdylib filename for a node type.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// A fixture's source file, copied into the workspace so the source-is-truth
/// metadata walker can parse the node's trigger policy.
fn fixture_src(fixture: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("repo root")
        .join("test_fixtures")
        .join(fixture)
        .join("src/lib.rs")
}

/// A pid + nanosecond unique graph prefix so topics on the shared global
/// iceoryx2 namespace can never collide across tests or processes.
fn unique_prefix(tag: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    format!("simrun{tag}{}n{}", std::process::id(), nanos)
}

/// One node of the workspace: its id, the fixture it is a copy of.
struct NodeSpec {
    id: &'static str,
    fixture: &'static str,
}

/// Whether a hand-built workspace carries its nodes' prebuilt libraries.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Libraries {
    /// The fixture cdylibs are copied under `target/debug`: the run can load.
    Prebuilt,
    /// Sources only: the run's library resolution fails for every node.
    Missing,
}

/// Hand-build a workspace in `root`: a `[workspace]` Cargo.toml, the node
/// sources and prebuilt libraries under the node ids, and `graphs/demo.yaml`
/// from `graph_yaml`.
fn build_workspace(root: &Path, nodes: &[NodeSpec], graph_yaml: &str) {
    build_workspace_with(root, nodes, graph_yaml, Libraries::Prebuilt);
}

/// [`build_workspace`], with or without the prebuilt libraries.
fn build_workspace_with(root: &Path, nodes: &[NodeSpec], graph_yaml: &str, libraries: Libraries) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
    for node in nodes {
        std::fs::create_dir_all(root.join(format!("nodes/{}/src", node.id))).unwrap();
        std::fs::copy(
            fixture_src(node.fixture),
            root.join(format!("nodes/{}/src/lib.rs", node.id)),
        )
        .expect("copy fixture src");
        if libraries == Libraries::Prebuilt {
            std::fs::copy(
                cerulion_core::testing::find_fixture_cdylib(node.fixture),
                root.join("target/debug").join(dylib_file(node.id)),
            )
            .expect("copy fixture cdylib");
        }
    }
}

/// The one-ticker sim-time graph, with `extra` appended at the top level.
fn ticker_graph(prefix: &str, extra: &str) -> String {
    format!(
        "name: demo\nprefix: {prefix}\nsim_time: true\nnodes:\n\
         - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n{extra}"
    )
}

const TICKER: NodeSpec = NodeSpec {
    id: "ticker",
    fixture: PERIOD_FIXTURE,
};

/// RAII: set an env var for the test body and restore the prior value on drop.
struct EnvGuard {
    key: &'static str,
    prior: Option<std::ffi::OsString>,
}

impl EnvGuard {
    fn set(key: &'static str, value: &str) -> Self {
        let prior = std::env::var_os(key);
        std::env::set_var(key, value);
        Self { key, prior }
    }

    fn cleared(key: &'static str) -> Self {
        let prior = std::env::var_os(key);
        std::env::remove_var(key);
        Self { key, prior }
    }
}

impl Drop for EnvGuard {
    fn drop(&mut self) {
        match self.prior.take() {
            Some(v) => std::env::set_var(self.key, v),
            None => std::env::remove_var(self.key),
        }
    }
}

/// The process env every arm runs under: a private `CERULION_HOME` (so the run
/// directory lands in the tempdir) and no `CARGO_TARGET_DIR` (so the
/// workspace's own `target/debug` lookup wins).
fn arm_env(home: &Path) -> Vec<EnvGuard> {
    vec![
        EnvGuard::set("CERULION_HOME", home.to_str().expect("utf-8 tempdir")),
        EnvGuard::cleared("CARGO_TARGET_DIR"),
    ]
}

/// The run every arm makes, differing only in what the arm is about.
struct RunArgs<'a> {
    root: &'a Path,
    running: Arc<AtomicBool>,
    time_source: TimeSource,
    single_process: bool,
    record: Option<String>,
    /// `--no-validate`: skip the validation report (arm 7 needs the run to
    /// get past the report's own library check).
    skip_validate: bool,
}

fn run(args: RunArgs<'_>) -> Result<(), String> {
    graph_run(
        args.root,
        &args.root.join("graphs"),
        "demo",
        args.running,
        args.time_source,
        CpuDmaLockMode::Auto,
        MonitorWaitMode::Auto,
        args.skip_validate,
        false,
        None,
        args.single_process,
        true,
        1_000,
        args.record,
        RecordEnvMode::Allowlist,
        RecordCpu::Auto,
        None,
        false,
        &[],
    )
    .map_err(|e| e.to_string())
}

/// The run directories `CERULION_HOME` holds: none when `runs/` does not
/// exist. Any other read error fails the test, so a refusal arm can never
/// pass because it could not look.
fn run_dirs(home: &Path) -> Vec<PathBuf> {
    let runs = home.join("runs");
    match std::fs::read_dir(&runs) {
        Ok(entries) => entries.map(|e| e.expect("entry").path()).collect(),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
        Err(e) => panic!("cannot read {}: {e}", runs.display()),
    }
}

// ==========================================================================
// 1. Two clock sources: refused before any file is written.
// ==========================================================================

#[test]
#[serial]
fn sim_time_plus_the_virtual_flag_is_refused_before_any_file_is_written() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let yaml = ticker_graph(&unique_prefix("virt"), "");
    build_workspace(&root, &[TICKER], &yaml);

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source: TimeSource::Virtual,
        single_process: true,
        record: None,
        skip_validate: false,
    })
    .expect_err("two clock sources are refused");
    assert!(
        err.contains(TWO_CLOCK_SOURCES_REJECTION),
        "the pinned two-sources text, prefixed by the graph: {err}"
    );
    assert!(err.contains("graph 'demo'"), "names the graph: {err}");
    assert_eq!(
        std::fs::read_to_string(root.join("graphs/demo.yaml")).expect("graph file"),
        yaml,
        "a refused run leaves the graph file byte-identical"
    );
    assert!(
        run_dirs(tmp.path()).is_empty(),
        "the refusal comes before the run directory is created"
    );
}

// ==========================================================================
// 2. `--record` under sim time: the sim-time rejection text.
// ==========================================================================

#[test]
#[serial]
fn sim_time_plus_record_is_refused_with_the_sim_time_text() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    build_workspace(&root, &[TICKER], &ticker_graph(&unique_prefix("rec"), ""));
    let bag_dir = tmp.path().join("bag");

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source: TimeSource::Real,
        single_process: true,
        record: Some(bag_dir.to_str().expect("utf-8").to_string()),
        skip_validate: false,
    })
    .expect_err("a sim-time run is not recordable");
    assert!(
        err.contains("--record") && err.contains("sim time"),
        "names the flag and the clock: {err}"
    );
    assert!(
        err.contains("`sim_time: true` in the graph YAML") && err.contains("sim_time: false"),
        "names the YAML switch that selected sim time and the fix: {err}"
    );
    assert!(
        !bag_dir.exists(),
        "no bag directory is created by a refused run"
    );
}

// ==========================================================================
// 3. `process_groups:` under sim time: refused unless `--single-process`.
// ==========================================================================

#[test]
#[serial]
fn sim_time_with_process_groups_is_refused_unless_single_process() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let yaml = ticker_graph(
        &unique_prefix("pg"),
        "process_groups:\n  only:\n    - ticker\n",
    );
    build_workspace(&root, &[TICKER], &yaml);

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source: TimeSource::Real,
        single_process: false,
        record: None,
        skip_validate: false,
    })
    .expect_err("one /clock stream cannot drive several processes");
    assert!(
        err.contains("process_groups") && err.contains("sim time"),
        "names the shape and the clock: {err}"
    );
    assert!(
        err.contains("--single-process") && err.contains("sim_time: false"),
        "offers both fixes: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(root.join("graphs/demo.yaml")).expect("graph file"),
        yaml,
        "a refused run leaves the graph file byte-identical"
    );
    assert!(
        run_dirs(tmp.path()).is_empty(),
        "the deployment is judged before the run directory is created"
    );
}

// ==========================================================================
// 4. A graph that publishes its own `/clock`: refused at load.
// ==========================================================================

#[test]
#[serial]
fn sim_time_refuses_a_graph_whose_own_node_publishes_the_clock() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    // The ticker's own output is pointed at `/clock` (the type is not checked
    // here: a run never gets as far as the schema report, the load rule
    // refuses first).
    let yaml = format!(
        "name: demo\nprefix: {}\nsim_time: true\nnodes:\n\
         - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: rosgraph_msgs/Clock\n    topic: {SIM_CLOCK_TOPIC}\n",
        unique_prefix("own")
    );
    build_workspace(&root, &[TICKER], &yaml);

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source: TimeSource::Real,
        single_process: true,
        record: None,
        skip_validate: false,
    })
    .expect_err("a graph cannot be its own clock source");
    assert!(
        err.contains("ticker/cmd") && err.contains("outside this graph"),
        "names the output and says where the clock has to come from: {err}"
    );
    assert!(
        run_dirs(tmp.path()).is_empty(),
        "the refusal comes before the run directory is created"
    );
}

/// The flag's spelling is held to the same load rules as the YAML switch: a
/// graph whose YAML says nothing about sim time, run with `--time-source
/// external`, is refused for a second clock source exactly as `sim_time: true`
/// would be, before any file is written.
#[test]
#[serial]
fn the_external_flag_is_held_to_the_one_clock_source_rule() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let yaml = format!(
        "name: demo\nprefix: {}\nnodes:\n\
         - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
         multi_publisher_topics:\n  - {SIM_CLOCK_TOPIC}\n",
        unique_prefix("flag")
    );
    build_workspace(&root, &[TICKER], &yaml);

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source: TimeSource::External,
        single_process: true,
        record: None,
        skip_validate: false,
    })
    .expect_err("two clock sources are refused whichever spelling turned sim time on");
    assert!(
        err.contains("one clock source") && err.contains("multi_publisher_topics"),
        "the same load refusal the YAML switch gets: {err}"
    );
    assert!(
        run_dirs(tmp.path()).is_empty(),
        "the refusal comes before the run directory is created"
    );
}

// ==========================================================================
// 5. An external-policy node: refused at launch, before the wait for a clock.
// ==========================================================================

#[test]
#[serial]
fn sim_time_with_an_external_node_is_refused_at_launch_without_waiting_for_a_clock() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let yaml = format!(
        "name: demo\nprefix: {}\nsim_time: true\nnodes:\n\
         - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
         - id: camera\n  type: camera\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n",
        unique_prefix("ext")
    );
    build_workspace(
        &root,
        &[
            TICKER,
            NodeSpec {
                id: "camera",
                fixture: EXTERNAL_FIXTURE,
            },
        ],
        &yaml,
    );

    // Nothing ever publishes `/clock` in this arm. A refusal placed after the
    // wait would never return, so the watchdog is what turns that into a
    // failure instead of a hang: it clears `running` well after any launch
    // refusal must have come back, and a run that merely ended on it returns
    // `Ok`, which the assertion below rejects.
    let running = Arc::new(AtomicBool::new(true));
    let watchdog_flag = Arc::clone(&running);
    let watchdog = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_secs(60));
        watchdog_flag.store(false, Ordering::Relaxed);
    });
    let started = Instant::now();
    let result = run(RunArgs {
        root: &root,
        running: Arc::clone(&running),
        time_source: TimeSource::Real,
        single_process: true,
        record: None,
        skip_validate: false,
    });
    assert!(
        running.load(Ordering::Relaxed),
        "the run returned on its own, before the watchdog cleared `running` ({:?})",
        started.elapsed()
    );
    let err = result.expect_err("an external-policy node cannot fire under sim time");
    assert!(
        err.contains("[camera]") && err.contains(SIM_TIME_EXTERNAL_NODES_REJECTION),
        "names the offender alone (the ticker is fine) with the pinned sim-time text: {err}"
    );
    assert!(
        !err.contains("--time-source real"),
        "the virtual loop's fix is not offered: it resolves back to sim time here: {err}"
    );
    drop(watchdog);
}

// ==========================================================================
// 6. The fire oracle over the real `/clock` topic.
// ==========================================================================

/// The hand-written schedule: a 50 ms period node, epoch at 1 s, three
/// forward samples each exactly one period on, so each sample fires the
/// deadline it lands on and the frame is stamped with that sim time. Then a
/// jump to 1.400 s passes the five deadlines 1.200 .. 1.400 s: the node fires
/// once, at 1.400 s, and its next deadline is 1.450 s, which the last sample
/// lands on.
const EPOCH_NS: u64 = 1_000_000_000;
const FORWARD_SAMPLES_NS: [u64; 5] = [
    1_050_000_000,
    1_100_000_000,
    1_150_000_000,
    1_400_000_000,
    1_450_000_000,
];

/// Publish one `rosgraph_msgs/Clock` sample of `ns` nanoseconds.
fn publish_clock(publisher: &mut cerulion_core::CerulionPublisher, ns: u64) {
    let mut proxy = publisher
        .loan_proxy::<Clock>()
        .expect("loan a clock sample");
    proxy.clock.sec = i32::try_from(ns / 1_000_000_000).expect("fits");
    proxy.clock.nanosec = u32::try_from(ns % 1_000_000_000).expect("fits");
}

/// Poll `probe` every 20 ms until it is true or `deadline` passes.
fn wait_until(deadline: Duration, what: &str, mut probe: impl FnMut() -> bool) {
    let started = Instant::now();
    while !probe() {
        assert!(
            started.elapsed() < deadline,
            "waited {deadline:?} for {what} and it did not happen"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
#[serial]
#[cfg_attr(
    target_os = "macos",
    ignore = "upstream iceoryx2 0.10.0 defect 2034: on macOS a process that has loaded a plugin node cannot create any further iceoryx2 resource, and a sim-time run builds several. Runs normally on Linux."
)]
fn a_period_node_fires_once_per_clock_sample_stamped_with_the_sim_time() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let prefix = unique_prefix("fire");
    build_workspace(&root, &[TICKER], &ticker_graph(&prefix, ""));
    let output_topic = format!("/{prefix}/ticker/cmd");

    let running = Arc::new(AtomicBool::new(true));
    let run_flag = Arc::clone(&running);
    let run_root = root.clone();
    let run_thread = std::thread::spawn(move || {
        run(RunArgs {
            root: &run_root,
            running: run_flag,
            time_source: TimeSource::Real,
            single_process: true,
            record: None,
            skip_validate: false,
        })
    });

    // The run owns the process-global transport; the clock publisher and the
    // output tap ride the same one, exactly as a sim bridge in this process
    // would.
    wait_until(
        Duration::from_secs(60),
        "the run's transport to exist",
        || TransportManager::get().is_ok(),
    );
    let transport = TransportManager::get().expect("the run's transport");
    let mut clock = transport
        .create_publisher(SIM_CLOCK_TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("the clock publisher");

    // Before the epoch nothing is built: the ticker's topic does not exist.
    assert!(
        transport.data_service_missing(&output_topic),
        "nothing is built before the first clock sample"
    );
    // The run's clock tap opens the topic only once this publisher has
    // created it, and a tap keeps no history: a sample published before it
    // attaches is never delivered. So the epoch waits for the tap.
    wait_until(
        Duration::from_secs(60),
        "the run's clock tap to attach",
        || transport.topic_subscriber_count(SIM_CLOCK_TOPIC) > 0,
    );
    publish_clock(&mut clock, EPOCH_NS);
    wait_until(
        Duration::from_secs(60),
        "the epoch to build the graph",
        || !transport.data_service_missing(&output_topic),
    );
    // The tap attaches before the first deadline can fire (the first forward
    // sample is published only after it exists), so it sees every frame.
    let mut tap = transport
        .create_data_only_subscriber_with_buffer(&output_topic, 16)
        .expect("tap the ticker's output");

    for &ns in &FORWARD_SAMPLES_NS {
        publish_clock(&mut clock, ns);
    }

    // Each poll drains within the tap's borrow budget and keeps only the
    // stamps, so no pinned sample outlives the poll that read it.
    let budget = tap.max_borrowed_samples().max(1);
    let mut frames = Vec::with_capacity(budget);
    let mut stamps = Vec::new();
    wait_until(
        Duration::from_secs(60),
        "five frames from the ticker",
        || {
            frames.clear();
            let _ = tap.drain_owned(budget, &mut frames).expect("drain the tap");
            for frame in &frames {
                let header = frame.wire_header().expect("a published frame has a header");
                stamps.push(header.timestamp_ns);
            }
            stamps.len() >= FORWARD_SAMPLES_NS.len()
        },
    );
    assert_eq!(
        stamps, FORWARD_SAMPLES_NS,
        "one fire per forward sample, each stamped with the sim time of its deadline, one \
         fire for the jump across five deadlines, and no fire at the epoch"
    );

    running.store(false, Ordering::Relaxed);
    run_thread
        .join()
        .expect("the run thread does not panic")
        .expect("a sim-time run ends cleanly when `running` clears");
}

// ==========================================================================
// 7. A missing node library beside a `ros2:` entry: refused by the library's
//    name before the entry starts, on every clock.
// ==========================================================================

/// The ticker graph with a `ros2:` launch entry whose launch file is missing.
/// The entry's own preflight refuses a missing launch file by name before it
/// spawns anything, so whichever refusal the run returns says which came
/// first: the library's, or the entry's.
fn ticker_beside_a_ros2_entry(prefix: &str, sim_time: bool) -> String {
    format!(
        "name: demo\nprefix: {prefix}\nsim_time: {sim_time}\nnodes:\n\
         - id: ticker\n  type: ticker\n  inputs: []\n  outputs:\n\
         \x20 - name: cmd\n    schema: geometry_msgs/Vector3\n\
         - id: bringup\n  ros2:\n    launch: launch/missing.launch.py\n"
    )
}

/// How the run names the ticker's missing library.
const TICKER_LIBRARY_MISSING: &str = "cdylib for node 'ticker' not found";

/// How the `ros2:` entry's own preflight names the entry: the text the run
/// must NOT return, because the entry never gets that far.
const ROS2_ENTRY_PREFLIGHT: &str = "ros2 entry 'bringup'";

/// One arm of 7: the ticker's library is missing and the report is skipped
/// (`--no-validate`), so the run's own library refusal is the only one left
/// ahead of the `ros2:` spawn.
fn a_missing_library_is_refused_before_the_ros2_entry_starts(
    tag: &str,
    sim_time: bool,
    time_source: TimeSource,
) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let _env = arm_env(tmp.path());
    let root = tmp.path().join("ws");
    let yaml = ticker_beside_a_ros2_entry(&unique_prefix(tag), sim_time);
    build_workspace_with(&root, &[TICKER], &yaml, Libraries::Missing);

    let err = run(RunArgs {
        root: &root,
        running: Arc::new(AtomicBool::new(true)),
        time_source,
        single_process: true,
        record: None,
        skip_validate: true,
    })
    .expect_err("a graph whose library is missing cannot run");
    assert!(
        err.contains(TICKER_LIBRARY_MISSING),
        "refused by the missing library's name: {err}"
    );
    assert!(
        !err.contains(ROS2_ENTRY_PREFLIGHT),
        "the ros2 entry never started, so its own preflight (the missing launch file) is \
         not the refusal: {err}"
    );
    assert!(
        run_dirs(tmp.path()).is_empty(),
        "the refusal comes before the run directory is created"
    );
}

#[test]
#[serial]
fn sim_time_with_a_missing_library_is_refused_before_its_ros2_entry_starts() {
    a_missing_library_is_refused_before_the_ros2_entry_starts("nolibsim", true, TimeSource::Real);
}

#[test]
#[serial]
fn the_real_clock_with_a_missing_library_is_refused_before_its_ros2_entry_starts() {
    a_missing_library_is_refused_before_the_ros2_entry_starts("nolibreal", false, TimeSource::Real);
}
