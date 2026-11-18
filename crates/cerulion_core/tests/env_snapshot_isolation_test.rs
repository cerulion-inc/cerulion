// SPDX-License-Identifier: AGPL-3.0-only
//! Env-snapshot isolation property test.
//!
//! Locks the contract that `NodeContext::env_snapshot` (captured at
//! `GraphRuntime::build` / `build_for_test` time from
//! `std::env::vars_os()`, keeping each variable whose name and value are
//! valid UTF-8) ISOLATES nodes from post-build env
//! mutations. Without this file the property exists in code but has no
//! test exercising it end-to-end — a regression that reverts to live
//! `std::env::var` reads (e.g. dropping the snapshot wiring) would
//! silently re-break determinism without any test firing.
//!
//! There is no
//! "without a snapshot" code path. Every NodeContext
//! constructor produces an `Arc<HashMap>` snapshot; the only
//! difference is whether it's empty (test-only `for_tests`) or
//! pre-populated (production `with_runtime_env`). Tests below
//! exclusively exercise the populated-snapshot isolation property
//! through `GraphRuntime::build_for_test`, which captures live
//! env once at build time.
//!
//! The arms for a name or value that is not valid UTF-8 never put one in
//! this process's environment. Each re-executes this binary as the child
//! test [`subprocess_child_builds_a_graph_and_prints_what_its_node_reads`]
//! with the variables set only in the child's environment, and reads what
//! the child's node read, the names each of its two builds left out and what
//! its logging printed.

use std::ffi::OsStr;
use std::io::Read;
use std::os::unix::ffi::OsStrExt as _;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportResult;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::{BuildPurpose, GraphRuntime};
use cerulion_core::testing::TestTransport;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;
use serial_test::serial;

// The isolation tests in this file mutate process env vars
// (`CER_TEST_ENV_ISO_*`); the non-UTF-8 arms set theirs only in a child's
// environment. Each `#[test]` is annotated with `#[serial]`
// (from the `serial_test` dev-dep) so they run one at a time within
// this binary.

static NODE_ID_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_id(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = NODE_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{base}_{nanos}_{id}")
}

/// Build a single-node runtime whose tick captures the result of
/// `ctx.env_str(key, default)` into the shared `recorded_value`.
fn build_runtime_recording_env_var(
    key: &'static str,
    default: &'static str,
    recorded_value: Arc<Mutex<Vec<String>>>,
) -> GraphRuntime {
    let recorded_for_tick = Arc::clone(&recorded_value);
    build_single_node_runtime(move |ctx: &mut NodeContext| {
        let v = ctx.env_str(key, default);
        recorded_for_tick.lock().unwrap().push(v);
        Ok(())
    })
}

/// Build a single-node runtime, period 1 ms, whose tick is `tick`.
fn build_single_node_runtime(
    tick: impl FnMut(&mut NodeContext) -> TransportResult<()> + Send + 'static,
) -> GraphRuntime {
    let (config, factories) = single_node_graph(tick);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test")
}

/// A single-node graph, period 1 ms, whose tick is `tick`, with its factory.
fn single_node_graph(
    tick: impl FnMut(&mut NodeContext) -> TransportResult<()> + Send + 'static,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let entry = ClosureNodeEntry::new(info, tick);

    let node_id = unique_id("env_iso");
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "env_iso".to_string(),
        prefix: "envt".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: node_id.clone(),
            node_type: node_id,
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "noop".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(config.nodes[0].id.clone(), Box::new(entry));
    (config, factories)
}

/// Step the runtime once: advance simulated time by 1ms and run one
/// scheduler step so the period-1ms node fires.
fn tick_once(rt: &mut GraphRuntime) -> TransportResult<()> {
    rt.step(std::time::Duration::from_millis(1));
    Ok(())
}

/// Happy path: build runtime → mutate env → tick → assert tick saw the
/// PRE-BUILD value (i.e. the snapshot froze at build time).
#[test]
#[serial]
fn env_snapshot_isolates_post_build_env_mutations() {
    let key = "CER_TEST_ENV_ISO_SET";
    // Defensive cleanup at test ENTRY (in case a prior panicking test
    // leaked the var). Mirrors `chunk_c_ffi_error_test.rs` pattern.
    std::env::remove_var(key);
    std::env::set_var(key, "before_build");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default", Arc::clone(&recorded));

    // Mutate env AFTER build. The snapshot was captured at build time,
    // from one read of `std::env::vars_os()`.
    std::env::set_var(key, "after_build");

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured.len(),
        1,
        "expected exactly one tick capture; got {captured:?}"
    );
    assert_eq!(
        captured[0], "before_build",
        "node must observe the env value at the time of GraphRuntime::build_for_test, \
         NOT the post-build mutation; got: {:?}",
        captured[0]
    );

    std::env::remove_var(key);
}

/// Edge: env var REMOVED after build — node must still observe the
/// snapshotted value, not "default".
#[test]
#[serial]
fn env_snapshot_unaffected_by_remove_var_post_build() {
    let key = "CER_TEST_ENV_ISO_REMOVE";
    std::env::remove_var(key); // defensive entry cleanup
    std::env::set_var(key, "captured");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default_when_unset", Arc::clone(&recorded));

    std::env::remove_var(key);

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured[0], "captured",
        "post-build remove_var must NOT cause the snapshot's value to disappear; \
         got: {:?}",
        captured[0]
    );
}

/// Adversarial: env var was UNSET at build time → snapshot lacks the
/// key → tick must return the user-supplied default. Then setting the
/// var post-build must NOT cause the node to suddenly see it.
#[test]
#[serial]
fn env_snapshot_returns_default_when_var_unset_at_build_time() {
    let key = "CER_TEST_ENV_ISO_UNSET";
    std::env::remove_var(key); // ensure unset

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default_at_build", Arc::clone(&recorded));

    // Set the var post-build. Snapshot must NOT see it.
    std::env::set_var(key, "set_after_build");

    tick_once(&mut rt).unwrap();
    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(
        captured[0], "default_at_build",
        "env_str must return user-supplied default for keys absent from \
         the snapshot, regardless of post-build mutations; got: {:?}",
        captured[0]
    );

    std::env::remove_var(key);
}

/// Determinism: TWO ticks of the same node observe the SAME snapshot
/// value, even when env mutates between ticks. (Locks that the snapshot
/// is read fresh on each tick from the FROZEN map, not snapshotted at
/// init and re-read.)
#[test]
#[serial]
fn env_snapshot_value_is_stable_across_ticks_under_env_churn() {
    let key = "CER_TEST_ENV_ISO_CHURN";
    std::env::remove_var(key); // defensive entry cleanup
    std::env::set_var(key, "stable");

    let recorded = Arc::new(Mutex::new(Vec::<String>::new()));
    let mut rt = build_runtime_recording_env_var(key, "default", Arc::clone(&recorded));

    tick_once(&mut rt).unwrap();

    std::env::set_var(key, "churned_between_ticks");
    tick_once(&mut rt).unwrap();

    std::env::remove_var(key);
    tick_once(&mut rt).unwrap();

    drop(rt);

    let captured = recorded.lock().unwrap();
    assert_eq!(captured.len(), 3, "got: {captured:?}");
    assert!(
        captured.iter().all(|v| v == "stable"),
        "all ticks must observe the snapshotted value 'stable' regardless \
         of env churn between ticks; got: {captured:?}"
    );

    std::env::remove_var(key);
}

// ===========================================================================
// A name or value that is not valid UTF-8, set only in a child's environment
// ===========================================================================

/// The keys the child's node reads, comma-separated. Set only on the child
/// this binary re-executes; the child test returns at once without it.
const CHILD_READS_ENV: &str = "CER_TEST_ENV_ISO_CHILD_READS";

/// The child test the arms below re-execute this binary as.
const CHILD_TEST: &str = "subprocess_child_builds_a_graph_and_prints_what_its_node_reads";

/// Set on the child when the arm wants it to report, once, what its builds
/// left out, as the process that owns a run does.
const CHILD_REPORTS_ENV: &str = "CER_TEST_ENV_ISO_CHILD_REPORTS";

/// The start of each line the child prints for one key its node read.
const READ_LINE: &str = "ENV_ISO_READ ";

/// The start of each line the child prints for the names one build left out,
/// followed by that build's purpose, `planning` or `run`.
const LEFT_OUT_LINE: &str = "ENV_ISO_LEFT_OUT ";

/// The warning the owner's report prints for a left-out variable, typed out
/// by hand.
const LEFT_OUT_WARNING: &str = "node environment: variables whose name or value is not valid \
                                UTF-8 are left out, so ctx.env and ctx.env_str return the \
                                default for them";

/// The bound on one child run: two builds, one tick, two teardowns.
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);

/// The child: install the logging a default `cerulion graph run` installs
/// (`cerulion=warn`, on stderr) and build a one-node graph twice, as a
/// multi-process run does. The first build is planning only, read and torn
/// down, as the supervisor's is. The second is built to run, as a worker's or
/// a single process's is: its tick reads each key in [`CHILD_READS_ENV`]
/// through `ctx.env_str` and `ctx.env_opt`, and it steps once. With
/// [`CHILD_REPORTS_ENV`] set, the child then reports the second build's
/// left-out names once, as the process that owns a run does. It prints one
/// [`LEFT_OUT_LINE`] per build and one [`READ_LINE`] per key, in order.
#[test]
#[ignore = "child process entry point, driven by the non-UTF-8 arms in this file"]
#[serial]
fn subprocess_child_builds_a_graph_and_prints_what_its_node_reads() {
    let Some(keys) = std::env::var_os(CHILD_READS_ENV) else {
        return;
    };
    let keys: Vec<String> = keys
        .to_str()
        .expect("the parent passes UTF-8 keys")
        .split(',')
        .map(str::to_string)
        .collect();
    cerulion_core::init_logging(false, true);
    let planning_transport = TestTransport::new();
    let (config, factories) = single_node_graph(|_: &mut NodeContext| Ok(()));
    let planning = GraphRuntime::build_with_schema_hashes_provisioning_and_recorded(
        config,
        factories,
        planning_transport.manager(),
        Arc::new(VirtualClock::new()),
        None,
        None,
        None,
        BuildPurpose::PlanningOnly,
    )
    .expect("the planning build");
    let planning_left_out = planning.node_env_left_out().clone();
    planning.shutdown();
    let reads = Arc::new(Mutex::new(Vec::<(String, String, Option<String>)>::new()));
    let reads_for_tick = Arc::clone(&reads);
    let mut rt = build_single_node_runtime(move |ctx: &mut NodeContext| {
        let mut reads = reads_for_tick.lock().unwrap();
        for key in &keys {
            reads.push((key.clone(), ctx.env_str(key, "default"), ctx.env_opt(key)));
        }
        Ok(())
    });
    tick_once(&mut rt).unwrap();
    let run_left_out = rt.node_env_left_out().clone();
    if std::env::var_os(CHILD_REPORTS_ENV).is_some() {
        rt.node_env_left_out().warn();
    }
    drop(rt);
    // A leading newline, because libtest's `--nocapture` progress line ends
    // without one.
    for (purpose, left_out) in [("planning", &planning_left_out), ("run", &run_left_out)] {
        println!("\n{LEFT_OUT_LINE}{purpose} {:?}", left_out.names());
    }
    for (key, env_str, env_opt) in reads.lock().unwrap().iter() {
        println!("\n{READ_LINE}{key} env_str={env_str:?} env_opt={env_opt:?}");
    }
}

/// What one child run printed.
struct ChildOutput {
    /// The child's [`READ_LINE`] lines, without the prefix, in read order.
    reads: Vec<String>,
    /// The child's [`LEFT_OUT_LINE`] lines, without the prefix: the planning
    /// build's, then the run build's.
    left_out: Vec<String>,
    /// The child's stderr, where its logging writes.
    stderr: String,
}

/// Whether the child reports what its builds left out, as the process that
/// owns a run does.
#[derive(Clone, Copy)]
enum Report {
    /// The child builds and reports nothing.
    No,
    /// The child reports once, after both builds.
    Once,
}

/// Re-execute this binary as [`CHILD_TEST`], its node reading `reads`, with
/// `vars` (raw bytes) added to its environment, and return what it printed.
///
/// The child inherits this process's environment without `RUST_LOG`, so its
/// logging takes the default filter, and without any variable whose name or
/// value is not valid UTF-8, so the only such variables in the child are the
/// ones in `vars`. Fails unless the child exits 0 within [`CHILD_TIMEOUT`] and
/// prints only UTF-8.
fn run_child(reads: &[&str], vars: &[(&[u8], &[u8])], report: Report) -> ChildOutput {
    let mut cmd = Command::new(std::env::current_exe().expect("current_exe"));
    cmd.args([
        "--exact",
        CHILD_TEST,
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(CHILD_READS_ENV, reads.join(","))
    .env_remove(CHILD_REPORTS_ENV)
    .env("NO_COLOR", "1")
    .env_remove("RUST_LOG")
    .stdin(Stdio::null())
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    for (name, value) in std::env::vars_os() {
        if name.to_str().is_none() || value.to_str().is_none() {
            cmd.env_remove(name);
        }
    }
    for (name, value) in vars {
        cmd.env(OsStr::from_bytes(name), OsStr::from_bytes(value));
    }
    if let Report::Once = report {
        cmd.env(CHILD_REPORTS_ENV, "1");
    }
    let mut child = cmd.spawn().expect("spawn the child");
    let stdout = drain(child.stdout.take().expect("the child's stdout"));
    let stderr = drain(child.stderr.take().expect("the child's stderr"));
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        if let Some(status) = child.try_wait().expect("try_wait on the child") {
            break status;
        }
        if Instant::now() > deadline {
            let _ = child.kill();
            let _ = child.wait();
            let stdout =
                String::from_utf8_lossy(&stdout.join().expect("the stdout reader")).into_owned();
            let stderr =
                String::from_utf8_lossy(&stderr.join().expect("the stderr reader")).into_owned();
            panic!(
                "the child did not exit within {CHILD_TIMEOUT:?} and was killed\nstdout:\n{stdout}\nstderr:\n{stderr}"
            );
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    let stdout = String::from_utf8(stdout.join().expect("the stdout reader"))
        .expect("the child's stdout is UTF-8");
    let stderr = String::from_utf8(stderr.join().expect("the stderr reader"))
        .expect("the child's stderr is UTF-8");
    assert!(
        status.success(),
        "the child's graph build must succeed (status {status:?})\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
    let lines_after = |prefix: &str| -> Vec<String> {
        stdout
            .lines()
            .filter_map(|line| line.strip_prefix(prefix))
            .map(str::to_string)
            .collect()
    };
    ChildOutput {
        reads: lines_after(READ_LINE),
        left_out: lines_after(LEFT_OUT_LINE),
        stderr,
    }
}

/// The lines of `stderr` that carry [`LEFT_OUT_WARNING`].
fn warning_lines(stderr: &str) -> Vec<&str> {
    stderr
        .lines()
        .filter(|line| line.contains(LEFT_OUT_WARNING))
        .collect()
}

/// Read `pipe` to its end on its own thread, so neither of the child's pipes
/// can fill while the other is read.
fn drain(mut pipe: impl Read + Send + 'static) -> std::thread::JoinHandle<Vec<u8>> {
    std::thread::spawn(move || {
        let mut bytes = Vec::new();
        pipe.read_to_end(&mut bytes)
            .expect("read the child's pipe to its end");
        bytes
    })
}

/// The read line of the UTF-8 neighbour every arm sets.
const SIBLING_READ: &str = r#"CER_TEST_ENV_ISO_SIBLING env_str="kept" env_opt=Some("kept")"#;

/// A value that is not valid UTF-8 no longer fails the build: the node reads
/// the default for it, as `std::env::var` reports such a value as an error,
/// and the UTF-8 variable set beside it still reaches the node's tick.
#[test]
#[serial]
fn env_snapshot_build_succeeds_and_a_non_utf8_value_reads_as_the_default() {
    let out = run_child(
        &["CER_TEST_ENV_ISO_NOT_UTF8", "CER_TEST_ENV_ISO_SIBLING"],
        &[
            (b"CER_TEST_ENV_ISO_NOT_UTF8", b"\xff\xfe"),
            (b"CER_TEST_ENV_ISO_SIBLING", b"kept"),
        ],
        Report::No,
    );
    assert_eq!(
        out.reads,
        [
            r#"CER_TEST_ENV_ISO_NOT_UTF8 env_str="default" env_opt=None"#,
            SIBLING_READ,
        ]
    );
}

/// A NAME that is not valid UTF-8 no longer fails the build either. Its
/// neighbour is read, and no lossy spelling of it (U+FFFD for the invalid
/// byte) is a key the node can read.
#[test]
#[serial]
fn env_snapshot_build_succeeds_with_a_non_utf8_name_and_keeps_its_neighbours() {
    let out = run_child(
        &["CER_TEST_ENV_ISO_SIBLING", "CER_TEST_ENV_ISO_NAME_\u{FFFD}"],
        &[
            (b"CER_TEST_ENV_ISO_NAME_\xff", b"v"),
            (b"CER_TEST_ENV_ISO_SIBLING", b"kept"),
        ],
        Report::No,
    );
    assert_eq!(
        out.reads,
        [
            SIBLING_READ,
            "CER_TEST_ENV_ISO_NAME_\u{FFFD} env_str=\"default\" env_opt=None",
        ]
    );
}

/// The environment of the next two arms: a name that is not valid UTF-8, a
/// secret-named value that is not, and a UTF-8 neighbour.
const MIXED_VARS: &[(&[u8], &[u8])] = &[
    (b"CER_TEST_ENV_ISO_NAME_\xff", b"plainvalue"),
    (b"CER_TEST_ENV_ISO_TOKEN", b"\xffhunter2"),
    (b"CER_TEST_ENV_ISO_SIBLING", b"kept"),
];

/// What each build of [`MIXED_VARS`] left out, in byte order, each invalid
/// byte escaped.
const MIXED_LEFT_OUT: [&str; 2] = [
    r#"planning ["CER_TEST_ENV_ISO_NAME_\xFF", "CER_TEST_ENV_ISO_TOKEN"]"#,
    r#"run ["CER_TEST_ENV_ISO_NAME_\xFF", "CER_TEST_ENV_ISO_TOKEN"]"#,
];

/// A build logs nothing about the variables it leaves out, the planning-only
/// build and the build that runs alike: each keeps their names on the runtime
/// and says nothing, so a run that builds its graph more than once is not
/// told the same thing once per build.
#[test]
#[serial]
fn env_snapshot_builds_print_nothing_and_keep_each_left_out_name() {
    let out = run_child(&["CER_TEST_ENV_ISO_SIBLING"], MIXED_VARS, Report::No);
    assert_eq!(out.left_out, MIXED_LEFT_OUT);
    assert!(
        !out.stderr.contains("node environment"),
        "no build prints the line:\n{}",
        out.stderr
    );
    assert_eq!(out.reads, [SIBLING_READ]);
}

/// After two builds, the owner's one report prints exactly one line, at the
/// level a default `cerulion graph run` shows, naming each left-out variable
/// with its invalid byte escaped, and no value: neither the secret-named value
/// that is not UTF-8 nor the value of the name that is not.
#[test]
#[serial]
fn env_snapshot_owner_report_after_two_builds_prints_one_line_naming_each_variable_and_no_value() {
    let out = run_child(&["CER_TEST_ENV_ISO_SIBLING"], MIXED_VARS, Report::Once);
    match warning_lines(&out.stderr).as_slice() {
        [line] => {
            assert!(line.contains(" WARN "), "a WARN line: {line}");
            assert!(
                line.contains(r#"names=["CER_TEST_ENV_ISO_NAME_\xFF", "CER_TEST_ENV_ISO_TOKEN"]"#),
                "both names, in byte order: {line}"
            );
        }
        other => panic!("one warning line: {other:#?}\nstderr:\n{}", out.stderr),
    }
    for value in ["hunter2", "plainvalue"] {
        assert!(
            !out.stderr.contains(value),
            "no value is printed, {value} was:\n{}",
            out.stderr
        );
    }
    assert_eq!(out.left_out, MIXED_LEFT_OUT);
    assert_eq!(out.reads, [SIBLING_READ]);
}

/// The control: a build whose environment is all UTF-8 leaves nothing out, so
/// even the owner's report prints no node-environment line.
#[test]
#[serial]
fn env_snapshot_build_of_a_utf8_environment_prints_no_node_environment_line() {
    let out = run_child(
        &["CER_TEST_ENV_ISO_SIBLING"],
        &[(b"CER_TEST_ENV_ISO_SIBLING", b"kept")],
        Report::Once,
    );
    assert!(
        !out.stderr.contains("node environment"),
        "nothing is left out, so nothing is said:\n{}",
        out.stderr
    );
    assert_eq!(out.left_out, ["planning []", "run []"]);
    assert_eq!(out.reads, [SIBLING_READ]);
}

/// Two runs under one environment that holds both kinds of left-out
/// variable read the same values, keep the same names and print the same one
/// warning, each equal to the hand-written oracle.
#[test]
#[serial]
fn env_snapshot_builds_under_one_non_utf8_environment_are_identical() {
    let vars: &[(&[u8], &[u8])] = &[
        (b"CER_TEST_ENV_ISO_NOT_UTF8", b"\xff\xfe"),
        (b"CER_TEST_ENV_ISO_NAME_\xff", b"v"),
        (b"CER_TEST_ENV_ISO_SIBLING", b"kept"),
    ];
    let reads = ["CER_TEST_ENV_ISO_NOT_UTF8", "CER_TEST_ENV_ISO_SIBLING"];
    let names = r#"["CER_TEST_ENV_ISO_NAME_\xFF", "CER_TEST_ENV_ISO_NOT_UTF8"]"#;
    let warning = format!("{LEFT_OUT_WARNING} names={names}");
    for run in 0..2 {
        let out = run_child(&reads, vars, Report::Once);
        assert_eq!(
            out.reads,
            [
                r#"CER_TEST_ENV_ISO_NOT_UTF8 env_str="default" env_opt=None"#,
                SIBLING_READ,
            ],
            "run {run}"
        );
        assert_eq!(
            out.left_out,
            [format!("planning {names}"), format!("run {names}")],
            "run {run}"
        );
        // From the message on: the timestamp before it differs by design.
        let printed: Vec<&str> = warning_lines(&out.stderr)
            .into_iter()
            .filter_map(|line| line.find(LEFT_OUT_WARNING).map(|at| &line[at..]))
            .collect();
        assert_eq!(printed, [warning.as_str()], "run {run}");
    }
}
