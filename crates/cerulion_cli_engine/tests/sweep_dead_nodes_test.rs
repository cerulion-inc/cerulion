// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion clean`'s dead-node sweep, through the ENGINE, over a real
//! registry: what `SweepMode::ReportOnly` leaves alone, what
//! `SweepMode::Remove` takes off disk, and that either reaches only the root it
//! was handed.
//!
//! # Why this binary exists
//!
//! These three arms were the behavioural half of the orphan port-tag suite,
//! which this branch deleted along with the reclaim it tested: iceoryx2 0.10
//! removes a dead port's tag with the rest of its stale resources, so the shape
//! that reclaim healed cannot arise. The sweep MODE has nothing to do with that
//! shape, and deleting the file took the only caller of
//! `sweep_dead_nodes_with_config` with it, leaving the `Remove` pass through
//! the engine with no coverage at all. That is what this restores.
//!
//! # What it proves
//!
//! A real child process mints a real dead node on an ISOLATED registry root,
//! and the engine entry point `cerulion clean` reaches is driven over it:
//!
//! * a `ReportOnly` walk names the node, attempts nothing (both counters zero,
//!   no refusal), and leaves the registry BYTE FOR BYTE as it found it,
//!   compared path by path and byte by byte, not by a count;
//! * a `Remove` walk classifies the SAME node, reports it cleaned, refuses
//!   nothing, and the directory is gone from disk;
//! * a walk handed a DIFFERENT root touches neither, and the node then comes
//!   off when its own root is swept, so "untouched" can only mean the sweep
//!   never reached it.
//!
//! The second arm is the control the first one needs. Without it, a sweep that
//! had stopped removing anything at all would satisfy every report-only
//! assertion in the file.
//!
//! # What it does NOT prove
//!
//! Nothing about the orphan port-tag shape, which no longer exists, and nothing
//! about the CLI's own rendering or its flag parsing: those are
//! `cerulion_cli`'s `clean_diagnostic_tests` and
//! `trace_inspect_and_clean_cli_test`. This binary is the engine's behaviour
//! over a real registry, and it is deliberately the only place that makes a
//! real dead node to check it.
//!
//! # One isolated root per test PROCESS
//!
//! The root is a process-global `OnceLock`, minted on first use and removed by
//! its last holder (`RootUse`). Every arm is `#[serial]`: the root IS the
//! process's iceoryx2 namespace, so two arms sharing it concurrently would
//! sweep each other's nodes. Each arm leaves the root CONVERGED, because a node
//! still registered when the root is removed strands its shared-memory objects
//! under the same prefix, which the next arm's node then collides with.
//!
//! The child is this same binary re-executed (`CHILD_TEST`, `#[ignore]`d so a
//! plain run never picks it up), in `plain` mode: it creates a publisher on the
//! isolated config, then exits without deregistering, which is what leaves a
//! dead node behind for the parent to sweep.

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use cerulion_cli_engine::ipc_cleanup::{
    sweep_dead_nodes_with_config, CleanupReport, FailedNodeCleanup, SweepMode,
};
use cerulion_cli_engine::shm_state::{creator_verdict, CreatorVerdict};
use cerulion_core::prelude::MaxSliceLen;
use cerulion_core::transport::{TransportConfig, TransportManager};
use iceoryx2::config::Config;
use iceoryx2::prelude::{FileName, FilePath, Path as IoxPath, SemanticString};
use serial_test::serial;

/// Set to "1" ONLY on the spawned child, so a bare `-- --ignored` run no-ops.
const ENV_CHILD: &str = "CER_ORPHAN_PORT_TAG_CHILD";
/// The isolated iceoryx2 root directory (absolute, trailing slash).
const ENV_ROOT: &str = "CER_ORPHAN_PORT_TAG_IOX2_ROOT";
/// The isolated iceoryx2 file prefix.
const ENV_PREFIX: &str = "CER_ORPHAN_PORT_TAG_IOX2_PREFIX";
/// The topic the child publishes on (unique per run).
const ENV_TOPIC: &str = "CER_ORPHAN_PORT_TAG_TOPIC";
/// `exit` (mint the shape and die), `linger` (mint it and stay alive until
/// killed — the live-pid arm), or `stall` (the harness's own fixture: print
/// the pid, then hang WITHOUT the lingering probe and without touching the
/// transport — the child the linger reader must give up on and reap).
const ENV_MODE: &str = "CER_ORPHAN_PORT_TAG_MODE";
/// "1" ⇒ the child uses `Config::global_config()` (the desk's real
/// registry) instead of the isolated root. Operator-driven only.
const ENV_DEFAULT_ROOT: &str = "CER_ORPHAN_PORT_TAG_DEFAULT_ROOT";
const CHILD_TEST: &str = "subprocess_child_mint_orphan_port_tag";
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);
/// Child → parent probes (stdout, one per line).
const PROBE_PID: &str = "ORPHAN_CHILD_PID=";
const PROBE_TOPIC: &str = "ORPHAN_TOPIC=";
const PROBE_LINGERING: &str = "ORPHAN_LINGERING=1";

fn isolated_config(root: &str, prefix: &str) -> Config {
    let mut cfg = Config::default();
    cfg.global
        .set_root_path(&IoxPath::new(root.as_bytes()).expect("iceoryx2 root path"));
    cfg.global.prefix = FileName::new(prefix.as_bytes()).expect("iceoryx2 prefix");
    cfg
}

/// The process's ONE isolated root — also its global iceoryx2 config (see the
/// module docs for why that is load-bearing).
struct IsolatedRoot {
    dir: PathBuf,
    root: String,
    prefix: String,
}

static ROOT: OnceLock<IsolatedRoot> = OnceLock::new();

/// How many arms hold the root RIGHT NOW. The root lives exactly as long as
/// its last holder: `acquire` creates it (or re-creates it) and counts up,
/// `Drop` counts down and removes it at zero — so a FILTERED run (one arm,
/// `--exact`) leaves nothing behind either. An arm COUNT waited for arms
/// that never ran, and every filtered run leaked its
/// `/tmp/iceoryx2/orphan_*` root. A mutex rather than an atomic, so a
/// holder arriving between the count reaching zero and the removal cannot
/// have its freshly re-created root swept from under it.
static LIVE_USES: std::sync::Mutex<usize> = std::sync::Mutex::new(0);

/// Held by every arm for as long as it uses the root; see [`LIVE_USES`].
struct RootUse;

impl RootUse {
    fn acquire() -> Self {
        let root = IsolatedRoot::get();
        let mut live = LIVE_USES.lock().unwrap_or_else(|e| e.into_inner());
        std::fs::create_dir_all(&root.dir).expect("create the isolated iceoryx2 root");
        *live += 1;
        Self
    }
}

impl Drop for RootUse {
    fn drop(&mut self) {
        let mut live = LIVE_USES.lock().unwrap_or_else(|e| e.into_inner());
        *live -= 1;
        if *live == 0 {
            if let Some(root) = ROOT.get() {
                let _ = std::fs::remove_dir_all(&root.dir);
            }
        }
    }
}

impl IsolatedRoot {
    /// Mint the root, write it as an `iceoryx2.toml`, and make it THIS
    /// process's global config — before anything else touches iceoryx2. The
    /// DIRECTORY comes and goes with its holders (`RootUse`); the config
    /// installed here is in-process state and needs the file only once.
    fn get() -> &'static Self {
        ROOT.get_or_init(|| {
            let nanos = std::time::SystemTime::now()
                .duration_since(std::time::SystemTime::UNIX_EPOCH)
                .expect("clock")
                .as_nanos();
            let dir = PathBuf::from(format!(
                "/tmp/iceoryx2/orphan_{}_{}",
                std::process::id(),
                nanos
            ));
            std::fs::create_dir_all(&dir).expect("create the isolated iceoryx2 root");
            let root = format!("{}/", dir.display());
            let prefix = format!("orphan_{}_", std::process::id());
            let this = Self { dir, root, prefix };

            let toml = toml::to_string(&this.config()).expect("serialise the isolated config");
            let file = this.dir.join("iceoryx2.toml");
            std::fs::write(&file, toml).expect("write the isolated config file");
            let global = Config::setup_global_config_from_file(
                &FilePath::new(file.as_os_str().as_bytes()).expect("config file path"),
            )
            .expect("install the isolated config as the process's global config");
            // PRECONDITION, asserted: the global config really is the isolated
            // root. If anything in this process had initialised the global
            // config first, every second sweep below would be a phantom success.
            assert!(
                Path::new(&String::from(global.global.root_path()))
                    .components()
                    .eq(this.dir.components()),
                "the process's global iceoryx2 config must be the isolated root {} (got {})",
                this.dir.display(),
                String::from(global.global.root_path())
            );
            assert_eq!(
                String::from_utf8_lossy(global.global.prefix.as_bytes()),
                this.prefix,
                "the global config must carry the isolated prefix"
            );
            this
        })
    }

    fn config(&self) -> Config {
        isolated_config(&self.root, &self.prefix)
    }

    fn nodes_dir(&self) -> PathBuf {
        self.dir.join("nodes")
    }

    fn topic(&self, arm: &str) -> String {
        format!("/orphan/{arm}/{}", std::process::id())
    }
}

/// `#[ignore]`d: only the parent spawns it (with `ENV_CHILD=1`); a bare
/// `-- --ignored` run returns at the env check.
// P12 exemption, scoped to this fn (the `barrier_test.rs` `child_worker`
// precedent): this is the body of a SELF-RE-EXEC CHILD process — a process
// entrypoint by construction — and exiting WITHOUT dropping the transport
// manager is the whole point (a dead process removes nothing, which is what
// leaves the orphan tag on disk for the parent's sweep). The ban stays armed
// for every other line in this binary.
#[allow(clippy::disallowed_methods)]
#[test]
#[ignore]
fn subprocess_child_mint_orphan_port_tag() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let topic = std::env::var(ENV_TOPIC).expect("the parent sets the topic");
    let mode = std::env::var(ENV_MODE).unwrap_or_else(|_| "exit".to_string());
    if mode == "stall" {
        let mut out = std::io::stdout().lock();
        writeln!(out).expect("probe");
        writeln!(out, "{PROBE_PID}{}", std::process::id()).expect("probe");
        out.flush().expect("flush probes");
        drop(out);
        loop {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
    let cfg = if std::env::var(ENV_DEFAULT_ROOT).as_deref() == Ok("1") {
        Config::global_config().clone()
    } else {
        let root = std::env::var(ENV_ROOT).expect("the parent sets the isolated root");
        let prefix = std::env::var(ENV_PREFIX).expect("the parent sets the isolated prefix");
        isolated_config(&root, &prefix)
    };

    // A fresh, NON-singleton manager on the handed config — nothing in this
    // child ever asks for the process singleton, so the non-singleton
    // constructor is the right one (it also disables iceoryx2's own
    // on-creation dead-node sweep, so the child never sweeps the root).
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("orphan_child_{}", std::process::id()),
            ..Default::default()
        },
        cfg,
    )
    .expect("the child initialises a transport manager on the handed config");
    // `plain` mints a node with NO port of its own: registered, never
    // deregistered, and removable by an ordinary sweep. Every other mode
    // mints the orphan-tag shape below, which a sweep REFUSES until its tags
    // are reclaimed — so an arm that needs to watch a removal SUCCEED cannot
    // use it, or "the report held the node back" could not be told apart from
    // "the removal would have failed anyway".
    if mode != "plain" {
        let mut publisher = manager
            .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
            .expect("publisher");

        // THE LEAK: a raw loan that is never returned. The sample holds the
        // publisher's shared state, and that state owns the port's on-disk tag.
        let loan = publisher.loan_raw_uninit(64).expect("raw loan");
        std::mem::forget(loan);
        // THE DESTROY: the port is deregistered from the service, the tag is
        // not — exactly the shape a leaked rmw loan leaves behind.
        drop(publisher);
    }

    let mut out = std::io::stdout().lock();
    // A leading newline: libtest has already printed `test <name> ... ` on
    // this line without a newline.
    writeln!(out).expect("probe");
    writeln!(out, "{PROBE_PID}{}", std::process::id()).expect("probe");
    writeln!(out, "{PROBE_TOPIC}{topic}").expect("probe");
    out.flush().expect("flush probes");

    // Never drop the manager: a graceful teardown is not the shape.
    std::mem::forget(manager);
    if mode == "linger" {
        writeln!(out, "{PROBE_LINGERING}").expect("probe");
        out.flush().expect("flush probes");
        drop(out);
        loop {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
    std::process::exit(0);
}

fn child_command(root: &IsolatedRoot, topic: &str, mode: &str) -> std::process::Command {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = std::process::Command::new(exe);
    cmd.args([
        "--exact",
        CHILD_TEST,
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(ENV_CHILD, "1")
    .env(ENV_ROOT, &root.root)
    .env(ENV_PREFIX, &root.prefix)
    .env(ENV_TOPIC, topic)
    .env(ENV_MODE, mode)
    .env_remove(ENV_DEFAULT_ROOT)
    .stdin(std::process::Stdio::null())
    .stdout(std::process::Stdio::piped())
    .stderr(std::process::Stdio::piped());
    cmd
}

/// A spawned child, killed and REAPED on drop — held from the instant of
/// `spawn`, so every exit from the code that drives it (a deadline, a
/// closed pipe, a failed `expect`, an assertion in the test itself) reaps
/// the child. Before it, a panic on the way to `LingerChild` left the child
/// running past the test.
struct ChildGuard(std::process::Child);

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// What these arms need from a finished child: the pid, so the sweep's
/// refusals can be attributed and the death checked. The child's streams are
/// read to completion (the pipes must drain or the child blocks) and then
/// dropped, since no arm here reads them.
struct ChildRun {
    pid: u32,
}

/// Spawn the child in `plain` mode (a dead node with no port of its own, so an
/// ordinary sweep removes it) and wait for it to die.
fn run_plain_child(root: &IsolatedRoot, arm: &str) -> ChildRun {
    run_child_in_mode(root, arm, "plain")
}

fn run_child_in_mode(root: &IsolatedRoot, arm: &str, mode: &str) -> ChildRun {
    let topic = root.topic(arm);
    let mut child = ChildGuard(
        child_command(root, &topic, mode)
            .spawn()
            .expect("spawn child"),
    );
    let mut out = child.0.stdout.take().expect("child stdout");
    let mut err = child.0.stderr.take().expect("child stderr");
    let drain_out = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = out.read_to_string(&mut buf);
        buf
    });
    let drain_err = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = err.read_to_string(&mut buf);
        buf
    });
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        match child.0.try_wait().expect("try_wait") {
            Some(status) => break status,
            None if Instant::now() > deadline => {
                // The guard's drop kills and reaps it.
                panic!("the orphan-tag child did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    };
    let stdout = drain_out.join().expect("stdout drain thread");
    let stderr = drain_err.join().expect("stderr drain thread");
    assert!(
        status.success(),
        "the orphan-tag child failed ({status:?});\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
    );
    let pid: u32 = probe(&stdout, PROBE_PID).unwrap_or_else(|| {
        panic!("the child never printed its pid probe;\n--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}")
    });
    let reported_topic: String = probe(&stdout, PROBE_TOPIC).expect("topic probe");
    assert_eq!(
        reported_topic, topic,
        "the child published on the handed topic"
    );
    // `stdout` and `stderr` are read to completion above, which is what keeps
    // the child from blocking on a full pipe; no arm here reads them, so they
    // are dropped rather than carried.
    let _ = stderr;
    ChildRun { pid }
}

/// Find `<prefix><value>` ANYWHERE in a stdout line: libtest prints
/// `test <name> ... ` WITHOUT a newline before the test body runs, so the
/// child's first probe lands on that same line.
fn probe<T: std::str::FromStr>(stdout: &str, prefix: &str) -> Option<T> {
    stdout
        .lines()
        .find_map(|l| l.find(prefix).map(|at| &l[at + prefix.len()..]))
        .and_then(|v| v.split_whitespace().next()?.parse::<T>().ok())
}

fn list_dir(dir: &Path) -> Vec<PathBuf> {
    let mut entries: Vec<PathBuf> = match std::fs::read_dir(dir) {
        Ok(entries) => entries.filter_map(|e| e.ok().map(|e| e.path())).collect(),
        Err(_) => Vec::new(),
    };
    entries.sort();
    entries
}

/// The node directories under the isolated root, sorted.
fn node_dirs(root: &IsolatedRoot) -> Vec<PathBuf> {
    list_dir(&root.nodes_dir())
        .into_iter()
        .filter(|p| p.is_dir())
        .collect()
}

/// The ONE node directory that appeared since `before` — how an arm finds
/// its own child's node without depending on the root being otherwise empty.
fn new_node_dir(root: &IsolatedRoot, before: &[PathBuf], who: &str) -> PathBuf {
    let new: Vec<PathBuf> = node_dirs(root)
        .into_iter()
        .filter(|d| !before.contains(d))
        .collect();
    assert_eq!(
        new.len(),
        1,
        "exactly one node directory must have appeared for {who}; new: {new:?}, before: {before:?}"
    );
    new[0].clone()
}

/// The refusal a sweep attributed to the node minted by `pid`, if any — the
/// node token renders `pid: <pid>,`.
fn own_failure(report: &CleanupReport, pid: u32) -> Option<&FailedNodeCleanup> {
    let key = format!("pid: {pid},");
    report.failures.iter().find(|f| f.node.contains(&key))
}

/// `shm_state`'s one liveness verdict, exactly as `cerulion clean` hands it to
/// the reclaim (the creation stamp is left `None`: the pin asserts on the pid).
fn liveness(pid: u32) -> CreatorVerdict {
    creator_verdict(pid, None)
}

fn render_report(report: &CleanupReport) -> String {
    let mut s = format!(
        "cleanups={} failed_cleanups={} failures_by_cause={:?} unclassified={:?}",
        report.cleanups, report.failed_cleanups, report.failures_by_cause, report.unclassified
    );
    for f in &report.failures {
        s.push_str(&format!(
            "\n  node {} variant {:?} causes:",
            f.node, f.variant
        ));
        for c in &f.causes {
            s.push_str(&format!("\n    {c}"));
        }
    }
    s
}

/// Every path under `dir`, relative to it, sorted, each carrying its CONTENT:
/// the bytes of a file, or `None` for a directory.
///
/// Paths alone would not be enough. A sweep that truncated or rewrote a node's
/// `iox2_node.details` in place, leaving every name where it was, would
/// compare equal to an untouched registry. A file that cannot be read is
/// recorded as its error rather than skipped, so a permission change is a
/// difference too.
fn registry_contents(dir: &Path) -> BTreeMap<PathBuf, Option<Result<Vec<u8>, String>>> {
    let mut out = BTreeMap::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(next) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&next) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let content = if path.is_dir() {
                stack.push(path.clone());
                None
            } else {
                Some(std::fs::read(&path).map_err(|e| e.to_string()))
            };
            out.insert(
                path.strip_prefix(dir)
                    .expect("a walked path is under the directory walked")
                    .to_path_buf(),
                content,
            );
        }
    }
    out
}

/// The sweep under an EXPLICIT mode, over an explicit config. Same entry point
/// `cerulion clean` reaches, with the mode the flag decides.
fn sweep_with(config: &Config, mode: SweepMode) -> CleanupReport {
    let _ = cerulion_core::iceoryx_logger::install_iceoryx2_tracing_bridge();
    sweep_dead_nodes_with_config(config, mode)
}

#[test]
#[serial]
fn a_report_only_sweep_leaves_the_planted_node_byte_for_byte() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let before_dirs = node_dirs(root);
    let child = run_plain_child(root, "report_only_untouched");
    let node_dir = new_node_dir(root, &before_dirs, "report_only_untouched");
    let before = registry_contents(&node_dir);
    assert!(
        !before.is_empty(),
        "precondition: the child must leave state under {}",
        node_dir.display()
    );

    let report = sweep_with(&root.config(), SweepMode::ReportOnly);

    // The classification names the node, and the counters stay at zero
    // because nothing was ATTEMPTED, not because nothing was refused.
    let name = node_dir
        .file_name()
        .expect("the node directory has a name")
        .to_string_lossy()
        .into_owned();
    assert!(
        report.dead_nodes.iter().any(|n| n.name == name),
        "the report must name the planted node {name} among {:?}",
        report
            .dead_nodes
            .iter()
            .map(|n| &n.name)
            .collect::<Vec<_>>()
    );
    assert_eq!(
        (report.cleanups, report.failed_cleanups),
        (0, 0),
        "a report attempts nothing, so both counters are zero: {}",
        render_report(&report)
    );
    assert!(
        report.failures.is_empty(),
        "a report refuses nothing: {}",
        render_report(&report)
    );

    // The claim, on disk: every path and every byte exactly as they were.
    assert_eq!(
        registry_contents(&node_dir),
        before,
        "a `SweepMode::ReportOnly` walk must leave the registry byte for byte as it found it"
    );
    assert!(
        liveness(child.pid) == CreatorVerdict::Gone,
        "precondition: the child must really be dead, or the node was never sweepable"
    );

    // Leave the root CONVERGED. Every arm over this harness must: the root is
    // removed by its last holder, and a node still registered at that moment
    // strands its shared-memory objects under the same prefix, which the next
    // arm's node then collides with. Sweeping here is also the cheap control
    // that the node this report held back was removable all along.
    let swept = sweep_with(&root.config(), SweepMode::Remove);
    assert!(
        swept.cleanups >= 1,
        "the node the report held back must sweep on a Remove pass: {}",
        render_report(&swept)
    );
    assert!(!node_dir.exists(), "teardown must leave the registry empty");
}

#[test]
#[serial]
fn a_removing_sweep_takes_the_node_off_disk() {
    // The other direction of the control the report-only arm needs: the node
    // it held back was removable all along. Without this, a sweep that had
    // stopped removing anything would satisfy every report-only assertion.
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let before_dirs = node_dirs(root);
    let child = run_plain_child(root, "remove_takes_it_off");
    let node_dir = new_node_dir(root, &before_dirs, "remove_takes_it_off");
    assert!(node_dir.exists(), "precondition: the node directory exists");

    // One report FIRST, over the same root, so the two modes are compared on
    // one planted node rather than on two different ones.
    let reported = sweep_with(&root.config(), SweepMode::ReportOnly);
    let name = node_dir
        .file_name()
        .expect("the node directory has a name")
        .to_string_lossy()
        .into_owned();
    assert!(
        reported.dead_nodes.iter().any(|n| n.name == name),
        "the report must name the node the removal then takes: {name}"
    );
    assert!(node_dir.exists(), "the report must not have removed it");

    let removed = sweep_with(&root.config(), SweepMode::Remove);

    assert!(
        removed.dead_nodes.iter().any(|n| n.name == name),
        "the removing sweep classifies the SAME node: {name}"
    );
    assert!(
        removed.cleanups >= 1,
        "the removing sweep must report at least this node cleaned: {}",
        render_report(&removed)
    );
    assert!(
        own_failure(&removed, child.pid).is_none(),
        "the child's node must not be refused: {}",
        render_report(&removed)
    );
    assert!(
        !node_dir.exists(),
        "the node directory must be gone after a `SweepMode::Remove` walk"
    );
}

#[test]
#[serial]
fn a_sweep_reaches_only_the_root_it_is_handed() {
    // The boundary the confinement is worth anything at, in both directions.
    // A sweep pointed at ANOTHER root must leave this one byte for byte, and
    // the node must then come off when its OWN root is swept, so "untouched"
    // can only mean the sweep never reached it rather than that there was
    // nothing there to reach.
    //
    // The other root is EMPTY rather than a second live namespace on purpose:
    // macOS caps a shared-memory name at 31 bytes, which a second prefix beside
    // this harness's own does not fit, and the scoping claim does not need a
    // second node to be true. What it needs is a config naming a different
    // root, which is exactly what the sweep is handed.
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let before_dirs = node_dirs(root);
    let child = run_plain_child(root, "boundary_scope");
    let node_dir = new_node_dir(root, &before_dirs, "boundary_scope");
    let before = registry_contents(&node_dir);
    assert!(
        !before.is_empty(),
        "precondition: the node left state on disk"
    );

    let elsewhere_dir = root.dir.with_file_name(format!(
        "{}-elsewhere",
        root.dir
            .file_name()
            .expect("the root has a name")
            .to_string_lossy()
    ));
    std::fs::create_dir_all(&elsewhere_dir).expect("create the other root");
    let elsewhere = isolated_config(&elsewhere_dir.to_string_lossy(), &root.prefix);

    let away = sweep_with(&elsewhere, SweepMode::Remove);

    assert_eq!(
        (away.cleanups, away.failed_cleanups),
        (0, 0),
        "a sweep of an empty root reaches no node: {}",
        render_report(&away)
    );
    assert!(
        away.dead_nodes.is_empty(),
        "a sweep of another root must classify none of this root's nodes: {:?}",
        away.dead_nodes
    );
    assert_eq!(
        registry_contents(&node_dir),
        before,
        "a sweep pointed at another root must leave this one byte for byte"
    );

    // The other direction: the node was removable all along, so "untouched"
    // above means "never reached", not "nothing was sweepable".
    let home = sweep_with(&root.config(), SweepMode::Remove);
    assert!(
        home.cleanups >= 1,
        "the node must sweep once its OWN root is swept: {}",
        render_report(&home)
    );
    assert!(
        own_failure(&home, child.pid).is_none(),
        "the child's node must not be refused: {}",
        render_report(&home)
    );
    assert!(!node_dir.exists(), "the node directory must be gone");
    std::fs::remove_dir_all(&elsewhere_dir).ok();
}
