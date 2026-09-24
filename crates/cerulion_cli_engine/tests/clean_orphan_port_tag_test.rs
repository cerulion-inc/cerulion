// SPDX-License-Identifier: AGPL-3.0-only
// `LibcProbe` (the real `kill(2)` liveness oracle) and the self-re-exec
// child both need a Unix process model.
#![cfg(unix)]
//! The leaked port-tag shape: iceoryx2 0.10 heals it by itself, and
//! `cerulion clean`'s reclaim still refuses everything that is not exactly it
//! — end to end, over a REAL shape minted by a real process, on an ISOLATED
//! registry root.
//!
//! # The shape
//!
//! A publisher destroyed while one of its loaned samples had been leaked
//! (`mem::forget` of the loan — the rmw destroy path does not do this, but any
//! leaked loan can) deregisters its port but leaves the port's on-disk tag
//! under `<root>/nodes/<node id>/`: the tag is owned by the publisher's shared
//! state, which every forgotten sample keeps alive until the process dies, and
//! a dead process removes nothing. The leak is still real under 0.10 — the
//! headline arm asserts the tag is on disk before it sweeps, so it cannot pass
//! because nothing was minted.
//!
//! # What 0.10 changed, and what this binary pins now
//!
//! Under 0.9.1 the dead-node sweep reclaimed a dead port's data segment and its
//! connections but never deleted the TAG
//! (`service/stale_resource_cleanup.rs::remove_stale_port_resources` took
//! `(port_id, config)` and stopped there), so `remove_node`'s final `rmdir`
//! failed `ENOTEMPTY` on EVERY sweep, forever: `failed_cleanups` never reached
//! 0, the `.shm_state` reclamation stood down for good, and the desk filled up
//! (13,647 stale mappings behind eleven such directories on a development
//! machine). `cerulion clean`'s orphan port-tag reclaim
//! (`cerulion_cli_engine::orphan_port_tags`) is what healed it.
//!
//! In 0.10 the same function takes `(node_id, port_id, config)` and ends in
//! `remove_port_tag::<Service>(node_id, port_id, config)`, so the tag goes with
//! the port and the node directory is removed on the FIRST sweep. The headline
//! arm is now the REGRESSION TEST for that upstream fix — one child, one sweep,
//! `cleanups == 1`, `failed_cleanups == 0`, no refusal, no surviving
//! `<prefix>*.port_tag` and no surviving directory — so an iceoryx2 that
//! regresses it is caught on the next run rather than after a desk fills up.
//!
//! # The reclaim's refusals, over a HAND-PLANTED shape
//!
//! The remaining arms are about what the RECLAIM refuses, not about who
//! produced the shape, so they plant the directory themselves
//! (`plant_orphan_tags`) instead of waiting for one the library will no longer
//! leave standing. Each names the change that fails it:
//!
//! * a directory holding a stray entry is refused whole, the stray named, and
//!   nothing is removed — not even the tags that do qualify (skipping the
//!   offender refusal in `orphan_tags_in`);
//! * a candidate carrying a LIVE pid is refused by the death guard with its
//!   tags untouched (treating `CreatorVerdict::Alive` as gone) — beside the two
//!   facts the sweep supplies for a live process: its node's directory survives
//!   a sweep untouched, and once it dies the library heals that node too;
//! * `dry_run` lists the tags and takes nothing off disk;
//! * a node directory replaced by a SYMBOLIC LINK to a directory outside the
//!   root is refused, the link named and the outside tag untouched — the
//!   reclaim binds identity by descriptor, never by path;
//! * an ALREADY-EMPTY directory is `AlreadyEmpty` — not a refusal, nothing
//!   removed, and the directory left standing for a sweep's `remove_node`.
//!
//! A planted directory is INERT to the sweep by construction, which is why
//! those arms assert on the reclaim and on the disk rather than on a following
//! sweep: `Node::list` enumerates nodes from the MONITOR entries beside the
//! detail directories (`<root>/nodes/<prefix><node id><monitor suffix>`, built
//! by `node_monitoring_config`), so a directory with no monitor entry beside it
//! is never listed, never classified dead, and never swept.
//!
//! The child also has a DEFAULT-ROOT mode (`CER_ORPHAN_PORT_TAG_DEFAULT_ROOT=1`),
//! never used by these tests, so an operator can mint exactly ONE such
//! directory on a desk's real registry and watch what the library does with it.
//!
//! # The identity an arm hands the reclaim
//!
//! 0.10's node identity is `UniqueNodeId(UniqueId { payload_value, unique_value })`:
//! it carries NO pid and NO creation stamp, where 0.9.1's carried both
//! (`UniqueSystemId { value, pid, creation_time }`). Two consequences, both
//! read off the linked library rather than assumed:
//!
//! * a refusal can be attributed to a node by its ID only, never by the pid
//!   that minted it, so the arms here scope their sweep assertions with the
//!   whole of `CleanupReport::failures` (empty, over a root that holds exactly
//!   the arm's own nodes) and name the node in the failure message;
//! * `orphan_port_tags::orphan_port_tag_candidates` cannot select ANYTHING
//!   under 0.10 — it reads `pid:` out of the identity and gives up when it is
//!   absent — so the arms build their `OrphanTagNode` directly, rendering the
//!   identity exactly as `Node::list` does for a given id (`node_identity`).
//!   The reclaim reads that string only to look for a `Realtime` creation
//!   stamp; it finds none and falls through to `kill(2)` on the candidate's
//!   pid, which is what every arm here turns on.
//!
//! # ONE isolated root per test PROCESS, and it IS the process's global config
//!
//! A node listed `Dead` WITHOUT its details storage is cleaned against
//! `Config::global_config()` rather than the swept config
//! (`iceoryx2-0.10.0/src/node/mod.rs:622-626`), so a per-test root that is not
//! the process's global config aims part of a sweep at the DESK's registry, and
//! a cleaner acquisition against the wrong root answers `DoesNotExist`, which
//! `blocking_remove_stale_resources` counts as `ResourcesAlreadyCleanedUp ==
//! Ok` — a phantom success (MEASURED under 0.9.1: every second sweep reported
//! `cleanups == 1` while the node directory, tag and all, survived). So this
//! binary mints ONE root, writes it as an `iceoryx2.toml`, and installs it as
//! the process's global config through `Config::setup_global_config_from_file`
//! before anything touches iceoryx2 — asserted, not assumed. Every arm keeps
//! its assertions scoped to its own nodes and its own directories, so an arm
//! that fails cannot cascade into the next one's counts.
//!
//! `#[serial]`: one root, one process-global log level (the diagnostics sweep
//! pins iceoryx2 at `Trace` for its duration and restores it after — two
//! sweeps in flight could restore the level under each other and drop the
//! lines the classifier reads).
//!
//! ```bash
//! cargo test -p cerulion_cli_engine --test clean_orphan_port_tag_test
//! ```

use std::io::{BufRead, BufReader, Read, Write};
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::{Duration, Instant};

use cerulion_cli_engine::ipc_cleanup::{
    cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config, CleanupReport,
};
use cerulion_cli_engine::orphan_port_tags::{
    reclaim_orphan_port_tags, OrphanTagNode, OrphanTagReclaim, ReclaimVerdict,
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

// =====================================================================
// The isolated config: root + prefix travel by env var so parent and
// child rebuild byte-identical `Config`s.
// =====================================================================

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

/// The linger reader's deadline CAN fire, and its child is REAPED on every
/// exit. A `stall` child prints its pid and then hangs WITHOUT the lingering
/// probe: the reader must give up on it AT the deadline (not before, not
/// much after), and afterwards `kill(pid, 0)` must say the process is GONE —
/// killed AND reaped, since a zombie still answers `kill(pid, 0)` (the
/// `.shm_state` reclamation's own liveness oracle, so `Gone` here is the
/// reap, not just the kill). Before the reader thread, a blocking
/// `read_line` never returned to check the deadline — this arm HUNG; before
/// the guard-from-spawn, the failure left the child running.
#[test]
#[serial]
fn a_child_that_never_reports_lingering_is_given_up_on_at_the_deadline_and_reaped() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let deadline = Duration::from_secs(5);
    let started = Instant::now();
    let failure = match LingerChild::try_spawn(root, "stall", "stall", deadline) {
        Ok(child) => panic!(
            "a stalling child must not be accepted as lingering (pid {})",
            child.pid
        ),
        Err(failure) => failure,
    };
    let elapsed = started.elapsed();
    assert!(
        elapsed >= deadline,
        "given up on AT the deadline, not before: {elapsed:?} ({failure})"
    );
    assert!(
        elapsed < deadline + Duration::from_secs(30),
        "given up on at the deadline, not long after — the kill and reap are immediate: \
         {elapsed:?} ({failure})"
    );
    assert!(
        failure.reason.contains("did not report it was lingering"),
        "{failure}"
    );
    // The OS pid, not the probe: the reap is proved even on a runner where
    // the child never got as far as printing (its first exec can stall).
    assert_eq!(
        liveness(failure.pid),
        CreatorVerdict::Gone,
        "the child is killed AND reaped when the failure is handed back (a zombie would still \
         read Alive): {failure}"
    );
    if let Some(probe) = failure.probe_pid {
        assert_eq!(
            probe, failure.pid,
            "the child's own probe names the spawned process"
        );
    }
}

/// The root lives exactly as long as its LAST holder: an inner holder's drop
/// leaves it for the outer one, the last drop removes it, and the next
/// holder re-creates it — the property a filtered run relies on.
#[test]
#[serial]
fn the_isolated_root_is_removed_by_its_last_holder_and_recreated_by_the_next() {
    let root = IsolatedRoot::get();
    {
        let _outer = RootUse::acquire();
        assert!(root.dir.is_dir(), "a holder has a root");
        {
            let _inner = RootUse::acquire();
            assert!(root.dir.is_dir());
        }
        assert!(
            root.dir.is_dir(),
            "an inner holder's drop leaves the root for the outer one"
        );
    }
    assert!(
        !root.dir.exists(),
        "the last holder's drop removes the root: {}",
        root.dir.display()
    );
    let _again = RootUse::acquire();
    assert!(root.dir.is_dir(), "the next holder re-creates it");
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
            // config first, a sweep that met a node without its details storage
            // would be aimed at the DESK's registry instead of this one (see the
            // module docs) — and would answer with a phantom success.
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

// =====================================================================
// THE CHILD: mint the shape, then die (or linger).
// =====================================================================

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
    let mut publisher = manager
        .create_publisher(&topic, MaxSliceLen::const_new(4096), 0)
        .expect("publisher");

    // THE LEAK: a raw loan that is never returned. The sample holds the
    // publisher's shared state, and that state owns the port's on-disk tag.
    let loan = publisher.loan_raw_uninit(64).expect("raw loan");
    std::mem::forget(loan);
    // THE DESTROY: the port is deregistered from the service, the tag is not
    // — exactly the shape a leaked rmw loan leaves behind.
    drop(publisher);

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

// =====================================================================
// THE PARENT — helpers
// =====================================================================

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

impl ChildGuard {
    fn pid(&self) -> u32 {
        self.0.id()
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

struct ChildRun {
    pid: u32,
    stdout: String,
    stderr: String,
}

/// Spawn the child in `exit` mode and wait for it to die.
fn run_exit_child(root: &IsolatedRoot, arm: &str) -> ChildRun {
    let topic = root.topic(arm);
    let mut child = ChildGuard(
        child_command(root, &topic, "exit")
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
    ChildRun {
        pid,
        stdout,
        stderr,
    }
}

/// A child in `linger` mode: alive until dropped (killed + reaped).
struct LingerChild {
    _child: ChildGuard,
    pid: u32,
}

/// Why a linger child was not accepted — with the OS pid the harness
/// spawned (always known, so the caller can prove the guard reaped it even
/// if the child never got as far as printing its own) and the pid it
/// reported, if it did.
struct LingerSpawnFailure {
    pid: u32,
    probe_pid: Option<u32>,
    reason: String,
}

impl std::fmt::Display for LingerSpawnFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} (pid {}, pid probe: {:?})",
            self.reason, self.pid, self.probe_pid
        )
    }
}

impl LingerChild {
    fn spawn(root: &IsolatedRoot, arm: &str) -> Self {
        Self::try_spawn(root, arm, "linger", CHILD_TIMEOUT).unwrap_or_else(|e| panic!("{e}"))
    }

    /// Spawn the child in `mode` and wait up to `timeout` for its lingering
    /// probe. On EVERY failure the child is already killed and reaped when
    /// this returns: the guard is taken at the instant of `spawn`, and the
    /// `Err` is built after it drops.
    fn try_spawn(
        root: &IsolatedRoot,
        arm: &str,
        mode: &str,
        timeout: Duration,
    ) -> Result<Self, LingerSpawnFailure> {
        let topic = root.topic(arm);
        let mut child = ChildGuard(
            child_command(root, &topic, mode)
                .spawn()
                .expect("spawn linger child"),
        );
        let out = child.0.stdout.take().expect("child stdout");
        // A reader THREAD hands lines over a channel, so the deadline is
        // checked between WAITS rather than between LINES: a blocking
        // `read_line` on a child that printed its pid and then hung never
        // returned to check it, and the "deadline" was a line the harness
        // could not reach. The thread outlives this call — it keeps draining
        // after the probe so the child can never block on a full pipe — and
        // ends when the pipe closes.
        let (tx, rx) = std::sync::mpsc::channel::<String>();
        std::thread::spawn(move || {
            let mut reader = BufReader::new(out);
            loop {
                let mut line = String::new();
                match reader.read_line(&mut line) {
                    Ok(0) | Err(_) => return,
                    Ok(_) => {
                        if tx.send(line).is_err() {
                            // The parent stopped listening (it has its
                            // answer); drain to the end for the child's sake.
                            let mut sink = String::new();
                            let _ = reader.read_to_string(&mut sink);
                            return;
                        }
                    }
                }
            }
        });
        let mut pid = None;
        let deadline = Instant::now() + timeout;
        let failure = loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(remaining) {
                Ok(line) => {
                    if let Some(p) = probe::<u32>(&line, PROBE_PID) {
                        pid = Some(p);
                    }
                    if line.contains(PROBE_LINGERING) {
                        break None;
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    break Some(format!(
                        "the linger child did not report it was lingering within {timeout:?}"
                    ));
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    break Some("the linger child closed stdout before lingering".to_string());
                }
            }
        };
        if let Some(reason) = failure {
            // Kill + reap BEFORE the failure is handed back, so a caller can
            // assert the child is gone.
            let os_pid = child.pid();
            drop(child);
            return Err(LingerSpawnFailure {
                pid: os_pid,
                probe_pid: pid,
                reason,
            });
        }
        let reported = pid.expect("the linger child printed its pid before lingering");
        assert_eq!(
            reported,
            child.pid(),
            "the child's own pid probe is the process the harness spawned"
        );
        Ok(Self {
            _child: child,
            pid: reported,
        })
    }
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

fn names_in(dir: &Path) -> Vec<String> {
    list_dir(dir)
        .iter()
        .filter_map(|p| p.file_name().map(|n| n.to_string_lossy().into_owned()))
        .collect()
}

fn port_tags(node_dir: &Path, prefix: &str) -> Vec<PathBuf> {
    list_dir(node_dir)
        .into_iter()
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with(prefix) && n.ends_with(".port_tag"))
        })
        .collect()
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

/// The node identity `Node::list` renders for `node_id` under iceoryx2 0.10.
/// The walk rebuilds the identity from the directory name —
/// `UniqueNodeId(UniqueId::from_raw_id(value))`, `iceoryx2-0.10.0/src/node/mod.rs:1212-1215`
/// — and `UniqueId`'s derived `Debug` prints the two halves it stores: the low
/// 64 bits of the value as `payload_value`, the high 64 as `unique_value`
/// (`iceoryx2-0.10.0/src/unique_id_generator/mod.rs:33-54`). Neither a pid nor
/// a creation stamp appears in it, which is why the arms below carry the pid in
/// the candidate's own field and name a node by its ID.
fn node_identity(node_id: u128) -> String {
    format!(
        "UniqueNodeId(UniqueId {{ payload_value: {}, unique_value: {} }})",
        node_id as u64,
        (node_id >> 64) as u64
    )
}

/// The diagnostics sweep, exactly as `cerulion clean` runs it, over the
/// isolated config. The bridge must be installed for the capture to see
/// iceoryx2's lines; `install_iceoryx2_tracing_bridge` is `Once`-guarded, so
/// calling it per test is harmless.
fn sweep(root: &IsolatedRoot) -> CleanupReport {
    let _ = cerulion_core::iceoryx_logger::install_iceoryx2_tracing_bridge();
    // The explicit config equals the process's global one (see `IsolatedRoot::get`),
    // exactly as `cerulion clean` hands `Config::global_config()` to the same fn.
    cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config(&root.config())
}

/// `shm_state`'s one liveness verdict, exactly as `cerulion clean` hands it to
/// the reclaim (the creation stamp is left `None`: the pin asserts on the pid).
fn liveness(pid: u32) -> CreatorVerdict {
    creator_verdict(pid, None)
}

fn reclaim(
    root: &IsolatedRoot,
    candidates: &[OrphanTagNode],
    dry_run: bool,
) -> Vec<OrphanTagReclaim> {
    reclaim_orphan_port_tags(candidates, &root.config(), dry_run, &creator_verdict)
}

fn render_report(report: &CleanupReport) -> String {
    let mut s = format!(
        "cleanups={} failed_cleanups={} failures_by_cause={:?} unclassified={:?}",
        report.cleanups, report.failed_cleanups, report.failures_by_cause, report.unclassified
    );
    for e in &report.registry_errors {
        s.push_str(&format!("\n  registry_error {e}"));
    }
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

// =====================================================================
// A HAND-PLANTED orphan shape — the directory 0.9.1's sweep used to leave
// standing, built by this binary because 0.10's no longer produces one.
// =====================================================================

/// A libtest filter that selects nothing, so the process runs no test at all
/// and exits 0. Its only product is a pid this process has reaped.
const NO_SUCH_TEST: &str = "subprocess_no_test_matches_this_filter";

/// A pid that is PROVABLY gone: a child of this process, spawned on this test
/// binary with a filter that selects no test, then WAITED on — which reaps it,
/// so `kill(pid, 0)` answers `ESRCH` rather than finding a zombie. Asserted
/// through the same predicate the reclaim's death guard uses, so a candidate
/// built on it starts from a measured death rather than a chosen number.
///
/// Pid reuse between here and the reclaim would read `Alive` and REFUSE: a
/// false red, never a false green.
fn reaped_pid() -> u32 {
    let exe = std::env::current_exe().expect("current_exe");
    let child = std::process::Command::new(exe)
        .args(["--exact", NO_SUCH_TEST])
        .env_remove(ENV_CHILD)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("spawn a short-lived child for its pid");
    let pid = child.id();
    let out = child
        .wait_with_output()
        .expect("wait for the short-lived child");
    assert!(
        out.status.success(),
        "a run that selects no test exits 0 ({:?});\n--- stdout ---\n{}\n--- stderr ---\n{}",
        out.status,
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(
        liveness(pid),
        CreatorVerdict::Gone,
        "a reaped child's pid must read Gone — a zombie would still read Alive and the \
         death guard would refuse every arm below for the wrong reason"
    );
    pid
}

/// The registry's node directory as the LIBRARY derives it: the reclaim
/// re-checks every candidate against `<config.global.node_dir()>/<node id>`, so
/// a planted directory has to land exactly there. The assertion holds this
/// binary's own idea of the layout (`<root>/nodes`) against the library's
/// answer, so a layout change fails here rather than turning every planted arm
/// into a refusal nobody expected.
fn registry_nodes_dir(root: &IsolatedRoot) -> PathBuf {
    let config = root.config();
    let from_config = PathBuf::from(String::from(&config.global.node_dir()));
    assert!(
        from_config.components().eq(root.nodes_dir().components()),
        "the library keeps node directories in `{}`; this binary plants and lists them in `{}`",
        from_config.display(),
        root.nodes_dir().display()
    );
    from_config
}

/// The configured port-tag suffix, read off the config rather than spelled out,
/// so a planted tag is named the way the library names one and the reclaim's
/// `<prefix><port id><suffix>` proof is exercised against the real spelling.
fn port_tag_suffix(root: &IsolatedRoot) -> String {
    let config = root.config();
    String::from_utf8_lossy(config.global.node.port_tag_suffix.as_bytes()).into_owned()
}

/// Node ids for planted directories: the high half is this process's pid, the
/// low half a per-process counter. That is the exact split
/// `UniqueId::from_raw_id` makes of a value (low 64 bits `payload_value`, high
/// 64 `unique_value`), so [`node_identity`] renders a planted id the way the
/// library would, and no two plants — in this process or a concurrent one —
/// can name the same directory.
static PLANTED_NODES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn planted_node_id() -> u128 {
    let n = PLANTED_NODES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    (u128::from(std::process::id()) << 64) | u128::from(n + 1)
}

/// A node directory planted by hand, and the candidate that aims the reclaim at
/// it.
struct PlantedShape {
    /// The directory's name, and what the reclaim re-derives
    /// `<node dir>/<node id>` from.
    node_id: u128,
    dir: PathBuf,
    /// The planted tag names, sorted — an arm asserts against these to say
    /// "untouched" without re-deriving them.
    tag_names: Vec<String>,
    candidate: OrphanTagNode,
}

/// Plant a node directory holding one `<prefix><port id><suffix>` per entry of
/// `port_ids` AND NOTHING ELSE — the shape 0.9.1's sweep left behind and 0.10
/// removes on the spot, so an arm about the reclaim's refusals can no longer
/// get it from the library.
///
/// The candidate carries a REAPED pid, so the death guard passes for the right
/// reason; an arm that wants that guard to bite overrides the pid with a live
/// one. The directory is invisible to `Node::list` (no monitor entry beside
/// it), so nothing sweeps it out from under the arm.
fn plant_orphan_tags(root: &IsolatedRoot, port_ids: &[u128]) -> PlantedShape {
    let node_id = planted_node_id();
    let dir = registry_nodes_dir(root).join(node_id.to_string());
    std::fs::create_dir_all(&dir).expect("plant the node directory");
    let suffix = port_tag_suffix(root);
    let mut tag_names: Vec<String> = port_ids
        .iter()
        .map(|port_id| {
            let name = format!("{}{port_id}{suffix}", root.prefix);
            std::fs::write(dir.join(&name), b"").expect("plant a port tag");
            name
        })
        .collect();
    tag_names.sort();
    assert_eq!(
        names_in(&dir),
        tag_names,
        "the planted directory must hold the tags and nothing else"
    );
    assert_eq!(
        port_tags(&dir, &root.prefix).len(),
        port_ids.len(),
        "a planted tag must be recognisable as one to this binary's own listing: {:?}",
        names_in(&dir)
    );
    PlantedShape {
        node_id,
        dir: dir.clone(),
        tag_names,
        candidate: OrphanTagNode {
            node: node_identity(node_id),
            node_id,
            pid: reaped_pid(),
            dir,
        },
    }
}

// =====================================================================
// The REAL shape, minted by a real process.
// =====================================================================

/// Mint the shape with an exit-mode child and assert the preconditions the
/// headline arm needs — all NODE-SCOPED to the child's own directory: exactly
/// one node directory appeared for the child, it is named after a node id, it
/// carries a `<prefix>*.port_tag` (the child's port outlived it — MEASURED on
/// the linked library, never assumed from what 0.9.1 did), and the child is
/// dead. No sweep happens here: the arm is what sweeps, once, and the sweep is
/// what is under test.
struct MintedShape {
    child: ChildRun,
    node_dir: PathBuf,
    node_id: u128,
    /// The tags on disk BEFORE the sweep — quoted back when the sweep is
    /// supposed to have removed them and did not.
    tags_before: Vec<PathBuf>,
}

fn mint_leaked_tag_shape(root: &IsolatedRoot, arm: &str) -> MintedShape {
    let before = node_dirs(root);
    let child = run_exit_child(root, arm);
    let node_dir = new_node_dir(root, &before, arm);
    let tags_before = port_tags(&node_dir, &root.prefix);
    assert!(
        !tags_before.is_empty(),
        "precondition: the dead node's directory carries no `{}*.port_tag` — the child's \
         port did not outlive it, so this run pins nothing; contents: {:?}\n--- child stdout ---\n{}\n--- child stderr ---\n{}",
        root.prefix,
        names_in(&node_dir),
        child.stdout,
        child.stderr
    );
    let node_id = node_dir
        .file_name()
        .and_then(|n| n.to_str())
        .and_then(|n| n.parse::<u128>().ok())
        .unwrap_or_else(|| {
            panic!(
                "a node directory is named after its node id in decimal: {}",
                node_dir.display()
            )
        });
    assert_eq!(
        liveness(child.pid),
        CreatorVerdict::Gone,
        "the exit-mode child is dead by now;\n--- child stderr ---\n{}",
        child.stderr
    );

    MintedShape {
        child,
        node_dir,
        node_id,
        tags_before,
    }
}

// =====================================================================
// THE ARMS
// =====================================================================

/// THE UPSTREAM FIX, pinned: the LIBRARY removes a leaked port tag and
/// converges the node on the FIRST sweep.
///
/// Until iceoryx2 0.10 this was Cerulion's job. `remove_stale_port_resources`
/// reclaimed a dead port's data segment and its connections and left the
/// port's on-disk tag where it was, so `remove_node`'s `rmdir` failed
/// `ENOTEMPTY` on every sweep and the node directory stood forever; what healed
/// it was `cerulion clean`'s orphan port-tag reclaim, which removed the tag and
/// let the next sweep finish. 0.10's
/// `remove_stale_port_resources(node_id, port_id, config)` ends in
/// `remove_port_tag`, and this arm is the proof of that over a real minted
/// shape: the tag IS on disk when the sweep starts (asserted, so the arm cannot
/// pass because nothing leaked), and after ONE sweep the node is counted
/// cleaned, nothing is refused, no `<prefix>*.port_tag` survives, and the
/// directory itself is gone.
///
/// It stays here so an iceoryx2 that regresses the fix is caught on the next
/// run: the failure mode it replaces announced itself only after a desk had
/// accumulated 13,647 stale mappings behind eleven stranded directories.
#[test]
#[serial]
fn the_library_removes_a_leaked_port_tag_and_the_sweep_converges() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_leaked_tag_shape(root, "headline");

    let report = sweep(root);
    assert!(
        report.failures.is_empty(),
        "the sweep must refuse nothing; the node it just met is {}: {}\n--- child stderr ---\n{}",
        node_identity(shape.node_id),
        render_report(&report),
        shape.child.stderr
    );
    assert_eq!(report.failed_cleanups, 0, "{}", render_report(&report));
    assert_eq!(
        report.cleanups,
        1,
        "the child's node is the only node under this root, and it is cleaned: {}",
        render_report(&report)
    );
    assert!(
        port_tags(&shape.node_dir, &root.prefix).is_empty(),
        "the library must remove the tag it used to leave behind (on disk before the sweep: \
         {:?}); the directory now holds {:?}",
        shape.tags_before,
        names_in(&shape.node_dir)
    );
    assert!(
        !shape.node_dir.exists(),
        "the node directory must be gone after the sweep; it holds {:?}",
        names_in(&shape.node_dir)
    );
}

/// ANTI-TAUTOLOGY (skipping the offender refusal in `orphan_tags_in`): a node
/// directory holding anything that is not an orphan port tag is refused WHOLE,
/// the offender is named, and nothing is removed — not even the tags that do
/// qualify. A concurrent session can write into that directory between the
/// selection and the reclaim, and the reclaim's safety argument covers a
/// directory of pure residue only.
///
/// The shape is planted by hand: 0.10 removes a leaked tag on the first sweep,
/// so a directory of nothing but tags is no longer something the library will
/// hold still for. The control is that the stray was the ONLY obstacle —
/// remove it by hand and the same call reclaims.
#[test]
#[serial]
fn a_stray_entry_planted_before_the_reclaim_is_refused_named_and_nothing_is_removed() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = plant_orphan_tags(root, &[7]);
    let stray = shape.dir.join("stray.txt");
    std::fs::write(&stray, b"not a tag").expect("plant the stray");
    let before = names_in(&shape.dir);

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    let refused = reclaims[0]
        .refused()
        .unwrap_or_else(|| panic!("a directory holding a stray must be refused: {reclaims:?}"));
    assert!(
        refused.contains("`stray.txt`"),
        "the refusal must name the stray: {refused}"
    );
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert_eq!(names_in(&shape.dir), before, "nothing removed");
    assert_eq!(
        port_tags(&shape.dir, &root.prefix).len(),
        1,
        "the qualifying tag is refused along with the rest: {:?}",
        names_in(&shape.dir)
    );

    // The control: the refusal was the ONLY obstacle.
    std::fs::remove_file(&stray).expect("remove the stray by hand");
    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(
        reclaims[0].removed,
        vec![7_u128],
        "the reclaim reports the port id it removed: {reclaims:?}"
    );
    assert!(
        names_in(&shape.dir).is_empty(),
        "the tag is gone from disk: {:?}",
        names_in(&shape.dir)
    );
    assert!(
        shape.dir.is_dir(),
        "the reclaim leaves the directory for a sweep's `remove_node`"
    );
}

/// ANTI-TAUTOLOGY (treating `CreatorVerdict::Alive` as gone): a candidate whose
/// pid is ALIVE is refused by the death guard, the pid named, its tags
/// untouched — the guard is what stands between the reclaim and a running
/// process's registry state, and a pid reused by an unrelated process is
/// refused with it, which is the safe direction.
///
/// Around that, the two things the 0.10 sweep says about a live process, both
/// measured here rather than assumed: a LIVE node is not swept at all (its
/// directory and its own leaked tag survive untouched, `cleanups == 0`), and
/// once that same process dies the library heals its node exactly as it heals
/// the headline's — the linger child mints the same leaked-loan shape, so this
/// is the second minting path through this pin.
#[test]
#[serial]
fn a_live_nodes_directory_survives_the_sweep_and_its_pid_refuses_the_reclaim() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let before = node_dirs(root);
    let linger = LingerChild::spawn(root, "linger");
    let linger_dir = new_node_dir(root, &before, "the linger child");
    assert_eq!(
        liveness(linger.pid),
        CreatorVerdict::Alive,
        "the linger child is alive"
    );
    let linger_tags = port_tags(&linger_dir, &root.prefix);
    assert_eq!(
        linger_tags.len(),
        1,
        "the linger child leaked exactly one port tag: {:?}",
        names_in(&linger_dir)
    );

    // A live node is none of a sweep's business.
    let first = sweep(root);
    assert!(first.failures.is_empty(), "{}", render_report(&first));
    assert_eq!(
        first.cleanups,
        0,
        "the only node under this root is ALIVE, so nothing is cleaned: {}",
        render_report(&first)
    );
    assert!(linger_dir.is_dir(), "the live node's directory stands");
    assert_eq!(
        port_tags(&linger_dir, &root.prefix),
        linger_tags,
        "and so does its tag"
    );

    // THE DEATH GUARD: an orphan directory that is real, aimed at with a pid
    // that is alive.
    let shape = plant_orphan_tags(root, &[11]);
    let live_pid_candidate = OrphanTagNode {
        pid: linger.pid,
        ..shape.candidate.clone()
    };
    let reclaims = reclaim(root, &[live_pid_candidate], false);
    let refused = reclaims[0]
        .refused()
        .unwrap_or_else(|| panic!("a live pid must refuse the reclaim: {reclaims:?}"));
    assert!(
        refused.contains("still alive") && refused.contains(&linger.pid.to_string()),
        "the refusal must say the process is alive and name it: {refused}"
    );
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert_eq!(
        names_in(&shape.dir),
        shape.tag_names,
        "the tags must survive a refused reclaim"
    );

    // The control: the SAME directory, carrying the pid this binary reaped.
    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed, vec![11_u128], "{reclaims:?}");
    assert!(
        names_in(&shape.dir).is_empty(),
        "the tag is gone once the pid is provably dead: {:?}",
        names_in(&shape.dir)
    );

    // The linger child dies: the library heals its node the way it healed the
    // headline's, and the planted directory beside it is invisible to the sweep
    // (no monitor entry), so the counters below are about the linger node alone.
    let linger_pid = linger.pid;
    drop(linger);
    assert_eq!(
        liveness(linger_pid),
        CreatorVerdict::Gone,
        "the linger child is killed AND reaped when its handle drops"
    );
    let second = sweep(root);
    assert!(second.failures.is_empty(), "{}", render_report(&second));
    assert_eq!(
        second.cleanups,
        1,
        "the now-dead linger node, tag and all, is cleaned: {}",
        render_report(&second)
    );
    assert!(!linger_dir.exists(), "the linger node's directory is gone");
}

/// `dry_run` LISTS the tags it would remove and takes nothing off disk. Under
/// 0.9.1 the proof of "heals nothing" was that the next sweep still refused the
/// node; 0.10's sweep refuses nothing and never sees a planted directory
/// anyway, so the oracle is the disk itself — the tags are still there, and the
/// same call without `dry_run` is what removes them.
#[test]
#[serial]
fn a_dry_run_lists_the_tag_and_removes_nothing() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = plant_orphan_tags(root, &[3, 5]);

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), true);
    assert_eq!(reclaims.len(), 1);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(
        reclaims[0].removed,
        vec![3_u128, 5],
        "both port ids are listed: {reclaims:?}"
    );
    assert_eq!(
        names_in(&shape.dir),
        shape.tag_names,
        "nothing removed under dry_run"
    );

    // The control: the real reclaim does.
    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed, vec![3_u128, 5], "{reclaims:?}");
    assert!(
        names_in(&shape.dir).is_empty(),
        "both tags are gone: {:?}",
        names_in(&shape.dir)
    );
    assert!(
        shape.dir.is_dir(),
        "the reclaim leaves the directory for a sweep's `remove_node`"
    );
}

/// An ALREADY-EMPTY candidate directory — the tags came off by other means
/// between the selection and the reclaim (a concurrent session's reclaim, or an
/// earlier `cerulion clean` interrupted after its unlinks and before its second
/// sweep) — is `AlreadyEmpty`: not a refusal, nothing removed, and the
/// directory LEFT STANDING, because removing it is a sweep's `remove_node`'s
/// job and never the reclaim's. This is the verdict that makes the verb run its
/// second sweep whenever candidates EXISTED: a sweep gated on "a tag came off"
/// would leave exactly this node standing for one more run.
#[test]
#[serial]
fn an_already_empty_candidate_is_converged_pending_sweep_and_the_directory_is_left_standing() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = plant_orphan_tags(root, &[9]);
    for tag in port_tags(&shape.dir, &root.prefix) {
        std::fs::remove_file(&tag).expect("remove the tag by other means");
    }
    assert!(
        names_in(&shape.dir).is_empty() && shape.dir.is_dir(),
        "precondition: an empty directory still standing"
    );

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims.len(), 1);
    assert_eq!(
        reclaims[0].verdict,
        ReclaimVerdict::AlreadyEmpty,
        "{reclaims:?}"
    );
    assert_eq!(reclaims[0].refused(), None, "not a refusal: {reclaims:?}");
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert!(
        shape.dir.is_dir(),
        "the reclaim never removes a directory, empty or not"
    );
    assert_eq!(
        reclaims[0].node_id, shape.node_id,
        "the verdict is reported against the candidate it was asked about: {reclaims:?}"
    );
}

/// A sibling directory OUTSIDE the isolated root, removed on drop.
struct OutsideDir {
    dir: PathBuf,
}

impl OutsideDir {
    fn mint() -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .expect("clock")
            .as_nanos();
        let dir = PathBuf::from(format!(
            "/tmp/iceoryx2/orphan_outside_{}_{}",
            std::process::id(),
            nanos
        ));
        std::fs::create_dir_all(&dir).expect("create the outside directory");
        Self { dir }
    }
}

impl Drop for OutsideDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// ANTI-TAUTOLOGY (resolving the node directory by PATH): IDENTITY, not path.
/// The node directory is moved — real tag and all — to a sibling directory
/// OUTSIDE the isolated root, and a symbolic link is planted in its place. A
/// path-driven reclaim would follow the link and delete the tag out there; this
/// one `openat`s the node directory from the registry root by bare name with
/// `O_DIRECTORY|O_NOFOLLOW`, so the link fails `ELOOP` (or `ENOTDIR` on macOS,
/// whose `O_DIRECTORY` check answers first) and the whole candidate is refused
/// and named. The outside tag must survive, the link must be left alone, and
/// putting the directory back is the control that reclaims.
///
/// The directory is planted by hand rather than minted: 0.10 removes a leaked
/// tag on the first sweep, so the library no longer leaves one standing to be
/// swapped.
#[test]
#[serial]
fn a_node_directory_swapped_for_a_symlink_is_refused_and_the_outside_tag_survives() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = plant_orphan_tags(root, &[13]);
    let outside = OutsideDir::mint();
    let moved = outside.dir.join(shape.node_id.to_string());
    std::fs::rename(&shape.dir, &moved).expect("move the node directory outside the root");
    std::os::unix::fs::symlink(&moved, &shape.dir).expect("plant the link in its place");
    let outside_tags = port_tags(&moved, &root.prefix);
    assert_eq!(
        outside_tags.len(),
        1,
        "the real tag travelled with the directory"
    );

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    let refused = reclaims[0]
        .refused()
        .unwrap_or_else(|| panic!("a node directory that is a link must be refused: {reclaims:?}"));
    assert!(
        refused.contains("symbolic link") && refused.contains(&shape.dir.display().to_string()),
        "the refusal must say the directory is a link and name it: {refused}"
    );
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert_eq!(
        port_tags(&moved, &root.prefix),
        outside_tags,
        "the tag outside the root must survive untouched"
    );
    assert!(
        std::fs::symlink_metadata(&shape.dir)
            .map(|m| m.file_type().is_symlink())
            .unwrap_or(false),
        "the link itself is left alone"
    );

    // The control: put the directory back, and the same call heals it.
    std::fs::remove_file(&shape.dir).expect("remove the link");
    std::fs::rename(&moved, &shape.dir).expect("move the node directory back");
    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed, vec![13_u128], "{reclaims:?}");
    assert!(
        names_in(&shape.dir).is_empty(),
        "the tag is gone once the directory is a directory again: {:?}",
        names_in(&shape.dir)
    );
}
