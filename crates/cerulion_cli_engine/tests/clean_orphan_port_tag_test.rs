// SPDX-License-Identifier: AGPL-3.0-only
// `LibcProbe` (the real `kill(2)` liveness oracle) and the self-re-exec
// child both need a Unix process model.
#![cfg(unix)]
//! `cerulion clean`'s orphan port-tag reclaim heals the ONE
//! dead-node shape iceoryx2's sweep can never clear — end to end, over a REAL
//! shape minted by a real process, on an ISOLATED registry root.
//!
//! # The shape
//!
//! A publisher destroyed while one of its loaned samples had been leaked
//! (`mem::forget` of the loan — the rmw destroy path does not do this,
//! but any leaked loan can) deregisters its port but leaves the port's on-disk tag under
//! `<root>/nodes/<node id>/`: the tag is owned by the publisher's shared
//! state, which every forgotten sample keeps alive until the process dies.
//! iceoryx2's dead-node sweep then reclaims the port's resources but never
//! deletes the tag, removes the node's `.details`, and fails the final
//! `rmdir` with `ENOTEMPTY` — on EVERY sweep, forever, so `cerulion clean`
//! never converges and the `.shm_state` reclamation stands down for good
//! (measured: 43 of 43 refusals in one test run were this chain; one
//! machine held eleven such directories behind 13,647 stale mappings).
//!
//! # Shape of the pin
//!
//! Self-re-exec (the `cerulion_core/tests/cdylib_iox2_log_level_test.rs`
//! shape): the PARENT mints a unique iceoryx2 root + prefix (removed by a
//! `Drop` guard), spawns THIS binary as a child on that config, and the child
//! — a plain `cerulion_core` publisher, no rmw — loans a raw sample,
//! `mem::forget`s the loan, DROPS the publisher (port deregistered, tag
//! alive: the exact production shape), `mem::forget`s the transport manager
//! and `process::exit(0)`s. The parent then drives the SAME engine path
//! `cerulion clean` runs, over the isolated config:
//!
//! 1. the diagnostics sweep (`ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config`)
//!    refuses the node with `InternalError` and the four-line chain, and
//!    `orphan_port_tags::orphan_port_tag_candidates` selects exactly that node;
//! 2. `reclaim_orphan_port_tags` removes exactly the one tag;
//! 3. a second sweep reports `cleanups == 1, failed_cleanups == 0` and the
//!    node directory is gone.
//!
//! ANTI-TAUTOLOGY arms, each naming the change that fails it:
//!
//! * a stray file planted in the directory AFTER selection and BEFORE the
//!   reclaim is refused, named, and nothing is removed — the second sweep
//!   still fails, and removing the stray by hand is what lets it converge
//!   (skipping the offender refusal in `orphan_tags_in`);
//! * a node whose process is ALIVE is never a candidate, and a hand-built
//!   candidate carrying a live pid is refused by the death guard with its
//!   tags untouched (treating `CreatorVerdict::Alive` as gone);
//! * `dry_run` lists the tag and removes nothing — the second sweep still
//!   fails until a real reclaim runs;
//! * the node directory swapped for a SYMBOLIC LINK to a sibling directory
//!   outside the root that holds the real tag is refused, the link named, the
//!   outside tag untouched, and the next sweep still refuses the node — the
//!   reclaim binds identity by descriptor, never by path.
//!
//! The child also has a DEFAULT-ROOT mode (`CER_ORPHAN_PORT_TAG_DEFAULT_ROOT=1`),
//! never used by these tests, so an operator can mint exactly ONE such
//! directory on a desk's real registry and watch `cerulion clean --report-only`
//! then `cerulion clean` heal it.
//!
//! # ONE isolated root per test PROCESS, and it IS the process's global config
//!
//! MEASURED: with a per-test root that is NOT
//! the global config, every second sweep reports `cleanups == 1` while the
//! node directory — tag, stray and all — survives. The cause is in
//! `iceoryx2-0.9.1/src/node/mod.rs:605-609`: the first sweep removes the
//! node's `.details` storage before the `rmdir` fails, so the node is listed
//! `Dead` WITHOUT details next time, and `remove_stale_resources_impl` then
//! falls back to `Config::global_config()` — the cleaner acquisition against
//! the wrong root answers `DoesNotExist`, which `blocking_remove_stale_resources`
//! counts as `ResourcesAlreadyCleanedUp == Ok`. A phantom success, on every
//! later sweep, forever. On a real machine the global config IS the node's
//! config, so that fallback is correct there (the CI oracle's node was refused
//! on all 22 later sweeps, as modelled); the phantom is an artefact of sweeping
//! a non-global root twice. So this binary mints ONE root, writes it as an
//! `iceoryx2.toml`, and installs it as the process's global config through
//! `Config::setup_global_config_from_file` before anything touches iceoryx2 —
//! asserted, not assumed. Every arm then keeps its assertions NODE-SCOPED (its
//! own child's pid and directory), so an arm that fails cannot cascade into the
//! next one's counts — which is what keeps a failure attributable to
//! exactly the arm that names it.
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
    cleanup_dead_iceoryx2_nodes_with_diagnostics_with_config, CleanupReport, FailedNodeCleanup,
};
use cerulion_cli_engine::orphan_port_tags::{
    orphan_port_tag_candidates, reclaim_orphan_port_tags, OrphanTagNode, OrphanTagReclaim,
    ReclaimVerdict,
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

/// The refusal a sweep attributed to the node minted by `pid`, if any — the
/// node token renders `pid: <pid>,`.
fn own_failure(report: &CleanupReport, pid: u32) -> Option<&FailedNodeCleanup> {
    let key = format!("pid: {pid},");
    report.failures.iter().find(|f| f.node.contains(&key))
}

/// The candidates a sweep's refusals yield for the node minted by `pid`.
fn own_candidates(report: &CleanupReport, root: &IsolatedRoot, pid: u32) -> Vec<OrphanTagNode> {
    orphan_port_tag_candidates(&report.failures, &root.config())
        .into_iter()
        .filter(|c| c.pid == pid)
        .collect()
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

/// Mint the shape with an exit-mode child and run the FIRST sweep, asserting
/// the preconditions every arm shares — all NODE-SCOPED to the child's pid
/// and directory: exactly one node directory appeared for the child and it
/// carries a `<prefix>*.port_tag` (the child's port outlived it), the sweep
/// refuses THAT node with `InternalError`, the selector picks exactly it, and
/// after the sweep the directory holds NOTHING but port tags (the `.details`
/// storage is gone — the `rmdir` was the only thing that failed).
struct MintedShape {
    child: ChildRun,
    node_dir: PathBuf,
    first: CleanupReport,
    candidate: OrphanTagNode,
}

fn mint_and_first_sweep(root: &IsolatedRoot, arm: &str) -> MintedShape {
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

    let first = sweep(root);
    let refusal = own_failure(&first, child.pid).unwrap_or_else(|| {
        panic!(
            "the first sweep must refuse the child's node (pid {}): {}",
            child.pid,
            render_report(&first)
        )
    });
    assert_eq!(
        refusal.variant,
        "InternalError",
        "{}",
        render_report(&first)
    );
    let candidates = own_candidates(&first, root, child.pid);
    assert_eq!(
        candidates.len(),
        1,
        "the refusal must classify as the orphan-tag chain (exactly one candidate for pid {}): {}",
        child.pid,
        render_report(&first)
    );
    let candidate = candidates[0].clone();
    assert_eq!(
        candidate.dir.components().collect::<Vec<_>>(),
        node_dir.components().collect::<Vec<_>>(),
        "the candidate's directory is the node's directory"
    );
    assert_eq!(
        node_dir.file_name().and_then(|n| n.to_str()),
        Some(candidate.node_id.to_string().as_str()),
        "the directory is named after the node id"
    );
    // After the sweep the directory holds ONLY port tags: everything else
    // was removed on the way to the failing `rmdir`.
    let names = names_in(&node_dir);
    assert!(
        !names.is_empty()
            && names
                .iter()
                .all(|n| n.starts_with(&root.prefix) && n.ends_with(".port_tag")),
        "after the first sweep the directory must hold nothing but port tags: {names:?}"
    );
    assert_eq!(
        liveness(child.pid),
        CreatorVerdict::Gone,
        "the exit-mode child is dead by now"
    );

    MintedShape {
        child,
        node_dir,
        first,
        candidate,
    }
}

// =====================================================================
// THE ARMS
// =====================================================================

/// THE HEADLINE: mint → first sweep refuses → reclaim removes exactly the
/// one tag → the next sweep no longer refuses the node and its directory is
/// gone.
#[test]
#[serial]
fn the_orphan_port_tag_shape_is_reclaimed_and_the_next_sweep_converges() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_and_first_sweep(root, "headline");
    // The minting child is dead before anything is reclaimed — the death
    // guard's premise, re-proved on the child's own pid.
    assert_eq!(
        liveness(shape.child.pid),
        CreatorVerdict::Gone,
        "the minting child (pid {}) must be dead before the reclaim;\n--- child stderr ---\n{}",
        shape.child.pid,
        shape.child.stderr
    );

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    assert_eq!(reclaims.len(), 1);
    assert_eq!(
        reclaims[0].refused(),
        None,
        "the reclaim must not refuse a directory holding only orphan tags: {reclaims:?}"
    );
    assert_eq!(
        reclaims[0].removed.len(),
        1,
        "one publisher, one port, one tag removed: {reclaims:?}"
    );
    assert!(
        port_tags(&shape.node_dir, &root.prefix).is_empty(),
        "the tag is gone from disk: {:?}",
        names_in(&shape.node_dir)
    );
    assert!(
        shape.node_dir.is_dir(),
        "the reclaim leaves the directory for the sweep's `remove_node`"
    );

    let second = sweep(root);
    assert!(
        own_failure(&second, shape.child.pid).is_none(),
        "the second sweep must not refuse the node any more: {}\n--- child stderr ---\n{}",
        render_report(&second),
        shape.child.stderr
    );
    assert!(second.cleanups >= 1, "{}", render_report(&second));
    assert!(
        !shape.node_dir.exists(),
        "the node directory must be gone after the second sweep"
    );
    assert!(
        shape.first.failed_cleanups >= 1,
        "(first sweep, for the record)"
    );
}

/// ANTI-TAUTOLOGY (skipping the offender refusal in `orphan_tags_in`):
/// a stray entry planted AFTER selection and BEFORE the reclaim — the window
/// a concurrent session can write into — refuses the whole directory, names
/// the stray, removes nothing; the next sweep still refuses the node; and
/// removing the stray by hand is exactly what lets the reclaim + sweep
/// converge.
#[test]
#[serial]
fn a_stray_entry_planted_before_the_reclaim_is_refused_named_and_nothing_is_removed() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_and_first_sweep(root, "stray");
    let stray = shape.node_dir.join("stray.txt");
    std::fs::write(&stray, b"not a tag").expect("plant the stray");
    let before = names_in(&shape.node_dir);

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), false);
    let refused = reclaims[0]
        .refused()
        .unwrap_or_else(|| panic!("a directory holding a stray must be refused: {reclaims:?}"));
    assert!(
        refused.contains("`stray.txt`"),
        "the refusal must name the stray: {refused}"
    );
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert_eq!(names_in(&shape.node_dir), before, "nothing removed");

    let second = sweep(root);
    assert!(
        own_failure(&second, shape.child.pid).is_some(),
        "with the stray in place the node is still unreclaimable: {}",
        render_report(&second)
    );
    assert!(
        shape.node_dir.is_dir(),
        "the directory survives with the stray in it"
    );

    // The control: the refusal was the ONLY obstacle.
    std::fs::remove_file(&stray).expect("remove the stray by hand");
    let candidates = own_candidates(&second, root, shape.child.pid);
    assert_eq!(candidates.len(), 1, "{}", render_report(&second));
    let reclaims = reclaim(root, &candidates, false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed.len(), 1);
    let third = sweep(root);
    assert!(
        own_failure(&third, shape.child.pid).is_none(),
        "{}",
        render_report(&third)
    );
    assert!(!shape.node_dir.exists());
}

/// ANTI-TAUTOLOGY (treating `CreatorVerdict::Alive` as gone): a node
/// whose process is ALIVE is never a candidate — the sweep does not even
/// count it as dead — and a hand-built candidate that points the reclaimer at
/// a real orphan directory but carries a LIVE pid is refused by the death
/// guard with the tags untouched. The lingering child mints the SAME shape
/// (it too drops its publisher with a leaked loan), so once it dies it is
/// refused by the sweep, selected, reclaimed and converged like any other —
/// the second minting path through the same pin.
#[test]
#[serial]
fn a_live_process_is_never_a_candidate_and_its_pid_refuses_the_reclaim() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    // A live node on the root: its process lingers until this test drops
    // the handle.
    let before = node_dirs(root);
    let linger = LingerChild::spawn(root, "linger");
    let linger_dir = new_node_dir(root, &before, "the linger child");
    assert_eq!(
        liveness(linger.pid),
        CreatorVerdict::Alive,
        "the linger child is alive"
    );
    // And a real orphan directory beside it, minted by a dead child.
    let before = node_dirs(root);
    let dead = run_exit_child(root, "live-arm-orphan");
    let dead_dir = new_node_dir(root, &before, "the dead child");

    let first = sweep(root);
    assert!(
        own_failure(&first, linger.pid).is_none(),
        "the LIVE node is not even attempted, let alone refused: {}",
        render_report(&first)
    );
    assert!(
        own_failure(&first, dead.pid).is_some(),
        "the DEAD node is refused: {}",
        render_report(&first)
    );
    assert!(
        own_candidates(&first, root, linger.pid).is_empty(),
        "the live child's node is never a candidate"
    );
    let candidates = own_candidates(&first, root, dead.pid);
    assert_eq!(candidates.len(), 1, "{}", render_report(&first));
    let orphan = candidates[0].clone();
    assert!(orphan.dir.components().eq(dead_dir.components()));
    let tags_before = port_tags(&orphan.dir, &root.prefix);
    assert_eq!(tags_before.len(), 1);
    assert!(linger_dir.is_dir(), "the live node's directory stands");

    // THE DEATH GUARD: the same directory, a live pid.
    let live_pid_candidate = OrphanTagNode {
        pid: linger.pid,
        ..orphan.clone()
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
        port_tags(&orphan.dir, &root.prefix),
        tags_before,
        "the tags must survive a refused reclaim"
    );

    // The control: the genuine candidate (its pid really is gone) reclaims
    // and its node converges while the live node still stands.
    let reclaims = reclaim(root, std::slice::from_ref(&orphan), false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed.len(), 1);
    let second = sweep(root);
    assert!(
        own_failure(&second, dead.pid).is_none(),
        "{}",
        render_report(&second)
    );
    assert!(!orphan.dir.exists(), "the orphan's directory is gone");
    assert!(
        linger_dir.is_dir(),
        "the live node's directory still stands"
    );
    assert!(own_failure(&second, linger.pid).is_none());

    // The linger child dies: its node is now the same shape, and the same
    // path heals it.
    let linger_pid = linger.pid;
    drop(linger);
    let third = sweep(root);
    let linger_candidates = own_candidates(&third, root, linger_pid);
    assert_eq!(
        linger_candidates.len(),
        1,
        "once dead, the linger child's node is refused with the exact chain: {}",
        render_report(&third)
    );
    let reclaims = reclaim(root, &linger_candidates, false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed.len(), 1);
    let fourth = sweep(root);
    assert!(
        own_failure(&fourth, linger_pid).is_none(),
        "{}",
        render_report(&fourth)
    );
    assert!(!linger_dir.exists(), "the linger node's directory is gone");
}

/// `dry_run` lists the tag and removes nothing: the next sweep still refuses
/// the node until a real reclaim runs.
#[test]
#[serial]
fn a_dry_run_lists_the_tag_and_removes_nothing() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_and_first_sweep(root, "dry-run");
    let before = names_in(&shape.node_dir);

    let reclaims = reclaim(root, std::slice::from_ref(&shape.candidate), true);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed.len(), 1, "listed: {reclaims:?}");
    assert_eq!(
        names_in(&shape.node_dir),
        before,
        "nothing removed under dry_run"
    );

    let second = sweep(root);
    assert!(
        own_failure(&second, shape.child.pid).is_some(),
        "a dry run heals nothing: {}",
        render_report(&second)
    );
    assert!(shape.node_dir.is_dir());

    // The control: the real reclaim does.
    let candidates = own_candidates(&second, root, shape.child.pid);
    assert_eq!(candidates.len(), 1, "{}", render_report(&second));
    let reclaims = reclaim(root, &candidates, false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    let third = sweep(root);
    assert!(
        own_failure(&third, shape.child.pid).is_none(),
        "{}",
        render_report(&third)
    );
    assert!(!shape.node_dir.exists());
}

/// An ALREADY-EMPTY candidate directory — the tag came off by other means
/// between the sweep that selected it and the reclaim (a concurrent session's
/// reclaim, or an earlier `cerulion clean` interrupted after its unlinks and
/// before its second sweep) — is `AlreadyEmpty`: not a refusal, nothing
/// removed, and the NEXT sweep converges it, directory gone. This is the
/// verdict that makes the verb's second sweep run whenever candidates
/// EXISTED: a sweep gated on "a tag came off" would leave this node standing
/// for one more run.
#[test]
#[serial]
fn an_already_empty_candidate_is_converged_pending_sweep_and_the_next_sweep_removes_it() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_and_first_sweep(root, "already-empty");
    for tag in port_tags(&shape.node_dir, &root.prefix) {
        std::fs::remove_file(&tag).expect("remove the tag by other means");
    }
    assert!(
        names_in(&shape.node_dir).is_empty() && shape.node_dir.is_dir(),
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
    assert!(shape.node_dir.is_dir(), "left for the sweep");

    let second = sweep(root);
    assert!(
        own_failure(&second, shape.child.pid).is_none(),
        "the next sweep converges an already-empty directory: {}",
        render_report(&second)
    );
    assert!(
        !shape.node_dir.exists(),
        "the node directory must be gone after the sweep"
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

/// IDENTITY, not path, over the REAL shape: after the child minted it and
/// the first sweep refused it, the node directory is MOVED — real tag and
/// all — to a sibling directory OUTSIDE the isolated root and a symbolic
/// link is planted in its place. A path-driven reclaim would follow the
/// link and delete the tag out there. The reclaim must refuse naming the
/// link, the outside tag must survive, the link must be left alone, and the
/// next sweep must still refuse the node. Putting the directory back is the
/// control that converges.
#[test]
#[serial]
fn a_node_directory_swapped_for_a_symlink_is_refused_and_the_outside_tag_survives() {
    let _use = RootUse::acquire();
    let root = IsolatedRoot::get();
    let shape = mint_and_first_sweep(root, "symlink-swap");
    let outside = OutsideDir::mint();
    let moved = outside.dir.join(
        shape
            .node_dir
            .file_name()
            .expect("the node directory has a name"),
    );
    std::fs::rename(&shape.node_dir, &moved).expect("move the node directory outside the root");
    std::os::unix::fs::symlink(&moved, &shape.node_dir).expect("plant the link in its place");
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
        refused.contains("symbolic link")
            && refused.contains(&shape.node_dir.display().to_string()),
        "the refusal must say the directory is a link and name it: {refused}"
    );
    assert!(reclaims[0].removed.is_empty(), "{reclaims:?}");
    assert_eq!(
        port_tags(&moved, &root.prefix),
        outside_tags,
        "the tag outside the root must survive untouched"
    );
    assert!(
        std::fs::symlink_metadata(&shape.node_dir)
            .map(|m| m.file_type().is_symlink())
            .unwrap_or(false),
        "the link itself is left alone"
    );

    let second = sweep(root);
    assert!(
        own_failure(&second, shape.child.pid).is_some(),
        "with a link in the directory's place the node is still refused: {}",
        render_report(&second)
    );

    // The control: put the directory back, and the same path heals it.
    std::fs::remove_file(&shape.node_dir).expect("remove the link");
    std::fs::rename(&moved, &shape.node_dir).expect("move the node directory back");
    let third = sweep(root);
    let candidates = own_candidates(&third, root, shape.child.pid);
    assert_eq!(candidates.len(), 1, "{}", render_report(&third));
    let reclaims = reclaim(root, &candidates, false);
    assert_eq!(reclaims[0].refused(), None, "{reclaims:?}");
    assert_eq!(reclaims[0].removed.len(), 1);
    let fourth = sweep(root);
    assert!(
        own_failure(&fourth, shape.child.pid).is_none(),
        "{}",
        render_report(&fourth)
    );
    assert!(!shape.node_dir.exists());
}
