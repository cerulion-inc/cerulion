// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for the permissive network egress
//! over the REAL `cerulion` binary.
//!
//! **The desk default:** a permissive `graph run`'s egress goes through
//! the machine's ONE `cerulion-netd` session BY DEFAULT (no per-run gateway child);
//! the per-run gateway CHILD is the STRICT + netd-unreachable fallback. So the arms:
//!
//! (a) **Default path → netd, NO child** — a single-process real-clock `graph run`
//!     with NO `network:` block, pointed at an IN-PROCESS `cerulion-netd` (a counting
//!     spy egress plane), routes its egress THROUGH netd: it fires the netd-register
//!     breadcrumb + the `PERMISSIVE_EGRESS_NOTICE`, spawns NO `run-gateway` child, and
//!     on a clean SIGINT/SIGTERM the connection-close RELEASES the egress registration
//!     (the crash-safe refcount — the spy observes the release) + exits 0.
//!
//! (b) **netd unreachable → per-run gateway child (the fallback)** — the SAME run with
//!     netd forced UNREACHABLE (`CERULION_NETD_BIN` at a nonexistent path + a
//!     nonexistent socket) degrades LOUDLY to a per-run gateway child: the degrade warn
//!     fires, a `run-gateway` child exists during the run, and a clean SIGINT/SIGTERM
//!     reaps it with no orphan + the gateway's own graceful-shutdown line.
//!
//! (control) **`CERULION_NETWORK=off`** suppresses BOTH — no breadcrumb, no netd
//!     register, no gateway child (the anti-tautology control).
//!
//! (c) **Record keeps the network** — `graph run --record` (single-process, permissive,
//!     netd unreachable → child) records a bag AND keeps the gateway up during the run.
//!     Its refusal TWIN: an explicit `network:` block that declares `ingress:` +
//!     `--record` exits nonzero naming BOTH workarounds, graph file
//!     byte-untouched.
//!
//! The STRICT multi-process delivery arm (Strict ⇒ per-run gateway child, a
//! PERMANENT routing rule) lives in its sibling `network_gateway_mp_e2e_test.rs`.
//!
//! Harness cribs `mp_record_e2e_test.rs` / `graph_record_e2e_test.rs` (tempdir
//! workspace, PREBUILT fixture cdylibs, redirected child logs, bounded waits,
//! `ChildGuard` group teardown + reap, `#[serial]`, unique prefixes). Every arm sets a UNIQUE
//! `CERULION_NETD_SOCK` (isolation) + `CERULION_NETD_BIN` at a nonexistent path so a
//! test NEVER spawns a real netd — the default-path arm connects to the test's OWN
//! in-process daemon; the fallback arm finds nothing. The gateway child's listen port
//! is pinned to a probed free port via `CERULION_GATEWAY_PORT` so it never collides on
//! 7683.
//!
//! Prerequisites (the repo's fixture pattern — the helpers PANIC with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]` (NOT linux-only): the netd daemon + gateway spawn + pgrep/SIGINT
//! teardown are Unix; macOS is a supported local real-run platform.

#![cfg(unix)]

use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serial_test::serial;

// Shared record-harness helpers (ChildGuard group teardown, bounded waits,
// send_signal, read_file, the prebuilt-fixture resolvers). `#![allow(dead_code)]`
// in the module covers the helpers this binary does not use.
mod mp_support;
use mp_support::{
    dylib_file, fixture_cdylib, read_file, send_signal, wait_for_bag_state, ChildGuard,
    RECORDED_WINDOW_TIMEOUT,
};

use cerulion_cli_engine::graph_cmd::PERMISSIVE_EGRESS_NOTICE;
use cerulion_netd::daemon::{self, NetdConfig, RunningNetd};
use cerulion_netd::egress::{EgressError, EgressId, EgressPlane};
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::registry::TopicKey;

use cerulion_core::{GatewayPlan, SchemaServing};

/// The netd-register breadcrumb the run logs when its egress converges onto the
/// shared cerulion-netd session (the DEFAULT desk path — arm a).
const NETD_REGISTER_LINE: &str =
    "pushed this desk graph's egress plan into the shared cerulion-netd daemon";

/// The loud degrade warn when netd is unreachable and the run falls back to a per-run
/// gateway child (arm b).
const NETD_DEGRADE_LINE: &str = "FALLING BACK to a per-run gateway child";

// ── An in-process cerulion-netd with a counting spy egress plane ──────────
// A DI double (Principle #13, not fake data): the spy egress plane ACCEPTS every
// register (counting it) and counts every release. The default-path arm points the
// real `cerulion graph run` subprocess at this in-process daemon over UDS, so the
// register + the connection-close release are OBSERVABLE from the test process.

#[derive(Default)]
struct SpyEgress {
    registers: AtomicUsize,
    releases: AtomicUsize,
}
impl EgressPlane for SpyEgress {
    fn register_egress(
        &self,
        _id: EgressId,
        _plan: &GatewayPlan,
        _serving: &SchemaServing,
        _ix_config_json: Option<&str>,
    ) -> Result<bool, EgressError> {
        // First register "boots" the (spied) gateway.
        let first = self.registers.fetch_add(1, Ordering::SeqCst) == 0;
        Ok(first)
    }
    fn release_egress(&self, _id: EgressId) {
        self.releases.fetch_add(1, Ordering::SeqCst);
    }
}

/// An unused mirror plane — the daemon requires one, but this egress-only harness
/// never demands ingress (ensuring a mirror errs; releasing is a no-op).
struct UnusedMirror;
impl MirrorPlane for UnusedMirror {
    fn ensure_mirror(&self, key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        Err(MirrorError::Register {
            key: key.clone(),
            source: Box::new(cerulion_core::TransportError::Internal {
                reason: "the network-gateway e2e never demands ingress".to_string(),
            }),
        })
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        MirrorRelease::Retired
    }
}

/// A unique, SHORT control-socket path (under the system temp dir — a long tempdir
/// path would blow the sockaddr_un limit). Returns the dir (held for cleanup) + the
/// socket path.
fn unique_netd_socket(tag: &str) -> (PathBuf, PathBuf) {
    static N: AtomicUsize = AtomicUsize::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_gwnd_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk netd sock dir");
    let sock = dir.join("netd.sock");
    (dir, sock)
}

/// Start an in-process `cerulion-netd` (spy egress + unused mirror) bound to `sock`.
fn start_spy_netd(sock: &Path) -> (RunningNetd, Arc<SpyEgress>) {
    let egress = Arc::new(SpyEgress::default());
    let mirror: Arc<dyn MirrorPlane> = Arc::new(UnusedMirror);
    let egress_dyn: Arc<dyn EgressPlane> = Arc::clone(&egress) as _;
    let netd = daemon::start_with_egress(
        sock.to_path_buf(),
        mirror,
        egress_dyn,
        NetdConfig {
            idle_grace: Duration::from_secs(3600),
            idle_watch_poll: Duration::from_millis(25),
            ..NetdConfig::default()
        },
    )
    .expect("start in-process spy netd");
    (netd, egress)
}

/// A nonexistent `cerulion-netd` binary path — set as `CERULION_NETD_BIN` on EVERY arm
/// so a test never spawns a REAL netd (the default arm connects to its own in-process
/// daemon; the fallback arm finds nothing → child).
const NONEXISTENT_NETD_BIN: &str = "/nonexistent/cerulion-netd-c6b-e2e";

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";
/// The data-trigger fixture (input `trigger_in`, output `cmd`) — the
/// refusal fixture's ingress-consuming `sink` node type.
const DATA_FIXTURE: &str = "test_node_macro_data_trigger_cdylib";

/// GRACEFUL-FORWARD discriminator. The gateway's `run-gateway` drive
/// loop logs this `info!` (graph_cmd.rs) ONLY after its own `running` flag
/// flips and the loop drains — reached EXCLUSIVELY when the graph process
/// forwarded it a graceful SIGINT and it shut itself down. A Drop-SIGKILL-only
/// teardown (the backstop) kills it mid-loop and
/// this line is NEVER printed. Asserting it separates the graceful forward from
/// the SIGKILL backstop that the exit-0 + no-orphan asserts alone cannot.
const GATEWAY_GRACEFUL_SHUTDOWN_LINE: &str = "network gateway shutting down";

/// A probed-free ephemeral TCP port (drops the listener so the port is free
/// again — the gateway then binds it). Pinning `CERULION_GATEWAY_PORT` here
/// keeps two concurrent test binaries off the shared well-known 7683.
fn probe_ephemeral_port() -> u16 {
    let l = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = l.local_addr().expect("probe local_addr").port();
    drop(l);
    port
}

/// Hand-build a MINIMAL single-process workspace in `root`: a `[workspace]`
/// Cargo.toml + one `ticker` (period 50ms) node producing `cmd`
/// (`geometry_msgs/Vector3`) + the prebuilt period fixture cdylib copied to
/// `target/debug/libticker.*`. `network_block` is appended verbatim (empty =
/// no block = permissive default). Returns the written graph YAML.
fn build_single_producer_workspace(
    root: &Path,
    prefix: &str,
    graph_name: &str,
    network_block: &str,
) -> String {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    // Source copy (the metadata / staleness walkers) + the prebuilt cdylib.
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    std::fs::copy(
        fixtures.join(PERIOD_FIXTURE).join("src/lib.rs"),
        root.join("nodes/ticker/src/lib.rs"),
    )
    .expect("copy period fixture src");
    std::fs::copy(
        fixture_cdylib(PERIOD_FIXTURE),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy period fixture cdylib");
    let yaml = format!(
        "name: {graph_name}\n\
         prefix: {prefix}\n\
         nodes:\n\
         - id: ticker\n\
         \x20 type: ticker\n\
         \x20 inputs: []\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         {network_block}"
    );
    std::fs::write(root.join(format!("graphs/{graph_name}.yaml")), &yaml).unwrap();
    yaml
}

/// The `run-gateway` child of `parent_pid` (the `graph run` process spawns it
/// directly), if present. `pgrep -P <ppid> -f run-gateway` — the `run-gateway`
/// subcommand token is distinctive, so no bracket self-match dance is needed
/// (the test binary's own cmdline never contains it).
fn gateway_child_pid(parent_pid: u32) -> Option<u32> {
    let out = Command::new("pgrep")
        .args(["-P", &parent_pid.to_string(), "-f", "run-gateway"])
        .output()
        .ok()?;
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|l| l.trim().parse::<u32>().ok())
        .next()
}

/// Bounded poll until `parent_pid` has a `run-gateway` child, else `None`.
fn wait_for_gateway_child(parent_pid: u32, timeout: Duration) -> Option<u32> {
    let start = Instant::now();
    loop {
        if let Some(pid) = gateway_child_pid(parent_pid) {
            return Some(pid);
        }
        if start.elapsed() > timeout {
            return None;
        }
        std::thread::sleep(Duration::from_millis(150));
    }
}

/// `kill(pid, 0)` liveness probe (ESRCH ⇒ dead/reaped).
fn pid_alive(pid: u32) -> bool {
    // SAFETY: kill(2) with signal 0 only checks existence; no memory touched.
    unsafe { libc::kill(pid as libc::pid_t, 0) == 0 }
}

/// Bounded poll until `pid` is gone (reaped), else `false`.
fn wait_until_dead(pid: u32, timeout: Duration) -> bool {
    let start = Instant::now();
    while pid_alive(pid) {
        if start.elapsed() > timeout {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    true
}

/// Bounded poll for `needle` across BOTH captured streams (the CLI's tracing
/// writer choice must not decide the sync); panics at the deadline (the
/// caller's `ChildGuard` reaps on the panic unwind).
fn wait_for_merged_line(stdout_path: &Path, stderr_path: &Path, needle: &str, timeout: Duration) {
    let start = Instant::now();
    loop {
        let content = merged(stdout_path, stderr_path);
        if content.contains(needle) {
            return;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; combined log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(80));
    }
}

/// Spawn `cerulion graph run <graph> --single-process [extra...]` in `root`.
/// `network_off` sets the `CERULION_NETWORK=off` kill-switch; a probed
/// `CERULION_GATEWAY_PORT` keeps a fallback gateway child off the well-known 7683.
///
/// `netd_sock` is the `CERULION_NETD_SOCK` the run's egress uses — point it at the
/// test's IN-PROCESS spy daemon (the default netd path) OR at a NONEXISTENT path (the
/// netd-unreachable → child fallback). `CERULION_NETD_BIN` is ALWAYS a nonexistent path
/// so a test never spawns a real netd.
fn spawn_graph_run(
    root: &Path,
    graph: &str,
    network_off: bool,
    gateway_port: u16,
    netd_sock: &Path,
    extra: &[&str],
) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join(format!("{graph}.stdout"));
    let stderr_path = root.join(format!("{graph}.stderr"));
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", graph, "--single-process"])
        .args(extra)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        // Plain output so a message-substring match never trips on ANSI codes.
        .env("NO_COLOR", "1")
        // Pin the fallback gateway child's listen port off the shared 7683.
        .env("CERULION_GATEWAY_PORT", gateway_port.to_string())
        // The egress netd socket + a nonexistent bin (never spawn a real netd).
        .env("CERULION_NETD_SOCK", netd_sock)
        .env("CERULION_NETD_BIN", NONEXISTENT_NETD_BIN)
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    if network_off {
        cmd.env("CERULION_NETWORK", "off");
    }
    // The gateway runs as a CHILD of this process, so teardown must reach the
    // group — an orphaned gateway holds the listen port for the next arm.
    let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
    (guard, stdout_path, stderr_path)
}

/// Bounded poll until `f()` returns true, else false at the deadline.
fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let start = Instant::now();
    while !f() {
        if start.elapsed() > timeout {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    true
}

/// Merge both captured streams (tracing's writer choice must not decide a pin).
fn merged(stdout_path: &Path, stderr_path: &Path) -> String {
    format!("{}\n{}", read_file(stdout_path), read_file(stderr_path))
}

// ── (a) DEFAULT path: egress converges onto netd, NO gateway child ─────────

/// DEFAULT: a permissive run pointed at a reachable (in-process) `cerulion-netd`
/// routes its egress THROUGH netd — it fires the netd-register breadcrumb + the
/// permissive notice, spawns NO `run-gateway` child, and on `signal` the
/// connection-close RELEASES the egress registration (the spy observes it) + exits 0.
fn netd_default_arm(tag: &str, signal: libc::c_int) {
    let tmp = tempfile::tempdir().unwrap();
    build_single_producer_workspace(tmp.path(), tag, tag, "");
    let port = probe_ephemeral_port();
    // The test's OWN in-process netd (spy egress). Held for the whole run.
    let (sock_dir, sock) = unique_netd_socket(tag);
    let (netd, spy) = start_spy_netd(&sock);

    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), tag, false, port, &sock, &[]);

    wait_for_merged_line(
        &stdout_path,
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );

    let run_pid = guard.id();
    // The egress registered with netd (the spy observed the register) — the
    // structural "went through netd" oracle.
    assert!(
        wait_until(
            || spy.registers.load(Ordering::SeqCst) >= 1,
            Duration::from_secs(15)
        ),
        "the run's egress must register with the shared cerulion-netd (spy saw no register)\n\
         stdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    // NO per-run gateway child — the whole point of the desk default.
    assert!(
        wait_for_gateway_child(run_pid, Duration::from_secs(3)).is_none(),
        "the netd default path must spawn NO `run-gateway` child (egress goes through netd)"
    );
    let out = merged(&stdout_path, &stderr_path);
    assert!(
        out.contains(NETD_REGISTER_LINE),
        "the netd-register breadcrumb must fire on the default path\ncombined:\n{out}"
    );
    // The permissive notice still fires EXACTLY ONCE (parity with the child path).
    assert_eq!(
        out.matches(PERMISSIVE_EGRESS_NOTICE).count(),
        1,
        "the permissive breadcrumb must fire EXACTLY ONCE on the netd path\ncombined:\n{out}"
    );

    // Signal → exit 0 AND the connection-close RELEASES the registration (the
    // netd-release semantics — the guard's Drop closes the UDS, netd sees EOF).
    send_signal(run_pid, signal);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit after signal");
    assert!(
        status.success(),
        "a clean signal must exit 0, got {status:?}"
    );
    assert!(
        wait_until(
            || spy.releases.load(Ordering::SeqCst) >= 1,
            Duration::from_secs(15)
        ),
        "closing the run (connection-close) must RELEASE the netd egress registration (spy saw no \
         release) — the crash-safe refcount"
    );
    // No gateway child was ever spawned, so there is nothing to reap.
    assert!(
        wait_for_gateway_child(run_pid, Duration::from_secs(1)).is_none(),
        "still no gateway child after exit"
    );

    drop(netd);
    let _ = std::fs::remove_dir_all(&sock_dir);
}

/// DEFAULT path + a clean SIGINT: netd register, no child, connection-close release.
#[test]
#[serial]
fn permissive_default_routes_through_netd_no_child_release_on_sigint() {
    netd_default_arm("gwndint", libc::SIGINT);
}

/// DEFAULT path + a directed SIGTERM (systemd stop / `kill <pid>`): same contract —
/// netd register, no child, and the connection-close release on the graceful exit.
#[test]
#[serial]
fn permissive_default_routes_through_netd_no_child_release_on_sigterm() {
    netd_default_arm("gwndterm", libc::SIGTERM);
}

// ── (b) FALLBACK: netd unreachable → per-run gateway child, reaped ─────────

/// FALLBACK: with netd forced UNREACHABLE (a nonexistent socket + a nonexistent
/// bin — set by `spawn_graph_run`), a permissive run degrades LOUDLY to a per-run
/// gateway child: the degrade warn fires, a `run-gateway` child exists during the run,
/// and `signal` reaps it with no orphan + the gateway's own graceful-shutdown line.
fn netd_unreachable_fallback_arm(tag: &str, signal: libc::c_int) {
    let tmp = tempfile::tempdir().unwrap();
    build_single_producer_workspace(tmp.path(), tag, tag, "");
    let port = probe_ephemeral_port();
    // A NONEXISTENT netd socket → the run's netd client cannot connect (and the
    // nonexistent bin means it cannot spawn one) → degrade to the child.
    let missing_sock = std::env::temp_dir().join(format!(
        "cer_gwnd_missing_{tag}_{}.sock",
        std::process::id()
    ));
    let _ = std::fs::remove_file(&missing_sock);
    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), tag, false, port, &missing_sock, &[]);

    wait_for_merged_line(
        &stdout_path,
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );

    let run_pid = guard.id();
    let gw_pid = wait_for_gateway_child(run_pid, Duration::from_secs(15)).unwrap_or_else(|| {
        panic!(
            "netd-unreachable run must FALL BACK to a `run-gateway` child\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });

    let out = merged(&stdout_path, &stderr_path);
    // The LOUD degrade warn fired (never a silent fallback).
    assert!(
        out.contains(NETD_DEGRADE_LINE),
        "the netd-unreachable run must degrade LOUDLY to the child\ncombined:\n{out}"
    );
    // The permissive breadcrumb fires EXACTLY ONCE (the child path's parity).
    assert_eq!(
        out.matches(PERMISSIVE_EGRESS_NOTICE).count(),
        1,
        "the permissive breadcrumb must fire EXACTLY ONCE on the fallback path\ncombined:\n{out}"
    );

    // Signal (to the graph process ONLY — `send_signal` is `kill(pid)`, not the
    // group) → the graph process FORWARDS a graceful SIGINT to the gateway, which
    // reaps with no orphan.
    send_signal(run_pid, signal);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit after signal");
    assert!(
        status.success(),
        "a clean signal must exit 0, got {status:?}"
    );
    assert!(
        wait_until_dead(gw_pid, Duration::from_secs(15)),
        "the fallback gateway child {gw_pid} must be reaped after the run exits (no orphan)"
    );
    // Graceful-forward discriminator: the gateway drained its OWN loop on the forwarded graceful
    // SIGINT (reaching its shutdown `info!`) — a GRACEFUL forward, NOT the Drop SIGKILL
    // backstop (which exit-0 + reap alone would also satisfy).
    let out = merged(&stdout_path, &stderr_path);
    assert!(
        out.contains(GATEWAY_GRACEFUL_SHUTDOWN_LINE),
        "the fallback gateway must shut down GRACEFULLY on the forwarded signal, not via \
         the Drop SIGKILL backstop; expected {GATEWAY_GRACEFUL_SHUTDOWN_LINE:?}\ncombined:\n{out}"
    );
}

/// FALLBACK + SIGINT: netd unreachable → child spawned + graceful-reaped.
#[test]
#[serial]
fn netd_unreachable_falls_back_to_reaped_gateway_child_on_sigint() {
    netd_unreachable_fallback_arm("gwfbint", libc::SIGINT);
}

/// FALLBACK + directed SIGTERM (the directed-kill gap): netd unreachable →
/// child spawned + graceful-forward-reaped on a SIGTERM to the graph process ONLY.
#[test]
#[serial]
fn netd_unreachable_falls_back_to_reaped_gateway_child_on_sigterm() {
    netd_unreachable_fallback_arm("gwfbterm", libc::SIGTERM);
}

// ── (control) CERULION_NETWORK=off suppresses BOTH netd and the child ──────

/// Anti-tautology control: the SAME run under `CERULION_NETWORK=off` fires NO
/// breadcrumb, registers with NO netd, and spawns NO gateway (the kill-switch is
/// genuinely applied). A reachable in-process netd is running, yet the spy sees ZERO
/// registers — the kill-switch suppresses the netd path too.
#[test]
#[serial]
fn network_off_suppresses_breadcrumb_netd_and_gateway() {
    let tmp = tempfile::tempdir().unwrap();
    build_single_producer_workspace(tmp.path(), "gwaoff", "gwaoff", "");
    let port = probe_ephemeral_port();
    let (sock_dir, sock) = unique_netd_socket("gwaoff");
    let (netd, spy) = start_spy_netd(&sock);
    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), "gwaoff", true, port, &sock, &[]);

    // Sync on readiness (NOT the breadcrumb — it must never appear).
    wait_for_merged_line(
        &stdout_path,
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );
    let run_pid = guard.id();

    // No gateway child within a generous window (it is never spawned).
    assert!(
        wait_for_gateway_child(run_pid, Duration::from_secs(3)).is_none(),
        "CERULION_NETWORK=off must spawn NO gateway child"
    );
    // And no netd register — the kill-switch suppresses the netd path (the spy,
    // reachable, saw nothing).
    assert_eq!(
        spy.registers.load(Ordering::SeqCst),
        0,
        "CERULION_NETWORK=off must register with NO netd (the spy saw a register)"
    );
    let out = merged(&stdout_path, &stderr_path);
    assert!(
        !out.contains(PERMISSIVE_EGRESS_NOTICE),
        "CERULION_NETWORK=off must fire NO permissive breadcrumb\ncombined:\n{out}"
    );
    assert!(
        !out.contains(NETD_REGISTER_LINE),
        "CERULION_NETWORK=off must fire NO netd-register breadcrumb\ncombined:\n{out}"
    );

    send_signal(run_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("run did not exit after SIGINT");
    assert!(status.success(), "clean SIGINT must exit 0, got {status:?}");
    drop(netd);
    let _ = std::fs::remove_dir_all(&sock_dir);
}

// ── (c) record keeps the network + the refusal twin ───────────────

/// A `bagd` grandchild leak guard for THIS file's record run (graph `gwrec`).
/// bagd runs in its own process group, so a mid-run panic that SIGKILLs the
/// supervisor would orphan it — pgrep-discover any bagd this file spawned and
/// SIGKILL its group. Best-effort; the `recordings/gwrec_` needle is unique.
struct BagdGuard;
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep")
            .args(["-f", "bagd --out recordings/gwrec_"])
            .output()
        {
            for pid in String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<i32>().ok())
            {
                // SAFETY: killpg(2) on bagd's own process group; no memory
                // touched. ESRCH after a clean exit is the expected no-op.
                unsafe {
                    libc::killpg(pid, libc::SIGKILL);
                }
            }
        }
    }
}

fn wait_for_bag(recordings: &Path, timeout: Duration) -> Option<PathBuf> {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if let Ok(rd) = std::fs::read_dir(recordings) {
            for e in rd.flatten() {
                let p = e.path();
                if p.extension().and_then(|x| x.to_str()) == Some("mcap") {
                    return Some(p);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    None
}

/// `--record` KEEPS the network (deliberately): a permissive
/// single-process record run fires the breadcrumb, keeps a `run-gateway` child
/// up WHILE recording, and finalizes a bag on clean SIGINT. Routed through the
/// netd-unreachable FALLBACK (a nonexistent socket) so the child path is
/// deterministic — record + netd is the same convergence as arm (a), and record +
/// child is the fallback this arm pins alongside bag finalization.
#[test]
#[serial]
fn record_keeps_network_gateway_up_and_bag_finalizes() {
    let tmp = tempfile::tempdir().unwrap();
    build_single_producer_workspace(tmp.path(), "gwrec", "gwrec", "");
    let port = probe_ephemeral_port();
    let missing_sock =
        std::env::temp_dir().join(format!("cer_gwnd_missing_rec_{}.sock", std::process::id()));
    let _ = std::fs::remove_file(&missing_sock);
    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(
        tmp.path(),
        "gwrec",
        false,
        port,
        &missing_sock,
        &["--record=recordings"],
    );
    let _bagd_guard = BagdGuard;

    // Recording works: the bag file appears (bagd handshake completed).
    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });

    // Network stays UP while recording: the gateway child exists + the
    // permissive breadcrumb fired.
    let run_pid = guard.id();
    let gw_pid = wait_for_gateway_child(run_pid, Duration::from_secs(15))
        .expect("record + permissive must keep a `run-gateway` child up during the run");
    assert!(
        merged(&stdout_path, &stderr_path).contains(PERMISSIVE_EGRESS_NOTICE),
        "record + permissive must still fire the breadcrumb (network stays live)"
    );

    // Record a healthy window, then clean SIGINT → bag finalizes + gateway reaped.
    //
    // WAITED FOR, not slept: the window exists so the recording has real content
    // when the teardown finalizes it, and a user topic carrying frames IS that
    // content (reserved `__cerulion/…` channels are excluded — they are written
    // whether or not the data plane moved). A mid-run read sees only flushed
    // chunks, so this is a lower bound on the finalized bag.
    wait_for_bag_state(
        &bag,
        "a recorded user topic carrying frames",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.user_topics_with_frames(1) >= 1,
    );
    send_signal(run_pid, libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("record run did not exit after SIGINT");
    assert!(
        status.success(),
        "record run must exit 0 on SIGINT, got {status:?}"
    );
    assert!(
        wait_until_dead(gw_pid, Duration::from_secs(15)),
        "the gateway child must be reaped after the record run exits"
    );

    let reader = cerulion_bag::BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the record teardown must FINALIZE the bag, got {completeness:?}"
    );
}

/// Refusal twin: a `network:` block that declares `ingress:` +
/// `--record` exits NONZERO naming BOTH workarounds, with the graph
/// file byte-untouched (the refusal fires in `resolve_run_network`, before any
/// file mutation or gateway spawn).
///
/// The fixture block is INGRESS-ONLY (no `egress:` list — an ingress-only
/// enabled block is valid) and the ingress topic is CONSUMED by an in-graph
/// `sink` node with no in-graph producer, so the graph passes every load-time
/// network validation rule and the ONLY refusal in play is record+ingress.
#[test]
#[serial]
fn record_plus_declared_ingress_is_refused_naming_the_cause() {
    let tmp = tempfile::tempdir().unwrap();
    // Scaffolding (workspace + ticker node) from the shared helper, then
    // OVERWRITE the graph with the two-node ingress-consuming shape.
    build_single_producer_workspace(tmp.path(), "gwref", "gwref", "");
    // The sink node type: the data-trigger fixture (input `trigger_in`,
    // output `cmd`), consuming the ingress topic as its trigger source.
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    std::fs::create_dir_all(tmp.path().join("nodes/sink/src")).unwrap();
    std::fs::copy(
        fixtures.join(DATA_FIXTURE).join("src/lib.rs"),
        tmp.path().join("nodes/sink/src/lib.rs"),
    )
    .expect("copy data-trigger fixture src");
    std::fs::copy(
        fixture_cdylib(DATA_FIXTURE),
        tmp.path().join("target/debug").join(dylib_file("sink")),
    )
    .expect("copy data-trigger fixture cdylib");
    let original = "name: gwref\n\
                    prefix: gwref\n\
                    nodes:\n\
                    - id: ticker\n\
                    \x20 type: ticker\n\
                    \x20 inputs: []\n\
                    \x20 outputs:\n\
                    \x20 - name: cmd\n\
                    \x20\x20\x20 schema: geometry_msgs/Vector3\n\
                    - id: sink\n\
                    \x20 type: sink\n\
                    \x20 inputs:\n\
                    \x20 - name: trigger_in\n\
                    \x20\x20\x20 source: /gwref/incoming\n\
                    \x20 outputs:\n\
                    \x20 - name: cmd\n\
                    \x20\x20\x20 schema: geometry_msgs/Vector3\n\
                    network:\n\
                    \x20 mode: peer\n\
                    \x20 listen:\n\
                    \x20 - tcp/127.0.0.1:0\n\
                    \x20 ingress:\n\
                    \x20 - /gwref/incoming\n"
        .to_string();
    let graph_path = tmp.path().join("graphs/gwref.yaml");
    std::fs::write(&graph_path, &original).unwrap();
    let bak_path = tmp.path().join("graphs/gwref.yaml.bak");

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "graph",
            "run",
            "gwref",
            "--single-process",
            "--record=recordings",
        ])
        .current_dir(tmp.path())
        .env_remove("CARGO_TARGET_DIR")
        // The refusal fires in resolve_run_network BEFORE any egress; a nonexistent
        // netd bin keeps the run hermetic (never spawns a real netd) regardless.
        .env("CERULION_NETD_BIN", NONEXISTENT_NETD_BIN)
        .stdin(Stdio::null())
        .output()
        .expect("run cerulion graph run (record + declared ingress)");

    assert!(
        !out.status.success(),
        "record + declared-ingress must be refused, got {:?}",
        out.status
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("not yet replay-faithful"),
        "the refusal must name the cause; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("--network off") && stderr.contains("drop `--record`"),
        "the refusal must name BOTH workarounds (`--network off` / drop `--record`); stderr:\n{stderr}"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("graph readable"),
        original,
        "a REFUSED run must never mutate the graph file"
    );
    assert!(!bak_path.exists(), "no .bak on the refused path");
}
