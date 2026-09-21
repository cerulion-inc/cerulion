// SPDX-License-Identifier: AGPL-3.0-only
//! The idle-self-exit LIFECYCLE + the bounded hard-exit watchdog over the
//! REAL `cerulion-netd` BINARY (a spawned process, `CARGO_BIN_EXE_cerulion-netd`).
//!
//! The hazard: a fresh netd serves its first demand, but after the last consumer leaves
//! and the idle grace elapses it enters a HALF-DEAD state — it never exits, keeps
//! its socket bound, and accepts-then-drops every later connection FOREVER, so
//! clients bind to the zombie instead of respawning. The daemon-library half of
//! the defense (unlink the socket FIRST + the atomic exit decision + the client retry)
//! is pinned in-process with the spy plane (`daemon_e2e_test.rs` /
//! `client_e2e_test.rs`); THIS file pins the PROCESS half over the real binary:
//!
//! 1. `real_daemon_idle_self_exit_terminates_and_removes_socket` — the core
//!    regression: a spawned real daemon that is never demanded (or whose consumer
//!    leaves) ACTUALLY exits after the grace AND removes its socket. This
//!    is exactly the surface that would otherwise wedge.
//! 2. `real_daemon_hard_exits_a_wedged_teardown` — the watchdog is load-bearing: a
//!    teardown deliberately STALLED (a test-only env seam simulating a wedged
//!    zenoh/mirror teardown, which unlinks the socket FIRST then hangs) is
//!    HARD-exited within the bounded deadline, so a real wedge can never leave a
//!    zombie.
//!
//! SCOPE — the real production wedge is a BLOCKED zenoh session close /
//! network-thread join, reproducible only against a LIVE robot (the SpyPlane and a
//! `CERULION_NETD_NETWORK=off` binary have no zenoh threads to wedge). Test 2 uses
//! the stall seam to exercise the watchdog path deterministically; the exact
//! zenoh-thread-hang trigger is a residual that needs a live robot. Both tests are hermetic
//! (`CERULION_NETD_NETWORK=off` — no zenoh) and pass the daemon's env DIRECTLY to
//! the child (no parent-env mutation); a file-local mutex serializes them because
//! the spawned daemons share the default iceoryx2 namespace.

use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock};
use std::time::{Duration, Instant};

/// Serialize the two real-binary spawns — they share the default iceoryx2
/// namespace, so running them sequentially avoids any cross-daemon SHM contention.
fn iox_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

/// A unique short temp dir + socket (the socket path must fit the sockaddr_un limit).
fn unique_socket(tag: &str) -> (PathBuf, PathBuf) {
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_netd_idle_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    let sock = dir.join("netd.sock");
    (dir, sock)
}

/// SIGKILL + reap the child on drop so a failing/hung test never leaks the daemon.
struct ChildGuard {
    child: Child,
    dir: PathBuf,
}
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    f()
}

#[test]
fn real_daemon_idle_self_exit_terminates_and_removes_socket() {
    let _lk = iox_lock();
    let (dir, sock) = unique_socket("term");

    // Spawn the REAL binary: local-only (no zenoh), a short idle grace so it
    // self-exits quickly. Env passed DIRECTLY to the child (no parent-env mutation).
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .env("CERULION_NETD_SOCK", &sock)
        .env("CERULION_NETD_NETWORK", "off")
        .env("CERULION_NETD_IDLE_GRACE_MS", "300")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn cerulion-netd");
    let mut guard = ChildGuard { child, dir };

    // It boots + binds the socket.
    assert!(
        wait_until(|| sock.exists(), Duration::from_secs(8)),
        "the real daemon booted + bound its control socket"
    );

    // A never-demanded (idle) daemon self-exits after the grace. The CORE
    // regression: the PROCESS actually terminates (a zombie would wedge forever).
    let exited = wait_until(
        || matches!(guard.child.try_wait(), Ok(Some(_))),
        Duration::from_secs(10),
    );
    assert!(
        exited,
        "the idle daemon ACTUALLY terminated after the grace (not a permanent zombie)"
    );
    let status = guard.child.try_wait().expect("try_wait").expect("exited");
    assert_eq!(
        status.code(),
        Some(0),
        "a clean idle self-exit returns 0 ({status:?})"
    );
    // And it removed its own socket (unlinked on shutdown) — a client would respawn.
    assert!(
        wait_until(|| !sock.exists(), Duration::from_secs(2)),
        "the terminated daemon removed its control socket (not left for a zombie)"
    );
}

#[test]
fn real_daemon_hard_exits_a_wedged_teardown() {
    let _lk = iox_lock();
    let (dir, sock) = unique_socket("wedge");

    // Spawn the REAL binary with a SHORT hard-exit deadline + the test-only stall
    // seam: after the idle self-exit, shutdown() unlinks the socket then the run()
    // STALLS forever (simulating a wedged teardown). WITHOUT the watchdog
    // the process would hang forever; WITH it, the daemon hard-exits within the
    // deadline. `HARD_EXIT_MS` is well under the assert bound below, so a
    // non-firing watchdog fails this test.
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .env("CERULION_NETD_SOCK", &sock)
        .env("CERULION_NETD_NETWORK", "off")
        .env("CERULION_NETD_IDLE_GRACE_MS", "300")
        .env("CERULION_NETD_HARD_EXIT_MS", "600")
        .env("CERULION_NETD_STALL_SHUTDOWN_FOR_TEST", "1")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn cerulion-netd");
    let mut guard = ChildGuard { child, dir };

    assert!(
        wait_until(|| sock.exists(), Duration::from_secs(8)),
        "the real daemon booted + bound its control socket"
    );

    // Despite the STALLED teardown, the watchdog HARD-exits well within this bound.
    // The stall loop is 3600 s, so a missing/broken watchdog leaves the process
    // running and this assert fails (never a silent pass).
    let exited = wait_until(
        || matches!(guard.child.try_wait(), Ok(Some(_))),
        Duration::from_secs(8),
    );
    assert!(
        exited,
        "the hard-exit watchdog terminated a WEDGED teardown within the deadline (no zombie)"
    );
    let status = guard.child.try_wait().expect("try_wait").expect("exited");
    assert_eq!(
        status.code(),
        Some(1),
        "the watchdog hard-exits with a NONZERO status (abnormal wedge-escape) ({status:?})"
    );
    // The socket was unlinked FIRST (inside shutdown, before the stall), so even a
    // hard-exited wedge leaves no socket for a client to bind to.
    assert!(
        !sock.exists(),
        "the socket was unlinked before the wedge — a hard-exited daemon leaves none behind"
    );
}
