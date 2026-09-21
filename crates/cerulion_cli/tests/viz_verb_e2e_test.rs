// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion viz` verb as a REAL-BINARY `cerulion-vizd`
//! CLIENT, end-to-end.
//!
//! The daemon side is thoroughly e2e-tested in-process over a real socket + real
//! iceoryx2 (`cerulion_vizd/tests/vizd_e2e_test.rs`). This file closes the last
//! gap: the `run_viz` orchestration glue over the ACTUAL `cerulion` binary —
//! spawning the ACTUAL `cerulion-vizd`, connecting over its UDS, attaching, and
//! surfacing results / errors. All three arms drive the real binary:
//!
//! 1. **Live attach** — a real SHM producer publishes `geometry_msgs/Vector3`;
//!    `cerulion viz /topic --detach` spawns the daemon, attaches, and
//!    prints the RESOLVED schema; a UDS round-trip proves the daemon is hosting
//!    (banner `rerun_url`) with the topic in its `list`.
//! 2. **Error surface** — attaching a non-existent topic surfaces the daemon's
//!    "topic does not exist" error VERBATIM + a nonzero exit.
//! 3. **Daemon reuse** — a second `viz` invocation reuses the running daemon (no
//!    "started the daemon" breadcrumb the second time).
//!
//! PREREQ: `cargo build -p cerulion_vizd` (the daemon binary is out of
//! `default-members`, so `cargo test -p cerulion_cli` does not build it). Each
//! test PRECONDITION-panics with the recipe if the sibling binary is absent.
//!
//! Uses the DEFAULT iceoryx2 root (the producer here + the spawned daemon must
//! share it), so `#[serial]` + a unique pid-stamped topic. `CERULION_VIZD_SOCK`
//! points the daemon + client at a temp socket; `CERULION_NETWORK=off` keeps the
//! CLI-side run local (no gateway) and **`CERULION_VIZD_NETWORK=off`** keeps the
//! spawned DAEMON local (see [`run_viz`]). The detached daemon is reaped
//! via its pidfile by an RAII guard (SIGKILL) even on a panic.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;
use serde_json::Value;
use serial_test::serial;

/// The `cerulion` binary under test.
fn cerulion_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_cerulion"))
}

/// The `cerulion-vizd` binary the verb spawns (sibling of the `cerulion` exe).
/// PRECONDITION-panics with the build recipe if absent.
fn require_vizd_binary() -> PathBuf {
    let vizd = cerulion_bin()
        .parent()
        .expect("cerulion exe has a parent dir")
        .join("cerulion-vizd");
    assert!(
        vizd.is_file(),
        "cerulion-vizd not found at {} — run `cargo build -p cerulion_vizd` first \
         (it is out of default-members, so `cargo test -p cerulion_cli` does not build it)",
        vizd.display()
    );
    vizd
}

/// A unique temp dir + socket path (sockaddr_un ~104-char limit → short).
fn temp_socket(tag: &str) -> (PathBuf, PathBuf) {
    use std::sync::atomic::AtomicU64;
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_viz_verb_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    (dir.join("vizd.sock"), dir)
}

/// A full `geometry_msgs/Vector3` wire frame (crib `vizd_e2e_test`).
fn build_vector3_frame(seq: u32, ts: u64, v: [f64; 3]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    for x in v {
        payload.extend_from_slice(&x.to_le_bytes());
    }
    let header = WireHeader {
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A background producer on the DEFAULT iceoryx2 root (the root the spawned
/// daemon taps). Publishes `Vector3` at ~10 ms until dropped.
struct Producer {
    stop: Arc<AtomicBool>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Producer {
    fn spawn(topic: &str) -> Self {
        let mgr = TransportManager::get_or_init().expect("default transport");
        let stop = Arc::new(AtomicBool::new(false));
        let topic = topic.to_string();
        let stop_c = Arc::clone(&stop);
        let handle = std::thread::spawn(move || {
            let mut publisher = mgr
                .create_publisher(&topic, MaxSliceLen::const_new(1 << 16), 0)
                .expect("producer attaches");
            let seq = AtomicU32::new(0);
            while !stop_c.load(Ordering::Relaxed) {
                let s = seq.fetch_add(1, Ordering::Relaxed);
                let frame = build_vector3_frame(s, 1_000 * (s as u64 + 1), [s as f64, 2.0, 3.0]);
                let _ = publisher.publish_raw(&frame);
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        Producer {
            stop,
            handle: Some(handle),
        }
    }
}

impl Drop for Producer {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// Reaps the detached `cerulion-vizd` (via its pidfile) on drop — even on panic.
struct DaemonReaper {
    socket: PathBuf,
    dir: PathBuf,
}

impl DaemonReaper {
    fn pidfile(&self) -> PathBuf {
        self.socket.with_extension("pid")
    }
}

impl Drop for DaemonReaper {
    fn drop(&mut self) {
        if let Ok(s) = std::fs::read_to_string(self.pidfile()) {
            if let Ok(pid) = s.trim().parse::<u32>() {
                // SIGKILL the detached daemon; ignore ESRCH (already gone).
                unsafe {
                    libc::kill(pid as libc::pid_t, libc::SIGKILL);
                }
            }
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// Run `cerulion viz <args>` with the temp socket + local-only env, capturing
/// (success, stdout, stderr).
///
/// Hermeticity: `CERULION_NETWORK=off` is the **CLI-side** kill switch —
/// it does NOT reach the spawned daemon's network plane, which is gated by its own
/// `CERULION_VIZD_NETWORK` (`cerulion_vizd::net::NETWORK_ENV`; unset ⇒ the
/// scouting-ON desk default). Without `CERULION_VIZD_NETWORK=off` the daemon is
/// network-configured, so an attach of a topic that is not locally visible (arm 2)
/// takes the remote-resolution path → the production `NetdDemandPlane` →
/// `NetdClient::connect_or_spawn()` on the WELL-KNOWN socket: the test would talk
/// to the desk's live `cerulion-netd`, or SPAWN a detached machine-wide one that
/// `DaemonReaper` (vizd-only) never reaps. The daemon inherits this env from the
/// `cerulion viz` child that spawns it, so setting it here covers both processes.
fn run_viz(socket: &Path, args: &[&str]) -> (bool, String, String) {
    let out = Command::new(cerulion_bin())
        .arg("viz")
        .args(args)
        .env("CERULION_VIZD_SOCK", socket)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_VIZD_NETWORK", "off")
        // Unset any ambient override so the daemon HOSTS (host mode) and the
        // banner advertises a rerun_url.
        .env_remove("CERULION_RERUN_URL")
        .output()
        .expect("spawn cerulion viz");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// Connect to the daemon socket (bounded retry), read the Hello banner.
fn connect_and_banner(socket: &Path) -> (UnixStream, BufReader<UnixStream>, Value) {
    let deadline = Instant::now() + Duration::from_secs(5);
    let stream = loop {
        match UnixStream::connect(socket) {
            Ok(s) => break s,
            Err(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(20)),
            Err(e) => panic!("could not connect to {}: {e}", socket.display()),
        }
    };
    let writer = stream.try_clone().expect("clone");
    let mut reader = BufReader::new(stream);
    let mut banner = String::new();
    reader.read_line(&mut banner).expect("banner");
    let v: Value = serde_json::from_str(&banner).expect("banner json");
    (writer, reader, v)
}

fn request(writer: &mut UnixStream, reader: &mut BufReader<UnixStream>, line: &str) -> Value {
    writeln!(writer, "{line}").expect("write");
    let mut resp = String::new();
    reader.read_line(&mut resp).expect("read");
    serde_json::from_str(&resp).expect("resp json")
}

// ── Arm 1 + 3: live attach over the real binary + daemon reuse ───────────────

#[test]
#[serial]
fn viz_verb_attaches_a_live_topic_over_the_real_binary_and_reuses_the_daemon() {
    require_vizd_binary();
    let topic = format!("/vizverbe2e/vel_{}", std::process::id());
    let _producer = Producer::spawn(&topic);
    let (socket, dir) = temp_socket("live");
    let _reaper = DaemonReaper {
        socket: socket.clone(),
        dir,
    };

    // FIRST run: no daemon yet → the verb SPAWNS it, attaches, prints the resolved
    // schema, and returns (--detach). No viewer is started: Studio is the viewer.
    let (ok, stdout, stderr) = run_viz(&socket, &[&topic, "--detach"]);
    assert!(
        ok,
        "cerulion viz --detach must exit 0. stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(
        stdout.contains("started the cerulion-vizd daemon"),
        "the first run spawns the daemon:\n{stdout}"
    );
    assert!(
        stdout.contains(&format!("attached {topic}")) && stdout.contains("geometry_msgs/Vector3"),
        "the verb prints the attach with the RESOLVED schema:\n{stdout}"
    );

    // UDS round-trip: the daemon is HOSTING (banner rerun_url) and `list` shows
    // the tapped topic — the attach really landed in the running daemon.
    let (mut w, mut r, banner) = connect_and_banner(&socket);
    assert_eq!(banner["vizd"].as_str(), Some("cerulion-vizd"));
    assert!(
        banner["rerun_url"]
            .as_str()
            .is_some_and(|u| u.contains("/proxy")),
        "host mode advertises a rerun_url: {banner}"
    );
    let list = request(&mut w, &mut r, r#"{"id":1,"method":"list"}"#);
    let found = list["attached"]
        .as_array()
        .expect("attached array")
        .iter()
        .any(|e| e["topic"].as_str() == Some(topic.as_str()));
    assert!(found, "the tapped topic is in the daemon's list: {list}");
    drop((w, r));

    // SECOND run: the daemon is already up → REUSE it (no spawn breadcrumb), and
    // the attach is idempotent (still ok, still Vector3).
    let (ok2, stdout2, stderr2) = run_viz(&socket, &[&topic, "--detach"]);
    assert!(ok2, "second viz exits 0. stderr:\n{stderr2}");
    assert!(
        !stdout2.contains("started the cerulion-vizd daemon"),
        "a LIVE daemon is REUSED — no second spawn breadcrumb:\n{stdout2}"
    );
    assert!(
        stdout2.contains(&format!("attached {topic}")),
        "the reused daemon still attaches:\n{stdout2}"
    );
}

// ── Arm 2: a daemon attach error surfaces VERBATIM + a nonzero exit ──────────

#[test]
#[serial]
fn viz_verb_surfaces_a_daemon_attach_error_and_exits_nonzero() {
    require_vizd_binary();
    let (socket, dir) = temp_socket("err");
    let _reaper = DaemonReaper {
        socket: socket.clone(),
        dir,
    };

    // A topic nobody produces → the verb exits nonzero (attached 0 topics — never a
    // silent empty viewer) and an ACTIONABLE topic-not-found error surfaces VERBATIM,
    // naming the topic. `run_viz` sets `CERULION_VIZD_NETWORK=off`, so the
    // daemon is LOCAL-ONLY (`manager.network() == None`) and `attach` provably takes
    // the genuine-local tap path — never the remote resolution that would
    // reach the well-known `cerulion-netd`. So the expected arm is the local tap's
    // "does not exist"; asserting it (rather than an OR with the network-configured
    // "ROBOTS list" arm) also makes this test the behavioral pin that the daemon-side
    // kill switch really bit.
    let missing = format!("/vizverbe2e/no_such_{}", std::process::id());
    let (ok, stdout, stderr) = run_viz(&socket, &[&missing, "--detach"]);
    assert!(
        !ok,
        "attaching a non-existent topic exits nonzero. stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    let combined = format!("{stdout}\n{stderr}");
    assert!(
        combined.contains(&missing) && combined.contains("does not exist"),
        "a LOCAL-ONLY daemon's actionable topic-not-found error surfaces VERBATIM \
         (if this reads like a remote/ROBOTS-list error the CERULION_VIZD_NETWORK=off \
         hermeticity switch stopped biting):\n{combined}"
    );

    // The daemon itself came up fine (the error was per-topic, not a daemon
    // failure) — its banner is reachable.
    let (_w, _r, banner) = connect_and_banner(&socket);
    assert_eq!(banner["protocol"].as_u64(), Some(1));
}

// ── Ctrl+C detaches ONLY the taps THIS run created ───────────────────────────
// The daemon's taps are GLOBAL shared state. A foreground `cerulion viz /topic`
// that re-attaches a topic another controller (a `--detach` run / Studio) already
// tapped must, on Ctrl+C, leave that tap ALIVE — not silently freeze the other
// controller's scene. A verb that recorded EVERY ok-attach and detached them
// all on Ctrl+C (discarding the daemon's `already_attached`) would do exactly that.

#[test]
#[serial]
fn viz_verb_ctrlc_leaves_another_controllers_tap_alive() {
    require_vizd_binary();
    let topic = format!("/vizverbe2e/shared_{}", std::process::id());
    let _producer = Producer::spawn(&topic);
    let (socket, dir) = temp_socket("shared");
    let _reaper = DaemonReaper {
        socket: socket.clone(),
        dir,
    };

    // Controller A (--detach): CREATES the tap and leaves it + the daemon running.
    let (ok_a, stdout_a, stderr_a) = run_viz(&socket, &[&topic, "--detach"]);
    assert!(
        ok_a,
        "controller A --detach exits 0. stdout:\n{stdout_a}\nstderr:\n{stderr_a}"
    );
    assert!(
        stdout_a.contains(&format!("attached {topic}")),
        "controller A attached the topic:\n{stdout_a}"
    );

    // Controller B (FOREGROUND): re-attaches the SAME topic (already_attached), then
    // waits on Ctrl+C. Spawn it with a piped stdout so we can wait until it reaches
    // its foreground loop before signalling.
    let mut child = Command::new(cerulion_bin())
        .arg("viz")
        .arg(&topic)
        .env("CERULION_VIZD_SOCK", &socket)
        .env("CERULION_NETWORK", "off")
        // The DAEMON-side kill switch too (see `run_viz`) — this child can
        // also be the one that spawns the daemon, and it inherits this env.
        .env("CERULION_VIZD_NETWORK", "off")
        .env_remove("CERULION_RERUN_URL")
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("spawn foreground viz");
    let pid = child.id();

    // Wait until B prints "visualizing" (it reached the foreground Ctrl+C loop).
    let stdout = child.stdout.take().expect("piped stdout");
    let (tx, rx) = std::sync::mpsc::channel();
    let reader = std::thread::spawn(move || {
        let mut r = BufReader::new(stdout);
        let mut collected = String::new();
        let mut line = String::new();
        loop {
            line.clear();
            match r.read_line(&mut line) {
                Ok(0) => break,
                Ok(_) => {
                    if line.contains("visualizing") {
                        let _ = tx.send(());
                    }
                    collected.push_str(&line);
                }
                Err(_) => break,
            }
        }
        collected
    });
    let reached_loop = rx.recv_timeout(Duration::from_secs(15)).is_ok();
    assert!(
        reached_loop,
        "controller B must reach its foreground loop (attach already_attached, then wait)"
    );

    // SIGINT B → it exits cleanly, detaching only taps IT created (none).
    unsafe {
        libc::kill(pid as libc::pid_t, libc::SIGINT);
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    let exited = loop {
        match child.try_wait() {
            Ok(Some(_)) => break true,
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(50)),
            _ => break false,
        }
    };
    if !exited {
        let _ = child.kill();
    }
    let b_stdout = reader.join().unwrap_or_default();
    assert!(
        exited,
        "controller B must exit on SIGINT (not hang):\n{b_stdout}"
    );

    // THE PIN: controller A's tap SURVIVES B's Ctrl+C — the daemon's `list` still
    // carries the topic. If B detached it, the array would be empty.
    let (mut w, mut r, _banner) = connect_and_banner(&socket);
    let list = request(&mut w, &mut r, r#"{"id":1,"method":"list"}"#);
    let still_tapped = list["attached"]
        .as_array()
        .expect("attached array")
        .iter()
        .any(|e| e["topic"].as_str() == Some(topic.as_str()));
    assert!(
        still_tapped,
        "controller B's Ctrl+C must NOT tear down the tap controller A created \
         (that would freeze A's scene). B stdout:\n{b_stdout}\nlist: {list}"
    );
}

// ── Arg validation runs BEFORE spawning the daemon ───────────────────────────
// A typo'd command must never leave an orphan daemon running.

#[test]
#[serial]
fn viz_verb_validates_args_before_spawning_the_daemon() {
    require_vizd_binary();
    let (socket, dir) = temp_socket("badargs");
    let _reaper = DaemonReaper {
        socket: socket.clone(),
        dir,
    };

    // A typo'd remote command: `--robot` with NO topics. That is an arg-shape error
    // the verb must reject up front, BEFORE ensuring/spawning the daemon —
    // otherwise a bad command orphans a long-lived daemon + viewer. (A schema pin
    // is NO LONGER required for a remote topic — the daemon catalog-resolves the
    // type — so a bare `--robot go2 /topic` is valid; the up-front guard
    // is the missing-topic case.)
    let (ok, stdout, stderr) = run_viz(&socket, &["--robot", "go2"]);
    assert!(
        !ok,
        "a --robot command with no topics exits nonzero. stdout:\n{stdout}\nstderr:\n{stderr}"
    );
    let combined = format!("{stdout}\n{stderr}");
    assert!(
        combined.contains("needs at least one TOPIC"),
        "the actionable missing-topic error surfaces:\n{combined}"
    );
    // The daemon was NEVER spawned (validation ran first): no spawn breadcrumb and
    // the socket the daemon would bind does not exist. With validation after the
    // spawn both would be present (the daemon spawned, THEN the schema-pin error fired).
    assert!(
        !stdout.contains("started the cerulion-vizd daemon"),
        "a validation error must not spawn the daemon:\n{stdout}"
    );
    assert!(
        !socket.exists(),
        "the daemon socket must not exist — validation ran before any spawn"
    );
}
