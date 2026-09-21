// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END coverage of
//! the `run_graph_recording` ORCHESTRATION over the REAL binary — spawn
//! `cerulion graph run <g> --record=DIR` against a hand-built tempdir workspace
//! whose node cdylib is a COPY of the prebuilt `test_node_macro_period_cdylib`
//! fixture (no in-test cargo build), and pin the whole-run contracts the
//! unit-pinned helpers cannot see:
//!
//! 1. **Happy path / Ctrl-C** — SIGINT to the graph-run child (the production
//!    Ctrl-C) exits 0 with a FINALIZED bag carrying real frames + the
//!    graph.yaml/env.json attachments, and the merged child log shows the
//!    ordered lifecycle (spawn → ready → run → bagd final-drain → the terminal
//!    "recording complete").
//! 2. **Recording-failed exit-code mapping** — SIGKILLing the bagd grandchild
//!    mid-run makes `graph run` exit NONZERO naming the INCOMPLETE bag.
//! 3. (Linux-only) **--record-cpu pin-proof** — the bagd grandchild's
//!    `/proc/<pid>/status` `Cpus_allowed_list` equals the requested core.
//!
//! Prerequisite (repo pattern, mirrors `rayon_fire_cdylib_serial_test`):
//! `cargo build -p test_node_macro_period_cdylib` — the test PANICS with that
//! instruction if the fixture artifact is missing.
//!
//! Runs on the GLOBAL iceoryx2 namespace (the production `graph run` path has
//! no isolation seam — that is the point of an orchestration e2e); `#[serial]`
//! + unique per-run prefixes keep it safe.

#![cfg(unix)]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_core::message::ShmMessage;
use cerulion_core::trace_ring::{RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY};
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// The shared mp record-harness module, for the ONE helper this file needs from
// it: the mid-run bag poll that replaced this file's fixed recording windows.
// Imported by name (not `*`) because this file carries its own `wait_for_bag` /
// `ChildGuard` and they must keep winning.
mod mp_support;
use mp_support::{wait_for_bag_state, RECORDED_WINDOW_TIMEOUT};

/// SIGKILL + reap on drop so a panicking test never leaks the child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// The bagd GRANDCHILD leak guard. `ChildGuard` owns only
/// the graph-run parent; bagd runs in its OWN process group (the production
/// `process_group(0)`), so killing the parent on a mid-window panic ORPHANS
/// bagd (holding the .mcap + SHM taps past tempdir teardown). On Drop this
/// pgrep-discovers any bagd spawned by THIS file's runs — the relative
/// `--out recordings/demo_...` cmdline form is unique to the recording
/// driver (the sibling subprocess suite passes absolute temp paths) —
/// and SIGKILLs its process group (bagd is its own group leader, so
/// `killpg(pid)` is exact). Best-effort; init reaps the orphan. Armed
/// IMMEDIATELY after every spawn, not only in the kill variant.
struct BagdGuard;
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep")
            .args(["-f", "bagd --out recordings/demo_"])
            .output()
        {
            for pid in String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<i32>().ok())
            {
                // SAFETY: killpg(2) on the grandchild's own process group
                // (pgid == pid -- it was spawned with process_group(0)); no
                // memory is touched. Best-effort: ESRCH after a clean exit is
                // the expected no-op.
                unsafe {
                    libc::killpg(pid, libc::SIGKILL);
                }
            }
        }
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
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

fn send_signal(pid: u32, sig: libc::c_int) {
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

/// The platform cdylib filename for a crate/node name.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// The prebuilt fixture cdylib (`test_node_macro_period_cdylib` — a
/// `#[cerulion_node(period_ms = 50)]` node with one `cmd: Vector3` output).
/// PANICS with the build instruction if missing (the repo's fixture pattern).
fn fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib")
}

/// Hand-build a minimal recording workspace in `root`: a `[workspace]`
/// Cargo.toml + graphs/demo.yaml (one `ticker` period node producing
/// `/{prefix}/ticker/cmd`) + nodes/ticker/src/lib.rs (a copy of the fixture
/// source, for the metadata/staleness walkers) + target/debug/libticker.*
/// (a copy of the PREBUILT fixture cdylib — no in-test cargo build).
///
/// `prefix` must be unique per test — the runs share the global iceoryx2
/// namespace, so topic names must not collide across tests/runs.
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join("graphs/demo.yaml"),
        format!(
            "name: demo\nprefix: {prefix}\nnodes:\n- id: ticker\n  type: ticker\n  inputs: []\n  \
             outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    // The fixture's source, copied for the metadata/staleness walkers (the
    // runtime's port truth comes from the cdylib's own info() FFI).
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures/test_node_macro_period_cdylib/src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    // The prebuilt cdylib under the node-type name the resolver looks for.
    std::fs::copy(
        fixture_cdylib(),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy fixture cdylib");
}

/// The `env.json` redaction probe — a NON-allowlisted
/// variable name (the allowlist is `CERULION_*`, `RUST_LOG`, `IOX2_*`), so the
/// default `--record-env allowlist` must redact its value.
const PROBE_SECRET_NAME: &str = "CER_E2E_PROBE_SECRET";

/// The probe's value. Deliberately a long, distinctive literal so the
/// "absent from the attachment bytes" half of the oracle cannot pass by
/// coincidence, and so a plain `grep` of a failing bag finds it instantly.
const PROBE_SECRET_VALUE: &str = "cer-record-probe-secret-value";

/// FNV-1a 64 of [`PROBE_SECRET_VALUE`], lowercase hex, zero-padded to 16 — the
/// HAND-WRITTEN half of the oracle. Computed independently of the tree (offset
/// basis `0xcbf29ce484222325`, prime `0x100000001b3`), not read back from
/// `cerulion_core::wire::fnv1a_hash`, so the assertion is a statement about the
/// bag's contents rather than a round trip through the code that wrote them.
/// `fnv1a_64` below re-derives it and is itself anchored on the published
/// FNV-1a 64 test vectors.
const PROBE_SECRET_FNV64: &str = "f63ebe3c0e65672d";

/// FNV-1a 64 over `bytes`, written out here rather than imported: the production
/// renderer hashes with `cerulion_core::wire::fnv1a_hash`, and calling that same
/// function to check its own output would assert nothing. Anchored on the
/// published vectors in the happy-path arm.
fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Spawn `cerulion graph run demo --record=<dir> --single-process [extra...]`
/// in `root`, with stdout+stderr redirected to files (readable while the child
/// runs — no pipe deadlock). Returns (guard, stdout_path, stderr_path).
///
/// `--single-process`: every arm in this file pins the
/// WALL-FAITHFUL single-process recording contracts on the unpartitioned
/// `demo` graph. Without the flag, the multi-process default would
/// derive an in-memory process-per-node partition on this no-TTY subprocess
/// and record the MULTI-PROCESS (quantum-timed) bag instead —
/// a different artifact than the one these pins exist for. (No arm here
/// deliberately tests the unpartitioned default; the mp recording contracts
/// live in `mp_record_e2e_test.rs`.)
fn spawn_graph_record(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    // NO `--no-validate`: a recording made with the schema checks off is
    // refused at parse (by design, because such a bag cannot be reliably
    // replay-verified). The `demo` workspace validates, so the flag bought
    // this harness nothing.
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "graph",
            "run",
            "demo",
            "--record=recordings",
            "--single-process",
        ])
        .args(extra)
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target) + info-level lifecycle logs.
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        // The `env.json` REDACTION probe. This value is
        // non-allowlisted (the allowlist is `CERULION_*` / `RUST_LOG` / `IOX2_*`),
        // so the default `--record-env allowlist` must replace it with a
        // divergence hash rather than record it verbatim. The happy-path arm
        // asserts the hash against a hand-computed FNV-1a 64 — see
        // `PROBE_SECRET_FNV64` there. Set in the harness (not one arm) so every
        // recording in this file is made in the presence of a secret.
        .env(PROBE_SECRET_NAME, PROBE_SECRET_VALUE)
        // Keep the captured log PLAIN. `cerulion_core::init_logging` builds a
        // `fmt::layer()` with tracing-subscriber's `ansi` feature on, which
        // italicises field NAMES and dims the `=`, so a field arrives as
        // `\x1b[3mnode_wire_fixed_size\x1b[0m\x1b[2m=\x1b[0m24` and no
        // whole-token match can ever see it (the existing arms here assert on
        // message text, which is never styled — which is why nothing needed
        // this before the field assertions). Same line as
        // `graph_start_order_e2e_test`'s harness, for the same reason.
        .env("NO_COLOR", "1")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run --record");
    (ChildGuard(child), stdout_path, stderr_path)
}

/// Block until the recordings dir contains a `.mcap` (bagd created the bag —
/// the handshake completed) or `timeout` elapses. Returns the bag path.
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

/// Find the `cerulion bagd` grandchild's pid by its unique bag FILENAME
/// (bounded pgrep poll). The grandchild is in its OWN process group, so it is
/// only reachable this way. The FILENAME (not the full path) is the needle:
/// the recording driver passes `--out` relative to the workspace cwd, so the
/// grandchild's cmdline carries the relative form — but the stamped filename
/// is unique either way.
fn find_bagd_pid(bag_path: &Path, timeout: Duration) -> Option<u32> {
    let needle = bag_path
        .file_name()
        .expect("bag filename")
        .to_string_lossy()
        .to_string();
    let start = Instant::now();
    while start.elapsed() < timeout {
        let out = Command::new("pgrep")
            .args(["-f", &needle])
            .output()
            .expect("pgrep");
        if let Some(pid) = String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .next()
        {
            return Some(pid);
        }
        std::thread::sleep(Duration::from_millis(30));
    }
    None
}

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

/// Arms (1)+(4): the HAPPY/Ctrl-C path. SIGINT (the production
/// Ctrl-C) → exit 0, a FINALIZED bag with real frames + both attachments, and
/// the merged log shows the ordered lifecycle: spawn → ready → run → bagd's
/// final drain → the terminal "recording complete" (the ordered-teardown
/// observable: bagd's drain-and-finalize happens INSIDE the run's teardown,
/// before the recording driver declares completion).
#[test]
#[serial]
fn record_e2e_ctrl_c_exits_zero_with_finalized_bag_and_ordered_lifecycle() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2ea");
    let (mut guard, _stdout_path, stderr_path) = spawn_graph_record(tmp.path(), &[]);
    // Kill any orphaned bagd grandchild on ANY exit.
    let _bagd_guard = BagdGuard;

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    // Let the 50ms-period node publish a healthy batch — waited for on the
    // recorded frames the assertions below read, not on the clock. A mid-run read
    // sees only flushed chunks, so this is a lower bound on the finalized bag,
    // and a machine too slow for a fixed window does not flake here.
    wait_for_bag_state(
        &bag,
        "a recorded user topic carrying frames",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.user_topics_with_frames(1) >= 1,
    );

    // The production Ctrl-C: SIGINT to the graph-run child (bagd is in its own
    // process group and must NOT receive it — the recording driver drives it).
    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(status.success(), "Ctrl-C must exit 0, got {status:?}");

    // FINALIZED bag with real frames on the derived topic + both attachments.
    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the Ctrl-C teardown must FINALIZE the bag, got {completeness:?}"
    );
    let frames = msgs
        .iter()
        .filter(|m| m.topic == "/rece2ea/ticker/cmd")
        .count();
    assert!(
        frames > 0,
        "the bag must contain frames published up to shutdown (teardown order: \
         publishers stopped BEFORE bagd's final drain, so nothing in flight was cut)"
    );
    assert!(
        reader.attachment("graph.yaml").expect("read").is_some(),
        "graph.yaml attachment present"
    );
    // The `env.json` attachment is present AND the
    // DEFAULT `--record-env allowlist` really redacted a secret on the way in.
    //
    // This is the one place the whole `--record-env` chain is observed end to
    // end: the clap default → `RecordEnvMode::from` (unit-pinned in
    // `cli.rs::time_source_flag_tests`) → `render_env_json` → the bag bytes a
    // user shares. Both halves below are needed:
    //
    //  * the HASH, not mere absence. A swap of the `From` arm to
    //    `RecordEnvMode::None` also removes the secret from the bytes, so an
    //    absence-only assertion also passes a change that silently destroys replay
    //    env-fidelity. The hash is what says "redacted, and divergence is still
    //    detectable".
    //  * an ALLOWLISTED value verbatim. Without it, a change that redacted
    //    EVERYTHING (or mapped the default to `None`) would satisfy the
    //    redaction half.
    let env_att = reader
        .attachment("env.json")
        .expect("read")
        .expect("env.json attachment present");
    // Anchor the hand-written hasher on the published FNV-1a 64 vectors before
    // trusting it as an oracle.
    assert_eq!(format!("{:016x}", fnv1a_64(b"")), "cbf29ce484222325");
    assert_eq!(format!("{:016x}", fnv1a_64(b"a")), "af63dc4c8601ec8c");
    assert_eq!(format!("{:016x}", fnv1a_64(b"foobar")), "85944171f73967e8");
    assert_eq!(
        format!("{:016x}", fnv1a_64(PROBE_SECRET_VALUE.as_bytes())),
        PROBE_SECRET_FNV64,
        "the recorded constant must be the hash of the probe value it names"
    );

    let env_json: serde_json::Value =
        serde_json::from_slice(&env_att.data).expect("env.json parses as JSON");
    let probe = &env_json[PROBE_SECRET_NAME];
    assert_eq!(
        probe["redacted"],
        serde_json::Value::Bool(true),
        // Deliberately prints ONLY the probe's own rendered value, never the whole
        // attachment: under a regression that records verbatim values this
        // document IS the process environment, and a failing CI job must not
        // copy it into a build log.
        "a non-allowlisted value must be recorded as {{\"redacted\":true,\"fnv64\":…}} under the \
         DEFAULT --record-env allowlist; got {probe}"
    );
    assert_eq!(
        probe["fnv64"].as_str(),
        Some(PROBE_SECRET_FNV64),
        "the redaction must carry the value's FNV-1a 64 so replay can DETECT divergence; got \
         {probe}"
    );
    assert_eq!(
        env_json["CERULION_NETWORK"],
        serde_json::Value::String("off".to_string()),
        "an ALLOWLISTED name (CERULION_*) keeps its value verbatim — this is the half that fails \
         if the default is mapped to `none` or if everything is redacted"
    );
    // The secret itself never reaches the artifact. Checked over the RAW bytes,
    // not the parsed value, so a second copy anywhere in the attachment (a
    // duplicated key, a nested render) is caught too.
    assert!(
        !String::from_utf8_lossy(&env_att.data).contains(PROBE_SECRET_VALUE),
        "the env.json attachment must not contain the secret literal"
    );
    // A REAL `--record` bag carries the recorder-host
    // identity attachment, stamped with THIS host's arch/os (replay warns on
    // a cross-arch/os bag — never refuses).
    let recorder_att = reader
        .attachment("__cerulion/recorder.json")
        .expect("read")
        .expect("__cerulion/recorder.json attachment present");
    let recorder: serde_json::Value =
        serde_json::from_slice(&recorder_att.data).expect("recorder.json parses as JSON");
    assert_eq!(
        recorder["arch"],
        std::env::consts::ARCH,
        "recorder.json carries the recording host's arch"
    );
    assert_eq!(
        recorder["os"],
        std::env::consts::OS,
        "recorder.json carries the recording host's os"
    );
    assert_eq!(
        recorder["cerulion_version"],
        env!("CARGO_PKG_VERSION"),
        "recorder.json carries the recording binary's version (workspace-lockstep)"
    );
    assert!(
        recorder["recorded_at_ns"].as_u64().unwrap_or(0) > 0,
        "recorder.json carries the recording start timestamp: {recorder}"
    );

    // Ordered lifecycle in the merged child log. The lifecycle breadcrumbs are
    // `tracing::info!` events, which go to STDERR (init_logging writes there so
    // a command's stdout stays clean data); bagd inherits the same stderr fd, so
    // line order == write order across both processes in this one file. bagd's
    // terminal line is `bagd recording complete` (its ONE info line of a clean
    // finalize; the final-drain and finalized lines ride `debug!` and are
    // filtered at this harness's `cerulion_bagd=info`), so that is the mark
    // bagd's shutdown must reach BEFORE the CLI reports the bag.
    let log = read_file(&stderr_path);
    let idx = |needle: &str| {
        log.find(needle)
            .unwrap_or_else(|| panic!("log must contain `{needle}`; log was:\n{log}"))
    };
    let spawned = idx("spawned bagd recorder");
    let ready = idx("bagd ready — starting recorded graph run");
    let bagd_done = idx("bagd recording complete");
    let complete = idx("recording complete — bag finalized");
    assert!(
        spawned < ready && ready < bagd_done && bagd_done < complete,
        "lifecycle order must be spawn({spawned}) < ready({ready}) < bagd-complete({bagd_done}) \
         < recording-complete({complete}); log was:\n{log}"
    );
}

/// Arm (3): the recording-failed EXIT-CODE mapping. SIGKILL the
/// bagd grandchild mid-run → the graph-run teardown observes the dead recorder
/// (a non-success exit) and `graph run` exits NONZERO naming the INCOMPLETE
/// bag.
#[test]
#[serial]
fn record_e2e_killed_bagd_maps_to_nonzero_exit_naming_incomplete_bag() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2eb");
    let (mut guard, _stdout_path, stderr_path) = spawn_graph_record(tmp.path(), &[]);
    // Kill any orphaned bagd grandchild on ANY exit.
    let _bagd_guard = BagdGuard;

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    let bagd_pid = find_bagd_pid(&bag, Duration::from_secs(10))
        .expect("could not locate the bagd grandchild by its --out path");

    // Kill the recorder mid-run, then Ctrl-C the graph run.
    send_signal(bagd_pid, libc::SIGKILL);
    std::thread::sleep(Duration::from_millis(200));
    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(
        !status.success(),
        "a dead recorder must map to a NONZERO graph-run exit, got {status:?}"
    );
    let stderr = read_file(&stderr_path);
    assert!(
        stderr.contains("INCOMPLETE"),
        "the exit error must say the recording is INCOMPLETE; stderr was:\n{stderr}"
    );
    assert!(
        stderr.contains(".mcap"),
        "the exit error must name the bag path; stderr was:\n{stderr}"
    );
}

/// The `--record-cpu=<k>` PIN-PROOF (Linux-only):
/// the bagd grandchild's `/proc/<pid>/status` `Cpus_allowed_list` equals the
/// requested core. macOS compiles this out (pinning is Linux-only there and
/// the flag warns + floats — covered by the resolver unit tests).
#[cfg(target_os = "linux")]
#[test]
#[serial]
fn record_e2e_record_cpu_pins_bagd_on_linux() {
    // Pick the pin target from this process's ACTUAL
    // permitted set (/proc/self/status Cpus_allowed_list — cpuset/isolation
    // honored), never from a bare count. The child inherits our mask, so any
    // permitted id is pinnable. Pick the MAX permitted id (mirrors AUTO).
    let allowed = std::fs::read_to_string("/proc/self/status")
        .expect("read /proc/self/status")
        .lines()
        .find(|l| l.starts_with("Cpus_allowed_list:"))
        .expect("Cpus_allowed_list line")
        .split(':')
        .nth(1)
        .unwrap()
        .trim()
        .to_string();
    // Parse "0-3,5,7-9" → the max id.
    let core: u32 = allowed
        .split(',')
        .flat_map(|part| {
            let part = part.trim();
            match part.split_once('-') {
                Some((_, hi)) => hi.trim().parse::<u32>().ok(),
                None => part.parse::<u32>().ok(),
            }
        })
        .max()
        .expect("at least one permitted core");

    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2ec");
    let (mut guard, _stdout_path, _stderr_path) =
        spawn_graph_record(tmp.path(), &[&format!("--record-cpu={core}")]);
    // Kill any orphaned bagd grandchild on ANY exit.
    let _bagd_guard = BagdGuard;

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    let bagd_pid =
        find_bagd_pid(&bag, Duration::from_secs(10)).expect("could not locate the bagd grandchild");

    // /proc/<pid>/status: Cpus_allowed_list must be EXACTLY the pinned core.
    let status_txt = std::fs::read_to_string(format!("/proc/{bagd_pid}/status"))
        .expect("read bagd /proc status");
    let allowed = status_txt
        .lines()
        .find(|l| l.starts_with("Cpus_allowed_list:"))
        .expect("Cpus_allowed_list line")
        .split(':')
        .nth(1)
        .unwrap()
        .trim()
        .to_string();
    assert_eq!(
        allowed,
        core.to_string(),
        "bagd must be pinned to exactly core {core} (Cpus_allowed_list)"
    );

    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(status.success(), "pinned run must still exit 0: {status:?}");
}

/// The finalized bag carries a NON-EMPTY `scheduler_trace` —
/// the scheduler → trace-ring hook is wired end-to-end. With the hook unwired the
/// ring is created and handed to bagd but NEVER fed, so this channel is
/// always empty; this is the real `graph run --record` acceptance that the hook
/// fires. Every record is a FIRE record for the single "ticker" node
/// (`node_idx` 0, the only manifest entry), `global_level` 0 (flat
/// single-process schedule), and the fire count is at least the number of
/// recorded data frames (a `period_ms` publisher publishes once per fire and
/// the ring is complete/ungated by the trace cap).
#[test]
#[serial]
fn record_e2e_bag_carries_nonempty_scheduler_trace() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2ez");
    let (mut guard, _stdout_path, _stderr_path) = spawn_graph_record(tmp.path(), &[]);
    // Kill any orphaned bagd grandchild on ANY exit.
    let _bagd_guard = BagdGuard;

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    // Let the 50ms-period ticker fire a healthy batch — waited for on BOTH things
    // this arm reads out of the bag afterwards: the frames and the scheduler
    // trace.
    wait_for_bag_state(
        &bag,
        "recorded frames and a non-empty scheduler trace",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.user_topics_with_frames(1) >= 1 && !snap.trace.is_empty(),
    );

    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(status.success(), "Ctrl-C must exit 0, got {status:?}");

    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the Ctrl-C teardown must FINALIZE the bag, got {completeness:?}"
    );
    let frames = msgs
        .iter()
        .filter(|m| m.topic == "/rece2ez/ticker/cmd")
        .count();
    assert!(
        frames > 0,
        "the ticker published at least one frame before shutdown"
    );

    // The headline: the scheduler_trace channel is fed by the hook —
    // FIRE records for every fire, plus one kind-3 StepBoundary per scheduler
    // step.
    let trace = reader
        .scheduler_trace()
        .expect("read scheduler_trace channel");
    assert!(
        !trace.is_empty(),
        "the scheduler → ring hook must feed the bag's scheduler_trace \
         (this channel was ALWAYS EMPTY before the hook was wired)"
    );
    let mut fires = Vec::new();
    let mut boundaries = Vec::new();
    for r in &trace {
        assert_eq!(r.reserved, 0, "reserved is always written 0");
        match r.record_type {
            RECORD_TYPE_FIRE => {
                assert_eq!(
                    r.node_idx, 0,
                    "single-node graph → the only manifest index is 0 (ticker)"
                );
                assert_eq!(
                    r.global_level, 0,
                    "flat single-process schedule → global_level 0"
                );
                fires.push(r);
            }
            RECORD_TYPE_STEP_BOUNDARY => {
                assert_eq!(r.node_idx, 0, "boundary node_idx is 0 by contract");
                assert_eq!(r.duration_ns, 0, "boundary duration is 0 by contract");
                boundaries.push(r);
            }
            other => panic!(
                "unexpected record kind {other} in the bag (only FIRE=1 and \
                 STEP_BOUNDARY=3 are minted; DEPARTURE is multi-process only, kinds 4+ reserved)"
            ),
        }
    }
    assert!(
        !fires.is_empty() && !boundaries.is_empty(),
        "the bag must carry BOTH fire records ({}) and per-step kind-3 \
         boundaries ({})",
        fires.len(),
        boundaries.len()
    );
    assert!(
        fires.len() >= frames,
        "one fire per published frame, ring complete/ungated: {} fires >= {} frames",
        fires.len(),
        frames
    );
    // Addendum acceptance, e2e form: the boundary clock never regresses
    // (non-decreasing under the live wall clock; the STRICT
    // strictly-increasing pin lives in the VirtualClock unit suites), and
    // every fire's step has a boundary at-or-after the fire's stamped time.
    for w in boundaries.windows(2) {
        assert!(
            w[0].fire_time_ns <= w[1].fire_time_ns,
            "boundary stream must be non-decreasing in fire_time_ns"
        );
    }
    for f in &fires {
        let b = boundaries
            .iter()
            .find(|b| b.step == f.step)
            .unwrap_or_else(|| panic!("fire at step {} has no boundary record", f.step));
        assert!(
            b.fire_time_ns >= f.fire_time_ns,
            "a step's boundary carries the ADVANCED clock — never before its fires"
        );
    }
}

// ===========================================================================
// The recording's SCHEMA PROVENANCE crosses the bagd spawn.
//
// `cerulion bag record` bags carry a `__cerulion/schemas.json`
// attachment carrying the verbatim text of every custom type the recording's
// frames use, so a bag plays on a desk that never compiled them. That path runs
// bagd IN-PROCESS. `graph run --record` spawns bagd as a SUBPROCESS, so no
// in-process catalog could cross and its bags carried NAMED-but-textless
// channels. These arms pin the closed loop over the REAL binary.
// ===========================================================================

/// The workspace's shadowing `geometry_msgs/Vector3` definition — the byte
/// oracle the text arm compares the bag's doc against.
///
/// The leading COMMENT is load-bearing: it appears in no built-in and in no
/// generated type, so a recorder that wrote the compiled-in text instead of
/// THIS workspace's file fails the comparison rather than passing on a
/// coincidence.
const SHADOW_VECTOR3: &str = "# this workspace's OWN definition, not the compiled-in one\n\
     float64 x\nfloat64 y\nfloat64 z\n";

/// A custom type NOTHING in the graph publishes (plus the nested type it
/// depends on) — the prune oracle. bagd must ship only the closure the RECORDED
/// hashes need, so neither of these may reach the bag.
const UNUSED: &str = "msgs/Leaf leaf\nint32 count\n";
const LEAF: &str = "float32 value\n";

/// [`build_workspace`] plus a `schemas/` corpus: a SHADOW of the built-in the
/// fixture cdylib publishes, and an unpublished custom type + its dep.
///
/// The shadow is what makes a TEXT assertion possible without compiling a new
/// node. The recorded topic's schema hash is AUTHORITATIVE (it comes from the
/// prebuilt cdylib's `OutputMeta`), and `closure_for_hashes` resolves that hash
/// to a NAME — through any binding, the built-in tier included — then looks the
/// doc up BY NAME. So what a temp workspace needs is a doc NAMED
/// `geometry_msgs/Vector3`, which is exactly a shadow. (Note:
/// this does NOT hinge on the shadow re-deriving the same hash. That
/// it does anyway is a coherence property pinned separately by
/// `cerulion_cli_engine::graph_cmd`'s
/// `a_workspace_msg_shadowing_a_builtin_re_derives_the_builtins_own_wire_hash`.)
/// A workspace shadowing a built-in is itself a documented, supported shape, and
/// its text IS recorded.
fn build_workspace_with_schemas(root: &Path, prefix: &str) {
    build_workspace(root, prefix);
    let schemas = root.join("schemas");
    std::fs::create_dir_all(schemas.join("geometry_msgs/msg")).unwrap();
    std::fs::create_dir_all(schemas.join("msgs/msg")).unwrap();
    std::fs::write(
        schemas.join("geometry_msgs/msg/Vector3.msg"),
        SHADOW_VECTOR3,
    )
    .unwrap();
    std::fs::write(schemas.join("msgs/msg/Unused.msg"), UNUSED).unwrap();
    std::fs::write(schemas.join("msgs/msg/Leaf.msg"), LEAF).unwrap();
}

/// [`record_until_finalized`], also handing back the path of the run's STDERR
/// log (`spawn_graph_record` redirects stdout and stderr to two SEPARATE
/// files; bagd inherits the same stderr fd, so that one file carries both
/// processes' `tracing` output in write order).
///
/// The recorder's layout cross-check says what it did on STDERR, so
/// an arm about WHICH source sized a channel needs the log as well as the bag.
fn record_until_finalized_with_log(root: &Path) -> (PathBuf, PathBuf) {
    let (mut guard, _stdout_path, stderr_path) = spawn_graph_record(root, &[]);
    let _bagd_guard = BagdGuard;
    let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    std::thread::sleep(Duration::from_millis(800));
    send_signal(guard.0.id(), libc::SIGINT);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "Ctrl-C must exit 0, got {status:?}; log:\n{}",
        read_file(&stderr_path)
    );
    (bag, stderr_path)
}

/// Run the recording to a finalized bag: spawn, wait for the bag, let the
/// 50 ms-period ticker publish, SIGINT (the production Ctrl-C), require exit 0.
/// Returns the finalized bag's path.
fn record_until_finalized(root: &Path) -> PathBuf {
    let (bag, _log) = record_until_finalized_with_log(root);
    bag
}

/// THE headline: a `graph run --record` bag carries the recording machine's
/// schema TEXT, pruned to what the recording actually needs.
///
/// A bag that carries no `__cerulion/schemas.json` leaves a
/// custom type recorded on a robot rendering NOTHING on any other desk.
#[test]
#[serial]
fn record_e2e_bag_carries_the_workspace_schema_text_across_the_bagd_spawn() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace_with_schemas(tmp.path(), "rece2esc");
    let bag = record_until_finalized(tmp.path());

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the Ctrl-C teardown must FINALIZE the bag, got {completeness:?}"
    );
    let catalog = reader
        .schema_catalog()
        .expect("a graph-run recording must carry its schema provenance");

    // The TEXT crosses the spawn, byte for byte — the whole point.
    let docs: Vec<&str> = catalog.docs.iter().map(|d| d.qualified.as_str()).collect();
    assert_eq!(
        docs,
        vec!["geometry_msgs/Vector3"],
        "the bag ships exactly the closure the recorded hash needs — the workspace's \
         UNPUBLISHED `msgs/*` types must be pruned across the spawn"
    );
    assert_eq!(
        catalog.docs[0].text, SHADOW_VECTOR3,
        "the doc must be THIS workspace's definition, verbatim"
    );

    // …and the hash→name binding that lets a reader name the channel.
    assert_eq!(catalog.hashes.len(), 1, "pruned to the one recorded hash");
    assert_eq!(catalog.hashes[0].qualified, "geometry_msgs/Vector3");
    assert_eq!(
        catalog.name_for_hash(catalog.hashes[0].schema_hash),
        Some("geometry_msgs/Vector3")
    );
}

/// The ANTI-TAUTOLOGY control: the SAME graph recorded from a workspace holding
/// NO schema files ships no text — so the doc in the arm above is earned by the
/// workspace's content, not produced unconditionally.
///
/// It is also its own contract: the built-in NAME binding still crosses, which
/// is what lets a reader resolve the channel's type even where the bag ships no
/// definition (the deliberate two-tier split — see
/// `bag_cmd::build_record_schema_catalog`).
#[test]
#[serial]
fn record_e2e_a_schema_less_workspace_ships_bindings_but_no_text() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2esn");
    let bag = record_until_finalized(tmp.path());

    let reader = BagReader::open(&bag).expect("open bag");
    let catalog = reader
        .schema_catalog()
        .expect("built-in hash bindings cross even with no workspace schemas");
    assert!(
        catalog.docs.is_empty(),
        "no workspace schema files means no TEXT: {:?}",
        catalog
            .docs
            .iter()
            .map(|d| &d.qualified)
            .collect::<Vec<_>>()
    );
    assert_eq!(catalog.hashes.len(), 1, "pruned to the one recorded hash");
    assert_eq!(catalog.hashes[0].qualified, "geometry_msgs/Vector3");
}

// ======================================================================
// The node-derived `wire_fixed_size` reaches the BAG.
//
// A recorded channel's fixed size moved off the workspace
// `schemas/` file and onto the producing node's own `OutputMeta`, with the
// file kept as a cross-check.
//
// SCOPE, because the pieces are already pinned elsewhere and these arms would
// otherwise read as coverage they do not add. The DECISION
// (`resolve_recorded_wire_size`) and its wording are oracle-pinned in
// `cerulion_cli_engine::graph_cmd` — `a_stale_schema_file_does_not_
// decide_the_recorded_layout`, `a_schema_the_workspace_cannot_size_
// records_the_producers_layout`, `a_disagreeing_port_warns_once_
// naming_both_layouts` (which also matches the WARN level token and asserts
// exactly one line PER PORT — two in that test, so both "once per resolution"
// and "once per inner-fold pass" fail it); the macro really emitting the size through the cdylib FFI is
// `cerulion_core`'s `macro_cdylib_test`; a `topics.json` `wire_fixed_size`
// reaching an MCAP descriptor is `bagd_subprocess_test`. What NOTHING drove
// end to end is the COMPOSITION: a real `graph run --record` over the real
// binary, against a workspace whose file cannot size the port — or sizes it
// DIFFERENTLY — resolving through `resolve_recorded_topics`, crossing the
// bagd spawn as `topics.json`, and landing in the channel descriptor a
// reader gets back. That, and only that, is what these two arms add.
// ======================================================================

/// The fixture node's `#[output] cmd: Vector3`, sized BY HAND from the
/// upstream `.msg`: `geometry_msgs/Vector3` is three `float64`s, so its fixed
/// wire section is `3 × 8 = 24` bytes. Deliberately a literal and not
/// `<Vector3 as ShmMessage>::WIRE_FIXED_SIZE` — reading the size from the very
/// type the node publishes would compare the recorder against its own input.
///
/// The HASH assertions in these arms deliberately do the opposite and use
/// `<Vector3 as ShmMessage>::SCHEMA_HASH`, which IS that self-compare: no hand
/// oracle for a 64-bit FNV fold is worth writing, and in the disagreeing-file
/// arm a second, genuinely different candidate exists in the workspace (the
/// shadow's own store hash), so there the assertion discriminates WHICH SOURCE
/// was used rather than what the value is. In the other arm it is a shape
/// check only. Neither is independent evidence about the hash RECIPE — that is
/// `schema_hash_pin_test`'s job.
const NODE_VECTOR3_WIRE_FIXED_SIZE: u32 = 24;

/// A `schemas/` shadow of `geometry_msgs/Vector3` that describes a DIFFERENT
/// layout from the compiled node: two `float64`s, i.e. `2 × 8 = 16` bytes.
///
/// This is the shipping failure these arms are about: a workspace schema edited
/// after the node was built, or a type generated into a crate's `OUT_DIR` the
/// CLI cannot see. The port's `schema:` spelling is unchanged, so the
/// validation gate (which compares resolved schema IDENTITIES, not layouts)
/// passes and the run records.
const DISAGREEING_SHADOW_VECTOR3: &str = "# two fields where the node's compiled type has three\n\
     float64 x\nfloat64 y\n";
/// `2 × 8` — the size the file claims, and the one the bag must NOT carry.
const DISAGREEING_SHADOW_WIRE_FIXED_SIZE: u32 = 16;

/// Whether `text` carries `want` (a `key=value` field) as a WHOLE
/// whitespace-delimited token.
///
/// Not a substring test, and the difference bites here: `contains("…=24")` is
/// satisfied by `…=240`, and `contains("…=16")` by `…=160` — so a recorder
/// that reported a ten-fold wrong size would pass a substring assertion about
/// the right one. `tracing`'s compact format separates fields with spaces, so
/// the token is the exact unit to match.
fn has_field_token(text: &str, want: &str) -> bool {
    strip_ansi(text).split_whitespace().any(|tok| tok == want)
}

/// `s` with ANSI CSI escape sequences removed.
///
/// The harness sets `NO_COLOR`, so in-tree the input is already plain — this
/// is the second half of a belt-and-braces pair, and it is the half that
/// cannot be forgotten by a future caller. A styled field is ONE
/// `split_whitespace` token (`\x1b[3mkey\x1b[0m\x1b[2m=\x1b[0m24`), so a
/// predicate handed styled text would answer a confident, wrong `false` — an
/// absence claim about data that is right there.
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\u{1b}' {
            out.push(c);
            continue;
        }
        // CSI: ESC '[' , parameter/intermediate bytes, then a final byte in
        // 0x40..=0x7e. Anything else after ESC is dropped with the ESC.
        if chars.next() == Some('[') {
            for f in chars.by_ref() {
                if ('\u{40}'..='\u{7e}').contains(&f) {
                    break;
                }
            }
        }
    }
    out
}

/// The two log predicates the size arms rest on, against hand-written inputs.
///
/// They have to be pinned rather than trusted: both are about what must NOT
/// match, and a predicate that answers `false` too readily makes a presence
/// assertion fail loudly (fine) while making the ABSENCE assertion in the
/// sibling arm pass vacuously. Cheap, parallel-safe, no transport.
#[test]
fn the_log_predicates_match_whole_tokens_on_the_right_line() {
    // Plain, as the harness's `NO_COLOR` run produces it.
    assert!(has_field_token(
        "topic=/a/b node_wire_fixed_size=24 x=1",
        "node_wire_fixed_size=24"
    ));
    // Styled, as a run WITHOUT `NO_COLOR` produces it: tracing italicises the
    // field name and dims the `=`, making the whole field ONE token.
    let styled = "\u{1b}[3mfile_wire_fixed_size\u{1b}[0m\u{1b}[2m=\u{1b}[0m16";
    assert!(
        !styled
            .split_whitespace()
            .any(|t| t == "file_wire_fixed_size=16"),
        "the fixture must really be styled, or the next assertion proves nothing"
    );
    assert!(has_field_token(styled, "file_wire_fixed_size=16"));
    // The substring hazard these predicates exist for.
    assert!(!has_field_token(
        "node_wire_fixed_size=240",
        "node_wire_fixed_size=24"
    ));
    // A prefixed key is a different key; absent is absent.
    assert!(!has_field_token(
        "x_node_wire_fixed_size=24",
        "node_wire_fixed_size=24"
    ));
    assert!(!has_field_token("topic=/a/b", "node_wire_fixed_size=24"));

    // The line selector takes the line at the LEVEL, not merely the text.
    let log = " INFO marker here field=1\n WARN marker here field=2\n";
    assert_eq!(
        log_line_at_level(log, "marker here", "WARN"),
        " WARN marker here field=2"
    );
    assert!(has_field_token(
        log_line_at_level(log, "marker here", "WARN"),
        "field=2"
    ));
    // …so a field that lives only on the INFO line is NOT on the WARN line —
    // which is the whole reason the arm asserts per-line.
    assert!(!has_field_token(
        log_line_at_level(log, "marker here", "WARN"),
        "field=1"
    ));
    // TWO matching lines must be REFUSED, not first-matched. Without this the
    // helper's strongest claim — the recorder resolves each topic ONCE at
    // bring-up — is pinned by nothing: deleting the `hits.next().is_none()`
    // assert degrades it to a first-match with the whole repo green.
    let two = " WARN marker here field=1\n WARN marker here field=2\n";
    assert!(
        std::panic::catch_unwind(|| log_line_at_level(two, "marker here", "WARN")).is_err(),
        "two matching lines must be REFUSED, not silently first-matched"
    );
    // And zero must be refused too — a helper that returned an empty line
    // would make every per-line field assertion vacuous.
    assert!(
        std::panic::catch_unwind(|| log_line_at_level(" INFO marker here", "marker here", "WARN"))
            .is_err(),
        "no matching line must PANIC, not return something empty"
    );
}

/// The recorded channel for `topic` as `(schema_name, schema_hash,
/// wire_fixed_size)`, or a panic naming every channel in the bag (a silent
/// `None` here would make the assertions vacuous).
///
/// All three, not just the size. The hash is NOT a live contest: the recorder
/// takes it unconditionally from `OutputMeta` (`resolve_recorded_topics` feeds
/// the file only into `build_workspace_schema_wire_sizes`).
/// Returning it here is defence in depth against a
/// resolution that consults the file for IDENTITY as well as
/// layout, the same shape the size arm below rejects.
fn recorded_channel_descriptor(reader: &BagReader, topic: &str) -> (String, u64, u32) {
    let channels = reader.channels().expect("read channels");
    let chan = channels
        .iter()
        .find(|c| c.topic == topic)
        .unwrap_or_else(|| {
            panic!(
                "the bag has no channel for {topic:?}; it carries {:?}",
                channels.iter().map(|c| &c.topic).collect::<Vec<_>>()
            )
        });
    let descriptor = chan.descriptor.unwrap_or_else(|| {
        panic!(
            "channel {topic:?} carries no `cerulion` schema descriptor \
             (schema_encoding {:?})",
            chan.schema_encoding
        )
    });
    (
        chan.schema_name.clone(),
        descriptor.schema_hash,
        descriptor.wire_fixed_size,
    )
}

/// The ONE log line carrying `needle` at `level`, or a panic.
///
/// Fields must be asserted on the LINE, not on the whole capture: three
/// independent whole-log searches are satisfied by a run in which the message
/// and the two numbers never met, and by a message demoted out of `WARN`
/// (a demoted cross-check keeps its wording, so the level is part of the
/// pin). This mirrors the sibling unit arm in `graph_cmd`, which selects the
/// line the same way and for the same stated reason.
fn log_line_at_level<'a>(log: &'a str, needle: &str, level: &str) -> &'a str {
    let mut hits = log
        .lines()
        .filter(|l| l.contains(needle) && l.split_whitespace().any(|tok| tok == level));
    let line = hits
        .next()
        .unwrap_or_else(|| panic!("no {level} line carrying {needle:?}; log:\n{log}"));
    assert!(
        hits.next().is_none(),
        "expected EXACTLY one {level} line carrying {needle:?} — the recorder \
         resolves each topic once at bring-up; log:\n{log}"
    );
    line
}

/// THE headline for item 2: a workspace with NO schema files still records the
/// node's fixed size into the bag.
///
/// The `demo` workspace ships no `schemas/` directory at all, so the file half
/// of the resolution has nothing to offer and the node is the only source:
/// `RecordedWireSize::Producer`, carried through `resolve_recorded_topics` →
/// `topics.json` → bagd → the channel descriptor.
///
/// SCOPE, stated because the reasoning is tempting and wrong: this does NOT
/// discriminate the fold's built-in-tier skip (`SchemaOrigin::Builtin =>
/// continue`). The built-in `geometry_msgs/Vector3` is ALSO 24, so serving the
/// built-in tier would resolve to the same number by a different route. What
/// this arm discriminates is 24 vs 0 — that the node half reaches the bag at
/// all, which is the composition the node-derived size changed and which `0` (the
/// descriptor's "nobody could say") makes attributable.
///
/// Hand oracle, not a read-back: 24 is `3 × float64` computed from the `.msg`.
#[test]
#[serial]
fn record_e2e_channel_wire_size_comes_from_the_node_when_the_workspace_cannot_size_it() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2ews");
    let (bag, stderr_path) = record_until_finalized_with_log(tmp.path());

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the Ctrl-C teardown must FINALIZE the bag, got {completeness:?}"
    );
    let log = strip_ansi(&read_file(&stderr_path));
    let (schema_name, schema_hash, wire_fixed_size) =
        recorded_channel_descriptor(&reader, "/rece2ews/ticker/cmd");
    assert_eq!(
        wire_fixed_size, NODE_VECTOR3_WIRE_FIXED_SIZE,
        "the bag channel must carry the PRODUCING NODE's fixed wire size; \
         log:\n{log}"
    );
    assert_eq!(schema_name, "geometry_msgs/Vector3");
    assert_eq!(
        schema_hash,
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        "and the node's own wire identity (the half the recorder always \
         takes from the node) — the sibling arm asserts this AGAINST a \
         disagreeing file, so the two together say the file moves neither"
    );
    // Nothing disagreed, so nothing may be reported as disagreeing — without
    // this the sibling arm's warn assertion could be satisfied by a recorder
    // that warns unconditionally. PREMISE first: an EMPTY capture would
    // satisfy the negative while proving nothing, and this is the one
    // assertion in the pair whose whole job is to be the control.
    assert!(
        log.contains("recording complete — bag finalized"),
        "the capture must be the whole run's log, or the negative below is \
         vacuous; log:\n{log}"
    );
    assert!(
        !log.contains("DIFFERENT wire layouts"),
        "a workspace with no schema files has nothing to disagree WITH; log:\n{log}"
    );
}

/// The other half: when the workspace file sizes the port DIFFERENTLY, the
/// NODE still wins in the bag — and the disagreement is reported.
///
/// Both numbers are asserted, and that is the point. The size alone cannot
/// distinguish this arm from the one above (both record 24), so the file being
/// consulted-and-overruled is proven by the warn, which carries BOTH sizes as
/// structured fields. A recorder that let the file win writes 16; one that
/// never read the file writes 24 but says nothing.
#[test]
#[serial]
fn record_e2e_channel_wire_size_comes_from_the_node_when_the_workspace_file_disagrees() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "rece2ewd");
    let schemas = tmp.path().join("schemas/geometry_msgs/msg");
    std::fs::create_dir_all(&schemas).unwrap();
    std::fs::write(schemas.join("Vector3.msg"), DISAGREEING_SHADOW_VECTOR3).unwrap();

    let (bag, stderr_path) = record_until_finalized_with_log(tmp.path());

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the Ctrl-C teardown must FINALIZE the bag, got {completeness:?}"
    );
    // Strip ONCE, at the capture, so every assertion below reads plain text
    // and none can be quietly answered by styling.
    let log = strip_ansi(&read_file(&stderr_path));
    let (schema_name, schema_hash, wire_fixed_size) =
        recorded_channel_descriptor(&reader, "/rece2ewd/ticker/cmd");
    assert_eq!(
        wire_fixed_size, NODE_VECTOR3_WIRE_FIXED_SIZE,
        "the NODE's fixed size must reach the bag even when the workspace file \
         claims another; log:\n{log}"
    );
    // The file moved NEITHER descriptor field. The hash was never a live
    // contest — `resolve_recorded_topics` takes it straight from `OutputMeta`
    // and feeds the file only into the wire-size fold — so this is a forward
    // regression guard: the shadow IS a second, differently-hashing definition
    // sitting right there in the workspace, and the assertion says the
    // recorder does not consult it for identity, whatever it consults for
    // layout.
    assert_eq!(schema_name, "geometry_msgs/Vector3");
    assert_eq!(
        schema_hash,
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        "the recorded hash must still be the NODE's, not the shadow's; \
         log:\n{log}"
    );
    // The file really was read, and really was overruled: both sizes ride
    // structured fields (the house logging rule), so they are greppable.
    let warn = log_line_at_level(&log, "DIFFERENT wire layouts", "WARN");
    for want in [
        "topic=/rece2ewd/ticker/cmd".to_string(),
        format!("node_wire_fixed_size={NODE_VECTOR3_WIRE_FIXED_SIZE}"),
        format!("file_wire_fixed_size={DISAGREEING_SHADOW_WIRE_FIXED_SIZE}"),
    ] {
        assert!(
            has_field_token(warn, &want),
            "the cross-check line must carry {want:?} as a whole field token \
             — an operator greps by key, and a value read off ANOTHER line \
             proves nothing about this one; line:\n{warn}"
        );
    }
}
