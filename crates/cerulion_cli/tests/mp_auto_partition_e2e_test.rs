// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for the LITERAL multi-process
//! default + the `graph partition` verb over the REAL binary — the full
//! launch story on an UNPARTITIONED 3-node chain (`ticker`(Period 50ms) →
//! `relay`(data-trigger) → `sink`(data-trigger)):
//!
//! 1. **The no-TTY floor, live** — `cerulion graph run apdemo --record=DIR`
//!    with stdin explicitly `/dev/null` (the binary's REAL
//!    `stdin().is_terminal()` probe — the main.rs consent wiring that
//!    cannot be tested purely — resolves false): the run derives the
//!    process-per-node partition IN-MEMORY, the floor notice lands in the
//!    child log naming `--yes` AND `--single-process`, the graph file stays
//!    BYTE-UNTOUCHED (no `.bak`), and the run is REALLY multi-process — 3
//!    `run-worker` children observable under the supervisor, the GO
//!    breadcrumb fires, and the multi-process recording captures the
//!    DERIVED split (rank manifests `rank0=[ticker]`, `rank1=[relay]`,
//!    `rank2=[sink]`) with real frames (delivery accounting) — the
//!    binary-level pin of the record×in-memory-mp composition.
//!    It also covers the embed: the bag's `graph.yaml` attachment must carry the
//!    partition the run EXECUTED (not the file's absence of one), must AGREE
//!    with the per-rank manifests, and must survive the exact
//!    `parse_graph` + `validate_graph` gauntlet `cerulion replay` puts it
//!    through. This arm is where that composition is observable at all: the
//!    engine-side pins call `prepare_recording_inputs` directly and so cannot
//!    see the supervisor handing it the wrong config.
//!
//!    - *(1b) the control + the monolith half* — `--single-process
//!      --record` on the same unpartitioned graph (written with NO `prefix:`
//!      line) embeds a graph with NO `process_groups:`, because that is what it
//!      ran; and its prefix — absent on disk, resolved at run time — is the one
//!      every recorded channel is named under. Covers `run_graph_recording`,
//!      the path arm 1 does not reach.
//! 2. **Persist then respect** — `cerulion graph partition apdemo --yes`
//!    writes the block (re-parse == the derived groups; `.bak` carries the
//!    original bytes), and a second `graph run` RESPECTS it: supervisor GO
//!    breadcrumb present, NO auto-partition notice of any kind (the absence
//!    pin — a persisted partition must not re-derive or re-notice).
//! 3. **The opt-out** — `graph run --single-process` on the unpartitioned
//!    twin runs the MONOLITH: zero `run-worker` children across the window,
//!    no GO breadcrumb, the skip notice present, file untouched.
//!
//! Harness cribs `mp_record_e2e_test.rs` (tempdir workspace, PREBUILT fixture
//! cdylibs — no in-test cargo build, redirected child logs, bounded waits,
//! `BagdGuard` grandchild reaping, `#[serial]`, unique prefixes) and
//! `mp_supervisor_box_test.rs` (pgrep-based worker-pid observation).
//! Prerequisites (the repo's fixture pattern — PANICS with the instruction if
//! missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]` (NOT linux-only): the multi-process
//! supervisor is real on macOS, and the auto-partition default is Unix-wide.

#![cfg(unix)]

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use serial_test::serial;

// The shared mp record-harness module, for the ONE helper this file needs from
// it: the mid-run bag poll that replaced this file's fixed recording windows.
// Imported by name (not `*`) because this file carries its own `wait_for_bag` /
// `read_manifest` / `ChildGuard` and they must keep winning.
mod mp_support;
use mp_support::{wait_for_bag_state, RECORDED_WINDOW_BOUNDARIES, RECORDED_WINDOW_TIMEOUT};

/// SIGKILL + reap on drop so a panicking test never leaks the child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// bagd GRANDCHILD leak guard (the `graph_record_e2e_test` pattern): bagd runs
/// in its OWN process group, so killing the supervisor on a mid-window panic
/// orphans it. The relative `--out recordings/apdemo_` cmdline form is unique
/// to THIS file's graph name.
struct BagdGuard;
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep")
            .args(["-f", "bagd --out recordings/apdemo_"])
            .output()
        {
            for pid in String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<i32>().ok())
            {
                // SAFETY: killpg(2) on the grandchild's own process group
                // (pgid == pid — spawned with process_group(0)); no memory is
                // touched. ESRCH after a clean exit is the expected no-op.
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

/// The platform cdylib filename for a crate/node name.
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// A prebuilt fixture cdylib by crate name. PANICS with the build instruction
/// if missing (the repo's fixture pattern).
fn fixture_cdylib(crate_name: &str) -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

/// Hand-build the UNPARTITIONED workspace in `root`: the same 3-node chain as
/// `mp_record_e2e_test`'s `mpdemo` but with NO `process_groups:` block — the
/// graph the default derives a partition FOR. Returns the graph
/// file's exact bytes (the byte-untouched oracles diff against them).
fn build_unpartitioned_workspace(root: &Path, prefix: &str) -> String {
    build_unpartitioned_workspace_opt_prefix(root, Some(prefix))
}

/// The same workspace with the `prefix:` line OPTIONAL. `None` is the
/// hand-written / earlier graph shape — `graph_read` (`parse_graph_raw`)
/// leaves the absent prefix absent and `graph_run` resolves it at run time, so
/// the on-disk file and the executed config genuinely differ on a field that
/// NAMES every recorded channel.
fn build_unpartitioned_workspace_opt_prefix(root: &Path, prefix: Option<&str>) -> String {
    build_workspace_with_sink(root, prefix, "test_node_macro_data_trigger_cdylib")
}

/// The same 3-node chain with the `sink` node TYPE staged from a
/// caller-chosen fixture, so the block-co-location arm can make `sink` a
/// `block` consumer while every other arm keeps the plain data-trigger sink.
///
/// The chain's SHAPE is deliberately identical (same graph name, same topics,
/// same node ids) — the only difference between a run that works and the run
/// that dies when its `block` edge is split is one `#[input(...)]` attribute in the staged source,
/// which is exactly the claim the arm makes.
fn build_workspace_with_sink(root: &Path, prefix: Option<&str>, sink_fixture: &str) -> String {
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
    let sources = [
        ("ticker", "test_node_macro_period_cdylib"),
        ("relay", "test_node_macro_data_trigger_cdylib"),
        ("sink", sink_fixture),
    ];
    for (node_type, fixture) in sources {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    let prefix_line = match prefix {
        Some(p) => format!("prefix: {p}\n"),
        None => String::new(),
    };
    let yaml = format!(
        "name: apdemo\n\
         {prefix_line}\
         nodes:\n\
         - id: ticker\n\
         \x20 type: ticker\n\
         \x20 inputs: []\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: relay\n\
         \x20 type: relay\n\
         \x20 inputs:\n\
         \x20 - name: trigger_in\n\
         \x20\x20\x20 source: ticker/cmd\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: sink\n\
         \x20 type: sink\n\
         \x20 inputs:\n\
         \x20 - name: trigger_in\n\
         \x20\x20\x20 source: relay/cmd\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n"
    );
    std::fs::write(root.join("graphs/apdemo.yaml"), &yaml).unwrap();
    yaml
}

/// Spawn `cerulion graph run apdemo [extra...]` with stdin EXPLICITLY
/// `/dev/null` — the deterministic no-TTY probe (the binary's real
/// `stdin().is_terminal()` consent wiring resolves false). Child stdout +
/// stderr are redirected to files (readable while the child runs).
fn spawn_graph_run(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf, PathBuf) {
    spawn_graph_run_with_env(root, extra, &[])
}

/// [`spawn_graph_run`] plus extra process env — the run-directory writer needs `CERULION_HOME`
/// redirected so the run directory the REAL binary writes lands under the
/// test's tempdir rather than the developer's home.
fn spawn_graph_run_with_env(
    root: &Path,
    extra: &[&str],
    env: &[(&str, &Path)],
) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    for (k, v) in env {
        cmd.env(k, v);
    }
    let child = cmd
        // NO `--no-validate`: several arms below add `--record` through
        // `extra`, and a recording made with the schema checks off is refused
        // at parse. The `apdemo` workspace validates, so the flag bought this
        // harness nothing anyway.
        .args(["graph", "run", "apdemo"])
        .args(extra)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    (ChildGuard(child), stdout_path, stderr_path)
}

/// stdout + stderr merged (tracing's writer choice must not decide a pin).
fn merged_log(stdout_path: &Path, stderr_path: &Path) -> String {
    format!("{}\n{}", read_file(stdout_path), read_file(stderr_path))
}

/// The `run-worker` children of a supervisor pid (crib of
/// `mp_supervisor_box_test::worker_pids`).
fn worker_pids(supervisor_pid: u32) -> Vec<u32> {
    let out = Command::new("pgrep")
        .args(["-P", &supervisor_pid.to_string(), "-f", "run-worker"])
        .output();
    match out {
        Ok(o) => String::from_utf8_lossy(&o.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<u32>().ok())
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Bounded poll until the supervisor has ≥ `n` worker children (the
/// READY-gated spawn brings them up sequentially). Returns the last observed
/// set.
fn wait_for_workers(supervisor_pid: u32, n: usize, deadline: Duration) -> Vec<u32> {
    let start = Instant::now();
    loop {
        let pids = worker_pids(supervisor_pid);
        if pids.len() >= n || start.elapsed() >= deadline {
            return pids;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Bounded poll until `path`'s contents contain `needle`.
fn wait_for_log(path: &Path, needle: &str, deadline: Duration) -> bool {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return true;
        }
        if start.elapsed() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Block until the recordings dir contains a `.mcap` or `timeout` elapses.
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

/// A bag rank manifest's node ids (crib of `mp_record_e2e_test`).
fn read_manifest(reader: &BagReader, rank: u32) -> Option<Vec<String>> {
    let att = reader
        .attachment(&format!("__cerulion/trace_manifest_rank{rank}.json"))
        .expect("read attachments")?;
    let v: serde_json::Value = serde_json::from_slice(&att.data).expect("manifest json parses");
    Some(
        v["node_ids"]
            .as_array()
            .expect("manifest node_ids array")
            .iter()
            .map(|s| s.as_str().expect("node id string").to_string())
            .collect(),
    )
}

/// The bag's `graph.yaml` attachment, parsed through the EXACT gauntlet
/// `cerulion replay` puts it through (`parse_graph` then `validate_graph` —
/// `replay_cmd.rs` step 4). Returning the parsed config means every caller's
/// assertions are made against what replay would actually reconstruct, and a
/// bag that replay would refuse fails HERE with the reason named, rather than
/// passing a text-only `contains` check and dying at playback.
fn read_embedded_graph(reader: &BagReader) -> (String, cerulion_core::graph::GraphConfig) {
    let att = reader
        .attachment("graph.yaml")
        .expect("read attachments")
        .expect("the graph.yaml attachment must be present");
    let text = String::from_utf8(att.data).expect("the embedded graph must be UTF-8");
    let config = cerulion_core::graph::parse_graph(&text).unwrap_or_else(|e| {
        panic!("replay parses the embed with parse_graph; it failed: {e}\n{text}")
    });
    cerulion_core::graph::validate_graph(&config).unwrap_or_else(|e| {
        panic!(
            "the embedded graph must pass the SAME validation replay runs, or the bag is exit-2 \
             unreplayable ('corrupt or hand-edited'): {e}\n{text}"
        )
    });
    (text, config)
}

/// The embed's prefix must be the one the recording NAMED ITS CHANNELS
/// under — the cross-artifact oracle that does not need the host's name.
///
/// `resolve_recorded_topics` derives every channel from the EFFECTIVE
/// `config.prefix`, while replay re-derives produced/consumed topic names from
/// whatever the embed says (filling an absent prefix from the REPLAY host). So
/// if these two disagree, `classify_topics` matches nothing and the bag is a
/// `BagGraphMismatch` on any machine but the recorder's.
/// `topics` is every channel in the bag; the recorder's OWN reserved channels
/// (`__cerulion/…` — the scheduler trace) are filtered here rather than at each
/// call site, since they are not graph topics and carry no prefix by design.
fn assert_embed_prefix_names_the_recorded_channels(
    embedded: &cerulion_core::graph::GraphConfig,
    topics: &[String],
    text: &str,
) {
    // The assertion must be made on the RAW TEXT, not on `embedded.prefix`.
    // `read_embedded_graph` parses with `parse_graph` — the same parser replay
    // uses — which FILLS an absent prefix from the local hostname. On a test
    // host that is also the recording host, that fill reproduces the right
    // answer, so a parsed-value check passes against an embed carrying no
    // prefix at all and pins nothing. (Stripping the prefix
    // from the render passed this arm until the text check was added.) The
    // cross-machine hazard is precisely that a DIFFERENT host fills it
    // differently, so what must be pinned is that the bytes carry it.
    assert!(
        text.lines().any(|l| l.starts_with("prefix:")),
        "the embedded graph must literally CARRY a `prefix:` line — an absent one is re-filled \
         from the REPLAY host's hostname, so the bag would replay only where it was \
         recorded\n{text}"
    );
    assert!(
        !embedded.prefix.is_empty(),
        "the embedded prefix must be non-empty\n{text}"
    );
    let want = format!("/{}/", embedded.prefix);
    let graph_topics: Vec<&String> = topics
        .iter()
        .filter(|t| !t.starts_with(cerulion_bag::RESERVED_PREFIX))
        .collect();
    assert!(
        !graph_topics.is_empty(),
        "precondition: the bag must carry recorded GRAPH channels to compare against (saw only \
         reserved ones: {topics:?})"
    );
    for t in graph_topics {
        assert!(
            t.starts_with(&want),
            "recorded channel `{t}` is not named under the embedded graph's prefix `{}` — the \
             bag's channels and its graph disagree about topic names\n{text}",
            embedded.prefix
        );
    }
}

/// The floor-notice marker (a substring of `partition_emit`'s warn — pinned
/// loosely enough to survive tracing-field formatting, tightly enough that
/// only the auto-partition floor emits it).
const IN_MEMORY_NOTICE_MARKER: &str = "derived process groups IN-MEMORY";
/// The supervisor's deployment-live breadcrumb (the mp-path marker).
/// `tracing`'s fmt layer wraps a field's NAME and its `=` in ANSI escapes (its
/// `ansi` default is a compile-time feature, NOT a tty probe), so
/// `topic=/x` is not a substring of the raw capture at all and a `key=value`
/// assertion against it is SILENTLY UNSATISFIABLE — it fails for a reason that
/// has nothing to do with the behaviour under test. Third copy of this helper
/// in the tree (see `flashback_argv_e2e_test` and `mp_split_pair_e2e_test`);
/// separate test binaries cannot share it.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && i + 1 < bytes.len() && bytes[i + 1] == b'[' {
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1;
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    // Every dropped sequence is whole and ASCII, so what remains is still
    // valid UTF-8; the lossy decode is the belt.
    String::from_utf8_lossy(&out).into_owned()
}

/// `key=value` as a whole WHITESPACE token, ANSI stripped first.
fn has_field(log: &str, field: &str) -> bool {
    strip_ansi(log).split_whitespace().any(|t| t == field)
}

const GO_MARKER: &str = "GO signaled; deployment live";

/// (1) The LITERAL default on a no-TTY run: in-memory mp floor + real
/// supervisor/workers + the mp recording of the DERIVED split.
#[test]
#[serial]
fn no_tty_default_derives_in_memory_mp_and_records_mp_shaped() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_unpartitioned_workspace(tmp.path(), "apda");
    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), &["--record=recordings"]);
    let _bagd_guard = BagdGuard;
    let sup_pid = guard.0.id();

    // The mp bring-up completes: bag file appears (planning + 3 worker spawns
    // + bagd handshake precede it — generous bound for slow CI VMs).
    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag (mp bring-up failed?)\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });

    // REALLY multi-process: the supervisor carries 3 `run-worker` children
    // (one per derived process-per-node group).
    let workers = wait_for_workers(sup_pid, 3, Duration::from_secs(60));
    assert!(
        workers.len() >= 3,
        "the derived process-per-node partition must spawn 3 workers, observed {workers:?}\n\
         stdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // A healthy window, WAITED FOR rather than slept: the acceptance below reads
    // all three derived ranks out of the bag and asserts the ticker's frames, so
    // the window ends when that evidence is recorded. A mid-run read sees only
    // flushed chunks, so this is a lower bound on the finalized bag.
    wait_for_bag_state(
        &bag,
        "all three derived ranks' boundary streams and the ticker's frames",
        RECORDED_WINDOW_TIMEOUT,
        |snap| {
            (0..3).all(|rank| snap.boundaries_for_rank(rank) >= RECORDED_WINDOW_BOUNDARIES)
                && snap.frames_on("/apda/ticker/cmd") > 0
        },
    );
    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "the in-memory mp run must exit 0 on Ctrl-C, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // The floor notice: in-memory, file untouched, BOTH escape hatches named.
    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        log.contains(IN_MEMORY_NOTICE_MARKER),
        "the no-TTY floor notice must land in the child log; log:\n{log}"
    );
    assert!(
        log.contains("--yes") && log.contains("--single-process"),
        "the floor notice names both escape hatches; log:\n{log}"
    );
    // The supervisor path really ran.
    assert!(
        log.contains(GO_MARKER),
        "the GO breadcrumb proves the supervisor deployment went live; log:\n{log}"
    );
    // This graph is CORRECT, and `relay` and `sink` each read a topic a sibling
    // worker produces. Each worker validates only its own slice, twice (the
    // worker entry, then the runtime build), and must not report that edge as
    // a possible typo either time. The GO marker above is what keeps this
    // absence meaningful: the workers were built, so the validations ran.
    assert!(
        !log.contains("matches no declared output"),
        "a worker must not report a sibling-produced source as a possible typo; log:\n{log}"
    );

    // The required floor (never mutate): the graph file is byte-untouched, no backup churn.
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        original_yaml,
        "a no-TTY run must NEVER mutate the graph file"
    );
    assert!(
        !tmp.path().join("graphs/apdemo.yaml.bak").exists(),
        "no .bak on the floor path"
    );

    // The mp recording captured the DERIVED split: per-rank manifests
    // in pipeline rank order (grp_ticker, grp_relay, grp_sink — singletons).
    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the mp teardown must FINALIZE the bag, got {completeness:?}"
    );
    assert_eq!(
        read_manifest(&reader, 0).expect("rank0 manifest"),
        vec!["ticker".to_string()],
        "rank 0 = the derived grp_ticker singleton"
    );
    assert_eq!(
        read_manifest(&reader, 1).expect("rank1 manifest"),
        vec!["relay".to_string()],
        "rank 1 = the derived grp_relay singleton"
    );
    assert_eq!(
        read_manifest(&reader, 2).expect("rank2 manifest"),
        vec!["sink".to_string()],
        "rank 2 = the derived grp_sink singleton"
    );

    // Delivery accounting: the recorded topic carries real frames.
    let frames = msgs
        .iter()
        .filter(|m| m.topic == "/apda/ticker/cmd")
        .count();
    assert!(
        frames > 0,
        "the bag must contain the ticker's published frames (got {frames}; a \
         zero-frame recording is a silent failure)"
    );

    // ---- The bag's graph is the graph that RAN. ----
    //
    // This is the arm no unit test can stand in for. The engine-side pins call
    // `prepare_recording_inputs` directly, so they prove the SEAM renders
    // whatever config it is handed — they cannot see the supervisor handing it
    // the WRONG one (a pre-preflight config, or a re-read of the file). Only a
    // real no-TTY `graph run --record` composes the derivation with the
    // multi-process recording, and that composition is the defect.
    //
    // The assertion is meaningful precisely BECAUSE the assertions above proved
    // the on-disk file is byte-untouched and therefore still carries no
    // `process_groups:` — so the two candidate sources for this attachment give
    // DIFFERENT answers here, and only one of them matches the run.
    let (embed_text, embedded) = read_embedded_graph(&reader);
    assert!(
        !original_yaml.contains("process_groups"),
        "precondition: the on-disk graph must NOT declare process_groups, or embedding either \
         source would pass and this arm would prove nothing"
    );

    // The executed partition, against a HAND oracle (the derived
    // process-per-node split) — not a re-read of anything the recorder wrote.
    let mut embedded_groups: Vec<(String, Vec<String>)> = embedded
        .process_groups
        .iter()
        .map(|(g, ns)| (g.clone(), ns.clone()))
        .collect();
    embedded_groups.sort();
    assert_eq!(
        embedded_groups,
        vec![
            ("grp_relay".to_string(), vec!["relay".to_string()]),
            ("grp_sink".to_string(), vec!["sink".to_string()]),
            ("grp_ticker".to_string(), vec!["ticker".to_string()]),
        ],
        "the bag's graph must carry the process_groups the run EXECUTED (the derived \
         process-per-node split), not the unpartitioned file's absence of them\n{embed_text}"
    );

    // The cross-artifact agreement this arm pins: the graph
    // attachment and the per-rank manifests must describe ONE run. Each
    // rank's manifest node set must be exactly some group of the embedded
    // partition, and every group must be claimed by exactly one rank.
    let mut manifest_sets: Vec<Vec<String>> = (0..3)
        .map(|r| read_manifest(&reader, r).expect("rank manifest"))
        .collect();
    manifest_sets.sort();
    let mut group_sets: Vec<Vec<String>> =
        embedded_groups.iter().map(|(_, ns)| ns.clone()).collect();
    group_sets.sort();
    assert_eq!(
        manifest_sets, group_sets,
        "the bag's graph and its per-rank trace manifests must describe the SAME run — this is \
         the self-contradiction this arm rejects\n{embed_text}"
    );

    // The prefix half, tied to the bag's own channels.
    let topics: Vec<String> = {
        let mut t: Vec<String> = msgs.iter().map(|m| m.topic.clone()).collect();
        t.sort();
        t.dedup();
        t
    };
    assert_embed_prefix_names_the_recorded_channels(&embedded, &topics, &embed_text);
}

/// (1b) The CONTROL + the MONOLITH half: a `--single-process --record` run
/// of the SAME unpartitioned graph must embed a graph with NO `process_groups:`
/// — because that is what it EXECUTED.
///
/// Two things this arm does that (1) cannot:
///
/// * **It is the control.** Without it, "the embed carries the derived groups"
///   is satisfied by an implementation that always writes a partition. Here the
///   run really is single-process, so the correct embed is the empty one.
/// * **It covers the other code path.** (1) exercises
///   `start_supervisor_recording`; this exercises `run_graph_recording`, whose
///   config is CLONED before being moved into the runtime build. Nothing in (1)
///   would notice that clone going stale or being taken from the wrong value.
///
/// It also carries the prefix divergence end-to-end: the graph file is written with
/// NO `prefix:` line, so the embed's prefix can only have come from the run's
/// own resolution — and every recorded channel is named under it.
#[test]
#[serial]
fn a_single_process_record_embeds_the_unpartitioned_graph_it_actually_ran() {
    let tmp = tempfile::tempdir().unwrap();
    // NO `prefix:` line — the hand-written graph shape (see the builder's doc).
    let original_yaml = build_unpartitioned_workspace_opt_prefix(tmp.path(), None);
    assert!(
        !original_yaml.contains("prefix:"),
        "precondition: the on-disk graph must carry NO prefix, or the prefix half of this arm \
         proves nothing"
    );
    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), &["--single-process", "--record=recordings"]);
    let _bagd_guard = BagdGuard;

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag (single-process record bring-up failed?)\n\
             stdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    // Waited for on the recorded content the channel assertions below read,
    // instead of a fixed window.
    wait_for_bag_state(
        &bag,
        "a recorded user topic carrying frames",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.user_topics_with_frames(1) >= 1,
    );
    send_sigint(guard.0.id());
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("the single-process record run did not exit after SIGINT");
    assert!(
        status.success(),
        "the single-process record run must exit 0 on Ctrl-C, got {status:?}\n\
         stdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    // It really was the monolith: no supervisor deployment went live.
    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        !log.contains(GO_MARKER),
        "--single-process must NOT reach the supervisor; log:\n{log}"
    );

    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the single-process teardown must FINALIZE the bag, got {completeness:?}"
    );
    let (embed_text, embedded) = read_embedded_graph(&reader);

    // THE CONTROL: what ran was unpartitioned, so what is embedded must be too.
    assert!(
        embedded.process_groups.is_empty(),
        "a --single-process run executed NO partition, so its bag must not claim one — \
         otherwise 'the embed carries the derived groups' is satisfied by always writing a \
         partition\n{embed_text}"
    );
    // And it must NOT be the file verbatim either: the file has no prefix.
    let topics: Vec<String> = {
        let mut t: Vec<String> = msgs.iter().map(|m| m.topic.clone()).collect();
        t.sort();
        t.dedup();
        t
    };
    assert_embed_prefix_names_the_recorded_channels(&embedded, &topics, &embed_text);
    // The file itself stays as authored — recording never writes it.
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        original_yaml,
        "recording must never mutate the user's graph file"
    );
}

/// (2) Persist via the verb, then the run RESPECTS the file — no notice.
#[test]
#[serial]
fn partition_yes_persists_then_run_respects_without_notice() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_unpartitioned_workspace(tmp.path(), "apdb");

    // `cerulion graph partition apdemo --yes` — the persisted consent arm
    // over the REAL binary (no transport; bounded by construction).
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "partition", "apdemo", "--yes"])
        .current_dir(tmp.path())
        .stdin(Stdio::null())
        .output()
        .expect("run cerulion graph partition");
    assert!(
        out.status.success(),
        "graph partition --yes must exit 0, got {:?}\nstdout:\n{}\nstderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("Partition written to"),
        "the verb announces the write; stdout:\n{stdout}"
    );

    // The file gained EXACTLY the derived block; the backup is the original.
    let written = std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable");
    let config = cerulion_core::graph::parse_graph_raw(&written).expect("written graph parses");
    let groups: Vec<(String, Vec<String>)> = config
        .process_groups
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    assert_eq!(
        groups,
        vec![
            ("grp_ticker".to_string(), vec!["ticker".to_string()]),
            ("grp_relay".to_string(), vec!["relay".to_string()]),
            ("grp_sink".to_string(), vec!["sink".to_string()]),
        ],
        "the written block is the derived process-per-node partition in rank order"
    );
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml.bak")).expect("bak"),
        original_yaml,
        "the .bak carries the pre-write bytes"
    );

    // Second run: the hand-persisted (formerly derived) block is RESPECTED —
    // supervisor path, NO auto-partition notice of any kind.
    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &[]);
    let sup_pid = guard.0.id();
    // GO is a `tracing::info!` → STDERR (init_logging writes there so stdout
    // stays clean data) — poll stderr primary, short stdout fallback.
    assert!(
        wait_for_log(&stderr_path, GO_MARKER, Duration::from_secs(90))
            || wait_for_log(&stdout_path, GO_MARKER, Duration::from_secs(1)),
        "the respected process_groups run must reach the supervisor GO;\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    // No window: this arm records nothing and asserts only the exit code, the
    // ABSENCE of every auto-partition notice, and that the file was not
    // rewritten. GO — the thing it does wait on — is already awaited above, and
    // `graph run` installs its signal handler before bring-up, so there is
    // nothing left for a sleep to wait for.
    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(status.success(), "respected-run exit 0, got {status:?}");

    let log = merged_log(&stdout_path, &stderr_path);
    // THE ABSENCE PIN: a declared partition derives nothing and notices
    // nothing (no in-memory floor, no decline, no auto-partition breadcrumb).
    assert!(
        !log.contains(IN_MEMORY_NOTICE_MARKER) && !log.contains("auto-partition"),
        "a run on a declared process_groups block must emit NO auto-partition \
         notice; log:\n{log}"
    );
    // And the file was not touched again.
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        written,
        "the respected run must not rewrite the file"
    );
}

/// (3) `--single-process` on the unpartitioned twin = the plain monolith:
/// no derivation, no workers, no GO.
#[test]
#[serial]
fn single_process_opt_out_runs_monolith_with_no_workers() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_unpartitioned_workspace(tmp.path(), "apdc");
    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &["--single-process"]);
    let sup_pid = guard.0.id();

    // The skip notice is the positive liveness marker for this path. It is a
    // `tracing::info!` → STDERR (init_logging writes there so stdout stays clean
    // data) — poll stderr primary, short stdout fallback.
    assert!(
        wait_for_log(&stderr_path, "running the single-process monolith", Duration::from_secs(60))
            || wait_for_log(
                &stdout_path,
                "running the single-process monolith",
                Duration::from_secs(1)
            ),
        "--single-process must announce the skipped auto-partition default;\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // Across a 3s window: NEVER a run-worker child (monolith = one process).
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(3) {
        let pids = worker_pids(sup_pid);
        assert!(
            pids.is_empty(),
            "--single-process must spawn NO workers, observed {pids:?}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }

    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("monolith did not exit after SIGINT");
    assert!(
        status.success(),
        "monolith run must exit 0 on Ctrl-C, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        !log.contains(GO_MARKER),
        "the monolith never runs the supervisor GO; log:\n{log}"
    );
    assert!(
        !log.contains(IN_MEMORY_NOTICE_MARKER),
        "--single-process skips the derivation entirely — no in-memory notice; log:\n{log}"
    );
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        original_yaml,
        "the opt-out run never touches the file"
    );
}

/// (4) A run REFUSED by a pre-run rejection must
/// leave the graph file BYTE-UNTOUCHED even under `--auto-partition --yes` —
/// otherwise the pre-flight writes the block + `.bak` and THEN dies at the
/// external-clock multi-process rejection (a failed run mutating the user's
/// file). Two arms over the real binary:
/// (a) `--time-source external` — the early external+Derive rejection (same
///     text as `resolve_deployment`'s: ONE error surface);
/// (b) `--record --time-source virtual` — the record clock guard (pins the
///     claim that it runs BEFORE the pre-flight can write).
#[test]
#[serial]
fn failed_pre_run_rejections_leave_the_graph_file_untouched() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_unpartitioned_workspace(tmp.path(), "apdd");
    let graph_path = tmp.path().join("graphs/apdemo.yaml");
    let bak_path = tmp.path().join("graphs/apdemo.yaml.bak");

    // (a) external clock: rejected BEFORE any consent/write.
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "graph",
            "run",
            "apdemo",
            "--no-validate",
            "--auto-partition",
            "--yes",
            "--time-source",
            "external",
        ])
        .current_dir(tmp.path())
        .stdin(Stdio::null())
        .output()
        .expect("run cerulion graph run (external)");
    assert!(
        !out.status.success(),
        "external + derived-mp must be refused, got {:?}",
        out.status
    );
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("cannot run under `--time-source external`"),
        "the refusal carries the ONE external-mp error surface; stderr:\n{stderr}"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        original_yaml,
        "a REFUSED run must never mutate the graph file (no block written, no .bak)"
    );
    assert!(!bak_path.exists(), "no .bak on the refused path");

    // (b) --record + virtual: the record guard fires before the pre-flight.
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "graph",
            "run",
            "apdemo",
            "--auto-partition",
            "--yes",
            "--record=recordings",
            "--time-source",
            "virtual",
        ])
        .current_dir(tmp.path())
        .stdin(Stdio::null())
        .output()
        .expect("run cerulion graph run (record+virtual)");
    assert!(
        !out.status.success(),
        "--record + virtual must be refused, got {:?}",
        out.status
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        original_yaml,
        "the record-guard refusal must also precede any file mutation"
    );
    assert!(!bak_path.exists(), "no .bak on the record-guard path");
}

/// The MULTI-PROCESS default (the shape whose supervisor arm
/// `return`s from inside the deployment match) writes and announces its run
/// directory, WITHOUT `--record`.
///
/// This is the arm no source walk can replace. A structural guard pins where
/// the call SITS; only a real run proves the supervisor path reaches it, that
/// the directory really lands with its four artifacts, that `run.json`
/// describes the DERIVED partition the file does not carry, and that the whole
/// thing is gone at exit. This exists to catch a descriptor block moved
/// below the dispatch: every multi-process run then gets nothing, while a
/// comment mentioning the call can still satisfy a text walk.
#[test]
#[serial_test::serial]
fn no_tty_default_mp_run_writes_and_announces_its_run_directory() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_unpartitioned_workspace(tmp.path(), "apdrd");
    let home = tmp.path().join("cerhome");
    std::fs::create_dir_all(&home).unwrap();

    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run_with_env(tmp.path(), &[], &[("CERULION_HOME", home.as_path())]);
    let sup_pid = guard.0.id();

    // REALLY multi-process (the supervisor arm, which returns from inside the
    // deployment match) — the precondition that makes this arm the one the
    // structural walk cannot stand in for.
    let workers = wait_for_workers(sup_pid, 3, Duration::from_secs(60));
    assert!(
        workers.len() >= 3,
        "expected the derived process-per-node partition to spawn 3 workers, observed \
         {workers:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // The run directory, found by WAITING for it (a bound in seconds; load can
    // delay a spawn, it cannot make an unwritten directory appear).
    let runs = home.join("runs");
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    let run_dir = loop {
        let found = std::fs::read_dir(&runs)
            .ok()
            .and_then(|d| d.filter_map(Result::ok).map(|e| e.path()).next());
        if let Some(dir) = found {
            if dir.join("run.json").exists() {
                break dir;
            }
        }
        assert!(
            std::time::Instant::now() < deadline,
            "a multi-process `graph run` (no --record) never wrote its run directory under \
             {}\nstdout:\n{}\nstderr:\n{}",
            runs.display(),
            read_file(&stdout_path),
            read_file(&stderr_path)
        );
        std::thread::sleep(Duration::from_millis(100));
    };

    for name in ["run.json", "graph.yaml", "env.json", "recorder.json"] {
        assert!(
            run_dir.join(name).exists(),
            "the run directory must carry `{name}`; got {:?}",
            std::fs::read_dir(&run_dir).map(|d| d
                .filter_map(Result::ok)
                .map(|e| e.file_name())
                .collect::<Vec<_>>())
        );
    }

    // End to end over the REAL binary: the run directory
    // describes the DERIVED partition while the graph file on disk does not.
    let embedded = std::fs::read_to_string(run_dir.join("graph.yaml")).unwrap();
    assert!(
        embedded.contains("process_groups:"),
        "the run directory must carry the partition the run EXECUTES; got:\n{embedded}"
    );
    assert!(
        !original_yaml.contains("process_groups:"),
        "the DISCRIMINATOR: the graph file carries no partition"
    );
    let manifest: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(run_dir.join("run.json")).unwrap()).unwrap();
    assert_eq!(
        manifest["partition"],
        serde_json::json!({"provenance": "derived-in-memory", "process_groups": true}),
        "run.json must name the provenance AND report the EXECUTED deployment"
    );
    assert_eq!(manifest["supervisor_pid"], serde_json::json!(sup_pid));

    // ANNOUNCED: the registry record points at this very directory. Gathered on
    // the process-global namespace (what the real binary publishes on) and
    // filtered by run_dir, so a co-tenant run cannot satisfy or break it.
    let want = run_dir.display().to_string();
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    loop {
        let gather = cerulion_core::transport::run_registry::gather_current_runs()
            .expect("gather the run registry");
        if let Some(rec) = gather.records.iter().find(|r| r.run_dir == want) {
            assert_eq!(rec.supervisor_pid, sup_pid);
            assert_eq!(
                rec.state,
                cerulion_core::transport::run_registry::RunState::Live
            );
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "the live mp run was never announced on /__cerulion/runs (saw {} other run(s))\
             \nstdout:\n{}\nstderr:\n{}",
            gather.records.len(),
            read_file(&stdout_path),
            read_file(&stderr_path)
        );
        std::thread::sleep(Duration::from_millis(100));
    }

    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "the run must exit 0 on Ctrl-C, got {status:?}"
    );

    assert!(
        !run_dir.exists(),
        "a clean exit must remove the run directory — leftovers are indistinguishable \
         from a SIGKILLed run's while being neither"
    );
}

// ==========================================================================
// A `block` graph RUNS on the default no-TTY multi-process path.
//
// This is the arm that regression-guards the exact user-visible break. If
// the derivation split every `block` edge (a no-costs
// baseline performing zero unions), `subgraph_for` would drop the foreign producer
// from the consumer's subgraph, and the consumer's worker would die at
// `GraphTopology::validate` blaming an external publisher the user does not
// have — surfacing only as "worker process for group '…' exited … before
// signaling READY".
//
// Fixture prerequisite (the repo's pattern — PANICS with the instruction):
//   cargo build -p test_node_macro_period_cdylib \
//               -p test_node_macro_data_trigger_cdylib \
//               -p test_node_macro_trigger_block_cdylib
//
// `test_node_macro_trigger_block_cdylib` exists because neither other block
// fixture is usable here: both are `#[cerulion_node(external)]` returning
// `ExternalSource::HostDriven`, which is refused at launch on the live
// path.
// ==========================================================================

/// The block fixture staged as the chain's `sink`.
const BLOCK_SINK_FIXTURE: &str = "test_node_macro_trigger_block_cdylib";

/// (4) THE REGRESSION GUARD. `ticker -> relay -> sink`, where `sink`'s trigger
/// input declares `backpressure = block`, run through the LITERAL default
/// (no flags, stdin `/dev/null`):
///
/// * the run reaches GO and exits 0 on Ctrl-C — i.e. no worker died at build;
/// * the derived partition put the `block` producer `relay` and its `block`
///   consumer `sink` in ONE worker's manifest, while the unconstrained
///   `ticker` stays its own worker; and
/// * the graph file is byte-untouched (the no-TTY floor is unchanged by this
///   feature).
#[test]
#[serial]
fn a_block_graph_runs_on_the_no_tty_default_and_co_locates_the_edge() {
    let tmp = tempfile::tempdir().unwrap();
    let original_yaml = build_workspace_with_sink(tmp.path(), Some("apdbk"), BLOCK_SINK_FIXTURE);
    let (mut guard, stdout_path, stderr_path) =
        spawn_graph_run(tmp.path(), &["--record=recordings"]);
    let _bagd_guard = BagdGuard;
    let sup_pid = guard.0.id();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag — the defect shape is a `block` consumer's \
             worker dying at graph build, which is exactly what this arm exists to \
             catch\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });

    // TWO workers, not three: the block edge is co-located, `ticker` is not.
    let workers = wait_for_workers(sup_pid, 2, Duration::from_secs(60));
    assert!(
        workers.len() >= 2,
        "expected at least 2 workers (the co-located block group + ticker), observed \
         {workers:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // Waited for on both ranks' recorded streams — the manifests and trace the
    // acceptance below reads — rather than a fixed window.
    wait_for_bag_state(
        &bag,
        "both ranks' boundary streams",
        RECORDED_WINDOW_TIMEOUT,
        |snap| (0..2).all(|rank| snap.boundaries_for_rank(rank) >= RECORDED_WINDOW_BOUNDARIES),
    );
    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "a `block` graph must run and exit 0 on the default mp path, got {status:?}\n\
         stdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        log.contains(GO_MARKER),
        "the supervisor deployment must go live; log:\n{log}"
    );
    // The earlier failure signature must be absent — a run that exits 0
    // with a dead worker would be a different bug, and this names the one we
    // fixed.
    assert!(
        !log.contains("before signaling READY"),
        "no worker may die before READY; log:\n{log}"
    );
    // The refusal's OWN sentence, not the substring it shares with the benign
    // "external topic (no in-graph producer)" provisioning INFO that a genuinely
    // cross-worker `drop_oldest` edge legitimately emits.
    assert!(
        !log.contains("this scheduler can only defer producers in THIS"),
        "no worker may refuse a `block` input for a missing producer; log:\n{log}"
    );
    // And the co-location was ANNOUNCED (loud-over-silent: the derived shape
    // differs from the documented process-per-node baseline).
    assert!(
        log.contains("co-locating this topic's producer(s) and `block` consumer(s)"),
        "the co-location must be announced; log:\n{log}"
    );

    // THE SHAPE PIN, read off the recording's per-rank manifests: `relay` and
    // `sink` are in ONE worker.
    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the mp teardown must FINALIZE the bag, got {completeness:?}"
    );
    assert_eq!(
        read_manifest(&reader, 0).expect("rank0 manifest"),
        vec!["ticker".to_string()],
        "rank 0 = the unconstrained ticker singleton"
    );
    assert_eq!(
        read_manifest(&reader, 1).expect("rank1 manifest"),
        vec!["relay".to_string(), "sink".to_string()],
        "rank 1 = the co-located `block` group: the producer AND its block consumer"
    );
    assert!(
        read_manifest(&reader, 2).is_none(),
        "there must be no third rank — the block edge is co-located"
    );

    // The no-TTY floor is unchanged by this feature.
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        original_yaml,
        "a no-TTY run must NEVER mutate the graph file"
    );
    assert!(!tmp.path().join("graphs/apdemo.yaml.bak").exists());
}

/// Under flow mode, a HAND-WRITTEN
/// partition that splits a CREDITABLE `block` edge RUNS.
///
/// The fixture is the plain block chain (`ticker -> relay -> sink`, `sink.trigger_in` =
/// `block`, split `front:[ticker,relay] / back:[sink]`) and that is the point:
/// `relay/cmd` has exactly ONE in-graph producer and exactly one consumer,
/// which declares `block` — the shape `credit_edges_for` mints a cross-process
/// credit word for. Without cross-process credit this partition is refused pre-spawn; that
/// refusal is a statement about the WORD (a process-local heap cell one
/// address space cannot show another), and flow mode changes which words exist.
///
/// This is the ONLY arm that proves a credited split runs through the REAL
/// supervisor — every other credit pin stops at the plan-time verdict. It asserts
/// the three things that verdict cannot: the deployment reaches GO, the run
/// exits 0 on the production Ctrl-C, and the log NAMES the credited edge (so
/// an operator can tell a credited split from a co-located one).
#[test]
#[serial]
fn a_hand_written_split_of_a_creditable_block_edge_runs_pre_spawn() {
    let tmp = tempfile::tempdir().unwrap();
    let base = build_workspace_with_sink(tmp.path(), Some("apdbs"), BLOCK_SINK_FIXTURE);
    let split = base.replace(
        "nodes:\n",
        "process_groups:\n  front: [ticker, relay]\n  back: [sink]\nnodes:\n",
    );
    assert!(
        split.contains("process_groups:"),
        "fixture must carry a partition"
    );
    std::fs::write(tmp.path().join("graphs/apdemo.yaml"), &split).unwrap();

    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &[]);
    let sup_pid = guard.0.id();

    // It must REACH the spawn loop; the arm this replaces asserted the opposite.
    let reached_go = wait_for_log(&stderr_path, GO_MARKER, Duration::from_secs(90))
        || wait_for_log(&stdout_path, GO_MARKER, Duration::from_secs(1));
    assert!(
        reached_go,
        "a creditable split must reach GO, not be refused\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // A healthy window, then the production Ctrl-C.
    let workers = wait_for_workers(sup_pid, 2, Duration::from_secs(30));
    std::thread::sleep(Duration::from_secs(3));
    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90)).unwrap_or_else(|| {
        send_sigint(sup_pid);
        panic!(
            "the run must exit on SIGINT\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    assert!(
        status.success(),
        "a credited split run must exit 0 on Ctrl-C, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    let log = merged_log(&stdout_path, &stderr_path);
    // The flow-mode operator evidence: the acceptance and the mint are
    // ANNOUNCED, and they name the EDGE. `credit_edges = 1` alone cannot tell
    // an operator whether THEIR topic got a word.
    assert!(
        log.contains("split `block` edge accepted"),
        "the plan-time acceptance must be announced where it is decided; log:\n{log}"
    );
    assert!(
        log.contains("cross-process block credit edges minted"),
        "the mint must be announced; log:\n{log}"
    );
    // The WHOLE rendered edge, not two substrings that could be satisfied by
    // unrelated lines (the topic appears in the band listing; the input name
    // appears in the node declaration).
    assert!(
        log.contains("/apdbs/relay/cmd -> sink.trigger_in"),
        "the mint line must render the edge WHOLE; log:\n{log}"
    );
    // The acceptance line's structured fields, as whole `key=value` tokens —
    // a bare `contains("sink")` is satisfied by half the log.
    for field in [
        "topic=/apdbs/relay/cmd",
        "consumer_node=sink",
        "consumer_input=trigger_in",
    ] {
        assert!(
            has_field(&log, field),
            "the acceptance line must carry `{field}` as a structured field; log:\n{}",
            strip_ansi(&log)
        );
    }
    // The deployment really SPAWNED (this arm's whole point is the spawn path;
    // the credit word's FUNCTION is pinned by `credit_block_iox2_test`).
    assert!(
        !workers.is_empty(),
        "a creditable split must spawn workers, observed none\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );
    // And the run never printed the split-edge refusal.
    assert!(
        !log.contains("SPLITS a `block` edge"),
        "a creditable split must not be refused; log:\n{log}"
    );
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        split,
        "a run never mutates the graph file"
    );
}

/// Under flow mode, a STALE BUILD is refused PRE-SPAWN, over the real
/// binary — the arm that actually covers the reconciliation WIRING.
///
/// `validate_partition` judges the partition on the node SOURCES (a
/// partition must be judgeable without building anything) while the credit
/// words are minted from the BUILT cdylibs, so the two views can disagree.
/// This builds exactly that disagreement: the `sink` node type gets the
/// `drop_oldest` fixture's CDYLIB and the `block` fixture's SOURCE. The source
/// view then finds a creditable split and ACCEPTS the partition; the loaded
/// view sees a mixed topic and mints NOTHING; and without the reconciliation
/// the deployment would spawn and the consumer's worker would die at
/// `GraphTopology::validate` blaming an external publisher that does not exist.
///
/// Why HERE and not in an engine unit test: `graph_run_supervisor` is private
/// and reachable only through `graph run`, so a change that discards the
/// reconciliation's verdict at that call site can only be caught by driving
/// the real route. The pure function has its own oracle in
/// `multiprocess::tests`; this pins that it is WIRED.
#[test]
#[serial]
fn a_stale_build_whose_source_and_cdylib_disagree_is_refused_pre_spawn() {
    let tmp = tempfile::tempdir().unwrap();
    // The sink's CDYLIB declares `drop_oldest` (the data-trigger fixture)...
    let base = build_workspace_with_sink(
        tmp.path(),
        Some("apdrf"),
        "test_node_macro_data_trigger_cdylib",
    );
    // ...while its SOURCE declares `block`. That is a stale build: the file on
    // disk was edited and the cdylib was never rebuilt.
    let block_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures/test_node_macro_trigger_block_cdylib/src/lib.rs");
    std::fs::copy(&block_src, tmp.path().join("nodes/sink/src/lib.rs"))
        .expect("stage the disagreeing source");

    let split = base.replace(
        "nodes:\n",
        "process_groups:\n  front: [ticker, relay]\n  back: [sink]\nnodes:\n",
    );
    std::fs::write(tmp.path().join("graphs/apdemo.yaml"), &split).unwrap();

    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &[]);
    let sup_pid = guard.0.id();
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90)).unwrap_or_else(|| {
        send_sigint(sup_pid);
        panic!(
            "a stale build must REFUSE and exit, not hang\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    assert!(
        !status.success(),
        "a source/cdylib disagreement on a split `block` edge must refuse, got {status:?}"
    );
    let log = strip_ansi(&merged_log(&stdout_path, &stderr_path));
    for needle in [
        // The EDGE, whole.
        "'/apdrf/relay/cmd' -> 'sink.trigger_in'",
        // The DIRECTION — which of the two views was ahead.
        "accepted at plan time but NOT minted",
        // The CLASS, not one guessed cause.
        "STALE BUILD",
        "cerulion node build",
        "Refused before any worker was spawned",
    ] {
        assert!(
            log.contains(needle),
            "the pre-spawn refusal must name `{needle}`; log:\n{log}"
        );
    }
    assert!(
        !log.contains(GO_MARKER),
        "the refusal must land BEFORE the spawn loop; log:\n{log}"
    );
}

/// The twin that did NOT flip: splitting a MULTI-PRODUCER `block` topic is
/// still refused pre-spawn, over the REAL binary, with the NEW sentences.
///
/// `multi_publisher_topics:` + two `topic:`-overridden producers is the only
/// way to give one topic two in-graph producers — `GraphTopology::build`
/// refuses a second producer otherwise.
#[test]
#[serial]
fn a_hand_written_split_of_a_multi_producer_block_edge_is_refused_pre_spawn() {
    let tmp = tempfile::tempdir().unwrap();
    // `build_workspace_with_sink` stages the node dirs + cdylibs; the graph
    // itself is written explicitly, because two IN-GRAPH producers of one
    // topic cannot be produced by rewriting the single-producer chain (they
    // need `multi_publisher_topics:` plus a `topic:` override on each output,
    // and `GraphTopology::build` refuses the second producer otherwise).
    build_workspace_with_sink(tmp.path(), Some("apdmp"), BLOCK_SINK_FIXTURE);
    let split = "name: apdemo\n\
         prefix: apdmp\n\
         multi_publisher_topics:\n\
         - /shared/cmd\n\
         process_groups:\n\
         \x20 front: [ticker, relay, relay2]\n\
         \x20 back: [sink]\n\
         nodes:\n\
         - id: ticker\n\
         \x20 type: ticker\n\
         \x20 inputs: []\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: relay\n\
         \x20 type: relay\n\
         \x20 inputs:\n\
         \x20 - name: trigger_in\n\
         \x20\x20\x20 source: ticker/cmd\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 topic: /shared/cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: relay2\n\
         \x20 type: relay\n\
         \x20 inputs:\n\
         \x20 - name: trigger_in\n\
         \x20\x20\x20 source: ticker/cmd\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 topic: /shared/cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n\
         - id: sink\n\
         \x20 type: sink\n\
         \x20 inputs:\n\
         \x20 - name: trigger_in\n\
         \x20\x20\x20 source: /shared/cmd\n\
         \x20 outputs:\n\
         \x20 - name: cmd\n\
         \x20\x20\x20 schema: geometry_msgs/Vector3\n"
        .to_string();
    std::fs::write(tmp.path().join("graphs/apdemo.yaml"), &split).unwrap();

    let (mut guard, stdout_path, stderr_path) = spawn_graph_run(tmp.path(), &[]);
    let sup_pid = guard.0.id();
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90)).unwrap_or_else(|| {
        send_sigint(sup_pid);
        panic!(
            "an uncreditable split must REFUSE and exit, not hang\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    assert!(
        !status.success(),
        "a split MULTI-PRODUCER block edge must still be refused, got {status:?}"
    );
    let log = merged_log(&stdout_path, &stderr_path);
    for needle in [
        "SPLITS a `block` edge",
        "sink.trigger_in",
        "front",
        "back",
        "--single-process",
        // BOTH producers, EACH asserted, because the remedy says "move EVERY
        // producer" and a refusal naming only one is a remedy the operator
        // cannot follow.
        //
        // Asserting `"relay2"` ALONE while claiming "BOTH
        // producers, exactly" would leave the FIRST producer with no oracle at all,
        // so a refusal that dropped `relay` would pass. Asserting the bare `"relay"`
        // would not fix it either: that substring is satisfied by
        // `relay2`. The needle is therefore the QUOTED rendered form the
        // violation line actually emits (`producer '<name>' (group '<g>')`,
        // partition.rs), whose closing quote makes `'relay'` unsatisfiable by
        // `relay2` — one assertion per producer, neither able to stand in for
        // the other.
        "producer 'relay'",
        "producer 'relay2'",
        // The other half: WHICH bar, with the count.
        "has 2 in-graph producers",
    ] {
        assert!(
            log.contains(needle),
            "the refusal must name `{needle}`; log:\n{log}"
        );
    }
    // WHOLE SENTENCES (the embedded-spaces class: a botched line-join bakes
    // continuation indentation into the literal, and `cargo fmt` never
    // rewrites a string's contents).
    for sentence in [
        "A SPLIT edge can carry that mirror in shared memory — a cross-process \
         credit word — but only for a topic with exactly ONE in-graph producer and NO \
         non-`block` consumers",
        "Rewriting it to `drop_oldest` would silently LOSE data, so this is refused instead.",
        "Deleting the `process_groups:` block entirely also works",
    ] {
        assert!(
            log.contains(sentence),
            "the refusal renders mangled whitespace — expected `{sentence}`; log:\n{log}"
        );
    }
    assert!(
        !log.contains(GO_MARKER),
        "a refused partition must never reach the spawn loop; log:\n{log}"
    );
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).expect("readable"),
        split,
        "a refused run never mutates the graph file"
    );
}
