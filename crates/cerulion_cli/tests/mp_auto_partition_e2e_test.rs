// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for the LITERAL multi-process
//! default + the `graph partition` verb over the REAL binary — the full
//! launch story on an UNPARTITIONED 3-node chain (`ticker`(Period 50ms) →
//! `relay`(data-trigger) → `sink`(data-trigger)):
//!
//! 1. **The ephemeral default, live**: `cerulion graph run apdemo --record=DIR`
//!    with stdin explicitly `/dev/null`: the run derives the
//!    process-per-node partition in memory, the lifecycle notice says the
//!    graph file is unchanged, the graph file stays
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

/// The recorder has its own process group. Stop only the recorder belonging
/// to this test's supervisor if an assertion fails before normal teardown.
struct BagdGuard {
    supervisor_pid: u32,
    supervisor_identity: Option<ProcessIdentity>,
    spawn_logs: Option<(PathBuf, [PathBuf; 2])>,
    observed: std::cell::RefCell<Vec<ObservedProcess>>,
}
impl BagdGuard {
    fn new(supervisor_pid: u32) -> Self {
        Self {
            supervisor_pid,
            supervisor_identity: process_identity(supervisor_pid),
            spawn_logs: None,
            observed: std::cell::RefCell::new(Vec::new()),
        }
    }

    fn for_recording(supervisor_pid: u32, root: &Path, stdout: &Path, stderr: &Path) -> Self {
        let mut guard = Self::new(supervisor_pid);
        guard.spawn_logs = std::fs::canonicalize(root)
            .ok()
            .map(|root| (root, [stdout.to_owned(), stderr.to_owned()]));
        guard
    }

    /// Recover a spawn that happened between polls from this test's private
    /// supervisor logs. Exact argv, private cwd and a stable live identity
    /// are required; a log PID alone never authorizes a signal.
    fn observe_spawn_logs(&self) {
        let Some((root, logs)) = &self.spawn_logs else {
            return;
        };
        for log in logs {
            for line in strip_ansi(&read_file(log)).lines() {
                if !line.contains("spawned bagd recorder") {
                    continue;
                }
                let fields: Vec<_> = line.split_whitespace().collect();
                let pid = fields
                    .iter()
                    .find_map(|field| field.strip_prefix("pid="))
                    .and_then(|field| field.parse::<u32>().ok())
                    .filter(|pid| *pid > 1);
                let bag = fields
                    .iter()
                    .find_map(|field| field.strip_prefix("bag="))
                    .map(|bag| bag.trim_matches('"'));
                let (Some(pid), Some(bag)) = (pid, bag) else {
                    continue;
                };
                if self
                    .observed
                    .borrow()
                    .iter()
                    .any(|process| process.pid == pid)
                {
                    continue; // never refresh a previously captured PID's identity
                }
                if !bag.starts_with("recordings/apdemo_") || !bag.ends_with(".mcap") {
                    continue;
                }
                let Some(identity) = process_identity(pid) else {
                    continue;
                };
                if identity.pgid != pid || process_cwd(pid).as_ref() != Some(root) {
                    continue;
                }
                let Ok(command) = Command::new("ps")
                    .args(["-p", &pid.to_string(), "-o", "command="])
                    .output()
                else {
                    continue;
                };
                if !command.status.success() {
                    continue;
                }
                let command = String::from_utf8_lossy(&command.stdout);
                let args: Vec<_> = command.split_whitespace().collect();
                if args.windows(3).any(|args| args == ["bagd", "--out", bag])
                    && process_identity(pid) == Some(identity)
                {
                    let mut observed = self.observed.borrow_mut();
                    if !observed.iter().any(|process| process.pid == pid) {
                        observed.push(ObservedProcess {
                            pid,
                            identity: Some(identity),
                        });
                    }
                }
            }
        }
    }

    /// Production observations come from the live supervisor's child listing.
    /// Tests can supply an already-owned process to exercise orphan cleanup.
    fn remember_owned(&self, pids: &[u32]) {
        let mut observed = self.observed.borrow_mut();
        for &pid in pids {
            if pid > 1 && !observed.iter().any(|process| process.pid == pid) {
                observed.push(ObservedProcess {
                    pid,
                    identity: process_identity(pid),
                });
            }
        }
    }

    fn pids(&self) -> Vec<u32> {
        let mut live: Vec<u32> = if self.supervisor_pid > 1
            && self.supervisor_identity.is_some()
            && process_identity(self.supervisor_pid) == self.supervisor_identity
        {
            Command::new("pgrep")
                .args([
                    "-P",
                    &self.supervisor_pid.to_string(),
                    "-f",
                    "bagd --out recordings/apdemo_",
                ])
                .output()
                .map(|output| {
                    String::from_utf8_lossy(&output.stdout)
                        .lines()
                        .filter_map(|line| line.trim().parse().ok())
                        .collect()
                })
                .unwrap_or_default()
        } else {
            Vec::new() // dead or replaced supervisors cannot contribute new children
        };
        // The parent may exit while pgrep runs. Admit its live-child listing
        // only while the same observed supervisor still owns that PID.
        if self.supervisor_identity.is_none()
            || process_identity(self.supervisor_pid) != self.supervisor_identity
        {
            live.clear();
        }
        self.remember_owned(&live);
        self.observe_spawn_logs();
        self.observed
            .borrow()
            .iter()
            .map(|process| process.pid)
            .collect()
    }
}
impl Drop for BagdGuard {
    fn drop(&mut self) {
        let _ = self.pids();
        for process in self.observed.borrow().iter() {
            let Some(original) = process.identity else {
                continue; // metadata errors and unsupported platforms fail closed
            };
            if original.pgid == process.pid && process_identity(process.pid) == Some(original) {
                // SAFETY: this PID still has the observed start identity and
                // owned group. No memory is touched. This check is best-effort,
                // not an atomic identity-bound signal (macOS has no pidfd).
                unsafe {
                    libc::killpg(process.pid as libc::pid_t, libc::SIGKILL);
                }
            }
        }
    }
}

struct ObservedProcess {
    pid: u32,
    identity: Option<ProcessIdentity>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ProcessIdentity {
    pgid: u32,
    started: (u64, u64),
}

#[cfg(target_os = "linux")]
fn process_identity(pid: u32) -> Option<ProcessIdentity> {
    let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    // comm (field 2) can contain spaces and parentheses. The remaining
    // numeric fields follow its LAST closing parenthesis; starttime is 22.
    let (_, fields) = stat.rsplit_once(')')?;
    let fields: Vec<_> = fields.split_whitespace().collect();
    if fields.first().copied() == Some("Z") {
        return None;
    }
    Some(ProcessIdentity {
        pgid: fields.get(2)?.parse().ok()?,
        started: (0, fields.get(19)?.parse().ok()?),
    })
}

#[cfg(target_os = "macos")]
fn process_identity(pid: u32) -> Option<ProcessIdentity> {
    let mut info = std::mem::MaybeUninit::<libc::proc_bsdinfo>::uninit();
    let size = std::mem::size_of::<libc::proc_bsdinfo>();
    // SAFETY: the buffer is aligned for the exact libc BSD-info layout and
    // holds size bytes. Only a complete proc_pidinfo result is read below.
    let result = unsafe {
        libc::proc_pidinfo(
            i32::try_from(pid).ok()?,
            libc::PROC_PIDTBSDINFO,
            0,
            info.as_mut_ptr().cast(),
            i32::try_from(size).ok()?,
        )
    };
    if usize::try_from(result).ok()? != size {
        return None;
    }
    // SAFETY: the exact-size successful query initialized the complete struct.
    let info = unsafe { info.assume_init() };
    if info.pbi_pid != pid || info.pbi_status == libc::SZOMB {
        return None;
    }
    Some(ProcessIdentity {
        pgid: info.pbi_pgid,
        started: (info.pbi_start_tvsec, info.pbi_start_tvusec),
    })
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn process_identity(_pid: u32) -> Option<ProcessIdentity> {
    None // no reliable start identity available: never signal cached PIDs
}

#[cfg(target_os = "linux")]
fn process_cwd(pid: u32) -> Option<PathBuf> {
    std::fs::read_link(format!("/proc/{pid}/cwd"))
        .ok()?
        .canonicalize()
        .ok()
}

#[cfg(target_os = "macos")]
fn process_cwd(pid: u32) -> Option<PathBuf> {
    use std::os::unix::ffi::OsStringExt as _;
    let mut info = std::mem::MaybeUninit::<libc::proc_vnodepathinfo>::uninit();
    let size = std::mem::size_of::<libc::proc_vnodepathinfo>();
    // SAFETY: aligned buffer for the exact vnode-path libc layout, with size
    // bytes available. Read it only after a complete successful query.
    let result = unsafe {
        libc::proc_pidinfo(
            i32::try_from(pid).ok()?,
            libc::PROC_PIDVNODEPATHINFO,
            0,
            info.as_mut_ptr().cast(),
            i32::try_from(size).ok()?,
        )
    };
    if usize::try_from(result).ok()? != size {
        return None;
    }
    // SAFETY: the exact-size result initialized the entire struct. Find the
    // terminator within its fixed buffer before reading any path bytes.
    let info = unsafe { info.assume_init() };
    let path: Vec<u8> = info
        .pvi_cdir
        .vip_path
        .iter()
        .flatten()
        .map(|&byte| byte as u8)
        .collect();
    let end = path.iter().position(|&byte| byte == 0)?;
    let path = std::ffi::OsString::from_vec(path[..end].to_vec());
    PathBuf::from(path).canonicalize().ok()
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn process_cwd(_pid: u32) -> Option<PathBuf> {
    None
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
fn wait_for_bag(recordings: &Path, timeout: Duration, guard: &BagdGuard) -> Option<PathBuf> {
    let start = Instant::now();
    while start.elapsed() < timeout {
        let _ = guard.pids(); // observe before every fallible startup poll/assertion
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

/// The ephemeral lifecycle marker (a substring of `partition_emit`'s info - pinned
/// loosely enough to survive tracing-field formatting, tightly enough that
/// only auto-partition adoption emits it).
const IN_MEMORY_NOTICE_MARKER: &str = "derived process groups in memory";
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
    let _bagd_guard =
        BagdGuard::for_recording(guard.0.id(), tmp.path(), &stdout_path, &stderr_path);
    let sup_pid = guard.0.id();

    // The mp bring-up completes: bag file appears (planning + 3 worker spawns
    // + bagd handshake precede it — generous bound for slow CI VMs).
    let recordings = tmp.path().join("recordings");
    let bag =
        wait_for_bag(&recordings, Duration::from_secs(90), &_bagd_guard).unwrap_or_else(|| {
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
    let _ = _bagd_guard.pids(); // retain the owned recorder before reaping the parent
    send_sigint(sup_pid);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "the in-memory mp run must exit 0 on Ctrl-C, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // The ordinary run announces in-memory adoption without a persistence question.
    let log = merged_log(&stdout_path, &stderr_path);
    assert!(
        log.contains(IN_MEMORY_NOTICE_MARKER),
        "the no-TTY floor notice must land in the child log; log:\n{log}"
    );
    assert!(
        log.contains("graph file unchanged"),
        "the ephemeral run names its file policy; log:\n{log}"
    );
    assert!(
        !log.contains("Apply this partition") && !log.contains("+++ proposed"),
        "ordinary runs show neither a save prompt nor a partition diff; log:\n{log}"
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
    let _bagd_guard =
        BagdGuard::for_recording(guard.0.id(), tmp.path(), &stdout_path, &stderr_path);

    let recordings = tmp.path().join("recordings");
    let bag =
        wait_for_bag(&recordings, Duration::from_secs(90), &_bagd_guard).unwrap_or_else(|| {
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
    let _ = _bagd_guard.pids(); // retain the owned recorder before reaping the parent
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
    let _bagd_guard =
        BagdGuard::for_recording(guard.0.id(), tmp.path(), &stdout_path, &stderr_path);
    let sup_pid = guard.0.id();

    let recordings = tmp.path().join("recordings");
    let bag =
        wait_for_bag(&recordings, Duration::from_secs(90), &_bagd_guard).unwrap_or_else(|| {
            panic!(
                "bagd never created the bag: the defect shape is a `block` consumer's \
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
    let _ = _bagd_guard.pids(); // retain the owned recorder before reaping the parent
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

/// Empty terminal stdin must reach real delivery without an answer. The
/// non-terminal control exercises the same command and hand-written oracles.
#[test]
#[serial]
fn tty_default_runs_two_nodes_without_a_choice_and_stops_owned_workers() {
    assert_two_node_ephemeral_run(true);
}

#[test]
#[serial]
fn non_tty_default_runs_two_nodes_without_a_choice_and_stops_owned_workers() {
    assert_two_node_ephemeral_run(false);
}

fn assert_two_node_ephemeral_run(terminal: bool) {
    let tmp = tempfile::tempdir().unwrap();
    let home = tmp.path().join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "acct-ephemeral-run-test")
        .expect("seed private logged-in identity");
    let prefix = if terminal { "apdtty" } else { "apdpipe" };
    let three_node_yaml = build_unpartitioned_workspace(tmp.path(), prefix);
    let original_yaml = three_node_yaml
        .split("- id: sink\n")
        .next()
        .unwrap()
        .to_owned();
    std::fs::write(tmp.path().join("graphs/apdemo.yaml"), &original_yaml).unwrap();
    let stdout_path = tmp.path().join("choice.stdout");
    let stderr_path = tmp.path().join("choice.stderr");
    let pty = terminal.then(open_terminal);
    let stdin = pty.as_ref().map_or_else(Stdio::null, |(_, slave)| {
        Stdio::from(slave.try_clone().expect("clone terminal stdin"))
    });
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args([
        "graph",
        "run",
        "apdemo",
        "--network",
        "off",
        "--record=recordings",
    ])
    .current_dir(tmp.path())
    .env_remove("CARGO_TARGET_DIR")
    .env("CERULION_HOME", &home)
    .env_remove("CERULION_LOGIN_GATE")
    .env("CERULION_NETWORK", "off")
    .env("CERULION_FLASHBACK", "off")
    .env(
        "RUST_LOG",
        "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
    )
    .stdin(stdin)
    .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
    .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    let mut guard = mp_support::ChildGuard::spawn_group_leader(&mut cmd).expect("spawn graph run");
    let bagd_guard = BagdGuard::for_recording(guard.id(), tmp.path(), &stdout_path, &stderr_path);
    let sup_pid = guard.id();
    let bag = wait_for_bag(
        &tmp.path().join("recordings"),
        Duration::from_secs(90),
        &bagd_guard,
    )
    .unwrap_or_else(|| {
        panic!(
            "no output without answering stdin; terminal={terminal}\n{}",
            merged_log(&stdout_path, &stderr_path)
        )
    });
    let workers = wait_for_workers(sup_pid, 2, Duration::from_secs(60));
    assert_eq!(workers.len(), 2, "each node retains its own process");
    guard.note_workers_explicit(workers);
    let ticker = format!("/{prefix}/ticker/cmd");
    let relay = format!("/{prefix}/relay/cmd");
    wait_for_bag_state(
        &bag,
        "both nodes' output",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(&ticker) > 0 && snap.frames_on(&relay) > 0,
    );
    let recorders = bagd_guard.pids();
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    assert_eq!(
        recorders.len(),
        1,
        "the run must own one continuous recorder"
    );
    send_sigint(sup_pid);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("Ctrl-C stops the run");
    assert_eq!(status.code(), Some(0), "Ctrl-C must be a clean shutdown");
    guard.finish().assert_clean();
    for pid in recorders {
        assert!(
            mp_support::pid_is_gone(pid),
            "owned recorder {pid} must be reaped"
        );
    }
    drop(pty);

    let log = strip_ansi(&merged_log(&stdout_path, &stderr_path));
    assert!(
        log.contains("graph running")
            && log.contains("nodes=2")
            && log.contains("mode=\"multi-process\"")
            && log.contains("press Ctrl+C to stop"),
        "the useful running status carries the real shape and stop instruction:\n{log}"
    );
    for unwanted in ["Apply this partition", "+++ proposed", "proposed change to"] {
        assert!(
            !log.contains(unwanted),
            "ordinary run displayed {unwanted}:\n{log}"
        );
    }
    assert_eq!(
        std::fs::read_to_string(tmp.path().join("graphs/apdemo.yaml")).unwrap(),
        original_yaml
    );
    assert!(!tmp.path().join("graphs/apdemo.yaml.bak").exists());
    let reader = BagReader::open(&bag).expect("open recording");
    let (messages, completeness) = reader.recover_messages().expect("recover recording");
    assert!(
        completeness.is_finalized(),
        "clean stop must finalize the recording"
    );
    for topic in [&ticker, &relay] {
        let frames: Vec<_> = messages.iter().filter(|m| &m.topic == topic).collect();
        assert!(!frames.is_empty(), "{topic} must publish actual frames");
        for frame in frames {
            assert_eq!(
                frame.data.len(),
                56,
                "32-byte wire header and fixed Vector3"
            );
            assert_eq!(
                &frame.data[32..],
                &[0u8; 24],
                "ticker writes zero x and relay forwards it; untouched y/z remain zero"
            );
        }
    }
    assert_eq!(read_manifest(&reader, 0).unwrap(), vec!["ticker"]);
    assert_eq!(read_manifest(&reader, 1).unwrap(), vec!["relay"]);
    assert!(read_manifest(&reader, 2).is_none());
}

/// Keep the master open so terminal stdin stays readable but receives no
/// answer. Descriptor ownership closes both ends on every exit path.
fn open_terminal() -> (std::fs::File, std::fs::File) {
    use std::io::IsTerminal as _;
    use std::os::fd::FromRawFd as _;
    let (mut master, mut slave) = (-1, -1);
    // SAFETY: openpty writes two initialized descriptor slots, allocates no
    // Rust memory, and receives null pointers for the optional name/settings.
    let result = unsafe {
        libc::openpty(
            &mut master,
            &mut slave,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(result, 0, "openpty: {}", std::io::Error::last_os_error());
    // SAFETY: successful openpty returned two distinct owned descriptors;
    // each is transferred to exactly one File and closed by its Drop.
    let files = unsafe {
        (
            std::fs::File::from_raw_fd(master),
            std::fs::File::from_raw_fd(slave),
        )
    };
    assert!(
        files.1.is_terminal(),
        "this arm must exercise real terminal stdin"
    );
    files
}

/// Exercise the cached-identity error path over real owned processes. The
/// observation seam avoids timing a supervisor crash against a recorder spawn.
#[test]
#[serial]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_guard_remembers_owned_pid_after_the_supervisor_is_gone() {
    use std::os::unix::process::{CommandExt as _, ExitStatusExt as _};
    let mut parent = mp_support::ChildGuard::single_process(
        Command::new("true")
            .spawn()
            .expect("spawn short-lived parent"),
    );
    let parent_pid = parent.id();
    parent
        .wait_bounded(Duration::from_secs(5))
        .expect("parent exits");
    assert!(mp_support::pid_is_gone(parent_pid));
    let mut child = mp_support::ChildGuard::single_process(
        Command::new("sleep")
            .arg("60")
            .process_group(0)
            .spawn()
            .expect("spawn owned process"),
    );
    let child_pid = child.id();
    let recorder = BagdGuard::new(parent_pid);
    recorder.remember_owned(&[child_pid]);
    assert_eq!(
        recorder.pids(),
        vec![child_pid],
        "an empty dead-parent lookup must retain the observed identity"
    );
    drop(recorder);
    let status = child
        .wait_bounded(Duration::from_secs(5))
        .expect("guard stops observed process");
    assert_eq!(status.signal(), Some(libc::SIGKILL));
    assert!(mp_support::pid_is_gone(child_pid));
}

/// The existing observation seam models a cached PID whose start identity
/// changed. The victim is a real owned process; surviving Drop is the oracle.
#[test]
#[serial]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_guard_preserves_a_live_process_with_a_different_start_identity() {
    use std::os::unix::process::{CommandExt as _, ExitStatusExt as _};
    let mut child = mp_support::ChildGuard::single_process(
        Command::new("sleep")
            .arg("60")
            .process_group(0)
            .spawn()
            .expect("spawn owned process"),
    );
    let pid = child.id();
    let recorder = BagdGuard::new(0); // no live supervisor contributes discoveries
    recorder.remember_owned(&[pid]);
    {
        let mut observed = recorder.observed.borrow_mut();
        let identity = observed[0]
            .identity
            .as_mut()
            .expect("live identity available");
        identity.started.1 = identity.started.1.wrapping_add(1);
    }
    drop(recorder);
    assert!(
        child
            .try_wait_noting()
            .expect("poll live process")
            .is_none(),
        "changed identity must not receive SIGKILL"
    );
    send_sigint(pid);
    let status = child
        .wait_bounded(Duration::from_secs(5))
        .expect("stop surviving process");
    assert_eq!(status.signal(), Some(libc::SIGINT));
    child.finish().assert_clean();
}

/// A cached supervisor identity that changed must not admit newly discovered
/// children. A real owned, recorder-named process proves the refusal is safe.
#[test]
#[serial]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_guard_rejects_children_of_a_different_supervisor_identity() {
    use std::os::unix::process::{CommandExt as _, ExitStatusExt as _};
    let mut child = mp_support::ChildGuard::single_process(
        Command::new("sleep")
            .arg0("bagd --out recordings/apdemo_identity")
            .arg("60")
            .process_group(0)
            .spawn()
            .expect("spawn recorder-named owned process"),
    );
    let pid = child.id();
    let parent_pid = std::process::id();
    let found = Command::new("pgrep")
        .args([
            "-P",
            &parent_pid.to_string(),
            "-f",
            "bagd --out recordings/apdemo_",
        ])
        .output()
        .expect("observe recorder-named child");
    assert!(
        String::from_utf8_lossy(&found.stdout)
            .lines()
            .any(|line| line.trim().parse::<u32>() == Ok(pid)),
        "the live parent's child must be discoverable before identity rejection"
    );
    let mut recorder = BagdGuard::new(parent_pid);
    let identity = recorder
        .supervisor_identity
        .as_mut()
        .expect("live parent identity available");
    identity.started.1 = identity.started.1.wrapping_add(1);
    assert!(
        recorder.pids().is_empty(),
        "a replaced parent must not admit new children"
    );
    drop(recorder);
    assert!(
        child
            .try_wait_noting()
            .expect("poll live process")
            .is_none(),
        "replacement parent's child must survive Drop"
    );
    send_sigint(pid);
    let status = child
        .wait_bounded(Duration::from_secs(5))
        .expect("stop surviving process");
    assert_eq!(status.signal(), Some(libc::SIGINT));
    child.finish().assert_clean();
}

/// A real subprocess fixture writes the PID of its own recorder-named child,
/// then exits without observing it. The calling test owns the orphan cleanup.
#[test]
#[ignore = "subprocess fixture invoked only by the logged-orphan regression"]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_spawn_fixture() {
    use std::os::unix::process::CommandExt as _;
    let Some(log) = std::env::var_os("CERULION_TEST_RECORDER_LOG") else {
        // A direct --ignored run has no parent that owns an intentional orphan.
        return;
    };
    let child = mp_support::ChildGuard::single_process(
        Command::new("sleep")
            .arg0("bagd --out recordings/apdemo_owned.mcap")
            .arg("60")
            .process_group(0)
            .spawn()
            .expect("spawn fixture's recorder-named child"),
    );
    std::fs::write(
        log,
        format!(
            "spawned bagd recorder pid={} bag=recordings/apdemo_owned.mcap\n",
            child.id()
        ),
    )
    .expect("write actual spawn evidence");
    // This error-path fixture intentionally orphans its child. The parent test
    // retains the real spawn log and a fallback guard so failure still cleans it.
    std::mem::forget(child);
}

#[test]
#[serial]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_guard_recovers_logged_owned_orphan_before_its_first_poll() {
    let root = tempfile::tempdir().unwrap();
    let stdout = root.path().join("supervisor.stdout");
    let stderr = root.path().join("supervisor.stderr");
    let mut parent = mp_support::ChildGuard::single_process(
        Command::new(std::env::current_exe().unwrap())
            .args(["--ignored", "--exact", "recorder_spawn_fixture"])
            .env("CERULION_TEST_RECORDER_LOG", &stderr)
            .current_dir(root.path())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("spawn real supervisor fixture"),
    );
    let recorder = BagdGuard::for_recording(parent.id(), root.path(), &stdout, &stderr);
    let status = parent
        .wait_bounded(Duration::from_secs(5))
        .expect("supervisor exits");
    assert_eq!(status.code(), Some(0));
    let pid = read_file(&stderr)
        .split_whitespace()
        .find_map(|field| {
            field
                .strip_prefix("pid=")
                .and_then(|pid| pid.parse::<u32>().ok())
        })
        .expect("actual logged child PID");
    let fallback = BagdGuard::new(0);
    fallback.remember_owned(&[pid]);
    assert!(
        recorder.observed.borrow().is_empty(),
        "no recorder observation before parent exit"
    );
    assert!(
        !mp_support::pid_is_gone(pid),
        "the fixture must leave a real owned orphan"
    );
    drop(recorder); // first observation must recover the private log, not dead-parent children
    let deadline = Instant::now() + Duration::from_secs(5);
    while !mp_support::pid_is_gone(pid) && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        mp_support::pid_is_gone(pid),
        "logged owned orphan must be stopped and reaped"
    );
    drop(fallback);
    parent.finish().assert_clean();
}

#[test]
#[serial]
#[cfg(any(target_os = "linux", target_os = "macos"))]
fn recorder_guard_preserves_logged_live_process_with_wrong_cwd_or_bag() {
    use std::os::unix::process::{CommandExt as _, ExitStatusExt as _};
    let root = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let stdout = root.path().join("supervisor.stdout");
    let stderr = root.path().join("supervisor.stderr");
    for (cwd, logged_bag) in [
        (elsewhere.path(), "recordings/apdemo_owned.mcap"),
        (root.path(), "recordings/apdemo_different.mcap"),
    ] {
        let mut child = mp_support::ChildGuard::single_process(
            Command::new("sleep")
                .arg0("bagd --out recordings/apdemo_owned.mcap")
                .arg("60")
                .current_dir(cwd)
                .process_group(0)
                .spawn()
                .expect("spawn owned recorder-named process"),
        );
        let pid = child.id();
        std::fs::write(
            &stderr,
            format!("spawned bagd recorder pid={pid} bag={logged_bag}\n"),
        )
        .unwrap();
        let recorder = BagdGuard::for_recording(0, root.path(), &stdout, &stderr);
        assert!(
            recorder.pids().is_empty(),
            "wrong cwd or exact bag argument must refuse ownership"
        );
        drop(recorder);
        assert!(
            child
                .try_wait_noting()
                .expect("poll live process")
                .is_none(),
            "nonmatching process must survive Drop"
        );
        send_sigint(pid);
        let status = child
            .wait_bounded(Duration::from_secs(5))
            .expect("stop surviving process");
        assert_eq!(status.signal(), Some(libc::SIGINT));
        child.finish().assert_clean();
    }
}
