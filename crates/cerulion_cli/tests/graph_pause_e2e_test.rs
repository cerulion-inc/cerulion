// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END coverage of `cerulion graph pause` and `cerulion graph resume` over
//! the REAL binary: spawn a recorded `graph run`, pause it for five seconds, resume
//! it, stop it, and read what the bag says happened.
//!
//! # What is pinned
//!
//! 1. **A pause leaves no gap and no burst in a recording.** The ticker publishes
//!    every 50 ms. Across a five-second pause the recorded timestamps of its
//!    frames stay contiguous (the largest gap between two consecutive frames is a
//!    fraction of the pause, not the pause), and no two frames land on top of each
//!    other when the run resumes (the burst that follows a stopped process). The
//!    frame count also stands still for the whole hold.
//! 2. **The counter-test: stopping the process does leave the gap.** The same run,
//!    with `SIGSTOP` and `SIGCONT` in place of the verbs, records a gap about as
//!    long as the stop. It is what makes the assertion above able to fail.
//! 3. **Every process of a multi-process run is held**, free-run (the default) and
//!    lockstep, and the lockstep run is not poisoned by a pause longer than its
//!    five-second boundary timeout.
//! 4. **The state is reported.** `run.json` carries `"paused": true` while the run
//!    is held and `"paused": false` after, and a repeat of a verb succeeds and says
//!    "already".
//! 5. **The exit codes.** 4 for a run that is not running, 1 for a run that cannot
//!    be paused (virtual time has no clock to stop), 2 for a missing argument.
//!
//! Harness follows `mp_record_e2e_test.rs` (tempdir workspace, prebuilt fixture
//! cdylibs, redirected child logs, bounded waits, `#[serial]`, unique prefixes).
//! Prerequisite, as for that file:
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! Runs on the GLOBAL iceoryx2 namespace (the production `graph run` path has no
//! isolation seam), so the verbs always address a run by its full run id, read from
//! the run's own `run.json` in an isolated `CERULION_HOME`, never by graph name.

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. Every arm here
// spawns a `cerulion` child that loads plugin nodes, and the multi-process arms ask
// that supervisor for further event resources after the load, which fails on macOS
// (measured: both multi-process arms refused to pre-create their first topic). The
// whole binary is gated rather than those arms for the reason stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use serial_test::serial;

mod mp_support;
use mp_support::*;

/// How long every run in this file is held.
const PAUSE: Duration = Duration::from_secs(5);

/// The largest gap between two consecutive frames that a five-second pause may
/// leave: half the pause. A hold that was not a stop of the clock leaves the whole
/// pause (five seconds and a tenth, measured); the wake interval a paused graph can
/// overshoot by is two orders of magnitude under this.
const MAX_ALLOWED_GAP_NS: u64 = PAUSE.as_nanos() as u64 / 5;

/// Two consecutive frames closer than this are a BURST: the ticker is 50 ms, so a
/// healthy pair is tens of milliseconds apart, and the burst a stopped process
/// produces on resume lands four frames within five.
const BURST_GAP_NS: u64 = 5_000_000;

/// What the verbs were given and what they said.
struct VerbOutcome {
    code: Option<i32>,
    stdout: String,
    stderr: String,
}

/// Run `cerulion graph <verb> <target>` with its output captured.
fn graph_verb(home: &Path, verb: &str, target: &str) -> VerbOutcome {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", verb, target])
        .env("CERULION_HOME", home)
        .env("CERULION_NETWORK", "off")
        .env("NO_COLOR", "1")
        .stdin(Stdio::null())
        .output()
        .expect("spawn cerulion graph pause|resume");
    VerbOutcome {
        code: out.status.code(),
        stdout: String::from_utf8_lossy(&out.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
    }
}

/// The run id a `graph run` wrote into its (isolated) run directory, polled until it
/// appears.
fn wait_for_run_id(home: &Path, timeout: Duration) -> String {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if let Ok(runs) = std::fs::read_dir(home.join("runs")) {
            for dir in runs.flatten() {
                if let Ok(bytes) = std::fs::read(dir.path().join("run.json")) {
                    if let Ok(doc) = serde_json::from_slice::<serde_json::Value>(&bytes) {
                        if let Some(id) = doc["run_id"].as_str() {
                            return id.to_string();
                        }
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    panic!("no run.json appeared under {}", home.display());
}

/// `"paused"` as the run's `run.json` says it right now (`None` when the key is
/// absent).
fn manifest_paused(home: &Path) -> Option<bool> {
    let runs = std::fs::read_dir(home.join("runs")).ok()?;
    let dir = runs.flatten().next()?;
    let bytes = std::fs::read(dir.path().join("run.json")).ok()?;
    let doc: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    doc.get("paused").and_then(serde_json::Value::as_bool)
}

/// The consecutive-frame spacing of one topic in a finalized bag.
#[derive(Debug)]
struct Spacing {
    frames: usize,
    max_gap_ns: u64,
    /// How many consecutive frame pairs are closer than [`BURST_GAP_NS`].
    coincident_pairs: usize,
}

fn spacing(bag: &Path, topic: &str) -> Spacing {
    let reader = BagReader::open(bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover messages");
    assert!(
        completeness.is_finalized(),
        "the bag must be finalized, got {completeness:?}"
    );
    let mut times: Vec<u64> = msgs
        .iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.log_time)
        .collect();
    times.sort_unstable();
    assert!(times.len() >= 20, "{topic}: only {} frames", times.len());
    let gaps: Vec<u64> = times.windows(2).map(|w| w[1] - w[0]).collect();
    Spacing {
        frames: times.len(),
        max_gap_ns: gaps.iter().copied().max().expect("gaps"),
        coincident_pairs: gaps.iter().filter(|g| **g <= BURST_GAP_NS).count(),
    }
}

/// Reaps the bagd THIS run started, whatever point the run reached. bagd runs in its
/// own process group, so a panic that kills the run would otherwise orphan it.
///
/// Armed before the spawn with the run's own recordings directory, and it decides
/// only when dropped: every bag in that directory names its recorder by the bag's
/// unique filename, which is looked up THEN, so the guard never acts on a stale
/// process group (a recorder that already exited is simply not found) and never on
/// a recorder another test started, whatever order the two spawned in.
struct RunBagd {
    recordings: PathBuf,
}

impl Drop for RunBagd {
    fn drop(&mut self) {
        let Ok(entries) = std::fs::read_dir(&self.recordings) else {
            return;
        };
        for bag in entries.filter_map(Result::ok) {
            let Ok(out) = Command::new("pgrep")
                .arg("-f")
                .arg(bag.file_name())
                .output()
            else {
                continue;
            };
            let pids = String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<libc::pid_t>().ok())
                .filter(|pid| *pid > 1)
                .collect::<Vec<_>>();
            for pid in pids {
                // SAFETY: killpg(2) on the recorder's own process group (pgid == pid,
                // it is spawned with process_group(0)); no memory is touched.
                unsafe {
                    libc::killpg(pid, libc::SIGKILL);
                }
            }
        }
    }
}

/// A recorded run in flight: its guard, bag, run id and isolated home.
struct Recorded {
    guard: ChildGuard,
    _bagd: RunBagd,
    /// Held so the workspace outlives the run.
    _tmp: tempfile::TempDir,
    stderr_path: PathBuf,
    bag: PathBuf,
    run_id: String,
    home: PathBuf,
}

impl Recorded {
    fn home_of(tmp: &tempfile::TempDir) -> PathBuf {
        tmp.path().join("home")
    }

    fn start(prefix: &str, extra: &[&str], mode: SpawnExecutionMode) -> Self {
        let tmp = tempfile::tempdir().unwrap();
        build_mp_workspace(tmp.path(), prefix);
        let home = Self::home_of(&tmp);
        std::fs::create_dir_all(&home).unwrap();
        let home_str = home.display().to_string();
        let bagd = RunBagd {
            recordings: tmp.path().join("recordings"),
        };
        let (guard, _stdout, stderr_path) = spawn_graph_run_graph(
            tmp.path(),
            "mpdemo",
            true,
            extra,
            &[("CERULION_HOME", home_str.as_str())],
            mode,
        );
        let bag = wait_for_bag(&tmp.path().join("recordings"), Duration::from_secs(90))
            .unwrap_or_else(|| {
                panic!(
                    "bagd never created the bag\nstderr:\n{}",
                    read_file(&stderr_path)
                )
            });
        let run_id = wait_for_run_id(&home, Duration::from_secs(30));
        let run = Self {
            guard,
            _bagd: bagd,
            _tmp: tmp,
            stderr_path,
            bag,
            run_id,
            home,
        };
        // Wait for the run to be stepping and recording, so every arm pauses a run
        // that is demonstrably live.
        let topic = format!("/{prefix}/ticker/cmd");
        wait_for_bag_state(
            &run.bag,
            "the ticker's first frames",
            RECORDED_WINDOW_TIMEOUT,
            |snap| snap.frames_on(&topic) >= 10,
        );
        run
    }

    fn frames_on(&self, topic: &str) -> usize {
        bag_snapshot(&self.bag).map_or(0, |s| s.frames_on(topic))
    }

    /// Stop the run with the production Ctrl-C and return its log.
    fn stop(&mut self) -> String {
        send_signal(self.guard.id(), libc::SIGINT);
        let status = self
            .guard
            .wait_bounded(Duration::from_secs(90))
            .expect("the run did not exit after SIGINT");
        let log = read_file(&self.stderr_path);
        assert!(
            status.success(),
            "the run must exit 0 on SIGINT, got {status:?}\nlog:\n{log}"
        );
        log
    }

    fn assert_no_gap_no_burst(&self, topic: &str) -> Spacing {
        let s = spacing(&self.bag, topic);
        assert!(
            s.max_gap_ns < MAX_ALLOWED_GAP_NS,
            "{topic}: a pause must leave no gap in the recording, but the largest gap between \
             two frames is {} ms (limit {} ms); {s:?}",
            s.max_gap_ns / 1_000_000,
            MAX_ALLOWED_GAP_NS / 1_000_000
        );
        // The burst a stopped process makes on resume lands four frames within five
        // milliseconds: three coincident pairs. One pair can appear on a host that
        // stalls a step for a whole tick, with or without a pause, so a single pair
        // is not a burst.
        assert!(
            s.coincident_pairs <= 1,
            "{topic}: resuming must not burst, but {} frame pairs are within {} ns of each \
             other; {s:?}",
            s.coincident_pairs,
            BURST_GAP_NS
        );
        s
    }
}

/// Pause the run, hold it for [`PAUSE`], resume it, and wait for it to publish on
/// `topic` again. The verbs' own output is checked on the way.
fn pause_hold_resume(run: &Recorded, topic: &str, hold: Duration) {
    let first = graph_verb(&run.home, "pause", &run.run_id);
    assert_eq!(first.code, Some(0), "pause: {}", first.stderr);
    assert!(
        first.stdout.contains("is now paused"),
        "the verb says what it did: {}",
        first.stdout
    );
    assert_eq!(
        manifest_paused(&run.home),
        Some(true),
        "run.json must say the run is paused"
    );
    let again = graph_verb(&run.home, "pause", &run.run_id);
    assert_eq!(
        again.code,
        Some(0),
        "a repeated pause succeeds: {}",
        again.stderr
    );
    assert!(
        again.stdout.contains("already paused"),
        "a repeated pause says so: {}",
        again.stdout
    );

    // The hold publishes NOTHING: once the run has reached its boundary and the bag
    // has caught up, the frame count stands still until the resume. Without this a
    // runtime that ignored the page could keep publishing and still satisfy the
    // spacing assertions that follow.
    const SETTLE: Duration = Duration::from_secs(2);
    assert!(hold > SETTLE, "the hold must outlast the settle");
    std::thread::sleep(SETTLE);
    let held_start = run.frames_on(topic);
    std::thread::sleep(hold - SETTLE);
    let held_end = run.frames_on(topic);
    assert_eq!(
        held_end, held_start,
        "{topic}: a paused run must publish nothing, but the bag grew from {held_start} to \
         {held_end} frames during the hold"
    );

    let resumed = graph_verb(&run.home, "resume", &run.run_id);
    assert_eq!(resumed.code, Some(0), "resume: {}", resumed.stderr);
    assert!(
        resumed.stdout.contains("is now running"),
        "the verb says what it did: {}",
        resumed.stdout
    );
    assert_eq!(
        manifest_paused(&run.home),
        Some(false),
        "run.json must say the run is no longer paused"
    );
    let again = graph_verb(&run.home, "resume", &run.run_id);
    assert_eq!(
        again.code,
        Some(0),
        "a repeated resume succeeds: {}",
        again.stderr
    );
    assert!(again.stdout.contains("already running"), "{}", again.stdout);

    // The run publishes again: waited for in the bag, not slept.
    let before = run.frames_on(topic);
    wait_for_bag_state(
        &run.bag,
        "the ticker publishing again after the resume",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(topic) >= before + 20,
    );
}

/// Arm 1: a single-process recorded run, paused for five seconds.
#[test]
#[serial]
fn a_pause_leaves_no_gap_and_no_burst_in_a_single_process_recording() {
    let prefix = "gpe2ea";
    let mut run = Recorded::start(prefix, &["--single-process"], SpawnExecutionMode::Default);
    let topic = format!("/{prefix}/ticker/cmd");

    pause_hold_resume(&run, &topic, PAUSE);
    let log = run.stop();
    assert!(
        log.contains("run paused at a step boundary") && log.contains("run resumed"),
        "the run logs the hold it took; log was:\n{log}"
    );
    let s = run.assert_no_gap_no_burst(&topic);
    eprintln!("single-process spacing: {s:?}");
    assert!(
        s.frames >= 40,
        "frames published before and after the pause are both in the bag: {s:?}"
    );
}

/// Arm 2: the counter-test. `SIGSTOP` and `SIGCONT` in place of the verbs leave the
/// gap the verbs exist to remove. Without this arm, arm 1's assertion could pass
/// for a reason that has nothing to do with the pause.
#[test]
#[serial]
fn stopping_the_process_instead_of_pausing_it_leaves_the_gap() {
    let prefix = "gpe2eb";
    let mut run = Recorded::start(prefix, &["--single-process"], SpawnExecutionMode::Default);
    let topic = format!("/{prefix}/ticker/cmd");

    let pid = run.guard.id();
    send_signal(pid, libc::SIGSTOP);
    std::thread::sleep(PAUSE);
    send_signal(pid, libc::SIGCONT);
    let before = run.frames_on(&topic);
    wait_for_bag_state(
        &run.bag,
        "the ticker publishing again after SIGCONT",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(&topic) >= before + 20,
    );
    run.stop();

    let s = spacing(&run.bag, &topic);
    eprintln!("SIGSTOP spacing: {s:?}");
    assert!(
        s.max_gap_ns >= 4_500_000_000,
        "stopping the process for five seconds must show as a gap of about five seconds in the \
         recording (that is the fault a pause removes), got {} ms; {s:?}",
        s.max_gap_ns / 1_000_000
    );
}

/// Arm 3: a multi-process run on the free-run default. Every worker is held, and every
/// worker's recorded stream is contiguous across the pause.
#[test]
#[serial]
fn a_pause_holds_every_worker_of_a_free_run_deployment() {
    let prefix = "gpe2ec";
    let mut run = Recorded::start(prefix, &[], SpawnExecutionMode::Default);
    let ticker = format!("/{prefix}/ticker/cmd");

    pause_hold_resume(&run, &ticker, PAUSE);
    let log = run.stop();
    assert_eq!(
        log.matches("run paused at a step boundary").count(),
        2,
        "BOTH workers hold; log was:\n{log}"
    );
    assert_eq!(
        log.matches("run resumed").count(),
        2,
        "BOTH workers resume; log was:\n{log}"
    );
    for node in ["ticker", "relay", "sink"] {
        let s = run.assert_no_gap_no_burst(&format!("/{prefix}/{node}/cmd"));
        eprintln!("free-run {node} spacing: {s:?}");
    }
}

/// Arm 4: a lockstep deployment held for longer than its five-second boundary
/// timeout. The workers wait at a barrier for each other, so a pause that outlasts
/// the timeout must not read as a dead peer.
#[test]
#[serial]
fn a_pause_longer_than_the_barrier_timeout_does_not_poison_a_lockstep_run() {
    let prefix = "gpe2ed";
    let mut run = Recorded::start(prefix, &[], SpawnExecutionMode::Lockstep);
    let ticker = format!("/{prefix}/ticker/cmd");

    pause_hold_resume(&run, &ticker, Duration::from_secs(7));
    let log = run.stop();
    assert!(
        !log.contains("barrier boundary wait timed out")
            && !log.contains("barrier mid-level wait timed out"),
        "a held lockstep run must not be poisoned by its own pause; log was:\n{log}"
    );
    assert_eq!(
        log.matches("run paused at a step boundary").count(),
        2,
        "BOTH lockstep workers hold; log was:\n{log}"
    );
    run.assert_no_gap_no_burst(&ticker);
}

/// Arm 5: the verbs' exit codes.
#[test]
#[serial]
fn the_verbs_exit_4_for_a_run_that_is_not_running_and_2_for_a_missing_argument() {
    let ghost = "0x00000000000000000000000000000001";
    let empty_home = tempfile::tempdir().unwrap();
    for verb in ["pause", "resume"] {
        let out = graph_verb(empty_home.path(), verb, ghost);
        assert_eq!(
            out.code,
            Some(4),
            "{verb} of a run that is not running exits 4: {}",
            out.stderr
        );
        assert!(
            out.stderr.contains("is not running"),
            "the message says why: {}",
            out.stderr
        );
    }
    for verb in ["pause", "resume"] {
        let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["graph", verb])
            .env("CERULION_NETWORK", "off")
            .output()
            .expect("spawn");
        assert_eq!(
            out.status.code(),
            Some(2),
            "{verb} without a run is a usage error"
        );
    }
}

/// Arm 6: a run on virtual time has no clock to stop, so it has no pause page, and the
/// verb says so rather than pretending.
#[test]
#[serial]
fn a_virtual_time_run_cannot_be_paused_and_the_verb_says_why() {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), "gpe2ee");
    let home = Recorded::home_of(&tmp);
    std::fs::create_dir_all(&home).unwrap();
    let home_str = home.display().to_string();
    let (mut guard, _out, stderr_path) = spawn_graph_run_graph(
        tmp.path(),
        "mpdemo",
        false,
        &["--single-process", "--time-source", "virtual"],
        &[("CERULION_HOME", home_str.as_str())],
        SpawnExecutionMode::Default,
    );
    let run_id = wait_for_run_id(&home, Duration::from_secs(60));
    // Let the run register in the live-run registry.
    std::thread::sleep(Duration::from_secs(2));

    let out = graph_verb(&home, "pause", &run_id);
    assert_eq!(
        out.code,
        Some(1),
        "a run with no pause page is refused with exit 1: {}",
        out.stderr
    );
    assert!(
        out.stderr.contains("no pause page") && out.stderr.contains("virtual"),
        "the refusal names the cause: {}",
        out.stderr
    );
    assert_eq!(
        manifest_paused(&home),
        None,
        "a refused verb must not write the manifest"
    );
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("the run did not stop\nlog:\n{}", read_file(&stderr_path)));
    assert!(status.success(), "{status:?}");
}
