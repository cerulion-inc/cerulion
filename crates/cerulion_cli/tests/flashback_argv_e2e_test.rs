// SPDX-License-Identifier: AGPL-3.0-only
//! What the ALWAYS-ON Flashback recorder is really handed, over
//! the REAL `cerulion` binary.
//!
//! # Why this file exists (the composition gap, named)
//!
//! The other pins each cover ONE seam and are structurally blind to the next:
//! `graph_cmd::tests` drives the pure argv builder against a hand oracle and the
//! REAL `BagdArgs` parser; `the_handoff_offers_exactly_the_run_artifacts_that_can_be_read`
//! drives the shared run-dir reader over a real directory; `state_arm_adoption_test`
//! walks the two call sites' source. What NO arm crossed is the COMPOSITION — a
//! real `cerulion graph run` writing a real run directory, resolving a
//! real handoff out of it, and spawning a real recorder with it — and nothing
//! else in the repo crosses it: `cerulion_cli/tests/` has NO other
//! flashback arm, so without this file the whole always-on spawn has its
//! real-binary behaviour unpinned.
//!
//! That gap is what this file closes. The neighbouring fixtures are NOT part
//! of it, and the distinction is recorded here so it is not re-argued:
//! `flashback_e2e_test.rs` setting `BagdConfig` fields by
//! hand is a DEPENDENCY-INJECTED fixture through a real seam, not fake data
//! (Principle #13) — the same shape as `daemon_e2e_test`'s counting spy
//! `MirrorPlane`, `vizd_e2e_test`'s recording spy `DemandPlane` ("a DI test
//! double — Principle #13, not fake data"), and that very file's own
//! `build_frame` hand-built wire frames. The banned thing is fabricating data and
//! presenting it AS a measurement; a hand-built INPUT driven through the real
//! `run_bagd`, the real `close_capture` and a real `.mcap` is the required
//! oracle discipline. What those arms genuinely could not see is this one.
//!
//! # What is asserted, and why it is the ARGV rather than a capture
//!
//! The recorder's argv is the WHOLE contract between the two processes, and it is
//! the thing PR-β changed. Reading it off the live child proves the composition
//! end to end — real run dir, real resolver, real spawn — without driving a
//! capture cycle (which needs a trigger, a post window and a finalize, i.e. a
//! minute-long arm for a property the argv already carries). A capture's CONTENT
//! is pinned by `cerulion_bagd/tests/flashback_e2e_test.rs`.
//!
//! Hermetic: `CERULION_NETWORK=off` (no gateway, no scouting), `CERULION_HOME`
//! and `CERULION_FLASHBACK_DIR` both under the test's own tempdir, so the run
//! directory this arm reads is one it created and nothing lands in the
//! developer's `~/.cerulion`.
//!
//! Needs `cargo build -p test_node_macro_period_cdylib` first. `#![cfg(unix)]`
//! (signals + `ps`); `#[serial]` + a unique prefix (the run uses the DEFAULT
//! iceoryx2 namespace).

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";

/// A prefix no other run on this machine can be using.
///
/// These arms publish on the DEFAULT iceoryx2 namespace and graph topics are
/// SINGLE-WRITER, so a fixed prefix collides with a publisher port left behind
/// by an earlier SIGKILLed run of the same test — which is exactly what a
/// failing arm's own `ChildGuard` teardown produces, so the next run fails for a
/// reason that has nothing to do with what it asserts. `signal_matrix_e2e_test`
/// carries unique per-test prefixes for the same reason.
fn unique_prefix(stem: &str) -> String {
    // A process-global COUNTER, not the clock alone. `subsec_nanos()` is the
    // sub-second part, so two calls a second apart can produce the same value
    // and two arms of this binary would then share a prefix — at which point a
    // leftover namespace from an earlier failing arm's `ChildGuard` teardown
    // collides and the next run fails for a reason unrelated to what it
    // asserts, which is the exact hazard this helper exists to prevent. The
    // counter cannot repeat within a process; the pid separates processes; the
    // nanos are kept so two processes started in the same second still differ.
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .subsec_nanos();
    format!("{stem}{}_{nanos}_{seq}", std::process::id())
}
/// Generous liveness ceilings — load can delay these, never invert them.
const READY_DEADLINE: Duration = Duration::from_secs(30);
const EXIT_DEADLINE: Duration = Duration::from_secs(40);

/// The run has FINISHED deciding whether to hold a rolling window.
///
/// Emitted unconditionally after the spawn decision on both the monolith and
/// the supervisor arm, which is what makes it usable as a landmark: the GO
/// breadcrumb is emitted BEFORE that decision, so a log snapshot taken there is
/// a snapshot of the moment before the code under test has run.
const FLASHBACK_DECISION: &str = "the window-recorder decision for this run is taken";

/// Strip CSI escape sequences.
///
/// `tracing`'s fmt layer wraps a field's NAME and its `=` in escapes (its `ansi`
/// default is a compile-time feature, not a tty probe), so `window="…"` is NOT a
/// substring of the raw capture and a `key=value` assertion against it is
/// silently unsatisfiable. Same helper, same reason, as
/// `mp_split_pair_e2e_test` — but rebuilt through bytes rather than
/// `byte as char`, because every failure here prints the whole log and this
/// tree's messages are full of em-dashes: the cast version renders them as
/// mojibake, which turns an attributable failure into a puzzle.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && i + 1 < bytes.len() && bytes[i + 1] == b'[' {
            // Skip to the final byte of the CSI sequence (`@`..`~`).
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1; // consume the final byte
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    // Every dropped sequence is whole and ASCII, so what remains is still valid
    // UTF-8; the lossy decode is the belt.
    String::from_utf8_lossy(&out).into_owned()
}

/// `key=value` as a whole WHITESPACE token.
///
/// `tracing` renders fields space-separated, so a bare `contains` would also be
/// satisfied by a longer token that merely starts the same way, or by the same
/// characters appearing in a message's PROSE (the `has_field` rule).
fn has_field(log: &str, field: &str) -> bool {
    log.split_whitespace().any(|tok| tok == field)
}

/// The same minimal single-node workspace `signal_matrix_e2e_test` builds.
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
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures")
        .join(PERIOD_FIXTURE)
        .join("src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        fixture_cdylib(PERIOD_FIXTURE),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("copy fixture cdylib");
}

/// Bounded poll for `needle` in the growing log; panics at the deadline (the
/// caller's `ChildGuard` reaps on the unwind).
fn wait_for_log_line(child: &mut ChildGuard, path: &Path, needle: &str, timeout: Duration) {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return;
        }
        // A DEAD graph is a different diagnosis from a slow one, and both look
        // identical from a log poll — the run's own stderr is empty in exactly
        // the case where it failed before logging anything.
        if let Ok(Some(status)) = child.try_wait_noting() {
            panic!(
                "graph run EXITED ({status:?}) before logging {needle:?}; stderr:\n{}",
                read_file(path)
            );
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{}",
            read_file(path)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// The full argv of every DIRECT child of `pid`, one per line.
///
/// `pgrep -P` + `ps -o args=` — the technique `network_gateway_e2e_test` already
/// uses to find the spawned gateway. Bounded-polled, because the spawn is not
/// waited for by the graph (deliberately: see `spawn_flashback_recorder`), so the
/// child can appear a moment after the breadcrumb.
fn await_child_argv(pid: u32, needle: &str, timeout: Duration, log: &Path) -> String {
    let start = Instant::now();
    let mut seen: Vec<String> = Vec::new();
    loop {
        let children = Command::new("pgrep")
            .args(["-P", &pid.to_string()])
            .output()
            .expect("pgrep");
        let stdout = String::from_utf8_lossy(&children.stdout).to_string();
        seen.clear();
        for child in stdout.split_whitespace() {
            // `-ww` = never truncate to a terminal width. Both BSD and procps
            // already treat a piped stdout as unlimited, but `--state-tag` is the
            // LAST argv pair `flashback_argv` emits, so an environment that
            // decided otherwise would silently truncate exactly the assertion
            // that reads the full 40-character tag.
            let args = Command::new("ps")
                .args(["-ww", "-o", "args=", "-p", child])
                .output()
                .expect("ps");
            let line = String::from_utf8_lossy(&args.stdout).trim().to_string();
            if line.contains(needle) {
                return line;
            }
            seen.push(line);
        }
        // ATTRIBUTABLE on failure: the children that WERE there, plus the run's
        // own log — a recorder that spawned and then exited (a rejected argv, an
        // unwritable directory) is a different diagnosis from one never spawned,
        // and both look like "no child" from here.
        assert!(
            start.elapsed() < timeout,
            "no child of {pid} whose argv contains {needle:?} within {timeout:?}.\n\
             children seen: {seen:#?}\nrun log:\n{}",
            read_file(log)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// **THE ARM.** A real `graph run` writes a real run directory, resolves a real
/// handoff out of it, and hands the always-on recorder the capture plane's tag
/// plus all three run-context artifacts — read off the LIVE child's argv.
///
/// Every assertion names the seam it covers, because each one is invisible to a
/// different sibling pin: the pure argv arms cannot see a call site passing
/// `None`, the structural walk cannot see a resolver that returns an empty
/// handoff at run time, and neither can see a run directory that was never
/// written.
#[test]
#[serial]
fn a_real_graph_run_hands_its_recorder_the_plane_and_the_run_context() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, &unique_prefix("fbargv"));
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();

    let stderr_path = root.join("run.stderr");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "demo", "--no-validate", "--single-process"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic on all three axes: no network, our own run-directory root,
        // our own capture directory.
        .env("CERULION_NETWORK", "off")
        .env("CERULION_HOME", &home)
        .env("CERULION_FLASHBACK_DIR", &flashbacks)
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    // `--single-process`: a monolith with `CERULION_NETWORK=off` spawns no
    // worker and no gateway, so there is no subtree to signal.
    let mut guard = ChildGuard::single_process(child);
    let pid = guard.id();

    // The always-on spawn happens on the way into the live loop.
    wait_for_log_line(
        &mut guard,
        &stderr_path,
        "flashback: holding a rolling window",
        READY_DEADLINE,
    );
    let argv = await_child_argv(pid, "--flashback-window-only", READY_DEADLINE, &stderr_path);

    // The run directory this run really wrote — resolved from the filesystem,
    // not from the argv, so the argv's paths are checked against an independent
    // answer rather than against themselves.
    // `CERULION_HOME` IS the config root (`run_registry::run_dir_root` joins
    // `runs` directly onto it — the `.cerulion` component is only in the
    // home-directory fallback), so this is where C0 writes.
    let runs = home.join("runs");
    let run_dir: PathBuf = std::fs::read_dir(&runs)
        .unwrap_or_else(|e| panic!("the run directory must exist under {}: {e}", runs.display()))
        .filter_map(Result::ok)
        .map(|e| e.path())
        .find(|p| p.is_dir())
        .unwrap_or_else(|| panic!("no run directory under {}", runs.display()));

    // (1) THE HEADLINE: the capture plane's tag reached the recorder — the EXACT
    // tag this run armed, not merely something tag-SHAPED. Without the tag the
    // sweep never runs and the anchors every serving run now takes go into a ring
    // no process was ever told about; with the WRONG one it sweeps somebody
    // else's rank space, which is the adoption hazard rather than a
    // missing feature. A prefix check cannot tell those three apart.
    //
    // Derived from the run's OWN artifact and cross-checked against the run
    // directory's identity, so neither the argv nor a single file is trusted
    // alone: `run.json` carries `"run_id": "0x<32 hex>"` and the
    // directory is named `<graph>-<32 hex>` from the same mint.
    let manifest: serde_json::Value = serde_json::from_str(&read_file(&run_dir.join("run.json")))
        .expect("the run directory must carry a parseable run.json");
    let run_id_hex = manifest["run_id"]
        .as_str()
        .expect("run.json carries run_id as a string")
        .strip_prefix("0x")
        .expect("run.json renders run_id as 0x-prefixed hex")
        .to_string();
    assert_eq!(run_id_hex.len(), 32, "run_id is a u128 in 32 hex digits");
    let dir_name = run_dir.file_name().unwrap().to_string_lossy().to_string();
    assert!(
        dir_name.ends_with(&format!("-{run_id_hex}")),
        "the run directory's own name must carry the same run_id as its manifest \
         ({dir_name} vs {run_id_hex})"
    );
    // HAND oracle for the tag recipe (`state_arm_tag_for_run`), so this arm fails
    // on a changed recipe rather than following it.
    let expected_tag = format!("cer_run_{run_id_hex}");
    assert!(
        argv.contains(&format!("--state-tag {expected_tag}")),
        "the always-on recorder must be handed THIS run's armed capture-plane tag \
         (expected `--state-tag {expected_tag}`): {argv}"
    );

    // (2) The run CONTEXT, each from THIS run's own directory. A capture with no
    // graph.yaml can be viewed and never re-executed.
    for artifact in ["graph.yaml", "env.json"] {
        let expected = format!("--attach {artifact}:{}", run_dir.join(artifact).display());
        assert!(
            argv.contains(&expected),
            "the recorder must be handed {artifact} from this run's own directory \
             (expected {expected:?}): {argv}"
        );
    }
    assert!(
        argv.contains(&format!(
            "--recorder-json {}",
            run_dir.join("recorder.json").display()
        )),
        "the host identity rides its own path-only flag: {argv}"
    );

    // (3) …and the DELIBERATE absences, so adding one is a decision rather than
    // a drift (each reason is stated at `flashback_argv`).
    assert!(
        !argv.contains("--ring "),
        "**A later change re-anchored this reason.** It used to read \"a non-recording run mints no \
         trace ring\" — which is no longer why. Every MULTI-PROCESS run provisions rings now; \
         this arm runs `--single-process`, whose gating clock is wall-driven, and the \
         design deliberately leaves those shapes ringless because a trace taken there \
         would carry boundaries a resim cannot re-advance to. So there is still none to hand \
         over — for a different reason, and the mp twin below asserts the other half: {argv}"
    );
    assert!(
        !argv.contains("--armed-before-producers"),
        "this recorder is not waited for, so it cannot make the arm-ordering guarantee: {argv}"
    );

    // Clean teardown: the graph reaps its recorder on a graceful SIGINT.
    send_signal(pid, libc::SIGINT);
    let status = guard.wait_bounded(EXIT_DEADLINE).unwrap_or_else(|| {
        panic!(
            "graph run did not exit after SIGINT; stderr so far:\n{}",
            read_file(&stderr_path)
        )
    });
    assert_eq!(
        status.code(),
        Some(0),
        "a clean SIGINT shutdown returns 0, got {status:?}"
    );
}

/// Read a live run's `run.json` out of a `CERULION_HOME` this test owns.
fn read_run_manifest(home: &Path) -> serde_json::Value {
    let runs = home.join("runs");
    let run_dir: PathBuf = std::fs::read_dir(&runs)
        .unwrap_or_else(|e| panic!("the run directory must exist under {}: {e}", runs.display()))
        .filter_map(Result::ok)
        .map(|e| e.path())
        .find(|p| p.is_dir())
        .unwrap_or_else(|| panic!("no run directory under {}", runs.display()));
    serde_json::from_str(&read_file(&run_dir.join("run.json")))
        .expect("the run directory must carry a parseable run.json")
}

/// Spawn a `graph run` with the given extra flags, hermetic on all three axes.
fn spawn_run(root: &Path, home: &Path, flashbacks: &Path, extra: &[&str]) -> (ChildGuard, PathBuf) {
    spawn_run_with_env(root, home, flashbacks, extra, &[])
}

fn spawn_run_with_env(
    root: &Path,
    home: &Path,
    flashbacks: &Path,
    extra: &[&str],
    env: &[(&str, &str)],
) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join(format!("run{}.stderr", extra.join("")));
    let mut args = vec!["graph", "run", "demo", "--no-validate"];
    args.extend_from_slice(extra);
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    for (k, v) in env {
        cmd.env(k, v);
    }
    cmd.args(&args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .env("CERULION_HOME", home)
        .env("CERULION_FLASHBACK_DIR", flashbacks)
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        // The no-TTY consent floor: the partition is derived IN MEMORY
        // and the graph file is never touched, which is the headless/robot shape
        // and the one this arm is about.
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    // The derived partition runs MULTI-PROCESS, so this supervisor has workers
    // and must lead its own group for teardown to reach them.
    let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
    (guard, stderr_path)
}

/// **THE HEADLINE ACCEPTANCE: a plain multi-process `graph run` hands
/// its window recorder the trace rings it created.**
///
/// This is the property the always-on rings rule turns on. Before rings were unconditional the
/// supervisor stamped ring tags under `if record.is_some()`, so the DEFAULT run
/// shape minted nothing, `flashback_argv` deliberately passed no `--ring`, and
/// every capture of that run reported `ResimGap::NoTrace`. The single-process
/// arm above is now the ONLY shape that still hands none, and for a different
/// reason (the wall-gated shapes stay ringless by design).
///
/// The oracle is the LIVE child's argv plus the run's OWN manifest, cross-checked
/// against each other: the manifest is what a later `cerulion bag record --run`
/// reads, the argv is what the standing recorder got, and the two disagreeing is
/// exactly the shape where one of them is lying. Neither is derived from the
/// other — the argv is read off `ps`, the manifest off the filesystem.
///
/// It is the 1-WORKER-GROUP shape (a one-node graph, so process-per-node derives
/// one worker), which is also the smallest deployment there is and the one the
/// capture judge used to refuse `AmbiguousNodeMap` for counting the departure
/// ring.
#[test]
#[serial]
fn a_plain_multi_process_run_hands_its_recorder_the_rings_it_created() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, &unique_prefix("fbmp"));
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();

    let (mut guard, stderr_path) = spawn_run(root, &home, &flashbacks, &[]);
    wait_for_log_line(
        &mut guard,
        &stderr_path,
        "flashback: holding a rolling window",
        READY_DEADLINE,
    );
    let argv = await_child_argv(
        guard.id(),
        "--flashback-window-only",
        READY_DEADLINE,
        &stderr_path,
    );

    // (1) The recorder really was handed rings. TWO — the one worker's, plus the
    // supervisor's departure ring, which exists on a plain run only because
    // it was lifted out of the recording session.
    let handed: Vec<&str> = argv
        .split_whitespace()
        .zip(argv.split_whitespace().skip(1))
        .filter(|(flag, _)| *flag == "--ring")
        .map(|(_, name)| name)
        .collect();
    assert_eq!(
        handed.len(),
        2,
        "a 1-group multi-process run holds ONE worker ring plus the departure ring, and both \
         must reach the standing recorder — without them a capture of this run reports \
         `ResimGap::NoTrace`, which is the whole point: {argv}"
    );
    for name in &handed {
        assert!(
            name.starts_with("/cer_rg_"),
            "a `--ring` value is a RESOLVED POSIX SHM name, not the supervisor's tag — handing \
             the tag over reaches `shm_open` verbatim and ENAMETOOLONGs on macOS: {name}"
        );
    }

    // (2) …and the run SAYS so, in the manifest a later `bag record --run` reads.
    // Independently sourced from the argv, so the two corroborate rather than
    // one echoing the other.
    let manifest = read_run_manifest(&home);
    assert_eq!(
        manifest["trace_rings"],
        serde_json::json!("declared"),
        "a run that created rings declares them — an absent or degraded state sends every \
         later reader to the UNKNOWN arm: {manifest}"
    );
    let declared = manifest["rings"]
        .as_array()
        .expect("a declared run lists its rings");
    assert_eq!(
        declared.len(),
        2,
        "the manifest names the same two rings the recorder was handed: {manifest}"
    );
    assert_eq!(
        manifest["declared_unavailable"],
        serde_json::json!([]),
        "no rank failed, so none is named unavailable — a populated list here on a healthy run \
         would tell a reader not to open a ring that exists: {manifest}"
    );
    // The declared TAGS resolve to the handed NAMES. This is the cross-check: a
    // manifest declaring rings the recorder was not given, or the reverse, is a
    // run whose two statements about itself disagree.
    for ring in declared {
        let tag = ring["tag"].as_str().expect("each ring declares its tag");
        let resolved = cerulion_core::shm_ring::ring_shm_name(tag);
        assert!(
            handed.contains(&resolved.as_str()),
            "every declared ring must be one the recorder was handed ({tag} -> {resolved}): \
             {argv}"
        );
    }

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(EXIT_DEADLINE)
        .expect("graph run must exit");
    assert_eq!(status.code(), Some(0), "a graceful SIGINT exits cleanly");
}

/// **By design, `--no-rings` declines the rings and the
/// window recorder.**
///
/// With no trace rings nothing captured could ever be re-executed, and the project's rule
/// says a capture must always be. Rather than take frames-only captures under a
/// labelled exception, the run takes NONE — so the assertion that matters is the
/// ABSENCE of the recorder child, not merely the absence of `--ring`.
///
/// The ANTI-TAUTOLOGY half is the arm above, on the same workspace and the same
/// harness: without it, "no recorder child" is satisfied by a build whose
/// always-on spawn is broken outright.
#[test]
#[serial]
fn no_rings_declines_both_the_rings_and_the_window_recorder() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, &unique_prefix("fbnorings"));
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();

    let (mut guard, stderr_path) = spawn_run(root, &home, &flashbacks, &["--no-rings"]);
    // Drive to LIVE on a landmark that has nothing to do with Flashback, so the
    // absence assertions below are made against a run that really started.
    wait_for_log_line(
        &mut guard,
        &stderr_path,
        "GO signaled; deployment live",
        READY_DEADLINE,
    );
    // …then to the point where the decision under test has actually been TAKEN.
    // The supervisor spawns (or declines to spawn) the window recorder AFTER
    // emitting GO, so a snapshot read at the line above is a snapshot of the
    // moment BEFORE this test's subject ran: on a broken `--no-rings` path that
    // spawned a recorder which exited at once, the breadcrumb would not be
    // written yet and the child would be gone by the time the poll below looks,
    // so BOTH checks would pass. Waiting on a landmark emitted after the
    // decision is what makes the absences below mean something.
    wait_for_log_line(&mut guard, &stderr_path, FLASHBACK_DECISION, READY_DEADLINE);
    let log = strip_ansi(&read_file(&stderr_path));
    assert!(
        log.contains("--no-rings: this run declines"),
        "the flag is announced at launch rather than silently swallowed:\n{log}"
    );

    // The decision stated POSITIVELY, rather than inferred from an absence: the
    // supervisor names the outcome it reached, and it must be the `--no-rings`
    // decline. A recorder that spawned and then died is a SUCCESSFUL spawn —
    // the guard is `Some` the instant `spawn()` returns — so it reads
    // `own_recorder` here whether or not the child is still alive.
    assert!(
        has_field(&log, r#"window="declined_no_rings""#),
        "`--no-rings` must reach its window-recorder decision and NAME that flag as the cause — \
         a spawn that succeeded and then died still reports `window=\"own_recorder\"`:\n{log}"
    );

    // The SUPERVISOR-SIDE half, and the authoritative one: a recorder that was
    // spawned and then exited at once — invalid argv, an immediate failure —
    // disappears between two polls of the child list below, so live-child
    // polling alone cannot tell "never spawned" from "spawned and gone".
    // `spawn_flashback_recorder` logs this line on a SUCCESSFUL spawn and
    // nothing removes it afterwards, so its absence is evidence about the
    // ATTEMPT rather than about the survivor.
    assert!(
        !log.contains("holding a rolling window for this run"),
        "`--no-rings` must not even ATTEMPT the window recorder — this line is logged on a \
         successful spawn, so its presence means one was started whether or not it is still \
         alive:\n{log}"
    );

    // NO recorder child. RETAINED alongside the two log assertions, and given a
    // bounded window ON PURPOSE: they are read ONCE, at the decision landmark,
    // so between them they cover a recorder started up to that point — while
    // this covers one started AFTERWARDS, from anywhere the log assertions
    // cannot see (a later seam, a re-spawn, a child of a child).
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        let children = std::process::Command::new("pgrep")
            .args(["-P", &guard.id().to_string()])
            .output()
            .expect("pgrep");
        for child in String::from_utf8_lossy(&children.stdout).split_whitespace() {
            let args = std::process::Command::new("ps")
                .args(["-ww", "-o", "args=", "-p", child])
                .output()
                .expect("ps");
            let line = String::from_utf8_lossy(&args.stdout).trim().to_string();
            assert!(
                !line.contains("--flashback-window-only"),
                "`--no-rings` must not start a window recorder: with no trace rings nothing it \
                 captured could be re-executed, and the project rule admits no frames-only \
                 exception. Child argv was: {line}"
            );
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    let manifest = read_run_manifest(&home);
    let state = manifest["trace_rings"]
        .as_str()
        .expect("a declining run states its choice");
    assert!(
        state.starts_with("declined:"),
        "the run records the CHOICE, so a later reader can tell it apart from a run that \
         wanted rings and could not have them: {manifest}"
    );
    assert_eq!(
        manifest["rings"],
        serde_json::json!([]),
        "…and names none, departure ring included: {manifest}"
    );
    // By design, an absence explanation names the cause, never another verb's
    // flag. The reader of `run.json` is holding a bag, not a command line.
    assert!(
        !state.contains("--no-rings"),
        "the run's own statement names the cause, not the flag: {state}"
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(EXIT_DEADLINE)
        .expect("graph run must exit");
    assert_eq!(status.code(), Some(0), "a graceful SIGINT exits cleanly");
}

/// **`--no-rings --record` is refused, and a refused run mutates
/// nothing.**
///
/// A recording without a scheduler trace is not a recording — the bag would
/// finalize looking complete while `bag play --resim` refused it. Refused at
/// PARSE, so the refusal costs no SHM, no workers and no bag; the graph file
/// being byte-identical afterwards is the never-mutate floor, asserted
/// here because a run rejected AFTER the auto-partition consent ladder would
/// have touched it.
#[test]
#[serial]
fn no_rings_with_record_is_refused_and_the_graph_file_is_untouched() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, &unique_prefix("fbconflict"));
    let graph = root.join("graphs/demo.yaml");
    let before = std::fs::read(&graph).unwrap();

    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "demo", "--no-rings", "--record=recordings"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .stdin(Stdio::null())
        .output()
        .expect("spawn cerulion graph run");

    assert!(
        !out.status.success(),
        "a recording with no scheduler trace is not a recording — it must be refused: \
         {out:?}"
    );
    let text = String::from_utf8_lossy(&out.stderr);
    assert!(
        text.contains("--no-rings") && text.contains("--record"),
        "…and the refusal names BOTH flags, since either one is the operator's to drop: {text}"
    );
    assert_eq!(
        std::fs::read(&graph).unwrap(),
        before,
        "a refused run never mutates the graph file (the never-mutate floor)"
    );
    assert!(
        !root.join("recordings").exists(),
        "…and writes no bag directory: the refusal is at parse, before anything is created"
    );
}

/// **A rank whose ring create FAILS degrades, and every surface says
/// which rank, rather than any of them claiming it works.**
///
/// This is the path a plain run is DESIGNED to take when `/dev/shm` cannot hold
/// another 40 MiB, and it is otherwise unreachable from a test: a ring create
/// fails only when the filesystem genuinely refuses, and filling a runner's tmpfs
/// to produce that is neither hermetic nor safe. `CER_FAIL_MODE_TRACE_RING_RANK`
/// makes the named rank fail as if the OS had refused it — read from the
/// WORKER's own env, so it can never reach a recording.
///
/// THREE claims, and the third is the one a stamped tag cannot support: the run
/// still runs (a serving graph that loses its black box must still serve), the
/// manifest names the failed rank WITH its reason, and the standing recorder is
/// handed no `--ring` for it. Tags are spelled BEFORE the rings exist, so
/// handing one over on the strength of the stamp points `shm_open` at a name
/// nothing created.
#[test]
#[serial]
fn a_rank_whose_ring_create_fails_is_declared_unavailable_and_handed_to_no_recorder() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(root, &unique_prefix("fbfail"));
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();

    let (mut guard, stderr_path) = spawn_run_with_env(
        root,
        &home,
        &flashbacks,
        &[],
        &[("CER_FAIL_MODE_TRACE_RING_RANK", "0")],
    );
    // (1) The run still RUNS. A plain run's ring failure is a DEGRADE, not a
    // refusal — asserted on a landmark that has nothing to do with Flashback.
    wait_for_log_line(
        &mut guard,
        &stderr_path,
        "GO signaled; deployment live",
        READY_DEADLINE,
    );
    let argv = await_child_argv(
        guard.id(),
        "--flashback-window-only",
        READY_DEADLINE,
        &stderr_path,
    );

    // (2) The manifest names the failed rank, WITH the reason the worker gave.
    let manifest = read_run_manifest(&home);
    let unavailable = manifest["declared_unavailable"]
        .as_array()
        .expect("a declared run lists the ranks whose ring was never created");
    assert_eq!(
        unavailable.len(),
        1,
        "exactly the failing rank is named — naming none loses the fact, naming more claims \
         failures nobody observed: {manifest}"
    );
    assert_eq!(unavailable[0]["rank"], serde_json::json!(0));
    assert!(
        unavailable[0]["reason"]
            .as_str()
            .is_some_and(|r| !r.trim().is_empty()),
        "…and it carries WHY, because a rank named unavailable with no reason is a shrug: \
         {manifest}"
    );

    // (3) The recorder is handed ONLY the departure ring — the failed rank's tag
    // is in `rings` (stamped before creation) and must NOT be handed over.
    let handed: Vec<&str> = argv
        .split_whitespace()
        .zip(argv.split_whitespace().skip(1))
        .filter(|(flag, _)| *flag == "--ring")
        .map(|(_, name)| name)
        .collect();
    assert_eq!(
        handed.len(),
        1,
        "only the departure ring survives: handing over a tag whose ring was never created \
         points `shm_open` at a name nothing made, and a stamped tag is not evidence: {argv}"
    );
    let failed_tag = manifest["rings"]
        .as_array()
        .expect("rings are stamped before creation, so the failed rank is still listed")
        .iter()
        .find(|r| r["rank"] == serde_json::json!(0))
        .and_then(|r| r["tag"].as_str())
        .expect("the failed rank's tag is declared")
        .to_string();
    assert!(
        !handed.contains(&cerulion_core::shm_ring::ring_shm_name(&failed_tag).as_str()),
        "…and specifically not THAT one: {argv}"
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(EXIT_DEADLINE)
        .expect("graph run must exit");
    assert_eq!(
        status.code(),
        Some(0),
        "a degraded ring never fails the run"
    );
}
