// SPDX-License-Identifier: AGPL-3.0-only
//! Stage A, THE CLOSED LOOP: a Flashback capture taken off a real
//! `graph run --record` is a bag `cerulion bag play --resim` really accepts.
//!
//! # Why this file exists at all
//!
//! Stage A's product is a SENTENCE — a capture's manifest says `resimmable`,
//! and `cerulion flashback` prints a copy-paste `bag play --resim` command
//! beside it. Every other test in this change proves a HALF of that: the shared
//! judge's arms are oracle-tested in `cerulion_core::flashback::resim`, the
//! trim and the manifest attachments are pinned over a real recorder in
//! `cerulion_bagd/tests/flashback_trace_e2e_test.rs`, and the replay gate's
//! manifest contract is pinned on crafted bags in
//! `cerulion_cli_engine/tests/replay_gates_test.rs`.
//!
//! None of them can see the two halves DISAGREE, and that is the failure this
//! change was opened for: the capture writer published `resimmable: true` while
//! writing no `__cerulion/trace_manifest_rank<N>.json` at all, and
//! `replay_cmd::load_trace_manifests` refuses a bag with zero manifests
//! outright (`BagMissingAttachment`, exit 2). So the shipped verdict was a
//! confident claim the shipped gate contradicted, and the only thing that can
//! observe it is a run that takes a REAL capture and hands it to the REAL
//! verb.
//!
//! # The recipe (every leg is the shipped binary)
//!
//! 1. **RECORD** — `cerulion graph run fbresim --record=recordings
//!    --single-process` against a hand-built tempdir workspace whose 3-node
//!    chain (`ticker`(Period) → `relay` → `sink`) is the repo's standard record
//!    fixture. `--single-process` is deliberate: one worker means ONE trace
//!    ring and ONE state ring, which is the shape the shared judge can call
//!    resimmable at all (several state rings is its `MultiRing` arm).
//! 2. **CAPTURE** — `cerulion flashback`, the operator's own verb, into a
//!    per-run `CERULION_FLASHBACK_DIR`.
//! 3. **RESIM** — `cerulion bag play <capture> --resim all` on the capture.
//!
//! # What is asserted, and why in this shape
//!
//! The headline is EXIT 0 from leg 3, which is reachable only through the
//! manifest gate. Beside it the arm asserts the two things that stop an exit-0
//! from being vacuous: the capture's own manifest says `resimmable: true` (so
//! the two answers AGREE rather than both being absent), and the resim summary
//! reports a NONZERO re-executed step count (so a resim that loaded the bag and
//! ran nothing cannot pass).
//!
//! `--resim all` WITHOUT `--verify` on purpose. Exit 0 there means "the
//! re-execution could be performed" — bag replay-grade, cdylibs loaded, no
//! candidate panic — which is exactly the gate under test. `--verify` would
//! additionally couple this arm to byte-exactness, a separate property with its
//! own e2e (`mp_record_replay_e2e_test`), and would let an unrelated
//! determinism regression report itself here as a manifest failure.
//!
//! The ASSERTION ORDER is load-bearing.
//! The rank-0 attachment is checked LAST, after the resim, because read earlier it
//! PREEMPTS the headline: reverting the manifest write then fails on an attachment
//! count rather than on `bag play --resim` REFUSING the bag, which is the
//! failure this arm exists to reproduce.
//!
//! Deleting the capture's manifest write in
//! `cerulion_bagd::Recorder::close_capture` fails this arm at LEG 3 with
//! `Error: bag is missing the required attachment
//! __cerulion/trace_manifest_rank0.json` and exit 2 — while the capture's own
//! manifest, asserted a few lines above and unaffected by that deletion, reads
//! `resimmable: true`. The arm reports exactly the disagreement it exists for.
//!
//! # WHAT THIS ARM USED TO REPRODUCE, AND WHY IT NOW RUNS
//!
//! It shipped `#[ignore]`d, as a REPRODUCTION of a defect Stage A did not fix.
//! MEASURED on this desk, 6 consecutive runs of the arm exactly as written:
//! **3 passed, 3 failed**, every failure the SAME signature and never the
//! manifest —
//!
//! ```text
//! Error: bag internal consistency check failed: frame 311 on graph-produced
//! topic '/fbresim73027/ticker/cmd' is stamped 16102963042 ns, which matches NO
//! recorded STEP_BOUNDARY target — the frame and the trace disagree about when
//! it was published — the recording is corrupt or hand-edited
//! ```
//!
//! The three failing stamps were 15.55 s, 15.80 s and 16.10 s against a
//! ~16.1 s capture span: the very TAIL of the window, every time.
//!
//! The mechanism is read off the source, not inferred from the timing. A
//! capture has TWO producers and they are on different threads: frames are
//! staged by the DRIVE LOOP (`Recorder::harvest_window`) while the scheduler
//! trace is drained and banked by the WRITER THREAD
//! (`WriterCore::write_batch` -> `admit_trace_batch`). `close_capture` runs on
//! the drive loop and reads both with no rendezvous between them, so the frame
//! window routinely ends one writer cycle AHEAD of the trace window — and
//! `replay_engine`'s consistency check requires every graph-produced frame's
//! timestamp to match a recorded `STEP_BOUNDARY` target. So roughly half of all
//! Flashback captures on a `--record` run were stamped `resimmable: true` and
//! then refused by `bag play --resim` as "corrupt".
//!
//! **The fix landed at the CLAIM rather than at the data**.
//! The bag keeps every frame (a black box
//! never discards evidence), the capture DECLARES the range its resume covers
//! (`anchor.resim_covered_through_ns`, the last step boundary its own trace
//! carries), and resim replays exactly that far and stops CLEANLY, reporting the
//! remainder. Neither rejected candidate was taken: no frame is trimmed from the
//! bag, and the drain still never waits on the writer.
//!
//! So this arm is now a GATE rather than a repro, and it is the only thing in
//! the repo that can see the two halves disagree: the recorder's measurement of
//! what it carries and the replayer's use of it live in two crates that meet in
//! no unit test. It carries the covered-range assertions to match — a capture
//! off a real run must STATE its range, and the replay must report having
//! covered it.
//!
//! It is no longer `#[ignore]`d. It stays out of the fast lane by COST, not by
//! flakiness: three real binaries plus the shipped 15 s post window, ~60-90 s.
//! MEASURED after the fix: 6 consecutive runs, 6 passed.
//!
//! Run it with:
//! `cargo test -p cerulion_cli --test flashback_resim_e2e_test -- --nocapture`
//!
//! `#[cfg(unix)]` (the recorder, the trigger channel and the replay reader all
//! are) and `#[serial]`: the run's DATA plane and the `/__cerulion/flashback`
//! trigger channel are on the DEFAULT iceoryx2 namespace, so two of these at
//! once would answer each other's requests. Per-run unique graph prefix keeps
//! topic names apart from the sibling suites.
//!
//! Prerequisites (the repo's fixture pattern — the helper PANICS with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// Liveness ceilings, in seconds. Load can only DELAY these, never invert them
/// (the load-inversion rule): none of them is a wall stated in units of the thing
/// under test.
const BAG_APPEARS: Duration = Duration::from_secs(90);
/// The capture verb waits out the shipped post window (15 s) and then the bag
/// write, so its ceiling is generous by a wide margin.
const CAPTURE_COMPLETES: Duration = Duration::from_secs(180);
const RUN_EXITS: Duration = Duration::from_secs(60);
const RESIM_COMPLETES: Duration = Duration::from_secs(300);

/// SIGKILL + reap on drop so a panicking test never leaks the run.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// bagd GRANDCHILD leak guard: bagd runs in its OWN process group, so killing
/// the run on a mid-window panic orphans it. The relative `--out
/// recordings/fbresim_` cmdline form is unique to THIS file's graph name (the
/// sibling suites use `demo_` / `mpdemo_`).
struct BagdGuard;
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep")
            .args(["-f", "bagd --out recordings/fbresim_"])
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

fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<std::process::ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// Hand-build the recording workspace: a `[workspace]` Cargo.toml, the 3-node
/// chain's graph, per-type fixture SOURCE copies (the metadata walkers read
/// them) and the prebuilt fixture cdylibs under `target/debug/`.
///
/// Deliberately NOT `mp_support::build_mp_workspace`: that one writes a
/// `process_groups:` block, and this arm wants the single-ring shape.
fn build_workspace(root: &Path, prefix: &str) {
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
    for (node_type, fixture) in [
        ("ticker", "test_node_macro_period_cdylib"),
        ("relay", "test_node_macro_data_trigger_cdylib"),
        ("sink", "test_node_macro_data_trigger_cdylib"),
    ] {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            cerulion_core::testing::find_fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    std::fs::write(
        root.join("graphs/fbresim.yaml"),
        format!(
            "name: fbresim\n\
             prefix: {prefix}\n\
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
        ),
    )
    .unwrap();
}

/// Every `.mcap` directly under `dir`, sorted.
fn mcaps(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
                .collect()
        })
        .unwrap_or_default();
    out.sort();
    out
}

fn read_file(p: &Path) -> String {
    std::fs::read_to_string(p).unwrap_or_default()
}

/// THE CLOSED LOOP — see the module docs.
#[test]
#[serial]
fn a_capture_taken_off_a_real_record_run_is_a_bag_bag_play_resim_accepts() {
    let dir = tempfile::tempdir().expect("tempdir");
    let root = dir.path();
    let prefix = format!("fbresim{}", std::process::id());
    build_workspace(root, &prefix);
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&flashbacks).unwrap();
    let _bagd_guard = BagdGuard;

    // ---------------------------------------------------------------- leg 1
    // RECORD. `--single-process` is the shape under test: one worker, so ONE
    // trace ring and ONE state ring — several state rings is the shared
    // judge's `MultiRing` refusal and could never reach a resimmable verdict.
    let run_out = root.join("run.stdout");
    let run_err = root.join("run.stderr");
    let mut run = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args([
                "graph",
                "run",
                "fbresim",
                "--record=recordings",
                "--single-process",
            ])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            // Hermetic: a real-clock run is permissive-by-default, and the
            // kill-switch keeps this LOCAL-ONLY with no scouting session.
            .env("CERULION_NETWORK", "off")
            .env("CERULION_FLASHBACK_DIR", &flashbacks)
            .env("RUST_LOG", "cerulion=info,cerulion_bagd=info")
            .stdout(Stdio::from(std::fs::File::create(&run_out).unwrap()))
            .stderr(Stdio::from(std::fs::File::create(&run_err).unwrap()))
            .spawn()
            .expect("spawn cerulion graph run --record"),
    );

    // The continuous bag appearing is the recorder's own proof of life: it is
    // created after the taps are armed, which is the point past which a capture
    // has a window to draw from.
    let start = Instant::now();
    let recordings = root.join("recordings");
    while mcaps(&recordings).is_empty() {
        assert!(
            start.elapsed() < BAG_APPEARS,
            "the recorder never wrote its continuous bag — stderr:\n{}",
            read_file(&run_err)
        );
        assert!(
            run.0.try_wait().expect("try_wait").is_none(),
            "the run exited before recording started — stderr:\n{}",
            read_file(&run_err)
        );
        std::thread::sleep(Duration::from_millis(100));
    }

    // ---------------------------------------------------------------- leg 2
    // CAPTURE, through the operator's own verb. It waits out the post window
    // and prints the FINISHED verdict, so its exit is the capture's.
    let mut flash = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["flashback", "--note", "closed-loop probe"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_FLASHBACK_DIR", &flashbacks)
            .stdout(Stdio::from(
                std::fs::File::create(root.join("flash.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(
                std::fs::File::create(root.join("flash.stderr")).unwrap(),
            ))
            .spawn()
            .expect("spawn cerulion flashback"),
    );
    let flash_status = wait_bounded(&mut flash.0, CAPTURE_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
    let flash_stdout = read_file(&root.join("flash.stdout"));
    assert!(
        flash_status.success(),
        "`cerulion flashback` must capture: {flash_status:?}\nstdout:\n{flash_stdout}\nstderr:\n{}",
        read_file(&root.join("flash.stderr"))
    );

    // Our OWN capture directory, because the request is BROADCAST: any other
    // recorder serving on this desk answers it too, into ITS dir.
    let captures = mcaps(&flashbacks);
    assert_eq!(
        captures.len(),
        1,
        "exactly one capture in this run's own directory, got {captures:?}\nstdout:\n{flash_stdout}"
    );
    let capture = captures[0].clone();

    // The run has served its purpose; stop it before the resim so the replay's
    // transport does not share a desk with the recording's.
    let run_pid = run.0.id();
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(run_pid as libc::pid_t, libc::SIGINT);
    }
    let run_status = wait_bounded(&mut run.0, RUN_EXITS).unwrap_or_else(|| {
        panic!(
            "the run did not exit on SIGINT — stderr:\n{}",
            read_file(&run_err)
        )
    });
    assert!(
        run_status.success(),
        "the run must shut down cleanly: {run_status:?} — stderr:\n{}",
        read_file(&run_err)
    );

    // --------------------------------------------------- the capture's CLAIM
    // Read BEFORE the resim, so a failing resim below is reported beside the
    // verdict it contradicts rather than instead of it.
    let reader = cerulion_bag::BagReader::open(&capture).expect("open the capture");
    let manifest: serde_json::Value = serde_json::from_slice(
        &reader
            .attachment("__cerulion/flashback.json")
            .expect("read the capture manifest")
            .expect("a capture carries its own manifest")
            .data,
    )
    .expect("the capture manifest is valid JSON");
    // `resimmable` rides the manifest's `anchor` block (a verdict ABOUT the
    // embedded anchor), not the top level — the same path
    // `flashback_trace_e2e_test` reads it at.
    assert_eq!(
        manifest["anchor"]["resimmable"],
        serde_json::json!(true),
        "this fixture's capture must CLAIM resimmable, or the exit-0 below proves nothing about \
         the two answers agreeing: {manifest}"
    );
    // …and the claim must name the RANGE it covers. This is the one
    // place the recorder's measurement is read off a REAL capture: everything
    // else that asserts on this field feeds a hand-built `CaptureManifest`, and
    // could not tell a plumbed value from a hardcoded one.
    let covered = manifest["anchor"]["resim_covered_through_ns"]
        .as_u64()
        .unwrap_or_else(|| {
            panic!(
                "a resimmable capture off a real run must STATE the instant its claim reaches — a \
                 `null` here means the recorder measured no boundary in the trace it carries, \
                 which the verdict above contradicts: {manifest}"
            )
        });
    assert!(
        covered > 0,
        "the range must be a real gating-clock instant from this run's own trace, not a \
         default: {manifest}"
    );
    assert!(
        manifest["anchor"]["resimmable_reason"]
            .as_str()
            .unwrap_or_default()
            .contains(&format!("{covered} ns")),
        "the operator-facing sentence names the same instant the field does: {manifest}"
    );
    // ---------------------------------------------------------------- leg 3
    // RESIM, through the verb the capture's own hint tells the operator to run.
    let mut resim = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "play"])
            .arg(&capture)
            .args(["--resim", "all"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .stdout(Stdio::from(
                std::fs::File::create(root.join("resim.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(
                std::fs::File::create(root.join("resim.stderr")).unwrap(),
            ))
            .spawn()
            .expect("spawn cerulion bag play --resim"),
    );
    let resim_status = wait_bounded(&mut resim.0, RESIM_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion bag play --resim` never returned"));
    let resim_err = read_file(&root.join("resim.stderr"));
    assert_eq!(
        resim_status.code(),
        Some(0),
        "THE CLOSED LOOP: a capture this recorder stamped `resimmable: true` must be a bag \
         `bag play --resim` accepts. Exit 2 naming a missing \
         `__cerulion/trace_manifest_rank<N>.json` is the original defect itself — the verdict \
         and the gate disagreeing about the same bag. Exit 2 naming a frame that \
         'matches NO recorded STEP_BOUNDARY target' is the tail race — the capture's \
         frames outran its trace and the covered range was not honoured — \
         {resim_status:?}\nstderr:\n{resim_err}"
    );
    // …and it must have re-executed something. Exit 0 on a resim that loaded
    // the bag and ran nothing would satisfy the assertion above while proving
    // nothing, and the summary is where the engine states what it did.
    let executed = resim_err
        .lines()
        .find_map(|l| {
            let l = l.trim();
            let rest = l.strip_prefix("re-executed ")?;
            rest.split_once(" step(s)")?.0.parse::<u64>().ok()
        })
        .unwrap_or_else(|| panic!("the resim summary must report its step count:\n{resim_err}"));
    assert!(
        executed > 0,
        "the resim must actually re-execute the capture's suffix, got {executed} step(s):\n\
         {resim_err}"
    );
    // When the replay reports a covered range, it is the SAME instant
    // the capture declared — the closed loop's own half of the range, which no
    // single-crate arm can observe (the recorder's measurement and the
    // replayer's use of it live in two crates that meet in no unit test).
    //
    // CONDITIONAL, and deliberately so. The neutral summary prints the line only
    // when frames actually fell outside the range, and whether THIS run's frame
    // window outran its trace window is a race — a capture whose two windows
    // closed together is a perfectly good capture. Demanding the line
    // unconditionally would make the arm flaky in the one direction that proves
    // nothing; demanding no line at all would let a replay report somebody
    // else's number. So the assertion is AGREEMENT: if a range is reported, it
    // is this capture's. The deterministic both-ways coverage is in
    // `replay_engine_test`'s crafted-bag arms and `resim_cmd`'s renderer oracle.
    const COVERED_LINE: &str = "resim covered this recording through";
    if resim_err.contains(COVERED_LINE) {
        assert!(
            resim_err.contains(&format!("{COVERED_LINE} {covered} ns")),
            "the resim reported a covered range that is not the one the capture declared \
             ({covered} ns):\n{resim_err}"
        );
    }

    // …and the attachment the gate above refuses a bag without. Asserted LAST,
    // deliberately: it is a REFINEMENT of the exit-0 claim, not a substitute for
    // it. Read earlier it would preempt the headline — reverting the manifest write
    // would fail on an attachment count rather than on `bag play --resim`
    // REFUSING the bag, which is the failure this arm exists to reproduce
    // (MEASURED: it fails here first, and the reorder was made because of it).
    assert!(
        reader
            .attachment("__cerulion/trace_manifest_rank0.json")
            .expect("read the rank-0 trace manifest")
            .is_some(),
        "the resim accepted this bag, so a missing rank-0 trace manifest would mean the gate \
         stopped reading one — `load_trace_manifests` refuses a bag with zero manifests"
    );
}
