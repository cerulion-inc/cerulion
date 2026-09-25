// SPDX-License-Identifier: AGPL-3.0-only
//! **THE PLAIN-RUN ACCEPTANCE.** The project's rule is that
//! every Flashback capture must be re-executable. This file is that claim on the
//! shape a robot actually runs: a plain `cerulion graph run`, no flags.
//!
//! # Why this file, when `flashback_resim_e2e_test` exists
//!
//! That file's arm (`a_capture_taken_off_a_real_record_run_is_a_bag_bag_play_resim_accepts`)
//! is the SAME closed loop driven by `graph run --record --single-process`. It is
//! the loop's proof on the shape somebody deliberately set up to record. Before
//! the always-on rings, that was the ONLY shape that could pass it: trace-ring tags were
//! stamped under `if record.is_some()`, so a plain run minted none, the always-on
//! window recorder was handed no `--ring`, and every capture of the DEFAULT run
//! shape reported `ResimGap::NoTrace`. The gate is `!no_rings` instead;
//! this file is the acceptance test for that flip, and it is deliberately its own
//! binary rather than an arm in the record file — the harnesses share no
//! workspace, no graph and no guard (that file's `BagdGuard` keys on the
//! `--record` bagd's own `--out recordings/fbresim_` cmdline, which a plain run
//! never produces).
//!
//! # The arms, and what each one alone cannot see
//!
//! 1. **THE LOOP** (`a_capture_taken_off_a_plain_run_is_a_bag_bag_play_resim_accepts`) —
//!    a plain 1-group multi-process run, a real `cerulion flashback`, and
//!    `cerulion bag play --resim all` on what it wrote. Exit 0 is the headline;
//!    the recorded frames are anchored to a HAND ORACLE first, so the exit code
//!    is a claim about real data rather than about an empty bag.
//! 2. **`--no-rings`** (`no_rings_takes_no_capture_and_its_attach_bag_is_refused_by_resim`) —
//!    by design, with no rings the window recorder is not spawned at
//!    all, so there is no capture to be un-resimmable. What an operator gets
//!    instead is asserted on BOTH surfaces: `cerulion flashback` reports that
//!    nothing answered, and a `bag record --run` attach to that same run carries
//!    the run's own DECLINED statement and is refused by `bag play --resim`.
//! 3. **THE ATTACH** (`a_bag_record_run_attach_to_a_plain_run_carries_its_trace`) —
//!    the OTHER consumer of the same rings. A capture proves the standing
//!    recorder can read them; this proves a recorder that arrives later can too,
//!    which is the half `run.json`'s ring declaration exists for.
//! 4. **NO-INERT-SHIPPING** (`a_flashback_switched_off_run_still_stamps_rings_for_a_later_attach`) —
//!    rings are NOT gated on the Flashback kill switch, or `bag record
//!    --run` would lose the trace on every `CERULION_FLASHBACK=off` run.
//! 5. **THE CO-TENANT** and the D4 run-half (`a_capture_holding_a_co_tenants_topic_is_still_a_bag_resim_accepts`,
//!    `a_run_writes_its_window_recorder_decision_into_run_json`); see their docs.
//! 6. **THE ONE-RANK FREE-RUN LOOP** (`a_free_run_one_rank_capture_resims_and_verifies_byte_exact_and_catches_a_changed_constant`):
//!    the same run under `CERULION_EXECUTION_MODE=free_run` (the free-run
//!    default is not on `main` yet, so the opt-in is set explicitly), a capture taken
//!    MID-RUN, `bag play --resim all --verify` exit 0 twice with one report, and the
//!    perturbed ticker caught at exit 1 naming its topic. The mutant is the base
//!    commit: before the admission the same arm exits 2 by name at the resim.
//!
//! # What each arm alone catches
//!
//! Each fails EXACTLY the arm that is named for it, with the others green — which
//! is the evidence the arms cover different seams rather than several spellings
//! of one. (Arm 5, the co-tenant acceptance, was added later and has its
//! own verification recorded separately below.)
//!
//! - `&& !flashback_switched_off()` added to the ring-stamping predicate
//!   (`graph_cmd.rs`'s `if !no_rings`) fails EXACTLY arm 4, at the run's own
//!   `DECLINED scheduler-trace rings at launch` — the kill switch reaching a
//!   plane it has no business reaching. Arms 1-3 run with the plane ON, so that
//!   addition is inert for them and they stay green.
//! - The supervisor handing its window recorder NO rings
//!   (`ring_names: &flashback_ring_names[..0]` — the slice rather than `&[]`,
//!   because `unused_variables = deny` refuses the naive form) fails EXACTLY
//!   arm 1, and it fails it with exactly this report:
//!   `"resimmable": false`, `"trace_rings_configured": 0`, and the
//!   `ResimGap::NoTrace` sentence. Arms 2-4 never look at a capture.
//! - Deleting the declined-plane refusal (`if state_rings.declines_state_plane()` →
//!   `if false`, so the attach SWEEPS the run's state rings) fails EXACTLY
//!   arm 3, on the attachment the sweep produces: `a DECLINED attach sweeps no
//!   state rings, so it writes no `__cerulion/state_coverage.json``. Arm 3 sees
//!   this only because the attach does not race the worker's first
//!   step — see the note in its body.
//!
//! # The declined-plane refusal, and where each half of it is pinned
//!
//! A `bag record --run` attach must REFUSE state-ring
//! discovery when a standing recorder already holds the run's state rings —
//! those are `OverrunPolicy::Backpressure`, where every consumer stores its
//! cursor into ONE shared header slot, so two of them lap each other. Under
//! the default configuration every plain run has a standing window recorder, so arm 3
//! drives exactly that shape.
//!
//! It IS implemented, and the two halves are pinned apart because
//! they fail apart. The RUN's half — that a real `graph run` writes its
//! window-recorder decision into `run.json` under `state_ring_consumer` — is
//! arm 5 below, which needs a real binary and is why it lives here. The
//! ATTACH's half — that the reader refuses on `standing`, proceeds on `none`,
//! and proceeds-with-a-warning on an absent key — is pinned over hand-written
//! manifests in `cerulion_cli_engine/tests/bag_record_run_attach_test.rs`,
//! where every state is reachable without spawning a graph to produce it.
//!
//! Arm 3 drives the `standing` row of that table over a REAL run: it asserts
//! the bag's own `state_rings` statement as well as the trace, which is the one
//! thing a hand-written manifest cannot check — that a real plain run and a real
//! attach reach the decline at all. (See the note
//! in its body about what is timing-shaped.) The refusal is scoped to
//! the STATE plane, which is the point of the note below — the two ring kinds
//! have different consumer rules, and the bag still carries its TRACE.
//!
//! Note the contrast, because the two rings differ and the difference is the
//! whole reason arm 3 works at all: a TRACE ring is `FailLoud`, where each
//! consumer holds a LOCAL cursor and publishes nothing, so one producer and N
//! independent readers is sound by design, which is exactly what
//! arm 3 exercises, a standing window recorder and a mid-run attach reading the
//! same rings.
//!
//! # The hand oracle, and why it is what it is
//!
//! The graph is the repo's standard record fixture pair — `ticker`
//! (`test_node_macro_period_cdylib`, `period_ms = 50`) feeding `relay`
//! (`test_node_macro_data_trigger_cdylib`), both publishing `geometry_msgs/Vector3`.
//! Read off those two fixtures' SOURCE, every frame either node publishes is a
//! 32-byte wire header carrying `Vector3::SCHEMA_HASH` followed by 24 bytes of
//! zero: the ticker writes `cmd.x = 0.0` and touches nothing else, and the relay
//! forwards that same `x`. So the oracle is written from the fixtures, not read
//! off the run, and it is checked BEFORE the resim — a bag whose frames are not
//! what those nodes produce makes an exit-0 resim prove nothing.
//!
//! The rest of the wire HEADER carries the other half — `total_size`, an empty
//! offset table, a strictly ascending `sequence` and a strictly advancing
//! gating-clock `timestamp_ns`. That is the part a constant payload cannot
//! state, and it is what a bag carrying a frame twice, or one whose headers were
//! re-stamped, would break.
//!
//! # `--resim all`, and why there is no `--verify` leg
//!
//! Exit 0 from a bare `--resim all` means "the re-execution could be performed" —
//! replay-grade bag, cdylibs loaded, no candidate panic — which is the gate
//! the always-on rings moved.
//!
//! `--verify` is deliberately NOT driven, and that was MEASURED rather than
//! inherited from the `--record` sibling's convention: a rolling window is lossy
//! by construction, so the re-execution produces frames the window dropped and
//! every frame after the first hole carries a shifted `sequence`. Driven for
//! real: `recorded 817 frame(s), replay produced 837`, first difference at byte
//! 20 — the sequence field — 0 of 2 topics matched, on a capture the same
//! command had just accepted. The full reasoning sits at the end of arm 1.
//!
//! Hermetic on every axis: `CERULION_NETWORK=off` (no gateway, no scouting),
//! `CERULION_HOME` and `CERULION_FLASHBACK_DIR` under the test's own tempdir.
//! `#![cfg(unix)]` (signals, `pgrep`, the recorder and the trigger channel all
//! are) and `#[serial]`: the run's data plane AND the `/__cerulion/flashback`
//! trigger channel are on the DEFAULT iceoryx2 namespace, so two of these at once
//! would answer each other's requests. Per-arm unique graph prefixes keep topic
//! names apart from the sibling suites and from a SIGKILLed earlier run of this
//! one.
//!
//! Prerequisites:
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib \
//!  -p test_node_macro_period_perturbed_cdylib`

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// The capture's coverage manifest is read back through the SAME type
// a `--record` bag's is — that the two artifacts carry ONE document is the whole
// point, so parsing it some other way would not check the claim.
use cerulion_bagd::{
    RecordCoverage, TapSource, CAPTURE_RECORDER_HEALTH_ATTACHMENT, RECORD_COVERAGE_ATTACHMENT,
};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_STEP_BOUNDARY};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportManager;
use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";
const RELAY_FIXTURE: &str = "test_node_macro_data_trigger_cdylib";

/// Generous liveness ceilings. Load can delay every one of these; none is a wall
/// stated in units of the thing under test (the load-robustness rule).
const READY_DEADLINE: Duration = Duration::from_secs(90);
/// The capture verb waits out the default post window (15 s) and then the bag
/// write, so its ceiling is generous by a wide margin.
const CAPTURE_COMPLETES: Duration = Duration::from_secs(240);
const EXIT_DEADLINE: Duration = Duration::from_secs(60);
const RESIM_COMPLETES: Duration = Duration::from_secs(300);
/// `bag record --run --duration N` stops itself; this bounds the wait for it.
const ATTACH_COMPLETES: Duration = Duration::from_secs(180);
/// Arm 5 (the co-tenant): a LIVENESS ceiling for the wait until the recorder's
/// discovery rescan has attached a tap to the co-tenant's topic.
///
/// A ceiling on a CONDITION, never a sleep — the arm waits for the tap to exist
/// and then publishes into it. A fixed sleep here is the load-fragile shape, and it
/// loses in the direction that destroys data: a data-only tap requests no
/// late-joiner history, so a frame committed before the tap attaches lands in no
/// queue at all and the arm's verdict inverts on a starved runner. Stated in
/// seconds rather than in multiples of the 250 ms rescan cadence, for the same
/// reason.
const CO_TENANT_TAP_DEADLINE: Duration = Duration::from_secs(30);
/// How often that wait asks.
const CO_TENANT_TAP_POLL: Duration = Duration::from_millis(20);
/// How long the co-tenant then publishes INTO its attached tap, so the rolling
/// window has frames of it to hold. Pacing, not a rendezvous: the tap provably
/// exists by then, so load can only make this span WIDER.
const CO_TENANT_FILL: Duration = Duration::from_secs(2);

/// A LIVENESS ceiling for the wait until the run's worker has provably executed
/// steps (arm 3's mid-run precondition).
///
/// A ceiling on a CONDITION, never a sleep, and stated in seconds rather than in
/// multiples of the fixture's 50 ms period — the same rule as
/// [`CO_TENANT_TAP_DEADLINE`]. Load can only make the wait LONGER, which is the
/// direction that keeps the property true.
const WORKER_STEPPED_DEADLINE: Duration = Duration::from_secs(90);
/// How often that wait asks.
const WORKER_STEPPED_POLL: Duration = Duration::from_millis(10);
/// How many ticker frames prove the worker is PAST its first step.
///
/// ONE would not be enough, and the reason is the push order: a step's boundary
/// record and the fires inside it are two separate pushes, so a single observed
/// frame leaves it open whether step 0's boundary has been pushed yet. Frames
/// from several distinct steps close that whichever order the two take — the
/// ticker fires at most once per step, so N frames means N steps executed.
///
/// 5 rather than 2 is margin, and it is nearly free: the fixture's period is
/// 50 ms, so this is ~250 ms of a run whose recorder then takes a whole process
/// spawn to arrive.
const WORKER_STEPPED_FRAMES: usize = 5;

/// The run has FINISHED deciding whether to hold a rolling window.
///
/// Emitted unconditionally after the spawn decision, which is what makes it
/// usable as a landmark: the GO breadcrumb is emitted BEFORE that decision, so a
/// snapshot taken there is a snapshot of the moment before the code under test
/// has run.
const FLASHBACK_DECISION: &str = "the window-recorder decision for this run is taken";

/// The window recorder is up and holding.
const WINDOW_HELD: &str = "flashback: holding a rolling window";

/// The ATTACHING recorders in this file record the run's DECLARED topics and
/// nothing else, so their assertions are about the run under test rather than
/// about whatever else the desk happens to be publishing.
///
/// It is set only where it is LEGAL. The always-on WINDOW recorder cannot take
/// it: `write_flashback_topics` hands that child an empty `--topics-json` ON
/// PURPOSE (the live-discovery argument applied to a black box: a static
/// declaration can name four topics while ~71 dynamically-registered bridge
/// routes stream unrecorded, as measured on a Unitree Go2), so with discovery off it refuses
/// to arm at all — in its own words: `the topic list is empty and live discovery
/// is OFF, so this recorder would tap NOTHING for its whole life`. See
/// [`assert_capture_holds_only_this_runs_topics`] for what that costs the
/// capture arm.
const DISCOVERY_OFF: (&str, &str) = ("CERULION_RECORD_DISCOVERY", "off");

/// A prefix no other run on this machine can be using.
///
/// These arms publish on the DEFAULT iceoryx2 namespace and graph topics are
/// SINGLE-WRITER, so a fixed prefix collides with a publisher port left behind by
/// an earlier SIGKILLed run of the same test — which is exactly what a failing
/// arm's own `ChildGuard` teardown produces, so the next run fails for a reason
/// that has nothing to do with what it asserts.
fn unique_prefix(stem: &str) -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    format!("{stem}{}{nanos}", std::process::id())
}

/// Strip CSI escape sequences.
///
/// `tracing`'s fmt layer wraps a field's NAME and its `=` in escapes (its `ansi`
/// default is a compile-time feature, not a tty probe), so `window="…"` is NOT a
/// substring of the raw capture and a `key=value` assertion against it is
/// silently unsatisfiable. Same helper, same reason, as `flashback_argv_e2e_test`.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b {
            i += 1;
            if i < bytes.len() && bytes[i] == b'[' {
                i += 1;
                while i < bytes.len() && !bytes[i].is_ascii_alphabetic() {
                    i += 1;
                }
                i += 1;
            }
            continue;
        }
        out.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// The 1-GROUP multi-process workspace: `ticker` (Period) -> `relay`
/// (DataTrigger), both in ONE process group.
///
/// The group is DECLARED rather than derived, and that is the arm's shape rather
/// than a convenience. An unpartitioned two-node graph derives PROCESS-PER-NODE
/// under the default, i.e. two workers, two state rings, and the capture
/// judge's `MultiRing` refusal — a real limitation, and not the one
/// this file is about. One group gives a genuine multi-process run (the
/// supervisor never collapses to a monolith, even at one group) with the single
/// worker ring the judge can reason about.
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
    for (node_type, fixture) in [("ticker", PERIOD_FIXTURE), ("relay", RELAY_FIXTURE)] {
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
    std::fs::write(
        root.join("graphs/plainrun.yaml"),
        format!(
            "name: plainrun\n\
             prefix: {prefix}\n\
             process_groups:\n\
             \x20 p0:\n\
             \x20 - ticker\n\
             \x20 - relay\n\
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
             \x20\x20\x20 schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
}

/// A running `graph run` that reaps its WORKERS on the way out, panic or not.
///
/// `RunGuard` exists because a SIGKILLed supervisor cannot run its own teardown
/// — its `graph run-worker` children would be re-parented to init and keep
/// publishing on the DEFAULT iceoryx2 namespace, where the NEXT arm's window
/// recorder taps them (see [`assert_capture_holds_only_this_runs_topics`]). The
/// inner `ChildGuard` tears down the whole process GROUP and reports any
/// survivor, so this type's job is narrow: send the SIGINT the
/// supervisor HANDLES, so the graceful path runs before the group teardown.
///
/// MEASURED, and it is why this type exists rather than a bare `ChildGuard`: an
/// early version of arm 2 panicked on an assertion, leaked its worker, and the
/// next full-file run failed arm 1 with `/plainnr…/ticker/cmd` in a capture that
/// had nothing to do with it. A leaked worker outlives the whole test binary, so
/// the damage is not confined to the run that leaked it.
///
/// SIGINT first, because that is the signal the supervisor HANDLES: it drives
/// the graceful shutdown that reaps the workers. The inner `ChildGuard`'s
/// SIGKILL still runs afterwards and is a no-op on an already-exited child.
struct RunGuard(ChildGuard);

impl RunGuard {
    fn pid(&self) -> u32 {
        self.0.id()
    }
}

impl Drop for RunGuard {
    fn drop(&mut self) {
        // A REAPED child's pid is free for the kernel to reuse, and a raw
        // kill(2) — unlike `Child::kill`, which refuses after a wait — would
        // deliver SIGINT to whatever unrelated process now holds it. Every
        // happy-path arm reaches this Drop AFTER `stop_run` has already
        // reaped, so consult the cached status first and signal only a child
        // this guard still owns; an unobservable child (`Err`) is treated as
        // not-ours, because an unjustified signal is worse than a leak
        // the inner `ChildGuard`'s (reap-safe) SIGKILL still bounds.
        if !matches!(self.0.try_wait_noting(), Ok(None)) {
            return;
        }
        let pid = self.pid();
        // SAFETY: kill(2) with a pid this process still owns (un-reaped, per
        // the try_wait gate above) and a valid signal; no memory is touched.
        // ESRCH on a child that exits between the gate and this call is the
        // expected no-op — un-reaped, its pid cannot be reused.
        unsafe {
            libc::kill(pid as libc::pid_t, libc::SIGINT);
        }
        let _ = self.0.wait_bounded(EXIT_DEADLINE);
    }
}

/// Spawn `cerulion graph run plainrun` with the given extra flags and env,
/// hermetic on all three axes.
fn spawn_run(
    root: &Path,
    home: &Path,
    flashbacks: &Path,
    extra: &[&str],
    env: &[(&str, &str)],
) -> (RunGuard, PathBuf) {
    let stderr_path = root.join("run.stderr");
    let mut args = vec!["graph", "run", "plainrun", "--no-validate"];
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
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    // A PLAIN `graph run`: no `--single-process`, so the auto-partitioner derives a partition
    // and this supervisor has workers to tear down.
    let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
    (RunGuard(guard), stderr_path)
}

/// Bounded poll for `needle` in the growing log; panics at the deadline (the
/// caller's `ChildGuard` reaps on the unwind).
fn wait_for_log_line(child: &mut ChildGuard, path: &Path, needle: &str) {
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
            start.elapsed() < READY_DEADLINE,
            "log line {needle:?} not seen within {READY_DEADLINE:?}; log so far:\n{}",
            read_file(path)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
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

/// SIGINT the run and require a clean exit.
fn stop_run(guard: &mut RunGuard, stderr_path: &Path) {
    send_signal(guard.pid(), libc::SIGINT);
    let status = guard.0.wait_bounded(EXIT_DEADLINE).unwrap_or_else(|| {
        panic!(
            "the run did not exit on SIGINT — stderr:\n{}",
            read_file(stderr_path)
        )
    });
    assert_eq!(
        status.code(),
        Some(0),
        "a graceful SIGINT exits cleanly: {status:?} — stderr:\n{}",
        read_file(stderr_path)
    );
}

/// One millisecond, in the nanoseconds the wire header stamps.
const NS_PER_MS: u64 = 1_000_000;

/// The fixture ticker's `period_ms = 50`, in nanoseconds — the unit the
/// free-run burst bound below is DERIVED in rather than tuned against.
const FIXTURE_PERIOD_NS: u64 = 50 * NS_PER_MS;

/// The most frames one step may stamp with the SAME gating-clock target,
/// whatever its own advance says it owes.
///
/// A run this long means a step that charged the clock at least
/// `8 * 50 ms = 400 ms` on a two-node fixture whose whole job is to publish 24
/// zero bytes — which is the "the clock stopped advancing per step" shape, not a
/// catch-up. The deepest burst ever measured is TWO frames (see
/// [`gating_stamp_violation`]), so desk load has 4x of room before it reaches
/// this.
const MAX_CATCH_UP_BURST_FRAMES: usize = 8;

/// Which coordination contract the run under test executed under.
///
/// A PARAMETER rather than one rule for every caller because the gating clock
/// stamps frames differently in the two, and the looser of the two rules is
/// blind to a fault the stricter one catches. See [`gating_stamp_violation`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gating {
    /// A plain `graph run`: the quantum is the TIGHTEST period in the graph, so
    /// a step never owes a `period_ms` node a second fire and every frame
    /// carries its own boundary target.
    Lockstep,
    /// `CERULION_EXECUTION_MODE=free_run`: the rank's controlled clock advances
    /// ONCE per step by the measured wall elapsed and is CONSTANT within the
    /// step.
    FreeRun,
}

/// **THE GATING-CLOCK STAMP RULE**, as a pure function over one topic's stamps
/// so that BOTH of its sides can be pinned by hand oracles rather than by a run
/// that happens to produce the shape.
///
/// Returns the reason the series breaks the rule, or `None`.
///
/// # Lockstep: STRICTLY increasing, no exception
///
/// The quantum is the tightest period, so a step cannot owe a `period_ms` node
/// two fires and two frames of one topic cannot share a boundary target.
///
/// # Free run: strictly increasing EXCEPT a catch-up burst, which must LOOK like one
///
/// A free-run rank's clock advances once per step by the measured wall elapsed
/// and is constant within the step, so a wall-delayed step fires the node for
/// every period it owes and stamps each of those frames with the same target —
/// which is exactly what a resim re-advances to. Measured on one Linux capture:
/// 345 ticker frames, 338 deltas at 50 ms, and exactly TWO 0 ns deltas, each
/// right after a 101 ms / 110 ms step and followed by a 49 ms / 40 ms one.
///
/// # Why a BOUND, and why these
///
/// "Never decreasing, and the last exceeds the first" is not a rule: over N
/// frames it admits N-2 consecutive EQUAL stamps. A gating clock that stopped
/// advancing per step — one quantised to a coarse tick, or advanced on the
/// anchor cadence instead of per step — passes it unseen, and the STRICT
/// sequence check cannot see it either, because sequences are the producer's
/// commit counter and keep counting through a frozen clock. The measured
/// justification for admitting anything at all was 2 zero deltas in 345 frames;
/// that rule admits 343.
///
/// So a zero delta is admitted only where it is SHAPED like the burst:
///
/// 1. **The advance that produced the repeated target exceeded one period.** A
///    step the clock charged 50 ms or less owes exactly ONE fire, so a second
///    frame at that target is not a catch-up.
/// 2. **The run is no longer than that advance OWES.** An advance of `d` crosses
///    `d / 50 ms` period boundaries and so owes that many fires: the measured
///    101 ms step owes 2, which is exactly the 2 frames that capture carried.
///    [`MAX_CATCH_UP_BURST_FRAMES`] caps it on top of that, which is what makes
///    a coarse-tick or anchor-cadence clock FAIL — those repeat a target far
///    longer than any advance a healthy step takes.
///
/// The one place rule 1 cannot apply is the series' FIRST stamp run: a rolling
/// window trims its head at an arbitrary frame, so a capture may legitimately
/// begin part-way through a burst with the advance that produced it cut away.
/// There the absolute ceiling stands alone — a long stall at the head still
/// fails, a trimmed burst does not.
fn gating_stamp_violation(stamps: &[u64], gating: Gating) -> Option<String> {
    // The advance the most recent step charged the clock (`None` until the
    // series shows one), and how many frames already carry the target it
    // produced.
    let mut last_advance: Option<u64> = None;
    let mut run_len: usize = 1;
    for (i, pair) in stamps.windows(2).enumerate() {
        let (prev, cur) = (pair[0], pair[1]);
        let frame = i + 1;
        if cur < prev {
            return Some(format!(
                "runs the gating clock BACKWARDS at frame {frame} ({prev} -> {cur})"
            ));
        }
        if cur > prev {
            last_advance = Some(cur - prev);
            run_len = 1;
            continue;
        }
        if gating == Gating::Lockstep {
            return Some(format!(
                "repeats the stamp {cur} at frame {frame}, and under lockstep the quantum is \
                 the tightest period — a step never owes a `period_ms` node a second fire, so \
                 every frame carries its own boundary target"
            ));
        }
        run_len += 1;
        let ceiling = match last_advance {
            // Head of the series: the window trimmed whatever advance produced
            // this target, so only the absolute ceiling can speak.
            None => MAX_CATCH_UP_BURST_FRAMES,
            Some(d) if d <= FIXTURE_PERIOD_NS => {
                return Some(format!(
                    "repeats the stamp {cur} at frame {frame} after a step advance of {d} ns, \
                     which is not more than the fixture's {FIXTURE_PERIOD_NS} ns period — a step \
                     the clock charged one period owes ONE fire, so this is a clock that stopped \
                     advancing per step and not a CATCH-UP BURST"
                ));
            }
            Some(d) => ((d / FIXTURE_PERIOD_NS) as usize).min(MAX_CATCH_UP_BURST_FRAMES),
        };
        if run_len > ceiling {
            let advance = last_advance.map_or_else(
                || "an advance the window trimmed away".to_string(),
                |d| format!("a {d} ns step advance"),
            );
            return Some(format!(
                "carries the stamp {cur} on {run_len} frames (through frame {frame}), past the \
                 {ceiling} that {advance} owes at a {FIXTURE_PERIOD_NS} ns period (absolute \
                 ceiling {MAX_CATCH_UP_BURST_FRAMES}) — a CATCH-UP BURST cannot be that long, \
                 and a clock quantised to a coarse tick or advanced on the anchor cadence \
                 instead of per step is exactly this shape"
            ));
        }
    }
    None
}

/// **THE HAND ORACLE.** Every frame these two fixtures publish, written from
/// their SOURCE rather than read off the run.
///
/// `ticker` (`period_ms = 50`) writes `cmd.x = 0.0` and touches no other field;
/// `relay` forwards `cmd.x = trigger_in.x`. `geometry_msgs/Vector3` is three
/// `f64`s and carries no variable field, so a frame on either topic is exactly a
/// 32-byte wire header plus 24 zero bytes.
///
/// Also asserts the per-topic wire SEQUENCE is STRICTLY ASCENDING, which is the
/// half a constant payload cannot state: a bag carrying a frame twice, or one
/// whose sequences were re-stamped, breaks it while every payload still matches.
///
/// STRICTLY ASCENDING and not GAP-FREE, and that distinction is MEASURED rather
/// than assumed. An oracle that demands gap-free fails
/// against a real capture on `…/ticker/cmd` at `[…, 58, 59, 65, 66, …]` — a
/// five-frame hole. A rolling window is fed by ordinary data-only taps, so a
/// burst past a tap's provisioned queue depth is the documented `frames_lost`
/// class and the capture's own `record_health.json` accounts for it;
/// gap-free is a property of the PRODUCER's commit counter, never a
/// promise about what a recorder caught. Demanding it here would make this arm
/// fail on a healthy robot for a reason that has nothing to do with the property under test.
///
/// `gating` is the coordination contract the run under test executed under, and
/// it decides the STAMP rule: strictly increasing under lockstep, strictly
/// increasing except a bounded catch-up burst under free run. The caller states
/// it rather than the bag, so an arm that drives a lockstep run keeps the strict
/// rule even though this helper is shared with the free-run arm — see
/// [`gating_stamp_violation`] for what the loose rule cannot catch.
fn assert_frames_match_the_fixture_oracle(
    bag: &Path,
    prefix: &str,
    context: &str,
    gating: Gating,
) -> usize {
    let reader = cerulion_bag::BagReader::open(bag)
        .unwrap_or_else(|e| panic!("{context}: open {}: {e}", bag.display()));
    let (msgs, completeness) = reader
        .recover_messages()
        .unwrap_or_else(|e| panic!("{context}: recover: {e}"));
    assert!(
        completeness.is_finalized(),
        "{context}: the bag must be finalized, got {completeness:?}"
    );

    const VECTOR3_WIRE_BYTES: usize = 24;
    const HEADER_BYTES: usize = 32;
    let hash =
        <native_ros2_messages::geometry_msgs::Vector3 as cerulion_core::ShmMessage>::SCHEMA_HASH;

    let mut graph_frames = 0usize;
    for topic_suffix in ["ticker/cmd", "relay/cmd"] {
        let topic = format!("/{prefix}/{topic_suffix}");
        let mut seqs: Vec<u32> = Vec::new();
        let mut stamps: Vec<u64> = Vec::new();
        for m in msgs.iter().filter(|m| m.topic == topic) {
            assert_eq!(
                m.data.len(),
                HEADER_BYTES + VECTOR3_WIRE_BYTES,
                "{context}: a {topic} frame is a wire header plus a fixed Vector3 — these \
                 fixtures publish nothing else"
            );
            let header = cerulion_core::wire::WireHeader::read_from_buf(&m.data[..HEADER_BYTES])
                .unwrap_or_else(|| panic!("{context}: {topic} frame carries no readable header"));
            assert_eq!(
                header.schema_hash, hash,
                "{context}: {topic} carries geometry_msgs/Vector3"
            );
            assert_eq!(
                &m.data[HEADER_BYTES..],
                &[0u8; VECTOR3_WIRE_BYTES][..],
                "{context}: `ticker` writes `cmd.x = 0.0` and touches nothing else, and `relay` \
                 forwards that x — so every {topic} payload is 24 zero bytes. A frame that is \
                 not makes the resim below a claim about data these nodes did not produce."
            );
            assert_eq!(
                header.total_size as usize,
                HEADER_BYTES + VECTOR3_WIRE_BYTES,
                "{context}: {topic} the header's own `total_size` agrees with the frame it \
                 heads — a header that describes a different frame is the one shape a payload \
                 comparison cannot see"
            );
            assert_eq!(
                header.offset_table_count, 0,
                "{context}: {topic} `geometry_msgs/Vector3` carries no variable field, so its \
                 offset table is empty"
            );
            seqs.push(header.sequence);
            stamps.push(header.timestamp_ns);
            graph_frames += 1;
        }
        assert!(
            !seqs.is_empty(),
            "{context}: {topic} must carry frames, or the resim's exit code says nothing"
        );
        for pair in seqs.windows(2) {
            assert!(
                pair[1] > pair[0],
                "{context}: {topic} sequences are STRICTLY ascending — a bag carrying a frame \
                 twice, or one whose sequences were re-stamped, breaks this while every \
                 constant payload above still matches. Got {seqs:?}"
            );
        }
        // The gating-clock stamp: nonzero, then the mode's own rule. The STRICT
        // sequence check above is what catches a frame carried twice or
        // re-stamped, in either mode; the rule below is what catches a clock
        // that stopped advancing per step, which the sequences cannot see.
        assert!(
            stamps.first().is_some_and(|s| *s > 0),
            "{context}: {topic} frames carry a real loan-time stamp, not a default"
        );
        if let Some(reason) = gating_stamp_violation(&stamps, gating) {
            panic!("{context}: {topic} under {gating:?} {reason}. Got {stamps:?}");
        }
        assert!(
            stamps.len() < 2 || stamps[stamps.len() - 1] > stamps[0],
            "{context}: {topic} wire timestamps advance with the gating clock over the \
             capture. Got {stamps:?}"
        );
    }
    graph_frames
}

/// A stamp series built from its DELTAS, so an oracle below states the shape it
/// means (`50 ms, 101 ms, 0, 49 ms`) rather than a column of absolute numbers a
/// reader has to subtract.
fn stamps_from_deltas(first: u64, deltas: &[u64]) -> Vec<u64> {
    let mut stamps = vec![first];
    let mut at = first;
    for d in deltas {
        at += d;
        stamps.push(at);
    }
    stamps
}

/// **THE BURST, ADMITTED.** The measured free-run capture's own shape, the one
/// the rule exists to let through: two catch-up bursts, each a 0 ns delta right
/// after a step the clock charged 101 ms / 110 ms and followed by a short one.
#[test]
fn the_stamp_rule_admits_the_measured_free_run_catch_up_burst() {
    let stamps = stamps_from_deltas(
        1_000 * NS_PER_MS,
        &[
            50 * NS_PER_MS,
            50 * NS_PER_MS,
            101 * NS_PER_MS,
            0,
            49 * NS_PER_MS,
            50 * NS_PER_MS,
            110 * NS_PER_MS,
            0,
            40 * NS_PER_MS,
            50 * NS_PER_MS,
        ],
    );
    assert_eq!(
        gating_stamp_violation(&stamps, Gating::FreeRun),
        None,
        "the shape MEASURED on a real free-run capture must pass: {stamps:?}"
    );
}

/// **THE STALL, REFUSED** — and the arm that states what the relaxed rule cost.
///
/// A gating clock quantised to a coarse 1 s tick with the fixture's 50 ms
/// period: it never decreases and its last stamp exceeds its first, so the
/// "never decreasing plus last > first" rule this replaced accepted it in full.
/// That is asserted here rather than described, so the oracle is a comparison of
/// the two rules and not a claim about one.
#[test]
fn the_stamp_rule_refuses_a_stalled_stretch_the_relaxed_rule_admitted() {
    let mut deltas: Vec<u64> = Vec::new();
    for _ in 0..3 {
        deltas.push(1_000 * NS_PER_MS);
        deltas.extend_from_slice(&[0; 19]);
    }
    let stamps = stamps_from_deltas(1_000 * NS_PER_MS, &deltas);

    // The rule this replaced, spelled out: it passes.
    assert!(
        stamps.windows(2).all(|p| p[1] >= p[0])
            && stamps[stamps.len() - 1] > stamps[0]
            && stamps[0] > 0,
        "the relaxed rule must ACCEPT this series, or this arm proves nothing: {stamps:?}"
    );

    let reason = gating_stamp_violation(&stamps, Gating::FreeRun)
        .unwrap_or_else(|| panic!("a clock that stamps 20 frames per tick must be REFUSED"));
    assert!(
        reason.contains("CATCH-UP BURST"),
        "…and refused as the burst it is not: {reason}"
    );
}

/// A healthy series — one stamp per step, every step — passes under BOTH modes.
/// The free-run relaxation is an EXCEPTION, not a different rule.
#[test]
fn the_stamp_rule_admits_a_strictly_increasing_series_under_both_modes() {
    let stamps = stamps_from_deltas(7 * NS_PER_MS, &[50 * NS_PER_MS; 8]);
    for gating in [Gating::Lockstep, Gating::FreeRun] {
        assert_eq!(
            gating_stamp_violation(&stamps, gating),
            None,
            "{gating:?}: {stamps:?}"
        );
    }
}

/// A clock that runs BACKWARDS is refused under both modes — the half neither
/// relaxation may ever reach.
#[test]
fn the_stamp_rule_refuses_a_decreasing_pair_under_both_modes() {
    let stamps = vec![
        100 * NS_PER_MS,
        150 * NS_PER_MS,
        120 * NS_PER_MS,
        200 * NS_PER_MS,
    ];
    for gating in [Gating::Lockstep, Gating::FreeRun] {
        let reason = gating_stamp_violation(&stamps, gating)
            .unwrap_or_else(|| panic!("{gating:?} must refuse a decreasing pair: {stamps:?}"));
        assert!(reason.contains("BACKWARDS"), "{gating:?}: {reason}");
    }
}

/// **THE MODE IS LOAD-BEARING.** The very burst the free-run rule admits is a
/// FAILURE under lockstep, which is what makes passing the mode in worth doing:
/// the three lockstep callers keep the strict rule the free-run arm cannot.
#[test]
fn the_stamp_rule_refuses_under_lockstep_the_burst_it_admits_under_free_run() {
    let stamps = stamps_from_deltas(
        1_000 * NS_PER_MS,
        &[101 * NS_PER_MS, 0, 49 * NS_PER_MS, 50 * NS_PER_MS],
    );
    assert_eq!(gating_stamp_violation(&stamps, Gating::FreeRun), None);
    let reason = gating_stamp_violation(&stamps, Gating::Lockstep)
        .unwrap_or_else(|| panic!("lockstep must refuse a repeated boundary target: {stamps:?}"));
    assert!(reason.contains("under lockstep"), "{reason}");
}

/// A zero delta after a step the clock charged ONE period or less owes no second
/// fire, so it is a stopped clock however short the run of it is — the half the
/// absolute ceiling alone would miss.
#[test]
fn the_stamp_rule_refuses_a_zero_delta_after_an_ordinary_step() {
    let stamps = stamps_from_deltas(
        1_000 * NS_PER_MS,
        &[50 * NS_PER_MS, 50 * NS_PER_MS, 0, 50 * NS_PER_MS],
    );
    let reason = gating_stamp_violation(&stamps, Gating::FreeRun)
        .unwrap_or_else(|| panic!("a 50 ms step owes ONE fire: {stamps:?}"));
    assert!(reason.contains("CATCH-UP BURST"), "{reason}");
}

/// A burst LONGER than its own advance owes is refused even though the advance
/// cleared one period: the owed-fire count is the bound, not the mere fact of an
/// overrun.
#[test]
fn the_stamp_rule_refuses_a_burst_longer_than_its_advance_owes() {
    // A 101 ms advance owes two fires, so a THIRD frame at that target is one
    // the step never owed.
    let stamps = stamps_from_deltas(1_000 * NS_PER_MS, &[101 * NS_PER_MS, 0, 0, 49 * NS_PER_MS]);
    let reason = gating_stamp_violation(&stamps, Gating::FreeRun)
        .unwrap_or_else(|| panic!("a 101 ms step owes TWO fires, not three: {stamps:?}"));
    assert!(reason.contains("CATCH-UP BURST"), "{reason}");
}

/// **The co-tenancy hole, CLOSED, and this check
/// is its INVERSE.**
///
/// The always-on window recorder records what is LIVE on the DEFAULT iceoryx2
/// namespace (see [`DISCOVERY_OFF`] for why it cannot be told otherwise), and
/// every `graph run` on the machine publishes there. So a co-tenant graph — or
/// an ORPHANED `graph run-worker` from a crashed one, or a leaked service
/// directory `cerulion clean` does not sweep — puts a foreign topic in this
/// run's capture.
///
/// Without coverage this would be a PRECONDITION demanding a clean desk, because
/// `replay_engine::classify_topics` would then REFUSE the whole bag: an unmodelled
/// recorded topic is tolerated only when `record_coverage.json` marks it
/// `source: discovered`, and a capture with no coverage manifest at
/// all has nothing to mark it. MEASURED against a stale service left by a
/// long-dead sibling test:
///
/// ```text
/// bag/graph mismatch on topic `/fbnorings…/ticker/cmd`: … the bag and its
/// embedded graph disagree (corrupt or hand-edited recording)
/// ```
///
/// That is a resimmability hole (an un-resimmable capture) reported as a
/// corruption that has not happened. A capture therefore carries the manifest, and
/// replay's escape for discovered topics applies UNCHANGED.
///
/// So the check does not demand a clean desk. It demands the
/// capture EXPLAIN ITSELF, which is strictly stronger — it holds on a dirty desk
/// AND on a clean one, and it fails if the manifest ever stops being written.
/// The `source` assertion is the load-bearing one: `classify_topics` skips an
/// unmodelled topic ONLY on `discovered`, so a manifest marking these
/// `declared` is refused exactly as no manifest at all is.
///
/// Returns the topics from OUTSIDE this run's graph, so the caller can assert on
/// the resim's own `unmodelled` warn.
fn assert_the_capture_accounts_for_every_topic_it_holds(bag: &Path, prefix: &str) -> Vec<String> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let topics: Vec<String> = reader
        .channels()
        .expect("channels")
        .iter()
        .map(|c| c.topic.clone())
        .filter(|t| !t.starts_with(cerulion_bag::RESERVED_PREFIX))
        .collect();

    let attachment = reader
        .attachment(RECORD_COVERAGE_ATTACHMENT)
        .expect("attachment lookup")
        .unwrap_or_else(|| {
            panic!(
                "this capture carries NO `{RECORD_COVERAGE_ATTACHMENT}`. Without it \
                 `bag play --resim` refuses the whole bag as `corrupt or hand-edited` the moment \
                 it holds any topic this run's graph does not model — which the window recorder \
                 has no way to avoid, since it records what is LIVE on the shared namespace."
            )
        });
    let coverage: RecordCoverage = serde_json::from_slice(&attachment.data).unwrap_or_else(|e| {
        panic!(
            "a capture's coverage manifest must parse as the SAME `RecordCoverage` a recording's \
             does: {e}\n{}",
            String::from_utf8_lossy(&attachment.data)
        )
    });

    assert!(
        coverage.window_capture,
        "a capture's manifest must declare itself a WINDOW capture, or its frame counts read as \
         the recorder's lifetime account: {coverage:?}"
    );
    let foreign: Vec<String> = topics
        .iter()
        .filter(|t| !t.starts_with(&format!("/{prefix}/")))
        .cloned()
        .collect();
    for topic in &topics {
        let tap = coverage.tapped.get(topic).unwrap_or_else(|| {
            panic!("the manifest describes every channel in the bag; `{topic}` is missing")
        });
        // The `source` mark DECIDES only for a topic the graph models nowhere:
        // `classify_topics` tests produced-then-consumed FIRST, so this run's own
        // topics are replayed normally whatever the manifest says about how their
        // taps were opened. Asserting `Discovered` on them too would pin the
        // window recorder's SPAWN decision (its `--topics-json` is empty),
        // which is not what this file is about — and the failure message would be
        // wrong for them, since `declared` costs a graph-modelled topic nothing.
        if !foreign.contains(topic) {
            continue;
        }
        assert_eq!(
            tap.source,
            TapSource::Discovered,
            "`{topic}` comes from OUTSIDE this run's graph and must be marked `discovered` — \
             that mark is the whole of the evidence `replay_engine::classify_topics` accepts for \
             skipping a topic the graph does not model, and `declared` is refused exactly as an \
             absent manifest is (the bag would be rejected at exit 2 as `corrupt or \
             hand-edited`). If this is unexpected, an ORPHANED `cerulion graph run-worker` \
             (check `pgrep -f 'graph run-worker'`) or a leaked service directory is publishing \
             on the shared namespace: {coverage:?}"
        );
    }

    foreign
}

/// How many messages a bag holds on `topic`.
fn capture_frames_on(bag: &Path, topic: &str) -> usize {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let (msgs, _) = reader.recover_messages().expect("recover the capture");
    msgs.iter().filter(|m| m.topic == topic).count()
}

/// Run `cerulion bag play <bag> --resim all [extra…]` and return
/// `(exit code, stderr)`.
fn resim(root: &Path, bag: &Path, extra: &[&str], stem: &str) -> (Option<i32>, String) {
    let err_path = root.join(format!("{stem}.stderr"));
    let mut child = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "play"])
            .arg(bag)
            .args(["--resim", "all"])
            .args(extra)
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .stdout(Stdio::from(
                std::fs::File::create(root.join(format!("{stem}.stdout"))).unwrap(),
            ))
            .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
            .spawn()
            .expect("spawn cerulion bag play --resim"),
    );
    let status = child
        .wait_bounded(RESIM_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion bag play --resim` never returned"));
    (status.code(), read_file(&err_path))
}

/// What ONE line of a resim's stderr says about the re-execution, if anything.
#[derive(Debug, PartialEq, Eq)]
enum ResimSummary {
    /// The re-executed step count.
    Steps(u64),
    /// A verdict saying the re-execution did not pass, verbatim.
    Failed(String),
}

/// Is this line a top-level FAIL verdict header, `<HEADER>: <bag>`?
///
/// The class headers come from [`cerulion_cli_engine::replay_engine::DivergenceClass`]
/// itself rather than from transcribed literals, so a rename of the vocabulary
/// moves this parser with it instead of silently making it read nothing.
/// `NODE FAILURE` (the verdict's panic-class block) and `resim FAILED` (the
/// NEUTRAL renderer's crash line) are not classes and are named here.
///
/// The head must be the WHOLE of what precedes the first `": "`, which is what
/// keeps the report-only `NOTE: EDGE-READ DIVERGENCE (...)` line — rendered on
/// the pass path too, and never a verdict — out: its head is `NOTE`.
fn fail_verdict(line: &str) -> Option<&str> {
    let (head, _) = line.split_once(": ")?;
    let is_class = cerulion_cli_engine::replay_engine::DivergenceClass::ALL
        .iter()
        .any(|c| c.header() == head);
    (is_class || head == "NODE FAILURE" || head == "resim FAILED").then_some(line)
}

/// Classify one line of a resim's stderr. PURE, so the wordings below can be
/// pinned by hand oracles rather than by a run.
fn classify_resim_line(line: &str) -> Option<ResimSummary> {
    let line = line.trim();
    // The FAIL verdicts FIRST. A failing `--verify` run prints no line this
    // parser can read a count off, and the two failures need different next
    // steps from whoever reads the log.
    if let Some(verdict) = fail_verdict(line) {
        return Some(ResimSummary::Failed(verdict.to_string()));
    }
    // A bare `--resim` run: `re-executed N step(s), M produced topic(s)`.
    if let Some(rest) = line.strip_prefix("re-executed ") {
        return rest
            .split_once(" step(s)")?
            .0
            .parse::<u64>()
            .ok()
            .map(ResimSummary::Steps);
    }
    // A `--verify` run's PASS verdict, whose count REPLACES the bare summary:
    // `replay PASS: <bag> (N tick(s) replayed, ...)`.
    //
    // Anchored on the ` tick(s) replayed` marker and walked BACKWARDS over the
    // digits in front of it, never on the opening ` (`: the verdict may carry a
    // DECLINED suffix that opens a SECOND parenthesis
    // (`; 1 DECLINED (1 of them matched under the fallback, not credited)`), so
    // a right-split on ` (` lands inside that one and reads no count at all.
    let rest = line.strip_prefix("replay PASS: ")?;
    let head = rest.split_once(" tick(s) replayed")?.0;
    let digits = head.len() - head.trim_end_matches(|c: char| c.is_ascii_digit()).len();
    head[head.len() - digits..]
        .parse::<u64>()
        .ok()
        .map(ResimSummary::Steps)
}

/// The re-executed step count a resim's whole stderr reports, or the sentence
/// [`executed_steps`] panics with. PURE, so the failure wording is an oracle's
/// to check rather than a panic's.
fn resim_step_count(stderr: &str) -> Result<u64, String> {
    match stderr.lines().find_map(classify_resim_line) {
        Some(ResimSummary::Steps(n)) => Ok(n),
        Some(ResimSummary::Failed(verdict)) => Err(format!(
            "the resim was JUDGED and did not pass, so it reported no step count: {verdict}"
        )),
        None => Err("the resim summary must report its step count".to_string()),
    }
}

/// The re-executed step count the resim summary reports, in either wording:
/// a bare `--resim` run's `re-executed N step(s)`, or a `--verify` run's
/// verdict line `replay PASS: <bag> (N tick(s) replayed, ...)`, which REPLACES
/// the bare summary.
///
/// Every end-to-end arm in this file drives a BARE `--resim` — the module doc
/// says why `--verify` is deliberately not driven here — so the verdict
/// wordings are pinned by the hand oracles below and by nothing else in this
/// file. Their first end-to-end caller is the verify arm the next change in
/// this series adds.
fn executed_steps(stderr: &str) -> u64 {
    match resim_step_count(stderr) {
        Ok(n) => n,
        Err(why) => panic!("{why}\n{stderr}"),
    }
}

// The oracles below are PURE — no process, no transport — and they are still
// `#[serial]`, for the reason `cerulion_core`'s `serial_discipline_test` states:
// this file reaches the process-global iceoryx2 manager
// (`TransportManager::get_or_init` in `await_the_worker_has_stepped`) and the
// `cerulion_cli` package is NOT run with `-- --test-threads=1` in CI, so the
// rule is per FILE, not per test. A pure arm left unmarked would run beside a
// live one on a CI lane and cost the live one its serialisation.

/// The bare `--resim` wording, which the live arms also drive.
#[test]
#[serial]
fn a_bare_resim_summary_line_reports_its_step_count() {
    assert_eq!(
        classify_resim_line("  re-executed 47 step(s), 2 produced topic(s)"),
        Some(ResimSummary::Steps(47)),
        "the neutral renderer indents its summary, so the line is trimmed first"
    );
    assert_eq!(
        resim_step_count(
            "resim (no verdict): /cap.mcap\n  re-executed 47 step(s), 2 produced topic(s)\n  \
             no verdict was made\n"
        ),
        Ok(47)
    );
}

/// The `--verify` PASS wording, which no arm in this file drives.
#[test]
#[serial]
fn a_verify_pass_verdict_reports_its_tick_count() {
    let line = "replay PASS: /cap.mcap (47 tick(s) replayed, 2/2 topic(s) matched \
                byte-for-byte and were credited)";
    assert_eq!(
        classify_resim_line(line),
        Some(ResimSummary::Steps(47)),
        "the verdict REPLACES the bare summary, so its tick count is the step count"
    );
    assert_eq!(
        resim_step_count(&format!("coordination: lockstep\n{line}\n")),
        Ok(47)
    );
}

/// The same verdict carrying the DECLINED suffix — a SECOND parenthesis inside
/// the first, which is the shape a right-split on ` (` reads nothing from.
#[test]
#[serial]
fn a_verify_pass_verdict_with_a_declined_suffix_still_reports_its_tick_count() {
    assert_eq!(
        classify_resim_line(
            "replay PASS: /cap.mcap (47 tick(s) replayed, 1/2 topic(s) matched byte-for-byte \
             and were credited; 1 DECLINED (1 of them matched under the fallback, not \
             credited))"
        ),
        Some(ResimSummary::Steps(47))
    );
}

/// A verdict that did NOT pass is named, rather than reported as a missing
/// summary: the panic an operator reads must say the replay was judged and
/// lost, not send them looking at this parser.
#[test]
#[serial]
fn a_failing_verify_verdict_is_named_rather_than_read_as_a_missing_summary() {
    let stderr = "coordination: lockstep\n\
                  FRAME-CONTENT DIVERGENCE: /cap.mcap\n  \
                  47 tick(s) replayed; 1/2 topic(s) matched and were credited; 1 violation(s):\n  \
                  - /xr/relay/out [frame content]: first difference at byte 20\n";
    let why = resim_step_count(stderr).expect_err("a failing verdict reports no step count");
    assert!(
        why.contains("did not pass") && why.contains("FRAME-CONTENT DIVERGENCE: /cap.mcap"),
        "the verdict must be NAMED: {why}"
    );
    // The panic-class block and the neutral renderer's crash line are verdicts
    // too, and the report-only `NOTE:` line that carries a class header is NOT.
    assert_eq!(
        classify_resim_line("NODE FAILURE: /cap.mcap"),
        Some(ResimSummary::Failed("NODE FAILURE: /cap.mcap".to_string()))
    );
    assert_eq!(
        classify_resim_line("resim FAILED: /cap.mcap"),
        Some(ResimSummary::Failed("resim FAILED: /cap.mcap".to_string()))
    );
    assert_eq!(
        classify_resim_line(
            "NOTE: EDGE-READ DIVERGENCE (redundant per-edge verifier; the verdict and exit \
             code are UNAFFECTED): 2 edge(s) diverged across 3 step(s):"
        ),
        None,
        "a report-only note on the PASS path must never read as a verdict"
    );
}

/// A line the parser cannot read a count off yields NOTHING, rather than a
/// number it invented.
#[test]
#[serial]
fn a_malformed_summary_line_is_not_read_as_a_count() {
    for line in [
        "re-executed many step(s), 2 produced topic(s)",
        "re-executed 47 steps",
        "replay PASS: /cap.mcap (no tick count at all)",
        "replay PASS: /cap.mcap (many tick(s) replayed, 2/2 topic(s) matched)",
        "read log: verified clean (4 edge(s) compared)",
        "",
    ] {
        assert_eq!(classify_resim_line(line), None, "line: {line:?}");
    }
    assert_eq!(
        resim_step_count("resim (no verdict): /cap.mcap\nread log: inert\n"),
        Err("the resim summary must report its step count".to_string())
    );
}

/// Attach a full recorder to the live run for `secs` and return the bag path.
///
/// `--duration` rather than a signal: the verb stops itself, so the arm needs no
/// second signal path and the recorder's own finalize is the one under test.
fn record_the_run(root: &Path, home: &Path, out: &Path, secs: u64) -> String {
    let err_path = root.join("attach.stderr");
    let mut child = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "record", "--run"])
            .arg("-o")
            .arg(out)
            .args(["--duration", &secs.to_string()])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", home)
            .env(DISCOVERY_OFF.0, DISCOVERY_OFF.1)
            .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
            .stdout(Stdio::from(
                std::fs::File::create(root.join("attach.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
            .spawn()
            .expect("spawn cerulion bag record --run"),
    );
    let status = child
        .wait_bounded(ATTACH_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion bag record --run` never returned"));
    let stderr = read_file(&err_path);
    assert_eq!(
        status.code(),
        Some(0),
        "`bag record --run` must attach to the live run and finalize: {status:?}\nstderr:\n\
         {stderr}\nstdout:\n{}",
        read_file(&root.join("attach.stdout"))
    );
    stderr
}

/// Block until the run's worker has provably EXECUTED steps, by observing the
/// frames its ticker published.
///
/// # Why arm 3 cannot attach without this
///
/// `bag record --run` opens the run's trace ring at its LIVE cursor
/// (`open_at_live`), so the bag's trace begins at whatever step the worker
/// writes NEXT. If the worker has pushed nothing yet, that cursor is ZERO and
/// the recording genuinely begins at STEP 0 — where `plan_restore` answers
/// `FromStart`, because the constructor's state IS the state at step 0. Such a
/// bag is replay-grade with no anchors at all, and `bag play --resim` re-executes
/// it and exits 0, correctly.
///
/// That is not the shape arm 3 is about, and MEASURED on an idle desk the margin
/// was THREE STEPS: the run reaching [`WINDOW_HELD`] says nothing about the
/// worker, which the supervisor spawns around the same moment. Under CI load the
/// margin went to zero and the arm read its own `Some(0)` as a product failure.
///
/// So this is the rendezvous that makes the attach a MID-RUN attach by
/// CONSTRUCTION. It is a CONDITION on the run's own data plane rather than a
/// wall: load delays the frames and the wait with them, so the ordering it
/// establishes — worker steps, THEN recorder opens the ring — holds at any speed.
///
/// The subscriber is dropped before returning, so the slot it borrowed is free
/// again before the recorder attaches its own tap.
fn await_the_worker_has_stepped(topic: &str) {
    await_the_worker_has_published(topic, WORKER_STEPPED_FRAMES);
}

/// The same wait, for a caller that needs the worker FURTHER along than arm 3's
/// rendezvous does — see [`FREE_RUN_MID_RUN_FRAMES`].
fn await_the_worker_has_published(topic: &str, wanted: usize) {
    let mgr = TransportManager::get_or_init().expect("the step probe's transport");
    let start = Instant::now();
    let mut subscriber = None;
    let mut frames = 0usize;
    // The topic does not exist until the worker builds its graph, and
    // `create_subscriber_open_only` never creates one — so an early failure here
    // is "not yet", and only its LAST reason is worth reporting at the deadline.
    let mut last_open_error = String::from("(never attempted)");
    loop {
        if subscriber.is_none() {
            match mgr.create_subscriber_open_only(topic) {
                Ok(sub) => subscriber = Some(sub),
                Err(err) => last_open_error = format!("{err}"),
            }
        }
        if let Some(sub) = &subscriber {
            // Drain what is queued: the ticker publishes once per fire, so each
            // frame is one executed step.
            while sub.try_receive_one(|_| {}).unwrap_or(false) {
                frames += 1;
                if frames >= wanted {
                    return;
                }
            }
        }
        assert!(
            start.elapsed() < WORKER_STEPPED_DEADLINE,
            "the run never published {wanted} frame(s) on `{topic}` after {:?} \
             (saw {frames}; last open error: {last_open_error}). Every assertion below is about a \
             MID-RUN attach, so this is a FAILURE of the run or of this harness rather than the \
             property under test",
            start.elapsed()
        );
        std::thread::sleep(WORKER_STEPPED_POLL);
    }
}

/// The step of the bag's FIRST rank-0 `STEP_BOUNDARY` record — the number
/// `bag play --resim` derives its resume point from.
///
/// This MIRRORS `replay_engine`'s `first_recorded_boundary` (a rank-0 walk of the
/// same records) rather than deriving anything of its own: the whole point is to
/// read the value the engine will read. Zero means the recording begins at step
/// 0 and needs no anchor; anything above it is a mid-run recording, which needs
/// a complete anchor at `step - 1`.
///
/// The departure ring cannot be mistaken for rank 0 — its records carry the
/// `u32::MAX` sentinel, which `rank()` masks to `0x7FFF_FFFF`.
fn first_rank0_boundary_step(bag: &Path) -> Option<u64> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the bag");
    let (msgs, _) = reader.recover_messages().expect("recover the bag");
    msgs.into_iter()
        .filter(|m| m.topic == cerulion_bag::SCHEDULER_TRACE_TOPIC)
        .map(|m| TraceRingRecord::from_bytes(m.data[..40].try_into().expect("a 40-byte record")))
        .find(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY && r.rank() == 0)
        .map(|r| r.step)
}

/// The bag's own `__cerulion/run.json` — what the attaching recorder recorded
/// about the run it attached to.
fn attach_run_manifest(bag: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the attach bag");
    let att = reader
        .attachment("__cerulion/run.json")
        .expect("read attachments")
        .expect("a `--run` attach bag carries the run manifest it attached to");
    serde_json::from_slice(&att.data).expect("the attach run manifest is valid JSON")
}

/// **ARM 1 — THE CLOSED LOOP ON THE DEFAULT RUN SHAPE.**
///
/// The property the resimmability rule turns on, on the shape a robot runs: a plain
/// `cerulion graph run` with no flags, a real `cerulion flashback`, and a real
/// `cerulion bag play --resim` on what it wrote.
///
/// The assertion ORDER is load-bearing, and it matches the order the
/// `--record` sibling uses. The hand oracle comes FIRST (a bag whose
/// frames are not what these nodes produce makes every later assertion a claim
/// about the wrong data), then the capture's own `resimmable` verdict (so an
/// exit-0 below is the two answers AGREEING rather than both being absent), then
/// the resim itself.
#[test]
#[serial]
fn a_capture_taken_off_a_plain_run_is_a_bag_bag_play_resim_accepts() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("plainfb");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    // ------------------------------------------------------------------ leg 1
    // A PLAIN run. No `--record`, no `--single-process`, no `--no-rings`.
    let (mut run, stderr_path) = spawn_run(root, &home, &flashbacks, &[], &[]);
    wait_for_log_line(&mut run.0, &stderr_path, WINDOW_HELD);

    // Let the window fill: the ticker is `period_ms = 50`, so a second of run
    // time is ~20 frames per topic. A capture over an empty window is a
    // perfectly good capture and carries no frames to compare.
    std::thread::sleep(Duration::from_secs(2));

    // ------------------------------------------------------------------ leg 2
    // CAPTURE, through the operator's own verb. It waits out the post window and
    // prints the FINISHED verdict, so its exit is the capture's.
    let mut flash = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["flashback", "--note", "plain-run probe"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", &home)
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
    let flash_status = flash
        .wait_bounded(CAPTURE_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
    let flash_stdout = read_file(&root.join("flash.stdout"));
    assert!(
        flash_status.success(),
        "a PLAIN `graph run` holds a rolling window, so `cerulion flashback` must capture: \
         {flash_status:?}\nstdout:\n{flash_stdout}\nstderr:\n{}\nrun log:\n{}",
        read_file(&root.join("flash.stderr")),
        read_file(&stderr_path)
    );

    let captures = mcaps(&flashbacks);
    assert_eq!(
        captures.len(),
        1,
        "exactly one capture in this run's own directory, got {captures:?}\n{flash_stdout}"
    );
    let capture = captures[0].clone();

    // The run has served its purpose; stop it before the resim so the replay's
    // transport does not share a desk with the run's.
    stop_run(&mut run, &stderr_path);

    // ------------------------------------------------- the oracle, then the CLAIM
    let foreign = assert_the_capture_accounts_for_every_topic_it_holds(&capture, &prefix);
    let frames =
        assert_frames_match_the_fixture_oracle(&capture, &prefix, "the capture", Gating::Lockstep);
    assert!(
        frames > 0,
        "a capture over a live window carries frames from both graph topics"
    );

    let reader = cerulion_bag::BagReader::open(&capture).expect("open the capture");
    let manifest: serde_json::Value = serde_json::from_slice(
        &reader
            .attachment("__cerulion/flashback.json")
            .expect("read the capture manifest")
            .expect("a capture carries its own manifest")
            .data,
    )
    .expect("the capture manifest is valid JSON");
    assert_eq!(
        manifest["anchor"]["resimmable"],
        serde_json::json!(true),
        "THE PLAIN-RUN PROPERTY: a capture off a PLAIN run must CLAIM resimmable. Before the \
         always-on rings this read false with `ResimGap::NoTrace`, because the supervisor \
         stamped ring tags only under `--record`: {manifest}"
    );
    let covered = manifest["anchor"]["resim_covered_through_ns"]
        .as_u64()
        .unwrap_or_else(|| {
            panic!(
                "a resimmable capture must STATE the instant its claim reaches — a `null` here \
                 means the recorder measured no boundary in the trace it carries, which the \
                 verdict above contradicts: {manifest}"
            )
        });
    assert!(
        covered > 0,
        "the range is a real gating-clock instant from this run's own trace: {manifest}"
    );

    // ------------------------------------------------------------------ leg 3
    // RESIM. NEUTRAL first: exit 0 means the re-execution could be PERFORMED,
    // which is the gate the always-on rings moved.
    let (code, resim_err) = resim(root, &capture, &[], "resim");
    assert_eq!(
        code,
        Some(0),
        "THE CLOSED LOOP on the default run shape: a capture this recorder stamped \
         `resimmable: true` must be a bag `bag play --resim` accepts. Exit 2 naming a missing \
         `__cerulion/trace_manifest_rank<N>.json` is a run that minted no rings — the plain-run \
         defect itself; exit 2 naming a `bag/graph mismatch` on one of {foreign:?} is the \
         co-tenancy hole, which this desk happened to reproduce.\nstderr:\n{resim_err}"
    );
    let executed = executed_steps(&resim_err);
    assert!(
        executed > 0,
        "the resim must actually re-execute the capture's suffix, got {executed} step(s):\n\
         {resim_err}"
    );

    // The OVER-BROAD-ESCAPE control. On a desk where nothing else is
    // publishing, a capture holds only this run's topics — every one of which
    // the graph MODELS — so the escape must not fire at all.
    //
    // Precisely what it covers, since arm 5's own third assertion (the line does
    // NOT name this run's topics) already rules out a classifier that reports them:
    // this catches an escape that fires when there is nothing foreign to escape
    // — a classifier that skipped topics on some other evidence, or one whose
    // `discovered` set was not read from the manifest at all.
    //
    // Secondary cost: it re-introduces a dependency on a clean desk. A
    // co-tenant left running by something else on this machine fails HERE rather
    // than at a labelled precondition — check `pgrep -f 'graph run-worker'` and
    // the iceoryx2 service directory before suspecting the classifier.
    let warns = unmodelled_lines(&resim_err);
    assert!(
        warns.is_empty(),
        "a capture of a clean desk models every topic it holds, so the replay skips NOTHING. \
         A line here names a topic that got into this capture from outside the run — an \
         ORPHANED `cerulion graph run-worker` (check `pgrep -f 'graph run-worker'`) or a leaked \
         service directory will each produce it: {warns:?}"
    );

    // …and the rank-0 trace manifest the gate above refuses a bag without.
    // Asserted AFTER the resim, deliberately: it is a REFINEMENT of the exit-0
    // claim, not a substitute — read earlier it PREEMPTS the headline, so a
    // ring-less run would fail on an attachment count rather than on the verb
    // REFUSING the bag, which is the failure this arm exists to reproduce.
    assert!(
        reader
            .attachment("__cerulion/trace_manifest_rank0.json")
            .expect("read the rank-0 trace manifest")
            .is_some(),
        "the resim accepted this bag, so a missing rank-0 trace manifest would mean the gate \
         stopped reading one — `load_trace_manifests` refuses a bag with zero manifests"
    );

    // ---------------------------------------------------- what `--verify` is NOT
    //
    // The byte-identity half of this arm is the HAND ORACLE above, not a
    // `--verify` leg, and that is a MEASURED conclusion rather than the
    // `--record` sibling's convention inherited unexamined.
    //
    // `--verify` is the byte-exact diff of what the re-execution PRODUCED
    // against what the bag HOLDS, and a rolling window is lossy by construction:
    // it is fed by ordinary data-only taps, so a burst past a tap's provisioned
    // queue depth is gone (counted as `frames_lost`). The replay re-executes
    // every step and produces them all, so the two counts differ and every frame
    // after the first hole carries a shifted wire `sequence`. MEASURED on this
    // fixture, driven for real before this leg was dropped:
    //
    // ```text
    // recorded 817 frame(s), replay produced 837 on '…/ticker/cmd'
    // frame 43 differs at byte 20 …          (byte 20 IS the sequence field)
    // ```
    //
    // — 0 of 2 topics matched, on a capture the same command had just accepted.
    // So a `--verify` leg here would not be a stronger plain-run assertion; it
    // would be an assertion that the window is lossless, which it does not claim
    // to be. Byte-exactness against a LOSSLESS recording is `--record`'s
    // property and has its own e2e (`mp_record_replay_e2e_test`).
    //
    // What is asserted instead, and what it is worth: the recorded frames are
    // byte-exact against a hand oracle written from the fixtures' source (their
    // full 32-byte header AND their payload), and this same bag re-executes.
    // The golden is independently checked; nothing here is a self-compare.
}

/// A live producer on the DEFAULT namespace that this run's graph knows nothing
/// about: a CO-TENANT, the shape the capture's coverage manifest exists for.
///
/// It publishes on its own thread so its frames span the whole capture window,
/// and stops on `Drop` — a leaked publisher on the shared namespace outlives the
/// whole test binary and shows up in the NEXT arm's capture (the hazard
/// [`RunGuard`] guards against for leaked workers).
struct CoTenant {
    stop: Arc<AtomicBool>,
    joiner: Option<std::thread::JoinHandle<()>>,
}

impl CoTenant {
    /// Publish ~50 Hz on `topic` until dropped.
    ///
    /// A hand-built wire frame, not a typed publish: what matters is that the
    /// recorder's discovery finds a LIVE producer and taps it, and a valid
    /// 32-byte header is all a tap needs to record one (a headerless frame is
    /// recorded too, but with a FABRICATED sequence, which would make the
    /// capture's own health report describe this test rather than the feature).
    fn spawn(topic: &str) -> Self {
        let mgr = TransportManager::get_or_init().expect("the co-tenant's transport");
        let mut publisher = mgr
            .create_publisher(topic, MaxSliceLen::const_new(256), 0)
            .expect("the co-tenant's publisher");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let joiner = std::thread::spawn(move || {
            let mut seq = 0u32;
            while !flag.load(Ordering::Relaxed) {
                let body = b"co-tenant";
                let total = WireHeader::SIZE + body.len();
                let mut header =
                    WireHeader::new(0x1420_1420_1420_1420, seq, 1_000 + u64::from(seq));
                header.total_size = total as u32;
                let mut frame = vec![0u8; total];
                header.write_to_buf(&mut frame[..WireHeader::SIZE]);
                frame[WireHeader::SIZE..].copy_from_slice(body);
                // A publish failure is not this arm's subject and must not abort
                // a detached thread mid-run; the arm's own PRECONDITION (the
                // topic really is in the capture) is what fails if nothing lands.
                let _ = publisher.publish_raw(&frame);
                seq = seq.wrapping_add(1);
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        Self {
            stop,
            joiner: Some(joiner),
        }
    }
}

impl CoTenant {
    /// Block until the recorder's discovery rescan has ATTACHED a tap to this
    /// topic — the STATE the next publish depends on.
    ///
    /// `topic_subscriber_count` is a production accessor that reads iceoryx2's
    /// dynamic config without creating a port, so asking costs the recorder
    /// nothing. `>= 1` is exact here: this process creates the topic's only
    /// publisher and attaches no subscriber of its own, so the only port that can
    /// appear is the recorder's tap. Same helper, same reasoning, as
    /// `discovery_e2e_test::await_rescan_tap`.
    fn await_tapped(&self, topic: &str) {
        let mgr = TransportManager::get_or_init().expect("the co-tenant's transport");
        let start = Instant::now();
        loop {
            if mgr.topic_subscriber_count(topic) >= 1 {
                return;
            }
            assert!(
                start.elapsed() < CO_TENANT_TAP_DEADLINE,
                "the window recorder's discovery rescan never attached a tap to the co-tenant's \
                 topic `{topic}` after {:?}. Every frame published from here lands in no queue at \
                 all, so this is a FAILURE of the recorder or of this harness — not the co-tenancy \
                 property the assertions below are about",
                start.elapsed()
            );
            std::thread::sleep(CO_TENANT_TAP_POLL);
        }
    }
}

impl Drop for CoTenant {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(joiner) = self.joiner.take() {
            let _ = joiner.join();
        }
    }
}

/// Every `unmodelled` line a resim printed — the replay emits ONE for the whole
/// skipped set, so the COUNT is a claim about the reporting as well as about
/// the classifying.
fn unmodelled_lines(stderr: &str) -> Vec<&str> {
    stderr
        .lines()
        .filter(|l| l.contains("its graph does not model"))
        .collect()
}

/// **ARM 5: THE CO-TENANT.**
///
/// Arm 1 is the closed loop on a desk where nothing else is publishing. This is
/// the same loop on the desk a robot actually has: a SECOND live producer on the
/// DEFAULT iceoryx2 namespace that this run's graph neither produces nor
/// consumes — another graph, an orphaned `graph run-worker`, a leaked service.
///
/// Without a coverage manifest that ONE stranger topic makes the capture un-resimmable.
/// `replay_engine::classify_topics` tolerates an unmodelled recorded topic only
/// when the bag's own `record_coverage.json` marks it `source: discovered`; a
/// capture carrying no such manifest is refused at exit 2 as
/// "corrupt or hand-edited" — a corruption that has not happened, on a black box
/// the operator most needs when something else on the machine has gone wrong.
///
/// The oracle is the OUTCOME (exit 0) plus EXACTLY ONE `unmodelled` warn naming
/// the stranger and NOT naming this run's own topics. The run's own frames are
/// anchored to the SAME hand oracle arm 1 uses BEFORE the resim, so an exit-0
/// here is a statement about real data rather than about a bag that parses.
///
/// Two of the three assertions are self-sufficient: a build that reported the
/// run's own topics as unmodelled fails the third (the line does NOT name this
/// run's prefix). What arm 1 adds is the OTHER direction — an escape that fires
/// when there is nothing foreign to escape — which no assertion here can see.
#[test]
#[serial]
fn a_capture_holding_a_co_tenants_topic_is_still_a_bag_resim_accepts() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("plainco");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    // The stranger. Started BEFORE the run so the window recorder's arm-time
    // scan (and every rescan after it) can see a live producer; its topic is
    // process-unique for the reason `unique_prefix` exists — a fixed name
    // collides with a service left behind by a SIGKILLed earlier run.
    let stranger_topic = format!("/{}/stranger", unique_prefix("cotenant"));
    let stranger = CoTenant::spawn(&stranger_topic);

    let (mut run, stderr_path) = spawn_run(root, &home, &flashbacks, &[], &[]);
    wait_for_log_line(&mut run.0, &stderr_path, WINDOW_HELD);

    // RENDEZVOUS on the tap, then fill. The arm's verdict needs the stranger's
    // FRAMES in the window (`classify_topics` walks recorded MESSAGES, not
    // channels), and a tap has no back-fill — so waiting for the tap to exist
    // before counting on the frames is what keeps a starved runner from
    // inverting the verdict instead of failing the precondition.
    stranger.await_tapped(&stranger_topic);
    std::thread::sleep(CO_TENANT_FILL);

    let mut flash = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["flashback", "--note", "co-tenant probe"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", &home)
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
    let flash_status = flash
        .wait_bounded(CAPTURE_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
    let flash_stdout = read_file(&root.join("flash.stdout"));
    assert!(
        flash_status.success(),
        "a PLAIN `graph run` holds a rolling window, so `cerulion flashback` must capture: \
         {flash_status:?}\nstdout:\n{flash_stdout}\nstderr:\n{}\nrun log:\n{}",
        read_file(&root.join("flash.stderr")),
        read_file(&stderr_path)
    );

    let captures = mcaps(&flashbacks);
    assert_eq!(
        captures.len(),
        1,
        "exactly one capture in this run's own directory, got {captures:?}\n{flash_stdout}"
    );
    let capture = captures[0].clone();

    stop_run(&mut run, &stderr_path);
    // The stranger has served its purpose; stop it before the resim so the
    // replay's transport does not share a namespace with a live publisher.
    drop(stranger);

    // THE PRECONDITION, asserted rather than assumed: without the stranger IN
    // the capture this arm reduces to arm 1 and proves nothing about co-tenancy.
    let foreign = assert_the_capture_accounts_for_every_topic_it_holds(&capture, &prefix);
    assert!(
        foreign.contains(&stranger_topic),
        "PRECONDITION: the capture must hold a CHANNEL for the co-tenant's topic \
         `{stranger_topic}`. Without it this arm is arm 1 with extra steps. Foreign topics \
         found: {foreign:?}"
    );
    // …and FRAMES, not merely a channel. The verdict below counts `unmodelled`
    // lines, and `classify_topics` walks the recording's MESSAGES — a topic with
    // a channel and no frame is classified by nothing and reported nowhere. A
    // precondition weaker than the verdict sends the diagnosis to the wrong
    // place on exactly the runs that need it.
    let stranger_frames = capture_frames_on(&capture, &stranger_topic);
    assert!(
        stranger_frames > 0,
        "PRECONDITION: the capture's window must hold FRAMES of `{stranger_topic}`, not just a \
         channel. The co-tenant publishes from before the run starts, and frames committed \
         BEFORE the recorder's tap attached are gone by construction (a data-only tap requests \
         no late-joiner history) — so what must have landed is the \
         {CO_TENANT_FILL:?} of publishing AFTER the rendezvous. Zero here means the window \
         evicted even those, or the publisher thread died"
    );

    // The run's OWN frames still match the fixtures' hand oracle, so exit 0
    // below is a claim about real data.
    let frames =
        assert_frames_match_the_fixture_oracle(&capture, &prefix, "the capture", Gating::Lockstep);
    assert!(frames > 0, "a capture over a live window carries frames");

    let (code, resim_err) = resim(root, &capture, &[], "resim");
    assert_eq!(
        code,
        Some(0),
        "project rule: a capture holding a co-tenant's topic must still be a \
         bag `bag play --resim` ACCEPTS. Exit 2 naming a `bag/graph mismatch` on \
         `{stranger_topic}` is the co-tenancy hole itself — the capture's coverage manifest is \
         missing, unreadable, or marks that topic `declared`.\nstderr:\n{resim_err}"
    );

    // …and it SAYS what it skipped.
    let warns = unmodelled_lines(&resim_err);
    assert_eq!(
        warns.len(),
        1,
        "the replay reports the topics it skipped ONCE for the whole set, never one \
         per topic: {warns:?}\n{resim_err}"
    );
    assert!(
        warns[0].contains(&stranger_topic),
        "…and the line names the co-tenant `{stranger_topic}`: {}\n{resim_err}",
        warns[0]
    );
    assert!(
        !warns[0].contains(&format!("/{prefix}/")),
        "…and does NOT name this run's own topics, which the graph DOES model: {}",
        warns[0]
    );

    let executed = executed_steps(&resim_err);
    assert!(
        executed > 0,
        "the resim must actually re-execute the capture's suffix, got {executed} step(s):\n\
         {resim_err}"
    );
}

/// **ARM 2: `--no-rings`, on BOTH surfaces.**
///
/// There is no "capture that says declined": with no
/// rings nothing captured could be re-executed, and there is no
/// frames-only exception, so the run takes NO captures rather than un-resimmable
/// ones. What an operator gets is asserted on both surfaces instead:
///
/// 1. `cerulion flashback` reports that NOTHING answered, and writes no bag.
/// 2. A `bag record --run` attach to that same run carries the run's own
///    DECLINED statement, and `bag play --resim` refuses the bag it makes.
///
/// (2) is what makes this more than a restatement of
/// `flashback_argv_e2e_test::no_rings_declines_both_the_rings_and_the_window_recorder`:
/// that arm reads the run's `run.json` off the filesystem, this one follows the
/// same statement into a BAG and then into the replay verb's exit code.
#[test]
#[serial]
fn no_rings_takes_no_capture_and_its_attach_bag_is_refused_by_resim() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("plainnr");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    let (mut run, stderr_path) = spawn_run(root, &home, &flashbacks, &["--no-rings"], &[]);
    // Wait for the point where the decision under test has actually been TAKEN.
    wait_for_log_line(&mut run.0, &stderr_path, FLASHBACK_DECISION);
    let log = strip_ansi(&read_file(&stderr_path));
    assert!(
        log.contains(r#"window="declined_no_rings""#),
        "`--no-rings` must reach its window-recorder decision and NAME that flag as the \
         cause:\n{log}"
    );

    // (1) The operator's own verb, against a run that is holding no window.
    let mut flash = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["flashback", "--note", "no-rings probe"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", &home)
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
    let flash_status = flash
        .wait_bounded(CAPTURE_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
    let flash_all = format!(
        "{}\n{}",
        read_file(&root.join("flash.stdout")),
        read_file(&root.join("flash.stderr"))
    );
    assert_ne!(
        flash_status.code(),
        Some(0),
        "a verb that printed 'asked' and exited 0 while nothing was holding a window would be \
         lying about the artifact it implies: {flash_status:?}\n{flash_all}"
    );
    assert!(
        strip_ansi(&flash_all).contains("no serving graph on this machine answered"),
        "…and it must SAY nothing answered, rather than failing opaquely:\n{flash_all}"
    );
    assert!(
        mcaps(&flashbacks).is_empty(),
        "the project rule admits no frames-only capture, so a `--no-rings` run writes NONE: {:?}",
        mcaps(&flashbacks)
    );

    // (2) The SAME run, followed into a bag: a mid-run attach carries the run's
    // own DECLINED statement, and the replay verb refuses what it made.
    let out = root.join("attach.mcap");
    let attach_log = record_the_run(root, &home, &out, 3);
    let bag_manifest = attach_run_manifest(&out);
    let trace = bag_manifest["trace"]
        .as_str()
        .unwrap_or_else(|| panic!("a `--run` attach bag states its trace: {bag_manifest}"));
    assert!(
        trace.contains("DECLINED scheduler-trace rings at launch"),
        "the run DECLINED rings, and that is a different fact from a run that wanted them and \
         could not have them, or one that predates them. Collapsing it into the legacy \
         'gave no reason' arm tells the reader nothing they can act on: {trace}\n\
         attach log:\n{attach_log}"
    );
    // …and it is not the LEGACY arm wearing a different sentence: that one is
    // what an empty ring list would be silently collapsed into, and it names
    // three causes none of which is this one.
    assert!(
        !trace.contains("gave no reason"),
        "a run that stated its choice must not be reported as one that said nothing: {trace}"
    );
    // By design an absence explanation names the CAUSE, never another verb's
    // flag. The reader of this bag is holding an artifact, not a command line.
    assert!(
        !trace.contains("--no-rings"),
        "the bag's statement names the cause, not the flag: {trace}"
    );

    let (code, resim_err) = resim(root, &out, &[], "resim_nr");
    assert_eq!(
        code,
        Some(2),
        "a bag with no scheduler trace is NOT replay-grade, and `bag play --resim` must refuse \
         it (exit 2) rather than re-execute nothing and report success:\n{resim_err}"
    );
    // …and it must refuse it for the RIGHT reason. Exit 2 is also what a corrupt
    // bag, a schema drift and a bag/graph mismatch get, so the code alone is
    // satisfied by refusals that say nothing about the trace — and a co-tenant
    // topic in the bag produces exactly one of those (see DISCOVERY_OFF).
    //
    // The refusal names the CHANNEL rather than a missing rank manifest, because
    // a run that declared no rings still gets a scheduler-trace channel — it is
    // simply empty, which is a more precise thing to be told than "an
    // attachment is missing".
    assert!(
        resim_err.contains("scheduler_trace"),
        "the refusal must name the MISSING SCHEDULER TRACE — a bare exit 2 is also what a \
         corrupt or mismatched bag gets, and this bag is neither:\n{resim_err}"
    );

    stop_run(&mut run, &stderr_path);
}

/// **ARM 3 — the OTHER consumer of the same rings.**
///
/// A capture proves the STANDING recorder can read a plain run's trace rings.
/// This proves a recorder that arrives LATER can too, which is the half
/// `run.json`'s ring declaration exists for: the standing recorder is handed its
/// ring names on the command line and never reads the manifest, so a declaration
/// that is wrong (or absent) is invisible to arm 1.
///
/// The oracle is the bag's own `__cerulion/run.json` — `TRACE_FROM_ATTACH`, the
/// verdict that says rings were found AND opened — plus the trace manifest the
/// replay gate reads, plus the coverage manifest's own `attached_late` marker
/// (an attach records from where it attached, and a bag that did not say so
/// would imply a coverage it does not have).
///
/// It then follows the same bag into `bag play --resim`, which must REFUSE it on
/// anchor coverage. That half rests on two facts read off the bag rather than on
/// when the attach happened to land — the trace begins above step 0, and the
/// attach declined the state plane — and the run is driven so that both hold by
/// construction. The note in the body says what that replaced and why.
#[test]
#[serial]
fn a_bag_record_run_attach_to_a_plain_run_carries_its_trace() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("plainatt");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    let (mut run, stderr_path) = spawn_run(root, &home, &flashbacks, &[], &[]);
    wait_for_log_line(&mut run.0, &stderr_path, WINDOW_HELD);
    // THE MID-RUN RENDEZVOUS. `WINDOW_HELD` is the SUPERVISOR's landmark and says
    // nothing about the worker, so without this the recorder can open the trace
    // ring before a single boundary has been pushed and the bag begins at step 0
    // — a from-start recording wearing a mid-run attach's name. See
    // `await_the_worker_has_stepped`.
    await_the_worker_has_stepped(&format!("/{prefix}/ticker/cmd"));

    let out = root.join("attach.mcap");
    let attach_log = record_the_run(root, &home, &out, 4);
    stop_run(&mut run, &stderr_path);

    let manifest = attach_run_manifest(&out);
    assert_eq!(
        manifest["attached_mid_run"],
        serde_json::json!(true),
        "an attach records from where it attached, and the bag says so: {manifest}"
    );
    let trace = manifest["trace"]
        .as_str()
        .unwrap_or_else(|| panic!("a `--run` attach bag states its trace: {manifest}"));
    assert!(
        trace.contains("from the attach point"),
        "THE PLAIN-RUN PROPERTY for the late consumer: a plain run DECLARES rings and a mid-run \
         recorder OPENS them. Before this, a plain run's manifest declared none and this read \
         the legacy 'declares no trace rings' arm: {trace}\nattach log:\n{attach_log}"
    );

    let reader = cerulion_bag::BagReader::open(&out).expect("open the attach bag");
    assert!(
        reader
            .attachment("__cerulion/trace_manifest_rank0.json")
            .expect("read attachments")
            .is_some(),
        "the rings it opened produced a rank-0 trace manifest — the attachment `bag play \
         --resim`'s own gate refuses a bag without"
    );

    // COVERAGE: the recorder states what it tapped and that it began mid-run.
    let coverage: serde_json::Value = serde_json::from_slice(
        &reader
            .attachment("__cerulion/record_coverage.json")
            .expect("read attachments")
            .expect("a `--run` attach bag carries its coverage manifest")
            .data,
    )
    .expect("the coverage manifest is valid JSON");
    let tapped = coverage["tapped"]
        .as_object()
        .unwrap_or_else(|| panic!("coverage lists what was tapped: {coverage}"));
    let ticker = format!("/{prefix}/ticker/cmd");
    let entry = tapped.get(&ticker).unwrap_or_else(|| {
        panic!("the run DECLARES `ticker/cmd`, so a `--run` attach taps it: {coverage}")
    });
    assert_eq!(
        entry["source"],
        serde_json::json!("declared"),
        "`--run` records the topics the RUN DECLARES — that is what makes the flag mean \
         'record this run' rather than 'record whatever is live here': {coverage}"
    );
    assert!(
        tapped
            .values()
            .all(|t| t["attached_late"] == serde_json::json!(true)),
        "every channel of a mid-run attach is marked `attached_late` — a data-only tap requests \
         no back-fill, so coverage genuinely begins at the attach instant and nothing implies \
         otherwise: {coverage}"
    );
    // The same property, stated in the durable artifact as well as in the `trace`
    // sentence above: the run DECLARED rings and this recorder counted them.
    assert_eq!(
        coverage["rings_declared"],
        serde_json::json!(2),
        "a 1-group multi-process run declares ONE worker ring plus the departure ring, and a \
         mid-run attach reads both out of `run.json`: {coverage}"
    );

    // …and the frames it did capture are the ones these fixtures produce.
    assert_frames_match_the_fixture_oracle(&out, &prefix, "the attach bag", Gating::Lockstep);

    // The declined-plane refusal — THE CONSEQUENCE, measured rather than
    // assumed.
    //
    // This run has a standing Flashback recorder (the always-on plane makes that the
    // DEFAULT shape), so the attach above DECLINED state-ring discovery: those
    // rings admit exactly one consumer and the standing recorder is it. The
    // question the refusal leaves open is what that costs the bag, and the answer is not
    // derivable from the refusal — the bag still carries its scheduler TRACE
    // (asserted above) and its FRAMES, so a reader could reasonably expect a
    // resim to work.
    //
    // It does not. MEASURED: exit 2, refused on ANCHOR COVERAGE — a mid-run
    // resume needs every executed node's state at one step, and a declined
    // sweep records none. The exit code alone is not the pin: 2 is also what a
    // corrupt bag, a schema drift and a bag/graph mismatch get, so the arm names
    // the reason as well.
    //
    // The distinction from its `--no-rings` sibling is the point. That bag is
    // refused for a MISSING TRACE (`scheduler_trace`); this one has a trace and
    // is refused for missing ANCHORS. Two different declined-plane bags, two
    // different refusals, and asserting only the code would let either message
    // drift into the other's.
    //
    // # What is TIMING-SHAPED here, and what pins it
    //
    // The `Some(2)` below must not rest on a wall nobody has written down. An
    // arm that attached as soon as the SUPERVISOR said it was holding a window and
    // simply hoped the WORKER had already stepped would race it: `bag record --run` opens
    // the run's trace ring at its LIVE cursor, so a worker that has pushed
    // nothing yet yields a recording that begins at STEP 0. At step 0
    // `plan_restore` answers `FromStart` (the constructor's state IS the state),
    // the bag is replay-grade with no anchors at all, and the resim re-executes
    // it and exits 0 — correctly, about a bag that is not the one this arm is
    // about. MEASURED on an idle desk, the whole margin is THREE STEPS; on a
    // loaded machine it goes to zero and the arm reports a product failure
    // (`left: Some(0)  right: Some(2)`, 81 steps re-executed).
    //
    // Nothing below is loosened. The shape is
    // established by CONSTRUCTION — `await_the_worker_has_stepped` above orders
    // the worker's steps before the recorder's ring attach, on the run's own
    // data plane rather than on a clock — and the two facts the refusal actually
    // rests on are READ OFF THE BAG instead of assumed:
    //
    //   * its trace begins ABOVE step 0, so a resume needs an anchor at all;
    //   * the attach DECLINED the state plane, so it recorded none.
    //
    // Both are load-independent, and a run that failed to produce either fails
    // on its own precondition naming which — never by inverting the verdict
    // below into a claim about the product.
    let first_boundary = first_rank0_boundary_step(&out).unwrap_or_else(|| {
        panic!(
            "PRECONDITION: this bag carries a rank-0 trace manifest (asserted above), so its \
             trace must carry a rank-0 STEP_BOUNDARY to resume from. A bag with none is the \
             `--no-rings` sibling's shape, not this one's"
        )
    });
    assert!(
        first_boundary > 0,
        "PRECONDITION: a MID-RUN attach records from where it attached, so its trace begins \
         ABOVE step 0 — which is what makes an anchor necessary at all. A trace beginning at \
         step 0 is a from-start recording, which `plan_restore` answers `FromStart` for and \
         `--resim` then accepts; that is a different bag and the refusal below would not be \
         about it. `await_the_worker_has_stepped` exists to make this impossible, so reaching \
         it means the worker pushed no boundary despite publishing \
         {WORKER_STEPPED_FRAMES} frame(s)"
    );
    // …and THE DECLINE ITSELF, which is why the bag carries no anchors.
    //
    // Timing matters for this claim: if the attach raced the worker's
    // first step, a sweeping attach would find no complete anchor either (the
    // cadence is 15 s and the run lives seconds), so deleting the refusal would
    // leave the arm green. The manifest states the
    // decision directly, and a real `graph run` + a real attach is the only
    // place that statement can be checked end to end: the three state-plane
    // arms in `cerulion_cli_engine/tests/bag_record_run_attach_test.rs` drive
    // HAND-WRITTEN manifests, and arm 5 below pins only the RUN's half of the
    // exchange.
    //
    // The two assertions are BOTH here because they fail apart, and that was
    // MEASURED rather than reasoned: the verdict string is
    // rendered from the reading taken ABOVE the branch that acts on it, so
    // `if false` leaves it saying `refused` while the recorder sweeps. It is the
    // ATTACHMENT that moves, and that is what this assertion is designed to catch.
    assert_eq!(
        manifest[cerulion_cli_engine::bag_cmd::STATE_RINGS_KEY].as_str(),
        Some(cerulion_cli_engine::bag_cmd::STATE_RINGS_REFUSED_STANDING),
        "PRECONDITION, and the decline pin: by design this plain run has a standing window \
         recorder holding its per-rank node-state rings, so the attach must DECLINE them and \
         say so in the bag. An attach that swept instead would record anchors whose presence \
         depends on where the 15 s cadence fell — which is exactly the timing-shaped verdict \
         this arm does not have: {manifest}"
    );
    // …and it must really have declined, not merely said so. A verdict string is
    // rendered from the reading taken BEFORE the branch that acts on it, so a
    // build that kept the words and dropped the refusal would satisfy the
    // assertion above while sweeping. `state_coverage.json` is written exactly
    // when the recorder was configured for checkpoints, so its ABSENCE is the
    // behavioural half — the same oracle, over a real run, that
    // `bag_record_run_attach_test`'s `a_standing_recorder_…` reads off
    // `BagdSummary::state_coverage`.
    assert!(
        reader
            .attachment(cerulion_bagd::STATE_COVERAGE_ATTACHMENT)
            .expect("read attachments")
            .is_none(),
        "a DECLINED attach sweeps no state rings, so it writes no \
         `{}` — its presence would mean the sweep ran anyway, and the refusal below would then \
         be about whichever anchors the 15 s cadence happened to land in this run's few seconds",
        cerulion_bagd::STATE_COVERAGE_ATTACHMENT
    );
    let (resim_code, resim_err) = resim(root, &out, &[], "d4attach");
    assert_eq!(
        resim_code,
        Some(2),
        "a mid-run attach that declined the state plane carries no anchors, so it is not \
         replay-grade and `bag play --resim` must refuse it rather than re-execute from a \
         state it does not have (this bag's trace begins at step {first_boundary}, so a \
         complete anchor at step {} was required):\n{resim_err}",
        first_boundary - 1
    );
    assert!(
        resim_err.contains("no anchor recorded") && resim_err.contains("resumes mid-run"),
        "…and refuse it for the ANCHOR reason, not the trace one: this bag HAS a scheduler \
         trace (asserted above), which is exactly what makes a bare exit 2 ambiguous \
         here:\n{resim_err}"
    );
    assert!(
        !resim_err.contains("scheduler_trace"),
        "…and must NOT report the missing-trace refusal its `--no-rings` sibling gets — the \
         two bags fail for different reasons and an operator acting on the wrong one \
         re-records a run that was already carrying what they were told it lacked:\n{resim_err}"
    );
}

/// **ARM 4 — NO-INERT-SHIPPING: rings are not gated on the kill switch.**
///
/// `CERULION_FLASHBACK=off` turns off the capture plane — the arm word, the
/// per-rank state rings, and the standing window recorder. It must NOT turn off
/// the scheduler-trace rings, or `cerulion bag record --run` would silently lose
/// the trace on every switched-off run: the two planes are orthogonal, and a
/// robot that turned the black box off to save memory has not asked to make a
/// deliberate recording unreplayable.
///
/// This arm is the ONLY one that fails the change it exists for — adding
/// `&& !flashback_switched_off()` to the ring-stamping predicate. Arms 1 and 3
/// both run with the plane ON, so both stay green under it.
#[test]
#[serial]
fn a_flashback_switched_off_run_still_stamps_rings_for_a_later_attach() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("plainoff");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    let (mut run, stderr_path) = spawn_run(
        root,
        &home,
        &flashbacks,
        &[],
        &[("CERULION_FLASHBACK", "off")],
    );
    wait_for_log_line(&mut run.0, &stderr_path, FLASHBACK_DECISION);
    // The PRECONDITION, asserted rather than assumed: this arm is only about a
    // switched-off plane if the plane really is off. A build that ignored the
    // kill switch would satisfy every assertion below while testing arm 3 again.
    let log = strip_ansi(&read_file(&stderr_path));
    assert!(
        log.contains(r#"window="switched_off""#),
        "the kill switch must really have taken the window recorder off, or this arm is arm 3 \
         in disguise:\n{log}"
    );

    let out = root.join("attach.mcap");
    let attach_log = record_the_run(root, &home, &out, 4);
    stop_run(&mut run, &stderr_path);

    let manifest = attach_run_manifest(&out);
    let trace = manifest["trace"]
        .as_str()
        .unwrap_or_else(|| panic!("a `--run` attach bag states its trace: {manifest}"));
    assert!(
        trace.contains("from the attach point"),
        "trace rings are NOT gated on `CERULION_FLASHBACK`. Gating them there loses the \
         scheduler trace on every switched-off run's deliberate recording: {trace}\n\
         attach log:\n{attach_log}"
    );

    // The BEHAVIOURAL half: STEP BOUNDARIES really are in the bag. A verdict
    // alone is satisfied by rings that were declared and opened and yielded
    // nothing, and a bare record count by a trace of FIREs with no boundary —
    // which is `ResimGap::NoBoundary`, i.e. still not re-executable. The
    // boundary is the record carrying the step's gating-clock value, and it is
    // what a resume re-advances to.
    let reader = cerulion_bag::BagReader::open(&out).expect("open the attach bag");
    let trace_records = reader
        .scheduler_trace()
        .expect("the bag's scheduler-trace channel decodes");
    let boundaries = trace_records
        .iter()
        .filter(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_STEP_BOUNDARY)
        .count();
    assert!(
        boundaries > 0,
        "the rings must have YIELDED step boundaries, not merely been opened — a resume derives \
         its step from a boundary, so a trace without one is `ResimGap::NoBoundary`. The bag \
         carries {} trace record(s) in total.",
        trace_records.len()
    );
}

/// The `run.json` of the ONE run this test's `CERULION_HOME` holds.
///
/// Read while the run is LIVE, deliberately: `RunDescriptor::drop` removes the
/// directory on a clean exit, so a manifest read after `stop_run` is a manifest
/// read from a directory that no longer exists.
fn live_run_manifest(home: &Path) -> serde_json::Value {
    // `CERULION_HOME` is taken as the config ROOT itself, so the runs live
    // directly under it — NOT under a `.cerulion` component, which is only
    // appended when the env is unset and the home directory is used.
    let runs = home.join("runs");
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(&runs)
        .unwrap_or_else(|e| panic!("read {}: {e}", runs.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .collect();
    dirs.sort();
    assert_eq!(
        dirs.len(),
        1,
        "this arm's HOME holds exactly one run: {dirs:?}"
    );
    let path = dirs[0].join("run.json");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    serde_json::from_slice(&bytes).unwrap_or_else(|e| panic!("{} is JSON: {e}", path.display()))
}

/// **The declined-plane refusal, arm 5 — the RUN's half:** a real `graph run` writes its
/// window-recorder decision into `run.json`, so a later attach can read it.
///
/// The attach's half is pinned over hand-written manifests in
/// `cerulion_cli_engine/tests/bag_record_run_attach_test.rs`, where every state
/// is reachable without spawning a graph to produce it. What CANNOT be pinned
/// there is that a real run produces the state those arms assume — and the
/// failure mode is silent in exactly the way that matters: a build that never
/// wrote the key leaves every attach on the UNKNOWN arm, which by design
/// PROCEEDS, so the refusal would ship inert with the attach suite green.
///
/// Both shapes in ONE body, because the discriminator is the pair: a run that
/// starts a standing recorder against a run whose kill switch stops one. Either
/// assertion alone is satisfied by a writer that hardcodes its answer.
#[test]
#[serial]
fn a_run_writes_its_window_recorder_decision_into_run_json() {
    // (a) The DEFAULT shape: it gets a standing window recorder by design, and
    //     that recorder holds the run's capture-plane tag — so it IS a
    //     state-ring consumer and the manifest must say so.
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = unique_prefix("d4write");
    build_workspace(root, &prefix);
    let home = root.join("home");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();

    let (mut run, stderr_path) = spawn_run(root, &home, &flashbacks, &[], &[]);
    wait_for_log_line(&mut run.0, &stderr_path, FLASHBACK_DECISION);
    // The PRECONDITION: this arm is only about a standing recorder if one was
    // really started. A spawn that best-effort failed takes the `not_started`
    // arm, whose manifest value is a `none:` — correct, and not what this half
    // is asserting.
    let log = strip_ansi(&read_file(&stderr_path));
    assert!(
        log.contains(r#"window="own_recorder""#),
        "this arm needs a run that really started its window recorder:\n{log}"
    );
    let manifest = live_run_manifest(&home);
    let standing = manifest["state_ring_consumer"].as_str().unwrap_or_else(|| {
        panic!(
            "a run must record its window-recorder decision where a later attach can read it \
             — a log line is not readable by another process: {manifest}"
        )
    });
    assert_eq!(
        standing, "standing",
        "a run holding a standing recorder must say STANDING, or every attach lands on the \
         UNKNOWN arm and the refusal ships inert"
    );
    stop_run(&mut run, &stderr_path);

    // (b) The kill switch: no recorder is started, so nothing is draining the
    //     state rings and an attach must be free to sweep them. A `none:` with
    //     the CAUSE, never a flag, by design.
    let tmp_off = tempfile::tempdir().expect("tempdir");
    let root_off = tmp_off.path();
    let prefix_off = unique_prefix("d4writeoff");
    build_workspace(root_off, &prefix_off);
    let home_off = root_off.join("home");
    let flashbacks_off = root_off.join("flashbacks");
    std::fs::create_dir_all(&home_off).unwrap();
    std::fs::create_dir_all(&flashbacks_off).unwrap();

    let (mut run_off, stderr_off) = spawn_run(
        root_off,
        &home_off,
        &flashbacks_off,
        &[],
        &[("CERULION_FLASHBACK", "off")],
    );
    wait_for_log_line(&mut run_off.0, &stderr_off, FLASHBACK_DECISION);
    let log_off = strip_ansi(&read_file(&stderr_off));
    assert!(
        log_off.contains(r#"window="switched_off""#),
        "this half needs the kill switch to have really taken the recorder off:\n{log_off}"
    );
    let manifest_off = live_run_manifest(&home_off);
    let none = manifest_off["state_ring_consumer"]
        .as_str()
        .unwrap_or_else(|| panic!("the switched-off run states its decision too: {manifest_off}"));
    assert!(
        none.starts_with("none: "),
        "a run with no standing consumer must say so positively — an absent key is UNKNOWN, \
         which is a different (and weaker) answer: {none}"
    );
    assert!(
        none.contains("switched off"),
        "…and name the CAUSE, so an operator reading a bag months later knows why: {none}"
    );
    assert!(
        !none.contains("--no-rings") && !none.contains("--record"),
        "project rule: an absence names the cause, never another verb's flag: {none}"
    );
    stop_run(&mut run_off, &stderr_off);
}

/// The PERTURBED twin of `ticker`: the same `#[cerulion_node(period_ms = 50)]`
/// shape and port, publishing `cmd.x = 1000.0` where the original publishes
/// `0.0`, the CI stand-in for "rebuild the node with a changed constant".
const PERTURBED_FIXTURE: &str = "test_node_macro_period_perturbed_cdylib";

/// How many run+capture attempts arm 6 makes to obtain a capture its oracle can
/// judge, before failing loudly.
///
/// TWO preconditions are the desk's rather than the candidate's, and both are
/// retried here rather than weakened:
///
/// * LOSS-FREE. A window tap that overflowed under desk load drops frames the
///   re-execution then reproduces, which `--verify` reports as a divergence.
/// * MID-RUN. The capture must begin past step 0, or `plan_restore` answers
///   `FromStart` and the arm never reaches the admission it exists to pin. That
///   is a race with the window's head trim, not a property of the code.
///
/// A fresh attempt, never a weaker oracle (the `--record` siblings do the same).
const CLEAN_CAPTURE_ATTEMPTS: usize = 3;

/// How many ticker frames arm 6 waits out before taking its capture.
///
/// [`WORKER_STEPPED_FRAMES`] proves the worker is PAST its first step, which is
/// all arm 3's attach needs. This arm needs the run FURTHER along: the rolling
/// window must hold a deep enough suffix that its head is genuinely trimmed, or
/// the capture's first rank-0 boundary is step 0. A CONDITION rather than a
/// sleep, for the usual reason and in the usual direction — a fixed span loses
/// on a starved runner, which sleeps it out having executed almost nothing and
/// inverts the arm's precondition. The fixture's period is 50 ms, so this is
/// ~2 s of a healthy run.
const FREE_RUN_MID_RUN_FRAMES: usize = 40;

/// The capture's RECORDER health (`CAPTURE_RECORDER_HEALTH_ATTACHMENT`, the
/// run-cumulative document, so an UPPER bound on what this window lost), summed
/// over this run's two graph topics. `None` means the attachment is absent,
/// which on a finalized capture is a harness failure rather than health.
fn capture_graph_topic_loss(bag: &Path, prefix: &str) -> Option<u64> {
    let reader = cerulion_bag::BagReader::open(bag).expect("open the capture");
    let att = reader
        .attachment(CAPTURE_RECORDER_HEALTH_ATTACHMENT)
        .expect("read attachments")?;
    let raw = String::from_utf8_lossy(&att.data).into_owned();
    let v: serde_json::Value = serde_json::from_str(&raw).unwrap_or_else(|e| {
        panic!("the capture's recorder health is not valid JSON ({e}):\n{raw}")
    });
    let topics = v["topics"].as_object().unwrap_or_else(|| {
        panic!(
            "the capture's recorder health carries no `topics` object; the per-topic \
             frames_lost term cannot be read, so this guard would report health it never \
             looked for:\n{raw}"
        )
    });
    Some(
        ["ticker/cmd", "relay/cmd"]
            .iter()
            .map(|suffix| {
                let topic = format!("/{prefix}/{suffix}");
                // PRESENT, not merely non-lossy. A topic the health document
                // does not carry contributes 0 to the gate this feeds, so a
                // recorder that tapped ONE of the two graph topics would read
                // as a loss-free capture of both — the gate's whole job is to
                // refuse a capture the re-execution will out-produce.
                let health = topics.get(&topic).unwrap_or_else(|| {
                    panic!(
                        "the capture's recorder health carries no entry for `{topic}`, so the \
                         loss gate would pass it by DEFAULT. It accounts for {:?}:\n{raw}",
                        topics.keys().collect::<Vec<_>>()
                    )
                });
                health["frames_lost"].as_u64().unwrap_or_else(|| {
                    panic!("`{topic}` carries no readable `frames_lost`: {health}")
                })
            })
            .sum(),
    )
}

/// **ARM 6: THE ONE-RANK FREE-RUN LOOP (the mid-run resume).**
///
/// The SAME plain 1-group run as arm 1, executed under
/// `CERULION_EXECUTION_MODE=free_run`, set EXPLICITLY here because the
/// free-run DEFAULT is not on `main` yet (that flip rebases onto this change);
/// on `main` the env opt-in is what selects the free-run supervisor path for a
/// `process_groups` graph. What the capture then is: `coordination: free_run`,
/// ONE worker rank, a recording that begins MID-RUN (the capture is taken after
/// the worker has stepped, and the window trims the head), and a complete
/// anchor at `S`: exactly the one-worker-rank bag `resolve_resume` admits by rank count
/// (`FreeRunResumeUnsupported`) while the capture's own manifest claimed
/// `resimmable: true`. For one worker rank the three assumptions that refusal
/// named hold trivially, so the bag now takes the ordinary resume.
///
/// The claims, in the order the sibling arms settled on: the recorded frames
/// match the fixture HAND ORACLE; the capture is genuinely free-run AND
/// genuinely mid-run (a lockstep or from-start capture would pass the rest of
/// this arm without exercising the admission) AND claims `resimmable: true`;
/// then `bag play --resim all --verify` exits 0 TWICE with byte-identical
/// `--report` JSON whose `resume` block names the anchor step the capture's own
/// first boundary implies (the restore, the seed, the clock placement and the
/// prefix skip are functions of the recording); and (the anti-tautology) the
/// SAME capture re-executed against the PERTURBED ticker exits 1 with a
/// `FRAME-CONTENT DIVERGENCE` naming the ticker's topic.
///
/// `--verify` IS driven here, unlike arm 1, and arm 1's reason for not driving
/// it does not apply: that arm's capture may begin at step 0 with a lossy head,
/// so the re-execution produces frames the window dropped; this arm's capture
/// RESUMES from its anchor, so the comparison begins where the recording does.
/// What can still break it is a tap overflow INSIDE the window on a loaded
/// desk, which is why the capture is gated on the recorder's own loss count and
/// retried rather than the oracle loosened (see `CLEAN_CAPTURE_ATTEMPTS`).
///
/// TWO mutants, both RUN: at the base tree the same arm exits 2 at leg 3 with
/// the refusal naming the free-run stamp and the mid-run first boundary; at
/// the admission WITHOUT the worker's clock fix (the free-run rank still on
/// the `RealClock` "live arm"), it exits 2 at leg 3 with check 3 naming the
/// first admitted ticker frame as matching no boundary target, the kept
/// capture behind that verdict carried ticker frames at target+42..84 us and
/// relay frames at target+154..222 us on every step, and its worker log said
/// `build_path=FreeRunLive`. This arm is therefore the real-binary oracle for
/// BOTH halves of the change: the engine's admission and the worker's clock.
///
/// Prerequisite beyond the file's: `cargo build -p test_node_macro_period_perturbed_cdylib`.
#[test]
#[serial]
fn a_free_run_one_rank_capture_resims_and_verifies_byte_exact_and_catches_a_changed_constant() {
    let mut last_retry = String::new();
    for attempt in 1..=CLEAN_CAPTURE_ATTEMPTS {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let prefix = unique_prefix("plainfr");
        build_workspace(root, &prefix);
        let home = root.join("home");
        let flashbacks = root.join("flashbacks");
        std::fs::create_dir_all(&home).unwrap();
        std::fs::create_dir_all(&flashbacks).unwrap();

        // ------------------------------------------------------------ leg 1
        // The run, opted into free-run EXPLICITLY (the flip is not on main).
        let (mut run, stderr_path) = spawn_run(
            root,
            &home,
            &flashbacks,
            &[],
            &[("CERULION_EXECUTION_MODE", "free_run")],
        );
        wait_for_log_line(&mut run.0, &stderr_path, WINDOW_HELD);
        // THE MID-RUN RENDEZVOUS (arm 3's, deepened): a capture whose trace
        // begins at step 0 needs no anchor and would not exercise the
        // admission, so this waits on the CONDITION that the run is well past
        // its first step rather than sleeping out a span that a loaded desk
        // would spend doing nothing.
        await_the_worker_has_published(&format!("/{prefix}/ticker/cmd"), FREE_RUN_MID_RUN_FRAMES);
        // run.json's `gating` label follows the SAME predicate the
        // worker keys its clock discipline on ("this run mints trace rings"),
        // so a plain free-run run reads `recorded_wall` -- never `wall`, the
        // read-only RealClock arm that is now `--no-rings` only. Read while the
        // run is LIVE (`live_run_manifest`: the directory goes on exit). This
        // is the one place the supervisor's call site is observable, so it is
        // what kills a call site that hands the classifier `--record`.
        let run_json = live_run_manifest(&home);
        assert_eq!(
            run_json["gating"],
            serde_json::json!("recorded_wall"),
            "a plain free-run run mints its trace rings, so every rank runs the controlled \
             wall-following clock and run.json must say so: {run_json}"
        );

        // ------------------------------------------------------------ leg 2
        // CAPTURE, through the operator's own verb.
        let mut flash = ChildGuard::single_process(
            Command::new(env!("CARGO_BIN_EXE_cerulion"))
                .args(["flashback", "--note", "one-rank free-run probe"])
                .current_dir(root)
                .env_remove("CARGO_TARGET_DIR")
                .env("CERULION_NETWORK", "off")
                .env("CERULION_HOME", &home)
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
        let flash_status = flash
            .wait_bounded(CAPTURE_COMPLETES)
            .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
        assert!(
            flash_status.success(),
            "a free-run `graph run` holds a rolling window too, so `cerulion flashback` must \
             capture: {flash_status:?}\nstdout:\n{}\nstderr:\n{}\nrun log:\n{}",
            read_file(&root.join("flash.stdout")),
            read_file(&root.join("flash.stderr")),
            read_file(&stderr_path)
        );
        let captures = mcaps(&flashbacks);
        assert_eq!(
            captures.len(),
            1,
            "exactly one capture in this run's own directory, got {captures:?}"
        );
        let capture = captures[0].clone();
        stop_run(&mut run, &stderr_path);

        // ----------------------------------------------- the loss-free gate
        let loss = capture_graph_topic_loss(&capture, &prefix).unwrap_or_else(|| {
            panic!("a finalized capture carries `{CAPTURE_RECORDER_HEALTH_ATTACHMENT}`")
        });
        if loss > 0 {
            last_retry = format!(
                "attempt {attempt}: the recorder reports {loss} lost frame(s) on this run's \
                 topics: a window tap overflowed (loaded desk?), so the re-execution would \
                 reproduce frames the capture does not hold"
            );
            eprintln!("{last_retry}");
            continue;
        }

        // ------------------------------------------- the oracle, then the CLAIM
        let foreign = assert_the_capture_accounts_for_every_topic_it_holds(&capture, &prefix);
        // The helper asserts each graph topic carries frames, so a total here
        // would restate what it already refused to return without.
        assert_frames_match_the_fixture_oracle(
            &capture,
            &prefix,
            "the free-run capture",
            Gating::FreeRun,
        );

        let reader = cerulion_bag::BagReader::open(&capture).expect("open the capture");
        let recorder: serde_json::Value = serde_json::from_slice(
            &reader
                .attachment("__cerulion/recorder.json")
                .expect("read the recorder identity")
                .expect("a capture carries the recorder identity")
                .data,
        )
        .expect("the recorder identity is valid JSON");
        assert_eq!(
            recorder["coordination"],
            serde_json::json!("free_run"),
            "this arm is about the FREE-RUN capture; a lockstep stamp here means the env \
             opt-in did not reach the supervisor: {recorder}"
        );
        let first = first_rank0_boundary_step(&capture)
            .expect("a capture with a trace carries a rank-0 STEP_BOUNDARY");
        if first == 0 {
            // A RACE with the window's head trim, exactly like the loss gate
            // above, so it is retried the same way: a from-start capture needs
            // no anchor and would not exercise the admission, but nothing about
            // the code under test made it come out that way.
            last_retry = format!(
                "attempt {attempt}: the capture's first rank-0 STEP_BOUNDARY is step 0, so it \
                 begins FROM START and `plan_restore` would answer `FromStart` — the admission \
                 under test is never reached"
            );
            eprintln!("{last_retry}");
            continue;
        }
        let manifest: serde_json::Value = serde_json::from_slice(
            &reader
                .attachment("__cerulion/flashback.json")
                .expect("read the capture manifest")
                .expect("a capture carries its own manifest")
                .data,
        )
        .expect("the capture manifest is valid JSON");
        assert_eq!(
            manifest["anchor"]["resimmable"],
            serde_json::json!(true),
            "the capture judge's claim, which the resim below must honour: {manifest}"
        );
        drop(reader);

        // ------------------------------------------------------------ leg 3
        // RESIM with the VERDICT, twice: exit 0 and ONE report.
        let report_a = root.join("resim_a.json");
        let report_b = root.join("resim_b.json");
        let (code, resim_err) = resim(
            root,
            &capture,
            &["--verify", "--report", report_a.to_str().unwrap()],
            "resim_a",
        );
        assert_eq!(
            code,
            Some(0),
            "THE ADMISSION: a one-rank free-run capture beginning mid-run (first rank-0 \
             boundary {first}) must resume and verify byte-exact. Exit 2 naming \
             `coordination: free_run` is the refusal of every free-run mid-run bag itself; exit 1 is a \
             frame the re-execution produced that the capture does not hold (foreign \
             topics: {foreign:?}).\nstderr:\n{resim_err}"
        );
        assert!(
            executed_steps(&resim_err) > 0,
            "the resim must actually re-execute the capture's suffix:\n{resim_err}"
        );
        let (code_b, resim_err_b) = resim(
            root,
            &capture,
            &["--verify", "--report", report_b.to_str().unwrap()],
            "resim_b",
        );
        assert_eq!(
            code_b,
            Some(0),
            "the second resim of the same capture:\n{resim_err_b}"
        );
        let report_json = read_file(&report_a);
        assert_eq!(
            report_json,
            read_file(&report_b),
            "two resims of one capture must produce ONE report"
        );
        let report: serde_json::Value =
            serde_json::from_str(&report_json).expect("the report is valid JSON");
        assert_eq!(report["passed"], serde_json::json!(true), "{report}");
        // `passed` is a claim about the comparisons that RAN. A report that
        // compared nothing passes too, so exit 0 says nothing until this does.
        assert!(
            report["topics_checked"].as_u64().unwrap_or(0) > 0,
            "the verify must have COMPARED a topic; `passed` over zero comparisons is the \
             tautology this arm exists to avoid: {report}"
        );
        assert_eq!(
            report["coordination"]["mode"],
            serde_json::json!("free_run"),
            "the contract applied is the one the capture stamped: {report}"
        );
        assert_eq!(
            report["resume"]["first_replay_step"],
            serde_json::json!(first),
            "the resume begins at the capture's first recorded boundary: {report}"
        );
        assert_eq!(
            report["resume"]["anchor_step"],
            serde_json::json!(first - 1),
            "…from the anchor taken at the step before it: {report}"
        );

        // ------------------------------------------------------------ leg 4
        // The ANTI-TAUTOLOGY: the same capture against a candidate whose ONE
        // constant changed is a data violation naming the ticker's topic.
        std::fs::copy(
            fixture_cdylib(PERTURBED_FIXTURE),
            root.join("target/debug").join(dylib_file("ticker")),
        )
        .expect("overwrite the ticker cdylib with the perturbed twin");
        let (code_p, resim_err_p) = resim(root, &capture, &["--verify"], "resim_perturbed");
        let resim_err_p = strip_ansi(&resim_err_p);
        assert_eq!(
            code_p,
            Some(1),
            "the SAME capture against a perturbed candidate must exit 1 (a data violation, \
             never 2 and never 6):\n{resim_err_p}"
        );
        let ticker_topic = format!("/{prefix}/ticker/cmd");
        assert!(
            resim_err_p.contains("FRAME-CONTENT DIVERGENCE") && resim_err_p.contains(&ticker_topic),
            "the verdict names the data-divergence class and the ticker's topic:\n{resim_err_p}"
        );
        return;
    }
    panic!(
        "could not obtain a LOSS-FREE, MID-RUN capture in {CLEAN_CAPTURE_ATTEMPTS} attempts; \
         REFUSING to weaken the `--verify` exit-0 oracle. Last: {last_retry}"
    );
}
