// SPDX-License-Identifier: AGPL-3.0-only
//! END-TO-END acceptance for MULTI-PROCESS `--record` over the
//! REAL binary — spawn `cerulion graph run mpdemo --record=DIR` against a
//! hand-built tempdir workspace whose 3-node chain (`ticker`(Period 50ms) →
//! `relay`(data-trigger) → `sink`(data-trigger)) is declared in TWO
//! `process_groups` (`p0:[ticker,relay]` rank 0, `p1:[sink]` rank 1), SIGINT
//! the SUPERVISOR (directed: macOS has no setsid tooling, so the supervisor
//! itself fans the SIGINT out to its workers), and pin the multi-process bag
//! contracts:
//!
//! 1. **mp happy path** — exit 0; ONE finalized bag; per-rank manifests
//!    (`rank0` = `[ticker,relay]`, `rank1` = `[sink]`, the `rank4294967295`
//!    departure sentinel = `[]`); every trace record's `reserved` ∈ {0,1} with
//!    NO departure records (happy path); per-rank STEP-BOUNDARY streams
//!    0-based gap-free + non-decreasing with fire ≤ boundary per step; the
//!    cross-rank boundary AGREEMENT (equal `fire_time_ns` per shared step —
//!    the handed-quantum lockstep contract the replay merge relies on);
//!    per-rank FIRE records resolve through THEIR OWN rank's manifest to
//!    exactly that group's nodes; ≥1 recorded frame (delivery accounting).
//!    Plus two bring-up pins: the ORDERING (bagd-armed breadcrumb
//!    strictly before the GO breadcrumb — the step-0 capture contract) and
//!    the ring sweep (no ring name survives the run).
//! 2. **`--single-process --record` dispatch pin** — the SAME pg graph takes
//!    the single-process monolith path: ONLY the rank-0 manifest (full-graph node
//!    ids), no rank-1, no departure sentinel, every `reserved` == 0.
//! 3. **`--time-source virtual` still rejected** — subprocess-level validator
//!    pin (exit nonzero naming the live-clock requirement).
//! 4. **Departure path** — SIGKILL the rank-1 worker under
//!    `--peer-loss continue`: exit 0 (degraded), the bag carries kind-2
//!    departure record(s) with `reserved` == u32::MAX + `node_idx` == 1, the
//!    survivor out-steps the dead rank, the sentinel manifest stays `[]`, and
//!    EVERY ring name (the SIGKILLed worker's included) is swept.
//! 5. **Killed bagd** — SIGKILL the bagd grandchild
//!    mid-run: NONZERO exit naming the INCOMPLETE bag (the single-process
//!    path's mapping, on the mp path).
//!
//! Harness follows `graph_record_e2e_test.rs` (tempdir workspace, PREBUILT
//! fixture cdylibs — no in-test cargo build, redirected child logs, bounded
//! waits, `BagdGuard` grandchild reaping, `#[serial]`, unique prefixes).
//! Prerequisites (the repo's fixture pattern — the tests PANIC with the
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]` (NOT linux-only): the multi-process supervisor is
//! REAL on macOS too, so this file runs on macOS and on Linux.

#![cfg(unix)]

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use cerulion_core::trace_ring::{
    unpack_read_outcome_meta, TraceRingRecord, RECORD_TYPE_DEPARTURE, RECORD_TYPE_FIRE,
    RECORD_TYPE_READ_OUTCOME, RECORD_TYPE_STEP_BOUNDARY,
};
use serial_test::serial;

// The reusable record-harness helpers (`build_mp_workspace`, `spawn_mp_record`,
// `ChildGuard`/`BagdGuard`, the manifest/trace readers, the ring-sweep assert,
// the `DEPARTURE_SENTINEL` const, …) live in the shared `mp_support` module —
// shared so the mp record→replay exit-0 e2e can drive
// the SAME recipe without copy-paste.
mod mp_support;
use mp_support::*;

/// The mp `--record` acceptance BODY (2 process groups → one bag whose per-rank
/// rank-stamped streams satisfy the multi-process contracts), parameterized by the
/// shutdown `signal` delivered to the SUPERVISOR so the SIGINT and
/// SIGTERM twins assert the IDENTICAL postconditions without copy-paste drift.
/// `prefix` must be unique per twin (the data plane runs on the DEFAULT
/// namespace); `sig_label` is the human name for the assert messages.
/// Which execution mode the contracts arm runs the deployment
/// under. `Lockstep` is the default; `FreeRun` sets the
/// `CERULION_EXECUTION_MODE=free_run` opt-in on the spawned supervisor.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RecordMode {
    Lockstep,
    FreeRun,
}

impl RecordMode {
    /// The child environment that PINS this mode — both arms explicit,
    /// because the spawn helper forwards the parent's
    /// environment, so a lockstep arm that passed nothing would run free-run
    /// under a developer's `CERULION_EXECUTION_MODE=free_run` and test the
    /// wrong contract.
    ///
    /// `RUST_LOG` raises the engine to `debug`: the worker's clean-shutdown
    /// cohort lines ("worker left the barrier cohort", "no barrier cohort to
    /// leave") that the contracts arm counts are lifecycle bookkeeping and
    /// ride `debug!`, filtered at the spawn helper's `cerulion_cli_engine=info`
    /// default. This arm reads them as EVIDENCE that every worker took its
    /// clean-exit path, so it asks for them explicitly (a dev-profile binary,
    /// where `debug!` is compiled in).
    fn envs(self) -> &'static [(&'static str, &'static str)] {
        const ENGINE_DEBUG: (&str, &str) = (
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=debug,cerulion_bagd=info",
        );
        match self {
            Self::Lockstep => &[("CERULION_EXECUTION_MODE", "lockstep"), ENGINE_DEBUG],
            Self::FreeRun => &[("CERULION_EXECUTION_MODE", "free_run"), ENGINE_DEBUG],
        }
    }
}

fn mp_record_contracts_under_signal(
    prefix: &str,
    signal: libc::c_int,
    sig_label: &str,
    mode: RecordMode,
) {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), prefix);
    // A free-run recording rank places its gating clock at a
    // `real_ns()` epoch read AFTER GO — so every rank's first boundary must
    // stamp a time at or after THIS read, taken before the supervisor even
    // spawns (same machine, same boot-monotonic domain). A lockstep bag's
    // first boundary is the handed quantum (5e7 ns — this fixture's 50 ms
    // ticker is its only timing source), which cannot pass it.
    let pre_spawn_ns = cerulion_core::clock::real_ns();
    let (mut guard, stdout_path, stderr_path) =
        spawn_mp_record_with_env(tmp.path(), &[], mode.envs());
    let _bagd_guard = BagdGuard::arm();

    // Planning build + 2 worker spawns + bagd handshake precede the bag file —
    // generous bound for slow CI VMs.
    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag (mp handshake failed?)\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    // Record a healthy window — WAITED FOR, not slept. The assertions below rest
    // on recorded evidence (both ranks' boundary streams, a shared step, frames
    // on the ticker's topic), so the window ends when that evidence is in the
    // bag. `RECORDED_WINDOW_BOUNDARIES` keeps the free-run delta-variance arm a
    // large sample; the frame floor is the `frames > 0` accounting assert's own
    // premise. A mid-run read sees only flushed chunks, so this is a lower
    // bound on the finalized bag — late, never early.
    let ticker_topic = format!("/{prefix}/ticker/cmd");
    wait_for_bag_state(
        &bag,
        "both ranks' boundary streams and the ticker's frames",
        RECORDED_WINDOW_TIMEOUT,
        |snap| {
            snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES
                && snap.boundaries_for_rank(1) >= RECORDED_WINDOW_BOUNDARIES
                && snap.frames_on(&ticker_topic) > 0
        },
    );

    // Directed shutdown signal to the SUPERVISOR (SIGINT = the production
    // Ctrl-C; SIGTERM = systemd stop / `kill <pid>`). Both flip the
    // supervisor's `running` (SIGTERM via the ctrlc `termination` feature) and
    // the supervisor fans SIGINT out to the workers. Teardown: workers drain → JOIN
    // loop reaps → bagd SIGTERM final-drain → finalize.
    send_signal(guard.id(), signal);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .unwrap_or_else(|| panic!("supervisor did not exit after {sig_label}"));
    // The upper bound of the epoch domain — every epoch was read
    // while the deployment was alive, so it sits at or before THIS read.
    let post_exit_ns = cerulion_core::clock::real_ns();
    assert!(
        status.success(),
        "mp recorded run must exit 0 on {sig_label}, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // The bring-up ORDERING pin: bagd's taps-ready handshake
    // must complete BEFORE the GO sentinel releases the workers to step 0 (so
    // step-0 publishes AND step-0 trace records are guaranteed drained into
    // the bag). Both breadcrumbs are `tracing::info!` supervisor-process log
    // lines, which go to STDERR (init_logging writes there so a command's
    // stdout stays clean data), in the SAME stderr file, so byte offsets order
    // the events — a bring-up reorder regression flips the offsets and fails here.
    let log = read_file(&stderr_path);
    // The deployment ran under the mode it was asked for. The
    // worker breadcrumb is the anti-vacuity premise for the per-worker pins
    // below (workers inherit the supervisor's redirected stderr).
    assert_eq!(
        log.matches("worker build path resolved").count(),
        2,
        "both workers log their resolved build path to the shared stderr; log was:\n{log}"
    );
    match mode {
        RecordMode::Lockstep => {
            assert!(
                !log.contains("free-run deployment: no shared barrier is created")
                    && !log.contains("no barrier cohort to leave"),
                "a lockstep deployment creates the barrier and every worker leaves the cohort"
            );
            assert_eq!(
                log.matches("worker left the barrier cohort").count(),
                2,
                "BOTH lockstep workers leave the cohort on clean exit (a worker that skipped \
                 `leave_barrier_cohort` would not show in the trace or the bag); log was:\n{log}"
            );
        }
        RecordMode::FreeRun => {
            assert_eq!(
                log.matches("free-run deployment: no shared barrier is created")
                    .count(),
                1,
                "the supervisor creates no barrier for a free-run deployment; log was:\n{log}"
            );
            assert_eq!(
                log.matches("no barrier cohort to leave").count(),
                2,
                "both free-run workers exit with no cohort to leave; log was:\n{log}"
            );
            assert_eq!(
                log.matches("worker left the barrier cohort").count(),
                0,
                "no free-run worker ever leaves a cohort"
            );
            assert_eq!(
                log.matches("gating clock epoch armed for the live loop")
                    .count(),
                2,
                "both free-run recording workers arm the shared epoch; log was:\n{log}"
            );
            assert_eq!(
                log.matches("gating clock placed at the shared real_ns() epoch")
                    .count(),
                2,
                "…and both place it at their live-loop anchor; log was:\n{log}"
            );
        }
    }
    let armed = log
        .find("multi-process recording armed")
        .unwrap_or_else(|| panic!("log must contain the bagd-armed breadcrumb; log was:\n{log}"));
    let go = log
        .find("GO signaled; deployment live")
        .unwrap_or_else(|| panic!("log must contain the GO breadcrumb; log was:\n{log}"));
    assert!(
        armed < go,
        "bagd must be armed BEFORE GO (armed@{armed} < go@{go}) — the step-0 capture contract"
    );

    // The clean-path sweep: no ring name survives the run
    // (workers unlinked their own on clean exit; the sweeper's ENOENT arm is
    // silent; the departure ring dropped at finalize).
    assert_rings_swept(guard.id());

    // (a) ONE finalized bag.
    let bag2 = assert_single_bag(&recordings);
    assert_eq!(bag, bag2, "the bag observed mid-run is the final one");
    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the mp teardown must FINALIZE the bag, got {completeness:?}"
    );

    // The bag says which coordination contract it was recorded
    // under — the ONE resolved value, threaded to the supervisor's recording
    // start, never re-derived.
    let recorder_att = reader
        .attachment("__cerulion/recorder.json")
        .expect("read attachments")
        .expect("__cerulion/recorder.json attachment present");
    let recorder: serde_json::Value =
        serde_json::from_slice(&recorder_att.data).expect("recorder.json parses as JSON");
    let want_coordination = match mode {
        RecordMode::Lockstep => "lockstep",
        RecordMode::FreeRun => "free_run",
    };
    assert_eq!(
        recorder["coordination"], want_coordination,
        "the bag's coordination stamp follows the resolved execution mode: {recorder}"
    );

    // (b) per-rank manifests + the departure sentinel manifest.
    let (r0, ids0) = read_manifest(&reader, 0).expect("rank0 manifest present");
    assert_eq!(r0, 0);
    assert_eq!(
        ids0,
        vec!["ticker".to_string(), "relay".to_string()],
        "rank-0 manifest = p0's subgraph node ids in config order"
    );
    let (r1, ids1) = read_manifest(&reader, 1).expect("rank1 manifest present");
    assert_eq!(r1, 1);
    assert_eq!(
        ids1,
        vec!["sink".to_string()],
        "rank-1 manifest = p1's subgraph node ids"
    );
    let (rd, idsd) =
        read_manifest(&reader, DEPARTURE_SENTINEL).expect("departure sentinel manifest present");
    assert_eq!(rd, u64::from(DEPARTURE_SENTINEL));
    assert!(
        idsd.is_empty(),
        "the departure ring's manifest is EMPTY (node_idx carries a worker rank)"
    );

    // (c) every record's provenance ∈ {0, 1}; NO departures on the happy path.
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    assert!(!trace.is_empty(), "the mp bag must carry trace records");
    for r in &trace {
        assert!(
            r.reserved == 0 || r.reserved == 1,
            "happy path: every record's bagd-stamped rank must be 0 or 1, got {} \
             (u32::MAX would be departure provenance)",
            r.reserved
        );
        assert_ne!(
            r.record_type, RECORD_TYPE_DEPARTURE,
            "happy path: no worker died, so no DEPARTURE record may exist"
        );
    }
    let ranks = by_rank(&trace);
    assert_eq!(
        ranks.keys().copied().collect::<Vec<_>>(),
        vec![0, 1],
        "both workers' rings must reach the bag"
    );

    // (d) per-rank STEP-BOUNDARY streams: 0-based gap-free steps,
    // non-decreasing fire_time_ns, fire ≤ boundary per step per rank.
    // (e) cross-rank boundary AGREEMENT on shared steps (handed-quantum
    // lockstep — the machine-check the replay merge relies on).
    let mut boundary_times: BTreeMap<u32, BTreeMap<u64, u64>> = BTreeMap::new();
    for (&rank, records) in &ranks {
        let boundaries: Vec<&&TraceRingRecord> = records
            .iter()
            .filter(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
            .collect();
        assert!(
            !boundaries.is_empty(),
            "rank {rank} must carry step-boundary records"
        );
        for (i, b) in boundaries.iter().enumerate() {
            assert_eq!(
                b.step, i as u64,
                "rank {rank}: boundary steps must be 0-based and gap-free"
            );
        }
        for w in boundaries.windows(2) {
            assert!(
                w[0].fire_time_ns <= w[1].fire_time_ns,
                "rank {rank}: boundary stream must be non-decreasing in fire_time_ns"
            );
        }
        let times: BTreeMap<u64, u64> = boundaries
            .iter()
            .map(|b| (b.step, b.fire_time_ns))
            .collect();
        for r in records.iter().filter(|r| r.record_type == RECORD_TYPE_FIRE) {
            let b = times.get(&r.step).unwrap_or_else(|| {
                panic!(
                    "rank {rank}: fire at step {} has no boundary record",
                    r.step
                )
            });
            assert!(
                *b >= r.fire_time_ns,
                "rank {rank}: a step's boundary carries the ADVANCED clock — never \
                 before its fires"
            );
        }
        boundary_times.insert(rank, times);
    }
    match mode {
        RecordMode::Lockstep => {
            let (t0, t1) = (&boundary_times[&0], &boundary_times[&1]);
            let mut shared_steps = 0usize;
            for (step, time0) in t0 {
                if let Some(time1) = t1.get(step) {
                    assert_eq!(
                        time0, time1,
                        "step {step}: both ranks' boundary fire_time_ns must be EQUAL — the \
                         workers advance the SAME handed quantum in barrier lockstep \
                         (rank0={time0}, rank1={time1})"
                    );
                    shared_steps += 1;
                }
            }
            assert!(
                shared_steps > 0,
                "the two ranks must share at least one boundary step to prove lockstep"
            );
        }
        RecordMode::FreeRun => {
            // (e') Free-run: each rank records its OWN wall-faithful timeline
            // from the shared `real_ns()` epoch. Two discriminators against
            // the lockstep shape, both hand oracles: the first boundary sits
            // INSIDE the run's real_ns() window — at or after the read taken
            // before the spawn (a lockstep bag's is the handed quantum, 5e7 ns
            // for this fixture's 50 ms ticker) AND at or before the read taken
            // after the exit (a one-sided bound would also pass an
            // epoch from a wrong-but-larger domain, e.g. unix-ns) — and
            // consecutive boundary deltas are NOT all equal (under lockstep
            // every delta is EXACTLY the quantum; a wall-following clock
            // measures ns-resolution jitter that cannot coincide across a
            // whole run).
            for (&rank, times) in &boundary_times {
                let first = times.values().next().copied().unwrap_or(0);
                assert!(
                    pre_spawn_ns <= first && first <= post_exit_ns,
                    "rank {rank}: the first boundary ({first}) sits inside the run's real_ns() \
                     window [{pre_spawn_ns}, {post_exit_ns}] — the epoch placed it in the \
                     boot-monotonic domain, not at a quantum from zero and not in some other \
                     clock's domain"
                );
                let stamps: Vec<u64> = times.values().copied().collect();
                assert!(
                    stamps.len() >= 3,
                    "rank {rank}: at least three boundaries to compare deltas"
                );
                let deltas: Vec<u64> = stamps.windows(2).map(|w| w[1] - w[0]).collect();
                assert!(
                    deltas.iter().any(|d| *d != deltas[0]),
                    "rank {rank}: a wall-following clock's boundary deltas cannot all be equal \
                     (a lockstep clock's are exactly the quantum): {deltas:?}"
                );
            }
        }
    }

    // (f) per-rank FIRE records resolve through THEIR rank's manifest to
    // exactly that group's nodes.
    let manifests: BTreeMap<u32, Vec<String>> = [(0u32, ids0), (1u32, ids1)].into();
    for (&rank, records) in &ranks {
        let ids = &manifests[&rank];
        let mut fired: Vec<&str> = Vec::new();
        for r in records.iter().filter(|r| r.record_type == RECORD_TYPE_FIRE) {
            let name = ids
                .get(r.node_idx as usize)
                .unwrap_or_else(|| {
                    panic!(
                        "rank {rank}: fire node_idx {} out of range for manifest {ids:?}",
                        r.node_idx
                    )
                })
                .as_str();
            if !fired.contains(&name) {
                fired.push(name);
            }
        }
        assert!(
            !fired.is_empty(),
            "rank {rank} must record at least one fire (rank 1 firing proves \
             CROSS-GROUP data flow reached the sink)"
        );
        for name in &fired {
            assert!(
                ids.iter().any(|i| i == name),
                "rank {rank}: fired node {name} must belong to its own group {ids:?}"
            );
        }
    }

    // (g) delivery accounting: the recorded topic carries real frames.
    let frames = msgs.iter().filter(|m| m.topic == ticker_topic).count();
    assert!(
        frames > 0,
        "the bag must contain the ticker's published frames (got {frames}; \
         delivery accounting — a recording with zero frames is a silent failure)"
    );

    // (h) The REAL-BAG read-log inert-shipping pin:
    // arming the capture through the production worker path must land kind-6
    // records in the finalized bag (a silent arming regression keeps every
    // other contract green — this is the assertion that goes red).
    let kind6: Vec<&TraceRingRecord> = trace
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .collect();
    assert!(
        !kind6.is_empty(),
        "the mp bag must carry at least one kind-6 READ-OUTCOME record \
         (recording arms the read log; its absence is a silent arming regression)"
    );
    // Each kind-6 record's bagd-stamped rank resolves through THAT rank's
    // manifest: node_idx into the rank's node table, input_idx into that
    // node's `inputs` list (read off the raw manifest attachment JSON).
    let raw_manifest = |rank: u32| -> serde_json::Value {
        let att = reader
            .attachment(&format!("__cerulion/trace_manifest_rank{rank}.json"))
            .expect("read attachments")
            .expect("manifest attachment present");
        serde_json::from_slice(&att.data).expect("manifest json parses")
    };
    for r in &kind6 {
        let rank = r.rank();
        assert!(
            rank == 0 || rank == 1,
            "kind-6 rank stamp must be a worker rank, got {rank}"
        );
        let manifest = raw_manifest(rank);
        let node_ids: Vec<&str> = manifest["node_ids"]
            .as_array()
            .expect("node_ids array")
            .iter()
            .map(|s| s.as_str().expect("node id string"))
            .collect();
        let node = node_ids.get(r.node_idx as usize).unwrap_or_else(|| {
            panic!(
                "rank {rank}: kind-6 node_idx {} out of range for manifest {node_ids:?}",
                r.node_idx
            )
        });
        let (input_idx, _kind) = unpack_read_outcome_meta(r.global_level);
        let inputs = manifest["inputs"][node]
            .as_array()
            .unwrap_or_else(|| panic!("manifest `inputs` carries node {node}"));
        assert!(
            (input_idx as usize) < inputs.len(),
            "rank {rank}: kind-6 input_idx {input_idx} must resolve through node \
             {node}'s input list {inputs:?}"
        );
    }
    // The manifest attachment JSON carries the `inputs` key with a NON-empty
    // list for the consumer nodes (relay on rank 0, sink on rank 1) — the
    // offline resolver bagd stamps from the ring's input section.
    for (rank, consumer) in [(0u32, "relay"), (1u32, "sink")] {
        let manifest = raw_manifest(rank);
        assert_eq!(
            manifest["inputs"][consumer],
            serde_json::json!(["trigger_in"]),
            "rank-{rank} manifest `inputs` must name {consumer}'s wired input"
        );
    }
}

/// (1b) The SAME contracts under the FREE-RUN opt-in
/// (`CERULION_EXECUTION_MODE=free_run`) — behavioural evidence for
/// the worker's free-run branch through a REAL supervisor: no barrier created,
/// both workers arm and place the shared epoch, exit with no cohort to leave,
/// the bag's `coordination` stamp reads `free_run`, and each rank's boundary
/// stream is a wall-faithful timeline from the epoch (not a quantum from zero) —
/// while every lockstep-independent contract (one bag, manifests, gap-free
/// steps, fires resolving through their rank's manifest, DELIVERY across the
/// split, kind-6 read-outcome records) holds unchanged.
#[test]
#[serial]
fn mp_record_free_run_opt_in_records_per_rank_wall_streams() {
    mp_record_contracts_under_signal("mpfr", libc::SIGINT, "SIGINT", RecordMode::FreeRun);
}

/// (1) The mp `--record` acceptance under a DIRECTED SIGINT to the supervisor
/// (the interactive Ctrl-C path; the supervisor fans it out to the workers).
#[test]
#[serial]
fn mp_record_e2e_one_bag_rank_stamped_lockstep_and_manifests() {
    mp_record_contracts_under_signal("mpreca", libc::SIGINT, "SIGINT", RecordMode::Lockstep);
}

/// SIGTERM twin: the IDENTICAL multi-process contracts under a DIRECTED SIGTERM to
/// the supervisor (systemd stop / `kill <pid>`). SIGTERM rides the ctrlc
/// `termination` feature, so the supervisor's `running` flips exactly as on
/// Ctrl-C, it fans SIGINT out to the workers, and bagd receives the supervisor's
/// TERM fan-out and FINALIZES the bag EXACTLY ONCE — the double-finalize
/// guard is `assert_single_bag` + `is_finalized` inside the shared body (exactly
/// one `.mcap`, finalized).
#[test]
#[serial]
fn mp_record_e2e_one_bag_rank_stamped_lockstep_and_manifests_on_sigterm() {
    mp_record_contracts_under_signal("mprecterm", libc::SIGTERM, "SIGTERM", RecordMode::Lockstep);
}

/// (2) The dispatch-guard pin, e2e level: `--single-process --record` on the
/// SAME `process_groups:` graph takes the SINGLE-PROCESS monolith path — one rank-0
/// ring over the FULL graph, no worker rings, no departure sentinel.
#[test]
#[serial]
fn mp_graph_single_process_record_takes_stage1_path() {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), "mprecb");
    let (mut guard, stdout_path, stderr_path) = spawn_mp_record(tmp.path(), &["--single-process"]);
    let _bagd_guard = BagdGuard::arm();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(60)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag (recorder handshake failed?)\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    // Wait for rank-0's recorded boundary stream rather than sleeping a window:
    // the single-process contracts below are read off the trace, and this is the same
    // evidence.
    wait_for_bag_state(
        &bag,
        "the single-process rank-0 boundary stream",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES,
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "single-process recorded run must exit 0, got {status:?}\nstderr:\n{}",
        read_file(&stderr_path)
    );

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "bag finalized");

    // Single-process shape: ONE rank-0 manifest carrying the FULL graph's node ids
    // (config order) — no per-worker manifests, no departure sentinel.
    let (r0, ids0) = read_manifest(&reader, 0).expect("rank0 manifest present");
    assert_eq!(r0, 0);
    assert_eq!(
        ids0,
        vec![
            "ticker".to_string(),
            "relay".to_string(),
            "sink".to_string()
        ],
        "single-process manifest = the FULL graph's node ids (monolith recording)"
    );
    assert!(
        read_manifest(&reader, 1).is_none(),
        "--single-process must NOT produce a rank-1 (worker) manifest"
    );
    assert!(
        read_manifest(&reader, DEPARTURE_SENTINEL).is_none(),
        "--single-process must NOT produce the departure sentinel manifest"
    );

    // Every record is rank-0 provenance (the single-process ring's header rank).
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    assert!(!trace.is_empty(), "single-process trace records present");
    for r in &trace {
        assert_eq!(r.reserved, 0, "single-process records all carry rank 0");
        assert_ne!(r.record_type, RECORD_TYPE_DEPARTURE, "no departures");
    }
}

/// (3) Subprocess-level validator pin: `--record --time-source virtual` on the
/// pg graph is REJECTED (exit nonzero, message names the live-clock
/// requirement) — the validator accepts `process_groups` under
/// `--record` but keeps the clock arm.
#[test]
#[serial]
fn mp_graph_record_with_virtual_clock_rejected() {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), "mprecc");
    let (mut guard, _stdout_path, stderr_path) =
        spawn_mp_record(tmp.path(), &["--time-source", "virtual"]);
    let _bagd_guard = BagdGuard::arm();

    let status = guard
        .wait_bounded(Duration::from_secs(30))
        .expect("the validator rejection must exit quickly");
    assert!(
        !status.success(),
        "--record + --time-source virtual must be rejected, got {status:?}"
    );
    let stderr = read_file(&stderr_path);
    assert!(
        stderr.contains("requires the live clock"),
        "the rejection must name the live-clock requirement; stderr was:\n{stderr}"
    );
    assert!(
        !tmp.path().join("recordings").exists()
            || wait_for_bag(&tmp.path().join("recordings"), Duration::from_millis(1)).is_none(),
        "no bag may be created on the rejected run"
    );
}

/// (4) The DEPARTURE path, e2e (the no-inert-shipping
/// class): SIGKILL the rank-1 (sink) worker mid-run under
/// `--peer-loss continue` → the run continues DEGRADED and exits 0 on Ctrl-C,
/// and the bag carries the departure evidence:
/// - ≥1 `RECORD_TYPE_DEPARTURE` record, `reserved` == u32::MAX (the departure
///   ring's header rank, bagd-stamped) and `node_idx` == 1 (the killed
///   worker's plan rank);
/// - the SURVIVING rank-0 stream kept going past the kill — under LOCKSTEP its
///   boundary stream runs strictly longer than the dead rank's (the barrier
///   couples them, so rank 0's lead is the post-kill window); under FREE-RUN the
///   ranks are uncoupled and rank 1 may legitimately lead on step COUNT, so the
///   claim is anchored in TIME instead: ≥2 rank-0 boundaries stamped after the
///   `real_ns()` kill instant;
/// - the departure-sentinel manifest is still `[]`;
/// - the ring sweep: EVERY ring name (both workers' + the departure ring) is
///   unlinked after exit — the SIGKILLed worker's ring is exactly the leak
///   the supervisor sweep exists for.
///
/// Run under both execution modes, so the supervisor's
/// free-run peer-death arm (no barrier ⇒ no drop, no stall grace) is covered too.
/// The departure record and the ring sweep are mode-invariant; the SURVIVOR pin
/// is NOT — lockstep compares step counts across ranks (the barrier couples
/// them), free-run compares rank 0's boundary `fire_time_ns` against the kill
/// instant (the ranks are uncoupled and either may lead on steps). The stderr
/// says which cohort arm ran.
#[test]
#[serial]
fn mp_record_departure_lands_in_bag_under_peer_loss_continue() {
    departure_lands_in_bag("mprecd", RecordMode::Lockstep);
}

#[test]
#[serial]
fn mp_record_departure_lands_in_bag_under_peer_loss_continue_free_run() {
    departure_lands_in_bag("mprecdf", RecordMode::FreeRun);
}

fn departure_lands_in_bag(prefix: &str, mode: RecordMode) {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), prefix);
    let (mut guard, stdout_path, stderr_path) =
        spawn_mp_record_with_env(tmp.path(), &["--peer-loss", "continue"], mode.envs());
    let _bagd_guard = BagdGuard::arm();
    let sup_pid = guard.id();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag (mp handshake failed?)\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    // A healthy pre-kill window so BOTH ranks bank fires + boundaries — waited
    // for on the evidence itself, so the kill lands only once both ranks really
    // have a recorded stream to compare afterwards.
    wait_for_bag_state(
        &bag,
        "both ranks' pre-kill boundary streams",
        RECORDED_WINDOW_TIMEOUT,
        |snap| {
            snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES
                && snap.boundaries_for_rank(1) >= RECORDED_WINDOW_BOUNDARIES
        },
    );

    // SIGKILL the rank-1 (sink / group p1) worker — located as the
    // supervisor's direct child running `plan_p1.json` (the `mp_supervisor_box_test` pattern).
    let victim = worker_pid_for_group(sup_pid, "p1", Duration::from_secs(10))
        .expect("could not locate the p1 worker pid (pgrep -P <supervisor> -f plan_p1.json)");
    // THE KILL INSTANT, in the ranks' OWN time domain. Under free-run every rank
    // stamps its boundaries from the shared `real_ns()` epoch — this file already
    // asserts exactly that for BOTH ranks above, bounding each rank's first
    // boundary inside `[pre_spawn_ns, post_exit_ns]` — so a `real_ns()` read here
    // is directly comparable to a boundary's `fire_time_ns` out of the finalized
    // bag. Read BEFORE the signal, so "after the kill" can never be satisfied by
    // a boundary that was already banked.
    let kill_ns = cerulion_core::clock::real_ns();
    send_signal(victim, libc::SIGKILL);

    // Degraded window as an EVIDENCE WAIT, not a fixed sleep.
    //
    // A fixed `sleep(2500ms)` asserts a RACE: on a loaded machine the
    // survivor sometimes banks NO post-kill boundary inside the window and
    // the arm fails with `last0 == last1` — a TIE, measured at 100/100,
    // 216/216 and 5/5 across three back-to-back runs under that sleep. A
    // tie is the harness saying "I did not look long enough", and it reads
    // exactly like the regression this arm exists to catch.
    //
    // WHAT IS POLLED, and why not the obvious thing: polling
    // the growing bag's `scheduler_trace()` reads `rank0=0 rank1=0`
    // throughout — a partially-written bag does not serve its trace, so that
    // source can never observe progress and a wait on it is inert. What DOES
    // advance live is the bag FILE: `bagd` closes a chunk every
    // `CHUNK_TIME_FLOOR_MS` (1 s) or 4 MiB, so growth after the kill is direct
    // evidence that post-kill records were drained and written. Two distinct
    // growth observations are required, so a single in-flight chunk that was
    // already being written before the kill cannot satisfy it.
    let bag_len = || std::fs::metadata(&bag).map(|m| m.len()).unwrap_or(0);
    let len_at_kill = bag_len();
    let evidence_deadline = Instant::now() + Duration::from_secs(15);
    let mut growths = 0u32;
    let mut last_len = len_at_kill;
    while Instant::now() < evidence_deadline && growths < 2 {
        std::thread::sleep(Duration::from_millis(100));
        let now_len = bag_len();
        if now_len > last_len {
            growths += 1;
            last_len = now_len;
        }
    }
    // SCOPE, stated so the next reader does not over-trust it: growth proves the
    // bag FILE grew, from ANY source — the 20 Hz data frames alone keep it
    // growing, so this says nothing about whose TRACE ring was drained. It is a
    // readiness gate, not an oracle: it makes the mode-specific assertions below
    // ask their question only once post-kill bytes exist, so a failure there is a
    // real result rather than "I looked too early". The claim that the SURVIVOR
    // recorded past the kill is made below, against boundary `fire_time_ns`.
    assert!(
        growths >= 2,
        "the recorder never flushed twice in the 15s after the SIGKILL (bag {len_at_kill} \
         -> {last_len} bytes, {growths} growth(s)) — nothing post-kill reached the bag, so \
         the out-step assertion below could only ever tie. This is a RECORDER or bring-up \
         problem, not a scheduling one.\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // The live worker set, captured BEFORE shutdown so the no-orphan assertion
    // below has something to check. (A SIGKILLed supervisor cannot reap its
    // workers, which is why the harness teardown kills the process GROUP.)
    // Noted THROUGH THE GUARD, while the supervisor is alive, so its orphan
    // verdict is taken over a real set. PREMISE, not decoration: an empty vec
    // (the pgrep missed) would make every later check vacuous. Rank 1 was
    // SIGKILLed above, so exactly one worker — the SURVIVOR, the one a pid-only
    // teardown would leak — must be alive here.
    let workers_before_shutdown = guard.note_workers().to_vec();
    assert_eq!(
        workers_before_shutdown.len(),
        1,
        "exactly the surviving rank-0 worker must be alive before shutdown, saw \
         {workers_before_shutdown:?}"
    );

    // NOTE: the statement above about `scheduler_trace()` is true of THAT reader
    // and not of the bag.
    // `BagReader::scheduler_trace` walks from the footer's `summary_start`, which
    // a bag still being written has not got — but `recover_scheduler_trace` is the
    // documented crash-tolerant twin and DOES decode every complete chunk of a
    // partially-written bag (measured: 72 trace records mid-run). This arm keeps
    // the file-growth gate; the other sites in this
    // file use the evidence wait instead, and the two gates are not unified.

    // Ctrl-C: a degraded-continue run is a CLEAN exit (0).
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "degraded-continue must exit 0, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "degraded bag still finalized");

    // The DEPARTURE evidence: kind-2 record(s), departure-ring provenance
    // (reserved == u32::MAX), naming the killed worker's rank (node_idx == 1).
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    let departures: Vec<&TraceRingRecord> = trace
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_DEPARTURE)
        .collect();
    assert!(
        !departures.is_empty(),
        "the bag must carry at least one DEPARTURE record for the killed worker"
    );
    for d in &departures {
        assert_eq!(
            d.reserved, DEPARTURE_SENTINEL,
            "departure provenance is the departure ring's u32::MAX header rank"
        );
        assert_eq!(
            d.node_idx, 1,
            "the departure names the killed worker's plan rank (p1 = rank 1)"
        );
    }
    // Only the departure ring may carry u32::MAX provenance.
    for r in &trace {
        if r.reserved == DEPARTURE_SENTINEL {
            assert_eq!(
                r.record_type, RECORD_TYPE_DEPARTURE,
                "u32::MAX provenance is exclusively departure records"
            );
        }
    }

    // THE SURVIVOR KEPT RECORDING PAST THE KILL — but WHICH oracle proves that
    // is MODE-DEPENDENT, and conflating the two makes this arm red wherever
    // rank 1's loop turns faster (measured on aarch64 Linux).
    //
    // Both ranks must carry boundary records (the closure panics otherwise);
    // that much is mode-independent.
    let last_boundary_step = |rank: u32| -> u64 {
        trace
            .iter()
            .filter(|r| r.reserved == rank && r.record_type == RECORD_TYPE_STEP_BOUNDARY)
            .map(|r| r.step)
            .max()
            .unwrap_or_else(|| panic!("rank {rank} must carry boundary records"))
    };
    let (last0, last1) = (last_boundary_step(0), last_boundary_step(1));
    // Printed on EVERY run, pass or fail: these numbers are the whole diagnosis
    // for this arm. Run with `--nocapture` to see the `[prefix/Mode]` line.
    eprintln!("[{prefix}/{mode:?}] last0={last0} last1={last1} growths={growths}");

    match mode {
        // LOCKSTEP: the shared barrier couples the ranks — neither can advance a
        // level boundary until the other arrives — so at the kill instant they
        // are within one generation of each other, and rank 0 then free-runs
        // alone for the whole post-kill window. `last0 > last1` is true BY
        // CONSTRUCTION, and a tie is a real finding.
        RecordMode::Lockstep => assert!(
            last0 > last1,
            "the surviving rank 0 must out-step the killed rank 1 in the FINALIZED bag \
             (rank0 last step {last0} vs rank1 {last1}) — the recorder flushed \
             {growths} time(s) after the kill, so post-kill records DID reach the bag; \
             a tie here is a scheduling finding, not a recording one"
        ),
        // FREE-RUN: there is NO barrier cohort, so each rank advances its OWN
        // gating clock at its own rate for the whole pre-kill window. p1 runs ONE
        // node, p0 runs two (a period ticker plus a relay), so on a machine where p1's
        // loop turns faster it banks a lead p0's post-kill window need not close.
        // MEASURED on aarch64 Linux: `last0=91 last1=97` — rank 1 legitimately AHEAD. So a
        // cross-rank STEP comparison is not a statement free-run makes.
        //
        // What it does make, and what this arm exists to prove, is that the
        // SURVIVOR kept recording after the kill. That needs an anchor in rank 0's
        // own timeline, and there is one: boundary `fire_time_ns` is stamped from
        // the shared `real_ns()` epoch (asserted for both ranks above), so it is
        // directly comparable to the `kill_ns` read taken immediately before the
        // SIGKILL. The file-growth wait is no substitute for that anchor —
        // it measures BYTES FROM ANY SOURCE, so
        // a recorder that stopped draining rank 0's ring entirely would
        // pass it while the 20 Hz data frames kept the file growing.
        //
        // TWO boundaries, not one: a single straggler can be a record already in
        // flight when the kill landed, while two prove the survivor was still
        // being drained into the bag afterwards.
        RecordMode::FreeRun => {
            let survivor_after: Vec<u64> = trace
                .iter()
                .filter(|r| {
                    r.reserved == 0
                        && r.record_type == RECORD_TYPE_STEP_BOUNDARY
                        && r.fire_time_ns > kill_ns
                })
                .map(|r| r.fire_time_ns)
                .collect();
            // DIAGNOSTIC only — every element of `survivor_after` is already
            // filtered `> kill_ns`, so asserting the max exceeds it again would be
            // a dead assertion.
            let max_after = survivor_after.iter().copied().max().unwrap_or(0);
            eprintln!(
                "[{prefix}/{mode:?}] kill_ns={kill_ns} survivor_boundaries_after_kill={} max_after={max_after}",
                survivor_after.len()
            );
            assert!(
                survivor_after.len() >= 2,
                "free-run: the SURVIVOR (rank 0) must have at least two step boundaries \
                 stamped AFTER the kill in the FINALIZED bag — found {} (kill_ns={kill_ns}, \
                 rank0 last step {last0}, rank1 last step {last1}, {growths} post-kill file \
                 growth(s)). Zero means rank 0's trace ring stopped reaching the bag when \
                 its peer died; one can be a record already in flight.",
                survivor_after.len()
            );
            // PRECONDITION, stated as one: without it the arm could pass against a
            // kill that landed before p1 ever stepped, which proves nothing about
            // a SURVIVOR outliving a peer.
            assert!(
                last1 > 0,
                "free-run: the killed rank must have banked boundaries before it died \
                 (rank1 last step {last1})"
            );
        }
    }

    // NO ORPHANS: every worker this run spawned is gone, the way
    // `network_gateway_e2e_test` asserts after its own teardown. The victim
    // was SIGKILLed, so it is expected gone; the SURVIVOR is the one a pid-only
    // teardown would leak.
    guard.finish().assert_clean();

    // The sentinel manifest stays [] even on the degraded path.
    let (_rd, idsd) =
        read_manifest(&reader, DEPARTURE_SENTINEL).expect("departure sentinel manifest present");
    assert!(idsd.is_empty(), "departure manifest stays empty");

    // EVERY ring name is gone — including the SIGKILLed
    // worker's (its own owner never ran Drop; the supervisor sweep unlinked it).
    assert_rings_swept(sup_pid);

    // The cohort arm that ran is the one the
    // mode implies, and ONLY that one. Lockstep repairs the barrier cohort
    // (`drop_dead_peers_batch`'s line); free-run has no cohort, says so once
    // per death, and its post-loop summary must not claim a drop it never did.
    const LOCKSTEP_REPAIR: &str = "dropped the dead peer from the barrier cohort";
    const FREE_RUN_NO_DROP: &str = "no drop and pay no stall grace";
    const LOCKSTEP_SUMMARY: &str = "were dropped from the barrier cohort";
    const FREE_RUN_SUMMARY: &str = "shared no barrier cohort with them";
    let log = read_file(&stderr_path);
    match mode {
        RecordMode::Lockstep => {
            assert_eq!(
                log.matches(LOCKSTEP_REPAIR).count(),
                1,
                "a lockstep supervisor repairs the cohort EXACTLY once for the one killed \
                 worker; log was:\n{log}"
            );
            assert!(
                !log.contains(FREE_RUN_NO_DROP) && !log.contains(FREE_RUN_SUMMARY),
                "a lockstep supervisor never takes the free-run arm; log was:\n{log}"
            );
            assert_eq!(
                log.matches(LOCKSTEP_SUMMARY).count(),
                1,
                "the degraded summary names the cohort drop exactly once; log was:\n{log}"
            );
        }
        RecordMode::FreeRun => {
            assert_eq!(
                log.matches(FREE_RUN_NO_DROP).count(),
                1,
                "a free-run supervisor says once per death that there is no cohort to repair; \
                 log was:\n{log}"
            );
            assert!(
                !log.contains(LOCKSTEP_REPAIR) && !log.contains(LOCKSTEP_SUMMARY),
                "a free-run supervisor never repairs a cohort it did not create; log was:\n{log}"
            );
            assert_eq!(
                log.matches(FREE_RUN_SUMMARY).count(),
                1,
                "the degraded summary says the survivors shared no cohort; log was:\n{log}"
            );
        }
    }
}

/// (5) The mp killed-bagd exit-code mapping (mirrors the single-process
/// `record_e2e_killed_bagd_maps_to_nonzero_exit_naming_incomplete_bag`):
/// SIGKILL the bagd grandchild mid-run → the supervisor's finalize observes
/// the dead recorder and `graph run` exits NONZERO naming the INCOMPLETE bag.
#[test]
#[serial]
fn mp_record_killed_bagd_maps_to_nonzero_exit() {
    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), "mprece");
    let (mut guard, _stdout_path, stderr_path) = spawn_mp_record(tmp.path(), &[]);
    let _bagd_guard = BagdGuard::arm();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90))
        .expect("bagd never created the bag (mp handshake failed?)");
    let bagd_pid = find_bagd_pid(&bag, Duration::from_secs(10))
        .expect("could not locate the bagd grandchild by its bag filename");

    // Kill the recorder mid-run, then Ctrl-C the supervisor. The workers drain
    // cleanly; the post-JOIN finalize reaps the SIGKILLed recorder and maps it
    // to the recording failure (same `report_bagd_outcome` arm as the single-process path).
    send_signal(bagd_pid, libc::SIGKILL);
    std::thread::sleep(Duration::from_millis(200));
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        !status.success(),
        "a dead recorder must map to a NONZERO supervisor exit, got {status:?}"
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

/// (6) The file-stem identity rule: a graph whose file name differs from its legacy
/// `name:` key stems its bag by the FILE, end to end through the real binary.
///
/// Without the rule the bag filename would follow the INTERNAL `name:` while
/// only the `graph.yaml` ATTACH path followed the file, the split the
/// file-stem rule removes.
/// The fixture is deliberately divergent: `graphs/mpdemo_remap.yaml`
/// declaring `name: legacy_internal_name` must produce
/// `recordings/mpdemo_remap_<stamp>.mcap`, never `legacy_internal_name_*`.
///
/// It runs over the REAL supervisor because the stem is minted deep inside the
/// mp recording bring-up (`resolve_recording_paths`, off the spec the
/// supervisor fills) — an engine-level test constructs that spec by hand and so
/// cannot see a supervisor that fills it from the wrong string.
///
/// THREE claims in one body, because each is independently reachable-wrong:
/// the bag STEM is the file; the run still records mp-shaped (a wrong attach
/// path aborts bagd and no bag appears at all);
/// and the deprecated key still ROUND-TRIPS into the embedded `graph.yaml`,
/// which is the back-compat half of the file-stem identity rule.
///
/// The file stem keeps the `mpdemo_` prefix so the run's bag filename is
/// covered by the shared `BagdGuard` orphan-sweep needle — which under this
/// rule is exactly what makes the guard follow the file.
#[test]
#[serial]
fn mp_record_stems_the_bag_by_the_file_not_the_declared_name() {
    let tmp = tempfile::tempdir().unwrap();
    // FILE `graphs/mpdemo_remap.yaml`; legacy `name: legacy_internal_name`.
    build_mp_workspace_named(tmp.path(), "mprecf", "mpdemo_remap", "legacy_internal_name");
    let (mut guard, stdout_path, stderr_path) =
        spawn_mp_record_graph(tmp.path(), "mpdemo_remap", &[], &[]);
    let _bagd_guard = BagdGuard::arm();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
        panic!(
            "bagd never created the bag — the mp path must attach the LOADED \
             `graphs/mpdemo_remap.yaml`.\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        )
    });
    // The file-stem oracle, on the filename itself.
    let stem = bag
        .file_name()
        .and_then(|s| s.to_str())
        .expect("bag filename")
        .to_string();
    assert!(
        stem.starts_with("mpdemo_remap_"),
        "project rule: the bag is stemmed by the FILE (`mpdemo_remap`), got `{stem}`"
    );
    assert!(
        !stem.starts_with("legacy_internal_name"),
        "the deprecated `name:` key must not stem anything, got `{stem}`"
    );
    // A healthy window, waited for on the recorded stream (the acceptance below
    // reuses the arm-1 helpers, which read the per-rank trace).
    wait_for_bag_state(
        &bag,
        "both ranks' boundary streams",
        RECORDED_WINDOW_TIMEOUT,
        |snap| {
            snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES
                && snap.boundaries_for_rank(1) >= RECORDED_WINDOW_BOUNDARIES
        },
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "the mismatched-name mp recorded run must exit 0, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    // ONE finalized, mp-shaped bag (reuse the arm-1 acceptance helpers).
    let bag2 = assert_single_bag(&recordings);
    assert_eq!(bag, bag2, "the bag observed mid-run is the final one");
    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the mp teardown must FINALIZE the bag, got {completeness:?}"
    );

    // mp shape: both worker manifests + the empty departure sentinel.
    let (r0, ids0) = read_manifest(&reader, 0).expect("rank0 manifest present");
    assert_eq!(r0, 0);
    assert_eq!(
        ids0,
        vec!["ticker".to_string(), "relay".to_string()],
        "rank-0 manifest = p0's subgraph node ids"
    );
    let (r1, ids1) = read_manifest(&reader, 1).expect("rank1 manifest present");
    assert_eq!(r1, 1);
    assert_eq!(
        ids1,
        vec!["sink".to_string()],
        "rank-1 manifest = p1's subgraph node ids"
    );
    let (_rd, idsd) =
        read_manifest(&reader, DEPARTURE_SENTINEL).expect("departure sentinel manifest present");
    assert!(idsd.is_empty(), "departure manifest stays empty");

    // Back-compat, on the bytes: the file-stem rule keeps the deprecated key
    // parsing and ROUND-TRIPPING, so the embedded config — which is
    // the EFFECTIVE `GraphConfig` rendered by `render_effective_graph_yaml`,
    // not the file's bytes — still carries the `name:` line the author wrote.
    // It just decides nothing: the bag it sits in is stemmed by the file.
    let att = reader
        .attachment("graph.yaml")
        .expect("read attachments")
        .expect("the graph.yaml attachment must be present");
    let graph_src = String::from_utf8_lossy(&att.data);
    assert!(
        graph_src.contains("name: legacy_internal_name"),
        "the deprecated key must ROUND-TRIP into the embed, got:\n{graph_src}"
    );

    // Provenance sanity: both ranks reach the bag; no departures on the happy path.
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    assert!(!trace.is_empty(), "the mp bag must carry trace records");
    for r in &trace {
        assert_ne!(
            r.record_type, RECORD_TYPE_DEPARTURE,
            "happy path: no worker died, so no DEPARTURE record may exist"
        );
    }
    let ranks = by_rank(&trace);
    assert_eq!(
        ranks.keys().copied().collect::<Vec<_>>(),
        vec![0, 1],
        "both workers' rings must reach the bag"
    );
}

/// The MULTI-PROCESS record path hands bagd the schema catalog too.
///
/// The two record paths spawn bagd from DIFFERENT call sites (the single-process
/// `run_graph_recording` and the supervisor's
/// `start_supervisor_recording`), each building its own `BagdSpawnSpec`. The
/// single-process loop is pinned in `graph_record_e2e_test.rs`; a supervisor
/// site that passed no catalog would leave every multi-process recording — the
/// DEFAULT shape on Unix — silently textless, and no arm there
/// could see it.
///
/// The workspace ships a SHADOW of the built-in the fixture cdylibs publish
/// (the only doc a temp workspace can hold whose hash a prebuilt node's
/// AUTHORITATIVE `OutputMeta` carries — see the sibling file's
/// `build_workspace_with_schemas` for the full argument) plus an UNPUBLISHED
/// custom type that must be pruned away.
#[test]
#[serial]
fn mp_record_bag_carries_the_workspace_schema_text_across_the_supervisor_spawn() {
    const SHADOW_VECTOR3: &str = "# this workspace's OWN definition, not the compiled-in \
                                  one\nfloat64 x\nfloat64 y\nfloat64 z\n";

    let tmp = tempfile::tempdir().unwrap();
    build_mp_workspace(tmp.path(), "mprecsc");
    let schemas = tmp.path().join("schemas");
    std::fs::create_dir_all(schemas.join("geometry_msgs/msg")).unwrap();
    std::fs::create_dir_all(schemas.join("msgs/msg")).unwrap();
    std::fs::write(
        schemas.join("geometry_msgs/msg/Vector3.msg"),
        SHADOW_VECTOR3,
    )
    .unwrap();
    std::fs::write(
        schemas.join("msgs/msg/Unused.msg"),
        "msgs/Leaf leaf\nint32 count\n",
    )
    .unwrap();
    std::fs::write(schemas.join("msgs/msg/Leaf.msg"), "float32 value\n").unwrap();

    let (mut guard, stdout_path, stderr_path) =
        spawn_mp_record_graph(tmp.path(), "mpdemo", &[], &[]);
    let _bagd_guard = BagdGuard::arm();

    let recordings = tmp.path().join("recordings");
    let bag = wait_for_bag(&recordings, Duration::from_secs(90))
        .expect("bagd never created the bag (mp handshake failed?)");
    // Wait for a recorded stream instead of a window: the catalog assertions
    // below need the bag to carry this run's channels, which is the same
    // evidence.
    wait_for_bag_state(
        &bag,
        "rank-0's boundary stream",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES,
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("supervisor did not exit after SIGINT");
    assert!(
        status.success(),
        "the recorded mp run must exit 0, got {status:?}\nstdout:\n{}\nstderr:\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    );

    let reader = BagReader::open(&bag).expect("open bag");
    let (_msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "the mp teardown must FINALIZE the bag, got {completeness:?}"
    );
    let catalog = reader
        .schema_catalog()
        .expect("a MULTI-PROCESS recording must carry its schema provenance");
    let docs: Vec<&str> = catalog.docs.iter().map(|d| d.qualified.as_str()).collect();
    assert_eq!(
        docs,
        vec!["geometry_msgs/Vector3"],
        "the supervisor's bagd ships exactly the recorded closure — the unpublished \
         `msgs/*` types must be pruned"
    );
    assert_eq!(
        catalog.docs[0].text, SHADOW_VECTOR3,
        "the doc must be THIS workspace's definition, verbatim"
    );
    // All three mp topics carry the same schema, so the pruned bindings dedup
    // to one — the recorded closure, not the whole corpus.
    assert_eq!(catalog.hashes.len(), 1);
    assert_eq!(catalog.hashes[0].qualified, "geometry_msgs/Vector3");
}
