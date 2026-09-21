// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary proof that a MULTI-PROCESS,
//! multi-rank, QUANTUM-timed recording replays byte-exact to EXIT 0 through the
//! replay engine.
//!
//! Recipe (the proven `mp_record_e2e` record harness, shared via `mp_support`):
//!
//!   1. RECORD — spawn the REAL `cerulion graph run mpdemo --record` against a
//!      hand-built tempdir workspace whose 3-node chain
//!      (`ticker`(Period 50ms) → `relay`(data-trigger) → `sink`(data-trigger))
//!      is split into TWO `process_groups` (`p0:[ticker,relay]` rank 0,
//!      `p1:[sink]` rank 1). The supervisor spawns one worker per group; a
//!      directed SIGINT to the supervisor drives the orchestrated teardown →
//!      ONE finalized, QUANTUM-timed (handed-quantum barrier-lockstep) bag.
//!   2. REPLAY — run the REAL `cerulion bag play <bag> --resim all --verify
//!      --report <file>` with
//!      cwd = the workspace root (the engine resolves the SAME
//!      ticker/relay/sink cdylibs as the candidate) and assert EXIT CODE 0 (the
//!      byte-exact self-replay of the multi-rank bag through the engine's
//!      k-way rank demux + `(global_level,rank,seq)` merge + boundary-driven
//!      clock re-advance). mp bags are QUANTUM-timed by construction, so
//!      replay honors the recorded quantum — there are NO wall-clock asserts.
//!
//! Genuinely-multi-rank guard (the test cannot silently pass on a single-rank
//! bag): BEFORE replaying, the bag is asserted to carry TWO worker manifests
//! (`rank0` + `rank1`) AND trace records stamped from BOTH ranks (`reserved`
//! 0 AND 1) — k >= 2.
//!
//! This arm also asserts the `--report` JSON's `read_log`
//! block reports the redundant per-edge read-log verifier `verified_clean`
//! over at least one compared edge (`assert_read_log_verified_clean`, shared
//! via `mp_support`). The verifier itself is REPORT-ONLY, so without this
//! assert a divergence — or a verifier that silently went inert — would leave
//! CI green with its `warn!` in libtest's discarded stderr. This assert is
//! what makes a green run evidence that the verifier compared and agreed,
//! rather than an absence of news.
//!
//! Robustness guard (a LOADED machine must not weaken the exit-0 contract): a
//! record whose finalize shows record-side frame loss (`record_health.json`
//! `frames_lost` / `dropped_unwritten` != 0) produces a lossy bag that would
//! replay to a spurious missing-frame violation (exit 1). Rather than soften the
//! exit-0 assert, the record leg RETRIES (up to 3 total attempts on fresh
//! workspaces); only a loss-free bag is replayed. If no clean bag is producible
//! the test fails LOUDLY with the health summary — it never lowers the bar.
//!
//! GATED `#[cfg(unix)]` (NOT linux-only): the multi-process supervisor is
//! REAL on macOS too, so this file runs on macOS and on Linux.
//! `#[serial]`: the mp DATA plane,
//! the supervisor's PLANNING build, and the replay all touch shared iceoryx2
//! namespaces (unique per-run prefixes keep topic names apart, serialization
//! keeps the transports from overlapping).
//!
//! Prerequisites (the repo's fixture pattern — the helpers PANIC with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`

#![cfg(unix)]

use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use cerulion_bag::BagReader;
use serial_test::serial;

// The reusable mp record-harness helpers (`build_mp_workspace`,
// `spawn_mp_record`, `ChildGuard`/`BagdGuard`, the manifest/trace readers, …)
// are LIFTED into the shared `mp_support` module so this
// file drives the SAME proven recipe as `mp_record_e2e_test.rs` without copy-
// paste. Each test binary compiles the module in (the `tests/common` pattern).
mod mp_support;
use mp_support::*;

/// The on-bag record-health attachment (bagd's `RECORD_HEALTH_ATTACHMENT`; a
/// stable on-bag contract, restated here to avoid a `cerulion_bagd` dev-dep —
/// the manifest attachment names are hardcoded the same way).
const RECORD_HEALTH_ATTACHMENT: &str = "__cerulion/record_health.json";

/// Total record-side frame loss for a finalized bag, read from its
/// `record_health.json` stamp: `dropped_unwritten` (frames drained off a tap
/// that never reached the bag) + the sum of every topic's `frames_lost`
/// (tap-queue overflow). Returns
/// `(loss_total, raw_health_json)`; an ABSENT attachment is treated as healthy
/// (`0`), since that is an older bag shape and never a false retry trigger.
fn total_record_loss(reader: &BagReader) -> (u64, String) {
    match reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
    {
        Some(att) => {
            let raw = String::from_utf8_lossy(&att.data).into_owned();
            let v: serde_json::Value = match serde_json::from_slice(&att.data) {
                Ok(v) => v,
                // A malformed stamp is never exit-affecting for replay; treat it
                // as unknown-but-not-a-loss-signal (0) and surface the raw bytes.
                Err(_) => return (0, raw),
            };
            let dropped = v["dropped_unwritten"].as_u64().unwrap_or(0);
            let frames_lost: u64 = v["topics"]
                .as_object()
                .map(|topics| {
                    topics
                        .values()
                        .map(|t| t["frames_lost"].as_u64().unwrap_or(0))
                        .sum()
                })
                .unwrap_or(0);
            (dropped + frames_lost, raw)
        }
        None => (
            0,
            "<record_health.json absent — treated as healthy>".to_string(),
        ),
    }
}

/// Record ONE loss-free multi-process `mpdemo` bag via the real binary, keeping
/// the tempdir workspace alive (the replay leg runs with cwd = its root so the
/// engine resolves the same cdylibs). RETRIES up to 3 total attempts on fresh
/// workspaces if a finalize shows record-side loss (loaded machine); PANICS
/// loudly with the health summary if no clean bag is producible — never
/// weakening the caller's exit-0 contract.
fn record_clean_mp_bag() -> (tempfile::TempDir, PathBuf) {
    const ATTEMPTS: u32 = 3;
    let mut last_health = String::from("<no attempt completed>");
    for attempt in 0..ATTEMPTS {
        let tmp = tempfile::tempdir().unwrap();
        // Unique per attempt — the supervisor's PLANNING build shares the global
        // iceoryx2 namespace, so topic names must not collide across retries.
        let prefix = format!("mprr{}a{attempt}", std::process::id() % 100_000);
        build_mp_workspace(tmp.path(), &prefix);
        let (mut guard, stdout_path, stderr_path) = spawn_mp_record(tmp.path(), &[]);
        let _bagd_guard = BagdGuard::arm();

        let recordings = tmp.path().join("recordings");
        // Planning build + 2 worker spawns + bagd handshake precede the bag —
        // generous bound for slow CI VMs.
        if wait_for_bag(&recordings, Duration::from_secs(90)).is_none() {
            panic!(
                "bagd never created the bag (mp handshake failed?) on attempt {attempt}\n\
                 stdout:\n{}\nstderr:\n{}",
                read_file(&stdout_path),
                read_file(&stderr_path)
            );
        }
        // A healthy window (the 50ms ticker fires ~80-120 times across the chain).
        std::thread::sleep(Duration::from_secs(5));

        // Directed SIGINT to the SUPERVISOR (the production Ctrl-C; the supervisor fans
        // it out to the workers → drain → JOIN reap → bagd final-drain + finalize).
        send_signal(guard.id(), libc::SIGINT);
        let status = guard
            .wait_bounded(Duration::from_secs(90))
            .expect("supervisor did not exit after SIGINT");
        assert!(
            status.success(),
            "the mp record run must exit 0 on Ctrl-C, got {status:?}\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );

        // Exactly one finalized bag, then check record-side loss.
        let bag = assert_single_bag(&recordings);
        let reader = BagReader::open(&bag).expect("open bag");
        let (_msgs, completeness) = reader.recover_messages().expect("recover");
        assert!(
            completeness.is_finalized(),
            "the mp teardown must FINALIZE the bag, got {completeness:?}"
        );
        let (loss, health) = total_record_loss(&reader);
        if loss == 0 {
            drop(reader);
            return (tmp, bag);
        }
        last_health = format!(
            "attempt {attempt}: record-side loss = {loss} frame(s) — the bag would replay to a \
             spurious missing-frame violation; health = {health}"
        );
        // `tmp` drops at loop end → fresh workspace + prefix on the next attempt.
    }
    panic!(
        "could not produce a loss-free multi-process recording in {ATTEMPTS} attempts \
         (loaded machine?) — REFUSING to weaken the exit-0 self-replay assert. \
         Last health: {last_health}"
    );
}

/// The headline pin: a REAL multi-process, multi-rank, QUANTUM-timed recording
/// replays byte-exact to EXIT 0 through the replay engine.
#[test]
#[serial]
fn mp_recorded_bag_replays_to_exit_0_multi_rank() {
    let (tmp, bag) = record_clean_mp_bag();
    let root = tmp.path();

    // --- Genuinely-multi-rank guard (BEFORE replay): two worker manifests AND
    // trace records stamped from BOTH ranks → k >= 2. Without this the test
    // could silently pass on a degenerate single-rank bag. ---
    let reader = BagReader::open(&bag).expect("open bag");
    let (r0, _ids0) = read_manifest(&reader, 0).expect("rank0 manifest present");
    assert_eq!(r0, 0, "rank-0 manifest carries rank 0");
    let (r1, ids1) = read_manifest(&reader, 1)
        .expect("rank1 (worker) manifest present — the run MUST be multi-rank (k>=2)");
    assert_eq!(r1, 1, "rank-1 manifest carries rank 1");
    assert_eq!(
        ids1,
        vec!["sink".to_string()],
        "rank-1 manifest = p1's subgraph node ids (the cross-group sink)"
    );
    let trace = reader.scheduler_trace().expect("scheduler_trace");
    let ranks = by_rank(&trace);
    assert!(
        ranks.contains_key(&0) && ranks.contains_key(&1),
        "the bag's trace must carry records stamped from BOTH ranks (genuinely \
         multi-process, k>=2); ranks present: {:?}",
        ranks.keys().collect::<Vec<_>>()
    );
    drop(reader);

    // --- Replay the mp bag through the REAL binary, cwd = the workspace root
    // (the engine resolves the workspace's ticker/relay/sink cdylibs as the
    // candidate). mp bags are QUANTUM-timed → the clock re-advance from the
    // recorded StepBoundary trace reproduces the wire timestamps byte-exact; NO
    // wall-clock asserts. ---
    let report = root.join("mp_replay_report.json");
    // The verb is `bag play --resim all --verify`; it drives the replay
    // engine and its 0-6 exit contract.
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "--resim", "all", "--verify"])
        .arg(&bag)
        .arg("--report")
        .arg(&report)
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target — the workspace copy).
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("failed to spawn the cerulion binary");
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    let report_txt =
        std::fs::read_to_string(&report).unwrap_or_else(|_| "<no report written>".to_string());

    assert_eq!(
        out.status.code(),
        Some(0),
        "a REAL multi-rank QUANTUM-timed mp bag must replay byte-exact to EXIT 0 through the \
         replay engine;\nstderr:\n{stderr}\nreport:\n{report_txt}"
    );
    assert!(
        stderr.contains("replay PASS"),
        "the PASS verdict is printed to stderr:\n{stderr}"
    );

    // --- The `--report` JSON proves the replay was NON-VACUOUS (text-free): a
    // fed trace was replayed AND at least one topic was byte-compared + matched,
    // with no node failures and no data violations on a byte-exact self-replay. ---
    let report_json: serde_json::Value =
        serde_json::from_str(&report_txt).expect("replay report is valid JSON");
    assert_eq!(
        report_json["passed"], true,
        "the report records a clean PASS: {report_json}"
    );
    assert!(
        report_json["ticks_replayed"].as_u64().unwrap_or(0) > 0,
        "the replayed trace must be NON-EMPTY (mp bags carry the per-rank fed trace): {report_json}"
    );
    assert!(
        report_json["topics_passed"].as_u64().unwrap_or(0) > 0,
        "at least one topic was byte-compared and matched (non-vacuous diff): {report_json}"
    );
    assert!(
        report_json["node_failures"]
            .as_array()
            .map(|a| a.is_empty())
            // absent/malformed field = FAIL (schema drift must not pass vacuously)
            .unwrap_or(false),
        "a clean self-replay has NO node failures (and the field must exist): {report_json}"
    );
    assert!(
        report_json["violations"]
            .as_array()
            .map(|a| a.is_empty())
            // absent/malformed field = FAIL (schema drift must not pass vacuously)
            .unwrap_or(false),
        "a byte-exact self-replay has NO data violations (and the field must exist): {report_json}"
    );

    // --- The read-log verifier must have RUN and AGREED. ---
    assert_read_log_verified_clean(&report_json, "the multi-rank mp self-replay");
}
