// SPDX-License-Identifier: AGPL-3.0-only
//! **The missing shape, over the REAL binary**: two `period_ms`
//! nodes at ONE global level, joined by a plain NON-TRIGGER `#[input]`, split
//! across two process groups, recorded and re-executed.
//!
//! # Why this file exists
//!
//! This is the coverage gap that let the split-pair nondeterminism ship. `mp_record_replay_e2e_test`
//! records a `ticker -> relay -> sink` chain in which EVERY edge is
//! `#[input(trigger)]`, and a trigger edge levelizes its consumer strictly BELOW
//! its producer — so every pair there is separated by a level boundary and
//! ordered by the end-of-level barrier BY CONSTRUCTION. No test anywhere drove
//! the one edge the DAG does not model, so the runtime shipped a shape whose
//! live pairing was decided by OS scheduling and whose bag could not replay.
//!
//! MEASURED on the pre-retarget `examples/obstacle_avoidance` (the decisive
//! experiment for this defect): 7 of 12 process-per-node runs failed their own
//! re-execution, every divergence the controller's `Vector3.x`, and the
//! recordings differed **from each other**. Co-locating both nodes in ONE group
//! fixed it 5/5 — pinning causality to the split.
//!
//! # The graph
//!
//! ```text
//! scanner    (period_ms, output `cmd`)              -> group p0, rank 0
//! controller (period_ms, plain `#[input] inp`)      -> group p1, rank 1
//! ```
//!
//! `controller` reads `scanner/cmd` through a NON-trigger input, so nothing
//! levelizes it below the scanner and both own global level 0: the split-pair
//! shape, end to end, through the real supervisor.
//!
//! The controller FORWARDS what it read (`out.x = inp.x`), so the pairing it
//! chose is not an internal detail: it is recorded, frame by frame, on
//! `<prefix>/controller/out`. That is what makes both arms below assertable.
//!
//! # What this file proves — and what it does NOT
//!
//! **It IS the plumbing discriminator.** The five-hop chain
//! classify -> stamp -> serialise -> deserialise -> install runs only in a real
//! supervisor+worker deployment, and the in-process pin
//! (`cerulion_core/tests/mid_level_barrier_iox2_test.rs`) structurally cannot
//! see it — that test injects the barrier participant directly. Each arm here
//! asserts, from the WORKER's own build line, that it installed exactly one
//! mid-level level and a two-generation-per-step law. MEASURED: a
//! `stamp_mid_level_barrier` that is made a no-op fails both arms on that
//! assertion.
//!
//! **It is NOT the ordering discriminator, and it must not be read as one.**
//! MEASURED: with the mid-level rendezvous neutralised in `step_live` (`mid`
//! forced false) both arms below still PASS. That is not a gap in the asserts,
//! it is a property of the fixture: the end-of-level barrier already keeps the
//! two workers in tight lockstep, so this graph's interleave resolves the same
//! way run after run and the unordered read is stable rather than flipping.
//! The original repro needed 12-second runs of an OSCILLATING signal whose
//! threshold a one-step shift flips, and even then only ~58 % of runs diverged
//! — a ~2 % per-boundary flip rate. An arm built on that would be a coin-toss
//! gate, so the ORDERING guarantee is pinned where it can be made deterministic:
//! the in-process A/B, which reproduces the defect on demand.
//!
//! # The two arms
//!
//! 1. **Record -> re-execute -> EXIT 0.** The bag passes its own
//!    `cerulion bag play --resim all --verify` WITH the extra rendezvous in
//!    place — so the added generation does not desynchronise the deployment,
//!    starve a worker, lose frames or break the recorded trace. `--verify` is
//!    load-bearing: a bare `--resim all` is neutral and exits 0 on any
//!    completed run.
//! 2. **Two live runs agree.** Two recordings of the SAME graph carry
//!    byte-identical controller frames. This is the property a bag-side fix
//!    (recording the consumed pairing) could never have delivered — two mp runs
//!    would still differ, breaking "deterministic runtime, repeatable"
//!    independently of replay.
//!
//! `#[cfg(unix)]` (the mp supervisor is real on Linux AND macOS)
//! and `#[serial]`: the supervisor's planning build and the data plane share
//! process-global iceoryx2 namespaces.
//!
//! Prerequisites — the repo's fixture pattern (the helpers PANIC with the exact
//! instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_period_input_cdylib`

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

use cerulion_bag::BagReader;
use serial_test::serial;

mod mp_support;
use mp_support::*;

/// The on-bag record-health attachment (bagd's `RECORD_HEALTH_ATTACHMENT`).
const RECORD_HEALTH_ATTACHMENT: &str = "__cerulion/record_health.json";

/// Strip ANSI SGR escapes from captured output.
///
/// LOAD-BEARING, not cosmetic: `tracing`'s default formatter wraps a structured
/// field's name and its `=` in separate escape sequences, so the rendered line
/// contains `<esc>[3mmid_level_barriers<esc>[0m<esc>[2m=<esc>[0m1` and the
/// literal `mid_level_barriers=1` is NOT a substring of it. Matching the raw
/// capture makes every `key=value` assertion in this file silently unsatisfiable
/// — which is exactly how the first version of these asserts failed against a
/// run whose own log showed the right values.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(s.len());
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
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

/// Total record-side frame loss for a finalized bag, and how many topic entries
/// that reading actually walked.
///
/// A lossy bag would replay to a spurious missing-frame violation, so the record
/// leg retries rather than the assert being softened.
///
/// # `topics` is an OBJECT, and reading it as an array was a vacuous guard
///
/// `RecordHealth::topics` is a `BTreeMap<String, TopicHealth>`, so it serialises
/// as a JSON object keyed by canonical topic name. Read with `as_array()` it is
/// always `None`, so the per-topic `frames_lost` term silently contributed ZERO
/// and the loss-free retry accepted lossy bags — the guard reported healthy for
/// a reason unrelated to health. (The sibling `mp_record_replay_e2e_test`
/// iterates `as_object().values()`, which is correct; this helper was rewritten
/// from it and lost that.)
///
/// # The count is returned so the guard cannot go vacuous again
///
/// A parse that finds NOTHING sums to zero and reads exactly like a clean
/// recording, which is how the array bug hid. `None` means the attachment was
/// absent and nothing was walked; `Some(n)` is the number of topic entries the
/// sum really ran over. The caller requires `Some(n >= 1)`, so a shape change
/// fails LOUDLY rather than quietly reporting health it never looked for.
fn total_record_loss(reader: &BagReader) -> (u64, String, Option<usize>) {
    match reader
        .attachment(RECORD_HEALTH_ATTACHMENT)
        .expect("read attachments")
    {
        Some(att) => {
            let raw = String::from_utf8_lossy(&att.data).into_owned();
            // An unreadable stamp makes the loss-free guard UNVERIFIABLE, and
            // the retry loop would then accept a lossy bag on the strength of a
            // number nobody computed. Fail loudly instead.
            let v: serde_json::Value = serde_json::from_str(&raw)
                .unwrap_or_else(|e| panic!("record_health.json is not valid JSON ({e}):\n{raw}"));
            let dropped = v["dropped_unwritten"].as_u64().unwrap_or(0);
            let topics = v["topics"].as_object().unwrap_or_else(|| {
                panic!(
                    "record_health.json `topics` is not a JSON object — the per-topic \
                     frames_lost term cannot be read, so this guard would report health it \
                     never looked for:\n{raw}"
                )
            });
            let lost: u64 = topics
                .values()
                .map(|t| t["frames_lost"].as_u64().unwrap_or(0))
                .sum();
            (dropped + lost, raw, Some(topics.len()))
        }
        None => (
            0,
            "<record_health.json absent>".to_string(),
            // Nothing was walked. bagd stamps this attachment on every finalized
            // bag, so on THIS path an absent one means the guard is
            // unverifiable — the caller says so rather than reading the zero as
            // health.
            None,
        ),
    }
}

/// Build a workspace whose graph is EXACTLY the split-pair shape: two `period_ms`
/// nodes at one global level joined by a plain non-trigger `#[input]`, split
/// across two process groups.
///
/// Both node types are existing fixtures, unmodified —
/// `test_node_macro_period_cdylib` (output-only period producer) and
/// `test_node_macro_period_input_cdylib`, which is the ONLY macro cdylib
/// fixture that is a `period_ms` node carrying a plain non-trigger `#[input]`
/// (its own module docs say so, and that is precisely the shape needed here).
fn build_split_pair_workspace(root: &Path, prefix: &str) {
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
        ("scanner", "test_node_macro_period_cdylib"),
        ("controller", "test_node_macro_period_input_cdylib"),
    ] {
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

    // Declaration order gives p0 rank 0, p1 rank 1. `controller`'s input is
    // PLAIN (the fixture declares `#[input] inp`, no `trigger`), so the two
    // nodes share global level 0 — the whole point.
    std::fs::write(
        root.join("graphs/splitpair.yaml"),
        format!(
            "name: splitpair\n\
             prefix: {prefix}\n\
             process_groups:\n\
             \x20 p0:\n\
             \x20 - scanner\n\
             \x20 p1:\n\
             \x20 - controller\n\
             nodes:\n\
             - id: scanner\n\
             \x20 type: scanner\n\
             \x20 inputs: []\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             - id: controller\n\
             \x20 type: controller\n\
             \x20 inputs:\n\
             \x20 - name: inp\n\
             \x20\x20\x20 source: scanner/cmd\n\
             \x20 outputs:\n\
             \x20 - name: out\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
}

/// Record ONE loss-free bag of the split-pair graph through the REAL binary,
/// keeping the workspace alive (the replay leg runs with cwd = its root).
///
/// Retries on record-side loss exactly as `mp_record_replay_e2e_test` does — a
/// lossy bag replays to a spurious missing-frame violation, and the exit-0
/// contract must never be weakened to absorb it.
fn record_split_pair_bag(tag: &str) -> (tempfile::TempDir, PathBuf) {
    const ATTEMPTS: u32 = 3;
    let mut last_health = String::from("<no attempt completed>");
    for attempt in 0..ATTEMPTS {
        let tmp = tempfile::tempdir().unwrap();
        let prefix = format!("sp{}{tag}a{attempt}", std::process::id() % 10_000);
        build_split_pair_workspace(tmp.path(), &prefix);

        let stdout_path = tmp.path().join("run.stdout");
        let stderr_path = tmp.path().join("run.stderr");
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        cmd.args(["graph", "run", "splitpair", "--record=recordings"])
            .current_dir(tmp.path())
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env(
                "RUST_LOG",
                "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
            )
            .stdout(std::process::Stdio::from(
                std::fs::File::create(&stdout_path).unwrap(),
            ))
            .stderr(std::process::Stdio::from(
                std::fs::File::create(&stderr_path).unwrap(),
            ));
        let mut guard = ChildGuard::spawn_group_leader(&mut cmd)
            .expect("spawn cerulion graph run --record (split pair)");
        let _bagd_guard = BagdGuard::arm();

        let recordings = tmp.path().join("recordings");
        let bag_path = wait_for_bag(&recordings, Duration::from_secs(90)).unwrap_or_else(|| {
            panic!(
                "bagd never created the bag on attempt {attempt}\nstdout:\n{}\nstderr:\n{}",
                read_file(&stdout_path),
                read_file(&stderr_path)
            )
        });
        // The recording window, WAITED FOR rather than slept: this helper's
        // verdict is read off the bag (per-rank trace + per-topic frames + a
        // loss-free `record_health.json`), so the window ends when both ranks
        // have a recorded boundary stream and both of this graph's topics carry
        // frames. Attempt 1 therefore costs about one chunk flush instead of a
        // flat 4 s, and the retry loop above is untouched.
        wait_for_bag_state(
            &bag_path,
            "both ranks' boundary streams and both topics' frames",
            RECORDED_WINDOW_TIMEOUT,
            |snap| {
                snap.boundaries_for_rank(0) >= RECORDED_WINDOW_BOUNDARIES
                    && snap.boundaries_for_rank(1) >= RECORDED_WINDOW_BOUNDARIES
                    && snap.user_topics_with_frames(1) >= 2
            },
        );

        send_signal(guard.id(), libc::SIGINT);
        let status = guard
            .wait_bounded(Duration::from_secs(90))
            .expect("supervisor did not exit after SIGINT");
        assert!(
            status.success(),
            "the split-pair record run must exit 0 on Ctrl-C, got {status:?}\n\
             stdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );

        // --- THE PLUMBING CHAIN, end to end, deterministically. ---
        //
        // This is what this file discriminates on (see the module docs): the
        // supervisor must CLASSIFY the split pair and STAMP its level, and the
        // WORKER must deserialise that vector and INSTALL it — a chain of five
        // hops (classify -> stamp -> serialise -> deserialise -> install) that
        // the in-process test structurally cannot see, because it injects the
        // participant directly.
        //
        // The worker's own build line carries the law it installed, so
        // `generations_per_step=2` on a ONE-level graph is the whole chain
        // proven from the far end. Workers inherit the supervisor's redirected
        // stderr, so their lines land in this run's capture.
        let log = strip_ansi(&format!(
            "{}{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        ));
        assert!(
            log.contains("stamping mid-level barrier levels into every worker plan"),
            "the supervisor must FIND the split same-level non-trigger pair and stamp its \
             level\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );
        assert!(
            log.contains("mid_level_barriers=1"),
            "a worker must report installing EXACTLY ONE mid-level barrier level — if this is \
             absent or 0 the flags never reached the worker and the run predates the mid-level barrier\n\
             stdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );
        assert!(
            log.contains("generations_per_step=2"),
            "the one-level split-pair graph must install a TWO-generation-per-step law (the \
             mid-level rendezvous plus the end-of-level one)\nstdout:\n{}\nstderr:\n{}",
            read_file(&stdout_path),
            read_file(&stderr_path)
        );

        let bag = assert_single_bag(&recordings);
        let reader = BagReader::open(&bag).expect("open bag");
        let (_msgs, completeness) = reader.recover_messages().expect("recover");
        assert!(
            completeness.is_finalized(),
            "the teardown must FINALIZE the bag, got {completeness:?}"
        );
        let (loss, health, topics_walked) = total_record_loss(&reader);
        // ANTI-VACUITY: `loss == 0` is the retry gate, and a reading that walked
        // NOTHING produces exactly that answer — which is how the `as_array()`
        // bug hid. Require the sum to have run over real entries, so a shape
        // change fails here rather than quietly certifying every bag healthy.
        // This graph produces two topics, so `>= 1` is a floor, not the count.
        let walked = topics_walked.unwrap_or_else(|| {
            panic!(
                "record_health.json is ABSENT on a bag this run just finalized, so the \
                 loss-free guard read nothing and cannot certify anything\nhealth: {health}"
            )
        });
        assert!(
            walked >= 1,
            "record_health.json carries ZERO topic entries, so the per-topic frames_lost sum \
             walked nothing and `loss == 0` says only that it looked nowhere\nhealth: {health}"
        );
        if loss == 0 {
            drop(reader);
            return (tmp, bag);
        }
        last_health = format!(
            "attempt {attempt}: record-side loss = {loss} over {walked} topic entries; \
             health = {health}"
        );
    }
    panic!(
        "could not produce a loss-free split-pair recording in {ATTEMPTS} attempts — REFUSING \
         to weaken the assert. Last health: {last_health}"
    );
}

/// Every recorded frame on the topic whose name ENDS WITH `suffix`, in bag
/// order, as raw bytes.
///
/// Matched by suffix because the topic carries the run's unique prefix, which
/// differs between the two recordings the determinism arm compares — the
/// PAYLOADS are the invariant, not the namespace.
fn frames_on_topic_suffix(bag: &Path, suffix: &str) -> Vec<Vec<u8>> {
    let reader = BagReader::open(bag).expect("open bag");
    let index = reader.user_message_index().expect("user message index");
    let (topic, spans) = index
        .iter()
        .find(|(t, _)| t.ends_with(suffix))
        .unwrap_or_else(|| {
            panic!(
                "no recorded topic ends with `{suffix}`; recorded: {:?}",
                index.keys().collect::<Vec<_>>()
            )
        });
    assert!(
        !spans.is_empty(),
        "topic `{topic}` was recorded with ZERO frames — the oracle would be vacuous"
    );
    spans.iter().map(|s| reader.frame(s).to_vec()).collect()
}

/// ARM 1 — the headline. The recorded split-pair run passes its OWN
/// re-execution.
///
/// `--verify` is load-bearing: a bare `--resim all` is NEUTRAL and exits 0 on
/// any completed re-execution, so it is precisely the invocation that could not
/// show this.
#[test]
#[serial]
fn a_split_same_level_non_trigger_recording_re_executes_to_exit_0() {
    let (tmp, bag) = record_split_pair_bag("v");
    let root = tmp.path();

    // Genuinely multi-rank: rank 1 exists and owns exactly the controller. A
    // single-rank bag would make the whole file vacuous.
    let reader = BagReader::open(&bag).expect("open bag");
    let (r1, ids1) = read_manifest(&reader, 1)
        .expect("rank-1 manifest present — the run MUST be multi-rank (k>=2)");
    assert_eq!(r1, 1);
    assert_eq!(
        ids1,
        vec!["controller".to_string()],
        "rank 1 is p1's subgraph — the cross-group consumer"
    );
    drop(reader);

    let report = root.join("resim.json");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "bag",
            "play",
            bag.to_str().unwrap(),
            "--resim",
            "all",
            "--verify",
            "--report",
            report.to_str().unwrap(),
        ])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .output()
        .expect("failed to spawn the cerulion binary");

    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    let report_txt =
        std::fs::read_to_string(&report).unwrap_or_else(|_| "<no report written>".to_string());

    assert_eq!(
        out.status.code(),
        Some(0),
        "a split same-level non-trigger mp recording must re-execute to EXIT 0 — this is the \
         shape that failed ~58% of runs before the mid-level barrier;\nstderr:\n{stderr}\nreport:\n{report_txt}"
    );

    // Non-vacuity, from the machine-readable report rather than the prose: a
    // trace was replayed AND at least one topic was byte-compared and matched.
    // Without these a bag carrying nothing would also exit 0.
    let json: serde_json::Value =
        serde_json::from_str(&report_txt).expect("resim report is valid JSON");
    assert_eq!(
        json["passed"], true,
        "the report records a clean PASS: {json}"
    );
    assert!(
        json["ticks_replayed"].as_u64().unwrap_or(0) > 0,
        "the replayed trace must be NON-EMPTY: {json}"
    );
    assert!(
        json["topics_passed"].as_u64().unwrap_or(0) > 0,
        "at least one topic was byte-compared and matched: {json}"
    );
    assert!(
        json["violations"]
            .as_array()
            .map(|a| a.is_empty())
            .unwrap_or(false),
        "no data violations, and the field must exist: {json}"
    );

    // The read-log verifier must have RUN and AGREED. This
    // graph's ONE read edge is the split pair's plain NON-TRIGGER `#[input]` —
    // a different edge class from the data-trigger chain the sibling
    // `mp_record_replay_e2e_test` carries — so the two e2es give the
    // promotion-evidence window independent coverage rather than one shape
    // twice. Measured here: `edges_compared: 1`.
    assert_read_log_verified_clean(&json, "the split same-level non-trigger mp re-execution");
}

/// ARM 2 — the property `--verify` alone cannot show, and the one a bag-format
/// fix could never have delivered: TWO live runs of the same graph produce
/// byte-identical controller frames.
///
/// Before the mid-level barrier the two recordings differed from EACH OTHER (the
/// decisive experiment measured 0-3 flipped frames per run), which is
/// nondeterminism in the LIVE run — "deterministic runtime, repeatable" broken
/// independently of replay.
///
/// The controller forwards what it read, so its frames ARE the pairing it
/// chose. Compared over the common prefix: the two runs are stopped by
/// independent SIGINTs, so their LENGTHS legitimately differ — what must not
/// differ is any frame they both contain.
#[test]
#[serial]
fn two_live_runs_of_the_split_pair_graph_record_identical_frames() {
    let (tmp_a, bag_a) = record_split_pair_bag("d1");
    let (tmp_b, bag_b) = record_split_pair_bag("d2");

    let a = frames_on_topic_suffix(&bag_a, "/controller/out");
    let b = frames_on_topic_suffix(&bag_b, "/controller/out");

    let common = a.len().min(b.len());
    // Anti-vacuity: a handful of frames could agree by luck. The 50ms-class
    // fixtures fire many times across the ~4s window; require a real sample.
    assert!(
        common >= 20,
        "too few comparable frames ({common}) for the determinism claim to mean anything — \
         run A recorded {}, run B {}",
        a.len(),
        b.len()
    );

    for (i, (fa, fb)) in a.iter().zip(b.iter()).take(common).enumerate() {
        assert_eq!(
            fa, fb,
            "frame {i} on /controller/out differs between two live runs of the SAME graph — \
             the mid-level barrier must make the same-level cross-group pairing deterministic, \
             so two runs cannot disagree (the measured failure was exactly this)"
        );
    }

    drop(tmp_a);
    drop(tmp_b);
}
