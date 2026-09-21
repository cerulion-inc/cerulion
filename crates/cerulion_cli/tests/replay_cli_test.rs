// SPDX-License-Identifier: AGPL-3.0-only
//! The re-execution verb's CLI-level exit-code contract.
//!
//! Spawns the REAL `cerulion` binary against bags built in a tempdir with
//! `cerulion_bag::BagWriter`, and asserts the process exit code EXACTLY plus a
//! discriminating stderr fragment for each class.
//!
//! **The verb is `cerulion bag play <bag> --resim all --verify`.** It drives
//! the replay engine and the 0–6 codes these tests pin;
//! there is no `cerulion replay <bag>` verb. The `--verify` half of
//! that spelling is what every arm below drives ([`RESIM_VERIFY_ARGV`]); the
//! NEUTRAL half (a bare `--resim all`, which claims no verdict) is pinned at the
//! end of the file, next to the arm proving `cerulion replay` is an
//! unrecognized subcommand. The exhaustive per-variant
//! coverage lives in `cerulion_cli_engine/tests/replay_gates_test.rs`; this file
//! proves the CLI wiring maps the engine's typed result to real process exit
//! codes and surfaces the messages on stderr.
//!
//! No iceoryx2 / no TransportManager on these paths — no `#[serial]` needed.
//! Unix-gated (the bag reader + the re-execution verb are `cfg(unix)`).

#![cfg(unix)]

use std::path::Path;
use std::process::{Child, Command, ExitStatus, Output, Stdio};

use cerulion_bag::{BagWriter, BagWriterConfig, TopicSchema};
use cerulion_core::trace_ring::{TraceRingRecord, RECORD_TYPE_FIRE};

const USER_TOPIC: &str = "/data";

/// The argv prefix that carries the verifier. `--verify` is
/// what asks for a verdict; without it the same run makes no claim and exits 0
/// (pinned by `neutral_resim_*` at the end of this file).
const RESIM_VERIFY_ARGV: &[&str] = &["bag", "play", "--resim", "all", "--verify"];

/// One attachment: (name, media_type, bytes).
type Att = (String, String, Vec<u8>);

fn att(name: &str, media_type: &str, bytes: Vec<u8>) -> Att {
    (name.to_string(), media_type.to_string(), bytes)
}

fn topics() -> Vec<TopicSchema> {
    vec![TopicSchema {
        topic: USER_TOPIC.into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: 0x0102_0304_0506_0708,
        wire_fixed_size: 8,
    }]
}

fn fire(node_idx: u32, step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: 1000 + step,
        duration_ns: 10,
        node_idx,
        global_level: 0,
        record_type: RECORD_TYPE_FIRE,
        reserved: 0,
    }
}

fn valid_graph_yaml() -> Vec<u8> {
    b"name: replaytest\nprefix: replaytest\nnodes:\n  - id: pub1\n    type: test_pub\n    \
      outputs:\n      - name: data\n        schema: test/Data\n"
        .to_vec()
}

fn manifest_bytes(rank: u32, node_ids: &[&str]) -> Vec<u8> {
    let ids: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    serde_json::to_vec(&serde_json::json!({
        "rank": rank,
        "generation": 0,
        "node_ids": ids,
    }))
    .unwrap()
}

fn replay_grade_attachments() -> Vec<Att> {
    vec![
        att("graph.yaml", "application/yaml", valid_graph_yaml()),
        att(
            "env.json",
            "application/json",
            br#"{"RUST_LOG":"info"}"#.to_vec(),
        ),
        att(
            "__cerulion/trace_manifest_rank0.json",
            "application/json",
            manifest_bytes(0, &["node0"]),
        ),
    ]
}

fn write_bag(path: &Path, trace: &[TraceRingRecord], attachments: &[Att]) {
    let payload = [0xABu8; 8];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics()).unwrap();
    w.write_chunk(|c| {
        c.write_message(USER_TOPIC, 0, 1000, 1000, &[&payload[..]])?;
        for (i, rec) in trace.iter().enumerate() {
            c.write_scheduler_trace(i as u32, rec.fire_time_ns, rec.fire_time_ns, rec)?;
        }
        Ok(())
    })
    .unwrap();
    for (name, media_type, data) in attachments {
        w.write_attachment(name, media_type, 0, 0, data).unwrap();
    }
    w.finalize().unwrap();
}

fn run_replay_bin(bag: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(bag)
        .output()
        .expect("failed to spawn cerulion binary")
}

fn stderr_of(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

#[test]
fn missing_file_exits_2() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("nope.mcap");
    let out = run_replay_bin(&path);
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr_of(&out));
    assert!(
        stderr_of(&out).contains("failed to read bag"),
        "stderr: {}",
        stderr_of(&out)
    );
}

#[test]
fn garbage_file_exits_2_with_not_mcap_message() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("garbage.mcap");
    std::fs::write(&path, b"definitely not an mcap file").unwrap();
    let out = run_replay_bin(&path);
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr_of(&out));
    assert!(
        stderr_of(&out).contains("not an MCAP bag"),
        "stderr: {}",
        stderr_of(&out)
    );
}

#[test]
fn empty_trace_bag_exits_2_and_names_where_the_reason_lives() {
    // A finalized, replay-grade bag whose scheduler-trace channel is empty (the
    // shape of EVERY real bag today) is refused, and the refusal points at the
    // artifacts that state WHY (`run.json`'s `trace_rings`, `record_coverage`)
    // instead of a blanket re-record hint. (This CLI
    // twin pins the same message as the engine-side tests.)
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("empty_trace.mcap");
    write_bag(&path, &[], &replay_grade_attachments());
    let out = run_replay_bin(&path);
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr_of(&out));
    let stderr = stderr_of(&out);
    assert!(
        stderr.contains("scheduler_trace") && stderr.contains("states why under `trace_rings`"),
        "stderr: {stderr}"
    );
}

#[test]
fn replay_grade_bag_reaches_engine_and_fails_node_load_exit_3() {
    // Once every gate passes, `cerulion replay` hands off to
    // the engine, which loads the graph's node cdylibs from the workspace build.
    // This bag's graph declares node type `test_pub`, which is not built in the
    // subprocess CWD (not a node workspace), so the engine fails at load time
    // with a NodeLoad (exit 3) — proving the gate→engine→NodeLoad wiring end to
    // end, without needing a compiled cdylib. (The byte-exact happy path is
    // covered at the engine level in `replay_engine_test.rs`, which can inject
    // in-process factories the subprocess CLI cannot.)
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("replay_grade.mcap");
    write_bag(
        &path,
        &[fire(0, 1), fire(0, 2)],
        &replay_grade_attachments(),
    );
    let out = run_replay_bin(&path);
    assert_eq!(out.status.code(), Some(3), "stderr: {}", stderr_of(&out));
    let stderr = stderr_of(&out);
    assert!(
        stderr.contains("failed to load node cdylib") && stderr.contains("test_pub"),
        "stderr: {stderr}"
    );
}

/// Run `cerulion bag play <bag> --resim all --verify --tolerance <tol>` over
/// the real binary.
fn run_replay_bin_with_tolerance(bag: &Path, tol: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(bag)
        .arg("--tolerance")
        .arg(tol)
        .output()
        .expect("failed to spawn cerulion binary")
}

#[test]
fn tolerance_unknown_key_exits_4_over_the_real_binary() {
    // The `--tolerance` flag threads through main.rs and a bad
    // document is exit 4 (ToleranceInvalid), pinning the real-binary flag wiring
    // + exit-code mapping.
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("tol_unknown.mcap");
    write_bag(&bag, &[fire(0, 1), fire(0, 2)], &replay_grade_attachments());
    let tol = dir.path().join("bad.yaml");
    std::fs::write(&tol, "defualt_metric:\n  kind: bit_exact\n").unwrap();
    let out = run_replay_bin_with_tolerance(&bag, &tol);
    assert_eq!(out.status.code(), Some(4), "stderr: {}", stderr_of(&out));
    assert!(
        stderr_of(&out).contains("tolerance") && stderr_of(&out).contains("defualt_metric"),
        "stderr: {}",
        stderr_of(&out)
    );
}

#[test]
fn tolerance_typoed_topic_exits_4_and_preempts_node_load() {
    // A typoed topic is exit 4 and PREEMPTS the node-load exit 3 this
    // replay-grade bag would otherwise hit — proving the gate fires before the
    // engine loads a cdylib. Produced topic is `/replaytest/pub1/data`.
    let dir = tempfile::tempdir().unwrap();
    let bag = dir.path().join("tol_topic.mcap");
    write_bag(&bag, &[fire(0, 1), fire(0, 2)], &replay_grade_attachments());
    let tol = dir.path().join("topic.yaml");
    std::fs::write(&tol, "topics:\n  /replaytest/pub1/dat: {}\n").unwrap();
    let out = run_replay_bin_with_tolerance(&bag, &tol);
    assert_eq!(out.status.code(), Some(4), "stderr: {}", stderr_of(&out));
    assert!(
        stderr_of(&out).contains("did you mean")
            && stderr_of(&out).contains("/replaytest/pub1/data"),
        "stderr: {}",
        stderr_of(&out)
    );
}

#[test]
fn duration_and_report_flags_are_accepted_and_do_not_change_a_gate_failure() {
    // The `--duration` / `--report` flags parse and thread through, but a NON-replay-grade bag
    // still fails its gate FIRST (exit 2) — the flags only affect a bag that
    // reaches the engine.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("empty_trace.mcap");
    write_bag(&path, &[], &replay_grade_attachments()); // empty trace → exit 2
    let report = dir.path().join("report.json");
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(&path)
        .arg("--duration")
        .arg("0.015")
        .arg("--report")
        .arg(&report)
        .output()
        .expect("failed to spawn cerulion binary");
    assert_eq!(out.status.code(), Some(2), "stderr: {}", stderr_of(&out));
    // The gate failed before the engine, so no report was written.
    assert!(!report.exists(), "no report on a pre-engine gate failure");
}

#[test]
fn no_bag_arg_is_a_clap_usage_error() {
    // A missing required positional is a CLAP PARSE error, which happens BEFORE
    // the 0–6 contract applies. clap emits its OWN stock exit code (2) for
    // usage errors — numerically equal to the "not-replay-grade" code, but a
    // distinct surface: the 0–6 contract governs SUCCESSFULLY PARSED
    // invocations, while this failure never enters `run_replay`. The "Usage"
    // banner + the `<BAG>` arg name in stderr distinguish it from a gate
    // failure (which never prints usage).
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .output()
        .expect("failed to spawn cerulion binary");
    assert_eq!(
        out.status.code(),
        Some(2),
        "clap uses exit 2 for usage errors; stderr: {}",
        stderr_of(&out)
    );
    let stderr = stderr_of(&out);
    assert!(stderr.contains("Usage"), "stderr: {stderr}");
    // clap renders a required positional's name in upper-case (`<BAG>`).
    assert!(
        stderr.to_lowercase().contains("bag"),
        "usage error must name the missing `bag` arg; stderr: {stderr}"
    );
}

#[test]
fn help_lists_the_whole_re_execution_surface() {
    // `bag play --help` must
    // document the bag arg and EVERY flag of BOTH halves — `--resim` and
    // `--verify` (the mode dial), `--report` / `--tolerance` (the two
    // verdict flags), and the playback/bag-time bounds
    // `--rate` / `--duration` / `--start-offset` — so the whole thing is
    // discoverable from one help text.
    //
    // `--max-ticks` is deliberately ABSENT and is asserted absent below: there is
    // no such flag (no alias, no shim, like the removed `replay` verb), so a stale
    // invocation must be clap's unknown-argument error rather than a silently
    // accepted no-op.
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "--help"])
        .output()
        .expect("failed to spawn cerulion binary");
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.to_lowercase().contains("bag"),
        "help must document the bag arg; stdout: {stdout}"
    );
    for flag in [
        "--resim",
        "--verify",
        "--report",
        "--tolerance",
        "--rate",
        "--duration",
        "--start-offset",
    ] {
        assert!(
            stdout.contains(flag),
            "`bag play --help` must document {flag}; stdout: {stdout}"
        );
    }
    assert!(
        !stdout.contains("--max-ticks"),
        "`--max-ticks` was deleted outright; help must not offer it: {stdout}"
    );
}

// ===========================================================================
// The SURFACE: the removed verb, the
// refusals, and the neutral-vs-`--verify` discriminator over the real binary.
// ===========================================================================

/// Run the real binary with `argv` and return its output.
fn run_bin(argv: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(argv)
        .output()
        .expect("failed to spawn cerulion binary")
}

#[test]
fn the_removed_replay_verb_is_an_unrecognized_subcommand() {
    // `replay` is unknown to clap: no variant parses it, not even a hidden one
    // that prints a migration notice.
    // The REAL-BINARY half of `cli.rs`'s parse pin — the unit test proves
    // the parser refuses, this proves the built process does.
    //
    // Every old invocation shape is driven, because a hidden variant has to be
    // declared `trailing_var_arg` + `allow_hyphen_values` so that all
    // of them reach it: a bare verb, a bag path, the two flag shapes a legacy
    // CI script would carry, and `--help` (which clap answers ITSELF, before
    // `main` runs — a hidden variant needs `disable_help_flag` to stop it printing
    // a removed verb's help page at exit 0).
    for argv in [
        vec!["replay"],
        vec!["replay", "somebag.mcap"],
        vec!["replay", "b.mcap", "--tolerance", "t.yaml"],
        vec!["replay", "b.mcap", "--max-ticks", "5", "--report", "r.json"],
        vec!["replay", "--help"],
    ] {
        let out = run_bin(&argv);
        let stderr = stderr_of(&out);
        // Exit 2 is clap's usage-error code, the one a caller of the removed
        // verb already handles.
        assert_eq!(out.status.code(), Some(2), "{argv:?} stderr: {stderr}");
        assert!(
            stderr.contains("unrecognized subcommand"),
            "{argv:?} must die as an unknown subcommand; stderr: {stderr}"
        );
        // And there must be NO migration notice, not merely none on one
        // path: a hidden diagnosing variant (or an alias added later) would still
        // exit 2, so the exit code alone cannot see it.
        assert!(
            !stderr.contains("has been REMOVED"),
            "{argv:?} must NOT print a migration notice; \
             stderr: {stderr}"
        );
    }
}

/// A malformed resim invocation is answered
/// BEFORE the login gate.
///
/// `bag play` is identity-gated: `command_needs_identity` exempts only `login`,
/// `completions` and the two internal `graph run-worker` / `run-gateway`
/// subprocess verbs. On a machine that has never signed in the gate answers
/// first, so this exact argv would exit 7 (auth) rather than 2 (usage), never
/// reaching the refusal that names `--resim`.
///
/// This is one of the few arms whose SUBJECT is the gate, so it removes the
/// value this repository's own runs carry and meets the gate as a user's
/// machine does. The isolated `CERULION_HOME` is what makes it real rather than
/// incidental: the gate reads local auth state, and on a developer desk that IS
/// signed in it would proceed and mask the ordering entirely. The account
/// service is a reserved dead port, so the arm reaches no network.
///
/// The anti-tautology half is in the same body: a WELL-FORMED resim under the
/// same gate must STILL authenticate (exit 7). Without it, a fix that simply
/// exempted `bag play` from the gate would pass, and that is the wrong fix,
/// since re-execution is a runtime verb.
#[test]
fn a_malformed_resim_is_refused_before_the_login_gate() {
    let home = tempfile::tempdir().expect("tempdir");

    // MALFORMED (`--verify` with no `--resim`): usage, not auth.
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "nonexistent.mcap", "--verify"])
        .env("HOME", home.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("CERULION_LOGIN_GATE")
        .output()
        .expect("failed to spawn cerulion binary");
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(2),
        "a malformed invocation must be a USAGE refusal (2), not an auth refusal (7); \
         stderr: {stderr}"
    );
    assert!(
        stderr.contains("`--verify` needs `--resim`"),
        "the refusal must name the actual problem; stderr: {stderr}"
    );
    assert!(
        !stderr.contains("not signed in"),
        "a malformed command line must not trigger the login flow; stderr: {stderr}"
    );

    // ANTI-TAUTOLOGY: a WELL-FORMED resim still needs identity. If this ever
    // returns 2, the usage check has exempted the verb from the gate.
    let gated = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "nonexistent.mcap", "--resim", "all"])
        .env("HOME", home.path())
        .env("CERULION_HOME", home.path())
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("CERULION_LOGIN_GATE")
        .output()
        .expect("failed to spawn cerulion binary");
    assert_eq!(
        gated.status.code(),
        Some(7),
        "a well-formed resim must still authenticate; stderr: {}",
        stderr_of(&gated)
    );
}

#[test]
fn the_removed_verb_is_not_offered_anywhere() {
    // A removal, not a deprecation: the name must not appear as a subcommand
    // row in `--help`. (It is `hide`den, which also keeps it out of shell
    // completion — pinned structurally in `completion_wiring_tests`.)
    let out = run_bin(&["--help"]);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", stderr_of(&out));
    let stdout = String::from_utf8_lossy(&out.stdout);
    for line in stdout.lines() {
        assert!(
            !line.trim_start().starts_with("replay "),
            "`replay` is still listed as a subcommand: {line}"
        );
    }
}

#[test]
fn a_node_subset_is_refused_over_the_real_binary() {
    // Partial resim is sequenced separately, so it is not built yet. The refusal
    // must say so by NAME rather than silently widening to the whole graph.
    let out = run_bin(&["bag", "play", "x.mcap", "--resim", "planner,controller"]);
    let stderr = stderr_of(&out);
    assert_eq!(out.status.code(), Some(2), "stderr: {stderr}");
    assert!(stderr.contains("not built yet"), "stderr: {stderr}");
    assert!(stderr.contains("--resim all"), "stderr: {stderr}");
}

#[test]
fn illegal_flag_combinations_are_refused_by_name_and_never_as_a_violation() {
    // The exit code is the load-bearing half. Under `--resim --verify`, 1 means
    // "your code diverged from the recording" — so a MISUSE reported as 1 would
    // make CI announce a regression that does not exist. Every refusal is 2,
    // the code clap already spends on a usage error in this binary.
    let cases: &[(&[&str], &str)] = &[
        (
            &["bag", "play", "x.mcap", "--resim", "all", "--rate", "2"],
            "--rate",
        ),
        (
            &["bag", "play", "x.mcap", "--resim", "all", "--loop"],
            "--loop",
        ),
        (
            &["bag", "play", "x.mcap", "--resim", "all", "--topics", "/a"],
            "--topics",
        ),
        (&["bag", "play", "x.mcap", "--verify"], "--verify"),
        (
            &[
                "bag",
                "play",
                "x.mcap",
                "--resim",
                "all",
                "--start-offset",
                "1",
            ],
            "--start-offset",
        ),
        (&["bag", "play", "x.mcap", "--report", "r.json"], "--report"),
        (
            &["bag", "play", "x.mcap", "--tolerance", "t.yaml"],
            "--tolerance",
        ),
        (
            &["bag", "play", "x.mcap", "--strict-state"],
            "--strict-state",
        ),
        (
            &[
                "bag", "play", "x.mcap", "--resim", "all", "--report", "r.json",
            ],
            "--report",
        ),
        (
            &[
                "bag",
                "play",
                "x.mcap",
                "--resim",
                "all",
                "--tolerance",
                "t.yaml",
            ],
            "--tolerance",
        ),
    ];
    for (argv, flag) in cases {
        let out = run_bin(argv);
        let stderr = stderr_of(&out);
        assert_eq!(
            out.status.code(),
            Some(2),
            "{argv:?} must be a usage error, never a verdict; stderr: {stderr}"
        );
        assert!(
            stderr.contains(flag),
            "{argv:?} refusal must name {flag}; stderr: {stderr}"
        );
        // A refusal that never reached the engine cannot have written anything.
        assert!(
            !std::path::Path::new("r.json").exists(),
            "a refused invocation must not write a report"
        );
    }
}

// ===========================================================================
// Full subprocess exit 0 / exit 1 pins over a REAL
// workspace + a REAL cdylib.
//
// A tempdir workspace is hand-built around the PREBUILT
// `test_node_macro_period_cdylib` fixture (a `#[cerulion_node(period_ms = 50)]`
// node with one `cmd: Vector3` output), copied in as `nodes/ticker` +
// `target/debug/libticker.*` — the `graph_record_e2e_test.rs` pattern. The
// replay-grade bag is CRAFTED by an in-test reference run that loads the SAME
// cdylib through `DylibNodeEntry` over an isolated per-test transport, then the
// REAL `cerulion bag play <bag> --resim all --verify` binary is spawned with
// cwd = the workspace root:
// its engine resolves the same cdylib as the candidate and replays on the
// GLOBAL transport namespace.
//
// `#[serial]`: the parent's reference runs load a cdylib (the process-global
// NODES registry) and the subprocesses use the GLOBAL iceoryx2 namespace —
// unique per-run topic prefixes make collisions structurally unlikely, but the
// serialization keeps the two subprocess transports from ever overlapping.
// ===========================================================================

use std::io::Read as _;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::trace_ring::RECORD_TYPE_STEP_BOUNDARY;
use cerulion_core::wire::WireHeader;
use cerulion_core::{Clock, DylibNodeEntry, GraphRuntime, NodeEntry, VirtualClock};
use serial_test::serial;

/// Platform cdylib file name (`lib{name}.dylib` / `lib{name}.so`).
fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

/// The prebuilt fixture cdylib. PANICS with the build instruction if missing
/// (the repo's fixture pattern).
fn fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_cdylib")
}

/// The PERTURBED twin fixture cdylib (`test_node_macro_period_perturbed_cdylib`
/// — same `#[cerulion_node(period_ms = 50)]` shape as `fixture_cdylib`, but its
/// tick publishes `cmd.x = 1000.0` instead of `0.0`). Copied over `libticker.*`
/// AFTER the matching replay, this is the CI stand-in for "rebuild the node with
/// a changed constant" — the real-bag e2e proves the SAME recorded bag now
/// exits 1 (data violation) against the perturbed candidate. PANICS with the
/// build instruction if missing (the repo's fixture pattern).
fn perturbed_fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_perturbed_cdylib")
}

/// The PANICKING twin fixture cdylib (`test_node_macro_period_panic_cdylib` —
/// same node shape as `fixture_cdylib`, but its tick PANICS from its third
/// fire). Copied over `libticker.*` for the e2e's replay #3, this is the CI
/// stand-in for "the candidate build crashes mid-replay" — the same recorded
/// bag must exit 3 (node failure: panic-class execution, the exit-3
/// widening). PANICS with the build instruction if missing.
fn panic_twin_fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_panic_cdylib")
}

/// The DELIBERATELY nondeterministic fixture cdylib
/// (`test_node_nondeterministic_cdylib` — a `#[cerulion_node(period_ms = 50)]`
/// whose tick folds `SystemTime::now()` + a `RandomState` seed into `cmd.x`).
/// Same output shape as `fixture_cdylib`, so it drops into the replay workspace
/// as `libticker.*`. Recording then replaying it against ITSELF exits 1: the
/// fire schedule reproduces exactly but each run mints a fresh payload value.
/// PANICS with the build instruction if missing (the repo's fixture pattern).
fn nondet_fixture_cdylib() -> PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_nondeterministic_cdylib")
}

/// Hand-build a minimal replay workspace in `root`: a `[workspace]` Cargo.toml,
/// graphs/demo.yaml (one `ticker` period node producing `/{prefix}/ticker/cmd`),
/// nodes/ticker/src/lib.rs (fixture source copy, for the metadata walkers), and
/// target/debug/libticker.* (the PREBUILT fixture cdylib under the node-type
/// name the resolver looks for). `prefix` must be unique per reference run —
/// the subprocess replays on the global iceoryx2 namespace.
fn build_replay_workspace(root: &Path, prefix: &str) -> String {
    build_replay_workspace_from(
        root,
        prefix,
        fixture_cdylib(),
        "test_fixtures/test_node_macro_period_cdylib/src/lib.rs",
    )
}

/// [`build_replay_workspace`] with an explicit `ticker` cdylib + source (both a
/// `#[cerulion_node(period_ms = 50)]` producing `cmd: geometry_msgs/Vector3`).
/// Lets a test swap in a DIFFERENT node body (e.g. the nondeterministic fixture)
/// under the same `ticker` node-type name the resolver + graph.yaml reference.
fn build_replay_workspace_from(
    root: &Path,
    prefix: &str,
    cdylib: PathBuf,
    src_rel: &str,
) -> String {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/ticker/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let graph_yaml = format!(
        "name: demo\nprefix: {prefix}\nnodes:\n- id: ticker\n  type: ticker\n  inputs: []\n  \
         outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n"
    );
    std::fs::write(root.join("graphs/demo.yaml"), &graph_yaml).unwrap();
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join(src_rel);
    std::fs::copy(&fixture_src, root.join("nodes/ticker/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(cdylib, root.join("target/debug").join(dylib_file("ticker")))
        .expect("copy fixture cdylib");
    graph_yaml
}

/// Run the reference graph (the workspace's ticker cdylib via `DylibNodeEntry`)
/// on an isolated transport for `steps` × 50 ms, capturing the produced frames
/// and the scheduler trace (one STEP_BOUNDARY per step, then that step's FIRE
/// records — the production push order). Returns (frames, merged trace).
fn reference_run_cdylib(
    root: &Path,
    prefix: &str,
    graph_yaml: &str,
    steps: usize,
) -> (Vec<Vec<u8>>, Vec<TraceRingRecord>) {
    let config = cerulion_core::graph::parse_graph(graph_yaml).expect("workspace graph parses");
    let cdylib = root.join("target/debug").join(dylib_file("ticker"));
    let entry = DylibNodeEntry::load(&cdylib).expect("load the ticker cdylib");
    let mut factories: indexmap::IndexMap<String, Box<dyn NodeEntry>> = indexmap::IndexMap::new();
    factories.insert("ticker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock.clone(), 16)
        .expect("reference cdylib runtime builds");
    let mgr = runtime
        .test_transport()
        .expect("test transport parked")
        .clone();
    let topic = format!("/{prefix}/ticker/cmd");
    let mut sub = mgr
        .create_data_only_subscriber(&topic)
        .expect("capture sub opens on the graph-created service");

    let mut frames: Vec<Vec<u8>> = Vec::new();
    let mut boundaries: Vec<TraceRingRecord> = Vec::new();
    for step in 0..steps {
        runtime.step(Duration::from_millis(50));
        boundaries.push(TraceRingRecord {
            step: step as u64,
            fire_time_ns: clock.now_ns(),
            duration_ns: 0,
            node_idx: 0,
            global_level: 0,
            record_type: RECORD_TYPE_STEP_BOUNDARY,
            reserved: 0,
        });
        let budget = sub.max_borrowed_samples().max(1);
        let mut scratch = Vec::new();
        loop {
            scratch.clear();
            let n = sub.drain_owned(budget, &mut scratch).expect("drain");
            for s in scratch.drain(..) {
                frames.push(s.payload().to_vec());
            }
            if n < budget {
                break;
            }
        }
    }
    let fires: Vec<TraceRingRecord> = runtime
        .trace()
        .iter()
        .map(|e| TraceRingRecord {
            step: e.step,
            fire_time_ns: e.fire_time_ns,
            duration_ns: 0,
            node_idx: 0, // single-node graph: "ticker" is manifest index 0.
            global_level: e.global_level as u32,
            record_type: RECORD_TYPE_FIRE,
            reserved: 0,
        })
        .collect();
    runtime.shutdown();

    let mut trace: Vec<TraceRingRecord> = Vec::new();
    for b in &boundaries {
        trace.push(*b);
        trace.extend(fires.iter().filter(|f| f.step == b.step).copied());
    }
    (frames, trace)
}

/// Write a replay-grade bag for the reference run: the topic channel with its
/// frames (seq/ts from each frame's wire header), the merged trace, and the
/// graph.yaml and rank-0 manifest attachments.
fn write_replay_bag(
    path: &Path,
    prefix: &str,
    graph_yaml: &str,
    frames: &[Vec<u8>],
    trace: &[TraceRingRecord],
) {
    write_replay_bag_with_extra(path, prefix, graph_yaml, frames, trace, &[]);
}

/// As [`write_replay_bag`], plus arbitrary EXTRA attachments — used by the
/// recorder.json subprocess pins to craft bags with a mismatched /
/// malformed `__cerulion/recorder.json`.
fn write_replay_bag_with_extra(
    path: &Path,
    prefix: &str,
    graph_yaml: &str,
    frames: &[Vec<u8>],
    trace: &[TraceRingRecord],
    extra: &[Att],
) {
    let topic = format!("/{prefix}/ticker/cmd");
    let topics = vec![TopicSchema {
        topic: topic.clone(),
        schema_name: "geometry_msgs/Vector3".into(),
        schema_hash: 0,
        wire_fixed_size: 24,
    }];
    let mut w = BagWriter::create(path, BagWriterConfig::default(), &topics).unwrap();
    w.write_chunk(|c| {
        for frame in frames {
            let (seq, ts) = WireHeader::read_from_buf(frame)
                .map(|h| (h.sequence, h.timestamp_ns))
                .unwrap_or((0, 0));
            c.write_message(&topic, seq, ts, ts, &[&frame[..]])?;
        }
        for (i, rec) in trace.iter().enumerate() {
            c.write_scheduler_trace(i as u32, rec.fire_time_ns, rec.fire_time_ns, rec)?;
        }
        Ok(())
    })
    .unwrap();
    w.write_attachment(
        "graph.yaml",
        "application/yaml",
        0,
        0,
        graph_yaml.as_bytes(),
    )
    .unwrap();
    w.write_attachment(
        "__cerulion/trace_manifest_rank0.json",
        "application/json",
        0,
        0,
        &manifest_bytes(0, &["ticker"]),
    )
    .unwrap();
    for (name, media_type, data) in extra {
        w.write_attachment(name, media_type, 0, 0, data).unwrap();
    }
    w.finalize().unwrap();
}

/// Spawn `cerulion bag play <bag> --resim all --verify` with cwd = the
/// workspace root (the engine
/// resolves the workspace's ticker cdylib as the candidate).
fn run_replay_bin_in_workspace(root: &Path, bag: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(bag)
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target — the workspace copy).
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("failed to spawn cerulion binary")
}

/// As [`run_replay_bin_in_workspace`] but WITHOUT `--verify` — a bare
/// `--resim all`, the neutral mode that claims no verdict. `extra` appends any
/// further flags.
fn run_resim_bin_in_workspace(root: &Path, bag: &Path, extra: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["bag", "play", "--resim", "all"])
        .arg(bag)
        .args(extra)
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target — the workspace copy).
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("failed to spawn cerulion binary")
}

/// Overwrite the workspace's `ticker` cdylib with the PANICKING twin
/// (same shape; its tick panics from the 3rd fire) — the CI stand-in for "the
/// candidate build crashes mid-run".
fn swap_in_panicking_cdylib(root: &Path) {
    std::fs::copy(
        panic_twin_fixture_cdylib(),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("overwrite ticker cdylib with the panicking twin");
}

/// As [`run_replay_bin_in_workspace`], plus `--report <path>` so the caller can
/// parse the machine-readable `ReplayOutcome` JSON (asserting `passed` +
/// `ticks_replayed > 0` on the real-bag e2e — a text-free proof the recorded
/// trace was non-empty).
fn run_replay_bin_in_workspace_with_report(root: &Path, bag: &Path, report: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(bag)
        .arg("--report")
        .arg(report)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("failed to spawn cerulion binary")
}

// ===========================================================================
// Real-bag e2e: subprocess-management helpers PORTED from
// `graph_record_e2e_test.rs` (test binaries can't share code across crates, so
// these are copies — keep them in sync with the origin file). They drive the
// PROVEN record harness: spawn `graph run --record`, wait for bagd to create
// the bag, let it accumulate ticks, SIGINT the parent (bagd finalizes on the
// orchestrated teardown), and guard both the parent child AND the bagd
// grandchild so a failing test leaks no processes.
// ===========================================================================

/// SIGKILL + reap on drop so a panicking test never leaks the child.
/// (Ported from `graph_record_e2e_test.rs::ChildGuard`.)
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// The bagd GRANDCHILD leak guard. `ChildGuard` owns only the graph-run parent;
/// bagd runs in its OWN process group (`process_group(0)`), so killing the
/// parent on a mid-window panic ORPHANS bagd. On Drop this pgrep-discovers the
/// bagd spawned with THIS run's cmdline and SIGKILLs its process group.
/// Best-effort; init reaps the orphan. (Ported from
/// `graph_record_e2e_test.rs::BagdGuard`, HARDENED: the needle carries the
/// run's ABSOLUTE recordings path — unique per tempdir — so a Drop here can
/// never match a sibling binary's actively-recording bagd. The origin file's
/// relative `--out recordings/demo_` needle was unique only while that file
/// was its sole user; this file passes `--record=<abs>` precisely so the
/// grandchild's cmdline is distinguishable.)
struct BagdGuard {
    needle: String,
}
impl Drop for BagdGuard {
    fn drop(&mut self) {
        if let Ok(out) = Command::new("pgrep").args(["-f", &self.needle]).output() {
            for pid in String::from_utf8_lossy(&out.stdout)
                .lines()
                .filter_map(|l| l.trim().parse::<i32>().ok())
            {
                // SAFETY: killpg(2) on the grandchild's own process group
                // (pgid == pid — spawned with process_group(0)); no memory is
                // touched. Best-effort: ESRCH after a clean exit is a no-op.
                unsafe {
                    libc::killpg(pid, libc::SIGKILL);
                }
            }
        }
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
/// (Ported from `graph_record_e2e_test.rs::wait_bounded`.)
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

/// Block until the recordings dir contains a `.mcap` (bagd created the bag — the
/// handshake completed) or `timeout` elapses. Returns the bag path.
/// (Ported from `graph_record_e2e_test.rs::wait_for_bag`.)
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

fn read_file(p: &Path) -> String {
    let mut s = String::new();
    if let Ok(mut f) = std::fs::File::open(p) {
        let _ = f.read_to_string(&mut s);
    }
    s
}

#[test]
#[serial]
fn subprocess_replay_of_matching_workspace_exits_0_with_pass() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}a", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let steps = 3;
    let (frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, steps);
    // Hand oracle: period 50 ms driven at 50 ms steps fires once per step, and
    // the fixture writes cmd.x = 0.0 — three 56-byte frames stamped 50/100/150.
    assert_eq!(frames.len(), steps, "one frame per step");
    for (k, frame) in frames.iter().enumerate() {
        let h = WireHeader::read_from_buf(frame).expect("frame header");
        assert_eq!(h.timestamp_ns, (k as u64 + 1) * 50_000_000);
    }

    let bag = root.join("reference.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(0),
        "byte-exact replay must exit 0; stderr: {stderr}"
    );
    assert!(
        stderr.contains("replay PASS"),
        "the PASS verdict is printed to stderr: {stderr}"
    );
}

#[test]
#[serial]
fn subprocess_replay_of_flipped_bag_exits_1_with_regression_report() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}b", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (mut frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    // Flip one payload byte of the middle frame — the recording now disagrees
    // with what the (unchanged) candidate deterministically produces.
    frames[1][WireHeader::SIZE] ^= 0xFF;

    let bag = root.join("flipped.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(1),
        "a byte divergence must exit 1 (EXIT_VIOLATION); stderr: {stderr}"
    );
    assert!(
        stderr.contains("FRAME-CONTENT DIVERGENCE") && stderr.contains("byte-mismatch"),
        "the regression verdict names the divergence class on stderr: {stderr}"
    );
}

/// **THE headline discriminator**: ONE divergent
/// bag, two modes, two answers, over the real binary and a real cdylib.
///
/// `--verify` calls the byte divergence a violation (exit 1, the arm above).
/// A bare `--resim all` re-executes the SAME bag against the SAME workspace and
/// exits **0**, because divergence is what a no-verdict resim is FOR — the
/// agent-iteration loop (edit → build → resim → score) consumes exactly this.
///
/// Both halves live in one test on purpose. Separately, each is satisfiable by
/// a mode that always answers the same thing; together they can only pass if
/// `--verify` is genuinely the switch. The anti-tautology third leg is
/// `subprocess_replay_of_matching_workspace_exits_0_with_pass`'s neutral twin
/// below: a CLEAN run must be 0 in both modes, so neutral mode is not merely
/// "always 0" — it agrees with the verifier everywhere except on a comparison.
///
/// Prerequisite: `cargo build -p test_node_macro_period_cdylib`.
#[test]
#[serial]
fn neutral_resim_exits_0_on_the_exact_bag_verify_calls_a_violation() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}n", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (mut frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    // The same single flipped payload byte the `--verify` arm above uses.
    frames[1][WireHeader::SIZE] ^= 0xFF;

    let bag = root.join("flipped.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    // Leg 1 — VERIFY: the divergence is a verdict.
    let verified = run_replay_bin_in_workspace(root, &bag);
    assert_eq!(
        verified.status.code(),
        Some(1),
        "control leg: `--verify` must still call this a violation; stderr: {}",
        stderr_of(&verified)
    );

    // Leg 2 — NEUTRAL: the same divergence, no verdict, exit 0.
    let neutral = run_resim_bin_in_workspace(root, &bag, &[]);
    let stderr = stderr_of(&neutral);
    assert_eq!(
        neutral.status.code(),
        Some(0),
        "a bare `--resim all` claims nothing about matching the recording; stderr: {stderr}"
    );
    // It reports the divergence as an OBSERVATION — silence would leave the
    // operator with no signal from a re-execution they asked for.
    assert!(
        stderr.contains("no verdict"),
        "neutral mode must say it made no verdict; stderr: {stderr}"
    );
    assert!(
        stderr.contains("differ from the recording"),
        "neutral mode must still report the divergence it saw; stderr: {stderr}"
    );
    assert!(
        stderr.contains("--verify"),
        "neutral mode must name how to ask for a verdict; stderr: {stderr}"
    );
    // The verifier's vocabulary must NOT leak into a mode that made no claim:
    // a divergence BANNER above an exit 0 is worse than silence.
    //
    // The whole banner vocabulary, not just this bag's class. Two reasons the
    // narrow form is not enough. (1) A byte mismatch renders
    // `FRAME-CONTENT DIVERGENCE`, but the headers are DERIVED
    // (`DivergenceClass::header()`), so a neutral renderer that leaked a
    // SIBLING class's banner — or leaked one on a different bag shape — would pass
    // an arm whose docstring claims the verdict vocabulary does not leak at
    // all. (2) The two LEGACY spellings (`REPLAY REGRESSION` and
    // `TRACE DIVERGENCE`) appear nowhere in production source,
    // so nothing in the tree can emit them —
    // which is exactly when a negative check is cheapest to keep and a
    // re-introduction is loudest to catch. `replay_engine_test.rs`
    // keeps the same pair for the same reason.
    for banner in [
        "FRAME-CONTENT DIVERGENCE",
        "FIRE-SCHEDULE DIVERGENCE",
        "EDGE-READ DIVERGENCE",
        // Legacy, pre-fork-3:
        "REPLAY REGRESSION",
        "TRACE DIVERGENCE",
    ] {
        assert!(
            !stderr.contains(banner),
            "neutral mode must not render the verdict banner {banner:?}; stderr: {stderr}"
        );
    }
}

/// The anti-tautology leg for the discriminator above: on a CLEAN
/// bag, neutral mode agrees with `--verify` (both exit 0). Without this, a
/// neutral mode hardcoded to `exit 0` would pass the discriminator.
///
/// Prerequisite: `cargo build -p test_node_macro_period_cdylib`.
#[test]
#[serial]
fn neutral_resim_agrees_with_verify_on_a_clean_bag() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}nc", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    let bag = root.join("clean.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    let verified = run_replay_bin_in_workspace(root, &bag);
    assert_eq!(
        verified.status.code(),
        Some(0),
        "control: an unmodified bag must verify; stderr: {}",
        stderr_of(&verified)
    );

    let neutral = run_resim_bin_in_workspace(root, &bag, &[]);
    let stderr = stderr_of(&neutral);
    assert_eq!(neutral.status.code(), Some(0), "stderr: {stderr}");
    assert!(
        stderr.contains("match the recording byte-for-byte"),
        "neutral mode must report that nothing diverged; stderr: {stderr}"
    );
}

/// `--strict-state` crossed the rename and
/// is accepted in BOTH resim modes.
///
/// The flag is a re-execution PRECONDITION (`enforce_strict_state` runs at the
/// restore phase, before the first step; its failure is
/// `ReplayError::StateRestore` → exit 2), not a claim about the recording — so
/// it is NOT verify-gated, and a neutral `--resim all --strict-state` must be a
/// legal invocation rather than a refusal.
///
/// The oracle is INERTNESS, which is the flag's own documented behaviour on a bag
/// that begins at step 0: nothing is restored there, so the flag cannot change
/// the verdict. Asserting the verdict is UNCHANGED proves the flag reached the
/// engine and was honoured, without re-litigating the refusal semantics — those
/// are engine-pinned twice already
/// (`replay_engine_test::strict_state_refuses_a_node_with_no_state_…` and
/// `…_judges_the_executed_nodes_and_only_those`), and a CLI-level refusal arm
/// would need a mid-run checkpoint bag this file has no harness for.
///
/// Prerequisite: `cargo build -p test_node_macro_period_cdylib`.
#[test]
#[serial]
fn strict_state_is_accepted_in_both_resim_modes_and_is_inert_from_step_zero() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}ss", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    let bag = root.join("strict.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    // VERIFY mode: accepted, and the verdict is unchanged (exit 0 — the control
    // for this same bag shape without the flag is
    // `neutral_resim_agrees_with_verify_on_a_clean_bag`).
    let strict_verify = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(RESIM_VERIFY_ARGV)
        .arg(&bag)
        .arg("--strict-state")
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("failed to spawn the cerulion binary");
    assert_eq!(
        strict_verify.status.code(),
        Some(0),
        "`--strict-state` is inert on a from-start bag and must not change the \
         verify verdict; stderr: {}",
        stderr_of(&strict_verify)
    );

    // NEUTRAL mode: legal, NOT a `--verify`-gated refusal.
    let strict_neutral = run_resim_bin_in_workspace(root, &bag, &["--strict-state"]);
    let stderr = stderr_of(&strict_neutral);
    assert_eq!(
        strict_neutral.status.code(),
        Some(0),
        "`--strict-state` must be legal WITHOUT `--verify` — it is a \
         precondition on the run, not a claim about the recording; stderr: {stderr}"
    );
    assert!(
        !stderr.contains("needs `--verify`"),
        "`--strict-state` must not be refused as a verify-only flag; stderr: {stderr}"
    );
    assert!(
        stderr.contains("no verdict"),
        "the neutral summary still renders under `--strict-state`; stderr: {stderr}"
    );
}

/// A bag that cannot be re-executed at all is loud in neutral mode
/// too. An empty scheduler trace is not-replay-grade (exit 2), and a neutral
/// resim that swallowed it would report success for a run that never happened.
#[test]
fn neutral_resim_still_refuses_a_bag_it_cannot_re_execute() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("empty_trace.mcap");
    write_bag(&path, &[], &replay_grade_attachments());
    let out = run_bin(&["bag", "play", path.to_str().unwrap(), "--resim", "all"]);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(2),
        "neutral mode does not swallow a not-replay-grade bag; stderr: {stderr}"
    );
    assert!(
        !stderr.contains("no verdict"),
        "a refused bag never reaches the neutral summary; stderr: {stderr}"
    );
}

/// The moat-credibility PRODUCTION-PATH pin: the diff CATCHES
/// nondeterminism as an exit-1 data violation through the REAL binary +
/// `DylibNodeEntry`. The `ticker` node is the DELIBERATELY nondeterministic
/// fixture (`test_node_nondeterministic_cdylib`): the recording captures run-A's
/// freshly-minted payload, then `cerulion replay` re-executes the SAME cdylib in
/// a subprocess, which mints run-B's value on the identical fire schedule → a
/// byte mismatch on the node's topic → exit 1 (NOT exit 6: the exit-code
/// precedence 3 > 6 > 1 means an exit of 1 PROVES no structural trace divergence
/// fired — the schedule reproduced, only the DATA differs). The verdict is
/// deterministic (record and replay independently mint 64-bit folds of an
/// advancing clock — a collision is ~1-in-2^64 and time monotonically advances).
///
/// This is the production twin of the engine-suite structured pin
/// `nondeterministic_replay_is_a_byte_mismatch_not_a_trace_divergence`; it also
/// proves the E-2 schema-drift preflight does NOT false-fire on a same-schema
/// nondeterministic bag (record + replay share the cdylib → matching schema hash).
///
/// Prerequisite: `cargo build -p test_node_nondeterministic_cdylib`.
#[test]
#[serial]
fn subprocess_replay_of_nondeterministic_node_exits_1_with_regression() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}nd", std::process::id());
    let graph_yaml = build_replay_workspace_from(
        root,
        &prefix,
        nondet_fixture_cdylib(),
        "test_fixtures/test_node_nondeterministic_cdylib/src/lib.rs",
    );

    // Reference run captures run-A's nondeterministic payloads on the identical
    // fire schedule (period 50 ms at 50 ms steps → one frame per step).
    let (frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    assert_eq!(frames.len(), 3, "one frame per step");

    let bag = root.join("nondet.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    // Replay re-executes the SAME cdylib → run-B mints different payload bytes.
    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(1),
        "a nondeterministic payload must exit 1 (data violation, NOT exit-6 structural); \
         stderr: {stderr}"
    );
    assert!(
        stderr.contains("FRAME-CONTENT DIVERGENCE") && stderr.contains("byte-mismatch"),
        "the regression verdict names the byte-mismatch class on stderr: {stderr}"
    );
    assert!(
        stderr.contains(&format!("/{prefix}/ticker/cmd")),
        "the violation is on the node's topic: {stderr}"
    );
}

/// A structural trace divergence exits 6. Mirrors the exit-1
/// flipped-bag pin, but perturbs the SCHEDULE (a dropped FIRE record) while
/// leaving every frame intact — so the data diff is clean and only the exit-6
/// structural detector fires. Reuses the same workspace + reference-run + bag
/// harness as the exit-0/exit-1 pins.
#[test]
#[serial]
fn subprocess_replay_of_thinned_trace_bag_exits_6_with_trace_divergence() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}c", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (frames, mut trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    // Drop ONE ticker FIRE record (its StepBoundary + all frames stay): the
    // recorded schedule is now thinner than the deterministic replay produces →
    // a structural divergence (exit 6), the DATA diff still clean.
    let i = trace
        .iter()
        .position(|r| r.record_type == RECORD_TYPE_FIRE && r.step == 1)
        .expect("a step-1 ticker fire to drop");
    trace.remove(i);

    let bag = root.join("thinned.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(6),
        "a structural trace divergence must exit 6 (EXIT_TRACE_DIVERGENCE); stderr: {stderr}"
    );
    assert!(
        stderr.contains("FIRE-SCHEDULE DIVERGENCE"),
        "the divergence verdict is printed on stderr: {stderr}"
    );
}

/// The REAL-bag end-to-end pin — the FIRST closed record→replay loop
/// over a PRODUCTION-recorded bag. Record a live run via
/// `cerulion graph run --record` (the real `graph run` process + bagd recorder, NOT a
/// hand-built bag), then `cerulion replay` the produced bag THREE times from
/// ONE recording:
///
///   1. against the MATCHING candidate (the same `libticker.*` that recorded)
///      → byte-exact PASS (exit 0), and the `--report` JSON proves the trace was
///      non-empty (`passed == true`, `ticks_replayed > 0`);
///   2. against a PERTURBED candidate (`libticker.*` overwritten with
///      `test_node_macro_period_perturbed_cdylib`, whose tick publishes a
///      different constant on the SAME fire schedule) → a data violation, exit 1
///      with the `FRAME-CONTENT DIVERGENCE` / `byte-mismatch` markers;
///   3. against a PANICKING candidate (`libticker.*` overwritten with
///      `test_node_macro_period_panic_cdylib`, whose tick PANICS from its third
///      fire) → a panic-class node failure, exit 3 (the exit-3 widening: node
///      failure = LOAD or EXECUTION) with the `NODE FAILURE` stderr marker —
///      PREEMPTING the downstream missing-frame violations (3 > 6 > 1, the
///      root-cause-over-symptoms precedence, pinned here at the process-exit
///      layer).
///
/// Recording once + replaying three times pins that the SAME bag distinguishes
/// candidate builds — the demo story.
///
/// `graph run --record`
/// writes FIRE + per-step StepBoundary trace records, so the boundary-driven
/// clock re-advance reproduces the recorded wire timestamps byte-exact
/// (a bag whose trace channel is empty is refused by replay with
/// `BagNoSchedulerTrace`, exit 2).
///
/// Prerequisite (the repo's fixture pattern): `cargo build -p
/// test_node_macro_period_cdylib -p test_node_macro_period_perturbed_cdylib \
/// -p test_node_macro_period_panic_cdylib` (or a bare `cargo build
/// --workspace`) — the helpers PANIC with the exact build instruction if any
/// artifact is missing.
///
/// Runs the record + all three replays on the GLOBAL iceoryx2 namespace (the
/// production `graph run` path has no isolation seam — that IS the point of a
/// closed-loop e2e); `#[serial]` + a unique per-run prefix keep it safe.
#[test]
#[serial]
fn real_recorded_bag_replays_end_to_end() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}real", std::process::id());
    let _graph_yaml = build_replay_workspace(root, &prefix);

    // ---- Record a live run via the PROVEN graph_record_e2e_test harness ----
    // stdout/stderr → files (readable while the child runs, no pipe deadlock).
    let run_stdout = root.join("run.stdout");
    let run_stderr = root.join("run.stderr");
    // ABSOLUTE --record path: bagd's cmdline then carries the per-run tempdir,
    // making the BagdGuard pgrep needle unique to THIS run (never a sibling
    // binary's bagd).
    let recordings = root.join("recordings");
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "graph",
            "run",
            "demo",
            &format!("--record={}", recordings.display()),
            // The replay leg needs the WALL-FAITHFUL
            // single-process bag (replay's boundary-driven clock re-advance). Without
            // the flag, the mp default would derive an in-memory partition on
            // this no-TTY subprocess and record the mp bag instead.
            "--single-process",
        ])
        .current_dir(root)
        // Deterministic cdylib lookup (<ws>/target) + info-level lifecycle logs.
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        .stdout(Stdio::from(std::fs::File::create(&run_stdout).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&run_stderr).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run --record");
    let mut guard = ChildGuard(child);
    // Kill any orphaned bagd grandchild on ANY exit (panic-safe). The needle's
    // absolute path scopes it to THIS run's bagd only.
    let _bagd_guard = BagdGuard {
        needle: format!("bagd --out {}/demo_", recordings.display()),
    };

    let bag = wait_for_bag(&recordings, Duration::from_secs(30))
        .expect("bagd never created the bag (handshake failed?)");
    // Let the 50ms-period ticker fire a healthy batch of ticks before shutdown.
    std::thread::sleep(Duration::from_millis(1500));

    // The production Ctrl-C: SIGINT the PARENT only (bagd is in its own process
    // group; the `graph run` parent drives its final drain + bag finalize). A killed
    // recorder would yield a non-finalized bag (replay exit 2).
    unsafe {
        libc::kill(guard.0.id() as libc::pid_t, libc::SIGINT);
    }
    let status = wait_bounded(&mut guard.0, Duration::from_secs(40))
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "the record run (Ctrl-C) must exit 0, got {status:?}; run.stderr:\n{}",
        read_file(&run_stderr)
    );

    // ---- Replay #1: the MATCHING candidate → byte-exact PASS (exit 0) ----
    let report = root.join("replay_report.json");
    let out = run_replay_bin_in_workspace_with_report(root, &bag, &report);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(0),
        "a real recorded bag must replay byte-exact against the recording candidate; \
         stderr: {stderr}"
    );
    assert!(
        stderr.contains("replay PASS"),
        "the PASS verdict is printed to stderr: {stderr}"
    );
    // Non-empty-trace proof via the machine-readable report (text-free): a
    // real bag carries a fed trace.
    let report_json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&report).expect("replay report written"))
            .expect("replay report is valid JSON");
    assert_eq!(
        report_json["passed"], true,
        "the report records a clean PASS: {report_json}"
    );
    assert!(
        report_json["ticks_replayed"].as_u64().unwrap_or(0) > 0,
        "the replayed trace must be NON-EMPTY (real bags carry the fed scheduler \
         trace): {report_json}"
    );
    // Non-VACUITY, locally: `ticks_replayed` counts replayed steps, not
    // compared frames — a zero-frame recording replayed by a zero-frame
    // candidate would pass every assert above without one byte compared. At
    // least one topic must have been byte-compared AND matched.
    assert!(
        report_json["topics_passed"].as_u64().unwrap_or(0) > 0,
        "at least one topic was byte-compared and matched (non-vacuous diff): {report_json}"
    );
    // This arm contributes NO evidence to the read-log
    // verifier's promotion window, and the pin below is what says so out loud
    // rather than leaving the gap to be rediscovered.
    //
    // The design intent was to assert `read_log.status == "verified_clean"` on
    // BOTH production record→replay e2es, but it is STRUCTURALLY UNREACHABLE here:
    // `build_replay_workspace` records a `demo` graph of ONE `period_ms`
    // `ticker` node declaring `inputs: []`, so the run has no read edge, the
    // bag carries no kind-6 READ-OUTCOME records, and the verifier correctly
    // reports `inert` — a state in which it verified NOTHING. MEASURED on this
    // exact arm: `{"status":"inert"}`. Asserting `verified_clean` here would
    // fail every run; asserting nothing would let a future change quietly turn
    // this into an exercised-but-DIVERGING edge with no one the wiser.
    //
    // So the truthful `inert` is pinned BY NAME. It is a tripwire, never a clean
    // claim: give this graph an input and the read log starts recording, this
    // assert fails, and the remedy is to upgrade it to the shared
    // `assert_read_log_verified_clean` the two mp e2es use. The window's real
    // evidence lives in `mp_record_replay_e2e_test` (data-trigger edges) and
    // `mp_split_pair_e2e_test` (a plain non-trigger edge).
    assert_eq!(
        report_json["read_log"]["status"].as_str(),
        Some("inert"),
        "the `demo` graph declares `inputs: []`, so this bag has \
         no read edges and the verifier must report INERT — which verifies nothing and \
         counts toward NO promotion cycle. If this graph gained an input, replace this \
         pin with the `verified_clean` assert the mp e2es carry. read_log: {}",
        report_json["read_log"]
    );

    // ---- Replay #2: a PERTURBED candidate → data violation (exit 1) ----
    // Overwrite the workspace's ticker cdylib with the perturbed twin (same fire
    // schedule, different published constant) — the CI stand-in for rebuilding
    // the node with a changed constant. The SAME bag now diverges on the payload.
    std::fs::copy(
        perturbed_fixture_cdylib(),
        root.join("target/debug").join(dylib_file("ticker")),
    )
    .expect("overwrite ticker cdylib with the perturbed twin");
    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(1),
        "the SAME bag replayed against a perturbed candidate must exit 1 \
         (EXIT_VIOLATION); stderr: {stderr}"
    );
    assert!(
        stderr.contains("FRAME-CONTENT DIVERGENCE") && stderr.contains("byte-mismatch"),
        "the regression verdict names the data-divergence class on stderr: {stderr}"
    );

    // ---- Replay #3: a PANICKING candidate → node failure (exit 3) ----
    // Overwrite the ticker cdylib with the PANICKING twin (same shape, tick
    // panics from its 3rd fire) — the CI stand-in for "the candidate build
    // crashes mid-replay". The crash is the ROOT CAUSE: exit 3 preempts the
    // missing-frame data violations (and any schedule fallout) the panic
    // leaves downstream — pinning the 3-over-6-over-1 precedence at the
    // process-exit layer, over a REAL production-recorded bag.
    swap_in_panicking_cdylib(root);
    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(3),
        "the SAME bag replayed against a panicking candidate must exit 3 \
         (EXIT_NODE_FAILURE — the execution widening); stderr: {stderr}"
    );
    assert!(
        stderr.contains("NODE FAILURE") && stderr.contains("'ticker'"),
        "the node-failure verdict names the crashed node on stderr: {stderr}"
    );
    assert!(
        stderr.contains("panic-class"),
        "the verdict names the failure class: {stderr}"
    );
    // EXPLICIT 3-over-1 pin: the crashed candidate stops
    // publishing, so a missing-messages DATA violation genuinely CONTENDS —
    // exit 3 must win over 1 (root cause over symptom) while the regression
    // block still renders below the node-failure block. (No structural
    // divergence contends in this fixture: fires are recorded even on
    // panicking ticks, so the replayed fire schedule matches the recording —
    // the 3-over-6 arm is pinned at the OUTCOME level by the engine test
    // `panicking_candidate_reports_node_failure_and_renders_all_blocks`.)
    assert!(
        stderr.contains("FRAME-CONTENT DIVERGENCE") && stderr.contains("downstream fallout"),
        "the contending data violation renders under exit 3 (3-over-1 with \
         both blocks): {stderr}"
    );

    // ---- Replay #4: the SAME panicking candidate under a NEUTRAL
    // `--resim all`. "Neutral" scopes the two COMPARISON outcomes and nothing
    // else — a candidate that crashed did not re-execute, so exit 0 here would
    // be a silent failure. Both modes answer 3.
    //
    // Folded into this test rather than standing alone because the reference
    // run here is a SUBPROCESS: a standalone arm would have to build its
    // reference in-process, and overwriting a dlopen'd cdylib under the running
    // test process aborts it (measured).
    let out = run_resim_bin_in_workspace(root, &bag, &[]);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(3),
        "a panicking candidate is a node failure in NEUTRAL mode too — \
         `--resim` without `--verify` declines the two COMPARISON outcomes, \
         not a crash; stderr: {stderr}"
    );
    assert!(
        stderr.contains("NODE FAILURE"),
        "the crash is reported, not swallowed under a neutral summary: {stderr}"
    );
}

/// The 6-over-1 precedence, pinned at the
/// EXIT-CODE LAYER — the process exit is the product surface. A bag with BOTH a
/// flipped payload byte (a data violation, exit-1 class) AND a dropped FIRE
/// record (a structural divergence, exit-6 class) must exit 6 (the structural
/// divergence takes precedence) while stderr still renders BOTH the `TRACE
/// DIVERGENCE` and the `FRAME-CONTENT DIVERGENCE` sections (the engine completes the
/// data diff even when the schedule diverged). The outcome/verdict halves are
/// pinned at the engine level by
/// `both_data_and_structural_divergence_are_both_recorded_and_rendered` in
/// `cerulion_cli_engine/tests/replay_engine_test.rs`.
#[test]
#[serial]
fn subprocess_replay_with_both_divergence_classes_exits_6_and_renders_both() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}d", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);

    let (mut frames, mut trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);
    // (a) data violation: flip one payload byte of the middle frame.
    frames[1][WireHeader::SIZE] ^= 0xFF;
    // (b) structural divergence: drop the step-1 ticker FIRE (frames intact).
    let i = trace
        .iter()
        .position(|r| r.record_type == RECORD_TYPE_FIRE && r.step == 1)
        .expect("a step-1 ticker fire to drop");
    trace.remove(i);

    let bag = root.join("both_classes.mcap");
    write_replay_bag(&bag, &prefix, &graph_yaml, &frames, &trace);

    let out = run_replay_bin_in_workspace(root, &bag);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(6),
        "the structural divergence must take exit precedence over the data violation; \
         stderr: {stderr}"
    );
    assert!(
        stderr.contains("FIRE-SCHEDULE DIVERGENCE"),
        "the structural section renders: {stderr}"
    );
    assert!(
        stderr.contains("FRAME-CONTENT DIVERGENCE") && stderr.contains("byte-mismatch"),
        "the data section ALSO renders (the fuller report): {stderr}"
    );
}

/// The `__cerulion/recorder.json` advisory arms,
/// end-to-end over the REAL binary — THREE bags from ONE reference run:
///
///   (a) MISMATCHED arch in the attachment → the replay still runs to its
///       NORMAL verdict (exit 0, byte-exact PASS — a cross-arch bag is NEVER
///       refused) AND stderr carries the cross-arch ULP advisory; the
///       `--report` JSON carries the parsed `recorder` block (additive);
///   (b) ABSENT attachment (the shape of every pre-recorder.json bag) → the
///       same PASS with NO advisory (back-compat, no warn spam);
///   (c) **MALFORMED JSON → exit 2, REFUSED.**
///
/// Arm (c) is the one that matters. Asserting PASS
/// plus a loud warn, on the reading that this attachment is a host-identity
/// ADVISORY and an unreadable advisory costs nothing, is wrong
/// twice over: `trace_format` lives in this document and so does
/// `coordination`, and BOTH gates live behind "did it parse?" — so a
/// malformed one would skip the version refusal AND the free-run refusal, and the
/// verdict would print its `(inferred: ... no stamp)` line, a positive claim
/// about a bag whose stamp was right there and unreadable. The shape that
/// reads most reassuring is the dangerous one: a bag from a NEWER Cerulion,
/// whose unknown `coordination` string fails the whole document, replayed
/// under the lockstep contract while announcing it predated the stamp.
///
/// The ADVISORY half still degrades (a document whose host identity will not
/// decode loses only the cross-arch warn — see `read_recorder_info`); it is the
/// CONTRACT half that refuses.
///
/// The warn-CONTENT pins (structured fields, os-only arm, matching-host
/// silence) live at the unit level in `replay_cmd.rs`'s `#[traced_test]`s.
#[test]
#[serial]
fn recorder_json_mismatch_warns_absent_silent_malformed_loud() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    let prefix = format!("rp{}rec", std::process::id());
    let graph_yaml = build_replay_workspace(root, &prefix);
    let (frames, trace) = reference_run_cdylib(root, &prefix, &graph_yaml, 3);

    // ---- (a) mismatched arch (os matches the host — arch ALONE trips it) ----
    let mismatched_arch = format!("not-{}", std::env::consts::ARCH);
    let recorder_json = serde_json::to_vec(&serde_json::json!({
        "arch": mismatched_arch,
        "os": std::env::consts::OS,
        "cerulion_version": "0.1.0",
        "recorded_at_ns": 1u64,
    }))
    .unwrap();
    let bag_a = root.join("recorder_mismatch.mcap");
    write_replay_bag_with_extra(
        &bag_a,
        &prefix,
        &graph_yaml,
        &frames,
        &trace,
        &[att(
            "__cerulion/recorder.json",
            "application/json",
            recorder_json,
        )],
    );
    let report = root.join("recorder_report.json");
    let out = run_replay_bin_in_workspace_with_report(root, &bag_a, &report);
    // The verdict is `eprint!`ed to stderr; `tracing` warns now go to STDERR
    // too (init_logging writes there so a command's stdout stays clean data).
    // The COMBINED text is robust to the writer choice either way.
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        stderr_of(&out)
    );
    assert_eq!(
        out.status.code(),
        Some(0),
        "a cross-arch bag is NEVER refused (warn-only); output: {combined}"
    );
    assert!(
        combined.contains("replay PASS"),
        "normal verdict: {combined}"
    );
    assert!(
        combined.contains("cross-architecture/OS replay") && combined.contains("ULPs"),
        "the cross-arch advisory fires (tracing warn, stderr): {combined}"
    );
    let report_json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&report).expect("report written"))
            .expect("report parses");
    assert_eq!(
        report_json["recorder"]["arch"], mismatched_arch,
        "the report carries the parsed recorder block additively: {report_json}"
    );

    // ---- (b) absent attachment → PASS, NO advisory (back-compat) ----
    let bag_b = root.join("recorder_absent.mcap");
    write_replay_bag(&bag_b, &prefix, &graph_yaml, &frames, &trace);
    let out = run_replay_bin_in_workspace(root, &bag_b);
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        stderr_of(&out)
    );
    assert_eq!(out.status.code(), Some(0), "output: {combined}");
    assert!(
        combined.contains("replay PASS"),
        "normal verdict: {combined}"
    );
    assert!(
        !combined.contains("cross-architecture") && !combined.contains("recorder.json"),
        "an absent attachment is SILENT — no warn spam on pre-recorder.json bags: {combined}"
    );

    // ---- (c) malformed JSON → REFUSED, exit 2 ----
    let bag_c = root.join("recorder_malformed.mcap");
    write_replay_bag_with_extra(
        &bag_c,
        &prefix,
        &graph_yaml,
        &frames,
        &trace,
        &[att(
            "__cerulion/recorder.json",
            "application/json",
            b"definitely{not json".to_vec(),
        )],
    );
    let out = run_replay_bin_in_workspace(root, &bag_c);
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        stderr_of(&out)
    );
    assert_eq!(
        out.status.code(),
        Some(2),
        "an UNREADABLE contract carrier is not replay-grade: this document is what says \
         which trace format and which coordination contract the bag was recorded under, so \
         replaying it means guessing one. Output: {combined}"
    );
    assert!(
        !combined.contains("replay PASS"),
        "a refused bag must not also print a verdict: {combined}"
    );
    // The refusal NAMES what was lost — an operator who sees only "malformed"
    // cannot tell a cosmetic attachment from the contract carrier.
    assert!(
        combined.contains("recorder.json"),
        "the refusal names the attachment: {combined}"
    );
    assert!(
        combined.contains("trace_format") && combined.contains("coordination"),
        "…and the two CONTRACT keys that became unreadable, which is WHY it refuses: \
         {combined}"
    );
    // ANTI-TAUTOLOGY for the whole arm: the tempting claim is that such a
    // bag "is still replayable", so the refusal must not be reachable by any
    // OTHER route — arms (a) and (b) above replayed the very same frames and
    // trace to exit 0, differing only in this attachment's bytes.
}

// ===========================================================================
// The deleted flag, and ONE exit code for a malformed
// value on BOTH halves of `bag play`
// ===========================================================================

#[test]
fn a_stale_max_ticks_on_bag_play_is_an_unknown_argument() {
    // `docs/bag.md` states this (`--max-ticks` "is clap's
    // unknown-argument error (exit 2), never a silently accepted no-op") and
    // this arm drives it. The removed-verb arm above drives `--max-ticks` only
    // through `cerulion replay`, which dies as an unrecognized SUBCOMMAND long
    // before any flag is parsed — so it says nothing about `bag play`.
    //
    // Both halves, because the flag is legal on neither: a resim and a plain
    // playback.
    for argv in [
        vec![
            "bag",
            "play",
            "b.mcap",
            "--resim",
            "all",
            "--max-ticks",
            "3",
        ],
        vec!["bag", "play", "b.mcap", "--max-ticks", "3"],
    ] {
        let out = run_bin(&argv);
        let stderr = stderr_of(&out);
        assert_eq!(
            out.status.code(),
            Some(2),
            "{argv:?} must exit 2; stderr: {stderr}"
        );
        assert!(
            stderr.contains("unexpected argument") || stderr.contains("--max-ticks"),
            "{argv:?} must die naming the unknown argument; stderr: {stderr}"
        );
    }
}

#[test]
fn a_malformed_bag_time_value_exits_2_on_both_halves_of_bag_play() {
    // ONE exit code for one class of mistake. `--duration` is legal on BOTH
    // halves and `--start-offset` on playback only, so before this a
    // `bag play --duration -1` (no `--resim`) fell through to the generic
    // dispatch and exited **1** while the identical mistake on
    // `--start-offset` exited 2 — because that flag put the invocation in the
    // resim family and reached `resim_usage_refusal`.
    //
    // On this surface exit 1 means "your code diverged" under `--verify`, so a
    // CI job cannot be left to tell a typo from a regression by reading stderr.
    //
    // Re-narrowing the hoisted refusal to `is_resim_family(action)`
    // makes the FIRST arm exit 1 and this fails.
    // The `=` form is required: a bare `-1` is parsed by clap as a FLAG and
    // dies as an unexpected argument before any value parser runs, so it
    // exercises clap rather than this surface.
    for (argv, flag) in [
        (vec!["bag", "play", "b.mcap", "--duration=-1"], "--duration"),
        (
            vec!["bag", "play", "b.mcap", "--start-offset=-1"],
            "--start-offset",
        ),
        (
            vec!["bag", "play", "b.mcap", "--resim", "all", "--duration=-1"],
            "--duration",
        ),
    ] {
        let out = run_bin(&argv);
        let stderr = stderr_of(&out);
        assert_eq!(
            out.status.code(),
            Some(2),
            "{argv:?}: a malformed VALUE is a usage error (2), never a data violation (1); \
             stderr: {stderr}"
        );
        assert!(
            stderr.contains(flag),
            "{argv:?}: the refusal must name the offending flag; stderr: {stderr}"
        );
    }

    // ANTI-TAUTOLOGY: a WELL-FORMED value on the same (non-resim) half must
    // still reach the player and fail for its OWN reason — the widened gate
    // refuses nothing it did not refuse before. `b.mcap` does not exist, so
    // this is the ordinary missing-bag failure, which is NOT exit 2's usage
    // surface.
    let out = run_bin(&["bag", "play", "b.mcap", "--duration", "1.5"]);
    let stderr = stderr_of(&out);
    assert_ne!(
        out.status.code(),
        Some(2),
        "a well-formed bound must not be refused by the usage gate; stderr: {stderr}"
    );
    assert!(
        !stderr.contains("--duration"),
        "…and the failure must be about the BAG, not the flag; stderr: {stderr}"
    );
}

#[test]
fn a_playback_start_offset_reaches_the_player_not_the_resim_surface() {
    // `--start-offset` is PLAYBACK-ONLY. If
    // `main::is_resim_family` listed it, a well-formed
    // `bag play <bag> --start-offset 1.5` with NO `--resim` would be routed into
    // `resim_cmd::run_play_resim`, which resolves it to `PlayMode::Playback`,
    // takes its own "unreachable from the real caller" arm and answers
    // "no `--resim` given" with exit 2. The seek the engine implements
    // (`bag_cmd`'s `start_offset_ns` window, pinned by
    // `bag_play_iox2_test::a_start_offset_skips_the_prefix_…`) would be
    // unreachable from the CLI: the one flag combination that surface exists
    // for could not be spelled.
    //
    // The oracle is the ROUTE, not the play: `b.mcap` does not exist, so a
    // correctly-routed invocation fails on the BAG. Exit 2 and the resim
    // refusal's own words are what a mis-route produces, and neither may
    // appear. Its sibling above pins the same property for `--duration`.
    //
    // Restoring `|| start_offset.is_some()` to `is_resim_family`
    // fails this with exit 2 and "no `--resim` given" in stderr.
    for argv in [
        vec!["bag", "play", "b.mcap", "--start-offset", "1.5"],
        // …and beside its both-halves sibling, which is the combination an
        // operator seeking a window in the middle of a bag actually types.
        vec![
            "bag",
            "play",
            "b.mcap",
            "--start-offset",
            "1.5",
            "--duration",
            "2.5",
        ],
    ] {
        let out = run_bin(&argv);
        let stderr = stderr_of(&out);
        assert_ne!(
            out.status.code(),
            Some(2),
            "{argv:?}: playback with a bag-time window must not be refused as a resim \
             misuse; stderr: {stderr}"
        );
        assert!(
            !stderr.contains("no `--resim` given"),
            "{argv:?}: this invocation NEEDS no `--resim` — it is playback; stderr: {stderr}"
        );
        assert!(
            !stderr.contains("--start-offset"),
            "{argv:?}: the failure must be about the BAG, not the flag; stderr: {stderr}"
        );
    }

    // ANTI-TAUTOLOGY: the same flag WITH `--resim` is a genuine misuse and must
    // still exit 2 naming it — without this arm, deleting the whole refusal
    // would pass the loop above.
    let out = run_bin(&[
        "bag",
        "play",
        "b.mcap",
        "--resim",
        "all",
        "--start-offset",
        "1.5",
    ]);
    let stderr = stderr_of(&out);
    assert_eq!(
        out.status.code(),
        Some(2),
        "`--start-offset` under `--resim` is a resim-surface misuse; stderr: {stderr}"
    );
    assert!(
        stderr.contains("--start-offset"),
        "…and the refusal must name it; stderr: {stderr}"
    );
}
