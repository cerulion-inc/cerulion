// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run` FAILS CLOSED on a failing
//! validation check, and `--no-validate` is the opt-out.
//!
//! REAL-BINARY, not engine-level, and that is the whole point: the defect
//! this gate fixes is that the VERB downgraded the report. `graph validate`
//! exited nonzero on a failing report while `graph run` printed
//! `graph validation has failures — continuing anyway` and ran — two verbs
//! disagreeing about whether a failing check is fatal. An engine-level arm
//! cannot see that, because the engine returns the same report either way; only
//! driving the binary can.
//!
//! The defect chosen is UNIQUE TO THE REPORT — a `schema:` value that
//! disagrees with what the node itself declares. `GraphRuntime::build` does not
//! catch it (the wire layout comes from the node's Rust type, so the graph runs
//! perfectly well with a mislabelled output), which is exactly why it must be
//! the gate that stops it, and exactly why the `--no-validate` arm can still
//! reach live readiness and exit 0.
//!
//! Needs `cargo build -p test_node_macro_period_cdylib
//! -p test_node_macro_data_trigger_cdylib` first (the unwired-trigger arm uses
//! the second one). `#[cfg(unix)]`
//! (signals); `#[serial]` + unique per-test prefixes (the run looks on the
//! DEFAULT iceoryx2 namespace, so topic names must not collide).

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

mod mp_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";

/// The schema the fixture's `#[output] cmd: Vector3` really publishes — the
/// value a HEALTHY graph declares.
const TRUE_SCHEMA: &str = "geometry_msgs/Vector3";
/// A resolvable but WRONG label. Resolvable matters: it slips past the
/// "does this name a schema at all" half, so the only thing that catches it is
/// the cross-check against the node's own declaration — a report-unique check.
const WRONG_SCHEMA: &str = "std_msgs/String";

/// A single-node workspace whose `graphs/demo.yaml` declares `schema` on the
/// ticker's output. Cribbed from `signal_matrix_e2e_test::build_workspace`.
fn build_workspace(root: &Path, prefix: &str, schema: &str) {
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
            "prefix: {prefix}\nnodes:\n  - id: ticker\n    type: ticker\n    outputs:\n      \
             - name: cmd\n        schema: {schema}\n"
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

fn spawn_run(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join("run.stderr");
    let mut args = vec!["graph", "run", "demo", "--single-process"];
    args.extend_from_slice(extra);
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(&args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic: LOCAL-ONLY, no gateway/scouting.
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    // `--single-process` + network off: a monolith with no subtree.
    (ChildGuard::single_process(child), stderr_path)
}

/// The flag's `--help` text must describe the reach the rest of this file
/// PROVES, not a broader reach.
///
/// Unpinned, this text drifts: a doc comment
/// reading "Skip workspace validation before running" would contradict the four
/// `..._without_promising_a_false_escape` arms below, which prove the flag cannot skip
/// the topology, cdylib or trigger-wiring checks — `graph_run` re-runs
/// `validate_graph` a few lines past its own gate, and `GraphRuntime::build`
/// runs it again. A flag whose help text claims a power its own contract suite
/// disproves is the misleading-surface class, so it is pinned here beside the
/// behaviour it describes.
///
/// Asserted as CLAIMS, not as prose: the false claim must be absent, and the
/// two true ones (what it cannot skip; what it still can) must be present. That
/// survives rewording and still fails on a revert.
#[test]
fn the_no_validate_help_text_describes_the_reach_this_file_proves() {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", "--help"])
        .env_remove("CARGO_TARGET_DIR")
        .output()
        .expect("run `cerulion graph run --help`");
    assert!(
        out.status.success(),
        "`graph run --help` must exit 0, got {:?}",
        out.status
    );
    let help = String::from_utf8_lossy(&out.stdout);
    let (flag_help, _) = help
        .split_once("--release")
        .expect("`--help` must list `--release` after `--no-validate`");
    let flag_help = flag_help
        .split_once("--no-validate")
        .expect("`--help` must list `--no-validate`")
        .1;
    // Clap WRAPS help text, so a two-word claim can straddle a line break and a
    // naive `contains` would fail on a different terminal width rather than on a
    // real regression. Match against a whitespace-collapsed view instead.
    let flag_help = flag_help.split_whitespace().collect::<Vec<_>>().join(" ");
    let flag_help = flag_help.as_str();

    // The overclaim. It is FALSE: validation as such is not
    // skippable — only the report is.
    assert!(
        !flag_help.contains("Skip workspace validation"),
        "`--no-validate` must not claim to skip workspace validation — the topology, \
         trigger-wiring and cdylib checks are re-run unconditionally; help:\n{flag_help}"
    );
    // What it cannot skip (the four negative arms below).
    for needle in ["topology", "trigger wiring", "node librar"] {
        assert!(
            flag_help.contains(needle),
            "`--no-validate` help must name what it CANNOT skip ({needle:?}); \
             help:\n{flag_help}"
        );
    }
    // What it still can (`no_validate_runs_the_same_defective_graph_...`).
    assert!(
        flag_help.contains("AMBIGUOUS") && flag_help.contains("report or no report"),
        "`--no-validate` help must say an AMBIGUOUS spelling is refused report or no report; \
         help:\n{flag_help}"
    );
    assert!(
        flag_help.contains("schema:"),
        "`--no-validate` help must name the `schema:` family it CAN still skip; \
         help:\n{flag_help}"
    );
    // ...but NOT the whole family. An ABSENT or empty `schema:` is
    // `validate_graph`'s to refuse — the report skips it rather than
    // double-report it — so it is a TOPOLOGY failure and the flag cannot skip it
    // either, which is what `a_schema_less_graph_is_refused_without_\
    // promising_a_false_escape` pins on the error side. Help text saying "a
    // value that names no resolvable schema" reads as covering the empty case
    // and re-creates that false promise on the `--help` side, where no test was
    // looking.
    assert!(
        flag_help.contains("NON-EMPTY") || flag_help.contains("non-empty"),
        "`--no-validate` help must restrict its `schema:` promise to NON-EMPTY values — \
         an absent/empty `schema:` is a topology refusal the flag cannot skip; \
         help:\n{flag_help}"
    );
}

/// **Decision: a recording is never made with the schema checks off.**
///
/// `--no-validate` skips exactly the `schema:` family, and everything else in
/// this file is about a RUN paying for that later. A BAG pays for it
/// permanently: the mislabelled channel is written into the artifact, so
/// `cerulion bag play --resim` can refuse a healthy bag or replay one under a
/// label nothing ever checked. The two flags are therefore refused TOGETHER, at
/// parse.
///
/// REAL-BINARY, and driven on the exact workspace the rest of the file uses to
/// prove the hazard is real: `WRONG_SCHEMA` is the defect
/// `no_validate_runs_the_same_defective_graph_to_a_clean_exit` shows the flag
/// waving through. Before this refusal, adding `--record` to that argv wrote a
/// bag whose channel carried `std_msgs/String` for a `geometry_msgs/Vector3`
/// producer, with nothing in the pipeline having looked.
///
/// The refusal is LAYERED, and this arm pins which layer answers. Two
/// independent guards refuse this pair — clap's `conflicts_with` at PARSE, and
/// the engine's own `validate_record_flags` arm — and the engine one sits
/// before every side effect too, so exit code, both flag names, no live-loop
/// line, no `recordings/` and a byte-identical graph file are all satisfied by
/// EITHER. Those assertions therefore pin *refusal before side effects*, which
/// is the property that matters to an operator, but they do not discriminate
/// the layers. What does is clap's own conflict phrasing, `cannot be used
/// with`: the engine twin deliberately words its message `cannot be combined
/// with`, so requiring the clap form here fails if `conflicts_with` is dropped
/// and the engine silently takes over. Matched as that fragment only — the full
/// sentence and the usage block are clap-version-brittle.
///
/// (The conflict across all four SPELLINGS, and the each-flag-alone
/// anti-tautology, are the cheap unit twin,
/// `cli::tests::graph_run_no_validate_conflicts_with_record_in_every_spelling`.
/// This arm carries the halves a parse test cannot see: the artifacts.)
///
/// Bounded wait + `ChildGuard`, like every other refusal arm in this file: a
/// regressed conflict runs the live loop forever, and a blocking `.output()`
/// would hang the `#[serial]` binary into CI's job-level cancel — an
/// unattributable timeout instead of a named assertion.
#[test]
#[serial]
fn recording_with_no_validate_is_refused_before_anything_is_written() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "norecord", WRONG_SCHEMA);
    let graph_path = tmp.path().join("graphs/demo.yaml");
    let before = std::fs::read(&graph_path).expect("read graph before");

    // BOTH record spellings — `--record` takes an optional `require_equals`
    // value, so the bare flag and `--record=DIR` reach clap as different
    // shapes, and a conflict is order-independent.
    for extra in [
        vec!["--no-validate", "--record=recordings"],
        vec!["--record", "--no-validate"],
    ] {
        let (mut guard, stderr_path) = spawn_run(tmp.path(), &extra);
        let status = guard
            .wait_bounded(Duration::from_secs(60))
            .unwrap_or_else(|| {
                panic!(
                    "run did not exit ({extra:?}) — a refusal that regressed leaves the live \
                 loop running; stderr:\n{}",
                    read_file(&stderr_path)
                )
            });
        let stderr = read_file(&stderr_path);

        assert!(
            !status.success(),
            "a recording with the schema checks off must be refused ({extra:?}), \
             got {status:?}; stderr:\n{stderr}"
        );
        // NAMED, both of them: either flag is the operator's to drop, and a
        // refusal naming one leaves them guessing which.
        assert!(
            stderr.contains("--no-validate") && stderr.contains("--record"),
            "the refusal must name BOTH flags ({extra:?}); stderr:\n{stderr}"
        );
        // THE layer discriminator (see the header): clap's conflict wording,
        // which the engine twin deliberately does not use.
        assert!(
            stderr.contains("cannot be used with"),
            "the refusal must come from clap's `conflicts_with` at PARSE, not from the \
             engine twin one layer down ({extra:?}); stderr:\n{stderr}"
        );
        // Refused before the run's own machinery started.
        assert!(
            !stderr.contains("starting graph (live)"),
            "the refusal must precede the live loop ({extra:?}); stderr:\n{stderr}"
        );
        assert!(
            !tmp.path().join("recordings").exists(),
            "a refused run must write no bag ({extra:?})"
        );
        assert_eq!(
            std::fs::read(&graph_path).expect("read graph after"),
            before,
            "a refused run must leave the graph file byte-untouched ({extra:?})"
        );
    }
}

fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) {
    let start = Instant::now();
    loop {
        if read_file(path).contains(needle) {
            return;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{}",
            read_file(path)
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// THE gate. A graph whose only defect is a report-unique failing check must
/// REFUSE — nonzero exit, with the failing check NAMED on stderr.
#[test]
#[serial]
fn a_failing_validation_check_refuses_the_run_and_names_it() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "gatea", WRONG_SCHEMA);

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);

    assert!(
        !status.success(),
        "a failing validation check must REFUSE the run, got {status:?}; stderr:\n{stderr}"
    );
    // The refusal NAMES the check — a warn-and-continue would give the
    // operator the whole report, so a refusal saying less would be a step
    // backwards.
    assert!(
        stderr.contains("refusing to run graph 'demo'"),
        "the refusal must name the graph; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("schema 'ticker'.cmd"),
        "the refusal must NAME the failing check; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("--no-validate"),
        "the refusal must name the documented opt-out; stderr:\n{stderr}"
    );
    // And it must NOT warn and carry on.
    assert!(
        !stderr.contains("continuing anyway"),
        "`graph run` must not continue anyway; stderr:\n{stderr}"
    );
    // It refused BEFORE reaching the live loop — a refusal that started the
    // graph first would have already opened the topic it was refusing over.
    assert!(
        !stderr.contains("starting graph (live)"),
        "the refusal must precede the live loop; stderr:\n{stderr}"
    );
}

/// The documented opt-out. THE SAME defective graph runs under `--no-validate`
/// and exits 0 on SIGINT — which is also what proves the defect is genuinely
/// report-unique (a graph the runtime itself refused could not reach here).
#[test]
#[serial]
fn no_validate_runs_the_same_defective_graph_to_a_clean_exit() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "gateb", WRONG_SCHEMA);

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &["--no-validate"]);
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));

    let stderr = read_file(&stderr_path);
    assert!(
        status.success(),
        "`--no-validate` must run the graph anyway and exit 0, got {status:?}; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains("refusing to run graph"),
        "`--no-validate` must not refuse; stderr:\n{stderr}"
    );
}

/// The two halves of the `--no-validate` `schema:` story must
/// be told apart in `USER_API.md`, because the sentence that describes one of
/// them can easily read as describing the other.
///
/// The row said, in order: an ABSENT or empty `schema:` "is refused by
/// `validate_graph` … which this flag cannot skip either. Such a graph RUNS".
/// Read linearly, "Such a graph" binds to the nearest antecedent — the case
/// that had just been said to be REFUSED — so the document told a user that the
/// graph the flag cannot rescue runs anyway. The intended antecedent was the
/// NON-EMPTY invalid-label family two sentences earlier — one member of which
/// (`WRONG_SCHEMA`: resolvable, but contradicting the node's own declaration)
/// is what `no_validate_runs_the_same_defective_graph_to_a_clean_exit` above
/// drives to a clean exit.
///
/// BOTH claims are already DRIVEN in this file, which is why this arm pins the
/// SAYING only and adds no third subprocess run: the runs-anyway half by that
/// test, and the refuses-even-under-the-flag half by
/// `a_schema_less_graph_is_refused_without_promising_a_false_escape`
/// below, whose second leg runs an empty `schema:` under `--no-validate` and
/// requires a nonzero exit. (`OutputDef::schema` is `#[serde(default)]`, so an
/// OMITTED key and an empty one are the same value by the time `validate_graph`
/// sees them — there is no separate absent-key path to drive.)
///
/// Asserted over the whitespace-normalized document — the row is ONE very long
/// line today, so normalizing buys nothing yet and everything the day it is
/// wrapped — and as CONTIGUOUS sentences, not as loose phrases: the defect is a PROXIMITY one — a dangling
/// antecedent — so two independent `contains` over a 30 kB row would be
/// satisfied by a rewrite that re-introduced the dangling binding and happened
/// to carry both phrases somewhere.
///
/// That makes claims (2) and (3) BYTE-EXACT sentences, deliberately, and the
/// cost is stated rather than wished away: rewording either one fails this
/// arm, and updating it is the price of rewording a user-facing claim about
/// what a flag refuses. Only claim (1) — the shape that must never come back —
/// is reword-tolerant.
///
/// What the pinned spans deliberately do NOT contain is a countable
/// cross-reference. A row that says "two sentences above" is one
/// this arm would freeze as WORDS while verifying nothing about
/// the COUNT: inserting a sentence into the row would make the document
/// wrong and leave the arm green, and fixing the count would make the arm
/// red for no defect.
#[test]
fn the_user_api_no_validate_row_binds_the_runs_sentence_to_the_non_empty_case() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .join("docs/user-api.md");
    let raw = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}) — this arm pins the document users read; \
             it must FAIL rather than skip",
            path.display()
        )
    });
    let doc = raw.split_whitespace().collect::<Vec<_>>().join(" ");

    // ANTI-TAUTOLOGY: without a reachable anchor every claim below is vacuous
    // against a moved or reworded row. EXACTLY once — a stale duplicate row
    // below a fixed one would be checked by nothing.
    let anchor = "An ABSENT or empty `schema:` is NOT in that set";
    assert_eq!(
        doc.matches(anchor).count(),
        1,
        "the `graph run` row's absent-schema sentence must appear EXACTLY once \
         ({anchor:?}) — this arm is pinning nothing until its anchor is updated"
    );

    // (1) The dangling binding is gone.
    assert!(
        !doc.contains("which this flag cannot skip either. Such a graph RUNS"),
        "`USER_API.md` must not bind \"Such a graph RUNS\" to the ABSENT/empty \
         `schema:` case it has just said is REFUSED"
    );
    // (2) The refusal is stated as holding under the flag — the claim a reader
    //     needs and an unqualified row omits. CONTIGUOUS with the clause it
    //     qualifies, so it cannot drift back onto another sentence.
    assert!(
        doc.contains(
            "which this flag cannot skip either: such a graph is REFUSED whether or not \
             `--no-validate` is passed."
        ),
        "`USER_API.md` must say an absent/empty `schema:` refuses EVEN UNDER \
         `--no-validate`, in the sentence that raises the case"
    );
    // (3) The RUNS sentence names the case it is actually about — one
    //     contiguous span, so "RUNS" cannot be separated from its subject.
    assert!(
        doc.contains(
            "A graph whose `schema:` is NON-EMPTY but wrong (the family the flag DOES \
             skip) RUNS instead"
        ),
        "`USER_API.md`'s \"runs anyway\" sentence must name the NON-EMPTY \
         invalid-label family it describes, in one sentence with it"
    );
}

/// A graph with NO `prefix:` line still RUNS.
///
/// `graph_read` uses the raw parser (an absent `prefix:` stays absent, so the
/// verbs that round-trip the YAML do not invent one), `validate_graph` rejects
/// an EMPTY prefix, and a `graph_run` that filled the default AFTER its
/// validation gate would be invisible under a warn-and-continue report (the
/// run would carry on to fill it a few lines later) — but with the gate failing
/// CLOSED it would REFUSE a graph `graph validate` passes, which is the two-verb
/// disagreement the gate exists to remove, re-created in the opposite
/// direction.
///
/// REAL-BINARY, and it has to be: the ordering lives inside `graph_run`, so an
/// engine-level arm asking `graph_validate` cannot see it (that verb has
/// long filled the prefix itself; reverting the fill to below the gate
/// leaves such an arm GREEN).
#[test]
#[serial]
fn a_graph_with_no_prefix_line_still_runs() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "noprefix", TRUE_SCHEMA);

    // Rewrite the graph WITHOUT a `prefix:` line. Everything else — the node,
    // its output, the correct schema — is the healthy arm's fixture, so the
    // absent prefix is the only variable.
    std::fs::write(
        tmp.path().join("graphs/demo.yaml"),
        format!(
            "nodes:\n  - id: ticker\n    type: ticker\n    outputs:\n      - name: cmd\n        \
             schema: {TRUE_SCHEMA}\n"
        ),
    )
    .unwrap();

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));

    let stderr = read_file(&stderr_path);
    assert!(
        status.success(),
        "a prefix-less graph must still run, got {status:?}; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains("refusing to run graph"),
        "the run gate must not refuse it — the default prefix is filled BEFORE \
         validation; stderr:\n{stderr}"
    );
}

/// ANTI-TAUTOLOGY. The identical harness with the CORRECT schema reaches live
/// readiness and exits 0 WITHOUT `--no-validate` — so the refusal above is
/// attributable to the defect, not to the gate refusing everything.
#[test]
#[serial]
fn a_healthy_graph_still_runs_with_validation_on() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "gatec", TRUE_SCHEMA);

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );

    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));

    let stderr = read_file(&stderr_path);
    assert!(
        status.success(),
        "a healthy graph must still run with validation ON, got {status:?}; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains("refusing to run graph"),
        "a healthy graph must not be refused; stderr:\n{stderr}"
    );
}

// ==========================================================================
// The fail-closed gate must not lock out the RECOVERY
// tool for the very thing it refuses.
//
// `graph run --auto-partition` is the documented repair for a stale or broken
// `process_groups:` block: the derive paths validate REPLACE-SCOPED
// precisely so the block being replaced cannot be what refuses the run. A
// fail-closed report that ran BEFORE that decision would reject
// the block first and make the repair unreachable — the regression these
// arms pin.
// ==========================================================================

/// A `process_groups:` block that `validate_graph` refuses: an EMPTY group.
const BROKEN_PARTITION: &str = "process_groups:\n  p0: [ticker]\n  p1: []\n";

/// `spawn_run`'s sibling with NO `--single-process` — that flag is mutually
/// exclusive with `--auto-partition`, so the recovery arms cannot use it.
fn spawn_run_no_single(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join("run.stderr");
    let mut args = vec!["graph", "run", "demo"];
    args.extend_from_slice(extra);
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(&args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    // NO `--single-process`: these arms run the DERIVED partition, so the
    // supervisor has workers and must lead its own group.
    let guard = ChildGuard::spawn_group_leader(&mut cmd).expect("spawn cerulion graph run");
    (guard, stderr_path)
}

/// Append a broken `process_groups:` block to the workspace's graph.
fn break_the_partition(root: &Path) {
    let path = root.join("graphs/demo.yaml");
    let mut yaml = std::fs::read_to_string(&path).unwrap();
    yaml.push_str(BROKEN_PARTITION);
    std::fs::write(&path, yaml).unwrap();
}

/// THE regression. Both halves in one body, ONE flag apart, so neither can
/// pass for a reason unrelated to the other:
///
/// * WITHOUT `--auto-partition` the broken block refuses the run — which is
///   both the correct behaviour and the PRECONDITION that makes the second
///   half mean something (if the block were not refused, the recovery arm
///   would prove nothing).
/// * WITH `--auto-partition --yes` the SAME graph is repaired and runs.
#[test]
#[serial]
fn auto_partition_still_recovers_a_broken_process_groups_block() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "recov", TRUE_SCHEMA);
    break_the_partition(tmp.path());

    // ---- PRECONDITION: the block really is refused on the respect-file path.
    let (mut guard, stderr_path) = spawn_run_no_single(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(
        !status.success() && stderr.contains("refusing to run graph 'demo'"),
        "precondition: a broken process_groups block must refuse a plain run, \
         got {status:?}; stderr:\n{stderr}"
    );

    // ---- RECOVERY: --auto-partition repairs it and the run goes live.
    let (mut guard, stderr_path) = spawn_run_no_single(tmp.path(), &["--auto-partition", "--yes"]);
    wait_for_log_line(
        &stderr_path,
        "GO signaled; deployment live",
        Duration::from_secs(90),
    );
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));

    let stderr = read_file(&stderr_path);
    assert!(
        !stderr.contains("refusing to run graph"),
        "--auto-partition is the RECOVERY for a broken block — the gate must not \
         refuse it first; stderr:\n{stderr}"
    );
    assert!(
        status.success(),
        "the repaired run must exit cleanly, got {status:?}; stderr:\n{stderr}"
    );
    // AND it must leave nothing behind. This is the only arm in the file that
    // reaches a LIVE deployment ("GO signaled"), so it is the only one that can
    // leak a `run-worker` — and without this call it takes no verdict at all. `Drop`
    // merely REPORTS a leak to stderr, which libtest discards on a passing
    // test, so a repaired run that exited 0 while orphaning its workers would be
    // indistinguishable from a clean one. `finish()` turns that report into an
    // assertion.
    guard.finish().assert_clean();
}

/// The other half of the scoping, and the reason it is REPLACE-scoped rather
/// than skipped: on the derive path every NON-partition check stays mandatory.
/// Without this, "the derive path is not refused" would be satisfied by a
/// derive path that validates nothing at all.
#[test]
#[serial]
fn a_deriving_run_still_fails_closed_on_a_non_partition_check() {
    let tmp = tempfile::tempdir().unwrap();
    // BOTH defects at once: the broken block (which must be forgiven on this
    // path) and the wrong schema (which must not).
    build_workspace(tmp.path(), "recovb", WRONG_SCHEMA);
    break_the_partition(tmp.path());

    let (mut guard, stderr_path) = spawn_run_no_single(tmp.path(), &["--auto-partition", "--yes"]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);

    assert!(
        !status.success(),
        "a non-partition failure must still refuse a DERIVING run, got {status:?}; \
         stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("refusing to run graph 'demo'"),
        "and it must say so; stderr:\n{stderr}"
    );
    // It refused for the SCHEMA, not for the partition block — the scoping
    // forgave the block and the mandatory half caught the real defect.
    assert!(
        stderr.contains(WRONG_SCHEMA),
        "the refusal must name the schema defect, not the forgiven block; \
         stderr:\n{stderr}"
    );
}

/// The refusal only advertises `--no-validate` when
/// that flag can actually deliver.
///
/// `node stage` writes a raw-FFI output with an EMPTY `schema:` on purpose
/// (warning as it does so, because the info-JSON format carries a schema hash
/// but no schema NAME), and `validate_graph` refuses that. Offering
/// `--no-validate` there would not help — it skips the REPORT but not
/// `GraphRuntime::build`, and the build runs `validate_graph` itself. So the
/// flag would move the user from one refusal to an identical one: a remedy that
/// does not work, which is worse than no remedy.
///
/// The graph is STILL refused both ways — that is the intended design ("staging
/// WARNS and writes the entry, and the graph is refused when it is RUN"). The
/// message must not promise an escape that does not
/// exist. Both halves in one body, ONE flag apart.
#[test]
#[serial]
fn a_schema_less_graph_is_refused_without_promising_a_false_escape() {
    let tmp = tempfile::tempdir().unwrap();
    // The exact shape `node stage` writes for a raw-FFI output.
    build_workspace(tmp.path(), "noschema", "\"\"");

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);

    assert!(
        !status.success(),
        "an empty schema must refuse the run; stderr:\n{stderr}"
    );
    // It names the REAL fix — the message `validate_graph` already carried.
    assert!(
        stderr.contains("`schema:` is missing"),
        "the refusal must name the actual defect; stderr:\n{stderr}"
    );
    // THE PIN: it must not offer an escape it cannot honour.
    assert!(
        !stderr.contains("re-run with `--no-validate` to run anyway"),
        "`--no-validate` cannot skip the topology check, so it must not be \
         advertised for one; stderr:\n{stderr}"
    );

    // And the flag really does not rescue it — the promise would have been
    // false. Without this half, removing the advertisement could be mistaken
    // for the flag having started to work.
    let (mut guard, stderr_path) = spawn_run(tmp.path(), &["--no-validate"]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(
        !status.success(),
        "--no-validate must not run a graph the runtime refuses; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("`schema:` is missing"),
        "and it reaches the same, accurate message; stderr:\n{stderr}"
    );
}

/// The no-false-escape rule covers the CDYLIB class
/// too — a staged-but-never-BUILT node.
///
/// The `--no-validate` advertisement is narrowed away from the topology
/// check because the runtime re-performs it. A failing `cdylib '<type>'` check
/// has exactly the same shape: `load_node_factories_from_cache` runs
/// unconditionally off the SAME cdylib cache the report used, so the flag
/// lands on an identical `CdylibNotFound`. And this is arguably the most
/// common way a new user meets the gate at all — `node stage`, then
/// `graph run`, without `node build` in between.
///
/// Both halves in one body, ONE flag apart.
#[test]
#[serial]
fn an_unbuilt_node_is_refused_without_promising_a_false_escape() {
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "unbuilt", TRUE_SCHEMA);
    // The node is staged and its source is there — it was simply never built.
    std::fs::remove_file(tmp.path().join("target/debug").join(dylib_file("ticker")))
        .expect("un-build the node");

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);

    assert!(
        !status.success(),
        "an unbuilt node must refuse the run; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("cdylib 'ticker'"),
        "the refusal must name the failing check; stderr:\n{stderr}"
    );
    // THE PIN: no escape it cannot honour.
    assert!(
        !stderr.contains("re-run with `--no-validate` to run anyway"),
        "`--no-validate` cannot get past a missing cdylib, so it must not be \
         advertised for one; stderr:\n{stderr}"
    );

    // And it really cannot — without this half, removing the advertisement
    // could be mistaken for the flag having started to work.
    let (mut guard, stderr_path) = spawn_run(tmp.path(), &["--no-validate"]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(
        !status.success(),
        "--no-validate must not run a graph whose cdylib is missing; stderr:\n{stderr}"
    );
}

/// A failing `node crate` check is only
/// a no-escape when the node's LIBRARY is gone too.
///
/// Putting `node crate '` in the blocking set wholesale would be
/// an overclaim in the opposite direction: the run never
/// reads the crate DIRECTORY, it loads the built library. So a crate deleted
/// while its previously-built dylib survives in `target/` really does run under
/// `--no-validate`, and telling that user the flag cannot help is just as false
/// as telling the schema-less user that it can.
///
/// Both directions in one body, because the discriminator is the whole point.
#[test]
#[serial]
fn a_missing_crate_is_a_no_escape_only_when_its_library_is_gone_too() {
    // ---- (a) ORPHANED DYLIB: crate gone, library survives ⇒ the flag WORKS,
    // so the refusal must offer it.
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "orphan", TRUE_SCHEMA);
    std::fs::remove_dir_all(tmp.path().join("nodes/ticker")).expect("delete the crate");

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(
        !status.success(),
        "the report still refuses; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("node crate 'ticker'"),
        "and names the failing check; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("re-run with `--no-validate` to run anyway"),
        "the flag DOES work here, so it must be offered; stderr:\n{stderr}"
    );

    // And it really does work — the claim above is checked, not assumed.
    let (mut guard, stderr_path) = spawn_run(tmp.path(), &["--no-validate"]);
    wait_for_log_line(
        &stderr_path,
        "starting graph (live)",
        Duration::from_secs(60),
    );
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(40))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    assert!(
        status.success(),
        "a surviving library runs under --no-validate, got {status:?}; stderr:\n{}",
        read_file(&stderr_path)
    );

    // ---- (b) TRULY GONE: crate AND library ⇒ no escape, and none offered.
    // Note the report emits NO `cdylib` check here (the loop stops at the
    // missing directory), so this arm is what proves the node-crate branch
    // consults the cdylib cache rather than leaning on the cdylib check.
    let tmp = tempfile::tempdir().unwrap();
    build_workspace(tmp.path(), "gonecrate", TRUE_SCHEMA);
    std::fs::remove_dir_all(tmp.path().join("nodes/ticker")).expect("delete the crate");
    std::fs::remove_file(tmp.path().join("target/debug").join(dylib_file("ticker")))
        .expect("delete the library");

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(!status.success(), "refused; stderr:\n{stderr}");
    assert!(
        !stderr.contains("re-run with `--no-validate` to run anyway"),
        "the flag cannot help once the library is gone, so it must not be \
         offered; stderr:\n{stderr}"
    );

    let (mut guard, stderr_path) = spawn_run(tmp.path(), &["--no-validate"]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    assert!(
        !status.success(),
        "--no-validate must not run a graph with no library; stderr:\n{}",
        read_file(&stderr_path)
    );
}

/// An unwired macro trigger is a no-escape too.
///
/// `GraphRuntime::build` runs `resolve_macro_data_trigger_input` itself, so a
/// `#[input(trigger)]` the graph never wires refuses again under
/// `--no-validate` — without this label the report advertises the flag,
/// and the flag then fails with the same sentence one layer down. Unrecognized
/// labels fall through to "bypassable", which is the right default and the
/// wrong answer here.
#[test]
#[serial]
fn an_unwired_trigger_input_is_refused_without_promising_a_false_escape() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/relay/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    // The node declares `#[input(trigger)] trigger_in`; the graph wires nothing.
    let fixture = "test_node_macro_data_trigger_cdylib";
    let src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures")
        .join(fixture)
        .join("src/lib.rs");
    std::fs::copy(&src, root.join("nodes/relay/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        fixture_cdylib(fixture),
        root.join("target/debug").join(dylib_file("relay")),
    )
    .expect("copy fixture cdylib");
    std::fs::write(
        root.join("graphs/demo.yaml"),
        "prefix: trig\nnodes:\n  - id: relay\n    type: relay\n",
    )
    .unwrap();

    let (mut guard, stderr_path) = spawn_run(root, &[]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);

    assert!(
        !status.success(),
        "an unwired trigger refuses; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("data_trigger binding 'relay'"),
        "the refusal names the failing check; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains("re-run with `--no-validate` to run anyway"),
        "the runtime re-enforces trigger wiring, so the flag must not be \
         advertised; stderr:\n{stderr}"
    );

    // And it really cannot rescue it.
    let (mut guard, stderr_path) = spawn_run(root, &["--no-validate"]);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .unwrap_or_else(|| panic!("run did not exit; stderr:\n{}", read_file(&stderr_path)));
    let stderr = read_file(&stderr_path);
    assert!(
        !status.success(),
        "--no-validate must not run a graph with an unwired trigger; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("trigger_in"),
        "and it reaches the same, accurate message; stderr:\n{stderr}"
    );
}
