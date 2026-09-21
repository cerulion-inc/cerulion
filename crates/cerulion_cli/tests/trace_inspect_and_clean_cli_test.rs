// SPDX-License-Identifier: AGPL-3.0-only
//! The `cli-tui-clean-trace` coverage gap: the "trace inspect" and "clean"
//! halves (NOT the TUI itself, which already has 8 behavioral `TestBackend`
//! render tests per the repository's test table).
//!
//! (a) `cerulion trace inspect <dir>` reads `trace_*.jsonl` bag files and
//! prints a human-readable timeline (`cerulion_cli/src/main.rs`'s
//! `trace_inspect` + `parse_jsonl_record`). Neither function had ANY test
//! coverage before this file. Reading `parse_jsonl_record` closely surfaced
//! a real, pre-existing bug — fixed in the same commit as this test file
//! (see the comment on `parse_jsonl_record` in `main.rs`): the
//! numeric-field extraction used an inclusive slice (`rest[..=end]` with
//! `end` the delimiter's OWN index) that captured the trailing `,`/`}`
//! delimiter INTO the substring handed to `.parse::<u64>()`, which always
//! rejects trailing punctuation. Since `seq`/`ts_ns` are never the LAST
//! field in the documented `{"topic":..,"seq":..,"ts_ns":..,"schema_hash":..}`
//! shape, this meant `parse_jsonl_record` returned `None` for EVERY
//! conforming line, and `trace_inspect` printed every well-formed record
//! through the "(malformed)" fallback — the documented pretty-print line
//! (`<topic> seq=.. t=..ns schema=..`) was unreachable dead code. Fixed to
//! slice up to (not including) the delimiter for both branches. This file's
//! happy-path test is the regression pin for that fix, driven through the
//! REAL subprocess — the only way to exercise `parse_jsonl_record` at all,
//! since it is a private fn in a binary crate with no unit-test seam.
//!
//! (b) `cerulion clean` (`main.rs`'s `clean_iceoryx2_state`) sweeps DEAD
//! iceoryx2 nodes. Only the happy path (no dead nodes present) is pinned
//! here — fabricating genuinely-dead iceoryx2 node state is fragile and
//! platform-specific, and is deliberately out of scope for this
//! gap-closing pass; the happy-path behavioral proof (the subcommand runs,
//! reports correctly, and exits 0) is sufficient to close the GAP row.

use std::path::Path;
use std::process::Command;

use serial_test::serial;

/// Spawn `cerulion <args>` in `cwd`, wait for it to exit (these are all
/// short-lived one-shot commands — no signals needed), and return
/// `(exit_success, stdout, stderr)`.
fn run_cerulion(args: &[&str], cwd: &Path) -> (bool, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(cwd)
        .output()
        .unwrap_or_else(|e| panic!("spawn cerulion {args:?}: {e}"));
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// Hand-build 2 `trace_*.jsonl` files: 4 well-formed records (2 topics × 2
/// records each, with distinct seq/ts_ns/schema_hash so filter/limit/
/// reverse are each observably distinguishable) + 1 line that doesn't parse
/// at all. Read in lexicographic FILE order, then within-file line order:
///
/// 1. seq=1 /a/b   (file 0000, line 1)
/// 2. malformed    (file 0000, line 2)
/// 3. seq=2 /c/d   (file 0000, line 3)
/// 4. seq=3 /a/b   (file 0001, line 1)
/// 5. seq=4 /c/d   (file 0001, line 2)
fn build_trace_dir(dir: &Path) {
    std::fs::create_dir_all(dir).unwrap();
    std::fs::write(
        dir.join("trace_0000.jsonl"),
        r#"{"topic":"/a/b","seq":1,"ts_ns":1111,"schema_hash":"0x1A"}
not a json line at all
{"topic":"/c/d","seq":2,"ts_ns":2222,"schema_hash":"0x2B"}
"#,
    )
    .unwrap();
    std::fs::write(
        dir.join("trace_0001.jsonl"),
        r#"{"topic":"/a/b","seq":3,"ts_ns":3333,"schema_hash":"0x3C"}
{"topic":"/c/d","seq":4,"ts_ns":4444,"schema_hash":"0x4D"}
"#,
    )
    .unwrap();
}

// Exact `println!("{} seq={} t={}ns schema=0x{:016X}", topic, seq, ts,
// schema)` renderings from `trace_inspect` (uppercase, zero-padded-16 hex —
// distinct from `topic_cmd::topic_echo`'s lowercase `{:016x}`).
const SEQ1: &str = "/a/b seq=1 t=1111ns schema=0x000000000000001A";
const SEQ2: &str = "/c/d seq=2 t=2222ns schema=0x000000000000002B";
const SEQ3: &str = "/a/b seq=3 t=3333ns schema=0x000000000000003C";
const SEQ4: &str = "/c/d seq=4 t=4444ns schema=0x000000000000004D";
const MALFORMED: &str = "(malformed) not a json line at all";

/// No flags: every line renders in FILE order (both files, lexicographic),
/// well-formed lines as the pretty `<topic> seq=.. t=..ns schema=..` format,
/// the unparseable line passed through as `(malformed) <line>`, and the
/// trailing summary names the total record + file counts.
#[test]
fn trace_inspect_no_flags_renders_all_records_in_file_order() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &["trace", "inspect", tmp.path().to_str().unwrap()],
        tmp.path(),
    );
    assert!(ok, "trace inspect must exit 0; stderr:\n{stderr}");

    for (needle, next) in [
        (SEQ1, MALFORMED),
        (MALFORMED, SEQ2),
        (SEQ2, SEQ3),
        (SEQ3, SEQ4),
    ] {
        let a = stdout
            .find(needle)
            .unwrap_or_else(|| panic!("missing {needle:?}; stdout:\n{stdout}"));
        let b = stdout
            .find(next)
            .unwrap_or_else(|| panic!("missing {next:?}; stdout:\n{stdout}"));
        assert!(
            a < b,
            "{needle:?} must render BEFORE {next:?} (file/line order); stdout:\n{stdout}"
        );
    }
    assert!(
        stdout.contains("[5 record(s) across 2 file(s)]"),
        "summary line missing or wrong; stdout:\n{stdout}"
    );
}

/// `--filter <topic>` restricts to exact-topic-substring matches only, and
/// the summary names the filter.
#[test]
fn trace_inspect_filter_restricts_to_matching_topic() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &[
            "trace",
            "inspect",
            tmp.path().to_str().unwrap(),
            "--filter",
            "/a/b",
        ],
        tmp.path(),
    );
    assert!(ok, "trace inspect --filter must exit 0; stderr:\n{stderr}");

    assert!(stdout.contains(SEQ1), "stdout:\n{stdout}");
    assert!(stdout.contains(SEQ3), "stdout:\n{stdout}");
    assert!(
        !stdout.contains(SEQ2) && !stdout.contains(SEQ4) && !stdout.contains("(malformed)"),
        "filter must exclude non-matching topics and the malformed line; stdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[2 record(s) across 2 file(s) filtered by topic=\"/a/b\"]"),
        "summary must name the filter; stdout:\n{stdout}"
    );
}

/// `--limit N --reverse`: the source reverses FIRST, then truncates — so the
/// surviving records are the LAST N in file order (most-recent-first), not
/// the first N then reversed. This is the order-of-operations discriminator:
/// a truncate-then-reverse bug would instead show `[malformed, seq=1]`.
#[test]
fn trace_inspect_limit_and_reverse_truncates_after_reversing() {
    let tmp = tempfile::tempdir().unwrap();
    build_trace_dir(tmp.path());

    let (ok, stdout, stderr) = run_cerulion(
        &[
            "trace",
            "inspect",
            tmp.path().to_str().unwrap(),
            "--limit",
            "2",
            "--reverse",
        ],
        tmp.path(),
    );
    assert!(
        ok,
        "trace inspect --limit --reverse must exit 0; stderr:\n{stderr}"
    );

    let seq4_at = stdout
        .find(SEQ4)
        .unwrap_or_else(|| panic!("missing {SEQ4:?}; stdout:\n{stdout}"));
    let seq3_at = stdout
        .find(SEQ3)
        .unwrap_or_else(|| panic!("missing {SEQ3:?}; stdout:\n{stdout}"));
    assert!(
        seq4_at < seq3_at,
        "reverse-then-truncate must show seq=4 before seq=3; stdout:\n{stdout}"
    );
    assert!(
        !stdout.contains(SEQ1) && !stdout.contains(SEQ2) && !stdout.contains("(malformed)"),
        "truncation to 2 (after reversing) must drop everything but seq=4/seq=3; \
         stdout:\n{stdout}"
    );
    assert!(
        stdout.contains("[2 record(s) across 2 file(s)]"),
        "summary count must reflect the post-truncation total; stdout:\n{stdout}"
    );
}

/// `cerulion clean` happy path: no dead iceoryx2 nodes present. Scope note
/// (see the module doc): this only pins the happy path, and `clean` operates
/// on the process-global iceoryx2 default namespace (same as every other
/// `cerulion` subprocess in this repo) rather than an isolated one, so
/// `#[serial]` here only guards against a hypothetical future SHM-heavy test
/// landing in THIS file — it cannot guarantee isolation from other
/// concurrently-running test BINARIES on the same machine.
///
/// Gotcha: a long-lived shared machine accumulates
/// genuinely-dead iceoryx2 nodes from unrelated prior test runs, so a bare
/// "clean reports nothing to clean" assertion is NOT robust to machine history —
/// it fails on its first run wherever stale state has accumulated.
/// Sweep first (this run's own report is unconstrained — it may legitimately
/// clean 0 or more nodes left by earlier, unrelated runs), then assert the
/// SECOND invocation converges to "nothing to clean" — a convergence
/// property, not a pristine-environment assumption.
#[test]
#[serial]
fn clean_happy_path_reports_nothing_to_clean() {
    let tmp = tempfile::tempdir().unwrap();
    let (ok, _stdout, stderr) = run_cerulion(&["clean"], tmp.path());
    assert!(ok, "cerulion clean (sweep) must exit 0; stderr:\n{stderr}");

    let (ok, stdout, stderr) = run_cerulion(&["clean"], tmp.path());
    assert!(ok, "cerulion clean must exit 0; stderr:\n{stderr}");
    assert!(
        stdout.contains("No dead iceoryx2 nodes found — nothing to clean."),
        "a second immediate `clean` must find nothing left (convergence) — stdout:\n{stdout}"
    );
}
