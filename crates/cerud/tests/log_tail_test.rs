// SPDX-License-Identifier: AGPL-3.0-only
//! Log-tail verb: path-traversal hardening + tail oracles + e2e.

use std::io::Write;
use std::path::Path;

use cerud::error::CerudError;
use cerud::verbs::log_tail::{resolve_under_root, tail_file, tail_lines, LogTailVerb};
use cerud::verbs::VerbHandler;

#[test]
fn resolve_under_root_accepts_in_root_names() {
    let root = Path::new("/var/log/cerulion");
    assert_eq!(
        resolve_under_root(root, "perception.log").unwrap(),
        Path::new("/var/log/cerulion/perception.log")
    );
    // A nested relative path stays under root.
    assert_eq!(
        resolve_under_root(root, "graphs/planning.log").unwrap(),
        Path::new("/var/log/cerulion/graphs/planning.log")
    );
    // A leading `./` is harmless.
    assert_eq!(
        resolve_under_root(root, "./perception.log").unwrap(),
        Path::new("/var/log/cerulion/./perception.log")
    );
}

#[test]
fn resolve_under_root_refuses_traversal() {
    let root = Path::new("/var/log/cerulion");
    for bad in [
        "",                      // empty
        "..",                    // parent
        "../etc/passwd",         // climb out
        "logs/../../etc/passwd", // climb out mid-path
        "/etc/passwd",           // absolute
        "/",                     // absolute root
    ] {
        match resolve_under_root(root, bad) {
            Err(CerudError::PathTraversal { requested, root: r }) => {
                assert_eq!(requested, bad);
                assert_eq!(r, "/var/log/cerulion");
            }
            other => panic!("expected PathTraversal for {bad:?}, got {other:?}"),
        }
    }
}

#[test]
fn tail_lines_oracles() {
    let content = "l1\nl2\nl3\nl4\nl5\n";
    // Fewer than total → truncated, exactly the last n.
    let (lines, truncated) = tail_lines(content, 2);
    assert_eq!(lines, vec!["l4".to_string(), "l5".to_string()]);
    assert!(truncated);

    // n >= total → all lines, not truncated. A trailing newline is NOT an
    // extra empty line (5 lines, not 6).
    let (lines, truncated) = tail_lines(content, 10);
    assert_eq!(lines.len(), 5);
    assert_eq!(lines[0], "l1");
    assert_eq!(lines[4], "l5");
    assert!(!truncated);

    // Exactly total → not truncated.
    let (_, truncated) = tail_lines(content, 5);
    assert!(!truncated);

    // Empty content → empty tail.
    let (lines, truncated) = tail_lines("", 5);
    assert!(lines.is_empty());
    assert!(!truncated);
}

#[test]
fn log_tail_is_declared_read_only() {
    // Log-tail is read-only → is_mutating = false.
    assert!(!LogTailVerb::new("/var/log/cerulion").is_mutating());
}

#[test]
fn log_tail_verb_returns_last_n_lines() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path();
    std::fs::write(root.join("app.log"), "a\nb\nc\nd\ne\n").unwrap();

    let verb = LogTailVerb::new(root);
    assert_eq!(verb.name(), "log-tail");

    let result = verb
        .execute(&serde_json::json!({"name": "app.log", "lines": 2}))
        .unwrap();
    assert_eq!(result["returned"], 2);
    assert_eq!(result["truncated"], true);
    assert_eq!(result["lines"][0], "d");
    assert_eq!(result["lines"][1], "e");
}

#[test]
fn log_tail_verb_refuses_traversal_args() {
    let dir = tempfile::tempdir().unwrap();
    let verb = LogTailVerb::new(dir.path());
    let err = verb
        .execute(&serde_json::json!({"name": "../../etc/passwd"}))
        .unwrap_err();
    assert!(
        matches!(err, CerudError::PathTraversal { .. }),
        "got {err:?}"
    );
}

#[cfg(unix)]
#[test]
fn log_tail_verb_refuses_symlink_escape() {
    // A symlink UNDER the allow-listed root that points OUTSIDE it must be
    // refused (the lexical guard passes the name, the canonical guard catches
    // it) — and the outside file must never be read.
    let root_dir = tempfile::tempdir().unwrap();
    let outside_dir = tempfile::tempdir().unwrap();
    let secret = outside_dir.path().join("secret.log");
    std::fs::write(&secret, "TOP SECRET\n").unwrap();

    let link = root_dir.path().join("escape.log");
    std::os::unix::fs::symlink(&secret, &link).unwrap();

    let verb = LogTailVerb::new(root_dir.path());
    let err = verb
        .execute(&serde_json::json!({"name": "escape.log"}))
        .unwrap_err();
    assert!(
        matches!(err, CerudError::PathTraversal { .. }),
        "got {err:?}"
    );
}

#[test]
fn tail_file_read_stays_bounded_on_a_large_file() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("big.log");

    // Write a file MUCH larger than the byte budget we will pass.
    let budget: u64 = 4096;
    {
        let mut f = std::fs::File::create(&path).unwrap();
        // 2000 lines of "line-<n>\n" — comfortably > 4 KiB.
        for i in 0..2000u64 {
            writeln!(f, "line-{i}").unwrap();
        }
    }
    let file_len = std::fs::metadata(&path).unwrap().len();
    assert!(file_len > budget * 4, "file must dwarf the budget");

    let outcome = tail_file(&path, 5, budget).unwrap();
    // The read is BOUNDED: never more than the budget bytes are scanned (memory
    // does not scale with file size).
    assert!(
        outcome.bytes_scanned <= budget,
        "bytes_scanned {} exceeded budget {budget}",
        outcome.bytes_scanned
    );
    // Only the tail is returned, and `truncated` reports that more existed.
    assert_eq!(outcome.lines.len(), 5);
    assert_eq!(outcome.lines[4], "line-1999");
    assert!(outcome.truncated);
}

#[test]
fn tail_file_lossy_decodes_non_utf8_logs() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("binary.log");
    // Two lines with invalid UTF-8 bytes (0xFF is never valid UTF-8).
    std::fs::write(&path, b"first\xff\xfeline\nsecond\xffline\n").unwrap();

    // Lossy decode: the tail is returned (with replacement chars), NOT an error.
    let outcome = tail_file(&path, 10, 4096).unwrap();
    assert_eq!(outcome.lines.len(), 2);
    assert!(outcome.lines[0].contains("first"));
    assert!(outcome.lines[1].contains("second"));
    assert!(!outcome.truncated);
}

#[test]
fn log_tail_verb_reports_bytes_scanned() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("app.log"), "a\nb\nc\n").unwrap();
    let verb = LogTailVerb::new(dir.path());
    let result = verb
        .execute(&serde_json::json!({"name": "app.log"}))
        .unwrap();
    // The bounded-read accounting is surfaced to the caller.
    assert!(result["bytes_scanned"].as_u64().is_some());
    assert_eq!(result["bytes_scanned"], 6);
}

#[test]
fn log_tail_verb_missing_file_is_a_clean_error() {
    let dir = tempfile::tempdir().unwrap();
    let verb = LogTailVerb::new(dir.path());
    let err = verb
        .execute(&serde_json::json!({"name": "nope.log"}))
        .unwrap_err();
    match err {
        CerudError::Verb(msg) => assert!(msg.contains("no such log")),
        other => panic!("expected Verb error, got {other:?}"),
    }
}
