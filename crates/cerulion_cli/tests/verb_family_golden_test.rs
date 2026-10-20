// SPDX-License-Identifier: AGPL-3.0-only
//! Byte-for-byte pins of the `schema` verb family over the REAL binary
//! (`CARGO_BIN_EXE_cerulion`): its help pages, its usage errors, a
//! representative run of each verb, and the shell-completion protocol for
//! every supported shell.
//!
//! `schema` is dispatched through the verb-family registry
//! (`src/families/`), not the central `Commands` enum. Moving a family onto
//! the registry must change nothing a user or a script can observe, so every
//! golden file under `tests/golden/verb_family/` was captured from the binary
//! BEFORE the family moved and is never regenerated from the code under test.
//! A diff here is a user-visible change: fix the code, do not re-capture.
//!
//! Each case is one subprocess under an isolated `HOME`, an empty cwd with no
//! workspace, the network kill switch and telemetry off, so there is no
//! shared state and no `#[serial]`.

use std::path::{Path, PathBuf};
use std::process::Command;

/// How a case drives the binary.
enum Mode {
    /// A plain invocation: `cerulion <args..>`.
    Run,
    /// The `COMPLETE=<shell>` protocol the installed shell code speaks,
    /// completing the LAST word of `cerulion <args..>`.
    Complete(&'static str),
    /// `COMPLETE=<shell> cerulion` with no words: the registration script.
    Register(&'static str),
}

struct Case {
    /// The golden file stem under `tests/golden/verb_family/`.
    name: &'static str,
    mode: Mode,
    args: &'static [&'static str],
}

const SHELLS: [&str; 5] = ["bash", "zsh", "fish", "elvish", "powershell"];

/// The completion inputs replayed for every shell: the top-level verb list,
/// a prefix that selects the family, the family's own verbs, a live schema
/// name source, and a flag list.
const COMPLETION_INPUTS: [(&str, &[&str]); 6] = [
    ("top", &[""]),
    ("prefix", &["sch"]),
    ("verbs", &["schema", ""]),
    ("info_name", &["schema", "info", "sensor_msgs/Ima"]),
    ("delete_name", &["schema", "delete", "geometry_msgs/Pose"]),
    ("flags", &["schema", "list", "--"]),
];

fn cases() -> Vec<Case> {
    let mut cases = vec![
        // Help: the root listing (where `schema` sits in it), every spelling of
        // the family's own page, and each verb's page.
        Case {
            name: "root_help",
            mode: Mode::Run,
            args: &["--help"],
        },
        Case {
            name: "root_h",
            mode: Mode::Run,
            args: &["-h"],
        },
        Case {
            name: "root_help_verb",
            mode: Mode::Run,
            args: &["help"],
        },
        Case {
            name: "root_bare",
            mode: Mode::Run,
            args: &[],
        },
        Case {
            name: "schema_help",
            mode: Mode::Run,
            args: &["schema", "--help"],
        },
        Case {
            name: "schema_h",
            mode: Mode::Run,
            args: &["schema", "-h"],
        },
        Case {
            name: "help_schema",
            mode: Mode::Run,
            args: &["help", "schema"],
        },
        Case {
            name: "help_schema_info",
            mode: Mode::Run,
            args: &["help", "schema", "info"],
        },
        Case {
            name: "schema_bare",
            mode: Mode::Run,
            args: &["schema"],
        },
        Case {
            name: "schema_create_help",
            mode: Mode::Run,
            args: &["schema", "create", "--help"],
        },
        Case {
            name: "schema_delete_help",
            mode: Mode::Run,
            args: &["schema", "delete", "--help"],
        },
        Case {
            name: "schema_info_help",
            mode: Mode::Run,
            args: &["schema", "info", "--help"],
        },
        Case {
            name: "schema_list_help",
            mode: Mode::Run,
            args: &["schema", "list", "--help"],
        },
        // Usage errors (clap's exit 2).
        Case {
            name: "schema_unknown_verb",
            mode: Mode::Run,
            args: &["schema", "bogus"],
        },
        Case {
            name: "schema_info_missing_name",
            mode: Mode::Run,
            args: &["schema", "info"],
        },
        Case {
            name: "schema_list_extra_arg",
            mode: Mode::Run,
            args: &["schema", "list", "x"],
        },
        // The global flag still parses after the family verb.
        Case {
            name: "schema_list_verbose",
            mode: Mode::Run,
            args: &["schema", "list", "-v"],
        },
        // Representative runs with no workspace: the built-in registry, a
        // built-in type, a miss, and the two writers refusing.
        Case {
            name: "schema_list",
            mode: Mode::Run,
            args: &["schema", "list"],
        },
        Case {
            name: "schema_info_builtin",
            mode: Mode::Run,
            args: &["schema", "info", "geometry_msgs/Point"],
        },
        Case {
            name: "schema_info_missing",
            mode: Mode::Run,
            args: &["schema", "info", "NoSuchType"],
        },
        Case {
            name: "schema_create_no_ws",
            mode: Mode::Run,
            args: &["schema", "create", "Foo"],
        },
        Case {
            name: "schema_delete_no_ws",
            mode: Mode::Run,
            args: &["schema", "delete", "Foo"],
        },
    ];
    for shell in SHELLS {
        cases.push(Case {
            name: leak(format!("register_{shell}")),
            mode: Mode::Register(shell),
            args: &[],
        });
        for (input, words) in COMPLETION_INPUTS {
            cases.push(Case {
                name: leak(format!("complete_{shell}_{input}")),
                mode: Mode::Complete(shell),
                args: words,
            });
        }
    }
    cases
}

fn leak(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

/// The record separator the generated bash and zsh functions forward as
/// `_CLAP_IFS`.
const IFS: &str = "\u{0b}";

/// Run one case and render everything observable: exit code, stdout, stderr.
fn observe(case: &Case, home: &Path, cwd: &Path) -> String {
    let bin = env!("CARGO_BIN_EXE_cerulion");
    let mut cmd = Command::new(bin);
    cmd.env("HOME", home)
        .env("CERULION_NETWORK", "off")
        .env("CERULION_LOGIN_GATE", "off")
        .env("DO_NOT_TRACK", "1")
        .env("NO_COLOR", "1")
        .env_remove("RUST_LOG")
        .env_remove("COMPLETE")
        .env_remove("CERULION_PEERS")
        .current_dir(cwd);
    match case.mode {
        Mode::Run => {
            cmd.args(case.args);
        }
        Mode::Register(shell) => {
            cmd.env("COMPLETE", shell);
        }
        Mode::Complete(shell) => {
            cmd.env("COMPLETE", shell)
                .env("_CLAP_IFS", IFS)
                .env("_CLAP_COMPLETE_INDEX", case.args.len().to_string())
                .arg("--")
                .arg("cerulion")
                .args(case.args);
        }
    }
    let out = cmd.output().expect("run cerulion");
    // The registration script names the binary by its absolute path and a
    // workspace-not-found error names the searched cwd; both differ per run
    // and are not part of what this file pins.
    // A `-v` log line opens with a wall-clock timestamp and names a source
    // line; both are masked the same way.
    let canonical_cwd = cwd.canonicalize().expect("canonical cwd");
    let norm = |b: &[u8]| {
        String::from_utf8_lossy(b)
            .replace(bin, "<BIN>")
            .replace(&*canonical_cwd.to_string_lossy(), "<CWD>")
            .replace(&*cwd.to_string_lossy(), "<CWD>")
            // Spelled as a token so the capture carries no typographic dash.
            .replace('\u{2014}', "<U+2014>")
            .split_inclusive('\n')
            .map(mask_log_line)
            .collect::<String>()
    };
    format!(
        "exit: {:?}\n--- stdout\n{}\n--- stderr\n{}",
        out.status.code(),
        norm(&out.stdout),
        norm(&out.stderr)
    )
}

/// Length of a leading RFC 3339 UTC timestamp and its trailing space
/// (`2026-01-02T03:04:05.123456Z `), or `None` when the line does not open
/// with one. Every byte of the prefix is checked and every date and time
/// field must be in range, so a malformed stamp stays in the capture and
/// fails the golden comparison.
fn timestamp_prefix_len(line: &str) -> Option<usize> {
    const SHAPE: &[u8; 19] = b"dddd-dd-ddTdd:dd:dd";
    let bytes = line.as_bytes();
    let head = bytes.get(..SHAPE.len())?;
    let shaped = head.iter().zip(SHAPE).all(|(b, s)| match s {
        b'd' => b.is_ascii_digit(),
        sep => b == sep,
    });
    if !shaped {
        return None;
    }
    // Two ASCII digits at `at`, already checked by the shape above.
    let field = |at: usize| u32::from(head[at] - b'0') * 10 + u32::from(head[at + 1] - b'0');
    let in_range = (1..=12).contains(&field(5))
        && (1..=31).contains(&field(8))
        && field(11) <= 23
        && field(14) <= 59
        // 60 is a leap second, which RFC 3339 allows.
        && field(17) <= 60;
    if !in_range {
        return None;
    }
    let mut end = SHAPE.len();
    if bytes.get(end) == Some(&b'.') {
        let digits = bytes[end + 1..]
            .iter()
            .take_while(|b| b.is_ascii_digit())
            .count();
        if digits == 0 {
            return None;
        }
        end += 1 + digits;
    }
    (bytes.get(end..end + 2) == Some(b"Z ")).then_some(end + 2)
}

/// Mask one log line: a leading RFC 3339 UTC timestamp
/// (`2026-01-02T03:04:05.123456Z `) becomes `<TS> ` and every `.rs:<digits>`
/// source location becomes `.rs:<LINE>`. Any other line is unchanged.
fn mask_log_line(line: &str) -> String {
    let line = match timestamp_prefix_len(line) {
        Some(end) => format!("<TS> {}", &line[end..]),
        None => line.to_string(),
    };
    let mut out = String::with_capacity(line.len());
    let mut rest = line.as_str();
    while let Some(at) = rest.find(".rs:") {
        let (head, tail) = rest.split_at(at + 4);
        out.push_str(head);
        let digits = tail.bytes().take_while(u8::is_ascii_digit).count();
        if digits > 0 {
            out.push_str("<LINE>");
        }
        rest = &tail[digits..];
    }
    out.push_str(rest);
    out
}

fn golden_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden/verb_family")
}

#[test]
fn the_schema_family_is_byte_identical_to_its_pre_registry_capture() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let home = tmp.path().join("home");
    let cwd = tmp.path().join("cwd");
    std::fs::create_dir_all(home.join(".cerulion")).expect("home");
    std::fs::create_dir_all(&cwd).expect("cwd");

    let mut drift = Vec::new();
    for case in cases() {
        let got = observe(&case, &home, &cwd);
        let path = golden_dir().join(format!("{}.txt", case.name));
        let want = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("golden {} unreadable: {e}", path.display()));
        if got != want {
            drift.push(format!(
                "== {} ==\n--- want\n{want}\n--- got\n{got}",
                case.name
            ));
        }
    }
    assert!(
        drift.is_empty(),
        "{} case(s) drifted from the pre-registry capture:\n{}",
        drift.len(),
        drift.join("\n")
    );
}

#[test]
fn every_golden_file_belongs_to_a_case() {
    // A stale file would pin nothing and hide a deleted case.
    let names: Vec<String> = cases().iter().map(|c| format!("{}.txt", c.name)).collect();
    let mut stale = Vec::new();
    for entry in std::fs::read_dir(golden_dir()).expect("golden dir") {
        let file = entry
            .expect("entry")
            .file_name()
            .to_string_lossy()
            .into_owned();
        if !names.contains(&file) {
            stale.push(file);
        }
    }
    assert!(stale.is_empty(), "golden files with no case: {stale:?}");
}

#[test]
fn a_well_formed_timestamp_prefix_is_masked() {
    assert_eq!(
        mask_log_line("2026-01-02T03:04:05.123456Z  INFO a.rs:12: hi\n"),
        "<TS>  INFO a.rs:<LINE>: hi\n"
    );
    assert_eq!(
        mask_log_line("2026-01-02T03:04:05Z WARN x\n"),
        "<TS> WARN x\n"
    );
    // The edges of every field's range still mask.
    assert_eq!(
        mask_log_line("2026-12-31T23:59:60Z WARN x\n"),
        "<TS> WARN x\n"
    );
    assert_eq!(
        mask_log_line("2026-01-01T00:00:00Z WARN x\n"),
        "<TS> WARN x\n"
    );
}

#[test]
fn a_malformed_timestamp_prefix_is_left_in_the_capture() {
    for line in [
        "2026-01-02TgarbledZ INFO x\n",
        "2026-01-02T03:04:05.Z INFO x\n",
        "2026-01-02T03:04:05.123456 INFO x\n",
        "2026-01-02T03-04-05.123456Z INFO x\n",
        "2026-01-02 03:04:05.123456Z INFO x\n",
        "2026-01-02T03:04:05.123456Z\n",
        "2026-01-02T03:04\n",
        "9999-99-99T99:99:99Z INFO x\n",
        "2026-00-02T03:04:05Z INFO x\n",
        "2026-13-02T03:04:05Z INFO x\n",
        "2026-01-00T03:04:05Z INFO x\n",
        "2026-01-32T03:04:05Z INFO x\n",
        "2026-01-02T24:04:05Z INFO x\n",
        "2026-01-02T03:60:05Z INFO x\n",
        "2026-01-02T03:04:61Z INFO x\n",
        "",
    ] {
        assert_eq!(mask_log_line(line), line, "{line:?}");
    }
}
