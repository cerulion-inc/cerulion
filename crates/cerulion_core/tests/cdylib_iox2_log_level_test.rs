// SPDX-License-Identifier: AGPL-3.0-only
//! `IOX2_LOG_LEVEL` reaches a **cdylib node's own** `iceoryx2-log`
//! static — the half of the bug that was actually inert.
//!
//! # Why this file exists separately from `iox2_log_level_test.rs`
//!
//! The HOST half already worked: `TransportManager::init*` has always called
//! `set_log_level_from_env_or(Error)`, so a host-emitted iceoryx2 warning
//! honored the env var. Yet on a running robot, `IOX2_LOG_LEVEL=error` (190 MB /
//! 30 s) and `IOX2_LOG_LEVEL=FATAL` (125 MB / 25 s) both left the flood
//! running.
//!
//! The reason is that iceoryx2's level lives in a `static AtomicU8` **inside the
//! `iceoryx2-log` crate**, and a cdylib node statically links its OWN copy of
//! `cerulion_core → iceoryx2 → iceoryx2-log`. The host initializing its static
//! does nothing for the cdylib's, which stayed at `iceoryx2-log`'s crate default
//! (`Info = 2`) — and the publisher doing the flooding (the `dds_bridge` node's
//! `RawIngressRoute`) lives in the cdylib. Same root shape as the cdylib tracing stopgap, where
//! the cdylib's `tracing` dispatcher was likewise a separate static.
//!
//! The macro-generated `cerulion_node_init` now calls
//! `cerulion_core::iceoryx_logger::init_iceoryx_log_level` with the node's
//! FROZEN env snapshot (replay determinism), right beside the tracing-stopgap
//! `install_cdylib_stderr_tracing` call.
//!
//! # What is asserted
//!
//! The `test_node_discard_probe_cdylib` fixture, under the env-gated
//! `CER_CDYLIB_IOX2_PROBE` switch (INERT for every tracing-stopgap test), prints the level
//! of ITS OWN linked copy via `iceoryx_logger::current_iox2_log_level()`. The
//! child loads that cdylib in a real `GraphRuntime`; the parent reads the
//! printed number off the child's stderr.
//!
//! * absent `IOX2_LOG_LEVEL` ⇒ the cdylib reports `ERROR_LEVEL` (4), NOT
//!   iceoryx2's crate default `INFO_LEVEL` (2). **This is the regression pin**:
//!   deleting the `init_iceoryx_log_level` call from `cerulion_macros`'
//!   `gen_cdylib` makes the cdylib report 2 and this test fail.
//! * `IOX2_LOG_LEVEL=warn` ⇒ the cdylib reports `WARN_LEVEL` (3) — the
//!   anti-tautology arm: the value is genuinely plumbed from the environment,
//!   not a hardcoded constant.
//!
//! # The RAW-FFI half
//!
//! `gen_cdylib` covers every `#[cerulion_node]` node, but a HAND-WRITTEN raw-FFI
//! cdylib's `cerulion_node_init` is one the macro never touches — and at the time of the fix
//! the product shipped exactly such a node, `cerulion_viz/nodes/rerun_sink`,
//! staged into every `cerulion ros2 attach` graph as the viz node and a real
//! transport consumer. The sweep MISSED it, because nothing enumerated
//! hand-written inits. (A later change deleted that crate outright — with the
//! robot-side staging removed it had no caller — so today the walk's only
//! non-excluded hit is the scaffolding TEMPLATE. The guard is unchanged and
//! matters MORE for it: the template is the seed every user-authored raw-FFI
//! node is copied from, and the next hand-written init anyone adds is caught by
//! construction.) Two tests cover this half, with deliberately different (and
//! explicitly scoped) strengths:
//!
//! * [`raw_ffi_cdylib_defaults_to_error_not_iceoryx2s_info`] +
//!   [`raw_ffi_cdylib_honors_an_explicit_env_level`] — BEHAVIORAL, over a real
//!   `dlopen` of the raw-FFI `test_node_cdylib` fixture: a hand-written
//!   `cerulion_node_init` calling `init_iceoryx_log_level` really does move
//!   that cdylib's own static. This proves the PATTERN works; it does not
//!   observe any particular production node (a cdylib's `iceoryx2-log` static is
//!   not an exported symbol, so nothing outside the cdylib can read it back
//!   without a probe export, and a production node is not the place for a test
//!   probe).
//! * [`every_hand_written_cdylib_init_applies_the_iox2_log_level`] —
//!   STRUCTURAL, and the one that covers production code (and any node added
//!   later): it WALKS THE REPO for a hand-written `cerulion_node_init`
//!   DEFINITION and asserts every non-excluded hit calls
//!   `init_iceoryx_log_level`. Both probes run over a COMMENT-STRIPPED view of
//!   each file, so prose that merely quotes the signature never becomes a
//!   required-to-comply "hand-written init", and a call that is commented out
//!   does not satisfy the requirement. Deleting the call from the template
//!   fails it, and so does ADDING a new hand-written init without it — the
//!   whole point, since the raw-FFI half was missed precisely because
//!   nothing enumerated hand-written inits automatically.
//!
//! # Running (needs the fixtures built first)
//!
//! ```bash
//! cargo build -p test_node_discard_probe_cdylib -p test_node_cdylib
//! cargo test -p cerulion_core --test cdylib_iox2_log_level_test -- --test-threads=1
//! ```

use std::io::Read;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use serial_test::serial;

/// Set to "1" ONLY on the spawned child, so a bare `-- --ignored` run no-ops.
const ENV_CHILD: &str = "CER_CDYLIB_LEVEL_CHILD";
/// Consumed by the FIXTURE: gate for its level probe.
const ENV_PROBE: &str = "CER_CDYLIB_IOX2_PROBE";
/// The MACRO fixture's probe line prefix.
const LEVEL_PREFIX: &str = "CDYLIB_IOX2_LEVEL=";
/// The RAW-FFI fixture's probe line prefix (`test_node_cdylib`).
const RAWFFI_LEVEL_PREFIX: &str = "RAWFFI_IOX2_LEVEL=";

/// `iceoryx2_log::LogLevel` discriminants (`Trace = 0 … Fatal = 5`). Hand-written
/// oracles; `level_discriminants_have_not_drifted` guards them against an
/// upstream reordering.
const INFO_LEVEL: u8 = 2;
const WARN_LEVEL: u8 = 3;
const ERROR_LEVEL: u8 = 4;

/// The fixture is `period_ms = 10`; step by the period so it fires.
const STEP: Duration = Duration::from_millis(10);
const CHILD_STEPS: usize = 3;
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);

fn discard_probe_cdylib_path() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_discard_probe_cdylib")
}

fn assert_fixture_built() {
    let path = discard_probe_cdylib_path();
    assert!(
        path.exists(),
        "discard-probe test cdylib not found at {path:?}. \
         Run `cargo build -p test_node_discard_probe_cdylib` first."
    );
}

fn raw_ffi_cdylib_path() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_cdylib")
}

fn assert_raw_ffi_fixture_built() {
    let path = raw_ffi_cdylib_path();
    assert!(
        path.exists(),
        "raw-FFI test cdylib not found at {path:?}. \
         Run `cargo build -p test_node_cdylib` first."
    );
}

/// A one-node graph loading the fixture cdylib. No consumer — the observable is
/// the child's stderr.
fn build_probe_graph() -> TransportResult<GraphRuntime> {
    let path = discard_probe_cdylib_path();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("probe0".to_string(), Box::new(DylibNodeEntry::load(&path)?));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_level".to_string(),
        prefix: "lvl".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "probe0".to_string(),
            node_type: "discard_probe".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "image_out".to_string(),
                schema: "Image".to_string(),
                max_slice_len: Some(4096),
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
}

/// Child entry point: build the graph (which runs the cdylib's
/// `cerulion_node_init`) and step it so the fixture's probe prints.
#[test]
#[ignore = "child process entry point — driven by the parent tests in this file"]
// P12 exemption, scoped to this fn rather than the file: this is the body of a
// SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
// code IS the channel the parent reads its verdict from. The ban stays armed for
// every other line in this binary, which is the half a file-wide allow gave up.
#[allow(clippy::disallowed_methods)]
fn subprocess_child_cdylib_level_probe() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    let mut rt = build_probe_graph().expect("build cdylib level-probe graph");
    for _ in 0..CHILD_STEPS {
        rt.step(STEP);
    }
    drop(rt);
    std::process::exit(0);
}

/// Spawn the child with a controlled `IOX2_LOG_LEVEL` and return the level the
/// MACRO cdylib reported for its own linked copy.
fn child_reported_level(level: Option<&str>) -> u8 {
    assert_fixture_built();
    child_reported_level_inner("subprocess_child_cdylib_level_probe", LEVEL_PREFIX, level)
}

/// Same, for the RAW-FFI (`test_node_cdylib`) fixture.
fn raw_ffi_child_reported_level(level: Option<&str>) -> u8 {
    assert_raw_ffi_fixture_built();
    child_reported_level_inner(
        "subprocess_child_raw_ffi_level_probe",
        RAWFFI_LEVEL_PREFIX,
        level,
    )
}

fn child_reported_level_inner(child_test: &str, prefix: &str, level: Option<&str>) -> u8 {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = std::process::Command::new(exe);
    cmd.args([
        "--exact",
        child_test,
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(ENV_CHILD, "1")
    .env(ENV_PROBE, "1")
    .stdin(std::process::Stdio::null())
    .stdout(std::process::Stdio::piped())
    .stderr(std::process::Stdio::piped());
    match level {
        Some(v) => {
            cmd.env("IOX2_LOG_LEVEL", v);
        }
        // The repo `.cargo/config.toml` exports IOX2_LOG_LEVEL=error for
        // `cargo test`, so "unset" must be constructed explicitly.
        None => {
            cmd.env_remove("IOX2_LOG_LEVEL");
        }
    }
    let mut child = cmd.spawn().expect("spawn child");
    let mut err = child.stderr.take().expect("child stderr");
    let drain = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = err.read_to_string(&mut buf);
        buf
    });

    let deadline = Instant::now() + CHILD_TIMEOUT;
    loop {
        match child.try_wait().expect("try_wait") {
            Some(_) => break,
            None if Instant::now() > deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("cdylib level child did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    let stderr = drain.join().expect("stderr drain thread");

    let reported: Vec<u8> = stderr
        .lines()
        .filter_map(|l| l.trim().strip_prefix(prefix))
        .filter_map(|v| v.trim().parse::<u8>().ok())
        .collect();
    assert!(
        !reported.is_empty(),
        "the cdylib never printed its iox2 level probe (child={child_test}, \
         level={level:?}); stderr:\n{stderr}"
    );
    // Every report is of the same static — a divergence would mean the level is
    // being mutated mid-run, which nothing should do.
    assert!(
        reported.iter().all(|v| *v == reported[0]),
        "the cdylib reported DIFFERENT levels ({reported:?}) — the level static must be \
         set once at init and stay put; stderr:\n{stderr}"
    );
    reported[0]
}

/// THE REGRESSION PIN: with `IOX2_LOG_LEVEL` unset, the cdylib's OWN copy must
/// sit at Cerulion's default `error` — not at `iceoryx2-log`'s crate default
/// `info`, which is what let node-side `warn!`s flood the robot regardless of
/// the env var.
///
/// Deleting the `init_iceoryx_log_level` block from
/// `cerulion_macros/src/codegen.rs`'s `cerulion_node_init` makes this report
/// `INFO_LEVEL` (2) instead of `ERROR_LEVEL` (4).
#[test]
#[serial]
fn cdylib_defaults_to_error_not_iceoryx2s_info() {
    let reported = child_reported_level(None);
    assert_eq!(
        reported, ERROR_LEVEL,
        "a cdylib node's OWN iceoryx2-log static must be initialized to Cerulion's \
         default `error` ({ERROR_LEVEL}) at cerulion_node_init; {INFO_LEVEL} means it was \
         left at iceoryx2's crate default `info` — the inert-level bug, where the host's \
         set_log_level did nothing for the cdylib's separate static"
    );
}

/// ANTI-TAUTOLOGY: the value is genuinely read from the environment and plumbed
/// through the node's frozen env snapshot — not a hardcoded `error`.
#[test]
#[serial]
fn cdylib_honors_an_explicit_env_level() {
    let reported = child_reported_level(Some("warn"));
    assert_eq!(
        reported, WARN_LEVEL,
        "IOX2_LOG_LEVEL=warn must reach the CDYLIB's own level static \
         (expected {WARN_LEVEL}); a hardcoded default would report {ERROR_LEVEL}"
    );
}

/// Drift guard for the hand-written discriminant oracles above: if
/// `iceoryx2_log::LogLevel` is ever reordered, fail HERE with a clear message
/// rather than silently inverting the assertions in the two tests above.
///
/// Uses the parent process's own linked copy (the same crate + static the cdylib
/// links), driven through the production entry point.
#[test]
#[serial]
fn level_discriminants_have_not_drifted() {
    use cerulion_core::iceoryx_logger::{current_iox2_log_level, init_iceoryx_log_level};
    init_iceoryx_log_level(Some("info"));
    assert_eq!(
        current_iox2_log_level(),
        INFO_LEVEL,
        "LogLevel::Info drifted"
    );
    init_iceoryx_log_level(Some("warn"));
    assert_eq!(
        current_iox2_log_level(),
        WARN_LEVEL,
        "LogLevel::Warn drifted"
    );
    init_iceoryx_log_level(Some("error"));
    assert_eq!(
        current_iox2_log_level(),
        ERROR_LEVEL,
        "LogLevel::Error drifted"
    );
    // Leave this process at the suite default.
    init_iceoryx_log_level(None);
}

// ============================================================
// The RAW-FFI half. See the module docs for why the
// behavioral arms use a FIXTURE and the production node is covered structurally.
// ============================================================

/// Child entry point for the RAW-FFI fixture: loading it runs its hand-written
/// `cerulion_node_init`, which prints the probe line.
#[test]
#[ignore = "child process entry point — driven by the parent tests in this file"]
// P12 exemption, scoped to this fn rather than the file: this is the body of a
// SELF-RE-EXEC CHILD process — a process entrypoint by construction, whose exit
// code IS the channel the parent reads its verdict from. The ban stays armed for
// every other line in this binary, which is the half a file-wide allow gave up.
#[allow(clippy::disallowed_methods)]
fn subprocess_child_raw_ffi_level_probe() {
    if std::env::var(ENV_CHILD).as_deref() != Ok("1") {
        return;
    }
    // Build through the REAL `GraphRuntime` (not `NodeContext::for_tests`,
    // whose env snapshot is empty): the frozen snapshot the node reads
    // `IOX2_LOG_LEVEL` from is captured by the production build path, so
    // driving anything else would test a different thing. `test_node_cdylib`
    // declares no ports; the probe prints from `cerulion_node_init` itself, so
    // the node never needs to fire.
    let path = raw_ffi_cdylib_path();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "raw0".to_string(),
        Box::new(DylibNodeEntry::load(&path).expect("load raw-FFI fixture")),
    );
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "rawffi_level".to_string(),
        prefix: "lvlr".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "raw0".to_string(),
            node_type: "raw_probe".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };
    let clock = Arc::new(VirtualClock::new());
    let rt = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build raw-FFI level-probe graph");
    drop(rt);
    std::process::exit(0);
}

/// THE RAW-FFI REGRESSION PIN: a HAND-WRITTEN `cerulion_node_init` that applies
/// the level really moves that cdylib's own static — with `IOX2_LOG_LEVEL`
/// unset it sits at Cerulion's default `error`, not at `iceoryx2-log`'s crate
/// default `info`.
///
/// Deleting the `init_iceoryx_log_level` block from
/// `test_fixtures/test_node_cdylib/src/lib.rs`'s `cerulion_node_init` makes
/// this report `INFO_LEVEL` (2) instead of `ERROR_LEVEL` (4).
#[test]
#[serial]
fn raw_ffi_cdylib_defaults_to_error_not_iceoryx2s_info() {
    let reported = raw_ffi_child_reported_level(None);
    assert_eq!(
        reported, ERROR_LEVEL,
        "a HAND-WRITTEN raw-FFI cdylib's own iceoryx2-log static must be initialized to \
         Cerulion's default `error` ({ERROR_LEVEL}) at cerulion_node_init; {INFO_LEVEL} means \
         it was left at iceoryx2's crate default `info` — the inert-level bug. The macro covers \
         `#[cerulion_node]` nodes; a raw-FFI node (anything grown from the \
         `cerulion node create --raw-ffi` template) must do it themselves"
    );
}

/// ANTI-TAUTOLOGY for the raw-FFI arm: the value is genuinely read from the
/// node's frozen env snapshot, not a hardcoded `error`.
#[test]
#[serial]
fn raw_ffi_cdylib_honors_an_explicit_env_level() {
    let reported = raw_ffi_child_reported_level(Some("warn"));
    assert_eq!(
        reported, WARN_LEVEL,
        "IOX2_LOG_LEVEL=warn must reach the RAW-FFI cdylib's own level static \
         (expected {WARN_LEVEL}); a hardcoded default would report {ERROR_LEVEL}"
    );
}

/// The Rust block-comment markers, spelled with an ESCAPED `*` so this file's
/// own source carries no raw opener. `code_only` does not model string literals
/// (see its docs), and the repo walk runs it over THIS very file — a literal
/// `/`+`*` here would open a block comment in that view and swallow the rest of
/// the file, taking with it the `fn cerulion_node_init` probe the walk must
/// still find here (the EXCLUSIONS reach-assert). Verified: reverting these to
/// plain literals fails `every_hand_written_cdylib_init_applies_the_iox2_log_level`.
const BLOCK_OPEN: &str = "/\u{2a}";
const BLOCK_CLOSE: &str = "\u{2a}/";

/// `src` with every COMMENT removed — `//`-to-end-of-line AND block comments
/// (which NEST in Rust, so an inner opener/closer pair inside an outer one is
/// tracked by depth) — so the scan below probes CODE rather than raw text.
///
/// Without this the guard is text-matching prose: its own module doc quotes the
/// signature it searches for (so the guard file scanned ITSELF), any future file
/// that mentions the signature in a comment would become a required-to-comply
/// "hand-written init", and a node whose `init_iceoryx_log_level` call was
/// COMMENTED OUT would still satisfy the requirement. That last hole is why
/// BOTH syntaxes are stripped: handling only `//` left `/* … */` around the call
/// as a working dodge.
///
/// Crude in exactly one direction: `//` and block-comment markers inside STRING
/// LITERALS are not modelled. Both resulting errors point the SAME way — a
/// literal quoting the signature reads as code, so its file is REQUIRED to
/// comply (that is why this file excludes itself, below), and a literal holding
/// an unbalanced opener swallows the rest of the file, so a real call is
/// stripped and the file FAILS LOUDLY. Neither direction lets a hand-written
/// init silently escape the requirement. An unterminated block comment is
/// treated the same way (stripped to EOF) for the same fail-closed reason.
fn code_only(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut rest = src;
    let mut depth = 0usize;
    while let Some(ch) = rest.chars().next() {
        if depth == 0 {
            if rest.starts_with("//") {
                // Line comment: drop through end-of-line, KEEP the newline.
                match rest.find('\n') {
                    Some(nl) => rest = &rest[nl..],
                    None => break,
                }
            } else if rest.starts_with(BLOCK_OPEN) {
                depth = 1;
                rest = &rest[BLOCK_OPEN.len()..];
            } else {
                out.push(ch);
                rest = &rest[ch.len_utf8()..];
            }
        } else if rest.starts_with(BLOCK_OPEN) {
            depth += 1;
            rest = &rest[BLOCK_OPEN.len()..];
        } else if rest.starts_with(BLOCK_CLOSE) {
            depth -= 1;
            rest = &rest[BLOCK_CLOSE.len()..];
        } else {
            // Inside a block comment: emit nothing but the newlines, so the
            // stripped view keeps the file's line structure and a `contains`
            // probe can never match across a line boundary it did not span.
            if ch == '\n' {
                out.push(ch);
            }
            rest = &rest[ch.len_utf8()..];
        }
    }
    out
}

/// Oracle vectors for [`code_only`] — the guard's whole no-false-positive /
/// no-silent-dodge claim rests on this one function, so pin it directly instead
/// of only through the repo walk (which cannot exhibit every shape).
#[test]
fn code_only_strips_both_comment_syntaxes_and_nothing_else() {
    // Fixtures use the ESCAPED markers for the reason `BLOCK_OPEN` documents: a
    // raw opener in THIS file's source would open a block comment in the repo
    // walk's own view of this file and swallow the `fn cerulion_node_init`
    // literal the walk must still find here.
    let (open, close, line) = (BLOCK_OPEN, BLOCK_CLOSE, "//");

    // Line comment: stripped, newline kept.
    assert_eq!(code_only(&format!("a {line} b\nc")), "a \nc");
    // Block comment on one line.
    assert_eq!(code_only(&format!("a {open} b {close} c")), "a  c");
    // Multi-line block comment: content gone, line structure preserved.
    assert_eq!(code_only(&format!("a {open} b\nc\n{close} d")), "a \n\n d");
    // NESTING (Rust block comments nest) — the OUTER close is the right one.
    assert_eq!(
        code_only(&format!("a {open} {open} b {close} c {close} d")),
        "a  d"
    );
    // A block opener inside a LINE comment must not open a block.
    assert_eq!(code_only(&format!("a {line} {open} b\nc")), "a \nc");
    // Unterminated block comment: fail CLOSED (stripped to EOF), so a call
    // sitting after it reads as absent and its file fails the guard LOUDLY
    // rather than passing on text the compiler would never see.
    assert_eq!(
        code_only(&format!("a {open} b\ninit_iceoryx_log_level")),
        "a \n"
    );
    // The two probe substrings, in each hiding place the guard cares about.
    assert!(
        !code_only(&format!("{open} init_iceoryx_log_level(x); {close}"))
            .contains("init_iceoryx_log_level")
    );
    assert!(!code_only(&format!("{line} fn cerulion_node_init")).contains("fn cerulion_node_init"));
    assert!(code_only(&format!("fn cerulion_node_init() {line} note"))
        .contains("fn cerulion_node_init"));
    // Multi-byte characters survive the byte-slicing.
    assert_eq!(code_only(&format!("é {open} ü {close} ø")), "é  ø");
}

/// Every `*.rs` file under `dir`, skipping build artifacts (`target/`) and
/// hidden directories (`.git/` and any other dot-directory).
fn collect_rs_files(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return; // unreadable dir: nothing to scan, never a spurious failure
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if path.is_dir() {
            if name == "target" || name.starts_with('.') {
                continue;
            }
            collect_rs_files(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// THE SWEEP, made durable: every HAND-WRITTEN `cerulion_node_init` in the repo
/// must apply `IOX2_LOG_LEVEL` to its own linked copy.
///
/// The scan WALKS THE TREE rather than reading a hand-maintained list, because a
/// hand-maintained list has the exact failure mode the inert-level bug was: the original
/// sweep missed the then-shipping `cerulion_viz/nodes/rerun_sink` (deleted in
/// a later change) precisely because nothing enumerated hand-written inits
/// automatically, and a list that a new node's author must remember to edit
/// reproduces that miss one-for-one. Here, ADDING an unlisted hand-written init
/// is the FAILING case, not the silent one.
///
/// It is a source scan, so be explicit about what it does and does not prove:
/// it verifies the CALL is present IN CODE — not inside a comment of EITHER
/// syntax (deleting it from a scanned file, or commenting it out with `//` or
/// with `/* … */`, fails this test) — not that the call had an effect, and not
/// that it sits inside the init's own body. The EFFECT is
/// proven behaviorally by the two arms above over a real `dlopen` of a raw-FFI
/// cdylib with the identical init shape.
///
/// The definition probe is `fn cerulion_node_init` over the comment-stripped
/// source — deliberately looser than the `extern "C" fn …` spelling, so the
/// equally-valid `#[no_mangle] pub extern fn cerulion_node_init` (where `"C"` is
/// implied and the exported symbol is identical) cannot dodge the walk.
///
/// Deliberately EXCLUDED, each for a stated reason — and each asserted to still
/// EXIST, so a rename cannot turn an exclusion into a silent hole:
/// * `cerulion_macros/src/codegen.rs` — that is the generator (its emitted
///   init carries the call), pinned behaviorally by
///   `cdylib_defaults_to_error_not_iceoryx2s_info`.
/// * `test_fixtures/**` — fixtures are minimal raw-FFI probes; requiring the
///   call in all of them would change what they pin. `test_node_cdylib` carries
///   it deliberately (it IS the raw-FFI behavioral vehicle above), and
///   `test_node_raw_ffi_template_cdylib` carries it because it is the
///   byte-for-byte emit of the scaffolding template, which does.
/// * this test file — it holds the definition probe as a string literal, which
///   comment-stripping cannot remove, so the walk still sees itself. It defines
///   no cdylib and no init; excluding it explicitly (and asserting the walk
///   still REACHES it, below) is correct, where passing on its own prose was
///   self-satisfying.
#[test]
fn every_hand_written_cdylib_init_applies_the_iox2_log_level() {
    let mut root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root.pop(); // crates/cerulion_core -> crates/
    root.pop(); // crates/ -> workspace root

    /// Paths (relative to the workspace root, `/`-separated) whose hand-written
    /// init is out of scope. A directory prefix ends with `/`.
    const EXCLUSIONS: [&str; 3] = [
        "crates/cerulion_macros/src/codegen.rs",
        "crates/test_fixtures/",
        "crates/cerulion_core/tests/cdylib_iox2_log_level_test.rs",
    ];

    let mut files = Vec::new();
    collect_rs_files(&root, &mut files);
    assert!(
        files.len() > 100,
        "the repo walk found only {} .rs files — it is not walking the tree, so \
         every assertion below would pass vacuously",
        files.len()
    );

    let mut scanned = Vec::new();
    let mut excluded = Vec::new();
    for path in &files {
        let Ok(src) = std::fs::read_to_string(path) else {
            continue;
        };
        let code = code_only(&src);
        if !code.contains("fn cerulion_node_init") {
            continue;
        }
        let rel = path
            .strip_prefix(&root)
            .expect("walked paths are under the root")
            .to_string_lossy()
            .replace(std::path::MAIN_SEPARATOR, "/");
        if EXCLUSIONS
            .iter()
            .any(|ex| rel == *ex || (ex.ends_with('/') && rel.starts_with(ex)))
        {
            excluded.push(rel);
            continue;
        }
        assert!(
            code.contains("init_iceoryx_log_level"),
            "{rel} defines a hand-written `cerulion_node_init` but never calls \
             `cerulion_core::iceoryx_logger::init_iceoryx_log_level`. A cdylib statically \
             links its OWN iceoryx2-log LOG_LEVEL static, so the host's set_log_level does \
             NOTHING for it and it stays at iceoryx2's crate default (Info) — IOX2_LOG_LEVEL \
             is inert for this node. The macro-generated init does this for every \
             #[cerulion_node] cdylib; a raw-FFI node must do it itself (crib the log-level block \
             the macro emits, in `cerulion_macros/src/codegen.rs`, or the copy in the raw-FFI \
             scaffolding template `cerulion_cli_engine/src/templates.rs`). If this file \
             genuinely never touches iceoryx2, add it to EXCLUSIONS in this test WITH a \
             stated reason"
        );
        scanned.push(rel);
    }

    // ANTI-TAUTOLOGY: the walk must actually have found the known
    // hand-written inits. A scan that matches nothing (wrong root, changed
    // signature spelling, a `continue` that swallows everything) would
    // otherwise pass silently — the same "nothing enumerates them" hole.
    //
    // This list held two entries until `cerulion_viz/nodes/rerun_sink` was
    // deleted (visualization is desk-side, so the node lost its last
    // caller). The scaffolding template is the survivor and is
    // the RIGHT anchor: it is the seed `cerulion node create --raw-ffi` emits,
    // so every user-authored raw-FFI node inherits the call by construction.
    // If a production hand-written init is ever added back, list it here too.
    // (Kept as a named array, not inlined, so re-adding an entry is a one-line
    // edit — and so `clippy::single_element_loop` does not fire at length 1.)
    const EXPECTED_HAND_WRITTEN_INITS: [&str; 1] = ["crates/cerulion_cli_engine/src/templates.rs"];
    for expected in EXPECTED_HAND_WRITTEN_INITS {
        assert!(
            scanned.iter().any(|s| s == expected),
            "the walk did not find {expected} among the hand-written inits it \
             scanned ({scanned:?}) — either the file moved (update this list) or \
             the scan is broken and every assertion above is vacuous"
        );
    }
    // Each exclusion must still be REACHED by the walk, so a moved or renamed
    // excluded file surfaces here instead of quietly dropping out of scope.
    for ex in EXCLUSIONS {
        assert!(
            excluded.iter().any(|e| e == ex || e.starts_with(ex)),
            "EXCLUSIONS lists {ex}, but the walk found no hand-written \
             `cerulion_node_init` there (found: {excluded:?}) — the file moved or \
             changed shape, so the exclusion is now dead weight"
        );
    }
}
