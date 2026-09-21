// SPDX-License-Identifier: AGPL-3.0-only
//! A cdylib whose `cerulion_node_info()` returns corrupted
//! JSON must refuse to load at `GraphRuntime::build` /
//! `build_in_process` time with a clear diagnostic — NOT silently
//! degrade to `NodeInfo::default()` (empty wiring → node never fires).
//!
//! Fixture: `test_fixtures/test_node_corrupt_info_cdylib` — a raw-FFI
//! ABI v5 cdylib identical in shape to `test_node_cdylib` except its
//! `cerulion_node_info()` returns deliberately malformed JSON
//! (truncated arrays, a bare `CORRUPTED!!` token). A dedicated fixture
//! is used (rather than a `CER_FAIL_MODE` arm on
//! `test_node_failing_cdylib`) because that fixture's info JSON is
//! macro-generated and static — the env var is not consulted at
//! info-call time.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_corrupt_info_cdylib
//! cargo test -p cerulion_core --test dylib_corrupt_info_test -- --test-threads=1
//! ```
//!
//! `--test-threads=1` + `#[serial]`: the IPC-build test touches the
//! iceoryx2 singleton (`TransportManager::get_or_init`), and all tests
//! load the cdylib whose `NODES` map is process-global (see
//! `chunk_c_ffi_error_test.rs` for the canonical `#[serial]` pattern).

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, NodeEntry};
use cerulion_core::graph::{parse_graph, GraphRuntime};
use cerulion_core::transport::TransportManager;
use cerulion_core::TransportError;
use indexmap::IndexMap;
use serial_test::serial;
use std::sync::Arc;

/// The fixture's corrupt JSON payload (sans NUL):
/// `{"inputs":["velocity_in",CORRUPTED!!,"outputs":[`.
///
/// This is a local copy for readability of the assertions below, but it
/// is NOT the source of truth — `corrupt_json_const_matches_fixture()`
/// derives the same bytes directly from the fixture's `lib.rs` at test
/// time (via `include_str!`) and asserts they match. So if the fixture
/// payload is ever edited, that test fails loudly instead of the
/// `json_len=<N>` Display assertions silently using a stale length.
const CORRUPT_JSON: &str = r#"{"inputs":["velocity_in",CORRUPTED!!,"outputs":["#;

/// Fixture source, pulled in at compile time so the test crate has a
/// hard link to the bytes the fixture actually ships. Used by
/// `corrupt_json_const_matches_fixture` to derive the payload rather
/// than trusting the hand-copied `CORRUPT_JSON` constant above.
const FIXTURE_SRC: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../test_fixtures/test_node_corrupt_info_cdylib/src/lib.rs"
));

/// Extract the fixture's `CORRUPT_INFO_BYTES` byte-string literal from
/// its source and decode it to the JSON payload string (sans the
/// trailing NUL terminator). Returns `None` if the marker can't be
/// found (which itself signals the fixture drifted in a way the test
/// must notice).
///
/// The fixture declares:
/// ```ignore
/// static CORRUPT_INFO_BYTES: &[u8] = b"{\"inputs\":[\"velocity_in\",CORRUPTED!!,\"outputs\":[\0";
/// ```
/// We locate the `b"..."` literal, take its contents, and decode the
/// two escapes the fixture actually uses (`\"` and `\0`).
fn fixture_corrupt_json() -> Option<String> {
    let marker = "CORRUPT_INFO_BYTES";
    let after = FIXTURE_SRC.split_once(marker)?.1;
    // The byte-string literal opens at `b"` and closes at the next
    // unescaped `"`. The fixture uses no escaped backslashes, so a plain
    // scan for `\"` (escaped quote) vs `"` (terminator) suffices.
    let open = after.find("b\"")? + 2;
    let body = &after[open..];
    let mut decoded = String::new();
    let mut chars = body.chars();
    while let Some(c) = chars.next() {
        match c {
            '"' => return Some(decoded), // closing quote of the literal
            '\\' => match chars.next()? {
                '"' => decoded.push('"'),
                '0' => {} // NUL terminator — drop it (the JSON is sans-NUL)
                'n' => decoded.push('\n'),
                't' => decoded.push('\t'),
                '\\' => decoded.push('\\'),
                other => decoded.push(other),
            },
            other => decoded.push(other),
        }
    }
    None // ran off the end without a closing quote → malformed
}

/// Find the corrupt-info test cdylib in the target directory.
///
/// Built by `cargo build -p test_node_corrupt_info_cdylib` (CI does
/// this explicitly in the "Build test cdylibs" step; locally `cargo
/// test --workspace` builds it transitively as a workspace member).
fn find_corrupt_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_corrupt_info_cdylib")
}

/// Assert the error Display satisfies the acceptance criteria:
/// json_len, the offending prefix, and the diag_label must ALL appear.
fn assert_diagnostics(msg: &str) {
    assert!(
        msg.contains(&format!("json_len={}", CORRUPT_JSON.len())),
        "error Display must contain the raw JSON length; got: {msg}"
    );
    assert!(
        msg.contains("CORRUPTED!!"),
        "error Display must contain the offending JSON prefix; got: {msg}"
    );
    assert!(
        msg.contains("test_node_corrupt_info_cdylib"),
        "error Display must contain the diag_label (cdylib path); got: {msg}"
    );
}

/// Minimal one-node graph wired to the corrupt fixture. No ports — the
/// info parse must fail before any transport resource is created.
const GRAPH_YAML: &str = r#"
name: corrupt_info_graph
prefix: test_corrupt_info
nodes:
  - id: corrupt
    type: test_node_corrupt_info
"#;

fn load_fixture() -> DylibNodeEntry {
    DylibNodeEntry::load(&find_corrupt_cdylib())
        .expect("load must succeed — corruption surfaces at info-parse time, not symbol-load time")
}

// ============================================================
// Fixture-desync guard
// ============================================================

/// The `CORRUPT_JSON` constant (and therefore every `json_len=<N>`
/// Display assertion) must stay byte-for-byte identical to the payload
/// the fixture actually returns over FFI. A
/// hand-copied 48-byte replica with no link to the fixture would let an edit to the
/// fixture's `CORRUPT_INFO_BYTES` silently desync the length while
/// `CORRUPT_JSON.len()` stayed 48, producing a misleading failure. This
/// test derives the payload from the fixture source at compile time
/// (`include_str!`) so a fixture edit fails HERE with a clear message
/// instead of masquerading as a Display regression.
#[test]
fn corrupt_json_const_matches_fixture() {
    let from_fixture =
        fixture_corrupt_json().expect("could not locate CORRUPT_INFO_BYTES literal in fixture src");
    assert_eq!(
        from_fixture, CORRUPT_JSON,
        "CORRUPT_JSON drifted from the fixture's CORRUPT_INFO_BYTES — \
         update the constant (and re-check the json_len assertions) to match"
    );
}

// ============================================================
// Direct `info()` surface
// ============================================================

/// `DylibNodeEntry::load` succeeds (all ABI v5 symbols present), then
/// `info()` returns `Err(NodeInfoParse)` whose fields and Display carry
/// json_len + prefix + diag_label.
#[test]
#[serial]
fn info_returns_node_info_parse_with_full_diagnostics() {
    let entry = load_fixture();

    let err = entry
        .info()
        .expect_err("corrupted info JSON must be an Err, not a defaulted NodeInfo");

    match &err {
        TransportError::NodeInfoParse {
            diag_label,
            json_len,
            prefix,
            reason,
        } => {
            assert_eq!(
                *json_len,
                CORRUPT_JSON.len(),
                "json_len must be the raw payload length"
            );
            assert!(
                prefix.len() <= 80,
                "prefix is capped at 80 bytes; got {} bytes",
                prefix.len()
            );
            // Payload is < 80 bytes so the prefix is the whole string.
            assert_eq!(prefix, CORRUPT_JSON, "prefix must be the offending bytes");
            assert!(
                diag_label.contains("test_node_corrupt_info_cdylib"),
                "diag_label must identify the cdylib; got: {diag_label}"
            );
            assert!(
                !reason.is_empty(),
                "reason must carry the underlying parse failure"
            );
        }
        other => panic!("expected TransportError::NodeInfoParse, got: {other:?}"),
    }

    assert_diagnostics(&err.to_string());
}

/// `init()` seeds the info cache via `info()?` — the corruption must
/// fail init too (a hand-rolled host driving DylibNodeEntry directly,
/// bypassing GraphRuntime, still cannot reach a ticking node).
#[test]
#[serial]
fn init_propagates_corrupt_info() {
    let mut entry = load_fixture();
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let err = entry
        .init(ctx)
        .expect_err("init must propagate the info-parse failure");
    assert!(
        matches!(err, TransportError::NodeInfoParse { .. }),
        "expected NodeInfoParse from init; got: {err:?}"
    );
    assert_diagnostics(&err.to_string());
}

// ============================================================
// Graph-build boundary (the acceptance criterion)
// ============================================================

// (The in-process `build_in_process` refusal test was removed:
// the in-process backend was deleted, so iceoryx2 is the
// only transport path. The `GraphRuntime::build` test below covers the
// same graph-build refusal contract.)

/// `GraphRuntime::build` refuses to construct a runtime containing the
/// corrupt node on the iceoryx2 transport path.
/// Requires the singleton `TransportManager` → `#[serial]` +
/// `--test-threads=1` per the iceoryx2 testing convention.
#[test]
#[serial]
fn build_ipc_refuses_corrupt_info_node() {
    let config = parse_graph(GRAPH_YAML).expect("graph YAML parses");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("corrupt".to_string(), Box::new(load_fixture()));

    let transport = TransportManager::get_or_init().expect("transport init");
    let clock = Arc::new(VirtualClock::new());
    let err = GraphRuntime::build(config, factories, &transport, clock)
        .err()
        .expect("build must refuse a node with corrupted info JSON");

    assert!(
        matches!(err, TransportError::NodeInfoParse { .. }),
        "expected NodeInfoParse from build; got: {err:?}"
    );
    assert_diagnostics(&err.to_string());
}
