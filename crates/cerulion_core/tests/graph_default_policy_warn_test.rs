// SPDX-License-Identifier: AGPL-3.0-only
//! Pins the
//! `macro_policy = None` default-Data-trigger tracing level + message.
//!
//! The "no declared policy" arm — `MacroPolicy = None` — used to log
//! at `debug!`, invisible at the default `RUST_LOG` filter (which
//! gates everything below `info`). After
//! `validate_no_silent_data_trigger` rejects the worst silent-fail
//! combo (a `#[input(trigger)]` field with no policy), the residual
//! case is "node has no trigger inputs and no declared policy" —
//! defensible default but worth surfacing. Both
//! `runtime.rs::build()` and `runtime.rs::build_for_test()` log it at
//! `warn!` with a refined message.
//!
//! Graph YAML now carries no `policy:` block, so the policy
//! match collapsed from a 4-arm `(yaml, macro)` Cartesian product to
//! a 2-arm match on `macro_policy` alone. The warn-phrase pinning is
//! still load-bearing — it loud-fails a future revert of the
//! debug→warn bump or any wording drift.
//!
//! These tests pin three contracts:
//!   1. The `None` arm fires `warn!` with the expected phrase
//!      and structured `node_id` field — exactly once per build.
//!   2. The `Some(macro_policy)` arm does NOT emit this warn.
//!   3. The silent-trigger reject path short-circuits BEFORE the policy match
//!      arm runs, so a silent-trigger combo never double-logs (no
//!      reject error AND default-Data warn for the same node).
//!
//! Captures `tracing::warn!` via the `tracing-test` crate's
//! `#[traced_test]` attribute, which scopes a subscriber to the test
//! thread. All test cases use `build_for_test` to keep the file
//! parallel-safe and SHM-free; the tracing call site in `build()` is
//! covered by symmetry — the two warn sites are byte-identical
//! modulo the surrounding policy-match code.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::{BackpressurePolicy, InputMeta};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo, TransportError, TransportResult};
use indexmap::IndexMap;
use tracing_test::traced_test;

const TEST_PREFIX: &str = "b_warn";

/// The exact phrase that must appear in the no-policy arm's warn!
/// log. Pinning a unique substring (rather than the whole message)
/// keeps the test resilient to incidental wording polish but loud-fails
/// the specific contract: the message references the macro side
/// (the YAML side no longer carries a policy block).
const WARN_PHRASE: &str = "no macro-declared policy";

/// The old `debug!` phrase. If this ever appears in captured logs, the
/// bump regressed.
const OLD_DEBUG_PHRASE: &str = "no policy declared, defaulting to Data trigger";

/// The phrase from when graph YAML still carried a policy block. If this
/// ever appears, someone reverted the YAML-policy removal.
const OLD_WARN_PHRASE: &str = "no policy in YAML and no macro-declared policy";

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xC0FFEE,
        trigger,
        depth: 1,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn make_entry(info: NodeInfo) -> Box<dyn NodeEntry> {
    Box::new(
        ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_label("warn_test_entry"),
    )
}

/// Single-node config; output is required so the runtime has something
/// to publish — but the validator + warn fire before the publisher is
/// actually used.
fn graph_config(node_id: &str) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("warn_{node_id}"),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: node_id.to_string(),
            node_type: node_id.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    }
}

fn try_build(
    config: GraphConfig,
    entry: Box<dyn NodeEntry>,
    node_id: &str,
) -> TransportResult<GraphRuntime> {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(node_id.to_string(), entry);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4)
}

// ===========================================================================
// 1. Happy path — the `None` arm fires `warn!` exactly once.
// ===========================================================================

#[test]
#[traced_test]
fn warn_fires_when_no_macro_policy_and_no_trigger_inputs() {
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()]);
    let config = graph_config("ticker");
    if try_build(config, make_entry(info), "ticker").is_err() {
        panic!("build_for_test should succeed for the no-policy-no-trigger case");
    }

    assert!(
        logs_contain(WARN_PHRASE),
        "the macro-policy=None arm must emit the warn phrase"
    );
    assert!(
        logs_contain("node_id=ticker"),
        "the warn must carry the structured `node_id=ticker` field"
    );
    assert!(
        !logs_contain(OLD_DEBUG_PHRASE),
        "the old debug phrase must not appear in the warn message"
    );
    assert!(
        !logs_contain(OLD_WARN_PHRASE),
        "the old 'YAML policy' phrasing must not appear (graph YAML no longer carries policy)"
    );
    // Pin "exactly once": if a future refactor double-logs the warn
    // (e.g. accidentally fires inside an inner loop), this loud-fails.
    logs_assert(|lines: &[&str]| {
        let count = lines.iter().filter(|l| l.contains(WARN_PHRASE)).count();
        if count == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 warn line containing the warn phrase, got {count}; \
                 captured lines: {lines:#?}"
            ))
        }
    });
}

// ===========================================================================
// 2. Negative coverage — macro-declared policy must not emit the warn.
// ===========================================================================

#[test]
#[traced_test]
fn warn_does_not_fire_when_macro_policy_present() {
    let info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let config = graph_config("ticker");
    if try_build(config, make_entry(info), "ticker").is_err() {
        panic!("macro Period policy build should succeed");
    }

    assert!(
        !logs_contain(WARN_PHRASE),
        "the macro-policy=Some arm must NOT emit the no-policy warn"
    );
}

// ===========================================================================
// 3. Validator interaction — the silent-trigger reject path must NOT
// reach the `None` arm. Verifies the early-return ordering: the silent-trigger
// validator rejects BEFORE the policy match arm runs, so an
// incorrectly-configured node fails loudly with ONE error and zero
// warns from this arm (no double-log).
// ===========================================================================

#[test]
#[traced_test]
fn chunk1_reject_path_does_not_emit_chunk2_warn() {
    let info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]);
    let config = graph_config("printer");
    let err = match try_build(config, make_entry(info), "printer") {
        Ok(_) => panic!("validator must reject the silent-trigger combo"),
        Err(e) => e,
    };

    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError from the silent-trigger validator, got: {other:?}"),
    };
    // Attribute the error to the silent-trigger validator by the CONDITION
    // it reports: a test must never pin wording beyond that condition in a
    // string users read.
    assert!(
        reason.contains("carries no trigger policy"),
        "rejection must come from the silent-trigger validator (the only one that \
         reports a missing trigger policy); got: {reason}"
    );

    assert!(
        !logs_contain(WARN_PHRASE),
        "the reject path must short-circuit before the macro-policy=None arm"
    );
}
