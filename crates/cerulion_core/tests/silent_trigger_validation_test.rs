// SPDX-License-Identifier: AGPL-3.0-only
//! Loud-fail validation for the
//! silent-never-fire combo `(macro NodeInfo.policy: None, macro
//! `#[input(trigger)]` field declared)`.
//!
//! Without this validation, a node defined as
//!
//! ```ignore
//! #[cerulion_node]
//! struct Printer { #[input(trigger)] count: Int32 }
//! ```
//!
//! whose `NodeInfo` arrives with no policy (the macro accepts the
//! configuration as canonical, `validate.rs:137-140`, but a cdylib whose
//! info JSON carries no `data_trigger` arm hands the host `policy: None`)
//! would build cleanly, init cleanly, never fire, and print only one
//! `tracing::debug!` line. `runtime.rs::build()` and `build_for_test()`
//! reject the combo at graph-build time so the silent failure is
//! impossible. A node whose macro policy is plumbed end-to-end
//! (`MacroPolicy::DataTrigger`) never reaches the validator.
//!
//! Graph YAML carries no `policy:` block, so there is no
//! "explicit YAML policy satisfies the validator" path — the
//! macro side is the only signal feeding the validator.
//!
//! These tests exercise the rejection path AND the regression-positive
//! cases (macro period policy, no trigger inputs) to make sure the
//! check doesn't break existing flows.
//!
//! All tests use `build_for_test` to keep the file parallel-safe and
//! free of any iceoryx2 SHM setup.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::{BackpressurePolicy, InputMeta};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo, TransportError, TransportResult};
use indexmap::IndexMap;

const TEST_PREFIX: &str = "b";

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xDEAD_BEEF,
        trigger,
        depth: 1,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn make_entry(info: NodeInfo) -> Box<dyn NodeEntry> {
    let entry = ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_label("b_test");
    Box::new(entry)
}

fn graph_config(node_id: &str) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("b_{node_id}"),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: node_id.to_string(),
            node_type: node_id.to_string(),
            inputs: vec![],
            outputs: vec![],
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
// 1. Rejection — single trigger input, no macro policy.
// ===========================================================================

#[test]
fn build_for_test_rejects_macro_trigger_input_without_policy() {
    let info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]);
    let config = graph_config("printer");

    let result = try_build(config, make_entry(info), "printer");

    let err = match result {
        Ok(_) => panic!("build_for_test must reject the silent-trigger combo"),
        Err(e) => e,
    };
    match err {
        TransportError::GraphError { reason } => {
            assert!(
                reason.contains("printer"),
                "error must name the offending node id; got: {reason}"
            );
            assert!(
                reason.contains("count"),
                "error must list the trigger field name; got: {reason}"
            );
        }
        other => panic!("expected GraphError, got: {other:?}"),
    }
}

// ===========================================================================
// 2. Rejection — multiple inputs, only one is a trigger.
// ===========================================================================

#[test]
fn build_for_test_rejects_when_only_one_input_is_a_trigger() {
    let info = NodeInfo::with_meta(
        vec![
            meta_input("non_trigger", false),
            meta_input("the_trigger", true),
        ],
        vec![],
    );
    let config = graph_config("printer");

    let result = try_build(config, make_entry(info), "printer");

    let err = match result {
        Ok(_) => panic!("a single trigger input is enough to fail the check"),
        Err(e) => e,
    };
    match err {
        TransportError::GraphError { reason } => {
            assert!(
                reason.contains("the_trigger"),
                "error must name the trigger-flagged input; got: {reason}"
            );
            assert!(
                !reason.contains("non_trigger"),
                "error must NOT list non-trigger inputs; got: {reason}"
            );
        }
        other => panic!("expected GraphError, got: {other:?}"),
    }
}

// ===========================================================================
// 3. Rejection error message — node id, field name, actionable remediation,
//    and NO internal issue-tracker link (a user cannot open one).
// ===========================================================================

#[test]
fn rejection_error_message_carries_remediation_and_no_internal_link() {
    let info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]);
    let config = graph_config("printer");

    let err = match try_build(config, make_entry(info), "printer") {
        Ok(_) => panic!("must reject"),
        Err(e) => e,
    };
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };

    assert!(reason.contains("printer"), "node id missing: {reason}");
    assert!(
        reason.contains("count"),
        "trigger field name missing: {reason}"
    );
    assert!(
        reason.contains("Rebuild the node"),
        "concrete remediation missing: {reason}"
    );
    assert!(
        reason.contains("cerulion node build"),
        "runnable remediation command missing: {reason}"
    );
    assert!(
        reason.contains("USER_API.md"),
        "public-doc pointer missing: {reason}"
    );
    // The regression guard this test exists for: a user-facing error must
    // never send a user to an issue tracker they cannot open. The remedy
    // above is a command and a shipped file name, so the error carries no
    // link of any kind.
    assert!(
        !reason.contains("://"),
        "user-facing error must not carry a link: {reason}"
    );
    // A bare tracker id is as useless to a user as a link, and a remedy that
    // satisfies every positive assertion above could still carry one, so the
    // guard also rejects the SHAPE of such an id without naming any tracker.
    assert!(
        !carries_ticket_shaped_token(&reason),
        "user-facing error must not carry a ticket-shaped token: {reason}"
    );
}

/// True when `s` carries a token shaped like an issue-tracker id: two to six
/// upper-case ASCII letters, a hyphen, then one or more digits, bounded by
/// non-alphanumerics or the string ends. Names no tracker, so the fixture
/// spells no real identifier.
fn carries_ticket_shaped_token(s: &str) -> bool {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() {
        let start = i;
        while i < b.len() && b[i].is_ascii_uppercase() {
            i += 1;
        }
        let letters = i - start;
        if (2..=6).contains(&letters) && i < b.len() && b[i] == b'-' {
            let mut j = i + 1;
            while j < b.len() && b[j].is_ascii_digit() {
                j += 1;
            }
            let digits = j - (i + 1);
            let left_ok = start == 0 || !b[start - 1].is_ascii_alphanumeric();
            let right_ok = j == b.len() || !b[j].is_ascii_alphanumeric();
            if digits >= 1 && left_ok && right_ok {
                return true;
            }
        }
        if i == start {
            i += 1;
        }
    }
    false
}

#[test]
fn ticket_shape_guard_matches_only_tracker_shaped_tokens() {
    // Hand oracles on both sides of every boundary the helper draws.
    for hit in ["see ABC-12 for details", "(XYZ-1)", "ISSUE-123456", "AB-7"] {
        assert!(carries_ticket_shaped_token(hit), "expected a hit: {hit}");
    }
    for miss in [
        "run cerulion node build and read USER_API.md",
        "ABC-",
        "abc-12",
        "X-1",
        "ABCDEFG-1",
        "xABC-12",
        "ABC-12x",
        "ABC 12",
    ] {
        assert!(
            !carries_ticket_shaped_token(miss),
            "expected a miss: {miss}"
        );
    }
}

// ===========================================================================
// 4. Regression-positive — macro-declared Period policy + a trigger
// input is unusual but legal: the period schedule wins, no silent fail.
// ===========================================================================

#[test]
fn build_for_test_accepts_macro_period_policy_with_trigger_input() {
    let info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let config = graph_config("printer");

    if try_build(config, make_entry(info), "printer").is_err() {
        panic!("macro policy should silence the validator");
    }
}

// ===========================================================================
// 5. Regression-positive — no trigger inputs, no policy anywhere: falls
// through to the existing default-Data-trigger arm without error.
// ===========================================================================

#[test]
fn build_for_test_accepts_node_with_no_trigger_inputs_and_no_policy() {
    let info = NodeInfo::from_names(vec![], vec![]);
    let config = graph_config("printer");

    if try_build(config, make_entry(info), "printer").is_err() {
        panic!("no triggers + no policy should pass (default-Data arm)");
    }

    let info2 = NodeInfo::with_meta(vec![meta_input("a", false), meta_input("b", false)], vec![]);
    let config2 = graph_config("printer");

    if try_build(config2, make_entry(info2), "printer").is_err() {
        panic!("no trigger=true entries should pass even with non-empty input_meta");
    }
}

// ===========================================================================
// 5a. The validator must go INERT for nodes the macro plumbs
// end-to-end (`MacroPolicy::DataTrigger`).
// ===========================================================================

#[test]
fn validator_inert_when_macro_emits_data_trigger() {
    let consumer_info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        },
    );

    let src_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a_validator_inert".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "src".to_string(),
                node_type: "src".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "u8".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "consumer".to_string(),
                // With no macro policy this would be rejected by the
                // silent-trigger validator. Here the macro provides
                // both the policy AND the input-name; the binding gets
                // synthesized; the validator's `macro_policy.is_some()`
                // early-return makes this build succeed.
                inputs: vec![InputDef {
                    name: "count".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), make_entry(src_info));
    factories.insert("consumer".to_string(), make_entry(consumer_info));

    let clock = Arc::new(VirtualClock::new());
    if GraphRuntime::build_for_test(config, factories, clock, 4).is_err() {
        panic!(
            "the macro DataTrigger path must build cleanly — \
             the silent-trigger validator must be inert for nodes the macro \
             plumbs end-to-end"
        );
    }
}

// ===========================================================================
// 6. Adversarial — 2+ trigger inputs (Sync-style macro shape) without
// any policy: must STILL reject (each trigger input is part of the
// silent-fail risk surface).
// ===========================================================================

#[test]
fn build_for_test_rejects_when_two_inputs_are_triggers_without_policy() {
    let info = NodeInfo::with_meta(
        vec![meta_input("left", true), meta_input("right", true)],
        vec![],
    );
    let config = graph_config("syncer");

    let err = match try_build(config, make_entry(info), "syncer") {
        Ok(_) => panic!("must reject"),
        Err(e) => e,
    };
    match err {
        TransportError::GraphError { reason } => {
            assert!(reason.contains("syncer"), "node id missing: {reason}");
            assert!(
                reason.contains("[left, right]"),
                "exact format `[left, right]` missing: {reason}"
            );
        }
        other => panic!("expected GraphError, got: {other:?}"),
    }
}
