// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end cdylib QoS round-trip over the v6
//! info-JSON FFI.
//!
//! Loads `test_node_macro_qos_cdylib` (a `#[cerulion_node]` fixture
//! declaring ALL FOUR QoS knobs) through `DylibNodeEntry`, calls
//! `info()`, and asserts every knob round-trips across the FFI:
//!
//!   - node-level `tick_within_ms = 500` → `NodeInfo::tick_within_ms()`
//!   - node-level `throttle_ms   = 5`    → `NodeInfo::throttle_ms()`
//!   - input  `expect_within_ms  = 20`   → `InputMeta::expect_within_ms`
//!   - output `promise_within_ms = 30`   → `OutputMeta::promise_within_ms`
//!
//! The fixture's `#[input(trigger)]` field also exercises that the v6
//! INPUT-OBJECT shape still parses the `MacroPolicy::DataTrigger`
//! correctly (the policy lives top-level, but the input objects sit
//! beside it). A regression on either side of the FFI — macro emission
//! in `gen_cdylib` OR host parsing in `parse_info_json` — fires here.
//!
//! No `#[serial]`: `info()` reads the cdylib's `cerulion_node_info()`
//! export and parses JSON; it does NOT touch iceoryx2 shared memory, so
//! it is parallel-safe (mirrors `macro_cdylib_policy_round_trip_test`).

use cerulion_core::graph::node::{DylibNodeEntry, MacroPolicy, NodeEntry};

/// Locate a cdylib produced by the workspace build (fixture crates are
/// workspace members, so `cargo test` builds them transitively).
fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

#[test]
fn cdylib_info_carries_all_four_qos_knobs() {
    let path = find_cdylib("test_node_macro_qos_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load QoS cdylib");
    let info = node.info().expect("QoS cdylib info should parse");

    // --- node-level QoS (top-level JSON, conditional presence) ---
    assert_eq!(
        info.tick_within_ms(),
        Some(500),
        "node-level `tick_within_ms = 500` must round-trip across the v6 \
         info-JSON FFI; got {:?}",
        info.tick_within_ms()
    );
    assert_eq!(
        info.throttle_ms(),
        Some(5),
        "node-level `throttle_ms = 5` must round-trip across the v6 \
         info-JSON FFI; got {:?}",
        info.throttle_ms()
    );

    // --- per-input QoS (input objects carry `expect_within_ms`) ---
    let input_meta = info.input_meta();
    assert_eq!(
        input_meta.len(),
        1,
        "fixture declares exactly one input; got {} meta entries",
        input_meta.len()
    );
    assert_eq!(input_meta[0].name, "velocity_in");
    assert_eq!(
        input_meta[0].expect_within_ms,
        Some(20),
        "input `expect_within_ms = 20` must round-trip into InputMeta; got {:?}",
        input_meta[0].expect_within_ms
    );

    // --- per-output QoS (output objects carry `promise_within_ms`) ---
    let output_meta = info.output_meta();
    assert_eq!(
        output_meta.len(),
        1,
        "fixture declares exactly one output; got {} meta entries",
        output_meta.len()
    );
    assert_eq!(output_meta[0].name, "cmd_out");
    assert_eq!(
        output_meta[0].promise_within_ms,
        Some(30),
        "output `promise_within_ms = 30` must round-trip into OutputMeta; got {:?}",
        output_meta[0].promise_within_ms
    );

    // The trigger input still yields the DataTrigger policy — the v6
    // input-object shape sits beside the top-level policy without
    // disturbing it.
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::DataTrigger {
            input_name: "velocity_in".to_string(),
        }),
        "trigger input must still parse as DataTrigger under the v6 \
         input-object shape; got {:?}",
        info.policy()
    );
}

/// SAFETY pin: for a cdylib input with NO explicit `depth` or
/// `backpressure` declaration (this fixture declares only `trigger` +
/// `expect_within_ms`), the synthesized `InputMeta` must carry the SAME
/// depth/backpressure defaults the empty-meta fallback implied, so
/// topology wiring is byte-identical (a critical invariant).
/// Since ABI v8 BOTH are behaviour-changing when DECLARED —
/// the declared paths are pinned by `cdylib_depth_ffi_test` — but
/// UNDECLARED values must keep resolving the host fallbacks, which is
/// what this pins.
#[test]
fn cdylib_input_meta_carries_topology_safe_defaults() {
    use cerulion_core::graph::node::BackpressurePolicy;

    let path = find_cdylib("test_node_macro_qos_cdylib");
    let node = DylibNodeEntry::load(&path).expect("load QoS cdylib");
    let info = node.info().expect("QoS cdylib info should parse");
    let m = &info.input_meta()[0];

    // These two fields are the ONLY ones `GraphTopology::build` reads off
    // a cdylib node's input_meta. They MUST equal the old None→fallback
    // `(BackpressurePolicy::default(), DEFAULT_CONSUMER_DEPTH)`.
    assert_eq!(
        m.backpressure,
        BackpressurePolicy::default(),
        "cdylib InputMeta backpressure must match the empty-meta topology fallback"
    );
    assert_eq!(
        m.depth,
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        "cdylib InputMeta depth must match the empty-meta topology fallback (DEFAULT_CONSUMER_DEPTH)"
    );
    // ABI v9: the `#[input(trigger)]` mark now rides the cdylib FFI
    // (pre-v9 the host HARDCODED `trigger: false` here and derived trigger
    // truth from `MacroPolicy` alone). `velocity_in` is declared
    // `#[input(trigger, ...)]`, so its parsed `InputMeta.trigger` is now
    // `true` — the real per-input value `validate_no_silent_data_trigger`
    // sees, matching the in-process macro path. (Trigger-EDGE classification
    // still flows from `MacroPolicy` on this path; carrying the flag is the
    // plumbing that trigger-scoped Sync consumes.)
    assert!(
        m.trigger,
        "cdylib InputMeta.trigger must round-trip `true` for a `#[input(trigger)]` \
         field via ABI v9 (was hardcoded false before trigger-scoped Sync)"
    );
}
