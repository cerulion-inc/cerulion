// SPDX-License-Identifier: AGPL-3.0-only
//! Build test_node_macro_cdylib first. Real init and shutdown panics preserve
//! their native phase classes and diagnostics in two independent runs.
//! Parallel-safe: contexts and environment snapshots are independently owned.

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, NodeEntry};
use cerulion_core::{ShutdownSignal, TransportError};
use indexmap::IndexMap;
use std::collections::HashMap;
use std::sync::Arc;

#[test]
fn stable_lifecycle_panics_keep_native_phase_classes_in_two_runs() {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");
    let mut runs = Vec::new();
    for _ in 0..2 {
        let mut reasons = Vec::new();
        for (phase, expected) in [
            (
                "init",
                "init failed: cerulion_node_init: panic caught by catch_unwind",
            ),
            (
                "shutdown",
                "cerulion_node_shutdown: panic caught by catch_unwind",
            ),
        ] {
            let mut entry = DylibNodeEntry::load(&path).expect("rebuilt macro fixture");
            let context = NodeContext::with_runtime_env(
                IndexMap::new(),
                IndexMap::new(),
                Arc::new(VirtualClock::new()),
                ShutdownSignal::new(),
                Arc::new(HashMap::from([(
                    "CER_TEST_STABLE_PANIC_PHASE".into(),
                    phase.into(),
                )])),
            );
            let error = if phase == "init" {
                entry
                    .init(context)
                    .expect_err("the actual user init panics")
            } else {
                entry.init(context).expect("the actual user init succeeds");
                entry
                    .shutdown()
                    .expect_err("the actual user shutdown panics")
            };
            let TransportError::NodeError { reason, .. } = error else {
                panic!("only a tick may become NodeTickPanicked: {error:?}");
            };
            assert_eq!(reason, expected);
            reasons.push(reason);
        }
        assert_eq!(
            reasons,
            [
                "init failed: cerulion_node_init: panic caught by catch_unwind",
                "cerulion_node_shutdown: panic caught by catch_unwind",
            ]
        );
        runs.push(reasons);
    }
    assert_eq!(runs[0], runs[1]);
}
