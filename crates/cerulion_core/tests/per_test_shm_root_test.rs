// SPDX-License-Identifier: AGPL-3.0-only
//! Per-test SHM root smoke test.
//!
//! Proves that `cerulion_core::testing::iceoryx_test_config` +
//! `TransportManager::init_for_test` lets multiple iceoryx2
//! Nodes coexist in a single process WITHOUT colliding on the
//! production `/tmp/iceoryx2/` singleton SHM region.
//!
//! Without an isolated root every iceoryx2 test has to run with `--test-threads=1`
//! and is at risk of clashing with sibling tests. With one, two
//! managers with different isolated configs can run side-by-side
//! in the same process — the test creates two of them and
//! exercises both.
//!
//! This test must be runnable in parallel with other iceoryx
//! tests in the workspace. If it fails when `cargo test
//! --workspace` runs without `--test-threads=1`, the per-test
//! isolation regressed.

use std::sync::Arc;

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::{TransportConfig, TransportManager, VirtualClock};

#[test]
fn two_managers_can_coexist_with_isolated_configs() {
    let mgr_a = TransportManager::init_for_test(
        TransportConfig {
            node_name: "mgr_a".to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 4,
            network: None,
        },
        iceoryx_test_config(),
    )
    .expect("manager A should initialize with isolated config");
    let mgr_b = TransportManager::init_for_test(
        TransportConfig {
            node_name: "mgr_b".to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 4,
            network: None,
        },
        iceoryx_test_config(),
    )
    .expect("manager B should initialize with a separate isolated config");
    // Both managers exist as Arc<Self> simultaneously — the API
    // didn't go through the OnceLock singleton, so neither
    // shadowed the other. Drop both at end of scope; each iceoryx2
    // node's own graceful `Drop` removes its resources from the temp
    // roots (the auto dead-node cleanup flags are disabled, and those
    // only govern reaping OTHER dead nodes — never a node's own
    // teardown).
    let _ = (mgr_a, mgr_b);
}

#[test]
fn each_test_gets_a_fresh_isolated_config() {
    // The configs returned by `iceoryx_test_config()` must be
    // distinct (different prefixes) so two parallel tests don't
    // accidentally share state.
    let a = iceoryx_test_config();
    let b = iceoryx_test_config();
    // The `prefix` is the per-config uniqueness handle; if they
    // collide, tests share resources. Compare the byte
    // representations (FileName has Eq).
    assert_ne!(
        a.global.prefix, b.global.prefix,
        "consecutive iceoryx_test_config() calls must produce distinct prefixes"
    );
}
