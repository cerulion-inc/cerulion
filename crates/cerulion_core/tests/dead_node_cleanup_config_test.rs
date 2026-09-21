// SPDX-License-Identifier: AGPL-3.0-only
//! A `TransportManager` node is ALWAYS built with iceoryx2's automatic
//! dead-node cleanup disabled.
//!
//! iceoryx2 0.9.1 defaults all three auto-cleanup flags ON. On a service
//! open/create/destroy, auto-cleanup probes each registered node's liveness and
//! reaps the services of any it judges dead. A dlopen'd cdylib links its OWN
//! iceoryx2 copy whose cached `UniqueProcessId` never matches the host node's, so
//! its probe falls through to a same-PID fcntl `F_GETLK` check that (POSIX record
//! locks never conflict within one PID) reads the LIVE host node's own `F_WRLCK`
//! as unlocked — judging the host node DEAD and reaping every service it owns.
//! The fix disables the flags on the config every constructor builds its node
//! from; this pins the BUILT manager's node config, end-to-end, over a real
//! iceoryx2 node. The pure transform + the singleton arm-selection (including the
//! `Config::default()`-is-ON oracle proving the flip is real) are pinned
//! hermetically in `transport/mod.rs`'s inline tests.
//!
//! Per-test SHM root via `iceoryx_test_config()` (a fresh isolated prefix per
//! call) + `init_for_test` (a non-singleton manager) — parallel-safe, no
//! `#[serial]`.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::{TransportConfig, TransportManager};

fn transport_config(node_name: &str) -> TransportConfig {
    TransportConfig {
        node_name: node_name.to_string(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    }
}

fn assert_all_cleanup_flags_off(config: &iceoryx2::config::Config) {
    assert!(
        !config.global.service.cleanup_dead_nodes_on_open,
        "service.cleanup_dead_nodes_on_open must be off (cdylib same-PID reap defeat)"
    );
    assert!(
        !config.global.node.cleanup_dead_nodes_on_creation,
        "node.cleanup_dead_nodes_on_creation must be off"
    );
    assert!(
        !config.global.node.cleanup_dead_nodes_on_destruction,
        "node.cleanup_dead_nodes_on_destruction must be off"
    );
}

/// A config handed to a constructor WITH all three flags forced ON must come out
/// of the built node with them OFF — the never-trust-callers contract, proven on
/// the real `init_for_test` path (not just the pure transform). The `_for_test`
/// accessor reads `node.config()` back, so this also proves iceoryx2 STORED the
/// disabled flags on the node rather than resetting them.
#[test]
fn built_manager_disables_cleanup_even_when_caller_sets_flags_true() {
    let mut ix_config = iceoryx_test_config();
    // Self-controlled precondition: force all three ON so the flip below cannot
    // be a no-op regardless of any machine-level iceoryx2 config file.
    ix_config.global.service.cleanup_dead_nodes_on_open = true;
    ix_config.global.node.cleanup_dead_nodes_on_creation = true;
    ix_config.global.node.cleanup_dead_nodes_on_destruction = true;

    let mgr = TransportManager::init_for_test(transport_config("hostile"), ix_config)
        .expect("init_for_test with hostile config");

    assert_all_cleanup_flags_off(&mgr.iox_config());
}

/// The common path — a plain isolated test config, untouched by the caller —
/// also yields a node built with the flags OFF. Asserts the OUTPUT contract only
/// (the genuine true→false flip is oracle-pinned inline against
/// `Config::default()`, so this is not a self-compare).
#[test]
fn built_manager_disables_cleanup_on_untouched_config() {
    let mgr = TransportManager::init_for_test(transport_config("default"), iceoryx_test_config())
        .expect("init_for_test with default config");

    assert_all_cleanup_flags_off(&mgr.iox_config());
}
