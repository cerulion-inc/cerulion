// SPDX-License-Identifier: AGPL-3.0-only
//! P0: the CONTEXT-CARRIED transport contract — every runtime-built
//! `NodeContext` carries the HOST's `TransportManager` (`Arc::ptr_eq` with
//! the manager the graph was built on), injected before `init()` moves the
//! context (for a cdylib node: across the FFI, the same path the ports
//! take).
//!
//! Exists because a CDYLIB links its OWN copy of cerulion_core: its
//! `TransportManager::get()` consults an `INSTANCE` static the host never
//! initialized (the cross-linkage-unit global trap — the live
//! dry-run failure: 149 backoff retries), and a `get_or_init()` there would
//! mint a SECOND manager on the DEFAULT SHM root, silently splitting
//! namespaces. Context-carriage is the only correct channel; this file pins
//! it at the runtime seam (the ABI v10 bump).
//!
//! Isolated per-test SHM roots (`init_for_test`) — parallel-safe, no
//! `#[serial]`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;

fn test_mgr(tag: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("ctx726_{tag}"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated transport")
}

#[test]
fn test_runtime_built_context_carries_the_build_manager() {
    // A probe node captures its context's transport at tick; the capture
    // must be THE manager the graph was built on — pointer identity, not
    // just "some manager" (a different Arc means a second manager on a
    // different SHM namespace: the exact cdylib failure class).
    let mgr = test_mgr("carry");
    let captured: Arc<Mutex<Option<Arc<TransportManager>>>> = Arc::new(Mutex::new(None));
    let cap = Arc::clone(&captured);
    let entry = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 1 }),
        move |ctx: &mut NodeContext| {
            *cap.lock().unwrap() = ctx.transport().cloned();
            Ok(())
        },
    );

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ctx726_carry".to_string(),
        prefix: "ctx726".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "probe".to_string(),
            node_type: "probe".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("probe".to_string(), Box::new(entry));
    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build(config, factories, &mgr, clock).expect("graph builds");

    rt.step(Duration::from_millis(1)); // fire the 1ms Period probe once

    let got = captured
        .lock()
        .unwrap()
        .clone()
        .expect("the tick observed a context-carried transport (None = the injection is gone)");
    assert!(
        Arc::ptr_eq(&got, &mgr),
        "the context's transport must be THE build manager (Arc::ptr_eq)"
    );
}

#[test]
fn test_hand_rolled_context_defaults_to_none_and_set_transport_installs() {
    // The documented contract: `None` only for hand-rolled contexts that
    // never called set_transport; embedders install their manager the same
    // way the runtime does.
    let mgr = test_mgr("hand");
    let mut ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    assert!(
        ctx.transport().is_none(),
        "a hand-rolled context carries no transport by default"
    );
    ctx.set_transport(Arc::clone(&mgr));
    assert!(
        Arc::ptr_eq(ctx.transport().expect("installed"), &mgr),
        "set_transport installs the SAME manager (ptr_eq)"
    );
}

#[test]
fn test_arc_self_mints_the_same_arc_identity_from_a_plain_reference() {
    // The runtime seam: the build chain holds `&TransportManager`, and
    // `arc_self` must recover the owned Arc with the SAME identity (the
    // construction-time `Arc::new_cyclic` self-ref) — never a copy/second
    // manager.
    let mgr = test_mgr("arcself");
    let plain: &TransportManager = &mgr;
    assert!(
        Arc::ptr_eq(&plain.arc_self(), &mgr),
        "arc_self must mint the same Arc identity from a plain reference"
    );
}
