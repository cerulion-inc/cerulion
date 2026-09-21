// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-graph same-topic publish collisions over real iceoryx2.
//!
//! Two same-host graphs legitimately COEXIST on one prefix (the in-prefix
//! absolute-source rejection became a warn — prefix is a namespace, not a
//! graph identity), but two graphs PUBLISHING the same topic must ERROR:
//! single-writer is a data-integrity contract, not a namespace convention.
//!
//! Three arms, each on ONE shared SHM root ("graph processes" = separate
//! `TransportManager`s over the same isolated iceoryx2 config):
//!
//! (a) derived-name collision — same prefix + node-id + output-name in two
//!     graphs derive the SAME topic; the second graph's build must die at
//!     single-writer publisher creation.
//! (b) `topic:` override collision — two graphs (different prefixes)
//!     override outputs to the same absolute name; same death. Collision
//!     is a property of the published topic, not derived-name coincidence.
//! (c) the degraded arm — a default opener pre-created the service at
//!     iceoryx2's create-default 2 publisher slots, where the port cap can
//!     no longer enforce single-writer (opens are at-least). The
//!     active-publisher pre-check must still refuse the second graph.
//!     Mutation oracle: deleting the pre-check (or loosening its `> 0`)
//!     makes graph B BUILD SUCCESSFULLY here — the panic in (c) is the
//!     pre-check's unique behavioral pin. Arms (a)/(b) still reject B
//!     under the same mutation (the port cap fires on the 1-slot service
//!     they created), but with the port-cap message — their failures are
//!     assertion-text mismatches, not silent attaches.
//!
//! The pre-check is best-effort against a CONCURRENT attach race (two
//! creators can both read 0 and both attach on a degraded service); all
//! three arms are deliberately sequential — they pin the static startup
//! collision, which is the realistic case. The port cap remains the hard
//! backstop on services the graph itself created.
//!
//! Parallel-safe: each test generates its own isolated iceoryx2 config.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Period producer — the publishing side of every collision arm.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct CamNode {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl CamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = f64::from(self.n);
        Ok(())
    }
}

fn producer_graph(
    prefix: &str,
    node_id: &str,
    topic_override: Option<&str>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_{node_id}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: node_id.to_string(),
            node_type: "cam".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: topic_override.map(str::to_string),
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(node_id.to_string(), Box::new(CamNodeEntry::new()));
    (config, factories)
}

/// One "graph process": its own manager (iceoryx2 node) over the shared root.
fn manager(name: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 8,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// Every collision arm must name the live publisher, the single-writer
/// contract, the cross-graph cause, and the multi-publisher remedy.
fn assert_collision_error(msg: &str) {
    assert!(
        msg.contains("attached publisher") && msg.contains("single-writer"),
        "the rejection must name the live publisher + the single-writer contract: {msg}"
    );
    assert!(
        msg.contains("another graph"),
        "the rejection must name the cross-graph collision cause: {msg}"
    );
    assert!(
        msg.contains("multi-publisher opt-in"),
        "the rejection must point at the multi-publisher remedy: {msg}"
    );
}

#[test]
fn same_prefix_derived_name_collision_errors_on_second_graph() {
    // (a) Graph A owns /shared/camera/out (created single-writer); graph
    // B derives the identical topic and must fail at publisher creation.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("cross_graph_a", ix.clone());
    let mgr_b = manager("cross_graph_b", ix);

    let (cfg_a, fac_a) = producer_graph("shared", "camera", None);
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("graph A must own /shared/camera/out");

    let (cfg_b, fac_b) = producer_graph("shared", "camera", None);
    let msg = match GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new())) {
        Ok(_) => panic!("graph B publishing the same derived topic must NOT build"),
        Err(e) => e.to_string(),
    };
    assert_collision_error(&msg);
    assert!(
        msg.contains("/shared/camera/out"),
        "the rejection must carry the colliding topic: {msg}"
    );

    // The rejection must be non-destructive to the
    // incumbent — graph A's producer still publishes after B's failed
    // build (B's partial build must not have consumed A's slot, poisoned
    // the service, or unwired A's publisher). The observer deliberately
    // rides on mgr_b ("graph B's process"): the rejected process can
    // still CONSUME the topic, and the pin doesn't depend on A's own
    // manager state.
    let observer = mgr_b
        .create_subscriber("/shared/camera/out")
        .expect("an observer subscriber attaches after the rejected build");
    rt_a.step(std::time::Duration::from_millis(2));
    let mut seen = 0u32;
    observer
        .try_receive(|_msg| {
            seen += 1;
        })
        .expect("receive from the incumbent");
    assert!(
        seen >= 1,
        "graph A's producer must still publish after graph B's rejection (saw {seen})"
    );
}

#[test]
fn topic_override_collision_errors_on_second_graph() {
    // (b) Two graphs in DIFFERENT prefixes, different node ids, both
    // overriding to /shared/tf — the second build must fail at publisher
    // creation. Pins that the collision check rides on the RESOLVED
    // (override-aware) topic name, not the derived formula.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("override_a", ix.clone());
    let mgr_b = manager("override_b", ix);

    let (cfg_a, fac_a) = producer_graph("ga", "bc_a", Some("/shared/tf"));
    let _rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("graph A must own /shared/tf");

    let (cfg_b, fac_b) = producer_graph("gb", "bc_b", Some("/shared/tf"));
    let msg = match GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new())) {
        Ok(_) => panic!("graph B overriding to the same absolute topic must NOT build"),
        Err(e) => e.to_string(),
    };
    assert_collision_error(&msg);
    assert!(
        msg.contains("/shared/tf"),
        "the rejection must carry the colliding topic: {msg}"
    );
}

#[test]
fn degraded_service_still_rejects_second_graph_publisher() {
    // (c) THE GAP the pre-check closes: a default opener (a
    // topic-introspection tool) creates the service FIRST with no port
    // requirements — iceoryx2 create-defaults give it 2 publisher slots,
    // so the port cap can no longer enforce single-writer and graph opens
    // are degraded (at-least semantics; the degraded-provisioning warn
    // fires). Graph A attaches. WITHOUT the active-publisher pre-check,
    // graph B would attach into the spare slot silently.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_tool = manager("tool_opener", ix.clone());
    let mgr_a = manager("degraded_a", ix.clone());
    let mgr_b = manager("degraded_b", ix);

    // Default opener: creates the service, holds NO publisher.
    let _tool_sub = mgr_tool
        .create_subscriber("/shared/camera/out")
        .expect("default opener pre-creates the service at iceoryx2 defaults");

    let (cfg_a, fac_a) = producer_graph("shared", "camera", None);
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("graph A attaches to the pre-existing 2-slot service (degraded, warned)");

    let (cfg_b, fac_b) = producer_graph("shared", "camera", None);
    let msg = match GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new())) {
        Ok(_) => panic!(
            "graph B must NOT attach into the spare publisher slot of the \
             degraded service (the active-publisher pre-check is the only \
             guard on this path)"
        ),
        Err(e) => e.to_string(),
    };
    assert_collision_error(&msg);
    // Symmetry with arms (a)/(b): the right topic collided, not an
    // unrelated one.
    assert!(
        msg.contains("/shared/camera/out"),
        "the rejection must carry the colliding topic: {msg}"
    );

    // Mirror of arm (a)'s tail: the incumbent on the degraded
    // service also survives the rejection — A's publisher keeps
    // publishing through the pre-existing 2-slot service after B's
    // refused attach.
    let observer = mgr_b
        .create_subscriber("/shared/camera/out")
        .expect("an observer subscriber attaches after the rejected build");
    rt_a.step(std::time::Duration::from_millis(2));
    let mut seen = 0u32;
    observer
        .try_receive(|_msg| {
            seen += 1;
        })
        .expect("receive from the incumbent on the degraded service");
    assert!(
        seen >= 1,
        "graph A's producer must still publish on the degraded service \
         after graph B's rejection (saw {seen})"
    );
}
