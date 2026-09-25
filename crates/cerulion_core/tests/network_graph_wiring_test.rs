// SPDX-License-Identifier: AGPL-3.0-only
//! The `network:` block → [`GatewayPlan`] derivation, and the pin that
//! a graph build starts NOTHING network (a graph process is network-free).
//!
//! Previously the graph build itself wired the network (installed the egress
//! allow-list, started the liveliness watch, announced, registered ingress).
//! Now that is a SEPARATE gateway process: the CLI computes its plan with the
//! PURE [`compute_gateway_plan`] and runs a `GatewayRuntime` beside the graph.
//!
//! What lives here vs elsewhere:
//! - the pure `NetworkBlock → NetworkConfig` mapping: inline oracle tests in
//!   `graph/config.rs`;
//! - the pure ingress-hash resolver arms: inline oracle tests in
//!   `graph/runtime.rs`;
//! - the gateway RUNTIME (tap attach/detach, forwarding, ingress e2e):
//!   `gateway_iox2_test.rs`;
//! - THIS file: `compute_gateway_plan` oracles (explicit block ⇒ AllowList +
//!   announces + resolved ingress table; permissive ⇒ AllowAll + all produced;
//!   strict-no-block ⇒ None; disabled falls to the posture; ingress-nobody-
//!   consumes refuses) + the build-starts-nothing-network pin.
//!
//! Per-test SHM roots (`init_for_test`) make the build pin parallel-safe; the
//! plan oracles are pure (no transport).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{
    GraphConfig, InputDef, NetworkBlock, NetworkMode, NodeDef, OutputDef,
};
use cerulion_core::graph::node::{NodeEntry, NodeInfo};
use cerulion_core::graph::{compute_gateway_plan, GraphRuntime, GraphTopology};
use cerulion_core::prelude::*;
use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan};
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{NetworkPosture, TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_topic(tag: &str) -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/nwwire/{tag}/{nanos}/{id}")
}

fn test_manager(node_name: &str, network: Option<NetworkConfig>) -> Arc<TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = TransportConfig {
        node_name: node_name.into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network,
    };
    TransportManager::init_for_test(transport_config, ix_config)
        .expect("init per-test transport manager")
}

// ===========================================================================
// Node types
// ===========================================================================

#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct NwProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl NwProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 7.0;
        Ok(())
    }
}

/// A data-trigger consumer of a Vector3 — its `#[input(trigger)]` carries
/// `Vector3::SCHEMA_HASH` in `InputMeta`, the ingress expected hash.
#[cerulion_node]
#[derive(Default)]
struct NwSink {
    #[input(trigger)]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl NwSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.cmd.x;
        Ok(())
    }
}

// ===========================================================================
// Graph builders
// ===========================================================================

fn peer_block(egress: Vec<String>, ingress: Vec<String>) -> NetworkBlock {
    NetworkBlock {
        mode: NetworkMode::Peer,
        connect: vec![],
        listen: vec![],
        egress,
        ingress,
    }
}

fn disabled_block(egress: Vec<String>, ingress: Vec<String>) -> NetworkBlock {
    NetworkBlock {
        mode: NetworkMode::Disabled,
        connect: vec![],
        listen: vec![],
        egress,
        ingress,
    }
}

/// A graph with one producer (publishing the absolute `egress_topic` override)
/// and one consumer (reading the absolute `ingress_topic`). Either topic may be
/// absent (empty string ⇒ that node is omitted).
fn mixed_graph(
    prefix: &str,
    egress_topic: &str,
    ingress_topic: &str,
    network: Option<NetworkBlock>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let mut nodes = Vec::new();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    if !egress_topic.is_empty() {
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: "src".to_string(),
            node_type: "nw_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                topic: Some(egress_topic.to_string()),
                history_size: 0,
            }],
        });
        factories.insert("src".to_string(), Box::new(NwProducerEntry::new()));
    }
    if !ingress_topic.is_empty() {
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "nw_sink".to_string(),
            inputs: vec![InputDef {
                name: "cmd".to_string(),
                source: ingress_topic.to_string(),
            }],
            outputs: vec![],
        });
        factories.insert("sink".to_string(), Box::new(NwSinkEntry::new()));
    }
    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "nw_wiring".to_string(),
        prefix: prefix.to_string(),
        nodes,
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Vec::new(),
        level_assignments: None,
        network,
    };
    (config, factories)
}

/// Compute the gateway plan for a graph (pure): entry_infos come from each
/// entry's `info()`, topology from `GraphTopology::build`.
fn plan_for(
    config: &GraphConfig,
    factories: &IndexMap<String, Box<dyn NodeEntry>>,
    posture: NetworkPosture,
) -> cerulion_core::error::TransportResult<Option<GatewayPlan>> {
    let infos: IndexMap<String, NodeInfo> = factories
        .iter()
        .map(|(id, e)| (id.clone(), e.info().expect("node info")))
        .collect();
    let topology = GraphTopology::build(config, &infos).expect("topology builds");
    compute_gateway_plan(config, &topology, &infos, posture)
}

// ===========================================================================
// Tests — compute_gateway_plan oracles
// ===========================================================================

/// An explicit ENABLED block ⇒ AllowList(declared egress) + announce = declared
/// egress + one ingress entry per declared ingress topic with the hash resolved
/// from the consuming input's macro metadata.
#[test]
fn explicit_block_yields_allowlist_announce_and_resolved_ingress() {
    let egress = unique_topic("e");
    let ingress = unique_topic("i");
    let (config, factories) = mixed_graph(
        "nwx",
        &egress,
        &ingress,
        Some(peer_block(vec![egress.clone()], vec![ingress.clone()])),
    );
    // Posture is irrelevant for an explicit block (the block wins).
    let plan = plan_for(&config, &factories, NetworkPosture::Strict)
        .expect("plan computes")
        .expect("an enabled block yields a plan");

    assert_eq!(
        plan.egress_policy,
        GatewayEgressPolicy::AllowList(vec![egress.clone()]),
        "explicit egress list becomes the allow-list"
    );
    assert_eq!(plan.announce, vec![egress], "announce = declared egress");
    assert_eq!(plan.ingress.len(), 1);
    assert_eq!(plan.ingress[0].topic, ingress);
    assert_eq!(
        plan.ingress[0].schema_hash,
        Vector3::SCHEMA_HASH,
        "ingress hash resolved from the consuming input's macro metadata"
    );
}

/// No enabled block + PermissiveDefault ⇒ AllowAll + announce every produced
/// topic + no ingress.
#[test]
fn permissive_default_yields_allow_all_and_all_produced() {
    let egress = unique_topic("prod");
    let (config, factories) = mixed_graph("nwp", &egress, "", None);
    let plan = plan_for(&config, &factories, NetworkPosture::PermissiveDefault)
        .expect("plan computes")
        .expect("permissive posture yields a plan");

    assert_eq!(plan.egress_policy, GatewayEgressPolicy::AllowAll);
    assert_eq!(
        plan.announce,
        vec![egress],
        "announce = every produced topic (the override name)"
    );
    assert!(plan.ingress.is_empty(), "permissive default has no ingress");
}

/// No enabled block + Strict (the default) ⇒ None (local-only, no gateway).
#[test]
fn strict_no_block_yields_no_plan() {
    let egress = unique_topic("prod");
    let (config, factories) = mixed_graph("nws", &egress, "", None);
    let plan = plan_for(&config, &factories, NetworkPosture::Strict).expect("plan computes");
    assert!(
        plan.is_none(),
        "strict + no block must be local-only (no plan)"
    );
}

/// A DISABLED block is not enabled ⇒ it falls through to the posture: Strict ⇒
/// None, PermissiveDefault ⇒ AllowAll (the disabled block's egress/ingress lists
/// are ignored — an inert block).
#[test]
fn disabled_block_falls_through_to_posture() {
    let egress = unique_topic("prod");
    // Disabled block carries lists that must be IGNORED.
    let net = Some(disabled_block(vec!["/should/ignore".to_string()], vec![]));
    let (config, factories) = mixed_graph("nwd", &egress, "", net);

    let strict = plan_for(&config, &factories, NetworkPosture::Strict).expect("plan computes");
    assert!(strict.is_none(), "disabled + strict ⇒ None");

    let permissive = plan_for(&config, &factories, NetworkPosture::PermissiveDefault)
        .expect("plan computes")
        .expect("disabled + permissive falls through to AllowAll");
    assert_eq!(permissive.egress_policy, GatewayEgressPolicy::AllowAll);
    assert_eq!(
        permissive.announce,
        vec![egress],
        "the disabled block's lists are ignored — announce the produced topic"
    );
}

/// An ingress topic nobody consumes refuses LOUDLY (naming the topic + fix).
#[test]
fn ingress_nobody_consumes_refuses_loudly() {
    let ghost = unique_topic("ghost");
    // Producer only (no consumer of `ghost`); block declares it as ingress.
    let egress = unique_topic("e");
    let (config, factories) = mixed_graph(
        "nwg",
        &egress,
        "", // no consumer node
        Some(peer_block(vec![egress.clone()], vec![ghost.clone()])),
    );
    let err = plan_for(&config, &factories, NetworkPosture::Strict)
        .expect_err("an unconsumed ingress topic must refuse");
    let msg = format!("{err}");
    assert!(
        msg.contains(&ghost) && msg.contains("no in-graph consumer"),
        "refusal must name the topic + the fix: {msg}"
    );
}

// ===========================================================================
// Test — the graph build starts NOTHING network
// ===========================================================================

/// Even with an enabled `network:` block AND a network-configured
/// transport, the graph build opens NO zenoh session and starts NO watch task —
/// a graph process is network-free (the gateway is separate). This is the strong
/// form of the "build wires nothing" pin (earlier a non-empty egress list
/// started the watch here).
#[test]
fn graph_build_starts_no_network_session_or_watch() {
    let egress = unique_topic("nostart");
    let mgr = test_manager("nw_nostart", Some(NetworkConfig::default()));
    let (config, factories) = mixed_graph(
        "nwns",
        &egress,
        "",
        Some(peer_block(vec![egress.clone()], vec![])),
    );
    let clock = Arc::new(VirtualClock::new());

    let _runtime = GraphRuntime::build(config, factories, &mgr, clock)
        .expect("the graph must still build over a network-configured transport");

    let net = mgr.network().expect("transport carries a network manager");
    assert!(
        !net.is_active(),
        "the graph build must open NO zenoh session (a graph process is network-free)"
    );
    assert!(
        !net.is_task_running(),
        "the graph build must start NO liveliness watch task"
    );
    // And no bridge flag was registered (publishers are network-free now).
    assert!(
        !mgr.bridge_manager()
            .is_registered(&egress)
            .expect("bridge registry readable"),
        "a graph publisher must register no egress bridge flag"
    );
}
