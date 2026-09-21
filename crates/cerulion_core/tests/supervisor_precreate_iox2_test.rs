// SPDX-License-Identifier: AGPL-3.0-only
//! The CORE SEAMS for supervisor-side pre-creation of every
//! graph-owned iceoryx2 service BEFORE any worker spawns.
//!
//! # The bug
//!
//! A multi-process deployment spawns one worker process per
//! `process_groups:` band. Each worker builds ONLY its own subgraph, so a
//! consumer-only worker that happens to spawn FIRST wins the iceoryx2 create
//! race and creates the shared topic's service at ITS local (under-provisioned)
//! view — a later producer-owning worker then fails to open at the higher
//! requirement, and the humanoid per-node baseline dies at spawn.
//!
//! # The fix (the seams)
//!
//! The supervisor pre-creates ALL graph-owned services at the FULL finalized
//! [`cerulion_core::transport::TopicServiceConfig`] (not the serde-portable
//! `TopicRequirements` REDUCTION, which drops `max_publishers` /
//! `publisher_provisioning` / `history_size`) BEFORE spawning any worker. A
//! consumer-only worker can then only OPEN the already-correct service.
//!
//! Two seams (the supervisor wiring that calls them lives in `cerulion_cli_engine`):
//!
//! - `GraphRuntime::owned_topic_configs()` — the FULL finalized config for every
//!   OWNED topic, the exact map the monolith precreate loop consumes.
//! - `TransportManager::detached_with_config()` — a production, NON-singleton
//!   iceoryx2 node the supervisor mints on the WORKERS' shared namespace to
//!   pre-create their services (its own `get()` singleton is the planning node).
//!
//! # What this file pins (`build_for_test`-based, real iceoryx2)
//!
//! - `owned_topic_configs_reduce_to_stamped_requirements`: the config-fidelity
//!   pin (plan risk #1) — for every owned topic, the REDUCTION of
//!   `owned_topic_configs()[t]` equals the stamped `topic_requirements()[t]`, so
//!   pre-creating with the full config provisions exactly what the workers union
//!   in. Exercises both the borrow floor and a raised buffer.
//! - `detached_with_config_creates_openable_service_and_stays_non_singleton`:
//!   two detached managers on one SHM root — a service created through one is
//!   openable + data-flows through the other, and the process singleton is
//!   untouched.
//!
//! Run:
//! ```bash
//! cargo test -p cerulion_core --test supervisor_precreate_iox2_test -- --test-threads=1
//! ```

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{
    PublisherProvisioning, TopicRequirements, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// iceoryx2's default `subscriber_max_borrowed_samples` — the concrete count the
/// harvest folds `None` to. Hardcoded here (not the `pub(crate)` constant) and
/// independently pinned by `non_trigger_hold_iox2_test::
/// iceoryx2_default_borrowed_samples_is_two`; a drift there fails BOTH files.
const ICEORYX2_DEFAULT_BORROWED: usize = 2;

/// The held-snapshot borrow floor (`ICEORYX2_DEFAULT_BORROWED + 1`) that
/// a non-trigger latest-value input raises its source topic to.
const HELD_BORROW: usize = ICEORYX2_DEFAULT_BORROWED + 1;

/// The deep consumer's declared `#[input(depth = 32)]` — the raised buffer
/// ceiling oracle (hand-pasted, above the global default 16 passed to
/// `build_for_test`).
const DEEP_DEPTH: usize = 32;

// ===========================================================================
// Node types exercising the three reduction axes on three distinct topics:
//   producer/a  -> snapshot consumer  => borrow HELD, default buffer
//   producer/b  -> deep snapshot      => borrow HELD, buffer DEEP_DEPTH
//   producer/c  -> data-trigger       => default borrow, default buffer
// ===========================================================================

/// Period source with three independent outputs (one per reduction axis).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PrecreateProducer {
    #[output]
    a: Vector3,
    #[output]
    b: Vector3,
    #[output]
    c: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl PrecreateProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.a.x = self.n as f64;
        self.b.x = self.n as f64;
        self.c.x = self.n as f64;
        Ok(())
    }
}

/// Non-trigger latest-value input (external → HOST-driven, so `snap` is a
/// snapshot source that raises its topic's borrow floor to HELD_BORROW).
#[cerulion_node(external)]
#[derive(Default)]
struct SnapConsumer {
    #[input]
    snap: Vector3,
}
#[cerulion_node_impl]
impl SnapConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.snap.x;
        Ok(())
    }
    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Deep non-trigger input — raises BOTH the borrow floor (snapshot) AND the
/// buffer ceiling (`#[input(depth = 32)]`) on its source topic.
#[cerulion_node(external)]
#[derive(Default)]
struct DeepConsumer {
    #[input(backpressure = drop_oldest, depth = 32)]
    deep: Vector3,
}
#[cerulion_node_impl]
impl DeepConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.deep.x;
        Ok(())
    }
    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Data-trigger consumer — a trigger input is NOT a snapshot source, so its
/// topic keeps the default borrow count (the `unwrap_or(default)` fold).
#[cerulion_node]
#[derive(Default)]
struct TrigConsumer {
    #[input(trigger)]
    trig: Vector3,
}
#[cerulion_node_impl]
impl TrigConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.trig.x;
        Ok(())
    }
}

fn out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn input(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

/// The three-axis graph, built via `build_for_test` (the monolith path). Global
/// default buffer 16 so `max(16, DEEP_DEPTH)` isolates the buffer raise to
/// producer/b.
fn build_three_axis_graph() -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "precreate".to_string(),
        prefix: "precreate".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "precreate_producer".to_string(),
                inputs: vec![],
                outputs: vec![out("a"), out("b"), out("c")],
            },
            NodeDef {
                ros2: None,
                id: "snap".to_string(),
                node_type: "snap_consumer".to_string(),
                inputs: vec![input("snap", "producer/a")],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "deep".to_string(),
                node_type: "deep_consumer".to_string(),
                inputs: vec![input("deep", "producer/b")],
                outputs: vec![],
            },
            NodeDef {
                ros2: None,
                id: "trig".to_string(),
                node_type: "trig_consumer".to_string(),
                inputs: vec![input("trig", "producer/c")],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(PrecreateProducerEntry::new()),
    );
    factories.insert("snap".to_string(), Box::new(SnapConsumerEntry::new()));
    factories.insert("deep".to_string(), Box::new(DeepConsumerEntry::new()));
    factories.insert("trig".to_string(), Box::new(TrigConsumerEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("three-axis graph must build via the monolith path")
}

/// The full config / requirement keyed by a `…/producer/<output>` suffix (robust
/// to the leading-slash form of the derived topic key).
fn by_suffix<'a, V>(
    map: &'a std::collections::BTreeMap<String, V>,
    suffix: &str,
) -> (&'a String, &'a V) {
    map.iter()
        .find(|(k, _)| k.ends_with(suffix))
        .unwrap_or_else(|| {
            panic!(
                "no owned topic ending in '{suffix}'; keys = {:?}",
                map.keys().collect::<Vec<_>>()
            )
        })
}

/// Hand-oracle reduction of a full config, mirroring
/// `TopicRequirements::from_service_config` using ONLY public fields (a genuine
/// oracle, not a call to the `pub(crate)` reducer — so a silent change to the
/// real reducer's formula surfaces here as a mismatch).
fn reduce(cfg: &TopicServiceConfig) -> TopicRequirements {
    TopicRequirements {
        min_borrowed_samples: cfg
            .subscriber_max_borrowed_samples
            .unwrap_or(ICEORYX2_DEFAULT_BORROWED),
        min_buffer: cfg.subscriber_max_buffer_size,
        min_subscribers: cfg.max_subscribers.unwrap_or(0),
        min_event_listeners: cfg.extra_event_listeners,
    }
}

#[test]
#[serial]
fn owned_topic_configs_reduce_to_stamped_requirements() {
    let runtime = build_three_axis_graph();
    let configs = runtime.owned_topic_configs();
    let reqs = runtime.topic_requirements();

    // Both maps derive from the SAME `topic_configs` under the SAME
    // `!= External` filter — identical key sets, or the two seams have drifted.
    let cfg_keys: Vec<&String> = configs.keys().collect();
    let req_keys: Vec<&String> = reqs.keys().collect();
    assert_eq!(
        cfg_keys, req_keys,
        "owned_topic_configs() and topic_requirements() must cover the SAME owned topics"
    );
    // The three owned producer outputs are present (nothing collapsed away).
    assert_eq!(
        configs.len(),
        3,
        "three owned topics expected (producer/a,b,c); got {:?}",
        configs.keys().collect::<Vec<_>>()
    );

    // THE FIDELITY PIN (plan risk #1): reducing the FULL config the supervisor
    // pre-creates with yields EXACTLY the requirement the workers get stamped —
    // no gap, no over/under-provisioning at the seam.
    for (topic, cfg) in configs {
        assert_eq!(
            reduce(cfg),
            reqs[topic],
            "the reduction of owned_topic_configs()['{topic}'] must equal the \
             stamped topic_requirements()['{topic}']"
        );
        // Every owned topic is single-writer — a term the REDUCTION cannot carry
        // (TopicRequirements has no publisher field), which is precisely why
        // pre-creation needs the full config, not the reduction.
        assert_eq!(
            cfg.publisher_provisioning,
            PublisherProvisioning::SingleWriter,
            "owned topic '{topic}' must be SingleWriter"
        );
        assert_eq!(
            cfg.max_publishers,
            Some(1),
            "owned topic '{topic}' must provision max_publishers = Some(1)"
        );
    }

    // Union-path oracles — prove each axis was genuinely exercised (not a
    // vacuous all-default equality).
    let (_, req_a) = by_suffix(reqs, "producer/a");
    assert_eq!(
        req_a.min_borrowed_samples, HELD_BORROW,
        "the snapshot topic producer/a must carry the held-borrow floor (3)"
    );
    assert_eq!(
        req_a.min_buffer, 16,
        "producer/a keeps the global default buffer (no depth override)"
    );

    let (_, cfg_b) = by_suffix(configs, "producer/b");
    let (_, req_b) = by_suffix(reqs, "producer/b");
    assert_eq!(
        cfg_b.subscriber_max_borrowed_samples,
        Some(HELD_BORROW),
        "producer/b's full config carries the borrow floor Some(3)"
    );
    assert_eq!(
        cfg_b.subscriber_max_buffer_size, DEEP_DEPTH,
        "producer/b's full config carries the raised buffer 32"
    );
    assert_eq!(
        req_b.min_buffer, DEEP_DEPTH,
        "producer/b's reduction must carry the raised buffer 32"
    );
    assert_eq!(
        req_b.min_borrowed_samples, HELD_BORROW,
        "producer/b's reduction must carry the borrow floor 3"
    );

    let (_, req_c) = by_suffix(reqs, "producer/c");
    assert_eq!(
        req_c.min_borrowed_samples, ICEORYX2_DEFAULT_BORROWED,
        "the data-trigger topic producer/c keeps the default borrow (the None -> 2 fold)"
    );
}

/// A fresh detached `TransportConfig` with the given distinct node name.
fn detached_config(node_name: &str, buffer: usize) -> TransportConfig {
    TransportConfig {
        node_name: node_name.into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: buffer,
        network: None,
    }
}

#[test]
#[serial]
fn detached_with_config_creates_openable_service_and_stays_non_singleton() {
    // Binary-scoped non-interference: this file initializes NO singleton, so the
    // process singleton must be uninitialized both before AND after minting
    // detached managers — a set INSTANCE would mean detached_with_config leaked
    // into it (the whole point is that it does NOT).
    assert!(
        TransportManager::get().is_err(),
        "no test in this file inits the singleton — get() must be NotInitialized"
    );

    let ix = cerulion_core::testing::iceoryx_test_config();
    // Two detached managers on the SAME shared namespace — the supervisor's
    // pre-creator + a stand-in for a worker opening the same SHM root.
    // Role-distinct node names for attribution only — iceoryx2 keys node
    // identity by a counter-derived UniqueNodeId, never by name (same-named
    // nodes are legal).
    let precreator =
        TransportManager::detached_with_config(detached_config("precreate_a", 16), ix.clone())
            .expect("detached pre-creator manager");
    let opener = TransportManager::detached_with_config(detached_config("precreate_b", 16), ix)
        .expect("detached opener manager");

    // Non-singleton: each call returns an INDEPENDENT manager.
    assert!(
        !Arc::ptr_eq(&precreator, &opener),
        "detached_with_config must return a fresh, independent manager per call"
    );
    // Singleton still untouched after two detached mints.
    assert!(
        TransportManager::get().is_err(),
        "detached_with_config must not populate the process singleton"
    );

    // A service created through the pre-creator is OPENABLE (open-only, which
    // structurally cannot create) through the other manager — the
    // supervisor-creates / worker-opens shape, plus a real data round-trip so
    // the detached manager is proven to be a working transport, not just a node.
    let topic = "/precreate/roundtrip";
    let mut publisher = precreator
        .create_publisher(topic, MaxSliceLen::const_new(256), 0)
        .expect("pre-creator creates the topic service");
    let subscriber = opener
        .create_subscriber_open_only(topic)
        .expect("opener attaches OPEN-ONLY to the pre-creator's service (proves visibility)");

    {
        let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan_proxy");
        proxy.x = 42.0;
    } // drop = publish

    let mut seen = 0u32;
    let received = subscriber
        .try_receive(|_msg| {
            seen += 1;
        })
        .expect("receive across detached managers");
    assert!(
        received >= 1 && seen >= 1,
        "data must flow from the pre-creator's publisher to the opener's subscriber \
         (received={received}, seen={seen})"
    );
}
