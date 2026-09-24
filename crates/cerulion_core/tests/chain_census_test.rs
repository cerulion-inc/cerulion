// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle vectors for the chain-fusion census: one rule, one qualify arm, one
//! refuse arm.
//!
//! Every test asserts BOTH halves of the answer against a hand-written vector:
//! the chains that qualify, and the reason recorded for every edge that does
//! not. Asserting only the chains would pass an analysis that refuses
//! everything for the wrong reason; asserting only the reasons would pass one
//! that records the right reason and then fuses the edge anyway.
//!
//! Most rules come in PAIRS built from the same graph with one declaration
//! changed, so the arm is what the test is sensitive to rather than the shape
//! of the fixture.
//!
//! Pure: no transport, no shared memory, no fixtures, so it is parallel-safe
//! and needs no serialization.
//!
//! ```bash
//! cargo test -p cerulion_core --test chain_census_test
//! ```

use indexmap::IndexMap;

use cerulion_core::graph::chain::{census_chains, ChainCensus, Colocation, MAX_FUSED_CHAIN_NODES};
use cerulion_core::graph::node::{BackpressurePolicy, InputMeta, MacroPolicy, NodeInfo};
use cerulion_core::graph::topology::{GraphTopology, Levels, TriggerEdges};
use cerulion_core::graph::{build_trigger_edges, parse_graph, GraphConfig};

// ==========================================================================
// Harness
// ==========================================================================

/// One node type's declarations, as the macro would have produced them.
struct Spec {
    policy: Option<MacroPolicy>,
    throttle_ms: Option<u64>,
    /// `(input name, trigger, backpressure)`.
    inputs: Vec<(&'static str, bool, BackpressurePolicy)>,
}

impl Spec {
    /// A clock-driven node.
    fn period(period_ms: u64) -> Self {
        Self {
            policy: Some(MacroPolicy::Period { period_ms }),
            throttle_ms: None,
            inputs: Vec::new(),
        }
    }

    /// A node woken by the named input.
    fn data(input: &'static str) -> Self {
        Self {
            policy: Some(MacroPolicy::DataTrigger {
                input_name: input.to_string(),
            }),
            throttle_ms: None,
            inputs: vec![(input, true, BackpressurePolicy::DropOldest)],
        }
    }

    /// A node with no macro policy: the runtime fires it on any input arrival.
    fn undeclared(inputs: &[&'static str]) -> Self {
        Self {
            policy: None,
            throttle_ms: None,
            inputs: inputs
                .iter()
                .map(|n| (*n, true, BackpressurePolicy::DropOldest))
                .collect(),
        }
    }

    /// Add a latest-value input.
    fn ctx(mut self, name: &'static str) -> Self {
        self.inputs
            .push((name, false, BackpressurePolicy::DropOldest));
        self
    }

    /// Replace one input's backpressure policy.
    fn backpressure(mut self, name: &str, policy: BackpressurePolicy) -> Self {
        for input in &mut self.inputs {
            if input.0 == name {
                input.2 = policy;
            }
        }
        self
    }

    fn throttled(mut self, throttle_ms: u64) -> Self {
        self.throttle_ms = Some(throttle_ms);
        self
    }

    fn sync(mut self, window_ms: u64) -> Self {
        self.policy = Some(MacroPolicy::Sync { window_ms });
        self
    }

    fn external(mut self) -> Self {
        self.policy = Some(MacroPolicy::External);
        self
    }

    fn unbounded_sync(mut self) -> Self {
        self.policy = Some(MacroPolicy::UnboundedSync);
        self
    }

    fn info(&self) -> NodeInfo {
        let meta: Vec<InputMeta> = self
            .inputs
            .iter()
            .map(|(name, trigger, backpressure)| InputMeta {
                name: (*name).to_string(),
                schema_hash: 0,
                trigger: *trigger,
                depth: 10,
                backpressure: *backpressure,
                expect_within_ms: None,
            })
            .collect();
        let mut info = NodeInfo::with_meta(meta, Vec::new());
        if let Some(policy) = self.policy.clone() {
            info = info.with_policy(policy);
        }
        if let Some(throttle_ms) = self.throttle_ms {
            info = info.with_throttle_ms(throttle_ms);
        }
        info
    }
}

/// A parsed graph plus everything the analysis reads.
struct Harness {
    config: GraphConfig,
    infos: IndexMap<String, NodeInfo>,
    topology: GraphTopology,
    trigger_edges: TriggerEdges,
    levels: Levels,
}

impl Harness {
    fn new(yaml: &str, specs: &[(&str, Spec)]) -> Self {
        let config = parse_graph(yaml).expect("the fixture graph must parse");
        let infos: IndexMap<String, NodeInfo> = specs
            .iter()
            .map(|(id, spec)| ((*id).to_string(), spec.info()))
            .collect();
        let trigger_edges = build_trigger_edges(&config, &infos);
        let topology = GraphTopology::build(&config, &infos).expect("the fixture must build");
        let levels = topology
            .derive_levels(&trigger_edges)
            .expect("the fixture must levelize");
        Self {
            config,
            infos,
            topology,
            trigger_edges,
            levels,
        }
    }

    /// The census with every node in one process.
    fn monolith(&self) -> ChainCensus {
        census_chains(
            &self.config,
            &self.infos,
            &self.topology,
            &self.trigger_edges,
            &self.levels,
            Colocation::SingleProcess,
        )
    }

    /// The census under the graph's own `process_groups:` block.
    fn groups(&self) -> ChainCensus {
        census_chains(
            &self.config,
            &self.infos,
            &self.topology,
            &self.trigger_edges,
            &self.levels,
            Colocation::ProcessGroups(&self.config.process_groups),
        )
    }
}

/// The chains a census found, one string per chain: `head->next->...`.
fn chains(census: &ChainCensus) -> Vec<String> {
    census.chains().iter().map(|c| c.nodes.join("->")).collect()
}

/// Every edge's verdict: `consumer.input <- producer : label`, where the label
/// is `FUSED` for a hop. Ordered as the census orders edges, which is graph
/// order.
fn verdicts(census: &ChainCensus) -> Vec<String> {
    census
        .edges()
        .iter()
        .map(|e| {
            format!(
                "{}.{} <- {} : {}",
                e.consumer,
                e.input,
                e.producer.as_deref().unwrap_or("-"),
                e.bar.as_ref().map(|b| b.label()).unwrap_or("FUSED"),
            )
        })
        .collect()
}

/// The rendered sentence of the first edge carrying `label`.
fn sentence(census: &ChainCensus, label: &str) -> String {
    census
        .edges()
        .iter()
        .find_map(|e| match &e.bar {
            Some(bar) if bar.label() == label => Some(bar.to_string()),
            _ => None,
        })
        .unwrap_or_else(|| panic!("no edge carries the bar '{label}'"))
}

/// The census accounts for every consumer edge exactly once.
///
/// Checked three ways, because the three are computed from different places
/// and a chain that held an edge nothing barred, or a bar count that lost one,
/// would show up here rather than in a test nobody wrote.
fn assert_total(census: &ChainCensus) {
    let ranked = census.bars().iter().map(|(_, n)| n).sum::<usize>();
    assert_eq!(
        ranked,
        census.queued_edge_count(),
        "the ranked reasons must name every queued edge"
    );
    assert_eq!(
        census.fused_hop_count() + census.queued_edge_count(),
        census.consumer_edge_count(),
        "the fused hops plus the queued edges must be every consumer edge"
    );
    assert_eq!(
        census.fused_hop_count(),
        census.edges().iter().filter(|e| e.is_fused()).count(),
        "every edge with no bar must be a hop of exactly one chain"
    );
}

// ==========================================================================
// The shape that fuses
// ==========================================================================

const CHAIN_YAML: &str = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: relay
    type: relay
    inputs:
      - name: inp
        source: src/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: relay/out
"#;

fn chain_specs() -> Vec<(&'static str, Spec)> {
    vec![
        ("src", Spec::period(10)),
        ("relay", Spec::data("inp")),
        ("sink", Spec::data("inp")),
    ]
}

#[test]
fn a_linear_single_consumer_chain_in_one_process_fuses_end_to_end() {
    let h = Harness::new(CHAIN_YAML, &chain_specs());
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->relay->sink"]);
    assert_eq!(
        verdicts(&census),
        vec!["relay.inp <- src : FUSED", "sink.inp <- relay : FUSED"]
    );
    assert_eq!(census.fused_hop_count(), 2);
    assert_eq!(census.longest_chain_nodes(), 3);
    assert_eq!(census.chains()[0].head_level, 0);
    assert_eq!(
        census.chains()[0].topics,
        vec!["/t/src/out", "/t/relay/out"]
    );
    // Chain order and level order coincide for a linear chain, which is what
    // lets a fused consumer be recorded in its own level while running inside
    // the head's level phase.
    for (i, node) in census.chains()[0].nodes.iter().enumerate() {
        assert_eq!(
            h.levels.level_of(node),
            Some(census.chains()[0].head_level + i),
            "node {i} of the chain must sit exactly {i} levels below its head"
        );
    }
    assert_total(&census);
}

#[test]
fn two_analyses_of_one_graph_produce_identical_output() {
    let h = Harness::new(CHAIN_YAML, &chain_specs());
    assert_eq!(h.monolith(), h.monolith());
}

// ==========================================================================
// Rule: the edge is a trigger edge
// ==========================================================================

#[test]
fn a_latest_value_read_is_not_a_hop() {
    // `sink` reads `relay/out` as context and is fired by its own clock, so the
    // edge is not a hop at all.
    let h = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            ("relay", Spec::data("inp")),
            ("sink", Spec::period(50).ctx("inp")),
        ],
    );
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->relay"]);
    assert_eq!(
        verdicts(&census),
        vec![
            "relay.inp <- src : FUSED",
            "sink.inp <- relay : latest-value-read",
        ]
    );
    assert_eq!(
        sentence(&census, "latest-value-read"),
        "the consumer reads this topic as a latest value and is not woken by it"
    );
    assert_total(&census);
}

#[test]
fn an_external_consumer_reads_its_input_as_a_latest_value() {
    // An external node fires from outside the graph, so none of its inputs is a
    // trigger edge and the analysis says exactly that rather than blaming the
    // policy for an edge that was never a hop.
    let h = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            ("relay", Spec::data("inp")),
            ("sink", Spec::data("inp").external()),
        ],
    );
    let census = h.monolith();
    assert_eq!(chains(&census), vec!["src->relay"]);
    assert_eq!(
        verdicts(&census),
        vec![
            "relay.inp <- src : FUSED",
            "sink.inp <- relay : latest-value-read",
        ]
    );
}

// ==========================================================================
// Rule: exactly one in-graph producer
// ==========================================================================

#[test]
fn a_topic_no_node_in_this_graph_publishes_is_not_a_hop() {
    let yaml = r#"
prefix: t
nodes:
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: /outside/feed
"#;
    let h = Harness::new(yaml, &[("sink", Spec::data("inp"))]);
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec!["sink.inp <- - : no-in-graph-producer"]
    );
    assert_total(&census);
}

#[test]
fn a_topic_two_nodes_publish_is_not_a_hop() {
    let yaml = r#"
prefix: t
multi_publisher_topics:
  - /shared/tf
nodes:
  - id: left
    type: left
    outputs:
      - name: out
        schema: std_msgs/Int32
        topic: /shared/tf
  - id: right
    type: right
    outputs:
      - name: out
        schema: std_msgs/Int32
        topic: /shared/tf
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: /shared/tf
"#;
    let h = Harness::new(
        yaml,
        &[
            ("left", Spec::period(10)),
            ("right", Spec::period(10)),
            ("sink", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec!["sink.inp <- - : multiple-producers"]
    );
    assert_eq!(
        sentence(&census, "multiple-producers"),
        "2 nodes in this graph publish the topic, so the consumer has no single producer to be called from"
    );
}

// ==========================================================================
// Rule: exactly one consumer edge, and one fusable edge per producer
// ==========================================================================

#[test]
fn a_topic_with_two_consumers_ends_the_chain() {
    let yaml = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: a
    type: a
    inputs:
      - name: inp
        source: src/out
  - id: b
    type: b
    inputs:
      - name: inp
        source: src/out
"#;
    let h = Harness::new(
        yaml,
        &[
            ("src", Spec::period(10)),
            ("a", Spec::data("inp")),
            ("b", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec!["a.inp <- src : fan-out", "b.inp <- src : fan-out"]
    );
    assert_eq!(
        sentence(&census, "fan-out"),
        "the topic has 2 consumers, and a chain ends at a fan-out"
    );
    assert_total(&census);
}

#[test]
fn a_producer_feeding_two_single_consumer_topics_ends_the_chain() {
    // Each topic has exactly one writer and one reader, so nothing the
    // topic-level rules see refuses them. The node is still a fan-out: fusing
    // both would make one consumer wait for the other's whole chain, and it is
    // what keeps a node in at most one chain.
    let yaml = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: left
        schema: std_msgs/Int32
      - name: right
        schema: std_msgs/Int32
  - id: a
    type: a
    inputs:
      - name: inp
        source: src/left
  - id: b
    type: b
    inputs:
      - name: inp
        source: src/right
"#;
    let h = Harness::new(
        yaml,
        &[
            ("src", Spec::period(10)),
            ("a", Spec::data("inp")),
            ("b", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec![
            "a.inp <- src : producer-branches",
            "b.inp <- src : producer-branches",
        ]
    );
    assert_eq!(
        sentence(&census, "producer-branches"),
        "the producer feeds 2 single-consumer topics, and a chain ends at a fan-out"
    );
    assert_total(&census);
}

#[test]
fn a_producer_whose_second_edge_is_already_refused_still_fuses_the_first() {
    // The branch rule counts the edges that SURVIVE the per-edge rules. A
    // second output whose consumer reads it as a latest value is not a branch,
    // so refusing the first edge as well would cost a chain for nothing.
    let yaml = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: left
        schema: std_msgs/Int32
      - name: right
        schema: std_msgs/Int32
  - id: a
    type: a
    inputs:
      - name: inp
        source: src/left
  - id: b
    type: b
    inputs:
      - name: ctx
        source: src/right
"#;
    let h = Harness::new(
        yaml,
        &[
            ("src", Spec::period(10)),
            ("a", Spec::data("inp")),
            ("b", Spec::period(50).ctx("ctx")),
        ],
    );
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->a"]);
    assert_eq!(
        verdicts(&census),
        vec!["a.inp <- src : FUSED", "b.ctx <- src : latest-value-read"]
    );
}

// ==========================================================================
// Rule: one process
// ==========================================================================

const SPLIT_YAML: &str = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: relay
    type: relay
    inputs:
      - name: inp
        source: src/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: relay/out
process_groups:
  g1: [src]
  g2: [relay, sink]
"#;

#[test]
fn a_hop_across_process_groups_keeps_the_queued_path() {
    let h = Harness::new(SPLIT_YAML, &chain_specs());
    let census = h.groups();

    assert_eq!(chains(&census), vec!["relay->sink"]);
    assert_eq!(
        verdicts(&census),
        vec![
            "relay.inp <- src : separate-processes",
            "sink.inp <- relay : FUSED",
        ]
    );
    assert_eq!(
        sentence(&census, "separate-processes"),
        "the producer runs in group 'g1' and the consumer in group 'g2'"
    );
    assert_eq!(census.chains()[0].head_level, 1);
    assert_total(&census);
}

#[test]
fn the_same_split_graph_run_as_one_process_fuses_the_whole_chain() {
    // The anti-tautology half of the pair: the graph is byte-identical, only
    // the colocation the census is judged against changes.
    let h = Harness::new(SPLIT_YAML, &chain_specs());
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->relay->sink"]);
    assert_eq!(
        verdicts(&census),
        vec!["relay.inp <- src : FUSED", "sink.inp <- relay : FUSED"]
    );
}

#[test]
fn a_node_in_no_declared_group_is_never_colocated() {
    let yaml = r#"
prefix: t
nodes:
  - id: src
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: relay
    type: relay
    inputs:
      - name: inp
        source: src/out
process_groups:
  g1: [src]
"#;
    let h = Harness::new(
        yaml,
        &[("src", Spec::period(10)), ("relay", Spec::data("inp"))],
    );
    let census = h.groups();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec!["relay.inp <- src : separate-processes"]
    );
    assert_eq!(
        sentence(&census, "separate-processes"),
        "the producer runs in group 'g1' and the consumer in no declared group"
    );
}

// ==========================================================================
// Rule: the edge is neither `block` nor `sample(N)`
// ==========================================================================

#[test]
fn a_block_input_keeps_the_queued_path_and_makes_its_ends_serial() {
    // `relay` declares `block` on the hop from `src`, which refuses that edge
    // outright. It also makes `relay` block-involved, which is why its OWN
    // output edge is refused: the executor already fires it serially against
    // the shared outstanding count.
    let h = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            (
                "relay",
                Spec::data("inp").backpressure("inp", BackpressurePolicy::Block),
            ),
            ("sink", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec![
            "relay.inp <- src : block",
            "sink.inp <- relay : block-involved",
        ]
    );
    assert_eq!(
        sentence(&census, "block"),
        "the input declares `block`, whose deferral a consumer running inside the producer's own fire could never reach"
    );
    assert_eq!(
        sentence(&census, "block-involved"),
        "the producer is on a `block` topic and already fired serially against the shared outstanding count"
    );
    assert_total(&census);
}

#[test]
fn a_sample_input_keeps_the_queued_path() {
    let h = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            ("relay", Spec::data("inp")),
            (
                "sink",
                Spec::data("inp").backpressure("inp", BackpressurePolicy::Sample(20)),
            ),
        ],
    );
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->relay"]);
    assert_eq!(
        verdicts(&census),
        vec!["relay.inp <- src : FUSED", "sink.inp <- relay : sample"]
    );
    assert_eq!(
        sentence(&census, "sample"),
        "the input declares `sample(20)`, which reads less often than the producer publishes"
    );
}

// ==========================================================================
// Rule: the consumer fires on this frame
// ==========================================================================

const JOIN_YAML: &str = r#"
prefix: t
nodes:
  - id: left
    type: left
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: right
    type: right
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: fuse
    type: fuse
    inputs:
      - name: a
        source: left/out
      - name: b
        source: right/out
"#;

#[test]
fn a_sync_consumer_is_never_a_fused_consumer() {
    let h = Harness::new(
        JOIN_YAML,
        &[
            ("left", Spec::period(10)),
            ("right", Spec::period(10)),
            ("fuse", Spec::undeclared(&["a", "b"]).sync(25)),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec![
            "fuse.a <- left : consumer-policy",
            "fuse.b <- right : consumer-policy",
        ]
    );
    assert_eq!(
        sentence(&census, "consumer-policy"),
        "the consumer fires on sync, not on this frame"
    );
    assert_total(&census);
}

#[test]
fn an_unbounded_sync_consumer_is_never_a_fused_consumer() {
    let h = Harness::new(
        JOIN_YAML,
        &[
            ("left", Spec::period(10)),
            ("right", Spec::period(10)),
            ("fuse", Spec::undeclared(&["a", "b"]).unbounded_sync()),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        sentence(&census, "consumer-policy"),
        "the consumer fires on unbounded sync, not on this frame"
    );
}

#[test]
fn an_undeclared_policy_fuses_on_one_input_and_is_a_join_on_two() {
    // A node with no macro policy fires on ANY input arrival. With one wired
    // input that IS "the producer committed a frame", so it fuses; with two it
    // fires on a set, which is the join rule. The pair is what keeps the
    // fallback from being judged by a rule it does not need.
    let one = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            ("relay", Spec::data("inp")),
            ("sink", Spec::undeclared(&["inp"])),
        ],
    );
    assert_eq!(chains(&one.monolith()), vec!["src->relay->sink"]);

    let two = Harness::new(
        JOIN_YAML,
        &[
            ("left", Spec::period(10)),
            ("right", Spec::period(10)),
            ("fuse", Spec::undeclared(&["a", "b"])),
        ],
    );
    let census = two.monolith();
    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec!["fuse.a <- left : join", "fuse.b <- right : join"]
    );
    assert_eq!(
        sentence(&census, "join"),
        "the consumer has 2 trigger inputs, so it fires on a set of frames"
    );
}

// ==========================================================================
// Rule: no rate cap on the consumer
// ==========================================================================

#[test]
fn a_throttled_consumer_keeps_the_queued_path() {
    let h = Harness::new(
        CHAIN_YAML,
        &[
            ("src", Spec::period(10)),
            ("relay", Spec::data("inp").throttled(5)),
            ("sink", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["relay->sink"]);
    assert_eq!(
        verdicts(&census),
        vec!["relay.inp <- src : throttle", "sink.inp <- relay : FUSED",]
    );
    assert_eq!(
        sentence(&census, "throttle"),
        "the consumer declares `throttle_ms = 5`, and a direct call has nowhere to defer to"
    );
    assert_total(&census);
}

// ==========================================================================
// Rule: the context a fused consumer reads is already final
// ==========================================================================

/// `root -> src -> mid`, with `mid` also reading a latest value. The graph
/// differs only in WHO writes that latest value, which is the whole point of
/// the pair.
fn context_yaml(ctx_source: &str) -> String {
    format!(
        r#"
prefix: t
nodes:
  - id: root
    type: root
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: src
    type: src
    inputs:
      - name: inp
        source: root/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: other
    type: other
    inputs:
      - name: inp
        source: root/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: mid
    type: mid
    inputs:
      - name: trig
        source: src/out
      - name: ctx
        source: {ctx_source}
"#
    )
}

fn context_specs() -> Vec<(&'static str, Spec)> {
    vec![
        ("root", Spec::period(10)),
        ("src", Spec::data("inp")),
        ("other", Spec::data("inp")),
        ("mid", Spec::data("trig").ctx("ctx")),
    ]
}

#[test]
fn a_fused_consumer_may_read_context_written_below_the_chain_head() {
    // `root` writes the context at level 0 and the chain head `src` is at level
    // 1, so the value is final before the head fires.
    let h = Harness::new(&context_yaml("root/out"), &context_specs());
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["src->mid"]);
    assert_eq!(census.chains()[0].head_level, 1);
    assert!(
        verdicts(&census).contains(&"mid.trig <- src : FUSED".to_string()),
        "got {:?}",
        verdicts(&census)
    );
    assert_total(&census);
}

#[test]
fn a_fused_consumer_may_not_read_context_written_at_the_chain_heads_level() {
    // The ONLY change from the arm above: `other`, at the head's own level,
    // writes the context. It has not ticked when the head fires, so the queued
    // run and the fused run could read different values.
    let h = Harness::new(&context_yaml("other/out"), &context_specs());
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert!(
        verdicts(&census).contains(&"mid.trig <- src : late-context-input".to_string()),
        "got {:?}",
        verdicts(&census)
    );
    assert_eq!(
        sentence(&census, "late-context-input"),
        "the consumer reads 'ctx' as a latest value and 'other' writes it at level 1, \
         at or after the chain head's level 1"
    );
    assert_total(&census);
}

#[test]
fn an_edge_the_context_rule_refuses_starts_a_new_chain_at_its_consumer() {
    // `a -> b -> c -> d`, with `c` reading a latest value written at level 0.
    // The chain is CUT at `b -> c` rather than killed: `c` is judged against
    // its own level, where everything below has already ticked, so `c -> d`
    // fuses.
    let yaml = r#"
prefix: t
nodes:
  - id: ctx
    type: ctx
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: a
    type: a
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: b
    type: b
    inputs:
      - name: inp
        source: a/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: c
    type: c
    inputs:
      - name: inp
        source: b/out
      - name: ctx
        source: ctx/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: d
    type: d
    inputs:
      - name: inp
        source: c/out
"#;
    let h = Harness::new(
        yaml,
        &[
            ("ctx", Spec::period(10)),
            ("a", Spec::period(10)),
            ("b", Spec::data("inp")),
            ("c", Spec::data("inp").ctx("ctx")),
            ("d", Spec::data("inp")),
        ],
    );
    let census = h.monolith();

    assert_eq!(chains(&census), vec!["a->b", "c->d"]);
    assert_eq!(
        verdicts(&census),
        vec![
            "c.ctx <- ctx : latest-value-read",
            "b.inp <- a : FUSED",
            "c.inp <- b : late-context-input",
            "d.inp <- c : FUSED",
        ]
    );
    assert_eq!(census.chains()[1].head_level, 2);
    assert_total(&census);
}

// ==========================================================================
// Rule: the chain length ceiling
// ==========================================================================

#[test]
fn a_chain_past_the_ceiling_is_split_rather_than_truncated() {
    const NODES: usize = MAX_FUSED_CHAIN_NODES + 8;
    let mut yaml = String::from("prefix: t\nnodes:\n");
    let ids: Vec<String> = (0..NODES).map(|i| format!("n{i}")).collect();
    for i in 0..NODES {
        yaml.push_str(&format!("  - id: n{i}\n    type: n{i}\n"));
        if i > 0 {
            yaml.push_str(&format!(
                "    inputs:\n      - name: inp\n        source: n{}/out\n",
                i - 1
            ));
        }
        if i + 1 < NODES {
            yaml.push_str("    outputs:\n      - name: out\n        schema: std_msgs/Int32\n");
        }
    }
    // Node 0 has no inputs, so the clock drives it; every other node is woken
    // by its one predecessor.
    let specs: Vec<(&str, Spec)> = ids
        .iter()
        .enumerate()
        .map(|(i, id)| {
            (
                id.as_str(),
                if i == 0 {
                    Spec::period(10)
                } else {
                    Spec::data("inp")
                },
            )
        })
        .collect();

    let h = Harness::new(&yaml, &specs);
    let census = h.monolith();

    assert_eq!(census.chains().len(), 2, "{:?}", chains(&census));
    assert_eq!(census.chains()[0].nodes.len(), MAX_FUSED_CHAIN_NODES);
    assert_eq!(census.chains()[0].nodes[0], "n0");
    assert_eq!(
        census.chains()[1].nodes.len(),
        NODES - MAX_FUSED_CHAIN_NODES
    );
    assert_eq!(
        census.chains()[1].nodes[0],
        format!("n{MAX_FUSED_CHAIN_NODES}")
    );
    assert_eq!(census.bars(), vec![("length-ceiling", 1)]);
    assert_eq!(
        sentence(&census, "length-ceiling"),
        format!(
            "the chain already holds {MAX_FUSED_CHAIN_NODES} nodes, the most one chain may \
             hold; the consumer starts a new chain"
        )
    );
    assert_total(&census);
}

// ==========================================================================
// The levelization contract
// ==========================================================================

#[test]
fn an_edge_the_levelization_cannot_place_or_order_is_refused_rather_than_assumed() {
    // A levelization derived from a DIFFERENT graph places no level on `src`
    // and places `relay` and `sink` on the SAME level. Both are refusals: a
    // missing level would switch the context rule off for the edge, and an
    // edge that does not step up a level is not a hop this levelization agrees
    // with. Refusing the second is also what keeps the surviving edges a set
    // of paths, which is what makes the verdicts add up.
    let flat = Harness::new(
        r#"
prefix: t
nodes:
  - id: relay
    type: relay
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: sink
    type: sink
    outputs:
      - name: out
        schema: std_msgs/Int32
"#,
        &[("relay", Spec::period(10)), ("sink", Spec::period(10))],
    );
    assert_eq!(flat.levels.level_of("relay"), Some(0));
    assert_eq!(flat.levels.level_of("sink"), Some(0));

    let full = Harness::new(CHAIN_YAML, &chain_specs());
    let census = census_chains(
        &full.config,
        &full.infos,
        &full.topology,
        &full.trigger_edges,
        &flat.levels,
        Colocation::SingleProcess,
    );

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec![
            "relay.inp <- src : level-unknown",
            "sink.inp <- relay : level-not-increasing",
        ]
    );
    assert_eq!(
        sentence(&census, "level-unknown"),
        "the levelization places no level on 'src'"
    );
    assert_eq!(
        sentence(&census, "level-not-increasing"),
        "the levelization places the consumer at level 0, not after the producer's level 0"
    );
    assert_total(&census);
}

// ==========================================================================
// The census adds up on a mixed graph
// ==========================================================================

#[test]
fn a_mixed_graph_accounts_for_every_edge_and_ranks_its_reasons() {
    // One graph carrying a fused hop, a fan-out, a join and a latest-value
    // read, so the ranking of `bars()` is exercised rather than a single
    // reason.
    let yaml = r#"
prefix: t
nodes:
  - id: cam
    type: cam
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: imu
    type: imu
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: rect
    type: rect
    inputs:
      - name: inp
        source: cam/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: viewer
    type: viewer
    inputs:
      - name: inp
        source: cam/out
  - id: fuse
    type: fuse
    inputs:
      - name: a
        source: rect/out
      - name: b
        source: imu/out
"#;
    let h = Harness::new(
        yaml,
        &[
            ("cam", Spec::period(33)),
            ("imu", Spec::period(10)),
            ("rect", Spec::data("inp")),
            ("viewer", Spec::period(100).ctx("inp")),
            ("fuse", Spec::undeclared(&["a", "b"])),
        ],
    );
    let census = h.monolith();

    assert!(chains(&census).is_empty());
    assert_eq!(
        verdicts(&census),
        vec![
            "rect.inp <- cam : fan-out",
            "viewer.inp <- cam : latest-value-read",
            "fuse.b <- imu : join",
            "fuse.a <- rect : join",
        ]
    );
    assert_eq!(
        census.bars(),
        vec![("join", 2), ("fan-out", 1), ("latest-value-read", 1)]
    );
    assert_eq!(census.consumer_edge_count(), 4);
    assert_eq!(census.fused_hop_count(), 0);
    assert_total(&census);
}
