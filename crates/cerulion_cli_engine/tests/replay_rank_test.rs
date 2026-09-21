// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle tests for the PURE per-rank replay planner
//! (`cerulion_cli_engine::replay_rank`).
//!
//! Every input here is HAND-BUILT and every expectation is a hand-written
//! oracle — the planner is never compared against itself, and no bag, no
//! transport and no filesystem is involved (the module is pure by
//! construction, which is what makes these arms run on any platform).
//!
//! The two structural properties the arms exist to hold:
//!
//! * **membership comes from the MANIFESTS, never from `process_groups:`** —
//!   the primary fixture declares no groups at all (the crafted mp bag's own
//!   shape), and a second fixture declares groups that CONTRADICT the
//!   manifests, so a planner that read the YAML block produces a different but
//!   perfectly plausible plan and is caught;
//! * **lockstep is ONE rank plan** covering the whole graph, so the integration
//!   wave keeps one code path across both coordination modes.

use cerulion_cli_engine::replay_rank::{
    plan_ranks, CrossRankEdge, RankBoundarySummary, RankManifest, RankPlan, RankPlanError,
    RemoteProducer,
};
use cerulion_core::graph::GraphConfig;

// ===========================================================================
// Fixtures
// ===========================================================================

/// The primary two-rank shape. Deliberately carries NO `process_groups:` block
/// (an auto-derived partition never writes one back without consent, so an
/// ordinary multi-rank bag embeds exactly this), and covers all four read
/// classes in one graph:
///
/// * `mid.inp  <- src/out`         RELATIVE, in-rank      (kept verbatim)
/// * `sink.up  <- mid/out`         RELATIVE, cross-rank   (ABSOLUTIZED)
/// * `sink.ext <- /external/feed`  ABSOLUTE, produced by nobody (external)
/// * `sink.bus <- /mp/special_bus` ABSOLUTE, cross-rank via a `topic:` override
///
/// `sink`'s inputs are declared `up`, `ext`, `bus` so the emitted cross-rank
/// edge order (`up`, `bus`) pins DECLARATION order rather than name order.
fn two_rank_yaml() -> &'static str {
    "name: mpplan\n\
     prefix: mp\n\
     nodes:\n\
     \x20 - id: src\n\
     \x20   type: source_node\n\
     \x20   outputs:\n\
     \x20     - name: out\n\
     \x20       schema: geometry_msgs/Vector3\n\
     \x20     - name: special\n\
     \x20       schema: geometry_msgs/Vector3\n\
     \x20       topic: /mp/special_bus\n\
     \x20 - id: mid\n\
     \x20   type: relay_node\n\
     \x20   inputs:\n\
     \x20     - name: inp\n\
     \x20       source: src/out\n\
     \x20   outputs:\n\
     \x20     - name: out\n\
     \x20       schema: geometry_msgs/Vector3\n\
     \x20 - id: sink\n\
     \x20   type: sink_node\n\
     \x20   inputs:\n\
     \x20     - name: up\n\
     \x20       source: mid/out\n\
     \x20     - name: ext\n\
     \x20       source: /external/feed\n\
     \x20     - name: bus\n\
     \x20       source: /mp/special_bus\n"
}

fn two_rank_config() -> GraphConfig {
    cerulion_core::graph::parse_graph(two_rank_yaml()).expect("fixture graph parses")
}

/// The recorded truth for [`two_rank_yaml`]: `src` + `mid` on rank 0, `sink` on
/// rank 1. Rank 0's `node_ids` are deliberately in REVERSE graph order so
/// `members` cannot inherit a ring-order accident.
fn two_rank_manifests() -> Vec<RankManifest> {
    vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["mid".into(), "src".into()],
        },
        RankManifest {
            rank: 1,
            node_ids: vec!["sink".into()],
        },
    ]
}

fn summary(
    rank: u32,
    first_step: u64,
    last_step: u64,
    first_target_ns: u64,
) -> RankBoundarySummary {
    RankBoundarySummary::new(rank, first_step, last_step, first_target_ns)
}

/// Per-rank boundary summaries with DISTINCT step slices and DISTINCT nonzero
/// epochs — free-run ranks share a clock DOMAIN, never a value, so equal
/// numbers here would let a plan that swapped the two summaries pass.
fn two_rank_boundaries() -> Vec<RankBoundarySummary> {
    vec![
        summary(0, 0, 41, 1_700_000_000_000),
        summary(1, 0, 39, 1_700_000_000_311),
    ]
}

fn plan_of(plans: &[RankPlan], rank: u32) -> &RankPlan {
    plans
        .iter()
        .find(|p| p.rank() == rank)
        .unwrap_or_else(|| panic!("no plan for rank {rank}"))
}

fn source_of<'a>(plan: &'a RankPlan, node: &str, input: &str) -> &'a str {
    let n = plan
        .subgraph()
        .nodes
        .iter()
        .find(|n| n.id == node)
        .unwrap_or_else(|| panic!("node `{node}` missing from rank {}'s subgraph", plan.rank()));
    let i = n
        .inputs
        .iter()
        .find(|i| i.name == input)
        .unwrap_or_else(|| panic!("input `{input}` missing from node `{node}`"));
    &i.source
}

fn topics(set: &std::collections::BTreeSet<String>) -> Vec<&str> {
    set.iter().map(String::as_str).collect()
}

// ===========================================================================
// The happy two-rank plan
// ===========================================================================

#[test]
fn a_two_rank_recording_plans_exactly_the_recorded_membership_and_edges() {
    let config = two_rank_config();
    let plans = plan_ranks(&config, &two_rank_manifests(), &two_rank_boundaries())
        .expect("the two-rank fixture plans");

    assert_eq!(plans.len(), 2, "one plan per recorded worker rank");
    assert_eq!(
        plans.iter().map(|p| p.rank()).collect::<Vec<_>>(),
        vec![0, 1],
        "plans are emitted rank-ascending"
    );

    let r0 = plan_of(&plans, 0);
    let r1 = plan_of(&plans, 1);

    // Membership: from the manifests, in GRAPH order (rank 0's manifest lists
    // `mid` before `src`; the graph declares `src` first).
    assert_eq!(r0.members(), vec!["src".to_string(), "mid".to_string()]);
    assert_eq!(r1.members(), vec!["sink".to_string()]);

    // Produced: every output's resolved absolute topic, `topic:` honored.
    assert_eq!(
        topics(r0.produced()),
        vec!["/mp/mid/out", "/mp/special_bus", "/mp/src/out"],
        "rank 0 produces both of src's outputs (override honored) plus mid's"
    );
    assert!(
        r1.produced().is_empty(),
        "rank 1 is a pure consumer: {:?}",
        r1.produced()
    );

    // Rank 0 reads nothing across a boundary and nothing external.
    assert!(r0.cross_rank_consumed().is_empty());
    assert!(r0.external_consumed().is_empty());

    // Rank 1's two cross-rank reads, in INPUT DECLARATION order (`up` then
    // `bus`; `ext` is external and is not an edge). The `bus` entry is the
    // classify-by-RESOLVED-topic pin: its source is already absolute, and it
    // still crosses a rank boundary.
    assert_eq!(
        r1.cross_rank_consumed(),
        vec![
            CrossRankEdge {
                topic: "/mp/mid/out".into(),
                consumer_node: "sink".into(),
                consumer_input: "up".into(),
                producers: vec![RemoteProducer {
                    rank: 0,
                    node: "mid".into(),
                }],
            },
            CrossRankEdge {
                topic: "/mp/special_bus".into(),
                consumer_node: "sink".into(),
                consumer_input: "bus".into(),
                producers: vec![RemoteProducer {
                    rank: 0,
                    node: "src".into(),
                }],
            },
        ]
    );
    assert_eq!(
        topics(r1.external_consumed()),
        vec!["/external/feed"],
        "only the source no rank produces is external"
    );

    // The recorded step slice + the nonzero per-rank epoch, carried per rank.
    assert_eq!((r0.first_step(), r0.last_step()), (0, 41));
    assert_eq!((r1.first_step(), r1.last_step()), (0, 39));
    assert_eq!(r0.first_target_ns(), 1_700_000_000_000);
    assert_eq!(r1.first_target_ns(), 1_700_000_000_311);
}

#[test]
fn a_cross_rank_relative_source_is_absolutized_and_an_in_rank_one_is_not() {
    let config = two_rank_config();
    let plans = plan_ranks(&config, &two_rank_manifests(), &two_rank_boundaries())
        .expect("the two-rank fixture plans");
    let r0 = plan_of(&plans, 0);
    let r1 = plan_of(&plans, 1);

    // THE pin: `sink.up` was authored `mid/out`; `mid` is FOREIGN to rank 1's
    // subgraph, so the source must arrive as the resolved absolute topic — the
    // external-source shape bag-fed injection publishes into.
    assert_eq!(
        source_of(r1, "sink", "up"),
        "/mp/mid/out",
        "a cross-rank relative source is absolutized"
    );
    // Controls: an already-absolute source passes through byte-identical...
    assert_eq!(source_of(r1, "sink", "ext"), "/external/feed");
    assert_eq!(source_of(r1, "sink", "bus"), "/mp/special_bus");
    // ...and an IN-rank relative source is NOT rewritten (it must keep
    // resolving locally against its co-resident producer).
    assert_eq!(
        source_of(r0, "mid", "inp"),
        "src/out",
        "an in-rank relative source stays verbatim"
    );

    // Each rank's subgraph carries only its own members.
    assert_eq!(
        r0.subgraph()
            .nodes
            .iter()
            .map(|n| n.id.as_str())
            .collect::<Vec<_>>(),
        vec!["src", "mid"]
    );
    assert_eq!(
        r1.subgraph()
            .nodes
            .iter()
            .map(|n| n.id.as_str())
            .collect::<Vec<_>>(),
        vec!["sink"]
    );
}

#[test]
fn plans_are_rank_ascending_whatever_order_the_inputs_arrive_in() {
    let config = two_rank_config();
    let mut manifests = two_rank_manifests();
    manifests.reverse(); // rank 1 first
    let mut boundaries = two_rank_boundaries();
    boundaries.reverse(); // rank 1 first

    let plans = plan_ranks(&config, &manifests, &boundaries).expect("input order is not a fault");

    assert_eq!(
        plans.iter().map(|p| p.rank()).collect::<Vec<_>>(),
        vec![0, 1],
        "reversed inputs still plan 0..k"
    );
    // Not just the rank labels — the CONTENT must ride the right plan.
    assert_eq!(
        plan_of(&plans, 0).members(),
        vec!["src".to_string(), "mid".to_string()]
    );
    assert_eq!(plan_of(&plans, 1).members(), vec!["sink".to_string()]);
    assert_eq!(plan_of(&plans, 0).last_step(), 41);
    assert_eq!(plan_of(&plans, 1).last_step(), 39);
    assert_eq!(plan_of(&plans, 1).first_target_ns(), 1_700_000_000_311);
}

// ===========================================================================
// Membership comes from the manifests, never from `process_groups:`
// ===========================================================================

#[test]
fn membership_follows_the_manifests_even_when_the_graph_declares_other_groups() {
    // The embedded graph declares a partition that DISAGREES with what ran:
    // groups say {g0: [src], g1: [mid, sink]}, the manifests say
    // {0: [src, mid], 1: [sink]}. The recording is the truth of what ran.
    let yaml = format!(
        "{}process_groups:\n\
         \x20 g0: [src]\n\
         \x20 g1: [mid, sink]\n",
        two_rank_yaml()
    );
    let config = cerulion_core::graph::parse_graph(&yaml).expect("group-bearing fixture parses");
    assert_eq!(
        config.process_groups.len(),
        2,
        "the block really is present"
    );

    let plans = plan_ranks(&config, &two_rank_manifests(), &two_rank_boundaries())
        .expect("a contradicting group block is not a fault — it is ignored");

    assert_eq!(
        plan_of(&plans, 0).members(),
        vec!["src".to_string(), "mid".to_string()],
        "rank 0 owns what the MANIFEST says, not what group g0 says"
    );
    assert_eq!(plan_of(&plans, 1).members(), vec!["sink".to_string()]);
    // The consequence that makes the difference observable: under the group
    // block `mid` would be foreign to rank 0 and `mid.inp` would have been
    // absolutized. It is not.
    assert_eq!(source_of(plan_of(&plans, 0), "mid", "inp"), "src/out");
}

// ===========================================================================
// Lockstep is ONE rank plan
// ===========================================================================

#[test]
fn a_lockstep_recording_is_one_rank_plan_over_the_verbatim_whole_graph() {
    let config = two_rank_config();
    let manifests = vec![RankManifest {
        rank: 0,
        node_ids: vec!["src".into(), "mid".into(), "sink".into()],
    }];
    let boundaries = vec![summary(0, 0, 41, 1_700_000_000_000)];

    let plans = plan_ranks(&config, &manifests, &boundaries).expect("the lockstep shape plans");

    assert_eq!(plans.len(), 1, "lockstep replay is ONE plan");
    let p = &plans[0];
    assert_eq!(
        p.members(),
        vec!["src".to_string(), "mid".to_string(), "sink".to_string()]
    );
    // The whole-graph rank is NOT restricted: the graph keeps its own IDENTITY
    // (the resolved stem, which feeds replay's iceoryx2 node name)
    // and every node.
    assert_eq!(
        p.subgraph().identity(),
        config.identity(),
        "a whole-graph rank is not renamed to `{}_rank0`",
        config.identity()
    );
    assert_eq!(p.subgraph().nodes.len(), config.nodes.len());
    // ...and no source is rewritten, because nothing is foreign.
    assert_eq!(source_of(p, "mid", "inp"), "src/out");
    assert_eq!(source_of(p, "sink", "up"), "mid/out");
    // No edge crosses a boundary when there is only one rank; the external
    // read is still external.
    assert!(
        p.cross_rank_consumed().is_empty(),
        "{:?}",
        p.cross_rank_consumed()
    );
    assert_eq!(topics(p.external_consumed()), vec!["/external/feed"]);
    assert_eq!(
        topics(p.produced()),
        vec!["/mp/mid/out", "/mp/special_bus", "/mp/src/out"]
    );
    assert_eq!((p.first_step(), p.last_step()), (0, 41));
}

#[test]
fn a_whole_graph_rank_keeps_the_parent_process_groups_block() {
    // The short-circuit's other half: `subgraph_for` CLEARS `process_groups`
    // (a worker is a monolith). A lockstep plan must hand the executor the
    // parent config, block and all, so the one-rank path stays byte-identical
    // to today's single-runtime replay.
    let yaml = format!(
        "{}process_groups:\n\
         \x20 g0: [src]\n\
         \x20 g1: [mid, sink]\n",
        two_rank_yaml()
    );
    let config = cerulion_core::graph::parse_graph(&yaml).expect("group-bearing fixture parses");
    let plans = plan_ranks(
        &config,
        &[RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into(), "sink".into()],
        }],
        &[summary(0, 0, 3, 7)],
    )
    .expect("the lockstep shape plans");
    assert_eq!(
        plans[0].subgraph().process_groups.len(),
        2,
        "the whole-graph rank gets the parent config verbatim"
    );

    // Anti-tautology: a PROPER subset really is restricted (groups cleared),
    // so the assertion above is about the short-circuit, not about
    // `subgraph_for` never clearing anything.
    let split = plan_ranks(&config, &two_rank_manifests(), &two_rank_boundaries())
        .expect("the split shape plans");
    assert!(
        plan_of(&split, 0).subgraph().process_groups.is_empty(),
        "a restricted rank's subgraph is a monolith"
    );
    // The restricted rank's IDENTITY is the renamed one (`name:` is
    // the deprecated, optional-and-ignored key — `subgraph_for` writes `None`).
    assert_eq!(plan_of(&split, 0).subgraph().identity(), "mpplan_rank0");
    assert_eq!(plan_of(&split, 0).subgraph().name, None);
}

// ===========================================================================
// Refusals — each names its offender
// ===========================================================================

#[test]
fn a_manifest_node_the_graph_does_not_declare_is_refused_by_name() {
    let config = two_rank_config();
    let manifests = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into()],
        },
        RankManifest {
            rank: 1,
            node_ids: vec!["sink".into(), "ghost_node".into()],
        },
    ];
    let err = plan_ranks(&config, &manifests, &two_rank_boundaries())
        .expect_err("a manifest node absent from the graph is refused");
    assert_eq!(
        err,
        RankPlanError::ManifestNodeNotInGraph {
            rank: 1,
            node: "ghost_node".into(),
            graph: "mpplan".into(),
        }
    );
    let msg = err.to_string();
    assert!(msg.contains("ghost_node"), "{msg}");
    assert!(msg.contains("rank 1"), "{msg}");
    assert!(msg.contains("mpplan"), "{msg}");
    assert!(msg.contains("re-record"), "the fix is named: {msg}");
}

#[test]
fn a_graph_node_in_no_manifest_is_refused_by_name() {
    let config = two_rank_config();
    let manifests = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into()],
        },
        // `sink` recorded nowhere.
        RankManifest {
            rank: 1,
            node_ids: vec![],
        },
    ];
    let err = plan_ranks(&config, &manifests, &two_rank_boundaries())
        .expect_err("an uncovered graph node is refused");
    assert_eq!(
        err,
        RankPlanError::GraphNodeInNoManifest {
            node: "sink".into(),
            graph: "mpplan".into(),
        }
    );
    assert!(err.to_string().contains("sink"), "{err}");
}

#[test]
fn a_node_claimed_by_two_manifests_is_refused_naming_both_ranks() {
    let config = two_rank_config();
    let manifests = vec![
        RankManifest {
            rank: 1,
            node_ids: vec!["mid".into(), "sink".into()],
        },
        RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into()],
        },
    ];
    let err = plan_ranks(&config, &manifests, &two_rank_boundaries())
        .expect_err("a doubly-claimed node is refused");
    // Input order is rank 1 first; the message still names the LOWER rank as
    // the first claimant (the walk is rank-ascending).
    assert_eq!(
        err,
        RankPlanError::NodeInTwoManifests {
            node: "mid".into(),
            first_rank: 0,
            second_rank: 1,
        }
    );
    let msg = err.to_string();
    assert!(msg.contains("`mid`"), "{msg}");
    assert!(msg.contains("rank 0") && msg.contains("rank 1"), "{msg}");
}

#[test]
fn a_rank_with_no_boundary_stream_is_refused_by_rank() {
    let config = two_rank_config();
    let err = plan_ranks(
        &config,
        &two_rank_manifests(),
        &[summary(0, 0, 41, 1_700_000_000_000)],
    )
    .expect_err("a rank with no recorded clock trajectory is refused");
    assert_eq!(err, RankPlanError::RankWithoutBoundaryStream { rank: 1 });
    let msg = err.to_string();
    assert!(msg.contains("rank 1"), "{msg}");
    assert!(msg.contains("STEP_BOUNDARY"), "{msg}");
}

#[test]
fn boundary_stream_faults_are_refused_by_rank() {
    let config = two_rank_config();
    let mut extra = two_rank_boundaries();
    extra.push(summary(4, 0, 3, 1));
    assert_eq!(
        plan_ranks(&config, &two_rank_manifests(), &extra)
            .expect_err("a boundary stream no manifest claims is refused"),
        RankPlanError::BoundaryStreamWithoutManifest { rank: 4 }
    );

    let dup = vec![
        summary(0, 0, 41, 1),
        summary(1, 0, 39, 2),
        summary(1, 0, 39, 2),
    ];
    assert_eq!(
        plan_ranks(&config, &two_rank_manifests(), &dup)
            .expect_err("two summaries for one rank is refused"),
        RankPlanError::DuplicateBoundaryStream { rank: 1 }
    );

    let inverted = vec![summary(0, 0, 41, 1), summary(1, 40, 39, 2)];
    let err = plan_ranks(&config, &two_rank_manifests(), &inverted)
        .expect_err("a backwards step range is refused");
    assert_eq!(
        err,
        RankPlanError::BoundaryRangeInverted {
            rank: 1,
            first_step: 40,
            last_step: 39,
        }
    );
    assert!(err.to_string().contains("40..=39"), "{err}");
}

#[test]
fn a_broken_rank_set_is_refused_before_anything_is_planned() {
    let config = two_rank_config();

    assert_eq!(
        plan_ranks(&config, &[], &[]).expect_err("no worker manifests is refused"),
        RankPlanError::NoRankManifests
    );

    let dup = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into()],
        },
        RankManifest {
            rank: 0,
            node_ids: vec!["sink".into()],
        },
    ];
    assert_eq!(
        plan_ranks(&config, &dup, &two_rank_boundaries())
            .expect_err("two manifests claiming one rank is refused"),
        RankPlanError::DuplicateRankManifest { rank: 0 }
    );

    // Rank 1 missing entirely: the set is {0, 2}.
    let gapped = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["src".into(), "mid".into()],
        },
        RankManifest {
            rank: 2,
            node_ids: vec!["sink".into()],
        },
    ];
    let err = plan_ranks(&config, &gapped, &two_rank_boundaries())
        .expect_err("a gap in the rank set is refused");
    assert_eq!(
        err,
        RankPlanError::RankNumberingNotContiguous {
            expected: 1,
            found: 2,
        }
    );
    assert!(err.to_string().contains("rank 2"), "{err}");
}

// ===========================================================================
// One topic, two ranks
// ===========================================================================

/// Two producers of ONE absolute topic (`/mp/shared_bus`, via `topic:`
/// overrides on `alpha` and `beta`) plus a consumer. `multi_publisher` decides
/// whether the graph opts the topic into multiple publishers.
fn shared_topic_yaml(multi_publisher: bool) -> String {
    let listing = if multi_publisher {
        "multi_publisher_topics: [/mp/shared_bus]\n"
    } else {
        ""
    };
    format!(
        "name: shared\n\
         prefix: mp\n\
         {listing}\
         nodes:\n\
         \x20 - id: alpha\n\
         \x20   type: source_node\n\
         \x20   outputs:\n\
         \x20     - name: out\n\
         \x20       schema: geometry_msgs/Vector3\n\
         \x20       topic: /mp/shared_bus\n\
         \x20 - id: beta\n\
         \x20   type: source_node\n\
         \x20   outputs:\n\
         \x20     - name: out\n\
         \x20       schema: geometry_msgs/Vector3\n\
         \x20       topic: /mp/shared_bus\n\
         \x20 - id: sink\n\
         \x20   type: sink_node\n\
         \x20   inputs:\n\
         \x20     - name: bus\n\
         \x20       source: /mp/shared_bus\n"
    )
}

/// `alpha` on rank 0, `beta` + `sink` on rank 1 — the SPLIT shape.
fn split_producer_manifests() -> Vec<RankManifest> {
    vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["alpha".into()],
        },
        RankManifest {
            rank: 1,
            node_ids: vec!["beta".into(), "sink".into()],
        },
    ]
}

#[test]
fn a_topic_produced_from_two_ranks_is_refused_naming_both_producers() {
    let config = cerulion_core::graph::parse_graph(&shared_topic_yaml(false))
        .expect("shared-topic fixture parses");
    let err = plan_ranks(
        &config,
        &split_producer_manifests(),
        &[summary(0, 0, 5, 11), summary(1, 0, 5, 12)],
    )
    .expect_err("one topic produced from two ranks is refused");
    assert_eq!(
        err,
        RankPlanError::TopicProducedByTwoRanks {
            topic: "/mp/shared_bus".into(),
            first_rank: 0,
            first_node: "alpha".into(),
            second_rank: 1,
            second_node: "beta".into(),
        }
    );
    let msg = err.to_string();
    assert!(msg.contains("/mp/shared_bus"), "{msg}");
    assert!(msg.contains("`alpha`") && msg.contains("`beta`"), "{msg}");
    assert!(msg.contains("rank 0") && msg.contains("rank 1"), "{msg}");
}

#[test]
fn a_declared_multi_publisher_topic_split_across_ranks_plans_on_both_ranks() {
    // THE DEMOTION. The shape that was
    // `RankPlanError::MultiPublisherTopicSplitAcrossRanks` — one declared
    // `multi_publisher_topics:` bus written by `alpha` on rank 0 and `beta` on
    // rank 1 — now PLANS. Decision: shared topics are how robotics
    // graphs are written, and refusing to replay one made those recordings
    // unreplayable rather than making them safe.
    let config = cerulion_core::graph::parse_graph(&shared_topic_yaml(true))
        .expect("multi-publisher fixture parses");
    let plans = plan_ranks(
        &config,
        &split_producer_manifests(),
        &[summary(0, 0, 5, 11), summary(1, 0, 5, 12)],
    )
    .expect("a split multi-publisher topic plans");

    // THE OVERLAP: the bus is PRODUCED on both ranks. Rank 1 also CONSUMES it
    // (`sink` is its member), so on that rank the topic is simultaneously
    // produced and injected — the shape the whole commit exists to serve.
    assert_eq!(
        topics(plan_of(&plans, 0).produced()),
        vec!["/mp/shared_bus"],
        "rank 0 produces the bus (alpha)"
    );
    assert_eq!(
        topics(plan_of(&plans, 1).produced()),
        vec!["/mp/shared_bus"],
        "and so does rank 1 (beta) — `produced` is not exclusive"
    );

    // Rank 0 has no consumer of the bus, so no edge; rank 1's `sink` reads it
    // and names ONLY the FOREIGN producer — its own `beta` publishes live.
    assert!(
        plan_of(&plans, 0).cross_rank_consumed().is_empty(),
        "rank 0 consumes nothing: {:?}",
        plan_of(&plans, 0).cross_rank_consumed()
    );
    assert_eq!(
        plan_of(&plans, 1).cross_rank_consumed(),
        vec![CrossRankEdge {
            topic: "/mp/shared_bus".into(),
            consumer_node: "sink".into(),
            consumer_input: "bus".into(),
            producers: vec![RemoteProducer {
                rank: 0,
                node: "alpha".into(),
            }],
        }],
        "the edge asks the bag for alpha's half and nothing else"
    );
    assert!(
        plan_of(&plans, 1).external_consumed().is_empty(),
        "a topic SOME rank produces is never external"
    );
}

#[test]
fn an_undeclared_topic_split_across_ranks_is_still_refused() {
    // The demotion above is scoped to a DECLARED multi-publisher topic. An
    // undeclared one still has exactly one writer by contract, so two ranks
    // claiming it is a recording that disagrees with its own graph — the
    // anti-tautology control for the demotion (which would otherwise be
    // indistinguishable from deleting the double-producer rule outright).
    let config = cerulion_core::graph::parse_graph(&shared_topic_yaml(false))
        .expect("shared-topic fixture parses");
    let err = plan_ranks(
        &config,
        &split_producer_manifests(),
        &[summary(0, 0, 5, 11), summary(1, 0, 5, 12)],
    )
    .expect_err("an undeclared topic produced from two ranks is refused");
    assert_eq!(
        err,
        RankPlanError::TopicProducedByTwoRanks {
            topic: "/mp/shared_bus".into(),
            first_rank: 0,
            first_node: "alpha".into(),
            second_rank: 1,
            second_node: "beta".into(),
        }
    );
    // The variant alone is not the contract: the refusal must also tell the
    // operator the ONE thing that turns this bag from unreplayable into
    // replayable, which is the opt-in list the demotion above keys on.
    let msg = err.to_string();
    assert!(msg.contains("multi_publisher_topics"), "{msg}");
}

#[test]
fn a_node_writing_one_listed_topic_twice_is_still_one_producer() {
    // A node may write ONE declared multi-publisher topic through TWO outputs:
    // `validate_graph`'s duplicate-output-topic check and
    // `GraphTopology::build`'s double-producer check both EXEMPT a listed
    // topic, and the topology's own producer list is per NODE and deduped for
    // exactly this shape. An output-keyed list here named `alpha` twice, which
    // is not cosmetic — `sole_producer()` answers "has this edge ONE producer
    // to model a pair with", so the duplicate made a single-writer-to-rank-1
    // edge look like a shared bus and SILENTLY declined the block-credit
    // observation the rederive plane would otherwise have made.
    let yaml = "name: shared\n\
                prefix: mp\n\
                multi_publisher_topics: [/mp/shared_bus]\n\
                nodes:\n\
                \x20 - id: alpha\n\
                \x20   type: source_node\n\
                \x20   outputs:\n\
                \x20     - name: out_a\n\
                \x20       schema: geometry_msgs/Vector3\n\
                \x20       topic: /mp/shared_bus\n\
                \x20     - name: out_b\n\
                \x20       schema: geometry_msgs/Vector3\n\
                \x20       topic: /mp/shared_bus\n\
                \x20 - id: sink\n\
                \x20   type: sink_node\n\
                \x20   inputs:\n\
                \x20     - name: bus\n\
                \x20       source: /mp/shared_bus\n";
    let config = cerulion_core::graph::parse_graph(yaml).expect("two-output fixture parses");
    let manifests = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["alpha".into()],
        },
        RankManifest {
            rank: 1,
            node_ids: vec!["sink".into()],
        },
    ];
    let plans = plan_ranks(
        &config,
        &manifests,
        &[summary(0, 0, 5, 11), summary(1, 0, 5, 12)],
    )
    .expect("one node writing a listed topic twice plans fine");

    let edges = plan_of(&plans, 1).cross_rank_consumed();
    assert_eq!(
        edges,
        vec![CrossRankEdge {
            topic: "/mp/shared_bus".into(),
            consumer_node: "sink".into(),
            consumer_input: "bus".into(),
            producers: vec![RemoteProducer {
                rank: 0,
                node: "alpha".into(),
            }],
        }],
        "one WRITER, however many outputs it writes the topic through"
    );
    assert_eq!(
        edges[0].sole_producer(),
        Some(&RemoteProducer {
            rank: 0,
            node: "alpha".into(),
        }),
        "so the edge still has a single producer to model a pair with"
    );
}

#[test]
fn a_co_located_multi_publisher_edge_names_every_producer_not_the_first() {
    // The blame anchor is LABEL-AWARE. Both `alpha` and `beta` sit
    // on rank 0 and both write the bus, so an edge that named only "the first
    // producer in graph order" attributed the whole injected stream to `alpha`
    // — a sentence that is wrong about half the frames.
    let config = cerulion_core::graph::parse_graph(&shared_topic_yaml(true))
        .expect("multi-publisher fixture parses");
    let manifests = vec![
        RankManifest {
            rank: 0,
            node_ids: vec!["alpha".into(), "beta".into()],
        },
        RankManifest {
            rank: 1,
            node_ids: vec!["sink".into()],
        },
    ];
    let plans = plan_ranks(
        &config,
        &manifests,
        &[summary(0, 0, 5, 11), summary(1, 0, 5, 12)],
    )
    .expect("co-located producers plan fine");

    assert_eq!(
        topics(plan_of(&plans, 0).produced()),
        vec!["/mp/shared_bus"]
    );
    let edges = plan_of(&plans, 1).cross_rank_consumed();
    assert_eq!(
        edges,
        vec![CrossRankEdge {
            topic: "/mp/shared_bus".into(),
            consumer_node: "sink".into(),
            consumer_input: "bus".into(),
            producers: vec![
                RemoteProducer {
                    rank: 0,
                    node: "alpha".into(),
                },
                RemoteProducer {
                    rank: 0,
                    node: "beta".into(),
                },
            ],
        }],
        "ONE edge (the read log's own key), naming BOTH writers"
    );
    assert!(
        edges[0].sole_producer().is_none(),
        "and a two-writer edge has no single producer to model a pair with"
    );
}
