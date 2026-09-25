// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph validate` must refuse a graph that `cerulion graph run`
//! REFUSES to build: a network `ingress:` topic consumed by a node whose
//! parsed [`InputMeta::schema_hash`] is the `0` "no declared schema" sentinel.
//!
//! With a config-only preflight, a two-host graph pair of this shape
//! validates clean and then dies at `graph run`
//! with "network ingress topic '<t>' has no consumer with a declared schema".
//!
//! ## Why a raw-FFI fixture reproduces it
//!
//! A macro cdylib emits a per-input `schema_hash` in its info JSON, so its
//! consumers resolve. The `0` sentinel is still reachable from a SHIPPED node
//! form: a raw-FFI cdylib (or any legacy `"inputs":["name"]` strings-shape info
//! JSON) carries no per-input hash at all. `test_node_snapshot_fail_cdylib` is
//! exactly that shape, so it drives the SAME resolver arm and the SAME operator-
//! facing error a real graph hits.
//!
//! ## The defect these tests pin
//!
//! `validate_graph` — the config-only check behind `cerulion graph validate` — reads
//! only `GraphConfig`. The ingress-hash resolution lives in
//! [`compute_gateway_plan`], which needs loaded `NodeInfo`s. So a config-only
//! preflight is structurally incapable of seeing
//! the failure that stops the graph seconds later; the infos-aware
//! `validate_network_ingress` gate is what sees it.
//!
//! Build the fixture first:
//! ```text
//! cargo build -p test_node_snapshot_fail_cdylib
//! ```

use cerulion_core::graph::node::InputMeta;
use cerulion_core::graph::{
    compute_gateway_plan, validate_graph, validate_network_ingress, DylibNodeEntry, GraphConfig,
    GraphTopology, InputDef, NetworkBlock, NetworkMode, NodeDef, NodeEntry, NodeInfo,
};
use cerulion_core::transport::NetworkPosture;
use indexmap::IndexMap;

/// The topic the graph imports from the network and the raw-FFI node consumes.
const INGRESS_TOPIC: &str = "/ext/cmd";

/// The fixture's single declared input, from its legacy `{"inputs":["inp"]}`
/// info JSON — the strings shape that carries no per-input schema hash.
const CONSUMER_INPUT: &str = "inp";

/// One node consuming [`INGRESS_TOPIC`], plus an enabled `network:` block that
/// declares that topic as ingress. This is the minimal shape of a two-host
/// graph: an external topic arriving over the mesh into one wired consumer.
fn ingress_graph() -> GraphConfig {
    ingress_graph_consuming(CONSUMER_INPUT)
}

/// [`ingress_graph`] with the consuming input's NAME as a parameter, so an arm
/// can wire a different fixture whose port is named differently. The name has to
/// match the loaded cdylib's declared port or the resolver would find no
/// metadata for it and fail for an incidental reason.
fn ingress_graph_consuming(input_name: &str) -> GraphConfig {
    GraphConfig {
        execution: None,
        name: None,
        identity: "ingress_preflight".to_string(),
        prefix: "ingress_preflight".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "snapshot_fail".to_string(),
            inputs: vec![InputDef {
                name: input_name.to_string(),
                source: INGRESS_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Vec::new(),
        level_assignments: None,
        network: Some(NetworkBlock {
            mode: NetworkMode::Peer,
            connect: vec![],
            listen: vec![],
            egress: vec![],
            ingress: vec![INGRESS_TOPIC.to_string()],
        }),
    }
}

/// Load the raw-FFI fixture and return its parsed `NodeInfo`, keyed as the
/// graph's `sink` node — the same map `graph run` builds from loaded cdylibs.
fn entry_infos() -> IndexMap<String, NodeInfo> {
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_snapshot_fail_cdylib");
    let node_entry = DylibNodeEntry::load(&path).expect("load raw-FFI fixture");
    let info = node_entry.info().expect("fixture info parses");
    let mut infos = IndexMap::new();
    infos.insert("sink".to_string(), info);
    infos
}

/// MECHANISM: a raw-FFI cdylib's input parses to the `0` sentinel. This is the
/// value the ingress resolver rejects.
#[test]
fn raw_ffi_cdylib_input_parses_to_zero_schema_hash() {
    let infos = entry_infos();
    // The legacy strings-shape info JSON populates BOTH halves: `input_names`
    // AND a SYNTHESIZED `input_meta` (builds one `InputMeta`
    // per cdylib input so the per-input watchdog reaches the runtime across the
    // FFI; its `schema_hash` serde-defaults to the `0` sentinel when the
    // info block omits it, which a raw-FFI block always does). They are not
    // alternatives — asserting both here so a reader cannot conclude that the
    // strings shape leaves `input_meta` empty and that the resolver arm below
    // is therefore unreachable.
    assert!(
        infos["sink"]
            .input_names()
            .iter()
            .any(|n| n == CONSUMER_INPUT),
        "the fixture's legacy `{{\"inputs\":[\"inp\"]}}` info JSON must declare \
         input '{CONSUMER_INPUT}' by name"
    );
    let meta = infos["sink"].input_meta();
    let inp = meta.iter().find(|m| m.name == CONSUMER_INPUT).expect(
        "the cdylib info parser SYNTHESIZES input_meta for the legacy \
             strings shape too, so 'inp' must be present here as well",
    );
    assert_eq!(
        inp.schema_hash, 0,
        "the raw-FFI legacy strings-shape info JSON carries no per-input hash, \
         so the parsed InputMeta must hold the 0 sentinel — if this ever becomes \
         non-zero this test no longer reproduces the preflight defect and must be re-pointed"
    );
}

/// SYMPTOM: the run-time plan refuses the graph with the exact operator-facing
/// error. This arm passes by reproducing that refusal.
#[test]
fn graph_run_refuses_ingress_without_a_declared_consumer_schema() {
    let config = ingress_graph();
    let infos = entry_infos();
    let topology = GraphTopology::build(&config, &infos).expect("topology builds");

    let err = compute_gateway_plan(&config, &topology, &infos, NetworkPosture::Strict)
        .expect_err("the ingress hash cannot be resolved, so the plan must refuse");
    let msg = err.to_string();
    assert!(
        msg.contains("has no consumer with a declared schema"),
        "must reproduce the exact resolver arm, not a neighbouring \
         error; got: {msg}"
    );
    assert!(
        msg.contains(INGRESS_TOPIC),
        "the diagnostic must name the offending topic; got: {msg}"
    );
}

/// THE PIN: preflight must reject what the run refuses.
///
/// Without the check, `validate_graph` returns `Ok` for a graph that cannot
/// start, so a graph can pass every preflight check and then die at
/// `graph run`. A preflight an operator is told to trust must not be structurally
/// blind to the ingress the graph depends on.
#[test]
fn preflight_rejects_an_ingress_consumer_without_a_declared_schema() {
    let config = ingress_graph();
    let infos = entry_infos();

    // The preflight an operator runs: config-only `validate_graph` FIRST (the
    // config half of `cerulion graph validate`), then the infos-aware
    // ingress gate. The first still passes — pinned here so the test keeps
    // proving that the config-only check alone is NOT enough.
    validate_graph(&config).expect("config-only validation still passes — that is the whole point");

    let result = validate_network_ingress(&config, &infos);

    let err = result.expect_err(
        "graph validate must REJECT an ingress topic whose only consumer carries no \
         declared schema — otherwise it passes and `graph run` fails seconds later",
    );
    let msg = err.to_string();
    assert!(
        msg.contains("has no consumer with a declared schema"),
        "preflight must fail with the SAME diagnostic the run emits, so an operator \
         reading it at preflight recognises it; got: {msg}"
    );
}

/// A graph whose ingress consumer DOES declare a schema must pass preflight —
/// the gate has to be a gate, not a blanket refusal of every networked graph.
/// Without this arm, "always `Err`" would satisfy the test above.
#[test]
fn preflight_passes_when_the_ingress_consumer_declares_a_schema() {
    let config = ingress_graph();
    let mut infos = IndexMap::new();
    infos.insert(
        "sink".to_string(),
        NodeInfo::with_meta(
            vec![InputMeta {
                name: CONSUMER_INPUT.to_string(),
                schema_hash: 0xfeed_face_dead_beef,
                trigger: true,
                depth: 1,
                backpressure: Default::default(),
                expect_within_ms: None,
            }],
            vec![],
        ),
    );

    validate_network_ingress(&config, &infos)
        .expect("a consumer carrying a real schema hash resolves cleanly");
}

/// THE NO-INERT-SHIPPING ARM: a REAL macro cdylib's input carries a NONZERO
/// schema hash across the FFI, and the gate lets that graph through.
///
/// The arm above is a pure control — it hand-builds `NodeInfo`, so it proves the
/// gate is not a blanket refusal but says nothing about what a real library
/// exports. That gap matters here more than usual, because refusing EVERY
/// cdylib-loaded consumer is not a hypothetical failure mode: it is exactly what
/// a macro that does not emit a per-input `schema_hash` produces
/// (a HARDCODED `0` there makes the resolver
/// refuse EVERY DylibNodeEntry-loaded consumer). With only the raw-FFI zero
/// fixture on the negative side and a hand-built map on the positive side, a
/// regression re-hardcoding that value would leave this entire file green.
///
/// So this arm loads `test_node_macro_data_trigger_cdylib` — a `#[cerulion_node]`
/// fixture declaring `#[input(trigger)] trigger_in: Vector3` — through the SAME
/// `DylibNodeEntry::load` + `info()` path the raw-FFI arms use, and checks the
/// parsed hash against an EXTERNAL oracle (`Vector3::SCHEMA_HASH`, computed by
/// the codegen recipe) rather than against itself. Asserting merely "nonzero"
/// would pass against a macro emitting an arbitrary constant.
///
/// Needs the fixture built (`cargo build -p test_node_macro_data_trigger_cdylib`);
/// CI's `cargo build --workspace` covers it.
#[test]
fn a_real_macro_cdylib_carries_a_nonzero_hash_across_the_ffi_and_passes_the_gate() {
    use cerulion_core::ShmMessage;
    use native_ros2_messages::geometry_msgs::Vector3;

    const MACRO_INPUT: &str = "trigger_in";

    let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_data_trigger_cdylib");
    let node_entry = DylibNodeEntry::load(&path).expect("load the macro cdylib fixture");
    let info = node_entry.info().expect("fixture info parses");

    let inp = info
        .input_meta()
        .iter()
        .find(|m| m.name == MACRO_INPUT)
        .expect("the macro fixture declares input 'trigger_in'");
    assert_eq!(
        inp.schema_hash,
        Vector3::SCHEMA_HASH,
        "a macro cdylib must carry its input's REAL schema hash across the FFI. \
         Reading the 0 sentinel here — or any other value — means the resolver would refuse \
         every macro-declared consumer, a defect this gate would then \
         report as a config error on every networked graph"
    );

    let config = ingress_graph_consuming(MACRO_INPUT);
    let mut infos = IndexMap::new();
    infos.insert("sink".to_string(), info);

    validate_network_ingress(&config, &infos)
        .expect("a real macro cdylib consumer declares a schema, so preflight must pass it");
}

/// A graph with no `network:` block must not pay for the gate — this is the
/// single-host shape, which has no ingress to resolve and never reaches the
/// resolver arm.
#[test]
fn preflight_is_a_noop_without_a_network_block() {
    let mut config = ingress_graph();
    config.network = None;

    validate_network_ingress(&config, &IndexMap::new())
        .expect("no network block means there is no ingress to gate");
}
