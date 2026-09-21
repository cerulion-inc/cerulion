// SPDX-License-Identifier: AGPL-3.0-only
//! Integration pins for [`schema_serve::build_schema_serving`] —
//! the engine builder that turns a workspace (`.msg` store + workspace YAML +
//! built-in classification) + a graph into the gateway's [`SchemaServing`]. No
//! transport; a tempdir workspace + a parsed graph, asserted against hand oracles.

use std::collections::BTreeMap;
use std::fs;

use cerulion_cli_engine::schema_serve::{build_schema_serving, resolve_bridge_config_path};
use cerulion_core::graph::parse_graph_raw;
use cerulion_core::{SchemaDoc, SchemaEncoding};
use serial_test::serial;
use tracing_test::traced_test;

/// RAII guard for the process-global `DDS_BRIDGE_CONFIG` env var:
/// set it for the test body, restore the prior value on drop — panic-safe. Pair
/// with `#[serial]` (env is process-global).
struct BridgeEnvGuard(Option<std::ffi::OsString>);
impl BridgeEnvGuard {
    fn set(value: &std::path::Path) -> Self {
        let prior = std::env::var_os("DDS_BRIDGE_CONFIG");
        std::env::set_var("DDS_BRIDGE_CONFIG", value);
        Self(prior)
    }
    fn unset() -> Self {
        let prior = std::env::var_os("DDS_BRIDGE_CONFIG");
        std::env::remove_var("DDS_BRIDGE_CONFIG");
        Self(prior)
    }
}
impl Drop for BridgeEnvGuard {
    fn drop(&mut self) {
        match self.0.take() {
            Some(v) => std::env::set_var("DDS_BRIDGE_CONFIG", v),
            None => std::env::remove_var("DDS_BRIDGE_CONFIG"),
        }
    }
}

/// Write a `schemas/<pkg>/msg/<Type>.msg` store file under `root`.
fn write_msg(root: &std::path::Path, pkg: &str, ty: &str, body: &str) {
    let dir = root.join("schemas").join(pkg).join("msg");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join(format!("{ty}.msg")), body).unwrap();
}

fn docs_by_name(docs: &[SchemaDoc]) -> BTreeMap<&str, &SchemaDoc> {
    docs.iter().map(|d| (d.qualified.as_str(), d)).collect()
}

/// A `.msg` store with a type nesting TWO custom types + a BUILT-IN
/// (`std_msgs/Header`) → the served set is the custom types ONLY (Header omitted),
/// and the parent's `deps` name the two custom children (deduped), verbatim text
/// preserved. Hand oracle.
#[test]
fn build_serving_serves_custom_closure_omits_builtins() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    // Widget nests Gadget (twice) + Gizmo + std_msgs/Header (a built-in).
    let widget_body =
        "std_msgs/Header header\nacme_msgs/Gadget primary\nacme_msgs/Gadget secondary\nacme_msgs/Gizmo gizmo\n";
    write_msg(root, "acme_msgs", "Widget", widget_body);
    write_msg(root, "acme_msgs", "Gadget", "int32 count\n");
    write_msg(root, "acme_msgs", "Gizmo", "float64 v\n");

    // A trivial graph (no outputs) — we only pin the schema_docs half here.
    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &root.join("schemas"), None);

    let by = docs_by_name(&serving.schema_docs);
    // Exactly the three CUSTOM types are served — no std_msgs/Header (built-in).
    let mut names: Vec<&str> = by.keys().copied().collect();
    names.sort();
    assert_eq!(
        names,
        vec!["acme_msgs/Gadget", "acme_msgs/Gizmo", "acme_msgs/Widget"]
    );
    // Widget's deps: the two custom children (deduped), NOT the built-in Header.
    let widget = by["acme_msgs/Widget"];
    let mut deps = widget.deps.clone();
    deps.sort();
    assert_eq!(
        deps,
        vec![
            "acme_msgs/Gadget".to_string(),
            "acme_msgs/Gizmo".to_string()
        ]
    );
    // Verbatim text + msg encoding preserved.
    assert_eq!(widget.encoding, SchemaEncoding::Msg);
    assert_eq!(widget.text, widget_body);
    // Leaves have no deps.
    assert!(by["acme_msgs/Gadget"].deps.is_empty());
}

/// A workspace YAML schema is served with encoding `yaml` + its verbatim text;
/// it coexists with `.msg` store types.
#[test]
fn build_serving_includes_workspace_yaml() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap();
    // Workspace YAML schema field format: the KEY is "<type> <name>".
    let yaml_text =
        "schemas:\n  MyState:\n    fields:\n      \"float64 x\":\n      \"float64 y\":\n";
    fs::write(root.join("schemas").join("my_state.yaml"), yaml_text).unwrap();
    write_msg(root, "acme_msgs", "Gadget", "int32 count\n");

    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &root.join("schemas"), None);
    let by = docs_by_name(&serving.schema_docs);
    // The YAML schema is served (bare qualified name) with encoding yaml + text.
    let mystate = by.get("MyState").expect("workspace YAML schema is served");
    assert_eq!(mystate.encoding, SchemaEncoding::Yaml);
    assert_eq!(mystate.text, yaml_text);
    // The .msg store type still serves alongside it.
    assert!(by.contains_key("acme_msgs/Gadget"));
}

/// The topic→schema-name catalog bindings resolve every produced output's
/// declared schema to its RESOLVED absolute topic name — derived (`/prefix/node/
/// out`) and `topic:`-overridden. Hand oracle.
#[test]
fn build_serving_resolves_topic_schema_names() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap();
    // Two producers: one derived-topic output, one absolute `topic:` override.
    let graph = "\
name: g
prefix: robo
nodes:
  - id: cam
    type: cam
    outputs:
      - name: image
        schema: sensor_msgs/Image
  - id: tf
    type: tf
    outputs:
      - name: out
        schema: tf2_msgs/TFMessage
        topic: /tf
";
    let config = parse_graph_raw(graph).unwrap();
    let serving = build_schema_serving(&config, &root.join("schemas"), None);
    let map: BTreeMap<&str, &str> = serving
        .topic_schemas
        .iter()
        .map(|ts| (ts.topic.as_str(), ts.schema_name.as_str()))
        .collect();
    // Derived topic → its schema name.
    assert_eq!(
        map.get("/robo/cam/image").copied(),
        Some("sensor_msgs/Image")
    );
    // Absolute `topic:` override → its schema name.
    assert_eq!(map.get("/tf").copied(), Some("tf2_msgs/TFMessage"));
    assert_eq!(serving.topic_schemas.len(), 2);
}

/// An empty workspace (no store, no YAML) yields NO served docs and NO topic
/// bindings (never a panic, never a fabricated doc), but it STILL
/// hands across the built-in corpus's hash→name bindings: a gateway on a robot
/// with no custom types at all must still be able to name an rmw
/// `std_msgs/String` topic. Asserting `schema_hashes.is_empty()` here would
/// pin the defect itself (catalog `schema_name: None` ⇒ desk
/// `SchemaUnavailable`), so the assertion is the inverse.
#[test]
fn build_serving_empty_workspace_serves_nothing_but_still_names_builtins() {
    use cerulion_core::message::ShmMessage as _;
    let tmp = tempfile::tempdir().unwrap();
    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &tmp.path().join("schemas"), None);
    assert!(serving.schema_docs.is_empty());
    assert!(serving.topic_schemas.is_empty());
    // Exactly the built-in corpus — one binding per vendored message.
    assert_eq!(
        serving.schema_hashes.len(),
        native_ros2_messages::BUILTIN_MSGS.len(),
        "with no custom types the bindings are exactly the built-in corpus"
    );
    let string = serving
        .schema_hashes
        .iter()
        .find(|b| b.qualified == "std_msgs/String")
        .expect("std_msgs/String is named — the rmw shape this pins");
    assert_eq!(
        string.schema_hash,
        native_ros2_messages::std_msgs::String::SCHEMA_HASH,
        "the binding carries the hash an rmw publisher stamps on the wire"
    );
}

/// The serving carries a hash→qualified-name REVERSE
/// binding for every served custom type — each hash EQUAL to the recipe-3 wire
/// hash a producer stamps (`MessageSchema::schema_hash`), so the gateway can NAME
/// a runtime-registered raw route (a `ros2 attach` topic, absent from
/// `topic_schemas`) by ITS reg-channel hash. Hand oracle (a flat custom type —
/// its wire hash is resolution-independent, so `parse_rosmsg` alone is the
/// oracle).
///
/// The built-in bindings are appended AFTER the customs. The order
/// is load-bearing for netd's `hash_for_topic` (name→hash by FIRST match): a
/// workspace type that SHADOWS a built-in's name must keep resolving to its OWN
/// hash, so the custom binding must come first. Pinned by asserting the custom
/// binding is at index 0 and the built-ins follow.
#[test]
fn build_serving_populates_hash_reverse_bindings_customs_first_then_builtins() {
    use cerulion_core::codegen::parse_rosmsg;
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let gadget_body = "int32 count\nfloat64 v\n";
    write_msg(root, "acme_msgs", "Gadget", gadget_body);

    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &root.join("schemas"), None);

    // The single custom type FIRST, then exactly the built-in corpus.
    assert_eq!(
        serving.schema_hashes.len(),
        1 + native_ros2_messages::BUILTIN_MSGS.len()
    );
    let binding = &serving.schema_hashes[0];
    assert_eq!(binding.qualified, "acme_msgs/Gadget");
    // The hash EQUALS the wire hash the producer stamps.
    let want_hash = parse_rosmsg(gadget_body, "Gadget", Some("acme_msgs"))
        .unwrap()
        .schema_hash();
    assert_eq!(binding.schema_hash, want_hash);
    // The built-in slice follows (a built-in is never at index 0).
    assert!(
        serving.schema_hashes[1..]
            .iter()
            .any(|b| b.qualified == "std_msgs/String"),
        "the built-in corpus follows the custom bindings"
    );
}

/// On the serving seam the ceiling is
/// judged on the RESOLVED bindings only. `Huge` is over the ceiling as
/// declared, `Parent` composes over through it. With a declared-first
/// retain `Huge` would leave the set BEFORE composition, `Parent` would resolve with
/// `h` still variable (a small frame) and BIND a hash codegen never emits —
/// a name the gateway would then serve a topic under. Both must be absent;
/// the clean sibling still binds first.
#[test]
fn serving_binds_no_hash_for_a_parent_composed_over_the_ceiling() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    write_msg(root, "acme_msgs", "Gadget", "int32 count\nfloat64 v\n");
    write_msg(
        root,
        "acme_msgs",
        "Huge",
        &format!("uint8[{}] a\n", u32::MAX),
    );
    write_msg(root, "acme_msgs", "Parent", "Huge h\nfloat64 x\n");

    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &root.join("schemas"), None);

    let customs: Vec<&str> = serving
        .schema_hashes
        .iter()
        .map(|b| b.qualified.as_str())
        .filter(|q| q.starts_with("acme_msgs/"))
        .collect();
    assert_eq!(
        customs,
        vec!["acme_msgs/Gadget"],
        "only the representable custom binds — never a parent whose frame crosses the \
         ceiling through inlining, nor its target"
    );
    assert_eq!(
        serving.schema_hashes.len(),
        1 + native_ros2_messages::BUILTIN_MSGS.len()
    );
}

/// A YAML entry spelled `acme::Thing` is served under
/// the ONE canonical identity `acme/Thing` — the key a declared output
/// normalizes to — for BOTH the served document and the hash binding, so a
/// topic declared `schema: acme::Thing` points at a doc and a binding that
/// exist.
#[test]
fn serving_keys_a_double_colon_yaml_entry_by_its_canonical_identity() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let dir = root.join("schemas");
    fs::create_dir_all(&dir).unwrap();
    fs::write(
        dir.join("thing.yaml"),
        "schemas:\n  acme::Thing:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();
    let graph = "\
name: g
prefix: robo
nodes:
  - id: n
    type: n
    outputs:
      - name: out
        schema: acme::Thing
";
    let mut config = parse_graph_raw(graph).unwrap();
    // Note: `parse_graph_raw` canonicalizes `schema:` at
    // deserialize time, so the serving code would otherwise only ever see
    // `acme/Thing`; hand it the `::` spelling back so THIS test exercises
    // the serving-side normalization (deleting that call fails this arm).
    config.nodes[0].outputs[0].schema = "acme::Thing".to_string();
    let serving = build_schema_serving(&config, &dir, None);

    let docs = docs_by_name(&serving.schema_docs);
    assert!(
        docs.contains_key("acme/Thing"),
        "doc keyed canonically: {:?}",
        docs.keys()
    );
    assert!(
        !docs.contains_key("acme::Thing"),
        "never under the raw spelling"
    );
    assert!(
        serving
            .schema_hashes
            .iter()
            .any(|b| b.qualified == "acme/Thing"),
        "hash binding keyed canonically"
    );
    let bound = serving
        .topic_schemas
        .iter()
        .find(|ts| ts.topic == "/robo/n/out")
        .map(|ts| ts.schema_name.as_str());
    assert_eq!(
        bound,
        Some("acme/Thing"),
        "the declared output binds the served key"
    );
}

/// A `pkg::Type` / `pkg/Type` twin pair is REFUSED at the claims
/// (one identity, two declarers), so serving serves NEITHER document under
/// the canonical key — the same verdict validation gives it — instead of
/// last-wins-overwriting one with the other.
#[test]
fn serving_refuses_a_double_colon_slash_twin_pair_under_the_canonical_key() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = tmp.path().join("schemas");
    fs::create_dir_all(&dir).unwrap();
    fs::write(
        dir.join("a_twin.yaml"),
        "schemas:\n  acme::Twin:\n    fields:\n      uint32 a:\n",
    )
    .unwrap();
    fs::write(
        dir.join("b_twin.yaml"),
        "schemas:\n  acme/Twin:\n    fields:\n      uint32 b:\n",
    )
    .unwrap();
    let config = parse_graph_raw("name: g\nprefix: robo\nnodes: []\n").unwrap();
    let serving = build_schema_serving(&config, &dir, None);
    let docs = docs_by_name(&serving.schema_docs);
    assert!(
        !docs.contains_key("acme/Twin") && !docs.contains_key("acme::Twin"),
        "a refused identity is served under NO spelling: {:?}",
        docs.keys()
    );
    assert!(
        !serving
            .schema_hashes
            .iter()
            .any(|b| b.qualified.starts_with("acme")),
        "no hash binding for a refused identity"
    );
}

// ─────────────────────────── Bridge-config fold ────────────────────

/// Map topic → schema_name over a serving's `topic_schemas`.
fn topic_map(serving: &cerulion_core::SchemaServing) -> BTreeMap<&str, &str> {
    serving
        .topic_schemas
        .iter()
        .map(|ts| (ts.topic.as_str(), ts.schema_name.as_str()))
        .collect()
}

/// THE headline: a `ros2 attach` graph declares only the
/// bridge's fixed TYPED ports, but the bridged RAW topics live in the bridge
/// config. Folding the bridge yaml NAMES every mapped topic — the raw route the
/// graph never declares (`/go2/utlidar/robot_odom` on a Go2) — and
/// serves the custom type's `.msg` closure so a schema-less remote attach can
/// decode it. Precedence: the GRAPH wins a topic collision.
#[test]
fn build_serving_folds_bridge_mappings_and_names_raw_routes() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    // The bridged custom type — materialized into the workspace store by attach.
    let widget_body = "int32 id\nfloat64 value\n";
    write_msg(root, "acme_msgs", "Widget", widget_body);

    // The lean attach graph: ONE typed port (topic override == the bridge's
    // cerulion_topic). Raw routes have NO graph port.
    let graph = "\
name: attach
prefix: go2
nodes:
  - id: dds_bridge
    type: dds_bridge
    outputs:
      - name: cloud
        schema: sensor_msgs/PointCloud2
        topic: /go2/cloud
";
    let config = parse_graph_raw(graph).unwrap();

    // The bridge config: the typed mapping COLLIDES with the graph's /go2/cloud
    // but declares a DIFFERENT type (sensor_msgs/LaserScan vs the graph's
    // sensor_msgs/PointCloud2) — so the graph-wins assertion below actually
    // BITES: a precedence inversion (bridge overriding graph) would resolve
    // /go2/cloud to LaserScan and fail. Plus two RAW routes (a custom type + a
    // built-in) the graph never declares.
    let bridge = "\
# Generated by `cerulion ros2 attach`.
domain_id: 0
only_networks: []
msg_dirs: [../schemas]
mappings:
  - dds_topic: /utlidar/cloud
    ros_type: sensor_msgs/LaserScan
    cerulion_topic: /go2/cloud
    qos: best_effort
  - dds_topic: /utlidar/robot_odom
    ros_type: unitree_go/SportModeState
    cerulion_topic: /go2/utlidar/robot_odom
    qos: best_effort
    route: raw
  - dds_topic: /widget
    ros_type: acme_msgs/Widget
    cerulion_topic: /go2/widget
    qos: best_effort
";
    let bridge_path = root.join("graphs").join("attach.bridge.yaml");
    fs::create_dir_all(bridge_path.parent().unwrap()).unwrap();
    fs::write(&bridge_path, bridge).unwrap();

    let serving = build_schema_serving(&config, &root.join("schemas"), Some(&bridge_path));
    let map = topic_map(&serving);

    // The raw routes the graph never declared are now NAMED.
    assert_eq!(
        map.get("/go2/utlidar/robot_odom").copied(),
        Some("unitree_go/SportModeState")
    );
    assert_eq!(map.get("/go2/widget").copied(), Some("acme_msgs/Widget"));
    // Collision: the GRAPH's declared schema (PointCloud2) wins over the bridge's
    // DIFFERENT type (LaserScan) — an inversion would resolve to LaserScan and
    // fail this assert — and the topic appears EXACTLY once (no duplicate binding).
    assert_eq!(
        map.get("/go2/cloud").copied(),
        Some("sensor_msgs/PointCloud2")
    );
    assert_ne!(
        map.get("/go2/cloud").copied(),
        Some("sensor_msgs/LaserScan"),
        "the bridge's colliding type must NOT override the graph binding"
    );
    assert_eq!(
        serving
            .topic_schemas
            .iter()
            .filter(|ts| ts.topic == "/go2/cloud")
            .count(),
        1
    );
    // The custom bridged type's `.msg` closure is servable (so the desk can fetch
    // + decode it); the built-in raw type (SportModeState is not in the store) is
    // NOT re-served (the desk has built-ins) — served set is the custom Widget.
    let by = docs_by_name(&serving.schema_docs);
    let widget = by
        .get("acme_msgs/Widget")
        .expect("bridged custom type served");
    assert_eq!(widget.text, widget_body);
    assert_eq!(widget.encoding, SchemaEncoding::Msg);
}

/// GENERAL msg-store fold: a bridged custom type whose `.msg` lives ONLY in a
/// store dir named by the bridge's `msg_dirs` (NOT the workspace `schemas/`) is
/// still served — the fold loads the bridge's own store dirs (relative to the
/// config file), not just `schemas/`.
#[test]
fn build_serving_folds_bridge_msg_dirs_store() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap(); // empty workspace store
                                                       // A SEPARATE store dir (not schemas/) holds the bridged type.
    let alt = root.join("alt_store");
    let dir = alt.join("acme_msgs").join("msg");
    fs::create_dir_all(&dir).unwrap();
    let widget_body = "int32 id\n";
    fs::write(dir.join("Widget.msg"), widget_body).unwrap();

    let config = parse_graph_raw("name: attach\nprefix: go2\nnodes: []\n").unwrap();
    // The bridge's msg_dirs points at the ALT store, RELATIVE to the config dir.
    let bridge = "\
msg_dirs: [../alt_store]
mappings:
  - dds_topic: /widget
    ros_type: acme_msgs/Widget
    cerulion_topic: /go2/widget
";
    let bridge_path = root.join("graphs").join("attach.bridge.yaml");
    fs::create_dir_all(bridge_path.parent().unwrap()).unwrap();
    fs::write(&bridge_path, bridge).unwrap();

    let serving = build_schema_serving(&config, &root.join("schemas"), Some(&bridge_path));
    // The type from the bridge's OWN store dir is served + named.
    let by = docs_by_name(&serving.schema_docs);
    assert_eq!(
        by.get("acme_msgs/Widget").map(|d| d.text.as_str()),
        Some(widget_body)
    );
    let map = topic_map(&serving);
    assert_eq!(map.get("/go2/widget").copied(), Some("acme_msgs/Widget"));
}

/// Graceful degradation: a bridge path pointing at a MALFORMED/absent config is a
/// best-effort no-op — the serving degrades to graph-only, BYTE-IDENTICAL to the
/// no-bridge (`None`) serving. A robot with no bridge config is unchanged.
#[test]
fn build_serving_malformed_bridge_degrades_to_graph_only() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap();
    let graph = "\
name: attach
prefix: go2
nodes:
  - id: cam
    type: cam
    outputs:
      - name: image
        schema: sensor_msgs/Image
";
    let config = parse_graph_raw(graph).unwrap();
    let no_bridge = build_schema_serving(&config, &root.join("schemas"), None);

    // A malformed bridge yaml → the fold is a no-op (loud warn, best-effort).
    let bridge_path = root.join("graphs").join("attach.bridge.yaml");
    fs::create_dir_all(bridge_path.parent().unwrap()).unwrap();
    fs::write(&bridge_path, "mappings: [ : broken : yaml\n").unwrap();
    let malformed = build_schema_serving(&config, &root.join("schemas"), Some(&bridge_path));

    // An absent bridge file → also a no-op.
    let absent = build_schema_serving(
        &config,
        &root.join("schemas"),
        Some(&root.join("graphs").join("nope.bridge.yaml")),
    );

    assert_eq!(malformed.topic_schemas, no_bridge.topic_schemas);
    assert_eq!(absent.topic_schemas, no_bridge.topic_schemas);
    // Only the graph-declared topic is named in every case.
    assert_eq!(no_bridge.topic_schemas.len(), 1);
    assert_eq!(no_bridge.topic_schemas[0].topic, "/go2/cam/image");
}

/// A hand-edited mapping with an EMPTY `ros_type` OR an empty
/// `cerulion_topic` is SKIPPED (never folds a phantom empty-named catalog binding
/// the graph path would refuse) with a loud `warn!` — both empty arms. `#[traced_test]`
/// asserts the warn fires (loud, not silent); the behavioral pin is that only the
/// valid mapping's topic is named.
#[test]
#[traced_test]
fn build_serving_skips_empty_field_bridge_mappings_loudly() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap();
    let config = parse_graph_raw("name: attach\nprefix: go2\nnodes: []\n").unwrap();

    // Mapping 0: empty ros_type. Mapping 1: empty cerulion_topic. Mapping 2: valid.
    let bridge = "\
mappings:
  - dds_topic: /a
    ros_type: \"\"
    cerulion_topic: /go2/a
  - dds_topic: /b
    ros_type: geometry_msgs/Twist
    cerulion_topic: \"\"
  - dds_topic: /c
    ros_type: sensor_msgs/Imu
    cerulion_topic: /go2/c
";
    let bridge_path = root.join("graphs").join("attach.bridge.yaml");
    fs::create_dir_all(bridge_path.parent().unwrap()).unwrap();
    fs::write(&bridge_path, bridge).unwrap();

    let serving = build_schema_serving(&config, &root.join("schemas"), Some(&bridge_path));
    let map = topic_map(&serving);
    // Only the valid mapping is named; neither empty arm folded a binding.
    assert_eq!(serving.topic_schemas.len(), 1);
    assert_eq!(map.get("/go2/c").copied(), Some("sensor_msgs/Imu"));
    assert_eq!(map.get("/go2/a"), None);
    // The empty cerulion_topic → no "" topic key anywhere.
    assert!(serving.topic_schemas.iter().all(|ts| !ts.topic.is_empty()));
    assert!(serving
        .topic_schemas
        .iter()
        .all(|ts| !ts.schema_name.is_empty()));
    // Loud, not silent: the skip warns naming the file (both empty arms hit it).
    assert!(logs_contain(
        "bridge mapping has an empty cerulion_topic or ros_type"
    ));
}

/// The WORKSPACE store WINS a `pkg/Type` collision with a
/// bridge `msg_dirs` store — the SAME qualified type in BOTH stores with
/// DIFFERENT text serves the WORKSPACE text (the fold fills gaps, never
/// overrides). A flipped `overwrite` bool would ship the bridge text here and
/// fail this hand oracle.
#[test]
fn build_serving_workspace_store_wins_bridge_msg_dir_collision() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    // The WORKSPACE store's Widget (the authoritative text).
    let ws_body = "int32 workspace_id\nfloat64 value\n";
    write_msg(root, "acme_msgs", "Widget", ws_body);
    // The bridge's OWN store dir defines the SAME type with DIFFERENT text.
    let bridge_body = "int32 bridge_id\n";
    let alt = root.join("alt_store");
    let dir = alt.join("acme_msgs").join("msg");
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join("Widget.msg"), bridge_body).unwrap();
    assert_ne!(ws_body, bridge_body, "the two stores must differ");

    let config = parse_graph_raw("name: attach\nprefix: go2\nnodes: []\n").unwrap();
    let bridge = "\
msg_dirs: [../alt_store]
mappings:
  - dds_topic: /widget
    ros_type: acme_msgs/Widget
    cerulion_topic: /go2/widget
";
    let bridge_path = root.join("graphs").join("attach.bridge.yaml");
    fs::create_dir_all(bridge_path.parent().unwrap()).unwrap();
    fs::write(&bridge_path, bridge).unwrap();

    let serving = build_schema_serving(&config, &root.join("schemas"), Some(&bridge_path));
    let by = docs_by_name(&serving.schema_docs);
    let widget = by.get("acme_msgs/Widget").expect("Widget served");
    // The WORKSPACE text serves — the bridge store did NOT override it.
    assert_eq!(widget.text, ws_body);
    assert_ne!(widget.text, bridge_body);
}

/// The PRODUCTION discovery path (`resolve_bridge_config_path`
/// reading `DDS_BRIDGE_CONFIG` via `std::env`) — env WINS over a real sibling.
/// Set the env to a config whose mappings DIFFER from a real
/// `attach.bridge.yaml` sibling; assert the ENV file's mappings fold and the
/// sibling's do NOT. `#[serial]` + RAII guard (process-global env).
#[test]
#[serial]
fn resolve_bridge_config_env_wins_over_sibling_through_production_path() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    fs::create_dir_all(root.join("schemas")).unwrap();
    let graphs = root.join("graphs");
    fs::create_dir_all(&graphs).unwrap();

    // The sibling `graphs/attach.bridge.yaml` maps /sibling/topic.
    fs::write(
        graphs.join("attach.bridge.yaml"),
        "mappings:\n  - dds_topic: /s\n    ros_type: sensor_msgs/Imu\n    \
         cerulion_topic: /sibling/topic\n",
    )
    .unwrap();
    // The ENV file (elsewhere) maps a DIFFERENT topic.
    let env_cfg = root.join("elsewhere.bridge.yaml");
    fs::write(
        &env_cfg,
        "mappings:\n  - dds_topic: /e\n    ros_type: geometry_msgs/Twist\n    \
         cerulion_topic: /env/topic\n",
    )
    .unwrap();

    let config = parse_graph_raw("name: attach\nprefix: go2\nnodes: []\n").unwrap();

    // ENV SET → env wins over the sibling.
    {
        let _guard = BridgeEnvGuard::set(&env_cfg);
        let path = resolve_bridge_config_path(&graphs, "attach");
        assert_eq!(path.as_deref(), Some(env_cfg.as_path()));
        let serving = build_schema_serving(&config, &root.join("schemas"), path.as_deref());
        let map = topic_map(&serving);
        assert_eq!(map.get("/env/topic").copied(), Some("geometry_msgs/Twist"));
        assert_eq!(map.get("/sibling/topic"), None, "env wins; sibling ignored");
    }

    // ENV UNSET → the sibling convention is used.
    {
        let _guard = BridgeEnvGuard::unset();
        let path = resolve_bridge_config_path(&graphs, "attach");
        assert_eq!(
            path.as_deref(),
            Some(graphs.join("attach.bridge.yaml").as_path())
        );
        let serving = build_schema_serving(&config, &root.join("schemas"), path.as_deref());
        let map = topic_map(&serving);
        assert_eq!(map.get("/sibling/topic").copied(), Some("sensor_msgs/Imu"));
        assert_eq!(map.get("/env/topic"), None);

        // A graph with NO sibling and no env ⇒ None (a normal graph, unchanged).
        assert_eq!(resolve_bridge_config_path(&graphs, "no_such_graph"), None);
    }
}
