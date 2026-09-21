// SPDX-License-Identifier: AGPL-3.0-only
//! Pins the tutorial's documented graph YAML against future drift.
//!
//! The Getting Started tutorial at
//! `docs/tutorials/01-getting-started.md` Step 5 documents a complete
//! `obstacle_avoidance.yaml` graph file. Users copy that YAML
//! verbatim and run it. This test loads the same YAML through
//! `parse_graph_raw` + `validate_graph` and pins structural
//! invariants — node ids, port wiring, schema names, slice-length —
//! so any future change to the tutorial's YAML (intended or
//! accidental drift) surfaces as a test failure rather than as a
//! broken first-contact tutorial.
//!
//! ## Why this exists
//!
//! Before the trigger-policy move the tutorial used YAML `policy:` blocks. That change
//! removed `NodeDef::policy` from `cerulion_core::graph::config`;
//! `warn_on_legacy_policy_block` now emits a `tracing::warn!` per
//! reintroduced `policy:` block. The tutorial was updated to drop
//! those blocks and macroify the
//! trigger-policies table. This test guards against a
//! future regression that reintroduces a `policy:` block, swaps a
//! schema name, breaks an input source reference, or otherwise
//! silently rots the YAML.
//!
//! ## Snapshot strategy (oracle vector — NOT self-comparison)
//!
//! `TUTORIAL_GRAPH_YAML` is a hand-pasted copy of the tutorial's
//! YAML, intentionally NOT re-read from the markdown. A doc-side
//! typo would parse the same way on both sides if we extracted it
//! dynamically — that is the self-comparison anti-pattern a review
//! flagged on an earlier issue. Instead, this test compares the parsed
//! `GraphConfig` against an oracle vector of structural assertions
//! (specific node ids, specific source strings, specific schema
//! names) — independent of the parser. A doc-side change that
//! corrupts the YAML structure fails the parse; a doc-side rename
//! fails the oracle assertion. Both surface loudly.
//!
//! When the tutorial's YAML is intentionally updated, this constant
//! must be updated in lockstep. The test failure message is the
//! reminder.

use cerulion_core::error::TransportError;
use cerulion_core::graph::{parse_graph_raw, validate_graph};
use tracing_test::traced_test;

/// Exact snapshot of `obstacle_avoidance.yaml` from
/// `docs/tutorials/01-getting-started.md` Step 5. Hand-maintained;
/// do not auto-derive from the markdown (see module doc-comment).
const TUTORIAL_GRAPH_YAML: &str = r#"prefix: demo
nodes:
  # LIDAR sensor: periodic at 10 Hz — policy declared on the macro side
  # by `#[cerulion_node(period_ms = 100)]` in Step 3a.
  - id: lidar_sensor
    type: lidar_sensor
    outputs:
      - name: scan
        schema: sensor_msgs/LaserScan
        max_slice_len: 8192

  # Safety controller: data-triggered — policy declared on the macro
  # side by `#[input(trigger)] scan: LaserScan` in Step 3b.
  - id: safety_controller
    type: safety_controller
    inputs:
      - name: scan
        source: lidar_sensor/scan
    outputs:
      - name: cmd_vel
        schema: geometry_msgs/Vector3

  # Drive base: data-triggered — policy declared on the macro side by
  # `#[input(trigger)] cmd_vel: Vector3` in Step 3c.
  - id: drive_base
    type: drive_base
    inputs:
      - name: cmd_vel
        source: safety_controller/cmd_vel
"#;

const WARN_PHRASE: &str = "carries a `policy:` block";

// ============================================================================
// Happy path: tutorial YAML parses, validates, and matches the oracle vector.
// ============================================================================

#[test]
#[traced_test]
fn tutorial_yaml_parses_validates_and_matches_oracle() {
    let config = parse_graph_raw(TUTORIAL_GRAPH_YAML).expect("tutorial YAML must parse");
    validate_graph(&config).expect("tutorial YAML must validate against the topology checker");

    // ---- Graph-level oracle ----
    // The tutorial teaches a graph named by its file
    // (`graphs/obstacle_avoidance.yaml`), so its YAML carries no `name:` key at
    // all — and a stemless parse of one has nothing to name itself with. The
    // CLI supplies the identity at load; see `graph::adopt_file_stem_identity`.
    assert!(
        config.name.is_none(),
        "the tutorial must not teach a `name:` key"
    );
    assert_eq!(config.identity(), cerulion_core::graph::UNNAMED_GRAPH);
    assert_eq!(
        config.prefix, "demo",
        "tutorial documents an explicit `prefix: demo`; parse_graph_raw must preserve it"
    );
    assert_eq!(
        config.nodes.len(),
        3,
        "tutorial pipeline has exactly 3 nodes: lidar_sensor -> safety_controller -> drive_base"
    );

    let ids: Vec<&str> = config.nodes.iter().map(|n| n.id.as_str()).collect();
    assert_eq!(
        ids,
        vec!["lidar_sensor", "safety_controller", "drive_base"],
        "node order in YAML matters (GraphConfig::nodes is Vec<NodeDef>; serde_yaml preserves document order = scheduling order)"
    );

    // ---- lidar_sensor: source-only, 0 inputs, 1 output ----
    let lidar = &config.nodes[0];
    assert_eq!(lidar.node_type, "lidar_sensor");
    assert!(
        lidar.inputs.is_empty(),
        "lidar_sensor is the pipeline source — must have no inputs"
    );
    assert_eq!(lidar.outputs.len(), 1);
    let lidar_scan = &lidar.outputs[0];
    assert_eq!(lidar_scan.name, "scan");
    assert_eq!(
        lidar_scan.schema, "sensor_msgs/LaserScan",
        "LaserScan schema is load-bearing — Step 3a tick body uses LaserScan fixed-field accessors"
    );
    assert_eq!(
        lidar_scan.max_slice_len,
        Some(8192),
        "tutorial's 8192-byte cap is sized for ~314 rays * 4B + headroom — see Step 5 callout"
    );

    // ---- safety_controller: 1 input, 1 output ----
    let safety = &config.nodes[1];
    assert_eq!(safety.node_type, "safety_controller");
    assert_eq!(safety.inputs.len(), 1);
    assert_eq!(safety.outputs.len(), 1);
    let safety_in = &safety.inputs[0];
    assert_eq!(safety_in.name, "scan");
    assert_eq!(
        safety_in.source, "lidar_sensor/scan",
        "tutorial wires safety_controller.scan to lidar_sensor/scan (the only legal source)"
    );
    let safety_out = &safety.outputs[0];
    assert_eq!(safety_out.name, "cmd_vel");
    assert_eq!(
        safety_out.schema, "geometry_msgs/Vector3",
        "Vector3 (not Twist) is documented — Twist's empty FixedSection breaks direct field writes"
    );
    assert_eq!(
        safety_out.max_slice_len, None,
        "Vector3 is fixed-only; tutorial intentionally omits max_slice_len (tier-2 codegen const wins)"
    );

    // ---- drive_base: sink, 1 input, 0 outputs ----
    let drive = &config.nodes[2];
    assert_eq!(drive.node_type, "drive_base");
    assert_eq!(drive.inputs.len(), 1);
    assert!(
        drive.outputs.is_empty(),
        "drive_base is the pipeline sink — must have no outputs"
    );
    let drive_in = &drive.inputs[0];
    assert_eq!(drive_in.name, "cmd_vel");
    assert_eq!(
        drive_in.source, "safety_controller/cmd_vel",
        "tutorial wires drive_base.cmd_vel to safety_controller/cmd_vel"
    );

    // ---- Negative log assertion: tutorial YAML must NOT trip the
    // legacy-policy warn. If a future doc editor accidentally lands a
    // `policy:` line, this fails.
    assert!(
        !logs_contain(WARN_PHRASE),
        "tutorial YAML must not contain a `policy:` block (trigger \
         policy lives on the macro side)"
    );
}

// ============================================================================
// Adversarial: reintroduced `policy:` block fires the warn AND is refused.
// ============================================================================

#[test]
#[traced_test]
fn reintroduced_policy_block_fires_legacy_warn_and_is_refused() {
    // Splice a `policy:` block back into the lidar_sensor node — the
    // exact regression we are guarding against.
    let yaml_with_legacy_policy = TUTORIAL_GRAPH_YAML.replace(
        "  - id: lidar_sensor\n    type: lidar_sensor\n    outputs:",
        "  - id: lidar_sensor\n    type: lidar_sensor\n    policy:\n      period_ms: 100\n    outputs:",
    );
    // Sanity: the spliced text must appear with its full distinctive
    // signature. Checking only for `policy:` would silently re-test the
    // original YAML if a future doc-side comment happened to contain
    // the substring `policy:` (e.g. a comment like
    // "# policy: lives on the macro side") — that has the same
    // silent-failure shape as the F11 anti-pattern. Match the exact
    // splice byte-sequence including the `period_ms: 100` value to
    // anchor on the unique post-splice form.
    assert!(
        yaml_with_legacy_policy.contains("    policy:\n      period_ms: 100\n    outputs:"),
        "test fixture builder failed to inject the `policy:` block in the expected form"
    );

    // Since the CI-hardening pass every graph-YAML type carries
    // `#[serde(deny_unknown_fields)]`, so a resurrected `policy:` block is a
    // hard parse error instead of a silently dropped trigger schedule. Both
    // halves are asserted: the serde error names the offending key AND the
    // accepted set (the fix), and the legacy-policy warn fires first to say
    // where the setting moved (serde cannot know that).
    let err = parse_graph_raw(&yaml_with_legacy_policy)
        .expect_err("a legacy `policy:` block must be REFUSED, not silently dropped");
    let reason = match err {
        TransportError::GraphParseError { reason } => reason,
        other => panic!("expected GraphParseError, got: {other:?}"),
    };
    assert!(
        reason.contains("unknown field `policy`"),
        "the parse error must name the offending key; got: {reason}"
    );
    assert!(
        reason.contains("expected one of")
            && reason.contains("`id`")
            && reason.contains("`type`")
            && reason.contains("`inputs`")
            && reason.contains("`outputs`"),
        "the parse error must list the accepted node keys (the fix); got: {reason}"
    );

    assert!(
        logs_contain(WARN_PHRASE),
        "warn_on_legacy_policy_block must fire when a `policy:` block reappears"
    );
    // Anchored on the CAUSE + REMEDY the operator acts on, never on an
    // internal tracker id: the warn must say where trigger policy now lives
    // and what to do with the stale block.
    // Matched WITHOUT the leading word: the message opens that sentence with a
    // capital `Trigger`, and `logs_contain` is case-SENSITIVE, so anchoring on
    // the lowercase form silently depends on where the sentence break falls.
    assert!(
        logs_contain("policy lives on the macro side only"),
        "warn message must name WHERE trigger policy lives now"
    );
    assert!(
        logs_contain("delete the YAML `policy:` block"),
        "warn message must state the remedy"
    );
}

// ============================================================================
// Adversarial: wrong input source string fails validation.
// ============================================================================

#[test]
fn wrong_input_source_fails_validation() {
    let yaml_with_wrong_source = TUTORIAL_GRAPH_YAML.replace(
        "        source: lidar_sensor/scan",
        "        source: lidar_sensor/wrong_topic_name",
    );
    assert!(
        yaml_with_wrong_source.contains("wrong_topic_name"),
        "test fixture builder failed to break the source string"
    );

    let config =
        parse_graph_raw(&yaml_with_wrong_source).expect("parse succeeds — string is well-formed");
    let err = validate_graph(&config).expect_err("validate must reject the dangling source");
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(
        reason.contains("non-existent source"),
        "validator must reject with `non-existent source`; got: {reason}"
    );
    assert!(
        reason.contains("safety_controller"),
        "error must name the offending node id (safety_controller); got: {reason}"
    );
    assert!(
        reason.contains("lidar_sensor/wrong_topic_name"),
        "error must name the malformed source string so users see what to fix; got: {reason}"
    );
}

// ============================================================================
// Adversarial: duplicate node id fails validation.
// ============================================================================

#[test]
fn duplicate_node_id_fails_validation() {
    // Rename drive_base → safety_controller, creating two nodes with
    // the same id. The duplicate-id check fires before any
    // source-resolution work, so we land on a clean GraphError.
    let yaml_with_dup_id = TUTORIAL_GRAPH_YAML.replace(
        "  - id: drive_base\n    type: drive_base",
        "  - id: safety_controller\n    type: drive_base",
    );
    assert_eq!(
        yaml_with_dup_id.matches("- id: safety_controller").count(),
        2,
        "test fixture builder must produce exactly 2 nodes with id `safety_controller`"
    );

    let config = parse_graph_raw(&yaml_with_dup_id).expect("parse succeeds");
    let err = validate_graph(&config).expect_err("validate must reject duplicate id");
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(
        reason.contains("duplicate node ID"),
        "validator must reject with `duplicate node ID`; got: {reason}"
    );
    assert!(
        reason.contains("safety_controller"),
        "error must name the duplicated id; got: {reason}"
    );
}

// ============================================================================
// Adversarial: max_slice_len below WireHeader::SIZE fails validation.
// ============================================================================

#[test]
fn undersized_max_slice_len_fails_validation() {
    // Tutorial documents max_slice_len: 8192 for LaserScan. If a
    // future edit drops the value below WireHeader::SIZE (32B), the
    // validator must reject — the WireHeader alone wouldn't fit.
    let yaml_with_small_slice =
        TUTORIAL_GRAPH_YAML.replace("        max_slice_len: 8192", "        max_slice_len: 8");
    assert!(
        yaml_with_small_slice.contains("max_slice_len: 8\n"),
        "test fixture builder failed to set max_slice_len to 8"
    );

    let config = parse_graph_raw(&yaml_with_small_slice).expect("parse succeeds");
    let err = validate_graph(&config).expect_err("validate must reject undersized slice");
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(
        reason.contains("max_slice_len"),
        "validator must reject with `max_slice_len`; got: {reason}"
    );
    // Pin the actionable numerics so a future error-message reword
    // can't silently drop the diagnostic that tells the user what to
    // fix. Validator format (see validation.rs:81-87): "node 'lidar_sensor'
    // output 'scan' max_slice_len (8) must be > WireHeader::SIZE (32)".
    assert!(
        reason.contains("(8)"),
        "error must report the bad value `(8)` for actionable diagnostics; got: {reason}"
    );
    assert!(
        reason.contains("(32)"),
        "error must report the WireHeader::SIZE threshold `(32)` so users see the rule; got: {reason}"
    );
    assert!(
        reason.contains("lidar_sensor"),
        "error must name the node `lidar_sensor` so users find the offending output; got: {reason}"
    );
    assert!(
        reason.contains("scan"),
        "error must name the output `scan` so users find the offending port; got: {reason}"
    );
}

// ============================================================================
// Adversarial: schema separator normalization (`::` → `/`).
//
// `cerulion_core::graph::config::OutputDef` deserializes the
// `schema:` field through `normalize_schema_separator`, which
// replaces `::` with `/` so `sensor_msgs::LaserScan` and
// `sensor_msgs/LaserScan` round-trip to the same canonical form.
// The tutorial uses the `/` form. A future doc edit that switches
// to `::` (or a future Rust-side example that copy-pastes a type
// path into YAML) should still load. This test guards the
// normalization path so a regression cannot silently break either
// the slash or the colon spelling.
// ============================================================================

#[test]
fn schema_separator_normalization_canonicalizes_double_colon() {
    let yaml_with_colon_separator = TUTORIAL_GRAPH_YAML.replace(
        "schema: sensor_msgs/LaserScan",
        "schema: sensor_msgs::LaserScan",
    );
    assert!(
        yaml_with_colon_separator.contains("schema: sensor_msgs::LaserScan"),
        "test fixture builder failed to switch to `::` separator"
    );

    let config = parse_graph_raw(&yaml_with_colon_separator).expect("parse succeeds");
    validate_graph(&config)
        .expect("validate succeeds — separator normalization preserves topology");

    let lidar_scan = &config.nodes[0].outputs[0];
    assert_eq!(
        lidar_scan.schema, "sensor_msgs/LaserScan",
        "OutputDef::deserialize must canonicalize `::` → `/`; got: {}",
        lidar_scan.schema
    );
}

// ============================================================================
// Adversarial: duplicate output topic fails validation.
//
// `validate_graph` rejects two outputs that resolve to the same
// full topic path (`{prefix}/{node_id}/{output_name}`). The
// tutorial's three outputs are distinct, but a future doc edit
// could inadvertently collide them (e.g. by adding a second
// output to drive_base that shadows a sibling). This test
// inserts a colliding output and confirms the validator catches it.
// ============================================================================

#[test]
fn duplicate_output_topic_fails_validation() {
    // Splice a second `scan` output onto lidar_sensor so both
    // resolve to the same topic path
    // (`/demo/lidar_sensor/scan`). The derived form
    // `/{prefix}/{node_id}/{output_name}` collides when two outputs
    // on the same node share `name`.
    //
    // Splicing on `max_slice_len: 8192\n` (unique in the tutorial)
    // anchors the insertion point precisely without false-matching
    // safety_controller's `scan` input port.
    let yaml_with_dup_output = TUTORIAL_GRAPH_YAML.replace(
        "        max_slice_len: 8192\n",
        "        max_slice_len: 8192\n      - name: scan\n        schema: sensor_msgs/LaserScan\n        max_slice_len: 8192\n",
    );
    // Sanity: exactly 2 output-flavored `scan` entries (the
    // original + the splice). Match the full output-block
    // signature `name: scan\n        schema:` so the count is
    // unaffected by safety_controller's `scan` input port
    // (which sits at the same indentation but is followed by
    // `source:` rather than `schema:`).
    assert_eq!(
        yaml_with_dup_output
            .matches("      - name: scan\n        schema: sensor_msgs/LaserScan\n")
            .count(),
        2,
        "test fixture builder must produce exactly 2 output entries named `scan` on lidar_sensor"
    );

    let config = parse_graph_raw(&yaml_with_dup_output).expect("parse succeeds");
    let err = validate_graph(&config).expect_err("validate must reject duplicate output topic");
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(
        reason.contains("duplicate output topic"),
        "validator must reject with `duplicate output topic`; got: {reason}"
    );
    assert!(
        reason.contains("/demo/lidar_sensor/scan"),
        "error must name the colliding full topic path so users see what duplicated; got: {reason}"
    );
}
