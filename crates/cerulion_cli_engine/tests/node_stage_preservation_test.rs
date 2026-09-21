// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion node stage` PRESERVES the
//! hand-authored graph file it appends to.
//!
//! A whole-file write through `graph_write` (`serde_yaml::to_string`
//! over the whole `GraphConfig` plus a truncating `std::fs::write`) would, on a file
//! with no typo at all, destroy comments, rewrite key order to
//! struct-declaration order, drop empty blocks via `skip_serializing_if`,
//! re-spell `pkg::Type` as `pkg/Type`, and leave no backup — on the surface
//! `docs/user-api.md` tells users to hand-edit, and which `graph partition` (via
//! `write_yaml_atomically`) promises byte-for-byte preservation on.
//! The other `node_stage` tests build their graphs from scratch,
//! so they exercise no preservation; this file is that coverage.
//!
//! The oracle is the DOCUMENT, not a fragment: each arm asserts the exact
//! surviving text outside the spliced region, so a splice that preserved
//! comments while (say) reordering `prefix:` under `nodes:` still fails.
//!
//! Parallel-safe — per-test `TempDir`, no transport, no process spawn.

use std::path::Path;

use cerulion_cli_engine::graph_cmd::{build_node_def, graph_read, node_stage};
use tempfile::TempDir;

/// A hand-authored graph: header comments, a per-node comment, a `network:`
/// block AFTER `nodes:` (so a serde round-trip would move it BEFORE), the
/// `pkg::Type` schema spelling serde normalizes away, and two-space list
/// indentation.
const HAND_AUTHORED: &str = "\
# Perception graph — hand-authored.
#
# The comments in this file explain WHY the overrides exist; a writer that
# drops them destroys the only record of that reasoning.
prefix: percep
nodes:
  # The camera is deliberately deep-buffered: the detector is bursty.
  - id: camera
    type: camera
    outputs:
      - name: image_raw
        schema: sensor_msgs::Image
        max_slice_len: 65536
        history_size: 4

# Only the detections leave this machine.
network:
  mode: peer
  egress:
    - /percep/camera/image_raw
";

fn write_graph(dir: &Path, name: &str, body: &str) {
    std::fs::write(dir.join(format!("{name}.yaml")), body).unwrap();
}

fn read_graph(dir: &Path, name: &str) -> String {
    std::fs::read_to_string(dir.join(format!("{name}.yaml"))).unwrap()
}

/// THE preservation oracle. Everything a whole-file write destroys survives, and
/// the appended entry lands inside the `nodes:` block at the document's own
/// indentation.
#[test]
fn staging_into_a_hand_authored_graph_preserves_every_byte_it_did_not_append() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "percep", HAND_AUTHORED);

    let def = build_node_def(
        "detector",
        None,
        &[(
            "detections".to_string(),
            Some("geometry_msgs/PoseArray".to_string()),
        )],
        &[("image_raw".to_string(), "camera/image_raw".to_string())],
    );
    node_stage(dir, "percep", def).unwrap();

    let after = read_graph(dir, "percep");

    // (1) Every comment survives, verbatim — including the one INSIDE the
    // `nodes:` block, which a splice could plausibly consume, and the one
    // BELOW it, which belongs to the next section.
    for comment in [
        "# Perception graph — hand-authored.",
        "# The comments in this file explain WHY the overrides exist; a writer that",
        "# drops them destroys the only record of that reasoning.",
        "  # The camera is deliberately deep-buffered: the detector is bursty.",
        "# Only the detections leave this machine.",
    ] {
        assert!(
            after.contains(comment),
            "comment must survive: {comment:?}\n--- got ---\n{after}"
        );
    }

    // (2) The `pkg::Type` spelling is UNTOUCHED. This is the sharpest
    // single-byte oracle in the file: `OutputDef.schema` carries
    // `deserialize_with = "normalize_schema_separator"`, so a parse→serialize
    // round trip silently rewrites it to `sensor_msgs/Image` — a user's chosen
    // spelling, changed by a command that was asked to append a different node.
    assert!(
        after.contains("schema: sensor_msgs::Image"),
        "the author's `::` spelling must survive\n--- got ---\n{after}"
    );

    // (3) Key ORDER survives: `network:` stays BELOW `nodes:`. Serde emits
    // struct-declaration order, which puts `network` last but would also
    // rewrite the whole document to get there.
    let nodes_at = after.find("\nnodes:").expect("nodes: present");
    let network_at = after.find("\nnetwork:").expect("network: present");
    assert!(
        network_at > nodes_at,
        "`network:` must stay below `nodes:`\n--- got ---\n{after}"
    );

    // (4) The whole PREFIX of the document — everything up to the appended
    // entry — is byte-identical to what was there before. This is the claim a
    // per-comment `contains` cannot make: it forbids reordering, requoting and
    // whitespace churn anywhere above the splice.
    let appended_at = after
        .find("  - id: detector")
        .expect("the entry was appended");
    let head_end = HAND_AUTHORED.find("history_size: 4\n").unwrap() + "history_size: 4\n".len();
    assert_eq!(
        &after[..appended_at],
        &HAND_AUTHORED[..head_end],
        "every byte above the splice must be untouched"
    );

    // (5) The entry landed at the document's OWN two-space list indentation,
    // with its nested keys following the corpus convention — not serde's
    // column-0 block-sequence style.
    assert!(
        after.contains(
            "  - id: detector\n    type: detector\n    inputs:\n      - name: image_raw\n        \
             source: camera/image_raw\n    outputs:\n      - name: detections\n        schema: \
             geometry_msgs/PoseArray\n"
        ),
        "the appended entry must adopt the document's indentation\n--- got ---\n{after}"
    );

    // (6) It re-parses, and the graph now holds both nodes.
    let config = graph_read(dir, "percep").unwrap();
    assert_eq!(config.nodes.len(), 2);
    assert_eq!(config.nodes[1].id, "detector");
    // The untouched `::` still normalizes on READ — preservation is about the
    // FILE, never about weakening the parser.
    assert_eq!(config.nodes[0].outputs[0].schema, "sensor_msgs/Image");
}

/// The `write_yaml_atomically` contract: a `.bak` of the PREVIOUS bytes, and no
/// temp file left behind. `graph_write` had neither — a Ctrl-C or ENOSPC
/// mid-`fs::write` left a truncated graph with nothing to restore from.
#[test]
fn staging_backs_up_the_previous_file_and_leaves_no_temp_behind() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "percep", HAND_AUTHORED);

    let def = build_node_def(
        "detector",
        None,
        &[(
            "detections".to_string(),
            Some("std_msgs/String".to_string()),
        )],
        &[],
    );
    node_stage(dir, "percep", def).unwrap();

    let bak = dir.join("percep.yaml.bak");
    assert!(bak.exists(), "a .bak must exist after an overwrite");
    assert_eq!(
        std::fs::read_to_string(&bak).unwrap(),
        HAND_AUTHORED,
        "the .bak must be the PREVIOUS bytes, byte-for-byte"
    );

    // temp+rename, not truncate-in-place: no `.percep.yaml.tmp.<pid>` survives.
    // (The window itself is unobservable from here; what IS observable is that
    // the writer left no debris, which a truncating writer cannot fail.)
    let leftovers: Vec<String> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.contains(".tmp."))
        .collect();
    assert!(leftovers.is_empty(), "stray temp files: {leftovers:?}");
}

/// The REFUSAL path, and the ORDER that makes it matter (the unknown-key
/// refusal composed with the preserving write): a graph carrying a misspelled
/// key is refused when `node_stage` READS it, so the write that would have
/// DELETED the evidence never runs.
///
/// The composition matters: a `node stage` that
/// re-serialized the whole file would let the next additive command silently erase
/// the typo'd line along with every comment explaining the overrides.
#[test]
fn a_misspelled_key_is_refused_at_read_so_the_write_never_erases_it() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    // `histroy_size:` — a near-miss of `history_size:`. Without the unknown-key
    // refusal it parses clean (silently VOLATILE), and a re-serializing `node stage` deletes it.
    let typo = "\
# A graph whose author meant late-joiner replay.
prefix: percep
nodes:
  - id: camera
    type: camera
    outputs:
      - name: image_raw
        schema: sensor_msgs/Image
        histroy_size: 4
";
    write_graph(dir, "percep", typo);

    let def = build_node_def(
        "detector",
        None,
        &[(
            "detections".to_string(),
            Some("std_msgs/String".to_string()),
        )],
        &[],
    );
    let err = node_stage(dir, "percep", def).unwrap_err().to_string();
    assert!(
        err.contains("histroy_size"),
        "the refusal must NAME the offending key, got: {err}"
    );

    // The evidence survives: the file is byte-untouched and no backup was made
    // (nothing was overwritten).
    assert_eq!(
        read_graph(dir, "percep"),
        typo,
        "a refused stage must leave the file byte-untouched"
    );
    assert!(
        !dir.join("percep.yaml.bak").exists(),
        "a refused stage must not write a backup"
    );
}

/// A fresh `graph create` writes `nodes: []` — serde's spelling for an empty
/// sequence — and block items cannot follow a FLOW-empty one. The splice drops
/// the `[]`, which is the only byte it rewrites that it did not append.
#[test]
fn staging_into_a_freshly_created_graph_replaces_the_empty_flow_sequence() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "fresh", "prefix: robot\nnodes: []\n");

    let def = build_node_def(
        "camera",
        None,
        &[("image".to_string(), Some("sensor_msgs/Image".to_string()))],
        &[],
    );
    node_stage(dir, "fresh", def).unwrap();

    let after = read_graph(dir, "fresh");
    assert_eq!(
        after,
        "prefix: robot\nnodes:\n  - id: camera\n    type: camera\n    outputs:\n      - name: \
         image\n        schema: sensor_msgs/Image\n",
        "a fresh graph must gain a block sequence, not `[]` followed by items"
    );
    // It re-parses (an `[]` left in place would be a YAML error, or worse,
    // would silently drop the node).
    let config = graph_read(dir, "fresh").unwrap();
    assert_eq!(config.nodes.len(), 1);
    assert_eq!(config.nodes[0].id, "camera");
}

/// `nodes: [] # comment` — the `[]` goes, the COMMENT stays,
/// and the result parses.
///
/// The strip keyed on `[]` being the line's suffix, so an inline comment made
/// it decline and block items were appended under a FLOW sequence — YAML that
/// does not parse at all. Refusing the stage would have been wrong too: the
/// document is valid and the comment is exactly the kind of byte this splice
/// exists to preserve.
#[test]
fn an_empty_flow_sequence_with_an_inline_comment_keeps_the_comment() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(
        dir,
        "fresh",
        "# a brand new graph\nprefix: robot\nnodes: []  # nothing staged yet\n",
    );

    let def = build_node_def(
        "camera",
        None,
        &[("image".to_string(), Some("sensor_msgs/Image".to_string()))],
        &[],
    );
    node_stage(dir, "fresh", def).unwrap();

    let after = read_graph(dir, "fresh");
    assert_eq!(
        after,
        "# a brand new graph\nprefix: robot\nnodes: # nothing staged yet\n  - id: camera\n    \
         type: camera\n    outputs:\n      - name: image\n        schema: sensor_msgs/Image\n",
        "the `[]` must go and the comment must stay\n--- got ---\n{after}"
    );

    // THE POINT: it re-parses. Left in place, the `[]` makes this a YAML
    // error, and the node is not in the graph at all.
    let config = graph_read(dir, "fresh").unwrap();
    assert_eq!(config.nodes.len(), 1);
    assert_eq!(config.nodes[0].id, "camera");
}

/// A flow-style `nodes:` is refused, with the
/// file byte-untouched — never appended to.
///
/// A block sequence item cannot follow a flow value, so appending one leaves
/// YAML that `graph_read` can no longer parse: `node stage` would report
/// success and hand back a corrupted graph. The empty-flow rewrite handles
/// `nodes: []`; everything else is refused rather than converted, because
/// rewriting entries this function did not author is the whole-file
/// re-serialization the splice exists to stop.
#[test]
fn a_flow_style_nodes_block_is_refused_with_the_file_untouched() {
    for original in [
        // The basic shape: a NON-EMPTY flow sequence.
        "prefix: robot\nnodes: [{id: camera, type: camera}]\n",
        // Two entries, and a trailing comment — the strip declines on both
        // counts, so this is the shape that must not slip through.
        "prefix: robot\nnodes: [{id: a, type: a}, {id: b, type: b}]  # inline\n",
    ] {
        let tmp = TempDir::new().unwrap();
        let dir = tmp.path();
        write_graph(dir, "flow", original);

        let def = build_node_def(
            "detector",
            None,
            &[("out".to_string(), Some("std_msgs/String".to_string()))],
            &[],
        );
        let err = node_stage(dir, "flow", def)
            .expect_err("a flow-style `nodes:` must be REFUSED, not appended to")
            .to_string();
        assert!(
            err.contains("INLINE (flow-style) value"),
            "the refusal must name the shape; got: {err}"
        );
        assert!(
            err.contains("nodes:\n  - id:"),
            "and show the block form that works; got: {err}"
        );

        // The FILE is byte-untouched and no backup was written — a refused
        // stage must leave the graph exactly as the user wrote it.
        assert_eq!(read_graph(dir, "flow"), original);
        assert!(!dir.join("flow.yaml.bak").exists());
    }

    // A flow MAPPING (`nodes: {a: b}`) is refused EARLIER, by the parser —
    // `GraphConfig.nodes` is a sequence, so `graph_read` rejects it before the
    // splice is reached. Asserted here so the boundary between the two
    // refusals is recorded rather than assumed, and so a future change that
    // made the parser lenient would surface here.
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    let original = "prefix: robot\nnodes: {a: b}\n";
    write_graph(dir, "flowmap", original);
    let def = build_node_def(
        "detector",
        None,
        &[("out".to_string(), Some("std_msgs/String".to_string()))],
        &[],
    );
    let err = node_stage(dir, "flowmap", def).unwrap_err().to_string();
    assert!(
        err.contains("expected a sequence"),
        "a flow mapping is the PARSER's refusal; got: {err}"
    );
    assert_eq!(read_graph(dir, "flowmap"), original);
}

/// The document's convention WINS, including serde's own column-0 style — a
/// graph written by a whole-file serde writer must not be re-indented by
/// the first surgical stage. The anti-tautology twin of the two-space arm: with
/// the indent hardcoded either way, exactly one of the two fails.
#[test]
fn staging_adopts_column_zero_indentation_when_that_is_what_the_file_uses() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(
        dir,
        "serdeish",
        "prefix: robot\nnodes:\n- id: camera\n  type: camera\n  outputs:\n  - name: image\n    \
         schema: sensor_msgs/Image\n",
    );

    let def = build_node_def(
        "detector",
        None,
        &[("out".to_string(), Some("std_msgs/String".to_string()))],
        &[],
    );
    node_stage(dir, "serdeish", def).unwrap();

    let after = read_graph(dir, "serdeish");
    assert!(
        after.contains(
            "\n- id: detector\n  type: detector\n  outputs:\n  - name: out\n    schema: \
                        std_msgs/String\n"
        ),
        "the entry must adopt the file's column-0 style\n--- got ---\n{after}"
    );
    assert!(
        !after.contains("  - id: detector"),
        "the two-space convention must NOT be imposed on a column-0 file\n--- got ---\n{after}"
    );
    assert_eq!(graph_read(dir, "serdeish").unwrap().nodes.len(), 2);
}

/// A CRLF document stays CRLF outside the splice. `split_lines` models the
/// carriage return, so this is a real property of the scanner rather than an
/// accident of the fixtures — and a `\n`-only rewrite of a Windows-authored
/// graph is exactly the invisible churn the splice exists to avoid.
#[test]
fn staging_preserves_crlf_line_endings_outside_the_splice() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    let crlf = "# windows-authored\r\nprefix: robot\r\nnodes:\r\n  - id: camera\r\n    type: \
                camera\r\n    outputs:\r\n      - name: image\r\n        schema: \
                sensor_msgs/Image\r\n";
    write_graph(dir, "crlf", crlf);

    let def = build_node_def(
        "detector",
        None,
        &[("out".to_string(), Some("std_msgs/String".to_string()))],
        &[],
    );
    node_stage(dir, "crlf", def).unwrap();

    let after = read_graph(dir, "crlf");
    assert!(
        after.starts_with(crlf),
        "the original CRLF document must be an untouched PREFIX\n--- got ---\n{after:?}"
    );
    assert_eq!(graph_read(dir, "crlf").unwrap().nodes.len(), 2);
}

/// A node whose output declares no
/// discoverable schema STAGES (with a loud warn) and is then REFUSED when the
/// graph is validated or run.
///
/// This shape is deliberate, and the decision turns on a format fact rather than
/// a preference. The `CERULION:INFO_START` document can express a port's
/// `schema_hash` but has NO field for a schema NAME — the info-JSON port types
/// in `cerulion_core::graph::node` are `deny_unknown_fields` over
/// `name` / `schema_hash` / `max_slice_len_default` / `promise_within_ms` —
/// and `cerulion node create --raw-ffi` scaffolds outputs as bare name strings
/// carrying neither. So `node_metadata::PortDef::schema` is `None` for EVERY
/// raw-FFI node, and a stage-time refusal would brick an entire node class
/// that `cerulion node create --raw-ffi` itself generates.
///
/// Deriving the name from the hash was the preferred remedy and does not work
/// at this seam: `LayoutResolver::schema_name_for_hash` exists and would do
/// it, but staging reads SOURCE, and a scaffolded raw-FFI node's source
/// carries no hash to look up (the hash lives in the COMPILED cdylib's
/// `OutputMeta`, which staging — normally run before `node build` — has not
/// got).
///
/// So the contract is loud at BOTH ends and a dead end at neither, and this
/// arm pins the whole round trip rather than either half alone.
#[tracing_test::traced_test]
#[test]
fn a_schema_less_output_stages_with_a_warn_and_is_refused_at_run() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "rawffi", "prefix: robot\nnodes: []\n");

    // `None` is what `build_node_def` receives for every raw-FFI port.
    let def = build_node_def("sensor", None, &[("out".to_string(), None)], &[]);

    // (1) It STAGES rather than bricking.
    let config = node_stage(dir, "rawffi", def).expect("staging must not be a dead end");
    assert_eq!(config.nodes.len(), 1);

    // (2) LOUDLY — naming the port, and both reasons the CLI could not derive
    // the name, so the user knows which fix applies to their node.
    assert!(
        logs_contain("declare no schema"),
        "staging a schema-less output must WARN"
    );
    assert!(logs_contain("outputs=out"), "the warn must name the port");
    assert!(
        logs_contain("carries a schema HASH but never a NAME"),
        "the warn must state the raw-FFI reason"
    );
    assert!(
        logs_contain("use native_ros2_messages"),
        "and the macro-node reason"
    );
    assert!(logs_contain("WARN"), "at WARN, not below it");

    // (3) The entry really is on disk, with an EMPTY `schema:` — nothing was
    // invented to fill it (Principle #13). The value is DOUBLE-quoted because
    // `render_scalar` quotes anything that is not a plain-safe token; asserted
    // on the re-parse as well, so the pin is about the VALUE rather than about
    // which of YAML's two empty-string spellings the renderer chose.
    let after = read_graph(dir, "rawffi");
    assert!(
        after.contains("- id: sensor") && after.contains("schema: \"\""),
        "the entry must be written with an EMPTY schema, not a fabricated one\n{after}"
    );
    assert_eq!(
        graph_read(dir, "rawffi").unwrap().nodes[0].outputs[0].schema,
        "",
        "and it must re-parse as empty, not as some invented placeholder"
    );

    // (4) THE OTHER END: the staged graph is REFUSED when validated/run, by
    // the unconditional `validate_graph` every run path inherits. Without this
    // half the warn would be the only signal and a graph that cannot run would
    // look staged-and-ready.
    let reread = graph_read(dir, "rawffi").unwrap();
    let err = cerulion_core::graph::validate_graph(&reread)
        .expect_err("a schema-less output must be refused at run")
        .to_string();
    assert!(
        err.contains("sensor") && err.contains("out") && err.contains("is missing"),
        "the run-time refusal must name node, output and cause; got: {err}"
    );

    // (5) ANTI-TAUTOLOGY: filling the schema in — the one-line fix the warn
    // names — makes the SAME graph valid. This is what proves the staged file
    // is a working starting point rather than a dead end.
    let fixed = read_graph(dir, "rawffi").replace("schema: \"\"", "schema: std_msgs/String");
    std::fs::write(dir.join("rawffi.yaml"), &fixed).unwrap();
    let repaired = graph_read(dir, "rawffi").unwrap();
    cerulion_core::graph::validate_graph(&repaired)
        .expect("filling the schema in must make the staged graph valid");
}

/// ANTI-TAUTOLOGY for the warn: a node whose ports DO carry schemas stages
/// silently. Without this, "schema-less staging warns" is satisfied by a stage
/// that warns on everything.
#[tracing_test::traced_test]
#[test]
fn a_well_declared_output_stages_without_the_schema_warn() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "ok", "prefix: robot\nnodes: []\n");

    let def = build_node_def(
        "camera",
        None,
        &[("image".to_string(), Some("sensor_msgs/Image".to_string()))],
        &[],
    );
    node_stage(dir, "ok", def).unwrap();

    assert!(
        !logs_contain("declare no schema"),
        "a well-declared output must stage silently"
    );
    cerulion_core::graph::validate_graph(&graph_read(dir, "ok").unwrap())
        .expect("and the staged graph must be valid");
}

/// The node-stage emitter reaches the SAME scalar
/// rule the `process_groups:` emitter does, so a numeric-looking node id or
/// schema is written QUOTED and re-parses as the string it was given.
///
/// The token CORPUS lives with the rule, in
/// `partition_emit_test::every_ambiguous_group_scalar_re_parses_as_the_string_it_was_given`
/// (one rule, one corpus — `render_scalar` is shared by both emitters). What
/// this arm adds is ADOPTION: that `node stage`'s own renderer really goes
/// through it, in the two positions only it emits.
///
/// The oracle is the UNTYPED `serde_yaml::Value`, deliberately. MEASURED on
/// the shipping `serde_yaml` 0.9.34: its TYPED `String` deserialization is
/// lenient enough that an unquoted `1e3` still comes back as `"1e3"` through
/// the real `parse_graph_raw`, so a re-parse oracle would pass against an
/// emitter that leaves it unquoted and pin nothing. Every other reader — an untyped walk,
/// PyYAML, `yq` — sees `1000.0`.
#[test]
fn a_numeric_looking_id_and_schema_are_emitted_as_strings() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "amb", "prefix: robot\nnodes: []\n");

    // `1e3` in BOTH positions the node-stage renderer emits a free-form value
    // into: the node `id:` and an output `schema:`.
    let def = build_node_def(
        "sensor",
        Some("1e3"),
        &[("out".to_string(), Some("1e3".to_string()))],
        &[],
    );
    node_stage(dir, "amb", def).expect("a numeric-looking id must still stage");

    let after = read_graph(dir, "amb");
    let doc: serde_yaml::Value =
        serde_yaml::from_str(&after).unwrap_or_else(|e| panic!("{e}\n{after}"));
    let node = doc
        .get("nodes")
        .and_then(|n| n.get(0))
        .unwrap_or_else(|| panic!("no staged node\n{after}"));

    assert_eq!(
        node.get("id").cloned(),
        Some(serde_yaml::Value::String("1e3".to_string())),
        "the node id must re-parse as the string `1e3`, not the number 1000\n{after}"
    );
    assert_eq!(
        node.get("outputs")
            .and_then(|o| o.get(0))
            .and_then(|o| o.get("schema"))
            .cloned(),
        Some(serde_yaml::Value::String("1e3".to_string())),
        "the schema must re-parse as the string `1e3`\n{after}"
    );

    // ANTI-TAUTOLOGY: an ordinary identifier in the same positions is NOT
    // quoted, so this is a rule about ambiguous tokens and not "quote
    // everything" (which would churn every graph file in the tree).
    let def = build_node_def(
        "camera",
        None,
        &[("image".to_string(), Some("sensor_msgs/Image".to_string()))],
        &[],
    );
    node_stage(dir, "amb", def).expect("stage");
    let after = read_graph(dir, "amb");
    assert!(
        after.contains("- id: camera\n") && after.contains("schema: sensor_msgs/Image\n"),
        "an ordinary identifier must stay plain\n{after}"
    );
}

/// `node_stage` reads the graph file exactly ONCE.
///
/// Reading twice — `graph_read` at the top, then `read_to_string` again
/// at splice time — would let the duplicate-id check, the schema warn and
/// `validate_graph_with` all reason about one revision of the file while the
/// splice lands on whatever is there a moment later. That gap is INSIDE the
/// window the write's `ExpectedPrior::Contents` precondition exists to close,
/// so leaving it would make the precondition cover less than it claims.
///
/// SCOPE: structural, and deliberately so. The gap is microseconds wide
/// inside one function call, so observing it behaviourally would need a
/// fault-injection seam in a shipping path. What the walk pins is the shape
/// that makes the precondition total: one read, and the write holding what
/// that read returned.
#[test]
fn node_stage_reads_the_graph_file_once_and_holds_those_bytes() {
    let src = std::fs::read_to_string("src/graph_cmd.rs").expect("readable");
    let body = code_only(fn_body(&src, "pub fn node_stage("));

    // Counting `graph_read` (not `graph_read_raw`) is what makes this see a
    // second read spelled `graph_read(`: `"graph_read("` is
    // not a substring of `"graph_read_raw("`, so the obvious reintroduction
    // slips a `_raw`-only needle set entirely. The `_raw` call contains
    // `graph_read`, so the expected count is still exactly one.
    assert_eq!(
        body.matches("graph_read").count(),
        1,
        "node_stage must take the config AND its bytes from ONE read — a second \
         `graph_read(` is the exact gap the one read closes\n{body}"
    );
    assert!(
        body.contains("graph_read_raw("),
        "and that one read must be the bytes-returning form\n{body}"
    );
    for spelling in ["read_to_string", "fs::read("] {
        assert!(
            !body.contains(spelling),
            "a SECOND read (`{spelling}`) re-opens the gap the one read closes\n{body}"
        );
    }
    assert!(
        body.contains("ExpectedPrior::Contents(&raw)"),
        "and the write must hold exactly the bytes that read returned\n{body}"
    );
}

/// ANTI-TAUTOLOGY for the walk above: `fn_body` must really isolate ONE
/// function, or its `!contains` assertions are over the whole file and mean
/// nothing.
#[test]
fn the_function_body_extractor_isolates_one_function() {
    let src = "fn a() {\n  read_to_string(x);\n}\nfn b() {\n  let y = 2;\n}\n";
    let b = fn_body(src, "fn b(");
    assert!(b.contains("let y"), "the named body must be present: {b:?}");
    assert!(
        !b.contains("read_to_string"),
        "another function's body must not be: {b:?}"
    );
}

/// The brace-matched body of the function whose signature starts with
/// `signature`, from its opening `{` to the matching `}`.
fn fn_body<'a>(src: &'a str, signature: &str) -> &'a str {
    let start = src
        .find(signature)
        .unwrap_or_else(|| panic!("no function matching {signature:?}"));
    let open = start
        + src[start..]
            .find('{')
            .unwrap_or_else(|| panic!("no body for {signature:?}"));
    let mut depth = 0usize;
    for (offset, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[open..=open + offset];
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced body for {signature:?}");
}

/// `node_stage` forwards an arbitrary `NodeDef.id` to
/// `render_scalar`, so a control character in it must survive the splice
/// byte-for-byte — or not be written at all.
///
/// Reachable: `cerulion node stage --id` takes any string, and `validate_graph`
/// has no charset rule for a node id (it checks uniqueness; the derived-topic
/// rule only bites on a node that HAS an output). A quoted form that
/// escapes only `\` and `"` lets a literal newline inside `"..."` FOLD to a
/// space, and the staged graph comes back naming a DIFFERENT node — silently,
/// in a file nothing would flag.
///
/// The token corpus lives with the rule in `partition_emit_test`; this arm is
/// ADOPTION — that `node stage`'s own renderer reaches it.
#[test]
fn a_control_character_in_a_staged_node_id_round_trips() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(dir, "ctrl", "prefix: robot\nnodes: []\n");

    let hostile = "a\nb";
    let def = build_node_def("sensor", Some(hostile), &[], &[]);
    node_stage(dir, "ctrl", def).expect("staging must not corrupt the file");

    // The file must still READ — the whole contract of the surgical splice.
    let reread = graph_read(dir, "ctrl").expect("the staged graph must re-read");
    assert_eq!(
        reread.nodes[0].id, hostile,
        "the node id must come back byte-for-byte, not folded to `a b`"
    );

    // No RAW control character reached the file (the only literal newlines are
    // the line terminators) — the anti-tautology for "it round-trips": a
    // parser more forgiving than the spec could otherwise hide a raw byte.
    let after = read_graph(dir, "ctrl");
    assert!(
        after.chars().all(|c| !c.is_control() || c == '\n'),
        "no raw control character may reach the graph file\n{after:?}"
    );
}

/// Staging must not touch
/// an unrelated `outputs: []` on the block's LAST line.
///
/// `strip_empty_flow_sequence` sliced from the `nodes:` key to the end of the
/// BLOCK and trimmed, so the block's final line became the candidate value: a
/// graph whose last node ends `outputs: []` had THAT `[]` stripped and
/// rewritten to a bare `outputs:`. An authored property silently changed by a
/// splice whose entire contract is to touch nothing it did not append — and
/// the quiet kind, because the file still looks plausible.
#[test]
fn an_empty_nested_sequence_on_the_last_line_survives_staging() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    let before = "\
prefix: robot
nodes:
  - id: sink
    type: sink
    outputs: []
";
    write_graph(dir, "nested", before);

    let def = build_node_def("camera", None, &[], &[]);
    node_stage(dir, "nested", def).expect("stage");

    let after = read_graph(dir, "nested");
    assert!(
        after.contains("    outputs: []\n"),
        "the existing node's `outputs: []` must survive byte-for-byte\n{after}"
    );
    assert!(
        !after.contains("    outputs:\n"),
        "it must not be rewritten to a bare `outputs:`\n{after}"
    );
    // The whole prefix of the document is untouched, and the file still reads.
    assert!(
        after.starts_with(before),
        "nothing before the append may move\n{after}"
    );
    let reread = graph_read(dir, "nested").expect("the staged graph must re-read");
    assert_eq!(reread.nodes.len(), 2);
    assert!(reread.nodes[0].outputs.is_empty());
}

/// A YAML collection
/// PROPERTY on the `nodes:` key line is not a flow value.
///
/// `nodes: &graph_nodes` and `nodes: !!seq` are ordinary BLOCK sequences whose
/// items sit on the following lines — both read fine before staging, so the
/// flow-style refusal must not reject them. Appending
/// an item under the property stays valid YAML and keeps the anchor/tag.
#[test]
fn an_anchored_or_tagged_nodes_block_still_stages() {
    for property in ["&graph_nodes", "!!seq"] {
        let tmp = TempDir::new().unwrap();
        let dir = tmp.path();
        let before = format!("prefix: robot\nnodes: {property}\n  - id: sink\n    type: sink\n");
        write_graph(dir, "prop", &before);
        // PRECONDITION: the document really is valid before we touch it, so a
        // refusal cannot be excused as "that graph was broken anyway".
        graph_read(dir, "prop").expect("the property-carrying graph must read BEFORE staging");

        let def = build_node_def("camera", None, &[], &[]);
        node_stage(dir, "prop", def).unwrap_or_else(|e| {
            panic!("`nodes: {property}` is a block sequence, not a flow value: {e}")
        });

        let after = read_graph(dir, "prop");
        assert!(
            after.contains(&format!("nodes: {property}\n")),
            "the property must be preserved verbatim\n{after}"
        );
        let reread = graph_read(dir, "prop").expect("the staged graph must re-read");
        assert_eq!(
            reread.nodes.len(),
            2,
            "the appended node must be there\n{after}"
        );
    }
}

/// ANTI-TAUTOLOGY for the arm above: a REAL flow value is still refused, with
/// the file untouched. Without this, "anchors are accepted" could be satisfied
/// by dropping the flow-style guard altogether — the corruption it exists
/// to prevent.
#[test]
fn an_anchored_flow_value_is_still_refused() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    let before = "prefix: robot\nnodes: &a [{id: sink, type: sink}]\n";
    write_graph(dir, "flow", before);

    let def = build_node_def("camera", None, &[], &[]);
    let err = node_stage(dir, "flow", def)
        .expect_err("an ANCHORED flow sequence is still an inline value");
    assert!(err.to_string().contains("INLINE (flow-style) value"));
    assert_eq!(
        read_graph(dir, "flow"),
        before,
        "a refused stage writes nothing"
    );
}

/// A flow value on the line AFTER `nodes:` must not
/// be staged into.
///
/// Every guard in this splice inspects the KEY LINE, and YAML lets the flow
/// value sit on the next line. All three of these parse fine before staging
/// and slipped every specific refusal: the entry was appended after a
/// completed flow value, `node stage` returned Ok, the CLI reported success,
/// and the next `graph_read` failed — the exact class the refusal text
/// describes ("node stage would report success and leave a graph graph_read
/// can no longer read").
///
/// What closes it is the TOTAL guard — the spliced document must parse before
/// it is written — so this arm is about the property, not these three inputs.
#[test]
fn a_flow_value_on_a_continuation_line_is_refused_with_the_file_untouched() {
    for body in [
        "nodes:\n  [{id: sink, type: sink}]\n",
        "nodes:\n  []\n",
        "nodes: &a\n  [{id: sink, type: sink}]\n",
    ] {
        let tmp = TempDir::new().unwrap();
        let dir = tmp.path();
        let before = format!("prefix: robot\n{body}");
        write_graph(dir, "cont", &before);
        // PRECONDITION: valid before we touch it, so a refusal cannot be
        // excused as "that graph was already broken".
        graph_read(dir, "cont")
            .unwrap_or_else(|e| panic!("fixture must parse BEFORE staging ({body:?}): {e}"));

        let def = build_node_def("camera", None, &[], &[]);
        let err = node_stage(dir, "cont", def).expect_err(&format!(
            "a flow value on a continuation line must be refused: {body:?}"
        ));

        assert!(
            err.to_string().contains("no longer parses")
                || err.to_string().contains("INLINE (flow-style) value"),
            "the refusal must name the shape; got: {err}"
        );
        assert_eq!(
            read_graph(dir, "cont"),
            before,
            "a refused stage writes NOTHING ({body:?})"
        );
    }
}

/// ANTI-TAUTOLOGY for the total guard: it must refuse only documents that
/// really do not parse. Without this, "the spliced document must parse" is
/// satisfied by a `node stage` that refuses everything.
#[test]
fn the_total_guard_does_not_refuse_a_healthy_stage() {
    let tmp = TempDir::new().unwrap();
    let dir = tmp.path();
    write_graph(
        dir,
        "ok",
        "prefix: robot\nnodes:\n  - id: sink\n    type: sink\n",
    );
    let def = build_node_def("camera", None, &[], &[]);
    node_stage(dir, "ok", def).expect("an ordinary block sequence must still stage");
    assert_eq!(graph_read(dir, "ok").unwrap().nodes.len(), 2);
}

/// `src` with COMMENTS removed — line and (depth-tracked, because Rust's
/// nest) block. The structural walks below count code tokens, and a comment
/// that merely NAMES one must neither satisfy a requirement nor trip a
/// prohibition. Rust block-comment markers are spelled with an escaped `*`
/// here so this function's own doc cannot confuse a reader.
fn code_only(src: &str) -> String {
    let bytes: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 && bytes[i] == '/' && bytes.get(i + 1) == Some(&'/') {
            while i < bytes.len() && bytes[i] != '\n' {
                i += 1;
            }
            continue;
        }
        if bytes[i] == '/' && bytes.get(i + 1) == Some(&'*') {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i] == '*' && bytes.get(i + 1) == Some(&'/') {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            out.push(bytes[i]);
        }
        i += 1;
    }
    out
}

/// ANTI-TAUTOLOGY for [`code_only`]: a broken stripper would make every
/// "must not contain" assertion above vacuous, and every "must contain" one
/// unsatisfiable.
#[test]
fn code_only_strips_comments_and_keeps_code() {
    let stripped = code_only("let a = 1; // graph_read\nlet b = 2;\n");
    assert!(stripped.contains("let a = 1;") && stripped.contains("let b = 2;"));
    assert!(!stripped.contains("graph_read"));
    let block = code_only("let a = 1; /* outer /* inner graph_read */ still */ let b = 2;");
    assert!(block.contains("let a = 1;") && block.contains("let b = 2;"));
    assert!(
        !block.contains("graph_read"),
        "nested block comments must strip: {block:?}"
    );
}
