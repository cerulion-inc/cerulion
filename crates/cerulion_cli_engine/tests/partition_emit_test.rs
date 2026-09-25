// SPDX-License-Identifier: AGPL-3.0-only
//! PURE tests for the auto-partition EMIT engine core:
//! the comment-preserving surgical `process_groups:` rewrite
//! ([`rewrite_process_groups_block`]), the emit glue
//! ([`derive_emit_groups`]), and the generalized atomic YAML writer
//! ([`write_yaml_atomically`]).
//!
//! No iceoryx2, no transport, no clock, no `GraphRuntime::build` — parallel-safe.
//! Oracles are hand-built (never a self-compare of the function under test):
//! full-string equality for the splice, hand-derived group shapes for the emit
//! glue, and real `GraphConfig` re-parses for round-trip validity.

use indexmap::IndexMap;

use cerulion_cli_engine::graph_cmd::{
    build_node_def, graph_create, graph_read, node_stage, write_profile_artifact_atomically,
    write_yaml_atomically, ExpectedPrior, ProfileArtifact, ProfileEdge, PROFILE_ARTIFACT_VERSION,
};
use cerulion_cli_engine::partition_emit::{derive_emit_groups, rewrite_process_groups_block};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::{parse_graph_raw, NodeInfo, TriggerEdges};

/// Build an `IndexMap` of groups from `(name, members)` pairs (insertion order
/// preserved — the rank/pipeline order the emitter promises).
fn groups(pairs: &[(&str, &[&str])]) -> IndexMap<String, Vec<String>> {
    pairs
        .iter()
        .map(|(name, members)| {
            (
                name.to_string(),
                members.iter().map(|m| m.to_string()).collect(),
            )
        })
        .collect()
}

/// Flatten a groups map into ordered `(name, members)` pairs for oracle
/// comparison.
fn shape(map: &IndexMap<String, Vec<String>>) -> Vec<(String, Vec<String>)> {
    map.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
}

// ==========================================================================
// Section A — rewrite_process_groups_block (pure string surgery).
// ==========================================================================

#[test]
fn replace_arm_preserves_every_byte_outside_the_block() {
    // Comments above, below, AND inside unrelated blocks + an inline comment;
    // an existing process_groups block to replace.
    let input = "\
# graph demo — comments must survive
name: demo
prefix: p    # inline kept

# --- old partition (replaced) ---
process_groups:
  old_a: [a, b]   # inner comment dropped with the block
  old_c: [c]

# --- nodes (untouched) ---
nodes:
  - id: a
    type: a
";
    // Hand-derived oracle: ONLY the process_groups block changes; the inline
    // comment inside it is authoritatively replaced, everything else verbatim.
    let expected = "\
# graph demo — comments must survive
name: demo
prefix: p    # inline kept

# --- old partition (replaced) ---
process_groups:
  grp_a: [a, b]
  grp_c: [c]

# --- nodes (untouched) ---
nodes:
  - id: a
    type: a
";
    let out = rewrite_process_groups_block(
        input,
        &groups(&[("grp_a", &["a", "b"]), ("grp_c", &["c"])]),
        "demo.yaml",
    )
    .expect("replace must succeed");
    assert_eq!(out, expected);
}

#[test]
fn insert_arm_places_block_before_nodes_with_a_blank_separator() {
    let input = "\
name: demo
prefix: p

nodes:
  - id: a
    type: a
";
    let expected = "\
name: demo
prefix: p

process_groups:
  grp_a: [a]

nodes:
  - id: a
    type: a
";
    let out = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "demo.yaml")
        .expect("insert must succeed");
    assert_eq!(out, expected);
}

#[test]
fn rewrite_is_idempotent_and_deterministic() {
    let input = "\
name: demo
prefix: p

nodes:
  - id: a
    type: a
";
    let g = groups(&[("grp_a", &["a"])]);

    // Two independent runs on the same input are byte-identical.
    let run1 = rewrite_process_groups_block(input, &g, "demo.yaml").expect("run1");
    let run2 = rewrite_process_groups_block(input, &g, "demo.yaml").expect("run2");
    assert_eq!(run1, run2, "pure function ⇒ identical across runs");

    // Rewriting the OUTPUT with the SAME groups is a no-op (this time a REPLACE, not
    // an insert) — pins that a second emit converges instead of stacking blocks.
    let twice = rewrite_process_groups_block(&run1, &g, "demo.yaml").expect("twice");
    assert_eq!(
        twice, run1,
        "second rewrite with the same groups is idempotent"
    );
    // And there is exactly ONE process_groups block afterward (no duplication).
    assert_eq!(
        twice.matches("process_groups:").count(),
        1,
        "idempotent rewrite must not stack a second block"
    );
}

#[test]
fn duplicate_process_groups_block_is_refused_with_diagnosis() {
    let input = "\
name: demo
process_groups:
  g1: [a]
nodes:
  - id: a
    type: a
process_groups:
  g2: [a]
";
    let err = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "twin.yaml")
        .expect_err("two top-level process_groups keys must be refused")
        .to_string();
    assert!(err.contains("twin.yaml"), "names the file: {err}");
    assert!(
        err.contains("process_groups"),
        "names what was searched: {err}"
    );
    assert!(
        err.contains("ambiguous"),
        "explains why it refused (ambiguous bounds): {err}"
    );
}

#[test]
fn insert_without_a_nodes_key_is_refused_with_diagnosis() {
    // No process_groups (⇒ insert arm) AND no top-level nodes: key.
    let input = "\
name: demo
prefix: p
";
    let err = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "no_nodes.yaml")
        .expect_err("insertion with no nodes: anchor must be refused")
        .to_string();
    assert!(err.contains("no_nodes.yaml"), "names the file: {err}");
    assert!(err.contains("nodes:"), "names the missing anchor: {err}");
}

#[test]
fn duplicate_nodes_key_on_insert_is_refused() {
    let input = "\
name: demo
nodes:
  - id: a
    type: a
nodes:
  - id: b
    type: b
";
    let err = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "dup_nodes.yaml")
        .expect_err("ambiguous insertion point must be refused")
        .to_string();
    assert!(
        err.contains("dup_nodes.yaml") && err.contains("ambiguous"),
        "{err}"
    );
}

#[test]
fn empty_groups_is_refused_as_internal_error() {
    let input = "name: demo\nnodes:\n  - id: a\n    type: a\n";
    let err = rewrite_process_groups_block(input, &IndexMap::new(), "empty.yaml")
        .expect_err("an empty partition must be refused")
        .to_string();
    assert!(
        err.contains("empty.yaml") && err.contains("at least one group"),
        "{err}"
    );
}

#[test]
fn spliced_output_reparses_to_the_emitted_groups() {
    let input = "\
name: demo
prefix: p
nodes:
  - id: a
    type: a
    outputs:
      - name: out
        schema: Vector3
  - id: b
    type: b
    inputs:
      - name: inp
        source: a/out
";
    let g = groups(&[("grp_a", &["a"]), ("grp_b", &["b"])]);
    let out = rewrite_process_groups_block(input, &g, "demo.yaml").expect("rewrite");

    let parsed = parse_graph_raw(&out).expect("the spliced output must re-parse as a graph");
    assert_eq!(
        shape(&parsed.process_groups),
        shape(&g),
        "the re-parsed process_groups must equal the emitted groups exactly (order + members)"
    );
    // The node topology is untouched by the splice.
    assert_eq!(parsed.nodes.len(), 2);
    // By design `name:` is optional-and-ignored, but a surgical
    // rewrite must still round-trip a line the author wrote.
    assert_eq!(parsed.name.as_deref(), Some("demo"));
}

#[test]
fn ambiguous_flow_scalars_are_quoted_and_reparse_exactly() {
    // Members that would corrupt a NAIVE flow render: a comma splits a token
    // into two; a `colon space` opens an inline mapping. The quoter must keep
    // each a single string. (Mutation guard: a no-op render_scalar fails here.)
    let g = groups(&[("grp_x", &["a,b", "c: d", "plain_1"])]);
    let input = "name: demo\nnodes:\n  - id: n\n    type: n\n";
    let out = rewrite_process_groups_block(input, &g, "quote.yaml").expect("rewrite");

    let parsed = parse_graph_raw(&out).expect("quoted flow scalars must re-parse");
    assert_eq!(
        parsed.process_groups.get("grp_x"),
        Some(&vec![
            "a,b".to_string(),
            "c: d".to_string(),
            "plain_1".to_string()
        ]),
        "each ambiguous token must round-trip as its exact original string"
    );
}

#[test]
fn crlf_line_endings_are_preserved_outside_the_block() {
    // A CRLF document with an existing block. The bytes OUTSIDE the block keep
    // their \r\n terminators; the output still re-parses.
    let input = "name: demo\r\n# keep this CRLF comment\r\nprocess_groups:\r\n  old: [a]\r\nnodes:\r\n  - id: a\r\n    type: a\r\n";
    let out = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "crlf.yaml")
        .expect("rewrite");

    assert!(
        out.contains("# keep this CRLF comment\r\n"),
        "the CRLF comment line must survive byte-for-byte: {out:?}"
    );
    assert!(
        out.contains("  - id: a\r\n"),
        "the CRLF node line must survive: {out:?}"
    );
    let parsed = parse_graph_raw(&out).expect("mixed-ending output still parses");
    assert_eq!(
        parsed.process_groups.get("grp_a"),
        Some(&vec!["a".to_string()])
    );
}

// ==========================================================================
// Section B — derive_emit_groups (the emit glue).
// ==========================================================================

/// The canonical demo graph: `ticker -> sink` (trigger edge) + a disconnected
/// `laggard`. Mirrors the `thawed_artifact_feeds_auto_partition` fixture so the
/// glue's two arms are checked against the SAME topology the partitioner tests
/// use.
fn demo_graph() -> (GraphConfig, IndexMap<String, NodeInfo>, TriggerEdges) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "demo".to_string(),
        prefix: "p".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "cmd".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "trigger_in".to_string(),
                    source: "ticker/cmd".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "laggard".to_string(),
                node_type: "laggard".to_string(),
                inputs: vec![],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    };
    let infos: IndexMap<String, NodeInfo> = config
        .nodes
        .iter()
        .map(|n| (n.id.clone(), NodeInfo::with_meta(Vec::new(), Vec::new())))
        .collect();
    let mut edges = TriggerEdges::new();
    edges.insert("sink", "/p/ticker/cmd");
    (config, infos, edges)
}

#[test]
fn no_costs_yields_process_per_node_baseline() {
    let (config, infos, edges) = demo_graph();
    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000)
        .expect("baseline derivation must succeed");
    // Every node its own group; pipeline order (ticker L0, laggard L0, sink L1).
    assert_eq!(
        shape(&out),
        vec![
            ("grp_ticker".to_string(), vec!["ticker".to_string()]),
            ("grp_laggard".to_string(), vec!["laggard".to_string()]),
            ("grp_sink".to_string(), vec!["sink".to_string()]),
        ],
        "no costs ⇒ process-per-node, named + ordered by the partitioner"
    );
}

#[test]
fn cost_artifact_yields_fused_groups() {
    let (config, infos, edges) = demo_graph();
    // ticker->sink is a rated trigger edge; laggard is isolated (no cost).
    let artifact = ProfileArtifact {
        version: PROFILE_ARTIFACT_VERSION,
        graph: "demo".to_string(),
        window_ns: 5_000_000_000,
        nodes: [("ticker".to_string(), 1200), ("sink".to_string(), 850)]
            .into_iter()
            .collect(),
        edges: vec![ProfileEdge {
            producer: "ticker".to_string(),
            consumer: "sink".to_string(),
            rate_mhz: 20_000,
        }],
        isolated: vec!["laggard".to_string()],
        // None ⇒ platform_default hop, which makes fusion profitable on every
        // target (cross > intra).
        hop: None,
        // No frozen budget in this fixture — the explicit budget arg
        // below is what this test exercises (the frozen-default resolution
        // point is pinned in `graph_partition_test`).
        derived_budget_ns: None,
        profile_cores: None,
    };
    let out = derive_emit_groups(&config, &infos, &edges, Some(&artifact), 100_000)
        .expect("cost-aware derivation must succeed");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_ticker".to_string(),
                vec!["ticker".to_string(), "sink".to_string()]
            ),
            ("grp_laggard".to_string(), vec!["laggard".to_string()]),
        ],
        "the rated edge fuses ticker+sink; the cost-less laggard stays singleton"
    );
}

#[test]
fn emit_then_rewrite_end_to_end() {
    // The full emit seam: derive baseline groups, splice them into a graph file,
    // re-parse, and confirm the file carries exactly those groups.
    let (config, infos, edges) = demo_graph();
    let g = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");

    let yaml = "\
name: demo
prefix: p
nodes:
  - id: ticker
    type: ticker
    outputs:
      - name: cmd
        schema: Vector3
  - id: sink
    type: sink
    inputs:
      - name: trigger_in
        source: ticker/cmd
  - id: laggard
    type: laggard
";
    let out = rewrite_process_groups_block(yaml, &g, "demo.yaml").expect("rewrite");
    let parsed = parse_graph_raw(&out).expect("re-parse");
    assert_eq!(shape(&parsed.process_groups), shape(&g));
    assert_eq!(parsed.nodes.len(), 3, "node topology untouched");
}

// ==========================================================================
// Section C — write_yaml_atomically (the generalized atomic writer).
// ==========================================================================

#[test]
fn write_yaml_atomically_fresh_write_has_no_backup() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.yaml");
    let backup = write_yaml_atomically(&dest, "name: demo\n", "graph", ExpectedPrior::Unchecked)
        .expect("write");
    assert!(backup.is_none(), "fresh write ⇒ no backup");
    assert_eq!(
        std::fs::read_to_string(&dest).expect("readable"),
        "name: demo\n"
    );
    // No leaked temp file.
    let mut names: Vec<String> = std::fs::read_dir(tmp.path())
        .expect("read_dir")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    assert_eq!(names, vec!["demo.yaml".to_string()]);
}

#[tracing_test::traced_test]
#[test]
fn write_yaml_atomically_backs_up_and_warns_naming_the_file_kind() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.yaml");
    std::fs::write(&dest, "name: old\n").expect("seed");

    let backup = write_yaml_atomically(&dest, "name: new\n", "graph", ExpectedPrior::Unchecked)
        .expect("write");
    let bak = backup.expect("existing file ⇒ backup");
    assert_eq!(
        std::fs::read_to_string(&bak).expect("bak readable"),
        "name: old\n",
        "backup carries the OLD bytes"
    );
    assert_eq!(
        std::fs::read_to_string(&dest).expect("dest readable"),
        "name: new\n"
    );
    assert!(
        logs_contain("overwriting an existing graph"),
        "the warn must name the file_kind (graph)"
    );
    assert!(
        logs_contain("was backed up"),
        "the warn must announce the backup"
    );
}

#[test]
fn profile_artifact_writer_still_delegates_cleanly() {
    // The re-pointed profile-artifact caller keeps working through the shared
    // writer (no behavior change for the profiler path).
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.costs.yaml");
    let backup = write_profile_artifact_atomically(&dest, "version: 1\n").expect("write");
    assert!(backup.is_none());
    assert_eq!(
        std::fs::read_to_string(&dest).expect("readable"),
        "version: 1\n"
    );
}

// ==========================================================================
// Section C2 — the CONCURRENT-DESTINATION refusal.
//
// A surgical rewrite reads the destination and splices its exact bytes, so
// between that read and the `rename(2)` another writer can replace the file
// and an unconditional rename discards their work SILENTLY: the spliced
// document is still a valid graph, so nothing downstream notices. Every arm
// here drives the real writer over a real temp directory — the oracle is the
// FILE, never the return value alone.
// ==========================================================================

/// The window this closes, at its widest: the caller read `old`, someone else
/// saved `theirs`, and the write must be REFUSED with nothing touched.
#[test]
fn a_destination_that_changed_since_the_caller_read_it_is_refused_with_nothing_written() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.yaml");
    std::fs::write(&dest, "name: theirs\n").expect("seed");

    // The caller derived its rewrite from `name: old` — which is no longer
    // what is on disk.
    let err = write_yaml_atomically(
        &dest,
        "name: mine\n",
        "graph file",
        ExpectedPrior::Contents("name: old\n"),
    )
    .expect_err("a changed destination must be refused, never clobbered");
    let msg = err.to_string();

    // The refusal NAMES the file and the remedy, in an operator's words — no
    // internal jargon about digests or preconditions.
    assert!(
        msg.contains(&dest.display().to_string()),
        "the refusal must name the file that changed; got: {msg}"
    );
    assert!(
        msg.contains("changed on disk"),
        "the refusal must say WHAT went wrong; got: {msg}"
    );
    assert!(
        msg.contains("Re-run"),
        "the refusal must state the remedy; got: {msg}"
    );

    // THE POINT: their file survives, byte for byte.
    assert_eq!(
        std::fs::read_to_string(&dest).expect("readable"),
        "name: theirs\n",
        "the concurrent writer's content must be untouched"
    );
    // And nothing else was left behind — no `.bak` claiming a backup of a
    // write that never happened, no staged temp file.
    let mut names: Vec<String> = std::fs::read_dir(tmp.path())
        .expect("read_dir")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    assert_eq!(
        names,
        vec!["demo.yaml".to_string()],
        "a refused write must leave no .bak and no temp file"
    );
}

/// ANTI-TAUTOLOGY. Without this, "a changed destination is refused" is
/// satisfied by a writer that refuses EVERYTHING — and the whole feature would
/// be a `graph partition` that can never write.
#[tracing_test::traced_test]
#[test]
fn a_destination_that_is_exactly_what_the_caller_read_is_written_and_backed_up() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.yaml");
    std::fs::write(&dest, "name: old\n").expect("seed");

    let backup = write_yaml_atomically(
        &dest,
        "name: mine\n",
        "graph file",
        ExpectedPrior::Contents("name: old\n"),
    )
    .expect("an unchanged destination must be written");

    assert_eq!(
        std::fs::read_to_string(&dest).expect("readable"),
        "name: mine\n"
    );
    let bak = backup.expect("existing file ⇒ backup");
    assert_eq!(
        std::fs::read_to_string(&bak).expect("bak readable"),
        "name: old\n",
        "the backup carries the bytes that were VERIFIED and replaced"
    );
    assert!(logs_contain("was backed up"), "and the warn still fires");
}

/// A destination DELETED underneath is the same class as one rewritten: the
/// caller's splice describes a file that is no longer there, so writing it
/// would resurrect content somebody removed on purpose.
#[test]
fn a_destination_deleted_underneath_the_caller_is_refused_too() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.yaml");
    // Never created — the caller read it, someone else deleted it.

    let err = write_yaml_atomically(
        &dest,
        "name: mine\n",
        "graph file",
        ExpectedPrior::Contents("name: old\n"),
    )
    .expect_err("a vanished destination must be refused");
    assert!(err.to_string().contains("changed on disk"));
    assert!(
        !dest.exists(),
        "a refused write must not create the file it refused to write"
    );
}

/// `Unchecked` is the GENERATE-and-overwrite path (`graph profile`, `ros
/// attach`) — it makes no claim about the prior content, so it must behave
/// exactly as a write with no precondition check. Without this arm the check could
/// silently start refusing writes those verbs are supposed to make.
#[test]
fn an_unchecked_write_still_overwrites_a_destination_that_changed() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let dest = tmp.path().join("demo.costs.yaml");
    std::fs::write(&dest, "version: someone-elses\n").expect("seed");

    let backup = write_yaml_atomically(
        &dest,
        "version: 2\n",
        "cost artifact",
        ExpectedPrior::Unchecked,
    )
    .expect("an unchecked write must not be refused");
    assert!(backup.is_some(), "it still backs the prior file up");
    assert_eq!(
        std::fs::read_to_string(&dest).expect("readable"),
        "version: 2\n"
    );
}

/// The writer checks its precondition TWICE — once before anything is staged,
/// and again immediately before the `rename(2)` — and each check READS the
/// destination itself.
///
/// The freshness half is not decoration. Splitting reading from
/// checking and handing BOTH checks the same snapshot makes the second compare
/// bytes read before the staging step against a precondition it has already
/// satisfied: a pure no-op, inert against exactly the writer it exists to
/// catch. A test that only counts and orders the two calls
/// cannot see that. The read is folded
/// into the check so a stale snapshot is unspellable, and the walk
/// asserts that shape rather than a call count.
///
/// SCOPE: still STRUCTURAL, and deliberately so. The second check's
/// whole window is inside one function call (the temp-file write), so making
/// it observable would need a fault-injection seam in a shipping code path.
/// The behavioural arms above pin the check that matters — the one covering
/// the caller's whole read → derive → validate → prompt span, which for
/// `graph partition` includes an interactive y/N and is therefore unbounded.
///
/// The public `write_yaml_atomically` is a wrapper that fixes the prior-file
/// policy (back up + warn); the staging and both checks live in the private
/// `write_yaml_atomically_with`, which is the body this walks. The wrapper is
/// pinned to DELEGATE, so a second write path can never grow behind the
/// public name.
#[test]
fn the_writer_re_checks_its_precondition_immediately_before_the_rename() {
    let src = std::fs::read_to_string("src/graph_cmd.rs").expect("readable");
    let wrapper = code_only(fn_body(&src, "pub fn write_yaml_atomically("));
    assert!(
        wrapper.contains("write_yaml_atomically_with(dest, contents, file_kind, expected,"),
        "the public writer must delegate to the checked writer, not write on its own\n{wrapper}"
    );
    let body = code_only(fn_body(&src, "fn write_yaml_atomically_with("));

    // The checker must be the READ-AND-VERIFY function, so neither call site
    // can be handed bytes somebody else read.
    assert!(
        !body.contains("verify_unchanged("),
        "a check that takes bytes as an argument can be handed a STALE snapshot — \
         the read must be folded in\n{body}"
    );
    let checks: Vec<usize> = body
        .match_indices("read_and_verify_destination(")
        .map(|(i, _)| i)
        .collect();
    assert_eq!(
        checks.len(),
        2,
        "the precondition must be held BEFORE staging and AGAIN before the rename; \
         found {} check(s)",
        checks.len()
    );

    let stage = body
        .find("fs::write(&tmp,")
        .expect("the writer must stage the contents into a temp file");
    let rename = body
        .find("fs::rename(&tmp,")
        .expect("the writer must publish via an atomic rename");
    assert!(
        checks[0] < stage,
        "the first check must run before anything is staged"
    );
    assert!(
        checks[1] > stage && checks[1] < rename,
        "the second check must run between staging and the rename \
         (check at {}, stage at {stage}, rename at {rename})",
        checks[1]
    );

    // AND what each call VERIFIES. The inert class is
    // reintroducible by ARGUMENT: passing `ExpectedPrior::Unchecked` at the
    // second site keeps the count, the ordering and the function name intact
    // while the staging-window protection silently dies — and no behavioural
    // arm can see it, because a test can only seed a concurrent change BEFORE
    // the writer is called, which the FIRST check catches. So the walk pins
    // that both calls carry the caller's `expected`, and that the literal
    // never appears in this body at all.
    assert_eq!(
        body.matches("read_and_verify_destination(dest, file_kind, expected)")
            .count(),
        2,
        "both checks must hold the CALLER's precondition\n{body}"
    );
    assert!(
        !body.contains("ExpectedPrior::"),
        "naming an ExpectedPrior variant inside the writer means a call site is \
         verifying something other than what the caller asked for\n{body}"
    );
}

/// ANTI-TAUTOLOGY for the walk above: `fn_body` must really isolate ONE
/// function, or every ordering assertion it makes is over the whole file and
/// means nothing.
#[test]
fn the_function_body_extractor_isolates_one_function() {
    let src = "fn a() {\n  let x = 1;\n}\nfn b() {\n  let y = 2;\n}\n";
    let a = fn_body(src, "fn a(");
    assert!(a.contains("let x"), "the named body must be present: {a:?}");
    assert!(
        !a.contains("let y"),
        "the NEXT function's body must not be: {a:?}"
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

// ==========================================================================
// Section C3 — every emitted scalar re-parses as a STRING.
// ==========================================================================

/// The corpus of tokens a YAML reader can resolve as something other than a
/// string. `1e3`, `1.0`, `0x10` and friends would survive a rule
/// that rejects only an ALL-DIGIT run and the bare boolean/null keywords.
const AMBIGUOUS_SCALARS: &[&str] = &[
    // The all-digit / keyword family even a narrow rule covers — kept so
    // a narrowing of the rule cannot pass by dropping them.
    "007",
    "00",
    "true",
    "False",
    "null",
    "NULL",
    "~",
    "yes",
    "no",
    "on",
    "off",
    "y",
    "n",
    // Floats, exponents and radix prefixes — the family that was emitted PLAIN.
    "1.0",
    "0.5",
    "1e3",
    "1E5",
    "2e2",
    "1e+3",
    "0x10",
    "0xFF",
    "0o17",
    "0b101",
    "1_000",
    // A date shape: `serde_yaml` calls it a string, PyYAML returns a date.
    "2024-01-01",
];

/// Tokens that must stay PLAIN. Without this half, "quote the ambiguous ones"
/// is satisfied by quoting everything — which would churn every graph file in
/// the tree and make the emitted YAML unreadable.
const PLAIN_SCALARS: &[&str] = &[
    "camera", "relay", "sink", "ticker", "n0", "p0", "grp_a", "1.2.3", "0.0.0.0", "0bad", "0xyz",
    "e5", "1e", "_private", "a-b", "node_1",
];

/// Read a scalar out of an UNTYPED `serde_yaml::Value` walk of `doc`, by
/// following `path` through mappings and sequences.
///
/// The oracle is deliberately the UNTYPED view. MEASURED on the shipping
/// `serde_yaml` 0.9.34: its TYPED `String` deserialization is lenient enough
/// that `1e3` comes back as `"1e3"` even unquoted, so a test that re-parsed
/// through `parse_graph_raw` would pass against an emitter that leaves it unquoted and pin
/// nothing. The untyped walk is what every other reader sees.
fn scalar_at(doc: &serde_yaml::Value, path: &[&str]) -> serde_yaml::Value {
    let mut cursor = doc;
    for step in path {
        cursor = match cursor {
            serde_yaml::Value::Mapping(m) => m
                .get(serde_yaml::Value::String((*step).to_string()))
                .unwrap_or_else(|| panic!("no key {step:?} in {cursor:?}")),
            serde_yaml::Value::Sequence(s) => {
                let idx: usize = step.parse().expect("sequence step must be an index");
                s.get(idx)
                    .unwrap_or_else(|| panic!("no index {idx} in {cursor:?}"))
            }
            other => panic!("cannot walk {step:?} into {other:?}"),
        };
    }
    cursor.clone()
}

#[test]
fn every_ambiguous_group_scalar_re_parses_as_the_string_it_was_given() {
    for token in AMBIGUOUS_SCALARS {
        // The token in BOTH positions the process-groups emitter renders: the
        // block-mapping KEY and a flow-sequence MEMBER.
        let g = groups(&[(token, &[*token, "plain_member"])]);
        let input = "name: demo\nnodes:\n  - id: n\n    type: n\n";
        let out = rewrite_process_groups_block(input, &g, "amb.yaml").expect("rewrite");
        let doc: serde_yaml::Value = serde_yaml::from_str(&out)
            .unwrap_or_else(|e| panic!("{token:?} broke the document: {e}\n{out}"));

        assert_eq!(
            scalar_at(&doc, &["process_groups"])
                .as_mapping()
                .expect("a mapping")
                .keys()
                .next()
                .cloned()
                .expect("one key"),
            serde_yaml::Value::String((*token).to_string()),
            "the group NAME {token:?} must re-parse as that exact string, not as a number, \
             boolean, null or date\n{out}"
        );
        assert_eq!(
            scalar_at(&doc, &["process_groups", token, "0"]),
            serde_yaml::Value::String((*token).to_string()),
            "the group MEMBER {token:?} must re-parse as that exact string\n{out}"
        );

        // AND the token was actually QUOTED in the bytes.
        //
        // This is not belt-and-braces — it is the only assertion that can see
        // part of the corpus, and leaving it out made one entry VACUOUS.
        // Neutering `is_yaml_date` leaves the
        // whole suite green, because `serde_yaml` resolves `2024-01-01` as a
        // STRING and so the re-parse oracle above cannot tell quoted from
        // plain for it. PyYAML returns a `datetime.date`, which is the reader
        // the rule is for and the one no test here can run. The two oracles
        // therefore cover different halves of the corpus: the re-parse is the
        // OUTCOME wherever `serde_yaml` agrees the token is ambiguous, and
        // this is the MECHANISM everywhere else.
        assert!(
            out.contains(&format!("  \"{token}\": [\"{token}\", plain_member]")),
            "{token:?} must be emitted QUOTED in both positions\n{out}"
        );
    }
}

/// Tokens carrying a CONTROL CHARACTER. A quoted form that
/// escapes only `\\` and `"` lets a literal newline inside `"..."` FOLD to a
/// space (a silently different value), and a `NUL`-class byte makes the document
/// unreadable outright.
const CONTROL_SCALARS: &[&str] = &[
    "a\nb",       // the folding case — unescaped, it comes back as `a b`
    "a\r\nb",     // CRLF
    "a\tb",       // tab
    "a\u{0}b",    // NUL — not in YAML's printable set at all
    "a\u{1}b",    // SOH — the \xNN arm
    "a\u{7f}b",   // DEL
    "a\u{85}b",   // NEL (C1)
    "a\u{2028}b", // LINE SEPARATOR — a line break to YAML 1.1
    "a\u{b}b",    // vertical tab
    "a\u{1b}b",   // ESC
];

#[test]
fn a_control_character_survives_the_splice_byte_for_byte() {
    for token in CONTROL_SCALARS {
        // Both positions the process-groups emitter renders a free value in.
        let g = groups(&[(token, &[*token, "plain_member"])]);
        let input = "name: demo\nnodes:\n  - id: n\n    type: n\n";
        let out = rewrite_process_groups_block(input, &g, "ctrl.yaml").expect("rewrite");

        // (a) The document must still PARSE. A raw NUL simply is not YAML, so
        // unescaped, this half fails outright for several of these.
        let doc: serde_yaml::Value = serde_yaml::from_str(&out)
            .unwrap_or_else(|e| panic!("{token:?} made the document unreadable: {e}\n{out:?}"));

        // (b) And the value must come back BYTE for byte — unescaped, the newline
        // case parses perfectly well and returns `a b`, which is the
        // quieter and worse half of this failure.
        assert_eq!(
            scalar_at(&doc, &["process_groups"])
                .as_mapping()
                .expect("a mapping")
                .keys()
                .next()
                .cloned()
                .expect("one key"),
            serde_yaml::Value::String((*token).to_string()),
            "the group NAME {token:?} must round-trip unchanged\n{out:?}"
        );
        assert_eq!(
            scalar_at(&doc, &["process_groups", token, "0"]),
            serde_yaml::Value::String((*token).to_string()),
            "the group MEMBER {token:?} must round-trip unchanged\n{out:?}"
        );

        // (c) ANTI-TAUTOLOGY: the escape really is an escape — no RAW control
        // character reached the file. Without this, an emitter that wrote the
        // byte verbatim could still pass (a) and (b) on a parser more
        // forgiving than the spec.
        assert!(
            !out.chars().any(|c| c.is_control() && c != '\n'),
            "no raw control character may reach the document (the only \
             literal newlines are the line terminators)\n{out:?}"
        );
    }
}

/// The escape must not disturb what was already correct: a token that needs
/// QUOTING but carries no control character renders exactly as before.
#[test]
fn a_quoted_token_without_control_characters_is_unchanged() {
    let g = groups(&[("grp_x", &["a,b", "c: d", "say \"hi\"", "back\\slash"])]);
    let input = "name: demo\nnodes:\n  - id: n\n    type: n\n";
    let out = rewrite_process_groups_block(input, &g, "quoted.yaml").expect("rewrite");
    assert!(
        out.contains(r#"  grp_x: ["a,b", "c: d", "say \"hi\"", "back\\slash"]"#),
        "the pre-existing quoting must be byte-unchanged\n{out}"
    );
    let parsed = parse_graph_raw(&out).expect("re-parse");
    assert_eq!(
        parsed.process_groups.get("grp_x"),
        Some(&vec![
            "a,b".to_string(),
            "c: d".to_string(),
            "say \"hi\"".to_string(),
            "back\\slash".to_string()
        ])
    );
}

#[test]
fn an_ordinary_identifier_is_still_emitted_plain() {
    for token in PLAIN_SCALARS {
        let g = groups(&[(token, &[*token])]);
        let input = "name: demo\nnodes:\n  - id: n\n    type: n\n";
        let out = rewrite_process_groups_block(input, &g, "plain.yaml").expect("rewrite");
        assert!(
            out.contains(&format!("  {token}: [{token}]")),
            "{token:?} needs no quoting and must not acquire any\n{out}"
        );
    }
}

// ==========================================================================
// Section D — remove_process_group_order_block (pure string surgery).
// The emitted process_groups LISTING order IS the rank order, so the emit
// path strips any explicit (stale-prone) order list. By design the fn is
// PURE (no warn) — the loud removal warn fires only at the WRITE-COMMITTING
// sites (pinned in graph_partition_test.rs), so a dry-run preview never
// claims a removal that was not written.
// ==========================================================================

#[tracing_test::traced_test]
#[test]
fn order_removal_single_line_collapses_the_doubled_blank_seam_and_is_silent() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "\
name: demo
process_groups:
  g1: [a]

process_group_order: [g1]

nodes:
  - id: a
    type: a
";
    // Hand oracle: the order line is gone and exactly ONE blank separator
    // survives between the groups block and `nodes:`.
    let expected = "\
name: demo
process_groups:
  g1: [a]

nodes:
  - id: a
    type: a
";
    let (out, removed) = remove_process_group_order_block(input, "demo.yaml").expect("remove");
    assert_eq!(out, expected);
    assert_eq!(
        removed.as_deref(),
        Some("process_group_order: [g1]\n"),
        "the removed text is the exact block bytes"
    );
    // By design the pure removal NEVER warns; write-committing callers do.
    assert!(
        !logs_contain("removed the process_group_order block"),
        "the pure removal fn must be silent; the warn belongs to the write sites"
    );
}

#[test]
fn order_removal_handles_multi_line_block_sequence() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "\
name: demo
process_group_order:
  - g2
  - g1
process_groups:
  g1: [a]
  g2: [b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    let expected = "\
name: demo
process_groups:
  g1: [a]
  g2: [b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    let (out, removed) = remove_process_group_order_block(input, "demo.yaml").expect("remove");
    assert_eq!(out, expected);
    assert_eq!(
        removed.as_deref(),
        Some("process_group_order:\n  - g2\n  - g1\n"),
        "the whole indented block is captured"
    );
}

#[test]
fn order_removal_at_eof_leaves_the_rest_untouched() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "\
name: demo
nodes:
  - id: a
    type: a
process_group_order: [g1]
";
    let expected = "\
name: demo
nodes:
  - id: a
    type: a
";
    let (out, removed) = remove_process_group_order_block(input, "demo.yaml").expect("remove");
    assert_eq!(out, expected);
    assert!(removed.is_some());
}

#[tracing_test::traced_test]
#[test]
fn order_removal_absent_is_byte_identical_and_silent() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "name: demo\nnodes:\n  - id: a\n    type: a\n";
    let (out, removed) = remove_process_group_order_block(input, "demo.yaml").expect("no-op");
    assert_eq!(out, input, "absent key => byte-identical passthrough");
    assert!(removed.is_none());
    assert!(
        !logs_contain("removed the process_group_order block"),
        "no removal => no warn"
    );
}

#[test]
fn order_removal_duplicate_keys_refused_with_diagnosis() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "\
name: demo
process_group_order: [g1]
nodes:
  - id: a
    type: a
process_group_order: [g2]
";
    let err = remove_process_group_order_block(input, "twin_order.yaml")
        .expect_err("two top-level process_group_order keys must be refused")
        .to_string();
    assert!(
        err.contains("twin_order.yaml")
            && err.contains("process_group_order")
            && err.contains("ambiguous"),
        "the error names the file, the key, and the ambiguity; got: {err}"
    );
}

// ==========================================================================
// Section E — column-0 comments/blanks inside a
// block must NOT terminate the scan. If a `# divider` between entries
// ended the block early, the rewrite would replace only the first half and the
// orphaned indented survivors would re-parse as members of the NEW block — silent
// corruption, the exact class the surgical splice exists to prevent.
// ==========================================================================

#[test]
fn divider_inside_block_is_replaced_whole_not_corrupted() {
    // The reported corruption shape: a COLUMN-0 `#` divider BETWEEN the
    // entries of the existing block, plus a trailing column-0 comment that
    // belongs to the NEXT section.
    let input = "\
name: demo
process_groups:
  old_a: [a]
# --- perception half (COLUMN-0 divider inside the block) ---
  old_b: [b]

# --- old partition halves below ---
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    let g = groups(&[("grp_a", &["a", "b"])]);
    let out = rewrite_process_groups_block(input, &g, "demo.yaml").expect("rewrite");

    // Hand oracle: the WHOLE old block — both entries AND the interior
    // divider — is replaced (the divider annotated entries that no longer
    // exist, so it is consumed WITH the old block: the documented choice);
    // the trailing column-0 comment BEFORE `nodes:` survives (it belongs to
    // the next section).
    let expected = "\
name: demo
process_groups:
  grp_a: [a, b]

# --- old partition halves below ---
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    assert_eq!(out, expected);

    // The corruption-class kill shot: the re-parsed file carries EXACTLY the
    // emitted groups — no orphaned `old_b` leaking into the new block.
    let parsed = parse_graph_raw(&out).expect("re-parses");
    assert_eq!(
        shape(&parsed.process_groups),
        shape(&g),
        "the orphaned post-divider entry must not re-parse INTO the new block"
    );
}

#[test]
fn trailing_comment_and_blanks_before_next_key_stay_outside() {
    let input = "\
name: demo
process_groups:
  old_a: [a]

# trailing comment — belongs to the NEXT section, must survive byte-identical

nodes:
  - id: a
    type: a
";
    let out = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "demo.yaml")
        .expect("rewrite");
    let expected = "\
name: demo
process_groups:
  grp_a: [a]

# trailing comment — belongs to the NEXT section, must survive byte-identical

nodes:
  - id: a
    type: a
";
    assert_eq!(out, expected);
}

#[test]
fn comment_at_eof_after_block_stays_outside() {
    let input = "\
name: demo
nodes:
  - id: a
    type: a
process_groups:
  old_a: [a]
# eof note
";
    let out = rewrite_process_groups_block(input, &groups(&[("grp_a", &["a"])]), "demo.yaml")
        .expect("rewrite");
    assert_eq!(
        out,
        "\
name: demo
nodes:
  - id: a
    type: a
process_groups:
  grp_a: [a]
# eof note
",
        "an EOF comment (nothing indented follows) stays outside the span"
    );
}

#[test]
fn blank_and_comment_mix_inside_block_is_consumed_with_it() {
    let input = "\
name: demo
process_groups:
  old_a: [a]

# mid divider

  old_b: [b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    let out = rewrite_process_groups_block(input, &groups(&[("grp_ab", &["a", "b"])]), "d.yaml")
        .expect("rewrite");
    let expected = "\
name: demo
process_groups:
  grp_ab: [a, b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    assert_eq!(
        out, expected,
        "blank+comment runs between entries belong to the block (consumed on replace)"
    );
    let parsed = parse_graph_raw(&out).expect("re-parses");
    assert_eq!(parsed.process_groups.len(), 1);
}

#[test]
fn order_removal_with_interior_divider_removes_whole_block() {
    use cerulion_cli_engine::partition_emit::remove_process_group_order_block;
    let input = "\
name: demo
process_group_order:
  - g2
# rank rationale (column-0, inside the block)
  - g1
process_groups:
  g1: [a]
  g2: [b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
";
    let (out, removed) = remove_process_group_order_block(input, "demo.yaml").expect("remove");
    assert_eq!(
        out,
        "\
name: demo
process_groups:
  g1: [a]
  g2: [b]
nodes:
  - id: a
    type: a
  - id: b
    type: b
",
        "the shared scanner fix applies to the removal too — nothing orphaned"
    );
    assert_eq!(
        removed.as_deref(),
        Some(
            "process_group_order:\n  - g2\n# rank rationale (column-0, inside the block)\n  - g1\n"
        ),
        "the removed text carries the interior divider with the block"
    );
}

// ==========================================================================
// The `level_assignments:` splice
// ([`rewrite_level_assignments_block`]) — same surgical discipline as the
// process-groups rewrite. Byte oracles, hand-built.
// ==========================================================================

/// Build an assignments map from `(node, level)` literals (insertion = graph
/// order — the order the emitter renders).
fn assignments(pairs: &[(&str, usize)]) -> IndexMap<String, usize> {
    pairs.iter().map(|(n, l)| (n.to_string(), *l)).collect()
}

#[test]
fn level_assignments_insert_lands_before_nodes_after_spliced_groups() {
    use cerulion_cli_engine::partition_emit::rewrite_level_assignments_block;
    // The emit order: groups spliced FIRST (inserted before `nodes:`), the
    // level block second (also before `nodes:` ⇒ it lands BETWEEN the two).
    let raw = "# header comment\nname: g\nprefix: p\n\nnodes:\n  - id: a\n    type: t\n";
    let with_groups =
        rewrite_process_groups_block(raw, &groups(&[("g0", &["a"])]), "test").expect("groups");
    let out = rewrite_level_assignments_block(&with_groups, &assignments(&[("a", 0)]), "test")
        .expect("levels");
    assert_eq!(
        out,
        "# header comment\nname: g\nprefix: p\n\nprocess_groups:\n  g0: [a]\n\n\
         level_assignments:\n  a: 0\n\nnodes:\n  - id: a\n    type: t\n",
        "groups block, then level block, then nodes — every other byte preserved"
    );
}

#[test]
fn level_assignments_replace_is_surgical_and_preserves_comments() {
    use cerulion_cli_engine::partition_emit::rewrite_level_assignments_block;
    let raw = "name: g\n\n# above the block — belongs to the NEXT section? no: trailing\n\
               level_assignments:\n  a: 0\n  b: 9\n\n# trailing comment survives\nnodes:\n  - id: a\n";
    let out = rewrite_level_assignments_block(raw, &assignments(&[("a", 0), ("b", 1)]), "test")
        .expect("replace");
    assert_eq!(
        out,
        "name: g\n\n# above the block — belongs to the NEXT section? no: trailing\n\
         level_assignments:\n  a: 0\n  b: 1\n\n# trailing comment survives\nnodes:\n  - id: a\n",
        "only the block's bytes change; comments above and below survive"
    );
}

#[test]
fn level_assignments_duplicate_key_refused_naming_lines() {
    use cerulion_cli_engine::partition_emit::rewrite_level_assignments_block;
    let raw = "level_assignments:\n  a: 0\nlevel_assignments:\n  a: 1\nnodes:\n  - id: a\n";
    let err = rewrite_level_assignments_block(raw, &assignments(&[("a", 0)]), "dup.yaml")
        .expect_err("duplicate top-level keys are ambiguous")
        .to_string();
    assert!(
        err.contains("level_assignments") && err.contains("dup.yaml") && err.contains("2"),
        "names the key, the file, and the count; got: {err}"
    );
}

#[test]
fn level_assignments_empty_map_is_internal_error() {
    use cerulion_cli_engine::partition_emit::rewrite_level_assignments_block;
    let err = rewrite_level_assignments_block("nodes:\n  - id: a\n", &IndexMap::new(), "t")
        .expect_err("an empty assignment is an internal error")
        .to_string();
    assert!(err.contains("internal error"), "got: {err}");
}

#[test]
fn refine_inputs_adapter_is_a_two_field_projection() {
    use cerulion_cli_engine::partition_emit::refine_inputs_from_costs;
    use std::collections::BTreeMap;
    let mut node_p50_ns = BTreeMap::new();
    node_p50_ns.insert("cam".to_string(), 512_000u64);
    node_p50_ns.insert("imu".to_string(), 4_000u64);
    let mut edge_rate_mhz = BTreeMap::new();
    edge_rate_mhz.insert(("imu".to_string(), "filt".to_string()), 1_000_000u64);
    let costs = cerulion_core::graph::PartitionCosts {
        node_p50_ns,
        edge_rate_mhz,
        hop: cerulion_core::graph::HopCosts {
            intra_ns: 1_000,
            cross_ns: 6_800,
        },
    };
    let inputs = refine_inputs_from_costs(&costs);
    assert_eq!(inputs.node_cost_ns.get("cam"), Some(&512_000));
    assert_eq!(inputs.node_cost_ns.get("imu"), Some(&4_000));
    assert_eq!(inputs.node_cost_ns.len(), 2);
    assert_eq!(
        inputs
            .edge_rate_mhz
            .get(&("imu".to_string(), "filt".to_string())),
        Some(&1_000_000)
    );
    assert_eq!(
        inputs.edge_rate_mhz.len(),
        1,
        "hop is dropped — two fields only"
    );
}

// ==========================================================================
// Section D — `block` edges are a HARD CO-LOCATION CONSTRAINT.
//
// The default (`cerulion graph run` on an unpartitioned graph) derives
// its partition from these two functions. `block` defers the producer's tick
// through a mirror the consumer's subscriber decrements, and that mirror is a
// PROCESS-LOCAL `Arc<AtomicU64>` — so a split `block` edge does not degrade the
// policy, it refuses to build: `subgraph_for` drops the foreign producer, and
// the consumer's worker dies at `GraphTopology::validate`.
//
// Oracles are hand-written group maps. Every fixture builds REAL `InputMeta`
// (the `demo_graph` fixture above uses `NodeInfo::with_meta(vec![], vec![])`,
// which makes `GraphTopology::build` apply its missing-meta fallback — a
// copy-paste of it would silently test nothing).
// ==========================================================================

use cerulion_core::graph::node::{BackpressurePolicy, InputMeta};
use cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH;

/// One `InputMeta` with a declared policy (everything else at the
/// topology-safe defaults `source_entry_infos` uses).
fn meta(name: &str, trigger: bool, backpressure: BackpressurePolicy) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0,
        trigger,
        depth: DEFAULT_CONSUMER_DEPTH,
        backpressure,
        expect_within_ms: None,
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

fn inp(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

fn node(id: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: id.to_string(),
        inputs,
        outputs,
    }
}

fn config_of(nodes: Vec<NodeDef>, multi: Vec<String>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "bp".to_string(),
        prefix: "p".to_string(),
        nodes,
        multi_publisher_topics: multi,
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    }
}

/// `prod -(block)-> cons`, plus a disconnected `spare`. `cons`'s input is
/// TRIGGER-marked unless `trigger` is false.
fn block_chain(trigger: bool) -> (GraphConfig, IndexMap<String, NodeInfo>, TriggerEdges) {
    let config = config_of(
        vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
            node("spare", vec![], vec![]),
        ],
        Vec::new(),
    );
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert(
        "cons".to_string(),
        NodeInfo::with_meta(
            vec![meta("gate", trigger, BackpressurePolicy::Block)],
            vec![],
        ),
    );
    infos.insert("spare".to_string(), NodeInfo::with_meta(vec![], vec![]));
    let mut edges = TriggerEdges::new();
    if trigger {
        edges.insert("cons", "/p/prod/cmd");
    }
    (config, infos, edges)
}

/// HEADLINE: the no-costs BASELINE — the literal default path — co-locates a
/// `block` producer with its `block` consumer. Before the co-location fix this path
/// performed ZERO unions, so it split every `block` edge in every unprofiled
/// graph.
#[test]
fn baseline_co_locates_a_block_producer_with_its_consumer() {
    let (config, infos, edges) = block_chain(true);
    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000)
        .expect("baseline derivation must succeed");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            ),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
        ],
        "the block edge must be co-located; the unrelated node stays a singleton"
    );
}

/// The seed is INDEPENDENT of the greedy fusion: a cost artifact whose edge
/// rate is 0 makes `coupling == 0`, which the loop rejects as `Unprofitable` —
/// yet the pair must still be one group.
#[test]
fn a_zero_rate_block_edge_is_co_located_on_the_fused_path_too() {
    let (config, infos, edges) = block_chain(true);
    let artifact = ProfileArtifact {
        version: PROFILE_ARTIFACT_VERSION,
        graph: "bp".to_string(),
        window_ns: 5_000_000_000,
        nodes: [
            ("prod".to_string(), 1_000),
            ("cons".to_string(), 1_000),
            ("spare".to_string(), 1_000),
        ]
        .into_iter()
        .collect(),
        // Rate 0 ⇒ coupling 0 ⇒ the greedy loop would never fuse this edge.
        edges: vec![ProfileEdge {
            producer: "prod".to_string(),
            consumer: "cons".to_string(),
            rate_mhz: 0,
        }],
        isolated: vec![],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let out = derive_emit_groups(&config, &infos, &edges, Some(&artifact), 100_000)
        .expect("cost-aware derivation must succeed");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            ),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
        ],
        "a block edge is a CONSTRAINT, not a candidate — a zero-coupling edge the greedy \
         loop rejects must still be co-located"
    );
}

/// The rule does not ride `is_triggering`. A NON-trigger `block` input is not
/// even a fusion candidate (the greedy loop's candidate set is trigger edges
/// only), which is the shape both block fixtures carry.
#[test]
fn a_non_trigger_block_input_is_co_located() {
    let (config, infos, edges) = block_chain(false);
    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            ),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
        ],
        "a non-trigger block input is never a fusion CANDIDATE, so only the seed can \
         co-locate it"
    );
}

/// THE INVERTED ARM. A MIXED topic (one `block` consumer + one `drop_oldest`
/// consumer) MUST be co-located — the WHOLE flow, sibling included.
///
/// The refusing gate is per-CONSUMER (`Block` on a producer-less flow) and the
/// mixed→`drop_oldest` degrade runs strictly LATER, at the runtime's install
/// site — so on a split mixed topic the degrade can never rescue anything, the
/// build is already dead. And in the block consumer's worker subgraph the
/// sibling `drop_oldest` consumer is a FOREIGN node and is filtered out, so
/// global mixedness is structurally invisible to the check that refuses.
/// Keying the seed on `is_all_block()` would leave this legal, documented,
/// separately-tested shape falling straight through into a split that cannot build.
///
/// The SIBLING is not optional, and this arm is what says so. Co-locate only
/// `{prod, blocker}` and the worker holding them reads consumers = `[block]`
/// ⇒ `is_all_block()` TRUE ⇒ it INSTALLS the defer, while the monolith sees
/// the mixed flow and DEGRADES to `drop_oldest`. The mixed-topic warn fires in
/// neither process, so one graph runs two semantics in silence and `lossy` is
/// throttled to `blocker`'s drain rate — the decision-K starvation the degrade
/// exists to prevent. So the oracle is the WHOLE flow in ONE group.
#[test]
fn a_mixed_block_topic_must_also_be_co_located() {
    let config = config_of(
        vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![]),
        ],
        Vec::new(),
    );
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert(
        "blocker".to_string(),
        NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
    );
    infos.insert(
        "lossy".to_string(),
        NodeInfo::with_meta(
            vec![meta("watch", true, BackpressurePolicy::DropOldest)],
            vec![],
        ),
    );
    let mut edges = TriggerEdges::new();
    edges.insert("blocker", "/p/prod/cmd");
    edges.insert("lossy", "/p/prod/cmd");

    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");
    let groups = shape(&out);
    let owner = |node: &str| -> String {
        groups
            .iter()
            .find(|(_, m)| m.iter().any(|x| x == node))
            .map(|(n, _)| n.clone())
            .unwrap_or_else(|| panic!("{node} is unplaced"))
    };
    assert_eq!(
        owner("prod"),
        owner("blocker"),
        "a MIXED block topic must still co-locate its producer with its BLOCK consumer — \
         the refusing gate is per-consumer and never sees the mixedness"
    );
    // THE SIBLING TOO. Its own seed claim, asserted directly rather than left
    // to an accident of the levelization repair: in this 3-node shape the group
    // {prod, blocker} owns a contiguous band {0,1} with no bridge, so no repair
    // ever runs and a seed that excluded `lossy` leaves it a singleton.
    assert_eq!(
        owner("prod"),
        owner("lossy"),
        "the non-`block` SIBLING of a mixed topic must be co-located too — split off, the \
         worker holding prod+blocker reads its flow as all-block and INSTALLS the defer \
         while --single-process degrades it"
    );
    assert!(
        groups.iter().any(|(_, m)| m.contains(&"prod".to_string())
            && m.contains(&"blocker".to_string())
            && m.contains(&"lossy".to_string())),
        "expected ONE group holding the whole flow (prod, blocker, lossy); got {groups:?}"
    );
}

/// The mixed-topic constraint is the FLOW, not a blanket "pull everything in":
/// a node that consumes a DIFFERENT topic is untouched by the seed.
///
/// Without this, `a_mixed_block_topic_must_also_be_co_located` is
/// satisfied by a seed that co-locates the whole graph.
#[test]
fn a_mixed_block_topic_pulls_in_its_own_flow_and_nothing_else() {
    let config = config_of(
        vec![
            node("prod", vec![], vec![out("cmd")]),
            node("blocker", vec![inp("gate", "prod/cmd")], vec![]),
            node("lossy", vec![inp("watch", "prod/cmd")], vec![out("aux")]),
            node("bystander", vec![inp("aux_in", "lossy/aux")], vec![]),
        ],
        Vec::new(),
    );
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert(
        "blocker".to_string(),
        NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
    );
    infos.insert(
        "lossy".to_string(),
        NodeInfo::with_meta(
            vec![meta("watch", true, BackpressurePolicy::DropOldest)],
            vec![],
        ),
    );
    infos.insert(
        "bystander".to_string(),
        NodeInfo::with_meta(
            vec![meta("aux_in", true, BackpressurePolicy::DropOldest)],
            vec![],
        ),
    );
    let mut edges = TriggerEdges::new();
    edges.insert("blocker", "/p/prod/cmd");
    edges.insert("lossy", "/p/prod/cmd");
    edges.insert("bystander", "/p/lossy/aux");

    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");
    let groups = shape(&out);
    let owner = |node: &str| -> String {
        groups
            .iter()
            .find(|(_, m)| m.iter().any(|x| x == node))
            .map(|(n, _)| n.clone())
            .unwrap_or_else(|| panic!("{node} is unplaced"))
    };
    assert_eq!(owner("prod"), owner("blocker"), "the block edge");
    assert_eq!(owner("prod"), owner("lossy"), "the mixed sibling");
    assert_ne!(
        owner("prod"),
        owner("bystander"),
        "a node on a DIFFERENT topic carries no co-location constraint; got {groups:?}"
    );
}

/// `multi_publisher_topics`: EVERY in-graph producer of a `block` topic
/// receives the same `BlockDeferEdge`, so the constraint is the whole flow —
/// both producers AND the consumer in ONE group.
#[test]
fn a_multi_publisher_block_topic_co_locates_every_producer() {
    let mut a = node("pa", vec![], vec![out("cmd")]);
    a.outputs[0].topic = Some("/shared/tf".to_string());
    let mut b = node("pb", vec![], vec![out("cmd")]);
    b.outputs[0].topic = Some("/shared/tf".to_string());
    let config = config_of(
        vec![a, b, node("cons", vec![inp("gate", "/shared/tf")], vec![])],
        vec!["/shared/tf".to_string()],
    );
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert("pa".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert("pb".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert(
        "cons".to_string(),
        NodeInfo::with_meta(vec![meta("gate", true, BackpressurePolicy::Block)], vec![]),
    );
    let mut edges = TriggerEdges::new();
    edges.insert("cons", "/shared/tf");

    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");
    let groups = shape(&out);
    assert_eq!(
        groups.len(),
        1,
        "both producers AND the block consumer must land in ONE group; got {groups:?}"
    );
    let mut members = groups[0].1.clone();
    members.sort();
    assert_eq!(
        members,
        vec!["cons".to_string(), "pa".to_string(), "pb".to_string()]
    );
}

/// The cost profile isolates a node that missed its fire target and keeps it a
/// SINGLETON. A `block` consumer is by construction the slow node the producer
/// is being deferred for, so it is the single most likely member of that set —
/// and co-location must WIN, because isolation is a cost policy while `block`
/// is a correctness constraint whose split cannot build.
#[test]
fn co_location_overrides_an_isolated_consumer() {
    let (config, infos, edges) = block_chain(true);
    let artifact = ProfileArtifact {
        version: PROFILE_ARTIFACT_VERSION,
        graph: "bp".to_string(),
        window_ns: 5_000_000_000,
        // `cons` is isolated ⇒ it carries NO cost entry.
        nodes: [("prod".to_string(), 1_000), ("spare".to_string(), 1_000)]
            .into_iter()
            .collect(),
        // An isolated node's edges carry no meaning to the cost model, and
        // the artifact validator refuses one that names it — the seed is the
        // ONLY thing that can co-locate this pair.
        edges: vec![],
        isolated: vec!["cons".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let out =
        derive_emit_groups(&config, &infos, &edges, Some(&artifact), 100_000).expect("derive");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            ),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
        ],
        "an ISOLATED block consumer must still be co-located — a singleton here cannot build"
    );
}

/// The twin: an isolated PRODUCER. Same decision, other end of the edge.
#[test]
fn co_location_overrides_an_isolated_producer() {
    let (config, infos, edges) = block_chain(true);
    let artifact = ProfileArtifact {
        version: PROFILE_ARTIFACT_VERSION,
        graph: "bp".to_string(),
        window_ns: 5_000_000_000,
        nodes: [("cons".to_string(), 1_000), ("spare".to_string(), 1_000)]
            .into_iter()
            .collect(),
        edges: vec![],
        isolated: vec!["prod".to_string()],
        hop: None,
        derived_budget_ns: None,
        profile_cores: None,
    };
    let out =
        derive_emit_groups(&config, &infos, &edges, Some(&artifact), 100_000).expect("derive");
    assert_eq!(
        shape(&out),
        vec![
            (
                "grp_prod".to_string(),
                vec!["prod".to_string(), "cons".to_string()]
            ),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
        ],
        "an ISOLATED block producer must still be co-located with its block consumer"
    );
}

/// ANTI-OVER-CONSTRAINT CONTROL: a graph with NO `block` input derives exactly
/// what it derived before the co-location fix. Without this every arm above is satisfied
/// by a seed that co-locates everything.
#[test]
fn a_graph_with_no_block_input_derives_the_untouched_baseline() {
    let config = config_of(
        vec![
            node("prod", vec![], vec![out("cmd")]),
            node("cons", vec![inp("gate", "prod/cmd")], vec![]),
            node("spare", vec![], vec![]),
        ],
        Vec::new(),
    );
    let mut infos: IndexMap<String, NodeInfo> = IndexMap::new();
    infos.insert("prod".to_string(), NodeInfo::with_meta(vec![], vec![]));
    infos.insert(
        "cons".to_string(),
        NodeInfo::with_meta(
            vec![meta("gate", true, BackpressurePolicy::DropOldest)],
            vec![],
        ),
    );
    infos.insert("spare".to_string(), NodeInfo::with_meta(vec![], vec![]));
    let mut edges = TriggerEdges::new();
    edges.insert("cons", "/p/prod/cmd");

    let out = derive_emit_groups(&config, &infos, &edges, None, 100_000).expect("derive");
    assert_eq!(
        shape(&out),
        vec![
            ("grp_prod".to_string(), vec!["prod".to_string()]),
            ("grp_spare".to_string(), vec!["spare".to_string()]),
            ("grp_cons".to_string(), vec!["cons".to_string()]),
        ],
        "process-per-node, exactly as before — the constraint must not fire without `block`"
    );
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

// ==========================================================================
// Section F — `node stage` over the untouched `graph create` scaffold.
//
// The README quick start is `graph create perception` then `node stage sensor
// -g perception`. Step 2 must not warn "overwriting an existing graph file —
// the previous file (including any hand edits) was backed up" or leave a
// `.bak` on a first run with no hand edit anywhere. Every arm drives the
// real verbs over a real scratch workspace (a `graphs/` dir; the workspace
// lock lands beside it) — still parallel-safe: no transport, no clock. The
// oracles are the LOG and the DIRECTORY LISTING: a warning that fires, or a
// `.bak` that appears, IS the regression.
// ==========================================================================

/// A scratch workspace's `graphs/` dir (its parent is the lock root).
fn scratch_graphs_dir() -> (tempfile::TempDir, std::path::PathBuf) {
    let tmp = tempfile::tempdir().expect("tempdir");
    let graphs = tmp.path().join("graphs");
    std::fs::create_dir_all(&graphs).expect("graphs dir");
    (tmp, graphs)
}

/// Sorted file names under `dir` — the listing is the "no `.bak`, no stray
/// temp file" oracle.
fn listing(dir: &std::path::Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .expect("read_dir")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    names
}

fn sensor_def() -> NodeDef {
    build_node_def(
        "sensor",
        None,
        &[("reading".to_string(), Some("std_msgs/Float64".to_string()))],
        &[],
    )
}

const OVERWRITE_WARNING: &str = "overwriting an existing graph file";

/// The scaffold with `sensor` staged into it, for an explicit `robot1` prefix.
/// Hand-written; the shape `append_node_to_nodes_block` gives a `nodes: []`.
const ROBOT1_WITH_SENSOR: &str = "prefix: robot1\nnodes:\n  - id: sensor\n    type: sensor\n    \
                                  outputs:\n      - name: reading\n        schema: std_msgs/Float64\n";

/// THE README path: `graph create` with the default (hostname) prefix, then
/// `node stage`. No warning, no `.bak`, one INFO line — and the node is in.
#[tracing_test::traced_test]
#[test]
fn staging_into_the_untouched_graph_create_scaffold_neither_warns_nor_backs_up() {
    let (_tmp, graphs) = scratch_graphs_dir();
    graph_create(&graphs, "perception", None).expect("graph create");

    let config = node_stage(&graphs, "perception", sensor_def()).expect("node stage");

    assert!(
        !logs_contain(OVERWRITE_WARNING),
        "a pristine scaffold holds no hand edits — nothing to warn about"
    );
    assert!(!logs_contain("was backed up"));
    assert!(
        logs_contain("untouched `graph create` scaffold"),
        "the one INFO line says why no backup was taken"
    );
    assert_eq!(
        listing(&graphs),
        vec!["perception.yaml".to_string()],
        "no `.bak`, no temp file"
    );
    assert_eq!(config.nodes.len(), 1);
    assert_eq!(config.nodes[0].id, "sensor");
    // The write succeeds: the file re-parses with the node in it.
    let reloaded = graph_read(&graphs, "perception").expect("re-read");
    assert_eq!(reloaded.nodes.len(), 1);
    assert_eq!(reloaded.nodes[0].id, "sensor");
}

/// User-edited prefixes must not
/// bypass backup protection: a user who edits ONLY the canonical `prefix:`
/// value — leaving `nodes: []` and every other byte untouched — must NOT be
/// mistaken for the pristine scaffold. The check renders the expected
/// scaffold from the CREATE-TIME input (the graph name → `default_prefix`
/// hostname default), never from the file's parsed prefix, so the edited prefix
/// does not match and the backup+warn fire. Rendered from the file's own prefix, this file
/// would match a scaffold for its edited prefix and be silently discarded.
#[tracing_test::traced_test]
#[test]
fn editing_only_the_prefix_line_still_warns_and_backs_up() {
    let (_tmp, graphs) = scratch_graphs_dir();
    // Create with the default (hostname) prefix — the file `node stage` can
    // prove pristine from the graph name alone.
    graph_create(&graphs, "perception", None).expect("graph create");
    let created = std::fs::read_to_string(graphs.join("perception.yaml")).expect("created");

    // Edit ONLY the prefix value; keep `nodes: []` and every other byte. The
    // edited value is derived from the create-time default so it is GUARANTEED
    // to differ from it, whatever this machine's hostname resolves to.
    let default_prefix = cerulion_core::graph::default_prefix("perception");
    let edited = format!("prefix: {default_prefix}-edited\nnodes: []\n");
    assert_ne!(edited, created, "the edit must actually change the file");
    std::fs::write(graphs.join("perception.yaml"), &edited).expect("edit prefix");

    node_stage(&graphs, "perception", sensor_def()).expect("node stage");

    assert!(
        logs_contain(OVERWRITE_WARNING),
        "an edited prefix is a hand edit — it must warn, not be taken for pristine"
    );
    assert!(logs_contain("was backed up"));
    assert_eq!(
        listing(&graphs),
        vec![
            "perception.yaml".to_string(),
            "perception.yaml.bak".to_string()
        ],
        "the edited prefix must be backed up, not silently discarded"
    );
    assert_eq!(
        std::fs::read_to_string(graphs.join("perception.yaml.bak")).expect("bak readable"),
        edited,
        "the backup carries the user's edited prefix bytes"
    );
}

/// `graph create -n <prefix>` freezes an explicit prefix whose only on-disk
/// record is the editable `prefix:` line — indistinguishable from a user who
/// edited that same line by hand. So it CANNOT be proven pristine from an
/// immutable input, and the check conservatively backs it up rather than trust
/// the file's own prefix. (Skipping this arm would reopen
/// that hole.) The common README `graph create <name>` (default prefix) path
/// stays backup-free; only the explicit-prefix flow pays one harmless `.bak`.
#[tracing_test::traced_test]
#[test]
fn an_explicit_prefix_scaffold_conservatively_backs_up() {
    let (_tmp, graphs) = scratch_graphs_dir();
    // An explicit prefix derived to differ from this host's default, so the
    // test is deterministic regardless of the machine's hostname.
    let explicit = format!(
        "{}-explicit",
        cerulion_core::graph::default_prefix("perception")
    );
    graph_create(&graphs, "perception", Some(&explicit)).expect("graph create");
    let created = std::fs::read_to_string(graphs.join("perception.yaml")).expect("created");

    node_stage(&graphs, "perception", sensor_def()).expect("node stage");

    assert!(
        logs_contain(OVERWRITE_WARNING),
        "an explicit prefix is unprovable from immutable inputs — back it up"
    );
    assert!(logs_contain("was backed up"));
    assert_eq!(
        listing(&graphs),
        vec![
            "perception.yaml".to_string(),
            "perception.yaml.bak".to_string()
        ]
    );
    assert_eq!(
        std::fs::read_to_string(graphs.join("perception.yaml.bak")).expect("bak readable"),
        created,
        "the backup carries the untouched explicit-prefix scaffold"
    );
}

/// ONE hand edit — a comment — and the file is no longer the scaffold: the
/// backup + warn contract applies unchanged, and the `.bak` carries the
/// edited bytes.
#[tracing_test::traced_test]
#[test]
fn a_hand_edited_scaffold_still_warns_and_backs_up() {
    let (_tmp, graphs) = scratch_graphs_dir();
    graph_create(&graphs, "perception", Some("robot1")).expect("graph create");
    let edited = "# the front camera rig\nprefix: robot1\nnodes: []\n";
    std::fs::write(graphs.join("perception.yaml"), edited).expect("hand edit");

    node_stage(&graphs, "perception", sensor_def()).expect("node stage");

    assert!(
        logs_contain(OVERWRITE_WARNING),
        "a hand edit must still warn"
    );
    assert!(logs_contain("was backed up"));
    assert_eq!(
        listing(&graphs),
        vec![
            "perception.yaml".to_string(),
            "perception.yaml.bak".to_string()
        ]
    );
    assert_eq!(
        std::fs::read_to_string(graphs.join("perception.yaml.bak")).expect("bak readable"),
        edited,
        "the backup carries the edited bytes"
    );
    assert_eq!(
        std::fs::read_to_string(graphs.join("perception.yaml")).expect("readable"),
        format!("# the front camera rig\n{ROBOT1_WITH_SENSOR}"),
        "the comment survives the splice"
    );
}

/// The check is EXACT: a graph that already holds one staged node is not the
/// scaffold, so the SECOND stage backs up and warns as it always did.
#[tracing_test::traced_test]
#[test]
fn a_second_stage_onto_the_same_graph_backs_up_as_before() {
    let (_tmp, graphs) = scratch_graphs_dir();
    // Default (hostname) prefix so the FIRST stage takes the pristine arm; an
    // explicit prefix would conservatively back up on the first stage too.
    graph_create(&graphs, "perception", None).expect("graph create");
    node_stage(&graphs, "perception", sensor_def()).expect("first stage");
    assert!(
        !logs_contain(OVERWRITE_WARNING),
        "the first stage is the pristine arm"
    );
    // The one-node revision the second stage will replace — captured because
    // the prefix is this machine's hostname, not a fixed literal.
    let after_first = std::fs::read_to_string(graphs.join("perception.yaml")).expect("after first");

    let filter = build_node_def(
        "filter",
        None,
        &[],
        &[("reading".to_string(), "sensor/reading".to_string())],
    );
    node_stage(&graphs, "perception", filter).expect("second stage");

    assert!(
        logs_contain(OVERWRITE_WARNING),
        "the file now holds a node — it is not the scaffold"
    );
    assert_eq!(
        listing(&graphs),
        vec![
            "perception.yaml".to_string(),
            "perception.yaml.bak".to_string()
        ]
    );
    assert_eq!(
        std::fs::read_to_string(graphs.join("perception.yaml.bak")).expect("bak readable"),
        after_first,
        "the backup is the one-node revision the second stage replaced"
    );
}
