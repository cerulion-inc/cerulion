// SPDX-License-Identifier: AGPL-3.0-only
//! Oracle pins for the tab-completion value sources.
//!
//! Every source is driven over a SYNTHETIC workspace / cache built in a
//! tempdir and compared against a HAND-WRITTEN candidate list — never against
//! a second run of the same code. The one structural test walks the module's
//! own source to pin the property no behavioural test can observe: that no
//! code path in the completer can start a process or open the network.
//!
//! Parallel-safe: no transport, no shared state, per-test tempdirs.

use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

use cerulion_cli_engine::completions::{
    graph_names_in, node_types_in, robot_names_at, schema_names_in, topic_names_from, Candidate,
};

/// Values only — most oracles care about WHICH names are offered, not their
/// provenance notes; the tests that pin help assert on the full `Candidate`.
fn values(candidates: &[Candidate]) -> Vec<&str> {
    candidates.iter().map(|c| c.value.as_str()).collect()
}

fn write(path: &Path, contents: &str) {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).expect("create parent");
    }
    fs::write(path, contents).expect("write fixture");
}

/// A node crate is a directory under `nodes/` with a `src/lib.rs`.
fn scaffold_node(nodes_dir: &Path, name: &str) {
    write(
        &nodes_dir.join(name).join("src").join("lib.rs"),
        "// a node\n",
    );
}

// ---------------------------------------------------------------------------
// Node types
// ---------------------------------------------------------------------------

#[test]
fn node_types_are_the_node_dirs_that_carry_a_lib_rs() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let nodes = tmp.path().join("nodes");

    // Two real node crates, deliberately created in reverse-alphabetical order
    // so a returned sorted list cannot be an accident of `read_dir`.
    scaffold_node(&nodes, "zeta_planner");
    scaffold_node(&nodes, "alpha_camera");

    // A bare directory with no `src/lib.rs` — an abandoned scaffold or an
    // unrelated crate. `cerulion node list` skips it, so completion must too.
    fs::create_dir_all(nodes.join("not_a_node")).expect("mkdir");

    // A FILE under `nodes/` is not a node type.
    write(&nodes.join("README.md"), "notes\n");

    let got = node_types_in(&nodes);
    assert_eq!(values(&got), vec!["alpha_camera", "zeta_planner"]);
    // Node types carry no provenance note — there is only one source.
    assert!(got.iter().all(|c| c.help.is_none()));
}

#[test]
fn a_missing_nodes_dir_completes_to_nothing_rather_than_failing() {
    let tmp = tempfile::tempdir().expect("tempdir");
    // The path does not exist at all — the shape outside a workspace, and the
    // shape of a workspace scaffolded without any nodes yet.
    let got = node_types_in(&tmp.path().join("nodes"));
    assert!(got.is_empty(), "expected no candidates, got {got:?}");
}

// ---------------------------------------------------------------------------
// Graph names
// ---------------------------------------------------------------------------

#[test]
fn graph_names_are_yaml_stems_and_exclude_what_graph_run_cannot_resolve() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let graphs = tmp.path().join("graphs");
    fs::create_dir_all(&graphs).expect("mkdir");

    write(&graphs.join("perception.yaml"), "nodes: {}\n");
    write(&graphs.join("control.yaml"), "nodes: {}\n");

    // `graph_read` joins `{name}.yaml`, so a `.yml` file is NOT runnable —
    // offering it would complete a name that then fails to resolve.
    write(&graphs.join("legacy.yml"), "nodes: {}\n");
    // The auto-partitioner's cost snapshot sits beside the graph and is not a
    // graph. Its stem is `perception.costs`, so a naive stem scan would offer
    // it; the `.yaml`-extension rule alone does not exclude it, which is why
    // this arm exists.
    write(&graphs.join("perception.costs.yaml"), "version: 2\n");

    let got = graph_names_in(&graphs);
    assert_eq!(
        values(&got),
        vec!["control", "perception", "perception.costs"],
        "the cost snapshot IS offered — it is a real `.yaml` stem; this pins \
         the known-and-accepted over-offer so a future filter is a deliberate \
         change, and pins that `legacy.yml` is excluded"
    );
}

#[test]
fn a_missing_graphs_dir_completes_to_nothing() {
    let tmp = tempfile::tempdir().expect("tempdir");
    assert!(graph_names_in(&tmp.path().join("graphs")).is_empty());
}

// ---------------------------------------------------------------------------
// Schemas
// ---------------------------------------------------------------------------

#[test]
fn schema_names_span_workspace_yaml_msg_store_and_builtins() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let schemas = tmp.path().join("schemas");
    fs::create_dir_all(&schemas).expect("mkdir");

    write(&schemas.join("my_detection.yaml"), "schemas: {}\n");
    write(
        &schemas.join("go2_msgs").join("msg").join("LowState.msg"),
        "uint8 level\n",
    );
    // Not a schema: wrong extension in the workspace dir, and a stray file in
    // a package's msg dir.
    write(&schemas.join("notes.txt"), "hi\n");
    write(&schemas.join("go2_msgs").join("msg").join("README"), "hi\n");

    let got = schema_names_in(Some(&schemas));
    let vals = values(&got);

    assert!(
        vals.contains(&"my_detection"),
        "workspace YAML stem missing from {vals:?}"
    );
    assert!(
        vals.contains(&"go2_msgs/LowState"),
        "msg-store entry missing"
    );
    // A built-in from the compile-time registry, in the `pkg/Type` slash form
    // the CLI normalizes to.
    assert!(
        vals.contains(&"sensor_msgs/Image"),
        "built-in registry missing"
    );
    assert!(!vals.contains(&"notes"), "non-YAML offered: {vals:?}");
    assert!(!vals.contains(&"go2_msgs/README"), "non-.msg offered");

    // Provenance is what tells an operator which of three sources a name came
    // from, so it is pinned rather than merely present.
    let help_of = |v: &str| {
        got.iter()
            .find(|c| c.value == v)
            .and_then(|c| c.help.clone())
    };
    assert_eq!(help_of("my_detection"), Some("workspace".to_string()));
    assert_eq!(help_of("go2_msgs/LowState"), Some("msg store".to_string()));
    assert_eq!(help_of("sensor_msgs/Image"), Some("built-in".to_string()));

    // Sorted and unique — the whole list, not just the rows asserted above.
    let mut sorted = vals.clone();
    sorted.sort_unstable();
    assert_eq!(vals, sorted, "candidates must be sorted");
    let unique: BTreeSet<&&str> = vals.iter().collect();
    assert_eq!(unique.len(), vals.len(), "candidates must be unique");
}

#[test]
fn a_workspace_schema_shadows_a_builtin_of_the_same_name() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let schemas = tmp.path().join("schemas");
    // The `.msg` store CAN legitimately hold a `pkg/Type` that also exists as a
    // built-in — that is exactly what `ros2 attach` materializes when a robot
    // serves its own copy. `schema info` resolves the workspace copy, so the
    // completion note must say so rather than claiming `built-in`.
    write(
        &schemas.join("sensor_msgs").join("msg").join("Image.msg"),
        "uint32 height\n",
    );

    let got = schema_names_in(Some(&schemas));
    let entry = got
        .iter()
        .find(|c| c.value == "sensor_msgs/Image")
        .expect("sensor_msgs/Image must be offered");
    assert_eq!(
        entry.help,
        Some("msg store".to_string()),
        "the shadowing workspace copy must win the help, not the built-in"
    );
    // And it must appear exactly once, not twice.
    assert_eq!(
        got.iter()
            .filter(|c| c.value == "sensor_msgs/Image")
            .count(),
        1
    );
}

#[test]
fn outside_a_workspace_schemas_degrade_to_the_builtin_registry() {
    let got = schema_names_in(None);
    let vals = values(&got);
    assert!(
        vals.contains(&"sensor_msgs/Image") && vals.contains(&"geometry_msgs/Twist"),
        "built-ins must complete anywhere — that is why `schema info` works \
         outside a workspace"
    );
    assert!(got.iter().all(|c| c.help.as_deref() == Some("built-in")));
}

// ---------------------------------------------------------------------------
// Robots
// ---------------------------------------------------------------------------

/// `peers.json` as `peer_cache::save_peers` writes it: a version tag plus rows
/// carrying a robot name, its gateway locator, and a last-seen stamp.
fn write_peer_cache(path: &Path, rows: &[(&str, &str, u64)]) {
    let entries: Vec<String> = rows
        .iter()
        .map(|(robot, locator, seen)| {
            format!(r#"{{"robot":"{robot}","locator":"{locator}","last_seen":{seen}}}"#)
        })
        .collect();
    write(
        path,
        &format!(r#"{{"v":1,"peers":[{}]}}"#, entries.join(",")),
    );
}

#[test]
fn robot_names_merge_paired_and_cached_with_precedence() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let peers = tmp.path().join("peers.json");
    let robots = tmp.path().join("robots.toml");
    let now = 1_000_000u64;

    write_peer_cache(
        &peers,
        &[
            ("cached_only", "tcp/10.0.0.9:7683", now - 60),
            // Also PAIRED — the precedence probe.
            ("go2", "tcp/10.0.0.2:7683", now - 60),
        ],
    );
    write(
        &robots,
        "[robots]\ngo2 = \"aabb\"\npaired_only = \"ccdd\"\n",
    );

    let got = robot_names_at(Some(&peers), Some(&robots), now);
    assert_eq!(
        values(&got),
        vec!["cached_only", "go2", "paired_only"],
        "every robot this desk has met, from both records, deduped"
    );

    let help_of = |v: &str| {
        got.iter()
            .find(|c| c.value == v)
            .and_then(|c| c.help.clone())
    };
    // A robot in BOTH records reports the stronger fact: a pin survives a
    // cache eviction, so `paired` outranks a locator that may go stale.
    assert_eq!(help_of("go2"), Some("paired".to_string()));
    assert_eq!(help_of("paired_only"), Some("paired".to_string()));
    // A cache row's help is its locator VERBATIM. zsh's `_describe` splits on
    // the first UNESCAPED colon — the separator zsh writes itself after the
    // escaped value — so a colon in help needs no repair, and an operator sees
    // a locator that looks like one.
    assert_eq!(
        help_of("cached_only"),
        Some("tcp/10.0.0.9:7683".to_string())
    );
}

#[test]
fn a_peer_older_than_the_cache_ttl_is_no_longer_offered() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let peers = tmp.path().join("peers.json");
    let now = 100_000_000u64;
    let ttl = 7 * 24 * 60 * 60;

    write_peer_cache(
        &peers,
        &[
            // One second inside the TTL, and exactly AT it (evicted — the
            // boundary `peer_cache` pins as `age >= TTL` drops).
            ("fresh", "tcp/10.0.0.1:7683", now - (ttl - 1)),
            ("stale", "tcp/10.0.0.2:7683", now - ttl),
        ],
    );

    let got = robot_names_at(Some(&peers), None, now);
    assert_eq!(
        values(&got),
        vec!["fresh"],
        "a robot gone longer than the cache TTL must stop being suggested"
    );
}

#[test]
fn missing_and_corrupt_robot_records_complete_to_nothing_rather_than_erroring() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let missing_peers = tmp.path().join("nope.json");
    let corrupt_robots = tmp.path().join("robots.toml");
    write(&corrupt_robots, "this is not { valid toml =\n");

    let got = robot_names_at(Some(&missing_peers), Some(&corrupt_robots), 1_000);
    assert!(got.is_empty(), "expected silence, got {got:?}");

    // And with no paths at all (no home directory).
    assert!(robot_names_at(None, None, 1_000).is_empty());
}

#[test]
fn a_hostile_robot_name_from_a_remote_record_is_dropped_not_inserted() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let peers = tmp.path().join("peers.json");
    let now = 1_000u64;

    // Robot names in the peer cache come from what a remote gateway announced,
    // so they are checked against the UNION of BOTH shells' failure modes —
    // one candidate stream feeds both, so a value only one shell mangles is
    // still unsafe:
    //   - a newline forges a second candidate on zsh's `\n`-separated wire;
    //   - a space breaks the word bash inserts unquoted;
    //   - `*` / `[` are re-expanded against the cwd by bash's UNQUOTED
    //     `COMPREPLY=( $( … ) )`;
    //   - `:` is in bash's default COMP_WORDBREAKS, so readline replaces only
    //     the post-colon segment while the script returns the whole candidate
    //     — MEASURED on real /bin/bash as `robot:` + TAB inserting
    //     `robot:robot:1`. (zsh escapes it and is fine; the union still wins.)
    write_peer_cache(
        &peers,
        &[
            ("line\\nbreak", "tcp/10.0.0.3:7683", now),
            ("has space", "tcp/10.0.0.4:7683", now),
            ("glob*star", "tcp/10.0.0.5:7683", now),
            ("brack[et", "tcp/10.0.0.6:7683", now),
            ("robot:1", "tcp/10.0.0.1:7683", now),
            ("ok_name", "tcp/10.0.0.2:7683", now),
        ],
    );

    let got = robot_names_at(Some(&peers), None, now);
    assert_eq!(
        values(&got),
        vec!["ok_name"],
        "only the wire-safe name survives"
    );
}

// ---------------------------------------------------------------------------
// Topics
// ---------------------------------------------------------------------------

#[test]
fn topics_keep_mirrors_and_carry_no_provenance_claim() {
    // The `ros2 attach` desk shape: two genuine local producers and one netd
    // mirror of a remote robot's topic (`/utlidar/cloud`). Read from the local
    // iceoryx2 service directory the three are indistinguishable — telling them
    // apart costs the 620 ms provenance read the budget rules out.
    let local = vec![
        "/utlidar/cloud".to_string(),
        "/camera/image".to_string(),
        "/local/odom".to_string(),
    ];

    let got = topic_names_from(&local);
    assert_eq!(
        got,
        vec![
            Candidate::bare("/camera/image"),
            Candidate::bare("/local/odom"),
            // Kept, not filtered: "one data source = one topic" — the mirror IS
            // the local handle `topic echo` / `viz` take by this exact name.
            Candidate::bare("/utlidar/cloud"),
        ],
        "no help at all: labelling the mirror `local` would be an affirmatively \
         WRONG claim on the one row that most needs telling apart, and the \
         right label is unaffordable here. A missing note makes no false claim."
    );
}

#[test]
fn a_desk_with_no_topics_completes_to_nothing() {
    assert!(topic_names_from(&[]).is_empty());
}

#[test]
fn a_hostile_topic_name_from_a_mirrored_service_is_dropped() {
    // A mirror's local service name is minted from what a remote robot
    // announced, so a topic name is no more trusted than a robot name. Two of
    // these are bash-specific: `/glob*` is re-globbed by the unquoted
    // `COMPREPLY=( $( … ) )`, and `/ns:topic` hits COMP_WORDBREAKS so bash
    // would insert `/ns:/ns:topic`.
    let local = vec![
        "/ok".to_string(),
        "/line\nbreak".to_string(),
        "/glob*".to_string(),
        "/has space".to_string(),
        "/ns:topic".to_string(),
    ];
    assert_eq!(topic_names_from(&local), vec![Candidate::bare("/ok")]);
}

// ---------------------------------------------------------------------------
// The structural guarantee
// ---------------------------------------------------------------------------

/// A view of `src` with `//` line comments and `/* */` block comments removed.
///
/// Load-bearing: the module's doc comments NAME the forbidden operations in
/// order to explain why they are absent (`connect_or_spawn`, "no zenoh
/// session", …). A raw substring scan would fire on the documentation and the
/// guard would have to be weakened into uselessness. Block comments nest in
/// Rust, so the scan is depth-tracked. String literals are deliberately NOT
/// modelled — this test file, not the module, is where the forbidden tokens
/// appear as literals, and the module is never scanned by itself.
fn code_only(src: &str) -> String {
    let bytes: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let mut i = 0;
    let mut block_depth = 0usize;
    while i < bytes.len() {
        if block_depth > 0 {
            if bytes[i] == '/' && bytes.get(i + 1) == Some(&'*') {
                block_depth += 1;
                i += 2;
            } else if bytes[i] == '*' && bytes.get(i + 1) == Some(&'/') {
                block_depth -= 1;
                i += 2;
            } else {
                // Keep newlines so line numbers in a failure stay meaningful.
                if bytes[i] == '\n' {
                    out.push('\n');
                }
                i += 1;
            }
            continue;
        }
        if bytes[i] == '/' && bytes.get(i + 1) == Some(&'*') {
            block_depth = 1;
            i += 2;
        } else if bytes[i] == '/' && bytes.get(i + 1) == Some(&'/') {
            while i < bytes.len() && bytes[i] != '\n' {
                i += 1;
            }
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    out
}

#[test]
fn the_comment_stripper_removes_both_syntaxes_and_nothing_else() {
    assert_eq!(
        code_only("let a = 1; // spawn\nlet b = 2;\n"),
        "let a = 1; \nlet b = 2;\n"
    );
    assert_eq!(code_only("a /* spawn */ b"), "a  b");
    // Nested block comments — Rust allows them, and a non-nesting stripper
    // would end the comment early and expose the tail as code.
    assert_eq!(code_only("a /* x /* y */ spawn */ b"), "a  b");
    // A block opener inside a line comment is not an opener.
    assert_eq!(code_only("// /* spawn\nreal code\n"), "\nreal code\n");
    // An unterminated block swallows the rest — fails CLOSED (the scan sees
    // less code, never more), which for a forbidden-token walk means a
    // malformed file cannot smuggle a token through as "not a comment".
    assert_eq!(code_only("code /* spawn"), "code ");
}

#[test]
fn the_completer_has_no_code_path_that_spawns_a_process_or_opens_the_network() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src")
        .join("completions.rs");
    let src = fs::read_to_string(&path).expect("read completions.rs");
    let code = code_only(&src);

    // Each entry: (token, why it is forbidden).
    let forbidden: &[(&str, &str)] = &[
        (
            "Command::new",
            "starting a process on a keystroke — the whole point of the budget \
             is that a TAB press does bounded local work",
        ),
        (
            "process::Command",
            "same as Command::new, via the qualified path",
        ),
        (
            "connect_or_spawn",
            "netd's only public constructors FORK a daemon and then block up to \
             ~10s polling for readiness (twice, by design) — up to a ~20s \
             stare at a TAB press",
        ),
        (
            "NetdClient",
            "every NetdClient verb rides a socket that may not exist yet; the \
             demand table it could answer adds no names the local SHM scan \
             lacks (see the module docs)",
        ),
        (
            "query_remote_topics",
            "opens a zenoh session and runs the network discovery ladder — \
             mDNS, TCP probes, a 1.5s ceiling",
        ),
        ("zenoh", "any network session at all"),
        (
            "discovery_ladder",
            "the ladder's rungs resolve hostnames and probe TCP ports",
        ),
        ("mdns", "a multicast browse is a network round-trip"),
        (
            "hostname_peers",
            "resolves `<name>.local` — a DNS round-trip that can stall",
        ),
        (
            "subnet_sweep",
            "a horizontal SYN sweep; unthinkable on a keystroke",
        ),
        (
            "node_list",
            "runs `syn::parse_file` over every node's lib.rs — unbounded in \
             workspace size",
        ),
        (
            "schema_list",
            "re-parses all built-in .msg texts to build a LayoutResolver",
        ),
        (
            "SchemaStore::load",
            "reads and parses every .msg file; completion needs names only",
        ),
    ];

    for (token, why) in forbidden {
        assert!(
            !code.contains(token),
            "completions.rs must never reach `{token}`: {why}\n\
             (found in the comment-stripped source — if this is a deliberate \
             design change, the module docs' hard-rule section must change too)"
        );
    }

    // Anti-tautology: the scan must be looking at real code, not an empty
    // string produced by a broken stripper. These are things the module DOES
    // do, and if the walk cannot see them it cannot see a violation either.
    for present in [
        "pub fn node_types_in",
        "run_bounded",
        "topic_list",
        "load_peers",
    ] {
        assert!(
            code.contains(present),
            "the stripped view lost `{present}` — the walk is not reading the \
             module's code, so every absence assertion above is vacuous"
        );
    }
}
