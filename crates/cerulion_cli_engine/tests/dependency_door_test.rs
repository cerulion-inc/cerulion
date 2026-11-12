// SPDX-License-Identifier: AGPL-3.0-only
//! The iroh and Rerun trees each have ONE door into the workspace.
//!
//! "`cerulion_link` is the one iroh seam" and "no viz on the robot" are
//! architecture rules about direct dependency edges: exactly which crates may
//! name one of those trees in their own `[dependencies]`.
//!
//! # Why this is a test and not `deny.toml`
//!
//! `deny.toml` carries `[[bans.deny]] name = "iroh"` / `"rerun"`, which is the
//! UMBRELLA spelling — and dependency direction runs umbrella → subcrate, so a
//! crate that depends directly on `re_chunk` or `iroh-base` pulls the whole tree
//! with the banned name never entering its graph. That is not hypothetical: this
//! workspace's own `cerulion_vizd` names SEVEN `re_*` crates directly, so the
//! subcrate spelling is the norm rather than the exotic case.
//!
//! Two things were MEASURED before writing this file, and both excluded doing
//! it in `deny.toml` (cargo-deny 0.20.2):
//!
//!  * a glob crate spec is accepted and then matches NOTHING. `name = "re_*"`
//!    parses, reports `bans ok`, and marks the wrappers "unmatched" — an
//!    inert ban that looks armed. (Control: the literal `name = "re_chunk"`
//!    reports `bans FAILED`, so the harness was real.)
//!  * concrete per-subcrate entries cannot express the rule either. `wrappers`
//!    lists who may DIRECTLY depend on a banned crate, and these crates are
//!    legitimately present — `re_chunk`'s direct dependents include
//!    `re_chunk_store`, `re_entity_db`, `re_grpc_client`, `re_log_channel`,
//!    `re_sdk` and `rerun`. Every entry would have to list the tree's whole
//!    internal graph (43 crates), per entry, and that list would rot on the next
//!    Rerun release.
//!
//! So the ban that CAN be written stays in `deny.toml` (the umbrellas), and the
//! invariant it cannot reach is enforced here, where the family is DERIVED at
//! test time and therefore cannot go stale when a new subcrate appears.
//!
//! # How the family is derived, and why not by name
//!
//! `see derived_family`. The short version: as the UNION of the `repository`
//! each package declares and a name-shape rule, because MEASURED on this tree
//! neither is complete. A name-shape rule alone was the original implementation
//! and was FAIL-OPEN: `sorted-index-buffer` is published from n0-computer/iroh
//! under a name no `iroh-` prefix can match, so it was classified into no
//! family, omitted from the edge set, and a direct edge onto it would have
//! passed a gate whose whole purpose is to catch exactly that. Four other
//! members go the other way — they publish from a different repository than the
//! umbrella — so a repository-only rule is fail-open too.
//!
//! # Two more spellings this reaches and `deny.toml` does not
//!
//! Reading `cargo metadata --no-deps` rather than resolving a graph buys two
//! more, both pinned by `assert_the_walk_sees_optional_and_renamed_dependencies`:
//!
//!  * FEATURE-GATED. cargo-deny resolves with default features and the `deps`
//!    job passes no flags, so an `iroh = { optional = true }` behind an
//!    off-by-default feature is outside `cargo deny check bans`. Metadata
//!    reports the DECLARED list, where an optional dep is present regardless.
//!  * RENAMED. Metadata's `name` is the PACKAGE name, with any alias in a
//!    separate `rename` key, so `desk = { package = "rerun" }` classifies as a
//!    rerun edge.
//!
//! What this file does NOT reach, stated so a green run is not over-read: it
//! sees DIRECT edges only. A tree appearing transitively — the case `deny.toml`
//! covers — is not this file's business.
//!
//! # Why the two metadata-reading arms share one `#[test]`
//!
//! Every document this file needs is read ONCE, in one test body, and handed to
//! the arms — rather than each arm asking cargo for its own copy. The arms are
//! unchanged; only who runs `cargo metadata` moved.
//!
//! The cost it removes is measured, not assumed: the file spawned FIVE
//! `cargo metadata` children (two full resolves and three `--no-deps`) for four
//! tests with no sleeps and no transport, and `cargo test -p cerulion_cli_engine`
//! passes no `--test-threads=1`, so the two expensive arms ran CONCURRENTLY and
//! serialised against each other on cargo's package-cache lock anyway.
//!
//! A `OnceLock` would have served under today's runner and NOT under a
//! process-per-test one (nextest), where each test is its own process and shares
//! no statics. One test body is correct under both.
//!
//! The two arms that read no metadata at all — `deny_toml_still_bans_the_\
//! umbrella_names` (reads `deny.toml`) and `every_checked_workspace_actually_\
//! has_members` (reads manifests and directories) — deliberately stay separate
//! `#[test]`s. They spawn no child, so folding them in would buy nothing and
//! would cost the thing a fold always costs: the first failure stops the rest,
//! so two invariants that can be reported independently should be.

mod common;

use common::{
    cargo_metadata_json_at, cargo_metadata_json_with_deps, declared_members_of, read_manifest,
    repo_root,
};
use std::collections::{BTreeMap, BTreeSet};

/// Workspaces whose members are checked. The root plus every workspace that
/// ships crates of its own (`examples/go2` is the ROBOT workspace, where a rerun
/// edge is the exact violation the no-viz-on-the-robot rule forbids).
const CHECKED_WORKSPACES: &[&str] = &["", "examples/go2"];

/// The ONE door into each tree: `(crate, tree, why)`.
///
/// Set equality, not a floor — a crate that gains an edge FAILS, and a door
/// that loses its edge fails as stale, so this table cannot drift into a list
/// of pre-authorised future doors.
const DOORS: &[(&str, &str, &str)] = &[
    (
        "cerulion_link",
        "iroh",
        "the dial-by-key QUIC wrapper every other crate goes through",
    ),
    (
        "cerulion_remoted",
        "iroh",
        "DEV edge only: its wire-plane test needs iroh's VarInt, which cerulion_link does not re-export",
    ),
    (
        "cerulion_viz",
        "rerun",
        "the generic desk-side viz support lib",
    ),
    (
        "cerulion_vizd",
        "rerun",
        "the desk daemon that rasterizes",
    ),
];

/// The upstream repository each tree publishes from — the AUTHORITATIVE half of
/// the family derivation, because it does not depend on how a crate is spelled.
const FAMILY_REPOSITORIES: &[(&str, &str)] = &[
    ("https://github.com/rerun-io/rerun", "rerun"),
    ("https://github.com/n0-computer/iroh", "iroh"),
];

/// Which tree a package name belongs to BY NAME SHAPE, or `None`.
///
/// This is one of the two halves of the derivation and is NOT sufficient alone
/// — see `derived_family`. It is kept because it is the half that survives a
/// package publishing no `repository` at all.
fn name_shape_tree(name: &str) -> Option<&'static str> {
    if name == "rerun" || name.starts_with("re_") {
        Some("rerun")
    } else if name == "iroh" || name.starts_with("iroh-") || name.starts_with("iroh_") {
        Some("iroh")
    } else {
        None
    }
}

/// `repository` as cargo reports it, normalised enough to compare.
fn normalise_repo(repo: &str) -> String {
    let r = repo.trim().trim_end_matches('/').to_ascii_lowercase();
    r.strip_suffix(".git").unwrap_or(&r).to_string()
}

/// `package name -> tree` for every package of either family, DERIVED from the
/// resolved dependency graph as the UNION of two independent signals.
///
/// Neither signal alone is complete, and that is MEASURED on this tree rather
/// than argued:
///
///  * REPOSITORY (authoritative). 46 packages here declare `repository =`
///    rerun-io/rerun or n0-computer/iroh. It catches `sorted-index-buffer`,
///    which iroh publishes under a name no `iroh-` prefix rule can see — the
///    live instance of the hole this function closes.
///  * NAME SHAPE. Catches 4 the repository signal does not, because they are
///    published from a DIFFERENT repository than the umbrella:
///    `iroh-metrics`, `iroh-metrics-derive` (n0-computer/iroh-metrics),
///    `re_mp4` (rerun-io/re_mp4) and `re_grpc_server` — which is OUR fork, so
///    its `repository` is a cerulion-inc URL. A repository-only derivation
///    would drop all four.
///
/// The union is therefore fail-CLOSED where the old prefix-only classifier was
/// fail-open: a family member the name rule cannot see is still classified, so
/// a direct edge onto it is still a door.
///
/// Takes the already-read resolve document: the caller reads it once and hands
/// it to every arm that needs it (see the module doc).
fn derived_family(json: &serde_json::Value) -> BTreeMap<String, &'static str> {
    let packages = json["packages"].as_array().expect("packages array");

    let mut family: BTreeMap<String, &'static str> = BTreeMap::new();
    let mut from_repo: BTreeSet<String> = BTreeSet::new();
    for pkg in packages {
        let name = pkg["name"].as_str().expect("package name");
        if let Some(repo) = pkg["repository"].as_str() {
            let repo = normalise_repo(repo);
            if let Some((_, tree)) = FAMILY_REPOSITORIES.iter().find(|(u, _)| *u == repo) {
                family.insert(name.to_string(), tree);
                from_repo.insert(name.to_string());
            }
        }
    }
    let mut from_shape: BTreeSet<String> = BTreeSet::new();
    for pkg in packages {
        let name = pkg["name"].as_str().expect("package name");
        if let Some(tree) = name_shape_tree(name) {
            family.entry(name.to_string()).or_insert(tree);
            from_shape.insert(name.to_string());
        }
    }

    // Both halves must be load-bearing, or this union is theatre. These are
    // deliberate TRIPWIRES, in the shape `every_cross_workspace_member_has_its_\
    // workspace_declared` uses: if upstream ever renames such that one half
    // becomes redundant, a human decides that, rather than the gate quietly
    // shrinking to the half that used to be a fail-open.
    let repo_only: Vec<&String> = from_repo.difference(&from_shape).collect();
    let shape_only: Vec<&String> = from_shape.difference(&from_repo).collect();
    assert!(
        !repo_only.is_empty(),
        "every repository-derived family member is also caught by the name-shape \
         rule, so the repository half currently adds nothing. It was added because \
         `sorted-index-buffer` (published from n0-computer/iroh) is NOT catchable \
         by any `iroh-` prefix rule and a prefix-only classifier let a direct edge \
         onto it pass. If upstream renamed it, say so here deliberately — do not \
         delete the half that closed the hole."
    );
    assert!(
        !shape_only.is_empty(),
        "every name-shape family member is also caught by the repository rule, so \
         the name-shape half currently adds nothing. It exists because four of \
         these crates publish from a DIFFERENT repository than the umbrella \
         (iroh-metrics, iroh-metrics-derive, re_mp4, and our own re_grpc_server \
         fork), which a repository-only derivation drops. Check that before \
         removing it."
    );
    assert!(
        family.len() >= 40,
        "the family derivation found only {} packages across BOTH signals — too \
         few to be real, so every assertion resting on it is vacuous",
        family.len()
    );
    family
}

/// `package -> {tree}` for every DIRECT dependency edge into either tree, read
/// out of ONE `cargo metadata --no-deps` document.
///
/// Split from its driver so the classification can be driven by hand-built
/// documents in `assert_the_walk_sees_optional_and_renamed_dependencies`:
/// against the real tree every one of those branches is unreachable, because no
/// crate here declares an optional or renamed edge into either tree today.
///
/// Reads cargo metadata's `dependencies`, which already folds in dev-, build-
/// and target-specific tables — a hand parse of `[dependencies]` alone would
/// miss `cerulion_remoted`'s dev edge, i.e. exactly one of the doors. It is also
/// the MANIFEST-declared list, so an `optional = true` dependency appears
/// whether or not any feature turns it on: this walk is not feature-blind the
/// way `cargo deny` is (see the feature paragraph in `deny.toml`). And metadata
/// spells `name` as the PACKAGE name with any alias in a separate `rename` key,
/// so `foo = { package = "iroh" }` is classified as an iroh edge, not as `foo`.
fn tree_edges_in(
    json: &serde_json::Value,
    family: &BTreeMap<String, &'static str>,
) -> BTreeMap<String, BTreeSet<String>> {
    let mut edges: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for pkg in json["packages"].as_array().expect("packages array") {
        let name = pkg["name"].as_str().expect("package name").to_string();
        for dep in pkg["dependencies"].as_array().expect("dependencies array") {
            let dep_name = dep["name"].as_str().expect("dependency name");
            // The DERIVED family first, name shape only as the fallback for a
            // package the resolved graph does not contain at all (a dep declared
            // in the mirrored `examples/go2` workspace but absent from the root's
            // resolve). Family-first is the fail-closed direction.
            if let Some(tree) = family
                .get(dep_name)
                .copied()
                .or_else(|| name_shape_tree(dep_name))
            {
                edges
                    .entry(name.clone())
                    .or_default()
                    .insert(tree.to_string());
            }
        }
    }
    edges
}

/// `tree_edges_in` over every checked workspace, merged.
///
/// Takes the per-workspace `--no-deps` documents the caller already read, keyed
/// by the `CHECKED_WORKSPACES` entry they came from.
fn direct_tree_edges(
    workspace_docs: &BTreeMap<&'static str, serde_json::Value>,
    family: &BTreeMap<String, &'static str>,
) -> BTreeMap<String, BTreeSet<String>> {
    let mut edges: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for ws in CHECKED_WORKSPACES {
        let json = workspace_docs
            .get(ws)
            .unwrap_or_else(|| panic!("no metadata document was read for workspace `{ws}`"));
        for (pkg, trees) in tree_edges_in(json, family) {
            edges.entry(pkg).or_default().extend(trees);
        }
    }
    edges
}

/// Every document the metadata-reading arms need, read ONCE.
///
/// `resolve` is the full `cargo metadata` (the family derivation needs
/// `repository`, which `Cargo.lock` does not carry); `workspace_docs` is the
/// `--no-deps` document per `CHECKED_WORKSPACES` entry, which is what restricts
/// the edge walk to workspace MEMBERS — a full-resolve document also contains
/// the trees' own internal edges (`re_sdk` -> `re_chunk`), and those are not
/// doors.
struct MetadataDocs {
    resolve: serde_json::Value,
    workspace_docs: BTreeMap<&'static str, serde_json::Value>,
}

fn read_metadata_docs(root: &std::path::Path) -> MetadataDocs {
    let resolve: serde_json::Value = serde_json::from_str(&cargo_metadata_json_with_deps(root, ""))
        .expect("cargo metadata emits JSON");
    let mut workspace_docs = BTreeMap::new();
    for ws in CHECKED_WORKSPACES {
        let json: serde_json::Value = serde_json::from_str(&cargo_metadata_json_at(root, ws))
            .expect("cargo metadata emits JSON");
        workspace_docs.insert(*ws, json);
    }
    MetadataDocs {
        resolve,
        workspace_docs,
    }
}

/// The two arms that read `cargo metadata`, over ONE read of every document.
///
/// They are one `#[test]` for cost (module doc: five children for four tests,
/// racing each other on cargo's package-cache lock) and they belong together
/// for meaning: the second arm pins the CLASSIFIER that the first arm's verdict
/// rests on, against documents the real tree cannot produce.
///
/// Breadcrumb for a grep, deliberately unbroken so the old name is findable:
/// the second arm was
/// `the_walk_sees_optional_and_renamed_dependencies`
/// and is now `assert_the_walk_sees_optional_and_renamed_dependencies` below,
/// with its assertions unchanged.
#[test]
fn only_the_declared_doors_depend_on_the_iroh_and_rerun_trees() {
    let root = repo_root();
    let docs = read_metadata_docs(&root);
    let family = derived_family(&docs.resolve);

    assert_only_the_declared_doors(&docs, &family);
    assert_the_walk_sees_optional_and_renamed_dependencies(&docs, &family);
}

fn assert_only_the_declared_doors(docs: &MetadataDocs, family: &BTreeMap<String, &'static str>) {
    // Anti-tautology: the derivation must classify a real, populated family, or
    // every assertion below is vacuous. `derived_family` carries its own floor
    // and its own both-halves-load-bearing tripwires.
    let rerun_family: Vec<&String> = family
        .iter()
        .filter(|(_, t)| **t == "rerun")
        .map(|(n, _)| n)
        .collect();
    let iroh_family: Vec<&String> = family
        .iter()
        .filter(|(_, t)| **t == "iroh")
        .map(|(n, _)| n)
        .collect();
    assert!(
        rerun_family.len() >= 20 && iroh_family.len() >= 2,
        "the family derivation found {} rerun-tree and {} iroh-tree packages — \
         too few to be real, so this gate proves nothing. It is supposed to cover \
         EVERY subcrate, which is the whole point: the umbrella-only ban in \
         deny.toml is bypassed by a direct subcrate edge.",
        rerun_family.len(),
        iroh_family.len()
    );
    // The umbrella names must be IN the derived family, or a rename upstream
    // would quietly shrink what this gate covers.
    assert!(rerun_family.iter().any(|n| n.as_str() == "rerun"));
    assert!(iroh_family.iter().any(|n| n.as_str() == "iroh"));
    // And the prefix-less member must be in it, by NAME: it is the one this
    // derivation exists for, and a silent loss of it is a silent re-opening of
    // the hole. (`derived_family`'s repo_only tripwire fires on the general
    // case; this pins the specific crate the finding was about.)
    assert_eq!(
        family.get("sorted-index-buffer").copied(),
        Some("iroh"),
        "`sorted-index-buffer` — published from n0-computer/iroh under a name no \
         `iroh-` prefix rule can see — is no longer classified into the iroh \
         family. A direct edge onto it would now pass this gate, which is exactly \
         the fail-open the repository-derived half was added to close."
    );

    let expected: BTreeSet<(String, String)> = DOORS
        .iter()
        .map(|(c, t, _)| (c.to_string(), t.to_string()))
        .collect();
    let actual: BTreeSet<(String, String)> = direct_tree_edges(&docs.workspace_docs, family)
        .into_iter()
        .flat_map(|(pkg, trees)| trees.into_iter().map(move |t| (pkg.clone(), t)))
        .collect();

    let new_doors: Vec<&(String, String)> = actual.difference(&expected).collect();
    assert!(
        new_doors.is_empty(),
        "these crates depend DIRECTLY on a banned dependency tree and are not \
         declared doors:\n  {}\n\nThe architecture rule is one door per tree — \
         `cerulion_link` for iroh, `cerulion_viz` /\n\
         `cerulion_vizd` for Rerun (project rule: NO viz on the \
         robot). A second door means a second place that tree's version pin, \
         configuration and error mapping live.\n\nNote this fires on SUBCRATE \
         edges too (`re_chunk`, `iroh-base`, …), which is why the gate exists: \
         deny.toml can only ban the umbrella names, and dependency direction \
         runs umbrella -> subcrate, so a direct subcrate edge pulls the whole \
         tree with the banned name never entering the graph.\n\nIf this really \
         is a new door, add it to DOORS in {} WITH A REASON — and expect that \
         to be the argument, not the paperwork.",
        new_doors
            .iter()
            .map(|(c, t)| format!("{c} -> {t} tree"))
            .collect::<Vec<_>>()
            .join("\n  "),
        file!()
    );

    let stale: Vec<&(String, String)> = expected.difference(&actual).collect();
    assert!(
        stale.is_empty(),
        "DOORS declares edges that no longer exist: {}. A door that describes \
         nothing is a pre-authorised future edge — delete the entries from {}.",
        stale
            .iter()
            .map(|(c, t)| format!("{c} -> {t}"))
            .collect::<Vec<_>>()
            .join(", "),
        file!()
    );
}

#[test]
fn deny_toml_still_bans_the_umbrella_names() {
    // The two mechanisms are complementary and BOTH have to stay: deny.toml
    // catches the umbrella spelling across the whole resolve graph (including
    // transitive appearances this test does not look at), while this file
    // catches the subcrate spelling on direct edges, which deny.toml cannot
    // express. Dropping the umbrella bans because "the test covers it" would
    // lose the half that runs in the `deps` CI job.
    let root = repo_root();
    let deny = read_manifest(&root.join("tools/release/deny.toml"));
    let banned: BTreeSet<&str> = deny
        .get("bans")
        .and_then(|b| b.get("deny"))
        .and_then(|d| d.as_array())
        .expect("deny.toml has [[bans.deny]] entries")
        .iter()
        .filter_map(|e| e.get("name").and_then(|n| n.as_str()))
        .collect();

    for umbrella in ["iroh", "rerun"] {
        assert!(
            banned.contains(umbrella),
            "deny.toml no longer bans `{umbrella}`. This test covers DIRECT \
             edges only; the umbrella ban is what covers the rest of the resolve \
             graph in the `deps` CI job."
        );
    }
}

fn assert_the_walk_sees_optional_and_renamed_dependencies(
    docs: &MetadataDocs,
    family: &BTreeMap<String, &'static str>,
) {
    // The two spellings that would dodge a naive read of `[dependencies]`, and
    // the reason this gate — not `deny.toml` — is where the direct-edge rule
    // lives.
    //
    //  * FEATURE-GATED. `cargo deny` resolves with each package's DEFAULT
    //    features and this repo passes no feature flags (see deny.toml's feature
    //    paragraph), so a `iroh = { optional = true }` behind an off-by-default
    //    feature is outside what `cargo deny check bans` inspects. This walk
    //    reads the MANIFEST-declared dependency list, where an optional dep is
    //    present whether or not anything turns it on. The pattern is already in
    //    the tree: `cerulion_netd`'s whole WAN plane is optional deps behind
    //    `wan`.
    //  * RENAMED. cargo metadata spells `name` as the PACKAGE name and puts any
    //    alias in a separate `rename` key, so `desk = { package = "rerun" }`
    //    classifies as a rerun edge rather than as `desk`.
    //  * PREFIX-LESS. A family member whose name matches no prefix rule at all
    //    (`sorted-index-buffer`, published from n0-computer/iroh) is classified
    //    by the DERIVED family. A prefix-only classifier omitted it from the
    //    edge set entirely, so a direct edge onto it was a silent pass.
    //
    // Driven by hand-built documents because the real tree contains neither
    // shape today — every branch below is unreachable against live metadata, so
    // asserting on the live document would pin nothing.
    let doc: serde_json::Value = serde_json::from_str(
        r#"{"packages":[
             {"name":"sneaky_optional","dependencies":[
               {"name":"iroh","optional":true,"rename":null}]},
             {"name":"sneaky_renamed","dependencies":[
               {"name":"rerun","optional":false,"rename":"desk"}]},
             {"name":"sneaky_subcrate","dependencies":[
               {"name":"re_chunk","optional":true,"rename":"chunks"}]},
             {"name":"sneaky_prefixless","dependencies":[
               {"name":"sorted-index-buffer","optional":false,"rename":null}]},
             {"name":"innocent","dependencies":[
               {"name":"serde","optional":true,"rename":null}]}]}"#,
    )
    .expect("hand-built metadata parses");

    // The family the production path derives, so this arm classifies exactly
    // the way the doors arm does rather than against a convenient stand-in —
    // it is the SAME `family` value, passed in by the caller.
    let edges = tree_edges_in(&doc, family);
    let tree_of_pkg = |p: &str| {
        edges
            .get(p)
            .map(|t| t.iter().cloned().collect::<Vec<_>>().join(","))
    };
    assert_eq!(
        tree_of_pkg("sneaky_optional").as_deref(),
        Some("iroh"),
        "an OPTIONAL direct edge into a banned tree is invisible to `cargo deny \
         check bans` (default features, no flags passed). If this walk misses it \
         too, nothing in the repo covers a feature-gated door."
    );
    assert_eq!(
        tree_of_pkg("sneaky_renamed").as_deref(),
        Some("rerun"),
        "a RENAMED dependency (`desk = {{ package = \"rerun\" }}`) must classify \
         by its PACKAGE name — cargo metadata puts the alias in `rename`, so \
         reading `name` is what makes the rename a non-dodge."
    );
    assert_eq!(
        tree_of_pkg("sneaky_subcrate").as_deref(),
        Some("rerun"),
        "both spellings at once — an optional, renamed SUBCRATE edge — is the \
         shape that evades deny.toml on all three counts"
    );
    assert_eq!(
        tree_of_pkg("sneaky_prefixless").as_deref(),
        Some("iroh"),
        "a direct edge onto `sorted-index-buffer` — an iroh-family crate whose \
         name matches NO `iroh-` prefix rule — must be a door. This is the arm \
         that fails if the classifier goes back to name shapes alone: the \
         package is real, it is published from n0-computer/iroh, and a \
         prefix-only rule omits it from `actual` entirely, so an unauthorised \
         direct edge onto it passes the gate."
    );
    assert_eq!(
        tree_of_pkg("innocent"),
        None,
        "anti-tautology: a crate with no edge into either tree must produce no \
         entry, or the three assertions above are satisfied by a classifier that \
         says yes to everything"
    );

    // …and the optional shape is one cargo really emits here, so the hand-built
    // document above is not testing a fiction. `--no-deps` reports the declared
    // list, optional entries included. (The caller's root document, not a fresh
    // `cargo metadata` child.)
    let live = docs
        .workspace_docs
        .get("")
        .expect("the root workspace document was read");
    let optional_deps = live["packages"]
        .as_array()
        .expect("packages array")
        .iter()
        .flat_map(|p| p["dependencies"].as_array().expect("dependencies array"))
        .filter(|d| d["optional"].as_bool() == Some(true))
        .count();
    assert!(
        optional_deps > 0,
        "cargo metadata --no-deps reported ZERO optional dependencies across the \
         workspace, but `cerulion_netd` and `cerulion_dds` both declare several. \
         Either the field moved or the parse is wrong — and then the \
         feature-gated half of this gate is checking a shape cargo does not \
         actually produce."
    );
}

#[test]
fn every_checked_workspace_actually_has_members() {
    // The door walk iterates CHECKED_WORKSPACES; a workspace that silently
    // resolved to nothing would remove its crates from the gate while leaving
    // it green.
    let root = repo_root();
    for ws in CHECKED_WORKSPACES {
        let members = declared_members_of(&root, ws);
        let label = if ws.is_empty() { "<root>" } else { ws };
        assert!(
            members.len() >= 2,
            "checked workspace `{label}` expanded to {} member(s) — its crates \
             are not being covered by the door gate",
            members.len()
        );
    }
}
