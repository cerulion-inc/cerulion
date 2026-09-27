// SPDX-License-Identifier: AGPL-3.0-only
//! The tree's dependency-architecture rules, as tests over `cargo metadata`.
//!
//! WHY THIS FILE EXISTS. Every rule asserted here was already written down: two
//! of them as jobs in `.github/workflows/ci.yml`, the rest as sentences in the
//! root `Cargo.toml`, in a crate manifest header, in a crate `AGENTS.md`, or in
//! `docs/internals/`. A sentence does not fail a build, and a CI job fails one
//! on another machine minutes after the edge was typed. Each rule below names
//! the sentence it is the machine half of, so the two cannot drift apart
//! silently.
//!
//! WHY `cargo metadata` AND NOT A MANIFEST GREP. The edges that break these
//! rules are the ones a grep cannot see: an OPTIONAL dependency behind a
//! default-on feature, a RENAMED one (`viz = { package = "rerun" }`), and a
//! TRANSITIVE one three crates down. `cargo metadata --format-version 1`
//! reports every declaration cargo itself read, so all three are visible here.
//!
//! WHY THE RESOLVE GRAPH IS RECOMPUTED AND NOT READ OFF `resolve.nodes`. This
//! is the one subtlety in the file, and getting it wrong makes the leanness
//! rules fire falsely. `cargo metadata` emits ONE resolve graph for the whole
//! workspace, with each package's features UNIFIED across every member that
//! selects it. `cerulion_netd` is a member whose `wan` feature is default-on,
//! so the emitted graph shows netd with `wan` enabled and the iroh tree hanging
//! off it, while the actual default build reaches netd only through
//! `cerulion_cli_engine`, which depends `default-features = false` and gets no
//! iroh at all. Walking `resolve.nodes` from the default members therefore
//! reports iroh in the default build (MEASURED on this tree: 731 packages
//! including `iroh`, `iroh-base` and `iroh-relay`), which is exactly the false
//! red this file must not produce. So [`Graph::resolve`] runs cargo's
//! feature algorithm from a chosen set of roots: default features,
//! `dep:`/`pkg/feat`/`pkg?/feat` expansion, implicit optional-dependency
//! features, to a fixed point. The emitted `resolve.nodes` are used only to map
//! a manifest dependency onto the package id cargo picked for it, never to
//! decide whether an edge is live.
//!
//! WHERE THE RESOLVER'S ORACLE COMES FROM. Not from itself, and not from a
//! measurement somebody took once.
//! [`the_resolver_covers_every_package_cargo_tree_reports`] runs
//! `cargo tree -e normal` over the same workspace and requires the computed set
//! to contain all of it. That is cargo's own resolver reached by a different
//! command and a different code path, so a future change to feature resolution
//! that this file gets wrong shows up as a red rather than as a quiet
//! under-count. MEASURED while writing: `cargo tree` reports 464 packages for
//! the host, all present here, and the resolver's 65 extras are platform-gated
//! crates (`windows-*`, `wasm-bindgen`, `redox_*`, `netlink-*`, ...) that only
//! a non-host target pulls. Superset by construction; see PLATFORMS below.
//!
//! WHAT PINS THE RESOLVER IN THE TREE. A feature resolver that enabled
//! nothing would report every rule clean. Two tests pin it from opposite sides:
//!
//!   * [`netd_per_package_build_pulls_the_iroh_wan_plane`] proves default
//!     features ARE followed: netd's `default = ["wan"]` reaches iroh through
//!     an optional `dep:` edge;
//!   * [`default_member_build_is_iroh_free`] proves `default-features = false`
//!     IS honoured: the same netd, reached through the CLI engine, does not.
//!
//! On top of those, [`the_checker_reports_a_forbidden_crate_when_one_is_present`]
//! points the very checker the rules use at roots that legitimately carry the
//! forbidden crate and requires it to complain.
//!
//! PLATFORMS. No `--filter-platform` and no target filtering: an edge that is
//! `cfg(windows)`-only counts. That makes every rule here at least as strict as
//! the `cargo tree` probes in CI, which default to the host platform, and makes
//! the verdict identical on every developer machine.
//!
//! COST. One `cargo metadata` invocation per test binary, shared through a
//! `OnceLock`; no compilation and no network beyond what resolving the
//! workspace already needs. The `examples/go2` rules read that workspace's
//! committed manifests and lockfile directly, so they need no second resolve
//! and no network at all.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

use serde::Deserialize;

// ---------------------------------------------------------------------------
// The forbidden families, derived the way the CI jobs derive theirs.
// ---------------------------------------------------------------------------

/// Packages whose name marks them as part of the iroh tree.
///
/// A FAMILY, not a name: dependency direction runs umbrella -> subcrate, so a
/// crate depending directly on `iroh-base` pulls the tree while `iroh` itself
/// never enters the graph and a name-exact probe stays green. The same
/// reasoning is written out in the `iroh-leanness` job, whose pattern
/// (`^iroh($|-|_)`) this mirrors.
fn is_iroh_family(name: &str) -> bool {
    name == "iroh" || name.starts_with("iroh-") || name.starts_with("iroh_")
}

/// Packages whose name marks them as part of the Rerun SDK.
///
/// `cerulion_vizd` names seven `re_*` crates directly, so the subcrate spelling
/// is the norm rather than the exotic case. Mirrors the `rerun-leanness` job's
/// `^(rerun|re_)`.
fn is_rerun_family(name: &str) -> bool {
    name.starts_with("rerun") || name.starts_with("re_")
}

/// Packages whose name marks them as part of the DDS stack.
///
/// The first two are the published Cerulion forks pinned in
/// `docs/packaging/dds-forks.md`; the upstream spellings are named too, so that
/// dropping a fork cannot silently drop the rule with it.
fn is_dds_stack(name: &str) -> bool {
    matches!(
        name,
        "cerulion-ros2-client" | "cerulion-rustdds" | "ros2-client" | "rustdds"
    )
}

/// The zenoh network transport.
fn is_zenoh(name: &str) -> bool {
    name == "zenoh" || name.starts_with("zenoh-")
}

/// The iceoryx2 shared-memory transport.
fn is_iceoryx2(name: &str) -> bool {
    name == "iceoryx2" || name.starts_with("iceoryx2-")
}

/// The workspace's own runtime crate.
fn is_cerulion_core(name: &str) -> bool {
    name == "cerulion_core"
}

/// The desk render stack, which is what the robot rule actually names: the
/// Rerun SDK family, and the `cerulion_viz` library that wraps it.
///
/// `go2_tf` is NOT in it. That crate sits in the `cerulion_viz` directory but
/// is a pure TFMessage codec with no transport and no rerun, and the demo's
/// producer nodes depend on it deliberately.
fn is_desk_viz(name: &str) -> bool {
    is_rerun_family(name) || name == "cerulion_viz"
}

// ---------------------------------------------------------------------------
// The rules that have no CI job, each beside the sentence it enforces.
// ---------------------------------------------------------------------------

/// A crate whose direct dependency list is pinned to an exact set, with the
/// sentence that pins it.
///
/// Each of these exists to be safe in `default-members`, and each says so by
/// naming what it is allowed to depend on. None of them had a gate.
struct LeanCrate {
    /// Workspace member whose dependencies are pinned.
    package: &'static str,
    /// Every dependency the crate may declare outside `[dev-dependencies]`.
    allowed: &'static [&'static str],
    /// The sentence this rule is the machine half of.
    source: &'static str,
}

const LEAN_CRATES: &[LeanCrate] = &[
    LeanCrate {
        package: "cerulion_discovery",
        // The sentence names four things and the manifest declares five. Its
        // "serde" is the serialization dependency as a whole, and the crate's
        // own manifest says what that means here: "The peer cache is a small
        // JSON document". `serde_json` is that, not a fifth dependency the
        // sentence forgot, so it is allowed and written down rather than left
        // to the next reader to reconcile.
        allowed: &["serde", "serde_json", "dirs", "tracing"],
        source: "crates/cerulion_discovery/Cargo.toml: \"NO transport dep (no cerulion_core, \
                 no iceoryx2, no zenoh, no iroh, no mdns-sd) - this crate is std + serde + \
                 dirs + tracing, so adding it to netd's dependency tree changes nothing \
                 about the CLI's or vizd's leanness.\" docs/internals/network-daemons.md: \
                 \"Dependency policy: std + serde + dirs + tracing ONLY (no transport \
                 crates), so it is safe in `default-members`.\"",
    },
    LeanCrate {
        package: "cerulion_hygiene",
        allowed: &["libc", "tracing"],
        source: "crates/cerulion_hygiene/Cargo.toml: \"std + libc + tracing ONLY: no \
                 transport, no tokio (the wsd wrapper converts to a tokio listener \
                 itself), so it adds nothing to the default build.\"",
    },
];

/// A predicate over a package name: what makes a package a member of a family.
type NamePredicate = fn(&str) -> bool;

/// One forbidden family: the label a failure prints, and what recognises it.
type ForbiddenFamily = (&'static str, NamePredicate);

/// A crate that must not reach a named family through any dependency, with the
/// sentence that says so.
///
/// Checked twice: over the resolved graph rooted at the crate (which is what
/// `cargo build -p <crate>` produces, transitive edges included) and over its
/// own declarations across every dependency kind. The second half is not
/// redundant: it is a DEV edge that put the Rerun SDK on the robot the one time
/// it happened, and a normal-edge resolve cannot see one.
struct ConfinedCrate {
    package: &'static str,
    forbidden: &'static [ForbiddenFamily],
    source: &'static str,
}

const CONFINED_CRATES: &[ConfinedCrate] = &[
    ConfinedCrate {
        package: "cerulion_pairing",
        forbidden: &[("iroh", is_iroh_family), ("rerun", is_rerun_family)],
        source: "docs/internals/remote-access.md crate map: `cerulion_pairing` is \"formats + \
                 crypto + state machines ONLY; no network I/O, no iroh dep (embeds in \
                 firmware and in the closed Studio client)\". Root Cargo.toml: \"Desk-half \
                 identity layer (light, permissive; no rerun/DDS pull).\"",
    },
    ConfinedCrate {
        package: "cerud",
        forbidden: &[("iceoryx2", is_iceoryx2), ("zenoh", is_zenoh)],
        source: "root Cargo.toml, the `members` entry for `cerud`: \"Transport-agnostic (iroh \
                 plugs into the transport seam later); light deps, no iceoryx2/zenoh.\"",
    },
    ConfinedCrate {
        package: "cerulion_link",
        forbidden: &[("cerulion_core", is_cerulion_core)],
        source: "docs/internals/remote-access.md crate map: `cerulion_link` is a \"thin iroh \
                 wrapper: endpoints, dial, framing, relay seam, ops-stream adapter; never \
                 depends on `cerulion_core`\".",
    },
];

/// Members deliberately kept out of `default-members`, each with what a plain
/// `cargo build` would gain if the exclusion were dropped.
///
/// The leanness rules assert the CONSEQUENCE; this one asserts the MECHANISM,
/// so a failing run names the manifest line to restore instead of handing over
/// a dependency path to work backwards from.
const EXCLUDED_FROM_DEFAULT_MEMBERS: &[(&str, &str)] = &[
    ("cerulion_netd", "iroh (its `wan` feature is default-on)"),
    ("cerulion_link", "iroh"),
    (
        "cerulion_wireclient",
        "iroh (it links cerulion_link unconditionally)",
    ),
    ("cerulion_remoted", "iroh"),
    ("cerulion_connectd", "iroh"),
    ("cerulion_accountd", "iroh and the web stack"),
    ("cerulion_viz", "the Rerun SDK"),
    ("cerulion_vizd", "the Rerun SDK"),
];

// ---------------------------------------------------------------------------
// `cargo metadata`, parsed.
// ---------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
struct Metadata {
    packages: Vec<Package>,
    workspace_members: Vec<String>,
    workspace_default_members: Vec<String>,
    resolve: Resolve,
}

#[derive(Debug, Deserialize)]
struct Package {
    id: String,
    name: String,
    manifest_path: String,
    #[serde(default)]
    features: BTreeMap<String, Vec<String>>,
    dependencies: Vec<ManifestDep>,
}

#[derive(Debug, Deserialize)]
struct ManifestDep {
    /// The real package name, dashes and all, never the local alias.
    name: String,
    /// The manifest key when the dependency is renamed. This, not `name`, is
    /// the spelling `dep:`/`pkg/feat` feature values use, and conflating the
    /// two silently drops every renamed optional edge.
    #[serde(default)]
    rename: Option<String>,
    optional: bool,
    uses_default_features: bool,
    #[serde(default)]
    features: Vec<String>,
    /// `null` for a normal dependency, `"dev"` or `"build"` otherwise.
    #[serde(default)]
    kind: Option<String>,
}

impl ManifestDep {
    /// The name this dependency answers to inside its own crate's feature
    /// table: the rename when there is one, the package name otherwise.
    fn local_name(&self) -> &str {
        self.rename.as_deref().unwrap_or(&self.name)
    }
}

#[derive(Debug, Deserialize)]
struct Resolve {
    nodes: Vec<Node>,
}

#[derive(Debug, Deserialize)]
struct Node {
    id: String,
    deps: Vec<NodeDep>,
}

#[derive(Debug, Deserialize)]
struct NodeDep {
    pkg: String,
    dep_kinds: Vec<NodeDepKind>,
}

#[derive(Debug, Deserialize)]
struct NodeDepKind {
    #[serde(default)]
    kind: Option<String>,
}

/// The one `cargo metadata` run, shared by every test in this binary.
fn metadata() -> &'static Metadata {
    static META: OnceLock<Metadata> = OnceLock::new();
    META.get_or_init(|| {
        // `CARGO` is set by cargo for every test it runs, so this is the very
        // toolchain that built the binary rather than whatever is on PATH.
        let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
        let out = Command::new(&cargo)
            .args(["metadata", "--format-version", "1"])
            .current_dir(repo_root())
            .output()
            .unwrap_or_else(|e| panic!("could not run `{cargo} metadata`: {e}"));
        assert!(
            out.status.success(),
            "`{cargo} metadata --format-version 1` failed ({}):\n{}\n\
             These rules are stated over the resolve graph and must FAIL rather than skip \
             when the workspace does not resolve.",
            out.status,
            String::from_utf8_lossy(&out.stderr),
        );
        serde_json::from_slice(&out.stdout).expect("cargo metadata emits valid JSON")
    })
}

/// The workspace root, two levels up from this crate.
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("the crate directory has a parent")
        .parent()
        .expect("crates/ has a parent (the repo root)")
        .to_path_buf()
}

// ---------------------------------------------------------------------------
// The feature-aware resolver.
// ---------------------------------------------------------------------------

/// Normal dependencies alone, the edge set `cargo tree -e normal` walks. Used
/// by the oracle test, which compares against exactly that command.
const NORMAL: &[Option<&str>] = &[None];

/// What a plain `cargo build` actually compiles: normal dependencies and the
/// build scripts' own dependencies.
///
/// Wider than the `cargo tree -e normal` probes in CI, on purpose. A build
/// dependency on a forbidden crate compiles that crate during a `cargo build`
/// exactly as a normal one does, and the jobs cannot see it. Including build
/// edges adds 9 packages to the default-member closure and no member of any
/// forbidden family, so it closes that hole at no cost. Dev edges are NOT
/// included: cargo builds those only for the package being tested, and the
/// rules that turn on a dev edge read the manifests directly instead.
const NORMAL_AND_BUILD: &[Option<&str>] = &[None, Some("build")];

/// An indexed view of one `cargo metadata` document.
struct Graph<'m> {
    by_id: BTreeMap<&'m str, &'m Package>,
    nodes: BTreeMap<&'m str, &'m Node>,
    meta: &'m Metadata,
}

/// The normal-edge graph cargo would build from a chosen set of roots.
struct Resolved<'m> {
    roots: Vec<&'m str>,
    /// Activated normal edges, package id -> package ids.
    edges: BTreeMap<&'m str, BTreeSet<&'m str>>,
}

impl<'m> Graph<'m> {
    fn new(meta: &'m Metadata) -> Self {
        Self {
            by_id: meta.packages.iter().map(|p| (p.id.as_str(), p)).collect(),
            nodes: meta
                .resolve
                .nodes
                .iter()
                .map(|n| (n.id.as_str(), n))
                .collect(),
            meta,
        }
    }

    fn name(&self, id: &str) -> &'m str {
        &self.by_id[id].name
    }

    /// The id of the workspace member called `name`.
    fn member(&self, name: &str) -> &'m str {
        self.meta
            .workspace_members
            .iter()
            .map(String::as_str)
            .find(|id| self.name(id) == name)
            .unwrap_or_else(|| {
                panic!(
                    "`{name}` is not a workspace member. A rule naming a package that no \
                     longer exists is a rule nobody is enforcing: rename it here, or retire \
                     the rule deliberately."
                )
            })
    }

    /// The package ids cargo resolved manifest dependency `dep` of `owner` to.
    ///
    /// Joined on the real package name plus the dependency kind. Two versions
    /// of one package can sit behind a single name (four such edges exist in
    /// this tree, all inside third-party crates), and the answer is then BOTH,
    /// which is the fail-closed direction: an extra edge can only make a
    /// "nothing forbidden is reachable" rule stricter, never weaker.
    fn edge_targets(&self, owner: &str, dep: &ManifestDep) -> Vec<&'m str> {
        self.nodes[owner]
            .deps
            .iter()
            .filter(|e| self.name(&e.pkg) == dep.name)
            .filter(|e| e.dep_kinds.iter().any(|k| k.kind == dep.kind))
            .map(|e| self.by_id[e.pkg.as_str()].id.as_str())
            .collect()
    }

    /// `pkg`'s feature table, plus the implicit feature every optional
    /// dependency gets when no feature turns it on with `dep:`.
    fn feature_table(&self, pkg: &'m Package) -> BTreeMap<String, Vec<String>> {
        let mut named_by_dep_syntax = BTreeSet::new();
        for values in pkg.features.values() {
            for v in values {
                if let Some(rest) = v.strip_prefix("dep:") {
                    named_by_dep_syntax.insert(rest.to_string());
                }
            }
        }
        let mut table = pkg.features.clone();
        for d in &pkg.dependencies {
            let local = d.local_name().to_string();
            if d.optional && !named_by_dep_syntax.contains(&local) {
                table
                    .entry(local.clone())
                    .or_insert_with(|| vec![format!("dep:{local}")]);
            }
        }
        table
    }

    /// Run cargo's feature algorithm over `kinds` edges from `roots`, each root
    /// taken with its default features, and report the activated graph.
    fn resolve(&self, roots: &[&'m str], kinds: &[Option<&str>]) -> Resolved<'m> {
        let mut requested: BTreeMap<&'m str, BTreeSet<String>> = BTreeMap::new();
        let mut with_defaults: BTreeSet<&'m str> = BTreeSet::new();
        let mut edges: BTreeMap<&'m str, BTreeSet<&'m str>> = BTreeMap::new();
        let mut reached: BTreeSet<&'m str> = BTreeSet::new();
        for root in roots {
            requested.entry(root).or_default();
            with_defaults.insert(root);
            reached.insert(root);
        }

        loop {
            let mut changed = false;
            for id in requested.keys().copied().collect::<Vec<_>>() {
                let pkg = self.by_id[id];
                let table = self.feature_table(pkg);
                let mut want = requested[id].clone();
                if with_defaults.contains(id) && table.contains_key("default") {
                    want.insert("default".to_string());
                }
                let (deps_on, dep_features) = close_features(&table, &want);

                let mut out: BTreeSet<&'m str> = BTreeSet::new();
                for dep in &pkg.dependencies {
                    if !kinds.contains(&dep.kind.as_deref()) {
                        continue;
                    }
                    if dep.optional && !deps_on.contains(dep.local_name()) {
                        continue;
                    }
                    for target in self.edge_targets(id, dep) {
                        out.insert(target);
                        let entry = requested.entry(target).or_default();
                        let before = entry.len();
                        entry.extend(dep.features.iter().cloned());
                        if let Some(extra) = dep_features.get(dep.local_name()) {
                            entry.extend(extra.iter().cloned());
                        }
                        changed |= entry.len() != before;
                        changed |= dep.uses_default_features && with_defaults.insert(target);
                        changed |= reached.insert(target);
                    }
                }
                if edges.get(id) != Some(&out) {
                    edges.insert(id, out);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }

        Resolved {
            roots: roots.to_vec(),
            edges,
        }
    }
}

/// Expand `want` over `table` to a fixed point and report what it turns on: the
/// local names of the dependencies activated, and the features requested on
/// each of them.
///
/// The three feature-value spellings cargo understands are all handled, and the
/// difference between them is the whole reason this is computed rather than
/// read off a list of edges: `dep:foo` turns an optional dependency on without
/// creating a feature, `foo/bar` turns it on AND asks it for `bar`, and
/// `foo?/bar` asks only if something else already turned it on.
fn close_features(
    table: &BTreeMap<String, Vec<String>>,
    want: &BTreeSet<String>,
) -> (BTreeSet<String>, BTreeMap<String, BTreeSet<String>>) {
    let mut deps_on: BTreeSet<String> = BTreeSet::new();
    let mut dep_features: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut weak: Vec<(String, String)> = Vec::new();
    let mut visited: BTreeSet<String> = BTreeSet::new();
    let mut stack: Vec<String> = want.iter().cloned().collect();

    while let Some(feature) = stack.pop() {
        if !visited.insert(feature.clone()) {
            continue;
        }
        let Some(values) = table.get(&feature) else {
            // A feature nothing defines enables nothing. This is also how an
            // optional dependency's implicit feature behaves once some other
            // feature has claimed it with `dep:`.
            continue;
        };
        for value in values {
            if let Some(dep) = value.strip_prefix("dep:") {
                deps_on.insert(dep.to_string());
            } else if let Some((dep, feat)) = value.split_once("?/") {
                weak.push((dep.to_string(), feat.to_string()));
            } else if let Some((dep, feat)) = value.split_once('/') {
                deps_on.insert(dep.to_string());
                dep_features
                    .entry(dep.to_string())
                    .or_default()
                    .insert(feat.to_string());
            } else {
                stack.push(value.clone());
            }
        }
    }
    // A weak value can only add a feature to an already-live dependency, never
    // bring one to life, so one pass after the closure is complete is enough.
    for (dep, feat) in weak {
        if deps_on.contains(&dep) {
            dep_features.entry(dep).or_default().insert(feat);
        }
    }
    (deps_on, dep_features)
}

impl<'m> Resolved<'m> {
    /// The names of every package reachable from the roots.
    fn reachable(&self, g: &Graph<'m>) -> BTreeSet<&'m str> {
        let mut seen: BTreeSet<&'m str> = self.roots.iter().copied().collect();
        let mut queue: VecDeque<&'m str> = self.roots.iter().copied().collect();
        while let Some(id) = queue.pop_front() {
            for next in self.edges.get(id).into_iter().flatten() {
                if seen.insert(next) {
                    queue.push_back(next);
                }
            }
        }
        seen.into_iter().map(|id| g.name(id)).collect()
    }

    /// One shortest dependency path, as package names, from a root to each
    /// reachable package whose name satisfies `forbidden`, optionally with one
    /// package removed from the graph.
    ///
    /// THE checker. Every leanness rule, the confinement rules and the
    /// anti-tautology control call this one function; only the roots, the
    /// predicate and the cut change. A path rather than a name, because a
    /// violation nobody can trace back to an edge is a red nobody can fix.
    fn violations(
        &self,
        g: &Graph<'m>,
        cut: Option<&str>,
        forbidden: NamePredicate,
    ) -> Vec<String> {
        let mut parent: BTreeMap<&'m str, &'m str> = BTreeMap::new();
        let mut seen: BTreeSet<&'m str> = self.roots.iter().copied().collect();
        let mut queue: VecDeque<&'m str> = self.roots.iter().copied().collect();
        let mut hits: Vec<&'m str> = Vec::new();
        while let Some(id) = queue.pop_front() {
            if Some(id) == cut {
                continue;
            }
            if forbidden(g.name(id)) {
                hits.push(id);
            }
            for next in self.edges.get(id).into_iter().flatten() {
                if Some(*next) != cut && seen.insert(next) {
                    parent.insert(next, id);
                    queue.push_back(next);
                }
            }
        }
        hits.iter()
            .map(|hit| {
                let mut chain = vec![g.name(hit)];
                let mut cursor = *hit;
                while let Some(up) = parent.get(cursor) {
                    chain.push(g.name(up));
                    cursor = up;
                }
                chain.reverse();
                chain.join(" -> ")
            })
            .collect()
    }
}

/// The ids of every default member, the roots of a plain `cargo build`.
fn default_member_ids(meta: &Metadata) -> Vec<&str> {
    meta.workspace_default_members
        .iter()
        .map(String::as_str)
        .collect()
}

// ---------------------------------------------------------------------------
// The forbidden families are not vacuous.
// ---------------------------------------------------------------------------

/// A family predicate that matches nothing in the whole workspace would let
/// every rule using it pass while proving nothing.
///
/// This is the `INVARIANT UNVERIFIABLE` arm both CI leanness jobs open with,
/// moved next to the rules it protects: they derive their family from
/// `Cargo.lock` and refuse to run on an empty one.
#[test]
fn the_forbidden_families_exist_in_this_workspace() {
    let g = Graph::new(metadata());
    let all: BTreeSet<&str> = g.by_id.values().map(|p| p.name.as_str()).collect();
    for (label, pred) in [
        ("iroh", is_iroh_family as NamePredicate),
        ("rerun", is_rerun_family),
        ("the desk viz stack", is_desk_viz),
        ("the DDS stack", is_dds_stack),
        ("zenoh", is_zenoh),
        ("iceoryx2", is_iceoryx2),
        ("cerulion_core", is_cerulion_core),
    ] {
        assert!(
            all.iter().copied().any(pred),
            "no package matching {label} appears in this workspace's resolve at all, so every \
             rule written over that family passes vacuously. Either the predicate stopped \
             matching the real crate names, or the tree genuinely dropped {label} and the \
             rules about it should be retired deliberately."
        );
    }
}

// ---------------------------------------------------------------------------
// The resolver agrees with cargo.
// ---------------------------------------------------------------------------

/// The resolver in this file is the thing every rule rests on, so it gets an
/// oracle that is not itself: `cargo tree -e normal`, cargo's own resolution
/// reached by a different command.
///
/// SUBSET, NOT EQUALITY, and the direction is the point. `cargo tree` filters
/// to the host platform and this file filters to none, so the computed set is a
/// superset and the extras are `cfg`-gated crates for other targets. What must
/// never happen is the other direction: a package cargo says the default build
/// pulls and this file does not see is a package a forbidden crate could hide
/// behind. MEASURED while writing this: 464 reported, 0 missing, 65 extras.
///
/// The floor keeps the oracle from passing on silence: a `cargo tree` that
/// printed nothing (a broken invocation, a changed output format) would satisfy
/// a bare subset check forever.
#[test]
fn the_resolver_covers_every_package_cargo_tree_reports() {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string());
    let out = Command::new(&cargo)
        .args([
            "tree", "-e", "normal", "--prefix", "none", "--format", "{p}",
        ])
        .current_dir(repo_root())
        .output()
        .unwrap_or_else(|e| panic!("could not run `{cargo} tree`: {e}"));
    assert!(
        out.status.success(),
        "`{cargo} tree -e normal` failed ({}):\n{}",
        out.status,
        String::from_utf8_lossy(&out.stderr),
    );

    // Each line is `<name> v<version>[ (<source>)][ (*)]`; the leading name is
    // all this needs, and a duplicate line collapses into the set.
    let reported: BTreeSet<&str> = std::str::from_utf8(&out.stdout)
        .expect("cargo tree emits UTF-8")
        .lines()
        .filter_map(|line| line.split_once(" v"))
        .map(|(name, _)| name.trim())
        .filter(|name| !name.is_empty())
        .collect();
    assert!(
        reported.len() > 100,
        "`cargo tree -e normal` reported only {} packages for a workspace whose default build \
         resolves several hundred. A subset check against an empty or truncated oracle passes \
         forever, so this reads as a broken invocation rather than a lean tree.",
        reported.len(),
    );

    let meta = metadata();
    let g = Graph::new(meta);
    let computed = g.resolve(&default_member_ids(meta), NORMAL).reachable(&g);
    let missing: Vec<&&str> = reported.difference(&computed).collect();
    assert!(
        missing.is_empty(),
        "cargo resolves {} package(s) into the default build that this file's resolver does \
         not see: {missing:?}. Every rule here is stated over that set, so a package missing \
         from it is a place a forbidden crate can sit unreported.",
        missing.len(),
    );
}

// ---------------------------------------------------------------------------
// Rule: the default build stays iroh-free; the shipped netd still pulls iroh.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, job
/// `Iroh leanness (default build stays iroh-free)`, probe (a): "INVARIANT
/// VIOLATED: $pkg is in the default-members dependency graph. / The CLI + vizd
/// default build must stay iroh-free: keep cerulion_netd OUT of
/// default-members."
///
/// Root `Cargo.toml`: "`cerulion_netd` is a `members` entry but is DELIBERATELY
/// NOT in `default-members` - its `wan` feature is default-on, and a
/// default-member netd would unify the iroh tree into the CLI + vizd via
/// per-package feature unification."
#[test]
fn default_member_build_is_iroh_free() {
    let meta = metadata();
    let g = Graph::new(meta);
    let paths = g
        .resolve(&default_member_ids(meta), NORMAL_AND_BUILD)
        .violations(&g, None, is_iroh_family);
    assert!(
        paths.is_empty(),
        "the default build reaches the iroh tree. A `cargo build` with no `-p` must stay \
         iroh-free: keep cerulion_netd, cerulion_link, cerulion_wireclient, \
         cerulion_remoted, cerulion_connectd and cerulion_accountd out of \
         `default-members`, and keep lean consumers on `default-features = false`.\n\
         Paths:\n  {}",
        paths.join("\n  "),
    );
}

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, same job, probe (b):
/// "INVARIANT VIOLATED: cerulion_netd (per-package) no longer pulls iroh. / The
/// shipped netd daemon must include the iroh WAN plane (wan default-on)."
/// `crates/cerulion_netd/AGENTS.md`: "`wan` is DEFAULT-ON and netd is
/// deliberately OUT of workspace `default-members`."
///
/// ALSO THE RESOLVER'S POSITIVE CONTROL, and the reason the rule above can be
/// believed. The two differ by nothing but the root set: netd reached as a root
/// takes its default features and pulls iroh through an optional `dep:` edge;
/// netd reached through `cerulion_cli_engine` does not, because that edge says
/// `default-features = false`. A resolver that ignored features, or dropped
/// optional edges, would report both clean and make the leanness rule
/// meaningless. It fails here instead.
#[test]
fn netd_per_package_build_pulls_the_iroh_wan_plane() {
    let g = Graph::new(metadata());
    let netd = g.member("cerulion_netd");
    let reached = g
        .resolve(&[netd], NORMAL_AND_BUILD)
        .violations(&g, None, is_iroh_family);
    assert!(
        !reached.is_empty(),
        "`cargo build -p cerulion_netd` no longer reaches iroh. Either the shipped daemon lost \
         its WAN plane (`wan` must stay default-on), or this file's resolver stopped following \
         default features and optional `dep:` edges, in which case every leanness rule here is \
         passing vacuously."
    );
}

// ---------------------------------------------------------------------------
// Rule: the default build stays rerun-free; the desk viz lib still pulls rerun.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, job
/// `Rerun leanness (robot + default build stay rerun-free)`, probe (a): "A
/// plain 'cargo build' must stay rerun-free: keep the cerulion_viz/* crates OUT
/// of default-members."
/// `crates/cerulion_viz/AGENTS.md`: "NOT default-members: a plain `cargo build`
/// must stay rerun-free (CI's rerun-leanness job enforces it); build with
/// `-p <crate>`."
#[test]
fn default_member_build_is_rerun_free() {
    let meta = metadata();
    let g = Graph::new(meta);
    let paths = g
        .resolve(&default_member_ids(meta), NORMAL_AND_BUILD)
        .violations(&g, None, is_rerun_family);
    assert!(
        paths.is_empty(),
        "the default build reaches the Rerun SDK. Rasterization is desk-side: keep \
         cerulion_viz and cerulion_vizd out of `default-members` and build them with \
         `-p <crate>`.\nPaths:\n  {}",
        paths.join("\n  "),
    );
}

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, same job, control (c1): "PROBE
/// BROKEN: cerulion_viz (per-package) no longer pulls rerun. / Either the desk
/// viz stack lost its rerun edge, or this job's probe is no longer probing
/// anything."
#[test]
fn desk_viz_still_pulls_rerun() {
    let g = Graph::new(metadata());
    let viz = g.member("cerulion_viz");
    let reached = g
        .resolve(&[viz], NORMAL_AND_BUILD)
        .violations(&g, None, is_rerun_family);
    assert!(
        !reached.is_empty(),
        "`cargo build -p cerulion_viz` no longer reaches rerun. Either the desk viz stack lost \
         its rerun edge, or the rerun rule above is probing nothing."
    );
}

// ---------------------------------------------------------------------------
// Rule: the robot stays rerun-free, dev edges included.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, job
/// `Rerun leanness (robot + default build stay rerun-free)`, probe (b):
/// "INVARIANT VIOLATED: a examples/go2 crate pulls $pkg (project rule: no viz
/// or rerun on the robot). / Robot crates ship RAW frames; rasterization is
/// desk-side." The job probes `-e normal,dev` because a DEV edge is exactly how
/// the Rerun SDK reached the robot the one time it did.
///
/// STRICTER THAN THE JOB, AND HERMETIC. `examples/go2` is a separate workspace
/// that patches a DDS fork in by git, so resolving it wants a warm git cache; a
/// test that reaches the network to prove an invariant goes red for the wrong
/// reason. Both halves are read from the committed artifacts instead, and
/// together they cover more than the job's probe: the lockfile is every
/// transitive package under every edge kind on every platform, and the manifest
/// walk catches a freshly declared edge whose lockfile update has not been
/// committed yet.
#[test]
fn the_robot_demo_workspace_is_rerun_free() {
    let go2 = repo_root().join("examples/go2");

    let locked = lockfile_packages(&go2.join("Cargo.lock"));
    let offending: Vec<&String> = locked.iter().filter(|n| is_desk_viz(n)).collect();
    assert!(
        offending.is_empty(),
        "examples/go2/Cargo.lock carries the Rerun SDK: {offending:?}. Robot crates ship RAW \
         frames; rasterization is desk-side. If this is a TEST that needs the viz stack, put \
         it in crates/cerulion_viz/lib/cerulion_viz/tests/ and dev-depend on the robot crate \
         from there (see tf_source_e2e_test.rs), not the other way round."
    );

    let manifests = workspace_manifests(&go2);
    assert!(
        manifests.len() >= 3,
        "the examples/go2 manifest walk found {} manifests, so it is not walking the \
         workspace it claims to walk",
        manifests.len(),
    );
    for manifest in manifests {
        let declared = declared_dependencies(&manifest);
        let offending: Vec<&String> = declared.iter().filter(|n| is_desk_viz(n)).collect();
        assert!(
            offending.is_empty(),
            "{} declares {offending:?}. No examples/go2 crate may take a viz or rerun dependency of \
             any kind, dev-dependencies included.",
            manifest.display(),
        );
    }
}

/// SOURCE SENTENCE. `.github/workflows/ci.yml`, same job, controls (c2) and
/// (c3): "PROBE BROKEN: the examples/go2 reverse-tree probe found no go2_tf
/// edge" and "PROBE BROKEN: probe (b) no longer traverses DEV dependencies. /
/// tempfile is a dev-only dependency of examples/go2's dds_bridge and MUST be
/// reachable under '-e normal,dev'."
///
/// The same two controls against the same two packages, for the reader above:
/// `go2_tf` proves it is reading the robot workspace and not the root one, and
/// `tempfile`, reachable in that workspace only through a dev edge, proves
/// dev-only packages are visible to it at all.
#[test]
fn the_robot_workspace_reader_sees_normal_and_dev_packages() {
    let locked = lockfile_packages(&repo_root().join("examples/go2/Cargo.lock"));
    for (pkg, why) in [
        ("go2_tf", "a normal dependency of the demo's producer nodes"),
        ("tempfile", "a DEV-only dependency of the demo's dds_bridge"),
    ] {
        assert!(
            locked.iter().any(|n| n == pkg),
            "examples/go2/Cargo.lock does not list `{pkg}` ({why}), so the rerun rule above is \
             reading the wrong file, or a lockfile that no longer covers this workspace"
        );
    }
}

/// Every package name in a lockfile.
fn lockfile_packages(path: &Path) -> Vec<String> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}): this rule is stated over that lockfile and must FAIL \
             rather than skip",
            path.display()
        )
    });
    text.lines()
        .filter_map(|line| line.strip_prefix("name = \""))
        .filter_map(|line| line.strip_suffix('"'))
        .map(str::to_string)
        .collect()
}

/// Every `Cargo.toml` of a workspace: the root manifest and each member,
/// including members reached through a `dir/*` glob.
fn workspace_manifests(root: &Path) -> Vec<PathBuf> {
    let text = std::fs::read_to_string(root.join("Cargo.toml"))
        .unwrap_or_else(|e| panic!("could not read {}/Cargo.toml ({e})", root.display()));
    let doc: toml::Table = text.parse().expect("a workspace manifest is valid TOML");
    let members = doc
        .get("workspace")
        .and_then(toml::Value::as_table)
        .and_then(|w| w.get("members"))
        .and_then(toml::Value::as_array)
        .unwrap_or_else(|| {
            panic!(
                "{}/Cargo.toml declares no workspace members",
                root.display()
            )
        });

    let mut out = vec![root.join("Cargo.toml")];
    for member in members {
        let pattern = member.as_str().expect("a member entry is a string");
        match pattern.strip_suffix("/*") {
            Some(dir) => {
                let listing = std::fs::read_dir(root.join(dir)).unwrap_or_else(|e| {
                    panic!("`{pattern}` names {dir}, which cannot be listed ({e})")
                });
                for entry in listing {
                    let path = entry.expect("a readable directory entry").path();
                    if path.join("Cargo.toml").is_file() {
                        out.push(path.join("Cargo.toml"));
                    }
                }
            }
            None => {
                assert!(
                    !pattern.contains('*'),
                    "{}/Cargo.toml uses the member pattern `{pattern}`, which this walk does \
                     not expand. Teach it the pattern rather than let the rule skip a crate.",
                    root.display(),
                );
                out.push(root.join(pattern).join("Cargo.toml"));
            }
        }
    }
    out
}

/// Every dependency name declared anywhere in a manifest, across all three
/// dependency kinds and every `[target.*]` block, counting both the manifest
/// key and any `package = "..."` rename target.
fn declared_dependencies(manifest: &Path) -> BTreeSet<String> {
    let text = std::fs::read_to_string(manifest)
        .unwrap_or_else(|e| panic!("could not read {} ({e})", manifest.display()));
    let doc: toml::Table = text
        .parse()
        .unwrap_or_else(|e| panic!("{} is not valid TOML ({e})", manifest.display()));
    let mut out = BTreeSet::new();
    collect_dependency_names(&doc, &mut out);
    out
}

/// Walk a manifest table for every `*dependencies` table, however nested.
fn collect_dependency_names(table: &toml::Table, out: &mut BTreeSet<String>) {
    for (key, value) in table {
        if key.ends_with("dependencies") {
            if let Some(deps) = value.as_table() {
                for (name, spec) in deps {
                    out.insert(name.clone());
                    if let Some(real) = spec.get("package").and_then(toml::Value::as_str) {
                        out.insert(real.to_string());
                    }
                }
            }
        } else if let Some(sub) = value.as_table() {
            collect_dependency_names(sub, out);
        }
    }
}

// ---------------------------------------------------------------------------
// Rule: the DDS stack is confined to cerulion_dds.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCE. `crates/cerulion_dds/AGENTS.md`: "This is the ONLY
/// main-workspace crate that pulls the DDS stack. Keep it that way:
/// `cerulion_cli_engine` depends with `default-features = false` (DDS-free
/// vocabulary only, `live` off), so the engine's isolated builds never compile
/// rustdds." Root `Cargo.toml`: "The ONLY main-workspace crate that pulls the
/// DDS stack; keep the command logic pure in cerulion_cli_engine::ros_cmd so
/// this crate is the sole flip point."
///
/// Two halves, because the sentence makes two claims. No other member may
/// DECLARE a DDS dependency of any kind, and in the resolved default build
/// every route to the DDS stack must pass through `cerulion_dds`: cut that one
/// package out of the graph and the stack becomes unreachable. The second half
/// is what "the sole flip point" means, and only a graph can say it.
#[test]
fn the_dds_stack_is_confined_to_cerulion_dds() {
    let meta = metadata();
    let g = Graph::new(meta);

    for id in &meta.workspace_members {
        let pkg = g.by_id[id.as_str()];
        if pkg.name == "cerulion_dds" {
            continue;
        }
        let offending: Vec<&str> = pkg
            .dependencies
            .iter()
            .map(|d| d.name.as_str())
            .filter(|n| is_dds_stack(n))
            .collect();
        assert!(
            offending.is_empty(),
            "{} declares {offending:?}. cerulion_dds is the ONLY main-workspace crate that may \
             pull the DDS stack: depend on cerulion_dds instead, and keep the command logic \
             pure in cerulion_cli_engine::ros_cmd.",
            pkg.manifest_path,
        );
    }

    let resolved = g.resolve(&default_member_ids(meta), NORMAL_AND_BUILD);
    let dds = g.member("cerulion_dds");
    let bypassing = resolved.violations(&g, Some(dds), is_dds_stack);
    assert!(
        bypassing.is_empty(),
        "the default build reaches the DDS stack without going through cerulion_dds, so \
         cerulion_dds is no longer the single flip point its AGENTS.md promises.\n\
         Paths:\n  {}",
        bypassing.join("\n  "),
    );
}

// ---------------------------------------------------------------------------
// Rules: the lean crates stay lean.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCES: the `source` field of each [`LeanCrate`] entry, printed
/// with any failure.
///
/// The allowed list is checked as an EXACT set. An arrival is a new cost to
/// every `cargo build`; a departure that nobody removed from the sentence
/// leaves a stale allowance behind, which pre-authorises the next arrival.
#[test]
fn the_lean_crates_declare_exactly_their_allowed_dependencies() {
    let g = Graph::new(metadata());
    for rule in LEAN_CRATES {
        let pkg = g.by_id[g.member(rule.package)];
        let declared: BTreeSet<&str> = pkg
            .dependencies
            .iter()
            // Everything but `[dev-dependencies]`, which is what the rule
            // says: a BUILD dependency compiles during a plain `cargo build`
            // as surely as a normal one, so excluding it here would leave a
            // lean crate able to take one and stay green.
            .filter(|d| d.kind.as_deref() != Some("dev"))
            .map(|d| d.name.as_str())
            .collect();
        let allowed: BTreeSet<&str> = rule.allowed.iter().copied().collect();
        let arrived: Vec<&&str> = declared.difference(&allowed).collect();
        let departed: Vec<&&str> = allowed.difference(&declared).collect();
        assert!(
            arrived.is_empty() && departed.is_empty(),
            "{}'s dependencies no longer match the rule it states.\narrived: {arrived:?}\n\
             departed: {departed:?}\nRULE: {}\nChange the sentence and this list together, or \
             not at all.",
            rule.package,
            rule.source,
        );
    }
}

/// SOURCE SENTENCES: the `source` field of each [`ConfinedCrate`] entry,
/// printed with any failure.
#[test]
fn the_confined_crates_reach_nothing_they_forbid() {
    let g = Graph::new(metadata());
    for rule in CONFINED_CRATES {
        let root = g.member(rule.package);
        let resolved = g.resolve(&[root], NORMAL_AND_BUILD);
        let declared: BTreeSet<&str> = g.by_id[root]
            .dependencies
            .iter()
            .map(|d| d.name.as_str())
            .collect();
        for (label, pred) in rule.forbidden {
            let paths = resolved.violations(&g, None, *pred);
            assert!(
                paths.is_empty(),
                "`cargo build -p {}` reaches {label}.\nRULE: {}\nPaths:\n  {}",
                rule.package,
                rule.source,
                paths.join("\n  "),
            );
            let declared_hits: Vec<&&str> = declared.iter().filter(|n| pred(n)).collect();
            assert!(
                declared_hits.is_empty(),
                "{} declares {declared_hits:?}; a dev or build dependency counts.\nRULE: {}",
                rule.package,
                rule.source,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Rule: the exclusions from default-members are still in place.
// ---------------------------------------------------------------------------

/// SOURCE SENTENCES. Root `Cargo.toml`: "`cerulion_netd` is a `members` entry
/// but is DELIBERATELY NOT in `default-members`."
/// `crates/cerulion_remoted/AGENTS.md`: "remoted, connectd, wireclient, link,
/// accountd are OUT of `default-members`; build them with `-p` (a plain `cargo
/// build` stays iroh-free)."
/// `crates/cerulion_viz/AGENTS.md`: "NOT default-members."
#[test]
fn the_heavy_members_stay_out_of_default_members() {
    let meta = metadata();
    let g = Graph::new(meta);
    let default: BTreeSet<&str> = meta
        .workspace_default_members
        .iter()
        .map(|id| g.name(id))
        .collect();
    let members: BTreeSet<&str> = meta.workspace_members.iter().map(|id| g.name(id)).collect();
    for (package, what_it_pulls) in EXCLUDED_FROM_DEFAULT_MEMBERS {
        assert!(
            members.contains(package),
            "`{package}` is named here as a deliberate `default-members` exclusion but is not a \
             workspace member at all; a stale exclusion hides the next real one"
        );
        assert!(
            !default.contains(package),
            "`{package}` is in `default-members`, which puts {what_it_pulls} into every plain \
             `cargo build`. It is a `members` entry on purpose; build it with `-p {package}`."
        );
    }
}

// ---------------------------------------------------------------------------
// Anti-tautology: the checker fires.
// ---------------------------------------------------------------------------

/// Every rule above asserts that a checker found NOTHING. A checker that can
/// never find anything satisfies all of them, and a resolver that activated no
/// optional edges would be exactly that checker.
///
/// So [`Resolved::violations`] is pointed at roots that legitimately carry each
/// forbidden family and required to complain. Three roots, one per mechanism
/// the rules depend on:
///
///   * `cerulion_remoted` reaches iroh through a plain normal edge;
///   * `cerulion_netd` reaches it only through an OPTIONAL edge behind a
///     default-on feature (`default = ["wan"]`,
///     `wan = ["dep:cerulion_link", ...]`), the shape a manifest grep cannot
///     see;
///   * `cerulion_viz` reaches the Rerun SDK.
///
/// Each answer must also be a PATH from the named root, so the failure text the
/// rules print is proven to carry the edge a reader has to delete.
#[test]
fn the_checker_reports_a_forbidden_crate_when_one_is_present() {
    let g = Graph::new(metadata());
    for (package, label, pred) in [
        ("cerulion_remoted", "iroh", is_iroh_family as NamePredicate),
        ("cerulion_netd", "iroh", is_iroh_family),
        ("cerulion_viz", "rerun", is_rerun_family),
    ] {
        let root = g.member(package);
        let paths = g
            .resolve(&[root], NORMAL_AND_BUILD)
            .violations(&g, None, pred);
        assert!(
            !paths.is_empty(),
            "the checker found no {label} under `{package}`, which does carry it. Every \
             leanness rule in this file asserts that this same checker finds nothing; if it \
             can find nothing anywhere, none of them proves a thing."
        );
        let first = &paths[0];
        assert!(
            first.starts_with(package) && first.contains(" -> "),
            "the checker reported `{first}` for {package}, which is not a path from that root: \
             a violation nobody can trace back to an edge is a red nobody can fix"
        );
    }
}
