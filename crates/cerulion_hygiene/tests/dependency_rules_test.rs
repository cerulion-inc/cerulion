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
//! COST. Two cargo subprocesses per test binary: one `cargo metadata`, shared
//! through a `OnceLock`, and one `cargo tree` for the oracle. Neither compiles
//! anything, and neither needs the network beyond what resolving the workspace
//! already did to build this binary. The `examples/go2` rules read that
//! workspace's committed manifests and lockfile directly, so they need no
//! second resolve and no network at all.
//!
//! EVERY COUNT in these comments is a measurement taken while the file was
//! written, recorded so a reader knows the order of magnitude. No assertion
//! depends on one, and nothing here needs re-measuring when the tree grows.

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
    is_family(name, "iroh")
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
///
/// The four names are a FLOOR, not a ceiling: each is also matched as a family
/// root, so a future `rustdds-security` or `cerulion-ros2-client-macros` is
/// covered with no edit. A move to a different DDS implementation is not, and
/// would need a name here.
fn is_dds_stack(name: &str) -> bool {
    [
        "cerulion-ros2-client",
        "cerulion-rustdds",
        "ros2-client",
        "rustdds",
    ]
    .iter()
    .any(|root| is_family(name, root))
}

/// True when `name` is `root`, or a subcrate of it under either separator.
///
/// One helper for every transport family, because the separator is not a
/// choice a crate's author makes consistently: the iroh tree spells subcrates
/// with `-`, and published zenoh subcrates appear with both. A family that
/// covered only one spelling would let a direct dependency on the other pull
/// the whole tree while the probe stayed green.
fn is_family(name: &str, root: &str) -> bool {
    name == root
        || (name.len() > root.len()
            && name.starts_with(root)
            && matches!(name.as_bytes()[root.len()], b'-' | b'_'))
}

/// The zenoh network transport.
fn is_zenoh(name: &str) -> bool {
    is_family(name, "zenoh")
}

/// The iceoryx2 shared-memory transport.
fn is_iceoryx2(name: &str) -> bool {
    is_family(name, "iceoryx2")
}

/// The workspace's own runtime crate.
fn is_cerulion_core(name: &str) -> bool {
    name == "cerulion_core"
}

/// The desk render stack, which is what the robot rule actually names: the
/// Rerun SDK family, and the `cerulion_viz` library that wraps it.
///
/// Both viz crates, not just the library: `cerulion_vizd` pulls the SDK through
/// `cerulion_viz`, so a robot crate depending on the daemon would carry the
/// whole tree while a `cerulion_viz`-only check stayed green.
///
/// `go2_tf` is NOT in it. That crate sits in the `cerulion_viz` directory but
/// is a pure TFMessage codec with no transport and no rerun, and the demo's
/// producer nodes depend on it deliberately.
fn is_desk_viz(name: &str) -> bool {
    is_rerun_family(name) || name == "cerulion_viz" || name == "cerulion_vizd"
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
        forbidden: &[
            ("iroh", is_iroh_family),
            ("rerun", is_rerun_family),
            ("the DDS stack", is_dds_stack),
        ],
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
/// EVERY member outside `default-members`, so the list can be checked in both
/// directions: a member that quietly leaves the default set is as much a
/// change to the build as one that joins it, and a row nobody removed
/// pre-authorises the next departure. Two entries carry no heavy tree and say
/// so; they are here because completeness is the property being asserted.
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
    ("cerulion_vizd", "the Rerun SDK (through cerulion_viz)"),
    (
        "cerulion_heaphook",
        "an allocator-interposing cdylib, which a plain build should not link \
         into anything (root Cargo.toml: \"a PRELOAD PAYLOAD, not a workspace tool\")",
    ),
    (
        "go2_tf",
        "nothing heavy (std + thiserror). It sits under crates/cerulion_viz/ \
         beside the crates that do, and stays out with them",
    ),
    (
        "go2_tf_source",
        "nothing heavy. It is a demo node under examples/go2/, built by that \
         workspace rather than by a plain root build",
    ),
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
    version: String,
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
        serde_json::from_slice(&out.stdout).unwrap_or_else(|e| {
            panic!(
                "`cargo metadata` output did not match the shape this file parses ({e}). A \
                 cargo upgrade may have renamed or dropped a field, or `resolve` may be null \
                 (it is, under --no-deps). The structs are at the top of this file."
            )
        })
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

/// What a plain `cargo test` compiles: the above plus each ROOT's own
/// `[dev-dependencies]`.
///
/// Dev edges are followed from the roots and nowhere else, which is cargo's
/// own rule: a dependency's dev-dependencies are built only when that
/// dependency is itself the package under test. [`Graph::resolve`] enforces
/// that; a walk that followed dev edges everywhere would drag in most of
/// crates.io and make every rule fire.
const NORMAL_BUILD_AND_ROOT_DEV: &[Option<&str>] = &[None, Some("build"), Some("dev")];

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

    fn version(&self, id: &str) -> &'m str {
        &self.by_id[id].version
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
    /// of one package can sit behind a single name (four such edges in this
    /// tree, all inside third-party crates), and the answer is then BOTH.
    ///
    /// THAT IS FAIL-CLOSED FOR THE ABSENCE RULES AND FAIL-OPEN FOR THE
    /// PRESENCE ONES, and the difference is worth stating rather than
    /// glossing. An extra edge can only make "nothing forbidden is reachable"
    /// stricter. It makes "something IS reachable" easier, which covers the
    /// three positive controls and the oracle, whose whole job is to catch an
    /// under-count. The oracle compensates by comparing NAME AND VERSION, so a
    /// spurious second version cannot stand in for a missing package. Doing
    /// better here means matching the version requirement, which needs a
    /// semver matcher this file does not otherwise want.
    fn edge_targets(&self, owner: &str, dep: &ManifestDep) -> Vec<&'m str> {
        self.nodes[owner]
            .deps
            .iter()
            .filter(|e| self.name(&e.pkg) == dep.name)
            .filter(|e| e.dep_kinds.iter().any(|k| k.kind == dep.kind))
            // Through `by_id` rather than straight off `e.pkg`: the lookup
            // panics on an id no package declares, which would mean the two
            // halves of one `cargo metadata` document disagree.
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
        // An empty root set resolves to an empty graph, and an empty graph
        // satisfies every "nothing forbidden is reachable" rule in this file.
        // Whatever produced it is a bug here, not a clean tree.
        assert!(
            !roots.is_empty(),
            "resolve() was asked for the closure of no packages at all, which would report \
             every absence rule clean while proving nothing"
        );
        let mut requested: BTreeMap<&'m str, BTreeSet<String>> = BTreeMap::new();
        let mut with_defaults: BTreeSet<&'m str> = BTreeSet::new();
        let mut edges: BTreeMap<&'m str, BTreeSet<&'m str>> = BTreeMap::new();
        for root in roots {
            requested.entry(root).or_default();
            with_defaults.insert(root);
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

                // Cargo builds a package's dev-dependencies only when that
                // package is the one being tested, so a dev edge is followed
                // out of a root and out of nothing else.
                let is_root = roots.contains(&id);
                let mut out: BTreeSet<&'m str> = BTreeSet::new();
                for dep in &pkg.dependencies {
                    if !kinds.contains(&dep.kind.as_deref()) {
                        continue;
                    }
                    if dep.kind.as_deref() == Some("dev") && !is_root {
                        continue;
                    }
                    if dep.optional && !deps_on.contains(dep.local_name()) {
                        continue;
                    }
                    for target in self.edge_targets(id, dep) {
                        out.insert(target);
                        // A package reached for the first time is processed on
                        // the next sweep, so its arrival is itself a change.
                        let first_sighting = !requested.contains_key(target);
                        let entry = requested.entry(target).or_default();
                        let before = entry.len();
                        entry.extend(dep.features.iter().cloned());
                        if let Some(extra) = dep_features.get(dep.local_name()) {
                            entry.extend(extra.iter().cloned());
                        }
                        changed |= entry.len() != before;
                        changed |= dep.uses_default_features && with_defaults.insert(target);
                        changed |= first_sighting;
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
    /// The package ids reachable from the roots.
    fn ids(&self) -> BTreeSet<&'m str> {
        let mut seen: BTreeSet<&'m str> = self.roots.iter().copied().collect();
        let mut queue: VecDeque<&'m str> = self.roots.iter().copied().collect();
        while let Some(id) = queue.pop_front() {
            for next in self.edges.get(id).into_iter().flatten() {
                if seen.insert(next) {
                    queue.push_back(next);
                }
            }
        }
        seen
    }

    /// The names of every package reachable from the roots.
    fn reachable(&self, g: &Graph<'m>) -> BTreeSet<&'m str> {
        self.ids().into_iter().map(|id| g.name(id)).collect()
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

/// The smallest default-build closure that is not a collapse.
///
/// Every absence rule over the default build reports clean on an empty graph,
/// and a resolver that stopped following edges at all would produce one. The
/// closure is around 470 packages as this is written, so a floor of 100 cannot
/// be reached by anything but a genuine break; it is a collapse detector, not
/// a budget, and nothing here asserts the measured figure.
const DEFAULT_BUILD_FLOOR: usize = 100;

/// Panic unless the closure is large enough to be real.
fn refuse_a_collapsed_closure(resolved: &Resolved<'_>, g: &Graph<'_>) {
    let reached = resolved.reachable(g).len();
    assert!(
        reached >= DEFAULT_BUILD_FLOOR,
        "the default-build closure came to {reached} packages, under the floor of \
         {DEFAULT_BUILD_FLOOR}. Every absence rule stated over this set would report clean, so \
         this reads as a resolver that stopped following edges rather than as a lean tree."
    );
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

    // Each line is `<name> v<version>[ (<source>)][ (*)]`. Name AND version,
    // because `edge_targets` answers an ambiguous join with every candidate:
    // comparing names alone would let a spurious second version of a package
    // stand in for the version cargo actually resolved.
    let reported: BTreeSet<(&str, &str)> = std::str::from_utf8(&out.stdout)
        .expect("cargo tree emits UTF-8")
        .lines()
        .filter_map(|line| line.split_once(" v"))
        .map(|(name, rest)| {
            (
                name.trim(),
                rest.split_whitespace().next().unwrap_or("").trim(),
            )
        })
        .filter(|(name, version)| !name.is_empty() && !version.is_empty())
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
    let resolved = g.resolve(&default_member_ids(meta), NORMAL);
    let computed: BTreeSet<(&str, &str)> = resolved
        .ids()
        .into_iter()
        .map(|id| (g.name(id), g.version(id)))
        .collect();
    let missing: Vec<&(&str, &str)> = reported.difference(&computed).collect();
    assert!(
        missing.is_empty(),
        "cargo resolves {} package(s) into the default build that this file's resolver does \
         not see: {missing:?}. Every rule here is stated over that set, so a package missing \
         from it is a place a forbidden crate can sit unreported. The fix is in this file's \
         resolver, not in the tree: start at close_features and Graph::resolve.",
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
    let resolved = g.resolve(&default_member_ids(meta), NORMAL_AND_BUILD);
    refuse_a_collapsed_closure(&resolved, &g);
    let paths = resolved.violations(&g, None, is_iroh_family);
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

/// SOURCE SENTENCE. Root `Cargo.toml`, the `default-members` comment: "`cargo
/// build` / `cargo test` with no `-p`/`--workspace` operate on THIS set, so
/// NEITHER the rerun SDK NOR the iroh tree is compiled on a plain build."
///
/// The sentence names two commands and the rules above cover one of them.
/// `cargo test` additionally compiles each default member's own
/// `[dev-dependencies]`, and that is not a theoretical extra edge:
/// `cerulion_cli_engine` dev-depends by path on `cerulion_accountd`, a member
/// held out of `default-members` to keep the web stack out of a plain build.
/// Nothing on that path reaches a forbidden family today, which is what makes
/// this the right moment to pin it.
#[test]
fn a_plain_cargo_test_compiles_no_iroh_and_no_rerun() {
    let meta = metadata();
    let g = Graph::new(meta);
    let resolved = g.resolve(&default_member_ids(meta), NORMAL_BUILD_AND_ROOT_DEV);
    refuse_a_collapsed_closure(&resolved, &g);
    for (label, pred) in [
        ("the iroh tree", is_iroh_family as NamePredicate),
        ("the Rerun SDK", is_rerun_family),
    ] {
        let paths = resolved.violations(&g, None, pred);
        assert!(
            paths.is_empty(),
            "`cargo test` with no `-p` compiles {label}, through a DEV dependency of a default \
             member. Dev-depend on the heavy crate from the heavy crate's own tests instead, so \
             the edge points away from the default build.\nPaths:\n  {}",
            paths.join("\n  "),
        );
    }
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
    let resolved = g.resolve(&default_member_ids(meta), NORMAL_AND_BUILD);
    refuse_a_collapsed_closure(&resolved, &g);
    let paths = resolved.violations(&g, None, is_rerun_family);
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

    // The walk refuses an empty glob expansion itself, so what is left to
    // check here is that it reached the crates the controls name. A count
    // floor would be a magic number; these two are the demo's producer nodes,
    // and they are what the rerun incident was about.
    let manifests = robot_manifests(&go2);
    for required in [
        go2.join("nodes/dds_bridge"),
        go2.join("nodes/go2_tf_source"),
        // Outside the workspace, and the reason the walk follows `path`:
        // this crate lives in the viz directory and ships to the robot.
        repo_root().join("crates/cerulion_viz/lib/go2_tf"),
    ] {
        let wanted = required.join("Cargo.toml");
        assert!(
            manifests.contains(&wanted),
            "the robot manifest walk did not reach {}, so it is not reading what it claims \
             to read. Check the `members` patterns in examples/go2/Cargo.toml and the `path` \
             dependencies the demo's nodes declare.",
            wanted.display(),
        );
    }
    for manifest in manifests {
        let declared = declared_dependencies(&manifest);
        let offending: Vec<&String> = declared.iter().filter(|n| is_desk_viz(n)).collect();
        assert!(
            offending.is_empty(),
            "{} declares {offending:?}. Nothing the robot builds may take a viz or rerun \
             dependency of any kind, dev-dependencies included. That covers the demo's own \
             crates and every crate they reach by `path`.",
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
/// The rule above has two halves and each gets the control that fits it.
///
///   * the LOCKFILE half is checked with `go2_tf`, a normal dependency of the
///     demo's producer nodes: it proves the reader is on the robot workspace
///     and not the root one (CI's (c2));
///   * the MANIFEST half is checked with `tempfile`, which `dds_bridge`
///     declares ONLY under `[dev-dependencies]`: it proves
///     [`collect_dependency_names`] descends into that table (CI's (c3)).
///
/// The second control has to be the manifest one. A lockfile lists every
/// package in the resolve whatever edge brought it in, so finding `tempfile`
/// there says nothing at all about whether anything in this file can see a dev
/// edge. Narrowing `collect_dependency_names` to `key == "dependencies"` is the
/// edit that reopens the hole L15 closed, and only this arm catches it.
#[test]
fn the_robot_workspace_reader_sees_normal_and_dev_packages() {
    let go2 = repo_root().join("examples/go2");

    let locked = lockfile_packages(&go2.join("Cargo.lock"));
    assert!(
        locked.iter().any(|n| n == "go2_tf"),
        "examples/go2/Cargo.lock does not list `go2_tf`, a normal dependency of the demo's \
         producer nodes, so the lockfile half of the rule above is reading the wrong file or a \
         lockfile that no longer covers this workspace"
    );

    // The fixture first: `tempfile` has to be dev-ONLY in that manifest, or
    // the reader below could find it through `[dependencies]` while the dev
    // table went unread. A control that a fixture change can satisfy is not a
    // control.
    let dds_bridge = go2.join("nodes/dds_bridge/Cargo.toml");
    let manifest: toml::Table = std::fs::read_to_string(&dds_bridge)
        .unwrap_or_else(|e| panic!("could not read {} ({e})", dds_bridge.display()))
        .parse()
        .unwrap_or_else(|e| panic!("{} is not valid TOML ({e})", dds_bridge.display()));
    let in_table = |name: &str, table: &str| {
        manifest
            .get(table)
            .and_then(toml::Value::as_table)
            .is_some_and(|t| t.contains_key(name))
    };
    assert!(
        in_table("tempfile", "dev-dependencies") && !in_table("tempfile", "dependencies"),
        "{} no longer declares `tempfile` under [dev-dependencies] and nowhere else, so it \
         cannot serve as the dev-table control below. Pick another dev-only dependency of a \
         robot crate and name it here, or the control proves nothing.",
        dds_bridge.display(),
    );
    assert!(
        declared_dependencies(&dds_bridge).contains("tempfile"),
        "{} declares `tempfile` under [dev-dependencies] and the manifest reader did not see \
         it, so the manifest half of the rule above no longer traverses dev dependencies. That \
         is the one edge kind the robot rule exists for: check that \
         `collect_dependency_names` still matches every table whose name ends in \
         \"dependencies\", not just `[dependencies]`.",
        dds_bridge.display(),
    );
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

/// Every manifest the robot workspace builds from: its own members, and the
/// crates they reach by `path`, however far outside the workspace those sit.
///
/// The members alone are not the robot's code. `examples/go2`'s producer nodes
/// path-depend on `crates/cerulion_viz/lib/go2_tf`, which lives inside the viz
/// directory, and on `crates/cerulion_core`; a rerun dependency added to one of
/// those ships to the robot exactly as one added to a node does, and a walk
/// over members only would not read the manifest it was added to.
///
/// Dev edges are followed OUT OF A MEMBER and no further. A robot crate's own
/// test dependencies ship to the robot's build; the test dependencies of a
/// library it links do not, and following those would drag in most of the
/// repository and red this rule for something that never reaches a robot.
fn robot_manifests(root: &Path) -> Vec<PathBuf> {
    let members: BTreeSet<PathBuf> = workspace_manifests(root)
        .into_iter()
        .map(lexically_normal)
        .collect();
    let mut queue: Vec<(PathBuf, bool)> = members.iter().cloned().map(|m| (m, true)).collect();
    let mut seen: BTreeSet<PathBuf> = BTreeSet::new();
    let mut out: Vec<PathBuf> = Vec::new();
    while let Some((manifest, reached_as_member)) = queue.pop() {
        if !manifest.is_file() || !seen.insert(manifest.clone()) {
            continue;
        }
        // Membership is a property of the MANIFEST, not of the route taken to
        // it. Two demo nodes path-depend on a third, so a member can be popped
        // as a followed path before its own queue entry comes up; keying only
        // on the route would drop that member's dev edges on the floor.
        let is_member = reached_as_member || members.contains(&manifest);
        out.push(manifest.clone());
        let dir = manifest.parent().expect("a manifest has a directory");
        for path in path_dependencies(&manifest, is_member) {
            queue.push((lexically_normal(dir.join(path).join("Cargo.toml")), false));
        }
    }
    out.sort();
    out
}

/// `path` with `.` and `..` resolved textually, no filesystem access.
///
/// A manifest reached through `../../../../crates/...` is the same file as one
/// named from the repository root, and only a normalized form says so. Without
/// this the walk still READS the right manifests and still scans them, but the
/// control that asserts it reached a named crate compares two spellings of one
/// path and fails. Textual rather than `canonicalize`, because resolving
/// symlinks would answer a different question than the manifests ask.
fn lexically_normal(path: PathBuf) -> PathBuf {
    let mut out = PathBuf::new();
    for part in path.components() {
        match part {
            std::path::Component::ParentDir => {
                out.pop();
            }
            std::path::Component::CurDir => {}
            other => out.push(other),
        }
    }
    out
}

/// Every `path = "..."` a manifest declares, with dev entries included only
/// when `follow_dev` is set.
fn path_dependencies(manifest: &Path, follow_dev: bool) -> Vec<String> {
    let text = std::fs::read_to_string(manifest)
        .unwrap_or_else(|e| panic!("could not read {} ({e})", manifest.display()));
    let doc: toml::Table = text
        .parse()
        .unwrap_or_else(|e| panic!("{} is not valid TOML ({e})", manifest.display()));
    let mut out = Vec::new();
    collect_path_dependencies(&doc, follow_dev, &mut out);
    out
}

fn collect_path_dependencies(table: &toml::Table, follow_dev: bool, out: &mut Vec<String>) {
    for (key, value) in table {
        if key.ends_with("dependencies") {
            if key == "dev-dependencies" && !follow_dev {
                continue;
            }
            if let Some(deps) = value.as_table() {
                for spec in deps.values() {
                    if let Some(path) = spec.get("path").and_then(toml::Value::as_str) {
                        out.push(path.to_string());
                    }
                }
            }
        } else if let Some(sub) = value.as_table() {
            collect_path_dependencies(sub, follow_dev, out);
        }
    }
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
                let mut found = 0usize;
                for entry in listing {
                    let path = entry.expect("a readable directory entry").path();
                    if path.join("Cargo.toml").is_file() {
                        out.push(path.join("Cargo.toml"));
                        found += 1;
                    }
                }
                // A glob that expands to nothing is how this walk goes quiet:
                // every rule stated over it would pass with no crate examined.
                assert!(
                    found > 0,
                    "the member pattern `{pattern}` in {}/Cargo.toml expanded to no crate at \
                     all. Either the directory moved, or every rule stated over this \
                     workspace is examining nothing.",
                    root.display(),
                );
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
///
/// Two things it deliberately does not report. A `[patch]` or `[replace]`
/// entry redirects an existing dependency rather than declaring one, so it is
/// not a name this crate takes on; and a dependency is keyed by NAME, so a
/// `path` or `git` source pointing at a forbidden crate under an innocuous key
/// is invisible here. The lockfile arm of the same rule catches both.
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

    // Rooted at EVERY member, not only the default ones. A non-default member
    // reaching the DDS stack through an intermediate crate declares no DDS
    // dependency of its own, so the scan above cannot see it, and a walk that
    // started at the default members would never visit it either.
    let all_members: Vec<&str> = meta.workspace_members.iter().map(String::as_str).collect();
    let resolved = g.resolve(&all_members, NORMAL_AND_BUILD);
    refuse_a_collapsed_closure(&resolved, &g);
    let dds = g.member("cerulion_dds");
    let bypassing = resolved.violations(&g, Some(dds), is_dds_stack);
    assert!(
        bypassing.is_empty(),
        "a workspace member reaches the DDS stack without going through cerulion_dds, so \
         cerulion_dds is no longer the single flip point its AGENTS.md promises. Route the \
         edge through cerulion_dds, or move the consumer's DDS use into cerulion_dds and \
         depend on the plain types it re-exports.\nPaths:\n  {}",
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
        // Dev edges out of the crate itself, because `cargo test -p <crate>`
        // compiles them: a dev-dependency on an intermediate crate that pulls
        // the forbidden family is a path the declaration scan below cannot
        // see, since all it reads there is the intermediate crate's name.
        let resolved = g.resolve(&[root], NORMAL_BUILD_AND_ROOT_DEV);
        let declared: BTreeSet<&str> = g.by_id[root]
            .dependencies
            .iter()
            .map(|d| d.name.as_str())
            .collect();
        for (label, pred) in rule.forbidden {
            let paths = resolved.violations(&g, None, *pred);
            assert!(
                paths.is_empty(),
                "`cargo test -p {}` reaches {label}.\nRULE: {}\nPaths:\n  {}",
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

    // Both directions, so neither half of the list can go stale on its own.
    let declared: BTreeSet<&str> = EXCLUDED_FROM_DEFAULT_MEMBERS
        .iter()
        .map(|(name, _)| *name)
        .collect();
    let unaccounted: Vec<&&str> = members
        .difference(&default)
        .filter(|name| !declared.contains(**name))
        .collect();
    assert!(
        unaccounted.is_empty(),
        "{unaccounted:?} left `default-members` and nothing here says why. A member drops out \
         of the default build for a reason; write the reason into \
         EXCLUDED_FROM_DEFAULT_MEMBERS, or put the member back."
    );

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
/// The netd root repeats what [`netd_per_package_build_pulls_the_iroh_wan_plane`]
/// already asserts, and that is deliberate: this test must keep proving the
/// checker fires on an optional default-on edge even if the netd rule is ever
/// retired, so it does not lean on it.
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
        // The first SEGMENT, not a prefix: `cerulion_viz` is a prefix of
        // `cerulion_vizd`, so a path rooted at the wrong crate would pass.
        assert!(
            first.split(" -> ").next() == Some(package) && first.contains(" -> "),
            "the checker reported `{first}` for {package}, which is not a path from that root: \
             a violation nobody can trace back to an edge is a red nobody can fix"
        );
    }
}
