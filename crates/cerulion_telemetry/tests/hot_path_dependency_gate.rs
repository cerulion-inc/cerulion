// SPDX-License-Identifier: AGPL-3.0-only
//! Only the desk surfaces may name `cerulion_telemetry`, in any dependency
//! table, optional or not, under any alias. Every other crate the tree builds,
//! root workspace member or robot demo node, is denied by default, so a crate
//! added later fails this gate until it is classified.
//!
//! ALLOWLIST, NOT DENY-LIST. A list of robot crates that must not depend on
//! the telemetry crate is fail-open: a new runtime crate is not on it and
//! takes the dependency with no red check. The list here is the one that may
//! depend, and the walk asserts every member outside it has no edge.
//!
//! The root workspace is read from `cargo metadata --no-deps`, whose
//! per-package `dependencies` is the DECLARED list (an `optional = true` dep
//! behind an off feature is still present, and `name` is the package name with
//! any alias in `rename`), so a feature-gated or renamed edge cannot slip past.
//! The robot demo under `examples/go2` is a separate workspace that patches a
//! DDS fork in by git, so it is read from its committed artifacts instead: the
//! lockfile (every transitive package under every edge kind) and each member
//! manifest (a freshly declared edge whose lockfile update is not committed yet).
//!
//! Direct edges only on the root walk: a transitive route would need a denied
//! crate to depend on a desk crate, which is its own architecture violation.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

const TELEMETRY: &str = "cerulion_telemetry";

/// The desk surfaces: the only workspace members that may depend on the
/// telemetry crate. Everything else is denied.
const DESK: &[&str] = &[
    "cerulion_cli",
    "cerulion_cli_engine",
    "cerulion_cli_tui",
    "cerulion_viz",
    "cerulion_vizd",
    "cerulion_wsd",
];

/// Robot and runtime crates the root walk must have visited. Not a rule of
/// their own (the allowlist denies them already); a control that the walk
/// reads the real workspace and not an empty or renamed one.
const HOT_PATH_CONTROLS: &[&str] = &[
    "cerulion_core",
    "rmw_cerulion",
    "cerulion_bagd",
    "cerulion_remoted",
    "native_ros2_messages",
];

fn crate_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn repo_root() -> PathBuf {
    crate_dir()
        .parent()
        .and_then(Path::parent)
        .expect("crates/<crate> sits two levels under the repository root")
        .to_path_buf()
}

fn metadata() -> serde_json::Value {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let out = Command::new(cargo)
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .current_dir(crate_dir())
        .output()
        .expect("cargo metadata runs");
    assert!(
        out.status.success(),
        "cargo metadata failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).expect("cargo metadata is JSON")
}

/// Every edge of `pkg` whose target is the telemetry crate, rendered for the
/// failure message.
fn telemetry_edges(pkg: &serde_json::Value) -> Vec<String> {
    let name = pkg["name"].as_str().expect("package name");
    pkg["dependencies"]
        .as_array()
        .expect("dependencies array")
        .iter()
        .filter(|dep| dep["name"].as_str() == Some(TELEMETRY))
        .map(|dep| {
            format!(
                "{name} -> {TELEMETRY} (kind={}, optional={}, rename={})",
                dep["kind"].as_str().unwrap_or("normal"),
                dep["optional"],
                dep["rename"]
            )
        })
        .collect()
}

#[test]
fn only_desk_crates_may_name_cerulion_telemetry() {
    let meta = metadata();
    let members: BTreeSet<&str> = meta["workspace_members"]
        .as_array()
        .expect("workspace_members array")
        .iter()
        .map(|id| id.as_str().expect("package id"))
        .collect();
    let mut walked = BTreeSet::new();
    let mut offenders = Vec::new();
    for pkg in meta["packages"].as_array().expect("packages array") {
        if !members.contains(pkg["id"].as_str().expect("package id")) {
            continue;
        }
        let name = pkg["name"].as_str().expect("package name");
        walked.insert(name);
        if name == TELEMETRY || DESK.contains(&name) {
            continue;
        }
        offenders.extend(telemetry_edges(pkg));
    }
    let missing: Vec<&&str> = DESK
        .iter()
        .chain(HOT_PATH_CONTROLS)
        .filter(|c| !walked.contains(**c))
        .collect();
    assert!(
        missing.is_empty(),
        "crates not found among the workspace members (renamed? update DESK or \
         HOT_PATH_CONTROLS): {missing:?}"
    );
    assert!(
        offenders.is_empty(),
        "only the desk surfaces {DESK:?} may depend on {TELEMETRY}; every other \
         workspace member is denied:\n{}",
        offenders.join("\n")
    );
}

/// Package names a `Cargo.lock` lists, every edge kind and platform included.
fn lockfile_packages(path: &Path) -> Vec<String> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| {
        panic!(
            "could not read {} ({e}): the rule is stated over that lockfile and must FAIL \
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

/// The manifests of a workspace's members, expanding the one-level `dir/*`
/// pattern its `members` list uses. Refuses a pattern that matches nothing: a
/// walk over no manifests would pass vacuously.
fn member_manifests(workspace: &Path) -> Vec<PathBuf> {
    let root_manifest = workspace.join("Cargo.toml");
    let text = std::fs::read_to_string(&root_manifest)
        .unwrap_or_else(|e| panic!("could not read {} ({e})", root_manifest.display()));
    let doc: toml::Table = text
        .parse()
        .unwrap_or_else(|e| panic!("{} is not valid TOML ({e})", root_manifest.display()));
    let patterns = doc["workspace"]["members"]
        .as_array()
        .expect("workspace.members array");
    let mut out = Vec::new();
    for pattern in patterns {
        let pattern = pattern.as_str().expect("member pattern string");
        let mut found = Vec::new();
        match pattern.strip_suffix("/*") {
            Some(dir) => {
                let entries = std::fs::read_dir(workspace.join(dir))
                    .unwrap_or_else(|e| panic!("could not list {dir} ({e})"));
                for entry in entries {
                    let manifest = entry.expect("directory entry").path().join("Cargo.toml");
                    if manifest.is_file() {
                        found.push(manifest);
                    }
                }
            }
            None => found.push(workspace.join(pattern).join("Cargo.toml")),
        }
        assert!(
            !found.is_empty(),
            "member pattern {pattern:?} in {} matched no manifest",
            root_manifest.display()
        );
        out.extend(found);
    }
    out.sort();
    out
}

/// Every dependency name a manifest declares, in any `*dependencies` table at
/// any depth (`[target.'cfg(unix)'.dev-dependencies]` counts), plus the real
/// package behind a `package = "..."` alias.
fn declared_dependencies(manifest: &Path) -> BTreeSet<String> {
    fn collect(table: &toml::Table, out: &mut BTreeSet<String>) {
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
                collect(sub, out);
            }
        }
    }
    let text = std::fs::read_to_string(manifest)
        .unwrap_or_else(|e| panic!("could not read {} ({e})", manifest.display()));
    let doc: toml::Table = text
        .parse()
        .unwrap_or_else(|e| panic!("{} is not valid TOML ({e})", manifest.display()));
    let mut out = BTreeSet::new();
    collect(&doc, &mut out);
    out
}

#[test]
fn the_robot_demo_workspace_never_names_cerulion_telemetry() {
    let go2 = repo_root().join("examples/go2");

    let locked = lockfile_packages(&go2.join("Cargo.lock"));
    assert!(
        locked.iter().any(|n| n == "cerulion_core"),
        "the demo lockfile does not list cerulion_core, which every demo node depends on, \
         so this is not the robot workspace's lockfile"
    );
    assert!(
        !locked.iter().any(|n| n == TELEMETRY),
        "the demo lockfile carries {TELEMETRY}: a robot node, or a crate one reaches, took \
         the dependency"
    );

    let manifests = member_manifests(&go2);
    let mut saw_control = false;
    for manifest in &manifests {
        let declared = declared_dependencies(manifest);
        saw_control |= declared.contains("cerulion_core");
        assert!(
            !declared.contains(TELEMETRY),
            "{} declares {TELEMETRY}. Robot crates never carry usage telemetry; the desk \
             surfaces are the only callers.",
            manifest.display()
        );
    }
    assert!(
        saw_control,
        "no demo manifest declares cerulion_core, so the manifest reader is not reading the \
         dependency tables it claims to read"
    );
}

#[test]
fn telemetry_default_features_are_off_and_posthog_owns_the_network_deps() {
    let meta = metadata();
    let pkg = meta["packages"]
        .as_array()
        .expect("packages array")
        .iter()
        .find(|p| p["name"] == TELEMETRY)
        .expect("cerulion_telemetry is a workspace member");
    let features = pkg["features"].as_object().expect("features table");
    assert_eq!(
        features["default"].as_array().map(Vec::len),
        Some(0),
        "default features must be empty"
    );
    let posthog: BTreeSet<&str> = features["posthog"]
        .as_array()
        .expect("posthog feature")
        .iter()
        .map(|v| v.as_str().expect("feature string"))
        .collect();
    let mut unconditional = BTreeSet::new();
    for dep in pkg["dependencies"].as_array().expect("dependencies array") {
        if dep["kind"].is_null() && dep["optional"] == false {
            unconditional.insert(dep["name"].as_str().expect("dep name"));
        }
    }
    assert_eq!(
        unconditional,
        BTreeSet::from(["sha2", "thiserror"]),
        "every network/serialization dep must be optional behind `posthog`"
    );
    for dep in [
        "reqwest",
        "rustls",
        "tokio",
        "serde",
        "serde_json",
        "uuid",
        "dirs",
    ] {
        assert!(
            posthog.contains(&*format!("dep:{dep}")),
            "`posthog` must enable dep:{dep}"
        );
    }
}
