// SPDX-License-Identifier: AGPL-3.0-only
//! No robot, embedded or runtime crate may name `cerulion_telemetry`, in any dependency
//! table, optional or not, under any alias.
//!
//! Reads `cargo metadata --no-deps`, whose per-package `dependencies` is the
//! DECLARED list (an `optional = true` dep behind an off feature is still
//! present, and `name` is the package name with any alias in `rename`), so a
//! feature-gated or renamed edge cannot slip past. Direct edges only, the
//! crate has no workspace dependents yet, and a transitive route would need
//! one of the listed crates to depend on a desk crate, which is its own
//! architecture violation.

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::process::Command;

const TELEMETRY: &str = "cerulion_telemetry";

/// Crates that run on the robot or in the runtime hot path.
const HOT_PATH: &[&str] = &[
    "cerulion_core",
    "cerulion_remoted",
    "rmw_cerulion",
    "cerulion_netd",
    "cerulion_bagd",
    "cerulion_bag",
    "cerulion_dds",
    "cerulion_discovery",
    "cerulion_mdns",
    "cerulion_macros",
    "cerulion_link",
    "cerulion_heaphook",
    "cerulion-wire",
    "cerud",
    "native_ros2_messages",
];

fn crate_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
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

#[test]
fn hot_path_crates_never_name_cerulion_telemetry() {
    let meta = metadata();
    let packages = meta["packages"].as_array().expect("packages array");
    let mut seen = BTreeSet::new();
    let mut offenders = Vec::new();
    for pkg in packages {
        let name = pkg["name"].as_str().expect("package name");
        if !HOT_PATH.contains(&name) {
            continue;
        }
        seen.insert(name);
        for dep in pkg["dependencies"].as_array().expect("dependencies array") {
            let dep_name = dep["name"].as_str().expect("dependency name");
            if dep_name == TELEMETRY {
                offenders.push(format!(
                    "{name} -> {TELEMETRY} (kind={}, optional={}, rename={})",
                    dep["kind"].as_str().unwrap_or("normal"),
                    dep["optional"],
                    dep["rename"]
                ));
            }
        }
    }
    let missing: Vec<_> = HOT_PATH.iter().filter(|c| !seen.contains(*c)).collect();
    assert!(
        missing.is_empty(),
        "hot-path crates not found in workspace metadata (renamed? update HOT_PATH): {missing:?}"
    );
    assert!(
        offenders.is_empty(),
        "robot and runtime crates must never depend on {TELEMETRY}:\n{}",
        offenders.join("\n")
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
    for dep in ["serde", "serde_json", "uuid", "dirs"] {
        assert!(
            posthog.contains(&*format!("dep:{dep}")),
            "`posthog` must enable dep:{dep}"
        );
    }
}
