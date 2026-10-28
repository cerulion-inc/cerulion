// SPDX-License-Identifier: AGPL-3.0-only
//! A workspace scaffolded from a source checkout offers `cerulion_tf`, and a
//! node in it that opts in with `cerulion_tf = { workspace = true }` compiles
//! with a `TfBuffer` field: the setup `docs/transforms.md` tells a node
//! author to use.
//!
//! The test binary runs from this checkout, so `workspace_init` takes the
//! checkout branch and writes absolute `path` dependencies. The node depends
//! on `cerulion_tf` alone, so the check compiles that crate and its one
//! dependency, not the runtime. `--offline` keeps the check off the network;
//! the registry cache already holds `thiserror` from building this crate.

use std::path::Path;
use std::process::Command;

use cerulion_cli_engine::workspace::{workspace_init, DependencySource};

const NODE_MANIFEST: &str = r#"[package]
name = "tf_listener"
version = "0.1.0"
edition = "2021"

[dependencies]
cerulion_tf = { workspace = true }
"#;

const NODE_SOURCE: &str = r#"use cerulion_tf::{TfBuffer, TfError, Transform};

pub struct TfListener {
    pub buffer: TfBuffer,
}

impl TfListener {
    pub fn map_lidar(&self, stamp_ns: u64) -> Result<Transform, TfError> {
        self.buffer.lookup("map", "lidar", stamp_ns)
    }
}
"#;

fn write(path: &Path, contents: &str) {
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path, contents).unwrap();
}

#[test]
fn a_node_in_a_checkout_workspace_compiles_against_cerulion_tf() {
    let dir = tempfile::tempdir().unwrap();
    let ws = workspace_init(dir.path()).unwrap();
    let Some(DependencySource::Checkout { base }) = &ws.dependency_source else {
        panic!(
            "this test runs from a checkout, so the workspace must use it; got {:?}",
            ws.dependency_source
        );
    };
    let manifest = std::fs::read_to_string(ws.root.join("Cargo.toml")).unwrap();
    let tf_path = base
        .join("crates/cerulion_tf")
        .display()
        .to_string()
        .replace('\\', "/");
    assert!(
        manifest.contains(&format!("cerulion_tf = {{ path = \"{tf_path}\" }}")),
        "the workspace manifest must offer cerulion_tf from the checkout:\n{manifest}"
    );

    let node = ws.root.join("nodes/tf_listener");
    write(&node.join("Cargo.toml"), NODE_MANIFEST);
    write(&node.join("src/lib.rs"), NODE_SOURCE);

    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let output = Command::new(cargo)
        .args(["check", "--offline", "--quiet", "-p", "tf_listener"])
        .current_dir(&ws.root)
        .env("CARGO_TARGET_DIR", dir.path().join("target"))
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "cargo check of a node importing TfBuffer failed:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}
