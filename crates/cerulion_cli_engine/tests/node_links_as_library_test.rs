// SPDX-License-Identifier: AGPL-3.0-only
//! A node crate made by `cerulion node new` links into another crate as a
//! plain library.
//!
//! An Android app runs its graph inside its own native library, so it cannot
//! `dlopen` a node cdylib: it depends on each node crate with
//! `default-features = false` and constructs the macro's `<Pascal>NodeEntry`
//! itself. Three things must hold for that, and this file builds a real
//! workspace to prove all three together:
//!
//! 1. the node crate has an `rlib` target, so a dependent crate can `use` it;
//! 2. the node struct is `pub`, so its `<Pascal>NodeEntry` can be named;
//! 3. with the `cdylib` feature off, the node's C exports are compiled out, so
//!    the app library does not export `cerulion_node_info` (two linked nodes
//!    would otherwise clash on that symbol).
//!
//! The control arm builds the same node on its own with default features and
//! finds `cerulion_node_info` in its cdylib, so an empty symbol search cannot
//! pass for the wrong reason, and the CLI's `dlopen` path is shown intact.
//!
//! Host only. It compiles `cerulion_core` into a temporary workspace, so the
//! first run is slow.

use std::path::{Path, PathBuf};
use std::process::Command;

use cerulion_cli_engine::node_cmd::{self, NodeCreateOptions, NodeLanguage};
use cerulion_cli_engine::workspace;

/// The shared-library file name the platform's linker writes for `name`.
fn cdylib_file(name: &str) -> String {
    format!(
        "{}{name}{}",
        std::env::consts::DLL_PREFIX,
        std::env::consts::DLL_SUFFIX
    )
}

fn cargo() -> Command {
    let mut cmd = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string()));
    // The test's own target dir would serialise against the outer `cargo test`
    // lock; the workspace builds into its own `target/`.
    cmd.env_remove("CARGO_TARGET_DIR");
    cmd
}

fn run_ok(mut cmd: Command, what: &str) -> String {
    let out = cmd
        .output()
        .unwrap_or_else(|e| panic!("{what}: spawn: {e}"));
    assert!(
        out.status.success(),
        "{what} failed ({}):\nstdout:\n{}\nstderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// True when `lib` DEFINES the global symbol `symbol` (Mach-O names carry a
/// leading underscore; an undefined `U` reference does not count).
fn defines_symbol(lib: &Path, symbol: &str) -> bool {
    let mut nm = Command::new("nm");
    nm.arg("-g").arg(lib);
    let listing = run_ok(nm, &format!("nm -g {}", lib.display()));
    listing.lines().any(|line| {
        let fields: Vec<&str> = line.split_whitespace().collect();
        let (kind, name) = match fields.as_slice() {
            [_, kind, name] => (*kind, *name),
            [kind, name] => (*kind, *name),
            _ => return false,
        };
        kind != "U" && (name == symbol || name.strip_prefix('_') == Some(symbol))
    })
}

/// A generated workspace with one `timer` node and a `host_app` member that
/// depends on it the way an app library does.
fn scaffold(tmp: &Path) -> PathBuf {
    let ws = workspace::workspace_create(tmp, "ws").expect("workspace_create");
    let options = NodeCreateOptions {
        outputs: vec![("std_msgs/Int32".to_string(), "count".to_string())],
        inputs: Vec::new(),
        trigger: None,
        raw_ffi: false,
        language: NodeLanguage::Rust,
    };
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &ws.root.join("Cargo.toml"),
        "timer",
        Some(cerulion_core::MacroPolicy::Period { period_ms: 50 }),
        &options,
    )
    .expect("node_create timer");

    // Pin the dependency graph to the versions this checkout builds with.
    let repo_lock = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../Cargo.lock");
    std::fs::copy(&repo_lock, ws.root.join("Cargo.lock")).expect("copy the checkout lockfile");

    let manifest_path = ws.root.join("Cargo.toml");
    let manifest = std::fs::read_to_string(&manifest_path).expect("read workspace manifest");
    let manifest = manifest.replacen(
        "members = [\"nodes/*\"]",
        "members = [\"nodes/*\", \"host_app\"]",
        1,
    );
    assert!(
        manifest.contains("\"host_app\""),
        "the generated workspace manifest no longer lists `members = [\"nodes/*\"]`"
    );
    std::fs::write(&manifest_path, manifest).expect("write workspace manifest");

    let host = ws.root.join("host_app");
    std::fs::create_dir_all(host.join("src")).expect("host_app/src");
    std::fs::write(
        host.join("Cargo.toml"),
        r#"[package]
name = "host_app"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
cerulion_core = { workspace = true }
timer = { path = "../nodes/timer", default-features = false }
"#,
    )
    .expect("host_app manifest");
    std::fs::write(
        host.join("src/lib.rs"),
        r#"use cerulion_core::graph::node::NodeEntry;

/// Builds the linked node through its macro entry, as an app runtime does.
pub fn linked_node() -> Box<dyn NodeEntry> {
    Box::new(timer::TimerNodeEntry::new())
}

/// Keeps `linked_node` reachable from the cdylib's export table.
#[no_mangle]
pub extern "C" fn host_app_linked_node_count() -> usize {
    let entry = linked_node();
    entry.info().map(|info| info.output_names().len()).unwrap_or(0)
}
"#,
    )
    .expect("host_app lib.rs");
    std::fs::write(
        host.join("src/main.rs"),
        r#"fn main() {
    let info = host_app::linked_node().info().expect("node info");
    println!("policy={:?} outputs={:?}", info.policy(), info.output_names());
}
"#,
    )
    .expect("host_app main.rs");
    ws.root
}

#[test]
fn a_generated_node_links_into_an_app_library_without_its_c_exports() {
    let tmp = tempfile::tempdir().expect("temp dir");
    let root = scaffold(tmp.path());
    let debug = root.join("target/debug");

    // The app side: the node as a library, its exports compiled out.
    let mut build = cargo();
    build.current_dir(&root).args(["build", "-p", "host_app"]);
    run_ok(build, "cargo build -p host_app");

    let mut run = cargo();
    run.current_dir(&root)
        .args(["run", "-q", "-p", "host_app", "--bin", "host_app"]);
    let stdout = run_ok(run, "cargo run -p host_app");
    assert_eq!(
        stdout.trim(),
        "policy=Some(Period { period_ms: 50 }) outputs=[\"count\"]",
        "the app must construct the node from its macro entry and read the \
         node's own metadata"
    );

    let host_lib = debug.join(cdylib_file("host_app"));
    assert!(
        defines_symbol(&host_lib, "host_app_linked_node_count"),
        "the symbol search must see the app library's own export in {}",
        host_lib.display()
    );
    assert!(
        !defines_symbol(&host_lib, "cerulion_node_info"),
        "{} exports `cerulion_node_info`: `default-features = false` did not \
         compile the node's C exports out",
        host_lib.display()
    );

    // The control: the same node built on its own keeps its loadable cdylib.
    let mut node_build = cargo();
    node_build.current_dir(&root).args(["build", "-p", "timer"]);
    run_ok(node_build, "cargo build -p timer");
    let node_lib = debug.join(cdylib_file("timer"));
    assert!(
        defines_symbol(&node_lib, "cerulion_node_info"),
        "{} must export `cerulion_node_info` for `cerulion graph run`",
        node_lib.display()
    );
}
