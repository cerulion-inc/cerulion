// SPDX-License-Identifier: AGPL-3.0-only
//! A node crate made by `cerulion node create` links into another crate as a
//! plain library.
//!
//! An Android app runs its graph inside its own native library, so it cannot
//! `dlopen` a node cdylib: it depends on each node crate with
//! `default-features = false` and constructs the macro's `<Pascal>NodeEntry`
//! itself. Three things must hold for that, and this file builds a node and an
//! app library to prove all three together:
//!
//! 1. the node crate has an `rlib` target, so a dependent crate can `use` it;
//! 2. the node struct is `pub`, so its `<Pascal>NodeEntry` can be named;
//! 3. with the `cdylib` feature off, the node's C exports are compiled out, so
//!    the app library holds no `cerulion_node_info` at all, exported or local
//!    (two linked nodes would otherwise clash on that symbol).
//!
//! The control arm builds the node as a workspace member with its default
//! features and finds `cerulion_node_info` in its cdylib's dynamic exports, the
//! table `dlopen` resolves against, which shows the symbol search finds the
//! export where it exists.
//!
//! # The fixtures
//!
//! The node is the committed crate `crates/test_fixtures/test_node_links_timer`
//! and the app is `crates/test_fixtures/test_node_links_host_app`. The node's
//! files are what `node create` writes for a `test_node_links_timer` node with
//! a 50 ms period and one `std_msgs/Int32` output named `count`, apart from
//! what this workspace's test fixtures add to a manifest: `publish = false`,
//! and the `[lints]` table every member inherits.
//! [`the_fixture_node_is_what_node_create_writes`] writes that node again with
//! the engine's `node create` and compares the file list and every file whole,
//! so any change to what `node create` writes for this node (a new dependency,
//! a `rust-version`, a `build.rs`) fails it until the fixture is written again.
//!
//! # The build
//!
//! Both crates build into this checkout's own `target/` directory, named with
//! `--target-dir`, against the workspace lockfile (`--locked`). Checkouts that
//! share a target directory share these cdylib file names, which carry no
//! hash, so the build never reads an ambient `CARGO_TARGET_DIR`: the child
//! cargo gets one pointing at an empty directory, which must still be empty
//! after every build. The test holds an exclusive lock on a file in the target
//! directory and runs `cargo clean -p` on both crates first, so every library
//! it inspects was built by this run.
//!
//! Each build is `cargo build --workspace` with one of the two crates
//! excluded. The other crates then resolve with the features of a plain
//! `cargo build --workspace`, which CI runs before this test, so only the two
//! fixtures compile, besides relinking any workspace binary CI's Linux test job
//! has deleted from `target/debug/deps` to reclaim disk. A `-p` build of the
//! app alone resolves its dependencies' features differently and compiles over
//! a hundred of them again, and a `--lib` build gives the workspace's cdylib
//! members a different profile and compiles those again. Excluding the node
//! leaves it built only as the app's dependency, without its default features;
//! excluding the app builds the node with them.
//!
//! Host only. Without a workspace build in `target/`, the first run builds the
//! workspace.

use std::path::{Path, PathBuf};
use std::process::Command;

use cerulion_cli_engine::node_cmd::{self, NodeCreateOptions, NodeLanguage};
use cerulion_cli_engine::workspace;

/// The node fixture: a `node create` node, as written.
const NODE: &str = "test_node_links_timer";
/// The app fixture that links [`NODE`] as a library.
const APP: &str = "test_node_links_host_app";

/// How to write the node fixture again after a template change.
const REGENERATE: &str = "the `right` value is what this checkout's `node create` \
     writes: put it in the fixture under `crates/test_fixtures/test_node_links_timer`, \
     and keep its Cargo.toml carrying `publish = false` once and ending with a blank \
     line, `[lints]` and `workspace = true`";

/// The per-package variables cargo or nextest can set in this test process,
/// beside every `CARGO_PKG_*` one. A build script that declares
/// `cargo:rerun-if-env-changed` on one of them (ring does, for `CARGO_PKG_NAME`
/// and `CARGO_MANIFEST_DIR`) reruns when the cargo that checks it runs with a
/// different value, and this process carries its own crate's, so a child cargo
/// that inherits them rebuilds that dependency and every crate above it. The
/// child gets none of them.
const PACKAGE_VARS: &[&str] = &[
    "CARGO_MANIFEST_DIR",
    "CARGO_MANIFEST_PATH",
    "CARGO_CRATE_NAME",
    "CARGO_PRIMARY_PACKAGE",
    "CARGO_TARGET_TMPDIR",
    "CARGO_RUSTC_CURRENT_DIR",
    "OUT_DIR",
];

/// The root of this checkout.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("the checkout root")
}

/// The shared-library file name the platform's linker writes for `name`.
fn cdylib_file(name: &str) -> String {
    format!(
        "{}{name}{}",
        std::env::consts::DLL_PREFIX,
        std::env::consts::DLL_SUFFIX
    )
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

/// The symbol names `nm` lists as DEFINED in `lib` with `args` (an undefined
/// `U` reference does not count; Mach-O names lose their leading underscore).
fn defined_symbols(lib: &Path, args: &[&str]) -> Vec<String> {
    let mut nm = Command::new("nm");
    nm.args(args).arg(lib);
    let listing = run_ok(nm, &format!("nm {} {}", args.join(" "), lib.display()));
    listing
        .lines()
        .filter_map(|line| {
            let fields: Vec<&str> = line.split_whitespace().collect();
            let (kind, name) = match fields.as_slice() {
                [_, kind, name] => (*kind, *name),
                [kind, name] => (*kind, *name),
                _ => return None,
            };
            if kind == "U" {
                return None;
            }
            let name = if cfg!(target_os = "macos") {
                name.strip_prefix('_').unwrap_or(name)
            } else {
                name
            };
            Some(name.to_string())
        })
        .collect()
}

/// True when `lib` EXPORTS `symbol` in the table `dlopen` resolves against:
/// the dynamic symbol table on ELF, the defined external symbols on Mach-O.
fn exports_symbol(lib: &Path, symbol: &str) -> bool {
    let args: &[&str] = if cfg!(target_os = "macos") {
        &["-g", "-U"]
    } else {
        &["-D", "--defined-only"]
    };
    defined_symbols(lib, args).iter().any(|name| name == symbol)
}

/// True when `lib` defines `symbol` anywhere in its full symbol table, local
/// or exported: a "compiled out" claim must not pass on a hidden copy.
fn contains_symbol(lib: &Path, symbol: &str) -> bool {
    defined_symbols(lib, &[]).iter().any(|name| name == symbol)
}

/// Every file under `dir`, as a sorted list of `/`-separated relative paths.
fn files_under(dir: &Path) -> Vec<String> {
    fn walk(base: &Path, dir: &Path, out: &mut Vec<String>) {
        let entries =
            std::fs::read_dir(dir).unwrap_or_else(|e| panic!("read {}: {e}", dir.display()));
        for entry in entries {
            let path = entry.expect("a directory entry").path();
            if path.is_dir() {
                walk(base, &path, out);
            } else {
                let rel = path.strip_prefix(base).expect("under the base");
                let parts: Vec<String> = rel
                    .components()
                    .map(|c| c.as_os_str().to_string_lossy().into_owned())
                    .collect();
                out.push(parts.join("/"));
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, dir, &mut out);
    out.sort();
    out
}

/// The fixture manifest without what this workspace's test fixtures add:
/// `publish = false`, and the inherited `[lints]` table at the end. Each must
/// be there exactly once.
fn without_member_lines(manifest: &str) -> String {
    let lints = "\n[lints]\nworkspace = true\n";
    let body = manifest
        .strip_suffix(lints)
        .unwrap_or_else(|| panic!("the fixture manifest must end with `{lints:?}`:\n{manifest}"));
    let publish = "publish = false\n";
    assert_eq!(
        body.matches(publish).count(),
        1,
        "the fixture manifest must carry `publish = false` once:\n{manifest}"
    );
    body.replacen(publish, "", 1)
}

/// The fixture node is what `node create` writes for it today: the same file
/// list and every file byte for byte, `Cargo.toml` once the fixture lines are
/// taken out. A mismatch names the file and how to write the fixture
/// again ([`REGENERATE`]).
#[test]
fn the_fixture_node_is_what_node_create_writes() {
    let tmp = tempfile::tempdir().expect("temp dir");
    let ws = workspace::workspace_create(tmp.path(), "ws").expect("workspace_create");
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &ws.root.join("Cargo.toml"),
        NODE,
        Some(cerulion_core::MacroPolicy::Period { period_ms: 50 }),
        &NodeCreateOptions {
            outputs: vec![("std_msgs/Int32".to_string(), "count".to_string())],
            inputs: Vec::new(),
            trigger: None,
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("node_create");
    let generated = ws.nodes_dir.join(NODE);
    let fixture = repo_root().join("crates/test_fixtures").join(NODE);

    let files = files_under(&generated);
    assert_eq!(
        files_under(&fixture),
        files,
        "the fixture's files are not the files `node create` writes; {REGENERATE}"
    );
    for file in &files {
        let read = |dir: &Path| {
            let path = dir.join(file);
            std::fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
        };
        let committed = read(&fixture);
        let committed = if file == "Cargo.toml" {
            without_member_lines(&committed)
        } else {
            committed
        };
        assert_eq!(
            committed,
            read(&generated),
            "the fixture's {file} is not what `node create` writes; {REGENERATE}"
        );
    }
}

#[test]
fn a_generated_node_links_into_an_app_library_without_its_c_exports() {
    let root = repo_root();
    let target = root.join("target");
    std::fs::create_dir_all(&target).expect("create the target directory");
    let lock = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(target.join("node_links_as_library_test.lock"))
        .expect("open the fixture build lock");
    lock.lock().expect("take the fixture build lock");
    let decoy = tempfile::tempdir().expect("decoy target dir");
    let cargo = |args: &[&str]| {
        let mut cmd = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_string()));
        for (name, _) in std::env::vars_os() {
            if name
                .to_str()
                .is_some_and(|name| PACKAGE_VARS.contains(&name) || name.starts_with("CARGO_PKG_"))
            {
                cmd.env_remove(name);
            }
        }
        cmd.current_dir(&root)
            .env("CARGO_TARGET_DIR", decoy.path())
            .env_remove("CARGO_BUILD_TARGET_DIR")
            .args(args)
            .arg("--target-dir")
            .arg(&target);
        cmd
    };
    let assert_no_leak = |after: &str| {
        let leaked = files_under(decoy.path());
        assert!(
            leaked.is_empty(),
            "{after} wrote into the ambient CARGO_TARGET_DIR instead of its \
             --target-dir: {leaked:?}"
        );
    };
    let debug = target.join("debug");
    let host_lib = debug.join(cdylib_file(APP));
    let host_exe = debug.join(format!("{APP}{}", std::env::consts::EXE_SUFFIX));
    let node_lib = debug.join(cdylib_file(NODE));

    run_ok(
        cargo(&["clean", "--locked", "-p", APP, "-p", NODE]),
        "cargo clean the fixtures",
    );
    for path in [&host_lib, &host_exe, &node_lib] {
        assert!(
            !path.exists(),
            "{} survived `cargo clean -p`",
            path.display()
        );
    }

    // The app side: the node as a library, its exports compiled out.
    run_ok(
        cargo(&["build", "--locked", "--workspace", "--exclude", NODE]),
        "cargo build the app fixture",
    );
    assert_no_leak("cargo build the app fixture");
    let stdout = run_ok(Command::new(&host_exe), "run the app fixture");
    assert_eq!(
        stdout.trim(),
        "policy=Some(Period { period_ms: 50 }) outputs=[\"count\"]",
        "the app must construct the node from its macro entry and read the \
         node's own metadata"
    );
    assert!(
        exports_symbol(&host_lib, "host_app_linked_node_count"),
        "the symbol search must see the app library's own export in {}",
        host_lib.display()
    );
    assert!(
        !contains_symbol(&host_lib, "cerulion_node_info"),
        "{} contains `cerulion_node_info`: `default-features = false` did not \
         compile the node's C exports out",
        host_lib.display()
    );

    // The control: the node built as a member, with its default features,
    // keeps the export `dlopen` resolves.
    run_ok(
        cargo(&["build", "--locked", "--workspace", "--exclude", APP]),
        "cargo build the node fixture",
    );
    assert_no_leak("cargo build the node fixture");
    assert!(
        exports_symbol(&node_lib, "cerulion_node_info"),
        "{} must export `cerulion_node_info` for `cerulion graph run`",
        node_lib.display()
    );
}
