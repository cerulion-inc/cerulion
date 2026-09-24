// SPDX-License-Identifier: AGPL-3.0-only
//! `cargo add cerulion` is the whole dependency list.
//!
//! This crate's own doctests cannot prove that. They run with
//! `CARGO_MANIFEST_DIR` pointing at this package, whose manifest names
//! `cerulion_core` directly (it has to, in order to re-export it), so the
//! macros resolve their root through the runtime crate exactly as they do
//! everywhere else in this workspace. The claim under test is about a package
//! that names ONLY the umbrella, and no package inside this workspace is in
//! that position.
//!
//! So the test builds one. It writes a bin crate into a temporary directory
//! outside the workspace, with a single path dependency on this crate and an
//! empty `[workspace]` table so it is not adopted by the repository's
//! workspace, puts a node in `main.rs`, and asks cargo to check it.
//!
//! The oracle is the compiler. Before the macros resolved their own root
//! every generated path began `::cerulion_core`, which such a package cannot
//! name, and this project failed with `E0433: failed to resolve: could not
//! find cerulion_core in the list of imported crates`. The negative control
//! below reproduces exactly that failure from the same scratch project, so a
//! green pass here cannot be a project that compiled for some unrelated
//! reason.
//!
//! Cost, stated plainly. The scratch project's dependency closure is this
//! crate's, which is the whole runtime, and it gets its OWN target directory
//! with its own fingerprint database, so nothing the workspace has already
//! built is reused. It is a `cargo check` rather than a build, measured at
//! about two minutes and 1.6 GB from cold on a desk. The separate directory
//! is not an oversight: two cargo invocations against one directory serialise
//! on its lock, and the outer `cargo test` may still be holding the
//! workspace's. `CERULION_SCRATCH_TARGET_DIR` moves it, which is what CI does
//! to keep 1.6 GB of artifacts for a package set nothing else builds out of a
//! cache every run of that job would pay to restore.
//!
//! This test is the only coverage of the umbrella spelling anywhere in the
//! repository, which is why its skip path refuses to be silent.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// The node the one-dependency project compiles.
///
/// It exercises the macro surface such a package reaches: the struct
/// attribute, the impl attribute, a port field write, the derive, and the
/// prelude glob. Every one of those expands to paths into the runtime.
///
/// `main` is the anti-tautology half. `SensorNodeEntry` exists only because
/// `#[cerulion_node]` generated it, and `STATE_SHAPE` resolves only because
/// the derive generated an impl whose own paths resolved, so this project
/// cannot check clean with the macros inert.
const NODE_SOURCE: &str = r#"
use cerulion::msgs::geometry_msgs::Vector3;
use cerulion::prelude::*;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SensorNode {
    #[output]
    reading: Vector3,
    tick_count: u32,
}

#[cerulion_node_impl]
impl SensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        self.reading.x = f64::from(self.tick_count);
        Ok(())
    }
}

#[derive(CerulionState, Default)]
struct Pose {
    x: f64,
    y: f64,
}

fn main() {
    let _entry = SensorNodeEntry::new();
    let _shape = <Pose as cerulion::core::state::CerulionState>::STATE_SHAPE;
}
"#;

/// The negative control's source: all three macros, reached through the macro
/// crate by path, so the project needs nothing else to compile.
///
/// A package in this position names neither the umbrella nor the runtime, so
/// the generated paths cannot resolve however they are spelled. What is under
/// test is that the compiler says which dependency is missing rather than
/// pointing at a path the user never wrote, and that each macro says it
/// independently. The port's type is a local struct: the macros never check
/// that a port type resolves, and the project is required to fail anyway.
const NO_RUNTIME_SOURCE: &str = r#"
#[derive(Default)]
struct Reading;

#[cerulion_macros::cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SensorNode {
    #[output]
    reading: Reading,
}

#[cerulion_macros::cerulion_node_impl]
impl SensorNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

#[derive(cerulion_macros::CerulionState, Default)]
struct Pose {
    x: f64,
}

fn main() {
    let _pose = Pose::default();
}
"#;

/// This package's directory, which is what the scratch project depends on by
/// path.
fn umbrella_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Where the scratch project puts its build artifacts.
///
/// The workspace target directory by default, so a tree that has already been
/// built has already checked everything but the scratch bin itself. It is a
/// sibling directory rather than the target directory itself, because two
/// cargo invocations against one directory serialise on its lock and the
/// outer `cargo test` may still be holding it.
fn scratch_target_dir(case: &str) -> PathBuf {
    if let Some(dir) = std::env::var_os("CERULION_SCRATCH_TARGET_DIR") {
        return PathBuf::from(dir).join(case);
    }
    // `crates/cerulion` -> `crates` -> the workspace root.
    let workspace_root = umbrella_dir()
        .parent()
        .and_then(Path::parent)
        .expect("this package sits two levels below the workspace root")
        .to_path_buf();
    workspace_root
        .join("target")
        .join("one-dependency-scratch")
        .join(case)
}

/// Write the scratch project and return its directory.
///
/// `dependencies` is the body of the manifest's `[dependencies]` table, so a
/// caller can build the project the umbrella is meant to serve or the one
/// that must still fail.
fn write_scratch_project(root: &Path, package: &str, dependencies: &str, source: &str) {
    let src = root.join("src");
    std::fs::create_dir_all(&src).expect("create the scratch source directory");

    let manifest = format!(
        // The empty `[workspace]` table is what keeps cargo from walking up
        // and adopting this project into the repository's workspace, which
        // would defeat the whole test: an adopted member inherits the
        // workspace's dependency table and stops being a package that names
        // only the umbrella.
        "[workspace]\n\
         \n\
         [package]\n\
         name = \"{package}\"\n\
         version = \"0.0.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         \n\
         [dependencies]\n\
         {dependencies}\n"
    );
    std::fs::write(root.join("Cargo.toml"), manifest).expect("write the scratch manifest");
    std::fs::write(src.join("main.rs"), source).expect("write the scratch node");
}

/// One `name = { path = '...' }` line, as a TOML LITERAL string.
///
/// A literal string takes no escapes at all, so the only characters that
/// could break the manifest are a single quote and a newline. Both are
/// asserted away rather than escaped, because a path carrying either means
/// the checkout itself is in a state this test has no business guessing at.
fn path_dependency(name: &str, path: &Path) -> String {
    let rendered = path.display().to_string();
    assert!(
        !rendered.contains('\'') && !rendered.contains('\n'),
        "the checkout path cannot be written as a TOML literal string: {rendered}"
    );
    format!("{name} = {{ path = '{rendered}' }}")
}

/// Run `cargo check` over the scratch project.
fn cargo_check(root: &Path, target_dir: &Path, offline: bool) -> Output {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let mut command = Command::new(cargo);
    command
        .arg("check")
        .arg("--quiet")
        .current_dir(root)
        .env("CARGO_TARGET_DIR", target_dir)
        // The outer job's flags have nothing to do with this project and
        // would only invalidate its fingerprints against the previous run.
        .env_remove("RUSTFLAGS");
    if offline {
        command.arg("--offline");
    }
    command
        .output()
        .expect("run cargo over the scratch project")
}

/// Whether a failed run failed because cargo could not reach a registry,
/// rather than because the code did not compile.
///
/// Matched on cargo's own wording for the two shapes a disconnected desk
/// produces: an `--offline` run that wanted a crate it does not have cached,
/// and an online run that could not reach the network. Deliberately narrow:
/// anything else is a real failure and must be reported as one.
fn is_registry_failure(output: &Output) -> bool {
    let text = String::from_utf8_lossy(&output.stderr);
    // A run that got as far as invoking rustc on the scratch source is not a
    // registry failure whatever else it says, and cargo appends its offline
    // reminder to a broad family of resolution errors including ones a bug in
    // this harness would cause. So a rustc diagnostic code anywhere in the
    // output disqualifies the whole classification.
    if text.contains("error[E") {
        return false;
    }
    text.contains("--offline")
        || text.contains("failed to download")
        || text.contains("failed to fetch")
        || text.contains("no matching package")
        || text.contains("registry index was not found")
}

/// `cargo check` the scratch project, offline first and online as a fallback,
/// reporting whether the registry was the obstacle.
fn check_scratch_project(root: &Path, target_dir: &Path) -> Result<Output, String> {
    let offline = cargo_check(root, target_dir, true);
    if offline.status.success() || !is_registry_failure(&offline) {
        return Ok(offline);
    }
    let online = cargo_check(root, target_dir, false);
    if online.status.success() || !is_registry_failure(&online) {
        return Ok(online);
    }
    Err(String::from_utf8_lossy(&online.stderr).into_owned())
}

/// The environment variable that turns a registry failure from a red into a
/// skip.
///
/// Deliberately opt-in, and deliberately NOT set in CI. libtest prints a
/// passing test's stderr only under `--nocapture`, so a skip that merely
/// printed would be a green dot standing in for the only coverage the
/// umbrella spelling has anywhere in this repository.
const ALLOW_SKIP: &str = "CERULION_ALLOW_OFFLINE_SKIP";

/// Turn a registry failure into a skip if the caller asked for one, and into
/// a failure otherwise.
fn allow_skip_or_fail(stderr: &str) {
    assert!(
        std::env::var_os(ALLOW_SKIP).is_some(),
        "cargo could not reach a registry, so the scratch project was never \
         checked and the umbrella spelling was never exercised. This test is \
         its only coverage, so an unreachable registry is a failure, not a \
         pass. Set {ALLOW_SKIP}=1 to accept that deliberately.\n{stderr}"
    );
    eprintln!("SKIPPED ({ALLOW_SKIP} is set): {stderr}");
}

#[test]
fn a_package_whose_only_dependency_is_the_umbrella_compiles_a_node() {
    let target_dir = scratch_target_dir("umbrella");
    let scratch = target_dir.join("consumer");
    let _ = std::fs::remove_dir_all(&scratch);
    write_scratch_project(
        &scratch,
        "one_dependency_consumer",
        &path_dependency("cerulion", &umbrella_dir()),
        NODE_SOURCE,
    );

    let output = match check_scratch_project(&scratch, &target_dir) {
        Ok(output) => output,
        Err(stderr) => return allow_skip_or_fail(&stderr),
    };

    assert!(
        output.status.success(),
        "a package whose only dependency is the umbrella crate must compile a \
         node.\n--- cargo stderr ---\n{}\n--- cargo stdout ---\n{}",
        String::from_utf8_lossy(&output.stderr),
        String::from_utf8_lossy(&output.stdout),
    );
}

#[test]
fn a_package_that_names_neither_crate_is_told_which_dependency_is_missing() {
    // The negative control, and the anti-tautology half of the test above.
    // Without it, a pass up there could be a project that compiled because
    // the macros expanded to nothing at all.
    let target_dir = scratch_target_dir("no-runtime");
    let scratch = target_dir.join("consumer");
    let _ = std::fs::remove_dir_all(&scratch);
    let macros = umbrella_dir()
        .parent()
        .expect("this package sits beside the macro crate")
        .join("cerulion_macros");
    write_scratch_project(
        &scratch,
        "no_runtime_consumer",
        &path_dependency("cerulion_macros", &macros),
        NO_RUNTIME_SOURCE,
    );

    let output = match check_scratch_project(&scratch, &target_dir) {
        Ok(output) => output,
        Err(stderr) => return allow_skip_or_fail(&stderr),
    };

    assert!(
        !output.status.success(),
        "a package that names neither the umbrella nor the runtime cannot \
         compile a node, so this project must not check clean"
    );
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("must name the runtime"),
        "the failure must name the missing dependency rather than leaving the \
         user with a resolution error against a path they never wrote.\n\
         --- cargo stderr ---\n{stderr}"
    );
    // All three macros report independently, because each is a separate
    // expansion with no way to see what another one emitted. Asserting more
    // than one keeps a fix that wires the diagnostic into a single entry
    // point from passing.
    let reports = stderr.matches("must name the runtime").count();
    assert!(
        reports >= 2,
        "each exported macro reports the missing dependency on its own, so a \
         source using three of them must produce more than one report; got \
         {reports}.\n--- cargo stderr ---\n{stderr}"
    );
}
