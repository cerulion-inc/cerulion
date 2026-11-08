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
//! So the tests build such packages. Each one writes a scratch cargo project
//! under a target directory, with path dependencies on crates in this
//! repository and a `[workspace]` table of its own so the repository's
//! workspace does not adopt it, and runs `cargo check` over it:
//!
//! - The positive case: three node crates whose only dependency is this
//!   crate, each with the `[lib]` and `[features]` tables
//!   `cerulion node create` writes, so `default = ["cdylib"]` compiles the
//!   `#[cfg(feature = "cdylib")]` FFI module the macros emit. Between them the
//!   nodes take a data trigger read with `consume = latest`, a sync window
//!   and an external source, and declare params, a sampled input, an
//!   accumulate input, outputs with and without a promise, `init` and
//!   `shutdown`, event handlers, a port-writing helper and every state escape.
//! - The negative control: a package that depends on the macro crate alone,
//!   which must fail with the named missing-dependency error from each macro.
//! - The diagnostic span cases: one package per root, umbrella and runtime,
//!   whose every Cerulion type error sits on a field, each of which must be
//!   reported at that field rather than at the macro attribute.
//!
//! The oracle is the compiler. Before the macros resolved their own root
//! every generated path began `::cerulion_core`, which a package naming only
//! this crate cannot resolve; rustc 1.95 reports that as "cannot find
//! `cerulion_core` in the crate root". The positive case names items only the
//! macros generate (each node's `<Name>Entry` and a derived `STATE_SHAPE`),
//! so it cannot check clean with the macros inert.
//!
//! Cost, stated plainly. The scratch projects' dependency closure is this
//! crate's, which is the whole runtime, and it gets its OWN target directory
//! with its own fingerprint database, so nothing the workspace has already
//! built is reused. It is a `cargo check` rather than a build, measured at
//! about two minutes and 1.6 GB from cold on a desk. The positive case and
//! both span cases check the same closure and share one target directory,
//! so they wait on its lock in turn and the closure is checked once. A
//! directory apart from the workspace's is not an oversight: two cargo
//! invocations against one directory serialise on its lock, and the outer
//! `cargo test` may still be holding the workspace's.
//! `CERULION_SCRATCH_TARGET_DIR` moves it, which is what CI does to keep
//! 1.6 GB of artifacts for a package set nothing else builds out of a cache
//! every run of that job would pay to restore.
//!
//! The positive case is the only coverage of the umbrella spelling anywhere
//! in the repository, which is why its skip path refuses to be silent.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// The `[lib]` and `[features]` tables of a node crate's manifest, as
/// `generate_cargo_toml` in `crates/cerulion_cli_engine/src/templates.rs`
/// writes them for `cerulion node create`.
///
/// `the_scratch_node_crates_use_the_tables_the_scaffold_writes` fails when
/// the two drift apart.
const NODE_CRATE_TABLES: &str = "[lib]
crate-type = [\"cdylib\", \"rlib\"]

[features]
cdylib = []
default = [\"cdylib\"]
";

/// A data-triggered node (its trigger read with `consume = latest`) with
/// params, three input kinds, two outputs, the lifecycle methods, event
/// handlers, a port-writing helper and every state escape, plus a derived enum
/// it holds as state.
///
/// `generated_items_exist` names `SensorNodeEntry`, which exists only because
/// `#[cerulion_node]` generated it, and `Mode`'s `STATE_SHAPE`, which
/// resolves only because the derive generated an impl whose own paths
/// resolved.
const SENSOR_SOURCE: &str = r#"
use std::collections::HashMap;

use cerulion::msgs::geometry_msgs::{Quaternion, Vector3};
use cerulion::prelude::*;

#[derive(CerulionState, Default)]
pub enum Mode {
    #[default]
    Idle,
    Tracking(u32),
    Lost { since: u64 },
}

#[cerulion_node]
#[derive(Default)]
pub struct SensorNode {
    #[input(trigger, consume = latest, expect_within_ms = 20)]
    imu: Vector3,
    #[input(backpressure = sample(3))]
    odom: Vector3,
    #[input(accumulate, cap = 8)]
    tf: Vector3,
    #[output(promise_within_ms = 25)]
    reading: Vector3,
    #[output]
    attitude: Quaternion,
    #[param(default = 1.0, min = 0.0, max = 10.0, description = "multiplier on imu.x")]
    gain: f64,
    #[param(default = 0, min = -100, max = 100)]
    offset: i32,
    #[param(default = "base_link")]
    frame_id: String,
    #[param(default = true)]
    enabled: bool,
    tick_count: u32,
    overflows: u32,
    mode: Mode,
    #[cerulion(serde)]
    labels: Vec<String>,
    #[cerulion(unordered)]
    seen: HashMap<u32, f64>,
    #[cerulion(reconstruct)]
    scratch: Vec<u8>,
}

#[cerulion_node_impl]
impl SensorNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.labels.push(self.frame_id.clone());
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.tick_count += 1;
        let mut weighted = 0.0;
        for frame in self.tf.iter() {
            weighted += frame.x;
        }
        self.seen.insert(self.tick_count, weighted);
        if self.enabled {
            self.reading.x = self.imu.x * self.gain + f64::from(self.offset);
            self.reading.y = self.odom.y + weighted;
        }
        self.write_attitude()?;
        Ok(())
    }

    fn write_attitude(&mut self) -> Result<(), NodeError> {
        self.attitude.w = f64::from(self.tick_count);
        Ok(())
    }

    #[on_event(input = "tf")]
    fn on_tf_overflow(&mut self, _event: BackpressureEvent) {
        self.overflows += 1;
    }

    #[on_event(input = "imu")]
    fn on_imu_late(&mut self, _event: ExpectWithinEvent) {
        self.mode = Mode::Lost { since: u64::from(self.tick_count) };
    }

    #[on_event(output = "reading")]
    fn on_reading_late(&mut self, _event: PromiseWithinEvent) {
        self.mode = Mode::Tracking(self.tick_count);
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        self.scratch.clear();
        Ok(())
    }
}

#[allow(dead_code)]
fn generated_items_exist() {
    let _entry = SensorNodeEntry::new();
    let _shape = <Mode as cerulion::core::state::CerulionState>::STATE_SHAPE;
}
"#;

/// A sync node: two trigger inputs aligned in a window, and a plain input.
const FUSION_SOURCE: &str = r#"
use cerulion::msgs::geometry_msgs::Vector3;
use cerulion::prelude::*;

#[cerulion_node(sync_window_ms = 25)]
#[derive(Default)]
pub struct FusionNode {
    #[input(trigger, depth = 1)]
    cam: Vector3,
    #[input(trigger, depth = 1)]
    imu: Vector3,
    #[input]
    bias: Vector3,
    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl FusionNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.cam.x + self.imu.x - self.bias.x;
        Ok(())
    }
}

#[allow(dead_code)]
fn generated_items_exist() {
    let _entry = FusionNodeEntry::new();
}
"#;

/// An external node, whose cdylib exports carry the external-source symbol.
const DRIVER_SOURCE: &str = r#"
use cerulion::msgs::geometry_msgs::Vector3;
use cerulion::prelude::*;

#[cerulion_node(external)]
#[derive(Default)]
pub struct DriverNode {
    #[input]
    setpoint: Vector3,
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl DriverNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.setpoint.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[allow(dead_code)]
fn generated_items_exist() {
    let _entry = DriverNodeEntry::new();
}
"#;

/// The positive case's node crates: directory, package name, `src/lib.rs`.
const NODE_CRATES: [(&str, &str, &str); 3] = [
    ("sensor", "sensor_node", SENSOR_SOURCE),
    ("fusion", "fusion_node", FUSION_SOURCE),
    ("driver", "driver_node", DRIVER_SOURCE),
];

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

/// The span cases' source after its `use` lines: one marked line per field
/// whose type fails a Cerulion state bound, and nothing else wrong in the
/// source. The serde line fails two bounds, and both are reported there.
///
/// A trailing `// E0277 at <text>` names the token the error must point at,
/// the first `<text>` on that line. Between them the marks cover every
/// `where`-clause predicate the derive emits on a field that names the crate
/// root (a captured field, both serde bounds, an unordered container's
/// element type, an enum variant's field) and the captured-field predicate
/// reached through `#[cerulion_node]`.
const FIELD_ERRORS_BODY: &str = r#"
pub struct CudaContext;

impl Default for CudaContext {
    fn default() -> Self {
        Self
    }
}

pub struct NotSerde;

#[derive(CerulionState)]
pub struct Slam {
    pub pose: f64,
    pub cuda: CudaContext, // E0277 at CudaContext
}

#[derive(CerulionState)]
pub struct Blob {
    #[cerulion(serde)]
    pub raw: NotSerde, // E0277 at NotSerde
}

#[derive(CerulionState)]
pub struct Cells {
    #[cerulion(unordered)]
    pub cells: std::collections::HashMap<u32, CudaContext>, // E0277 at CudaContext
}

#[derive(CerulionState)]
pub enum Link {
    Idle,
    Connected(CudaContext), // E0277 at CudaContext
}

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
pub struct SlamNode {
    #[output]
    cmd: Vector3,
    cuda: CudaContext, // E0277 at CudaContext
}

#[cerulion_node_impl]
impl SlamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = 1.0;
        Ok(())
    }
}
"#;

/// How many marks `FIELD_ERRORS_BODY` carries, so a marker parse that found
/// none cannot pass.
const FIELD_ERROR_MARKS: usize = 5;

/// This package's directory, which the scratch projects depend on by path.
fn umbrella_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// A sibling crate of this one in the repository.
fn sibling_crate(name: &str) -> PathBuf {
    umbrella_dir()
        .parent()
        .expect("this package sits beside the other crates")
        .join(name)
}

/// Where a scratch project puts its build artifacts.
///
/// `<CERULION_SCRATCH_TARGET_DIR>/<shared>` when that is set, else
/// `target/one-dependency-scratch/<shared>` under the workspace root. Cases
/// that check the same dependency closure pass the same `shared`.
fn scratch_target_dir(shared: &str) -> PathBuf {
    if let Some(dir) = std::env::var_os("CERULION_SCRATCH_TARGET_DIR") {
        return PathBuf::from(dir).join(shared);
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
        .join(shared)
}

/// The target directory of every case that checks this crate's closure.
const CLOSURE_TARGET: &str = "umbrella";

/// Write one package: its manifest and one source file under `src/`.
///
/// `tables` is placed between the `[package]` and `[dependencies]` tables.
/// `dependencies` is the body of `[dependencies]`.
fn write_package(
    dir: &Path,
    package: &str,
    tables: &str,
    dependencies: &str,
    file: &str,
    source: &str,
) {
    let src = dir.join("src");
    std::fs::create_dir_all(&src).expect("create the scratch source directory");
    let manifest = format!(
        "[package]\n\
         name = \"{package}\"\n\
         version = \"0.1.0\"\n\
         edition = \"2021\"\n\
         publish = false\n\
         \n\
         {tables}\n\
         [dependencies]\n\
         {dependencies}\n"
    );
    std::fs::write(dir.join("Cargo.toml"), manifest).expect("write the scratch manifest");
    std::fs::write(src.join(file), source).expect("write the scratch source");
}

/// Write a scratch project that is one package and its own workspace root.
///
/// The `[workspace]` table is what keeps cargo from walking up and adopting
/// the project into the repository's workspace, which would defeat the whole
/// test: an adopted member inherits the workspace's dependency table and
/// stops being a package that names only the umbrella.
fn write_single_package(
    root: &Path,
    package: &str,
    tables: &str,
    dependencies: &str,
    file: &str,
    source: &str,
) {
    write_package(root, package, tables, dependencies, file, source);
    let manifest = root.join("Cargo.toml");
    let text = std::fs::read_to_string(&manifest).expect("read the scratch manifest back");
    std::fs::write(&manifest, format!("[workspace]\n\n{text}"))
        .expect("write the scratch workspace table");
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

/// Run `cargo check --workspace` over a scratch project, with `extra`
/// arguments after it.
///
/// `--color never` because CI sets `CARGO_TERM_COLOR=always`, and the
/// assertions read rustc's text.
fn cargo_check(root: &Path, target_dir: &Path, offline: bool, extra: &[&str]) -> Output {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let mut command = Command::new(cargo);
    command
        .arg("check")
        .arg("--workspace")
        .arg("--quiet")
        .args(["--color", "never"])
        .args(extra)
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
    // output disqualifies the whole classification, in either message format.
    if text.contains("error[E") {
        return false;
    }
    text.contains("--offline")
        || text.contains("failed to download")
        || text.contains("failed to fetch")
        || text.contains("no matching package")
        || text.contains("registry index was not found")
}

/// `cargo check` a scratch project, offline first and online as a fallback,
/// reporting whether the registry was the obstacle.
fn check_scratch_project(root: &Path, target_dir: &Path, extra: &[&str]) -> Result<Output, String> {
    let offline = cargo_check(root, target_dir, true, extra);
    if offline.status.success() || !is_registry_failure(&offline) {
        return Ok(offline);
    }
    let online = cargo_check(root, target_dir, false, extra);
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
    let target_dir = scratch_target_dir(CLOSURE_TARGET);
    let scratch = target_dir.join("consumer");
    let _ = std::fs::remove_dir_all(&scratch);
    std::fs::create_dir_all(&scratch).expect("create the scratch workspace");

    let members: Vec<String> = NODE_CRATES
        .iter()
        .map(|(dir, _, _)| format!("\"{dir}\""))
        .collect();
    std::fs::write(
        scratch.join("Cargo.toml"),
        format!(
            "[workspace]\nmembers = [{}]\nresolver = \"2\"\n",
            members.join(", ")
        ),
    )
    .expect("write the scratch workspace manifest");
    let umbrella = path_dependency("cerulion", &umbrella_dir());
    for (dir, package, source) in NODE_CRATES {
        write_package(
            &scratch.join(dir),
            package,
            NODE_CRATE_TABLES,
            &umbrella,
            "lib.rs",
            source,
        );
    }

    let output = match check_scratch_project(&scratch, &target_dir, &[]) {
        Ok(output) => output,
        Err(stderr) => return allow_skip_or_fail(&stderr),
    };

    assert!(
        output.status.success(),
        "packages whose only dependency is the umbrella crate must compile \
         their nodes, with the cdylib feature on.\n--- cargo stderr ---\n{}\n\
         --- cargo stdout ---\n{}",
        String::from_utf8_lossy(&output.stderr),
        String::from_utf8_lossy(&output.stdout),
    );
}

#[test]
fn the_scratch_node_crates_use_the_tables_the_scaffold_writes() {
    let templates = sibling_crate("cerulion_cli_engine").join("src/templates.rs");
    let text = std::fs::read_to_string(&templates).expect("read the node scaffold templates");
    // The body of `generate_cargo_toml`, up to the next public function.
    let start = text
        .find("pub fn generate_cargo_toml(")
        .expect("the scaffold still defines generate_cargo_toml");
    let body = &text[start..];
    let body = &body[..body[1..].find("\npub fn ").map_or(body.len(), |at| at + 1)];
    // Everything the template writes from its `[lib]` table up to its
    // `[dependencies]` table, compared whole: a key or a table added there
    // fails this as surely as a changed one.
    let tables = body.find("[lib]").and_then(|from| {
        body[from..]
            .find("[dependencies]")
            .map(|to| &body[from..from + to])
    });
    assert_eq!(
        tables.map(str::trim_end),
        Some(NODE_CRATE_TABLES.trim_end()),
        "the tables between `[lib]` and `[dependencies]` in the manifest \
         `cerulion node create` writes are no longer the ones the \
         one-dependency scratch crates are built with, so those crates stopped \
         building the way a scaffolded node builds. Update NODE_CRATE_TABLES \
         to match generate_cargo_toml in {}",
        templates.display()
    );
}

#[test]
fn a_package_that_names_neither_crate_is_told_which_dependency_is_missing() {
    // The negative control. Without it, a pass of the positive case could be
    // a harness whose cargo invocation checks nothing.
    let target_dir = scratch_target_dir("no-runtime");
    let scratch = target_dir.join("consumer");
    let _ = std::fs::remove_dir_all(&scratch);
    write_single_package(
        &scratch,
        "no_runtime_consumer",
        "",
        &path_dependency("cerulion_macros", &sibling_crate("cerulion_macros")),
        "main.rs",
        NO_RUNTIME_SOURCE,
    );

    let output = match check_scratch_project(&scratch, &target_dir, &[]) {
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
    // Each of the three macros is a separate expansion and emits its own
    // report, each at its own span (the struct's name, the impl attribute,
    // the derived type's name), so rustc prints all three.
    let reports = stderr.matches("must name the runtime").count();
    assert_eq!(
        reports, 3,
        "the source uses three exported macros and each reports the missing \
         dependency on its own; got {reports} reports.\n\
         --- cargo stderr ---\n{stderr}"
    );
}

/// The `(line, column)` of every `// E0277 at <text>` mark in `source`, both
/// counted from one as rustc counts them.
fn marked_locations(source: &str) -> BTreeSet<(usize, usize)> {
    const MARK: &str = "// E0277 at ";
    let mut marks = BTreeSet::new();
    for (index, line) in source.lines().enumerate() {
        let Some(at) = line.find(MARK) else {
            continue;
        };
        let token = line[at + MARK.len()..].trim();
        let column = line
            .find(token)
            .expect("a mark names a token on its own line");
        assert!(
            column < at,
            "the marked token must precede its mark: {line}"
        );
        marks.insert((index + 1, column + 1));
    }
    marks
}

/// The `(line, column)` of every `error[<code>]` cargo reported against
/// `src/lib.rs`, read from `--message-format short` output, where each
/// diagnostic is one `<path>:<line>:<column>: error[<code>]: ...` line.
fn reported_locations(stderr: &str, code: &str) -> BTreeSet<(usize, usize)> {
    let tag = format!(": error[{code}]");
    let mut found = BTreeSet::new();
    for line in stderr.lines() {
        let Some(at) = line.find(&tag) else {
            continue;
        };
        let mut fields = line[..at].rsplitn(3, ':');
        let column = fields.next().and_then(|c| c.trim().parse::<usize>().ok());
        let row = fields.next().and_then(|r| r.trim().parse::<usize>().ok());
        let path = fields.next().unwrap_or("");
        match (path.ends_with("lib.rs"), row, column) {
            (true, Some(row), Some(column)) => {
                found.insert((row, column));
            }
            _ => panic!("an {code} line this parser cannot place: {line}"),
        }
    }
    found
}

/// Check the span case's package under one root and require every `E0277`
/// at exactly the marked fields.
fn field_errors_are_reported_at_the_field(case: &str, uses: &str, dependencies: &str) {
    let target_dir = scratch_target_dir(CLOSURE_TARGET);
    let scratch = target_dir.join(case);
    let _ = std::fs::remove_dir_all(&scratch);
    let source = format!("{uses}{FIELD_ERRORS_BODY}");
    let expected = marked_locations(&source);
    assert_eq!(
        expected.len(),
        FIELD_ERROR_MARKS,
        "the span case's marks did not parse: {expected:?}"
    );
    // `cdylib` is declared and left off, so the node's FFI module is cfg'd
    // out without an unexpected-cfg warning.
    write_single_package(
        &scratch,
        "field_errors",
        "[features]\ncdylib = []\n",
        dependencies,
        "lib.rs",
        &source,
    );

    let output = match check_scratch_project(&scratch, &target_dir, &["--message-format", "short"])
    {
        Ok(output) => output,
        Err(stderr) => return allow_skip_or_fail(&stderr),
    };
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !output.status.success(),
        "the span case holds {FIELD_ERROR_MARKS} type errors, so it must not \
         check clean.\n--- cargo stderr ---\n{stderr}"
    );
    let reported = reported_locations(&stderr, "E0277");
    let missing: Vec<_> = expected.difference(&reported).collect();
    // The node's generated `NodeEntry` impl carries its own
    // `SlamNode: CerulionState` predicate (`state_where_clause` in the macro
    // crate's codegen), built from the struct's name and call-site path
    // tokens, so rustc reports it at the start of the `#[cerulion_node]`
    // attribute. That report is allowed here, not pinned.
    let node_attribute = (attribute_line(&source, "#[cerulion_node("), 1);
    let stray: Vec<_> = reported
        .difference(&expected)
        .filter(|at| **at != node_attribute)
        .collect();
    assert!(
        missing.is_empty() && stray.is_empty(),
        "every unsatisfied state bound must be reported at the field whose \
         type fails it. Marked fields with no report there, as (line, column): \
         {missing:?}; reports anywhere else but the node attribute: {stray:?}. \
         A field's report moved to its `#[derive(..)]` or `#[cerulion_node]` \
         line means the predicate's crate root carries the call-site span.\n\
         --- source ---\n{source}\n--- cargo stderr ---\n{stderr}"
    );
}

/// The one-based line of the only line in `source` that starts, after
/// indentation, with `prefix`.
fn attribute_line(source: &str, prefix: &str) -> usize {
    let lines: Vec<usize> = source
        .lines()
        .enumerate()
        .filter(|(_, line)| line.trim_start().starts_with(prefix))
        .map(|(index, _)| index + 1)
        .collect();
    assert_eq!(lines.len(), 1, "exactly one line starts {prefix:?}");
    lines[0]
}

#[test]
fn a_field_that_fails_a_state_bound_is_reported_at_the_field_under_the_umbrella_root() {
    field_errors_are_reported_at_the_field(
        "field-errors-umbrella",
        "use cerulion::msgs::geometry_msgs::Vector3;\nuse cerulion::prelude::*;\n",
        &path_dependency("cerulion", &umbrella_dir()),
    );
}

#[test]
fn a_field_that_fails_a_state_bound_is_reported_at_the_field_under_the_runtime_root() {
    let dependencies = [
        path_dependency("cerulion_core", &sibling_crate("cerulion_core")),
        path_dependency(
            "native_ros2_messages",
            &sibling_crate("native_ros2_messages"),
        ),
    ]
    .join("\n");
    field_errors_are_reported_at_the_field(
        "field-errors-runtime",
        "use cerulion_core::prelude::*;\n\
         use cerulion_core::state::CerulionState;\n\
         use native_ros2_messages::geometry_msgs::Vector3;\n",
        &dependencies,
    );
}
