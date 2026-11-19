// SPDX-License-Identifier: AGPL-3.0-only
//! Permanent regression pin — producer-graph-FIRST start order at the
//! TRUE production surface (two `cerulion graph run` PROCESSES over cdylib
//! nodes whose source is the CLI scaffold's, byte for byte).
//!
//! # The contract
//!
//! The cross-step HOLD needs a snapshot CONSUMER to attach with iceoryx2
//! `subscriber_max_borrowed_samples >= 3`, but iceoryx2 bakes that cap in at
//! service CREATION (default 2) by whichever process creates the service
//! first. If a PRODUCER graph starting before its snapshot-consumer graph
//! created the shared topic at the default borrow 2, the consumer's open
//! would HARD-ABORT with
//! `PublishSubscribeOpenError::DoesNotSupportRequestedMinSubscriberBorrowedSamples`
//! — a real trap on robots, where processes restart in arbitrary order, and
//! one that only "always start the consumer first" would avoid. So
//! OWNED topics CREATE at the hold borrow
//! floor `max(genuine, SUBSCRIBER_MAX_BORROWED_HELD)`, and start order does not
//! matter. This file pins that contract end-to-end.
//!
//! # What this test does
//!
//! Scaffolds a workspace ENTIRELY through the real `cerulion` binary
//! (`env!("CARGO_BIN_EXE_cerulion")`):
//!   - `prod` — `period_ms=10`, one `geometry_msgs::Vector3` output — staged
//!     into graph `pgraph` (prefix `sop`), owning topic `/sop/prod/out`;
//!   - `cons` — `period_ms=10`, one PLAIN non-trigger `#[input]` (the cross-step
//!     hold path) — staged into graph `cgraph` (prefix `soc`), its input
//!     bound to the ABSOLUTE producer topic `/sop/prod/out`.
//!
//! The node libraries are PREBUILT fixtures, `test_node_start_order_prod_cdylib`
//! and `test_node_start_order_cons_cdylib`, compared against the scaffold before
//! they are loaded (see "The fixture pin" below). The test copies each to
//! `<ws>/target/debug/lib<node>.so`, where `graph run` resolves the node with
//! `CARGO_TARGET_DIR` unset. Then it
//! starts `graph run pgraph` FIRST (the load-bearing ordering — without the
//! floor this creates `/sop/prod/out` at borrow 2), waits for the producer to go live
//! (its owned service now exists — created at the borrow FLOOR), starts
//! `graph run cgraph`, and asserts the consumer builds PAST the borrow-open and
//! RUNS: it reaches the "starting graph (live)" readiness line, stays alive
//! over a bounded window, and its logs never carry the
//! `DoesNotSupportRequestedMinSubscriberBorrowedSamples` regression signature.
//! Both processes then exit 0 on SIGINT (the clean-shutdown contract).
//!
//! Both runs pass `--single-process` so each is ONE monolith process — the
//! Unix default would otherwise auto-partition each graph into a
//! supervisor + worker, obscuring the two-process cross-graph scenario this pin
//! is actually about (and changing the readiness/teardown shape).
//!
//! # The fixture pin
//!
//! After the scaffold and the staging, the scaffolded nodes must be exactly
//! `prod` and `cons`, and each node crate and its fixture crate must hold
//! exactly `Cargo.toml` and `src/lib.rs`. Each node's `src/lib.rs` must equal
//! its fixture's byte for byte, and its `Cargo.toml` must equal its fixture's
//! manifest once three edits a workspace member needs are applied: the package
//! renamed to the fixture's name, `publish = false` under it, and the
//! inherited `[lints]` table appended. A template change that adds a file, or
//! adds, drops or rewrites a line of either file, fails here, naming both
//! texts. The library must also be no older than the fixture's `src/lib.rs`,
//! so a library built from an earlier fixture source is refused rather than
//! loaded. A library file name carries no source hash, so a target directory
//! shared by two checkouts can hold the other checkout's build of the same
//! fixture; give each checkout its own. To follow an intended template change,
//! copy the scaffolded files over the fixture's, reapply the three edits and
//! rebuild the fixtures.
//!
//! What the fixtures do not exercise is the build of a generated workspace
//! itself: the unlocked resolve, the iceoryx2 patch and the generated message
//! crate. `read_identity_oracle_test` builds a generated workspace, a
//! `node create` template node included, through the engine's `node_build` on
//! every platform, and `node_build_notice_e2e_test` pins where `cerulion node
//! build` puts the library for each `CARGO_TARGET_DIR` form. Loading and
//! running a library built in a generated workspace is
//! `crates/cerulion_py/tests/test_cli_pynode.py`'s, which builds a Rust node
//! with `cerulion node build` and runs it in a graph.
//!
//! Build the fixtures first:
//! `cargo build -p test_node_start_order_prod_cdylib -p test_node_start_order_cons_cdylib`.
//!
//! # Anti-tautology
//!
//! The negative control was run by hand on Linux x86_64: a binary built WITH
//! the borrow floor PASSES this exact scenario, and one built WITHOUT it
//! ABORTS the consumer with `...DoesNotSupportRequestedMinSubscriberBorrowedSamples`.
//! That negative control is NOT reproducible in-tree — it needs a build
//! of the binary without the floor, which CI cannot produce from this
//! checkout — so it is recorded HERE rather than encoded as a second test arm.
//! The in-tree teeth are: (1) the signature assertion (the exact borrow-open
//! failure string, asserted absent), and (2) the scaffold-shape guard, which
//! fails LOUDLY if the CLI ever stops emitting a plain non-trigger `#[input]`
//! for `cons` — a trigger input would drain-not-hold, provision NO borrow-3
//! requirement, and make this whole test pass VACUOUSLY (green with or without
//! the borrow floor). The fixture pin carries that guard over to the loaded
//! library: the `cons` fixture is the scaffold's source.
//!
//! # Qualified schema names
//!
//! Ports are added with the QUALIFIED schema name `geometry_msgs::Vector3`.
//! Bare (`Vector3`) names generate broken node-crate imports, so the
//! scaffold would not compile.
//!
//! Every wait is bounded and generous for a slow CI runner; none asserts an
//! exact timing (liveness and the regression signature only).
//!
//! Runs on the GLOBAL iceoryx2 namespace (no isolation seam at the binary
//! layer, same as `node_run_e2e_test.rs` / `mp_record_e2e_test.rs`);
//! `#[serial]` + a unique prefix pair (`sop`/`soc`) + a unique tempdir keep
//! it safe.

#![cfg(not(target_os = "macos"))]
// WAIVED WHOLE on macOS: upstream iceoryx2 0.10.0 defect 2034. The start-order arm
// spawns `cerulion graph run` children that load plugin nodes, and on macOS such a
// process cannot create any further event resource; the two manifest unit tests
// spawn nothing and share the file gate. The mechanism, the derivation
// that selects this file, and the coverage this costs are stated once in
// `cerulion_core/tests/upstream_waivers_test.rs`. Runs normally on Linux.
#![cfg(unix)]

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use serial_test::serial;

/// The EXACT iceoryx2 open-error variant a hold consumer aborts with when its
/// topic was created below the borrow floor
/// (see `cerulion_core/src/transport/mod.rs`). This must NEVER appear in
/// the consumer's captured output — the owned topic is created at the
/// borrow floor, so the consumer's borrow-3 open succeeds.
const REGRESSION_SIGNATURE: &str = "DoesNotSupportRequestedMinSubscriberBorrowedSamples";

/// The live-loop readiness line `graph_cmd::graph_run` logs AFTER the runtime
/// (and hence every owned topic service) is built, right before entering the
/// live loop.
const READY_LINE: &str = "starting graph (live)";

// ── Timeout budget ──────────────────────────────────────────────────────────
// Every wait is bounded and generous; none asserts an exact timing.
/// Fast scaffold steps (workspace/node/graph file ops via the binary).
const SCAFFOLD_TIMEOUT: Duration = Duration::from_secs(60);
/// Producer readiness (mirrors `node_run_e2e_test.rs`'s 30 s readiness wait).
const PRODUCER_READY_TIMEOUT: Duration = Duration::from_secs(30);
/// Consumer readiness — a touch more headroom than the producer (it opens the
/// external service the producer created).
const CONSUMER_READY_TIMEOUT: Duration = Duration::from_secs(40);
/// The consumer must keep running for this window after going live (proves it
/// does not crash shortly after building).
const LIVENESS_WINDOW: Duration = Duration::from_secs(6);
/// Clean-shutdown exit after SIGINT (mirrors `node_run_e2e_test.rs`'s 40 s).
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(40);

/// SIGKILL + reap on drop so a panicking assert never leaks a child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses. `Some(status)` =
/// exited; `None` = still running at the deadline.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

/// `kill(2)` a real spawned child. Guarded against ever signalling pid <= 1
/// (never signal init / the whole process group).
fn send_signal(pid: u32, sig: libc::c_int) {
    assert!(pid > 1, "refusing to signal pid {pid} (<= 1)");
    // SAFETY: kill(2) with a valid pid + signal; no memory is touched.
    unsafe {
        libc::kill(pid as libc::pid_t, sig);
    }
}

fn read_file(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_default()
}

/// Bounded poll for `needle` in the (growing) log file at `path`; returns the
/// full content once seen, panics at the deadline with the log so far (the
/// caller's `ChildGuard` reaps on the panic unwind).
fn wait_for_log_line(path: &Path, needle: &str, timeout: Duration) -> String {
    let start = Instant::now();
    loop {
        let content = read_file(path);
        if content.contains(needle) {
            return content;
        }
        assert!(
            start.elapsed() < timeout,
            "log line {needle:?} not seen within {timeout:?}; log so far:\n{content}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Run one scaffold step through the REAL binary to completion in `root`,
/// asserting exit 0. Output is redirected to files so a full pipe never blocks
/// the child; on timeout or failure both are dumped.
///
/// `CARGO_TARGET_DIR` is stripped, as for `graph run` below, so no step reads
/// the caller's target directory. `NO_COLOR` keeps captured output plain.
fn run_step(root: &Path, args: &[&str], timeout: Duration) {
    let out_path = root.join("step.stdout");
    let err_path = root.join("step.stderr");
    let mut guard = ChildGuard(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(args)
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("NO_COLOR", "1")
            .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
            .stdout(Stdio::from(std::fs::File::create(&out_path).unwrap()))
            .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
            .spawn()
            .unwrap_or_else(|e| panic!("failed to spawn `cerulion {}`: {e}", args.join(" "))),
    );
    match wait_bounded(&mut guard.0, timeout) {
        None => panic!(
            "`cerulion {}` did not finish within {timeout:?}\nstdout:\n{}\nstderr:\n{}",
            args.join(" "),
            read_file(&out_path),
            read_file(&err_path)
        ),
        Some(status) => assert!(
            status.success(),
            "`cerulion {}` must exit 0, got {status:?}\nstdout:\n{}\nstderr:\n{}",
            args.join(" "),
            read_file(&out_path),
            read_file(&err_path)
        ),
    }
}

/// Spawn `cerulion graph run <graph> --single-process` as a background child in
/// `root`, stdout+stderr to distinct files (readable while the child runs).
fn spawn_graph_run(root: &Path, graph: &str) -> (ChildGuard, PathBuf, PathBuf) {
    let stdout_path = root.join(format!("{graph}.stdout"));
    let stderr_path = root.join(format!("{graph}.stderr"));
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["graph", "run", graph, "--single-process"])
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        // Hermetic — no scouting session/gateway in CI (a real-clock
        // run is permissive-by-default; the kill-switch env keeps it LOCAL-ONLY).
        .env("CERULION_NETWORK", "off")
        .env("NO_COLOR", "1")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .unwrap_or_else(|e| panic!("failed to spawn `cerulion graph run {graph}`: {e}"));
    (ChildGuard(child), stdout_path, stderr_path)
}

/// One prebuilt node library and the scaffolded node it stands in for.
struct Fixture {
    /// The node type the test scaffolds.
    node: &'static str,
    /// The fixture crate, from this package's directory. Its last segment is
    /// the package name, which is also the library's crate name.
    dir: &'static str,
}

impl Fixture {
    fn package(&self) -> &'static str {
        self.dir.rsplit_once('/').map_or(self.dir, |(_, last)| last)
    }
}

const FIXTURES: [Fixture; 2] = [
    Fixture {
        node: "prod",
        dir: "../test_fixtures/test_node_start_order_prod_cdylib",
    },
    Fixture {
        node: "cons",
        dir: "../test_fixtures/test_node_start_order_cons_cdylib",
    },
];

/// Every file a scaffolded Rust node crate holds, and so every file its
/// fixture may hold: a template that adds a file (a `build.rs`, a second
/// source) fails the pin even though the two compared files still match.
const CRATE_FILES: [&str; 2] = ["Cargo.toml", "src/lib.rs"];

/// `scaffold`, a node manifest `node create` wrote for `node`, as the manifest
/// of a member of this repository's workspace: the package renamed to
/// `package`, `publish = false` under it, and the inherited `[lints]` table
/// appended. A fixture manifest differs from its scaffold by these three edits
/// and nothing else.
fn as_fixture_manifest(scaffold: &str, node: &str, package: &str) -> String {
    let name_line = format!("name = \"{node}\"\n");
    assert_eq!(
        scaffold.matches(&name_line).count(),
        1,
        "fixture pin: the scaffolded manifest must carry {name_line:?} exactly once:\n{scaffold}"
    );
    let renamed = scaffold.replacen(
        &name_line,
        &format!("name = \"{package}\"\npublish = false\n"),
        1,
    );
    format!("{renamed}\n[lints]\nworkspace = true\n")
}

/// The whole file at `path`; a missing or unreadable file fails the test
/// rather than reading as empty.
fn read_required(path: &Path) -> String {
    std::fs::read_to_string(path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

/// Every file under `dir`, as `/`-separated paths relative to it.
fn files_under(dir: &Path) -> BTreeSet<String> {
    let mut files = BTreeSet::new();
    let mut pending = vec![dir.to_path_buf()];
    while let Some(at) = pending.pop() {
        let entries =
            std::fs::read_dir(&at).unwrap_or_else(|e| panic!("cannot list {}: {e}", at.display()));
        for entry in entries {
            let path = entry
                .unwrap_or_else(|e| panic!("cannot list {}: {e}", at.display()))
                .path();
            if path.is_dir() {
                pending.push(path);
            } else {
                let relative = path.strip_prefix(dir).expect("listed under dir");
                let parts: Vec<_> = relative
                    .components()
                    .map(|c| c.as_os_str().to_string_lossy().into_owned())
                    .collect();
                files.insert(parts.join("/"));
            }
        }
    }
    files
}

fn modified(path: &Path) -> std::time::SystemTime {
    std::fs::metadata(path)
        .and_then(|m| m.modified())
        .unwrap_or_else(|e| panic!("cannot read the mtime of {}: {e}", path.display()))
}

/// Compare the scaffolded `nodes/<node>/` crate in `root` with its fixture,
/// then copy the fixture's prebuilt library to where `graph run` resolves
/// `<node>`.
fn install_pinned_fixture(root: &Path, fixture: &Fixture) {
    let (node, package) = (fixture.node, fixture.package());
    let scaffold_dir = root.join("nodes").join(node);
    let fixture_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join(fixture.dir);
    let crate_files: BTreeSet<String> = CRATE_FILES.iter().map(|f| f.to_string()).collect();
    for (side, dir) in [("scaffolded", &scaffold_dir), ("fixture", &fixture_dir)] {
        assert_eq!(
            files_under(dir),
            crate_files,
            "fixture pin: the {side} crate {} must hold exactly {CRATE_FILES:?}",
            dir.display()
        );
    }

    let fixture_lib_rs = fixture_dir.join("src").join("lib.rs");
    assert_eq!(
        read_required(&scaffold_dir.join("src").join("lib.rs")),
        read_required(&fixture_lib_rs),
        "fixture pin: the scaffolded nodes/{node}/src/lib.rs (left) differs from {} (right); \
         copy the scaffold over the fixture and rebuild it",
        fixture_lib_rs.display()
    );
    let fixture_manifest = fixture_dir.join("Cargo.toml");
    assert_eq!(
        as_fixture_manifest(
            &read_required(&scaffold_dir.join("Cargo.toml")),
            node,
            package
        ),
        read_required(&fixture_manifest),
        "fixture pin: the scaffolded nodes/{node}/Cargo.toml, with the three member edits \
         applied (left), differs from {} (right)",
        fixture_manifest.display()
    );

    // Cargo rebuilds a library whose source is newer than its last build, so
    // an older library was built from some other source than the one above.
    let lib = cerulion_core::testing::find_fixture_cdylib(package);
    assert!(
        modified(&lib) >= modified(&fixture_lib_rs),
        "{} is older than {}, so it was not built from the source the pin just compared; \
         run `cargo build -p {package}`",
        lib.display(),
        fixture_lib_rs.display()
    );
    let target_debug = root.join("target").join("debug");
    std::fs::create_dir_all(&target_debug).expect("create <ws>/target/debug");
    std::fs::copy(
        &lib,
        target_debug.join(cerulion_core::testing::cdylib_file_name(node)),
    )
    .unwrap_or_else(|e| panic!("copy {} into the workspace: {e}", lib.display()));
}

/// The manifest edits against hand-written text: the package renamed with
/// `publish = false` under it, the lint table appended, nothing else moved.
#[test]
fn a_scaffold_manifest_becomes_its_fixture_manifest_by_the_three_edits() {
    let scaffold = "[package]\nname = \"prod\"\nversion = \"0.1.0\"\n\n\
                    [dependencies]\ncerulion_core = { workspace = true }\n";
    assert_eq!(
        as_fixture_manifest(scaffold, "prod", "pinned_prod"),
        "[package]\nname = \"pinned_prod\"\npublish = false\nversion = \"0.1.0\"\n\n\
         [dependencies]\ncerulion_core = { workspace = true }\n\n[lints]\nworkspace = true\n"
    );
}

/// A manifest that does not name the scaffolded package exactly once is
/// refused, rather than passed through with no rename.
#[test]
#[should_panic(expected = "exactly once")]
fn a_scaffold_manifest_without_its_package_name_line_is_refused() {
    as_fixture_manifest("[package]\nname = \"other\"\n", "prod", "pinned_prod");
}

/// THE pin: producer graph started FIRST, snapshot-hold consumer graph
/// starts and runs (without the borrow floor it aborts at the borrow-3 open).
#[test]
#[serial]
fn producer_first_then_snapshot_consumer_starts_and_runs() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();

    // ── Scaffold the workspace ENTIRELY through the real binary ──────────────
    // `workspace init .` in the (empty) tempdir; every later Node/Graph command
    // runs with cwd = root and is resolved by `discover_workspace()`.
    run_step(root, &["workspace", "init", "."], SCAFFOLD_TIMEOUT);

    // Producer: a source-only period node, then add its Vector3 output.
    run_step(
        root,
        &["node", "create", "prod", "--policy", "period_ms=10"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &[
            "node",
            "modify",
            "prod",
            "-o",
            "geometry_msgs::Vector3",
            "out",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // Consumer: a period node, then add a PLAIN non-trigger input (the hold
    // path). `-i` (not `-T`) yields `#[input]`, NOT `#[input(trigger)]`.
    run_step(
        root,
        &["node", "create", "cons", "--policy", "period_ms=10"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &[
            "node",
            "modify",
            "cons",
            "-i",
            "geometry_msgs::Vector3",
            "inp",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // ── Scaffold-shape guard (anti-tautology) ────────────────────────────────
    // The whole pin rests on `cons` being a HOLD consumer: a plain non-trigger
    // `#[input]` makes `/sop/prod/out` a snapshot-source topic → the consumer
    // opens it requiring borrow 3. If the CLI scaffold ever emits a trigger
    // input instead, that requirement vanishes and the test would pass
    // vacuously — so fail LOUDLY here. (`#[input]` is NOT a substring of
    // `#[input(trigger)]`, so these two checks are mutually distinguishing.)
    let cons_src = read_file(&root.join("nodes/cons/src/lib.rs"));
    assert!(
        cons_src.contains("#[input]"),
        "scaffold-shape guard: `cons` must carry a PLAIN non-trigger `#[input]` (the cross-step \
         hold path that provisions borrow-3); generated lib.rs was:\n{cons_src}"
    );
    assert!(
        !cons_src.contains("#[input(trigger)]"),
        "scaffold-shape guard: `cons` must NOT carry `#[input(trigger)]` — a trigger input \
         drains (not holds), provisions NO borrow-3 requirement, and makes this start-order pin \
         pass VACUOUSLY; generated lib.rs was:\n{cons_src}"
    );

    // ── Graphs + staging ─────────────────────────────────────────────────────
    run_step(
        root,
        &["graph", "create", "pgraph", "-n", "sop"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &["graph", "create", "cgraph", "-n", "soc"],
        SCAFFOLD_TIMEOUT,
    );
    run_step(
        root,
        &["node", "stage", "prod", "-g", "pgraph"],
        SCAFFOLD_TIMEOUT,
    );
    // `cons.inp` binds to the ABSOLUTE producer topic — the cross-graph
    // (cross-process) source that makes the two graphs share one iceoryx2
    // service, which is where the borrow-floor asymmetry lives.
    run_step(
        root,
        &[
            "node",
            "stage",
            "cons",
            "-g",
            "cgraph",
            "-I",
            "inp",
            "/sop/prod/out",
        ],
        SCAFFOLD_TIMEOUT,
    );

    // ── The fixture pin, then the prebuilt libraries (see module doc) ───────
    let fixture_nodes: BTreeSet<String> = FIXTURES.iter().map(|f| f.node.to_string()).collect();
    let scaffolded_nodes: BTreeSet<String> = std::fs::read_dir(root.join("nodes"))
        .expect("list nodes/")
        .map(|entry| {
            entry
                .expect("list nodes/")
                .file_name()
                .to_string_lossy()
                .into_owned()
        })
        .collect();
    assert_eq!(
        fixture_nodes.len(),
        FIXTURES.len(),
        "fixture pin: FIXTURES names a node twice"
    );
    assert_eq!(
        scaffolded_nodes, fixture_nodes,
        "fixture pin: every scaffolded node needs exactly one fixture"
    );
    for fixture in &FIXTURES {
        install_pinned_fixture(root, fixture);
    }

    // ── Start the PRODUCER graph FIRST (the load-bearing ordering) ───────────
    let (mut producer, _prod_stdout, prod_stderr) = spawn_graph_run(root, "pgraph");
    // Wait for the producer to go live — its owned `/sop/prod/out` service is
    // now created (at the borrow FLOOR), so the consumer that follows
    // opens an already-existing service (deterministic ordering, no race). The
    // READY_LINE is a `tracing::info!`, which goes to STDERR (init_logging
    // writes there so a command's stdout stays clean data).
    wait_for_log_line(&prod_stderr, READY_LINE, PRODUCER_READY_TIMEOUT);

    // ── Now start the CONSUMER graph ─────────────────────────────────────────
    let (mut consumer, cons_stdout, cons_stderr) = spawn_graph_run(root, "cgraph");

    // Positive proof it built PAST the borrow-open: it reaches the live-loop
    // readiness line. Without the floor the consumer aborts at build with the borrow
    // error and never logs this — on timeout we dump its stderr (which carries
    // the abort) so the failure is legible.
    {
        let start = Instant::now();
        loop {
            // READY_LINE is a `tracing::info!` → STDERR (see the producer wait
            // above); poll the consumer's stderr for it.
            if read_file(&cons_stderr).contains(READY_LINE) {
                break;
            }
            // A borrow-open abort exits fast; surface it immediately rather than
            // waiting out the whole readiness budget.
            if let Some(status) = consumer.0.try_wait().expect("try_wait consumer") {
                panic!(
                    "consumer graph exited before going live ({status:?}) — start-order regression? \
                     it must build past the borrow-3 open and run.\nstdout:\n{}\nstderr:\n{}",
                    read_file(&cons_stdout),
                    read_file(&cons_stderr)
                );
            }
            assert!(
                start.elapsed() < CONSUMER_READY_TIMEOUT,
                "consumer graph never reached {READY_LINE:?} within {CONSUMER_READY_TIMEOUT:?}\n\
                 stdout:\n{}\nstderr:\n{}",
                read_file(&cons_stdout),
                read_file(&cons_stderr)
            );
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    // Liveness: it must keep running (not crash shortly after going live).
    if let Some(status) = wait_bounded(&mut consumer.0, LIVENESS_WINDOW) {
        panic!(
            "consumer graph exited early ({status:?}) after going live — it must start AND run.\n\
             stdout:\n{}\nstderr:\n{}",
            read_file(&cons_stdout),
            read_file(&cons_stderr)
        );
    }

    // Signature: assert on the EXACT borrow-open failure string, not just liveness.
    let cons_out = read_file(&cons_stdout);
    let cons_err = read_file(&cons_stderr);
    assert!(
        !cons_out.contains(REGRESSION_SIGNATURE) && !cons_err.contains(REGRESSION_SIGNATURE),
        "start-order regression: the consumer's output carries {REGRESSION_SIGNATURE:?} — the \
         producer-first start order aborted the snapshot consumer's borrow-3 open.\n\
         stdout:\n{cons_out}\nstderr:\n{cons_err}"
    );

    // ── Clean shutdown: SIGINT both (reverse of start order); both exit 0 ────
    send_signal(consumer.0.id(), libc::SIGINT);
    let cons_status = wait_bounded(&mut consumer.0, SHUTDOWN_TIMEOUT)
        .expect("consumer graph did not exit after SIGINT");
    assert!(
        cons_status.success(),
        "consumer graph must exit 0 on Ctrl-C, got {cons_status:?}\nstderr:\n{}",
        read_file(&cons_stderr)
    );

    send_signal(producer.0.id(), libc::SIGINT);
    let prod_status = wait_bounded(&mut producer.0, SHUTDOWN_TIMEOUT)
        .expect("producer graph did not exit after SIGINT");
    assert!(
        prod_status.success(),
        "producer graph must exit 0 on Ctrl-C, got {prod_status:?}\nstderr:\n{}",
        read_file(&prod_stderr)
    );
}
