// SPDX-License-Identifier: AGPL-3.0-only
//! The `node build` → optional-SYSTEM-dependency WIRING.
//!
//! `system_deps`'s inline oracles pin the DECISION (parse, evaluate, render)
//! and `system_deps_manifest_test.rs` pins this repo's manifests. Neither
//! touches `node_build`'s body, which is where the decision has to actually
//! reach cargo and reach the user. Without this file, three mutations leave the
//! rest of the suite green while reverting the feature end to end:
//!
//! | Mutation | What ships | Killed by |
//! |---|---|---|
//! | drop the `--features` append | a capability-less cdylib on a machine that HAS the library | [`the_resolved_features_reach_the_cargo_argv`] |
//! | `.unwrap_or_default()` the metadata parse | a typo'd key silently disables the probe forever | [`a_malformed_metadata_block_stops_the_build_before_cargo_runs`] |
//! | report AFTER cargo | a build that fails BECAUSE of the injected feature never mentions it | [`the_report_reaches_the_user_before_cargo_runs`] |
//!
//! The same body owns the one line printed while cargo runs (its output is
//! captured, so without the line a first build is minutes of blank terminal);
//! the last three tests pin that it goes out, once, before cargo, and never
//! for a build refused up front.
//!
//! The tempdir workspaces hold a single dependency-free stub crate, so the
//! cargo invocations are hermetic (no network, no registry) and fast.

use std::path::{Path, PathBuf};

use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::node_cmd::{
    cargo_build_args, node_build_with_progress, node_build_with_reporter,
};

/// A pkg-config module name no machine can resolve, so the probe's verdict is
/// the same on a developer Mac, a GStreamer-laden Orin and a bare CI runner.
const ABSENT_MODULE: &str = "cerulion-definitely-absent-probe-module";

const FEATURE: &str = "probe";

/// Lay out a minimal Cerulion workspace holding ONE node crate.
///
/// `default_features` and `lib_rs` are the two axes the tests vary: whether
/// the gated feature is (wrongly) in `default`, and whether the crate compiles
/// at all.
fn stub_workspace(
    dir: &Path,
    metadata_block: &str,
    default_features: &str,
    lib_rs: &str,
) -> PathBuf {
    let root = dir.to_path_buf();
    let node = root.join("nodes").join("probe_node");
    std::fs::create_dir_all(node.join("src")).expect("create the node crate dirs");
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nmembers = [\"nodes/*\"]\nresolver = \"2\"\n",
    )
    .expect("write the workspace manifest");
    std::fs::write(
        node.join("Cargo.toml"),
        format!(
            "[package]\nname = \"probe_node\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n\
             [features]\n{FEATURE} = []\ndefault = [{default_features}]\n\n\
             {metadata_block}"
        ),
    )
    .expect("write the node manifest");
    std::fs::write(node.join("src").join("lib.rs"), lib_rs).expect("write the node source");
    root
}

/// A well-formed declaration whose module can never resolve.
fn absent_dep_block() -> String {
    format!(
        "[package.metadata.cerulion.optional-system-deps.{FEATURE}]\n\
         pkg-config = [\"{ABSENT_MODULE}\"]\n\
         summary = \"the wiring probe\"\n\
         without-it = \"the probe capability is unavailable\"\n\
         install.macos = \"brew install probe\"\n\
         install.debian = \"sudo apt install probe\"\n\
         install.linux = \"install probe\"\n"
    )
}

/// THE argv pin. Everything else this module does is in service of getting
/// `--features <resolved>` onto the cargo invocation; deleting that append
/// produced a feature-less artifact with every decision oracle still green,
/// because no test had ever looked at the command line.
#[test]
fn the_resolved_features_reach_the_cargo_argv() {
    assert_eq!(
        cargo_build_args("camera_jpeg", true, &["gstreamer".to_string()]),
        vec![
            "build",
            "-p",
            "camera_jpeg",
            "--release",
            "--features",
            "gstreamer"
        ],
        "a satisfied dep must put --features on the cargo command line"
    );
    // Debug build, same shape minus --release.
    assert_eq!(
        cargo_build_args("camera_jpeg", false, &["gstreamer".to_string()]),
        vec!["build", "-p", "camera_jpeg", "--features", "gstreamer"]
    );
    // MULTI-dep: cargo takes ONE comma-joined `--features` value, and the
    // order is the report's (feature-name sorted). Two `--features` flags
    // would also work for cargo, but the joined form is what the doc claims.
    assert_eq!(
        cargo_build_args("multi", false, &["alsa".to_string(), "zlib".to_string()]),
        vec!["build", "-p", "multi", "--features", "alsa,zlib"]
    );
    // Nothing satisfied ⇒ NO `--features` flag at all (an empty value would
    // be a different, and wrong, thing to hand cargo).
    assert_eq!(
        cargo_build_args("plain", false, &[]),
        vec!["build", "-p", "plain"]
    );
    assert_eq!(
        cargo_build_args("plain", true, &[]),
        vec!["build", "-p", "plain", "--release"]
    );
}

/// THE ordering pin, with a real order oracle rather than a claim: the sink
/// records whether the workspace's `Cargo.lock` exists AT REPORT TIME.
///
/// `Cargo.lock` and not `target/`: cargo writes the lockfile into the
/// WORKSPACE ROOT during resolution — before any compilation, and before the
/// stub crate's syntax error is reached — whereas the target directory is
/// relocatable by `CARGO_TARGET_DIR` or a user's `build.target-dir`, neither
/// of which this test controls. A developer with a shared target dir exported
/// would otherwise see the anti-vacuous assert below fail on a clean tree,
/// claiming cargo never ran when it ran and wrote elsewhere.
///
/// Were the notice printed by the CALLER after `node_build`
/// returned, a build that FAILED — including one that failed *because* of
/// the feature this module injected — would surface cargo's stderr and nothing
/// else. The user's next move (`cargo build -p <node>`) then SUCCEEDS, because
/// the feature is off by default, and the CLI's one decision is invisible in
/// the failure it caused.
///
/// The crate here does not compile, so this is the failure path end to end.
#[test]
fn the_report_reaches_the_user_before_cargo_runs() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(
        tmp.path(),
        &absent_dep_block(),
        "",
        "this is not valid rust and will not compile\n",
    );

    let mut notices: Vec<String> = Vec::new();
    let mut lock_existed_at_report: Option<bool> = None;
    let err = node_build_with_reporter(&ws, "probe_node", false, &mut |notice| {
        lock_existed_at_report = Some(ws.join("Cargo.lock").exists());
        notices.push(notice.to_string());
    })
    .expect_err("the stub crate must fail to compile");

    assert!(
        matches!(err, CliError::BuildFailed { .. }),
        "expected BuildFailed, got {err:?}"
    );
    assert_eq!(
        notices.len(),
        1,
        "the notice must be delivered exactly once even on the failure path"
    );
    // ORDER, observed rather than asserted: cargo writes `Cargo.lock` into the
    // workspace root as soon as it resolves, and nothing else in this test
    // creates it.
    assert_eq!(
        lock_existed_at_report,
        Some(false),
        "the report must be delivered BEFORE cargo is spawned"
    );
    assert!(
        ws.join("Cargo.lock").exists(),
        "cargo must really have run afterwards — otherwise the ordering \
         assertion above is vacuous"
    );

    // The notice names the node, the feature, the fact the feature is OFF, and
    // the install command. Which BRANCH renders (library-missing vs
    // pkg-config-tool-missing) depends on the machine, so assert only what both
    // branches must carry.
    let notice = &notices[0];
    assert!(notice.contains("probe_node"), "{notice}");
    assert!(
        notice.contains(&format!("Building WITHOUT `--features {FEATURE}`")),
        "{notice}"
    );
    assert!(
        notice.contains("probe"),
        "the install command must be present: {notice}"
    );
}

/// The parse must stop the build, not degrade to a plain one. Replacing the
/// `?` with `.unwrap_or_default()` makes a user's typo'd `pkgconfig` key ship
/// a feature-less node forever under a green build — the exact silent
/// capability loss the strict shapes exist to prevent, and unreachable by the
/// in-repo manifest walk (which can only see a cerulion checkout).
#[test]
fn a_malformed_metadata_block_stops_the_build_before_cargo_runs() {
    let tmp = tempfile::tempdir().expect("tempdir");
    // `pkgconfig` — the canonical typo. Valid TOML, refused by the strict shape.
    let typo = format!(
        "[package.metadata.cerulion.optional-system-deps.{FEATURE}]\n\
         pkgconfig = [\"{ABSENT_MODULE}\"]\n\
         summary = \"s\"\nwithout-it = \"w\"\ninstall.macos = \"c\"\n"
    );
    let ws = stub_workspace(tmp.path(), &typo, "", "pub fn ok() {}\n");

    let mut reported = 0usize;
    let err = node_build_with_reporter(&ws, "probe_node", false, &mut |_| reported += 1)
        .expect_err("a malformed block must refuse the build");

    match &err {
        CliError::BuildFailed { target, reason } => {
            assert_eq!(target, "probe_node");
            assert!(
                reason.contains("could not parse [package.metadata.cerulion]"),
                "the refusal must be the PARSE error, not a cargo failure: {reason}"
            );
            assert!(
                reason.contains("pkgconfig"),
                "name the offending key: {reason}"
            );
        }
        other => panic!("expected BuildFailed, got {other:?}"),
    }
    assert_eq!(
        reported, 0,
        "there is no decision to report — nothing was probed"
    );
    assert!(
        !ws.join("Cargo.lock").exists(),
        "cargo must NOT have run: a malformed declaration is refused up front, \
         never degraded into a plain build"
    );
}

/// The contract's central rule, enforced for a USER's node rather than only
/// for manifests inside this repo. A gated feature left in `default` makes the
/// system library a hard prerequisite for the plain `cargo build` again, which
/// is precisely the breakage the declaration removes — and the probe then buys
/// nothing, since the feature is already on.
///
/// It WARNS rather than refuses (the artifact is still correct), and the warn
/// must fire on the failure path too — the manifest smell is independent of
/// whether the code compiles.
#[tracing_test::traced_test]
#[test]
fn a_gated_feature_left_in_default_warns_at_node_build() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(
        tmp.path(),
        &absent_dep_block(),
        &format!("\"{FEATURE}\""),
        "this is not valid rust and will not compile\n",
    );
    let _ = node_build_with_reporter(&ws, "probe_node", false, &mut |_| {});

    assert!(
        logs_contain("is also in"),
        "a gated feature in `default` must be called out"
    );
    assert!(logs_contain(FEATURE), "the offending feature must be named");
}

/// The unsatisfied-dep warn must fire from the REAL build path.
///
/// `node_cmd`'s three inline `#[traced_test]` pins call `warn_unsatisfied_deps`
/// directly, which proves both TEXTS but not that anything invokes it:
/// deleting the `if report.has_missing()` call site leaves
/// all three green. This drives the production entry point instead.
///
/// SCOPE: which arm renders depends on the machine (does it have a
/// usable `pkg-config`?), so the expectation is derived from the same public
/// probe production uses. That pins the call site's EXISTENCE and its
/// `node_type` on every machine; it does not discriminate an implementation that hardcodes
/// the tool argument, since on any one machine that hardcoding agrees with reality
/// exactly half the time — and on THIS machine's half it agrees. The two
/// `#[traced_test]` arms in `node_cmd` cover the texts the local machine cannot
/// reach.
#[tracing_test::traced_test]
#[test]
fn the_unsatisfied_dep_warn_fires_from_the_real_build_path() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(
        tmp.path(),
        &absent_dep_block(),
        "",
        "this is not valid rust and will not compile\n",
    );
    let _ = node_build_with_reporter(&ws, "probe_node", false, &mut |_| {});

    assert!(
        logs_contain("probe_node"),
        "the warn must name the node it is about"
    );
    if cerulion_cli_engine::system_deps::pkg_config_tool().can_probe() {
        assert!(
            logs_contain("optional system dependencies are missing"),
            "this machine CAN probe, so an unresolved module really is a missing library"
        );
        assert!(!logs_contain("UNPROVEN"), "…and must not hedge");
    } else {
        assert!(
            logs_contain("UNPROVEN"),
            "this machine CANNOT probe, so nothing was looked at"
        );
        assert!(
            !logs_contain("dependencies are missing"),
            "…and the warn must not contradict the notice"
        );
    }
}

/// Anti-tautology for the warn above: the COMPLIANT shape — the same crate,
/// the same declaration, the feature simply out of `default` — must be silent.
/// Without this, a warn that fired unconditionally would pass the test above.
#[tracing_test::traced_test]
#[test]
fn a_compliant_manifest_produces_no_default_feature_warning() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(
        tmp.path(),
        &absent_dep_block(),
        "",
        "this is not valid rust and will not compile\n",
    );
    let _ = node_build_with_reporter(&ws, "probe_node", false, &mut |_| {});

    assert!(
        !logs_contain("is also in"),
        "a feature correctly kept out of `default` must not be warned about"
    );
}

/// A node declaring NOTHING is silent: no notice, no probe, no warning. The
/// overwhelmingly common case, and noise here would train people to ignore the
/// notice on the one node that has something to say.
#[test]
fn a_node_with_no_declaration_reports_nothing() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(tmp.path(), "", "", "this is not valid rust\n");

    let mut reported = 0usize;
    let err = node_build_with_reporter(&ws, "probe_node", false, &mut |_| reported += 1)
        .expect_err("the stub crate must fail to compile");
    assert!(matches!(err, CliError::BuildFailed { .. }), "{err:?}");
    assert_eq!(reported, 0, "nothing declared ⇒ nothing to say");
}

/// THE progress pin. Cargo's output is captured, so between the command and
/// its result the user sees only what this sink is handed; with nothing, a
/// first build (which compiles the runtime too) is minutes of blank terminal.
///
/// The node declares NO system deps on purpose: that is the shape every
/// scaffolded node has, and the one where `on_report` is correctly silent, so
/// the progress line cannot be riding on the notice. Order is observed the
/// same way the notice test observes it, by whether cargo has written
/// `Cargo.lock` yet.
#[test]
fn the_progress_line_reaches_its_sink_before_cargo_starts() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(tmp.path(), "", "", "this is not valid rust\n");

    let mut reported = 0usize;
    let mut lines: Vec<String> = Vec::new();
    let mut lock_existed_at_line: Option<bool> = None;
    let err = node_build_with_progress(
        &ws,
        "probe_node",
        false,
        &mut |_| reported += 1,
        &mut |line| {
            lock_existed_at_line = Some(ws.join("Cargo.lock").exists());
            lines.push(line.to_string());
        },
    )
    .expect_err("the stub crate must fail to compile");
    assert!(matches!(err, CliError::BuildFailed { .. }), "{err:?}");

    assert_eq!(
        reported, 0,
        "nothing declared, so the notice sink stays silent"
    );
    assert_eq!(
        lines.len(),
        1,
        "exactly one progress line per build: {lines:?}"
    );
    assert_eq!(
        lock_existed_at_line,
        Some(false),
        "the line must go out BEFORE cargo is spawned; after it, a failed \
         build would never print it and a slow one would print it too late"
    );
    assert!(
        ws.join("Cargo.lock").exists(),
        "cargo must really have run afterwards, or the ordering assertion \
         above is vacuous"
    );

    let line = &lines[0];
    assert!(line.contains("'probe_node'"), "name the node: {line}");
    assert!(
        line.contains("first build of a workspace also compiles the Cerulion runtime"),
        "say WHY the first build is slow: {line}"
    );
    assert!(line.contains("a few minutes"), "say how slow: {line}");
    assert!(
        !line.contains('\n'),
        "one line, and the caller owns the newline: {line:?}"
    );
}

/// With a notice to deliver as well, the progress line comes SECOND: it
/// explains the silence that follows, so it has to be the last thing printed
/// before that silence.
#[test]
fn the_progress_line_follows_the_system_dep_notice() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = stub_workspace(
        tmp.path(),
        &absent_dep_block(),
        "",
        "this is not valid rust\n",
    );

    // One log both sinks append to, so their relative order is observable.
    let events = std::cell::RefCell::new(Vec::<&'static str>::new());
    let _ = node_build_with_progress(
        &ws,
        "probe_node",
        false,
        &mut |_| events.borrow_mut().push("notice"),
        &mut |_| events.borrow_mut().push("progress"),
    );
    assert_eq!(*events.borrow(), vec!["notice", "progress"]);
}

/// Anti-tautology for the pin above: a build REFUSED before cargo is reached
/// says nothing about building. A sink called unconditionally at the top of
/// the function would pass the two tests above and fail here.
#[test]
fn a_build_refused_before_cargo_prints_no_progress_line() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let typo = format!(
        "[package.metadata.cerulion.optional-system-deps.{FEATURE}]\n\
         pkgconfig = [\"{ABSENT_MODULE}\"]\n\
         summary = \"s\"\nwithout-it = \"w\"\ninstall.macos = \"c\"\n"
    );
    let ws = stub_workspace(tmp.path(), &typo, "", "pub fn ok() {}\n");

    let mut lines = 0usize;
    let err =
        node_build_with_progress(&ws, "no_such_node", false, &mut |_| {}, &mut |_| lines += 1)
            .expect_err("an unknown node must be refused");
    assert!(matches!(err, CliError::NodeNotFound { .. }), "{err:?}");

    let err = node_build_with_progress(&ws, "probe_node", false, &mut |_| {}, &mut |_| lines += 1)
        .expect_err("a malformed metadata block must be refused");
    assert!(matches!(err, CliError::BuildFailed { .. }), "{err:?}");

    assert_eq!(lines, 0, "neither refusal reached cargo");
    assert!(!ws.join("Cargo.lock").exists(), "and cargo never ran");
}
