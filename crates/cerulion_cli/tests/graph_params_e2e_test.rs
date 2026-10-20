// SPDX-License-Identifier: AGPL-3.0-only
//! `params:` on the workspace seam, over the REAL `cerulion` binary: one node
//! type staged twice with different values publishes observably different
//! frames per instance; the recording carries the values in force, defaults
//! included, and `bag play --resim all --verify` re-executes it byte for
//! byte; each of the
//! three refusal classes exits non-zero from `graph validate` AND from
//! `graph run` naming the node and the param; `node info` lists the declared
//! params from the built library.
//!
//! The node is the prebuilt `test_node_macro_param_cdylib` fixture (a period
//! node whose `cmd` is a function of its four params: `x = gain`,
//! `y = offset`, `z = frame_id.len()` or `-1.0` when disabled; defaults
//! `1.0`, `0`, `"base_link"`, `true`). Every expected frame below is a HAND
//! ORACLE from that body, never a second run of the same code.
//!
//! Prerequisite: `cargo build -p test_node_macro_param_cdylib -p
//! test_node_macro_period_cdylib` (the harness panics with that instruction
//! when a fixture is missing). `#[serial]` arms run on the default iceoryx2
//! namespace; run this binary alone with `-- --test-threads=1`.
//!
//! The symbol-less library arms live at the loader seam
//! (`cerulion_core/tests/cdylib_params_test.rs`): a library without
//! `cerulion_node_init_with_params` loads unchanged and refuses a handoff by
//! name, and every other e2e in this tree runs such libraries.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, ExitStatus, Stdio};
use std::time::Duration;

use cerulion_bag::BagReader;
use cerulion_core::wire::WireHeader;
use serial_test::serial;

mod mp_support;
use mp_support::{
    dylib_file, fixture_cdylib, read_file, send_signal, wait_for_bag, wait_for_bag_state,
    ChildGuard, RECORDED_WINDOW_TIMEOUT,
};

const PARAM_FIXTURE: &str = "test_node_macro_param_cdylib";
const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";

/// The two-instance graph: `left` and `right` are the same type with
/// different `params:`; `right` leaves `frame_id` at its default.
fn two_instances_yaml(prefix: &str) -> String {
    format!(
        "prefix: {prefix}\n\
         nodes:\n\
         \x20 - id: left\n\
         \x20   type: cam\n\
         \x20   params:\n\
         \x20     gain: 2.5\n\
         \x20     frame_id: left_cam\n\
         \x20   outputs:\n\
         \x20     - name: cmd\n\
         \x20       schema: geometry_msgs/Vector3\n\
         \x20 - id: right\n\
         \x20   type: cam\n\
         \x20   params:\n\
         \x20     gain: 0.5\n\
         \x20     enabled: false\n\
         \x20   outputs:\n\
         \x20     - name: cmd\n\
         \x20       schema: geometry_msgs/Vector3\n"
    )
}

/// A single-node graph with the given `params:` block (indented under the
/// node) for the refusal arms.
fn one_node_yaml(prefix: &str, node_type: &str, params_block: &str) -> String {
    format!(
        "prefix: {prefix}\n\
         nodes:\n\
         \x20 - id: cam0\n\
         \x20   type: {node_type}\n\
         {params_block}\
         \x20   outputs:\n\
         \x20     - name: cmd\n\
         \x20       schema: geometry_msgs/Vector3\n"
    )
}

/// Stage a workspace: `nodes/<type>/src/lib.rs` is the fixture source (for the
/// metadata walkers), `target/debug/lib<type>.*` the prebuilt fixture cdylib
/// (no in-test cargo build), `graphs/demo.yaml` the given document.
fn build_workspace(root: &Path, types: &[(&str, &str)], graph_yaml: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    for (node_type, fixture) in types {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
}

/// Run `cerulion <args>` in `root` to completion: `(status, stdout, stderr)`.
fn cerulion(root: &Path, args: &[&str]) -> (ExitStatus, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .stdin(Stdio::null())
        .output()
        .expect("spawn cerulion");
    (
        out.status,
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// Spawn `cerulion graph run demo --single-process [extra..]` with stdio to
/// files; returns the guard and the stderr path.
fn spawn_run(root: &Path, extra: &[&str]) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join("run.stderr");
    let mut args = vec!["graph", "run", "demo", "--single-process"];
    args.extend_from_slice(extra);
    let child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(&args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env("CERULION_NETWORK", "off")
        .env(
            "RUST_LOG",
            "cerulion=info,cerulion_cli_engine=info,cerulion_bagd=info",
        )
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    (ChildGuard::single_process(child), stderr_path)
}

/// Decode a recorded `Vector3` frame: a 32-byte wire header then three
/// little-endian `f64`.
fn vector3(frame: &[u8]) -> (f64, f64, f64) {
    assert_eq!(
        frame.len(),
        WireHeader::SIZE + 24,
        "a Vector3 frame is the wire header plus 24 payload bytes"
    );
    let p = &frame[WireHeader::SIZE..];
    let f = |i: usize| f64::from_le_bytes(p[i * 8..i * 8 + 8].try_into().unwrap());
    (f(0), f(1), f(2))
}

/// Run `cerulion bag play <bag> --resim all --verify`, bounded.
fn resim_verify(root: &Path, bag: &Path, stem: &str) -> (ExitStatus, String) {
    let err_path = root.join(format!("{stem}.stderr"));
    let mut child = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "play"])
            .arg(bag)
            .args(["--resim", "all", "--verify"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .stdin(Stdio::null())
            .stdout(Stdio::from(
                std::fs::File::create(root.join(format!("{stem}.stdout"))).unwrap(),
            ))
            .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
            .spawn()
            .expect("spawn cerulion bag play --resim"),
    );
    let status = child
        .wait_bounded(Duration::from_secs(180))
        .expect("`cerulion bag play --resim` never returned");
    (status, read_file(&err_path))
}

#[test]
#[serial]
fn two_instances_publish_their_own_params_and_the_recording_resims_with_the_recorded_values() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let prefix = format!("prm2e2e{}", std::process::id());
    build_workspace(
        root,
        &[("cam", PARAM_FIXTURE)],
        &two_instances_yaml(&prefix),
    );
    let left = format!("/{prefix}/left/cmd");
    let right = format!("/{prefix}/right/cmd");

    let (mut guard, stderr_path) = spawn_run(root, &["--record=recordings"]);
    let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(60))
        .unwrap_or_else(|| panic!("no bag appeared; stderr:\n{}", read_file(&stderr_path)));
    wait_for_bag_state(
        &bag,
        "frames on both instances",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(&left) >= 3 && snap.frames_on(&right) >= 3,
    );
    send_signal(guard.id(), libc::SIGINT);
    let status = guard
        .wait_bounded(Duration::from_secs(60))
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "Ctrl-C must exit 0, got {status:?}; stderr:\n{}",
        read_file(&stderr_path)
    );

    // The frames: every `left` frame is (2.5, 0.0, 8.0) ("left_cam" is eight
    // bytes), every `right` frame is (0.5, 0.0, -1.0) (disabled).
    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "{completeness:?}");
    let frames_of = |topic: &str| -> Vec<(f64, f64, f64)> {
        msgs.iter()
            .filter(|m| m.topic == topic)
            .map(|m| vector3(&m.data))
            .collect()
    };
    let left_frames = frames_of(&left);
    let right_frames = frames_of(&right);
    assert!(left_frames.len() >= 3 && right_frames.len() >= 3);
    assert!(
        left_frames.iter().all(|f| *f == (2.5, 0.0, 8.0)),
        "left must publish its own params: {left_frames:?}"
    );
    assert!(
        right_frames.iter().all(|f| *f == (0.5, 0.0, -1.0)),
        "right must publish its own params: {right_frames:?}"
    );

    // The effective graph in the recording carries the values in force: the
    // graph's values AND the defaults the graph file left alone (`right`
    // never set `frame_id`, `left` never set `enabled`), so a later resim
    // applies what ran even after a node's default changes.
    let graph_yaml = reader
        .attachment("graph.yaml")
        .expect("read attachment")
        .expect("graph.yaml attachment present");
    let text = String::from_utf8_lossy(&graph_yaml.data);
    for line in [
        "params:",
        "gain: 2.5",
        "frame_id: left_cam",
        "enabled: true",
        "gain: 0.5",
        "frame_id: base_link",
        "enabled: false",
        "offset: 0",
    ] {
        assert!(
            text.contains(line),
            "the recorded graph must carry the resolved params ({line}):\n{text}"
        );
    }

    // The byte-exact re-execution verdict, and again after the workspace
    // file changed: the resim applies the RECORDED values.
    let (status, err) = resim_verify(root, &bag, "resim1");
    assert_eq!(status.code(), Some(0), "resim --verify must pass:\n{err}");
    std::fs::write(
        root.join("graphs/demo.yaml"),
        two_instances_yaml(&prefix).replace("gain: 2.5", "gain: 9.0"),
    )
    .unwrap();
    let (status, err) = resim_verify(root, &bag, "resim2");
    assert_eq!(
        status.code(),
        Some(0),
        "resim must use the recorded values, not the changed workspace file:\n{err}"
    );
    guard.finish().assert_clean();
}

/// Each refusal class, through BOTH verbs. `graph validate` reports it as a
/// failing `params 'cam0'` line and exits non-zero; `graph run --no-validate`
/// reaches the graph build, which refuses with the same sentence before any
/// node runs.
#[test]
#[serial]
fn graph_validate_and_graph_run_refuse_each_params_class_naming_node_and_param() {
    let cases: [(&str, &str); 3] = [
        (
            "    params:\n      gian: 2.5\n",
            "node 'cam0': unknown param 'gian' (did you mean 'gain'?); declared params: gain, offset, frame_id, enabled",
        ),
        (
            "    params:\n      gain: 2\n",
            "node 'cam0': param 'gain' expects f64, got integer 2; a float param takes a float value, for example 2.0",
        ),
        (
            "    params:\n      gain: 11.0\n",
            "node 'cam0': param 'gain' = 11.0 is outside [0.0, 10.0]",
        ),
    ];
    for (i, (block, expected)) in cases.iter().enumerate() {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let prefix = format!("prmref{}{}", std::process::id(), i);
        build_workspace(
            root,
            &[("cam", PARAM_FIXTURE)],
            &one_node_yaml(&prefix, "cam", block),
        );

        let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
        assert!(
            !status.success(),
            "graph validate must fail for {block:?}; stdout:\n{out}\nstderr:\n{err}"
        );
        assert!(
            out.contains("params 'cam0'") && out.contains(expected),
            "the report must carry the params line for {block:?}:\n{out}\n{err}"
        );

        let (mut guard, stderr_path) = spawn_run(root, &["--no-validate"]);
        let status = guard
            .wait_bounded(Duration::from_secs(90))
            .expect("graph run must exit on its own after refusing the params");
        let run_err = read_file(&stderr_path);
        assert!(
            !status.success(),
            "graph run must refuse {block:?} at load:\n{run_err}"
        );
        assert!(
            run_err.contains(expected),
            "graph run must name node, param and fix for {block:?}:\n{run_err}"
        );
        guard.finish().assert_clean();
    }
}

/// Native `params:` on a `ros2:` entry is refused by BOTH verbs, and by
/// `graph run --no-validate` too: the run lifts ros2 entries out of the graph
/// before it resolves native params, so a key that slipped past would reach
/// no process. The refusal comes before anything is spawned, so no ROS 2
/// install is needed.
#[test]
#[serial]
fn native_params_on_a_ros2_entry_are_refused_even_under_no_validate() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let prefix = format!("prmros{}", std::process::id());
    let yaml = format!(
        "{}\x20 - id: talker\n\
         \x20   ros2:\n\
         \x20     package: demo_nodes_cpp\n\
         \x20     executable: talker\n\
         \x20   params:\n\
         \x20     rate: 2\n",
        one_node_yaml(&prefix, "cam", "")
    );
    build_workspace(root, &[("cam", PARAM_FIXTURE)], &yaml);
    let expected = "node 'talker': `params:` is not accepted on a `ros2:` entry; put ROS 2 \
                    parameters under `ros2.params` on that entry";

    let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
    assert!(!status.success(), "{out}\n{err}");
    assert!(
        out.contains(expected) || err.contains(expected),
        "graph validate must name the entry and the fix:\n{out}\n{err}"
    );

    let (mut guard, stderr_path) = spawn_run(root, &["--no-validate"]);
    let status = guard
        .wait_bounded(Duration::from_secs(90))
        .expect("graph run must exit on its own after refusing the entry");
    let run_err = read_file(&stderr_path);
    assert!(
        !status.success(),
        "graph run must refuse the entry:\n{run_err}"
    );
    assert!(
        run_err.contains(expected),
        "graph run --no-validate must name the entry and the fix:\n{run_err}"
    );
    guard.finish().assert_clean();
}

#[test]
fn params_on_a_type_that_declares_none_fails_validate_by_name() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(
        root,
        &[("ticker", PERIOD_FIXTURE)],
        &one_node_yaml("prmnone", "ticker", "    params:\n      gain: 1.0\n"),
    );
    let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
    assert!(!status.success(), "{out}\n{err}");
    assert!(
        out.contains(
            "node 'cam0': unknown param 'gain'; this node type declares no params, so `params:` is not accepted on it"
        ),
        "{out}\n{err}"
    );
}

#[test]
fn graph_validate_passes_a_well_formed_params_block_and_lists_the_values_in_force() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(
        root,
        &[("cam", PARAM_FIXTURE)],
        &two_instances_yaml("prmok"),
    );
    let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
    assert!(status.success(), "{out}\n{err}");
    assert!(
        out.contains("gain=2.5, offset=0, frame_id=\"left_cam\", enabled=true"),
        "{out}"
    );
    assert!(
        out.contains("gain=0.5, offset=0, frame_id=\"base_link\", enabled=false"),
        "{out}"
    );
}

#[test]
fn node_info_lists_the_params_table_from_the_built_library() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    build_workspace(
        root,
        &[("cam", PARAM_FIXTURE)],
        &two_instances_yaml("prminfo"),
    );
    let (status, out, err) = cerulion(root, &["node", "info", "cam"]);
    assert!(status.success(), "{out}\n{err}");
    assert!(out.contains("Params:"), "{out}");
    let gain_row = out
        .lines()
        .find(|l| l.trim_start().starts_with("gain "))
        .unwrap_or_else(|| panic!("no gain row in:\n{out}"));
    for token in ["f64", "1.0", "0.0", "10.0", "multiplier written to cmd.x"] {
        assert!(gain_row.contains(token), "row {gain_row:?} lacks {token}");
    }
    let frame_row = out
        .lines()
        .find(|l| l.trim_start().starts_with("frame_id "))
        .unwrap_or_else(|| panic!("no frame_id row in:\n{out}"));
    assert!(
        frame_row.contains("String") && frame_row.contains("\"base_link\""),
        "{frame_row}"
    );
}
