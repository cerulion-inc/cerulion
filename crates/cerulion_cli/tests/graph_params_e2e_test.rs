// SPDX-License-Identifier: AGPL-3.0-only
//! `params:` on the workspace seam, over the REAL `cerulion` binary: one node
//! type staged twice with different values publishes observably different
//! frames per instance; the recording carries the values in force, defaults
//! included, and `bag play --resim all --verify` re-executes it byte for
//! byte; each of the
//! three refusal classes exits non-zero from `graph validate` AND from
//! `graph run` naming the node and the param; `node info` lists the declared
//! params from the built library; a capture that resumes mid-run is refused
//! when the node was rebuilt with a param the recording names no value for,
//! and resumes on that param's value with `--allow-param-change`.
//!
//! The node is the prebuilt `test_node_macro_param_cdylib` fixture (a period
//! node whose `cmd` is a function of its four params: `x = gain`,
//! `y = offset`, `z = frame_id.len()` or `-1.0` when disabled; defaults
//! `1.0`, `0`, `"base_link"`, `true`). The resume arm rebuilds it as
//! `test_node_macro_param_added_cdylib`, the same node with one more param,
//! `scale`, default `3.0` (`x = scale`, `y = gain + offset`). Every expected
//! frame below is a HAND ORACLE from those bodies, never a second run of the
//! same code.
//!
//! Prerequisite: `cargo build -p test_node_macro_param_cdylib -p
//! test_node_macro_param_added_cdylib -p test_node_macro_period_cdylib`
//! (the harness panics with that instruction when a fixture is missing).
//! `#[serial]` arms run on the default iceoryx2 namespace; run this binary
//! alone with `-- --test-threads=1`.
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
    dylib_file, first_rank0_boundary, fixture_cdylib, mcaps, read_file, signal_live_process,
    wait_for_bag, wait_for_bag_state, ChildGuard, RECORDED_WINDOW_TIMEOUT,
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
    for (node_type, fixture) in types {
        stage_node(root, node_type, fixture);
    }
    std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
}

/// Stage `fixture` as the build of node type `node_type`: its source as
/// `nodes/<type>/src/lib.rs`, its prebuilt cdylib as `target/debug/lib<type>.*`.
/// Staging over an earlier build is a REBUILD: the old library is removed
/// first, so the new one is a new file rather than bytes written over a
/// library a finished process had mapped.
fn stage_node(root: &Path, node_type: &str, fixture: &str) {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
    std::fs::copy(
        fixtures.join(fixture).join("src/lib.rs"),
        root.join(format!("nodes/{node_type}/src/lib.rs")),
    )
    .expect("copy fixture src");
    let lib = root.join("target/debug").join(dylib_file(node_type));
    if lib.exists() {
        std::fs::remove_file(&lib).expect("remove the earlier build");
    }
    std::fs::copy(fixture_cdylib(fixture), &lib).expect("copy fixture cdylib");
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

/// Spawn `cerulion graph run demo --single-process [extra..]` with `env` set
/// and stdio to files; returns the guard and the stderr path.
fn spawn_run(root: &Path, extra: &[&str], env: &[(&str, &str)]) -> (ChildGuard, PathBuf) {
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
        .envs(env.iter().copied())
        .stdin(Stdio::null())
        .stdout(Stdio::from(
            std::fs::File::create(root.join("run.stdout")).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    (ChildGuard::single_process(child), stderr_path)
}

/// Decode a recorded `Vector3` frame: a 40-byte wire header then three
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

/// Run `cerulion bag play <bag> --resim all [extra..]`, bounded.
fn resim(root: &Path, bag: &Path, extra: &[&str], stem: &str) -> (ExitStatus, String) {
    let err_path = root.join(format!("{stem}.stderr"));
    let mut child = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "play"])
            .arg(bag)
            .args(["--resim", "all"])
            .args(extra)
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

    let (mut guard, stderr_path) = spawn_run(root, &["--record=recordings"], &[]);
    let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(60))
        .unwrap_or_else(|| panic!("no bag appeared; stderr:\n{}", read_file(&stderr_path)));
    wait_for_bag_state(
        &bag,
        "frames on both instances",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(&left) >= 3 && snap.frames_on(&right) >= 3,
    );
    signal_live_process(guard.id(), libc::SIGINT);
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
    let (status, err) = resim(root, &bag, &["--verify"], "resim1");
    assert_eq!(status.code(), Some(0), "resim --verify must pass:\n{err}");
    std::fs::write(
        root.join("graphs/demo.yaml"),
        two_instances_yaml(&prefix).replace("gain: 2.5", "gain: 9.0"),
    )
    .unwrap();
    let (status, err) = resim(root, &bag, &["--verify"], "resim2");
    assert_eq!(
        status.code(),
        Some(0),
        "resim must use the recorded values, not the changed workspace file:\n{err}"
    );
    guard.finish().assert_clean();
}

/// [`PARAM_FIXTURE`] rebuilt with one more param, `scale`, default `3.0`:
/// `cmd = (scale, gain + offset, frame_id.len())`.
const ADDED_PARAM_FIXTURE: &str = "test_node_macro_param_added_cdylib";

/// The resume arm's Flashback window and anchor cadence: short, so a capture
/// taken a few seconds into the run begins past step 0 and carries an anchor
/// at or before its first step.
const RESUME_WINDOW_MS: &str = "1000";
const RESUME_CADENCE_MS: &str = "250";

/// Frames the resume arm waits for before it takes its capture: three seconds
/// of the fixture's 20 ms period, three times [`RESUME_WINDOW_MS`]. A
/// CONDITION rather than a sleep, so a loaded machine delays the capture
/// rather than taking it too early.
const RESUME_MID_RUN_FRAMES: usize = 150;

/// How many run and capture attempts the resume arm makes to obtain a capture
/// that begins mid-run. Whether a capture's first step is past step 0 is a
/// race with the window's head trim, not a property of the rule under test,
/// so it is retried, never weakened.
const RESUME_ATTEMPTS: usize = 3;

/// Every `Vector3` frame on `topic` in a finalized `bag`, in log order, with
/// the `timestamp_ns` its wire header carries.
fn stamped_vector3_frames(bag: &Path, topic: &str) -> Vec<(u64, (f64, f64, f64))> {
    let reader = BagReader::open(bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "{completeness:?}");
    msgs.iter()
        .filter(|m| m.topic == topic)
        .map(|m| {
            let header = WireHeader::read_from_buf(&m.data).expect("a frame starts with a header");
            (header.timestamp_ns, vector3(&m.data))
        })
        .collect()
}

/// The param resume rule over the real binary and the cdylib FFI: the twin of
/// the in-process
/// `a_resume_refuses_a_param_value_the_recording_does_not_hold_unless_allowed`
/// in `cerulion_cli_engine/tests/replay_engine_test.rs`.
///
/// The recording runs `cam0` at `gain: 2.0`, and a `cerulion flashback`
/// capture of it begins mid-run, so its resim restores `cam0` from an anchor.
/// A resim applies the RECORDED params, so no edit to the graph file can make
/// a resumed `cam0` run `gain` on anything but 2.0. What the rule judges is a
/// value in force that the recording does not hold, so the node is rebuilt
/// with one more param, `scale`, default 3.0, which the recording names no
/// value for. The recorded `gain` still applies to the rebuilt node.
///
/// 1. The unchanged build resumes without the opt-in, on the recorded 2.0, so
///    the rule does not refuse every resume.
/// 2. The rebuilt node is refused with exit 2 and the one line for `scale`,
///    before it writes a `--record-out` bag. The build declares every name the
///    recording holds, so the refusal is the check the resume makes after the
///    build, not the one for a param the build no longer declares. That check
///    judges only a node the anchor carries a state blob for, so the refusal
///    also shows the anchor carries one for `cam0`, a node whose fields are
///    all params and one output port.
/// 3. With `--allow-param-change` the rebuilt node resumes: its first resumed
///    frame carries `scale` 3.0 beside the recorded `gain` 2.0.
///
/// Each resumed run begins at the capture's first recorded step: its first
/// frame carries that step's clock target, read off the capture's trace, as
/// its wire stamp. A run from step 0 or from a later step cannot.
///
/// Hand oracles from the two fixture bodies: `(gain, offset, frame_id.len())`
/// is `(2.0, 0.0, 9.0)`, and `(scale, gain + offset, frame_id.len())` is
/// `(3.0, 2.0, 9.0)`.
#[test]
#[serial]
fn a_capture_resumed_on_a_build_that_added_a_param_is_refused_unless_allowed() {
    let mut last_retry = String::new();
    for attempt in 1..=RESUME_ATTEMPTS {
        let tmp = tempfile::tempdir().unwrap();
        let root = tmp.path();
        let prefix = format!("prmres{}{attempt}", std::process::id());
        build_workspace(
            root,
            &[("cam", PARAM_FIXTURE)],
            &one_node_yaml(&prefix, "cam", "    params:\n      gain: 2.0\n"),
        );
        let topic = format!("/{prefix}/cam0/cmd");
        let flashbacks = root.join("flashbacks");
        std::fs::create_dir_all(&flashbacks).unwrap();
        let flashbacks_dir = flashbacks.to_str().expect("a UTF-8 temp path");

        // RECORD, until the run is well past its first steps.
        let (mut run, run_err) = spawn_run(
            root,
            &["--record=recordings"],
            &[
                ("CERULION_FLASHBACK_DIR", flashbacks_dir),
                ("CERULION_FLASHBACK_WINDOW_MS", RESUME_WINDOW_MS),
                ("CERULION_FLASHBACK_CADENCE_MS", RESUME_CADENCE_MS),
            ],
        );
        let bag = wait_for_bag(&root.join("recordings"), Duration::from_secs(60))
            .unwrap_or_else(|| panic!("no bag appeared; stderr:\n{}", read_file(&run_err)));
        wait_for_bag_state(
            &bag,
            "the run past its first steps",
            RECORDED_WINDOW_TIMEOUT,
            |snap| snap.frames_on(&topic) >= RESUME_MID_RUN_FRAMES,
        );

        // CAPTURE, through the operator's own verb, then stop the run.
        let mut flash = ChildGuard::single_process(
            Command::new(env!("CARGO_BIN_EXE_cerulion"))
                .args(["flashback", "--note", "param resume probe"])
                .current_dir(root)
                .env_remove("CARGO_TARGET_DIR")
                .env("CERULION_NETWORK", "off")
                .env("CERULION_FLASHBACK_DIR", flashbacks_dir)
                .stdin(Stdio::null())
                .stdout(Stdio::from(
                    std::fs::File::create(root.join("flash.stdout")).unwrap(),
                ))
                .stderr(Stdio::from(
                    std::fs::File::create(root.join("flash.stderr")).unwrap(),
                ))
                .spawn()
                .expect("spawn cerulion flashback"),
        );
        let flash_status = flash
            .wait_bounded(Duration::from_secs(180))
            .expect("`cerulion flashback` never returned");
        assert!(
            flash_status.success(),
            "`cerulion flashback` must capture: {flash_status:?}\nstdout:\n{}\nstderr:\n{}",
            read_file(&root.join("flash.stdout")),
            read_file(&root.join("flash.stderr"))
        );
        signal_live_process(run.id(), libc::SIGINT);
        let status = run
            .wait_bounded(Duration::from_secs(60))
            .expect("graph run did not exit after SIGINT");
        assert!(
            status.success(),
            "Ctrl-C must exit 0, got {status:?}; stderr:\n{}",
            read_file(&run_err)
        );
        run.finish().assert_clean();
        let captures = mcaps(&flashbacks);
        assert_eq!(
            captures.len(),
            1,
            "exactly one capture in this run's own directory, got {captures:?}"
        );
        let capture = &captures[0];

        let boundary = first_rank0_boundary(capture)
            .expect("a capture of a `--record` run carries a rank-0 STEP_BOUNDARY");
        if boundary.step == 0 {
            last_retry = format!(
                "attempt {attempt}: the capture's first rank-0 STEP_BOUNDARY is step 0, so it \
                 begins from start and resumes from no anchor"
            );
            eprintln!("{last_retry}");
            continue;
        }
        // The clock the first resumed step runs at, and so the stamp of the
        // first frame a resumed `cam0` publishes.
        let first_step_stamp = boundary.fire_time_ns;
        let captured = stamped_vector3_frames(capture, &topic);
        assert!(!captured.is_empty(), "the capture holds no `{topic}` frame");
        assert!(
            captured.iter().all(|(_, f)| *f == (2.0, 0.0, 9.0)),
            "the anchor's run published gain 2.0: {captured:?}"
        );

        // 1. The unchanged build resumes on the recorded value, no opt-in.
        let unchanged = root.join("unchanged.mcap");
        let (status, err) = resim(
            root,
            capture,
            &["--record-out", unchanged.to_str().unwrap()],
            "unchanged",
        );
        assert_eq!(
            status.code(),
            Some(0),
            "the build the recording ran resumes without the opt-in:\n{err}"
        );
        let frames = stamped_vector3_frames(&unchanged, &topic);
        assert_eq!(
            frames.first().map(|(stamp, _)| *stamp),
            Some(first_step_stamp),
            "the unchanged build resumes at the capture's first step {}:\n{err}",
            boundary.step
        );
        assert!(
            frames.iter().all(|(_, f)| *f == (2.0, 0.0, 9.0)),
            "the resumed node runs on the recorded gain 2.0: {frames:?}"
        );

        // 2. The rebuilt node is refused, one line for `scale`, before any output.
        stage_node(root, "cam", ADDED_PARAM_FIXTURE);
        let refused = root.join("refused.mcap");
        let (status, err) = resim(
            root,
            capture,
            &["--record-out", refused.to_str().unwrap()],
            "refused",
        );
        assert_eq!(
            status.code(),
            Some(2),
            "a param the recording holds no value for refuses the resume:\n{err}"
        );
        let expected = "this recording resumes from a mid-run anchor, and 1 param value(s) in \
                        force differ from the ones the recording names. A param is not node \
                        state: the resumed nodes run on the values in force, not on the \
                        recorded ones.\n  \
                        node 'cam0' param 'scale': recorded no value, current 3.0\n\
                        Fix: resume with a build whose params match the recording, or pass \
                        `--allow-param-change` to resume on the current values.";
        assert!(err.contains(expected), "the refusal text:\n{err}");
        assert!(
            !refused.exists(),
            "a refused resume writes no `--record-out` bag: {}",
            refused.display()
        );

        // 3. The opt-in resumes the rebuilt node on the value in force.
        let allowed = root.join("allowed.mcap");
        let (status, err) = resim(
            root,
            capture,
            &[
                "--allow-param-change",
                "--record-out",
                allowed.to_str().unwrap(),
            ],
            "allowed",
        );
        assert_eq!(
            status.code(),
            Some(0),
            "`--allow-param-change` resumes on the current values:\n{err}"
        );
        let frames = stamped_vector3_frames(&allowed, &topic);
        assert_eq!(
            frames.first(),
            Some(&(first_step_stamp, (3.0, 2.0, 9.0))),
            "the first resumed tick, at the capture's first step {}, reads scale 3.0 and the \
             recorded gain 2.0:\n{err}",
            boundary.step
        );
        assert!(
            frames.iter().all(|(_, f)| *f == (3.0, 2.0, 9.0)),
            "every resumed tick reads scale 3.0 and gain 2.0: {frames:?}"
        );
        return;
    }
    panic!("no capture began mid-run in {RESUME_ATTEMPTS} attempts; last: {last_retry}");
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

        let (mut guard, stderr_path) = spawn_run(root, &["--no-validate"], &[]);
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

    let (mut guard, stderr_path) = spawn_run(root, &["--no-validate"], &[]);
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
