// SPDX-License-Identifier: AGPL-3.0-only
//! Launch arguments on the workspace seam, over the REAL `cerulion` binary: a
//! graph declares `args:` and references them in its `prefix`, an output's
//! `topic`, an input's `source` and a node's `params:`; `graph run` with no
//! override runs the declared defaults and with `name:=value` overrides runs
//! the operator's setup, and in both cases the delivered topic names and the
//! published values match hand-written oracles; the recorded `graph.yaml` is
//! the resolved graph (no `args:`, no reference, the overrides applied), and
//! `bag play --resim all --verify` re-executes it byte for byte; every load
//! error exits non-zero naming the argument and the YAML location.
//!
//! The nodes are two prebuilt fixtures: `test_node_macro_param_cdylib` as
//! `cam` (a period node whose `cmd` is `x = gain`, `y = offset`, `z =
//! frame_id.len()` or `-1.0` when disabled) and
//! `test_node_macro_data_trigger_cdylib` as `relay` (fires on `trigger_in`
//! and mirrors its `x` into its own `cmd`). Every expected frame below is a
//! HAND ORACLE from those two bodies, never a second run of the same code.
//!
//! Prerequisite: `cargo build -p test_node_macro_param_cdylib -p
//! test_node_macro_data_trigger_cdylib` (the harness panics with that
//! instruction when a fixture is missing). `#[serial]` arms run on the default
//! iceoryx2 namespace; run this binary alone with `-- --test-threads=1`.

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
const RELAY_FIXTURE: &str = "test_node_macro_data_trigger_cdylib";

/// The graph under test. `ns` names the prefix, `side` the camera side (in
/// the topic, the source and the frame id), `gain` and `enabled` feed two
/// typed params; `enabled` uses the long declaration form.
fn args_graph(ns: &str) -> String {
    format!(
        "args:\n\
         \x20 ns: {ns}\n\
         \x20 side: left\n\
         \x20 gain: 2.5\n\
         \x20 enabled:\n\
         \x20   default: true\n\
         \x20   description: false writes -1.0 to cmd.z\n\
         prefix: ${{ns}}\n\
         nodes:\n\
         \x20 - id: cam\n\
         \x20   type: cam\n\
         \x20   params:\n\
         \x20     gain: ${{gain}}\n\
         \x20     frame_id: ${{side}}_cam\n\
         \x20     enabled: ${{enabled}}\n\
         \x20   outputs:\n\
         \x20     - name: cmd\n\
         \x20       schema: geometry_msgs/Vector3\n\
         \x20       topic: /${{ns}}/${{side}}/cmd\n\
         \x20 - id: relay\n\
         \x20   type: relay\n\
         \x20   inputs:\n\
         \x20     - name: trigger_in\n\
         \x20       source: /${{ns}}/${{side}}/cmd\n\
         \x20   outputs:\n\
         \x20     - name: cmd\n\
         \x20       schema: geometry_msgs/Vector3\n"
    )
}

/// Stage a workspace: `nodes/<type>/src/lib.rs` is the fixture source (for the
/// metadata walkers), `target/debug/lib<type>.*` the prebuilt fixture cdylib
/// (no in-test cargo build), `graphs/demo.yaml` the given document.
fn build_workspace(root: &Path, graph_yaml: &str) {
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
    for (node_type, fixture) in [("cam", PARAM_FIXTURE), ("relay", RELAY_FIXTURE)] {
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
/// files named by `tag`; returns the guard and the stderr path.
fn spawn_run(root: &Path, tag: &str, extra: &[&str]) -> (ChildGuard, PathBuf) {
    let stderr_path = root.join(format!("{tag}.stderr"));
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
            std::fs::File::create(root.join(format!("{tag}.stdout"))).unwrap(),
        ))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()))
        .spawn()
        .expect("spawn cerulion graph run");
    (ChildGuard::single_process(child), stderr_path)
}

/// Decode a recorded `Vector3` frame: a wire header then three
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

/// One recorded run: spawn with `extra`, wait for frames on both topics,
/// Ctrl-C, and return the bag path with the frames of each topic.
#[allow(clippy::type_complexity)]
fn record_run(
    root: &Path,
    tag: &str,
    extra: &[&str],
    cam_topic: &str,
    relay_topic: &str,
) -> (PathBuf, Vec<(f64, f64, f64)>, Vec<(f64, f64, f64)>, String) {
    let recordings = format!("{tag}-recordings");
    let record_flag = format!("--record={recordings}");
    let mut run_args = vec![record_flag.as_str()];
    run_args.extend_from_slice(extra);
    let (mut guard, stderr_path) = spawn_run(root, tag, &run_args);
    let bag = wait_for_bag(&root.join(&recordings), Duration::from_secs(60))
        .unwrap_or_else(|| panic!("no bag appeared; stderr:\n{}", read_file(&stderr_path)));
    wait_for_bag_state(
        &bag,
        "frames on the camera and the relay",
        RECORDED_WINDOW_TIMEOUT,
        |snap| snap.frames_on(cam_topic) >= 3 && snap.frames_on(relay_topic) >= 3,
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
    guard.finish().assert_clean();

    let reader = BagReader::open(&bag).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "{completeness:?}");
    let frames_of = |topic: &str| -> Vec<(f64, f64, f64)> {
        msgs.iter()
            .filter(|m| m.topic == topic)
            .map(|m| vector3(&m.data))
            .collect()
    };
    let graph_yaml = reader
        .attachment("graph.yaml")
        .expect("read attachment")
        .expect("graph.yaml attachment present");
    (
        bag,
        frames_of(cam_topic),
        frames_of(relay_topic),
        String::from_utf8_lossy(&graph_yaml.data).into_owned(),
    )
}

#[test]
#[serial]
fn defaults_then_overrides_name_the_topics_feed_the_params_and_resim_from_the_resolved_graph() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let ns = format!("largs{}", std::process::id());
    build_workspace(root, &args_graph(&ns));
    let relay_topic = format!("/{ns}/relay/cmd");

    // `graph validate` resolves the declared defaults and lists the param
    // values in force.
    let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
    assert!(status.success(), "{out}\n{err}");
    assert!(
        out.contains("gain=2.5, offset=0, frame_id=\"left_cam\", enabled=true"),
        "{out}"
    );

    // Defaults: the camera publishes on `/<ns>/left/cmd` with (2.5, 0.0, 8.0)
    // ("left_cam" is eight bytes); the relay, wired by the same reference in
    // its `source`, mirrors x = 2.5.
    let left_topic = format!("/{ns}/left/cmd");
    let (bag, cam, relay, graph_yaml) =
        record_run(root, "defaults", &[], &left_topic, &relay_topic);
    assert!(
        cam.len() >= 3 && cam.iter().all(|f| *f == (2.5, 0.0, 8.0)),
        "{cam:?}"
    );
    assert!(
        relay.len() >= 3 && relay.iter().all(|f| f.0 == 2.5),
        "{relay:?}"
    );
    for line in [
        &format!("prefix: {ns}"),
        &format!("topic: /{ns}/left/cmd"),
        &format!("source: /{ns}/left/cmd"),
        "gain: 2.5",
        "frame_id: left_cam",
        "enabled: true",
    ] {
        assert!(
            graph_yaml.contains(line),
            "the recorded graph must be the resolved graph ({line}):\n{graph_yaml}"
        );
    }
    assert!(
        !graph_yaml.contains("args:") && !graph_yaml.contains("${"),
        "the recorded graph carries no `args:` and no reference:\n{graph_yaml}"
    );
    let (status, err) = resim_verify(root, &bag, "resim-defaults");
    assert_eq!(status.code(), Some(0), "resim --verify must pass:\n{err}");

    // Overrides: `side:=right gain:=0.5 enabled:=false` moves both topics to
    // `/<ns>/right/cmd` and the camera publishes (0.5, 0.0, -1.0).
    let right_topic = format!("/{ns}/right/cmd");
    let (bag, cam, relay, graph_yaml) = record_run(
        root,
        "overrides",
        &["side:=right", "gain:=0.5", "enabled:=false"],
        &right_topic,
        &relay_topic,
    );
    assert!(
        cam.len() >= 3 && cam.iter().all(|f| *f == (0.5, 0.0, -1.0)),
        "{cam:?}"
    );
    assert!(
        relay.len() >= 3 && relay.iter().all(|f| f.0 == 0.5),
        "{relay:?}"
    );
    for line in [
        &format!("topic: /{ns}/right/cmd"),
        &format!("source: /{ns}/right/cmd"),
        "gain: 0.5",
        "frame_id: right_cam",
        "enabled: false",
    ] {
        assert!(
            graph_yaml.contains(line),
            "the recorded graph must carry the overrides ({line}):\n{graph_yaml}"
        );
    }
    assert!(
        !graph_yaml.contains("left") && !graph_yaml.contains("args:"),
        "{graph_yaml}"
    );
    // The resim applies the RECORDED graph: the workspace file still declares
    // the defaults, and the verdict is still byte-exact.
    let (status, err) = resim_verify(root, &bag, "resim-overrides");
    assert_eq!(status.code(), Some(0), "resim --verify must pass:\n{err}");
}

/// Every load error, through the verb that can produce it: `graph run` for
/// an override problem, `graph validate` AND `graph run --no-validate` for a
/// problem in the file. Each exits non-zero before any node runs and names
/// the argument and the location.
#[test]
#[serial]
fn each_load_error_exits_non_zero_naming_the_argument_and_the_location() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    let ns = format!("largserr{}", std::process::id());
    let graph = args_graph(&ns);

    let refused_run = |graph_yaml: &str, tag: &str, extra: &[&str], expected: &str| {
        std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
        let mut run_args = vec!["--no-validate"];
        run_args.extend_from_slice(extra);
        let (mut guard, stderr_path) = spawn_run(root, tag, &run_args);
        let status = guard
            .wait_bounded(Duration::from_secs(90))
            .expect("graph run must exit on its own after refusing the graph");
        let run_err = read_file(&stderr_path);
        assert!(
            !status.success(),
            "{tag}: graph run must refuse:\n{run_err}"
        );
        assert!(
            run_err.contains(expected),
            "{tag}: graph run must say {expected:?}:\n{run_err}"
        );
        guard.finish().assert_clean();
    };
    let refused_validate = |graph_yaml: &str, expected: &str| {
        std::fs::write(root.join("graphs/demo.yaml"), graph_yaml).unwrap();
        let (status, out, err) = cerulion(root, &["graph", "validate", "demo"]);
        assert!(!status.success(), "graph validate must fail:\n{out}\n{err}");
        assert!(
            out.contains(expected) || err.contains(expected),
            "graph validate must say {expected:?}:\n{out}\n{err}"
        );
    };

    build_workspace(root, &graph);

    // An override for an argument the graph does not declare.
    refused_run(
        &graph,
        "unknown-override",
        &["sdie:=right"],
        "launch argument 'sdie' is not declared by this graph (did you mean 'side'?); declared \
         args: ns, side, gain, enabled",
    );

    // A reference to an undeclared argument, located to the key.
    let undeclared = graph.replace("topic: /${ns}/${side}/cmd", "topic: /${nz}/${side}/cmd");
    let expected = "node 'cam' output 'cmd' `topic` references launch argument 'nz', which this \
                    graph does not declare (did you mean 'ns'?)";
    refused_validate(&undeclared, expected);
    refused_run(&undeclared, "undeclared-reference", &[], expected);

    // An unterminated reference.
    let unterminated = graph.replace("prefix: ${ns}", "prefix: ${ns");
    let expected = "`prefix` has an unterminated substitution in \"${ns\"";
    refused_validate(&unterminated, expected);
    refused_run(&unterminated, "unterminated", &[], expected);

    // A value of the wrong type for the param it feeds, from an override and
    // from a declared default.
    refused_run(
        &graph,
        "wrong-type-override",
        &["gain:=fast"],
        "node 'cam': param 'gain' expects f64, got string \"fast\"",
    );
    refused_validate(
        &graph.replace("gain: 2.5", "gain: fast"),
        "node 'cam': param 'gain' expects f64, got string \"fast\"",
    );

    // A word that is not `name:=value` is a usage error at parse (exit 2).
    std::fs::write(root.join("graphs/demo.yaml"), &graph).unwrap();
    let (status, _, err) = cerulion(root, &["graph", "run", "demo", "side=right"]);
    assert_eq!(status.code(), Some(2), "{err}");
    assert!(
        err.contains("launch argument override 'side=right' is malformed"),
        "{err}"
    );
}
