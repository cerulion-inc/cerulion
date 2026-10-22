// SPDX-License-Identifier: AGPL-3.0-only
//! The launch-argument graph and workspace shared by the single-process
//! (`graph_launch_args_e2e_test`) and multi-process
//! (`graph_launch_args_mp_e2e_test`) binaries. Each binary uses all of it.

use std::path::Path;
use std::process::{Command, ExitStatus, Stdio};

use crate::mp_support::{dylib_file, fixture_cdylib};

const PARAM_FIXTURE: &str = "test_node_macro_param_cdylib";
const RELAY_FIXTURE: &str = "test_node_macro_data_trigger_cdylib";

/// The graph under test. `ns` names the prefix, `side` the camera side (in
/// the topic, the source and the frame id), `cam_gain` and `enabled` feed
/// two typed params (`cam_gain` under a name that differs from the param it
/// feeds, so a refusal has to name the argument itself); `enabled` uses the
/// long declaration form.
pub fn args_graph(ns: &str) -> String {
    format!(
        "args:\n\
         \x20 ns: {ns}\n\
         \x20 side: left\n\
         \x20 cam_gain: 2.5\n\
         \x20 enabled:\n\
         \x20   default: true\n\
         \x20   description: false writes -1.0 to cmd.z\n\
         prefix: ${{ns}}\n\
         nodes:\n\
         \x20 - id: cam\n\
         \x20   type: cam\n\
         \x20   params:\n\
         \x20     gain: ${{cam_gain}}\n\
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
pub fn build_workspace(root: &Path, graph_yaml: &str) {
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
pub fn cerulion(root: &Path, args: &[&str]) -> (ExitStatus, String, String) {
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
