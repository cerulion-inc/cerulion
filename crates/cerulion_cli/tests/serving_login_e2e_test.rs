// SPDX-License-Identifier: AGPL-3.0-only
//! Real CLI serving boundaries. Deliberately invalid node/config sentinels stop
//! allowed arms before any transport starts; no ambient discovery is performed.
use std::path::Path;
use std::process::{Command, Output, Stdio};

mod serving_login_support;

const REFUSAL: &str = "serving the network requires a prior login; run `cerulion login`";
const GRAPH: &str = "name: serving\nprefix: serving\nnodes:\n- id: absent\n  type: absent\n  inputs: []\n  outputs:\n  - name: out\n    schema: std_msgs/String\n";
const PRODUCER_FREE: &str = "name: serving\nprefix: serving\nnodes:\n- id: absent\n  type: absent\n  inputs: []\n  outputs: []\n";

fn workspace(root: &Path) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::write(root.join("Cargo.toml"), "[workspace]\nmembers = []\n").unwrap();
    std::fs::write(root.join("graphs/serving.yaml"), GRAPH).unwrap();
}

fn run(root: &Path, home: &Path, args: &[&str], network_off: bool) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(args)
        .current_dir(root)
        .env("CERULION_HOME", home)
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env("NO_COLOR", "1")
        // The command-wide login gate is off, so each arm reaches the serving
        // gate, which `CERULION_LOGIN_GATE` does not switch off.
        .env("CERULION_LOGIN_GATE", "off")
        .env_remove("CERULION_NETWORK")
        .env_remove("CARGO_TARGET_DIR")
        .stdin(Stdio::null());
    if network_off {
        cmd.env("CERULION_NETWORK", "off");
    }
    cmd.output().unwrap()
}

fn stderr(out: &Output) -> String {
    assert!(
        !out.status.success(),
        "every arm stops at a refusal or explicit sentinel"
    );
    String::from_utf8_lossy(&out.stderr).into_owned()
}

#[test]
fn graph_serving_refuses_explicit_egress_and_degrades_permissive_without_login() {
    let root = tempfile::tempdir().unwrap();
    workspace(root.path());
    let home = root.path().join("never-logged-in");
    let args = [
        "graph",
        "run",
        "serving",
        "--single-process",
        "--no-validate",
    ];

    // Explicit `network: egress:` stays refused before touching the home or run dir.
    let explicit_yaml = format!(
        "{GRAPH}network:\n  mode: peer\n  listen: [tcp/127.0.0.1:0]\n  egress: [/serving/absent/out]\n"
    );
    std::fs::write(root.path().join("graphs/serving.yaml"), &explicit_yaml).unwrap();
    let out = run(root.path(), &home, &args, false);
    assert!(stderr(&out).contains(REFUSAL), "{}", stderr(&out));
    assert!(
        !home.exists(),
        "the refusal must not initiate a login transaction"
    );
    assert_eq!(
        std::fs::read_to_string(root.path().join("graphs/serving.yaml")).unwrap(),
        explicit_yaml
    );
    assert!(!root.path().join(".cerulion/runs").exists());

    // Blockless permissive default degrades to local-only with one loud warning
    // and reaches the missing-cdylib sentinel without writing auth.json.
    std::fs::write(root.path().join("graphs/serving.yaml"), GRAPH).unwrap();
    let degraded = stderr(&run(root.path(), &home, &args, false));
    assert!(!degraded.contains(REFUSAL), "{degraded}");
    assert!(
        degraded.contains("running local-only")
            && degraded.contains("no prior login or robot pairing")
            && degraded.contains("cerulion login"),
        "permissive degradation must log one loud warning; stderr={degraded}"
    );
    assert!(
        degraded.contains("cdylib for node 'absent' not found"),
        "degraded run must proceed past the serving gate to node loading; stderr={degraded}"
    );
    assert!(!home.join("auth.json").exists());
    assert_eq!(
        std::fs::read_to_string(root.path().join("graphs/serving.yaml")).unwrap(),
        GRAPH
    );
}

#[test]
fn producer_free_graph_passes_the_serving_gate_without_a_login_transaction() {
    let root = tempfile::tempdir().unwrap();
    workspace(root.path());
    let home = root.path().join("never-logged-in");
    for network in ["", "network:\n  mode: peer\n  listen: [tcp/127.0.0.1:0]\n"] {
        let yaml = format!("{PRODUCER_FREE}{network}");
        std::fs::write(root.path().join("graphs/serving.yaml"), &yaml).unwrap();
        let error = stderr(&run(
            root.path(),
            &home,
            &[
                "graph",
                "run",
                "serving",
                "--single-process",
                "--no-validate",
            ],
            false,
        ));
        assert!(!error.contains(REFUSAL), "{error}");
        assert!(
            error.contains("cdylib for node 'absent' not found"),
            "{error}"
        );
        let entries = std::fs::read_dir(&home)
            .into_iter()
            .flatten()
            .map(|entry| entry.unwrap().file_name())
            .collect::<Vec<_>>();
        assert_eq!(
            entries,
            vec![std::ffi::OsString::from("runs")],
            "authentication and enrollment state must stay absent; only run history is permitted"
        );
        assert!(home.join("runs").is_dir());
        assert_eq!(std::fs::read_dir(home.join("runs")).unwrap().count(), 0);
        assert!(!home.join("auth.json").exists());
        assert_eq!(
            std::fs::read_to_string(root.path().join("graphs/serving.yaml")).unwrap(),
            yaml
        );
    }
}

#[test]
fn expired_prior_login_and_inert_graph_modes_pass_the_local_gate() {
    let root = tempfile::tempdir().unwrap();
    workspace(root.path());
    let prior = serving_login_support::expired_login(root.path());
    let missing = root.path().join("never-logged-in");
    for (home, off, clock, flag_off) in [
        (prior.as_path(), false, "real", false),
        (missing.as_path(), true, "real", false),
        (missing.as_path(), false, "real", true),
        (missing.as_path(), false, "virtual", false),
        (missing.as_path(), false, "external", false),
    ] {
        let mut args = vec![
            "graph",
            "run",
            "serving",
            "--single-process",
            "--no-validate",
            "--time-source",
            clock,
        ];
        if flag_off {
            args.extend(["--network", "off"]);
        }
        let out = run(root.path(), home, &args, off);
        let error = stderr(&out);
        assert!(!error.contains(REFUSAL), "{clock}, off={off}: {error}");
        assert!(
            error.contains("cdylib for node 'absent' not found"),
            "must reach the independent missing-node sentinel: {error}"
        );
    }
}

#[test]
fn attach_serving_gate_precedes_discovery_but_dry_run_and_network_off_are_exempt() {
    let root = tempfile::tempdir().unwrap();
    workspace(root.path());
    let home = root.path().join("never-logged-in");
    let base = [
        "ros2",
        "attach",
        "--iface",
        "127.0.0.1",
        "--graph-name",
        "invalid/name",
    ];
    let out = run(root.path(), &home, &base, false);
    assert!(stderr(&out).contains(REFUSAL), "{}", stderr(&out));
    for (dry_run, network_off) in [(true, false), (false, true)] {
        let mut args = base.to_vec();
        if dry_run {
            args.push("--dry-run");
        }
        let error = stderr(&run(root.path(), &home, &args, network_off));
        assert!(!error.contains(REFUSAL), "{error}");
        assert!(
            error.contains("invalid --graph-name"),
            "must stop before DDS at the independent name sentinel: {error}"
        );
    }
    assert!(!home.exists());
    assert_eq!(
        std::fs::read_dir(root.path().join("graphs"))
            .unwrap()
            .count(),
        1
    );
}

#[test]
fn direct_hidden_gateway_cannot_bypass_login_and_expired_login_reaches_config_validation() {
    let root = tempfile::tempdir().unwrap();
    workspace(root.path());
    let handoff = root.path().join("handoff.json");
    // The malformed ix_config_json is a second, independent refusal before any
    // transport allocation, even if the login guard were accidentally removed.
    std::fs::write(&handoff, br#"{"graph_name":"serving","robot":"test-robot","plan":{"egress_policy":"AllowAll","announce":[],"ingress":[]},"network":{"mode":"Peer","connect_endpoints":[],"listen_endpoints":[],"multicast_scouting":false,"gossip_scouting":false},"posture":"Strict","ix_config_json":"{"}"#).unwrap();
    let args = [
        "graph",
        "run-gateway",
        "--handoff",
        handoff.to_str().unwrap(),
    ];
    let out = run(
        root.path(),
        &root.path().join("never-logged-in"),
        &args,
        false,
    );
    assert!(stderr(&out).contains(REFUSAL), "{}", stderr(&out));
    let prior = serving_login_support::expired_login(root.path());
    let error = stderr(&run(root.path(), &prior, &args, false));
    assert!(!error.contains(REFUSAL), "{error}");
    assert!(error.contains("not a valid iceoryx2 Config"), "{error}");
}
