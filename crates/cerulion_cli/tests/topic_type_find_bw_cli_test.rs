// SPDX-License-Identifier: AGPL-3.0-only
//! Real-binary error arms of `topic type`, `topic find`, `topic bw` and
//! `topic list -t` with no graph running. Each spawn reads a private
//! shared-memory registry and Cerulion home, so no other process's topics
//! are seen. The live arms against a running graph are in
//! `topic_introspect_cli_e2e_test.rs`.

use std::path::Path;
use std::process::{Command, Output};

fn run(root: &Path, args: &[&str]) -> Output {
    let home = root.join("home");
    cerulion_cli_engine::auth::seed_logged_in_at(&home, "topic-introspect-test").unwrap();
    let config = root.join("config");
    std::fs::create_dir_all(&config).unwrap();
    let registry = root.join("shm");
    let prefix = root.file_name().unwrap().to_str().unwrap();
    std::fs::write(
        config.join("iceoryx2.toml"),
        format!(
            "[global]\nroot-path = \"{}\"\nprefix = \"intro_{prefix}_\"\n",
            registry
                .to_string_lossy()
                .replace('\\', "\\\\")
                .replace('"', "\\\"")
        ),
    )
    .unwrap();
    Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(root)
        .env("CERULION_HOME", &home)
        .env("CERULION_NETD_SOCK", root.join("netd.sock"))
        .env("CERULION_NETD_BIN", root.join("absent-netd"))
        .env("CERULION_NETWORK", "off")
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .output()
        .expect("run the real CLI")
}

fn stderr(out: &Output) -> String {
    String::from_utf8_lossy(&out.stderr).into_owned()
}

#[test]
fn a_missing_topic_on_type_and_bw_is_an_error_that_names_it() {
    for verb in ["type", "bw"] {
        for scope in [&[][..], &["--local"]] {
            let root = tempfile::tempdir().unwrap();
            let mut args = vec!["topic", verb, "/intro-test/missing"];
            args.extend_from_slice(scope);
            let out = run(root.path(), &args);
            let error = stderr(&out);
            assert_eq!(out.status.code(), Some(1), "{args:?}: {error}");
            assert!(
                error.starts_with("Error: topic '/intro-test/missing' not found locally"),
                "{args:?}: {error}"
            );
            assert!(out.stdout.is_empty(), "{args:?}");
        }
    }
}

#[test]
fn find_refuses_a_malformed_type_name() {
    let root = tempfile::tempdir().unwrap();
    for bad in ["geometry_msgs/", "/Vector3", "geometry_msgs//Vector3"] {
        let out = run(root.path(), &["topic", "find", bad]);
        assert_eq!(out.status.code(), Some(1), "{bad}");
        assert_eq!(
            stderr(&out),
            format!(
                "Error: malformed type name '{bad}': expected <package>/<Type>, \
                 <package>/msg/<Type> or a workspace <Type>, for example geometry_msgs/Vector3\n"
            )
        );
        assert!(out.stdout.is_empty(), "{bad}");
    }
}

#[test]
fn find_of_a_type_no_topic_carries_prints_nothing_and_exits_zero() {
    let root = tempfile::tempdir().unwrap();
    // A bare name is a workspace schema with no package, so it is a valid
    // spelling too.
    for spelling in [
        "geometry_msgs/Twist",
        "geometry_msgs/msg/Twist",
        "DetectionArray",
        "1scan",
        "Scan-2",
        "pkg/srv/Type",
    ] {
        for scope in [&[][..], &["--local"]] {
            let mut args = vec!["topic", "find", spelling];
            args.extend_from_slice(scope);
            let out = run(root.path(), &args);
            assert_eq!(out.status.code(), Some(0), "{}", stderr(&out));
            assert!(out.stdout.is_empty(), "{args:?}");
            assert!(out.stderr.is_empty(), "{args:?}: {}", stderr(&out));
        }
    }
}

#[test]
fn list_t_with_no_topics_prints_what_plain_list_prints() {
    let root = tempfile::tempdir().unwrap();
    let plain = run(root.path(), &["topic", "list", "--local"]);
    assert_eq!(plain.status.code(), Some(0), "{}", stderr(&plain));
    for flag in ["-t", "--show-types"] {
        let typed = run(root.path(), &["topic", "list", flag, "--local"]);
        assert_eq!(typed.status.code(), Some(0), "{}", stderr(&typed));
        assert_eq!(
            String::from_utf8_lossy(&typed.stdout),
            "No active local topics.\n"
        );
        assert_eq!(typed.stdout, plain.stdout);
        assert_eq!(typed.stderr, plain.stderr, "{flag}");
    }
}

#[test]
fn bw_refuses_a_window_that_cannot_give_a_rate() {
    let root = tempfile::tempdir().unwrap();
    let out = run(
        root.path(),
        &["topic", "bw", "/intro-test/missing", "--window", "1"],
    );
    assert_eq!(out.status.code(), Some(2));
    let error = stderr(&out);
    assert!(
        error.contains("invalid value '1' for '--window <N>'"),
        "{error}"
    );
}
