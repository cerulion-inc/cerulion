// SPDX-License-Identifier: AGPL-3.0-only
//! Starter acquisition through the real binary, without a repository cwd.

use std::process::Command;

fn auth_home() -> tempfile::TempDir {
    let home = tempfile::tempdir().unwrap();
    cerulion_cli_engine::auth::seed_logged_in_at(home.path(), "acct-starter-test").unwrap();
    home
}

fn command(root: &std::path::Path, home: &std::path::Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    command
        .current_dir(root)
        .env("CERULION_HOME", home)
        .env_remove("CERULION_LOGIN_GATE")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    command
}

#[test]
fn starter_command_installs_complete_sources_and_explains_dependency_selection() {
    let parent = tempfile::tempdir().unwrap();
    let home = auth_home();
    let output = command(parent.path(), home.path())
        .args([
            "workspace",
            "create",
            "demo",
            "--starter",
            "obstacle_avoidance",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(stdout.contains("starter: obstacle_avoidance (bundled with this CLI); see README.md"));
    assert!(stdout.contains("dependencies:"));
    let root = parent.path().join("demo");
    for path in [
        "Cargo.toml",
        "starter.toml",
        "README.md",
        ".cargo/config.toml",
        "graphs/obstacle_avoidance.yaml",
        "nodes/laser_scanner/Cargo.toml",
        "nodes/laser_scanner/src/lib.rs",
        "nodes/laser_scanner/src/tests.rs",
        "nodes/safety_controller/Cargo.toml",
        "nodes/safety_controller/src/lib.rs",
    ] {
        assert!(root.join(path).is_file(), "missing starter file: {path}");
    }
    let source = std::fs::read_to_string(root.join("nodes/laser_scanner/src/lib.rs")).unwrap();
    assert!(source.contains("self.scan.header.frame_id = \"laser\""));
    assert!(source.contains("self.scan.loan_ranges(180)?.fill(nearest)"));
    assert!(source.contains("self.scan.intensities = &[][..]"));
    assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);

    let again = command(parent.path(), home.path())
        .args([
            "workspace",
            "create",
            "demo",
            "--starter",
            "obstacle_avoidance",
        ])
        .output()
        .unwrap();
    assert_eq!(again.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&again.stderr).contains("Workspace already exists"));
    assert_eq!(
        std::fs::read_to_string(root.join("nodes/laser_scanner/src/lib.rs")).unwrap(),
        source
    );
    assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 1);
}

#[test]
fn unknown_starter_refuses_before_creating_any_workspace() {
    let parent = tempfile::tempdir().unwrap();
    let home = auth_home();
    let output = command(parent.path(), home.path())
        .args(["workspace", "create", "demo", "--starter", "main"])
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("obstacle_avoidance"));
    assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 0);
}

#[test]
fn bare_workspace_create_preserves_the_empty_authoring_workflow() {
    let parent = tempfile::tempdir().unwrap();
    let home = auth_home();
    let output = command(parent.path(), home.path())
        .args(["workspace", "create", "manual"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let root = parent.path().join("manual");
    assert_eq!(std::fs::read_dir(root.join("nodes")).unwrap().count(), 0);
    assert_eq!(std::fs::read_dir(root.join("graphs")).unwrap().count(), 0);
    assert!(!root.join("starter.toml").exists());
    assert!(!String::from_utf8_lossy(&output.stdout).contains("starter:"));
}

#[cfg(unix)]
#[test]
fn starter_root_permissions_match_ordinary_workspaces_under_explicit_umasks() {
    use std::os::unix::fs::PermissionsExt;
    for (mask, expected_mode) in [("0022", 0o755), ("0077", 0o700)] {
        let parent = tempfile::tempdir().unwrap();
        let home = auth_home();
        for (name, starter) in [("manual", false), ("demo", true)] {
            // The mask belongs to this child process, never the parallel test runner.
            let mut command = Command::new("sh");
            command.args([
                "-c",
                "umask \"$1\"; shift; exec \"$@\"",
                "starter-mode-test",
                mask,
            ]);
            command.arg(env!("CARGO_BIN_EXE_cerulion"));
            command.args(["workspace", "create", name]);
            if starter {
                command.args(["--starter", "obstacle_avoidance"]);
            }
            let output = command
                .current_dir(parent.path())
                .env("CERULION_HOME", home.path())
                .env_remove("CERULION_LOGIN_GATE")
                .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            assert_eq!(
                std::fs::metadata(parent.path().join(name))
                    .unwrap()
                    .permissions()
                    .mode()
                    & 0o777,
                expected_mode,
                "{name} under umask {mask}"
            );
        }
        assert_eq!(std::fs::read_dir(parent.path()).unwrap().count(), 2);
    }
}
