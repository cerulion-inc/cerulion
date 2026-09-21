// SPDX-License-Identifier: AGPL-3.0-only
//! Hand-written subprocess oracles prove that a PATH compiler warning never
//! prevents Cargo from running with its inherited RUSTC override. No toolchains
//! are compiled or installed; this tests command selection and error propagation.

#![cfg(unix)]

use std::os::unix::fs::PermissionsExt;
use std::path::Path;
use std::process::Command;

use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::node_cmd::node_build;

const CHILD: &str = "CERULION_COMPILER_OVERRIDE_TEST_CHILD";

fn executable(path: &Path, source: &str) {
    std::fs::write(path, source).unwrap();
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o755)).unwrap();
}

#[test]
fn path_compiler_mismatch_preserves_cargo_override_and_result() {
    if let Ok(mode) = std::env::var(CHILD) {
        let root = std::env::var_os("CERULION_COMPILER_TEST_ROOT").unwrap();
        let result = node_build(Path::new(&root), "probe_node", false);
        match mode.as_str() {
            "success" => assert!(
                result.is_ok(),
                "Cargo must run despite PATH mismatch: {result:?}"
            ),
            "failure" => match result {
                Err(CliError::BuildFailed { reason, .. }) => {
                    assert_eq!(reason.trim(), "compiler-override-cargo-failure");
                }
                other => panic!("expected the Cargo failure, got {other:?}"),
            },
            other => panic!("unknown child mode {other}"),
        }
        return;
    }

    for mode in ["success", "failure"] {
        let dir = tempfile::tempdir().unwrap();
        let canonical_root = dir.path().canonicalize().unwrap();
        let root = canonical_root.as_path();
        let bin = root.join("bin");
        let node = root.join("nodes/probe_node");
        std::fs::create_dir_all(&bin).unwrap();
        std::fs::create_dir_all(&node).unwrap();
        std::fs::write(
            node.join("Cargo.toml"),
            "[package]\nname = \"probe_node\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        // This release is deliberately distinct from the host. The command
        // validates its arguments so a skipped or changed probe cannot pass.
        executable(
            &bin.join("rustc"),
            "#!/bin/sh\n[ \"$#\" -eq 1 ] && [ \"$1\" = -vV ] || exit 71\nprintf 'release: 0.0.0\n'\nprintf 'probed\n' > \"$CERULION_COMPILER_TEST_ROOT/probe-ran\"\n",
        );
        let selected = bin.join("selected-rustc");
        executable(
            &selected,
            "#!/bin/sh\n[ \"$#\" -eq 1 ] && [ \"$1\" = -vV ] || exit 72\nprintf 'release: %s\n' \"$CERULION_COMPILER_TEST_HOST_RELEASE\"\n",
        );
        executable(
            &bin.join("cargo"),
            r#"#!/bin/sh
[ "$#" -eq 3 ] && [ "$1" = build ] && [ "$2" = -p ] && [ "$3" = probe_node ] || exit 73
[ "$PWD" = "$CERULION_COMPILER_TEST_ROOT" ] || exit 74
[ "$("$RUSTC" -vV)" = "release: $CERULION_COMPILER_TEST_HOST_RELEASE" ] || exit 75
printf 'build\n-p\nprobe_node\n' > "$CERULION_COMPILER_TEST_ROOT/cargo-ran"
if [ "$CERULION_COMPILER_OVERRIDE_TEST_CHILD" = failure ]; then
    printf 'compiler-override-cargo-failure\n' >&2
    exit 42
fi
"#,
        );
        let output = Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "path_compiler_mismatch_preserves_cargo_override_and_result",
                "--nocapture",
            ])
            .env(CHILD, mode)
            .env("CERULION_COMPILER_TEST_ROOT", root)
            .env(
                "CERULION_COMPILER_TEST_HOST_RELEASE",
                cerulion_core::RUSTC_RELEASE,
            )
            .env("PATH", &bin)
            .env("RUSTC", &selected)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{mode} child failed: {}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        assert_eq!(
            std::fs::read_to_string(root.join("probe-ran")).unwrap(),
            "probed\n"
        );
        assert_eq!(
            std::fs::read_to_string(root.join("cargo-ran")).unwrap(),
            "build\n-p\nprobe_node\n"
        );
    }
}
