// SPDX-License-Identifier: AGPL-3.0-only
//! Runs the Java JNI oracle against a previously built host cdylib.

use std::path::{Path, PathBuf};
use std::process::Command;

use tempfile::tempdir;

fn target_dir() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target"))
}

#[test]
#[ignore = "needs a JDK on PATH and the host cdylib built"]
fn java_api_matches_hand_written_oracle() {
    let target_dir = target_dir();
    let library = target_dir.join("debug/libcerulion_android.so");
    assert!(
        library.is_file(),
        "build the host cdylib first: cargo build -p cerulion_android"
    );

    let classes = tempdir().expect("Java class output directory");
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let java_api = manifest_dir.join("java/com/cerulion/android/CerulionRuntime.java");
    let smoke = manifest_dir.join("tests/java/JniSmoke.java");
    let javac = Command::new("javac")
        .arg("-d")
        .arg(classes.path())
        .arg(java_api)
        .arg(smoke)
        .output()
        .expect("run javac");
    assert!(
        javac.status.success(),
        "javac failed:\n{}",
        String::from_utf8_lossy(&javac.stderr)
    );

    let files_dir = tempdir().expect("JNI files directory");
    let java = Command::new("java")
        .arg(format!(
            "-Djava.library.path={}",
            target_dir.join("debug").display()
        ))
        .arg("-cp")
        .arg(classes.path())
        .arg("JniSmoke")
        .arg(files_dir.path())
        .arg("cerulion_android")
        .output()
        .expect("run Java JNI smoke test");
    assert!(
        java.status.success(),
        "Java smoke test failed:\n{}",
        String::from_utf8_lossy(&java.stderr)
    );

    let expected = "\
status-before-load=call CerulionRuntime.loadLibrary(<your runtime library>) first
load-again=no-exception
load-other=CerulionRuntime is bound to libcerulion_android.so
status-before-init=uninitialized
init-bad=3
init-ok=0
linked-nodes=true
publish-before-start=4
start-invalid=0 state=failed error=3
start-ok=0 state=running
publish=0
publish-unknown=8
stop=0 state=stopped";
    assert_eq!(String::from_utf8_lossy(&java.stdout).trim(), expected);
}
