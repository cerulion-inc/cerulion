// SPDX-License-Identifier: AGPL-3.0-only
//! Runs the Java JNI oracle against a host build of the sample app's
//! runtime library, the one library Java loads in an app.
//!
//! Build it first, then point `CERULION_APP_RUNTIME_LIB` at it:
//!
//! ```sh
//! cargo build --manifest-path crates/cerulion_android/android/runtime/Cargo.toml -p cerulion_sample_runtime
//! CERULION_APP_RUNTIME_LIB=$PWD/crates/cerulion_android/android/runtime/target/debug/libcerulion_sample_runtime.so \
//!   cargo test -p cerulion_android --test jni_java_test -- --ignored
//! ```
//!
//! On macOS the library ends in `.dylib`, and the run needs a short `TMPDIR`
//! (for example `TMPDIR=/tmp`): the runtime keeps its iceoryx2 files under
//! the files directory, and under the long default macOS temporary path the
//! first publisher fails with `ResourceCreationFailed`.

use std::path::{Path, PathBuf};
use std::process::Command;

use tempfile::tempdir;

/// The library file and the name Java's `System.loadLibrary` takes for it.
fn runtime_library() -> (PathBuf, String) {
    let library = std::env::var_os("CERULION_APP_RUNTIME_LIB")
        .map(PathBuf::from)
        .expect("set CERULION_APP_RUNTIME_LIB to the host build of libcerulion_sample_runtime");
    assert!(
        library.is_file(),
        "{} is not a file; build the runtime library first: cargo build --manifest-path crates/cerulion_android/android/runtime/Cargo.toml -p cerulion_sample_runtime",
        library.display()
    );
    let stem = library
        .file_stem()
        .and_then(|stem| stem.to_str())
        .expect("a UTF-8 library file name");
    let name = stem
        .strip_prefix("lib")
        .expect("a library file name starts with lib")
        .to_string();
    (library, name)
}

#[test]
#[ignore = "needs a JDK on PATH and the host build of the sample runtime library"]
fn java_api_matches_hand_written_oracle() {
    let (library, library_name) = runtime_library();
    let library_dir = library.parent().expect("library directory").to_path_buf();

    let classes = tempdir().expect("Java class output directory");
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let java_api = manifest_dir.join("java/com/cerulion/android/CerulionRuntime.java");
    let smoke = manifest_dir.join("tests/java/JniSmoke.java");
    let graph = manifest_dir.join("android/app/src/main/assets/graph.yaml");
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
        .arg(format!("-Djava.library.path={}", library_dir.display()))
        .arg("-cp")
        .arg(classes.path())
        .arg("JniSmoke")
        .arg(files_dir.path())
        .arg(&library_name)
        .arg(&graph)
        .output()
        .expect("run Java JNI smoke test");
    assert!(
        java.status.success(),
        "Java smoke test failed:\n{}",
        String::from_utf8_lossy(&java.stderr)
    );

    assert_eq!(library_name, "cerulion_sample_runtime");
    let expected = "\
status-before-load=call CerulionRuntime.loadLibrary(<your runtime library>) first
load-again=no-exception
load-other=CerulionRuntime is bound to libcerulion_sample_runtime.so
status-before-init=uninitialized
codes=14,15,16,17,18
init-bad=3
init-ok=0
linked-nodes=[\"printer\",\"relay\",\"timer\"]
publish-before-start=4
poll-before-start=4
start-invalid=0 state=failed error=3
start-ok=0 state=running
publish=0
publish-unknown=8
poll=0 {\"data\":\"hello\"}
poll-empty=15
poll-unknown=8
poll-null-out=3
poll-null-topic=3
poll-overflow=0 {\"data\":\"b\"}, 0 {\"data\":\"\u{e7}\"}, 15
subscribed={\"/jni/heartbeat\":{\"polled\":3,\"dropped\":1,\"undecodable\":0}}
publish-stamped=0
publish-stamped-negative=3
publish-stamped-ahead=3
publish-stamped-null=3
publish-stamped-ahead-oversized=3
poll-stamped=0 {\"data\":\"stamped\"}
poll-stamped-empty=15
stop=0 state=stopped
start-graph=0 state=running nodes=3
poll-graph=0 {\"data\":2}
stop-graph=0 state=stopped";
    assert_eq!(String::from_utf8_lossy(&java.stdout).trim(), expected);
}
