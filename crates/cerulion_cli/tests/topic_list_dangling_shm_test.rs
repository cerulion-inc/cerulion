// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic list --no-network` finishes on a namespace whose global
//! management segment a stopped process left half created, and tells the user
//! to run `cerulion clean`.
//!
//! macOS and FreeBSD keep one `/tmp/<name>.shm_state` file per shared memory
//! segment. A file that names absent shared memory made every node creation in
//! its namespace spin at full CPU, and `topic list` creates two nodes.
//!
//! Each run reads a project-local `config/iceoryx2.toml` that puts the real
//! binary on its own prefix and root, so no run reads or writes the default
//! namespace. The planted state file is named by that prefix and removed on
//! drop, and the namespace's own state files are reclaimed on drop.

#![cfg(any(target_os = "macos", target_os = "freebsd"))]

use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};
use std::time::{Duration, Instant};

/// A run still going after this long is taken to spin forever.
const WATCHDOG: Duration = Duration::from_secs(30);
/// What a refused run may take: two node creations, each giving up after the
/// default 1 s `creation_timeout`, plus slack for a loaded machine.
const REFUSAL_BOUND: Duration = Duration::from_secs(10);
/// The global management segment's name after a namespace's prefix: the hash
/// iceoryx2 derives from the segment's stored type, `_node`, then iceoryx2's
/// version and the segment suffix.
const GLOBAL_MGMT_STEM: &str = "9204dd183d1c4d8e1ff592a30766ffa6765aa8b5_node.0_10_0.global_mgmt";
/// A real shared memory name begins with its creator's pid, and no process has
/// pid 0, so no shared memory under this name exists. macOS reads it from a
/// 33-byte NUL-padded field.
const ABSENT_REAL_NAME: &[u8; 33] = b"0_0_0_0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

/// A private workspace directory whose iceoryx2 config names a fresh prefix.
struct IsolatedNamespace {
    dir: tempfile::TempDir,
    prefix: String,
}

impl IsolatedNamespace {
    fn new(tag: &str) -> Self {
        let dir = tempfile::tempdir().expect("create a private directory");
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock before the epoch")
            .as_nanos();
        let prefix = format!("test_prefix_{tag}_{}_{nanos}_", std::process::id());
        let config = dir.path().join("config");
        std::fs::create_dir_all(&config).expect("create the config directory");
        let root = dir.path().join("shm");
        std::fs::write(
            config.join("iceoryx2.toml"),
            format!(
                "[global]\nroot-path = \"{}/\"\nprefix = \"{prefix}\"\n",
                root.display()
            ),
        )
        .expect("write the project-local iceoryx2 config");
        Self { dir, prefix }
    }

    fn path(&self) -> &Path {
        self.dir.path()
    }

    fn global_mgmt_state_file(&self) -> PathBuf {
        PathBuf::from(format!("/tmp/{}{GLOBAL_MGMT_STEM}.shm_state", self.prefix))
    }

    /// Every state file in `/tmp` this namespace's prefix names, sorted.
    fn state_files(&self) -> Vec<PathBuf> {
        let mut files: Vec<PathBuf> = std::fs::read_dir("/tmp")
            .expect("read /tmp")
            .flatten()
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| {
                        name.starts_with(&self.prefix) && name.ends_with(".shm_state")
                    })
            })
            .collect();
        files.sort();
        files
    }
}

/// The runs exited, so the namespace's segments are unused: reclaim each with
/// the reclaimer `cerulion clean` uses, object first and state file second.
/// Its registry and age checks are not needed on a private prefix. A file it
/// refuses (such as a planted name it cannot parse) stays for its own guard.
impl Drop for IsolatedNamespace {
    fn drop(&mut self) {
        for file in self.state_files() {
            let _ = cerulion_cli_engine::shm_state::remove_unused_state_file(
                &file,
                &cerulion_cli_engine::shm_state::LibcProbe,
            );
        }
    }
}

/// A state file planted for one namespace, removed on drop.
struct PlantedStateFile(PathBuf);

impl PlantedStateFile {
    fn new(path: PathBuf, content: &[u8]) -> Self {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .expect("plant the state file");
        std::io::Write::write_all(&mut file, content).expect("write the state file");
        Self(path)
    }
}

impl Drop for PlantedStateFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

/// Run `cerulion topic list --no-network` in `namespace`, killing it and
/// failing the test if it is still running after [`WATCHDOG`].
fn topic_list(namespace: &IsolatedNamespace) -> (Output, Duration) {
    let home = namespace.path().join("home");
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(["topic", "list", "--no-network"])
        .current_dir(namespace.path())
        .env("CERULION_HOME", &home)
        .env("CERULION_NETD_SOCK", namespace.path().join("netd.sock"))
        .env("CERULION_NETD_BIN", namespace.path().join("absent-netd"))
        .env("CERULION_VIZD_SOCK", namespace.path().join("vizd.sock"))
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1")
        .env_remove("RUST_LOG")
        .env_remove("CERULION_NETWORK")
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn the real CLI");
    let start = Instant::now();
    loop {
        if child.try_wait().expect("poll the CLI").is_some() {
            let elapsed = start.elapsed();
            let output = child.wait_with_output().expect("collect the CLI output");
            return (output, elapsed);
        }
        if start.elapsed() > WATCHDOG {
            let _ = child.kill();
            let _ = child.wait();
            panic!("`cerulion topic list --no-network` still ran after {WATCHDOG:?}: it spins");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// The control: the same isolated setup with no stale state lists nothing.
#[test]
fn topic_list_no_network_lists_nothing_on_a_clean_isolated_namespace() {
    let namespace = IsolatedNamespace::new("clean");

    let (output, _) = topic_list(&namespace);

    assert_eq!(
        String::from_utf8_lossy(&output.stdout),
        "No active local topics.\n",
        "stderr:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(output.status.code(), Some(0));
}

/// The reported shape: a management segment state file naming absent shared
/// memory. The command fails within its bound, prints no listing, and names
/// the reason and the fix.
#[test]
fn topic_list_no_network_fails_within_its_bound_on_a_dangling_global_mgmt_state_file() {
    let namespace = IsolatedNamespace::new("dangling");
    let _planted = PlantedStateFile::new(namespace.global_mgmt_state_file(), ABSENT_REAL_NAME);

    let (output, elapsed) = topic_list(&namespace);

    assert_eq!(
        namespace.state_files(),
        vec![namespace.global_mgmt_state_file()],
        "a refused run creates no segment"
    );

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(1), "stderr:\n{stderr}");
    assert_eq!(String::from_utf8_lossy(&output.stdout), "");
    assert!(
        stderr.contains(
            "Failed to create node 'cerulion': UnableToGenerateUniqueNodeId. Check that no \
             other Cerulion process is using the same node name, and that shared memory \
             permissions are correct. If a Cerulion process stopped while it started, run \
             `cerulion clean` to remove the shared memory state it left behind."
        ),
        "stderr:\n{stderr}"
    );
    assert!(
        elapsed < REFUSAL_BOUND,
        "the refusal took {elapsed:?}, past {REFUSAL_BOUND:?}"
    );
}

/// The other shape an interrupted creation leaves: an empty state file.
#[test]
fn topic_list_no_network_fails_within_its_bound_on_an_empty_global_mgmt_state_file() {
    let namespace = IsolatedNamespace::new("empty");
    let _planted = PlantedStateFile::new(namespace.global_mgmt_state_file(), &[]);

    let (output, elapsed) = topic_list(&namespace);

    assert_eq!(
        namespace.state_files(),
        vec![namespace.global_mgmt_state_file()],
        "a refused run creates no segment"
    );

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(1), "stderr:\n{stderr}");
    assert_eq!(String::from_utf8_lossy(&output.stdout), "");
    assert!(stderr.contains("`cerulion clean`"), "stderr:\n{stderr}");
    assert!(
        elapsed < REFUSAL_BOUND,
        "the refusal took {elapsed:?}, past {REFUSAL_BOUND:?}"
    );
}
