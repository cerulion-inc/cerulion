// SPDX-License-Identifier: AGPL-3.0-only
#![cfg(not(feature = "local-transport"))]
// Every arm builds a real iceoryx2 node for other processes to see, which
// local-transport refuses.
//! A node whose shared memory cannot be set up fails within its creation
//! timeout, and the transport singleton reports that failure instead of
//! aborting the process.
//!
//! macOS and FreeBSD emulate POSIX shared memory with one `<name>.shm_state`
//! file per segment in `/tmp`. A process that stops between writing that file
//! and creating the segment leaves a file that names absent shared memory, or
//! an empty one. Every node creation opens or creates the namespace's global
//! management segment, so such a file on that segment used to make every later
//! node creation in the namespace spin at full CPU forever.
//!
//! Every arm runs on its own isolated prefix with a short creation timeout.
//! A planted state file is named by that prefix, so no arm reads or writes the
//! default namespace, and each planted file is removed on drop.
//!
//! The singleton arm runs alone (`-- --test-threads=1`): it is the only arm
//! that installs the process singleton, and every assertion about
//! `TransportManager::get()` lives inside it.

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::{TransportConfig, TransportError, TransportManager, VirtualClock};
use iceoryx2::config::Config;
use iceoryx2::prelude::CallbackProgression;
use serial_test::serial;
use std::sync::Arc;
use std::time::{Duration, Instant};

type Service = iceoryx2::service::ipc_threadsafe::Service;

/// The creation timeout every arm configures. Short, so a bounded failure is
/// quick, and long enough to tell "waited the timeout" from "failed at once".
const CREATION_TIMEOUT: Duration = Duration::from_millis(200);
/// Slack on top of [`CREATION_TIMEOUT`] for a loaded machine.
const RETURN_GRACE: Duration = Duration::from_secs(3);
/// A call still running after this long is taken to spin forever.
const WATCHDOG: Duration = Duration::from_secs(20);

fn isolated_config() -> Config {
    let mut config = iceoryx_test_config();
    config.global.creation_timeout = CREATION_TIMEOUT;
    config
}

fn transport_config(node_name: &str) -> TransportConfig {
    TransportConfig {
        node_name: node_name.to_string(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    }
}

/// Run `call` on a helper thread and return its result and duration, or fail
/// the test when it is still running after [`WATCHDOG`]. The helper thread is
/// what lets a spinning call fail the test instead of hanging it; a panic in
/// `call` is raised again here.
fn bounded<T: Send + 'static>(call: impl FnOnce() -> T + Send + 'static) -> (T, Duration) {
    let (done, finished) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        let start = Instant::now();
        let result = call();
        let _ = done.send(start.elapsed());
        result
    });
    match finished.recv_timeout(WATCHDOG) {
        Ok(elapsed) => match worker.join() {
            Ok(result) => (result, elapsed),
            Err(panic) => std::panic::resume_unwind(panic),
        },
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => match worker.join() {
            Err(panic) => std::panic::resume_unwind(panic),
            Ok(_) => unreachable!("the worker sends before it returns"),
        },
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
            panic!("the call was still running after {WATCHDOG:?}: it spins instead of failing")
        }
    }
}

fn registered_nodes(config: &Config) -> usize {
    let mut nodes = 0;
    iceoryx2::node::Node::<Service>::list(config, |_| {
        nodes += 1;
        CallbackProgression::Continue
    })
    .expect("list the nodes of the isolated namespace");
    nodes
}

/// The happy path the failure arms are measured against: a fresh namespace
/// gives a node, and the node is registered.
#[test]
#[serial]
fn node_creation_succeeds_on_a_clean_isolated_namespace() {
    let config = isolated_config();
    let probe = config.clone();

    let (manager, _) = bounded(move || {
        TransportManager::detached_with_config(transport_config("clean_probe"), config)
    });

    let manager = manager.expect("a clean namespace creates a node");
    assert_eq!(registered_nodes(&probe), 1);
    drop(manager);
}

/// The message a user reads when node creation fails names the command that
/// removes shared memory state a stopped process left behind.
#[test]
#[serial]
fn node_creation_error_names_cerulion_clean() {
    let message = TransportError::NodeCreation {
        node_name: "camera_node".to_string(),
        reason: "UnableToGenerateUniqueNodeId".to_string(),
    }
    .to_string();
    assert_eq!(
        message,
        "Failed to create node 'camera_node': UnableToGenerateUniqueNodeId. Check that no \
         other Cerulion process is using the same node name, and that shared memory \
         permissions are correct. If a Cerulion process stopped while it started, run \
         `cerulion clean` to remove the shared memory state it left behind."
    );
}

#[cfg(any(target_os = "macos", target_os = "freebsd"))]
mod dangling_state_file {
    use super::*;

    /// The directory the PAL keeps state files in, compiled into iceoryx2.
    const STATE_DIRECTORY: &str = "/tmp";
    /// The name of the global management segment after a namespace's prefix:
    /// the hash iceoryx2 derives from the segment's stored type, `_node`, then
    /// iceoryx2's version and the segment suffix. Read from the file a real node
    /// creation leaves in `/tmp` for the default `iox2_` prefix.
    const GLOBAL_MGMT_STEM: &str =
        "9204dd183d1c4d8e1ff592a30766ffa6765aa8b5_node.0_10_0.global_mgmt";
    /// On macOS a state file holds the real shared memory name in a 33-byte,
    /// NUL-padded field. FreeBSD only checks that the file exists.
    const REAL_NAME_FIELD_LEN: usize = 33;
    /// A real name begins with its creator's pid, and no process has pid 0, so
    /// no shared memory under this name exists.
    const ABSENT_REAL_NAME: &[u8] = b"0_0_0_0";
    /// What iceoryx2 reports when the management segment cannot be opened.
    const REASON: &str = "UnableToGenerateUniqueNodeId";

    /// A state file planted for one namespace, removed on drop.
    struct PlantedStateFile(std::path::PathBuf);

    impl PlantedStateFile {
        fn for_global_mgmt(config: &Config, content: &[u8]) -> Self {
            let prefix = config.global.prefix.to_string();
            assert!(
                prefix.starts_with("test_prefix_"),
                "only an isolated namespace may be planted, got {prefix}"
            );
            let path = std::path::PathBuf::from(format!(
                "{STATE_DIRECTORY}/{prefix}{GLOBAL_MGMT_STEM}.shm_state"
            ));
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
                .expect("plant the state file");
            std::io::Write::write_all(&mut file, content).expect("write the state file");
            Self(path)
        }

        fn remove(&self) {
            std::fs::remove_file(&self.0).expect("remove the planted state file");
        }
    }

    impl Drop for PlantedStateFile {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    fn dangling_name() -> [u8; REAL_NAME_FIELD_LEN] {
        let mut content = [0u8; REAL_NAME_FIELD_LEN];
        content[..ABSENT_REAL_NAME.len()].copy_from_slice(ABSENT_REAL_NAME);
        content
    }

    fn create_detached(
        config: &Config,
        node_name: &'static str,
    ) -> (Result<Arc<TransportManager>, TransportError>, Duration) {
        let config = config.clone();
        bounded(move || TransportManager::detached_with_config(transport_config(node_name), config))
    }

    /// A node creation that waited its timeout, then refused with the reason
    /// iceoryx2 gives, and registered nothing.
    fn assert_bounded_refusal(
        config: &Config,
        result: Result<Arc<TransportManager>, TransportError>,
        elapsed: Duration,
        node_name: &str,
    ) {
        match result {
            Err(TransportError::NodeCreation {
                node_name: refused,
                reason,
            }) => {
                assert_eq!(refused, node_name);
                assert_eq!(reason, REASON);
            }
            Err(other) => panic!("expected NodeCreation, got {other:?}"),
            Ok(_) => panic!("a dangling management segment cannot give a node"),
        }
        assert!(
            elapsed >= CREATION_TIMEOUT,
            "the refusal came after {elapsed:?}, before the {CREATION_TIMEOUT:?} timeout"
        );
        assert!(
            elapsed < CREATION_TIMEOUT + RETURN_GRACE,
            "the refusal took {elapsed:?}, far past the {CREATION_TIMEOUT:?} timeout"
        );
        assert_eq!(
            registered_nodes(config),
            0,
            "a refused node is not registered"
        );
    }

    /// The reported shape: the state file names shared memory that does not
    /// exist.
    #[test]
    #[serial]
    fn node_creation_fails_within_its_timeout_on_a_state_file_that_names_absent_shared_memory() {
        let config = isolated_config();
        let _planted = PlantedStateFile::for_global_mgmt(&config, &dangling_name());

        let (result, elapsed) = create_detached(&config, "dangling_probe");

        assert_bounded_refusal(&config, result, elapsed, "dangling_probe");
    }

    /// A state file with no content, the other shape an interrupted creation
    /// leaves.
    #[test]
    #[serial]
    fn node_creation_fails_within_its_timeout_on_an_empty_state_file() {
        let config = isolated_config();
        let _planted = PlantedStateFile::for_global_mgmt(&config, &[]);

        let (result, elapsed) = create_detached(&config, "empty_probe");

        assert_bounded_refusal(&config, result, elapsed, "empty_probe");
    }

    /// Two attempts against the same dangling state give the same verdict in
    /// the same bound: nothing in the first attempt changes the second.
    #[test]
    #[serial]
    fn node_creation_fails_the_same_way_on_every_attempt_against_a_dangling_state_file() {
        let config = isolated_config();
        let _planted = PlantedStateFile::for_global_mgmt(&config, &dangling_name());

        for _ in 0..2 {
            let (result, elapsed) = create_detached(&config, "repeat_probe");
            assert_bounded_refusal(&config, result, elapsed, "repeat_probe");
        }
    }

    /// The singleton reports the bounded failure as an error, stays unset
    /// through repeated attempts, and initializes once the stale state is gone.
    #[test]
    #[serial]
    fn singleton_init_returns_node_creation_and_stays_unset_until_the_state_is_removed() {
        let config = isolated_config();
        let planted = PlantedStateFile::for_global_mgmt(&config, &dangling_name());

        for _ in 0..2 {
            let attempt = config.clone();
            let (result, elapsed) = bounded(move || {
                TransportManager::init_with_config(transport_config("singleton_probe"), attempt)
            });
            assert_bounded_refusal(&config, result, elapsed, "singleton_probe");
            assert!(
                matches!(TransportManager::get(), Err(TransportError::NotInitialized)),
                "a failed init must leave the singleton unset"
            );
        }

        planted.remove();
        let attempt = config.clone();
        let (result, _) = bounded(move || {
            TransportManager::init_with_config(transport_config("singleton_probe"), attempt)
        });

        let manager = result.expect("the singleton initializes once the stale state is gone");
        let installed = TransportManager::get().expect("the singleton is installed");
        assert!(Arc::ptr_eq(&manager, &installed));
        assert_eq!(registered_nodes(&config), 1);
    }
}
