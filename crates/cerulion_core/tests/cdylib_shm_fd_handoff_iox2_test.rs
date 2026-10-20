// SPDX-License-Identifier: AGPL-3.0-only
//! A node cdylib that drops host-created ports must not poison the host's
//! later iceoryx2 calls (macOS).
//!
//! # The shape
//!
//! The host builds every node's `NodeContext` (its publishers and subscribers)
//! and hands it to the node cdylib through `cerulion_node_init`; the cdylib
//! drops it in `cerulion_node_shutdown`. Each cdylib statically links its own
//! copy of iceoryx2, so the drop runs the CDYLIB's `close()` on shared-memory
//! descriptors the HOST's copy opened.
//!
//! On macOS, iceoryx2 0.10 keeps a per-copy table from each shared-memory
//! descriptor to the descriptor of its `.shm_state` side file; `fstat`,
//! `fchmod` and `fchown` on a shared-memory descriptor are routed through it,
//! and `close` removes the entry. A close in the cdylib's copy removes nothing
//! from the host's table, so the host keeps an entry for a descriptor number
//! the kernel has already freed. With the table as released in 0.10.0, two
//! things then go wrong in the host:
//!
//! * the next `shm_open` that the kernel answers with that number finds the
//!   entry still present, refuses to register, and fails with `ENFILE`, which
//!   surfaces as `PublishSubscribeCreateError(InternalFailure)` from the next
//!   service the host creates (`cerulion graph run` pre-creating its topics);
//! * a regular file that the kernel answers with that number has its
//!   `fchmod` written into the stale side file instead of the file, and its
//!   `fstat` mode read from there. iceoryx2 uses file permissions as the
//!   "initialization finished" flag of its static storage, so that is a
//!   cross-process correctness failure, not only a wrong number.
//!
//! # What is asserted
//!
//! Each test runs in a fresh child process (its descriptor table is the
//! subject). The child loads `test_node_macro_period_cdylib` three times as a
//! three-node graph, steps it, shuts its nodes down (the cdylib drops the
//! host-built ports), and checks that the node shutdowns really freed
//! descriptors. Then:
//!
//! * [`host_creates_services_after_a_cdylib_dropped_its_ports`] creates
//!   publishers on fresh topics through the same transport manager. Every
//!   creation must succeed.
//! * [`host_file_permissions_are_exact_after_a_cdylib_dropped_its_ports`]
//!   creates regular files through iceoryx2's file API, sets their permission,
//!   and compares both the iceoryx2-reported permission and the kernel's mode
//!   against the requested one.
//!
//! On Linux there is no such table and both tests are plain controls.
//!
//! # Running (needs the fixture built first)
//!
//! ```bash
//! cargo build -p test_node_macro_period_cdylib
//! cargo nextest run -p cerulion_core --test cdylib_shm_fd_handoff_iox2_test
//! ```

use std::io::Read;
use std::os::unix::fs::PermissionsExt;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use iceoryx2_bb_container::semantic_string::SemanticString;
use iceoryx2_bb_posix::file::{CreationMode, File, FileBuilder, Permission};
use iceoryx2_bb_posix::file_descriptor::FileDescriptorManagement;
use iceoryx2_bb_system_types::file_path::FilePath;
use indexmap::IndexMap;

/// Set ONLY on the spawned child; its value names the scenario to run.
const ENV_CHILD: &str = "CER_SHM_FD_HANDOFF_CHILD";
const FIXTURE: &str = "test_node_macro_period_cdylib";
/// Three cdylib nodes, so the shutdown frees several host descriptors.
const NODES: usize = 3;
/// More creations than descriptors the shutdown frees, so the kernel is
/// certain to hand the freed numbers back.
const CREATIONS: usize = 24;
const CHILD_TIMEOUT: Duration = Duration::from_secs(120);

fn open_descriptor_count() -> usize {
    std::fs::read_dir("/dev/fd").expect("list /dev/fd").count()
}

/// Build the three-node graph, step it, shut it down, and return the transport
/// manager that outlives it (the shape of `graph run`'s supervisor, which
/// keeps its transport after the in-process nodes are gone).
fn run_and_shut_down_cdylib_graph(prefix: &str) -> Arc<cerulion_core::TransportManager> {
    let path = cerulion_core::testing::find_fixture_cdylib(FIXTURE);
    assert!(
        path.exists(),
        "fixture cdylib not found at {path:?}; run `cargo build -p {FIXTURE}` first"
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut nodes = Vec::new();
    for i in 0..NODES {
        let id = format!("n{i}");
        factories.insert(
            id.clone(),
            Box::new(DylibNodeEntry::load(&path).expect("load fixture cdylib")),
        );
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id,
            node_type: "period_node".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "cmd".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        });
    }
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("fdh_{prefix}"),
        prefix: prefix.to_string(),
        nodes,
    };
    let mut rt = GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the cdylib graph");
    for _ in 0..4 {
        rt.step(Duration::from_millis(50));
    }
    let mgr = Arc::clone(rt.test_transport().expect("test transport parked"));

    // Counted around the node shutdowns alone: each runs the cdylib's
    // `cerulion_node_shutdown`, which drops the host-built ports. The scenario
    // only exists if that drop really closed descriptors; without this check
    // the tests below could pass vacuously.
    let before = open_descriptor_count();
    rt.shutdown_all_nodes();
    let after = open_descriptor_count();
    assert!(
        after + 2 <= before,
        "the cdylib node shutdowns freed {} descriptor(s) ({before} -> {after}); \
         at least 2 are needed for this test to exercise anything",
        before.saturating_sub(after)
    );
    eprintln!("FDH_FREED={}", before - after);
    drop(rt);
    mgr
}

fn child_services() {
    let mgr = run_and_shut_down_cdylib_graph("fdhs");
    let mut publishers = Vec::new();
    for i in 0..CREATIONS {
        let topic = format!("/fdhs/after_shutdown/t{i}");
        match mgr.create_publisher(&topic, MaxSliceLen::const_new(64), 0) {
            Ok(p) => publishers.push(p),
            Err(e) => panic!(
                "creating publisher {i} of {CREATIONS} on {topic} after the cdylib \
                 dropped its ports failed: {e}"
            ),
        }
    }
    eprintln!("FDH_CREATED={}", publishers.len());
}

/// Removes the child's scratch directory, also when an assertion fails.
struct ScratchDir(std::path::PathBuf);

impl Drop for ScratchDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn child_permissions() {
    let _mgr = run_and_shut_down_cdylib_graph("fdhp");
    let scratch = ScratchDir(std::env::temp_dir().join(format!("cer_fdh_{}", std::process::id())));
    let dir = &scratch.0;
    std::fs::create_dir_all(dir).expect("create scratch dir");
    let requested = Permission::OWNER_READ;
    let mut files = Vec::new();
    let mut wrong = Vec::new();
    for i in 0..CREATIONS {
        let path = dir.join(format!("f{i}"));
        let file_path =
            FilePath::new(path.to_str().expect("utf-8 path").as_bytes()).expect("file path");
        let mut file = FileBuilder::new(&file_path)
            .creation_mode(CreationMode::PurgeAndCreate)
            .permission(Permission::OWNER_READ_WRITE)
            .create()
            .expect("create file through iceoryx2");
        file.set_permission(requested)
            .expect("set file permission through iceoryx2");
        let reported = file.permission().expect("read permission through iceoryx2");
        let kernel_mode = std::fs::metadata(&path).expect("stat").permissions().mode() & 0o7777;
        if kernel_mode != requested.bits() as u32 || reported != requested {
            wrong.push(format!(
                "f{i}: kernel mode {kernel_mode:o}, iceoryx2 reports {reported:?}, \
                 requested {:o}",
                requested.bits()
            ));
        }
        // Keep each file open so the next one takes the next free number.
        files.push((file, file_path));
    }
    for (file, file_path) in files {
        drop(file);
        let _ = File::remove(&file_path);
    }
    assert!(
        wrong.is_empty(),
        "{} of {CREATIONS} files created after the cdylib dropped its ports did not get \
         the requested permission:\n{}",
        wrong.len(),
        wrong.join("\n")
    );
    eprintln!("FDH_FILES_OK={CREATIONS}");
}

/// Child entry point: runs the scenario named by `ENV_CHILD`, a no-op otherwise.
#[test]
#[ignore = "child process entry point, driven by the parent tests in this file"]
// The exit code is how this re-executed child reports its verdict to the
// parent, so exiting is its contract.
#[allow(clippy::disallowed_methods)]
fn subprocess_child_shm_fd_handoff() {
    match std::env::var(ENV_CHILD).as_deref() {
        Ok("services") => child_services(),
        Ok("permissions") => child_permissions(),
        _ => return,
    }
    std::process::exit(0);
}

/// Run the scenario in a fresh child; return its stderr, panicking with it
/// when the child fails or hangs.
fn run_child(scenario: &str) -> String {
    let exe = std::env::current_exe().expect("current_exe");
    let mut child = std::process::Command::new(exe)
        .args([
            "--exact",
            "subprocess_child_shm_fd_handoff",
            "--ignored",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(ENV_CHILD, scenario)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .expect("spawn child");
    let mut err = child.stderr.take().expect("child stderr");
    let drain = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = err.read_to_string(&mut buf);
        buf
    });
    let deadline = Instant::now() + CHILD_TIMEOUT;
    let status = loop {
        if let Some(status) = child.try_wait().expect("try_wait") {
            break status;
        }
        if Instant::now() > deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("{scenario} child did not exit within {CHILD_TIMEOUT:?}");
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let stderr = drain.join().expect("stderr drain thread");
    assert!(
        status.success(),
        "{scenario} child failed ({status}); stderr:\n{stderr}"
    );
    stderr
}

fn marker(stderr: &str, key: &str) -> Option<usize> {
    stderr
        .lines()
        .find_map(|l| l.trim().strip_prefix(key))
        .and_then(|v| v.trim().parse().ok())
}

#[test]
fn host_creates_services_after_a_cdylib_dropped_its_ports() {
    let stderr = run_child("services");
    assert!(
        marker(&stderr, "FDH_FREED=").is_some_and(|n| n >= 2),
        "the child never reported the freed descriptors; stderr:\n{stderr}"
    );
    assert_eq!(
        marker(&stderr, "FDH_CREATED="),
        Some(CREATIONS),
        "the child did not report all {CREATIONS} creations; stderr:\n{stderr}"
    );
}

#[test]
fn host_file_permissions_are_exact_after_a_cdylib_dropped_its_ports() {
    let stderr = run_child("permissions");
    assert!(
        marker(&stderr, "FDH_FREED=").is_some_and(|n| n >= 2),
        "the child never reported the freed descriptors; stderr:\n{stderr}"
    );
    assert_eq!(
        marker(&stderr, "FDH_FILES_OK="),
        Some(CREATIONS),
        "the child did not report all {CREATIONS} files; stderr:\n{stderr}"
    );
}
