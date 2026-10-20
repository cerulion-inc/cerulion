// SPDX-License-Identifier: AGPL-3.0-only
//! A fatally stopped `external` node does not keep waking the live loop.
//!
//! The shape: a camera driver hands the runtime its device fd, the device
//! dies (the peer hangs up, so the fd is `POLLHUP`, ready forever), and the
//! node's next tick returns `NodeError::Fatal`. The scheduler stops the node,
//! so nothing drains or services the fd again. If its wake binding stayed in
//! the live wait set, every wait would return at once and the loop would spin
//! a core for the rest of the run.
//!
//! Oracles (hand-written): before the stop, a reactor wait on the hung-up fd
//! returns at once naming `cam`; the fatal fire runs once and is reported
//! once; after the stop, the same wait names nothing and lasts its whole
//! timeout, three live steps take at least their three timeouts, the node
//! fires no more, and its binding is kept (so a reset restores it).
//!
//! No fake data: the wake is a real pipe whose write end is closed.
//! `#[serial]`: real iceoryx2 over the process-global SHM singleton.
//! Run alone: `cargo test -p cerulion_core --test node_fatal_external_wake_iox2_test -- --test-threads=1`.

#![cfg(unix)]

use std::os::unix::io::RawFd;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::MacroPolicy;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The read end of a pipe whose write end is already CLOSED: `poll(2)`
/// reports `POLLHUP` on it from now on, the shape of a device whose peer is
/// gone. Closed on drop.
struct HungUpFd(RawFd);

impl HungUpFd {
    fn new() -> Self {
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: `fds` is a 2-element array; `pipe` writes exactly two fds.
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0, "pipe(2)");
        // SAFETY: `fds[1]` is the write end just returned by pipe(), closed once.
        assert_eq!(unsafe { libc::close(fds[1]) }, 0, "close the write end");
        Self(fds[0])
    }
}

impl Drop for HungUpFd {
    fn drop(&mut self) {
        // SAFETY: the read end this struct opened; closed exactly once, after
        // the runtime (which never closes a device fd) is gone.
        unsafe { libc::close(self.0) };
    }
}

/// An `external` driver over `fd` whose every tick returns `Fatal`, the way
/// a camera driver reports a device it cannot recover. It never reads `fd`.
struct DeadCamera {
    src: Option<ExternalSource>,
    fires: Arc<AtomicU64>,
}

impl NodeEntry for DeadCamera {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            vec![],
            vec![OutputMeta::new(
                "out".to_string(),
                Vector3::SCHEMA_HASH,
                Vector3::MAX_SLICE_LEN,
            )],
        )
        .with_policy(MacroPolicy::External))
    }

    fn init(&mut self, _ctx: NodeContext) -> TransportResult<()> {
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        Err(NodeError::Fatal("camera lost".to_string()).into_transport("cam"))
    }

    fn external_source(&mut self) -> Option<ExternalSource> {
        self.src.take()
    }
}

/// A data-triggered consumer of `cam/out`. Its listener keeps the reactor's
/// source list non-empty, so a wait with no ready source really blocks.
#[cerulion_node]
#[derive(Default)]
struct Viewer {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl Viewer {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

fn output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn graph(fd: RawFd, fires: Arc<AtomicU64>) -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "fatal_ext_wake".to_string(),
        prefix: "fatalextwake".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cam".to_string(),
                node_type: "dead_camera".to_string(),
                inputs: vec![],
                outputs: vec![output("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "viewer".to_string(),
                node_type: "viewer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "cam/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "cam".to_string(),
        Box::new(DeadCamera {
            src: Some(ExternalSource::Fd(fd)),
            fires,
        }),
    );
    factories.insert(
        "viewer".to_string(),
        Box::new(ViewerEntry::with_state(Viewer::default())),
    );
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("build the dead-camera graph")
}

const WAIT: Duration = Duration::from_millis(200);

#[test]
#[serial]
fn a_fatally_stopped_external_node_no_longer_wakes_the_live_wait() {
    let fd = HungUpFd::new();
    let fires = Arc::new(AtomicU64::new(0));
    let mut runtime = graph(fd.0, Arc::clone(&fires));
    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 1, "one device binding");

    // Drain first: a fresh listener may hold queued notifications (the
    // viewer's), and a fired listener is drained by the wait that saw it. The
    // ready fd makes this wait return at once; what it names is not judged.
    let _ = runtime.run_waitset_reactor_once_for_test(WAIT);

    // Control: the hung-up fd is ready, so the wait returns at once on it.
    let start = Instant::now();
    let woke = runtime.run_waitset_reactor_once_for_test(WAIT);
    assert_eq!(
        woke,
        vec!["cam".to_string()],
        "the hung-up fd wakes the wait"
    );
    assert!(
        start.elapsed() < WAIT,
        "a ready fd must not wait out the timeout: {:?}",
        start.elapsed()
    );

    // The fatal fire: the sweep marks `cam`, it fires once and is stopped.
    runtime.run_live_step_once_for_test(Duration::from_millis(100));
    assert_eq!(fires.load(Ordering::Relaxed), 1, "the fatal fire ran once");
    assert_eq!(
        runtime.node_fatal_errors(),
        vec![("cam".to_string(), "Fatal error: camera lost".to_string())],
        "the stop is reported once, with the node's message"
    );

    // After the stop the same fd is still hung up, but it no longer wakes the
    // wait: nothing fires, and the wait lasts its whole timeout.
    let start = Instant::now();
    let woke = runtime.run_waitset_reactor_once_for_test(WAIT);
    let waited = start.elapsed();
    assert!(
        woke.is_empty(),
        "a stopped node's fd must not wake: {woke:?}"
    );
    assert!(
        waited >= WAIT - Duration::from_millis(20),
        "the wait must block, not return on the stopped node's fd: {waited:?}"
    );

    // The live loop itself does not spin: three steps take three timeouts.
    let step = Duration::from_millis(100);
    let start = Instant::now();
    for _ in 0..3 {
        runtime.run_live_step_once_for_test(step);
    }
    let elapsed = start.elapsed();
    assert!(
        elapsed >= 3 * step - Duration::from_millis(30),
        "three live steps returned in {elapsed:?}: the stopped node's fd is spinning the loop"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "a stopped node never fires again"
    );
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the binding is kept, so a reset that re-enables the node restores the wake"
    );
}
