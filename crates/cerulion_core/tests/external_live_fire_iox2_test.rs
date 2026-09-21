// SPDX-License-Identifier: AGPL-3.0-only
//! The FIRE SEAM — an `#[cerulion_node(external)]` ingress
//! (driver) node SELF-TRIGGERS off its [`ExternalSource`] over real iceoryx2.
//!
//! The WaitSet reactor accepts raw-fd wake sources. This file covers
//! the scheduler side: `GraphRuntime::collect_external_sources` (run_live
//! entry) queries each External-policy node's `external_source()` once and builds
//! a wake binding; `sweep_external_sources` (run each `live_step`, BEFORE
//! `step()`) transcribes CURRENT readiness into a `Scheduler::trigger_external`
//! mark; the EXISTING deterministic `TriggerPolicy::External` arm inside `step()`
//! makes the actual fire decision (record-only wake — Principle #7).
//!
//! These tests drive the production live seam
//! (`run_live_step_once_for_test`, which calls the same `live_step` the spawned
//! `run_live` loop does) after priming the bindings via
//! `collect_external_sources_for_test()` — the production order (run_live
//! collects, THEN loops).
//!
//! Also pinned here: the park-active sweep pin (the no-reactor idle
//! path), Notified ring-coalescing, collect-time invalid-fd rejection, the
//! run_live-exit-keeps-helpers-alive resume regression (helpers
//! stop at `GraphRuntime::drop`, not at `run_live` exit), the mid-block fd wake
//! proof, and `#[traced_test]` pins on every loud degrade path (the repo's
//! established tracing-test/no-env-filter pattern).
//!
//! No fake data (Principle #13): every wake is a real pipe byte, a real mpsc
//! send, or a real published iceoryx2 frame; every oracle is a hand value, never
//! a self-compare. `#[serial]` — real iceoryx2 over the process-global SHM
//! singleton (per-test SHM root via `build_for_test`).

use std::os::unix::io::RawFd;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::MacroPolicy;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::MonitorWaitPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A `libc::pipe` with RAII cleanup; the read end is set NON-BLOCKING so a
/// draining `read(2)` in `tick()` never blocks when the pipe empties.
struct Pipe {
    read: RawFd,
    write: RawFd,
}

impl Pipe {
    fn new() -> Pipe {
        let mut fds = [0 as libc::c_int; 2];
        // SAFETY: `fds` is a 2-element array; `pipe` writes exactly two fds.
        let rc = unsafe { libc::pipe(fds.as_mut_ptr()) };
        assert_eq!(rc, 0, "libc::pipe must succeed");
        // Non-blocking read end: a `read(2)` on an empty pipe returns -1/EAGAIN
        // rather than blocking the node tick's drain loop.
        // SAFETY: `fds[0]` is a valid open fd just returned by pipe().
        let flags = unsafe { libc::fcntl(fds[0], libc::F_GETFL) };
        assert!(flags >= 0, "F_GETFL must succeed");
        // SAFETY: same valid fd; set O_NONBLOCK on the read end.
        let rc = unsafe { libc::fcntl(fds[0], libc::F_SETFL, flags | libc::O_NONBLOCK) };
        assert_eq!(rc, 0, "F_SETFL O_NONBLOCK must succeed");
        Pipe {
            read: fds[0],
            write: fds[1],
        }
    }

    fn write_byte(&self) {
        let b: u8 = 1;
        // SAFETY: `self.write` is a valid open fd; one byte from a live buffer.
        let n = unsafe { libc::write(self.write, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
        assert_eq!(n, 1, "pipe write must succeed");
    }

    /// Relinquish the READ fd so `Drop` won't close it (used when a test closes
    /// it out of band to model a dead fd).
    fn take_read(&mut self) -> RawFd {
        let r = self.read;
        self.read = -1;
        r
    }
}

impl Drop for Pipe {
    fn drop(&mut self) {
        // SAFETY: each is a fd this Pipe opened and has not relinquished; closed
        // at most once.
        if self.read >= 0 {
            unsafe { libc::close(self.read) };
        }
        if self.write >= 0 {
            unsafe { libc::close(self.write) };
        }
    }
}

/// A hand-written `external`-policy ingress (driver) node. `info()` declares
/// `MacroPolicy::External` + one `Vector3` output `out`; `external_source()`
/// hands the runtime the source ONCE (via `Option::take`); `tick()` counts fires,
/// optionally drains `drain_fd`, and publishes an incrementing value.
struct ExtProducer {
    /// Handed to the runtime by `external_source()` exactly once.
    src: Option<ExternalSource>,
    ctx: Option<NodeContext>,
    fires: Arc<AtomicU64>,
    next_val: f64,
    /// When `Some`, `tick()` drains this fd (single non-blocking drain-to-empty)
    /// so a level-triggered device stops signalling once serviced.
    drain_fd: Option<RawFd>,
    /// Bumped on every `external_source()` call — the collect-idempotency
    /// oracle (a runtime must query the source exactly ONCE per lifetime).
    src_queries: Arc<AtomicU64>,
    /// When true, `external_source_is_drained_doorbell_fd()`
    /// reports `true` — modelling a cdylib tier-2 `Blocking`-collapse pipe read
    /// end (an OWNED doorbell the host drains + closes), so collect takes the
    /// `is_doorbell` sub-arm of each verdict. Default `false` (a poll-only device
    /// fd the host never closes — every other test's behavior is unchanged).
    is_doorbell: bool,
}

impl ExtProducer {
    fn new(src: ExternalSource, fires: Arc<AtomicU64>, drain_fd: Option<RawFd>) -> Self {
        Self {
            src: Some(src),
            ctx: None,
            fires,
            next_val: 0.0,
            drain_fd,
            src_queries: Arc::new(AtomicU64::new(0)),
            is_doorbell: false,
        }
    }

    /// Attach a shared `external_source()`-query counter (the idempotency pin).
    fn with_query_counter(mut self, counter: Arc<AtomicU64>) -> Self {
        self.src_queries = counter;
        self
    }

    /// Report the returned `Fd` as a DRAINED DOORBELL read end (the
    /// `#[doc(hidden)]` cdylib-only marker) so collect binds/refuses it via the
    /// `is_doorbell` sub-arm — the ONLY arm that CLOSES a refused live fd.
    fn with_doorbell_marker(mut self) -> Self {
        self.is_doorbell = true;
        self
    }
}

impl NodeEntry for ExtProducer {
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

    fn init(&mut self, ctx: NodeContext) -> TransportResult<()> {
        self.ctx = Some(ctx);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        if let Some(fd) = self.drain_fd {
            // Drain the (non-blocking) fd to empty so it stops signalling.
            let mut buf = [0u8; 256];
            loop {
                // SAFETY: `fd` is a valid non-blocking fd; read at most buf.len()
                // bytes into a live buffer. <=0 (EOF / EAGAIN) ends the drain.
                let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
                if n <= 0 {
                    break;
                }
            }
        }
        self.next_val += 1.0;
        let val = self.next_val;
        let ctx = self.ctx.as_mut().expect("ExtProducer init() ran");
        let pubr = ctx.publisher_mut("out").expect("out publisher wired");
        let mut proxy = pubr.loan_proxy::<Vector3>()?;
        proxy.x = val;
        drop(proxy); // publish
        Ok(())
    }

    fn external_source(&mut self) -> Option<ExternalSource> {
        self.src_queries.fetch_add(1, Ordering::AcqRel);
        self.src.take()
    }

    fn external_source_is_drained_doorbell_fd(&self) -> bool {
        self.is_doorbell
    }
}

/// Data-trigger consumer recording the last received `x` (delivery oracle) and a
/// fire count.
#[cerulion_node]
#[derive(Default)]
struct RecvConsumer {
    #[input(trigger)]
    inp: Vector3,
    last: Arc<AtomicU64>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl RecvConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        self.last.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Graph parts: one `ExtProducer` (`producer`) publishing `producer/out`, one
/// data-trigger `RecvConsumer` (`consumer`) on it.
fn producer_consumer_parts(
    producer: ExtProducer,
    consumer_last: Arc<AtomicU64>,
    consumer_fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_live_fire".to_string(),
        prefix: "extlf".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "ext_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "recv_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    let consumer = RecvConsumer {
        last: consumer_last,
        fires: consumer_fires,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(RecvConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Build the producer→consumer graph over an isolated per-test SHM root.
fn producer_consumer_graph(
    producer: ExtProducer,
    consumer_last: Arc<AtomicU64>,
    consumer_fires: Arc<AtomicU64>,
) -> GraphRuntime {
    let (config, factories) = producer_consumer_parts(producer, consumer_last, consumer_fires);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ext live-fire graph")
}

/// Like [`producer_consumer_graph`] but built with a FORCED
/// [`MonitorWaitPolicy`] (the monitor-wait park), via the LIVE build
/// path — the park-active sweep pin's builder.
fn producer_consumer_graph_with_policy(
    producer: ExtProducer,
    consumer_last: Arc<AtomicU64>,
    consumer_fires: Arc<AtomicU64>,
    policy: MonitorWaitPolicy,
) -> GraphRuntime {
    let (config, factories) = producer_consumer_parts(producer, consumer_last, consumer_fires);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
        .expect("build ext live-fire graph under monitor-wait policy")
}

/// A producer-only NodeDef (an `ExtProducer` with output `out`, no consumer).
fn producer_only_node(id: &str) -> NodeDef {
    NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: "ext_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// Drive the live seam until `pred` holds or `deadline` elapses; returns whether
/// `pred` held. A short per-iteration timeout keeps the loop responsive.
fn step_until(
    runtime: &mut GraphRuntime,
    deadline: Duration,
    mut pred: impl FnMut() -> bool,
) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if pred() {
            return true;
        }
        runtime.run_live_step_once_for_test(Duration::from_millis(100));
    }
    pred()
}

/// Poll `pred` until true or `deadline` elapses (for observing a helper thread's
/// async effect, e.g. poison / exit).
fn wait_until(deadline: Duration, mut pred: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while start.elapsed() < deadline {
        if pred() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    pred()
}

// ---------------------------------------------------------------------------
// 1. An fd source fires the external node on the live seam + delivers downstream.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn external_fd_node_fires_on_live_seam() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read), // drain in tick
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the Fd external source must produce exactly one binding"
    );

    // Make the device readable, then drive ONE live step.
    pipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));

    // Hand oracle: the readable fd marks the producer → it fires ONCE and
    // publishes value 1.0 → the data-trigger consumer receives 1 the same step.
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "external Fd node fires exactly once on the readable pipe"
    );
    assert_eq!(
        c_fires.load(Ordering::Relaxed),
        1,
        "the downstream data-trigger consumer fires once (producer→consumer collapse)"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the consumer received the producer's published value (1)"
    );
}

// ---------------------------------------------------------------------------
// 2. Multiple wakes before a step coalesce to ONE fire; drained → no re-fire.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn multiple_wakes_coalesce_to_one_fire_per_step() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read), // tick drains the whole pipe
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();

    // FIVE readiness "events" before a single step.
    for _ in 0..5 {
        pipe.write_byte();
    }
    runtime.run_live_step_once_for_test(Duration::from_millis(500));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "5 pending bytes coalesce to exactly ONE fire (External is an idempotent bool)"
    );

    // Tick drained the pipe → a second step sees no readiness → no re-fire.
    runtime.run_live_step_once_for_test(Duration::from_millis(50));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "after the drain, a step with no fresh bytes must NOT re-fire"
    );
}

// ---------------------------------------------------------------------------
// 3. Level-trigger: an fd left readable (tick does NOT drain) re-fires next step.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_left_readable_refires_next_step() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        None, // NO drain → the fd stays readable
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();

    pipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "first fire on the readable fd"
    );

    // The byte was never drained → still readable → level-triggered re-fire.
    runtime.run_live_step_once_for_test(Duration::from_millis(500));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        2,
        "an undrained (still-readable) fd re-fires the node on the next step (level-trigger)"
    );
}

// ---------------------------------------------------------------------------
// 4. A Blocking source fires on its event, and its helper thread stops on exit.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn blocking_source_fires_and_stops() {
    /// Flips a shared flag on Drop — proves the helper closure (and thus the
    /// thread) was dropped after `stop`.
    struct ExitSignal(Arc<AtomicBool>);
    impl Drop for ExitSignal {
        fn drop(&mut self) {
            self.0.store(true, Ordering::Release);
        }
    }

    let (tx, rx) = mpsc::channel::<()>();
    let exited = Arc::new(AtomicBool::new(false));
    let sig = ExitSignal(Arc::clone(&exited));
    let closure = move || {
        // Keep `sig` owned by the closure; its Drop (on thread exit) flips
        // `exited`. Bounded recv so the thread observes `stop` between calls.
        let _sig = &sig;
        matches!(rx.recv_timeout(Duration::from_millis(50)), Ok(()))
    };

    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Blocking(Box::new(closure)),
        Arc::clone(&p_fires),
        None,
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the Blocking source must produce exactly one (doorbell) binding"
    );

    // Ring the doorbell once, then drive the live seam until the fire lands.
    tx.send(()).expect("send blocking event");
    let fired = step_until(&mut runtime, Duration::from_secs(3), || {
        c_fires.load(Ordering::Relaxed) >= 1
    });
    assert!(
        fired,
        "the Blocking event must fire the external node within the deadline"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "the blocking event fires the producer exactly once"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the consumer received the producer's published value (1)"
    );

    // Stop MECHANISM pin via the test seam (mirrors `GraphRuntime::drop`'s
    // helper-stop — the production stop lives off the `run_live`
    // exit, in Drop; the Drop location itself is pinned end-to-end by
    // `run_live_exit_keeps_blocking_helpers_alive_for_resume`). The detached
    // helper observes `stop` after its bounded recv and exits, dropping the
    // closure → `exited`.
    runtime.stop_external_sources_for_test();
    assert!(
        wait_until(Duration::from_secs(2), || exited.load(Ordering::Acquire)),
        "the detached Blocking helper thread must observe stop and exit (no join)"
    );
}

// ---------------------------------------------------------------------------
// 5. A panicking Blocking closure is contained + poisoned loudly; node never fires.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn blocking_panic_is_contained_and_loud() {
    let closure = move || -> bool { panic!("external blocking source boom") };
    let p_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_panic".to_string(),
        prefix: "extpan".to_string(),
        nodes: vec![producer_only_node("producer")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Blocking(Box::new(closure)),
            Arc::clone(&p_fires),
            None,
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build panic graph");

    runtime.collect_external_sources_for_test();
    // The helper runs the closure once → panics → caught → poisoned (async).
    assert!(
        wait_until(Duration::from_secs(2), || runtime
            .external_binding_poisoned_for_test("producer")
            == Some(true)),
        "a panicking Blocking closure must poison its source (caught, not aborting the process)"
    );

    // A poisoned source never marks the node → it never fires. Process is alive
    // (we reached here), proving containment.
    runtime.run_live_step_once_for_test(Duration::from_millis(100));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        0,
        "a poisoned Blocking source must never fire the node"
    );
}

// ---------------------------------------------------------------------------
// 6. HostDriven / None external nodes are REFUSED on the LIVE
// path (run_live returns Err naming the node + stating the fix) but STILL fire
// on the POLLED path (trigger_external + step) — the back-compat contract. The
// two arms log at error! with distinct wording so an operator can
// tell an explicit HostDriven from a forgotten external_source() override.
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn host_driven_and_none_refused_on_live_but_fire_when_polled() {
    for (label, is_none) in [("hostdriven", false), ("none", true)] {
        // ---- LIVE path: run_live REFUSES (host-driven can never fire while the
        // live loop owns the runtime). `running = false` so collect runs at
        // entry, returns the refusal Err, and the loop body never executes.
        let p_fires = Arc::new(AtomicU64::new(0));
        let mut producer = ExtProducer::new(ExternalSource::HostDriven, Arc::clone(&p_fires), None);
        if is_none {
            producer.src = None; // external_source() → None (no override)
        }
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: format!("ext_hd_live_{label}"),
            prefix: format!("exthdl{label}"),
            nodes: vec![producer_only_node("producer")],
        };
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("producer".to_string(), Box::new(producer));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("build host-driven live graph");

        let running = AtomicBool::new(false);
        let err = runtime.run_live(&running).expect_err(&format!(
            "{label}: the live run MUST be refused for a host-driven external node"
        ));
        // The reason distinguishes an explicit HostDriven from a
        // forgotten-override None.
        let want_reason = if is_none {
            cerulion_core::InertReason::NoSource
        } else {
            cerulion_core::InertReason::HostDriven
        };
        match err {
            cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
                // Exact (node, reason) pair — kills the prefix substring
                // false-positive AND pins the per-arm reason.
                assert_eq!(
                    nodes.as_slice(),
                    [("producer".to_string(), want_reason)],
                    "{label}: the refusal must name EXACTLY the offending node + reason; got {nodes:?}"
                );
            }
            other => panic!("{label}: expected ExternalNodesInertAtLaunch, got {other:?}"),
        }
        let msg = err.to_string();
        assert!(
            msg.contains("trigger_external") && msg.contains("step()"),
            "{label}: the refusal must state the FIX (a real source, or the polled step() \
             seam that can call trigger_external); got: {msg}"
        );
        assert_eq!(
            p_fires.load(Ordering::Relaxed),
            0,
            "{label}: a refused live run never fires the node"
        );
        // A refused HostDriven (or None) node materializes NO
        // external binding — assert on the runtime AFTER the refusal Err, BEFORE
        // drop (the refusal is sticky, bindings unchanged).
        assert_eq!(
            runtime.external_binding_count(),
            0,
            "{label}: a host-driven / None external node produces no external binding"
        );
        drop(runtime);

        // ---- POLLED path: back-compat, the node STILL fires via a host
        // trigger_external + step (the polled host owns stepping and CAN trigger;
        // collect / external_source() is never touched here).
        let p2_fires = Arc::new(AtomicU64::new(0));
        let mut producer2 =
            ExtProducer::new(ExternalSource::HostDriven, Arc::clone(&p2_fires), None);
        if is_none {
            producer2.src = None;
        }
        let config2 = GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: format!("ext_hd_polled_{label}"),
            prefix: format!("exthdp{label}"),
            nodes: vec![producer_only_node("producer")],
        };
        let mut factories2: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories2.insert("producer".to_string(), Box::new(producer2));
        let clock2 = Arc::new(VirtualClock::new());
        let mut polled = GraphRuntime::build_for_test(config2, factories2, clock2, 8)
            .expect("build host-driven polled graph");
        polled
            .trigger_external("producer")
            .expect("trigger_external");
        polled.step(Duration::from_millis(1));
        assert_eq!(
            p2_fires.load(Ordering::Relaxed),
            1,
            "{label}: a host-driven external node still fires via trigger_external + polled step \
             (back-compat — the polled seam is never refused)"
        );
    }

    // Log pin: the two live arms log at error! with DISTINCT
    // wording — exactly one HostDriven error and one None error, so an operator
    // can tell "explicitly declared host-driven" from "forgot the
    // external_source override". Fix 4 tightens this from a phrase-only count to
    // a SEVERITY pin: each captured line is asserted to carry the ERROR level
    // token, so a silent revert to warn!/info! fails here instead of passing.
    // (tracing-test's captured lines are the full-format fmt output, which
    // renders the level as an uppercase `ERROR`/`WARN` token; the arm messages
    // contain only a lowercase `error)`, so `contains("ERROR")` matches the
    // level token alone.)
    logs_assert(|lines: &[&str]| {
        let host_driven: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("external node declared HostDriven"))
            .collect();
        let none: Vec<&str> = lines
            .iter()
            .copied()
            .filter(|l| l.contains("returned no ExternalSource"))
            .collect();
        if host_driven.len() != 1 || none.len() != 1 {
            return Err(format!(
                "expected exactly one HostDriven error and one None error \
                 (distinct wording per arm); got host_driven={} none={}",
                host_driven.len(),
                none.len()
            ));
        }
        for l in host_driven.iter().chain(none.iter()) {
            if !l.contains("ERROR") {
                return Err(format!(
                    "inert-arm log lines must be at ERROR level (refusal fix 3); got: {l}"
                ));
            }
        }
        Ok(())
    });
}

// ---------------------------------------------------------------------------
// 7. Two external nodes returning the SAME raw fd → LAUNCH REFUSAL: the
// duplicate (p2) is named as the sole offender (reason `duplicate fd`), the
// collect pass eagerly tears down EVERY binding (zero bindings remain), and the
// error names BOTH nodes (the operator needs to know which pair collided).
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn duplicate_fd_rejected_loudly() {
    let pipe = Pipe::new();
    let p1_fires = Arc::new(AtomicU64::new(0));
    let p2_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_dup_fd".to_string(),
        prefix: "extdup".to_string(),
        nodes: vec![producer_only_node("p1"), producer_only_node("p2")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "p1".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(pipe.read),
            Arc::clone(&p1_fires),
            None, // do NOT drain — p1 stays fireable but shares the fd
        )),
    );
    factories.insert(
        "p2".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(pipe.read), // SAME raw fd as p1 → rejected
            Arc::clone(&p2_fires),
            None,
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build dup-fd graph");

    // A duplicate raw fd is a LAUNCH REFUSAL — the DUPLICATE (p2) is the
    // offender (the incumbent p1 keeps the descriptor, no-close semantics). The
    // collect pass then tears down every binding, so the whole run is refused.
    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("a duplicate raw fd must REFUSE the live run at collect");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [("p2".to_string(), cerulion_core::InertReason::DuplicateFd)],
                "the DUPLICATE (p2) is the sole named offender, reason duplicate fd; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the refusal EAGERLY tears down every binding (incl. the valid incumbent p1)"
    );

    // No-close semantics: the incumbent's shared device fd
    // must remain open after the refusal + teardown. `ExtProducer` reports
    // `is_doorbell == false`, so p1's incumbent binding is a NON-OWNING `DeviceFd`
    // — a device fd is the node's, never the host's to close. The dup arm skips
    // WITHOUT closing (closing `raw` would kill the incumbent's live descriptor
    // and double-close via its Drop), and the eager teardown only closes OWNED
    // doorbell read ends. So `pipe.read` (still owned by the live `Pipe`) is open.
    // A mutation that closes the shared fd in the dup arm or the teardown makes
    // this `fcntl(F_GETFD)` return -1.
    let still_open = unsafe { libc::fcntl(pipe.read, libc::F_GETFD) };
    assert_ne!(
        still_open,
        -1,
        "the incumbent's shared device fd must stay OPEN post-refusal (no-close); errno={}",
        std::io::Error::last_os_error()
    );

    // Log pin (refusal error-level): the dup-fd
    // error must name both nodes in one event (node_id = the skipped p2,
    // other_node = the incumbent p1). The message says the fd "aliases an
    // already-bound" fd (the skip is WITHOUT closing — a device fd is the node's,
    // and even a dup doorbell fd IS the incumbent's live descriptor).
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("aliases an already-bound") && l.contains("p1") && l.contains("p2"))
        {
            Ok(())
        } else {
            Err("expected an fd-aliases-already-bound error naming BOTH p1 and p2".to_string())
        }
    });

    // Neither node ever fires — the run was refused before any live step.
    assert_eq!(
        p1_fires.load(Ordering::Relaxed),
        0,
        "p1 never fires — the mixed dup-fd graph is refused at launch"
    );
    assert_eq!(
        p2_fires.load(Ordering::Relaxed),
        0,
        "p2 (the rejected duplicate) never fires"
    );
}

// ---------------------------------------------------------------------------
// 7b. A poisoned node entry mutex is a launch refusal.
// The poisoned-entry arm changed from warn-and-continue (earlier) to
// aggregated refusal; without this test its `inert_nodes.push((.., PoisonedEntry))`
// is deletable undetected. We poison the node's `Arc<Mutex<Box<dyn NodeEntry>>>`
// on a helper thread (lock + panic), then drive the collect seam and assert the
// aggregated Err names `(node, PoisonedEntry)`. Drop is poison-safe
// (`shutdown_all_nodes` matches on `lock()`), so no double-panic on teardown.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn poisoned_entry_refuses_the_run() {
    let fires = Arc::new(AtomicU64::new(0));
    // The source is never queried (the poisoned `lock()` fails first), so its
    // kind is irrelevant — HostDriven keeps the fixture trivial.
    let producer = ExtProducer::new(ExternalSource::HostDriven, Arc::clone(&fires), None);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_poison".to_string(),
        prefix: "extpsn".to_string(),
        nodes: vec![producer_only_node("cam")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("cam".to_string(), Box::new(producer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build poison graph");

    // Poison the node's entry mutex: lock it on a helper thread that then panics
    // while holding the guard (std poisons the mutex on a guard-holding panic).
    let entry = runtime
        .node_entry_arc_for_test("cam")
        .expect("cam node exists in the runtime node map");
    let poisoner = std::thread::spawn(move || {
        let _guard = entry.lock().expect("first lock of a fresh mutex succeeds");
        panic!("intentional panic to POISON the cam entry mutex");
    });
    assert!(
        poisoner.join().is_err(),
        "the poisoner thread must panic (poisoning the mutex)"
    );

    // Collect now → the poisoned `entry.lock()` returns Err → the node is the
    // sole aggregated offender with reason PoisonedEntry.
    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("a poisoned entry mutex must REFUSE the live run at collect");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [("cam".to_string(), cerulion_core::InertReason::PoisonedEntry)],
                "the poisoned node is the sole offender, reason PoisonedEntry; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the poisoned node never fires — the run was refused at collect"
    );
}

// ---------------------------------------------------------------------------
// 8. A DeviceFd closed out of band → sweep unbinds loudly; other bindings live.
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn pollnval_unbinds_loudly() {
    let pipe_b = Pipe::new();
    // Source pipe whose read end we dup onto a HIGH fd number. Closing a LOW fd
    // frees its number, which the reactor's WaitSet build then REUSES (making
    // the "closed" number valid again — the documented fd-reuse hazard). A high
    // number (well above the reactor's handful of low fds) stays closed, so the
    // sweep's `poll(2)` deterministically sees POLLNVAL.
    let src_pipe = Pipe::new();
    let high_fd: RawFd = 950; // < FD_SETSIZE (1024) so FdSource accepts it
                              // SAFETY: F_GETFD is a read-only liveness probe; the high fd must be FREE so
                              // dup2 below does not silently clobber a live fd.
    assert_eq!(
        unsafe { libc::fcntl(high_fd, libc::F_GETFD) },
        -1,
        "high fd must be free before dup2"
    );
    // SAFETY: `src_pipe.read` is a valid open fd; `high_fd` is a free in-range fd
    // number for dup2. Closed explicitly below.
    let rc = unsafe { libc::dup2(src_pipe.read, high_fd) };
    assert_eq!(rc, high_fd, "dup2 to the high fd must succeed");

    let a_fires = Arc::new(AtomicU64::new(0));
    let b_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_pollnval".to_string(),
        prefix: "extnval".to_string(),
        nodes: vec![producer_only_node("pa"), producer_only_node("pb")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "pa".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(high_fd),
            Arc::clone(&a_fires),
            None,
        )),
    );
    factories.insert(
        "pb".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(pipe_b.read),
            Arc::clone(&b_fires),
            None,
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build pollnval graph");

    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        2,
        "both fd sources bind before the out-of-band close"
    );

    // Close pa's (high) fd OUT OF BAND. The FdSource over it is non-owning, so
    // this is the only owner-close; `src_pipe.read` stays open.
    // SAFETY: sole close of the high dup fd we created above.
    unsafe { libc::close(high_fd) };

    // One live step: the sweep polls pa (POLLNVAL) → unbind; pb stays.
    runtime.run_live_step_once_for_test(Duration::from_millis(50));
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the POLLNVAL (closed) fd is unbound; the other (pb) binding survives — \
         and the process did NOT abort (we reached this assert)"
    );

    // Log pin: the unbind is loud and names the node.
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("POLLNVAL") && l.contains("pa"))
        {
            Ok(())
        } else {
            Err("expected a POLLNVAL unbind error naming node pa".to_string())
        }
    });
}

// ---------------------------------------------------------------------------
// 9. Zero external nodes → empty collection, sweep early-returns, live path OK.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn zero_external_bindings_zero_cost() {
    // A plain data-trigger consumer on an ABSOLUTE external topic — no
    // External-POLICY node in the graph.
    const EXT_TOPIC: &str = "/extz/ext/cam";
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_zero".to_string(),
        prefix: "extz".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "consumer".to_string(),
            node_type: "recv_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(RecvConsumerEntry::with_state(RecvConsumer {
            last: Arc::clone(&c_last),
            fires: Arc::clone(&c_fires),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build zero-ext graph");

    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "a graph with no external-policy node collects zero bindings (sweep early-returns)"
    );

    // The normal data-trigger live path is unaffected: an external publisher on
    // the absolute topic still delivers to the consumer.
    let mut pubr = {
        let mgr = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher attaches to the absolute topic")
    };
    {
        let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 7.0;
        drop(proxy);
    }
    let got = step_until(&mut runtime, Duration::from_secs(2), || {
        c_fires.load(Ordering::Relaxed) >= 1
    });
    assert!(
        got,
        "the zero-external live path still delivers a data-trigger fire"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        7,
        "delivery oracle: the consumer received the external publish (7) with zero external bindings"
    );
}

// ---------------------------------------------------------------------------
// 10. Park policy + fd source watches the fd.
//
// The degraded-tier external-fd exclusion is gone: the park now
// runs the NO-REACTOR idle path (`monitor_wait_block`) on BOTH tiers, and its
// recheck loop POLLS every external fd each recheck (`park_poll_fd_ready`), so a
// readable fd wakes the park at the recheck cadence (≤~100µs) on every tier —
// closing the ≤250ms sweep-cadence limp (~1.9Hz/hop) that primitive
// targets (the aarch64-WFE robot) used to suffer. collect warns ONCE about the
// recheck-cadence latency (both tiers now).
//
// The fire + delivery oracles are tier-independent; the park is now ACTIVE on
// BOTH tiers (a nonzero park-entry count and the single park warn), so nothing
// branches on `monitor_wait_available()` any more.
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn park_policy_fd_source_watches_fd_and_fires() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read), // drain in tick
    );
    let mut runtime = producer_consumer_graph_with_policy(
        producer,
        Arc::clone(&c_last),
        Arc::clone(&c_fires),
        MonitorWaitPolicy::new(true, false, "extpark".into()),
    );

    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 1);
    // Log pin (tier-INDEPENDENT now): EXACTLY one park-vs-external warn
    // (recheck-cadence latency) and ZERO of the deleted "degraded park disabled"
    // routing info — the exclusion is gone, so the park is active on every tier.
    logs_assert(|lines: &[&str]| {
        let warns = lines
            .iter()
            .filter(|l| l.contains("monitor-wait park active with external source"))
            .count();
        let stale_infos = lines
            .iter()
            .filter(|l| l.contains("degraded park disabled: external fd sources"))
            .count();
        match (warns, stale_infos) {
            (1, 0) => Ok(()),
            _ => Err(format!(
                "log mismatch: park-warns={warns} (want 1), \
                 stale-degraded-infos={stale_infos} (want 0)"
            )),
        }
    });

    // Make the device readable, then drive ONE live step. On BOTH tiers the park
    // recheck loop polls the fd, sees it readable, and wakes — the sweep AFTER it
    // marks and the fire lands the same step.
    pipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));

    assert!(
        runtime.park_entry_count_for_test() > 0,
        "the live iteration routes its idle through monitor_wait_block \
         (the park path) on EVERY tier — the exclusion is gone"
    );
    assert!(
        runtime.park_wakes_external_fd_count_for_test() > 0,
        "attribution: the readable fd WOKE the park (not the timeout)"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "the readable fd fires the external node (tier-independent oracle)"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the downstream consumer received the value"
    );
}

// ---------------------------------------------------------------------------
// 10b. The HEADLINE lost-wakeup pin: an external fd wakes the park
// promptly (attribution `park_wakes_external_fd > 0`, `wakes_timeout == 0` for
// that fire — the mutation kill: reverting `park_poll_fd_ready` makes the fire
// arrive only via the recheck timeout), plus the no-false-wakes control (no
// doorbell write ⇒ `park_wakes_external_fd == 0` across M idle iterations).
//
// `CERULION_LIVE_SPIN_US=0` is defensive env hygiene (a prior test's spin budget
// must not skew the park counters). It is NOT load-bearing for fd routing:
// `spin_sources` skips `WaitSource::Fd` entirely, so an external fd is only
// ever observed by the park's `park_poll_fd_ready` (or the blocking WaitSet).
// ---------------------------------------------------------------------------

/// RAII guard pinning `CERULION_LIVE_SPIN_US=0` (spin disabled → all idle routes
/// through the park), restored on drop.
struct SpinDisabledGuard {
    prior: Option<String>,
}
impl SpinDisabledGuard {
    fn new() -> Self {
        let prior = std::env::var("CERULION_LIVE_SPIN_US").ok();
        std::env::set_var("CERULION_LIVE_SPIN_US", "0");
        Self { prior }
    }
}
impl Drop for SpinDisabledGuard {
    fn drop(&mut self) {
        match &self.prior {
            Some(v) => std::env::set_var("CERULION_LIVE_SPIN_US", v),
            None => std::env::remove_var("CERULION_LIVE_SPIN_US"),
        }
    }
}

#[test]
#[serial]
fn park_fd_wake_is_prompt_and_attributed() {
    let _spin = SpinDisabledGuard::new();
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read), // drain in tick
    );
    let mut runtime = producer_consumer_graph_with_policy(
        producer,
        Arc::clone(&c_last),
        Arc::clone(&c_fires),
        MonitorWaitPolicy::new(true, false, "t1park".into()),
    );
    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 1);

    // Snapshot the timeout wakes, ring the doorbell, drive ONE step. The park's
    // fd poll must wake it on THIS iteration (no timeout wait).
    let (_, _, _, timeout_before) = runtime.park_wake_counts();
    pipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));

    // The park was entered and WOKEN BY THE FD, and the fire + delivery landed.
    assert!(
        runtime.park_entry_count_for_test() > 0,
        "the idle must route through the park (spin disabled)"
    );
    assert!(
        runtime.park_wakes_external_fd_count_for_test() > 0,
        "the fd woke the park (park_wakes_external_fd > 0)"
    );
    let (_, _, _, timeout_after) = runtime.park_wake_counts();
    assert_eq!(
        timeout_after, timeout_before,
        "the fire arrived via the FD wake, NOT the recheck \
         timeout — reverting the park fd-poll makes this delta nonzero"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "the fd fires the external node exactly once"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the downstream consumer received value 1"
    );
}

#[test]
#[serial]
fn park_no_false_wakes_without_fd_readiness() {
    let _spin = SpinDisabledGuard::new();
    let pipe = Pipe::new(); // never written → the fd stays not-ready
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read),
    );
    let mut runtime = producer_consumer_graph_with_policy(
        producer,
        Arc::clone(&c_last),
        Arc::clone(&c_fires),
        MonitorWaitPolicy::new(true, false, "t1nfw".into()),
    );
    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 1);

    // Drive M idle iterations WITHOUT writing the pipe: the park polls the fd,
    // sees it not-ready, and times out each time — never a false fd wake.
    const M: u32 = 8;
    for _ in 0..M {
        runtime.run_live_step_once_for_test(Duration::from_millis(2));
    }
    assert!(
        runtime.park_entry_count_for_test() >= M as u64,
        "each idle iteration must enter the park"
    );
    assert_eq!(
        runtime.park_wakes_external_fd_count_for_test(),
        0,
        "no-false-wakes control: an un-written fd must NEVER wake the park"
    );
    let (_, _, _, timeout_wakes) = runtime.park_wake_counts();
    assert!(
        timeout_wakes > 0,
        "the idle park woke by TIMEOUT, not the fd (anti-tautology)"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        0,
        "no readiness ⇒ the external node never fires"
    );
}

// ---------------------------------------------------------------------------
// 10c. The firewall: the park fd-wake is RECORD-ONLY. park-ON (fd
// wakes) delivers a byte-identical sequence to park-OFF (WaitSet wakes), both
// equal to a HAND oracle `1.0..=N` — the wake changes WHEN step() runs, never
// WHAT fires (Principle #7).
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn park_fd_wake_is_record_only_firewall() {
    const N: u64 = 5;

    // Drive one producer→consumer graph through N ring+step iterations, returning
    // the delivered value sequence. `policy` selects park-ON vs park-OFF.
    fn run_seq(policy: MonitorWaitPolicy) -> Vec<u64> {
        let pipe = Pipe::new();
        let p_fires = Arc::new(AtomicU64::new(0));
        let c_last = Arc::new(AtomicU64::new(0));
        let c_fires = Arc::new(AtomicU64::new(0));
        let producer = ExtProducer::new(
            ExternalSource::Fd(pipe.read),
            Arc::clone(&p_fires),
            Some(pipe.read),
        );
        let (config, factories) =
            producer_consumer_parts(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
                .expect("build firewall graph");
        runtime.collect_external_sources_for_test();
        let mut seq = Vec::new();
        for _ in 0..N {
            pipe.write_byte();
            // Bounded drive: step until the delivered value advances (fd wake or
            // WaitSet wake or, worst case, the recheck timeout — all deliver).
            let want = seq.len() as u64 + 1;
            let got = step_until(&mut runtime, Duration::from_secs(2), || {
                c_last.load(Ordering::Relaxed) == want
            });
            assert!(got, "value {want} must be delivered within the deadline");
            seq.push(c_last.load(Ordering::Relaxed));
        }
        seq
    }

    let _spin = SpinDisabledGuard::new();
    let oracle: Vec<u64> = (1..=N).collect();
    let park_on = run_seq(MonitorWaitPolicy::new(true, false, "t3on".into()));
    let park_off = run_seq(MonitorWaitPolicy::off());

    assert_eq!(park_on, oracle, "park-ON fd-wake delivery == hand oracle");
    assert_eq!(park_off, oracle, "park-OFF WaitSet delivery == hand oracle");
    assert_eq!(
        park_on, park_off,
        "firewall: park fd-wake changes only WHEN step() runs, never WHAT fires"
    );
}

// ---------------------------------------------------------------------------
// 11. Notified ring-coalescing: a Blocking source that rings twice
// before one step fires the node exactly once (the rings-delta oracle), and a
// quiet step does not refire (the `last_seen_rings` cursor advanced).
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn notified_rings_coalesce_to_one_fire_and_cursor_advances() {
    // The closure returns `true` on its first TWO calls (two back-to-back rings),
    // then settles into a bounded false-wait. `calls` observes progress: when
    // call 3 has STARTED, rings 1 and 2 have both been fully processed (the
    // helper bumps `rings` before looping into the next call).
    let calls = Arc::new(AtomicU64::new(0));
    let calls_c = Arc::clone(&calls);
    let closure = move || {
        let c = calls_c.fetch_add(1, Ordering::AcqRel) + 1;
        if c <= 2 {
            true
        } else {
            std::thread::sleep(Duration::from_millis(20));
            false
        }
    };

    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Blocking(Box::new(closure)),
        Arc::clone(&p_fires),
        None,
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();

    // Both rings have already registered by the time the 3rd closure call starts.
    assert!(
        wait_until(Duration::from_secs(2), || calls.load(Ordering::Acquire)
            >= 3),
        "the helper must have processed both `true` returns (rings == 2)"
    );

    // ONE live step: the two rings coalesce to ONE mark → ONE fire.
    runtime.run_live_step_once_for_test(Duration::from_millis(100));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "two rings before one step coalesce to exactly ONE fire"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the single coalesced fire published value 1"
    );

    // Quiet step: the cursor advanced to the observed ring count, so no refire.
    runtime.run_live_step_once_for_test(Duration::from_millis(50));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "a quiet step must NOT refire (last_seen_rings cursor advanced)"
    );
}

// ---------------------------------------------------------------------------
// 12. Invalid fd at collect (launch refusal): `Fd(-1)` and a dead
// (closed) fd are rejected at construction time — each is a LAUNCH offender
// (reason invalid fd), so the collect pass refuses the whole run in ONE
// aggregated error naming BOTH, and (per the shipped eager-teardown) the valid
// sibling's binding is torn down too. Distinct from POLLNVAL-at-sweep (test 8),
// which catches an fd that dies AFTER a successful collect (mid-run — still
// loud-but-running, out of the launch-refusal scope).
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn invalid_fd_at_collect_refuses_naming_both_offenders() {
    // Open ALL fixture fds BEFORE the close below (fd numbers are reused
    // lowest-first; nothing may open an fd between the close and collect).
    let mut dead_pipe = Pipe::new();
    let good_pipe = Pipe::new();

    let bad_neg_fires = Arc::new(AtomicU64::new(0));
    let bad_dead_fires = Arc::new(AtomicU64::new(0));
    let good_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_bad_collect".to_string(),
        prefix: "extbad".to_string(),
        nodes: vec![
            producer_only_node("bad_neg"),
            producer_only_node("bad_dead"),
            producer_only_node("good"),
        ],
    };
    // The dead fd's NUMBER is captured now; it is closed after build (below), so
    // it is a genuinely-dead fd by the time collect probes it.
    let dead_fd = dead_pipe.take_read();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "bad_neg".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(-1),
            Arc::clone(&bad_neg_fires),
            None,
        )),
    );
    factories.insert(
        "bad_dead".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(dead_fd),
            Arc::clone(&bad_dead_fires),
            None,
        )),
    );
    factories.insert(
        "good".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(good_pipe.read),
            Arc::clone(&good_fires),
            Some(good_pipe.read),
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build invalid-fd-at-collect graph");

    // Kill the dead fd AFTER build, BEFORE collect (collect opens no fds for Fd
    // sources, so the number cannot be reused before it is probed).
    // SAFETY: sole close of the fd relinquished from `dead_pipe` above.
    unsafe { libc::close(dead_fd) };

    // Both invalid fds refuse the run in ONE aggregated error, naming
    // both offenders in declaration order (each reason invalid fd). The `good`
    // sibling bound first, then was torn down by the eager refusal cleanup.
    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("invalid fds must REFUSE the live run at collect");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [
                    ("bad_neg".to_string(), cerulion_core::InertReason::InvalidFd),
                    ("bad_dead".to_string(), cerulion_core::InertReason::InvalidFd),
                ],
                "both invalid fds are named (reason invalid fd) in declaration order; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the refusal tears down every binding, including the valid `good` sibling"
    );
    // Log pin: each rejected node is named in its own error.
    logs_assert(|lines: &[&str]| {
        let neg = lines
            .iter()
            .any(|l| l.contains("external Fd source rejected") && l.contains("bad_neg"));
        let dead = lines
            .iter()
            .any(|l| l.contains("external Fd source rejected") && l.contains("bad_dead"));
        if neg && dead {
            Ok(())
        } else {
            Err(format!(
                "expected reject errors naming bad_neg AND bad_dead; neg={neg} dead={dead}"
            ))
        }
    });

    // No node ever fires — the run was refused before any live step.
    assert_eq!(good_fires.load(Ordering::Relaxed), 0);
    assert_eq!(bad_neg_fires.load(Ordering::Relaxed), 0);
    assert_eq!(bad_dead_fires.load(Ordering::Relaxed), 0);
}

// ---------------------------------------------------------------------------
// 13. Resume regression: a `run_live` session end must not stop
// the Blocking helper threads — collection is one-shot, so a resumed session
// reuses the bindings and needs the helpers ALIVE. Helpers stop at
// `GraphRuntime::drop` (the second arm).
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn run_live_exit_keeps_blocking_helpers_alive_for_resume() {
    /// Flips a shared flag on Drop — observes the helper thread's exit.
    struct ExitSignal(Arc<AtomicBool>);
    impl Drop for ExitSignal {
        fn drop(&mut self) {
            self.0.store(true, Ordering::Release);
        }
    }

    let (tx, rx) = mpsc::channel::<()>();
    let exited = Arc::new(AtomicBool::new(false));
    let sig = ExitSignal(Arc::clone(&exited));
    let closure = move || {
        let _sig = &sig;
        matches!(rx.recv_timeout(Duration::from_millis(50)), Ok(()))
    };

    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Blocking(Box::new(closure)),
        Arc::clone(&p_fires),
        None,
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

    // SESSION 1: the REAL `run_live` entry + exit. `running = false` ⇒ collect
    // runs at entry (the production collection site — NOT the test seam), the
    // loop body never runs, and run_live returns. This exit must not stop the
    // helpers.
    let running = AtomicBool::new(false);
    runtime
        .run_live(&running)
        .expect("run_live with a Blocking source must not be refused (only host-driven is)");
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "run_live's entry collected the Blocking binding (production order)"
    );

    // Give a wrongly stopped helper ample time to observe the flag and exit —
    // its bounded recv is 50ms. Nothing stopped it, so it stays alive.
    std::thread::sleep(Duration::from_millis(200));
    assert!(
        !exited.load(Ordering::Acquire),
        "the helper must SURVIVE a run_live session end (a \
         resumed run_live reuses the one-shot bindings, so stopping at exit \
         would leave Blocking sources permanently inert)"
    );

    // SESSION 2 (resume): ring the doorbell — the helper must still be there to
    // receive it (a stopped helper's receiver is dropped and this send fails).
    tx.send(())
        .expect("helper thread must still be alive after a run_live session ends");
    let fired = step_until(&mut runtime, Duration::from_secs(3), || {
        c_fires.load(Ordering::Relaxed) >= 1
    });
    assert!(fired, "the resumed session's Blocking source still fires");
    assert_eq!(p_fires.load(Ordering::Relaxed), 1);
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the resumed fire published value 1"
    );

    // ARM 2 — the production stop location: dropping the runtime stops the
    // helper (also covers the leak-on-unwind case: Drop runs even when a
    // live-loop panic unwinds past run_live).
    drop(runtime);
    assert!(
        wait_until(Duration::from_secs(2), || exited.load(Ordering::Acquire)),
        "GraphRuntime::drop must stop the detached Blocking helper thread"
    );
}

// ---------------------------------------------------------------------------
// 14. Mid-block fd wake: the byte arrives while the live seam is
// BLOCKED on the WaitSet — the fd wake must unblock it (reactor attach proof,
// not just sweep-after-immediate-return) and the fire lands the same step.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn fd_written_mid_block_wakes_live_seam_and_fires() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read),
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();

    // Prime: one live iteration with NO event pending, so the build-time
    // connection-lifecycle notifications queued on the consumer's listener are
    // drained — otherwise the measured iteration below would wake INSTANTLY on
    // that stale noise (before the 50ms write) and fire nothing.
    runtime.run_live_step_once_for_test(Duration::from_millis(100));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        0,
        "prime iteration: no byte yet, no fire"
    );

    // The delayed "device interrupt": write one byte ~50ms after the seam
    // parks on the WaitSet. RawFd is Copy + Send; the Pipe outlives the join.
    let write_fd = pipe.write;
    let writer = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(50));
        let b: u8 = 1;
        // SAFETY: `write_fd` is the pipe's valid open write end (the Pipe is
        // alive in the test thread until after join); one byte from a live
        // buffer.
        let n = unsafe { libc::write(write_fd, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
        assert_eq!(n, 1, "delayed pipe write must succeed");
    });

    // Generous timeout: a wake-on-write returns in ~50ms; only a broken fd
    // attach would sit out the full 3s.
    let start = Instant::now();
    runtime.run_live_step_once_for_test(Duration::from_secs(3));
    let elapsed = start.elapsed();
    writer.join().expect("writer thread must not panic");

    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "the mid-block fd write fires the external node in the same live step"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the woken step's publish reached the consumer"
    );
    assert!(
        elapsed < Duration::from_secs(2),
        "the seam must WAKE on the fd event (~50ms), not sit out the 3s timeout; \
         elapsed = {elapsed:?}"
    );
}

// ---------------------------------------------------------------------------
// 15. Blocking doorbell with no TransportManager (launch refusal):
// the production `build` path parks no test transport, and this test binary never
// initializes the process-global singleton (init_for_test deliberately does NOT
// store it) — so the Blocking arm has no manager: it is a LAUNCH offender (reason
// doorbell failed), refused at collect, loud, no binding, no thread.
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn blocking_doorbell_without_transport_manager_refused_at_launch() {
    let p_fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_no_mgr".to_string(),
        prefix: "extnomgr".to_string(),
        nodes: vec![producer_only_node("producer")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Blocking(Box::new(|| false)),
            Arc::clone(&p_fires),
            None,
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    // The PRODUCTION build path (caller-owned manager, `test_transport` = None).
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "no_mgr_test".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        ix_config,
    )
    .expect("isolated transport manager");
    let mut runtime =
        GraphRuntime::build(config, factories, &mgr, clock).expect("build no-mgr graph");

    // The un-resolvable doorbell is a launch offender — collect refuses.
    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("a Blocking source with no resolvable TransportManager must REFUSE at collect");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [(
                    "producer".to_string(),
                    cerulion_core::InertReason::DoorbellFailed
                )],
                "the doorbell-failed producer is the sole named offender; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "with no parked test transport AND an uninitialized global singleton, the \
         Blocking doorbell cannot be minted — no binding"
    );
    // Log pin: the doorbell-failed arm is LOUD and names the node.
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("needs a TransportManager") && l.contains("producer"))
        {
            Ok(())
        } else {
            Err("expected a loud needs-a-TransportManager error naming the node".to_string())
        }
    });
    // Drop the runtime BEFORE the manager it borrows (ports before transport).
    drop(runtime);
    drop(mgr);
}

// ---------------------------------------------------------------------------
// 15b. The `live_transport`
// doorbell arm is LOAD-BEARING. A production `build` runtime (test_transport =
// None) with a parked `live_transport` resolves its Blocking-doorbell manager to
// THAT parked handle — not the process-global singleton. Both the production mint
// (`spawn_blocking_doorbell`) and this pin's `resolve_doorbell_transport_for_test`
// delegate to the ONE shared `resolve_doorbell_transport` resolver, so deleting
// its `.or_else(|| self.live_transport.clone())` arm makes the `Arc::ptr_eq`
// below FAIL — resolution falls through to a different / uninitialized singleton,
// so a `build_live` embedder driving a Blocking source would mint the doorbell on
// the wrong node.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn live_transport_arm_is_load_bearing_for_doorbell_resolution() {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_live_mgr".to_string(),
        prefix: "extlivemgr".to_string(),
        nodes: vec![producer_only_node("producer")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Blocking(Box::new(|| false)),
            Arc::new(AtomicU64::new(0)),
            None,
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    // PRODUCTION build path (caller-owned manager) → `test_transport` stays None,
    // so the resolver's first arm is empty and the `live_transport` arm decides.
    let mgr_build = TransportManager::init_for_test(
        TransportConfig {
            node_name: "live_mgr_build".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated build manager");
    let mut runtime = GraphRuntime::build(config, factories, &mgr_build, clock.clone())
        .expect("build live-mgr graph");

    // A DISTINCT isolated manager parked as the live transport. The ONLY way the
    // resolver can return THIS Arc is via the `live_transport` arm: a fresh
    // `init_for_test` manager is never the process-global singleton, and
    // `test_transport` is None on this production build.
    let mgr_live = TransportManager::init_for_test(
        TransportConfig {
            node_name: "live_mgr_parked".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated live manager");
    runtime.set_live_transport(Arc::clone(&mgr_live));

    let resolved = runtime
        .resolve_doorbell_transport_for_test()
        .expect("with a parked live_transport the doorbell manager resolves");
    assert!(
        Arc::ptr_eq(&resolved, &mgr_live),
        "the doorbell manager must resolve to the parked live_transport (the load-bearing arm), \
         not the process-global singleton"
    );

    // Drop ports before the managers they borrow.
    drop(runtime);
    drop(mgr_live);
    drop(mgr_build);
}

// ---------------------------------------------------------------------------
// 15c. `set_live_transport` after collect warns.
// A mis-ordered embedder that parks the live transport AFTER `run_live`'s entry
// collect gets a loud warning — its handle will not apply to already-collected
// sources (correct order: `set_live_transport` BEFORE the first `run_live`).
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn set_live_transport_after_collect_warns() {
    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_late_park".to_string(),
        prefix: "extlatepark".to_string(),
        nodes: vec![producer_only_node("producer")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(ExtProducer::new(
            ExternalSource::Fd(pipe.read),
            Arc::clone(&p_fires),
            Some(pipe.read),
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock.clone(), 8)
        .expect("build late-park graph");

    // Collect FIRST (the run_live-entry order): flips `external_sources_collected`.
    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the Fd source binds once before the (mis-ordered) set_live_transport"
    );

    // Now park a live transport — the mis-ordered call. It MUST warn.
    let mgr_late = TransportManager::init_for_test(
        TransportConfig {
            node_name: "late_park_mgr".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated late-park manager");
    runtime.set_live_transport(Arc::clone(&mgr_late));

    // Log pin: the FIX-4b warn fires, names the mis-order AND the fix.
    logs_assert(|lines: &[&str]| {
        if lines.iter().any(|l| {
            l.contains("set_live_transport called AFTER external sources were collected")
                && l.contains("BEFORE the first run_live")
        }) {
            Ok(())
        } else {
            Err(
                "expected the loud after-collect set_live_transport warn naming the fix"
                    .to_string(),
            )
        }
    });

    // Drop the runtime BEFORE the manager it borrows (ports before transport).
    drop(runtime);
    drop(mgr_late);
}

// ---------------------------------------------------------------------------
// 16. Collect idempotency: calling collect twice queries each node's
// `external_source()` exactly once, keeps the binding count unchanged, spawns no
// second helper, and the existing bindings stay functional. The second call
// emits the resume health summary.
// ---------------------------------------------------------------------------
#[test]
#[serial]
#[traced_test]
fn collect_twice_is_idempotent_and_reports_resume() {
    let pipe = Pipe::new();
    let fd_queries = Arc::new(AtomicU64::new(0));
    let blocking_queries = Arc::new(AtomicU64::new(0));
    let fd_fires = Arc::new(AtomicU64::new(0));
    let blocking_fires = Arc::new(AtomicU64::new(0));
    let (tx, rx) = mpsc::channel::<()>();
    let closure = move || matches!(rx.recv_timeout(Duration::from_millis(50)), Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_idem".to_string(),
        prefix: "extidem".to_string(),
        nodes: vec![
            producer_only_node("fd_node"),
            producer_only_node("bell_node"),
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "fd_node".to_string(),
        Box::new(
            ExtProducer::new(
                ExternalSource::Fd(pipe.read),
                Arc::clone(&fd_fires),
                Some(pipe.read),
            )
            .with_query_counter(Arc::clone(&fd_queries)),
        ),
    );
    factories.insert(
        "bell_node".to_string(),
        Box::new(
            ExtProducer::new(
                ExternalSource::Blocking(Box::new(closure)),
                Arc::clone(&blocking_fires),
                None,
            )
            .with_query_counter(Arc::clone(&blocking_queries)),
        ),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build idem graph");

    // Collect TWICE — the second must early-return (no re-query, no re-spawn).
    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 2);
    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        2,
        "a second collect must not change the binding count"
    );
    assert_eq!(
        fd_queries.load(Ordering::Acquire),
        1,
        "the fd node's external_source() is queried exactly ONCE across two collects"
    );
    assert_eq!(
        blocking_queries.load(Ordering::Acquire),
        1,
        "the Blocking node's external_source() is queried exactly ONCE across two \
         collects (a re-query would also re-spawn a second helper thread)"
    );
    // Log pin: the second collect reports the resume health summary
    // (info arm — nothing is poisoned here).
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("resuming external bindings"))
        {
            Ok(())
        } else {
            Err("expected the resume health summary on the second collect".to_string())
        }
    });

    // The ORIGINAL bindings stay functional after the second collect: the fd
    // fires its node, and the ONE (original) helper still rings the doorbell.
    pipe.write_byte();
    tx.send(()).expect("original helper still alive");
    let both = step_until(&mut runtime, Duration::from_secs(3), || {
        fd_fires.load(Ordering::Relaxed) >= 1 && blocking_fires.load(Ordering::Relaxed) >= 1
    });
    assert!(
        both,
        "both original bindings must still fire after the idempotent re-collect"
    );
}

// ---------------------------------------------------------------------------
// 17. WAKE-STORM (portable): a writer thread FLOODS the pipe
// continuously (tight best-effort loop, small yields) while the live seam steps
// M times. The External wake is an idempotent bool, so a storm of readiness can
// never explode the fire count: assertions are (i) the loop stays LIVE (every
// step completes within a generous bound — no busy-wedge), (ii) at most ONE fire
// per step, (iii) downstream delivery stays consistent with the node's
// drain-to-EAGAIN semantics (the delivered value is MONOTONIC non-decreasing and
// never exceeds the producer's fire count — a hand-modelable invariant, NOT a
// fragile exact count under a nondeterministic flood), and (iv) NO WEDGE after
// the storm stops: quiesce, then one more write yields EXACTLY one more fire that
// DELIVERS (Principle #6 — no lost wakeup on the tail). CI-robust: no
// wall-clock-tight windows; step counts and per-step bounds are generous.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn wake_storm_stays_live_one_fire_per_step_no_lost_tail() {
    const STORM_STEPS: u32 = 30;

    let pipe = Pipe::new();
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        Arc::clone(&p_fires),
        Some(pipe.read), // tick drains the pipe to EAGAIN every fire
    );
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
    runtime.collect_external_sources_for_test();
    assert_eq!(runtime.external_binding_count(), 1);

    // Make the WRITE end non-blocking so a full pipe never wedges the flooder
    // (a blocking write on a full pipe would deadlock once we stop stepping).
    // SAFETY: `pipe.write` is a valid open fd owned by the live `pipe`.
    let write_fd = pipe.write;
    let flags = unsafe { libc::fcntl(write_fd, libc::F_GETFL) };
    assert!(flags >= 0, "F_GETFL on the write end must succeed");
    // SAFETY: same valid fd; add O_NONBLOCK.
    let rc = unsafe { libc::fcntl(write_fd, libc::F_SETFL, flags | libc::O_NONBLOCK) };
    assert_eq!(rc, 0, "F_SETFL O_NONBLOCK on the write end must succeed");

    // The flooder: a NON-Cerulion thread hammering the pipe until told to stop.
    // Best-effort — a full-pipe EAGAIN is expected and simply retried; the point
    // is a CONTINUOUS readiness storm, not delivery of every byte.
    //
    // Stop-on-Drop guard: an assertion panic inside the step loop below must
    // still stop + reap the flooder (a leaked spinner would keep write(2)-ing a
    // stale fd number that later #[serial] tests in this binary may reuse).
    struct FlooderGuard {
        stop: Arc<AtomicBool>,
        handle: Option<std::thread::JoinHandle<()>>,
    }
    impl Drop for FlooderGuard {
        fn drop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(handle) = self.handle.take() {
                let _ = handle.join();
            }
        }
    }
    let stop = Arc::new(AtomicBool::new(false));
    let stop_w = Arc::clone(&stop);
    let flooder = std::thread::spawn(move || {
        let b: u8 = 1;
        while !stop_w.load(Ordering::Relaxed) {
            // SAFETY: `write_fd` (Copy RawFd) is the pipe's valid write end (the
            // Pipe outlives the guard's join); one byte from a live buffer. The
            // return is intentionally ignored — EAGAIN on a full pipe is normal.
            let _ =
                unsafe { libc::write(write_fd, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
            std::thread::yield_now();
        }
    });
    let flooder_guard = FlooderGuard {
        stop,
        handle: Some(flooder),
    };

    // Drive M live steps under the storm and check the invariants each step.
    let mut prev_p = 0u64;
    let mut prev_last = 0u64;
    for i in 0..STORM_STEPS {
        let t0 = Instant::now();
        runtime.run_live_step_once_for_test(Duration::from_millis(100));
        let dt = t0.elapsed();
        // (i) liveness: a readable-fd wake returns fast. NOTE: a true wedge
        // (blocked forever) would HANG the step rather than trip this assert —
        // the bound catches gross slow-path degradation; 10s is deliberately
        // huge so a whole-VM stall on a shared CI runner cannot flake it.
        assert!(
            dt < Duration::from_secs(10),
            "step {i} must complete within a generous bound; took {dt:?}"
        );
        // (ii) at most ONE fire per step (idempotent External bool — a storm of
        // readiness coalesces to a single mark per sweep).
        let p = p_fires.load(Ordering::Relaxed);
        assert!(
            p - prev_p <= 1,
            "step {i}: at most ONE fire per step; observed delta {}",
            p - prev_p
        );
        prev_p = p;
        // (iii) delivery invariant: the producer publishes a monotonically
        // increasing value (next_val += 1 per fire), so the delivered value is
        // monotonic non-decreasing AND never exceeds the producer's fire count.
        let last = c_last.load(Ordering::Relaxed);
        assert!(
            last >= prev_last,
            "step {i}: delivered value must be monotonic non-decreasing ({last} < {prev_last})"
        );
        assert!(
            last <= p,
            "step {i}: delivered value {last} must never exceed the producer fire count {p}"
        );
        prev_last = last;
    }
    // The storm must actually have fired the node (the apparatus moves — an
    // anti-tautology guard against a silently-inert seam).
    assert!(
        p_fires.load(Ordering::Relaxed) >= 1,
        "the storm must have fired the external node at least once"
    );

    // Stop the storm and reap the flooder before touching the pipe again
    // (explicit drop = stop flag + join; the guard also covers every
    // assertion-panic path above).
    drop(flooder_guard);

    // (iv) Quiesce: the node drains-to-EAGAIN per tick, so a bounded number of
    // steps empties the pipe; quiescence = a step that produces no new fire.
    let mut base = p_fires.load(Ordering::Relaxed);
    let mut quiescent = false;
    for _ in 0..50 {
        runtime.run_live_step_once_for_test(Duration::from_millis(50));
        let now = p_fires.load(Ordering::Relaxed);
        if now == base {
            quiescent = true;
            break;
        }
        base = now;
    }
    assert!(
        quiescent,
        "after the storm stops the loop must reach a quiescent (no-fire) step \
         (the drain-to-EAGAIN tick empties the pipe)"
    );
    let fires_at_quiescence = p_fires.load(Ordering::Relaxed);

    // No-lost-wakeup TAIL (Principle #6): one fresh write on the now-empty pipe
    // produces EXACTLY one more fire that DELIVERS — the storm left no wedge.
    pipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        fires_at_quiescence + 1,
        "a post-storm write must produce EXACTLY one more fire (no wedge, no lost wakeup)"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        fires_at_quiescence + 1,
        "delivery oracle: the tail fire's published value reached the consumer (producer→consumer \
         collapse still works after the storm)"
    );
}

// ---------------------------------------------------------------------------
// 17b. The FD_SETSIZE ceiling is a SELECT-path concern only.
//
// An external `ExternalSource::Fd` whose value is >= FD_SETSIZE (1024) is
// out-of-bounds UB in the select-backed WaitSet's `FD_SET`, so a park-OFF run
// REFUSES it (reason FdAboveSelectLimit) with two workarounds. But the
// monitor-wait park polls each external fd via poll(2) (no fd-number ceiling), so
// a park-ON run ACCEPTS a live high fd and fires the node. These tests pin both
// sides + the exact FD_SETSIZE boundary. No `#[cfg(unix)]` gate — the whole file
// is already unix-only (unconditional `RawFd`/`libc` fd tricks).
// ---------------------------------------------------------------------------

/// Raise the `RLIMIT_NOFILE` soft limit to at least `at_least` (a no-op if it is
/// already high enough) so a `dup2` onto a high target fd is legal. Loud on any
/// libc failure — an unraised limit would fail the `dup2` and mask the real cause.
fn raise_nofile_to(at_least: libc::rlim_t) {
    // SAFETY: `getrlimit`/`setrlimit` read/write a local `rlimit` we own.
    unsafe {
        let mut rl = libc::rlimit {
            rlim_cur: 0,
            rlim_max: 0,
        };
        assert_eq!(
            libc::getrlimit(libc::RLIMIT_NOFILE, &mut rl),
            0,
            "getrlimit(RLIMIT_NOFILE) must succeed"
        );
        if rl.rlim_cur < at_least {
            rl.rlim_cur = at_least.min(rl.rlim_max);
            assert_eq!(
                libc::setrlimit(libc::RLIMIT_NOFILE, &rl),
                0,
                "setrlimit raising the NOFILE soft limit to {} must succeed",
                rl.rlim_cur
            );
            assert!(
                rl.rlim_cur >= at_least,
                "the hard NOFILE limit ({}) is below the {at_least} this test needs",
                rl.rlim_max
            );
        }
    }
}

/// A live pipe whose READ end is duplicated onto a chosen high fd number
/// (`>= FD_SETSIZE` for the high-fd tests). `write_byte` makes the high fd
/// readable; the original LOW read end is closed so only `high` refers to the read
/// side (the node watches/drains `high`). The dup shares the pipe's non-blocking
/// open file description, so a `read(2)` on `high` never blocks. RAII: `Drop`
/// closes `high` and the inner `Pipe` closes its write end.
struct HighFdPipe {
    pipe: Pipe,
    high: RawFd,
}

impl HighFdPipe {
    fn new(target: RawFd) -> Self {
        raise_nofile_to(target as libc::rlim_t + 16);
        let mut pipe = Pipe::new();
        // The target must be free so dup2 does not clobber a live fd.
        // SAFETY: F_GETFD is a read-only liveness probe.
        assert_eq!(
            unsafe { libc::fcntl(target, libc::F_GETFD) },
            -1,
            "the high fd {target} must be free before dup2"
        );
        // SAFETY: `pipe.read` is a valid open fd; `target` is a free high fd number.
        let rc = unsafe { libc::dup2(pipe.read, target) };
        assert_eq!(rc, target, "dup2 onto the high fd must return the target");
        // Close the original LOW read end so only `target` refers to the read side;
        // relinquish it so `Pipe::drop` does not double-close. The write end stays.
        let low_read = pipe.take_read();
        // SAFETY: sole close of the low read end we just relinquished.
        unsafe { libc::close(low_read) };
        HighFdPipe { pipe, high: target }
    }

    fn write_byte(&self) {
        self.pipe.write_byte();
    }

    /// Relinquish the high READ-end fd so `Drop` won't close it:
    /// used when the runtime OWNS and closes it (a refused DOORBELL read end), so
    /// the test must not double-close a fd number the process may have reused.
    fn take_high(&mut self) -> RawFd {
        let h = self.high;
        self.high = -1;
        h
    }

    /// Attempt one `write(2)` to the pipe's WRITE end, returning
    /// `(rc, errno)`. Unlike [`Self::write_byte`] this does NOT assert success —
    /// once the sole read reference is closed, the write fails `-1`/`EPIPE`, which
    /// is the observable oracle that a refused doorbell read end was closed.
    fn try_write_byte(&self) -> (isize, i32) {
        let b: u8 = 1;
        // SAFETY: `self.pipe.write` is a valid open fd; one byte from a live buffer.
        let rc = unsafe {
            libc::write(
                self.pipe.write,
                std::ptr::addr_of!(b) as *const libc::c_void,
                1,
            )
        };
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        (rc, errno)
    }
}

impl Drop for HighFdPipe {
    fn drop(&mut self) {
        // SAFETY: sole close of the high dup target we created; the inner `Pipe`
        // closes its own write end (its read end was relinquished in `new`).
        if self.high >= 0 {
            unsafe { libc::close(self.high) };
        }
    }
}

/// Headline: a park-ON run watches a LIVE fd `>= FD_SETSIZE` via `poll(2)`
/// (no fd-number ceiling), so collect ACCEPTS it and the fd fires the node.
/// Mutation kill: reverting the commit-3 park-aware classify refuses this high fd
/// at collect and the infallible primer panics.
#[test]
#[serial]
fn park_on_high_fd_runs_and_fires_via_poll() {
    let _spin = SpinDisabledGuard::new();
    let hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd + 76); // 1100 >= FD_SETSIZE
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(
        ExternalSource::Fd(hpipe.high),
        Arc::clone(&p_fires),
        Some(hpipe.high), // drain in tick
    );
    let mut runtime = producer_consumer_graph_with_policy(
        producer,
        Arc::clone(&c_last),
        Arc::clone(&c_fires),
        MonitorWaitPolicy::new(true, false, "hifdpark".into()),
    );

    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "park-ON: a live fd >= FD_SETSIZE is watchable via poll(2), so it binds"
    );

    hpipe.write_byte();
    runtime.run_live_step_once_for_test(Duration::from_millis(500));

    assert!(
        runtime.park_wakes_external_fd_count_for_test() > 0,
        "attribution: the high fd WOKE the poll(2)-based park (not the timeout)"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        1,
        "the high fd fires the external node exactly once"
    );
    assert_eq!(
        c_fires.load(Ordering::Relaxed),
        1,
        "the downstream data-trigger consumer fires once (producer→consumer collapse)"
    );
    assert_eq!(
        c_last.load(Ordering::Relaxed),
        1,
        "delivery oracle: the consumer received the producer's published value (1)"
    );
}

/// A park-OFF run's select-backed reactor cannot watch a fd
/// `>= FD_SETSIZE`, so collect REFUSES the run naming the sole offender
/// (FdAboveSelectLimit) and rendering BOTH workarounds. Nothing binds; the node
/// never fires. The refusal returns BEFORE any reactor is built, so the boundary
/// fd is never actually `select`ed on.
#[test]
#[serial]
fn park_off_high_fd_refused_with_two_workarounds() {
    let hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd + 76); // 1100 >= FD_SETSIZE
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(ExternalSource::Fd(hpipe.high), Arc::clone(&p_fires), None);
    // Park OFF (default `build_for_test`) → the select-backed reactor path.
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("a live fd >= FD_SETSIZE under the select path must REFUSE the live run");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [(
                    "producer".to_string(),
                    cerulion_core::InertReason::FdAboveSelectLimit
                )],
                "the high-fd producer is the sole offender, reason fd above select limit; got {nodes:?}"
            );
            let msg = err.to_string();
            assert!(
                msg.contains("monitor-wait park") && msg.contains("shrink the graph"),
                "the refusal must render BOTH workarounds (enable the park / shrink the graph); got: {msg}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the refused high fd binds nothing"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        0,
        "the refused node never fires"
    );
}

/// The EXACT FD_SETSIZE boundary on the select (park-OFF) path. `1023`
/// (FD_SETSIZE-1) is below the ceiling → ACCEPTED (binds); `1024` (== FD_SETSIZE)
/// is at the ceiling → FdAboveSelectLimit. Two sub-arms in one body. Neither arm
/// drives the live seam, so the boundary fds are never actually `select`ed on.
#[test]
#[serial]
fn fd_setsize_boundary_park_off() {
    // Sub-arm A: fd == FD_SETSIZE-1 (1023) → below the ceiling → binds.
    {
        let hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd - 1); // 1023 < FD_SETSIZE
        let p_fires = Arc::new(AtomicU64::new(0));
        let c_last = Arc::new(AtomicU64::new(0));
        let c_fires = Arc::new(AtomicU64::new(0));
        let producer = ExtProducer::new(ExternalSource::Fd(hpipe.high), Arc::clone(&p_fires), None);
        let mut runtime = producer_consumer_graph(producer, c_last, c_fires);
        runtime
            .try_collect_external_sources_for_test()
            .expect("fd FD_SETSIZE-1 (1023) is below the ceiling and must bind on the select path");
        assert_eq!(
            runtime.external_binding_count(),
            1,
            "fd 1023 (< FD_SETSIZE) binds on the select path"
        );
    }
    // Sub-arm B: fd == FD_SETSIZE (1024) → at the ceiling → FdAboveSelectLimit.
    {
        let hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd); // 1024 == FD_SETSIZE
        let p_fires = Arc::new(AtomicU64::new(0));
        let c_last = Arc::new(AtomicU64::new(0));
        let c_fires = Arc::new(AtomicU64::new(0));
        let producer = ExtProducer::new(ExternalSource::Fd(hpipe.high), Arc::clone(&p_fires), None);
        let mut runtime = producer_consumer_graph(producer, c_last, c_fires);
        let err = runtime
            .try_collect_external_sources_for_test()
            .expect_err("fd == FD_SETSIZE (1024) must be refused on the select path");
        match err {
            cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
                assert_eq!(
                    nodes.as_slice(),
                    [(
                        "producer".to_string(),
                        cerulion_core::InertReason::FdAboveSelectLimit
                    )],
                    "fd == FD_SETSIZE is exactly the ceiling → FdAboveSelectLimit; got {nodes:?}"
                );
            }
            other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
        }
        assert_eq!(
            runtime.external_binding_count(),
            0,
            "the boundary fd (== FD_SETSIZE) binds nothing"
        );
    }
}

/// The barrier-participant term of
/// `external_fds_are_polled_not_selected()` (`park_active() ||
/// barrier_participant.is_some()`) is LOAD-BEARING. A barrier participant idles in
/// `monitor_wait_block` even when the monitor-wait park is OFF, so its
/// external fds are polled via `poll(2)` — no `FD_SETSIZE` ceiling — exactly like a
/// park-ON run. So a PARK-OFF runtime holding a barrier participant must ACCEPT a
/// live fd `>= FD_SETSIZE` at collect, where the same fd on a park-OFF run WITHOUT
/// a barrier is refused FdAboveSelectLimit (the control is
/// `park_off_high_fd_refused_with_two_workarounds` above — the identical
/// `HighFdPipe::new(FD_SETSIZE + 76)` fd, park off, no barrier → refused).
///
/// Mutation kill: reverting the `|| barrier_participant.is_some()` term makes
/// `external_fds_are_polled_not_selected()` return false here, so classify reports
/// AboveSelectLimit → collect refuses → the infallible
/// `collect_external_sources_for_test()` PANICS (the whole test fails). Verified
/// locally (revert → fail → restore).
///
/// Collect-ONLY: a barrier participant makes `step()` rendezvous on the barrier,
/// but collect never touches it, so no step is driven (and no peer is needed).
/// `expected == 1` makes the barrier self-opening regardless.
#[test]
#[serial]
fn barrier_participant_lifts_select_ceiling_for_high_fd() {
    let hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd + 76); // 1100 >= FD_SETSIZE
    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    // A poll-only DEVICE fd (NOT a doorbell): the Usable arm binds it non-owning,
    // so `HighFdPipe::drop` is the sole closer (no double-close).
    let producer = ExtProducer::new(ExternalSource::Fd(hpipe.high), Arc::clone(&p_fires), None);
    // Park OFF (default `build_for_test`) → the select path, so the ONLY thing that
    // can lift the FD_SETSIZE ceiling is the injected barrier participant.
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

    // Inject a single-participant barrier (expected = 1 → self-opening). The
    // global-level map is the identity over this runtime's own levels — a valid
    // strictly-increasing bijection onto `0..levels.len()` regardless of the graph
    // shape (here producer@L0 → consumer@L1 = 2 levels).
    let ns = format!("f1_{}", std::process::id());
    let barrier = Arc::new(
        MappedBarrier::create_owned(&ns, "g", 1).expect("single-participant barrier create"),
    );
    let map: Vec<Option<usize>> = (0..runtime.levels().len()).map(Some).collect();
    runtime.set_barrier_participant_for_test(barrier, map, 0);

    // Infallible collect — PANICS on refusal (the mutation kill for the barrier
    // term). With the term intact, the high fd classifies Usable and binds.
    runtime.collect_external_sources_for_test();
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "park-OFF but a barrier participant polls fds via poll(2): a live fd >= FD_SETSIZE binds"
    );
}

/// The ONLY `AboveSelectLimit` sub-arm that CLOSES a
/// live fd is the `is_doorbell` arm — a cdylib tier-2 `Blocking`-collapse pipe read
/// end handed to the host. A park-OFF run whose external source reports a DOORBELL
/// fd `>= FD_SETSIZE` refuses (FdAboveSelectLimit) AND closes the read end so the
/// stranded cdylib helper sees EOF and self-terminates. A dropped close would
/// silently strand the helper — so the close is the load-bearing observable here.
///
/// The close is observed via the retained WRITE end (a hand oracle, not a
/// self-compare): once the sole read reference is closed, a `write(2)` returns
/// `-1`/`EPIPE`. SIGPIPE is masked (the Rust runtime ignores it by default, but
/// this is belt-and-suspenders) so the write reports the error instead of raising.
#[test]
#[serial]
fn park_off_high_fd_doorbell_is_refused_and_read_end_closed() {
    // SIGPIPE → ignore so a write to the reader-less pipe returns EPIPE, never a
    // signal. SAFETY: setting a signal disposition to SIG_IGN; this `#[serial]`
    // test owns process-global state for its duration (and Rust already SIG_IGNs
    // SIGPIPE, so this is idempotent).
    unsafe {
        libc::signal(libc::SIGPIPE, libc::SIG_IGN);
    }

    let mut hpipe = HighFdPipe::new(libc::FD_SETSIZE as RawFd + 76); // 1100 >= FD_SETSIZE
                                                                     // The runtime OWNS and closes a refused doorbell read end, so relinquish it
                                                                     // here — otherwise `HighFdPipe::drop` would double-close a fd number the
                                                                     // process may have reused.
    let high = hpipe.take_high();

    let p_fires = Arc::new(AtomicU64::new(0));
    let c_last = Arc::new(AtomicU64::new(0));
    let c_fires = Arc::new(AtomicU64::new(0));
    // `.with_doorbell_marker()` → the fd is reported as a DRAINED DOORBELL read
    // end, so collect takes the `is_doorbell` sub-arm of the AboveSelectLimit verdict.
    let producer = ExtProducer::new(ExternalSource::Fd(high), Arc::clone(&p_fires), None)
        .with_doorbell_marker();
    // Park OFF (default `build_for_test`) → the select path → AboveSelectLimit.
    let mut runtime = producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

    let err = runtime
        .try_collect_external_sources_for_test()
        .expect_err("a live doorbell fd >= FD_SETSIZE under the select path must REFUSE the run");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [(
                    "producer".to_string(),
                    cerulion_core::InertReason::FdAboveSelectLimit
                )],
                "the high-fd doorbell producer is the sole offender (FdAboveSelectLimit); got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the refused doorbell binds nothing"
    );
    assert_eq!(
        p_fires.load(Ordering::Relaxed),
        0,
        "the refused node never fires"
    );

    // The load-bearing pin: the `is_doorbell` AboveSelectLimit arm CLOSED the read
    // end. With the sole read reference gone, a write(2) to the retained write end
    // fails -1/EPIPE. A dropped close leaves the read end open and this write
    // SUCCEEDS (rc == 1) — that is the regression this arm guards against.
    let (rc, errno) = hpipe.try_write_byte();
    assert_eq!(
        rc, -1,
        "the doorbell read end must be closed → a write to the reader-less pipe returns -1 (got rc={rc})"
    );
    assert_eq!(
        errno,
        libc::EPIPE,
        "closing the sole read end makes the write fail EPIPE; got errno={errno}"
    );
}

// ---------------------------------------------------------------------------
// 18. EVENTFD arm (Linux-gated). The pipe is the portable
// primary wake source; this module proves the SAME tier-1 `ExternalSource::Fd`
// path handles a REAL non-pipe descriptor kind — a Linux `eventfd(2)` — including
// its accumulate-then-reset counter semantics. Whole module is
// `#[cfg(target_os = "linux")]`: every `libc::eventfd`/`EFD_NONBLOCK` reference
// (absent on macOS's `libc`) lives inside the gate, so the macOS CI job never
// sees an eventfd symbol.
// ---------------------------------------------------------------------------
#[cfg(target_os = "linux")]
mod eventfd_arm {
    use super::*;

    /// An RAII Linux `eventfd` (EFD_NONBLOCK). `add(n)` bumps the kernel counter
    /// by `n` (one 8-byte host-endian write); a single 8-byte `read` returns the
    /// accumulated counter and resets it to 0 (no `EFD_SEMAPHORE`), so one tick
    /// fully drains it and the level-trigger then goes quiet.
    struct EventFd {
        fd: RawFd,
    }

    impl EventFd {
        fn new() -> EventFd {
            // SAFETY: eventfd with a 0 initial count and the EFD_NONBLOCK flag; a
            // valid new fd (>= 0) or -1 on failure (asserted).
            let fd = unsafe { libc::eventfd(0, libc::EFD_NONBLOCK) };
            assert!(fd >= 0, "eventfd(0, EFD_NONBLOCK) must succeed");
            EventFd { fd }
        }

        /// Add `n` to the kernel counter (one 8-byte native-endian write).
        fn add(&self, n: u64) {
            let v = n.to_ne_bytes();
            // SAFETY: `self.fd` is a valid open eventfd; exactly 8 bytes from a
            // live stack buffer, which is the eventfd write contract.
            let w = unsafe { libc::write(self.fd, v.as_ptr() as *const libc::c_void, 8) };
            assert_eq!(w, 8, "eventfd write must write exactly 8 bytes");
        }
    }

    impl Drop for EventFd {
        fn drop(&mut self) {
            // SAFETY: sole close of the eventfd this struct opened.
            unsafe { libc::close(self.fd) };
        }
    }

    /// An `external`-policy eventfd ingress node. `external_source()` hands the
    /// runtime `ExternalSource::Fd(efd)` once; `tick()` reads the eventfd's 8-byte
    /// counter (draining + resetting it), records the OBSERVED counter into
    /// `last_counter` (the summed-value oracle), counts fires, and publishes an
    /// incrementing value.
    struct EventfdProducer {
        src: Option<ExternalSource>,
        ctx: Option<NodeContext>,
        fires: Arc<AtomicU64>,
        next_val: f64,
        /// The eventfd, drained in `tick()`. Non-owning (the test's `EventFd`
        /// owns the lifetime), mirroring the pipe-node `drain_fd` contract.
        efd: RawFd,
        /// Set to the counter value observed by the most recent fire's drain.
        last_counter: Arc<AtomicU64>,
    }

    impl EventfdProducer {
        fn new(efd: RawFd, fires: Arc<AtomicU64>, last_counter: Arc<AtomicU64>) -> Self {
            Self {
                src: Some(ExternalSource::Fd(efd)),
                ctx: None,
                fires,
                next_val: 0.0,
                efd,
                last_counter,
            }
        }
    }

    impl NodeEntry for EventfdProducer {
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

        fn init(&mut self, ctx: NodeContext) -> TransportResult<()> {
            self.ctx = Some(ctx);
            Ok(())
        }

        fn tick(&mut self) -> TransportResult<()> {
            self.fires.fetch_add(1, Ordering::Relaxed);
            // Drain the eventfd: ONE 8-byte read returns the accumulated counter
            // and resets it to 0. A zero counter (nothing to drain) returns
            // EAGAIN (n != 8) → leave last_counter unchanged.
            let mut buf = [0u8; 8];
            // SAFETY: `self.efd` is a valid non-blocking eventfd; exactly 8 bytes
            // into a live stack buffer, which is the eventfd read contract.
            let n = unsafe { libc::read(self.efd, buf.as_mut_ptr() as *mut libc::c_void, 8) };
            if n == 8 {
                self.last_counter
                    .store(u64::from_ne_bytes(buf), Ordering::Relaxed);
            }
            self.next_val += 1.0;
            let val = self.next_val;
            let ctx = self.ctx.as_mut().expect("EventfdProducer init() ran");
            let pubr = ctx.publisher_mut("out").expect("out publisher wired");
            let mut proxy = pubr.loan_proxy::<Vector3>()?;
            proxy.x = val;
            drop(proxy); // publish
            Ok(())
        }

        fn external_source(&mut self) -> Option<ExternalSource> {
            self.src.take()
        }
    }

    /// Build an eventfd-producer → data-trigger-consumer graph over an isolated
    /// per-test SHM root (reuses the file's `RecvConsumer` delivery oracle).
    fn eventfd_producer_consumer_graph(
        producer: EventfdProducer,
        consumer_last: Arc<AtomicU64>,
        consumer_fires: Arc<AtomicU64>,
    ) -> GraphRuntime {
        let config = GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "ext_eventfd".to_string(),
            prefix: "extefd".to_string(),
            nodes: vec![
                producer_only_node("producer"),
                NodeDef {
                    ros2: None,
                    id: "consumer".to_string(),
                    node_type: "recv_consumer".to_string(),
                    inputs: vec![InputDef {
                        name: "inp".to_string(),
                        source: "producer/out".to_string(),
                    }],
                    outputs: vec![],
                },
            ],
        };
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("producer".to_string(), Box::new(producer));
        factories.insert(
            "consumer".to_string(),
            Box::new(RecvConsumerEntry::with_state(RecvConsumer {
                last: consumer_last,
                fires: consumer_fires,
                ..Default::default()
            })),
        );
        let clock = Arc::new(VirtualClock::new());
        GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("build eventfd live-fire graph")
    }

    // -----------------------------------------------------------------------
    // 18a. A REAL eventfd fired by a NON-Cerulion thread fires the external node
    // on the live seam AND delivers the published value downstream (hand oracle).
    // -----------------------------------------------------------------------
    #[test]
    #[serial]
    fn eventfd_node_fires_on_live_seam_and_delivers() {
        let efd = EventFd::new();
        let p_fires = Arc::new(AtomicU64::new(0));
        let counter_obs = Arc::new(AtomicU64::new(0));
        let c_last = Arc::new(AtomicU64::new(0));
        let c_fires = Arc::new(AtomicU64::new(0));
        let producer = EventfdProducer::new(efd.fd, Arc::clone(&p_fires), Arc::clone(&counter_obs));
        let mut runtime =
            eventfd_producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));

        runtime.collect_external_sources_for_test();
        assert_eq!(
            runtime.external_binding_count(),
            1,
            "the eventfd Fd source binds exactly once (tier-1 fd path, non-pipe kind)"
        );

        // A NON-Cerulion thread signals the device (a real 8-byte eventfd write).
        let signal_fd = efd.fd; // Copy RawFd; `efd` outlives the join.
        let writer = std::thread::spawn(move || {
            let v = 1u64.to_ne_bytes();
            // SAFETY: `signal_fd` is the test's valid eventfd (alive until after
            // the join below); exactly 8 bytes from a live buffer.
            let w = unsafe { libc::write(signal_fd, v.as_ptr() as *const libc::c_void, 8) };
            assert_eq!(w, 8, "external-thread eventfd write must succeed");
        });
        writer.join().expect("eventfd writer thread must not panic");

        runtime.run_live_step_once_for_test(Duration::from_millis(500));

        // Hand oracle: readable eventfd → producer fires ONCE (drains counter 1),
        // publishes 1.0 → data-trigger consumer receives 1 the same step.
        assert_eq!(
            p_fires.load(Ordering::Relaxed),
            1,
            "the readable eventfd fires the external node exactly once"
        );
        assert_eq!(
            counter_obs.load(Ordering::Relaxed),
            1,
            "the tick drained the eventfd and observed counter 1"
        );
        assert_eq!(
            c_fires.load(Ordering::Relaxed),
            1,
            "the downstream data-trigger consumer fires once (producer→consumer collapse)"
        );
        assert_eq!(
            c_last.load(Ordering::Relaxed),
            1,
            "delivery oracle: the consumer received the producer's published value (1)"
        );
    }

    // -----------------------------------------------------------------------
    // 18b. EVENTFD COUNTER SEMANTICS: N accumulating writes before one step
    // coalesce to ONE fire whose tick observes the SUMMED counter; after the
    // 8-byte drain resets the counter to 0, a silent step does NOT re-fire (the
    // level-trigger goes quiet). Hand oracle on the summed value (1+1+1+2 = 5).
    // -----------------------------------------------------------------------
    #[test]
    #[serial]
    fn eventfd_counter_accumulates_coalesces_to_one_fire_with_summed_value() {
        let efd = EventFd::new();
        let p_fires = Arc::new(AtomicU64::new(0));
        let counter_obs = Arc::new(AtomicU64::new(0));
        let c_last = Arc::new(AtomicU64::new(0));
        let c_fires = Arc::new(AtomicU64::new(0));
        let producer = EventfdProducer::new(efd.fd, Arc::clone(&p_fires), Arc::clone(&counter_obs));
        let mut runtime =
            eventfd_producer_consumer_graph(producer, Arc::clone(&c_last), Arc::clone(&c_fires));
        runtime.collect_external_sources_for_test();

        // FOUR accumulating writes (three 1s + one 2) BEFORE a single step: the
        // kernel counter sums to 5 (hand oracle) — N events, one coalesced fire.
        efd.add(1);
        efd.add(1);
        efd.add(1);
        efd.add(2);
        runtime.run_live_step_once_for_test(Duration::from_millis(500));

        assert_eq!(
            p_fires.load(Ordering::Relaxed),
            1,
            "the accumulated eventfd counter coalesces to exactly ONE fire"
        );
        assert_eq!(
            counter_obs.load(Ordering::Relaxed),
            5,
            "the single coalesced fire's tick observes the SUMMED counter (1+1+1+2 = 5)"
        );
        assert_eq!(
            c_last.load(Ordering::Relaxed),
            1,
            "delivery oracle: the coalesced fire published value 1"
        );

        // The 8-byte read reset the counter to 0 → a silent step sees no
        // readiness → no re-fire, and the observed counter stays 5.
        runtime.run_live_step_once_for_test(Duration::from_millis(50));
        assert_eq!(
            p_fires.load(Ordering::Relaxed),
            1,
            "after the counter drains to 0 the level-trigger goes quiet (no re-fire)"
        );
        assert_eq!(
            counter_obs.load(Ordering::Relaxed),
            5,
            "no second fire → the observed summed counter stays 5"
        );
    }
}
