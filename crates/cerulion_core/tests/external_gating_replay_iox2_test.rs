// SPDX-License-Identifier: AGPL-3.0-only
//! LIVE/POLLED gating pins + the `graph run` HostDriven refusal
//! + the oracle-vector replay firewall, all over real iceoryx2.
//!
//! The body acceptance is: `external_source` is a LIVE-ONLY side
//! effect — "the fd is never attached/polled/read outside `run_live`", and
//! "replay feeds recorded output; the fd is never touched in replay". This file
//! pins that contract:
//!
//! - **Polled never queries** — a polled `build_for_test` graph whose external
//!   node's `external_source()` PANICS is stepped (`trigger_external` + `step`)
//!   and NEVER panics: the polled/replay path never touches the source.
//! - **Live queries exactly once** — `run_live` (entry collect, guarded by
//!   `external_sources_collected`) queries `external_source()` exactly once per
//!   runtime, even across a resumed session.
//! - **Inert-at-launch refusal (was HostDriven-only)** — `run_live`
//!   returns `TransportError::ExternalNodesInertAtLaunch` naming the node with
//!   its `InertReason` (and, for a multi-node graph, ALL such nodes with
//!   DISTINCT reasons) — a provably-inert external node can never fire while the
//!   live loop owns the runtime. Sibling `Blocking` helpers spawned in the same
//!   collect pass are torn down eagerly at the refusal.
//! - **Replay = Live firewall (Principle #7)** — a LIVE leg (an external Fd node
//!   fed by a NON-Cerulion writer thread over a plain pipe, publishing values
//!   DERIVED from what it drains) records the downstream DELIVERED sequence; a
//!   REPLAY leg feeds that recorded sequence downstream through a polled node
//!   whose fd path is never touched (its `external_source()` panics), and the
//!   delivered sequence is BYTE-IDENTICAL to the recording AND equals a
//!   HAND-COMPUTED oracle from the writer's known input vector. The live
//!   recording is anchored to the SAME oracle, so neither leg is a self-compare.
//!
//! No fake data (Principle #13): every live wake is a real pipe byte; every
//! oracle is a hand value. Deterministic: fixed input vector, `VirtualClock`
//! (`build_for_test`), wire-sequence delivery keying. `#[serial]` — real
//! iceoryx2 over the process-global SHM singleton (per-test SHM root via
//! `build_for_test`).

use std::os::unix::io::RawFd;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::MacroPolicy;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Extract just the node names from an `ExternalNodesInertAtLaunch`
/// offender list `[(id, reason)]`, so the exact-ordered-list assertions survive
/// the offender-list shape change (`Vec<String>` → `Vec<(String, reason)>`).
fn inert_names(nodes: &[(String, cerulion_core::InertReason)]) -> Vec<&str> {
    nodes.iter().map(|(n, _)| n.as_str()).collect()
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A `libc::pipe` with RAII cleanup; the read end is set NON-BLOCKING so a
/// draining `read(2)` never blocks when the pipe empties.
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
}

impl Drop for Pipe {
    fn drop(&mut self) {
        // SAFETY: each is a fd this Pipe opened; closed at most once.
        if self.read >= 0 {
            unsafe { libc::close(self.read) };
        }
        if self.write >= 0 {
            unsafe { libc::close(self.write) };
        }
    }
}

/// What an [`ExtProducer`]'s `tick()` publishes as `out.x`.
enum PublishMode {
    /// Publish an incrementing counter (`1.0, 2.0, ...`).
    Counter,
    /// Drain `fd` (non-blocking, to empty) and publish `100 + last_byte` — the
    /// value DERIVED from the drained device data (the LIVE replay leg).
    DrainDerive(RawFd),
    /// Publish `recorded[idx++]` — the recorded output sequence (the REPLAY leg).
    Replay(Vec<u64>),
}

/// A hand-written `external`-policy ingress (driver) node, parameterized for
/// every gating/replay pin: it may return any [`ExternalSource`] (once, via
/// `Option::take`), optionally PANIC when queried (the polled/replay guard),
/// count queries + fires, and publish via a [`PublishMode`].
struct ExtProducer {
    src: Option<ExternalSource>,
    panic_on_query: bool,
    ctx: Option<NodeContext>,
    fires: Arc<AtomicU64>,
    queries: Arc<AtomicU64>,
    mode: PublishMode,
    counter: f64,
    replay_idx: usize,
}

impl ExtProducer {
    fn new(src: ExternalSource, mode: PublishMode) -> Self {
        Self {
            src: Some(src),
            panic_on_query: false,
            ctx: None,
            fires: Arc::new(AtomicU64::new(0)),
            queries: Arc::new(AtomicU64::new(0)),
            mode,
            counter: 0.0,
            replay_idx: 0,
        }
    }

    /// Make `external_source()` PANIC — the polled/replay guard (a polled or
    /// replay step must NEVER query the source).
    fn panic_on_query(mut self) -> Self {
        self.panic_on_query = true;
        self
    }

    fn with_fires(mut self, fires: Arc<AtomicU64>) -> Self {
        self.fires = fires;
        self
    }

    fn with_queries(mut self, queries: Arc<AtomicU64>) -> Self {
        self.queries = queries;
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
        let val = match &self.mode {
            PublishMode::Counter => {
                self.counter += 1.0;
                self.counter
            }
            PublishMode::DrainDerive(fd) => {
                // Drain the non-blocking fd to empty; the LAST byte drained is
                // the device datum. tick fires ONLY when the sweep saw the fd
                // readable, so at least one byte is present.
                let fd = *fd;
                let mut last: Option<u8> = None;
                let mut buf = [0u8; 256];
                loop {
                    // SAFETY: `fd` is a valid non-blocking fd; read into a live
                    // buffer. <=0 (EOF / EAGAIN) ends the drain.
                    let n =
                        unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
                    if n <= 0 {
                        break;
                    }
                    last = Some(buf[(n - 1) as usize]);
                }
                let b = last.expect("DrainDerive tick fires only when the fd is readable");
                100.0 + b as f64
            }
            PublishMode::Replay(recorded) => {
                let v = recorded[self.replay_idx] as f64;
                self.replay_idx += 1;
                v
            }
        };
        let ctx = self.ctx.as_mut().expect("ExtProducer init() ran");
        let pubr = ctx.publisher_mut("out").expect("out publisher wired");
        let mut proxy = pubr.loan_proxy::<Vector3>()?;
        proxy.x = val;
        drop(proxy); // publish
        Ok(())
    }

    fn external_source(&mut self) -> Option<ExternalSource> {
        self.queries.fetch_add(1, Ordering::AcqRel);
        assert!(
            !self.panic_on_query,
            "external_source() must NOT be queried on the polled/replay path"
        );
        self.src.take()
    }
}

/// A data-trigger consumer that PUSHES every received `x` into a shared vector —
/// the downstream DELIVERED-sequence oracle.
#[cerulion_node]
#[derive(Default)]
struct SeqRecvConsumer {
    #[input(trigger)]
    inp: Vector3,
    seq: Arc<Mutex<Vec<u64>>>,
}

#[cerulion_node_impl]
impl SeqRecvConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seq.lock().expect("seq lock").push(self.inp.x as u64);
        Ok(())
    }
}

/// A producer-only NodeDef (`out`, no consumer).
fn producer_only_node(id: &str) -> NodeDef {
    NodeDef {
        fuse: None,
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

/// Build a `producer` (`ExtProducer`) → `consumer` (`SeqRecvConsumer`) graph
/// over an isolated per-test SHM root.
fn producer_consumer_graph(
    name: &str,
    prefix: &str,
    producer: ExtProducer,
    seq: Arc<Mutex<Vec<u64>>>,
) -> GraphRuntime {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: name.to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            producer_only_node("producer"),
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "seq_recv_consumer".to_string(),
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
        Box::new(SeqRecvConsumerEntry::with_state(SeqRecvConsumer {
            seq,
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build producer→consumer graph")
}

/// Drive the live seam until `pred` holds or `deadline` elapses.
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

/// Poll `pred` until true or `deadline` (for a helper thread's async effect).
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
// 1. GATING (4a): the POLLED path NEVER queries external_source().
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn polled_path_never_queries_external_source() {
    // The producer's `external_source()` PANICS. On the polled path (no
    // `collect_external_sources` / `run_live`), it must never be called — the
    // host owns stepping and fires the External node via `trigger_external`.
    let fires = Arc::new(AtomicU64::new(0));
    let queries = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(ExternalSource::Fd(-1), PublishMode::Counter)
        .panic_on_query()
        .with_fires(Arc::clone(&fires))
        .with_queries(Arc::clone(&queries));
    let seq = Arc::new(Mutex::new(Vec::<u64>::new()));
    let mut runtime =
        producer_consumer_graph("egr_polled_noquery", "egrpnq", producer, Arc::clone(&seq));

    // Polled stepping only: trigger_external + step, THREE times. If
    // external_source() were queried, the panic would abort this test.
    for _ in 0..3 {
        runtime
            .trigger_external("producer")
            .expect("trigger_external");
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        queries.load(Ordering::Acquire),
        0,
        "the polled path must NEVER query external_source() (the panic guard did not trip)"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        3,
        "the External node still fires via trigger_external + step on the polled path"
    );
    // Delivery oracle: the three fires published 1,2,3 → the consumer received them.
    assert_eq!(
        *seq.lock().expect("seq"),
        vec![1, 2, 3],
        "delivery oracle: polled fires deliver the incrementing sequence downstream"
    );
}

// ---------------------------------------------------------------------------
// 2. GATING (4b): run_live queries external_source() EXACTLY ONCE per runtime,
// even across a resumed session.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn run_live_queries_external_source_exactly_once() {
    let pipe = Pipe::new();
    let queries = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(ExternalSource::Fd(pipe.read), PublishMode::Counter)
        .with_queries(Arc::clone(&queries));
    let seq = Arc::new(Mutex::new(Vec::<u64>::new()));
    let mut runtime =
        producer_consumer_graph("egr_query_once", "egrqo", producer, Arc::clone(&seq));

    // First run_live (running=false): collect runs at entry, queries once, binds
    // the (valid, non-host-driven) Fd source, and returns Ok.
    let running = AtomicBool::new(false);
    runtime
        .run_live(&running)
        .expect("a valid Fd source is not refused");
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the Fd source produced exactly one binding"
    );
    assert_eq!(
        queries.load(Ordering::Acquire),
        1,
        "run_live queried external_source() exactly ONCE"
    );

    // A resumed run_live must NOT re-query (the collect idempotency guard).
    runtime
        .run_live(&running)
        .expect("resumed run_live is not refused");
    assert_eq!(
        queries.load(Ordering::Acquire),
        1,
        "a resumed run_live reuses the one-shot binding — no second query"
    );
}

// ---------------------------------------------------------------------------
// 3. REFUSAL (4c, single): run_live refuses a lone HostDriven node, naming it +
// stating the fix; the node never fires.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn run_live_refuses_single_host_driven_node() {
    let fires = Arc::new(AtomicU64::new(0));
    // A query counter proves the sticky re-refusal RE-RETURNS the
    // recorded error rather than RE-COLLECTING (which would re-query the source).
    let queries = Arc::new(AtomicU64::new(0));
    let producer = ExtProducer::new(ExternalSource::HostDriven, PublishMode::Counter)
        .with_fires(Arc::clone(&fires))
        .with_queries(Arc::clone(&queries));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "egr_refuse_one".to_string(),
        prefix: "egrr1".to_string(),
        nodes: vec![producer_only_node("camera")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("camera".to_string(), Box::new(producer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build refusal graph");

    let running = AtomicBool::new(false);
    let err = runtime
        .run_live(&running)
        .expect_err("a lone HostDriven external node must be REFUSED on the live path");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch {
            ref graph,
            ref nodes,
        } => {
            assert_eq!(graph, "egr_refuse_one", "the refusal names the graph");
            // Assert the EXACT (node, reason) pair — a bare substring
            // would false-positive on a prefix like "cam" vs "camera".
            assert_eq!(
                nodes.as_slice(),
                [("camera".to_string(), cerulion_core::InertReason::HostDriven)],
                "the refusal must name EXACTLY the offending node + reason; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    let msg = err.to_string();
    assert!(
        msg.contains("trigger_external"),
        "the refusal must explain WHY (nothing can call trigger_external); got: {msg}"
    );
    // The remediation hint names BOTH self-source kinds (Fd AND Blocking) plus
    // the polled step() escape hatch.
    assert!(
        msg.contains("ExternalSource::Fd")
            && msg.contains("ExternalSource::Blocking")
            && msg.contains("step()"),
        "the refusal must state BOTH self-source fixes (Fd + Blocking) and the polled step() seam; got: {msg}"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "a refused live run never fires the node"
    );
    // The FIRST refusing run_live ran the collect pass and queried
    // external_source() exactly once (reading the HostDriven source).
    assert_eq!(
        queries.load(Ordering::Acquire),
        1,
        "the first refusing run_live queried external_source() exactly once"
    );

    // The refusal is STICKY. A second `run_live` on the same refused
    // runtime must return the SAME refusal Err — the guard re-returns the recorded
    // refusal BEFORE the collect pass, so it never re-queries the source nor falls
    // through the idempotent resume branch to `Ok`.
    let err2 = runtime
        .run_live(&running)
        .expect_err("a refused runtime must REFUSE AGAIN on a second run_live, never resume Ok");
    match err2 {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            // The sticky re-return carries the EXACT same offender list.
            assert_eq!(
                nodes.as_slice(),
                [("camera".to_string(), cerulion_core::InertReason::HostDriven)],
                "the sticky re-refusal must name EXACTLY the offending node + reason; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    // The sticky re-return did NOT re-collect — external_source() was
    // NOT queried a second time (proves the guard short-circuits before the pass).
    assert_eq!(
        queries.load(Ordering::Acquire),
        1,
        "the sticky re-refusal must NOT re-query external_source() (re-return, not re-collect)"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the sticky-refused second run_live still never fires the node"
    );
}

// ---------------------------------------------------------------------------
// 4. REFUSAL (4c, multi): run_live names ALL host-driven nodes in ONE error
// (HostDriven AND None), so the operator fixes them in a single pass.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn run_live_refuses_multi_host_driven_nodes_in_one_error() {
    // Two offenders: an explicit HostDriven ("camera") and a no-override None
    // ("imu"). Both must be named in the single refusal error.
    let cam = ExtProducer::new(ExternalSource::HostDriven, PublishMode::Counter);
    let mut imu = ExtProducer::new(ExternalSource::HostDriven, PublishMode::Counter);
    imu.src = None; // external_source() → None

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "egr_refuse_multi".to_string(),
        prefix: "egrrm".to_string(),
        nodes: vec![producer_only_node("camera"), producer_only_node("imu")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("camera".to_string(), Box::new(cam));
    factories.insert("imu".to_string(), Box::new(imu));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build multi-refusal graph");

    let running = AtomicBool::new(false);
    let err = runtime
        .run_live(&running)
        .expect_err("two host-driven external nodes must be REFUSED on the live path");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            // Exact ordered (node, reason) list — both offenders, in
            // declaration order (`camera` HostDriven then `imu` NoSource), in ONE
            // error (no iterate-one-at-a-time). The two arms carry DISTINCT reasons
            // (an explicit HostDriven vs a forgotten override → NoSource).
            assert_eq!(
                nodes.as_slice(),
                [
                    ("camera".to_string(), cerulion_core::InertReason::HostDriven),
                    ("imu".to_string(), cerulion_core::InertReason::NoSource),
                ],
                "ONE error must name BOTH offenders + reasons in declaration order; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// 4b. MIXED-REASON REFUSAL: a graph with a HostDriven node AND an
// invalid-fd node is refused in ONE aggregated error naming BOTH offenders with
// DISTINCT reasons (host-driven / invalid fd). Sticky re-refusal holds for the
// new reasons; neither node fires; all sibling bindings are torn down.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn run_live_refuses_mixed_reason_nodes_in_one_error_with_distinct_reasons() {
    // camera = explicit HostDriven; lidar = an invalid fd (-1, rejected at
    // collect by FdSource::non_owning → None). Declaration order camera→lidar.
    let cam_fires = Arc::new(AtomicU64::new(0));
    let lidar_fires = Arc::new(AtomicU64::new(0));
    let cam = ExtProducer::new(ExternalSource::HostDriven, PublishMode::Counter)
        .with_fires(Arc::clone(&cam_fires));
    let lidar = ExtProducer::new(ExternalSource::Fd(-1), PublishMode::Counter)
        .with_fires(Arc::clone(&lidar_fires));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "egr_refuse_mixed".to_string(),
        prefix: "egrrx".to_string(),
        nodes: vec![producer_only_node("camera"), producer_only_node("lidar")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("camera".to_string(), Box::new(cam));
    factories.insert("lidar".to_string(), Box::new(lidar));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build mixed-reason refusal graph");

    let running = AtomicBool::new(false);
    let err = runtime
        .run_live(&running)
        .expect_err("a mixed HostDriven + invalid-fd graph must be REFUSED on the live path");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch {
            ref graph,
            ref nodes,
        } => {
            assert_eq!(graph, "egr_refuse_mixed", "the refusal names the graph");
            // ONE error names BOTH offenders in declaration order, each with its
            // OWN reason — the headline inert-at-launch contract.
            assert_eq!(
                nodes.as_slice(),
                [
                    ("camera".to_string(), cerulion_core::InertReason::HostDriven),
                    ("lidar".to_string(), cerulion_core::InertReason::InvalidFd),
                ],
                "ONE error must name BOTH offenders with DISTINCT reasons; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    // The rendered Display carries both `node (reason)` pairs verbatim.
    let msg = err.to_string();
    assert!(
        msg.contains("camera (host-driven)") && msg.contains("lidar (invalid fd)"),
        "the rendered refusal must show both distinct `node (reason)` pairs; got: {msg}"
    );

    // Neither offender fired; the collect pass tore down every binding.
    assert_eq!(cam_fires.load(Ordering::Relaxed), 0, "camera never fires");
    assert_eq!(lidar_fires.load(Ordering::Relaxed), 0, "lidar never fires");
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the mixed refusal EAGERLY clears every sibling binding"
    );

    // Sticky: a second run_live re-returns the SAME two-reason list (the guard
    // re-returns the recorded refusal, now proven to carry the NEW reasons too).
    let err2 = runtime
        .run_live(&running)
        .expect_err("a refused runtime must REFUSE AGAIN, carrying the same mixed reasons");
    match err2 {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            assert_eq!(
                nodes.as_slice(),
                [
                    ("camera".to_string(), cerulion_core::InertReason::HostDriven),
                    ("lidar".to_string(), cerulion_core::InertReason::InvalidFd),
                ],
                "the sticky re-refusal must carry the SAME mixed (node, reason) list; got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// 4c. ANTI-TAUTOLOGY CONTROL: a graph whose ONLY external node has a
// VALID `Fd` source is NOT refused — collect succeeds, the binding materializes,
// and a real pipe byte fires the node downstream. Proves the refusal is specific
// to provably-inert nodes, not a blanket rejection of every external graph.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn valid_fd_source_graph_is_not_refused_and_fires() {
    let pipe = Pipe::new();
    let seq = Arc::new(Mutex::new(Vec::<u64>::new()));
    // DrainDerive publishes `100 + last_byte`; a written byte of 5 → value 105.
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        PublishMode::DrainDerive(pipe.read),
    );
    let mut runtime =
        producer_consumer_graph("egr_valid_ctrl", "egrvc", producer, Arc::clone(&seq));

    // collect must SUCCEED (no refusal) and bind the one valid source.
    runtime
        .try_collect_external_sources_for_test()
        .expect("a valid Fd source must NOT be refused");
    assert_eq!(
        runtime.external_binding_count(),
        1,
        "the valid Fd source materializes exactly one binding"
    );

    // A real device byte fires the node → consumer records the derived value.
    // SAFETY: `pipe.write` is the valid open write end of a live pipe.
    let b: u8 = 5;
    let n = unsafe { libc::write(pipe.write, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
    assert_eq!(n, 1, "pipe write must succeed");
    let fired = step_until(&mut runtime, Duration::from_secs(2), || {
        !seq.lock().expect("seq lock").is_empty()
    });
    assert!(
        fired,
        "the valid external source must fire on the live path"
    );
    assert_eq!(
        seq.lock().expect("seq lock").as_slice(),
        [105],
        "delivery oracle: the fired node published 100 + drained byte (5) = 105"
    );
}

// ---------------------------------------------------------------------------
// 5. REFUSAL CLEANUP: a refused live run tears down a sibling
// Blocking source's helper thread (spawned earlier in the same collect pass)
// EAGERLY AT the refusal — the binding is cleared and the helper stopped BEFORE
// the refusal `Err` returns (observable without dropping the runtime), so a
// retained refused runtime does not keep the helper draining. Drop is then
// idempotent cleanup, not the teardown mechanism.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn refused_live_run_tears_down_sibling_blocking_helper() {
    /// Flips a shared flag on Drop — observes the helper thread's exit.
    struct ExitSignal(Arc<AtomicBool>);
    impl Drop for ExitSignal {
        fn drop(&mut self) {
            self.0.store(true, Ordering::Release);
        }
    }
    let exited = Arc::new(AtomicBool::new(false));
    let sig = ExitSignal(Arc::clone(&exited));
    // A bounded-wait closure that owns `sig`; its Drop (on thread exit) flips
    // `exited`. Never rings (no sender) — it just parks in a bounded recv.
    let (_tx, rx) = mpsc::channel::<()>();
    let closure = move || {
        let _sig = &sig;
        matches!(rx.recv_timeout(Duration::from_millis(50)), Ok(()))
    };

    // Declaration order: the Blocking node FIRST (collect spawns its helper),
    // the HostDriven node SECOND (collect refuses at the end of the pass).
    let bell = ExtProducer::new(
        ExternalSource::Blocking(Box::new(closure)),
        PublishMode::Counter,
    );
    let hd = ExtProducer::new(ExternalSource::HostDriven, PublishMode::Counter);
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "egr_refuse_cleanup".to_string(),
        prefix: "egrrc".to_string(),
        nodes: vec![producer_only_node("bell"), producer_only_node("hd")],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("bell".to_string(), Box::new(bell));
    factories.insert("hd".to_string(), Box::new(hd));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build cleanup graph");

    let running = AtomicBool::new(false);
    let err = runtime
        .run_live(&running)
        .expect_err("the HostDriven sibling must refuse the whole live run");
    match err {
        cerulion_core::TransportError::ExternalNodesInertAtLaunch { ref nodes, .. } => {
            // Exact list — ONLY the host-driven node "hd" is named (reason
            // HostDriven); the valid Blocking sibling "bell" must NOT appear.
            assert_eq!(
                inert_names(nodes),
                vec!["hd"],
                "the refusal names EXACTLY the host-driven node (not the valid Blocking sibling); got {nodes:?}"
            );
        }
        other => panic!("expected ExternalNodesInertAtLaunch, got {other:?}"),
    }
    // The sibling Blocking binding/helper (bell precedes hd, so it
    // WAS spawned before the refusal) is torn down EAGERLY AT the refusal —
    // observable on the STILL-ALIVE refused runtime, not deferred to Drop. The
    // binding is cleared...
    assert_eq!(
        runtime.external_binding_count(),
        0,
        "the refusal EAGERLY clears the sibling Blocking binding (no deferred-to-Drop leak)"
    );
    // ...and the detached helper observes the stop and exits — its closure owns
    // `sig`, whose Drop flips `exited` — WITHOUT dropping the runtime.
    assert!(
        wait_until(Duration::from_secs(2), || exited.load(Ordering::Acquire)),
        "the refusal must stop the sibling Blocking helper thread AT refusal (observable before drop)"
    );
    // The runtime is STILL ALIVE here (the eager teardown already ran); dropping
    // it now is idempotent cleanup, not the teardown mechanism.
    drop(runtime);
}

// ---------------------------------------------------------------------------
// 6. REPLAY = LIVE firewall (item 5): a LIVE Fd leg (non-Cerulion writer thread
// over a plain pipe) records the delivered sequence; a REPLAY leg feeds that
// recording downstream WITHOUT touching the fd (external_source() panics), and
// the delivered sequence is BYTE-IDENTICAL to the recording AND equals a
// HAND-COMPUTED oracle from the writer's known input vector.
// ---------------------------------------------------------------------------
#[test]
#[serial]
fn replay_feeds_recorded_output_without_touching_fd() {
    // The NON-Cerulion writer's known input vector (device bytes).
    const INPUT_VECTOR: [u8; 6] = [7, 42, 13, 99, 200, 1];
    // HAND ORACLE: the DrainDerive node publishes `100 + byte`. This is the
    // anti-tautology anchor — BOTH legs are compared to this, not to each other.
    let oracle: Vec<u64> = INPUT_VECTOR.iter().map(|&b| 100 + b as u64).collect();

    // ---- LIVE leg ----------------------------------------------------------
    let pipe = Pipe::new();
    let live_seq = Arc::new(Mutex::new(Vec::<u64>::new()));
    let producer = ExtProducer::new(
        ExternalSource::Fd(pipe.read),
        PublishMode::DrainDerive(pipe.read),
    );
    let mut live_rt =
        producer_consumer_graph("egr_replay_live", "egrrl", producer, Arc::clone(&live_seq));
    live_rt.collect_external_sources_for_test();

    // A genuine NON-Cerulion writer thread: writes one requested byte to the
    // plain pipe per handshake, so exactly one byte is readable per step
    // (deterministic one-byte→one-fire). The test drives the live seam.
    let (ask_tx, ask_rx) = mpsc::channel::<u8>();
    let (ack_tx, ack_rx) = mpsc::channel::<()>();
    let write_fd = pipe.write;
    let writer = std::thread::spawn(move || {
        while let Ok(b) = ask_rx.recv() {
            // SAFETY: `write_fd` is the pipe's valid open write end (the Pipe is
            // alive in the test thread until after join); one byte from a live
            // buffer.
            let n =
                unsafe { libc::write(write_fd, std::ptr::addr_of!(b) as *const libc::c_void, 1) };
            assert_eq!(n, 1, "writer: pipe write must succeed");
            ack_tx.send(()).expect("writer: ack");
        }
    });

    for (i, &b) in INPUT_VECTOR.iter().enumerate() {
        ask_tx.send(b).expect("ask writer to write byte");
        ack_rx.recv().expect("writer wrote the byte");
        let want_len = i + 1;
        let got = step_until(&mut live_rt, Duration::from_secs(3), || {
            live_seq.lock().expect("live_seq").len() >= want_len
        });
        assert!(
            got,
            "live leg: byte {i} (value {b}) must produce a delivery"
        );
    }
    drop(ask_tx); // let the writer thread finish
    writer.join().expect("writer thread must not panic");

    let recorded = live_seq.lock().expect("live_seq").clone();
    // ANCHOR: the live recording must equal the hand oracle (NOT a self-compare).
    assert_eq!(
        recorded, oracle,
        "live leg: the delivered sequence must equal the hand oracle (100 + input byte)"
    );
    drop(live_rt);

    // ---- REPLAY leg --------------------------------------------------------
    // A polled node replays the RECORDED output. Its `external_source()` PANICS,
    // so any accidental fd-path query in replay aborts the test — the fd is
    // never touched in replay.
    let replay_seq = Arc::new(Mutex::new(Vec::<u64>::new()));
    // Count queries on the replay leg too (mirror test 1) — the panic
    // guard proves external_source() is never CALLED; this counter proves the
    // same via a hard `== 0` even if the panic guard were ever weakened.
    let replay_queries = Arc::new(AtomicU64::new(0));
    let replay_producer = ExtProducer::new(
        ExternalSource::HostDriven,
        PublishMode::Replay(recorded.clone()),
    )
    .panic_on_query()
    .with_queries(Arc::clone(&replay_queries));
    let mut replay_rt = producer_consumer_graph(
        "egr_replay_replay",
        "egrrr",
        replay_producer,
        Arc::clone(&replay_seq),
    );

    // Polled trigger_external + step per recorded value — NO collect / run_live,
    // so the fd path is never consulted.
    for _ in 0..recorded.len() {
        replay_rt
            .trigger_external("producer")
            .expect("trigger_external");
        replay_rt.step(Duration::from_millis(1));
    }
    let replayed = replay_seq.lock().expect("replay_seq").clone();

    // FIREWALL: replay is BYTE-IDENTICAL to the live recording AND equals the
    // hand oracle (Principle #7 — Replay = Live), and the fd was never touched
    // (the panic guard did not trip).
    assert_eq!(
        replayed, recorded,
        "replay delivered sequence must be BYTE-IDENTICAL to the live recording"
    );
    assert_eq!(
        replayed, oracle,
        "replay delivered sequence must equal the hand oracle (anti-tautology anchor)"
    );
    // The fd path was NEVER consulted on the replay leg (mirror test 1).
    assert_eq!(
        replay_queries.load(Ordering::Acquire),
        0,
        "replay must NEVER query external_source() — the fd is untouched in replay"
    );
}
