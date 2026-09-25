// SPDX-License-Identifier: AGPL-3.0-only
//! End-to-end `promise_within_ms` OUTPUT watchdog over real
//! iceoryx2 via `GraphRuntime::build_for_test` (per-test SHM root —
//! parallel-safe).
//!
//! Symmetric to the input `expect_within_ms` watchdog: a publisher that
//! declares `#[output(promise_within_ms = N)]` promises to publish on that
//! output at least every N ms. The
//! scheduler has `set_promise_within` / `step()` / counter, and `GraphRuntime`
//! wires it from `OutputMeta.promise_within_ms`: every successful
//! send writes the publish time into the shared window anchor (the publisher's
//! `record_promise_within_published`), so `step()` resets the window on real
//! publishes and counts a miss when the producer is too slow.
//!
//! Rate-mismatch design (deterministic under `VirtualClock`): a producer
//! slower than its own promise window trips the watchdog; one faster than the
//! window keeps it quiet (the sequence-advance observation works). Counts are
//! bit-identical across runs (Principle #7 — `clock.now_ns()` on the
//! scheduler clock, never wall time).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{MacroPolicy, NodeEntry, NodeInfo, OutputMeta};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// --- Producers (declare a 30 ms publish promise; vary the actual rate) ------

/// Fast: publishes every 5 ms — keeps its 30 ms promise comfortably.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FastPromiseProducer {
    #[output(promise_within_ms = 30)]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FastPromiseProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Slow: publishes every 100 ms — breaks its 30 ms promise repeatedly.
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct SlowPromiseProducer {
    #[output(promise_within_ms = 30)]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SlowPromiseProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Minimal drain consumer (no QoS on the input — isolates the OUTPUT
/// watchdog) so the produced topic has an in-graph consumer edge.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PlainDrainConsumer {
    #[input]
    inp: Vector3,
    last: f64,
}
#[cerulion_node_impl]
impl PlainDrainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        Ok(())
    }
}

// --- Graph builder ---------------------------------------------------------

fn promise_graph(producer_type: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "promise_within_test".to_string(),
        prefix: "pw".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: producer_type.to_string(),
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
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "drain".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let producer: Box<dyn NodeEntry> = match producer_type {
        "fast" => Box::new(FastPromiseProducerEntry::new()),
        "slow" => Box::new(SlowPromiseProducerEntry::new()),
        other => panic!("unknown producer type {other}"),
    };
    // `build_for_test` keys the factory map by NODE ID (not node_type).
    factories.insert("prod".to_string(), producer);
    factories.insert("cons".to_string(), Box::new(PlainDrainConsumerEntry::new()));
    (config, factories)
}

/// Run for `steps` 5 ms steps; return the producer's `promise_within_missed_count`.
fn run_promise_within(producer_type: &str, steps: usize) -> u64 {
    let (config, factories) = promise_graph(producer_type);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build promise graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    runtime
        .node_handle("prod")
        .unwrap()
        .promise_within_missed_count()
}

// --- Tests -----------------------------------------------------------------

#[test]
fn output_watchdog_fires_when_producer_too_slow() {
    // 100 ms publish rate vs a 30 ms promise → misses between publishes.
    let misses = run_promise_within("slow", 40);
    assert!(
        misses > 0,
        "a 100 ms producer must break its 30 ms promise_within (got {misses})"
    );
}

#[test]
fn output_watchdog_quiet_when_producer_keeps_promise() {
    // 5 ms publish rate vs a 30 ms promise → every window has a fresh publish;
    // the sequence-of-sends keeps the anchor fresh, so no miss.
    let misses = run_promise_within("fast", 40);
    assert_eq!(
        misses, 0,
        "a 5 ms producer must keep its 30 ms promise_within (got {misses})"
    );
}

#[test]
fn promise_within_counter_is_deterministic() {
    let a = run_promise_within("slow", 40);
    let b = run_promise_within("slow", 40);
    assert_eq!(
        a, b,
        "promise_within miss count must be deterministic across runs"
    );
}

/// Hand-stamped Vector3 wire frame (header + zeroed fixed payload) — the
/// shape a raw-FFI node re-publishes through `publish_raw`.
fn vector3_frame(seq: u32) -> Vec<u8> {
    use cerulion_core::message::ShmMessage;
    use cerulion_core::wire::WireHeader;
    let payload_len = <Vector3 as ShmMessage>::WIRE_FIXED_SIZE;
    let header = WireHeader::new(
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        seq,
        u64::from(seq) * 1_000_000,
    );
    let mut buf = vec![0u8; WireHeader::SIZE + payload_len];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    let total = (WireHeader::SIZE + payload_len) as u32;
    buf[8..12].copy_from_slice(&total.to_le_bytes());
    buf
}

#[test]
fn publish_raw_resets_promise_within() {
    // The macro e2e path only exercises the
    // `OutputProxy::drop` reset site. `publish_raw` (the raw-FFI re-publish
    // path) is a DISTINCT `record_promise_within_published` call site — pin it
    // directly so deleting that line fails this test.
    //
    // Sentinel design: the anchor starts at u64::MAX (a value no real clock
    // returns); a successful publish_raw overwrites it with clock.now_ns().
    // Asserting it is no longer MAX is robust regardless of the test clock's
    // value, and fails iff the reset call is removed.
    let topic = "pwraw/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let anchor = Arc::new(AtomicU64::new(u64::MAX));
    publisher.register_promise_within_for_test(Arc::clone(&anchor));

    assert_eq!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "anchor untouched before any publish"
    );

    publisher
        .publish_raw(&vector3_frame(1))
        .expect("publish_raw should succeed");

    assert_ne!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "publish_raw must reset the promise_within anchor to clock.now_ns() \
         (record_promise_within_published call site at publisher.rs publish_raw)"
    );
}

#[test]
fn send_overflow_frame_resets_promise_within() {
    // `send_overflow_frame` (the variable-slice
    // loan-spill re-send) is byte-identical to the publish_raw site but
    // reachable only via a real overflow in production. Pin it directly via the
    // test-helper so deleting its `record_promise_within_published()` line
    // fails this test. Same sentinel design. (One of the FOUR
    // reset sites — the list, and which test pins each, lives on
    // `record_promise_within_published`; `send_raw_loan`'s arm is below.)
    let topic = "pwovf/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let anchor = Arc::new(AtomicU64::new(u64::MAX));
    publisher.register_promise_within_for_test(Arc::clone(&anchor));

    // Split a valid Vector3 frame into header (WireHeader::SIZE) + payload —
    // the same shape OutputProxy::drop hands send_overflow_frame on a spill.
    let frame = vector3_frame(1);
    let header = &frame[..cerulion_core::wire::WireHeader::SIZE];
    let payload = &frame[cerulion_core::wire::WireHeader::SIZE..];

    publisher
        .send_overflow_frame_for_test(header, payload)
        .expect("send_overflow_frame should succeed");

    assert_ne!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "send_overflow_frame must reset the promise_within anchor \
         (record_promise_within_published call site at publisher.rs send_overflow_frame)"
    );
}

/// **The FOURTH send path: `send_raw_loan` resets the `promise_within` window
/// AND bumps the `block` outstanding mirror.**
///
/// `send_raw_loan` is the rmw bridge's publish (flatten-into-loan, plus the
/// loaned-message borrow window). It had ZERO test callers anywhere in the tree:
/// `max_loaned_samples_test` and `clean_orphan_port_tag_test` loan and DROP
/// without sending, and every other caller is in `rmw_cerulion`'s own `api`
/// module. The reason nobody noticed is that `record_promise_within_published`'s
/// own doc asserted it was "covered e2e by `promise_within_iox2_test`" — the
/// three arms above, none of which touch it — and named it as the path
/// `OutputProxy::drop` publishes through, which it is not (the proxy sends its
/// own `SampleMut` and records itself). That doc is rewritten in the same commit
/// as this arm; the stale claim is what made the gap invisible.
///
/// Both invariants are asserted, because they fail differently:
///
/// * the promise anchor uses the two sibling arms' `u64::MAX` SENTINEL +
///   `assert_ne!`. `TestTransport` exposes no clock accessor, so an exact-value
///   oracle is not reachable without a new seam; `u64::MAX` is a value no clock
///   returns, and the arm reads it BEFORE the send too, so the `assert_ne!` is
///   attributable to the send rather than to a pre-set anchor.
/// * the `block` outstanding mirror is an EXACT hand oracle (`0` before, `1`
///   after). Every frame entering a subscriber queue must bump it, or a `block`
///   topic publishing through this path would hand the producer's pre-fire read
///   a depth short of the truth and silently overflow (Principle #6).
///
/// **Scope**, stated because the arm reads stronger than it is: both
/// calls are production-INERT on the rmw surface today.
/// `register_promise_within` and `register_block_outstanding` have one caller
/// each (`GraphRuntime::build_with_scheduler`, the shared build path), and a
/// graph `block` topic is SingleWriter, so no
/// shipping configuration reaches this path with either mirror registered. This
/// is a GUARD on the invariant a future `block`-wired rmw topic would need — and
/// the deletion of a doc claim that would have made the next reader skip it
/// again.
#[test]
fn send_raw_loan_resets_promise_within_and_bumps_block_outstanding() {
    use std::mem::MaybeUninit;

    let topic = "pwloan/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let anchor = Arc::new(AtomicU64::new(u64::MAX));
    // The `block` mirror is a `CreditWord` — LOCAL here (this
    // is a same-process edge), and `outstanding()` is a plain `Acquire` load
    // on it. The promise-within anchor is
    // a different seam and stays a plain `Arc<AtomicU64>`.
    let outstanding = cerulion_core::credit::CreditWord::local(u32::MAX);
    publisher.register_promise_within_for_test(Arc::clone(&anchor));
    publisher.register_block_outstanding_for_test(outstanding.clone());

    assert_eq!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "the promise anchor must be untouched before any send"
    );
    assert_eq!(
        outstanding.outstanding(),
        0,
        "nothing is outstanding before any send"
    );

    // The shape the rmw bridge builds: an exact-size uninitialized loan, filled
    // end to end, promoted, then sent. Every byte is written, which is
    // `assume_init`'s documented contract.
    let frame = vector3_frame(1);
    let mut loan = publisher
        .loan_raw_uninit(frame.len())
        .expect("loan_raw_uninit should succeed");
    let slot: &mut [MaybeUninit<u8>] = loan.bytes_uninit_mut();
    assert_eq!(
        slot.len(),
        frame.len(),
        "loan_raw_uninit must return EXACTLY the requested length — the rmw cursor's \
         `require_full` proof is tied to this slice length"
    );
    for (dst, src) in slot.iter_mut().zip(frame.iter()) {
        dst.write(*src);
    }
    // SAFETY: every byte of the slot was just written from `frame`, whose length
    // was asserted equal to the slot length immediately above.
    let loan = unsafe { loan.assume_init() };
    // The recipient COUNT is deliberately not asserted: the liveness
    // observer can hold a listener-less data-only tap on any topic, so "how many
    // subscribers did this reach" is a timing-dependent number here and not part
    // of the contract under test.
    publisher
        .send_raw_loan(loan)
        .expect("send_raw_loan should succeed");

    assert_ne!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "send_raw_loan must reset the promise_within anchor to clock.now_ns() \
         (the record_promise_within_published call site in send_raw_loan)"
    );
    assert_eq!(
        outstanding.outstanding(),
        1,
        "send_raw_loan must bump the block outstanding mirror by exactly one — a frame was \
         published, so every registered consumer mirror is bumped once (the \
         record_block_published call site in send_raw_loan)"
    );
}

/// **A FAILED `send_raw_loan` records nothing — and the fault seam that proves
/// it finally has a caller.**
///
/// The companion to the arm above, and the half that matters more for `block`:
/// the outstanding mirror is never decremented by the publisher, so a bump on a
/// frame that never reached a queue is a PERMANENT overcount — the producer's
/// pre-fire read would believe a consumer is one deeper than it is, forever. The
/// promise anchor fails the other way: a reset on a failed send would silence a
/// watchdog miss the operator is owed.
///
/// `fault_inject_send_raw_loan` (`publisher.rs`) had zero callers repo-wide,
/// so the early-return arm it exists to drive was unexecuted. It returns BEFORE
/// `sample.send()`, which drops the loan and releases the SHM slot without
/// publishing — the error-path contract (no partial frame on the wire). The arm
/// also pins the seam's FIRE-ONCE semantics: the next send succeeds and records
/// normally, which is what makes the two `assert_eq!`s below attributable to the
/// injected failure rather than to a publisher that had stopped recording at all.
#[test]
fn a_failed_send_raw_loan_records_neither_the_promise_nor_the_block_mirror() {
    use std::mem::MaybeUninit;

    let topic = "pwloanfail/out";
    let tt = TestTransport::with_buffer_size(8);
    let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(256), 0);
    let anchor = Arc::new(AtomicU64::new(u64::MAX));
    // The `block` mirror is a `CreditWord` — LOCAL here (this
    // is a same-process edge), and `outstanding()` is a plain `Acquire` load
    // on it. The promise-within anchor is
    // a different seam and stays a plain `Arc<AtomicU64>`.
    let outstanding = cerulion_core::credit::CreditWord::local(u32::MAX);
    publisher.register_promise_within_for_test(Arc::clone(&anchor));
    publisher.register_block_outstanding_for_test(outstanding.clone());

    let frame = vector3_frame(1);
    let fill = |publisher: &mut cerulion_core::CerulionPublisher| {
        let mut loan = publisher
            .loan_raw_uninit(frame.len())
            .expect("loan_raw_uninit should succeed");
        let slot: &mut [MaybeUninit<u8>] = loan.bytes_uninit_mut();
        assert_eq!(slot.len(), frame.len(), "exact-size loan");
        for (dst, src) in slot.iter_mut().zip(frame.iter()) {
            dst.write(*src);
        }
        // SAFETY: every byte of the slot was just written from `frame`, whose
        // length was asserted equal to the slot length immediately above.
        unsafe { loan.assume_init() }
    };

    publisher.fault_inject_send_raw_loan();
    let loan = fill(&mut publisher);
    let err = publisher
        .send_raw_loan(loan)
        .expect_err("the armed fault must make this send FAIL");
    assert!(
        matches!(err, TransportError::Publish { .. }),
        "a failed send must surface as a Publish error, got {err:?}"
    );
    assert_eq!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "a frame that never reached a queue must NOT reset the promise_within window — \
         doing so would silence a miss the operator is owed"
    );
    assert_eq!(
        outstanding.outstanding(),
        0,
        "a failed send must NOT bump the block outstanding mirror: nothing decrements it on \
         the publisher side, so one spurious bump is a permanent depth overcount"
    );

    // FIRE-ONCE: the seam disarms itself, so the very next send goes through and
    // records both — which is what makes the two assertions above attributable
    // to the injected failure rather than to a publisher that records nothing.
    let loan = fill(&mut publisher);
    publisher
        .send_raw_loan(loan)
        .expect("the fault is fire-once — the next send must succeed");
    assert_ne!(
        anchor.load(Ordering::Acquire),
        u64::MAX,
        "the successful send records the promise window"
    );
    assert_eq!(
        outstanding.outstanding(),
        1,
        "the successful send bumps the outstanding mirror exactly once"
    );
}

// --- c2: e2e reactable PromiseWithinEvent drain -----------

/// Hand-written PERIODIC producer that declares a `promise_within_ms = 30`
/// output, publishes only every 20th tick (100 ms — far slower than its
/// promise), and DRAINS the edge-triggered `PromiseWithinEvent` in its tick
/// body via `ctx.take_promise_within_event`. Proves the OUTPUT-side e2e wiring:
/// the runtime mints the per-node QoS store → injects it here → the scheduler
/// `push_*`es from the `output_promise_within` loop on a miss → the producer
/// drains it. Each sparse publish resets the promise anchor (via
/// `OutputProxy::drop` → `record_promise_within_published`), rearming the edge
/// latch — so multiple silence regimes fire multiple events.
struct DrainPromiseProducer {
    context: Option<NodeContext>,
    tick_n: u64,
    events_seen: Arc<AtomicU64>,
}

impl DrainPromiseProducer {
    fn new(events_seen: Arc<AtomicU64>) -> Self {
        Self {
            context: None,
            tick_n: 0,
            events_seen,
        }
    }
}

impl NodeEntry for DrainPromiseProducer {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::with_meta(
            Vec::new(),
            vec![OutputMeta::new(
                "out".to_string(),
                <Vector3 as ShmMessage>::SCHEMA_HASH,
                <Vector3 as ShmMessage>::MAX_SLICE_LEN,
            )
            .with_promise_within_ms(30)],
        )
        .with_policy(MacroPolicy::Period { period_ms: 5 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        // Publish every 20th tick (100 ms) — far slower than the 30 ms promise,
        // so the watchdog misses between publishes; each publish rearms it.
        let publish = self.tick_n.is_multiple_of(20);
        self.tick_n += 1;
        if let Some(ctx) = self.context.as_mut() {
            if publish {
                if let Some(pubr) = ctx.publisher_mut("out") {
                    let mut proxy = pubr.loan_proxy::<Vector3>()?;
                    proxy.x = 1.0;
                    // proxy drops here → publish → record_promise_within_published
                    // resets the shared anchor → the edge latch rearms.
                }
            }
            // Periodic node → this drains the watchdog event the SAME step the
            // miss is detected (push runs before evaluate_node in `step()`).
            if ctx.take_promise_within_event("out").is_some() {
                self.events_seen.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(())
    }

    fn shutdown(&mut self) -> TransportResult<()> {
        Ok(())
    }
}

/// Build a `DrainPromiseProducer` → `PlainDrainConsumer` graph, run `steps`
/// 5 ms steps, return `(events_drained_in_body, promise_within_missed_count)`.
fn run_drain_promise(steps: usize) -> (u64, u64) {
    let events_seen = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "promise_within_drain_test".to_string(),
        prefix: "pwd".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: "drainpromise".to_string(),
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
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "drain".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod".to_string(),
        Box::new(DrainPromiseProducer::new(Arc::clone(&events_seen))),
    );
    factories.insert("cons".to_string(), Box::new(PlainDrainConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build promise drain graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(5));
    }
    let misses = runtime
        .node_handle("prod")
        .unwrap()
        .promise_within_missed_count();
    (events_seen.load(Ordering::Relaxed), misses)
}

#[test]
fn promise_within_event_drained_in_node_body_e2e() {
    // Sparse publishes (every 100 ms) vs a 30 ms promise over 400 ms (80 steps):
    // multiple silence regimes, each publish rearms the latch.
    let (events, misses) = run_drain_promise(80);
    assert!(
        misses > 0,
        "sparse publishes must trip the promise watchdog (misses={misses})"
    );
    assert!(
        events >= 1,
        "the edge-triggered event must reach the producer body e2e (events={events})"
    );
    assert!(
        events < misses,
        "edge-trigger: strictly fewer events than counter bumps \
         (events={events}, misses={misses})"
    );
    assert!(
        events >= 2,
        "sparse publishes rearm the latch across regimes — fire again (events={events})"
    );
}

#[test]
fn promise_within_event_drain_is_deterministic_e2e() {
    // Principle #7: identical (events, misses) across two runs.
    let a = run_drain_promise(80);
    let b = run_drain_promise(80);
    assert_eq!(
        a, b,
        "e2e promise event drain must be deterministic across runs"
    );
}
