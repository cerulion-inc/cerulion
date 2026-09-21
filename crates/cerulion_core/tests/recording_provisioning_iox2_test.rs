// SPDX-License-Identifier: AGPL-3.0-only
//! The RECORD-side tap provisioning raise, exercised
//! through the REAL build path — `GraphRuntime`'s `recorded_topics` seam (the
//! exact parameter `graph run --record` passes) must raise each recorded
//! topic's service `subscriber_max_buffer_size` ceiling to the scaled tap
//! depth, so bagd's open-only tap (whose port buffer resolves to the service
//! ceiling) gets a deep receive queue.
//!
//! Four provisioning pins + the lossless-burst e2e:
//! - the ceiling equals `recording_tap_buffer_depth(max_slice_len)` EXACTLY
//!   (attach requiring the full depth succeeds; depth+1 is rejected by
//!   iceoryx2 itself);
//! - a NON-recording build keeps the stock ceiling (the zero-behavior-change
//!   firewall control);
//! - a `multi_publisher_topics` topic keeps the shared cross-graph
//!   `MULTI_TOPIC_BUFFER_CEILING` (the Multi-skip arm);
//! - a LARGER existing ceiling is never clobbered (the `max(existing,
//!   tap_depth)` arm);
//! - the lossless burst: a deep intra-window burst recorded through the REAL
//!   build loses ZERO frames at the tap under the raise, while the stock-depth
//!   control PROVABLY loses (reverting the runtime.rs raise makes the lossless
//!   arm fail).
//!
//! All tests build live-deterministic runtimes over isolated per-test iceoryx2
//! SHM roots; `#[serial]` (live seam + WaitSet).

use std::collections::HashSet;
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::{
    recording_tap_buffer_depth, PublisherProvisioning, TopicServiceConfig,
    RECORDING_TAP_BUFFER_DEPTH, RECORDING_TAP_BUFFER_DEPTH_FLOOR,
};
use cerulion_core::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// Period producer publishing one Vector3 per fire (wire `sequence` is the
/// publisher's own counter, starting at 0 — the gap oracle).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct RecProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl RecProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A producer-only graph: one node, one Vector3 output with an EXPLICIT
/// tier-1 `max_slice_len` (so the tap-depth scaling input is deterministic).
fn producer_graph(
    prefix: &str,
    max_slice_len: Option<usize>,
    topic_override: Option<String>,
    multi: Vec<String>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: multi,
        name: None,
        identity: format!("rec_prov_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "producer".to_string(),
            node_type: "rec_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len,
                history_size: 0,
                topic: topic_override,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(RecProducerEntry::new()));
    (config, factories)
}

/// Build through the REAL recorded-topics seam (the exact
/// `build_live_deterministic_with_schema_hashes_and_policy` call
/// `run_graph_recording` makes), over an isolated per-test SHM root.
/// Returns the runtime AND the manager (for tap/probe attaches).
fn build_recorded(
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    subscriber_buffer_size: usize,
    recorded: Option<&HashSet<String>>,
) -> (GraphRuntime, Arc<TransportManager>) {
    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "cerulion_rec_prov_test".into(),
            clock: clock.clone(),
            subscriber_buffer_size,
            network: None,
        },
        iceoryx_test_config(),
    )
    .expect("init_for_test");
    let runtime = GraphRuntime::build_live_deterministic_with_schema_hashes_and_policy(
        config,
        factories,
        &mgr,
        clock,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        recorded,
    )
    .expect("build recorded graph");
    (runtime, mgr)
}

/// An opener REQUIRING exactly `ceiling` on the topic's buffer axis (the
/// iceoryx2 open-requirement probe): `for_topology` with `max_consumer_depth =
/// ceiling` and the producer-only graph's subscriber term (0 in-graph + 4
/// introspection headroom — matching what the graph provisioned, so the
/// subscriber/publisher axes can never mask a buffer-axis verdict).
fn attach_requiring(
    mgr: &TransportManager,
    topic: &str,
    ceiling: usize,
) -> Result<(), cerulion_core::TransportError> {
    mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            ceiling,
            0,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        ),
        ceiling,
    )
    .map(|_| ())
}

// ============================================================
// Pin 1 — the recorded ceiling equals the SCALED tap depth exactly.
// ============================================================

#[test]
#[serial]
fn recorded_topic_ceiling_is_exactly_the_scaled_tap_depth() {
    // Tier-1 YAML max_slice_len = 512 KiB → the SCALED arm of the depth rule:
    // 1 GiB / 512 KiB = 2048 (strictly between the 64 floor and 4096 target,
    // so a clamp-only variant fails here).
    const MSL: usize = 512 * 1024;
    let expected = recording_tap_buffer_depth(MSL);
    assert_eq!(expected, 2048, "the scaling-rule oracle for 512 KiB");

    let (config, factories) = producer_graph("rp1", Some(MSL), None, vec![]);
    let topic = "/rp1/producer/out";
    let recorded: HashSet<String> = [topic.to_string()].into();
    let (_runtime, mgr) = build_recorded(config, factories, 16, Some(&recorded));

    // A DEFAULT opener (stock 16 requirement) still attaches — the raise
    // preserves the dominance contract.
    mgr.create_subscriber(topic)
        .expect("default opener must attach to the raised-ceiling topic");
    // An opener requiring the FULL scaled depth attaches — the service really
    // was provisioned at recording_tap_buffer_depth(msl), which is what the
    // bagd tap's open-only (buffer = None → service ceiling) attach inherits.
    attach_requiring(&mgr, topic, expected)
        .expect("opener requiring the scaled tap depth must attach");
    // ...and depth+1 is rejected BY ICEORYX2 (pins EXACTLY the scaled value,
    // making the full-depth attach above non-vacuous).
    assert!(
        attach_requiring(&mgr, topic, expected + 1).is_err(),
        "an opener requiring depth+1 must be rejected — the ceiling must be \
         EXACTLY the scaled tap depth"
    );
}

// ============================================================
// Pin 2 — the zero-behavior-change firewall: no `--record`, no raise.
// ============================================================

#[test]
#[serial]
fn non_recording_build_keeps_the_stock_ceiling() {
    const MSL: usize = 512 * 1024;
    let (config, factories) = producer_graph("rp2", Some(MSL), None, vec![]);
    let topic = "/rp2/producer/out";
    // recorded = None — the normal (non-record) build.
    let (_runtime, mgr) = build_recorded(config, factories, 16, None);

    mgr.create_subscriber(topic)
        .expect("default opener attaches at the stock ceiling");
    // The tap depth must NOT have been provisioned: requiring it fails.
    assert!(
        attach_requiring(&mgr, topic, recording_tap_buffer_depth(MSL)).is_err(),
        "a non-recording build must NOT raise the ceiling to the tap depth"
    );
}

// ============================================================
// Pin 3 — the Multi-skip arm: a multi_publisher_topics topic keeps the
// shared cross-graph ceiling (a per-graph raise would break the
// open-by-equality contract for a second graph).
// ============================================================

#[test]
#[serial]
fn recorded_multi_publisher_topic_keeps_the_shared_ceiling() {
    let (config, factories) = producer_graph(
        "rp3",
        Some(64 * 1024),
        Some("/tf".to_string()),
        vec!["/tf".to_string()],
    );
    let recorded: HashSet<String> = ["/tf".to_string()].into();
    let (_runtime, mgr) = build_recorded(config, factories, 16, Some(&recorded));

    // The shared Multi constants still open (equality contract intact).
    mgr.create_subscriber_with_buffers(
        "/tf",
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            cerulion_core::transport::MULTI_TOPIC_BUFFER_CEILING,
            0,
            PublisherProvisioning::Multi,
            0,
            0,
        ),
        cerulion_core::transport::MULTI_TOPIC_BUFFER_CEILING,
    )
    .expect("the shared Multi opener must attach — the ceiling must stay the shared constant");
    // The tap-depth raise must have been SKIPPED: requiring it fails.
    assert!(
        attach_requiring(&mgr, "/tf", RECORDING_TAP_BUFFER_DEPTH).is_err(),
        "a recorded Multi topic must keep MULTI_TOPIC_BUFFER_CEILING — the \
         per-graph tap raise would break cross-graph open-by-equality"
    );
}

// ============================================================
// Pin 4 — max(existing, tap_depth): a LARGER existing ceiling is not
// clobbered down to the tap depth.
// ============================================================

#[test]
#[serial]
fn recorded_ceiling_never_clobbers_a_larger_existing_ceiling() {
    // An existing ceiling (8192) that EXCEEDS the tap depth (4096 target for a
    // small slice), so the recorded raise must KEEP 8192 — a variant using `=`
    // instead of `max` clobbers to 4096 and fails the 8192-requiring attach.
    const SUB_BUF: usize = 8192;
    const MSL: usize = 1024; // → tap depth = the 4096 target < 8192
    assert_eq!(recording_tap_buffer_depth(MSL), RECORDING_TAP_BUFFER_DEPTH);

    let (config, factories) = producer_graph("rp4", Some(MSL), None, vec![]);
    let topic = "/rp4/producer/out";
    let recorded: HashSet<String> = [topic.to_string()].into();
    let (_runtime, mgr) = build_recorded(config, factories, SUB_BUF, Some(&recorded));

    attach_requiring(&mgr, topic, SUB_BUF)
        .expect("the larger existing ceiling (8192) must survive the recorded raise");
    assert!(
        attach_requiring(&mgr, topic, SUB_BUF + 1).is_err(),
        "the ceiling must be EXACTLY the larger existing value"
    );
}

// ============================================================
// Pin 5 — the e2e proof: a deep intra-window burst recorded through
// the REAL build path loses ZERO frames at the tap under the raise, and
// PROVABLY loses under stock depth (the in-test control). Reverting the
// runtime.rs ceiling raise makes the lossless arm behave like the control —
// this test FAILS.
// ============================================================

/// Drain an open-only tap one owned sample at a time (borrow-budget-safe under
/// BOTH the recorded borrow ceiling and the stock default), returning the wire
/// sequences in arrival order — exactly the surface bagd's drain reads.
fn drain_tap_sequences(
    tap: &mut cerulion_core::transport::subscriber::CerulionSubscriber,
) -> Vec<u32> {
    let mut seqs = Vec::new();
    // `try_receive` drains the whole tap queue (FIFO) — the surviving-sequence
    // set is fixed by the queue state the burst produced, not the read method.
    tap.try_receive(|msg| {
        seqs.push(msg.header().sequence);
    })
    .expect("try_receive");
    seqs
}

/// One arm of the lossless-burst pair: build (recorded or not), attach the tap
/// BEFORE step 0 (bagd's taps-armed-before-GO handshake), burst `steps` live
/// steps with the tap NEVER drained (the stalled-recorder window), then drain
/// and return the observed wire sequences.
fn burst_arm(prefix: &str, recorded: bool, steps: usize) -> Vec<u32> {
    const MSL: usize = 64 * 1024; // → tap depth 4096 (the cap/target arm)
    let (config, factories) = producer_graph(prefix, Some(MSL), None, vec![]);
    let topic = format!("/{prefix}/producer/out");
    let recorded_set: HashSet<String> = [topic.clone()].into();
    let (mut runtime, mgr) = build_recorded(
        config,
        factories,
        16,
        if recorded { Some(&recorded_set) } else { None },
    );

    // The tap attaches open-only (bagd's exact call): its queue depth IS the
    // service ceiling — raised (4096) or stock (16).
    let mut tap = mgr
        .create_subscriber_open_only(&topic)
        .expect("open-only tap");

    // The stalled window: `steps` fires with the tap never drained.
    for _ in 0..steps {
        runtime.run_live_step_once_for_test(Duration::from_millis(5));
    }

    drain_tap_sequences(&mut tap)
}

#[test]
#[serial]
fn recorded_tap_survives_deep_burst_lossless_while_stock_depth_provably_loses() {
    const STEPS: usize = 60;

    // ---- Lossless arm: the recorded build's raised ceiling absorbs the
    // whole burst — every published frame reaches the tap, zero gaps.
    let seqs = burst_arm("rp5a", true, STEPS);
    assert!(
        seqs.len() > 16,
        "the burst must exceed the stock depth for the arm to be probative \
         (got only {} frames)",
        seqs.len()
    );
    let expect: Vec<u32> = (0..seqs.len() as u32).collect();
    assert_eq!(
        seqs, expect,
        "the raised tap queue must deliver EVERY published frame — \
         consecutive wire sequences from 0, zero loss"
    );

    // ---- Control arm: the SAME burst against a stock (16) ceiling loses the
    // head — iceoryx2 keeps only the newest 16 (this is the measured
    // silent-loss mechanism, reproduced). This arm is what the lossless arm
    // above degrades to if the runtime.rs ceiling raise is reverted.
    let ctl = burst_arm("rp5b", false, STEPS);
    assert_eq!(
        ctl.len(),
        16,
        "the stock-depth tap must hold exactly the service ceiling (16)"
    );
    let max = *ctl.last().expect("nonempty");
    assert!(
        max >= 20,
        "the control burst must have published well past the stock depth \
         (max seq {max})"
    );
    let expect_ctl: Vec<u32> = (max - 15..=max).collect();
    assert_eq!(
        ctl, expect_ctl,
        "the stock tap must observe ONLY the newest 16 — the head of the \
         burst was reclaimed before the tap could drain it (real loss)"
    );
}

// ============================================================
// The floored-depth warn. A recorded topic whose
// resolved max_slice_len is the 128 MiB default gets the SHALLOWEST tap
// queue (the depth floor), and the build LOUDLY warns exactly once per
// such topic (loud-over-silent: the user must SEE why their tap is
// shallow). A topic with a declared (smaller) msl scales above the floor
// and stays quiet — the anti-tautology control.
// ============================================================

const FLOOR_WARN_PHRASE: &str = "recorded topic's max_slice_len resolves to the";

#[test]
#[serial]
#[traced_test]
fn recorded_default_msl_topic_warns_once_on_floored_tap_depth() {
    // An explicit tier-1 value EQUAL to the 128 MiB tier-3 default reaches the
    // SAME `msl == DEFAULT_MAX_SLICE_LEN` branch an unset tf-class topic (whose
    // 128 MiB schema/OutputMeta default flows through) reaches — the depth
    // floors to 64, tripping the warn. Using an explicit value keeps the
    // trigger deterministic regardless of a schema's tier-2 default. The huge
    // slot is provisioned lazily (pools are demand-paged) and never published,
    // so it faults in ~nothing.
    let msl = cerulion_core::graph::config::DEFAULT_MAX_SLICE_LEN;
    // Oracle: the 128 MiB default really DOES floor the tap depth (else the
    // warn would not be probative).
    assert_eq!(
        recording_tap_buffer_depth(msl),
        RECORDING_TAP_BUFFER_DEPTH_FLOOR,
        "the 128 MiB default must floor the tap depth for this pin to be probative"
    );

    let (config, factories) = producer_graph("rp6", Some(msl), None, vec![]);
    let topic = "/rp6/producer/out";
    let recorded: HashSet<String> = [topic.to_string()].into();
    let (_runtime, _mgr) = build_recorded(config, factories, 16, Some(&recorded));

    assert!(
        logs_contain(FLOOR_WARN_PHRASE),
        "a default-msl recorded topic must emit the loud floored-tap-depth warn"
    );
    // Exactly once: this recorded set has ONE topic and the raise loop visits
    // each recorded topic once per build — a count > 1 would be a loop/dedup bug.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(FLOOR_WARN_PHRASE))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected EXACTLY ONE floored-tap-depth warn, got {n}; lines: {lines:#?}"
            ))
        }
    });
}

#[test]
#[serial]
#[traced_test]
fn recorded_declared_msl_topic_does_not_warn() {
    // 64 KiB → tap depth 4096 (the cap), well above the floor → quiet. The
    // anti-tautology control: the warn is tied to the 128 MiB default, not to
    // "any recorded topic".
    let (config, factories) = producer_graph("rp7", Some(64 * 1024), None, vec![]);
    let topic = "/rp7/producer/out";
    let recorded: HashSet<String> = [topic.to_string()].into();
    let (_runtime, _mgr) = build_recorded(config, factories, 16, Some(&recorded));

    assert!(
        !logs_contain(FLOOR_WARN_PHRASE),
        "a topic with a declared (non-default) max_slice_len must NOT warn"
    );
}

// ============================================================
// The RECORD-side de-phantom pin: a producer whose tick Errs
// every 3rd fire (the macro discard class — loan, defer, drop
// without publish) records a stream with NO wire-sequence gaps, so bagd's
// gap detector (which books every gap as frames_lost) sees ZERO phantom
// losses. With a loan-time sequence stamp every discarded tick would burn
// a number: the tap would observe seqs [0,1,3,4,6,...] and bagd book the
// missing ones as lost frames that never existed (measured on a
// tf-class topic: 2,264 "lost" == exactly the collapsed ticks). Plus the
// determinism arm: two runs are byte-identical (seq AND payload).
// ============================================================

/// Period producer that Errs every 3rd tick AFTER writing its output —
/// the macro publish-on-success inversion discards those loans.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DiscardingProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl DiscardingProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        if self.n.is_multiple_of(3) {
            return Err(NodeError::Logic("every 3rd tick discards".to_string()));
        }
        Ok(())
    }
}

/// One discard-heavy recorded run: build through the REAL recorded-topics
/// seam, tap open-only BEFORE step 0 (bagd's taps-armed-before-GO), step
/// `steps` times, drain `(sequence, x)` pairs in arrival order — exactly
/// the surface bagd's gap detector reads.
fn discard_arm(prefix: &str, steps: usize) -> Vec<(u32, f64)> {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec![],
        name: None,
        identity: format!("rec_prov_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "producer".to_string(),
            node_type: "discarding_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: Some(64 * 1024),
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DiscardingProducerEntry::new()),
    );
    let topic = format!("/{prefix}/producer/out");
    let recorded_set: HashSet<String> = [topic.clone()].into();
    let (mut runtime, mgr) = build_recorded(config, factories, 16, Some(&recorded_set));

    let tap = mgr
        .create_subscriber_open_only(&topic)
        .expect("open-only tap");

    for _ in 0..steps {
        runtime.run_live_step_once_for_test(Duration::from_millis(5));
    }

    // Drain (sequence, x) — x is Vector3's first fixed field at frame
    // bytes [32..40] (f64 LE after the 32-byte WireHeader).
    let mut out = Vec::new();
    // `try_receive` drains the whole tap queue (FIFO). `msg.payload()` is the
    // body AFTER the 32-byte header, so Vector3's `x` is body bytes [0..8].
    tap.try_receive(|msg| {
        let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("x bytes"));
        out.push((msg.header().sequence, x));
    })
    .expect("try_receive");
    out
}

#[test]
#[serial]
fn discard_heavy_recorded_stream_is_gapless_and_deterministic() {
    const STEPS: usize = 45;
    let frames = discard_arm("rp8a", STEPS);

    // The arm must be probative: enough committed frames that the discard
    // class engaged repeatedly (L committed implies ~L/2 discards).
    assert!(
        frames.len() >= 15,
        "need a discard-heavy run to be probative (got {} frames)",
        frames.len()
    );

    // HAND ORACLE (not a self-compare): tick n writes x = n and discards
    // every n % 3 == 0, so the committed payload stream is exactly the
    // non-multiples of 3 in order — and the committed SEQUENCE
    // stream is exactly 0..L with NO gaps (with a loan-time stamp: [0,1,3,4,6,...] —
    // each discard burns a number bagd books as a phantom lost frame).
    let expected: Vec<(u32, f64)> = (1u32..)
        .filter(|n| n % 3 != 0)
        .take(frames.len())
        .enumerate()
        .map(|(i, n)| (i as u32, n as f64))
        .collect();
    assert_eq!(
        frames, expected,
        "recorded stream must be gap-free (consecutive sequences) with \
         exactly the committed payloads — any sequence gap here is a \
         phantom loss bagd would mis-book"
    );

    // Determinism (Principle #7): a second identical run over a fresh SHM
    // root produces a byte-identical (seq, payload) stream.
    let frames2 = discard_arm("rp8b", STEPS);
    assert_eq!(
        frames2, frames,
        "two discard-heavy runs must produce byte-identical streams \
         (sequence field included)"
    );
}
