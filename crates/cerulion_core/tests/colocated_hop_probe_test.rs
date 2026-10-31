// SPDX-License-Identifier: AGPL-3.0-only
//! What one in-process hop costs on the queued path, and what each part of it
//! is.
//!
//! # Status
//!
//! A MEASUREMENT, not a gate. The two `#[ignore]`d arms print a table and
//! assert only that every row ran to its full sample count; no CI job passes
//! `--ignored`, so the arms run when someone runs them, on a quiet machine:
//!
//! ```bash
//! cargo test -p cerulion_core --test colocated_hop_probe_test --release \
//!     -- --ignored --nocapture --test-threads=1
//! ```
//!
//! The two plain arms check the statistics the table is printed with, against
//! hand-computed values, and run on every pass.
//!
//! # The rows
//!
//! Every row is taken at two payload sizes, 64 B and 1 MiB. Every payload is
//! LOANED and only its first byte is touched, so no row carries an O(n) fill.
//!
//! - `queued-hop`: a period producer at one level and a data-triggered consumer
//!   at the next, one process, driven by `GraphRuntime::step`. The producer
//!   stamps a wall reading into the frame as the last statement of its tick and
//!   the consumer reads it as the first statement of its own, so the row is
//!   everything between the two: the commit and send, the end of the
//!   producer's level, the consumer's trigger drain, its fire decision, and the
//!   receive that hands it the frame.
//! - `queued-chain`: the same stamp carried through a four-node linear chain,
//!   source to two relays to a sink, one node per level. The row is the
//!   source-to-sink time over three hops; each relay's own tick (a loan, a
//!   header copy, a commit) is inside it, as it is in any real chain.
//! - `consumer-receive`: one frame pending on a graph-style subscriber, then
//!   one `try_view` that receives it and builds the typed view. In a graph the
//!   two halves are split: the level-boundary drain receives the frame and the
//!   tick's `try_view` serves the frame it froze. The row is their sum, which
//!   is what a consumer pays to take a frame off its own queue.
//! - `library-receive`: the shared-memory library's own receive of the same
//!   size of frame, with nothing of this crate on the timed path. Batch timed.
//! - `publish`: one loan, one first-byte touch and one commit through this
//!   crate's publisher, to one subscriber. Batch timed. Every hop keeps it,
//!   because a hop's frame is published whatever happens after.
//!
//! Each row prints the median gap between distinct readings as `grain`. On a
//! host whose clock ticks coarsely (Apple silicon: 41.667 ns), a per-operation
//! row's grain reads that tick and its median is good to one tick and no finer;
//! a batch-timed row reads a grain near 1, and `queued-chain/hop`, a third of
//! a per-operation reading, reads about a third of the tick. A row whose grain
//! is a large fraction of its median is at the clock's floor and is not a
//! measurement.
//!
//! # What is not measured here
//!
//! A wake. Every row runs on a thread that never parks, so the cost of waking
//! a parked executor is in none of them. The queued rows are therefore the
//! cheapest a hop gets on this path, not what a hop costs on a live, idling
//! graph.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;

/// The two payload sizes every row sweeps.
const SIZES: [usize; 2] = [64, 1024 * 1024];

/// Samples kept per per-operation row, after the warm-up. A batch-timed row
/// keeps one sample per batch, `ITERATIONS / BATCH` of them.
const ITERATIONS: usize = 20_000;

/// Samples discarded first, so no row is the first-touch cost of a fresh
/// mapping and a cold branch predictor.
const WARMUP: usize = 2_000;

/// Operations inside one timed region on the rows whose single operation is
/// near the clock's tick. Also the subscriber queue depth those rows need,
/// since a batch is published first and then received.
const BATCH: usize = 64;

/// Steps a graph row may take before it is declared short. A row that needs
/// more than this many steps to collect its samples is not firing once per
/// step, and its number would describe something else.
const STEP_CEILING: usize = 2 * (ITERATIONS + WARMUP);

/// Relay hops in the `queued-chain` graph.
const CHAIN_HOPS: u64 = 3;

/// The instant every stamp is relative to, so a nanosecond reading fits in a
/// `u64` and splits across two `u32` fields.
static REFERENCE: OnceLock<Instant> = OnceLock::new();

fn elapsed_ns() -> u64 {
    REFERENCE.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

/// The payload size the producing nodes loan, set by the row driving them.
static CURRENT_SIZE: AtomicUsize = AtomicUsize::new(SIZES[0]);

/// One stamp-to-read latency per sink tick, in nanoseconds.
static HOP_NS: Mutex<Vec<u64>> = Mutex::new(Vec::new());

// ---------------------------------------------------------------------------
// The statistics the table is printed with.
// ---------------------------------------------------------------------------

/// The `p`th percentile of an ascending sample, by nearest rank.
fn percentile(sorted: &[u64], p: f64) -> u64 {
    let rank = ((p / 100.0) * sorted.len() as f64).ceil() as usize;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

/// The median gap between distinct readings of an ascending sample: the grain
/// the clock delivered. 0 when the sample holds fewer than two distinct values.
///
/// The median rather than the smallest gap, because truncating a coarse tick to
/// whole nanoseconds puts two lattice points one nanosecond apart now and then,
/// and the smallest gap would read 1 on a sample that is quantised at 41.
fn median_gap(sorted: &[u64]) -> u64 {
    let mut gaps: Vec<u64> = sorted
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|d| *d > 0)
        .collect();
    if gaps.is_empty() {
        return 0;
    }
    gaps.sort_unstable();
    gaps[gaps.len() / 2]
}

#[test]
fn the_percentile_is_the_nearest_rank() {
    let one_to_hundred: Vec<u64> = (1..=100).collect();
    assert_eq!(percentile(&one_to_hundred, 50.0), 50);
    assert_eq!(percentile(&one_to_hundred, 99.0), 99);
    assert_eq!(percentile(&one_to_hundred, 100.0), 100);
    // Rank 0 clamps to the first element rather than underflowing.
    assert_eq!(percentile(&one_to_hundred, 0.0), 1);
    // Nearest rank rounds UP: the 50th percentile of four values is the second.
    assert_eq!(percentile(&[10, 20, 30, 40], 50.0), 20);
    assert_eq!(percentile(&[7], 99.0), 7);
}

#[test]
fn the_grain_reads_the_lattice_and_not_its_truncation_artifacts() {
    // A 41.667 ns tick truncated to whole nanoseconds: gaps of 41 and 42, with
    // repeats, and one 1 ns pair where two truncations land side by side.
    let quantised = [0, 0, 41, 83, 83, 125, 166, 167, 208, 250];
    assert_eq!(median_gap(&quantised), 41);
    // Batch-timed readings fall on every nanosecond.
    let fine = [100, 101, 102, 103, 104, 106];
    assert_eq!(median_gap(&fine), 1);
    // Nothing to measure a gap between.
    assert_eq!(median_gap(&[5, 5, 5]), 0);
    assert_eq!(median_gap(&[]), 0);
}

/// Print one row. Refuses a row that did not reach its full sample count,
/// since a short row is a different measurement wearing the same label.
fn report(label: &str, payload: usize, batch: usize, mut samples: Vec<u64>) {
    assert_eq!(
        samples.len(),
        ITERATIONS / batch,
        "{label} at {payload} B collected {} samples, not {}; the row did not run as built",
        samples.len(),
        ITERATIONS / batch
    );
    samples.sort_unstable();
    println!(
        "{label:<17} payload={payload:>8} batch={batch:>3} n={:>6} p50={:>7} ns p99={:>7} ns \
         min={:>7} ns grain={:>4} ns",
        samples.len(),
        percentile(&samples, 50.0),
        percentile(&samples, 99.0),
        samples[0],
        median_gap(&samples),
    );
}

/// Refuse a debug build: every row is a release figure.
fn require_release() {
    if cfg!(debug_assertions) {
        panic!("this measurement is meaningful only in a release build; rerun with --release");
    }
    println!(
        "host: {} {}; the grain column is the median gap between distinct readings",
        std::env::consts::OS,
        std::env::consts::ARCH
    );
}

// ---------------------------------------------------------------------------
// The graph rows' nodes.
// ---------------------------------------------------------------------------

/// Stamps a wall reading into the frame as the last statement of its tick.
/// The commit runs when the tick's output proxy drops, right after.
#[cerulion_node(period_ms = 1)]
struct HopSource {
    #[output]
    frame: Image,
}

#[cerulion_node_impl]
impl HopSource {
    fn tick(&mut self) -> Result<(), NodeError> {
        // The payload is LOANED and only its first byte is touched.
        let n = CURRENT_SIZE.load(Ordering::Relaxed);
        self.frame.step = 0;
        self.frame.is_bigendian = 0;
        self.frame
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.frame
            .set_encoding("hp")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .frame
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        let t = elapsed_ns();
        self.frame.height = (t >> 32) as u32;
        self.frame.width = (t & 0xFFFF_FFFF) as u32;
        Ok(())
    }
}

/// Forwards the source's stamp unchanged into a frame of its own.
#[cerulion_node]
struct HopRelay {
    #[input(trigger)]
    frame: Image,
    #[output]
    out: Image,
}

#[cerulion_node_impl]
impl HopRelay {
    fn tick(&mut self) -> Result<(), NodeError> {
        let (height, width) = (self.frame.height, self.frame.width);
        let n = CURRENT_SIZE.load(Ordering::Relaxed);
        self.out.step = 0;
        self.out.is_bigendian = 0;
        self.out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.out
            .set_encoding("hp")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        self.out.height = height;
        self.out.width = width;
        Ok(())
    }
}

/// Reads the stamp as the first statement of its tick and records the
/// difference.
#[cerulion_node]
struct HopSink {
    #[input(trigger)]
    frame: Image,
}

#[cerulion_node_impl]
impl HopSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = elapsed_ns();
        let stamped = ((self.frame.height as u64) << 32) | (self.frame.width as u64);
        if stamped > 0 {
            if let Some(hop) = now.checked_sub(stamped) {
                if let Ok(mut samples) = HOP_NS.lock() {
                    samples.push(hop);
                }
            }
        }
        Ok(())
    }
}

const HOP_GRAPH: &str = r#"
prefix: test/colocated_hop_probe/hop
nodes:
  - id: source
    type: hop_source
    outputs:
      - name: frame
        schema: sensor_msgs/Image
  - id: sink
    type: hop_sink
    inputs:
      - name: frame
        source: source/frame
"#;

const CHAIN_GRAPH: &str = r#"
prefix: test/colocated_hop_probe/chain
nodes:
  - id: source
    type: hop_source
    outputs:
      - name: frame
        schema: sensor_msgs/Image
  - id: relay_a
    type: hop_relay
    inputs:
      - name: frame
        source: source/frame
    outputs:
      - name: out
        schema: sensor_msgs/Image
  - id: relay_b
    type: hop_relay
    inputs:
      - name: frame
        source: relay_a/out
    outputs:
      - name: out
        schema: sensor_msgs/Image
  - id: sink
    type: hop_sink
    inputs:
      - name: frame
        source: relay_b/out
"#;

fn build(yaml: &str, factories: IndexMap<String, Box<dyn NodeEntry>>) -> GraphRuntime {
    let config = parse_graph(yaml).expect("the fixture graph must parse");
    validate_graph(&config).expect("the fixture graph must validate");
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("the fixture must build")
}

fn hop_graph() -> GraphRuntime {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    f.insert("source".into(), Box::new(HopSourceEntry::new()));
    f.insert("sink".into(), Box::new(HopSinkEntry::new()));
    build(HOP_GRAPH, f)
}

fn chain_graph() -> GraphRuntime {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    f.insert("source".into(), Box::new(HopSourceEntry::new()));
    f.insert("relay_a".into(), Box::new(HopRelayEntry::new()));
    f.insert("relay_b".into(), Box::new(HopRelayEntry::new()));
    f.insert("sink".into(), Box::new(HopSinkEntry::new()));
    build(CHAIN_GRAPH, f)
}

/// Step `runtime` until the sink has recorded the warm-up plus the kept
/// samples, and return the kept ones.
fn collect_graph_row(mut runtime: GraphRuntime, label: &str) -> Vec<u64> {
    HOP_NS.lock().expect("not poisoned").clear();
    let want = WARMUP + ITERATIONS;
    let mut steps = 0;
    while HOP_NS.lock().expect("not poisoned").len() < want {
        assert!(
            steps < STEP_CEILING,
            "{label}: {steps} steps produced {} samples; the sink is not firing once per step",
            HOP_NS.lock().expect("not poisoned").len()
        );
        runtime.step(Duration::from_millis(1));
        steps += 1;
    }
    let mut samples = std::mem::take(&mut *HOP_NS.lock().expect("not poisoned"));
    samples.truncate(want);
    samples.drain(..WARMUP);
    samples
}

// ---------------------------------------------------------------------------
// The transport rows: one publisher on its own topic, its own SHM root.
// ---------------------------------------------------------------------------

/// The slot a publisher needs for one frame of `payload` bytes, with room for
/// the header, the fixed section, the offset table and a margin.
fn slot_len_for(payload: usize) -> u32 {
    (WireHeader::SIZE + Image::WIRE_FIXED_SIZE + 8 * 3 + payload + 64) as u32
}

/// Publish one frame the way the producing nodes do.
fn publish(publisher: &mut CerulionPublisher, payload_len: usize) {
    let mut proxy = publisher.loan_proxy::<Image>().expect("a loan");
    proxy.height = 1;
    proxy.width = payload_len as u32;
    proxy.is_bigendian = 0;
    proxy.step = 1;
    proxy.set_header_bytes(&[]).expect("header bytes");
    proxy.set_encoding("hp").expect("an encoding");
    let dst = proxy.loan_data(payload_len).expect("a payload loan");
    if let Some(first) = dst.first_mut() {
        *first = 0x80;
    }
}

fn isolated_manager() -> Arc<TransportManager> {
    let config = TransportConfig {
        subscriber_buffer_size: BATCH,
        ..TransportConfig::default()
    };
    TransportManager::init_for_test(config, iceoryx_test_config()).expect("a transport manager")
}

// ---------------------------------------------------------------------------
// The harness.
// ---------------------------------------------------------------------------

#[test]
#[ignore = "a measurement, not a gate: run it with --release -- --ignored --nocapture"]
fn the_queued_hop_through_the_level_loop() {
    require_release();
    for size in SIZES {
        CURRENT_SIZE.store(size, Ordering::Relaxed);
        report(
            "queued-hop",
            size,
            1,
            collect_graph_row(hop_graph(), "queued-hop"),
        );
        let chain = collect_graph_row(chain_graph(), "queued-chain");
        let per_hop: Vec<u64> = chain.iter().map(|ns| ns / CHAIN_HOPS).collect();
        report("queued-chain", size, 1, chain);
        report("queued-chain/hop", size, 1, per_hop);
    }
}

#[test]
#[ignore = "a measurement, not a gate: run it with --release -- --ignored --nocapture"]
fn what_a_hop_is_made_of() {
    use iceoryx2::prelude::*;

    require_release();
    for size in SIZES {
        // --- consumer-receive: one pending frame, then the typed read. ---
        {
            let mgr = isolated_manager();
            let topic = "probe/colocated/receive";
            let mut publisher = mgr
                .create_publisher_simple(topic, MaxSliceLen::const_new(slot_len_for(size)))
                .expect("a publisher");
            let mut sub = mgr.create_subscriber(topic).expect("a subscriber");
            let mut kept = Vec::with_capacity(ITERATIONS);
            for i in 0..(WARMUP + ITERATIONS) {
                publish(&mut publisher, size);
                let start = Instant::now();
                let seen = sub
                    .try_view::<Image, _>(|view| (view.height, view.width, view.data().len()))
                    .expect("a read");
                let elapsed = start.elapsed().as_nanos() as u64;
                assert!(seen.is_some(), "the frame just published must be read");
                std::hint::black_box(seen);
                if i >= WARMUP {
                    kept.push(elapsed);
                }
            }
            report("consumer-receive", size, 1, kept);
        }

        // --- library-receive: the library's own receive, nothing of ours. ---
        {
            let node = NodeBuilder::new()
                .config(&iceoryx_test_config())
                .create::<ipc_threadsafe::Service>()
                .expect("an iceoryx2 node");
            let name: ServiceName = "probe/colocated/library"
                .try_into()
                .expect("a service name");
            let service = node
                .service_builder(&name)
                .publish_subscribe::<[u8]>()
                .subscriber_max_buffer_size(BATCH)
                .open_or_create()
                .expect("a service");
            let publisher = service
                .publisher_builder()
                .initial_max_slice_len(size)
                .create()
                .expect("a publisher");
            let subscriber = service
                .subscriber_builder()
                .buffer_size(BATCH)
                .create()
                .expect("a subscriber");
            let mut kept = Vec::with_capacity(ITERATIONS / BATCH);
            for b in 0..((WARMUP + ITERATIONS) / BATCH) {
                for _ in 0..BATCH {
                    let mut sample = publisher.loan_slice_uninit(size).expect("a loan");
                    sample.payload_mut()[0].write(0x80);
                    // SAFETY: only byte zero is written, and nothing reads a
                    // payload byte: the receive below reads the slice length.
                    let sample = unsafe { sample.assume_init() };
                    sample.send().expect("a send");
                }
                let start = Instant::now();
                for _ in 0..BATCH {
                    let received = subscriber.receive().expect("a receive");
                    assert!(received.is_some(), "a published frame must be received");
                    std::hint::black_box(received.map(|s| s.payload().len()));
                }
                let per_op = start.elapsed().as_nanos() as u64 / BATCH as u64;
                if b >= WARMUP / BATCH {
                    kept.push(per_op);
                }
            }
            report("library-receive", size, BATCH, kept);
        }

        // --- publish: loan, touch, commit, to one subscriber. ---
        {
            let mgr = isolated_manager();
            let topic = "probe/colocated/publish";
            let mut publisher = mgr
                .create_publisher_simple(topic, MaxSliceLen::const_new(slot_len_for(size)))
                .expect("a publisher");
            let sub = mgr.create_subscriber(topic).expect("a subscriber");
            let mut kept = Vec::with_capacity(ITERATIONS / BATCH);
            for b in 0..((WARMUP + ITERATIONS) / BATCH) {
                let start = Instant::now();
                for _ in 0..BATCH {
                    publish(&mut publisher, size);
                }
                let per_op = start.elapsed().as_nanos() as u64 / BATCH as u64;
                // Empty the queue outside the timed region, so every batch
                // publishes into the same occupancy.
                let mut drained = 0;
                while sub
                    .try_receive_one(|s| {
                        std::hint::black_box(s.payload().len());
                    })
                    .expect("a receive")
                {
                    drained += 1;
                }
                assert_eq!(drained, BATCH, "every frame of the batch must arrive");
                if b >= WARMUP / BATCH {
                    kept.push(per_op);
                }
            }
            report("publish", size, BATCH, kept);
        }
    }
}
