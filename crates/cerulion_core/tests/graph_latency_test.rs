// SPDX-License-Identifier: AGPL-3.0-only
//! Release-mode USER-POV graph latency gate.
//!
//! Unlike `flat_latency_test.rs` / `cross_thread_rtt_test.rs` — which call
//! the TRANSPORT layer directly (`CerulionPublisher` / `CerulionSubscriber`)
//! — this test measures latency the way a real user experiences it: through
//! a `#[cerulion_node]` graph wired via YAML and driven by
//! `GraphRuntime::step()`. That exercises the *full* user-POV path:
//!
//!   macro `tick()` dispatch
//!     → IndexMap port lookup
//!       → `AnyPublisher` / `AnySubscriber`
//!         → scheduler step (trigger evaluation + fire ordering)
//!           → iceoryx2 transport
//!
//! This test gates that whole chain end-to-end.
//!
//! ## Topology
//!
//! ```text
//!  PingNode  ── ping_out ──▶  PongNode  ── echo_out ──▶  LatencyNode
//!  (period_ms=1)              (data)                     (data)
//! ```
//!
//! - `PingNode` (period-triggered): per tick, stamps the current wall-clock
//!   `elapsed_ns()` into the outbound message (split across two `u32`
//!   `Image` fixed fields for full `u64` precision) and loans
//!   `payload_size` bytes for the variable `data` field (touching only the
//!   first byte — fill is EXCLUDED, matching the zero-copy convention of the
//!   transport-level latency tests).
//! - `PongNode` (data-triggered on `ping/ping_out`): forwards the timestamp
//!   to `echo_out` — the "pure echo" leg.
//! - `LatencyNode` (data-triggered on `pong/echo_out`): reads the timestamp
//!   back, computes `now - stamp`, and pushes the wall-clock RTT to a shared
//!   `Mutex<Vec<u64>>` collector.
//!
//! Each completed ping→pong→latency round contributes one wall-clock
//! latency sample. The simulated `VirtualClock` only drives scheduling;
//! `elapsed_ns()` reads the same real monotonic `Instant` in both `PingNode`
//! (write) and `LatencyNode` (read), so the measured RTT is wall-clock.
//!
//! ## Why it sweeps payload sizes
//!
//! Because the message is a variable `Image` published zero-copy
//! (`loan_data(N)` writes straight into the loaned SHM slot, no memcpy of
//! the payload), the per-round latency should stay ~flat as the payload
//! grows. A flatness assertion gates zero-copy AT THE GRAPH LEVEL — a
//! memcpy regression in the macro/transport stack would blow it up to 100x+.
//!
//! ## Global untimed warm-up
//!
//! The sweep is preceded by a ~2s wall-clock warm-up driving the same graph
//! machinery with every sample discarded. Measured (Jetson aarch64,
//! release): without it the FIRST-swept size runs on a DVFS-cold core — 64B p50s 34-46µs
//! vs the later warmed sizes' 21-25µs — tripping the <1.5× flatness gate on
//! 4/6 runs (a flake that x86 CI runners, which show no such
//! cold-start gap, never surface). The
//! per-size 100-sample warmup is only ~ms of wall work — far too short to
//! ramp a DVFS governor. Warming does NOT weaken copy detection: a real
//! O(n) copy shifts the steady-state LARGE sizes, not the sweep head. (Same
//! failure class that `flat_latency_test`/`cross_thread_rtt_test` fix
//! via drop-one-outlier floors; here the contamination is strictly the cold
//! head, so warming is the cleaner fix.) The warm-up does not change gate semantics.
//!
//! ## Runner-noise robustness
//!
//! A gate keyed on **p50** with none of the robustness of its two
//! transport-level siblings fires FALSE REDS under runner noise: measured,
//! one commit read `graph-level p50 flatness
//! 1.61x` on one CI run and **1.00x** on the next run of the SAME
//! commit. The tell is the SMALL payload — 6.6µs failing vs
//! 3.3µs passing — i.e. the whole runner is ~2× slow and the 1 MiB row (where
//! absolute cost is highest) diverges most. This is the moat metric, so a gate
//! that cries wolf here is worse than useless: an ignored gate is exactly where
//! a real memcpy regression lands unnoticed.
//!
//! Three defenses. NONE of them touches the `FLATNESS_MAX` ceiling, the
//! absolute p50 backstop, or the global warm-up above:
//!
//! * **Metric: p50 → per-size FLOOR.** Runner noise only ever ADDS latency, so
//!   the floor (min over the measured samples) is jitter-RESISTANT where p50 is
//!   not — the reasoning `flat_latency_test` and `cross_thread_rtt_test`
//!   also use. On that failing run's OWN recorded floors (5751 / 5720
//!   / 8176 ns for the three sizes it swept) the FULL max/min ratio is
//!   `8176/5720 = 1.429x`, which PASSES the 1.5× ceiling. The floor
//!   alone clears that failure; the two below harden other stall shapes.
//! * **INTERLEAVED measurement rounds.** The per-size sample
//!   budget is collected in [`MEASURE_ROUNDS`] rounds
//!   that each visit EVERY size, so one contiguous VM stall cannot own
//!   multiple sizes' floors — each floor is the min across rounds spread over
//!   the whole sweep timeline.
//! * **WINDOW-HEALTH RETRY.** Any size whose floor exceeds
//!   `FLATNESS_MAX × min_floor` is RE-MEASURED in a fresh temporal window
//!   (bounded per-size budget [`MAX_RETRIES`], per-attempt accounting printed).
//!   A transient stall clears in a fresh window; a real copy re-measures just as
//!   slow and never heals. A healthy sweep retries NOTHING and pays nothing.
//!
//! ### Why NOT drop-one-outlier, which the two siblings DO use
//!
//! Drop-one discards the single worst floor. That is correct for the siblings —
//! their sweeps put the 2nd-largest size at ~1 MiB under a ~1µs base, so a real
//! copy still inflates the surviving floor thousands-of-×. It is UNSAFE HERE,
//! for exactly the reason [`cerulion_core::testing::classify_flatness`] documents:
//! this sweep's 2nd-largest size is **16×
//! smaller** than its largest, and 64 KiB is L2-resident, so a memcpy costs
//! almost nothing there while costing ~100µs at 1 MiB. Drop-one discards the
//! 1 MiB floor — the one the copy actually lives in — and rests the whole gate
//! on the barely-moved 64 KiB floor. Computed against this test's measured
//! ~3.55µs base floor, a real O(n) copy at an effective 40 GB/s yields floors
//! `[3.55, 5.19, 29.76]µs`: the FULL max/min ratio is **8.38×** (fails, correctly)
//! while the drop-one ratio is **1.46×** — under the ceiling, i.e. a REAL COPY
//! SHIPS GREEN. At 100 GB/s it is 1.18×. So the gate is the FULL `max/min` floor
//! ratio, and the single-window-stall defense is the RETRY above, not a discarded
//! floor. This is the same architecture, for the same reason, as the sibling
//! `cli_e2e_graph_latency_test` (whose sweep geometry this file matches).
//!
//! ### What still fails NORMALLY
//!
//! A genuine O(n) copy inflates the floors MONOTONICALLY in payload size, so
//! across this sweep's `>= 16×` adjacent size steps its elevated floors span far
//! more than [`cerulion_core::testing::UNIFORM_STALL_BAND`] (1.3×). It therefore
//! can never satisfy the uniform-stall signature, and lands in
//! [`cerulion_core::testing::FlatnessVerdict::RealCopy`] — a loud, normal gate
//! failure. Only a SIZE-INDEPENDENT elevation (`>= 3` floors inside that tight
//! band, which no copy can produce) is routed to the ATTRIBUTABLE non-probative
//! panic. Pinned over THIS FILE'S OWN sweep constants by the pure oracle arms in
//! [`gate_decision_oracles`], which need no transport and no timing.
//!
//! ## Maintenance note
//!
//! This test couples to the `#[cerulion_node]` macro surface and the
//! `Image` message API (`set_encoding`, `set_header_bytes`, `loan_data`).
//! That coupling is intentional and accepted: it is the price of gating the
//! *user-POV* path. It WILL need updating on macro/message API bumps — when
//! those land, update the node defs here to match (the sibling that tracks
//! the same surface is `benches/latency/workspace/nodes/*` — the
//! public latency suite's workspace nodes).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test graph_latency_test --release \
//!   -- --nocapture --test-threads=1
//! ```
//!
//! Must run with `--release` (debug builds are 10-50x slower; absolute-µs
//! thresholds only make sense in release) and `--test-threads=1` (iceoryx2
//! singleton + shared memory requires serial access). The test is gated
//! behind `cfg(not(debug_assertions))` so a plain `cargo test` (no
//! `--release`) compiles it to an empty no-op rather than failing on
//! debug-build latencies.

#![cfg(not(debug_assertions))]

use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant, SystemTime};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime};
use cerulion_core::prelude::*;
use cerulion_core::testing::FlatnessVerdict;
use cerulion_core::transport::TransportManager;
use indexmap::IndexMap;
use native_ros2_messages::sensor_msgs::Image;

// ---------------------------------------------------------------------------
// Sweep / sampling parameters
// ---------------------------------------------------------------------------

/// Payload sizes swept for the graph-level flatness check.
///
/// The 4 KiB point makes this sweep byte-identical to the
/// sibling `cli_e2e_graph_latency_test`'s. It is NOT cosmetic: the
/// uniform-stall discriminator needs `>= UNIFORM_STALL_MIN_SIZES` (3) floors
/// ELEVATED above `FLATNESS_MAX × min_floor`, and the size holding `min_floor`
/// can never itself be elevated (for any ceiling `>= 1`), so a THREE-size sweep
/// can present at most TWO elevated floors and
/// [`cerulion_core::testing::is_uniform_stall_signature`] returns `false`
/// UNCONDITIONALLY. On a three-size sweep the attributable-stall arm is therefore
/// dead code that looks like a defense. Four sizes make it genuinely
/// reachable (pinned by [`gate_decision_oracles`]), and the extra point also
/// strictly tightens copy detection — one more chance to exceed max/min, never
/// fewer. The added measurement cost is ~1 ms of wall work.
const PAYLOAD_SIZES: &[usize] = &[64, 4 * 1024, 64 * 1024, 1024 * 1024];

/// Samples discarded at the start of a size's FIRST round (let the
/// data-trigger chain reach steady state and warm the SHM pool / caches).
const WARMUP_SAMPLES: usize = 100;

/// Latency samples KEPT per size, summed across all rounds (≥200 per the spec).
const MEASURE_SAMPLES: usize = 250;

/// Interleaving: the per-size sample budget is [`MEASURE_SAMPLES`]; interleaving just
/// splits it into `MEASURE_ROUNDS` rounds of [`ITERS_PER_ROUND`] kept samples,
/// and every round visits EVERY size. A contiguous VM stall inside one round
/// therefore hits at most the slice it overlaps, and each size's FLOOR is the
/// min across rounds spread over the whole sweep timeline — so a stall must
/// span nearly the entire sweep to inflate even one size's floor.
const MEASURE_ROUNDS: usize = 5;

/// Interleaving: kept samples per size PER round.
/// `MEASURE_ROUNDS * ITERS_PER_ROUND == MEASURE_SAMPLES` (const-asserted).
const ITERS_PER_ROUND: usize = MEASURE_SAMPLES / MEASURE_ROUNDS;

const _: () = assert!(
    MEASURE_ROUNDS * ITERS_PER_ROUND == MEASURE_SAMPLES,
    "interleaving must not change the per-size sample budget"
);

/// Samples discarded when RESUMING a size in rounds 1.. .
///
/// Interleaving parks a graph mid-chain, so up to one in-flight message can sit
/// in each of the two hops (ping→pong, pong→echo) while the other sizes take
/// their turn. Those frames carry an OLD stamp, so on resume they read as
/// wall-milliseconds. They cannot move the gated FLOOR (a min), but they would
/// pollute the printed p99/max and the p50 backstop, so each resumed round
/// drops this many samples first. Measured in-flight depth is <= 2; 8 is slack.
const RESUME_DISCARD: usize = 8;

/// Window-health retry: per-size budget of re-measurements. Bounds the
/// worst-case extra work at `MAX_RETRIES × PAYLOAD_SIZES.len()` fresh windows;
/// a healthy sweep re-measures NOTHING.
const MAX_RETRIES: usize = 2;

/// The TIGHT zero-copy flatness ceiling — the floor metric does not loosen it.
///
/// MEASURED 1.00–1.03× on a healthy machine (M3 Max, release, 64B→1MiB), so there is
/// a full 50% of headroom. A false red is never a threshold problem:
/// it is a p50 metric on a ~2×-slow runner. Do NOT widen this — it is the moat
/// gate. If it flakes, the metric or the window is wrong, not the ceiling.
const FLATNESS_MAX: f64 = 1.5;

/// Monotonic counter guaranteeing unique topic prefixes even within the
/// same nanosecond (iceoryx2 services are host-wide).
static TOPIC_COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_prefix(base: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let id = TOPIC_COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("glat/{}/{}/{}", base, nanos, id)
}

// ---------------------------------------------------------------------------
// Shared timing helpers (written by PingNode, read by LatencyNode)
// ---------------------------------------------------------------------------

/// Reference `Instant`, lazily initialised on first `elapsed_ns()`. Both
/// `PingNode` (which stamps the timestamp) and `LatencyNode` (which reads
/// it back) measure against this same monotonic reference.
static REFERENCE_TIME: OnceLock<Instant> = OnceLock::new();

/// All RTT samples (ns) recorded by `LatencyNode`. The harness drains this
/// between sizes.
static LATENCY_SAMPLES_NS: Mutex<Vec<u64>> = Mutex::new(Vec::new());

/// Byte count loaned for `Image`'s variable `data` field. The harness sets
/// this before each size's run.
static CURRENT_PAYLOAD_SIZE: AtomicUsize = AtomicUsize::new(64);

fn elapsed_ns() -> u64 {
    REFERENCE_TIME
        .get_or_init(Instant::now)
        .elapsed()
        .as_nanos() as u64
}

// ---------------------------------------------------------------------------
// Graph nodes (replicated from the round-trip bench's graph binary)
// ---------------------------------------------------------------------------

/// Source: stamps the current wall-clock elapsed-ns into `ping_out`
/// (split across the two `u32` fixed fields for full `u64` precision) and
/// loans `CURRENT_PAYLOAD_SIZE` bytes for the variable `data` field. Only
/// the first byte is touched — payload fill is EXCLUDED (zero-copy: the
/// loan writes straight into SHM, we don't pay an O(n) memset).
#[cerulion_node(period_ms = 1)]
struct PingNode {
    #[output]
    ping_out: Image,
}

#[cerulion_node_impl]
impl PingNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let n = CURRENT_PAYLOAD_SIZE.load(Ordering::Relaxed);
        let t = elapsed_ns();
        self.ping_out.height = (t >> 32) as u32;
        self.ping_out.width = (t & 0xFFFF_FFFF) as u32;
        self.ping_out.step = 0;
        self.ping_out.is_bigendian = 0;
        self.ping_out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.ping_out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .ping_out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        // Touch only the first byte: prove the slot is real without paying
        // the O(n) fill — flatness is what gates zero-copy.
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

/// Processor: forwards the timestamp from `ping_in` to `echo_out` and
/// re-loans the same payload size. Data-triggered on `ping/ping_out`.
#[cerulion_node]
struct PongNode {
    #[input(trigger)]
    ping_in: Image,
    #[output]
    echo_out: Image,
}

#[cerulion_node_impl]
impl PongNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let h = self.ping_in.height;
        let w = self.ping_in.width;
        let n = CURRENT_PAYLOAD_SIZE.load(Ordering::Relaxed);
        self.echo_out.height = h;
        self.echo_out.width = w;
        self.echo_out.step = 0;
        self.echo_out.is_bigendian = 0;
        self.echo_out
            .set_header_bytes(&[])
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        self.echo_out
            .set_encoding("rt")
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        let dst = self
            .echo_out
            .loan_data(n)
            .map_err(|e| NodeError::Logic(e.to_string()))?;
        if let Some(first) = dst.first_mut() {
            *first = 0x80;
        }
        Ok(())
    }
}

/// Sink: reads the embedded send-timestamp, computes wall-clock RTT, pushes
/// to the shared samples collector. Data-triggered on `pong/echo_out`.
#[cerulion_node]
struct LatencyNode {
    #[input(trigger)]
    echo_in: Image,
}

#[cerulion_node_impl]
impl LatencyNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let h = self.echo_in.height as u64;
        let w = self.echo_in.width as u64;
        let send_ns = (h << 32) | w;
        let now_ns = elapsed_ns();
        let rtt = now_ns.saturating_sub(send_ns);
        if send_ns > 0 && rtt > 0 {
            if let Ok(mut samples) = LATENCY_SAMPLES_NS.lock() {
                samples.push(rtt);
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Stats helpers
// ---------------------------------------------------------------------------

/// Median of a sorted slice (ns).
fn median(sorted: &[u64]) -> u64 {
    let n = sorted.len();
    if n.is_multiple_of(2) {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2
    } else {
        sorted[n / 2]
    }
}

/// p99 of a sorted slice (ns).
fn p99(sorted: &[u64]) -> u64 {
    let idx = ((sorted.len() as f64) * 0.99) as usize;
    sorted[idx.min(sorted.len() - 1)]
}

/// `(floor, p50, p99, max)` in ns from an UNSORTED sample vector.
///
/// The FLOOR is the min: runner noise only ever ADDS latency, so the min is the
/// jitter-RESISTANT read of the uncontended path and is what the flatness gate
/// keys on. The other three are printed for human insight, and p50
/// additionally carries the absolute catastrophe backstop.
///
/// `f64` because that is what the shared `cerulion_core::testing` floor helpers
/// consume.
fn stats_from_samples(samples: &[u64]) -> (f64, f64, f64, f64) {
    assert!(!samples.is_empty(), "stats over an empty sample vector");
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    (
        sorted[0] as f64,
        median(&sorted) as f64,
        p99(&sorted) as f64,
        sorted[sorted.len() - 1] as f64,
    )
}

// ---------------------------------------------------------------------------
// Graph construction shared by the measured per-size runs AND the global
// DVFS warm-up (same machinery, so the warm-up exercises the exact code path
// the measured windows do).
// ---------------------------------------------------------------------------

/// Build the ping→pong→latency graph over the global `TransportManager`
/// singleton for `payload_size` bytes of variable data. Sets
/// `CURRENT_PAYLOAD_SIZE` and clears the shared samples collector, so the
/// caller starts from a clean slate.
fn build_latency_graph(payload_size: usize) -> GraphRuntime {
    CURRENT_PAYLOAD_SIZE.store(payload_size, Ordering::Relaxed);

    // Force the reference time to initialise before the first tick so the
    // very first stamp sees a sensible elapsed_ns.
    let _ = elapsed_ns();

    // Drain any samples from a previous size (or the warm-up).
    LATENCY_SAMPLES_NS.lock().unwrap().clear();

    let prefix = unique_prefix(&format!("sz{}", payload_size));

    // Sized to fit the payload + WireHeader + Image fixed section + offset
    // table for the variable fields, with slop. Mirrors the bench's sizing.
    let max_slice_len = 32 + 64 + 8 * 3 + payload_size + 256;
    let yaml = format!(
        r#"
name: graph_latency_gate
prefix: {prefix}
nodes:
  - id: ping
    type: ping_node
    outputs:
      - name: ping_out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: pong
    type: pong_node
    inputs:
      - name: ping_in
        source: ping/ping_out
    outputs:
      - name: echo_out
        schema: sensor_msgs/Image
        max_slice_len: {max_slice_len}
  - id: latency
    type: latency_node
    inputs:
      - name: echo_in
        source: pong/echo_out
"#
    );

    let config = parse_graph(&yaml).expect("parse graph");
    validate_graph(&config).expect("validate graph");

    // `get_or_init` returns the process-global singleton `Arc`; dropping the
    // local handle at return does not tear down the transport.
    let mgr = TransportManager::get_or_init().expect("init TransportManager");
    let clock = Arc::new(VirtualClock::new());

    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("ping".to_string(), Box::new(PingNodeEntry::new()));
    nodes.insert("pong".to_string(), Box::new(PongNodeEntry::new()));
    nodes.insert("latency".to_string(), Box::new(LatencyNodeEntry::new()));

    GraphRuntime::build(config, nodes, &mgr, clock).expect("build graph")
}

// ---------------------------------------------------------------------------
// Global untimed warm-up: bring the core to steady frequency BEFORE the first
// measured size.
// ---------------------------------------------------------------------------

/// Wall-clock budget for [`global_dvfs_warmup`]. DVFS governors ramp on
/// SUSTAINED utilization over tens-to-hundreds of ms; the per-size 100-sample
/// warmup is only ~ms of wall work, far too short to ramp a cold core, so the
/// budget is wall-time-based (a fixed RTT count would be over in ~ms on a
/// fast machine and still-cold on a slow one).
const GLOBAL_WARMUP_WALL: Duration = Duration::from_secs(2);

/// Measured (Jetson aarch64): without this warm-up the first-SWEPT size
/// (64B) runs on a DVFS-cold core — floors 23-32µs / p50s 34-46µs vs the later
/// warmed sizes' floors 18-19µs / p50s 21-25µs — tripping the <1.5× flatness
/// gate (4/6 release runs). Same
/// failure class that `flat_latency_test` / `cross_thread_rtt_test` handle;
/// here the contamination is strictly the sweep's COLD HEAD (not a random
/// mid-sweep stall), so the cleaner fix is warming, not drop-one-outlier.
///
/// Drives the SAME graph machinery (build + step round-trips) untimed for
/// [`GLOBAL_WARMUP_WALL`], discarding every sample, so size[0]'s measured
/// window starts at steady frequency. This does NOT weaken copy detection: a
/// real O(n) memcpy regression shifts the steady-state LARGE sizes (the
/// flatness numerator), not the sweep head.
fn global_dvfs_warmup() {
    let mut runtime = build_latency_graph(PAYLOAD_SIZES[0]);
    let start = Instant::now();
    while start.elapsed() < GLOBAL_WARMUP_WALL {
        runtime.step(Duration::from_millis(1));
    }
    runtime.shutdown();
    // Discard everything the warm-up recorded — it never reaches the stats.
    LATENCY_SAMPLES_NS.lock().unwrap().clear();
}

// ---------------------------------------------------------------------------
// Per-size run: build the graph, drive it, return
// (floor_ns, p50_ns, p99_ns, max_ns) from the sorted steady-state samples.
// ---------------------------------------------------------------------------

/// Drive ONE already-built graph until it has recorded `discard + keep`
/// samples, then return the last `keep` of them.
///
/// The shared collector is cleared on entry, so the caller owns a clean slate;
/// `CURRENT_PAYLOAD_SIZE` is re-published because the interleaved sweep hops
/// between graphs and the nodes read it at tick time.
fn collect_segment(
    runtime: &mut GraphRuntime,
    payload_size: usize,
    discard: usize,
    keep: usize,
) -> Vec<u64> {
    CURRENT_PAYLOAD_SIZE.store(payload_size, Ordering::Relaxed);
    LATENCY_SAMPLES_NS.lock().unwrap().clear();

    // Each step(1ms) advances the simulated clock by 1ms (firing one period
    // ping) and processes pending data triggers. Empirically ~3 steps per
    // recorded round-trip (ping → pong → latency).
    let target = discard + keep;
    let max_steps = target * 50 + 1000; // safety cap against a stalled chain
    let mut steps = 0usize;
    loop {
        runtime.step(Duration::from_millis(1));
        steps += 1;
        let recorded = LATENCY_SAMPLES_NS.lock().unwrap().len();
        if recorded >= target {
            break;
        }
        assert!(
            steps < max_steps,
            "graph latency (payload={}B): only {} / {} samples after {} steps — \
             the data-trigger chain likely stalled",
            payload_size,
            recorded,
            target,
            steps
        );
    }

    let mut samples = LATENCY_SAMPLES_NS.lock().unwrap().clone();
    if samples.len() > discard {
        samples.drain(..discard);
    }
    samples.truncate(keep);
    samples
}

/// The window-health retry primitive: re-measure ONE size in a FRESH temporal window
/// (fresh graph, fresh warm-up, the full [`MEASURE_SAMPLES`] budget in one
/// pass), returning `(floor, p50, p99, max)` ns.
///
/// Used only for a size whose initial-sweep floor was elevated. A transient VM
/// stall almost always clears in a fresh window; a real O(n) copy re-measures
/// just as slow, so it never heals and still fails the gate.
fn measure_one_size(payload_size: usize) -> (f64, f64, f64, f64) {
    let mut runtime = build_latency_graph(payload_size);
    let samples = collect_segment(&mut runtime, payload_size, WARMUP_SAMPLES, MEASURE_SAMPLES);
    runtime.shutdown();
    stats_from_samples(&samples)
}

/// Interleaving: measure ALL `sizes` in [`MEASURE_ROUNDS`] INTERLEAVED rounds,
/// returning `(size, floor, p50, p99, max)` ns per size in sweep order.
///
/// Every round runs [`ITERS_PER_ROUND`] kept samples for EVERY size, so a single
/// contiguous VM stall cannot own multiple sizes' floors — each size's floor is
/// the min over `MEASURE_ROUNDS` windows spread across the whole sweep timeline.
/// Total kept samples per size is [`MEASURE_SAMPLES`], the same as a
/// single-pass sweep.
///
/// Every size owns its own graph with its own unique topic prefix, so there is
/// no cross-size traffic; parking a graph between rounds costs only the
/// [`RESUME_DISCARD`] stale in-flight frames dropped on resume.
fn measure_all_sizes_interleaved(sizes: &[usize]) -> Vec<(usize, f64, f64, f64, f64)> {
    // Build every size's graph up front and hold them ALL alive, so each round
    // can visit each size without paying a rebuild. iceoryx2 `Static` pools are
    // lazy/demand-paged and we touch only the first payload byte, so resident
    // memory stays at the working set (see `shm_footprint_probe`).
    let mut runtimes: Vec<(usize, GraphRuntime)> = sizes
        .iter()
        .map(|&size| (size, build_latency_graph(size)))
        .collect();
    let mut samples: Vec<Vec<u64>> = vec![Vec::with_capacity(MEASURE_SAMPLES); sizes.len()];

    for round in 0..MEASURE_ROUNDS {
        // Round 0 pays the full chain/SHM warm-up; later rounds only need to
        // flush the frames parked in flight while the other sizes ran.
        let discard = if round == 0 {
            WARMUP_SAMPLES
        } else {
            RESUME_DISCARD
        };
        for (idx, (size, runtime)) in runtimes.iter_mut().enumerate() {
            let kept = collect_segment(runtime, *size, discard, ITERS_PER_ROUND);
            samples[idx].extend_from_slice(&kept);
        }
    }

    // `shutdown` consumes the runtime, so drain the vector rather than
    // borrowing it. Every size's graph is torn down before the stats are
    // reported, exactly as a single-pass sweep would.
    for (_, runtime) in runtimes {
        runtime.shutdown();
    }

    sizes
        .iter()
        .enumerate()
        .map(|(idx, &size)| {
            let (floor, p50, p99_ns, max) = stats_from_samples(&samples[idx]);
            (size, floor, p50, p99_ns, max)
        })
        .collect()
}

/// Print the per-size stats table. The FLOOR gates flatness; p50/p99/max are
/// human insight (p50 additionally carries the absolute catastrophe backstop).
/// Shared by the initial-sweep and post-retry prints.
fn print_stats_table(label: &str, stats: &[(usize, f64, f64, f64, f64)]) {
    println!(
        "USER-POV graph latency (ping → pong → latency, wall-clock RTT) [{label}] \
         (FLOOR gates flatness):"
    );
    for &(size, floor, p50, p99_ns, max) in stats {
        println!(
            "  payload={:>9} B  floor={:>9.1} ns ({:>7.2} µs)  p50={:>9.1} ns ({:>7.2} µs)  \
             p99={:>9.1} ns ({:>7.2} µs)  max={:>9.1} ns ({:>7.2} µs)",
            size,
            floor,
            floor / 1000.0,
            p50,
            p50 / 1000.0,
            p99_ns,
            p99_ns / 1000.0,
            max,
            max / 1000.0,
        );
    }
}

// ============================================================
// USER-POV graph latency gate
// ============================================================

#[test]
fn test_graph_user_pov_latency() {
    // Global untimed warm-up BEFORE the first measured size —
    // the Jetson DVFS cold-start defense (see `global_dvfs_warmup`'s doc for the
    // measurements). It does not change the gate semantics below.
    global_dvfs_warmup();

    // --- 1. INTERLEAVED sweep --------------------------------------------
    // (size, floor, p50, p99, max) in ns. The FLOOR is the gated metric;
    // the rest are printed for human insight.
    let mut stats = measure_all_sizes_interleaved(PAYLOAD_SIZES);
    print_stats_table("initial interleaved sweep", &stats);

    // --- 2. Window-health retries ----------------------------------------
    // The pure orchestration lives in `cerulion_core::testing` (oracle-tested
    // there); only the re-measurement is injected here. Since this gate does
    // NOT drop an outlier (see the module header), the retry is the SOLE
    // single-window-stall defense: any size whose floor exceeds
    // `FLATNESS_MAX × min_floor` is re-measured in a fresh window, keeping the
    // better floor. A healthy sweep re-measures NOTHING and pays nothing.
    let retry_log = cerulion_core::testing::run_window_health_retries(
        &mut stats,
        FLATNESS_MAX,
        MAX_RETRIES,
        measure_one_size,
    );
    if retry_log.is_empty() {
        println!(
            "  window-health: every per-size floor within {FLATNESS_MAX:.1}x of the min floor \
             — no retries"
        );
    } else {
        println!(
            "  window-health: {} re-measurement attempt(s) on elevated size(s):",
            retry_log.len()
        );
        for r in &retry_log {
            println!(
                "    retry {} bytes round {} attempt {}/{}: old floor={:.2}µs -> \
                 measured={:.2}µs, kept={:.2}µs",
                r.size,
                r.round,
                r.attempt,
                MAX_RETRIES,
                r.old_floor / 1000.0,
                r.measured_floor / 1000.0,
                r.kept_floor / 1000.0,
            );
        }
        print_stats_table("after window-health retries", &stats);
    }

    // --- Assertion (a): absolute p50 sanity at the smallest size ---------
    // The smallest payload is the cleanest read of the pure graph-path
    // overhead (macro dispatch + port lookup + scheduler step + transport).
    let (_, _, smallest_p50_ns, _, _) = stats[0];
    let smallest_p50_us = smallest_p50_ns / 1000.0;
    // MEASURED ~7.7µs (M3 Max, release, 250-iter median) — the user-POV graph
    // RTT (ping→pong→latency through macro dispatch + scheduler + transport).
    // This is TRANSPORT-bound: each data-trigger hop pays a dual-subscriber
    // double-read (the runtime's trigger-drain `try_receive` + the node's body
    // `try_view`, two iceoryx2 receives of the same publish, ~1.3µs each) + a
    // publish, so the 2-hop chain ≈ 2×(2 recv + pub) ≈ 7µs; the executor
    // scaffolding is ~1µs (profiled separately). The RTT
    // captures both hops serialized within ONE step, so it is not
    // comparable with a ~5.2µs figure taken over a multi-step-lagged
    // window: the difference is what is measured, not a
    // regression. The latency LEVER is unifying the
    // dual subscriber; the transport-only moat floors (the
    // `flat_latency` one-way + `cross_thread_rtt` RTT tests) do not move with it. The 50µs ceiling =
    // a GENEROUS catastrophe backstop (CI latency runners are noisy —
    // latency_threshold uses ~200x margins). The flatness ratio below is the
    // TIGHT, VM-noise-invariant zero-copy gate; this absolute only catches a
    // gross user-path regression (accidental serialization / scheduler blowup).
    // If the runner flakes, loosen — don't tighten below its noise floor.
    assert!(
        smallest_p50_us < 50.0,
        "USER-POV graph p50 latency {:.1}µs (payload={}B) exceeds 50µs \
         placeholder — graph-path overhead regression (macro dispatch / \
         port lookup / scheduler / transport)",
        smallest_p50_us,
        stats[0].0,
    );

    // KNOWN, DELIBERATE GAP: the flatness gate below is a
    // RATIO, so it is structurally BLIND to a CONSTANT-overhead regression —
    // adding a fixed cost to every size makes the ratio SMALLER (greener). This
    // absolute backstop is the only assertion watching that axis, and at ~14×
    // the healthy value it will only catch a gross one. It is deliberately NOT
    // tightened here, and an absolute FLOOR band is deliberately not added: the
    // slow-runner measurement reads floors of 5751/5720/8176 ns against a
    // healthy 3571/3552/3535 — the runner moves the FLOOR itself by 1.6–2.3× —
    // so any threshold tight enough to catch a +2µs constant regression is also
    // tight enough for a slow runner to trip, which is the exact load-sensitivity
    // class the floor metric exists to remove. Closing that axis
    // needs a load-independent instrument (e.g. an instruction/alloc counter),
    // not a wall-clock number.

    // --- Assertion (b): graph-level zero-copy flatness across sizes ------
    // Because the payload is published zero-copy (loan_data writes straight into
    // SHM, no memcpy), the per-size FLOOR should stay ~flat as the payload grows.
    // A memcpy regression in the macro/transport stack would blow this up.
    //
    // The metric is the FULL max/min ratio over FLOORS, with NO outlier dropped
    // — see the module header for the arithmetic showing that drop-one would let
    // a real 40 GB/s copy through as green at this sweep geometry. Single-window
    // stalls were already absorbed in step 2 by the retry; a stall that survived
    // every retry fails here as `RealCopy` (the conservative residual — a human
    // re-runs on a quiescent host to disambiguate). The composed decision is the
    // pure, oracle-tested `classify_flatness`, so the ratio-first-then-
    // discriminator ordering cannot be mis-wired at this call site.
    let floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
    match cerulion_core::testing::classify_flatness(&floors, FLATNESS_MAX) {
        FlatnessVerdict::Pass(ff) => {
            println!(
                "  FLATNESS: PASS — FULL max/min FLOOR ratio={:.3}x (min={:.2}µs @ {}B, \
                 max={:.2}µs @ {}B, ceiling {:.1}x). A single VM-stalled window is absorbed \
                 by the window-health retry, not by dropping a floor.",
                ff.ratio,
                ff.min.1 / 1000.0,
                ff.min.0,
                ff.max.1 / 1000.0,
                ff.max.0,
                FLATNESS_MAX,
            );
        }
        FlatnessVerdict::UniformStall(ff) => {
            let floors_str = stats
                .iter()
                .map(|&(s, f, ..)| format!("{s}B={:.2}µs", f / 1000.0))
                .collect::<Vec<_>>()
                .join(", ");
            let retry_str = if retry_log.is_empty() {
                "none".to_string()
            } else {
                retry_log
                    .iter()
                    .map(|r| {
                        format!(
                            "{}B a{}: {:.2}->{:.2} kept {:.2}µs",
                            r.size,
                            r.attempt,
                            r.old_floor / 1000.0,
                            r.measured_floor / 1000.0,
                            r.kept_floor / 1000.0,
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("; ")
            };
            panic!(
                "NON-PROBATIVE: the USER-POV graph flatness gate failed with \
                 a SIZE-INDEPENDENT uniform-stall signature, NOT a payload-size-dependent copy. \
                 >= {} per-size floors are elevated (> {:.1}x the {:.2}µs min floor @ {}B) yet \
                 fall within a tight {:.2}x band of EACH OTHER — the shape of a shared-CI-VM \
                 stall spanning multiple measurement windows that survived every retry, which a \
                 real O(n) copy (adjacent floors differ by the >= 16x payload-size step) can \
                 never produce. This is an ATTRIBUTABLE infrastructure red, not a zero-copy \
                 regression — re-run on a quiescent host. Final per-size floors: [{}]. \
                 Window-health retry history ({} attempt(s)): [{}]. max/min ratio = {:.3}x, \
                 ceiling = {:.1}x.",
                cerulion_core::testing::UNIFORM_STALL_MIN_SIZES,
                FLATNESS_MAX,
                ff.min.1 / 1000.0,
                ff.min.0,
                cerulion_core::testing::UNIFORM_STALL_BAND,
                floors_str,
                retry_log.len(),
                retry_str,
                ff.ratio,
                FLATNESS_MAX,
            );
        }
        FlatnessVerdict::RealCopy(ff) => {
            panic!(
                "graph-level FLOOR flatness {:.3}x (FULL max/min, NO outlier dropped) exceeds \
                 the {:.1}x threshold — variable-payload publish is no longer zero-copy at the \
                 graph level (memcpy regression). A zero-copy path is O(1) in payload, so the \
                 floor is flat; a memcpy shows tens-of-x. The shape is NOT the uniform-stall \
                 signature and the window-health retry did NOT heal it, so this is a REAL \
                 regression. (Residual: a RARE persistent single/two-window VM stall that \
                 survives every retry is indistinguishable from a real single-size regression \
                 and lands here too — if a re-run on a quiescent host clears it, it was a \
                 stall, not a copy.) max floor = {:.2}µs @ {}B, min floor = {:.2}µs @ {}B.",
                ff.ratio,
                FLATNESS_MAX,
                ff.max.1 / 1000.0,
                ff.max.0,
                ff.min.1 / 1000.0,
                ff.min.0,
            );
        }
    }
}

// ============================================================
// The gate's WIRING (what the oracle arms cannot see)
// ============================================================
//
// The oracle arms below call `cerulion_core::testing` DIRECTLY, so they are
// structurally blind to this file's own call site: deleting the retry from the
// gate, or gating on p50 again, leaves every one of them green while the retry
// ships inert. The gate itself cannot cover that either — on a healthy machine it
// passes with or without the wiring, and reproducing a stall on demand is the
// very thing the retry exists to stop depending on.
//
// So the wiring is pinned STRUCTURALLY, over the gate function's own source.
// Same approach, for the same reason, as `cdylib_iox2_log_level_test`'s
// hand-written-init walk and the convergence adoption walks.
mod gate_wiring {
    const SRC: &str = include_str!("graph_latency_test.rs");

    /// The gate function's source, from its signature to the next top-level
    /// item. Brace matching is deliberately avoided: the body's `format!`
    /// strings carry `{}` placeholders, so a naive matcher would be reading
    /// braces out of string literals.
    fn raw_gate_body() -> &'static str {
        let start = SRC
            .find("fn test_graph_user_pov_latency")
            .expect("the gate function must exist");
        let rest = &SRC[start..];
        let end = rest
            .find("\n// ====")
            .expect("the gate must be followed by the next section banner");
        &rest[..end]
    }

    /// `raw_gate_body` with `//` comment lines removed.
    ///
    /// The strip is DEFENSIVE, not load-bearing today — MEASURED, not assumed.
    /// Neutering it (returning `raw_gate_body()` verbatim) fails ONLY the
    /// anti-tautology arm below; the forbidden-token assertion stays green,
    /// because the gate's comments name the rejected metric as PROSE
    /// ("drop-one") and never as the identifier `drop_one_outlier_robust_ratio`.
    /// What the strip buys is the future: the natural way to explain why this
    /// gate does not use drop-one is a comment naming the function, and that
    /// one comment would silently make the forbidden-token assertion vacuous.
    /// The arm below is what keeps the strip itself correct.
    fn code_only_gate_body() -> String {
        raw_gate_body()
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// `s` with ALL whitespace removed.
    ///
    /// Needle matching below runs BOTH sides through this, so a needle spanning
    /// several tokens survives rustfmt wrapping the statement it pins across
    /// lines. Collapsing runs of whitespace to a single space is NOT enough: a
    /// wrap also inserts whitespace where the source had none (`stats` +
    /// newline + `.iter()`), so only full removal is layout-independent. The
    /// needle literals themselves stay in their real, readable source form.
    fn strip_ws(s: &str) -> String {
        s.chars().filter(|c| !c.is_whitespace()).collect()
    }

    /// The number of `let [mut] <name>` bindings in `hay`.
    ///
    /// `<name>` is matched as a WHOLE identifier, so `let floors_str` is not a
    /// `floors` binding, and `mut` is skipped so a rebind cannot dodge the count
    /// by declaring itself mutable. Counted on the UN-stripped body: rustfmt
    /// never breaks between `let` and the name it binds.
    fn count_let_bindings(hay: &str, name: &str) -> usize {
        let mut count = 0;
        let mut cursor = 0;
        while let Some(rel) = hay[cursor..].find("let ") {
            let start = cursor + rel + "let ".len();
            cursor = start;
            let after_let = &hay[start..];
            let ident = match after_let.strip_prefix("mut ") {
                Some(rest) => rest,
                None => after_let,
            };
            if let Some(rest) = ident.strip_prefix(name) {
                if !rest.starts_with(|c: char| c.is_ascii_alphanumeric() || c == '_') {
                    count += 1;
                }
            }
        }
        count
    }

    /// The gate must actually READ the floor, RUN the retry, MEASURE
    /// interleaved, and DECIDE through the shared pure classifier.
    ///
    /// The floor pin is COMPOSED — the one `floors` binding IS the floor
    /// projection, and the classifier consumes THAT binding BY NAME — because
    /// two INDEPENDENT needles (a projection somewhere, a classifier call
    /// somewhere) are satisfied character for character by a gate that has
    /// restored the p50 gating the floor metric exists to kill. Two such evasions
    /// compile cleanly: a dead `let _floors = stats.iter().map(|&(s, f, ..)| (s,
    /// f)).collect();` parked beside a p50 vector handed to the classifier (the
    /// underscore keeps `unused_variables = deny` quiet), and a SHADOWING
    /// rebind — `let floors = <floors>; let floors = <p50s>;
    /// classify_flatness(&floors, ..)` — where even the by-name needle matches
    /// while the classifier reads the LATER binding. Composition closes the
    /// first; the occurrence pins close the second.
    #[test]
    fn the_gate_is_wired_to_the_shared_floor_decision() {
        let body = code_only_gate_body();
        let flat = strip_ws(&body);
        for (needle, why) in [
            (
                "measure_all_sizes_interleaved(",
                "the sweep must be measured in interleaved rounds, or one \
                 contiguous stall can own several sizes' floors",
            ),
            (
                "run_window_health_retries(",
                "with no outlier dropped, the retry is the SOLE \
                 single-window-stall defense — an unwired retry ships inert",
            ),
            (
                "classify_flatness(",
                "the verdict must come from the shared, oracle-tested classifier so the \
                 ratio-first-then-discriminator ordering cannot be mis-wired here",
            ),
            (
                "|&(s, f, ..)| (s, f)",
                "the gated metric must be the per-size FLOOR (tuple field 1), \
                 not p50 — gating on p50 is the whole defect",
            ),
            (
                "let floors: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();",
                "that floor projection must BE the `floors` binding — a projection \
                 that binds anything else is a dead statement standing beside a p50 gate",
            ),
            (
                "classify_flatness(&floors,",
                "the classifier must CONSUME that binding by name — a call handed any \
                 other vector gates on whatever that vector holds, however faithfully the \
                 projection above it reads",
            ),
        ] {
            assert!(
                flat.contains(&strip_ws(needle)),
                "the gate no longer contains `{needle}` — {why}"
            );
        }

        // The construction and the consumption above bind each other ONLY while
        // there is exactly one of each: a second `let floors` shadows the floor
        // vector with anything at all (both needles keep matching, and the
        // classifier reads the rebind), and a second `classify_flatness(` can
        // decide on a second vector while the first call keeps its needle
        // satisfied.
        assert_eq!(
            count_let_bindings(&body, "floors"),
            1,
            "the gate must bind `floors` EXACTLY once — a shadowing rebind leaves every needle \
             above matching while the classifier reads the LATER binding"
        );
        assert_eq!(
            count_let_bindings(&body, "_floors"),
            0,
            "the gate must not park a `_floors` binding — the underscore keeps `unused_variables \
             = deny` quiet, so a dead floor projection can stand beside a p50 gate"
        );
        assert_eq!(
            body.matches("classify_flatness(").count(),
            1,
            "the gate must call `classify_flatness` EXACTLY once — a second call can decide on a \
             second vector while the first keeps its needle above satisfied"
        );

        assert!(
            !body.contains("drop_one_outlier_robust_ratio"),
            "the gate must NOT use drop-one-outlier: this sweep's 2nd-largest size is 16x \
             smaller than its largest, so drop-one discards the 1 MiB floor a real copy lives \
             in and a 40 GB/s memcpy would ship GREEN (see the module header). It appears in \
             this file ONLY as the documented control in `gate_decision_oracles`."
        );
    }

    /// ANTI-TAUTOLOGY: without this, a walk over an empty or mis-extracted body
    /// would satisfy every "must NOT contain" assertion above for free.
    #[test]
    fn the_walk_reaches_real_code_and_the_comment_strip_works() {
        let raw = raw_gate_body();
        let body = code_only_gate_body();

        // It really is the gate, and it really does carry code.
        assert!(body.contains("FlatnessVerdict::Pass"));
        assert!(body.contains("global_dvfs_warmup()"));

        // The stripper genuinely removes comments: this phrase exists in the
        // gate's comments and nowhere in its code.
        assert!(raw.contains("conservative residual"));
        assert!(
            !body.contains("conservative residual"),
            "the comment strip is not working, so every forbidden-token assertion is vacuous"
        );

        // Line-only stripping is sound only while the file has no block
        // comments, so look for an opener OUTSIDE `//` line comments: text
        // after `//` is what the line rule already strips, so a path glob
        // such as the `nodes` directory wildcard in the module docs is not
        // an opener (a whole-file version of this check trips on
        // exactly that and fails on a healthy file). The
        // opener is spelled with an ESCAPED `*` so this literal is not
        // itself an occurrence — the same self-swallow guard `code_only` in
        // `cdylib_iox2_log_level_test` documents.
        let opener = "/\u{2a}";
        let block_comment_outside_line_comments = SRC
            .lines()
            .any(|line| line.split("//").next().unwrap_or("").contains(opener));
        assert!(
            !block_comment_outside_line_comments,
            "a block comment was introduced — the line-only strip is no longer sufficient"
        );
    }

    /// ANTI-TAUTOLOGY for the walk's own two helpers: a `count_let_bindings`
    /// that always answered 1, or a `strip_ws` that swallowed its input, would
    /// make every pin above pass against any gate at all.
    #[test]
    fn the_walk_helpers_count_and_strip_exactly_what_they_claim() {
        // Whole-identifier matching: the real gate body carries BOTH a `floors`
        // binding and a `floors_str` one, and conflating them inverts the
        // exactly-once pin.
        assert_eq!(count_let_bindings("let floors: Vec<u8> = v;", "floors"), 1);
        assert_eq!(count_let_bindings("let floors_str = v;", "floors"), 0);
        assert_eq!(count_let_bindings("let _floors = v;", "floors"), 0);
        assert_eq!(count_let_bindings("let _floors = v;", "_floors"), 1);
        // `mut` is skipped, so a rebind cannot dodge the count by taking it.
        assert_eq!(count_let_bindings("let mut floors = v;", "floors"), 1);
        // A shadowing pair is TWO bindings — the shape the pin refuses.
        assert_eq!(
            count_let_bindings("let floors = a; let floors = b;", "floors"),
            2
        );
        assert_eq!(count_let_bindings("floors = v;", "floors"), 0);

        // `strip_ws` erases layout and NOTHING else, so a needle written on one
        // line still matches a body rustfmt has wrapped.
        assert_eq!(strip_ws("a b\n\tc"), "abc");
        assert_eq!(strip_ws("stats\n    .iter()"), "stats.iter()");
        assert_eq!(strip_ws("|&(s, f, ..)| (s, f)"), "|&(s,f,..)|(s,f)");
        assert!(!strip_ws("let floors = p50s;").is_empty());
    }
}

// ============================================================
// Pure oracle arms pinning the GATE'S DECISION
// ============================================================
//
// The gate above needs a live graph, a quiet machine and seconds of wall time, so
// one run can only ever exercise ONE point of the decision surface — the
// healthy one. Every arm that matters (the false red the floor metric exists to
// stop, a real copy, an attributable stall, a healed window) is unreachable
// from a passing run by construction.
//
// These arms drive the SAME pure `cerulion_core::testing` decision the gate
// calls, over THIS FILE'S OWN constants (`PAYLOAD_SIZES`, `FLATNESS_MAX`,
// `MAX_RETRIES`), against hand-written floor vectors. No transport, no timing,
// no flake surface — so a wiring or constant regression that release timing
// cannot discriminate still fails loudly here.
mod gate_decision_oracles {
    use super::{FLATNESS_MAX, MAX_RETRIES, PAYLOAD_SIZES};
    use cerulion_core::testing::{
        classify_flatness, drop_one_outlier_robust_ratio, is_uniform_stall_signature,
        run_window_health_retries, FlatnessVerdict,
    };

    /// ns floors, in sweep order, paired with [`PAYLOAD_SIZES`].
    fn floors(v: [f64; 4]) -> Vec<(usize, f64)> {
        PAYLOAD_SIZES.iter().copied().zip(v).collect()
    }

    /// THE REGRESSION PIN, on a real false-red run's OWN recorded floors.
    ///
    /// A CI benchmark run read `p50 flatness 1.61x` on an error-string-only commit
    /// whose re-run measured 1.00x. That
    /// run's recorded per-size FLOORS were 5751 / 5720 / 8176 ns — a max/min of
    /// **1.429x**, i.e. UNDER the 1.5x ceiling. Gating on the floor instead of
    /// p50 is what makes that run pass, with no threshold
    /// change anywhere.
    ///
    /// Driven over the THREE sizes that run swept (it had no 4 KiB
    /// point), because inventing a fourth datum would be fabricating a
    /// measurement nobody took.
    #[test]
    fn the_false_red_passes_on_the_floor_metric() {
        let failing_run = vec![
            (64usize, 5751.0),
            (64 * 1024, 5720.0),
            (1024 * 1024, 8176.0),
        ];
        match classify_flatness(&failing_run, FLATNESS_MAX) {
            FlatnessVerdict::Pass(ff) => {
                assert!(
                    (ff.ratio - 8176.0 / 5720.0).abs() < 1e-9,
                    "ratio = {}",
                    ff.ratio
                );
                assert!(ff.ratio < FLATNESS_MAX, "ratio = {}", ff.ratio);
            }
            other => panic!("the recorded false red must PASS on floors, got {other:?}"),
        }

        // ANTI-TAUTOLOGY: the passing re-run of the same commit must also pass, or
        // "it passes" would be satisfied by a gate that passes everything. The
        // arms below supply the other half — inputs that must still FAIL.
        let passing_run = vec![
            (64usize, 3571.0),
            (64 * 1024, 3552.0),
            (1024 * 1024, 3535.0),
        ];
        assert!(matches!(
            classify_flatness(&passing_run, FLATNESS_MAX),
            FlatnessVerdict::Pass(_)
        ));
    }

    /// A REAL O(n) copy still fails NORMALLY — and the same vector documents why
    /// drop-one-outlier was rejected for this gate.
    ///
    /// Floors are computed from this test's measured ~3.55µs base plus an
    /// effective 40 GB/s memcpy (64 KiB is L2-resident, so that is a realistic
    /// bandwidth rather than a worst case): 4 KiB adds ~0.1µs, 64 KiB ~1.6µs,
    /// 1 MiB ~26µs. The FULL max/min ratio catches it at 8.4x; the drop-one
    /// ratio is 1.46x — UNDER the ceiling, i.e. drop-one would ship this copy
    /// GREEN because it discards the 1 MiB floor the copy actually lives in.
    #[test]
    fn a_realistic_o_n_copy_fails_the_full_ratio_but_would_slip_past_drop_one() {
        let copy = floors([3550.0, 3652.0, 5188.0, 29_764.0]);

        match classify_flatness(&copy, FLATNESS_MAX) {
            FlatnessVerdict::RealCopy(ff) => {
                assert!(ff.ratio > 8.0, "ratio = {}", ff.ratio);
                assert_eq!(
                    ff.max.0,
                    1024 * 1024,
                    "the copy must show at the LARGEST size"
                );
            }
            other => panic!("a real O(n) copy must fail as RealCopy, got {other:?}"),
        }

        // It is NOT the uniform signature: a copy's elevated floors span the
        // >= 16x payload-size step, far beyond the 1.3x band.
        assert!(
            !is_uniform_stall_signature(&copy, FLATNESS_MAX),
            "a monotonic-in-size copy must never be excused as a uniform stall"
        );

        // The reason drop-one is not used here (derived
        // on this file's own geometry): it discards the 1 MiB floor
        // and the surviving ratio passes.
        let rf = drop_one_outlier_robust_ratio(&copy);
        assert_eq!(rf.dropped.0, 1024 * 1024);
        assert!(
            rf.ratio < FLATNESS_MAX,
            "drop-one ratio {} should be a (wrongly) passing < {}x — the whole reason this \
             gate uses the FULL max/min",
            rf.ratio,
            FLATNESS_MAX
        );
    }

    /// A size-INDEPENDENT multi-window stall is routed to the ATTRIBUTABLE
    /// non-probative arm rather than reported as a copy.
    ///
    /// This is also the ANTI-INERT pin for the 4 KiB point:
    /// the arm is only reachable at all because the sweep is four sizes wide
    /// (see the companion arm below).
    #[test]
    fn a_uniform_multi_window_stall_is_attributable_at_this_sweep_width() {
        let stalled = floors([3550.0, 8000.0, 8200.0, 8400.0]);
        match classify_flatness(&stalled, FLATNESS_MAX) {
            FlatnessVerdict::UniformStall(ff) => {
                assert!(ff.ratio >= FLATNESS_MAX, "ratio = {}", ff.ratio);
            }
            other => panic!("an unhealed uniform stall must be attributable, got {other:?}"),
        }
    }

    /// What makes the 4 KiB point load-bearing:
    /// on a THREE-size sweep the uniform-stall arm is structurally unreachable,
    /// so on such a sweep it is dead code that looks like a defense.
    ///
    /// The reason is arithmetic, not tuning: the signature needs
    /// `>= UNIFORM_STALL_MIN_SIZES` (3) floors ABOVE `ceiling × min_floor`, and
    /// the size holding `min_floor` can never exceed `ceiling × min_floor` for
    /// any ceiling `>= 1` — so three sizes can present at most two elevated
    /// floors. Driven with the most extreme stall shape available (a 100x
    /// elevation on both non-min sizes), which still returns `false`.
    #[test]
    fn a_three_size_sweep_can_never_reach_the_uniform_stall_arm() {
        let extreme = vec![
            (64usize, 1000.0),
            (64 * 1024, 100_000.0),
            (1024 * 1024, 100_000.0),
        ];
        assert!(
            !is_uniform_stall_signature(&extreme, FLATNESS_MAX),
            "three sizes can present at most two elevated floors, so the >= 3 signature is \
             unreachable — this is why the sweep was widened to four"
        );
        // ...and it degrades to the conservative arm, never to a silent pass.
        assert!(matches!(
            classify_flatness(&extreme, FLATNESS_MAX),
            FlatnessVerdict::RealCopy(_)
        ));

        // The SAME shape at this file's four-size sweep IS attributable — so the
        // difference is the sweep WIDTH, not the numbers.
        let four = floors([1000.0, 100_000.0, 100_000.0, 100_000.0]);
        assert!(is_uniform_stall_signature(&four, FLATNESS_MAX));
    }

    /// Window-health retry: it is the SOLE single-window-stall defense when no
    /// outlier is dropped, so a would-fail single elevated window must HEAL to a
    /// pass when a fresh window measures it clean.
    #[test]
    fn the_window_health_retry_heals_a_single_stalled_window_to_pass() {
        let mut stats = vec![
            (PAYLOAD_SIZES[0], 3550.0, 3600.0, 4000.0, 5000.0),
            (PAYLOAD_SIZES[1], 3560.0, 3600.0, 4000.0, 5000.0),
            // One window stalled ~3x — would fail the gate outright.
            (PAYLOAD_SIZES[2], 11_000.0, 11_500.0, 12_000.0, 13_000.0),
            (PAYLOAD_SIZES[3], 3570.0, 3600.0, 4000.0, 5000.0),
        ];
        let pre: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(
            matches!(
                classify_flatness(&pre, FLATNESS_MAX),
                FlatnessVerdict::RealCopy(_)
            ),
            "precondition: the stalled sweep must FAIL before the retry"
        );

        // A fresh window finds the truer floor.
        let log = run_window_health_retries(&mut stats, FLATNESS_MAX, MAX_RETRIES, |_size| {
            (3555.0, 3600.0, 4000.0, 5000.0)
        });
        assert_eq!(log.len(), 1, "exactly the one elevated size is re-measured");
        assert_eq!(log[0].size, PAYLOAD_SIZES[2]);

        let post: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(matches!(
            classify_flatness(&post, FLATNESS_MAX),
            FlatnessVerdict::Pass(_)
        ));
    }

    /// A REAL copy never heals: the retry re-measures it just as slow, burns its
    /// per-size budget, and the gate still fails normally. Without this arm,
    /// "the retry heals a stall" would be indistinguishable from "the retry
    /// launders every failure into a pass".
    #[test]
    fn the_retry_cannot_launder_a_real_copy_into_a_pass() {
        let mut stats = vec![
            (PAYLOAD_SIZES[0], 3550.0, 3600.0, 4000.0, 5000.0),
            (PAYLOAD_SIZES[1], 3652.0, 3700.0, 4100.0, 5100.0),
            (PAYLOAD_SIZES[2], 5188.0, 5300.0, 5800.0, 6500.0),
            (PAYLOAD_SIZES[3], 29_764.0, 30_000.0, 31_000.0, 32_000.0),
        ];
        // The copy is deterministic: every fresh window measures the same cost.
        let log = run_window_health_retries(&mut stats, FLATNESS_MAX, MAX_RETRIES, |size| {
            let floor = if size == PAYLOAD_SIZES[2] {
                5188.0
            } else if size == PAYLOAD_SIZES[3] {
                29_764.0
            } else {
                3550.0
            };
            (floor, floor, floor, floor)
        });
        assert!(
            !log.is_empty(),
            "the elevated sizes must have been re-measured"
        );
        assert!(
            log.iter().all(|r| r.attempt <= MAX_RETRIES),
            "the per-size budget must bound the retries"
        );

        let post: Vec<(usize, f64)> = stats.iter().map(|&(s, f, ..)| (s, f)).collect();
        assert!(
            matches!(
                classify_flatness(&post, FLATNESS_MAX),
                FlatnessVerdict::RealCopy(_)
            ),
            "a real copy must survive the retry and still fail the gate"
        );
    }

    /// A healthy sweep pays NOTHING: the remeasure closure is never called
    /// (proven by panicking inside it), so the retry costs a clean machine zero
    /// extra graph runs.
    #[test]
    fn a_healthy_sweep_re_measures_nothing() {
        let mut stats = vec![
            (PAYLOAD_SIZES[0], 3571.0, 3600.0, 4000.0, 5000.0),
            (PAYLOAD_SIZES[1], 3552.0, 3600.0, 4000.0, 5000.0),
            (PAYLOAD_SIZES[2], 3535.0, 3600.0, 4000.0, 5000.0),
            (PAYLOAD_SIZES[3], 3560.0, 3600.0, 4000.0, 5000.0),
        ];
        let log = run_window_health_retries(&mut stats, FLATNESS_MAX, MAX_RETRIES, |size| {
            panic!("a healthy sweep must not re-measure any size (got {size})");
        });
        assert!(log.is_empty());
    }

    /// Drift guards on the constants the arms above reason about.
    #[test]
    fn the_gated_constants_are_what_the_reasoning_assumes() {
        assert_eq!(
            PAYLOAD_SIZES.len(),
            4,
            "the uniform-stall arm needs >= 4 sizes"
        );
        assert!(
            (FLATNESS_MAX - 1.5).abs() < 1e-9,
            "the floor rewrite must not widen the moat ceiling"
        );
        // The >= 16x adjacent step is what makes a copy inexpressible as a
        // uniform stall (see `UNIFORM_STALL_BAND`).
        for w in PAYLOAD_SIZES.windows(2) {
            assert!(
                w[1] >= w[0] * 16,
                "adjacent sizes must differ by >= 16x, got {} -> {}",
                w[0],
                w[1]
            );
        }
    }
}
