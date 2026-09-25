// SPDX-License-Identifier: AGPL-3.0-only
//! PER-HOP decomposition of the period-driven moat-graph RTT.
//!
//! # The question this bench answers
//!
//! The moat graph (`graph_latency_test` / `cli_e2e_graph_latency_test`:
//! `ping period_ms=1 → pong → latency`) has real-clock p50 ~18µs with the CPU
//! DMA-latency lock OFF and ~5µs with it ON. That ~13µs delta is
//! C-state-exit latency. The moat RTT is `latency_tick_real_ns -
//! ping_tick_real_ns` — both stamped when their tick RUNS (after any wake), so
//! the period TIMER wake itself is EXCLUDED from the measured window. Therefore
//! the ~13µs must live somewhere in the ping→pong→latency PROPAGATION (the
//! inter-node hops), not in the period scheduling. This bench measures EACH hop
//! separately to localize it.
//!
//! # The graph — a 3-node period-driven chain with per-hop stamps
//!
//! ```text
//! src (period_ms=1) ──▶ relay ──▶ sink
//!   (self-generating)    (L0)      (L1)
//! ```
//!
//! Two timestamps travel in ONE `geometry_msgs::Vector3` per message (RELATIVE
//! nanoseconds as `f64` off a process-global `T0` `Instant` — exact for integers
//! under 2^53 ns ≈ 2.5 h, plenty for a seconds-long bench, and it avoids defining
//! a new u64 schema; same trick as `streaming_latency_bench_test`):
//!   * `x` = `t_src`   — the period source's tick (publish) time.
//!   * `y` = `t_relay` — relay's tick (publish) time, overwritten by relay.
//!   * `z` = unused.
//!
//! The sink computes THREE per-message latencies from those stamps + its own
//! `now_rel()`:
//!   * `seg1 = t_relay - t_src` — the src→relay hop. If the period source's
//!     publish lands while the live loop is parked in the WaitSet `epoll`, relay
//!     pays the OS-wakeup + C-state-exit cost here. THIS is the candidate for the
//!     ~13µs.
//!   * `seg2 = now_rel() - t_relay` — the relay→sink hop. Within a single
//!     `step` the level executor collapses relay→sink (the
//!     within-step collapse), so sink fires in the SAME wake as relay — `seg2`
//!     does NOT cross a fresh OS block. This is the control: if it is small while
//!     `seg1` is large, the latency is the cross-step wake; if BOTH are large,
//!     the chain is NOT collapsing in-step.
//!   * `e2e  = now_rel() - t_src` — the full src→relay→sink path (= the moat RTT
//!     analog: `seg1 + seg2`).
//!
//! # Why a PERIOD source (no external publisher thread)
//!
//! This mirrors the moat graph precisely: the `period_ms=1` source self-drives
//! (the scheduler fires it on its own timer), so there is NO out-of-graph
//! publisher thread — the decomposition is of the exact period-driven topology,
//! not a data-driven analog. The period timer keeps the core warm every 1ms, so
//! whether the per-hop wake still pays a C-state exit on the benchmark machine is
//! exactly the open question; the per-hop split localizes it.
//!
//! # Real clock vs the build_for_test VirtualClock
//!
//! `build_for_test` mandates an `Arc<VirtualClock>` (the graph's deterministic
//! `watch_clock`). That does NOT undermine the bench: `run_live` advances the
//! VirtualClock by REAL elapsed wall time, and the nodes stamp REAL wall time via
//! `now_rel()` off a shared `Instant` epoch — exactly like
//! `streaming_latency_bench_test` / `cli_e2e_graph_latency_test` measure real
//! RTTs under a virtual graph clock. The C-state-exit cost is paid on real wall
//! time regardless of the graph clock.
//!
//! # Spin / DMA-lock — read externally, no test code
//!
//! `run_live` reads `CERULION_LIVE_SPIN_US` and `CERULION_CPU_DMA_LOCK` itself.
//! This bench takes NO such parameter: run the SAME binary with those env vars
//! set/unset to compare. The printed per-hop floors are the moat metric; the
//! C-state magnitude shows on a Linux benchmark machine, NOT a macOS development machine (the
//! macOS self-verify only proves the bench RUNS + DECOMPOSES).
//!
//! # Release-only + serial
//!
//! `#![cfg(not(debug_assertions))]` (like `graph_latency_test`): µs numbers are
//! meaningless in debug (10-50× slower), so a plain `cargo test` compiles this to
//! an empty no-op. `#[serial]` — the live WaitSet builds over the process-global
//! iceoryx2 SHM singleton.
//!
//! This is a MANUAL bench: it asserts only LIVENESS (got ≥ SAMPLES samples; all
//! three segments finite and e2e > 0) — NO absolute-µs gate (box-specific). The
//! value is the printed `[seg1|seg2|e2e] floor=.. p50=..` breakdown.
//!
//! # Status: NOTHING EXECUTES THIS FILE — it is TYPE-CHECKED only
//!
//! "Release-only" understates it, so state the whole of it. A debug `cargo test`
//! empties the file through the `cfg` gate above; the release steps name
//! SPECIFIC `--test` targets (`latency_threshold_test`, `graph_latency_test`,
//! `cerulion_core --lib`, and `cli_e2e_graph_latency_test` in its own job), and
//! this binary is named by none of them. So no job anywhere RUNS it. The one
//! thing that touches it is the release rot guard — `cargo check -p
//! cerulion_core --tests --release` — which type-checks it so a `cerulion_core`
//! API change cannot rot it with every job green. That guard is load-bearing
//! here and must stay: without it this file is compiled by nothing at all.
//!
//! It is kept despite that because of `seg1`, and the claim has to be narrowed
//! to stay true. Nothing under `benches/` splits an RTT per hop at all (verified
//! by grep: no `seg1`/`seg2`/`per-hop` anywhere in that tree), but inside
//! `tests/` the sibling `streaming_latency_bench_test` splits its own chain, and
//! its `hop2` is the SAME within-step-collapse measurement as `seg2` — its
//! header says so in as many words. So `seg2` is NOT unique; the TOPOLOGY is:
//! this is the only per-hop split on a PERIOD-driven graph, the moat graph's own
//! shape, which makes `seg1` (period source -> relay) the only measurement of
//! the hop the C-state delta was attributed to. The sibling's `hop1` crosses
//! from an out-of-graph publisher instead. Deleting this file would delete
//! `seg1`, not a duplicate of it. Run it by hand when the moat moves.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test period_decompose_bench_test --release \
//!     -- --nocapture --test-threads=1
//! # C-state baseline vs lock (on Linux):
//! CERULION_CPU_DMA_LOCK=1 cargo test -p cerulion_core --test \
//!     period_decompose_bench_test --release -- --nocapture --test-threads=1
//! ```

// Release-only: the absolute-µs numbers are production-profile, so under a plain
// `cargo test` (no `--release`) this whole module compiles to an empty no-op.
#![cfg(not(debug_assertions))]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Process-global monotonic epoch. Set ONCE at the start of the bench; the
/// source, relay, and sink ALL stamp `now_rel()` off it so every timestamp
/// shares the same zero — making `t_relay - t_src` a real cross-node delta.
static T0: OnceLock<Instant> = OnceLock::new();

/// Relative nanoseconds since [`T0`], as `f64`. Exact for integers under
/// 2^53 ns ≈ 2.5 h — the bench runs for seconds, so there is no precision loss.
/// Used as the wire timestamp in every `Vector3` field.
fn now_rel() -> f64 {
    T0.get()
        .expect("T0 must be initialized before now_rel()")
        .elapsed()
        .as_nanos() as f64
}

// ===========================================================================
// Nodes — a period source + 2-level forward chain (mirrors the moat graph).
// ===========================================================================

/// The PERIOD source (mirrors the moat graph's `ping period_ms=1`). Self-driving:
/// the scheduler fires it on its own 1ms timer — NO external publisher thread.
/// Stamps its tick time into `x` (`t_src`); `y`/`z` are zeroed (relay fills `y`).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Src {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Src {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = now_rel(); // t_src
        self.out.y = 0.0;
        self.out.z = 0.0;
        Ok(())
    }
}

/// L0: triggers on the source's output topic. Forwards the source's `t_src`
/// (in `x`) UNCHANGED and stamps its OWN publish time `t_relay` into `y`, so the
/// sink can split the src→relay hop (`seg1 = y - x`) from the relay→sink hop
/// (`seg2 = now - y`).
#[cerulion_node]
#[derive(Default)]
struct Relay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Relay {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Preserve the source publish time (the moat metric's t0)...
        self.out.x = self.inp.x;
        // ...and stamp relay's own publish time so seg1/seg2 are separable.
        self.out.y = now_rel();
        self.out.z = 0.0;
        Ok(())
    }
}

/// L1: triggers on `relay/out`. Reads `t_src` (`inp.x`) + `t_relay` (`inp.y`),
/// computes the three segment latencies against `now_rel()`, and pushes them
/// into a shared `Vec` (the measurement sink). Holds the shared
/// `Arc<Mutex<Vec<..>>>` as a state field, constructed via the macro's
/// `with_state` in the factory.
#[cerulion_node]
#[derive(Default)]
struct Sink {
    #[input(trigger)]
    inp: Vector3,
    /// Each entry: `(seg1_ns, seg2_ns, e2e_ns)` as `f64`.
    samples: Arc<Mutex<Vec<(f64, f64, f64)>>>,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = now_rel();
        let t_src = self.inp.x;
        let t_relay = self.inp.y;
        // seg1 = src → relay (the candidate cross-step external wake).
        let seg1 = t_relay - t_src;
        // seg2 = relay → sink (the internal/post-relay hop — flat if in-step).
        let seg2 = now - t_relay;
        // e2e = the full src → relay → sink path (= seg1 + seg2; the moat RTT).
        let e2e = now - t_src;
        self.samples.lock().unwrap().push((seg1, seg2, e2e));
        Ok(())
    }
}

// ===========================================================================
// Graph construction (hand-built, like polled_vs_live's chain_graph).
// ===========================================================================

/// An `OutputDef` for a `geometry_msgs/Vector3` producer with every resolution
/// knob at its default (derived topic, runtime-resolved slice len, volatile
/// history) — the shape `build_for_test` requires.
fn vector3_output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

/// Build the period-source + 2-level forward-chain config + factories.
/// `samples` is the sink's shared measurement Vec (captured via `with_state`).
fn chain_graph(
    samples: Arc<Mutex<Vec<(f64, f64, f64)>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "period_decompose_bench".to_string(),
        prefix: "pd".to_string(),
        nodes: vec![
            // The period source — an in-graph producer with NO inputs (a normal
            // in-graph topic, NOT an absolute external `source:`).
            NodeDef {
                fuse: None,
                ros2: None,
                id: "src".to_string(),
                node_type: "src".to_string(),
                inputs: vec![],
                outputs: vec![vector3_output("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    // relay triggers on the source's output topic.
                    source: "src/out".to_string(),
                }],
                outputs: vec![vector3_output("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    // sink triggers on relay's output topic.
                    source: "relay/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    // `build_for_test` keys the factory map by node ID (here ID == type).
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(SrcEntry::new()));
    factories.insert("relay".to_string(), Box::new(RelayEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(SinkEntry::with_state(Sink {
            samples,
            ..Default::default()
        })),
    );
    (config, factories)
}

// ===========================================================================
// Bench knobs (env-overridable, with defaults).
// ===========================================================================

/// Read a `usize` env knob, falling back to `default` when unset/empty/bad.
fn env_usize(key: &str, default: usize) -> usize {
    match std::env::var(key) {
        Ok(v) if !v.is_empty() => v.parse().unwrap_or(default),
        _ => default,
    }
}

/// Number of MEASURED samples (after warmup) the run targets.
fn samples_target() -> usize {
    env_usize("CER_PD_SAMPLES", 3_000)
}

/// Warmup samples discarded before measurement (warms caches + the SHM pool +
/// the pub↔sub connection handshake on both legs).
fn warmup() -> usize {
    env_usize("CER_PD_WARMUP", 500)
}

// ===========================================================================
// Percentile / floor helpers (logic reused from cli_e2e_graph_latency_test).
// ===========================================================================

/// Nearest-rank percentile from an ascending-sorted `f64` slice (`p` in
/// 0.0..=1.0). Mirrors `cli_e2e_graph_latency_test::percentile_ns`.
fn percentile(sorted: &[f64], p: f64) -> f64 {
    assert!(!sorted.is_empty(), "no samples for percentile");
    let rank = ((sorted.len() as f64 - 1.0) * p).round() as usize;
    sorted[rank.min(sorted.len() - 1)]
}

/// Minimum (uncontended floor) of an ascending-sorted slice. Mirrors
/// `cli_e2e_graph_latency_test::floor_ns` — robust to CI-VM jitter (noise only
/// adds latency, never lowers the floor).
fn floor(sorted: &[f64]) -> f64 {
    *sorted.first().expect("no samples for floor")
}

/// Sort one segment's samples ascending and print its `floor` + `p50` in µs.
/// Returns `(floor_ns, p50_ns)` for the liveness asserts.
fn report_seg(label: &str, mut seg_ns: Vec<f64>) -> (f64, f64) {
    seg_ns.sort_by(|a, b| a.partial_cmp(b).expect("latency is finite"));
    let f = floor(&seg_ns);
    let p50 = percentile(&seg_ns, 0.50);
    println!(
        "  [{label:>11}] floor = {:>9.3} µs   p50 = {:>9.3} µs   (n = {})",
        f / 1000.0,
        p50 / 1000.0,
        seg_ns.len()
    );
    (f, p50)
}

// ===========================================================================
// The bench.
// ===========================================================================

#[test]
#[serial]
fn period_decompose_per_hop_latency() {
    // The process-global epoch shared by src + relay + sink.
    let _ = T0.set(Instant::now());

    let target = samples_target();
    let warmup = warmup();
    let total = target + warmup;

    // Whether the derived/explicit scheduled-spin + DMA-lock are active, for the
    // header (the RUNTIME reads these itself; we only echo them).
    let spin_env = std::env::var("CERULION_LIVE_SPIN_US").ok();
    let dma_env = std::env::var("CERULION_CPU_DMA_LOCK").ok();
    println!(
        "=== period decompose bench (period src→relay→sink, per-hop) ===\n\
         target_samples={target} warmup={warmup}  \
         CERULION_LIVE_SPIN_US={}  CERULION_CPU_DMA_LOCK={}",
        spin_env.as_deref().unwrap_or("<unset:derived>"),
        dma_env.as_deref().unwrap_or("<unset>")
    );

    // --- Build the graph (VirtualClock per build_for_test; run_live advances it
    // by real elapsed and the nodes stamp real wall time via now_rel() — see the
    // module header). ---
    let samples = Arc::new(Mutex::new(Vec::<(f64, f64, f64)>::with_capacity(total)));
    let (config, factories) = chain_graph(Arc::clone(&samples));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build period-decompose bench graph");

    // `running` drives the live loop; the stopper thread clears it once enough
    // samples land (or the wall timeout trips). The period source self-drives —
    // NO publisher thread.
    let running = Arc::new(AtomicBool::new(true));

    // ---- STOPPER thread ----
    // Poll the shared Vec len; stop the live loop once `total` samples land OR a
    // generous wall timeout trips (so a stall can never hang the suite). At
    // period_ms=1 (~1000 samples/s) `total` (default 3500) lands in ~3.5s; the
    // 30s timeout is ~8× margin.
    let stop_handle = {
        let running = Arc::clone(&running);
        let samples = Arc::clone(&samples);
        std::thread::spawn(move || {
            let deadline = Instant::now() + std::time::Duration::from_secs(30);
            loop {
                if samples.lock().unwrap().len() >= total {
                    break;
                }
                if Instant::now() >= deadline {
                    eprintln!(
                        "period decompose bench: wall timeout (30s) before {total} samples \
                         — stopping the live loop with what we have"
                    );
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            running.store(false, Ordering::Relaxed);
        })
    };

    // ---- MAIN thread: drive the LIVE WaitSet reactor until stopped. ----
    // This is the production live path — it spins-then-blocks per Part B,
    // reading CERULION_LIVE_SPIN_US / CERULION_CPU_DMA_LOCK itself. It returns
    // when `running` clears. The period source fires on the scheduler's own 1ms
    // timer (the run_live timeout tracks the next Period deadline).
    runtime
        .run_live(&running)
        .expect("run_live must not be refused (no host-driven external nodes in this bench)");

    stop_handle.join().expect("stopper thread panicked");
    runtime.shutdown();

    // ---- Compute + report the three per-hop latencies. ----
    let all = samples.lock().unwrap().clone();
    assert!(
        all.len() > warmup,
        "bench collected only {} samples (<= warmup {warmup}) — data did NOT flow \
         through the period src→relay→sink live graph (check the period source / \
         WaitSet wiring)",
        all.len()
    );

    // (The optional inter-fire interval is intentionally omitted: the sink stores
    // only relative segment latencies, not absolute sink-arrival times, so a
    // consecutive-sample wall gap isn't reconstructable without changing the wire
    // payload. The period rate is fixed by `period_ms=1` ≈ 1000 samples/s, and
    // the total-samples / wall-time ratio in the run output already reflects it.)

    // Drop the warmup prefix; split into per-segment columns.
    let measured = &all[warmup..];
    let seg1: Vec<f64> = measured.iter().map(|&(s1, _, _)| s1).collect();
    let seg2: Vec<f64> = measured.iter().map(|&(_, s2, _)| s2).collect();
    let e2e: Vec<f64> = measured.iter().map(|&(_, _, e)| e).collect();

    println!(
        "--- per-hop latency over {} measured samples (warmup {warmup} dropped) ---\n\
         seg1 = src→relay   (cross-step wake candidate — the C-state cost lands HERE \
         if it lands anywhere)\n\
         seg2 = relay→sink  (internal/post-relay; FLAT if the level executor \
         collapses relay→sink in-step)\n\
         e2e  = src→relay→sink (full path = seg1 + seg2; the moat RTT analog)",
        measured.len()
    );
    let (seg1_floor, _seg1_p50) = report_seg("seg1 s→r", seg1);
    let (seg2_floor, _seg2_p50) = report_seg("seg2 r→s", seg2);
    let (e2e_floor, _e2e_p50) = report_seg("e2e", e2e);

    // ---- LIVENESS-ONLY asserts (no absolute-µs gate — box-specific). ----
    assert!(
        measured.len() >= target,
        "expected >= {target} measured samples, got {} — the live graph under-delivered",
        measured.len()
    );
    // seg1/seg2 must be finite (they can in principle be ~0 if both stamps land in
    // the same tick window, but never NaN/inf); e2e must be finite AND > 0 (the
    // sink always runs strictly after the source).
    assert!(
        seg1_floor.is_finite(),
        "[seg1] floor must be finite, got {seg1_floor} ns — a clock/stamp bug",
    );
    assert!(
        seg2_floor.is_finite(),
        "[seg2] floor must be finite, got {seg2_floor} ns — a clock/stamp bug",
    );
    assert!(
        e2e_floor.is_finite() && e2e_floor > 0.0,
        "[e2e] floor must be finite and > 0, got {e2e_floor} ns — the sink must run \
         strictly after the source (a clock/stamp bug)",
    );
}
