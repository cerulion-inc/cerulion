// SPDX-License-Identifier: AGPL-3.0-only
//! Scheduled spin-then-block latency bench on a DATA-DRIVEN
//! graph (the workload the spin actually helps).
//!
//! # Why this graph, not the period-driven moat graph
//!
//! The scheduled-spin optimization (`GraphRuntime::spin_budget` +
//! `spin_sources`, see `runtime.rs`) busy-polls the data-trigger listeners for a
//! short wall-clock window BEFORE falling back to the blocking WaitSet wait. Its
//! entire value is on a CROSS-STEP external data hop: a message that arrives
//! while the live loop is parked in the WaitSet `epoll` pays the OS wakeup +
//! C-state-exit latency (~18µs on an idle Linux machine, as measured). The
//! spin catches that wake in user space (the CPU never sleeps), recovering the
//! poll-loop floor (~5µs).
//!
//! The period-driven `graph_latency_test` / `cli_e2e_graph_latency_test` moat
//! graph does NOT expose this: a `period_ms` producer keeps the loop warm (its
//! own timer wake re-arms the core every period), so there is no idle C-state to
//! exit and the spin has nothing to recover. A VALID demonstration needs a
//! PURE-data-driven graph whose live loop genuinely IDLES between external
//! arrivals — which is exactly this graph (NO Period node anywhere; modeled on
//! `polled_vs_live_iox2_test`'s pure-external-data chain).
//!
//! # The graph (NO Period node)
//!
//! ```text
//! /sb/ext (absolute external) ──▶ relay ──▶ sink
//!   (out-of-graph publisher;        (L0)      (L1)
//!    relay's data-trigger)
//! ```
//!
//! Three timestamps travel in ONE `geometry_msgs::Vector3` per message (all
//! RELATIVE nanoseconds as `f64` off a process-global `T0` `Instant` — exact
//! under 2^53 ns ≈ 2.5 h, plenty for a short bench, and it avoids defining a new
//! u64 schema):
//!   * `x` = `t_ext`   — the out-of-graph publisher's publish time.
//!   * `y` = `t_relay` — relay's tick (publish) time, overwritten by relay.
//!
//! The sink computes THREE per-message latencies from those stamps + its own
//! `now_rel()`:
//!   * `hop1 = t_relay - t_ext`  — the CROSS-STEP external wake (ext → relay).
//!     THIS is the hop the spin helps: the external publish lands while the live
//!     loop is parked, so the wakeup mechanism (spin vs WaitSet block) is on the
//!     critical path.
//!   * `hop2 = now_rel() - t_relay` — the INTERNAL/post-relay hop (relay → sink).
//!     Within a single `step()` the level executor collapses relay→sink
//!     (the within-step level collapse), so sink fires in the SAME
//!     wake as relay — `hop2` does NOT cross a fresh OS block and is FLAT
//!     regardless of the spin setting. This is the control: it proves the spin
//!     effect is specific to the cross-step external hop, not a global shift.
//!   * `e2e  = now_rel() - t_ext`  — the full ext → relay → sink path.
//!
//! # Spin on vs off — read externally, no test code needed
//!
//! `GraphRuntime::run_live` reads `CERULION_LIVE_SPIN_US` itself
//! (`spin_budget`). The bench takes NO spin parameter: run the SAME binary
//! twice, once with `CERULION_LIVE_SPIN_US` unset/derived or a positive budget
//! (spin ON ⇒ `hop1` ≈ the poll floor) and once with `CERULION_LIVE_SPIN_US=0`
//! (spin OFF ⇒ `hop1` ≈ the blocked/C-state latency). The printed `hop1` floor
//! is the moat metric; the spin WIN (the ~18µs→~5µs `hop1` drop) shows on the
//! Linux benchmark machine, NOT on a macOS development machine (macOS idle wake latency differs
//! and this test only self-verifies that the bench RUNS + MEASURES locally).
//!
//! # Real clock vs the build_for_test VirtualClock
//!
//! `GraphRuntime::build_for_test` mandates an `Arc<VirtualClock>` (the graph's
//! deterministic `watch_clock`). That does NOT undermine the bench: the
//! spin/block wakeup mechanism in `live_step` uses the REAL [`std::time::Instant`]
//! wall clock for BOTH the spin deadline (`spin_sources`) AND the WaitSet
//! `run_once` block — by design (the record-only firewall keeps the spin off the
//! deterministic timeline). So the C-state-exit cost is paid on real wall time
//! regardless of the graph clock, and `run_live` blocks/spins on real time. The
//! THREE measured latencies are likewise computed from `real_ns()`-style
//! relative `Instant` stamps (`now_rel()`), independent of the graph clock —
//! mirroring how `cli_e2e_graph_latency_test` measures real RTTs under a virtual
//! graph clock. The VirtualClock is the right (and only) `build_for_test` choice;
//! the wakeup latency it measures is real.
//!
//! # Release-only + serial
//!
//! `#![cfg(not(debug_assertions))]` (like `cli_e2e_graph_latency_test` /
//! `graph_latency_test`): µs latency numbers are meaningless in debug (10-50×
//! slower), so a plain `cargo test` compiles this to an empty no-op. `#[serial]`
//! — the live WaitSet builds over the process-global iceoryx2 SHM singleton.
//!
//! This is a MANUAL bench: it asserts only LIVENESS (got ≥ SAMPLES samples; all
//! three latencies finite and > 0) — NO absolute-µs gate (box-specific). The
//! value is the printed `[hop1|hop2|e2e] floor=.. p50=..` numbers.
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
//! It is kept despite that because nothing else MEASURES the spin. Be precise
//! about what that does and does not claim: the spin's BEHAVIOUR is pinned in CI
//! by `live_spin_budget_test` (the mechanism + the budget derivation) and
//! `unified_stale_wake_park_test` (the stale-wake regression), and
//! `external_live_fire_iox2_test` sets the knob too. The sibling
//! `period_decompose_bench_test` also invites the same env A/B, but on a graph
//! whose `period_ms` source keeps the core warm every 1 ms — and it does NOT
//! conclude the spin has nothing to recover there: its header calls "whether the
//! per-hop wake still pays a C-state exit on the benchmark machine" exactly the open
//! question its split exists to localize, and names `seg1` as the candidate. The
//! "nothing to recover" reading is THIS file's own, stated at the top about the
//! `graph_latency_test` moat graph, and it is why the A/B below needs an idling
//! graph rather than a period-driven one. What has no other instrument anywhere —
//! `benches/` included, verified by grep for
//! `CERULION_LIVE_SPIN_US`/`spin_budget` in that tree — is the µs-level spin-ON
//! vs spin-OFF A/B on a graph whose live loop genuinely IDLES, which is the only
//! workload where the spin can show a number. Conversely this file does NOT
//! claim its `hop1`/`hop2` split is unique: the sibling splits a period-driven
//! chain the same way, and `hop2` and its `seg2` are the same within-step-collapse
//! measurement. Deleting this file would delete the idling-graph A/B, not a
//! duplicate of it. Run it by hand when the wake path changes.
//!
//! # Running
//!
//! ```bash
//! # Spin ON (derived default):
//! cargo test -p cerulion_core --test streaming_latency_bench_test --release \
//!     -- --nocapture --test-threads=1
//! # Spin OFF (block immediately — the C-state baseline):
//! CERULION_LIVE_SPIN_US=0 cargo test -p cerulion_core --test \
//!     streaming_latency_bench_test --release -- --nocapture --test-threads=1
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
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The absolute external trigger topic for `relay`. No in-graph producer ⇒ the
/// graph provisions it as `External` (buffer-ceiling only, no single-writer cap)
/// and the out-of-graph bench publisher attaches freely (mirrors
/// `polled_vs_live_iox2_test::EXT_TOPIC`).
const EXT_TOPIC: &str = "/sb/ext";

/// Process-global monotonic epoch. Set ONCE at the start of the bench; the
/// publisher, relay, and sink ALL stamp `now_rel()` off it so every timestamp
/// shares the same zero — making `t_relay - t_ext` a real cross-node delta.
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
// Nodes — a 2-level forward chain, pure data-trigger (NO Period anywhere).
// ===========================================================================

/// L0: triggers on the absolute external `/sb/ext`. Forwards the publisher's
/// `t_ext` (in `x`) UNCHANGED and stamps its OWN publish time `t_relay` into
/// `y`, so the sink can split the cross-step external hop (`hop1 = y - x`) from
/// the internal hop (`hop2 = now - y`).
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
        // Preserve the external publish time (the moat metric's t0)...
        self.out.x = self.inp.x;
        // ...and stamp relay's own publish time so hop1/hop2 are separable.
        self.out.y = now_rel();
        Ok(())
    }
}

/// L1: triggers on `relay/out`. Reads `t_ext` (`inp.x`) + `t_relay` (`inp.y`),
/// computes the three latencies against `now_rel()`, and pushes them into a
/// shared `Vec` (the measurement sink). Holds the shared `Arc<Mutex<Vec<..>>>`
/// as a state field, constructed via the macro's `with_state` in the factory.
#[cerulion_node]
#[derive(Default)]
struct Sink {
    #[input(trigger)]
    inp: Vector3,
    /// Each entry: `(hop1_ns, hop2_ns, e2e_ns)` as `f64`.
    samples: Arc<Mutex<Vec<(f64, f64, f64)>>>,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = now_rel();
        let t_ext = self.inp.x;
        let t_relay = self.inp.y;
        // hop1 = ext → relay (the CROSS-STEP external wake — the spin helps here).
        let hop1 = t_relay - t_ext;
        // hop2 = relay → sink (the INTERNAL/post-relay hop — flat regardless).
        let hop2 = now - t_relay;
        // e2e = the full ext → relay → sink path.
        let e2e = now - t_ext;
        self.samples.lock().unwrap().push((hop1, hop2, e2e));
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

/// Build the 2-level forward-chain config + factories. `samples` is the sink's
/// shared measurement Vec (captured into the sink via `with_state`).
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
        identity: "streaming_latency_bench".to_string(),
        prefix: "sb".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC.to_string(),
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
                    source: "relay/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    // `build_for_test` keys the factory map by node ID (here ID == type).
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
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
    env_usize("CER_SB_SAMPLES", 2_000)
}

/// Inter-arrival gap between external publishes, in microseconds. The default
/// (50µs) is chosen so the live loop IDLES between messages (the publisher
/// busy-waits the gap), so the spin-OFF run pays the C-state-exit cost on each
/// `hop1` and the spin-ON run recovers it.
fn interval_us() -> u64 {
    env_usize("CER_SB_INTERVAL_US", 50) as u64
}

/// Warmup samples discarded before measurement (warms caches + the SHM pool +
/// the pub↔sub connection handshake on both legs).
fn warmup() -> usize {
    env_usize("CER_SB_WARMUP", 200)
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

/// Sort one hop's samples ascending and print its `floor` + `p50` in µs.
/// Returns `(floor_ns, p50_ns)` for the liveness asserts.
fn report_hop(label: &str, mut hop_ns: Vec<f64>) -> (f64, f64) {
    hop_ns.sort_by(|a, b| a.partial_cmp(b).expect("latency is finite"));
    let f = floor(&hop_ns);
    let p50 = percentile(&hop_ns, 0.50);
    println!(
        "  [{label:>4}] floor = {:>9.3} µs   p50 = {:>9.3} µs   (n = {})",
        f / 1000.0,
        p50 / 1000.0,
        hop_ns.len()
    );
    (f, p50)
}

// ===========================================================================
// The bench.
// ===========================================================================

#[test]
#[serial]
fn streaming_latency_bench_spin_then_block() {
    // The process-global epoch shared by publisher + relay + sink.
    let _ = T0.set(Instant::now());

    let target = samples_target();
    let warmup = warmup();
    let interval_us = interval_us();
    let total = target + warmup;

    // Whether the derived/explicit scheduled-spin is active, for the header (the
    // RUNTIME reads this itself in spin_budget; we only echo it).
    let spin_env = std::env::var("CERULION_LIVE_SPIN_US").ok();
    println!(
        "=== streaming latency bench (data-driven ext→relay→sink) ===\n\
         target_samples={target} warmup={warmup} interval_us={interval_us}  \
         CERULION_LIVE_SPIN_US={}",
        spin_env.as_deref().unwrap_or("<unset:derived>")
    );

    // --- Build the graph (VirtualClock per build_for_test; the live-loop
    // wakeup latency is paid on real Instant wall time regardless — see the
    // module header). ---
    let samples = Arc::new(Mutex::new(Vec::<(f64, f64, f64)>::with_capacity(total)));
    let (config, factories) = chain_graph(Arc::clone(&samples));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build streaming-latency bench graph");

    // Attach the out-of-graph external publisher on the absolute trigger topic.
    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /sb/ext")
    };

    // `running` drives the live loop; the stopper thread clears it once enough
    // samples land (or the wall timeout trips).
    let running = Arc::new(AtomicBool::new(true));

    // ---- PUBLISHER thread ----
    // CLOSED-LOOP: keep loaning/publishing Vector3 frames carrying `x = t_ext`
    // (relay overwrites y/z) — each separated by a precise `interval_us`
    // inter-arrival — UNTIL the stopper clears `running` (it does so once the
    // sink has collected `total` samples). Closed-loop (vs a fixed `total`-count
    // open loop) is load-bearing: the external `/sb/ext` topic is a depth-bounded
    // `drop_oldest` queue, so a publish that lands while the live loop is mid-step
    // can be evicted before it is drained — an open loop would under-deliver by
    // exactly the eviction count and never reach `total`. Publishing until the
    // sink is satisfied transparently replaces any dropped frame. We busy-wait on
    // `Instant` (not `thread::sleep`, too coarse at µs scale) so the gaps are real
    // and the live loop genuinely idles between arrivals (the spin-OFF C-state
    // baseline depends on that idle).
    let pub_handle = {
        let running = Arc::clone(&running);
        std::thread::spawn(move || {
            // ^^^ pubr MOVED across the thread boundary — only compiles because
            // the ipc_threadsafe swap made CerulionPublisher Send
            // (cross_thread_rtt_test is the dedicated proof).
            while running.load(Ordering::Relaxed) {
                {
                    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan vector3");
                    proxy.x = now_rel(); // t_ext
                    proxy.y = 0.0;
                    proxy.z = 0.0;
                    // proxy drops here → published.
                }
                // Precise busy-wait inter-arrival.
                let deadline = Instant::now() + std::time::Duration::from_micros(interval_us);
                while Instant::now() < deadline {
                    std::hint::spin_loop();
                }
            }
        })
    };

    // ---- STOPPER thread ----
    // Poll the shared Vec len; stop the live loop once `total` samples land OR a
    // generous wall timeout trips (so a stall can never hang the suite).
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
                        "streaming bench: wall timeout (30s) before {total} samples — \
                         stopping the live loop with what we have"
                    );
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            running.store(false, Ordering::Relaxed);
        })
    };

    // ---- MAIN thread: drive the LIVE WaitSet reactor until stopped. ----
    // This is the production live path: it spins-then-blocks,
    // reading CERULION_LIVE_SPIN_US itself. It returns when `running` clears.
    runtime
        .run_live(&running)
        .expect("run_live must not be refused (no host-driven external nodes in this bench)");

    pub_handle.join().expect("publisher thread panicked");
    stop_handle.join().expect("stopper thread panicked");
    runtime.shutdown();

    // ---- Compute + report the three per-hop latencies. ----
    let all = samples.lock().unwrap().clone();
    assert!(
        all.len() > warmup,
        "bench collected only {} samples (<= warmup {warmup}) — data did NOT flow \
         through the ext→relay→sink live graph (check the external publisher / \
         WaitSet wiring)",
        all.len()
    );

    // Drop the warmup prefix; split into per-hop columns.
    let measured = &all[warmup..];
    let hop1: Vec<f64> = measured.iter().map(|&(h1, _, _)| h1).collect();
    let hop2: Vec<f64> = measured.iter().map(|&(_, h2, _)| h2).collect();
    let e2e: Vec<f64> = measured.iter().map(|&(_, _, e)| e).collect();

    println!(
        "--- per-hop latency over {} measured samples (warmup {warmup} dropped) ---\n\
         hop1 = ext→relay (cross-step external wake; the spin helps HERE)\n\
         hop2 = relay→sink (internal/post-relay; flat regardless of spin)\n\
         e2e  = ext→relay→sink (full path)",
        measured.len()
    );
    let (hop1_floor, _hop1_p50) = report_hop("hop1", hop1);
    let (hop2_floor, _hop2_p50) = report_hop("hop2", hop2);
    let (e2e_floor, _e2e_p50) = report_hop("e2e", e2e);

    // ---- LIVENESS-ONLY asserts (no absolute-µs gate — box-specific). ----
    assert!(
        measured.len() >= target,
        "expected >= {target} measured samples, got {} — the live graph under-delivered",
        measured.len()
    );
    for (label, f) in [
        ("hop1", hop1_floor),
        ("hop2", hop2_floor),
        ("e2e", e2e_floor),
    ] {
        assert!(
            f.is_finite() && f > 0.0,
            "[{label}] floor latency must be finite and > 0, got {f} ns — a clock/stamp bug",
        );
    }
}
