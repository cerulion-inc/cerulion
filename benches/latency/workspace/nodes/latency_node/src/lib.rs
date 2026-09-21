//! Sink node for the graph-runtime round-trip bench (benches/latency
//! workspace line — a port of
//! benches/cerulion_round_trip_quiescent/graph_rtt_bench).
//!
//! Reads the embedded send-timestamp from `echo_in.height/width`, computes
//! round-trip latency against the wall clock, and accumulates samples on
//! the node struct itself. `target_samples` (`CER_BENCH_TARGET_SAMPLES`) is
//! the MEASURED count — the suite-wide G1 contract: the runner passes
//! total − warmup. Once `target_samples + warmup` (= the schedule TOTAL)
//! samples are collected, accumulation stops (frames landing between the
//! shutdown request and the runtime honoring it are counted as received
//! but never pushed) and it calls `self.request_shutdown()`; the runtime
//! exits cleanly and `shutdown()` drops the warmup prefix and dumps
//! EXACTLY `target_samples` raw samples to disk (run_workspace.sh gates on
//! that equality). All percentile / ECDF analysis happens post-hoc via
//! `compile_csv.py` — this node never computes a percentile.
//!
//! ## Timestamp mechanism (WALL, UNCONDITIONAL)
//!
//! `self.real_ns()` — the macro shim's explicit-source escape hatch, always
//! kernel-monotonic wall time regardless of the runtime's clock injection —
//! on BOTH ends (ping stamps, this node reads), sharing one system-wide
//! clock that is valid across processes (the split leg runs this node in a
//! different process from ping). `CER_BENCH_WALL_STAMP` is NOT consulted:
//! stamps here are unconditional wall time (see `nodes/ping_node/src/lib.rs`
//! for the full note). If this node read the ACTIVE clock instead, RTT
//! would be zero within a scheduler step under `VirtualClock` — which is
//! exactly the backtoback (`--time-source virtual`) mono leg, so the wall
//! escape hatch is what makes that leg measurable at all.
//!
//! In the SPLIT leg this node runs in a WORKER process: a clean
//! `request_shutdown()` exit there is treated by the supervisor as graph
//! shutdown (it SIGINTs the remaining workers to self-drop and exit 0), so
//! the split leg self-terminates exactly like the mono leg.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

#[cerulion_node]
pub struct LatencyNode {
    #[input(trigger)]
    echo_in: Image,

    samples: Vec<u64>,
    /// MEASURED sample count (`CER_BENCH_TARGET_SAMPLES` — suite-wide G1
    /// contract: the runner passes total − warmup). The dump holds exactly
    /// this many samples.
    target_samples: usize,
    warmup: usize,
    payload_size: usize,
    shutdown_requested: bool,
    /// Delivery accounting: EVERY delivered frame observed by tick
    /// (including guard-skipped ones) — the `received=` half against ping's
    /// `published=` line.
    received_total: u64,
    /// Frames skipped by the zero-stamp / zero-rtt guard (pre-first-stamp
    /// default frames) — reported so `received` separates measured from
    /// merely delivered.
    skipped: u64,
    leg: String,
    /// Set on the first `tick`; the receipt prints only from an instance
    /// that ticked. See ping_node's `ticked`: the supervisor's planning
    /// build runs init + shutdown on every node without ticking it.
    ticked: bool,
}

#[cerulion_node_impl]
impl LatencyNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.target_samples = _ctx.env("CER_BENCH_TARGET_SAMPLES", 10_000);
        self.warmup = _ctx.env("CER_BENCH_WARMUP", 1_000);
        self.payload_size = _ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        self.leg = _ctx.env_str("CER_BENCH_LEG", "unknown");
        // Bound the budget before reserving. `ctx.env` accepts any usize,
        // so a fat-fingered pair either wraps `target + warmup` (a tiny
        // capacity and a completion threshold this node can never reach —
        // it collects forever) or asks for an absurd allocation. Same
        // rule and same ceiling as the ROS 2 nodes'
        // common.hpp::check_sample_budget, so the two stacks refuse the
        // same inputs; 100e6 samples is 800 MB of u64, four orders of
        // magnitude above the widest pinned schedule. An init Err aborts
        // the graph build (`entry.init(context)?`), so this refuses the
        // run rather than degrading it.
        const MAX_TOTAL_SAMPLES: usize = 100_000_000;
        let total = self
            .target_samples
            .checked_add(self.warmup)
            .filter(|t| *t <= MAX_TOTAL_SAMPLES)
            .ok_or_else(|| NodeError::InvalidInput {
                input: "CER_BENCH_TARGET_SAMPLES + CER_BENCH_WARMUP".to_string(),
                reason: format!(
                    "{} + {} exceeds the {MAX_TOTAL_SAMPLES}-sample ceiling \
                     (or overflows usize) — refusing to run a measurement \
                     whose capacity and completion threshold would wrap",
                    self.target_samples, self.warmup
                ),
            })?;
        // Pre-allocate the sample buffer so the hot path never reallocates.
        self.samples = Vec::with_capacity(total);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        self.received_total += 1;
        let h = self.echo_in.height as u64;
        let w = self.echo_in.width as u64;
        let send_ns = (h << 32) | w;

        let now_ns = self.real_ns();
        let rtt = now_ns.saturating_sub(send_ns);
        if send_ns == 0 || rtt == 0 {
            self.skipped += 1;
            return Ok(());
        }

        // Cap accumulation at the window: ticks can keep landing between
        // request_shutdown() below and the runtime honoring it, and the
        // dump must hold EXACTLY `target_samples` (= MEASURED) samples —
        // run_workspace.sh's sample-count gate is an equality check.
        let total_needed = self.target_samples + self.warmup;
        if self.samples.len() < total_needed {
            self.samples.push(rtt);
        }
        if !self.shutdown_requested && self.samples.len() >= total_needed {
            self.shutdown_requested = true;
            // The runtime checks `shutdown_requested()` between ticks
            // (monolith poll loop AND run_live's worker loop) and exits.
            self.request_shutdown();
        }
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.ticked {
            // A planning instance (init + shutdown, never ticked) has nothing
            // to account for and nothing to dump; see ping_node's `ticked`.
            // Returning here also silences the "NOT dumping a partial .bin"
            // complaint that instance used to print. The runner's gates
            // still fail closed when the live instance prints nothing.
            return Ok(());
        }
        // Delivery-accounting line FIRST (the mp_latency convention),
        // printed even on a killed/under-sampled run — run_workspace.sh
        // greps `RTT_DELIVERY` per leg. `measured` = samples actually
        // accumulated (warmup included at this point). received >= measured
        // + skipped is a tautology; published (ping) vs received (here) is
        // the loss surface, with latest-wins coalescing the benign
        // explanation and the host's `drop_oldest` telemetry the eviction
        // counter.
        println!(
            "RTT_DELIVERY role=latency leg={} received={} skipped={} measured={}",
            self.leg,
            self.received_total,
            self.skipped,
            self.samples.len(),
        );
        if self.samples.len() <= self.warmup {
            // Bench was killed before enough samples accumulated — skip the
            // dump rather than emit a misleading partial file, and say so
            // LOUDLY (the runner's data-flow gate then fails on the missing
            // .bin).
            eprintln!(
                "latency_node: payload={} only {} samples (<= warmup {}) — NOT dumping a partial .bin",
                self.payload_size,
                self.samples.len(),
                self.warmup,
            );
            return Ok(());
        }
        self.samples.drain(..self.warmup);
        let n = self.samples.len();
        dump_raw_samples(&self.samples, self.payload_size);
        eprintln!(
            "latency_node: payload={} dumped {} samples",
            self.payload_size, n,
        );
        let _ = std::io::stderr().flush();
        Ok(())
    }
}

/// Write raw round-trip samples (LE u64 ns) to
/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${payload_size}.bin`.
/// Inlined here rather than imported because the cdylib doesn't link
/// against the `benches/latency/native` crate's lib (separate workspace).
/// Missing env vars are reported LOUDLY but don't panic — the cdylib runs
/// inside the runtime and must not bring it down; the runner's data-flow
/// gate turns the missing .bin into a hard failure.
fn dump_raw_samples(samples: &[u64], payload_size: usize) {
    // `var` folds a present-but-non-UTF-8 value into the same Err as
    // absence. Refusing to dump is right either way (fail-closed — the
    // runner's .bin gate then fails the size loudly), but saying "unset"
    // when the value is merely undecodable sends the operator looking for
    // a missing export that is right there.
    let dir = match std::env::var("CER_BENCH_RAW_DUMP_DIR") {
        Ok(v) => v,
        Err(e) => {
            eprintln!(
                "dump_raw_samples: CER_BENCH_RAW_DUMP_DIR {e} — NOT dumping \
                 (the runner's .bin gate will fail this size loudly)"
            );
            return;
        }
    };
    // `var` folds a present-but-non-UTF-8 value into the same Err as
    // absence. Refusing to dump is right either way (fail-closed — the
    // runner's .bin gate then fails the size loudly), but saying "unset"
    // when the value is merely undecodable sends the operator looking for
    // a missing export that is right there.
    let name = match std::env::var("CER_BENCH_RAW_NAME") {
        Ok(v) => v,
        Err(e) => {
            eprintln!(
                "dump_raw_samples: CER_BENCH_RAW_NAME {e} — NOT dumping \
                 (the runner's .bin gate will fail this size loudly)"
            );
            return;
        }
    };
    let path = PathBuf::from(dir).join(format!("{name}_{payload_size}.bin"));
    let mut buf = Vec::with_capacity(samples.len() * 8);
    for &s in samples {
        buf.extend_from_slice(&s.to_le_bytes());
    }
    let mut f = match File::create(&path) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("dump_raw_samples: create {}: {e}", path.display());
            return;
        }
    };
    if let Err(e) = f.write_all(&buf) {
        eprintln!("dump_raw_samples: write {}: {e}", path.display());
    }
}
