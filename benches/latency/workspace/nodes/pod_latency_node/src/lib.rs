//! Sink node — the FIXED-POD type-class twin of `latency_node`
//! (the type-class axis). See `nodes/pod_ping_node/src/lib.rs`
//! for the class rationale + the baked-size mislabel guard, and
//! `nodes/latency_node/src/lib.rs` for the role, timestamp mechanism,
//! sample-count contract (G1: the dump holds EXACTLY
//! CER_BENCH_TARGET_SAMPLES samples), and the split-leg shutdown
//! behavior — all of which apply here unchanged. The only difference
//! is the input schema (`PodPayload`, stamp in `stamp_hi/stamp_lo`)
//! and the raw-dump inlining note (same reason: separate workspace).

use cerulion_core::prelude::*;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use cer_bench_msgs::{PodPayload, PODPAYLOAD_WIRE_SIZE};

#[cerulion_node]
pub struct PodLatencyNode {
    #[input(trigger)]
    echo_in: PodPayload,

    samples: Vec<u64>,
    /// MEASURED sample count (`CER_BENCH_TARGET_SAMPLES` — G1).
    target_samples: usize,
    warmup: usize,
    payload_size: usize,
    shutdown_requested: bool,
    /// Delivery accounting — see latency_node.
    received_total: u64,
    skipped: u64,
    leg: String,
    /// Set on the first `tick`; the receipt prints only from an instance
    /// that ticked. See ping_node's `ticked`: the supervisor's planning
    /// build runs init + shutdown on every node without ticking it.
    ticked: bool,
}

/// The sweep-point size, read so that a PRESENT but unparseable value is a
/// refusal rather than a silent 64.
///
/// `ctx.env` warns and returns the default, and the default here is 64 —
/// which is also a real sweep point. So `CER_BENCH_PAYLOAD_SIZE=1e3`
/// against a 64-baked build would pass the mislabel guard below and
/// measure 64 bytes under whatever name the runner gave the `.bin`. The
/// guard exists to make exactly that impossible, so it must not have a
/// door through its own input. An ABSENT var still means 64 (the pinned
/// sweep's first size, and what the build script bakes unset).
fn payload_size_from_env(node: &str, ctx: &NodeContext) -> Result<usize, NodeError> {
    // A sentinel no environment value can equal (NUL is not permitted in
    // an env string), so "absent" and "present but empty" are told apart.
    const UNSET: &str = "\u{0}unset";
    let raw = ctx.env_str("CER_BENCH_PAYLOAD_SIZE", UNSET);
    if raw == UNSET {
        return Ok(64);
    }
    raw.parse::<usize>().map_err(|_| NodeError::InvalidInput {
        input: "CER_BENCH_PAYLOAD_SIZE".to_string(),
        reason: format!(
            "{node}: {raw:?} is not a byte count — refusing rather than \
             falling back to 64, which is itself a sweep point and would \
             pass the baked-size guard on a 64-baked build"
        ),
    })
}

#[cerulion_node_impl]
impl PodLatencyNode {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.target_samples = _ctx.env("CER_BENCH_TARGET_SAMPLES", 10_000);
        self.warmup = _ctx.env("CER_BENCH_WARMUP", 1_000);
        self.payload_size = payload_size_from_env("pod_latency_node", _ctx)?;
        // Baked-size mislabel guard — see pod_ping_node.
        if PODPAYLOAD_WIRE_SIZE != self.payload_size {
            return Err(NodeError::Logic(format!(
                "pod_latency_node: baked PodPayload wire size is {PODPAYLOAD_WIRE_SIZE} B \
                 but CER_BENCH_PAYLOAD_SIZE={} — stale pod cdylib build; \
                 run_workspace.sh rebuilds the pod crates with \
                 CER_BENCH_POD_BYTES=<size> per sweep size",
                self.payload_size
            )));
        }
        self.leg = _ctx.env_str("CER_BENCH_LEG", "unknown");
        // Bound the budget before reserving, exactly as the incumbent
        // latency_node and the ROS 2 nodes' common.hpp::check_sample_budget
        // do. `ctx.env` accepts any usize, so a fat-fingered pair either
        // WRAPS `target + warmup` — in release the sum becomes a SMALL
        // number, so the buffer is under-reserved and the node stops
        // early, reporting a full measurement it never took (in debug the
        // add panics instead) — or asks for an absurd allocation. 100e6
        // samples is 800 MB of u64, four orders of magnitude above the
        // widest pinned schedule. An init Err aborts the graph build
        // (`entry.init(context)?`), so this refuses the run rather than
        // degrading it.
        const MAX_TOTAL_SAMPLES: usize = 100_000_000;
        let total = self
            .target_samples
            .checked_add(self.warmup)
            .filter(|t| *t <= MAX_TOTAL_SAMPLES)
            .ok_or_else(|| NodeError::InvalidInput {
                input: "CER_BENCH_TARGET_SAMPLES + CER_BENCH_WARMUP".to_string(),
                reason: format!(
                    "pod_latency_node: {} + {} exceeds the \
                     {MAX_TOTAL_SAMPLES}-sample ceiling (or overflows usize) \
                     — refusing to run a measurement whose capacity and \
                     completion threshold would wrap",
                    self.target_samples, self.warmup
                ),
            })?;
        // Pre-allocate so the hot path never reallocates.
        self.samples = Vec::with_capacity(total);
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        self.received_total += 1;
        let hi = self.echo_in.stamp_hi as u64;
        let lo = self.echo_in.stamp_lo as u64;
        let send_ns = (hi << 32) | lo;

        let now_ns = self.real_ns();
        let rtt = now_ns.saturating_sub(send_ns);
        if send_ns == 0 || rtt == 0 {
            self.skipped += 1;
            return Ok(());
        }

        // Cap accumulation at the window (G1 equality gate — see
        // latency_node).
        let total_needed = self.target_samples + self.warmup;
        if self.samples.len() < total_needed {
            self.samples.push(rtt);
        }
        if !self.shutdown_requested && self.samples.len() >= total_needed {
            self.shutdown_requested = true;
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
        // Delivery accounting FIRST (printed even on a killed run).
        println!(
            "RTT_DELIVERY role=latency leg={} received={} skipped={} measured={}",
            self.leg,
            self.received_total,
            self.skipped,
            self.samples.len(),
        );
        if self.samples.len() <= self.warmup {
            eprintln!(
                "pod_latency_node: payload={} only {} samples (<= warmup {}) — NOT dumping a partial .bin",
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
            "pod_latency_node: payload={} dumped {} samples",
            self.payload_size, n,
        );
        let _ = std::io::stderr().flush();
        Ok(())
    }
}

/// Write raw round-trip samples (LE u64 ns) to
/// `${CER_BENCH_RAW_DUMP_DIR}/${CER_BENCH_RAW_NAME}_${payload_size}.bin`.
/// Inlined here for the same reason as latency_node's copy (the cdylib
/// doesn't link the native crate's lib; missing env is loud-but-nonfatal
/// — the runner's data-flow gate turns the missing .bin into a hard
/// failure).
fn dump_raw_samples(samples: &[u64], payload_size: usize) {
    let Ok(dir) = std::env::var("CER_BENCH_RAW_DUMP_DIR") else {
        eprintln!(
            "dump_raw_samples: CER_BENCH_RAW_DUMP_DIR unset — NOT dumping \
             (the runner's .bin gate will fail this size loudly)"
        );
        return;
    };
    let Ok(name) = std::env::var("CER_BENCH_RAW_NAME") else {
        eprintln!(
            "dump_raw_samples: CER_BENCH_RAW_NAME unset — NOT dumping \
             (the runner's .bin gate will fail this size loudly)"
        );
        return;
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
