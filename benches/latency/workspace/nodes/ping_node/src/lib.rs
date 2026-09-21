//! Ping initiator for the graph-runtime round-trip latency benchmark
//! (benches/latency workspace line — a port of
//! benches/cerulion_round_trip_quiescent/graph_rtt_bench).
//!
//! Wired into `graphs/rtt_bench.yaml` as the periodic source. Per
//! tick, stamps a wall-clock nanosecond timestamp into the outbound
//! `Image.height` / `Image.width` pair (split across two u32 fields to
//! preserve full ns precision) and loans `payload_size` bytes for the
//! variable `data` field — Mode-A fill-exclusion: the payload bytes are
//! LOANED, never written, so no fill cost sits in the timed window.
//!
//! **G3 stamp-late discipline (audit finding):** the stamp is read and
//! written as the LAST port writes of the tick — every other port write
//! (including the lazy SHM loan the FIRST port write triggers,
//! and the variable-field `loan_data`) happens BEFORE `real_ns()` is
//! read, so the only work between the stamp and the Drop-commit publish
//! is the two u32 stamp-field writes. This matches the ROS2 ping
//! (`ping_node.cpp`: stamp after borrow/prealloc, only the 8-byte write
//! between stamp and publish), so the two stacks' RTT windows measure
//! the same event span — publisher commit → subscriber read — instead
//! of the Cerulion line paying loan/setup inside its window.
//!
//! ## Timestamp mechanism (WALL, UNCONDITIONAL — read before measuring)
//!
//! This node stamps **`self.real_ns()` unconditionally** — the macro shim's
//! explicit-source escape hatch that ALWAYS reads the kernel monotonic
//! clock (`CLOCK_MONOTONIC` on Linux, `CLOCK_UPTIME_RAW` on macOS),
//! independent of the runtime's clock injection. Two consequences:
//!
//! 1. Wall latency samples ARE collected on every leg with no env gate.
//!    Unlike nodes built as record and replay assets (which stamp the deterministic
//!    gating clock by default and only stamp wall time under
//!    `CER_BENCH_WALL_STAMP=1`), these bench-only fixtures do NOT consult
//!    `CER_BENCH_WALL_STAMP` — there is no record/replay use of this
//!    workspace, so the deterministic-stamp mode has nothing to protect.
//!    run_workspace.sh still exports `CER_BENCH_WALL_STAMP=1` for env-
//!    contract parity across the suite (a no-op here, load-bearing if these
//!    nodes ever adopt the robotics gating).
//! 2. The stamp is valid CROSS-PROCESS (the split leg): `CLOCK_MONOTONIC`
//!    is one system-wide clock, so a stamp taken in the ping worker is
//!    directly comparable in the latency worker's process.
//!
//! If the bench stamped the ACTIVE clock (`self.now_ns()`) instead, ping
//! (publish) and latency (receive) would read identical values within a
//! single scheduler step under `VirtualClock` — every RTT would be zero.
//! `real_ns()` matches the native `raw_iceoryx2_round_trip` floor bench's
//! clock source so the lines stay comparable.
//!
//! ## Rate knob — the WALL grid gate is the pacing authority
//!
//! The compile-time `#[cerulion_node(period_ms = N)]` attr below is the
//! TICK SOURCE only (graph YAML carries no policy override;
//! run_workspace.sh sed-rewrites it per payload size and rebuilds this
//! cdylib before each size's run). It is NOT sufficient as the rate
//! authority: on the SPLIT leg the graph runs multi-process on the
//! handed-quantum barrier clock, where `period_ms` is LOGICAL
//! time — the cohort steps as fast as the barrier turns, so the ping
//! "period" fires at whatever wall rate the hardware yields (MEASURED
//! 2026-08-15 on box-x86: ~920 Hz wall while the run was labeled 100 Hz —
//! every ungated split-leg rate label, fixed100 AND quiescent, is
//! false; the mono leg's wall-clock live loop makes the same label
//! true by accident of clock model).
//!
//! So the publish is gated on a WALL-clock slot grid: when
//! `CER_BENCH_TARGET_RATE_HZ` > 0 (exported by run_workspace.sh on every
//! quiescent/fixed100 size; 0 under backtoback = gate off), a tick whose
//! `real_ns()` has not reached the next grid slot publishes NOTHING (no
//! port write ⇒ no loan ⇒ no publish — the lazy-loan non-event),
//! and a tick that arrives late consumes exactly one slot, SKIPPING
//! missed slots rather than bursting (`slots_skipped`, printed in the
//! RTT_DELIVERY line — the same grid-slot semantics as the native bins'
//! `RateLimiter`). run_workspace.sh's fixed100 sustain verdict fails a
//! rung whose skip count exceeds 1% of its slots, mirroring the native
//! cannot-sustain observable — without it a barrier turning slower than
//! the grid would under-pace silently and re-mint the false label.
//!
//! Bench-only non-determinism (`real_ns()`, `ctx.env*` in init, `println!`
//! in shutdown) is the framework's documented escape-hatch set for latency
//! benches — wrong for production nodes.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 1)]
pub struct PingNode {
    #[output]
    ping_out: Image,

    payload_size: usize,
    /// Delivery accounting: frames this node WROTE COMPLETELY and
    /// handed to the output proxy — an upper bound on frames committed,
    /// not the commit count itself. The generated proxy publishes when it
    /// drops after `tick` returns, and that send can fail (the host logs
    /// every such path loudly, prefixed `OutputProxy`, and counts it in
    /// `output_discard_count`). Two things never increment it: a tick that
    /// fails BEFORE the counter (`loan_data` returning Err), and a
    /// wall-gated tick that skips its publish entirely (nothing committed).
    ///
    /// run_workspace.sh's delivery-accounting block greps the run log for
    /// those `OutputProxy` lines, so a failed commit shows up as a failed
    /// commit instead of looking like downstream coalescing loss.
    published: u64,
    leg: String,
    /// Wall-grid publish gate (see the module header): target rate from
    /// CER_BENCH_TARGET_RATE_HZ (0 = ungated, the backtoback shape).
    rate_hz: u64,
    period_ns: u64,
    next_slot_ns: u64,
    /// Grid slots the graph failed to reach in time (skip-forward, never
    /// burst) — the workspace twin of the native RateLimiter's
    /// slots_skipped cannot-sustain observable.
    slots_skipped: u64,
    /// Set on the first `tick`. The receipt in `shutdown` prints only when
    /// this is set. `cerulion graph run` on a `process_groups:` graph builds
    /// the FULL graph once inside the supervisor to plan the split (the
    /// planning build in `graph_run_supervisor`), which runs `init` and then
    /// `shutdown` on every node without ever ticking it. That instance's
    /// all-zero receipt landed in the same run log as the worker's real one,
    /// and run_workspace.sh's `delivery_count` reads two differing values
    /// for one role as a contradiction, so every split row was refused.
    ticked: bool,
}

#[cerulion_node_impl]
impl PingNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        // Leg tag for the delivery-accounting line (mono|split — set by
        // run_workspace.sh; workers inherit the supervisor's environment).
        self.leg = ctx.env_str("CER_BENCH_LEG", "unknown");
        // Wall-grid publish gate (module header): the rate label's
        // authority. 0 (backtoback / driven outside the runner) = ungated.
        // NOT `ctx.env::<u64>(..)`: that WARNS and returns the default on
        // an unparseable value, and the default here is 0 — which is the
        // UNGATED sentinel. So `CER_BENCH_TARGET_RATE_HZ=1e9` (or `100.0`,
        // or a value wider than u64) silently disabled the publish gate
        // while the runner, the `.rate` sidecar, the CSV and every plot
        // carried the requested rate as the label: exactly the defect the
        // wall-grid gate exists to remove, entered through the knob that
        // names it. Read the raw string and parse it here so a PRESENT but
        // unparseable value is a refusal; only an ABSENT var may mean 0.
        // The other four pacers already refuse it (`env_size` exits 2, the
        // native `RateLimiter::from_env` expects the parse); this node was
        // the one that defaulted.
        const RATE_ENV: &str = "CER_BENCH_TARGET_RATE_HZ";
        // A sentinel no environment value can equal (NUL is not permitted
        // in an env string), so "absent" and "present but empty" are told
        // apart: an exported-empty value is a typo, not a request for 0.
        const UNSET: &str = "\u{0}unset";
        let raw = ctx.env_str(RATE_ENV, UNSET);
        self.rate_hz = if raw == UNSET {
            0
        } else {
            raw.parse::<u64>().map_err(|_| NodeError::InvalidInput {
                input: RATE_ENV.to_string(),
                reason: format!(
                    "{raw:?} is not a non-negative integer — refusing rather \
                     than falling back to 0, which is the UNGATED value and \
                     would free-run this leg under a paced label"
                ),
            })?
        };
        // A rate above 1 GHz floors `1e9 / rate_hz` to a ZERO nanosecond
        // period, and the tick gate below is `if self.period_ns > 0`, so the
        // same free-run-under-a-paced-label follows. An init Err aborts the
        // graph build (`entry.init(context)?`), so this refuses the run
        // rather than measuring an ungated one under a paced name. 0 stays
        // the deliberate ungated (backtoback) value. Same ceiling as the
        // native `RateLimiter` and the three ROS 2 nodes.
        const MAX_RATE_HZ: u64 = 1_000_000_000;
        if self.rate_hz > MAX_RATE_HZ {
            return Err(NodeError::InvalidInput {
                input: RATE_ENV.to_string(),
                reason: format!(
                    "{} Hz exceeds the {MAX_RATE_HZ} Hz ceiling — the wall \
                     period 1e9/rate floors to 0 ns, which disables the \
                     publish gate entirely while the run keeps the \
                     requested rate as its label",
                    self.rate_hz
                ),
            });
        }
        // checked_div, not a `> 0` guard around a bare `/`: identical
        // behaviour (None only when the divisor is 0, which is the
        // ungated case that must leave period_ns at 0), and clippy 1.96's
        // manual_checked_ops rejects the guarded form.
        if let Some(period) = 1_000_000_000u64.checked_div(self.rate_hz) {
            self.period_ns = period;
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        // Wall-grid gate FIRST, before ANY port write: a tick before its
        // slot publishes nothing (no port write ⇒ no loan ⇒ no publish —
        // the lazy-loan non-event). This read of real_ns() is
        // OUTSIDE the stamp→publish span (the stamp is re-read below), so
        // the G3 stamp-late discipline is untouched.
        if self.period_ns > 0 {
            let now = self.real_ns();
            if self.next_slot_ns == 0 {
                // Anchor the grid at the first fire.
                self.next_slot_ns = now;
            }
            if now < self.next_slot_ns {
                return Ok(());
            }
            // Consume exactly this slot; if the graph failed to reach one
            // or more slots in time, SKIP them (never burst) and count —
            // the cannot-sustain observable the runner's fixed100 verdict
            // reads off the RTT_DELIVERY line.
            self.next_slot_ns += self.period_ns;
            if self.next_slot_ns <= now {
                let missed = (now - self.next_slot_ns) / self.period_ns + 1;
                self.slots_skipped += missed;
                self.next_slot_ns += missed * self.period_ns;
            }
        }
        // G3: all non-stamp work first — the first port write below
        // triggers the lazy SHM loan, so the loan + prep cost sits
        // OUTSIDE the stamp→publish span. Fixed-section writes are
        // position-independent, so stamping height/width last is safe.
        self.ping_out.step = 0;
        self.ping_out.is_bigendian = 0;
        self.ping_out.set_header_bytes(&[])?;
        self.ping_out.set_encoding("rt")?;
        let _ = self.ping_out.loan_data(self.payload_size)?;
        self.published += 1;
        // Stamp LAST: only these two u32 writes stand between the
        // real_ns() read and the tick-return Drop-commit publish.
        let t = self.real_ns();
        self.ping_out.height = (t >> 32) as u32;
        self.ping_out.width = (t & 0xFFFF_FFFF) as u32;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.ticked {
            // A planning instance (init + shutdown, never ticked) has nothing
            // to account for; see `ticked`. The runner's gate still fails
            // closed when the live instance prints nothing (MISSING).
            return Ok(());
        }
        // Machine-parsable delivery-accounting line (the mp_latency
        // convention): run_workspace.sh greps `RTT_DELIVERY` from the run
        // log per leg. In the split leg this prints from the PING worker
        // process; worker stdout is the supervisor's, so it reaches the
        // shared log. The NO-SILENT-LOSS check is published vs the latency
        // node's received (modulo data-trigger latest-wins coalescing) plus
        // the host's per-input `drop_oldest` telemetry lines.
        println!(
            "RTT_DELIVERY role=ping leg={} published={} slots_skipped={}",
            self.leg, self.published, self.slots_skipped
        );
        Ok(())
    }
}
