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
//! authority: under the `CERULION_EXECUTION_MODE=lockstep` opt-out the
//! SPLIT leg runs multi-process on the handed-quantum barrier clock,
//! where `period_ms` is LOGICAL time (the cohort steps as fast as the
//! barrier turns), so the ping "period" fires at whatever wall rate the
//! hardware yields: MEASURED 2026-08-15 on the bench machine in that
//! mode, ~920 Hz wall while the run was labeled 100 Hz, so every ungated
//! split-leg rate label, fixed100 AND quiescent, is false there. Under
//! the free-run default the rank's gating clock follows the wall, which
//! puts the split leg where the mono leg's wall-clock live loop already
//! is. The gate is the rate authority in BOTH modes, so the label never
//! depends on which one a run took.
//!
//! So the publish is gated on a WALL-clock slot grid: when
//! `CER_BENCH_TARGET_RATE_HZ` > 0 (exported by run_workspace.sh on every
//! quiescent/fixed100 size; 0 under backtoback = gate off), each tick is
//! assigned to its NEAREST slot on a grid anchored at the first tick
//! (`grid_slot`). A tick whose slot is already filled publishes NOTHING
//! (no port write ⇒ no loan ⇒ no publish — the lazy-loan non-event), and
//! a tick that lands one or more slots past the last one filled publishes
//! once, SKIPPING the slots in between rather than bursting
//! (`slots_skipped`, printed in the RTT_DELIVERY line — the workspace
//! counterpart of the native bins' `RateLimiter` skip count). Nearest rather
//! than at-or-before, because the tick source runs at the grid's own period
//! and wakes with jitter: see `grid_slot`. run_workspace.sh's fixed100
//! sustain verdict fails a
//! rung whose skip count exceeds 1% of its slots, mirroring the native
//! cannot-sustain observable: without it a run whose steps turn slower
//! than the grid would under-pace silently and re-mint the false label.
//!
//! Bench-only non-determinism (`real_ns()`, `ctx.env*` in init, `println!`
//! in shutdown) is the framework's documented escape-hatch set for latency
//! benches — wrong for production nodes.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

/// Wall-grid slot decision for one tick (pure, so it is unit tested).
///
/// `anchor_ns` is the first tick's wall time (slot 0) and `last_slot` the
/// slot the previous publish filled. The tick is assigned to its NEAREST
/// grid slot, `round((now - anchor) / period)`. It publishes when that slot
/// is later than `last_slot`, and every slot between the two is counted as
/// skipped. Returns `None` for a tick that publishes nothing, or
/// `Some((slot, skipped))`.
///
/// Nearest, not floor. Under fixed100 the tick source is the runtime's own
/// `Period` trigger at the grid's period (10 ms for 100 Hz), so tick and slot
/// differ only in phase, and each tick wakes late by a varying amount. With
/// a floor (`now < anchor + k * period` publishes nothing) the first tick's
/// wake lateness became the grid's phase: a later tick that woke earlier
/// than the first one published nothing, and the next tick then found its
/// slot passed and counted it as skipped. Wake jitter of tens of
/// microseconds therefore counted as missed slots, enough for a run to
/// cross the runner's 1% sustain line and be re-run at the next ladder
/// rung. Nearest-slot assignment counts a tick
/// against the grid only when it lands more than half a period from the
/// first tick's phase.
///
/// Where the rate does not divide 1000 the tick period is the integer
/// `period_ms` below the grid period, so some ticks find their slot filled
/// and publish nothing; no slot is skipped. With a tick source much faster
/// than the grid (the lockstep opt-out's logical period) the first publish
/// after the anchor comes half a period early and every later one a period
/// apart; the warmup samples absorb it.
pub(crate) fn grid_slot(
    anchor_ns: u64,
    last_slot: u64,
    now_ns: u64,
    period_ns: u64,
) -> Option<(u64, u64)> {
    let slot = (now_ns.saturating_sub(anchor_ns) + period_ns / 2) / period_ns;
    if slot <= last_slot {
        return None;
    }
    Some((slot, slot - last_slot - 1))
}

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
    /// Wall time of the first tick (grid slot 0); 0 until it arrives.
    anchor_ns: u64,
    /// The grid slot the last publish filled (see [`grid_slot`]).
    last_slot: u64,
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
        // Wall-grid gate FIRST, before ANY port write: a tick whose slot is
        // already filled publishes nothing (no port write ⇒ no loan ⇒ no publish —
        // the lazy-loan non-event). This read of real_ns() is
        // OUTSIDE the stamp→publish span (the stamp is re-read below), so
        // the G3 stamp-late discipline is untouched.
        if self.period_ns > 0 {
            let now = self.real_ns();
            if self.anchor_ns == 0 {
                // Anchor the grid at the first fire, which publishes.
                self.anchor_ns = now;
            } else {
                // A tick nearer the slot already filled than the next one
                // publishes nothing; every slot it jumped past is counted
                // as skipped (never burst). See `grid_slot`.
                match grid_slot(self.anchor_ns, self.last_slot, now, self.period_ns) {
                    None => return Ok(()),
                    Some((slot, skipped)) => {
                        self.last_slot = slot;
                        self.slots_skipped += skipped;
                    }
                }
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

#[cfg(test)]
mod grid_slot_tests {
    use super::grid_slot;

    const P: u64 = 10_000_000; // 100 Hz

    /// A tick that wakes earlier than the anchor tick did still fills its
    /// slot: the case the floor grid dropped and later counted as skipped.
    #[test]
    fn a_tick_waking_earlier_than_the_anchor_fills_its_slot() {
        let anchor = 1_000_000_000 + 300_000; // first tick woke 300 us late
        let now = 1_000_000_000 + P + 40_000; // second tick woke 40 us late
        assert_eq!(grid_slot(anchor, 0, now, P), Some((1, 0)));
    }

    /// Ticks on the runtime's period, each with its own wake lateness
    /// below half a period, publish once per slot and skip none.
    #[test]
    fn jittered_ticks_on_the_period_skip_no_slot() {
        let epoch = 5_000_000_000u64;
        let lateness = [300_000u64, 10_000, 4_000_000, 0, 250_000, 4_900_000, 1];
        let anchor = epoch + lateness[0];
        let mut last = 0;
        for (k, j) in lateness.iter().enumerate().skip(1) {
            let now = epoch + k as u64 * P + j;
            let (slot, skipped) = grid_slot(anchor, last, now, P).expect("publishes");
            assert_eq!((slot, skipped), (k as u64, 0), "tick {k}");
            last = slot;
        }
    }

    /// A stall longer than one and a half periods still counts the slots it
    /// missed, and the catch-up ticks the runtime fires in the same step
    /// publish nothing (no burst).
    #[test]
    fn a_stall_counts_its_missed_slots_and_never_bursts() {
        let anchor = 0;
        let now = 4 * P + 100_000; // ticks for slots 1..=3 never came
        assert_eq!(grid_slot(anchor, 0, now, P), Some((4, 3)));
        assert_eq!(grid_slot(anchor, 4, now + 1_000, P), None);
    }

    /// A tick source slower than the grid (here 2% slow) still counts its
    /// deficit: about one skipped slot per fifty ticks, 2% of the window,
    /// which is over the runner's 1% sustain line.
    #[test]
    fn a_slow_tick_source_counts_its_deficit() {
        let tick = P + P / 50;
        let mut last = 0;
        let mut skipped_total = 0;
        let ticks = 2_000u64;
        for k in 1..=ticks {
            if let Some((slot, skipped)) = grid_slot(0, last, k * tick, P) {
                last = slot;
                skipped_total += skipped;
            }
        }
        assert_eq!(skipped_total, 40);
        assert!(skipped_total * 100 > ticks);
    }

    /// A tick more than half a period late takes the later slot and counts
    /// the one it missed; the next on-time tick then finds its slot filled
    /// and publishes nothing.
    #[test]
    fn a_tick_more_than_half_a_period_late_counts_one_skip() {
        assert_eq!(grid_slot(0, 0, P + P / 2 + 1, P), Some((2, 1)));
        assert_eq!(grid_slot(0, 2, 2 * P + 30_000, P), None);
        assert_eq!(grid_slot(0, 2, 3 * P + 30_000, P), Some((3, 0)));
    }

    /// A tick source faster than the grid (the lockstep opt-out's logical
    /// period) publishes exactly once per slot.
    #[test]
    fn a_fast_tick_source_publishes_once_per_slot() {
        let mut last = 0;
        let mut published = 0;
        let mut skipped_total = 0;
        let mut now = 1_000;
        while now < 100 * P {
            if let Some((slot, skipped)) = grid_slot(0, last, now, P) {
                last = slot;
                published += 1;
                skipped_total += skipped;
            }
            now += 1_087_000; // about 920 Hz
        }
        assert_eq!(skipped_total, 0);
        assert_eq!(published, 100);
    }
}
