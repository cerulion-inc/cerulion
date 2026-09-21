//! Ping initiator — the FIXED-POD type-class twin of `ping_node`
//! (the type-class axis; METHODOLOGY § "The type-class axis").
//!
//! Same role, pacing machinery, G3 stamp-late discipline, and delivery
//! accounting as `nodes/ping_node/src/lib.rs` (read its module docs —
//! everything there applies here), with ONE deliberate difference: the
//! output schema is `PodPayload`, a purely-FIXED Cerulion schema
//! (`uint32 prep + uint32 stamp_hi + uint32 stamp_lo + uint8[N-12]
//! data` — generated per sweep size by `pod_schema/pod_codegen.rs`,
//! total wire fixed size exactly N bytes, mirroring the ROS 2
//! harness's `Pod<N>` family) instead of the variable `sensor_msgs/Image`
//! the incumbent legs loan. Between the two legs the ONLY moving part
//! is the message class, so their delta isolates what a variable field
//! costs Cerulion's loaned-slot write path — the hypothesis being that
//! it costs a few %, not a serialize cliff.
//!
//! Mode-A / G3 shape per tick:
//!   1. wall-grid gate (outside the stamp→publish span);
//!   2. `prep = 0` — the FIRST port write, triggering the lazy
//!      SHM loan BEFORE the stamp is read (the pod twin of the Image
//!      leg's `step`/`encoding`/`loan_data` prep writes, and of the
//!      ROS 2 loan path's borrow-before-stamp);
//!   3. stamp LAST: only the two u32 stamp writes stand between the
//!      `real_ns()` read and the tick-return Drop-commit publish.
//!
//! The fixed `data` array is NEVER written (fill exclusion — the
//! loaned slot holds whatever it holds, exactly like the raw-iceoryx2
//! floor bench and the ROS 2 pod ping).
//!
//! MISLABEL GUARD: the array length is baked at build time
//! (CER_BENCH_POD_BYTES → `PODPAYLOAD_WIRE_SIZE`), so `init` compares
//! it against the runtime `CER_BENCH_PAYLOAD_SIZE` and returns a loud
//! Err on mismatch — a stale build can never measure one size under
//! another size's label (the runner's data-flow gate then fails the
//! size loudly).

use cerulion_core::prelude::*;

use cer_bench_msgs::{PodPayload, PODPAYLOAD_WIRE_SIZE};

#[cerulion_node(period_ms = 1)]
pub struct PodPingNode {
    #[output]
    ping_out: PodPayload,

    payload_size: usize,
    /// Delivery accounting — see ping_node.
    published: u64,
    leg: String,
    /// Wall-grid publish gate (see ping_node's module header): target
    /// rate from CER_BENCH_TARGET_RATE_HZ (0 = ungated, backtoback).
    rate_hz: u64,
    period_ns: u64,
    next_slot_ns: u64,
    /// Grid slots the graph failed to reach in time (skip-forward,
    /// never burst) — the cannot-sustain observable.
    slots_skipped: u64,
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
impl PodPingNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = payload_size_from_env("pod_ping_node", ctx)?;
        // Baked-size mislabel guard (see the module header): the fixed
        // array length is a compile-time property, so the ONLY valid
        // run is one where the baked wire size equals the size label.
        if PODPAYLOAD_WIRE_SIZE != self.payload_size {
            return Err(NodeError::Logic(format!(
                "pod_ping_node: baked PodPayload wire size is {PODPAYLOAD_WIRE_SIZE} B \
                 but CER_BENCH_PAYLOAD_SIZE={} — stale pod cdylib build; \
                 run_workspace.sh rebuilds the pod crates with \
                 CER_BENCH_POD_BYTES=<size> per sweep size",
                self.payload_size
            )));
        }
        self.leg = ctx.env_str("CER_BENCH_LEG", "unknown");
        // Wall-grid publish gate — the rate label's authority, read
        // exactly as the incumbent (variable) ping_node reads it.
        //
        // NOT `ctx.env::<u64>(..)`: that WARNS and returns the default on
        // an unparseable value, and the default here is 0 — the UNGATED
        // sentinel. So `CER_BENCH_TARGET_RATE_HZ=1e9` (or `100.0`, or a
        // value wider than u64) would silently disable the publish gate
        // while the runner, the `.rate` sidecar, the CSV and every plot
        // carried the requested rate as the label: the exact defect the
        // wall-grid gate exists to remove, entered through the knob that
        // names it. Read the raw string and parse it here, so a PRESENT
        // but unparseable value is a refusal; only an ABSENT var means 0.
        const RATE_ENV: &str = "CER_BENCH_TARGET_RATE_HZ";
        const UNSET: &str = "\u{0}unset";
        let raw = ctx.env_str(RATE_ENV, UNSET);
        self.rate_hz = if raw == UNSET {
            0
        } else {
            raw.parse::<u64>().map_err(|_| NodeError::InvalidInput {
                input: RATE_ENV.to_string(),
                reason: format!(
                    "pod_ping_node: {raw:?} is not a non-negative integer — \
                     refusing rather than falling back to 0, which is the \
                     UNGATED value and would free-run this leg under a \
                     paced label"
                ),
            })?
        };
        // A rate above 1 GHz floors `1e9 / rate_hz` to a ZERO nanosecond
        // period, and the tick gate is `if self.period_ns > 0`, so the
        // same free-run-under-a-paced-label follows. Same ceiling as the
        // incumbent ping_node, the native RateLimiter and the three ROS 2
        // nodes; 0 stays the deliberate ungated (backtoback) value.
        const MAX_RATE_HZ: u64 = 1_000_000_000;
        if self.rate_hz > MAX_RATE_HZ {
            return Err(NodeError::InvalidInput {
                input: RATE_ENV.to_string(),
                reason: format!(
                    "pod_ping_node: {} Hz exceeds the {MAX_RATE_HZ} Hz \
                     ceiling — the wall period 1e9/rate floors to 0 ns, \
                     which disables the publish gate entirely while the run \
                     keeps the requested rate as its label",
                    self.rate_hz
                ),
            });
        }
        // checked_div, not a `> 0` guard around a bare `/`: identical
        // behaviour (None only when the divisor is 0, the ungated case
        // that must leave period_ns at 0), and clippy 1.96's
        // manual_checked_ops rejects the guarded form.
        if let Some(period) = 1_000_000_000u64.checked_div(self.rate_hz) {
            self.period_ns = period;
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        // Wall-grid gate FIRST, before ANY port write (a gated tick
        // publishes nothing — the lazy-loan non-event). This
        // real_ns() read is OUTSIDE the stamp→publish span.
        if self.period_ns > 0 {
            let now = self.real_ns();
            if self.next_slot_ns == 0 {
                self.next_slot_ns = now;
            }
            if now < self.next_slot_ns {
                return Ok(());
            }
            self.next_slot_ns += self.period_ns;
            if self.next_slot_ns <= now {
                let missed = (now - self.next_slot_ns) / self.period_ns + 1;
                self.slots_skipped += missed;
                self.next_slot_ns += missed * self.period_ns;
            }
        }
        // G3: the prep write triggers the lazy SHM loan BEFORE the
        // stamp is read. The fixed `data` array is never written
        // (Mode-A fill exclusion).
        self.ping_out.prep = 0;
        self.published += 1;
        // Stamp LAST: only these two u32 writes stand between the
        // real_ns() read and the tick-return Drop-commit publish.
        let t = self.real_ns();
        self.ping_out.stamp_hi = (t >> 32) as u32;
        self.ping_out.stamp_lo = (t & 0xFFFF_FFFF) as u32;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.ticked {
            // A planning instance (init + shutdown, never ticked) has nothing
            // to account for; see `ticked`. The runner's gate still fails
            // closed when the live instance prints nothing (MISSING).
            return Ok(());
        }
        // Machine-parsable delivery accounting — same contract as
        // ping_node (run_workspace.sh greps RTT_DELIVERY per leg).
        println!(
            "RTT_DELIVERY role=ping leg={} published={} slots_skipped={}",
            self.leg, self.published, self.slots_skipped
        );
        Ok(())
    }
}
