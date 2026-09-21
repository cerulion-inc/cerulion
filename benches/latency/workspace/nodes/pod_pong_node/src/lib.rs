//! Echo node — the FIXED-POD type-class twin of `pong_node`
//! (type-class axis). See `nodes/pod_ping_node/src/lib.rs` for the
//! class rationale + the baked-size mislabel guard, and
//! `nodes/pong_node/src/lib.rs` for the role (both apply here).
//!
//! Data-triggered on `ping/ping_out` (the pod graphs mirror the variable
//! pair's node ids exactly — only `type:`/`schema:` and the prefix
//! differ). Forwards the wall stamp from
//! `ping_in.stamp_hi/lo` to `echo_out.stamp_hi/lo` VERBATIM (no
//! re-stamp — the RTT window spans ping's stamp to latency's read).
//! G3 shape: `prep = 0` first (triggers the lazy SHM loan), the stamp
//! copies LAST. The fixed `data` array is never read or written —
//! payload bytes are not memcpy'd in → out, matching the ROS 2 pod
//! pong ("the ts_ns copy is the ONLY work between receive and
//! publish") and the Image pong's loan-only forward.

use cerulion_core::prelude::*;

use cer_bench_msgs::{PodPayload, PODPAYLOAD_WIRE_SIZE};

#[cerulion_node]
pub struct PodPongNode {
    #[input(trigger)]
    ping_in: PodPayload,
    #[output]
    echo_out: PodPayload,

    payload_size: usize,
    /// Delivery accounting: fires observed == frames forwarded.
    forwarded: u64,
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
impl PodPongNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = payload_size_from_env("pod_pong_node", ctx)?;
        // Baked-size mislabel guard — see pod_ping_node.
        if PODPAYLOAD_WIRE_SIZE != self.payload_size {
            return Err(NodeError::Logic(format!(
                "pod_pong_node: baked PodPayload wire size is {PODPAYLOAD_WIRE_SIZE} B \
                 but CER_BENCH_PAYLOAD_SIZE={} — stale pod cdylib build; \
                 run_workspace.sh rebuilds the pod crates with \
                 CER_BENCH_POD_BYTES=<size> per sweep size",
                self.payload_size
            )));
        }
        self.leg = ctx.env_str("CER_BENCH_LEG", "unknown");
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        // G3: prep first (the first port write triggers the lazy SHM
        // loan), the stamp copies last. `data` untouched (Mode-A).
        self.echo_out.prep = 0;
        self.forwarded += 1;
        let hi = self.ping_in.stamp_hi;
        let lo = self.ping_in.stamp_lo;
        self.echo_out.stamp_hi = hi;
        self.echo_out.stamp_lo = lo;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.ticked {
            // A planning instance (init + shutdown, never ticked) has nothing
            // to account for; see `ticked`. The runner's gate still fails
            // closed when the live instance prints nothing (MISSING).
            return Ok(());
        }
        // Delivery accounting — same contract as pong_node.
        println!(
            "RTT_DELIVERY role=pong leg={} forwarded={}",
            self.leg, self.forwarded
        );
        Ok(())
    }
}
