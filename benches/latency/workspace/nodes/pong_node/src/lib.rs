//! Echo node for the graph-runtime round-trip bench (benches/latency
//! workspace line — a port of
//! benches/cerulion_round_trip_quiescent/graph_rtt_bench).
//!
//! Data-triggered on `ping/ping_out`. Forwards the wall timestamp from
//! `ping_in.height/width` to `echo_out.height/width` VERBATIM (it does not
//! re-stamp — the RTT window spans ping's stamp to latency's read) and
//! re-loans the same byte count for the variable `data` field (loan only,
//! never filled — Mode-A fill-exclusion).
//!
//! G3 shape (audit finding): loan + prep writes first, the stamp copy
//! LAST — between reading the inbound stamp and the tick-return
//! Drop-commit publish the only work is the two u32 copies, mirroring
//! the ROS2 pong (`pong_node.cpp`: "the ts_ns copy is the ONLY work
//! between receive and publish"). Everything in this tick sits inside
//! the timed window either way (this node runs between ping's stamp and
//! latency's read), so the reorder changes no measurement — it keeps
//! the stamp-handling discipline uniform across the suite.
//!
//! In the SPLIT leg this node runs in group g2 with the latency node, so
//! the ping -> pong edge is the one real cross-process hop. See
//! `nodes/ping_node/src/lib.rs` for the full timestamp-mechanism note
//! (unconditional wall `real_ns()` stamps; `CER_BENCH_WALL_STAMP` not
//! consulted) and the bench-only non-determinism rationale.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node]
pub struct PongNode {
    #[input(trigger)]
    ping_in: Image,
    #[output]
    echo_out: Image,

    payload_size: usize,
    /// Delivery accounting: frames this node wrote completely and
    /// handed to the output proxy (see ping_node's `published` — the
    /// Drop-time commit can still fail, and run_workspace.sh surfaces any
    /// `OutputProxy:` line from the run log). Fires observed == frames
    /// forwarded
    /// (every tick forwards exactly once).
    forwarded: u64,
    leg: String,
    /// Set on the first `tick`; the receipt prints only from an instance
    /// that ticked. See ping_node's `ticked`: the supervisor's planning
    /// build runs init + shutdown on every node without ticking it.
    ticked: bool,
}

#[cerulion_node_impl]
impl PongNode {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.payload_size = ctx.env("CER_BENCH_PAYLOAD_SIZE", 64);
        self.leg = ctx.env_str("CER_BENCH_LEG", "unknown");
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticked = true;
        // G3: prep first (the first port write triggers the lazy SHM
        // loan), the stamp copy last — see the module docs.
        self.echo_out.step = 0;
        self.echo_out.is_bigendian = 0;
        self.echo_out.set_header_bytes(&[])?;
        self.echo_out.set_encoding("rt")?;
        let _ = self.echo_out.loan_data(self.payload_size)?;
        self.forwarded += 1;
        let h = self.ping_in.height;
        let w = self.ping_in.width;
        self.echo_out.height = h;
        self.echo_out.width = w;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.ticked {
            // A planning instance (init + shutdown, never ticked) has nothing
            // to account for; see `ticked`. The runner's gate still fails
            // closed when the live instance prints nothing (MISSING).
            return Ok(());
        }
        // Delivery-accounting line (the mp_latency convention) — greppable
        // as `RTT_DELIVERY` in the run log. published (ping) >= forwarded
        // (here) >= received (latency), with any deficit explained by
        // data-trigger latest-wins coalescing, NEVER silent loss (the
        // host's `drop_oldest` telemetry lines carry the eviction count).
        println!(
            "RTT_DELIVERY role=pong leg={} forwarded={}",
            self.leg, self.forwarded
        );
        Ok(())
    }
}
