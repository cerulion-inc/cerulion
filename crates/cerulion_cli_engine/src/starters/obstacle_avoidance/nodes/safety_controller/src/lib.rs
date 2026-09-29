// A node is a LIBRARY the runtime loads: log with `tracing`,
// never `println!`. (Your own #[cfg(test)] tests may print.)
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

// Data-triggered controller: fires when a scan arrives and emits a forward
// velocity. In the steady state (the controller keeping up with its sensor,
// which this graph's shape guarantees) that is once per published scan.
// Under backlog it deliberately does NOT process every measurement: the
// single-slot queue below keeps only the freshest scan, so ticks coalesce to
// latest-wins (see the `scan` field comment for why that is the right safety
// behavior). If any beam is closer than the 0.5 m threshold it stops;
// otherwise it cruises at 0.3 m/s. Mirrors the SafetyController in the
// "Define a node" guide.
//
// WHY a trigger rather than a `period_ms` poll. Reacting to each
// measurement is the better control shape: the loop cannot outrun its own
// sensor, and it does no work when there is nothing new to react to. It is
// also the shape that stays DETERMINISTIC once the graph is split across
// processes: `#[input(trigger)]` makes `scan` a DAG edge, so `laser_scanner`
// levelizes strictly above this node and the multi-process level-boundary
// barrier orders the scan's publish before this tick's read. A plain
// non-trigger `#[input]` polled on a timer is NOT a DAG edge: both nodes land
// on ONE level, and when `cerulion graph run` splits them into one process per
// node (the default) which scan a tick pairs with is decided by OS
// scheduling, so the run stops being reproducible and its recording stops
// replaying. See "Scope of the data guarantee" in `docs/multi_process.md`.
#[cerulion_node]
#[derive(Default)]
struct SafetyControllerNode {
    // `trigger` fires this node once per published scan. `depth = 1` keeps a
    // single-slot queue: if the controller ever falls behind its sensor, the
    // default `drop_oldest` policy evicts the stale scan, so a tick always
    // reacts to the freshest reading: a stale range reading is worse than a
    // skipped one for a safety loop. `expect_within_ms = 100` is a deadline,
    // not a trigger: if 100 ms pass with no new scan, the node's
    // `expect_within_missed_count` goes up and a warning is logged, so a
    // stalled sensor is observable instead of silent.
    #[input(trigger, depth = 1, expect_within_ms = 100)]
    scan: LaserScan,

    // Vector3 is a fixed-only schema (x/y/z: f64): assignment in tick()
    // writes straight into the loaned shared-memory slot.
    // `promise_within_ms = 100` is the publisher-side twin: a velocity command
    // is promised at least every 100 ms, and a missed promise is counted and
    // logged the same way.
    #[output(promise_within_ms = 100)]
    linear_velocity: Vector3,
}

#[cerulion_node_impl]
impl SafetyControllerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Variable field read via the typed accessor. `scan` is this node's
        // trigger, so a tick happens because a scan just arrived: there is no
        // held-over or fabricated value to reason about.
        let ranges = self.scan.ranges();
        // Fail-safe: a publisher that sends an empty `ranges` array yields an
        // empty slice. Treat missing data as an obstacle and stop, never as a
        // clear path. `any()` on an empty iterator returns false, so the
        // `is_empty()` guard is what keeps this from driving forward blind.
        // Same fail-safe intent covers a single NaN reading (ROS LaserScan's
        // sentinel for an invalid/out-of-range beam): `NaN < 0.5` is `false`
        // under IEEE 754, so without the explicit `is_nan()` check a bad beam
        // would silently read as clear path instead of stopping.
        let obstacle_close = ranges.is_empty() || ranges.iter().any(|&r| r.is_nan() || r < 0.5);
        // Fixed field write: goes straight to shared memory.
        self.linear_velocity.x = if obstacle_close { 0.0 } else { 0.3 };
        Ok(())
    }
}
