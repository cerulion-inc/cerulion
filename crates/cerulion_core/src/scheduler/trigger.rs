// SPDX-License-Identifier: AGPL-3.0-only
//! Trigger policies for scheduler node execution.
//!
//! Each policy determines when and how a node fires within the scheduler.

use std::time::Duration;

/// How a node is triggered for execution.
#[derive(Debug, Clone)]
pub enum TriggerPolicy {
    /// Fire every `interval` (e.g., 10ms for 100Hz camera).
    ///
    /// # Catch-up semantics
    ///
    /// When `step(delta)` is called with a delta larger than the interval,
    /// the node fires multiple times ("catch-up") to account for all missed
    /// intervals. For example, `step(30ms)` with a 10ms period fires 3 times.
    ///
    /// Use `max_catchup` to cap this behavior. When set, at most `max_catchup`
    /// fires occur per step, and the next_fire time skips ahead to current time.
    /// Set to `None` (default) for unlimited catch-up.
    ///
    /// **Note:** During catch-up, ALL fires for this node complete before the
    /// next node is evaluated (insertion order). This means data-triggered nodes
    /// downstream won't interleave with catch-up fires. Identical replay requires
    /// identical step sequences.
    Period {
        interval: Duration,
        /// Maximum catch-up fires per step. `None` = unlimited (default).
        max_catchup: Option<u32>,
    },
    /// Fire when data arrives on subscribed topic.
    Data,
    /// Fire once per COMPLETE SET: one message from every input.
    ///
    /// `inputs` carries resolved source TOPICS. The graph runtime populates
    /// it with the node's `#[input(trigger)]`-marked inputs ONLY (via
    /// `trigger_marked_input_names`); a sync node's non-trigger inputs
    /// are latest-value reads that never appear here and never gate the fire.
    ///
    /// Under the graph runtime a `#[cerulion_node]` sync node is served PER
    /// SET: one fire per complete set, in set order, each input's message
    /// consumed by at most one set. A burst holding k complete sets gives k
    /// fires, and the k-th tick reads the k-th set's members rather than the
    /// freshest frame on each topic (the verdicts come from
    /// `scheduler::sync_match`).
    ///
    /// * `window = Some(d)`: bounded sync. A set's timestamps must lie within
    ///   `d` of each other. A message that a partner has run more than `d`
    ///   past is in no set: it is discarded and counted per input
    ///   (`NodeHandle::sync_unmatched_discard_count`), and the node does not
    ///   fire on it.
    /// * `window = None`: unbounded sync. Fires as soon as every input has an
    ///   unconsumed message, with no timing bound, in set order as above: a
    ///   backlog is served set by set from each input's oldest unconsumed
    ///   message, and the matcher trades that oldest member for the input's
    ///   next message only when some other input has no later message waiting
    ///   and the trade strictly tightens the set. Worst-case fire
    ///   latency is the slowest publisher's inter-arrival interval; **not
    ///   recommended for control loops** because liveness degrades silently
    ///   if one publisher slows or stops.
    ///
    /// A `Scheduler` driven directly, with no graph runtime installing the
    /// per-set transport seam, keeps the earlier latest-wins head: a second
    /// `signal_sync_input` before a fire REPLACES the first, and the fire uses
    /// the freshest stamp offered on each input.
    Sync {
        inputs: Vec<String>,
        window: Option<Duration>,
    },
    /// Fire only via explicit `trigger_external()` call.
    ///
    /// # Edge-triggered semantics
    ///
    /// Multiple `trigger_external()` calls before `step()` result in a single
    /// fire. The trigger flag is consumed on the next step. There is no queueing.
    External,
}
