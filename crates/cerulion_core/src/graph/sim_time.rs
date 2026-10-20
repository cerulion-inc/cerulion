// SPDX-License-Identifier: AGPL-3.0-only
//! Sim time: the `sim_time:` graph YAML switch and the rules by which the graph
//! clock follows `/clock`.
//!
//! With `sim_time: true` the graph clock is no longer the real clock. It is a
//! controlled [`VirtualClock`](crate::clock::VirtualClock) that a feeder moves
//! from `rosgraph_msgs/Clock` samples on [`SIM_CLOCK_TOPIC`], so a simulator or
//! a sim bridge drives time: every period trigger, every input and output
//! deadline (`expect_within_ms`, `promise_within_ms`), every output stamp and
//! every `now_ns()` read follows the published clock. The graph advances only
//! when `/clock` does. `tick_within_ms` does not follow it: that budget times
//! the callback itself on the machine's monotonic clock, so it measures real
//! execution time even while `/clock` is paused.
//!
//! # Replay = Live (Principle #7)
//!
//! The clock moves ONLY through the scheduler's per-step advance: the feeder
//! turns each accepted sample into one `step(delta)`, so the clock-driven
//! schedule is a pure function of the `/clock` sample sequence: two runs over
//! the same samples step to the same stamps and fire the same period nodes at
//! them. Data-triggered nodes also depend on which input frames have arrived
//! by each step, which `/clock` alone does not fix. Nothing here reads the
//! wall clock. A sim-time run is not recordable yet (`--record` is refused);
//! when it becomes one, the step deltas are already the boundaries a
//! recording needs, and the epoch must be recorded as the first boundary.
//!
//! # The three rules
//!
//! 1. **Validity gate.** Before the first sample arrives time is not valid:
//!    the graph is not stepped, so nothing time-triggered fires.
//! 2. **Monotonic time.** A sample behind the last accepted one is REFUSED
//!    ([`SimClockVerdict::Backward`]) and never stored: a rewind would turn
//!    every deadline delta negative. The caller logs it; the clock stands.
//!    An equal sample is accepted as a zero-length step, so data keeps flowing
//!    while a paused simulator republishes the same time.
//! 3. **One clock source per graph, and it runs outside the graph.** `/clock`
//!    listed in `multi_publisher_topics:` while sim time is on is refused at
//!    graph load, and so is an output of the graph's own that publishes
//!    `/clock` ([`validate_sim_time`]): nothing is built until the first
//!    sample, so a graph that waited for its own node would wait forever. The
//!    CLI refuses a run that asks for the virtual clock on a sim-time graph.
//!
//! The decision functions here are pure and allocation-free; the transport
//! feeder and the run loop that call them live in the CLI engine.

use super::config::GraphConfig;
use crate::error::{TransportError, TransportResult};

/// The graph YAML key that turns sim time on: `sim_time: true`.
pub const SIM_TIME_KEY: &str = "sim_time";

/// The topic the graph clock follows under sim time.
pub const SIM_CLOCK_TOPIC: &str = "/clock";

/// The message type a sim-time publisher sends on [`SIM_CLOCK_TOPIC`].
pub const SIM_CLOCK_SCHEMA: &str = "rosgraph_msgs/Clock";

/// Nanoseconds in one second; the exclusive bound on a stamp's `nanosec`.
const NANOS_PER_SEC: u32 = 1_000_000_000;

/// The nanosecond value of a `builtin_interfaces/Time` stamp, or `None` when
/// the pair is not a point in sim time: a negative `sec`, or a `nanosec` of a
/// whole second or more.
#[inline]
pub const fn clock_sample_ns(sec: i32, nanosec: u32) -> Option<u64> {
    if sec < 0 || nanosec >= NANOS_PER_SEC {
        return None;
    }
    // `sec` is non-negative here, so the widening cast is exact; the product
    // of `i32::MAX` seconds and a billion fits a `u64` with room to spare.
    Some((sec as u64) * (NANOS_PER_SEC as u64) + nanosec as u64)
}

/// What the graph does with one `/clock` sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimClockVerdict {
    /// The first sample of the run: the clock epoch. The caller places the
    /// graph clock at it before the first step.
    First {
        /// The sample, in nanoseconds.
        epoch_ns: u64,
    },
    /// Time moved forward: the caller steps the graph by `delta_ns`.
    Forward {
        /// The difference to the last accepted sample.
        delta_ns: u64,
    },
    /// The same time as the last accepted sample: the caller steps the graph
    /// by zero, so data triggers still drain while the simulator stands still.
    Equal,
    /// Time went backwards: the sample is refused and nothing is stored.
    Backward {
        /// How far behind the last accepted sample it was.
        by_ns: u64,
    },
}

/// Judge `sample_ns` against the last ACCEPTED sample (`None` before the
/// first). Pure: the only input is the pair, the only output is the verdict.
#[inline]
pub const fn judge_sim_clock_sample(last_ns: Option<u64>, sample_ns: u64) -> SimClockVerdict {
    match last_ns {
        None => SimClockVerdict::First {
            epoch_ns: sample_ns,
        },
        Some(last) if sample_ns > last => SimClockVerdict::Forward {
            delta_ns: sample_ns - last,
        },
        Some(last) if sample_ns == last => SimClockVerdict::Equal,
        Some(last) => SimClockVerdict::Backward {
            by_ns: last - sample_ns,
        },
    }
}

/// The run's record of `/clock`: the last accepted sample and how many were
/// refused. Allocation-free; one per run, owned by the loop that feeds the
/// graph clock.
#[derive(Debug, Default)]
pub struct SimClockFeed {
    last_ns: Option<u64>,
    refused: u64,
}

impl SimClockFeed {
    /// A feed that has seen no sample: time is not yet valid.
    pub const fn new() -> Self {
        Self {
            last_ns: None,
            refused: 0,
        }
    }

    /// Offer one sample. An accepted sample ([`First`](SimClockVerdict::First),
    /// [`Forward`](SimClockVerdict::Forward), [`Equal`](SimClockVerdict::Equal))
    /// becomes the new reference; a [`Backward`](SimClockVerdict::Backward)
    /// one is counted and otherwise ignored.
    #[inline]
    pub fn offer(&mut self, sample_ns: u64) -> SimClockVerdict {
        let verdict = judge_sim_clock_sample(self.last_ns, sample_ns);
        match verdict {
            SimClockVerdict::Backward { .. } => self.refused += 1,
            SimClockVerdict::First { .. }
            | SimClockVerdict::Forward { .. }
            | SimClockVerdict::Equal => self.last_ns = Some(sample_ns),
        }
        verdict
    }

    /// The last accepted sample, `None` before the first.
    #[inline]
    pub const fn last_ns(&self) -> Option<u64> {
        self.last_ns
    }

    /// True once a first sample has arrived: sim time is valid.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.last_ns.is_some()
    }

    /// How many samples were refused for going backwards.
    #[inline]
    pub const fn refused(&self) -> u64 {
        self.refused
    }
}

/// The load-time rules for `sim_time: true`, judged on the graph's own switch.
///
/// Two refusals, both with the fix in the text:
///
/// * `/clock` opted into several publishers through `multi_publisher_topics:`
///   declares more than one clock source; a graph has one.
/// * An output of a native node of this graph that resolves to `/clock`
///   (`topic: /clock`). The run builds nothing until the first sample arrives,
///   so a node inside the graph can never publish the sample the build waits
///   for: the clock has to come from outside the graph (a simulator or its
///   bridge, or a separate graph on the real clock).
///
/// With the switch off the key changes nothing: the listing is an ordinary
/// opt-in and the output an ordinary topic. A run that turns sim time on from
/// the command line judges the same rules through [`validate_sim_time_for`].
pub fn validate_sim_time(config: &GraphConfig) -> TransportResult<()> {
    validate_sim_time_for(config, config.sim_time)
}

/// [`validate_sim_time`] with the EFFECTIVE sim-time switch: the graph's
/// `sim_time:` key folded with the run's `--time-source`. A run on
/// `--time-source external` is a sim-time run whatever the YAML says, so it
/// passes `true` here and gets the same refusals a `sim_time: true` graph gets
/// at load; `false` validates every graph.
pub fn validate_sim_time_for(config: &GraphConfig, sim_time: bool) -> TransportResult<()> {
    if !sim_time {
        return Ok(());
    }
    if config.is_multi_publisher(SIM_CLOCK_TOPIC) {
        return Err(TransportError::GraphError {
            reason: format!(
                "graph '{}': `{SIM_TIME_KEY}: true` takes the graph clock from `{SIM_CLOCK_TOPIC}`, \
                 and `multi_publisher_topics:` opts that topic into several publishers; a graph \
                 has one clock source. Remove `{SIM_CLOCK_TOPIC}` from `multi_publisher_topics:`, \
                 or set `{SIM_TIME_KEY}: false`",
                config.identity()
            ),
        });
    }
    if let Some((node, output)) = own_clock_output(config) {
        return Err(TransportError::GraphError {
            reason: format!(
                "graph '{}': `{SIM_TIME_KEY}: true` takes the graph clock from `{SIM_CLOCK_TOPIC}`, \
                 but output '{node}/{output}' of this graph publishes `{SIM_CLOCK_TOPIC}` itself. \
                 A sim-time graph builds nothing until the first clock sample arrives, so a node \
                 inside it can never publish that sample: the clock source has to run outside \
                 this graph (a simulator or its bridge, or a separate graph on the real clock). \
                 Move the clock publisher out of this graph, or set `{SIM_TIME_KEY}: false`",
                config.identity()
            ),
        });
    }
    Ok(())
}

/// The first native output of `config` whose resolved topic is
/// [`SIM_CLOCK_TOPIC`], as `(node id, output name)`, or `None`.
///
/// Resolution is the graph's: the `topic:` override verbatim when set, else
/// the derived `/<prefix>/<node>/<output>`. Only the override is compared:
/// the derived spelling always has three segments, and `/clock` has one, so
/// a derived topic can never be the clock topic.
fn own_clock_output(config: &GraphConfig) -> Option<(&str, &str)> {
    config.native_nodes().find_map(|node| {
        node.outputs
            .iter()
            .find(|output| output.topic.as_deref() == Some(SIM_CLOCK_TOPIC))
            .map(|output| (node.id.as_str(), output.name.as_str()))
    })
}
