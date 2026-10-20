// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run` on sim time: the graph clock follows `/clock`.
//!
//! A graph runs on sim time when its YAML says `sim_time: true`, or when the
//! command line says `--time-source external`; the two select the same path
//! ([`resolve_time_source`]), an external clock driven by `rosgraph_msgs/Clock`
//! samples on `/clock`. The rules the clock follows are
//! [`cerulion_core::graph::sim_time`]'s; this module is the loop that applies
//! them to a real graph:
//!
//! 1. **Wait for the epoch.** Nothing is built and nothing fires until the
//!    first sample (`await_first_clock`). After [`NO_CLOCK_GRACE`] without
//!    one, exactly one warning names `/clock` and the YAML key
//!    ([`NO_CLOCK_WARNING`]); a clock that arrives in time prints nothing.
//! 2. **Place the clock, then build.** The graph clock is set to the epoch
//!    BEFORE the runtime is built, so every period deadline is anchored one
//!    interval after the epoch and no deadline window counts a phantom miss,
//!    exactly as a build at that instant would have produced.
//! 3. **One step per sample** (`follow_clock`). A forward sample steps the
//!    graph by the difference, an equal one by zero (data triggers still
//!    drain while a paused simulator republishes the same time), and a
//!    backward one is refused with a warning and never stored.
//!
//! The graph advances only when `/clock` does. Wall time is read only to
//! pace the idle poll and to time the no-clock warning: both change WHEN a
//! diagnostic prints, never WHAT the graph computes (Replay = Live).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::graph::sim_time::{
    clock_sample_ns, SimClockFeed, SimClockVerdict, SIM_CLOCK_SCHEMA, SIM_CLOCK_TOPIC, SIM_TIME_KEY,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::{CerulionSubscriber, GraphRuntime, TransportManager, VirtualClock};
use native_ros2_messages::rosgraph_msgs::Clock;

use crate::error::{CliError, CliResult};
use crate::graph_cmd::TimeSource;

/// How long a sim-time run waits for the first `/clock` sample before it
/// prints [`NO_CLOCK_WARNING`]. A simulator can take a moment to start; a
/// graph that has waited this long with nothing on `/clock` is more likely
/// misconfigured than slow.
pub const NO_CLOCK_GRACE: Duration = Duration::from_secs(2);

/// The one warning a sim-time run prints when no clock has arrived. It names
/// the topic and the YAML key so a stalled graph explains itself.
pub const NO_CLOCK_WARNING: &str = "sim time is on (`sim_time: true` in the graph YAML, or \
    `--time-source external`) but no clock has arrived on `/clock`: time-triggered nodes stay \
    idle until the first `rosgraph_msgs/Clock` sample. Start the simulator or sim bridge that \
    publishes `/clock`.";

/// The refusal for a run that names two clock sources: `sim_time: true` in
/// the YAML and `--time-source virtual` on the command line.
pub const TWO_CLOCK_SOURCES_REJECTION: &str = "`sim_time: true` in the graph YAML and \
    `--time-source virtual` on the command line name two clock sources; a graph has one. Drop \
    `--time-source virtual` to run on sim time, or set `sim_time: false` to run on the virtual \
    clock.";

/// The clock a run uses once the graph's `sim_time:` switch and the
/// `--time-source` flag are combined.
///
/// * The switch on and the flag `real` (the default) or `external`: sim time
///   ([`TimeSource::External`]). The flag's `external` spelling is the
///   command-line way to turn sim time on for a graph whose YAML does not.
/// * The switch off: the flag as given.
/// * The switch on and the flag `virtual`: refused, two clock sources.
pub fn resolve_time_source(
    flag: TimeSource,
    sim_time: bool,
    graph_identity: &str,
) -> CliResult<TimeSource> {
    match (flag, sim_time) {
        (TimeSource::Virtual, true) => Err(CliError::Validation(format!(
            "graph '{graph_identity}': {TWO_CLOCK_SOURCES_REJECTION}"
        ))),
        (_, true) => Ok(TimeSource::External),
        (flag, false) => Ok(flag),
    }
}

/// Decides, from the wall time elapsed since the run started and whether a
/// clock has arrived, whether to print [`NO_CLOCK_WARNING`]: once, after the
/// grace period, and never when a clock arrived in time.
#[derive(Debug)]
pub struct NoClockWatch {
    grace: Duration,
    warned: bool,
}

impl NoClockWatch {
    /// A watch that warns after `grace` without a clock.
    pub const fn new(grace: Duration) -> Self {
        Self {
            grace,
            warned: false,
        }
    }

    /// `Some(text)` exactly once: the first poll at or past the grace period
    /// while no clock has arrived. `None` otherwise.
    pub fn poll(&mut self, elapsed: Duration, clock_valid: bool) -> Option<&'static str> {
        if self.warned || clock_valid || elapsed < self.grace {
            return None;
        }
        self.warned = true;
        Some(NO_CLOCK_WARNING)
    }
}

/// Why a frame on `/clock` was not a clock sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockRejection {
    /// The frame's schema hash is not `rosgraph_msgs/Clock`'s: another type
    /// is being published on the clock topic.
    WrongSchema {
        /// The hash the frame carried.
        got: u64,
    },
    /// The payload is shorter than a `rosgraph_msgs/Clock`.
    Undersized {
        /// The payload length.
        size_bytes: usize,
    },
    /// The stamp is not a point in time (negative seconds, or a nanosecond
    /// field of a whole second or more).
    MalformedStamp {
        /// The `sec` field.
        sec: i32,
        /// The `nanosec` field.
        nanosec: u32,
    },
}

/// One read of the clock topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockRead {
    /// A well-formed sample, in nanoseconds.
    Sample(u64),
    /// Nothing waiting.
    Empty,
    /// A frame that was not a clock sample; it was consumed and skipped.
    Rejected(ClockRejection),
}

/// Decode one frame from the clock topic. Pure: the header's schema hash and
/// the payload bytes in, a [`ClockRead`] out.
pub fn decode_clock_frame(schema_hash: u64, payload: &[u8]) -> ClockRead {
    if schema_hash != Clock::SCHEMA_HASH {
        return ClockRead::Rejected(ClockRejection::WrongSchema { got: schema_hash });
    }
    if payload.len() < Clock::WIRE_FIXED_SIZE {
        return ClockRead::Rejected(ClockRejection::Undersized {
            size_bytes: payload.len(),
        });
    }
    let view = <Clock as ShmMessage>::build_reader(payload);
    let (sec, nanosec) = (view.clock.sec, view.clock.nanosec);
    match clock_sample_ns(sec, nanosec) {
        Some(ns) => ClockRead::Sample(ns),
        None => ClockRead::Rejected(ClockRejection::MalformedStamp { sec, nanosec }),
    }
}

/// Where the loop reads clock samples from. The run reads the `/clock` topic
/// ([`ClockTopicSource`]); a test hands the loop a scripted sequence.
pub trait ClockSource {
    /// Pop at most one frame. `Err` is a transport failure that ends the run.
    fn read(&mut self) -> CliResult<ClockRead>;
}

/// The `/clock` topic on this process's transport, opened as soon as a
/// publisher creates it. Open-only: a sim-time graph never mints a phantom
/// clock topic that streams nothing.
pub struct ClockTopicSource {
    transport: Arc<TransportManager>,
    subscriber: Option<CerulionSubscriber>,
}

impl ClockTopicSource {
    /// A source over `transport`'s `/clock`.
    pub fn new(transport: Arc<TransportManager>) -> Self {
        Self {
            transport,
            subscriber: None,
        }
    }

    /// Open the topic if its publisher exists yet. A topic nobody has created
    /// is the normal state before the simulator starts, so it is not an error;
    /// any other failure is surfaced, it carries its own remedy.
    fn open_if_published(&mut self) -> CliResult<bool> {
        if self.subscriber.is_some() {
            return Ok(true);
        }
        match self.transport.create_subscriber_open_only(SIM_CLOCK_TOPIC) {
            Ok(subscriber) => {
                self.subscriber = Some(subscriber);
                Ok(true)
            }
            Err(_) if self.transport.data_service_missing(SIM_CLOCK_TOPIC) => Ok(false),
            Err(e) => Err(e.into()),
        }
    }
}

impl ClockSource for ClockTopicSource {
    fn read(&mut self) -> CliResult<ClockRead> {
        if !self.open_if_published()? {
            return Ok(ClockRead::Empty);
        }
        let Some(subscriber) = self.subscriber.as_ref() else {
            return Ok(ClockRead::Empty);
        };
        let mut read = ClockRead::Empty;
        let delivered = subscriber.try_receive_one(|msg| {
            read = decode_clock_frame(msg.header().schema_hash, msg.payload());
        })?;
        Ok(if delivered { read } else { ClockRead::Empty })
    }
}

/// The flood-suppressed diagnostics of a sim-time run: frames that were not
/// clock samples, and samples that went backwards. One latch per condition.
#[derive(Debug, Default)]
struct ClockDiagnostics {
    rejected: FailureRegimeLatch,
    backwards: FailureRegimeLatch,
}

impl ClockDiagnostics {
    fn note_rejected(&mut self, graph_name: &str, why: ClockRejection) {
        match self.rejected.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                schema = SIM_CLOCK_SCHEMA,
                rejection = ?why,
                "a frame on the clock topic is not a clock sample; it is skipped (publish rosgraph_msgs/Clock on /clock)"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                total_failures = total,
                suppressed,
                "frames on the clock topic are still not clock samples"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                suppressed,
                "frame on the clock topic skipped"
            ),
        }
    }

    fn note_backwards(&mut self, graph_name: &str, last_ns: u64, by_ns: u64) {
        match self.backwards.on_failure() {
            RegimeDecision::Loud => tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                last_ns,
                by_ns,
                "a clock sample went backwards; it is refused and the graph clock stands (a simulator restart needs a graph restart)"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                total_failures = total,
                suppressed,
                "clock samples are still going backwards"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                suppressed,
                "backwards clock sample refused"
            ),
        }
    }

    fn note_sample_accepted(&mut self, graph_name: &str) {
        for (latch, what) in [
            (
                &mut self.rejected,
                "frames on the clock topic are clock samples again",
            ),
            (&mut self.backwards, "clock samples move forward again"),
        ] {
            if let Some(suppressed) = latch.on_success() {
                tracing::info!(graph = %graph_name, topic = SIM_CLOCK_TOPIC, suppressed, what);
            }
        }
    }
}

/// Everything the sim-time loop needs from `graph run`.
pub struct SimTimeRun<'a, S, B>
where
    S: ClockSource,
    B: FnOnce() -> CliResult<GraphRuntime>,
{
    /// The graph, for log fields.
    pub graph_name: &'a str,
    /// Cleared by the signal handler to end the run.
    pub running: &'a AtomicBool,
    /// The graph clock. It is placed at the epoch before `build` runs.
    pub clock: &'a VirtualClock,
    /// Builds the runtime over `clock`, called once, after the first sample.
    pub build: B,
    /// Where clock samples come from.
    pub source: S,
    /// How long to wait for the first sample before warning.
    pub no_clock_grace: Duration,
    /// The pause between polls when nothing is waiting.
    pub idle: Duration,
}

/// Run a graph on sim time until `running` clears or a node requests
/// shutdown. Returns the runtime for the caller to harvest and shut down, or
/// `None` when the run ended before the first clock sample (nothing was
/// built).
pub fn run_sim_time<S, B>(run: SimTimeRun<'_, S, B>) -> CliResult<Option<GraphRuntime>>
where
    S: ClockSource,
    B: FnOnce() -> CliResult<GraphRuntime>,
{
    let SimTimeRun {
        graph_name,
        running,
        clock,
        build,
        mut source,
        no_clock_grace,
        idle,
    } = run;
    let mut diagnostics = ClockDiagnostics::default();
    let Some(epoch_ns) = await_first_clock(
        &mut source,
        running,
        graph_name,
        no_clock_grace,
        idle,
        &mut diagnostics,
    )?
    else {
        return Ok(None);
    };
    // Place the clock BEFORE the build: `Scheduler::add_node` anchors every
    // period deadline at `now + interval` and every deadline window at `now`.
    clock.set(epoch_ns);
    let mut runtime = build()?;
    let mut feed = SimClockFeed::new();
    feed.offer(epoch_ns);
    tracing::info!(
        graph = %graph_name,
        topic = SIM_CLOCK_TOPIC,
        epoch_ns,
        "graph running on sim time: the first clock sample is the epoch; press Ctrl+C to stop"
    );
    follow_clock(
        &mut runtime,
        &mut source,
        &mut feed,
        running,
        graph_name,
        idle,
        &mut diagnostics,
    )?;
    Ok(Some(runtime))
}

/// Phase 1: poll until the first well-formed sample. `None` when `running`
/// cleared first. Prints [`NO_CLOCK_WARNING`] once after `grace`.
fn await_first_clock<S: ClockSource>(
    source: &mut S,
    running: &AtomicBool,
    graph_name: &str,
    grace: Duration,
    idle: Duration,
    diagnostics: &mut ClockDiagnostics,
) -> CliResult<Option<u64>> {
    let started = Instant::now();
    let mut watch = NoClockWatch::new(grace);
    while running.load(Ordering::Relaxed) {
        match source.read()? {
            ClockRead::Sample(ns) => return Ok(Some(ns)),
            ClockRead::Rejected(why) => diagnostics.note_rejected(graph_name, why),
            ClockRead::Empty => {
                if watch.poll(started.elapsed(), false).is_some() {
                    tracing::warn!(
                        graph = %graph_name,
                        topic = SIM_CLOCK_TOPIC,
                        key = SIM_TIME_KEY,
                        "{NO_CLOCK_WARNING}"
                    );
                }
                std::thread::sleep(idle);
            }
        }
    }
    Ok(None)
}

/// Phase 2: one step per accepted sample, until `running` clears or a node
/// requests shutdown.
fn follow_clock<S: ClockSource>(
    runtime: &mut GraphRuntime,
    source: &mut S,
    feed: &mut SimClockFeed,
    running: &AtomicBool,
    graph_name: &str,
    idle: Duration,
    diagnostics: &mut ClockDiagnostics,
) -> CliResult<()> {
    while running.load(Ordering::Relaxed) {
        if runtime.shutdown_requested() {
            tracing::info!(graph = %graph_name, "node requested graph shutdown");
            break;
        }
        match source.read()? {
            ClockRead::Empty => std::thread::sleep(idle),
            ClockRead::Rejected(why) => diagnostics.note_rejected(graph_name, why),
            ClockRead::Sample(ns) => {
                let last_ns = feed.last_ns();
                match feed.offer(ns) {
                    SimClockVerdict::Forward { delta_ns } => {
                        diagnostics.note_sample_accepted(graph_name);
                        runtime.step(Duration::from_nanos(delta_ns));
                    }
                    // A feed that already holds its epoch never answers
                    // `First`; it is listed with `Equal` so the match stays
                    // total without a panic arm, and both mean "no advance".
                    SimClockVerdict::Equal | SimClockVerdict::First { .. } => {
                        diagnostics.note_sample_accepted(graph_name);
                        runtime.step(Duration::ZERO);
                    }
                    SimClockVerdict::Backward { by_ns } => {
                        diagnostics.note_backwards(graph_name, last_ns.unwrap_or(0), by_ns);
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::atomic::AtomicU64;

    // -----------------------------------------------------------------
    // resolve_time_source: the full matrix
    // -----------------------------------------------------------------

    #[test]
    fn the_yaml_switch_and_the_flag_combine_into_one_clock() {
        use TimeSource::*;
        // (flag, sim_time) -> resolved
        let cases: &[(TimeSource, bool, TimeSource)] = &[
            (Real, false, Real),
            (Virtual, false, Virtual),
            (External, false, External),
            (Real, true, External),
            (External, true, External),
        ];
        for &(flag, sim_time, want) in cases {
            assert_eq!(
                resolve_time_source(flag, sim_time, "g").expect("resolves"),
                want,
                "flag {flag:?} + sim_time {sim_time}"
            );
        }
    }

    #[test]
    fn the_virtual_clock_on_a_sim_time_graph_is_two_clock_sources() {
        let err = resolve_time_source(TimeSource::Virtual, true, "lab")
            .expect_err("refused")
            .to_string();
        assert!(err.starts_with("graph 'lab':"), "names the graph: {err}");
        assert!(
            err.contains("sim_time: true") && err.contains("--time-source virtual"),
            "names both sources: {err}"
        );
        assert!(err.contains("a graph has one"), "states the rule: {err}");
        assert!(
            err.contains("Drop `--time-source virtual`") && err.contains("sim_time: false"),
            "states both fixes: {err}"
        );
    }

    // -----------------------------------------------------------------
    // NoClockWatch: once, after the grace, never when the clock came in time
    // -----------------------------------------------------------------

    #[test]
    fn the_no_clock_warning_fires_once_after_the_grace_period() {
        let grace = Duration::from_secs(2);
        let mut watch = NoClockWatch::new(grace);
        assert_eq!(watch.poll(Duration::from_secs(1), false), None);
        assert_eq!(
            watch.poll(Duration::from_secs(2), false),
            Some(NO_CLOCK_WARNING),
            "at the grace boundary, inclusive"
        );
        assert_eq!(watch.poll(Duration::from_secs(3), false), None, "once");
        assert_eq!(watch.poll(Duration::from_secs(60), false), None);
    }

    #[test]
    fn the_no_clock_warning_never_fires_when_a_clock_arrived_in_time() {
        let mut watch = NoClockWatch::new(Duration::from_secs(2));
        assert_eq!(watch.poll(Duration::from_secs(1), false), None);
        assert_eq!(watch.poll(Duration::from_secs(5), true), None);
        assert_eq!(watch.poll(Duration::from_secs(50), true), None);
    }

    #[test]
    fn the_no_clock_warning_names_the_topic_and_the_yaml_key() {
        assert!(NO_CLOCK_WARNING.contains(SIM_CLOCK_TOPIC));
        assert!(NO_CLOCK_WARNING.contains("sim_time: true"));
        assert!(NO_CLOCK_WARNING.contains(SIM_CLOCK_SCHEMA));
    }

    // -----------------------------------------------------------------
    // decode_clock_frame: every arm, with hand-written bytes
    // -----------------------------------------------------------------

    /// A `rosgraph_msgs/Clock` payload: `sec` (i32, little endian) then
    /// `nanosec` (u32, little endian), at the alignment a shared-memory
    /// sample gives it.
    #[repr(C, align(8))]
    struct ClockBytes([u8; 8]);

    fn clock_bytes(sec: i32, nanosec: u32) -> ClockBytes {
        let mut b = [0u8; 8];
        b[..4].copy_from_slice(&sec.to_le_bytes());
        b[4..].copy_from_slice(&nanosec.to_le_bytes());
        ClockBytes(b)
    }

    #[test]
    fn a_clock_frame_decodes_to_its_nanoseconds() {
        let bytes = clock_bytes(12, 345);
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &bytes.0),
            ClockRead::Sample(12_000_000_345)
        );
        let zero = clock_bytes(0, 0);
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &zero.0),
            ClockRead::Sample(0)
        );
    }

    #[test]
    fn a_frame_of_another_type_on_the_clock_topic_is_rejected() {
        let bytes = clock_bytes(1, 0);
        let other = Clock::SCHEMA_HASH ^ 0x5a5a;
        assert_eq!(
            decode_clock_frame(other, &bytes.0),
            ClockRead::Rejected(ClockRejection::WrongSchema { got: other })
        );
    }

    #[test]
    fn an_undersized_clock_frame_is_rejected() {
        let bytes = clock_bytes(1, 0);
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &bytes.0[..4]),
            ClockRead::Rejected(ClockRejection::Undersized { size_bytes: 4 })
        );
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &[]),
            ClockRead::Rejected(ClockRejection::Undersized { size_bytes: 0 })
        );
    }

    #[test]
    fn a_malformed_stamp_is_rejected_with_its_fields() {
        let negative = clock_bytes(-1, 0);
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &negative.0),
            ClockRead::Rejected(ClockRejection::MalformedStamp {
                sec: -1,
                nanosec: 0
            })
        );
        let overflow = clock_bytes(3, 1_000_000_000);
        assert_eq!(
            decode_clock_frame(Clock::SCHEMA_HASH, &overflow.0),
            ClockRead::Rejected(ClockRejection::MalformedStamp {
                sec: 3,
                nanosec: 1_000_000_000
            })
        );
    }

    // -----------------------------------------------------------------
    // The loop over a real GraphRuntime, driven by a scripted clock
    // -----------------------------------------------------------------

    /// A scripted clock: hands out `reads` in order, then clears `running`
    /// so the loop ends the way a signal would.
    struct ScriptedClock {
        reads: VecDeque<ClockRead>,
        running: Arc<AtomicBool>,
    }

    impl ClockSource for ScriptedClock {
        fn read(&mut self) -> CliResult<ClockRead> {
            match self.reads.pop_front() {
                Some(read) => Ok(read),
                None => {
                    self.running.store(false, Ordering::Relaxed);
                    Ok(ClockRead::Empty)
                }
            }
        }
    }

    /// One period node (`period_ms`) over `clock`; `fires` counts its ticks.
    fn build_period_runtime(
        clock: Arc<VirtualClock>,
        period_ms: u64,
        fires: Arc<AtomicU64>,
    ) -> CliResult<GraphRuntime> {
        use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
        use cerulion_core::graph::node::{ClosureNodeEntry, NodeEntry, NodeInfo};
        use cerulion_core::MacroPolicy;
        use indexmap::IndexMap;

        let config = GraphConfig {
            sim_time: true,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "sim".to_string(),
            prefix: "sim_time_test".to_string(),
            nodes: vec![NodeDef {
                fuse: None,
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "u8".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            }],
        };
        let entry: Box<dyn NodeEntry> = Box::new(
            ClosureNodeEntry::new(
                NodeInfo::from_names(vec![], vec!["out".to_string()])
                    .with_policy(MacroPolicy::Period { period_ms }),
                move |_ctx| {
                    fires.fetch_add(1, Ordering::Relaxed);
                    Ok(())
                },
            )
            .with_label("sim_ticker"),
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("ticker".to_string(), entry);
        Ok(GraphRuntime::build_for_test(config, factories, clock, 4)?)
    }

    /// The hand-written schedule: a 10 ms period node, epoch at 1 s.
    ///
    /// A period fire is stamped at its DEADLINE (the scheduler's contract for
    /// every controlled clock), so the 15 ms jump from 1.010 s to 1.025 s
    /// fires the 1.020 s deadline once, stamped 1_020_000_000, not at the
    /// sample that crossed it.
    ///
    /// | sample (ns)     | verdict          | graph clock after | fires at        |
    /// |-----------------|------------------|-------------------|-----------------|
    /// | 1_000_000_000   | First (epoch)    | 1_000_000_000     | (anchor 1.010)  |
    /// | 1_005_000_000   | Forward 5 ms     | 1_005_000_000     |                 |
    /// | 1_010_000_000   | Forward 5 ms     | 1_010_000_000     | 1_010_000_000   |
    /// | 1_010_000_000   | Equal            | 1_010_000_000     |                 |
    /// | 1_000_000_000   | Backward, refused| 1_010_000_000     |                 |
    /// | 1_025_000_000   | Forward 15 ms    | 1_025_000_000     | 1_020_000_000   |
    /// | 1_030_000_000   | Forward 5 ms     | 1_030_000_000     | 1_030_000_000   |
    const SCHEDULE: [u64; 7] = [
        1_000_000_000,
        1_005_000_000,
        1_010_000_000,
        1_010_000_000,
        1_000_000_000,
        1_025_000_000,
        1_030_000_000,
    ];
    const EXPECTED_FIRE_TIMES_NS: [u64; 3] = [1_010_000_000, 1_020_000_000, 1_030_000_000];

    /// Run the schedule once; returns (fire times from the trace, fire count,
    /// final graph clock).
    fn run_schedule() -> (Vec<u64>, u64, u64) {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: SCHEDULE.iter().map(|&ns| ClockRead::Sample(ns)).collect(),
            running: Arc::clone(&running),
        };
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let runtime = run_sim_time(SimTimeRun {
            graph_name: "sim",
            running: &running,
            clock: &clock,
            build: move || build_period_runtime(build_clock, 10, build_fires),
            source,
            no_clock_grace: NO_CLOCK_GRACE,
            idle: Duration::from_millis(1),
        })
        .expect("the run completes");
        let mut runtime = runtime.expect("a clock arrived, so a runtime was built");
        let fire_times: Vec<u64> = runtime.trace().iter().map(|e| e.fire_time_ns).collect();
        use cerulion_core::Clock as _;
        let final_ns = clock.now_ns();
        runtime.shutdown();
        (fire_times, fires.load(Ordering::Relaxed), final_ns)
    }

    #[test]
    fn a_period_node_fires_at_the_sim_times_the_schedule_predicts() {
        let (fire_times, fires, final_ns) = run_schedule();
        assert_eq!(fires, 3, "three deadlines fall inside the schedule");
        assert_eq!(fire_times, EXPECTED_FIRE_TIMES_NS);
        assert_eq!(
            final_ns, 1_030_000_000,
            "the graph clock is the last accepted sample; the refused one left it alone"
        );
    }

    #[test]
    fn two_runs_of_the_same_schedule_are_identical() {
        let a = run_schedule();
        let b = run_schedule();
        assert_eq!(a, b);
        assert_eq!(a.0, EXPECTED_FIRE_TIMES_NS);
    }

    #[test]
    fn nothing_fires_and_nothing_is_built_before_the_first_clock_sample() {
        // A run that ends (signal) with no clock at all: no runtime, no fires.
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: VecDeque::from([ClockRead::Empty, ClockRead::Empty]),
            running: Arc::clone(&running),
        };
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let built = run_sim_time(SimTimeRun {
            graph_name: "sim",
            running: &running,
            clock: &clock,
            build: move || build_period_runtime(build_clock, 10, build_fires),
            source,
            no_clock_grace: NO_CLOCK_GRACE,
            idle: Duration::from_millis(1),
        })
        .expect("ends cleanly");
        assert!(built.is_none(), "no clock: nothing was built");
        assert_eq!(fires.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn rejected_frames_before_the_epoch_are_skipped_and_the_first_sample_still_wins() {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: VecDeque::from([
                ClockRead::Rejected(ClockRejection::WrongSchema { got: 7 }),
                ClockRead::Empty,
                ClockRead::Sample(2_000_000_000),
                ClockRead::Rejected(ClockRejection::Undersized { size_bytes: 1 }),
                ClockRead::Sample(2_010_000_000),
            ]),
            running: Arc::clone(&running),
        };
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let mut runtime = run_sim_time(SimTimeRun {
            graph_name: "sim",
            running: &running,
            clock: &clock,
            build: move || build_period_runtime(build_clock, 10, build_fires),
            source,
            no_clock_grace: NO_CLOCK_GRACE,
            idle: Duration::from_millis(1),
        })
        .expect("ends cleanly")
        .expect("built at the first sample");
        let fire_times: Vec<u64> = runtime.trace().iter().map(|e| e.fire_time_ns).collect();
        runtime.shutdown();
        assert_eq!(
            fire_times,
            [2_010_000_000],
            "one deadline, at the epoch plus 10 ms"
        );
        assert_eq!(fires.load(Ordering::Relaxed), 1);
    }

    #[test]
    #[tracing_test::traced_test]
    fn the_no_clock_warning_prints_exactly_once_and_names_the_topic() {
        // A zero grace makes the first empty poll late; the second empty poll
        // must not repeat the line.
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: VecDeque::from([ClockRead::Empty, ClockRead::Empty, ClockRead::Empty]),
            running: Arc::clone(&running),
        };
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let built = run_sim_time(SimTimeRun {
            graph_name: "sim",
            running: &running,
            clock: &clock,
            build: move || build_period_runtime(build_clock, 10, build_fires),
            source,
            no_clock_grace: Duration::ZERO,
            idle: Duration::ZERO,
        })
        .expect("ends cleanly");
        assert!(built.is_none());
        logs_assert(|lines: &[&str]| {
            let hits = lines
                .iter()
                .filter(|l| l.contains("no clock has arrived on `/clock`"))
                .count();
            if hits == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected exactly one no-clock warning, found {hits}"
                ))
            }
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_clock_that_arrives_in_time_prints_no_warning() {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: VecDeque::from([ClockRead::Empty, ClockRead::Sample(5_000_000_000)]),
            running: Arc::clone(&running),
        };
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let runtime = run_sim_time(SimTimeRun {
            graph_name: "sim",
            running: &running,
            clock: &clock,
            build: move || build_period_runtime(build_clock, 10, build_fires),
            source,
            no_clock_grace: NO_CLOCK_GRACE,
            idle: Duration::ZERO,
        })
        .expect("ends cleanly")
        .expect("built");
        runtime.shutdown();
        assert!(
            !logs_contain("no clock has arrived"),
            "a clock inside the grace period must print nothing"
        );
    }

    // -----------------------------------------------------------------
    // The real `/clock` topic: open-only, then samples in order
    // -----------------------------------------------------------------

    #[test]
    fn the_clock_topic_source_reads_published_samples_in_order() {
        use cerulion_core::transport::TransportConfig;
        use cerulion_core::wire::MaxSliceLen;

        let clock = Arc::new(VirtualClock::new());
        let transport = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("sim_clock_src_{}", std::process::id()),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("test transport");
        let mut source = ClockTopicSource::new(Arc::clone(&transport));

        // Nobody publishes yet: the topic does not exist, and that is Empty,
        // not an error (open-only never mints the topic).
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
        assert!(source.subscriber.is_none(), "nothing to open yet");

        let mut publisher = transport
            .create_publisher(SIM_CLOCK_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("clock publisher");
        // The topic exists now: the next read opens it (nothing waiting yet).
        // Samples published before a reader attaches are not replayed to it,
        // exactly as a sim bridge that started first is joined mid-stream.
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
        assert!(
            source.subscriber.is_some(),
            "opened on the first read after creation"
        );
        for (sec, nanosec) in [(12i32, 345u32), (12, 346), (13, 0)] {
            let mut proxy = publisher.loan_proxy::<Clock>().expect("loan");
            proxy.clock.sec = sec;
            proxy.clock.nanosec = nanosec;
        }
        // Every sample, in publish order: a lagging reader must not skip a
        // sample, or the fire times depend on scheduling.
        let mut got = Vec::new();
        for _ in 0..3 {
            got.push(source.read().expect("read"));
        }
        assert_eq!(
            got,
            [
                ClockRead::Sample(12_000_000_345),
                ClockRead::Sample(12_000_000_346),
                ClockRead::Sample(13_000_000_000),
            ]
        );
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
    }
}
