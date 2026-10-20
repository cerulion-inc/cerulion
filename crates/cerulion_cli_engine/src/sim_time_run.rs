// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run` on sim time: the graph clock follows `/clock`.
//!
//! A graph runs on sim time when its YAML says `sim_time: true`, or when the
//! command line says `--time-source external`; the two select the same path
//! ([`resolve_time_source`]): a controlled `VirtualClock` that this loop steps
//! from `rosgraph_msgs/Clock` samples on `/clock`. The rules the clock follows
//! are [`cerulion_core::graph::sim_time`]'s; this module is the loop that
//! applies them to a real graph:
//!
//! 1. **Refuse what cannot run here, before waiting.** An external-policy
//!    node never fires under sim time (nothing sweeps its fd or doorbell
//!    between samples), so a graph that has one is refused at launch
//!    ([`refuse_external_nodes_under_sim_time`]), not after the simulator
//!    starts.
//! 2. **Wait for the epoch.** Nothing is built and nothing fires until the
//!    first sample (`await_first_clock`). After [`NO_CLOCK_GRACE`] without
//!    one, exactly one warning names `/clock` and the YAML key
//!    ([`NO_CLOCK_WARNING`]); a clock that arrives in time prints nothing.
//! 3. **Place the clock, then build.** The graph clock is set to the epoch
//!    BEFORE the runtime is built, so every period deadline is anchored one
//!    interval after the epoch and no deadline window counts a phantom miss,
//!    exactly as a build at that instant would have produced.
//! 4. **One step per sample** (`follow_clock`). A forward sample steps the
//!    graph by the difference, an equal one by zero (data triggers still
//!    drain while a paused simulator republishes the same time), and a
//!    backward one is refused with a warning and never stored. A clock that
//!    stops mid-run is reported once per stall ([`CLOCK_STALLED_WARNING`]):
//!    the graph stands still, and says so.
//!
//! The graph advances only when `/clock` does. Wall time is read only to
//! pace the idle poll and to time the two clock warnings: all three change
//! WHEN a diagnostic prints, never WHAT the graph computes (Replay = Live).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::sim_time::{
    clock_sample_ns, SimClockFeed, SimClockVerdict, SIM_CLOCK_SCHEMA, SIM_CLOCK_TOPIC, SIM_TIME_KEY,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::transport::subscriber::{DataOnlySubscriber, OwnedInboundSample};
use cerulion_core::wire::WireHeader;
use cerulion_core::{GraphRuntime, MacroPolicy, TransportManager, VirtualClock};
use indexmap::IndexMap;
use native_ros2_messages::rosgraph_msgs::Clock;

use crate::error::{CliError, CliResult};
use crate::graph_cmd::TimeSource;

/// How long a sim-time run waits for the first `/clock` sample before it
/// prints [`NO_CLOCK_WARNING`], and how long it waits for the next one
/// mid-run before it prints [`CLOCK_STALLED_WARNING`]. A simulator can take a
/// moment to start; a graph that has waited this long with nothing on
/// `/clock` is more likely misconfigured than slow.
pub const NO_CLOCK_GRACE: Duration = Duration::from_secs(2);

/// The one warning a sim-time run prints when no clock has arrived. It names
/// the topic and the YAML key so a stalled graph explains itself.
pub const NO_CLOCK_WARNING: &str = "sim time is on (`sim_time: true` in the graph YAML, or \
    `--time-source external`) but no clock has arrived on `/clock`: time-triggered nodes stay \
    idle until the first `rosgraph_msgs/Clock` sample. Start the simulator or sim bridge that \
    publishes `/clock`.";

/// The warning a sim-time run prints once per stall when `/clock` stops
/// mid-run: the graph is built and has stepped, and nothing has arrived for
/// [`NO_CLOCK_GRACE`]. The graph clock stands at the last accepted sample, so
/// no period node fires and no data trigger drains until the next sample.
pub const CLOCK_STALLED_WARNING: &str = "no clock sample has arrived on `/clock` since the \
    last accepted one: the graph clock stands still and nothing fires or drains until the \
    simulator publishes `/clock` again. A simulator that was restarted needs the graph restarted \
    too (a sample behind the last accepted one is refused).";

/// The refusal for a run that names two clock sources: `sim_time: true` in
/// the YAML and `--time-source virtual` on the command line.
pub const TWO_CLOCK_SOURCES_REJECTION: &str = "`sim_time: true` in the graph YAML and \
    `--time-source virtual` on the command line name two clock sources; a graph has one. Drop \
    `--time-source virtual` to run on sim time, or set `sim_time: false` to run on the virtual \
    clock.";

/// The refusal for a sim-time run of a graph with external-policy nodes (the
/// dispatch site prefixes the graph name and the node ids). Under sim time the
/// graph steps once per `/clock` sample and nothing sweeps an external node's
/// fd or doorbell between samples, so such a node would never fire: the run
/// is refused at launch, before it waits for a clock, with a fix that works
/// (`--time-source real` would resolve back to sim time on a `sim_time: true`
/// graph, so it is not the fix here).
pub const SIM_TIME_EXTERNAL_NODES_REJECTION: &str = "cannot run under sim time (`sim_time: \
    true` in the graph YAML, or `--time-source external`): the sim-time loop steps the graph \
    once per `/clock` sample and nothing sweeps an external-policy node's fd or doorbell between \
    samples, so these nodes would never fire. Set `sim_time: false` and drop `--time-source \
    external` to run them on the live clock, or move them to a separate graph on the real \
    clock.";

/// The notice `cerulion graph profile` prints for a `sim_time: true` graph:
/// a profile is a real-clock measurement and the switch does not apply to it.
pub const SIM_TIME_PROFILE_IGNORED_NOTICE: &str = "`sim_time: true` does not apply to `graph \
    profile`: a profile measures per-node tick durations and fire rates on the real clock and \
    reads no `/clock`, so this graph is profiled on real time. The cost snapshot it writes is a \
    real-time measurement; `cerulion graph run` is the verb that runs the graph on sim time.";

/// The notice a sim-time run prints when the graph declares an enabled
/// `network:` block (the log fields name the egress and ingress topics): a
/// controlled-clock run opens no gateway, so the declared topics stay local.
pub const SIM_TIME_NETWORK_INERT_NOTICE: &str = "sim time (`sim_time: true` in the graph \
    YAML, or `--time-source external`) opens no network gateway: the graph's declared `network:` \
    egress and ingress topics will NOT cross the machine boundary for this run. Set `sim_time: \
    false` and drop `--time-source external` to run the gateway on the live clock.";

/// The ids of `config`'s native nodes whose entry declares the `external`
/// trigger policy, in declaration order. Read off the loaded entries' own
/// metadata (`NodeEntry::info`), the same source the built scheduler's
/// `external_policy_node_ids` would report, so the refusal can be made BEFORE
/// the build, which under sim time happens only after the first clock sample.
/// A node whose entry is missing from `factories` is not judged here: the
/// build refuses it on its own terms. A corrupt entry's metadata error is
/// surfaced; it would refuse the build anyway.
pub fn external_policy_node_ids(
    config: &GraphConfig,
    factories: &IndexMap<String, Box<dyn NodeEntry>>,
) -> CliResult<Vec<String>> {
    let mut ids = Vec::new();
    for node in config.native_nodes() {
        let Some(entry) = factories.get(&node.id) else {
            continue;
        };
        if entry.info()?.policy() == Some(MacroPolicy::External) {
            ids.push(node.id.clone());
        }
    }
    Ok(ids)
}

/// Refuse a sim-time run of a graph with external-policy nodes, naming every
/// offender and the fix ([`SIM_TIME_EXTERNAL_NODES_REJECTION`]). `Ok(())` for
/// a graph without one, the common path. Called before the run waits for its
/// first clock sample, so a graph that can never run says so at launch.
pub fn refuse_external_nodes_under_sim_time(
    graph_name: &str,
    config: &GraphConfig,
    factories: &IndexMap<String, Box<dyn NodeEntry>>,
) -> CliResult<()> {
    let ids = external_policy_node_ids(config, factories)?;
    if ids.is_empty() {
        return Ok(());
    }
    tracing::error!(
        graph = %graph_name,
        nodes = ?ids,
        key = SIM_TIME_KEY,
        "refusing to run graph on sim time: external-policy node(s) would never fire (nothing sweeps their fd/doorbell between clock samples); set `sim_time: false` and drop `--time-source external`"
    );
    Err(CliError::Validation(format!(
        "graph '{graph_name}': external-policy node(s) [{}] {SIM_TIME_EXTERNAL_NODES_REJECTION}",
        ids.join(", ")
    )))
}

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
    /// The frame's wire header does not say where its body ends: the header
    /// does not parse, or its `total_size` is smaller than a header or runs
    /// past the frame the transport delivered. No field of such a frame is
    /// read, whatever schema hash it carries.
    BadFraming {
        /// The header's `total_size`, `None` when no header parsed.
        total_size: Option<u32>,
        /// The bytes the transport delivered for the frame.
        frame_bytes: usize,
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

/// Split one frame from the clock topic into its schema hash and its body.
/// Pure: the parsed header (`None` when none parsed) and the frame bytes in.
///
/// `frame` is the frame as the subscriber exposes it: cut at the header's
/// `total_size` when that size lies inside the delivered slot, the whole slot
/// otherwise. So a `total_size` that is not exactly `frame.len()`, or one
/// smaller than a header, is a frame whose body bounds are unknown, and it is
/// refused as [`ClockRejection::BadFraming`] before its schema hash is
/// trusted. A frame whose framing holds yields the bytes after the header.
pub fn clock_frame_body(
    header: Option<WireHeader>,
    frame: &[u8],
) -> Result<(u64, &[u8]), ClockRejection> {
    let bad = |total_size| ClockRejection::BadFraming {
        total_size,
        frame_bytes: frame.len(),
    };
    let header = header.ok_or_else(|| bad(None))?;
    let total = header.total_size as usize;
    if total < WireHeader::SIZE || total != frame.len() {
        return Err(bad(Some(header.total_size)));
    }
    Ok((header.schema_hash, &frame[WireHeader::SIZE..]))
}

/// Where the loop reads clock samples from. The run reads the `/clock` topic
/// ([`ClockTopicSource`]); a test hands the loop a scripted sequence.
pub trait ClockSource {
    /// Pop at most one frame. `Err` is a transport failure that ends the run.
    fn read(&mut self) -> CliResult<ClockRead>;
}

/// How many times a period node fires per accepted `/clock` sample, at most.
///
/// A sample that jumps the clock across several of a node's periods is where
/// the simulator says time is now, not a backlog of fires the node owes: the
/// node fires once, at the latest deadline the jump passed, and its next
/// deadline is one period after that. The live and replay clocks keep their
/// catch-up bursts; only a sim-time run sets this ceiling.
pub const SIM_TIME_PERIOD_FIRES_PER_SAMPLE: u32 = 1;

/// How many `/clock` samples the run's tap can hold between two polls.
///
/// The tap is a polling reader, so its queue is what absorbs a simulator that
/// publishes faster than the loop drains (the loop steps the whole graph per
/// sample). The depth is a fixed, known number rather than the service
/// ceiling so the tap pins a bounded share of the publisher's pool. A sample
/// the queue could not hold is dropped by the publisher: the next accepted
/// sample then steps the graph by the whole gap, so two steps merge into one,
/// a deadline between them fires once at the later sample, and the graph clock
/// still lands on the latest published time.
pub const CLOCK_TAP_DEPTH: usize = 64;

/// The `/clock` topic on this process's transport, opened as soon as a
/// publisher creates it. Open-only: a sim-time graph never mints a phantom
/// clock topic that streams nothing.
///
/// A data-only tap ([`DataOnlySubscriber`]): the loop POLLS the topic and
/// never waits on an event listener, so it must register none; a listener
/// nothing drains would add a notify send to every publish the simulator
/// makes. Like every data-only tap it sees no history: samples published
/// before it attached are not replayed to it, exactly as a sim bridge that
/// started first is joined mid-stream.
pub struct ClockTopicSource {
    transport: Arc<TransportManager>,
    subscriber: Option<DataOnlySubscriber>,
    /// The one-sample drain buffer, reused across polls so a poll allocates
    /// nothing once the first sample has been read.
    drained: Vec<OwnedInboundSample>,
}

impl ClockTopicSource {
    /// A source over `transport`'s `/clock`.
    pub fn new(transport: Arc<TransportManager>) -> Self {
        Self {
            transport,
            subscriber: None,
            drained: Vec::with_capacity(1),
        }
    }

    /// Open the topic if its publisher exists yet. A topic nobody has created
    /// is the normal state before the simulator starts, so it is not an error;
    /// any other failure is surfaced, it carries its own remedy.
    fn open_if_published(&mut self) -> CliResult<bool> {
        if self.subscriber.is_some() {
            return Ok(true);
        }
        match self
            .transport
            .create_data_only_subscriber_with_buffer(SIM_CLOCK_TOPIC, CLOCK_TAP_DEPTH)
        {
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
        let Some(subscriber) = self.subscriber.as_mut() else {
            return Ok(ClockRead::Empty);
        };
        // One frame per poll: the loop steps the graph once per sample, in
        // publish order, so a poll never skips a sample.
        self.drained.clear();
        if subscriber.drain_owned(1, &mut self.drained)? == 0 {
            return Ok(ClockRead::Empty);
        }
        let Some(frame) = self.drained.pop() else {
            return Ok(ClockRead::Empty);
        };
        // The framing is judged first: a frame whose header does not bound
        // its body is skipped before any field of it is read.
        Ok(
            match clock_frame_body(frame.wire_header(), frame.payload()) {
                Ok((schema_hash, body)) => decode_clock_frame(schema_hash, body),
                Err(why) => ClockRead::Rejected(why),
            },
        )
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
    /// How long to wait for the first sample before warning, and how long a
    /// mid-run gap between samples may last before the stall warning.
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
    let pacing = Pacing {
        grace: no_clock_grace,
        idle,
    };
    let Some(epoch_ns) =
        await_first_clock(&mut source, running, graph_name, pacing, &mut diagnostics)?
    else {
        return Ok(None);
    };
    // Place the clock BEFORE the build: `Scheduler::add_node` anchors every
    // period deadline at `now + interval` and every deadline window at `now`.
    clock.set(epoch_ns);
    let mut runtime = build()?;
    runtime.set_period_fire_ceiling(Some(SIM_TIME_PERIOD_FIRES_PER_SAMPLE));
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
        pacing,
        &mut diagnostics,
    )?;
    Ok(Some(runtime))
}

/// The two wall-time knobs of the loop: how long a silence on `/clock` lasts
/// before a warning, and how long an idle poll sleeps.
#[derive(Debug, Clone, Copy)]
struct Pacing {
    grace: Duration,
    idle: Duration,
}

/// Poll until the first well-formed sample. `None` when `running`
/// cleared first. Prints [`NO_CLOCK_WARNING`] once after the grace.
fn await_first_clock<S: ClockSource>(
    source: &mut S,
    running: &AtomicBool,
    graph_name: &str,
    pacing: Pacing,
    diagnostics: &mut ClockDiagnostics,
) -> CliResult<Option<u64>> {
    let started = Instant::now();
    let mut watch = NoClockWatch::new(pacing.grace);
    while running.load(Ordering::Relaxed) {
        let read = source.read()?;
        if let ClockRead::Sample(ns) = read {
            return Ok(Some(ns));
        }
        // A rejected frame is not a clock either: a stream of them still
        // reaches the no-clock warning, next to its own rejection warning.
        if watch.poll(started.elapsed(), false).is_some() {
            tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                key = SIM_TIME_KEY,
                "{NO_CLOCK_WARNING}"
            );
        }
        match read {
            ClockRead::Rejected(why) => diagnostics.note_rejected(graph_name, why),
            ClockRead::Empty | ClockRead::Sample(_) => std::thread::sleep(pacing.idle),
        }
    }
    Ok(None)
}

/// Step the graph once per accepted sample, until `running` clears or a node
/// requests shutdown. Prints [`CLOCK_STALLED_WARNING`] once per stall: the
/// first idle poll a grace after the last accepted sample, re-armed by the
/// next accepted one.
fn follow_clock<S: ClockSource>(
    runtime: &mut GraphRuntime,
    source: &mut S,
    feed: &mut SimClockFeed,
    running: &AtomicBool,
    graph_name: &str,
    pacing: Pacing,
    diagnostics: &mut ClockDiagnostics,
) -> CliResult<()> {
    let mut stall = StallWatch::new(pacing.grace);
    while running.load(Ordering::Relaxed) {
        if runtime.shutdown_requested() {
            tracing::info!(graph = %graph_name, "node requested graph shutdown");
            break;
        }
        let read = source.read()?;
        // A rejected frame advances nothing, so it counts as silence for the
        // stall warning exactly as an empty poll does.
        if !matches!(read, ClockRead::Sample(_)) && stall.poll_idle().is_some() {
            tracing::warn!(
                graph = %graph_name,
                topic = SIM_CLOCK_TOPIC,
                last_ns = feed.last_ns().unwrap_or(0),
                "{CLOCK_STALLED_WARNING}"
            );
        }
        match read {
            ClockRead::Empty => std::thread::sleep(pacing.idle),
            ClockRead::Rejected(why) => diagnostics.note_rejected(graph_name, why),
            ClockRead::Sample(ns) => {
                let last_ns = feed.last_ns();
                // The advance an accepted sample makes, `None` for a refused one.
                let advance = match feed.offer(ns) {
                    SimClockVerdict::Forward { delta_ns } => Some(Duration::from_nanos(delta_ns)),
                    // A feed that already holds its epoch never answers
                    // `First`; it is listed with `Equal` so the match stays
                    // total without a panic arm, and both mean "no advance".
                    SimClockVerdict::Equal | SimClockVerdict::First { .. } => Some(Duration::ZERO),
                    SimClockVerdict::Backward { by_ns } => {
                        diagnostics.note_backwards(graph_name, last_ns.unwrap_or(0), by_ns);
                        None
                    }
                };
                if let Some(advance) = advance {
                    diagnostics.note_sample_accepted(graph_name);
                    if stall.sample_accepted() {
                        tracing::info!(
                            graph = %graph_name,
                            topic = SIM_CLOCK_TOPIC,
                            "clock samples are arriving again"
                        );
                    }
                    runtime.step(advance);
                }
            }
        }
    }
    Ok(())
}

/// The mid-run twin of [`NoClockWatch`]: warns once per STALL, measured from
/// the last accepted sample, and re-arms when a sample arrives. Wall time is
/// read here only to time the warning (it changes when a line prints, never
/// what the graph computes).
#[derive(Debug)]
struct StallWatch {
    grace: Duration,
    last_sample_at: Instant,
    warned: bool,
}

impl StallWatch {
    /// A watch armed now, as if a sample had just been accepted (the epoch).
    fn new(grace: Duration) -> Self {
        Self {
            grace,
            last_sample_at: Instant::now(),
            warned: false,
        }
    }

    /// An idle poll (nothing waiting, or a frame that was rejected):
    /// `Some(text)` exactly once per stall, the first idle poll at or past
    /// `grace` since the last accepted sample.
    fn poll_idle(&mut self) -> Option<&'static str> {
        if self.warned || self.last_sample_at.elapsed() < self.grace {
            return None;
        }
        self.warned = true;
        Some(CLOCK_STALLED_WARNING)
    }

    /// A sample was accepted: re-arm. Returns `true` when this sample ends a
    /// stall that was warned about, so the caller can say so.
    fn sample_accepted(&mut self) -> bool {
        self.last_sample_at = Instant::now();
        std::mem::take(&mut self.warned)
    }
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
    // External-policy nodes: refused before the run waits for a clock
    // -----------------------------------------------------------------

    /// A node definition with one output `out` and no inputs.
    fn node_def(id: &str) -> cerulion_core::graph::config::NodeDef {
        use cerulion_core::graph::config::{NodeDef, OutputDef};
        NodeDef {
            fuse: None,
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }
    }

    /// An in-memory entry with `policy`, keyed the way `graph run` keys its
    /// factories: by node id.
    fn entry_with_policy(policy: MacroPolicy) -> Box<dyn NodeEntry> {
        use cerulion_core::graph::node::{ClosureNodeEntry, NodeInfo};
        Box::new(ClosureNodeEntry::new(
            NodeInfo::from_names(vec![], vec!["out".to_string()]).with_policy(policy),
            |_ctx| Ok(()),
        ))
    }

    /// A sim-time graph of the given `(id, policy)` nodes and its factories.
    fn sim_graph(
        nodes: &[(&str, MacroPolicy)],
    ) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
        let config = GraphConfig {
            sim_time: true,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "lab".to_string(),
            prefix: "lab".to_string(),
            nodes: nodes.iter().map(|(id, _)| node_def(id)).collect(),
        };
        let factories = nodes
            .iter()
            .map(|(id, policy)| (id.to_string(), entry_with_policy(policy.clone())))
            .collect();
        (config, factories)
    }

    #[test]
    #[tracing_test::traced_test]
    fn external_policy_nodes_are_refused_under_sim_time_with_a_fix_that_works() {
        let (config, factories) = sim_graph(&[
            ("camera", MacroPolicy::External),
            ("ticker", MacroPolicy::Period { period_ms: 10 }),
            ("imu", MacroPolicy::External),
        ]);
        assert_eq!(
            external_policy_node_ids(&config, &factories).expect("metadata reads"),
            ["camera", "imu"],
            "every external node, in declaration order, and no other"
        );
        let err = refuse_external_nodes_under_sim_time("lab", &config, &factories)
            .expect_err("a sim-time run cannot fire an external node")
            .to_string();
        assert!(err.starts_with("graph 'lab':"), "names the graph: {err}");
        assert!(
            err.contains("[camera, imu]"),
            "names every offender and no other: {err}"
        );
        assert!(err.contains("sim time"), "names the reason: {err}");
        assert!(
            err.contains("Set `sim_time: false`") && err.contains("drop `--time-source external`"),
            "the fix is the one that works on a `sim_time: true` graph: {err}"
        );
        assert!(
            !err.contains("--time-source real"),
            "the virtual loop's fix resolves back to sim time here, so it is not offered: {err}"
        );
        assert!(
            logs_contain("refusing to run graph on sim time"),
            "the refusal reaches the structured log"
        );

        // The common path: no external node, no refusal, no log line.
        let (config, factories) = sim_graph(&[("ticker", MacroPolicy::Period { period_ms: 10 })]);
        assert!(external_policy_node_ids(&config, &factories)
            .expect("metadata reads")
            .is_empty());
        refuse_external_nodes_under_sim_time("lab", &config, &factories)
            .expect("a graph without external nodes runs");
    }

    #[test]
    fn a_node_without_a_loaded_entry_is_left_to_the_build() {
        // The refusal judges the entries it has; a missing one is the build's
        // refusal to make, with its own (library not found) text.
        let (config, mut factories) = sim_graph(&[("camera", MacroPolicy::External)]);
        factories.clear();
        assert!(external_policy_node_ids(&config, &factories)
            .expect("nothing to read")
            .is_empty());
    }

    // -----------------------------------------------------------------
    // `graph profile`: the switch does not apply, and the run says so
    // -----------------------------------------------------------------

    #[test]
    fn the_profile_notice_names_the_switch_the_clock_and_the_sim_time_verb() {
        let notice = SIM_TIME_PROFILE_IGNORED_NOTICE;
        assert!(notice.contains("sim_time: true") && notice.contains("graph profile"));
        assert!(
            notice.contains("real clock") && notice.contains("cerulion graph run"),
            "says what the profile measures and which verb runs sim time: {notice}"
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
    // clock_frame_body: the framing is judged before any field is read
    // -----------------------------------------------------------------

    /// A frame of `header_total` declared bytes whose delivered length is
    /// `frame_len`: a zeroed header region, then `clock_bytes(1, 0)`, then
    /// zero padding.
    fn framed(header_total: u32, frame_len: usize) -> (WireHeader, Vec<u8>) {
        let mut header = WireHeader::with_schema(Clock::SCHEMA_HASH);
        header.total_size = header_total;
        let mut frame = vec![0u8; frame_len];
        let body = clock_bytes(1, 0);
        let end = frame_len.min(WireHeader::SIZE + body.0.len());
        if end > WireHeader::SIZE {
            frame[WireHeader::SIZE..end].copy_from_slice(&body.0[..end - WireHeader::SIZE]);
        }
        (header, frame)
    }

    #[test]
    fn a_well_framed_clock_frame_yields_its_hash_and_body() {
        let (header, frame) = framed(40, 40);
        let (hash, body) = clock_frame_body(Some(header), &frame).expect("framing holds");
        assert_eq!(hash, Clock::SCHEMA_HASH);
        assert_eq!(body, &clock_bytes(1, 0).0[..]);
        assert_eq!(
            decode_clock_frame(hash, body),
            ClockRead::Sample(1_000_000_000)
        );
    }

    #[test]
    fn a_frame_with_no_header_is_bad_framing() {
        assert_eq!(
            clock_frame_body(None, &[0u8; 12]),
            Err(ClockRejection::BadFraming {
                total_size: None,
                frame_bytes: 12
            })
        );
    }

    /// The subscriber exposes the whole slot when `total_size` does not fit
    /// it, so a size past the slot arrives as a mismatch with the frame
    /// length, and a size smaller than a header is refused on its own. Both
    /// carry the clock's own schema hash: the hash is not what admits a frame.
    #[test]
    fn a_total_size_that_does_not_bound_the_frame_is_bad_framing() {
        let (past, frame) = framed(4_096, 64);
        assert_eq!(
            clock_frame_body(Some(past), &frame),
            Err(ClockRejection::BadFraming {
                total_size: Some(4_096),
                frame_bytes: 64
            })
        );
        let (short, frame) = framed(8, 64);
        assert_eq!(
            clock_frame_body(Some(short), &frame),
            Err(ClockRejection::BadFraming {
                total_size: Some(8),
                frame_bytes: 64
            })
        );
        let (zero, frame) = framed(0, 64);
        assert_eq!(
            clock_frame_body(Some(zero), &frame),
            Err(ClockRejection::BadFraming {
                total_size: Some(0),
                frame_bytes: 64
            })
        );
    }

    /// A header-only frame frames correctly and its empty body is then
    /// undersized: framing and payload size are two separate verdicts.
    #[test]
    fn a_header_only_frame_is_framed_but_undersized() {
        let (header, frame) = framed(32, 32);
        let (hash, body) = clock_frame_body(Some(header), &frame).expect("framing holds");
        assert!(body.is_empty());
        assert_eq!(
            decode_clock_frame(hash, body),
            ClockRead::Rejected(ClockRejection::Undersized { size_bytes: 0 })
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

    /// A jump across several periods fires the node ONCE, at the latest
    /// deadline the jump passed, and the next deadline is one period on.
    ///
    /// | sample (ns)     | deadlines passed           | fires at        |
    /// |-----------------|----------------------------|-----------------|
    /// | 1_000_000_000   | (epoch, anchor 1.010)      |                 |
    /// | 1_100_000_000   | 1.010 .. 1.100 (ten)       | 1_100_000_000   |
    /// | 1_105_000_000   | none (next is 1.110)       |                 |
    /// | 1_110_000_000   | 1.110                      | 1_110_000_000   |
    /// | 1_145_000_000   | 1.120, 1.130, 1.140 (three)| 1_140_000_000   |
    const JUMP_SCHEDULE: [u64; 5] = [
        1_000_000_000,
        1_100_000_000,
        1_105_000_000,
        1_110_000_000,
        1_145_000_000,
    ];
    const JUMP_FIRE_TIMES_NS: [u64; 3] = [1_100_000_000, 1_110_000_000, 1_140_000_000];

    fn run_jump_schedule() -> (Vec<u64>, u64) {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: JUMP_SCHEDULE
                .iter()
                .map(|&ns| ClockRead::Sample(ns))
                .collect(),
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
        assert_eq!(
            runtime.period_fire_ceiling(),
            Some(SIM_TIME_PERIOD_FIRES_PER_SAMPLE),
            "the sim-time run sets the ceiling on the runtime it built"
        );
        let fire_times: Vec<u64> = runtime.trace().iter().map(|e| e.fire_time_ns).collect();
        runtime.shutdown();
        (fire_times, fires.load(Ordering::Relaxed))
    }

    #[test]
    fn a_jump_across_several_periods_fires_once_at_the_latest_deadline() {
        let (fire_times, fires) = run_jump_schedule();
        assert_eq!(
            fires, 3,
            "one fire per sample that passed a deadline, not thirteen"
        );
        assert_eq!(fire_times, JUMP_FIRE_TIMES_NS);
    }

    #[test]
    fn two_runs_of_the_same_jump_schedule_are_identical() {
        assert_eq!(run_jump_schedule(), run_jump_schedule());
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

    /// A clock topic that only ever carries rejected frames: `samples` first,
    /// then a rejected frame on every read. It never answers `Empty`, so a
    /// warning that only an empty poll could reach is never printed. It
    /// clears `running` after `rejected` rejected reads.
    struct RejectingClock {
        samples: VecDeque<u64>,
        rejected: usize,
        running: Arc<AtomicBool>,
    }

    impl ClockSource for RejectingClock {
        fn read(&mut self) -> CliResult<ClockRead> {
            if let Some(ns) = self.samples.pop_front() {
                return Ok(ClockRead::Sample(ns));
            }
            if self.rejected == 0 {
                self.running.store(false, Ordering::Relaxed);
            } else {
                self.rejected -= 1;
            }
            Ok(ClockRead::Rejected(ClockRejection::WrongSchema { got: 7 }))
        }
    }

    /// Count the log lines containing `needle`.
    fn count_lines(lines: &[&str], needle: &str) -> usize {
        lines.iter().filter(|l| l.contains(needle)).count()
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_stream_of_rejected_frames_still_reaches_the_no_clock_warning() {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = RejectingClock {
            samples: VecDeque::new(),
            rejected: 3,
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
        assert!(built.is_none(), "no sample was accepted, nothing was built");
        logs_assert(|lines: &[&str]| {
            let no_clock = count_lines(lines, "no clock has arrived on `/clock`");
            let rejected = count_lines(lines, "is not a clock sample");
            if no_clock == 1 && rejected == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected one no-clock warning and one rejection warning, found {no_clock} and {rejected}"
                ))
            }
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_stream_of_rejected_frames_mid_run_is_a_stall() {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = RejectingClock {
            samples: VecDeque::from([1_000_000_000]),
            rejected: 3,
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
            no_clock_grace: Duration::ZERO,
            idle: Duration::ZERO,
        })
        .expect("ends cleanly")
        .expect("built at the epoch");
        runtime.shutdown();
        assert_eq!(fires.load(Ordering::Relaxed), 0, "the clock never moved");
        logs_assert(|lines: &[&str]| {
            let stalls = count_lines(lines, "no clock sample has arrived on `/clock` since");
            if stalls == 1 {
                Ok(())
            } else {
                Err(format!("expected one stall warning, found {stalls}"))
            }
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_clock_that_stops_mid_run_warns_once_per_stall_and_reports_its_return() {
        // Epoch, one forward sample, then silence (a zero grace makes the first
        // idle poll late): ONE stall warning, not one per poll. A sample ends
        // the stall and says so; a second silence is a second stall.
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let source = ScriptedClock {
            reads: VecDeque::from([
                ClockRead::Sample(1_000_000_000),
                ClockRead::Sample(1_010_000_000),
                ClockRead::Empty,
                ClockRead::Empty,
                ClockRead::Empty,
                ClockRead::Sample(1_020_000_000),
                ClockRead::Empty,
                ClockRead::Empty,
            ]),
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
            no_clock_grace: Duration::ZERO,
            idle: Duration::ZERO,
        })
        .expect("ends cleanly")
        .expect("built at the epoch");
        runtime.shutdown();
        assert_eq!(
            fires.load(Ordering::Relaxed),
            2,
            "the two forward samples fired"
        );
        logs_assert(|lines: &[&str]| {
            let stalls = lines
                .iter()
                .filter(|l| l.contains("no clock sample has arrived on `/clock` since"))
                .count();
            let returns = lines
                .iter()
                .filter(|l| l.contains("clock samples are arriving again"))
                .count();
            if stalls == 2 && returns == 1 {
                Ok(())
            } else {
                Err(format!(
                    "expected two stall warnings (one per stall) and one return line, found {stalls} and {returns}"
                ))
            }
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_clock_that_keeps_arriving_prints_no_stall_warning() {
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
            idle: Duration::ZERO,
        })
        .expect("ends cleanly")
        .expect("built");
        runtime.shutdown();
        assert!(
            !logs_contain("no clock sample has arrived"),
            "a clock inside the grace never trips the stall warning"
        );
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
