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
//!    first sample (the [`ClockFeeder`] emits no event before it). After
//!    [`NO_CLOCK_GRACE`] without one, exactly one warning names `/clock` and
//!    the YAML key ([`NO_CLOCK_WARNING`]); a clock that arrives in time
//!    prints nothing.
//! 3. **Place the clock, then build.** The graph clock is set to the epoch
//!    BEFORE the runtime is built, so every period deadline is anchored one
//!    interval after the epoch and no deadline window counts a phantom miss,
//!    exactly as a build at that instant would have produced.
//! 4. **One step per accepted sample.** The [`ClockFeeder`] owns the clock
//!    source and turns each accepted sample into one [`ClockEvent`], and the
//!    [`ClockApplier`] moves the graph from those events alone. A forward
//!    sample steps the graph by the difference, an equal one by zero (data
//!    triggers still drain while a paused simulator republishes the same
//!    time), and a backward one is refused with a warning and never stored.
//!    A sample the tap could not hold ([`CLOCK_TAP_DEPTH`]) never reaches the
//!    feeder, so it merges into the next step. A clock that stops mid-run is
//!    reported once per stall ([`CLOCK_STALLED_WARNING`]): the graph stands
//!    still, and says so.
//!
//! The graph advances only when `/clock` does. Wall time is read only to
//! pace the idle poll and to time the two clock warnings: all three change
//! WHEN a diagnostic prints, never WHAT the graph computes (Replay = Live).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::graph::config::{GraphConfig, NodeDef, Ros2NodeDef};
use cerulion_core::graph::node::NodeInfo;
use cerulion_core::graph::sim_time::{
    clock_sample_ns, SimClockFeed, SimClockVerdict, SIM_CLOCK_SCHEMA, SIM_CLOCK_TOPIC, SIM_TIME_KEY,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::transport::subscriber::{DataOnlySubscriber, OwnedInboundSample};
use cerulion_core::wire::WireHeader;
use cerulion_core::{GraphRuntime, MacroPolicy, TransportManager, VirtualClock};
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
/// trigger policy, in declaration order. Read off each node library's own
/// metadata (`NodeInfo`, through `info_of`), the same source the built
/// scheduler's `external_policy_node_ids` would report, so the refusal can be
/// made BEFORE anything starts: `graph run` reads it from the libraries its
/// params pass already holds, ahead of spawning the graph's `ros2:` entries.
/// A node `info_of` has no answer for (`None`: its library is not resolved)
/// is not judged here: the load refuses it on its own terms. An unreadable
/// library's metadata error is surfaced; it would refuse the load anyway.
pub fn external_policy_node_ids(
    config: &GraphConfig,
    info_of: impl Fn(&NodeDef) -> Option<CliResult<NodeInfo>>,
) -> CliResult<Vec<String>> {
    let mut ids = Vec::new();
    for node in config.native_nodes() {
        let Some(info) = info_of(node) else {
            continue;
        };
        if info?.policy() == Some(MacroPolicy::External) {
            ids.push(node.id.clone());
        }
    }
    Ok(ids)
}

/// Refuse a sim-time run of a graph with external-policy nodes, naming every
/// offender and the fix ([`SIM_TIME_EXTERNAL_NODES_REJECTION`]). `Ok(())` for
/// a graph without one, the common path. Called before the run starts any
/// child or waits for its first clock sample, so a graph that can never run
/// says so at launch and starts nothing.
pub fn refuse_external_nodes_under_sim_time(
    graph_name: &str,
    config: &GraphConfig,
    info_of: impl Fn(&NodeDef) -> Option<CliResult<NodeInfo>>,
) -> CliResult<()> {
    let ids = external_policy_node_ids(config, info_of)?;
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

/// The ROS 2 parameter, and the launch argument by convention, that puts a
/// ROS 2 node on `/clock`.
pub const ROS2_USE_SIM_TIME: &str = "use_sim_time";

/// What a sim-time run added to a `ros2:` entry's argv to put it on
/// `/clock`: the one decision [`put_ros2_entry_on_sim_time`] makes, returned
/// so the spawn log reports the same decision it spawned with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimTimeArgument {
    /// A `ros2 launch` entry got the launch argument `use_sim_time:=true`.
    /// It reaches the launched nodes only through the launch file, so the
    /// spawn logs [`ROS2_LAUNCH_USE_SIM_TIME_NOTICE`] for it.
    LaunchArgument,
    /// A `ros2 run` entry got the parameter `-p use_sim_time:=true`, which
    /// always reaches the node.
    Parameter,
    /// The entry sets `use_sim_time` itself: nothing was added.
    KeptEntryValue,
}

/// Put a `ros2:` entry of a sim-time graph on `/clock` too, so its nodes
/// stamp, time out and fire on the same clock as the native nodes.
///
/// `argv` is the entry's spawn argv (everything after `ros2`). A `ros2 run`
/// entry gets `-p use_sim_time:=true` in a `--ros-args` block (its own, or a
/// new one at the end); a `ros2 launch` entry gets the `use_sim_time:=true`
/// launch argument, which the launch file must declare and pass to its nodes.
/// A `use_sim_time` the entry already sets (in `params:`, or as an `args:`
/// word `use_sim_time:=...`) is the entry's own choice and is kept. A
/// `params_file:` may set it too, and the run cannot read the file: so the
/// parameter is placed BEFORE `--params-file`, where a later rule wins in
/// rcl, and a value in the file wins over the run's.
pub fn put_ros2_entry_on_sim_time(def: &Ros2NodeDef, argv: &mut Vec<String>) -> SimTimeArgument {
    if ros2_entry_sets_use_sim_time(def) {
        return SimTimeArgument::KeptEntryValue;
    }
    let value = format!("{ROS2_USE_SIM_TIME}:=true");
    if def.launch.is_some() {
        argv.push(value);
        return SimTimeArgument::LaunchArgument;
    }
    // `Ros2NodeDef::argv` closes a `run` argv with its own `--ros-args`
    // block exactly when the entry has a params file or params. With a
    // params file, that block's `--ros-args` is the LAST one in the argv,
    // and the parameter goes right after it, ahead of `--params-file`.
    let at = match argv.iter().rposition(|word| word == "--ros-args") {
        Some(ros_args) if def.params_file.is_some() => ros_args + 1,
        _ if def.params_file.is_none() && def.params.is_empty() => {
            argv.push("--ros-args".to_string());
            argv.len()
        }
        _ => argv.len(),
    };
    argv.insert(at, "-p".to_string());
    argv.insert(at + 1, value);
    SimTimeArgument::Parameter
}

/// Whether a `ros2:` entry sets `use_sim_time` itself: in `params:`, or as
/// an `args:` word `use_sim_time:=...`. Such a value is the entry's own
/// choice, and a sim-time run keeps it. A value inside a `params_file:` is
/// not read here; [`put_ros2_entry_on_sim_time`] lets it win by argv order.
pub fn ros2_entry_sets_use_sim_time(def: &Ros2NodeDef) -> bool {
    let assignment = format!("{ROS2_USE_SIM_TIME}:=");
    def.params.contains_key(ROS2_USE_SIM_TIME)
        || def.args.iter().any(|arg| arg.starts_with(&assignment))
}

/// The notice a sim-time run logs when it spawns a `ros2 launch` entry with
/// the `use_sim_time:=true` launch argument
/// ([`SimTimeArgument::LaunchArgument`]). A `-p` parameter on `ros2 run`
/// always reaches the node; a launch argument reaches the launched nodes
/// only through the launch file, so the run says so at spawn.
pub const ROS2_LAUNCH_USE_SIM_TIME_NOTICE: &str = "this `ros2 launch` entry is given the \
    launch argument `use_sim_time:=true`, which takes effect only if the launch file declares \
    a `use_sim_time` launch argument and passes it to its nodes; a launch file that does not \
    declare it leaves its nodes on their own clock";

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

    /// Start the sequence count over at the next frame: the feeder calls
    /// this once, on its first poll after the epoch, because the applier
    /// built the runtime between the two polls and the numbers that frame
    /// skips were lost to the build, not to the simulator's rate. A source
    /// that reads no sequence numbers ignores it.
    fn rebaseline_sequence(&mut self) {}

    /// How many sequence gaps the source has seen over the run, printed or
    /// not: the run logs the total at its end. Zero for a source that reads
    /// no sequence numbers.
    fn sequence_gaps(&self) -> u64 {
        0
    }
}

/// How many times a period node fires per accepted `/clock` sample, at most.
///
/// A sample that jumps the clock across several of a node's periods is where
/// the simulator says time is now, not a backlog of fires the node owes: the
/// node fires once, at the latest deadline the jump passed, and its next
/// deadline is one period after that. The live and replay clocks keep their
/// catch-up bursts; only a sim-time run sets this ceiling.
///
/// Hidden from the documented surface: the ceiling is how a sim-time run
/// behaves today, not part of its contract. A run that sub-steps to the
/// nodes' own deadlines between two samples needs no ceiling.
#[doc(hidden)]
pub const SIM_TIME_PERIOD_FIRES_PER_SAMPLE: u32 = 1;

/// How many `/clock` samples the run's tap can hold between two polls.
///
/// The tap is a polling reader, so its queue is what absorbs a simulator that
/// publishes faster than the loop drains (the loop steps the whole graph per
/// sample). The depth is a fixed, known number rather than the service
/// ceiling so the tap pins a bounded share of the publisher's pool. When the
/// queue is full, the next publish evicts the OLDEST unread sample: the next
/// accepted sample then steps the graph by the whole gap, so two steps merge
/// into one, a deadline between them fires once at the later sample, and the
/// graph clock still lands on the latest published time. The loss shows as a
/// gap in the wire sequence numbers the tap reads ([`SequenceGapWatch`]).
pub const CLOCK_TAP_DEPTH: usize = 64;

/// The warning the clock tap prints when two frames it reads in a row from
/// one publisher skip wire sequence numbers: on the first gap, and again at
/// each tenfold count ([`FailureRegimeLatch`]). A sample the tap's queue
/// ([`CLOCK_TAP_DEPTH`]) could not hold was evicted, so its step merged into
/// the next one. A gap is a hint, not proof: a publisher can consume a
/// sequence number without sending a frame.
pub const CLOCK_SEQUENCE_GAP_WARNING: &str = "the frames read from `/clock` skipped sequence \
    numbers: the simulator may be publishing faster than the graph steps, and a sample the \
    run's clock queue could not hold is lost (it merges into the next step, so this run can \
    step differently from one that saw every sample). A gap is a hint, not proof: a publisher \
    can consume a sequence number without sending a frame. If every sample must count, \
    publish `/clock` no faster than the graph steps.";

/// The warning the clock tap prints, once, when the first frame it reads
/// after the graph was built skips sequence numbers: the samples published
/// while the runtime was being built overran the tap's queue, so the first
/// step after the epoch jumps over them. That is the build's doing, not the
/// simulator's rate, and it is reported apart from
/// [`CLOCK_SEQUENCE_GAP_WARNING`], whose count starts at this frame.
pub const CLOCK_BUILD_WINDOW_LOSS_WARNING: &str = "samples published on `/clock` while the \
    graph was being built overran the run's clock queue and are lost: the first step after the \
    epoch jumps over them. Sequence gaps are counted from the first frame read after the \
    build. If the start must see every sample, start the simulator's clock after the graph \
    reports it is running on sim time.";

/// A skip in the wire sequence numbers of two frames the clock tap read in a
/// row from one publisher, as [`SequenceGapWatch`] names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SequenceGap {
    /// The sequence number of the frame read before this one.
    pub last_seq: u32,
    /// This frame's sequence number.
    pub seq: u32,
    /// How many sequence numbers lie between the two.
    pub skipped: u32,
    /// How to report it: loud for the first gap of the run, counted after,
    /// and announced again at each tenfold count.
    pub decision: RegimeDecision,
}

/// What [`SequenceGapWatch`] says about one frame's sequence number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SequenceVerdict {
    /// Not a gap: the first frame, the next number, a repeat, a number
    /// behind the last, or a frame from another publisher (its stream is
    /// counted from here).
    InOrder,
    /// Numbers were skipped inside one publisher's stream.
    Gap(SequenceGap),
    /// The first frame after [`SequenceGapWatch::rebaseline`]: `skipped`
    /// numbers lie between the last frame before the rebaseline and this
    /// one (zero when none). They were consumed while the watch was told to
    /// look away, so they are not a gap of the stream, which starts over
    /// here.
    Rebaselined {
        /// How many numbers the frame skipped, zero for none.
        skipped: u32,
    },
}

/// Tracks the wire sequence numbers of the frames the clock tap reads and
/// names a gap between two consecutive ones from the same publisher: loud
/// on the first gap, counted after, announced again at each tenfold count
/// ([`FailureRegimeLatch`]; never re-armed, because every gap after the first
/// is the same condition again).
///
/// Wire sequences are per publisher, so the baseline is keyed by the frame's
/// origin: a frame from another publisher starts the count over and is never
/// a gap. A gap is a hint, not proof that a sample was lost: a publisher can
/// consume a sequence number without sending a frame. The converse holds for
/// one publisher on `/clock`: a sample the tap's queue evicted had consumed
/// its number, so a loss always leaves a gap. With two publishers taking
/// turns, a loss can hide behind a switch.
#[derive(Debug, Default)]
pub struct SequenceGapWatch {
    /// The origin and sequence number of the last frame read.
    last: Option<(u128, u32)>,
    latch: FailureRegimeLatch,
    /// Set by [`Self::rebaseline`]: the next frame starts the stream over,
    /// and the numbers it skips are reported apart from the gap count.
    rebaseline: bool,
}

impl SequenceGapWatch {
    /// A watch that has seen no frame.
    pub const fn new() -> Self {
        Self {
            last: None,
            latch: FailureRegimeLatch::new(),
            rebaseline: false,
        }
    }

    /// Start the stream over at the next frame: the numbers it skips are
    /// reported as [`SequenceVerdict::Rebaselined`], not counted as a gap.
    pub fn rebaseline(&mut self) {
        self.rebaseline = true;
    }

    /// Note the origin and sequence number of the next frame read.
    /// [`SequenceVerdict::Gap`] when `seq` skips past the last number from
    /// the same origin. [`SequenceVerdict::InOrder`] for the first frame
    /// (the baseline), the next number, a repeat, a frame from another
    /// origin, and a number behind the last: a stream that starts over is a
    /// restarted publisher's, and the backward SAMPLE it carries is the
    /// feed's to judge. Behind means more than half the 32-bit range away,
    /// so the counter wraps without a false gap.
    pub fn observe(&mut self, origin: u128, seq: u32) -> SequenceVerdict {
        let last = self.last.replace((origin, seq));
        let skipped = match last {
            Some((last_origin, last_seq)) if last_origin == origin => {
                let ahead = seq.wrapping_sub(last_seq);
                (2..=u32::MAX / 2)
                    .contains(&ahead)
                    .then(|| (last_seq, ahead - 1))
            }
            _ => None,
        };
        if std::mem::take(&mut self.rebaseline) {
            return SequenceVerdict::Rebaselined {
                skipped: skipped.map_or(0, |(_, skipped)| skipped),
            };
        }
        match skipped {
            Some((last_seq, skipped)) => SequenceVerdict::Gap(SequenceGap {
                last_seq,
                seq,
                skipped,
                decision: self.latch.on_failure(),
            }),
            None => SequenceVerdict::InOrder,
        }
    }

    /// How many gaps the watch has seen over the whole run, printed or not.
    pub fn gaps(&self) -> u64 {
        self.latch.total_failures()
    }
}

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
    graph_name: String,
    transport: Arc<TransportManager>,
    subscriber: Option<DataOnlySubscriber>,
    /// The one-sample drain buffer, reused across polls so a poll allocates
    /// nothing once the first sample has been read.
    drained: Vec<OwnedInboundSample>,
    /// The sequence numbers of the frames read, for the gap warning.
    gaps: SequenceGapWatch,
}

impl ClockTopicSource {
    /// A source over `transport`'s `/clock`; `graph_name` is for log fields.
    pub fn new(graph_name: &str, transport: Arc<TransportManager>) -> Self {
        Self {
            graph_name: graph_name.to_string(),
            transport,
            subscriber: None,
            drained: Vec::with_capacity(1),
            gaps: SequenceGapWatch::new(),
        }
    }

    /// Note a read frame's origin and sequence number and report what the
    /// watch says: a gap loud on the first, counted after and announced
    /// again at each tenfold count ([`CLOCK_SEQUENCE_GAP_WARNING`]); the
    /// samples lost to the build once, on their own
    /// ([`CLOCK_BUILD_WINDOW_LOSS_WARNING`]).
    fn note_sequence(&mut self, origin: u128, seq: u32) {
        let gap = match self.gaps.observe(origin, seq) {
            SequenceVerdict::InOrder | SequenceVerdict::Rebaselined { skipped: 0 } => return,
            SequenceVerdict::Rebaselined { skipped } => {
                tracing::warn!(
                    graph = %self.graph_name,
                    topic = SIM_CLOCK_TOPIC,
                    seq,
                    skipped,
                    "{CLOCK_BUILD_WINDOW_LOSS_WARNING}"
                );
                return;
            }
            SequenceVerdict::Gap(gap) => gap,
        };
        match gap.decision {
            RegimeDecision::Loud => tracing::warn!(
                graph = %self.graph_name,
                topic = SIM_CLOCK_TOPIC,
                last_seq = gap.last_seq,
                seq = gap.seq,
                skipped = gap.skipped,
                "{CLOCK_SEQUENCE_GAP_WARNING}"
            ),
            RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
                graph = %self.graph_name,
                topic = SIM_CLOCK_TOPIC,
                total_failures = total,
                suppressed,
                skipped = gap.skipped,
                "the frames read from the clock topic are still skipping sequence numbers"
            ),
            RegimeDecision::Suppressed { suppressed } => tracing::debug!(
                graph = %self.graph_name,
                topic = SIM_CLOCK_TOPIC,
                last_seq = gap.last_seq,
                seq = gap.seq,
                skipped = gap.skipped,
                suppressed,
                "clock tap sequence gap"
            ),
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
        // Every frame with a header counts for the sequence, a rejected one
        // too: the number was consumed whatever the body holds.
        let header = frame.wire_header();
        if let Some(header) = header {
            self.note_sequence(frame.origin(), header.sequence);
        }
        // The framing is judged first: a frame whose header does not bound
        // its body is skipped before any field of it is read.
        Ok(match clock_frame_body(header, frame.payload()) {
            Ok((schema_hash, body)) => decode_clock_frame(schema_hash, body),
            Err(why) => ClockRead::Rejected(why),
        })
    }

    fn rebaseline_sequence(&mut self) {
        self.gaps.rebaseline();
    }

    fn sequence_gaps(&self) -> u64 {
        self.gaps.gaps()
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

/// What the feeder tells the applier: the graph clock's next move.
///
/// The feeder emits exactly one [`ClockEvent::Epoch`], then one
/// [`ClockEvent::Step`] per accepted sample. A refused sample (backward) and
/// a frame that is not a clock sample emit nothing. The event stream is the
/// whole of what moves the graph clock, so it is also what a recording of a
/// sim-time run, or a second process following the same clock, would carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockEvent {
    /// The first accepted sample: the graph clock is placed here before the
    /// runtime is built.
    Epoch {
        /// The sample, in nanoseconds.
        ns: u64,
    },
    /// An accepted sample after the epoch: step the graph by `delta_ns`
    /// (zero for a sample equal to the last accepted one).
    Step {
        /// The step's position in the run: 1 for the first step after the
        /// epoch, one more for each step after it.
        seq: u64,
        /// How far this step advances the graph clock.
        delta_ns: u64,
    },
}

/// One poll of the [`ClockFeeder`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeederPoll {
    /// The graph clock moves.
    Event(ClockEvent),
    /// A frame was read and skipped, or a sample was refused: poll again now,
    /// the next frame may already be waiting.
    Skipped,
    /// Nothing was waiting: pause before the next poll.
    Idle,
}

/// The feeder: the sole owner of the clock source and of the rules that turn
/// its frames into [`ClockEvent`]s. It holds the source, the sample feed, the
/// flood-suppressed diagnostics and the two silence watches (no clock yet,
/// and a stall after the epoch). It never touches the graph: the applier
/// does that, from the events alone.
pub struct ClockFeeder<'a, S: ClockSource> {
    graph_name: &'a str,
    source: S,
    feed: SimClockFeed,
    diagnostics: ClockDiagnostics,
    grace: Duration,
    /// When the feeder started waiting for its epoch.
    started: Instant,
    no_clock: NoClockWatch,
    /// Whether the epoch has been emitted.
    epoch_seen: bool,
    /// Armed by the first poll after the epoch, so the time the applier
    /// spends building the runtime does not count as a stall.
    stall: Option<StallWatch>,
    /// The `seq` of the last [`ClockEvent::Step`] emitted.
    seq: u64,
}

impl<'a, S: ClockSource> ClockFeeder<'a, S> {
    /// A feeder over `source` that warns after `grace` of silence on
    /// `/clock`, before the epoch and after it.
    pub fn new(graph_name: &'a str, source: S, grace: Duration) -> Self {
        Self {
            graph_name,
            source,
            feed: SimClockFeed::new(),
            diagnostics: ClockDiagnostics::default(),
            grace,
            started: Instant::now(),
            no_clock: NoClockWatch::new(grace),
            epoch_seen: false,
            stall: None,
            seq: 0,
        }
    }

    /// Read at most one frame and say what it does to the graph clock.
    /// `Err` is a transport failure that ends the run.
    pub fn poll(&mut self) -> CliResult<FeederPoll> {
        if self.epoch_seen && self.stall.is_none() {
            self.stall = Some(StallWatch::new(self.grace));
            // The applier built the runtime between the epoch poll and this
            // one: the numbers the next frame skips were lost to the build,
            // so the tap reports them on their own and counts gaps from
            // that frame.
            self.source.rebaseline_sequence();
        }
        let read = self.source.read()?;
        let idle = matches!(read, ClockRead::Empty);
        // The event this read makes; `None` for an empty poll, a rejected
        // frame and a refused (backward) sample alike.
        let event = match read {
            ClockRead::Empty => None,
            ClockRead::Rejected(why) => {
                self.diagnostics.note_rejected(self.graph_name, why);
                None
            }
            ClockRead::Sample(ns) => {
                let last_ns = self.feed.last_ns();
                match self.feed.offer(ns) {
                    SimClockVerdict::First { .. } => Some(ClockEvent::Epoch { ns }),
                    SimClockVerdict::Forward { delta_ns } => Some(self.step(delta_ns)),
                    SimClockVerdict::Equal => Some(self.step(0)),
                    SimClockVerdict::Backward { by_ns } => {
                        self.diagnostics.note_backwards(
                            self.graph_name,
                            last_ns.unwrap_or(0),
                            by_ns,
                        );
                        None
                    }
                }
            }
        };
        match event {
            Some(event) => {
                self.on_event(event);
                Ok(FeederPoll::Event(event))
            }
            None => {
                // Nothing moved graph time, so every kind of non-advance
                // counts as silence for the two clock warnings.
                self.on_silence();
                Ok(if idle {
                    FeederPoll::Idle
                } else {
                    FeederPoll::Skipped
                })
            }
        }
    }

    /// How many sequence gaps the source has seen over the run, printed or
    /// not ([`ClockSource::sequence_gaps`]).
    pub fn sequence_gaps(&self) -> u64 {
        self.source.sequence_gaps()
    }

    fn step(&mut self, delta_ns: u64) -> ClockEvent {
        self.seq += 1;
        ClockEvent::Step {
            seq: self.seq,
            delta_ns,
        }
    }

    fn on_event(&mut self, event: ClockEvent) {
        self.diagnostics.note_sample_accepted(self.graph_name);
        if matches!(event, ClockEvent::Epoch { .. }) {
            self.epoch_seen = true;
        }
        // A step is read only after the epoch, by a poll that armed the
        // watch first.
        if let Some(stall) = self.stall.as_mut() {
            if stall.sample_accepted() {
                tracing::info!(
                    graph = %self.graph_name,
                    topic = SIM_CLOCK_TOPIC,
                    "clock samples are arriving again"
                );
            }
        }
    }

    fn on_silence(&mut self) {
        // After the epoch the watch is always armed: this poll armed it.
        match self.stall.as_mut() {
            // A rejected frame is not a clock either: a stream of them still
            // reaches the no-clock warning, next to its own rejection warning.
            None => {
                if self.no_clock.poll(self.started.elapsed(), false).is_some() {
                    tracing::warn!(
                        graph = %self.graph_name,
                        topic = SIM_CLOCK_TOPIC,
                        key = SIM_TIME_KEY,
                        "{NO_CLOCK_WARNING}"
                    );
                }
            }
            Some(stall) => {
                if stall.poll_idle().is_some() {
                    tracing::warn!(
                        graph = %self.graph_name,
                        topic = SIM_CLOCK_TOPIC,
                        last_ns = self.feed.last_ns().unwrap_or(0),
                        "{CLOCK_STALLED_WARNING}"
                    );
                }
            }
        }
    }
}

/// The applier: moves the graph from [`ClockEvent`]s alone. It places the
/// clock and builds the runtime on the epoch, then steps the runtime once per
/// step event. It reads no clock source and judges no sample.
pub struct ClockApplier<'a, B>
where
    B: FnOnce() -> CliResult<GraphRuntime>,
{
    graph_name: &'a str,
    clock: &'a VirtualClock,
    build: Option<B>,
    runtime: Option<GraphRuntime>,
}

impl<'a, B> ClockApplier<'a, B>
where
    B: FnOnce() -> CliResult<GraphRuntime>,
{
    /// An applier that builds with `build` over `clock` on the epoch.
    pub fn new(graph_name: &'a str, clock: &'a VirtualClock, build: B) -> Self {
        Self {
            graph_name,
            clock,
            build: Some(build),
            runtime: None,
        }
    }

    /// Apply one event. The epoch builds the runtime (once); a step advances
    /// it. A step before the epoch, or a second epoch, moves nothing: the
    /// feeder emits neither.
    pub fn apply(&mut self, event: ClockEvent) -> CliResult<()> {
        match event {
            ClockEvent::Epoch { ns } => {
                let Some(build) = self.build.take() else {
                    return Ok(());
                };
                // Place the clock BEFORE the build: `Scheduler::add_node`
                // anchors every period deadline at `now + interval` and every
                // deadline window at `now`.
                self.clock.set(ns);
                let mut runtime = build()?;
                runtime.set_period_fire_ceiling(Some(SIM_TIME_PERIOD_FIRES_PER_SAMPLE));
                tracing::info!(
                    graph = %self.graph_name,
                    topic = SIM_CLOCK_TOPIC,
                    epoch_ns = ns,
                    "graph running on sim time: the first clock sample is the epoch; press Ctrl+C to stop"
                );
                self.runtime = Some(runtime);
            }
            ClockEvent::Step { delta_ns, .. } => {
                if let Some(runtime) = self.runtime.as_mut() {
                    runtime.step(Duration::from_nanos(delta_ns));
                }
            }
        }
        Ok(())
    }

    /// Whether a node of the built runtime asked the graph to shut down.
    fn shutdown_requested(&self) -> bool {
        self.runtime
            .as_ref()
            .is_some_and(GraphRuntime::shutdown_requested)
    }

    /// The built runtime, `None` when no epoch arrived.
    fn into_runtime(self) -> Option<GraphRuntime> {
        self.runtime
    }
}

/// Run a graph on sim time until `running` clears or a node requests
/// shutdown. Returns the runtime for the caller to harvest and shut down, or
/// `None` when the run ended before the first clock sample (nothing was
/// built).
///
/// The loop only pumps: the [`ClockFeeder`] turns frames into events and the
/// [`ClockApplier`] turns events into graph steps.
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
        source,
        no_clock_grace,
        idle,
    } = run;
    let mut feeder = ClockFeeder::new(graph_name, source, no_clock_grace);
    let mut applier = ClockApplier::new(graph_name, clock, build);
    while running.load(Ordering::Relaxed) {
        if applier.shutdown_requested() {
            tracing::info!(graph = %graph_name, "node requested graph shutdown");
            break;
        }
        match feeder.poll()? {
            FeederPoll::Event(event) => applier.apply(event)?,
            FeederPoll::Skipped => {}
            FeederPoll::Idle => std::thread::sleep(idle),
        }
    }
    // The run's gap total, once, so the count is observable without
    // reading the warnings; silence when the stream never skipped.
    let total_failures = feeder.sequence_gaps();
    if total_failures > 0 {
        tracing::info!(
            graph = %graph_name,
            topic = SIM_CLOCK_TOPIC,
            total_failures,
            "the run ended with sequence gaps on the clock topic: this many times the frames read skipped numbers (a hint of lost samples; see the warnings above)"
        );
    }
    Ok(applier.into_runtime())
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

    /// An idle poll (nothing waiting, a rejected frame, or a refused sample):
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
    use cerulion_core::graph::node::NodeEntry;
    use indexmap::IndexMap;
    use std::collections::VecDeque;
    use std::sync::atomic::AtomicU64;

    // -----------------------------------------------------------------
    // ros2: entries under sim time
    // -----------------------------------------------------------------

    fn sim_argv(def: &Ros2NodeDef) -> Vec<String> {
        let mut argv = def.argv(None);
        put_ros2_entry_on_sim_time(def, &mut argv);
        argv
    }

    #[test]
    fn a_ros2_run_entry_gets_use_sim_time_in_a_new_ros_args_block() {
        let def = Ros2NodeDef {
            package: Some("pkg".to_string()),
            executable: Some("exe".to_string()),
            args: vec!["--log-level".to_string(), "warn".to_string()],
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&def),
            [
                "run",
                "pkg",
                "exe",
                "--log-level",
                "warn",
                "--ros-args",
                "-p",
                "use_sim_time:=true"
            ]
        );
    }

    #[test]
    fn a_ros2_run_entry_with_params_gets_use_sim_time_in_its_own_block() {
        let mut params = IndexMap::new();
        params.insert("rate".to_string(), serde_yaml::Value::from(5));
        let def = Ros2NodeDef {
            package: Some("pkg".to_string()),
            executable: Some("exe".to_string()),
            params,
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&def),
            [
                "run",
                "pkg",
                "exe",
                "--ros-args",
                "-p",
                "rate:=5",
                "-p",
                "use_sim_time:=true"
            ]
        );
    }

    #[test]
    fn a_ros2_launch_entry_gets_the_use_sim_time_launch_argument() {
        let def = Ros2NodeDef {
            launch: Some("bringup.launch.py".to_string()),
            args: vec!["use_rviz:=false".to_string()],
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&def),
            [
                "launch",
                "bringup.launch.py",
                "use_rviz:=false",
                "use_sim_time:=true"
            ]
        );
    }

    #[test]
    fn a_use_sim_time_the_entry_sets_itself_is_kept() {
        let mut params = IndexMap::new();
        params.insert("use_sim_time".to_string(), serde_yaml::Value::from(false));
        let run = Ros2NodeDef {
            package: Some("pkg".to_string()),
            executable: Some("exe".to_string()),
            params,
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&run),
            [
                "run",
                "pkg",
                "exe",
                "--ros-args",
                "-p",
                "use_sim_time:=false"
            ]
        );
        let launch = Ros2NodeDef {
            launch: Some("bringup.launch.py".to_string()),
            args: vec!["use_sim_time:=false".to_string()],
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&launch),
            ["launch", "bringup.launch.py", "use_sim_time:=false"]
        );
    }

    /// The one decision names what it added, so the spawn log can report
    /// it: the launch argument (which only the launch file can carry to its
    /// nodes, hence the notice), the parameter (which always reaches the
    /// node), or nothing for an entry that sets `use_sim_time` itself.
    #[test]
    fn putting_an_entry_on_sim_time_names_what_it_added() {
        let launch = Ros2NodeDef {
            launch: Some("bringup.launch.py".to_string()),
            ..Default::default()
        };
        let mut argv = launch.argv(None);
        assert_eq!(
            put_ros2_entry_on_sim_time(&launch, &mut argv),
            SimTimeArgument::LaunchArgument,
            "the launch argument works only if the launch file declares it, so the spawn says so"
        );
        let own_value = Ros2NodeDef {
            launch: Some("bringup.launch.py".to_string()),
            args: vec!["use_sim_time:=false".to_string()],
            ..Default::default()
        };
        let before = own_value.argv(None);
        let mut argv = before.clone();
        assert_eq!(
            put_ros2_entry_on_sim_time(&own_value, &mut argv),
            SimTimeArgument::KeptEntryValue,
            "the entry's own value is kept as given, so there is nothing to declare"
        );
        assert_eq!(argv, before, "nothing was added");
        let run = Ros2NodeDef {
            package: Some("pkg".to_string()),
            executable: Some("exe".to_string()),
            ..Default::default()
        };
        let mut argv = run.argv(None);
        assert_eq!(
            put_ros2_entry_on_sim_time(&run, &mut argv),
            SimTimeArgument::Parameter,
            "a `-p` parameter on `ros2 run` always reaches the node"
        );
        assert!(
            ROS2_LAUNCH_USE_SIM_TIME_NOTICE.contains("use_sim_time:=true")
                && ROS2_LAUNCH_USE_SIM_TIME_NOTICE.contains("declares"),
            "names the argument and what the launch file must do: {ROS2_LAUNCH_USE_SIM_TIME_NOTICE}"
        );
    }

    /// A `ros2 run` entry with a params file: the run cannot read the file,
    /// so its parameter goes right after `--ros-args` and AHEAD of
    /// `--params-file`. In rcl the later rule wins, so a `use_sim_time` the
    /// file sets is the entry's own value and overrides the run's; a file
    /// that does not set it leaves the run's in force. The entry's `params:`
    /// still follow the file, as `Ros2NodeDef::argv` places them.
    #[test]
    fn a_params_file_entry_gets_the_parameter_ahead_of_the_file_so_the_file_wins() {
        let mut params = IndexMap::new();
        params.insert("rate".to_string(), serde_yaml::Value::from(5));
        let def = Ros2NodeDef {
            package: Some("pkg".to_string()),
            executable: Some("exe".to_string()),
            args: vec![
                "--ros-args".to_string(),
                "-r".to_string(),
                "a:=b".to_string(),
            ],
            params_file: Some("params.yaml".to_string()),
            params,
            ..Default::default()
        };
        assert_eq!(
            sim_argv(&def),
            [
                "run",
                "pkg",
                "exe",
                "--ros-args",
                "-r",
                "a:=b",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "--params-file",
                "params.yaml",
                "-p",
                "rate:=5"
            ],
            "the parameter sits in the entry's LAST `--ros-args` block, ahead of the file"
        );
    }

    #[test]
    fn the_spawn_plan_puts_ros2_entries_on_sim_time_only_under_sim_time() {
        use crate::ros2_graph::{spawn_plan, Ros2Entry};
        let root = tempfile::tempdir().expect("tempdir");
        let entry = Ros2Entry {
            id: "talker".to_string(),
            def: Ros2NodeDef {
                package: Some("demo_nodes_cpp".to_string()),
                executable: Some("talker".to_string()),
                ..Default::default()
            },
        };
        let wall = ["run", "demo_nodes_cpp", "talker"];
        for source in [TimeSource::Real, TimeSource::Virtual] {
            assert_eq!(
                spawn_plan(&entry, root.path(), source).expect("argv").argv,
                wall,
                "{source:?}"
            );
        }
        assert_eq!(
            spawn_plan(&entry, root.path(), TimeSource::External)
                .expect("argv")
                .argv,
            [
                "run",
                "demo_nodes_cpp",
                "talker",
                "--ros-args",
                "-p",
                "use_sim_time:=true"
            ]
        );
    }

    #[test]
    fn the_clock_guide_states_the_tap_depth() {
        let guide = include_str!("../../../docs/time.md").replace('\n', " ");
        assert!(
            guide.contains(&format!("a queue of {CLOCK_TAP_DEPTH} samples")),
            "docs/time.md must state CLOCK_TAP_DEPTH"
        );
    }

    /// The guide says what a sequence gap means and what it does not: a
    /// sample the queue evicted always leaves one, and a gap alone does not
    /// prove a loss, so a reader neither ignores the warning nor overreads it.
    #[test]
    fn the_clock_guide_says_a_sequence_gap_is_a_hint_not_proof() {
        let guide = include_str!("../../../docs/time.md")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        for phrase in [
            "evicts the oldest unread sample",
            "skip sequence numbers",
            "A gap is a hint, not proof of a lost sample",
            "can consume a sequence number without sending a frame",
        ] {
            assert!(
                guide.contains(phrase),
                "docs/time.md must say {phrase:?} next to the queue depth"
            );
        }
    }

    /// One fire per accepted sample is how the run behaves today, not part of
    /// the sim time contract: both user pages say so next to the rule, so a
    /// reader does not build on it.
    #[test]
    fn the_docs_state_the_fire_ceiling_as_current_behaviour_not_a_contract() {
        // The sentence wraps across indented list lines: one space per gap.
        let guide = include_str!("../../../docs/time.md")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        assert!(
            guide.contains("This is how the run behaves today, not a rule to build on"),
            "docs/time.md rule 3 must mark the one fire per sample as current behaviour"
        );
        let api = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs/user-api.md"),
        )
        .expect("docs/user-api.md is readable from the crate root");
        let row = api
            .lines()
            .find(|line| line.starts_with("| `sim_time:` |"))
            .expect("docs/user-api.md carries the sim_time row");
        assert!(
            row.contains("current behaviour, not a contract"),
            "the sim_time row must mark the one fire per sample as current behaviour: {row}"
        );
    }

    // -----------------------------------------------------------------
    // ClockFeeder: events from frames
    // -----------------------------------------------------------------

    /// A source that hands out a fixed list of reads, then `Empty`.
    struct ListedReads(VecDeque<ClockRead>);

    impl ClockSource for ListedReads {
        fn read(&mut self) -> CliResult<ClockRead> {
            Ok(self.0.pop_front().unwrap_or(ClockRead::Empty))
        }
    }

    #[test]
    fn the_feeder_emits_the_epoch_then_one_numbered_step_per_accepted_sample() {
        let reads = [
            ClockRead::Empty,
            ClockRead::Rejected(ClockRejection::Undersized { size_bytes: 1 }),
            ClockRead::Sample(1_000),
            ClockRead::Sample(1_500),
            ClockRead::Sample(1_500),
            ClockRead::Sample(1_200),
            ClockRead::Sample(4_000),
            ClockRead::Empty,
        ];
        let mut feeder = ClockFeeder::new(
            "g",
            ListedReads(reads.into_iter().collect()),
            Duration::from_secs(3600),
        );
        let polls: Vec<FeederPoll> = (0..reads.len())
            .map(|_| feeder.poll().expect("poll"))
            .collect();
        use ClockEvent::{Epoch, Step};
        use FeederPoll::{Event, Idle, Skipped};
        assert_eq!(
            polls,
            [
                Idle,
                Skipped,
                Event(Epoch { ns: 1_000 }),
                Event(Step {
                    seq: 1,
                    delta_ns: 500
                }),
                // Equal: a zero step, so data still drains.
                Event(Step {
                    seq: 2,
                    delta_ns: 0
                }),
                // Backward: refused, no event, and no step number used.
                Skipped,
                // The step after a refusal advances from the last ACCEPTED
                // sample (1500), not from the refused one.
                Event(Step {
                    seq: 3,
                    delta_ns: 2_500
                }),
                Idle,
            ]
        );
    }

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
            params: Default::default(),
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

    /// The metadata lookup `graph run` hands the refusal, answered from
    /// in-memory entries: a node without an entry has no answer.
    fn info_of(
        factories: &IndexMap<String, Box<dyn NodeEntry>>,
    ) -> impl Fn(&NodeDef) -> Option<CliResult<NodeInfo>> + '_ {
        |node| factories.get(&node.id).map(|e| Ok(e.info()?))
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
            external_policy_node_ids(&config, info_of(&factories)).expect("metadata reads"),
            ["camera", "imu"],
            "every external node, in declaration order, and no other"
        );
        let err = refuse_external_nodes_under_sim_time("lab", &config, info_of(&factories))
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
        assert!(external_policy_node_ids(&config, info_of(&factories))
            .expect("metadata reads")
            .is_empty());
        refuse_external_nodes_under_sim_time("lab", &config, info_of(&factories))
            .expect("a graph without external nodes runs");
    }

    #[test]
    fn a_node_without_a_loaded_entry_is_left_to_the_build() {
        // The refusal judges the entries it has; a missing one is the build's
        // refusal to make, with its own (library not found) text.
        let (config, mut factories) = sim_graph(&[("camera", MacroPolicy::External)]);
        factories.clear();
        assert!(external_policy_node_ids(&config, info_of(&factories))
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

    /// The delivered length of a frame that is exactly one wire header and
    /// one clock body, whatever the header's size is.
    const HEADER_AND_CLOCK_LEN: usize = WireHeader::SIZE + std::mem::size_of::<ClockBytes>();

    #[test]
    fn a_well_framed_clock_frame_yields_its_hash_and_body() {
        let (header, frame) = framed(HEADER_AND_CLOCK_LEN as u32, HEADER_AND_CLOCK_LEN);
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
        let one_short = WireHeader::SIZE as u32 - 1;
        let (short, frame) = framed(one_short, 64);
        assert_eq!(
            clock_frame_body(Some(short), &frame),
            Err(ClockRejection::BadFraming {
                total_size: Some(one_short),
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
        let (header, frame) = framed(WireHeader::SIZE as u32, WireHeader::SIZE);
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
                params: Default::default(),
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

    /// The events a feeder makes of `SCHEDULE` with no graph attached: the
    /// epoch, then one numbered step per accepted sample. Hand-written from
    /// the schedule table: the equal sample is a zero step and the refused
    /// (backward) sample leaves no event and uses no step number.
    const SCHEDULE_EVENTS: [ClockEvent; 6] = [
        ClockEvent::Epoch { ns: 1_000_000_000 },
        ClockEvent::Step {
            seq: 1,
            delta_ns: 5_000_000,
        },
        ClockEvent::Step {
            seq: 2,
            delta_ns: 5_000_000,
        },
        ClockEvent::Step {
            seq: 3,
            delta_ns: 0,
        },
        ClockEvent::Step {
            seq: 4,
            delta_ns: 15_000_000,
        },
        ClockEvent::Step {
            seq: 5,
            delta_ns: 5_000_000,
        },
    ];

    /// Collect the feeder's events for `SCHEDULE`, with nothing applying them.
    fn feed_schedule() -> Vec<ClockEvent> {
        let reads = SCHEDULE.iter().map(|&ns| ClockRead::Sample(ns)).collect();
        let mut feeder = ClockFeeder::new("sim", ListedReads(reads), NO_CLOCK_GRACE);
        (0..SCHEDULE.len())
            .filter_map(|_| match feeder.poll().expect("poll") {
                FeederPoll::Event(event) => Some(event),
                FeederPoll::Skipped | FeederPoll::Idle => None,
            })
            .collect()
    }

    /// Apply `events` to a fresh applier that has no feeder; returns the same
    /// triple as [`run_schedule`] (fire times from the trace, fire count,
    /// final graph clock).
    fn apply_events(events: &[ClockEvent]) -> (Vec<u64>, u64, u64) {
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        let build_clock = Arc::clone(&clock);
        let build_fires = Arc::clone(&fires);
        let mut applier = ClockApplier::new("sim", &clock, move || {
            build_period_runtime(build_clock, 10, build_fires)
        });
        for &event in events {
            applier.apply(event).expect("the event applies");
        }
        let mut runtime = applier.into_runtime().expect("the epoch built the runtime");
        let fire_times: Vec<u64> = runtime.trace().iter().map(|e| e.fire_time_ns).collect();
        use cerulion_core::Clock as _;
        let final_ns = clock.now_ns();
        runtime.shutdown();
        (fire_times, fires.load(Ordering::Relaxed), final_ns)
    }

    /// The feeder/applier seam: the feeder alone turns the schedule into the
    /// hand-written event list, and a bare applier fed that list (no source,
    /// no feeder, no loop) fires at the oracle stamps and lands where
    /// [`run_schedule`] lands. A graph that follows a recorded or relayed
    /// event stream therefore steps exactly as one reading `/clock` itself.
    #[test]
    fn the_feeder_events_replayed_through_a_bare_applier_match_the_run() {
        let events = feed_schedule();
        assert_eq!(events, SCHEDULE_EVENTS);
        let applied = apply_events(&events);
        assert_eq!(applied.0, EXPECTED_FIRE_TIMES_NS);
        assert_eq!(applied.1, 3, "three deadlines fall inside the schedule");
        assert_eq!(applied.2, 1_030_000_000, "the last accepted sample");
        assert_eq!(applied, run_schedule());
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

    /// A clock topic that, after `samples`, repeats `tail` on every read (a
    /// rejected frame, or a sample behind the epoch). It never answers
    /// `Empty`, so a warning that only an empty poll could reach is never
    /// printed. It clears `running` after `repeats` tail reads.
    struct RepeatingClock {
        samples: VecDeque<u64>,
        tail: ClockRead,
        repeats: usize,
        running: Arc<AtomicBool>,
    }

    impl ClockSource for RepeatingClock {
        fn read(&mut self) -> CliResult<ClockRead> {
            if let Some(ns) = self.samples.pop_front() {
                return Ok(ClockRead::Sample(ns));
            }
            if self.repeats == 0 {
                self.running.store(false, Ordering::Relaxed);
            } else {
                self.repeats -= 1;
            }
            Ok(self.tail)
        }
    }

    /// A [`RepeatingClock`] whose tail is a rejected frame.
    fn rejecting_clock(samples: &[u64], running: &Arc<AtomicBool>) -> RepeatingClock {
        RepeatingClock {
            samples: samples.iter().copied().collect(),
            tail: ClockRead::Rejected(ClockRejection::WrongSchema { got: 7 }),
            repeats: 3,
            running: Arc::clone(running),
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
        let source = rejecting_clock(&[], &running);
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
        let source = rejecting_clock(&[1_000_000_000], &running);
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
    fn a_stream_of_backward_samples_mid_run_is_a_stall() {
        let running = Arc::new(AtomicBool::new(true));
        let clock = Arc::new(VirtualClock::new());
        let fires = Arc::new(AtomicU64::new(0));
        // A stamp behind the epoch on every read after it: refused, so only
        // a refused sample can reach the stall warning.
        let source = RepeatingClock {
            samples: VecDeque::from([1_000_000_000]),
            tail: ClockRead::Sample(500_000_000),
            repeats: 3,
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
        use cerulion_core::Clock as _;
        assert_eq!(
            clock.now_ns(),
            1_000_000_000,
            "a refused sample is never stored"
        );
        logs_assert(|lines: &[&str]| {
            let stalls = count_lines(lines, "no clock sample has arrived on `/clock` since");
            let returns = count_lines(lines, "clock samples are arriving again");
            if stalls == 1 && returns == 0 {
                Ok(())
            } else {
                Err(format!(
                    "expected one stall warning and no return notice, found {stalls} and {returns}"
                ))
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
        let mut source = ClockTopicSource::new("sim", Arc::clone(&transport));

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

    // -----------------------------------------------------------------
    // Sequence gaps: a hint that the tap lost a sample, loud once, counted
    // -----------------------------------------------------------------

    /// Two publisher identities for the oracles below.
    const PUBLISHER_A: u128 = 0xA;
    const PUBLISHER_B: u128 = 0xB;

    /// Hand-written oracle over one publisher's stream: the first number
    /// sets the baseline, the next number and a repeat are not gaps, a skip
    /// names the numbers skipped (loud the first time, counted after), a
    /// number behind the last is a restarted stream and not a gap, and the
    /// counter wraps with the 32-bit sequence.
    #[test]
    fn the_sequence_gap_watch_names_skipped_numbers_loud_once_then_counted() {
        use RegimeDecision::{Loud, Suppressed};
        use SequenceVerdict::{Gap, InOrder};
        let mut watch = SequenceGapWatch::new();
        let a = PUBLISHER_A;
        let gap = |last_seq, seq, skipped, decision| {
            Gap(SequenceGap {
                last_seq,
                seq,
                skipped,
                decision,
            })
        };
        assert_eq!(
            watch.observe(a, 5),
            InOrder,
            "the first frame is the baseline"
        );
        assert_eq!(watch.observe(a, 6), InOrder, "the next number");
        assert_eq!(watch.observe(a, 6), InOrder, "a repeat skips nothing");
        assert_eq!(
            watch.observe(a, 9),
            gap(6, 9, 2, Loud),
            "7 and 8 were skipped: the first gap is loud"
        );
        assert_eq!(watch.observe(a, 10), InOrder);
        assert_eq!(
            watch.observe(a, 12),
            gap(10, 12, 1, Suppressed { suppressed: 1 }),
            "a second gap is counted, not printed"
        );
        assert_eq!(
            watch.observe(a, 3),
            InOrder,
            "a number behind the last is a restarted publisher's stream, not a gap"
        );
        assert_eq!(watch.observe(a, 4), InOrder, "the new stream continues");
        assert_eq!(
            watch.observe(a, u32::MAX),
            InOrder,
            "a jump of more than half the range is a new stream too"
        );
        assert_eq!(
            watch.observe(a, 0),
            InOrder,
            "the sequence wraps: 0 follows u32::MAX"
        );
        assert_eq!(watch.gaps(), 2, "every gap is counted, printed or not");
    }

    /// The latch's third arm, from its own rule (loud, then counted, then
    /// announced again at each tenfold count), by hand: gap 1 is loud, gaps
    /// 2 to 9 are counted with 1 to 8 suppressed so far, gap 10 is announced
    /// again with the total 10 and 8 suppressed, and gap 11 is counted again
    /// with 9. Each frame `2n` after `2(n - 1)` skips exactly one number.
    #[test]
    fn the_tenth_gap_is_announced_again_with_the_running_total() {
        use RegimeDecision::{Loud, StillFailing, Suppressed};
        let mut watch = SequenceGapWatch::new();
        let a = PUBLISHER_A;
        assert_eq!(watch.observe(a, 0), SequenceVerdict::InOrder);
        let decisions: Vec<RegimeDecision> = (1..=11u32)
            .map(|n| match watch.observe(a, 2 * n) {
                SequenceVerdict::Gap(gap) => {
                    assert_eq!(gap.skipped, 1, "frame {}: one number skipped", 2 * n);
                    gap.decision
                }
                other => panic!("frame {}: a gap was expected, got {other:?}", 2 * n),
            })
            .collect();
        let expected = [
            Loud,
            Suppressed { suppressed: 1 },
            Suppressed { suppressed: 2 },
            Suppressed { suppressed: 3 },
            Suppressed { suppressed: 4 },
            Suppressed { suppressed: 5 },
            Suppressed { suppressed: 6 },
            Suppressed { suppressed: 7 },
            Suppressed { suppressed: 8 },
            StillFailing {
                total: 10,
                suppressed: 8,
            },
            Suppressed { suppressed: 9 },
        ];
        assert_eq!(decisions, expected);
        assert_eq!(watch.gaps(), 11);
    }

    /// Wire sequences are per publisher. A frame from another publisher is
    /// never a gap and starts the count over, so two publishers taking turns
    /// draw no false gap on a switch (A=100, B=5, A=101 would otherwise
    /// read as 95 skipped); a skip inside either one's own run of frames is
    /// still a gap. The cost, stated in the guide: a loss that falls between
    /// two switches hides, so the always-leaves-a-gap claim holds for one
    /// publisher.
    #[test]
    fn a_frame_from_another_publisher_starts_the_count_over_and_is_never_a_gap() {
        use RegimeDecision::{Loud, Suppressed};
        use SequenceVerdict::{Gap, InOrder};
        let (a, b) = (PUBLISHER_A, PUBLISHER_B);
        let mut watch = SequenceGapWatch::new();
        assert_eq!(watch.observe(a, 100), InOrder);
        assert_eq!(
            watch.observe(b, 5),
            InOrder,
            "another publisher: its count starts here"
        );
        assert_eq!(
            watch.observe(a, 101),
            InOrder,
            "back to the first publisher: a new baseline, not 95 skipped"
        );
        assert_eq!(
            watch.observe(a, 103),
            Gap(SequenceGap {
                last_seq: 101,
                seq: 103,
                skipped: 1,
                decision: Loud,
            }),
            "a skip inside one publisher's own frames is a gap"
        );
        assert_eq!(watch.observe(b, 6), InOrder);
        assert_eq!(
            watch.observe(b, 8),
            Gap(SequenceGap {
                last_seq: 6,
                seq: 8,
                skipped: 1,
                decision: Suppressed { suppressed: 1 },
            }),
            "the second publisher's own skip is a gap too"
        );
        assert_eq!(watch.gaps(), 2);

        let mut hidden = SequenceGapWatch::new();
        assert_eq!(hidden.observe(a, 10), InOrder);
        assert_eq!(hidden.observe(b, 11), InOrder);
        assert_eq!(
            hidden.observe(a, 12),
            InOrder,
            "A's 11 was lost between two switches: the loss hides, as the guide says"
        );
        assert_eq!(hidden.gaps(), 0);
    }

    /// The first frame after `rebaseline` reports the numbers it skips apart
    /// from the gap count, and the stream starts over there. The latch is
    /// untouched, so the first gap after it is still the loud one; a
    /// rebaseline that skips nothing, or lands on another publisher's frame,
    /// reports zero.
    #[test]
    fn a_rebaseline_reports_the_skipped_numbers_apart_from_the_gap_count() {
        use SequenceVerdict::{Gap, InOrder, Rebaselined};
        let mut watch = SequenceGapWatch::new();
        let a = PUBLISHER_A;
        assert_eq!(watch.observe(a, 10), InOrder);
        watch.rebaseline();
        assert_eq!(
            watch.observe(a, 14),
            Rebaselined { skipped: 3 },
            "11, 12 and 13 were consumed while the watch looked away"
        );
        assert_eq!(watch.gaps(), 0, "not a gap of the stream");
        assert_eq!(
            watch.observe(a, 15),
            InOrder,
            "the stream continues from 14"
        );
        watch.rebaseline();
        assert_eq!(
            watch.observe(a, 16),
            Rebaselined { skipped: 0 },
            "nothing skipped: nothing to report"
        );
        assert_eq!(
            watch.observe(a, 18),
            Gap(SequenceGap {
                last_seq: 16,
                seq: 18,
                skipped: 1,
                decision: RegimeDecision::Loud,
            }),
            "the first gap of the stream is still the loud one"
        );
        watch.rebaseline();
        assert_eq!(
            watch.observe(PUBLISHER_B, 3),
            Rebaselined { skipped: 0 },
            "another publisher's frame: nothing to compare"
        );
        assert_eq!(watch.gaps(), 1);
    }

    /// A real `/clock` tap with a simulator that outruns it: a burst deeper
    /// than the tap's queue loses samples, and the next frame read after the
    /// loss skips their sequence numbers. The warning prints once; a second
    /// burst is counted and not printed.
    #[test]
    #[tracing_test::traced_test]
    fn the_clock_tap_warns_once_when_its_queue_overflows_and_counts_every_gap() {
        use cerulion_core::transport::TransportConfig;
        use cerulion_core::wire::MaxSliceLen;

        let clock = Arc::new(VirtualClock::new());
        let transport = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("sim_clock_gap_{}", std::process::id()),
                clock: clock.clone(),
                subscriber_buffer_size: 16,
                network: None,
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("test transport");
        let mut source = ClockTopicSource::new("sim", Arc::clone(&transport));
        let mut publisher = transport
            .create_publisher(SIM_CLOCK_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("clock publisher");
        // Sample `i` carries `i` whole seconds, so a read names its sample.
        let mut publish = |i: u32| {
            let mut proxy = publisher.loan_proxy::<Clock>().expect("loan");
            proxy.clock.sec = i as i32;
            proxy.clock.nanosec = 0;
        };
        let read_all = |source: &mut ClockTopicSource| -> Vec<u64> {
            let mut got = Vec::new();
            loop {
                match source.read().expect("read") {
                    ClockRead::Sample(ns) => got.push(ns / 1_000_000_000),
                    ClockRead::Empty => break got,
                    ClockRead::Rejected(why) => panic!("a clock frame was rejected: {why:?}"),
                }
            }
        };
        // Open the tap on the first frame, which sets the sequence baseline.
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
        publish(0);
        assert_eq!(read_all(&mut source), [0]);
        let depth = source
            .subscriber
            .as_ref()
            .expect("opened on the first read")
            .buffer_size();
        assert!(depth >= 2, "the tap holds at least two samples: {depth}");
        let depth = depth as u32;

        // A burst of depth + 3 samples with nothing reading: three are lost.
        for i in 1..=depth + 3 {
            publish(i);
        }
        let got = read_all(&mut source);
        assert_eq!(
            got.len(),
            depth as usize,
            "the tap holds exactly its depth; three samples were lost"
        );
        assert_eq!(got[0], 4, "the oldest unread samples (1, 2, 3) were lost");
        assert_eq!(
            got.last().copied(),
            Some(u64::from(depth + 3)),
            "the latest published sample is kept"
        );
        assert_eq!(source.sequence_gaps(), 1, "one gap: 1, 2 and 3 skipped");

        // A second burst: lost again, counted, not printed again.
        for i in depth + 4..=2 * depth + 7 {
            publish(i);
        }
        assert_eq!(read_all(&mut source).len(), depth as usize);
        assert_eq!(source.sequence_gaps(), 2);
        logs_assert(|lines: &[&str]| {
            let loud = count_lines(lines, "skipped sequence numbers");
            if loud == 1 {
                Ok(())
            } else {
                Err(format!("expected exactly one gap warning, found {loud}"))
            }
        });
    }

    /// A real `/clock` tap over a fresh test transport, with a publisher on
    /// the topic. `tag` keeps the node name apart from the other tap tests.
    fn clock_tap(tag: &str) -> (ClockTopicSource, cerulion_core::CerulionPublisher) {
        use cerulion_core::transport::TransportConfig;
        use cerulion_core::wire::MaxSliceLen;
        let clock = Arc::new(VirtualClock::new());
        let transport = TransportManager::init_for_test(
            TransportConfig {
                node_name: format!("sim_clock_{tag}_{}", std::process::id()),
                clock,
                subscriber_buffer_size: 16,
                network: None,
            },
            cerulion_core::testing::iceoryx_test_config(),
        )
        .expect("test transport");
        let source = ClockTopicSource::new("sim", Arc::clone(&transport));
        let publisher = transport
            .create_publisher(SIM_CLOCK_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("clock publisher");
        (source, publisher)
    }

    /// Publish sample `i` as `i` whole seconds, so a read names its sample.
    fn publish_second(publisher: &mut cerulion_core::CerulionPublisher, i: u32) {
        let mut proxy = publisher.loan_proxy::<Clock>().expect("loan");
        proxy.clock.sec = i as i32;
        proxy.clock.nanosec = 0;
    }

    /// Drain the tap: every sample's whole seconds, in read order, until it
    /// is empty. A rejected frame fails the test.
    fn read_seconds(source: &mut ClockTopicSource) -> Vec<u64> {
        let mut got = Vec::new();
        loop {
            match source.read().expect("read") {
                ClockRead::Sample(ns) => got.push(ns / 1_000_000_000),
                ClockRead::Empty => break got,
                ClockRead::Rejected(why) => panic!("a clock frame was rejected: {why:?}"),
            }
        }
    }

    /// A simulator that outruns the graph BUILD: the epoch is read, more
    /// samples than the queue holds arrive before the next poll (the applier
    /// is building), and the feeder tells the tap to start its count over at
    /// the next frame. That loss prints the build-window warning once and
    /// counts as no gap; the next overrun, after the build, is the stream's
    /// first gap and prints the gap warning, loud. The build line comes
    /// first.
    #[test]
    #[tracing_test::traced_test]
    fn the_clock_tap_reports_samples_lost_to_the_build_apart_from_the_gap_count() {
        let (mut source, mut publisher) = clock_tap("build_window");
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
        publish_second(&mut publisher, 0);
        assert_eq!(
            read_seconds(&mut source),
            [0],
            "the epoch sets the baseline"
        );
        let depth = source
            .subscriber
            .as_ref()
            .expect("opened on the first read")
            .buffer_size() as u32;
        assert!(depth >= 2, "the tap holds at least two samples: {depth}");

        // The build window: depth + 3 samples arrive before the next poll.
        for i in 1..=depth + 3 {
            publish_second(&mut publisher, i);
        }
        source.rebaseline_sequence();
        let got = read_seconds(&mut source);
        assert_eq!(got.len(), depth as usize);
        assert_eq!(got[0], 4, "1, 2 and 3 were lost to the build");
        assert_eq!(
            source.sequence_gaps(),
            0,
            "a loss during the build is reported on its own, not as a gap of the stream"
        );

        // After the build: the stream's first gap.
        for i in depth + 4..=2 * depth + 7 {
            publish_second(&mut publisher, i);
        }
        assert_eq!(read_seconds(&mut source).len(), depth as usize);
        assert_eq!(source.sequence_gaps(), 1);
        logs_assert(|lines: &[&str]| {
            let build = lines
                .iter()
                .position(|l| l.contains("while the graph was being built"));
            let gap = lines
                .iter()
                .position(|l| l.contains("skipped sequence numbers"));
            let builds = count_lines(lines, "while the graph was being built");
            let gaps = count_lines(lines, "skipped sequence numbers");
            match (build, gap) {
                (Some(b), Some(g)) if builds == 1 && gaps == 1 && b < g => Ok(()),
                _ => Err(format!(
                    "expected one build-window warning before one gap warning, found {builds} \
                     and {gaps} at {build:?} and {gap:?}"
                )),
            }
        });
    }

    /// A frame on `/clock` that is not a clock sample consumed its sequence
    /// number all the same: the tap counts it for the stream, so the good
    /// frame after it is in order and no gap is reported. Without that, the
    /// 0 to 2 step would read as one number skipped.
    #[test]
    fn a_rejected_frame_moves_the_sequence_baseline() {
        use native_ros2_messages::geometry_msgs::Vector3;
        let (mut source, mut publisher) = clock_tap("rejected_baseline");
        assert_eq!(source.read().expect("read"), ClockRead::Empty);
        publish_second(&mut publisher, 0);
        {
            // Another type on the clock topic: a wrong-schema frame.
            let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
            proxy.x = 1.0;
        }
        publish_second(&mut publisher, 2);
        assert_eq!(source.read().expect("read"), ClockRead::Sample(0));
        assert!(
            matches!(
                source.read().expect("read"),
                ClockRead::Rejected(ClockRejection::WrongSchema { .. })
            ),
            "the second frame is not a clock sample"
        );
        assert_eq!(
            source.read().expect("read"),
            ClockRead::Sample(2_000_000_000)
        );
        assert_eq!(
            source.sequence_gaps(),
            0,
            "the rejected frame's number was consumed, so 2 follows it in order"
        );
    }

    /// A scripted source that reports a gap total of its own.
    struct GappyClock {
        inner: ScriptedClock,
        gaps: u64,
    }

    impl ClockSource for GappyClock {
        fn read(&mut self) -> CliResult<ClockRead> {
            self.inner.read()
        }

        fn sequence_gaps(&self) -> u64 {
            self.gaps
        }
    }

    /// The run logs its source's gap total once at the end, as a counter
    /// (`total_failures`), and nothing when the stream never skipped.
    #[test]
    #[tracing_test::traced_test]
    fn the_run_logs_the_sequence_gap_total_once_at_its_end() {
        for gaps in [3u64, 0] {
            let running = Arc::new(AtomicBool::new(true));
            let clock = Arc::new(VirtualClock::new());
            let fires = Arc::new(AtomicU64::new(0));
            let source = GappyClock {
                inner: ScriptedClock {
                    reads: SCHEDULE.iter().map(|&ns| ClockRead::Sample(ns)).collect(),
                    running: Arc::clone(&running),
                },
                gaps,
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
            runtime.expect("a runtime was built").shutdown();
        }
        logs_assert(|lines: &[&str]| {
            let totals: Vec<&&str> = lines
                .iter()
                .filter(|l| l.contains("ended with sequence gaps"))
                .collect();
            match totals.as_slice() {
                [line] if line.contains("total_failures=3") => Ok(()),
                _ => Err(format!(
                    "expected one end-of-run total line with total_failures=3 (none for the \
                     run without gaps), found {totals:?}"
                )),
            }
        });
    }
}
