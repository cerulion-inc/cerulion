// SPDX-License-Identifier: AGPL-3.0-only
//! The RUN registry `/__cerulion/runs`.
//!
//! # Why this exists
//!
//! `cerulion bag record` must be able to find a LIVE run with no argument, and
//! `cerulion replay` must be able to say which run a bag belongs to. Nothing in
//! the tree carries a run identity today: `recorder.json` describes the HOST
//! (arch/os/version), the trace rings describe a PROCESS, and neither answers
//! "which invocation of `graph run` produced this?".
//!
//! So every `graph run` publishes a small RECORD saying *what it is* and *where
//! its run directory lives*, and anyone who wants to record it, describe it, or
//! bind their lifetime to it GATHERS that record. The bulk content (the
//! effective graph, the env snapshot, the recorder descriptor) stays on disk in
//! the run dir; the wire carries a pointer plus identity.
//!
//! # Modelled on the mirror registry, deliberately
//!
//! This is [`mirror_registry`](super::mirror_registry) with a different payload:
//! same fixed service name, same magic + version + length-prefixed record
//! discipline, same writer/reader-from-ONE-constant rule (so their static
//! configs cannot drift), same periodic republish so a LATE reader converges,
//! same doorbell so a gather ASKS rather than betting on a timer, and
//! the same [`GatherCompleteness`] verdict so an EMPTY answer is never mistaken
//! for an absence claim. **No new transport concept is introduced.**
//!
//! `GatherCompleteness` is REUSED from `mirror_registry` rather than copied. It
//! is not a mirror concept — it is "what a windowed LISTEN's answer is evidence
//! of", which is exactly as true here. The mirror fold, hoisted into one shared home
//! after it had come to exist twice, is the precedent for refusing a second copy
//! of one predicate.
//!
//! ## The one place it is SIMPLER, and why that matters
//!
//! A mirror writer's record map can legitimately be EMPTY (netd after its last
//! mirror retires), and the mirror registry had to invent a bare PRESENCE frame
//! because a writer holding zero records was SILENT and therefore indistinguishable
//! from a writer nobody heard. **A run writer holds exactly ONE record for its whole
//! life** — it is minted from the run it describes and dies with the process —
//! so the empty-map state is unreachable by construction and no presence frame
//! is needed. There is no `register`/`unregister` here: the only mutation is
//! [`RunRegistry::set_state`], which flips `Live` → `Ending` on the graceful
//! shutdown path. Pinned by
//! `tests::a_run_writer_always_has_exactly_one_record_to_send`.
//!
//! # Invisibility to topic surfaces
//!
//! [`RUN_REGISTRY_SERVICE_NAME`] carries NO `/data` suffix, and
//! `TransportManager::list_topics` derives topics only from `*/data` services —
//! so `cerulion topic list` never surfaces it and `cerulion_bagd`'s live-service
//! enumeration never taps it. Pinned here by
//! `tests::run_registry_service_names_have_no_data_suffix` and, at the recorder,
//! by `cerulion_bagd`'s discovery e2e arm.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use iceoryx2::node::Node;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::ServiceName;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use iceoryx2::service::port_factory::PortFactory as _;

use super::mirror_registry::GatherCompleteness;
use super::CerService;
use crate::error::{TransportError, TransportResult};

/// The fixed iceoryx2 service name of the run registry — canonical
/// `/__cerulion/runs`, under
/// [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX).
/// Deliberately carries NO `/data` suffix (unlike graph topic services
/// `{topic}/data`), so `cerulion topic list` — which enumerates `*/data`
/// services — never surfaces it, and `cerulion_bagd`'s live-topic enumeration
/// never taps it. Both the writer ([`RunRegistry`]) and the reader build their
/// service from this ONE constant, so their static configs cannot drift.
pub const RUN_REGISTRY_SERVICE_NAME: &str = "/__cerulion/runs";

/// The CANONICAL root every run directory lives under:
/// `~/.cerulion/runs`, honoring `CERULION_HOME` (the established isolation knob,
/// which every test in the tree already redirects).
///
/// # Why it lives HERE and not with the writer
///
/// `cerulion_cli_engine::run_dir` WRITES these directories and
/// [`run_dir_root`](Self) is its own root — but a [`RunRecord`] carries a
/// `run_dir` PATH, so anything that dereferences a gathered record has to know
/// which paths are legitimately reachable, and `cerulion_core` cannot depend on
/// the CLI engine. Two derivations of one policy is a known mistake (that is
/// literally how a mirror fold ended up existing twice), so the definition sits in
/// the crate the registry itself lives in — the lowest one both the writer and
/// every reader can reach — and `run_dir::run_dir_root` DELEGATES to it.
///
/// # Errors
///
/// `None` when neither `CERULION_HOME` nor a home directory resolves. Callers
/// treat that as "no run directory may be read", never as "read anything".
#[must_use]
pub fn run_dir_root() -> Option<std::path::PathBuf> {
    if let Some(home) = std::env::var_os("CERULION_HOME") {
        if !home.is_empty() {
            return Some(std::path::PathBuf::from(home).join("runs"));
        }
    }
    Some(dirs::home_dir()?.join(".cerulion").join("runs"))
}

/// The run registry's DOORBELL event service — a reader rings it to
/// ask every live run to republish NOW, so a gather never has to catch a
/// periodic broadcast (the measured lesson: macOS background-QoS timer
/// coalescing charged a nominal 150 ms interval as 1100–1696 ms, longer than
/// the whole gather window). A DISTINCT service name is required, not
/// cosmetic: iceoryx2 keys a service by name, and opening one name under two
/// messaging patterns is an incompatibility error.
pub const RUN_REGISTRY_DOORBELL_SERVICE_NAME: &str = "/__cerulion/runs/event";

/// `max_publishers` the registry is provisioned with — one per LIVE
/// `graph run` on the machine. Sized generously above any realistic count of
/// concurrent runs; nowhere near iceoryx2's port ceilings.
pub const RUN_REGISTRY_MAX_WRITERS: usize = 64;

/// `max_subscribers` — every concurrent READER of the registry.
///
/// **The watcher changed what a reader IS, so the sizing argument is restated.**
/// Originally these were all short-lived one-shot gathers (`cerulion bag
/// record`, `cerulion topic list`-class tooling, a Studio refresh), which hold a
/// slot for at most [`RUN_GATHER_WINDOW`]. A [`RunWatcher`] is the first reader
/// that holds one for the whole life of a RECORDING — hours — so slots are no
/// longer effectively free, and the budget is: one per concurrently-bound
/// recorder, plus whatever gathers happen to overlap.
///
/// Eight is still comfortable for the shipping shape (a robot runs ONE recorder;
/// gathers are transient), and it is deliberately NOT raised: this number is
/// part of the service's STATIC config, and both sides `open_or_create` with it
/// (in this module's one `open_registry_service` helper), so changing it would
/// make a new binary unable to open a service an OLD `graph run` on the same
/// machine had already created.
///
/// What happens AT the ceiling is therefore a shipped behaviour rather than a
/// theoretical one, and it degrades loudly rather than silently: the 9th
/// concurrent reader's open fails, and a recorder whose watcher could not be
/// opened records on WITHOUT a lifetime binding, says so at setup, says so again
/// on its terminal line, and stamps `watch_failed` into the bag's coverage
/// manifest — see `cerulion_bagd`'s `RunBindingCoverage`.
pub const RUN_REGISTRY_MAX_READERS: usize = 8;

/// `subscriber_max_buffer_size` — the per-pass reader queue.
/// Comfortably exceeds [`RUN_REGISTRY_MAX_WRITERS`] so every live run's record
/// lands in a single drain pass.
pub const RUN_REGISTRY_QUEUE_DEPTH: usize = 256;

/// `max_listeners` on the doorbell — one per live registry WRITER
/// (each writer's republish thread waits on its own listener).
pub const RUN_DOORBELL_MAX_LISTENERS: usize = RUN_REGISTRY_MAX_WRITERS;

/// `max_notifiers` on the doorbell. Every WRITER keeps one (it rings
/// its own pump awake at teardown) and every concurrent READER mints one for
/// the duration of a gather.
pub const RUN_DOORBELL_MAX_NOTIFIERS: usize = RUN_REGISTRY_MAX_WRITERS + RUN_REGISTRY_MAX_READERS;

/// How often the background republish thread re-sends this run's
/// record — the FALLBACK cadence (a gather rings the doorbell and is answered
/// without waiting for it), and the bound on how long a [`RunRegistry`] drop
/// can take on the degraded path where no doorbell could be opened.
pub const RUN_REPUBLISH_INTERVAL: Duration = Duration::from_millis(150);

/// The reader's bounded gather window CEILING. The gather stops EARLY
/// the instant it has heard from every live writer, so this is only fully paid
/// in the pathological case of a CRASHED writer whose stale iceoryx2 port
/// lingers un-swept.
pub const RUN_GATHER_WINDOW: Duration = Duration::from_millis(600);

/// How long a [`RunWatcher`] goes without hearing its OWN run
/// before that SILENCE counts as evidence the run is gone.
///
/// **Deliberately generous, because the two errors are not symmetric.** Deciding
/// too LATE costs a recorder a few seconds of recording a run that has already
/// stopped — and a stopped run publishes nothing, so those seconds cost no
/// frames and no disk. Deciding too EARLY truncates a recording of a run that is
/// still executing, which is unrecoverable. So this is sized for the worst
/// scheduling behaviour we have measured, not for the median.
///
/// The number: **33×** [`RUN_REPUBLISH_INTERVAL`]. The writer's pump waits one
/// interval per republish, and a nominal 150 ms wait was MEASURED being
/// charged as 1100–1696 ms under macOS background-QoS timer coalescing — so
/// even a ~10× stretch of every republish leaves ~3× headroom before a healthy
/// run could be called absent. It is const-asserted against that floor below, so
/// shrinking one constant without the other fails the build.
///
/// # When this grace is actually PAID — the exact condition
///
/// [`classify_run_signal`] has two routes that reach the verdict without waiting
/// it out, and BOTH are conditional, so this is the normal cost far more often
/// than an early draft of this doc implied.
///
/// * **Zero live writers.** Exact, and needs no grace — but
///   `live_writers` is [`number_of_publishers`] on the ONE machine-wide
///   `/__cerulion/runs` service, NOT a per-run reading, so it is zero only when
///   NOTHING on the machine is publishing a run record. A crashed run with any
///   other run live — a successor, a concurrent graph, a second recording — does
///   not take it.
///
///   There is no cheaper exact alternative, and the reason is a missing
///   CORRELATION rather than a missing API. iceoryx2 *does* expose the live
///   publisher SET, not merely a count: `DynamicConfig::list_publishers` yields
///   a `PublisherDetails` per live port, carrying a `UniquePublisherId`. What it
///   cannot be joined against is the identity a record carries on the WIRE —
///   this module's own `writer_id`, minted by `mint_writer_id` from the pid, a
///   nanosecond stamp and a process-global sequence, with no relation to the
///   iceoryx2 port id and no way to derive one from the other. So the exact
///   predicate — "every currently-live writer has spoken since ours last did" —
///   has no key to join the heard set and the live set on, and a COUNT alone
///   cannot exclude a writer that spoke and then died: acting on it would call a
///   merely-coalesced live run absent, the unrecoverable error direction.
///
///   Closing that gap is a DESIGN change rather than a platform limit: a writer
///   could publish its own port id inside its record and a reader could then
///   join the two sets. Deliberately not done here — it is a wire-format change
///   whose join is sound only once every writer on the machine emits the new
///   field, which is a much larger commitment than the grace this constant buys.
/// * **A successor replaced it** ([`RUN_REPLACED_GRACE`]) — the restart shape,
///   which is the one an operator actually hits.
///
/// So this full grace is paid by a run that crashes while another unrelated run
/// is live and nothing replaces it.
///
/// [`number_of_publishers`]: iceoryx2::service::static_config::publish_subscribe::StaticConfig
pub const RUN_VANISH_GRACE: Duration = Duration::from_secs(5);

/// The SHORTER silence grace applied when a SUCCESSOR run has
/// announced itself while the watched run has gone quiet — the restart shape.
///
/// # Why a separate, shorter grace exists at all
///
/// The silence grace above is sized for "a run crashed and nothing else says
/// anything", where the only evidence is absence. A restart is a different
/// evidential situation: a record for a DIFFERENT run of the SAME graph, started
/// no earlier than ours, is POSITIVE evidence that something is taking over the
/// topics this recording taps. A consumer bound to the predecessor keeps
/// recording throughout its detection window, and every millisecond of that
/// window is a millisecond in which the SUCCESSOR's frames land in the
/// predecessor's bag (the taps are keyed by topic NAME and hold the iceoryx2
/// service open across a producer swap — the same doctrine the epoch
/// reset exists for). So on this shape the window is not merely a latency, it is
/// the splice channel, and shortening it shrinks the splice.
///
/// # Why it is not ZERO
///
/// A same-graph successor CAN announce while the predecessor is still alive: an
/// operator running `cerulion graph run demo` twice announces the second run
/// (a run announces after preflight, BEFORE the graph builds) and only then
/// fails on the single-writer topic collision. If the predecessor's republishes
/// were being coalesced at that moment, a zero-grace rule would truncate a LIVE
/// recording — the unrecoverable error direction. **2 s is above the whole
/// measured coalescing band** (a nominal 150 ms wait charged as
/// 1100–1696 ms), const-asserted below against `COALESCED_REPUBLISH_CEILING`.
///
/// # What actually carries the margin
///
/// Stated because the headroom on a SINGLE reading is thinner than the 2 s
/// against 1696 ms suggests. `since_own_record` is anchored at the DRAIN of the
/// last record, not at its send, so a consumer polling on its own cadence (the
/// recorder's is `RUN_OBSERVE_INTERVAL`, 250 ms) can measure a coalesced
/// republish gap as roughly `gap + one poll interval` ≈ 1946 ms — 54 ms under
/// this threshold, not 304 ms.
///
/// What keeps that from mattering is [`RUN_ABSENCE_CONFIRMATIONS`] (2): a single
/// reading over the line is not a verdict, and by the next observation the
/// coalesced record has been drained, the streak is cleared, and nothing was
/// decided. So the guard against a healthy run falling through is the
/// CONFIRMATION, with this grace sized to make even the first reading unlikely
/// to cross.
///
/// Const-asserted below to stay at or under [`RUN_VANISH_GRACE`] — this is a
/// SHORTENING of the silence rule on stronger evidence, and a value above it
/// would make a successor sighting DELAY a verdict, which is backwards.
pub const RUN_REPLACED_GRACE: Duration = Duration::from_secs(2);

/// Consecutive [`RunSignal::Absent`] observations required before a
/// [`RunLifetimeTracker`] reports [`RunEnded::Vanished`].
///
/// ONE missed observation must never end a recording.
/// A single reading can be lost to a transient — a drain that returned an error
/// and kept its partial fill, a dynamic-config read racing a port teardown — and
/// the cost of believing it is a truncated bag.
///
/// Applies to ABSENCE only. A heard `Ending` record is POSITIVE evidence and is
/// terminal on its first sighting: there is no transient that fabricates one.
pub const RUN_ABSENCE_CONFIRMATIONS: u32 = 2;

/// The CEILING of the MEASURED timer-coalescing band,
/// rounded up — a nominal 150 ms wait was charged as 1100–1696 ms under macOS
/// background QoS.
///
/// It exists because the "order of magnitude above the republish interval" rule
/// does NOT on its own enforce what its own message claims: `10 ×` 150 ms is
/// 1500 ms, which sits INSIDE the band it is quoted as excluding. Both graces
/// are therefore asserted STRICTLY above this as well, so a future shrink cannot
/// land a grace in the range where a healthy run's coalesced republish looks
/// like silence.
const COALESCED_REPUBLISH_CEILING: Duration = Duration::from_millis(1_700);

const _: () = assert!(
    RUN_VANISH_GRACE.as_millis() >= 10 * RUN_REPUBLISH_INTERVAL.as_millis()
        && RUN_VANISH_GRACE.as_millis() > COALESCED_REPUBLISH_CEILING.as_millis(),
    "the vanish grace must stay an order of magnitude above the republish interval AND \
     strictly above the coalescing ceiling — a nominal 150 ms wait was measured charged as \
     1100-1696 ms under macOS background QoS, and a grace inside that band would call a HEALTHY \
     run vanished and truncate its recording"
);

const _: () = assert!(
    RUN_ABSENCE_CONFIRMATIONS >= 2,
    "absence must be CONFIRMED — acting on a single missed observation is exactly the \
     mutant this constant exists to fail"
);

const _: () = assert!(
    RUN_REPLACED_GRACE.as_millis() >= 10 * RUN_REPUBLISH_INTERVAL.as_millis()
        && RUN_REPLACED_GRACE.as_millis() > COALESCED_REPUBLISH_CEILING.as_millis(),
    "the replaced grace must ALSO stay an order of magnitude above the republish \
     interval AND strictly above the coalescing ceiling — a same-graph successor can \
     announce while the predecessor is still alive (it announces before it builds), so a value \
     inside that band would truncate a LIVE recording on an operator's double-launch typo"
);

const _: () = assert!(
    RUN_REPLACED_GRACE.as_millis() <= RUN_VANISH_GRACE.as_millis(),
    "a successor sighting is STRONGER evidence than silence alone, so it may only \
     SHORTEN the wait — a replaced grace above the vanish grace would make positive evidence \
     DELAY the verdict"
);

/// The record magic (`0xCE`, `0x81`). A frame
/// that does not open with this is not one of ours.
const RUN_MAGIC: [u8; 2] = [0xCE, 0x81];

/// The record wire-format version. Bumped only on an incompatible
/// record layout change; a mismatched version is rejected as malformed.
const RUN_VERSION: u8 = 1;

/// The fixed record header length — `magic(2) + version(1) +
/// writer_id(8) + run_id(16) + supervisor_pid(4) + run_started_at_ns(8) +
/// state(1) + graph_name_len(2) + run_dir_len(2)`. The graph-name bytes then
/// the run-dir bytes follow.
pub const RUN_HEADER_LEN: usize = 2 + 1 + 8 + 16 + 4 + 8 + 1 + 2 + 2;

/// The maximum graph-name length (bytes). Far above any realistic
/// graph name; bounds the record size.
pub const MAX_RUN_GRAPH_NAME_LEN: usize = 256;

/// The maximum run-directory path length (bytes). Comfortably above
/// any realistic path (`PATH_MAX` is 4096 on Linux, 1024 on macOS).
pub const MAX_RUN_DIR_LEN: usize = 1024;

/// The maximum record length — the writer's `initial_max_slice_len`.
pub const MAX_RUN_RECORD_LEN: usize = RUN_HEADER_LEN + MAX_RUN_GRAPH_NAME_LEN + MAX_RUN_DIR_LEN;

// Fixed field offsets, spelled once so encode and decode cannot disagree.
const OFF_VERSION: usize = 2;
const OFF_WRITER_ID: usize = 3;
const OFF_RUN_ID: usize = 11;
const OFF_PID: usize = 27;
const OFF_STARTED: usize = 31;
const OFF_STATE: usize = 39;
const OFF_NAME_LEN: usize = 40;
const OFF_DIR_LEN: usize = 42;

const _: () = assert!(OFF_DIR_LEN + 2 == RUN_HEADER_LEN);

/// What a live run says about itself.
///
/// Deliberately only TWO states. `Live` is the whole run; `Ending` is published
/// once on the GRACEFUL shutdown path, before teardown, so a consumer bound to
/// this run's lifetime can distinguish "the operator stopped it" from "the
/// process vanished" (a crash publishes nothing — the writer simply stops
/// republishing and its port is released, so a subsequent gather does not see
/// it). There is no `Ended`: a run that has ended has no writer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunState {
    /// The run is executing.
    Live,
    /// The run is shutting down gracefully.
    ///
    /// Sent ONCE, synchronously, by [`RunRegistry::set_state`]. Whether the
    /// republish pump ever re-sends it depends entirely on how long the process
    /// lives afterwards — and on the shipping caller
    /// (`cerulion_cli_engine::run_dir::RunDescriptor::drop`) that is
    /// microseconds, so in practice it is sent exactly once and the writer is
    /// gone before the next interval. MEASURED there: a gatherer racing the
    /// exit observed it 7/8 at a hot spin, 0/8 at a 250 ms cadence.
    ///
    /// So this is a LAST WORD, not a state a later reader can look up — only a
    /// consumer already subscribed inside that window hears it (the service
    /// requests no history). The pump alone
    /// "keeps republishing this value until the process exits", which is true
    /// of the pump in isolation and false of every caller that has one.
    Ending,
}

impl RunState {
    /// The wire byte. `0` is deliberately NOT a valid state, so a zeroed frame
    /// is rejected rather than read as `Live`.
    fn to_wire(self) -> u8 {
        match self {
            RunState::Live => 1,
            RunState::Ending => 2,
        }
    }

    fn from_wire(byte: u8) -> Option<Self> {
        match byte {
            1 => Some(RunState::Live),
            2 => Some(RunState::Ending),
            _ => None,
        }
    }
}

impl std::fmt::Display for RunState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunState::Live => write!(f, "live"),
            RunState::Ending => write!(f, "ending"),
        }
    }
}

/// A decoded run record — the pointer plus identity a consumer needs
/// to find a live run's directory and bind to its lifetime.
///
/// The wire ALSO carries a per-writer `writer_id` (see [`decode_record`]) used
/// only by the gather's completeness gate, deliberately NOT surfaced here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunRecord {
    /// The 128-bit run identity, minted once per `graph run` invocation. A
    /// restart mints a new one BY CONSTRUCTION, which is what makes "one bag =
    /// one run" checkable rather than promised.
    pub run_id: u128,
    /// The pid of the process that owns the run (the supervisor on the
    /// multi-process path, the graph process on the monolith path). A
    /// corroborating fast path only — pid recycling makes it insufficient
    /// alone, which is why `run_id` is the primary signal.
    pub supervisor_pid: u32,
    /// Wall-clock ns since the Unix epoch at which the run started. Used for
    /// rendering and for ordering concurrent runs; never for identity.
    pub run_started_at_ns: u64,
    /// What the run currently says about itself.
    pub state: RunState,
    /// The run's graph name (its identity: the file stem, including
    /// on the multi-process path).
    pub graph_name: String,
    /// The absolute path of the run directory holding `run.json`, the effective
    /// `graph.yaml`, `env.json` and `recorder.json`.
    pub run_dir: String,
}

/// Why a run record failed to encode or decode. Every variant is a
/// distinct, testable reason; the reader drain maps ANY decode error to a
/// counted + warn-once + continue (a poisoned/hostile record must never wedge
/// the drain).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunRecordError {
    /// The graph name is empty.
    EmptyGraphName,
    /// The run directory path is empty.
    EmptyRunDir,
    /// The graph name exceeds [`MAX_RUN_GRAPH_NAME_LEN`] bytes.
    GraphNameTooLong { len: usize },
    /// The run directory path exceeds [`MAX_RUN_DIR_LEN`] bytes.
    RunDirTooLong { len: usize },
    /// The frame is shorter than [`RUN_HEADER_LEN`] (truncated header).
    Truncated { len: usize },
    /// The leading magic bytes are not `RUN_MAGIC` (not a Cerulion run record).
    BadMagic,
    /// The version byte is not `RUN_VERSION`.
    BadVersion { found: u8 },
    /// The state byte is not a known [`RunState`] (a zeroed frame lands here).
    BadState { found: u8 },
    /// The declared name+dir lengths disagree with the frame length.
    LengthMismatch {
        declared: usize,
        actual_payload: usize,
    },
    /// The graph-name or run-dir bytes are not valid UTF-8.
    InvalidUtf8,
}

impl std::fmt::Display for RunRecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunRecordError::EmptyGraphName => write!(f, "empty graph name"),
            RunRecordError::EmptyRunDir => write!(f, "empty run directory path"),
            RunRecordError::GraphNameTooLong { len } => write!(
                f,
                "graph name length {len} exceeds the {MAX_RUN_GRAPH_NAME_LEN}-byte limit"
            ),
            RunRecordError::RunDirTooLong { len } => write!(
                f,
                "run directory length {len} exceeds the {MAX_RUN_DIR_LEN}-byte limit"
            ),
            RunRecordError::Truncated { len } => write!(
                f,
                "record is {len} bytes, shorter than the {RUN_HEADER_LEN}-byte header"
            ),
            RunRecordError::BadMagic => write!(f, "bad record magic (not a Cerulion run record)"),
            RunRecordError::BadVersion { found } => write!(
                f,
                "unsupported record version {found} (expected {RUN_VERSION})"
            ),
            RunRecordError::BadState { found } => {
                write!(
                    f,
                    "unknown run state byte {found} (expected 1=live, 2=ending)"
                )
            }
            RunRecordError::LengthMismatch {
                declared,
                actual_payload,
            } => write!(
                f,
                "record declares {declared} name+dir bytes but carries {actual_payload}"
            ),
            RunRecordError::InvalidUtf8 => {
                write!(f, "graph-name/run-dir bytes are not valid UTF-8")
            }
        }
    }
}

/// Encode a `(writer_id, RunRecord)` for the control wire. Pure (no
/// transport); the exact inverse of [`decode_record`]. The `writer_id`
/// distinguishes independent writer processes so a gather knows when it has
/// heard from all of them.
///
/// # Errors
///
/// - [`RunRecordError::EmptyGraphName`] / [`RunRecordError::EmptyRunDir`].
/// - [`RunRecordError::GraphNameTooLong`] / [`RunRecordError::RunDirTooLong`].
pub fn encode_record(writer_id: u64, record: &RunRecord) -> Result<Vec<u8>, RunRecordError> {
    if record.graph_name.is_empty() {
        return Err(RunRecordError::EmptyGraphName);
    }
    if record.run_dir.is_empty() {
        return Err(RunRecordError::EmptyRunDir);
    }
    if record.graph_name.len() > MAX_RUN_GRAPH_NAME_LEN {
        return Err(RunRecordError::GraphNameTooLong {
            len: record.graph_name.len(),
        });
    }
    if record.run_dir.len() > MAX_RUN_DIR_LEN {
        return Err(RunRecordError::RunDirTooLong {
            len: record.run_dir.len(),
        });
    }
    let mut buf =
        Vec::with_capacity(RUN_HEADER_LEN + record.graph_name.len() + record.run_dir.len());
    buf.extend_from_slice(&RUN_MAGIC);
    buf.push(RUN_VERSION);
    buf.extend_from_slice(&writer_id.to_le_bytes());
    buf.extend_from_slice(&record.run_id.to_le_bytes());
    buf.extend_from_slice(&record.supervisor_pid.to_le_bytes());
    buf.extend_from_slice(&record.run_started_at_ns.to_le_bytes());
    buf.push(record.state.to_wire());
    buf.extend_from_slice(&(record.graph_name.len() as u16).to_le_bytes());
    buf.extend_from_slice(&(record.run_dir.len() as u16).to_le_bytes());
    buf.extend_from_slice(record.graph_name.as_bytes());
    buf.extend_from_slice(record.run_dir.as_bytes());
    Ok(buf)
}

/// Decode a control-wire run record — the exact inverse of
/// [`encode_record`]. Returns the `(writer_id, RunRecord)` pair. Pure (no
/// transport). Validates the magic, version, state byte, the declared-vs-actual
/// payload length, and UTF-8, so a truncated / hostile / foreign frame is
/// rejected with a precise reason rather than silently misread.
///
/// # Errors
///
/// Any [`RunRecordError`] variant — each on its own condition (see the variant
/// docs).
pub fn decode_record(bytes: &[u8]) -> Result<(u64, RunRecord), RunRecordError> {
    if bytes.len() < RUN_HEADER_LEN {
        return Err(RunRecordError::Truncated { len: bytes.len() });
    }
    if bytes[0..2] != RUN_MAGIC {
        return Err(RunRecordError::BadMagic);
    }
    if bytes[OFF_VERSION] != RUN_VERSION {
        return Err(RunRecordError::BadVersion {
            found: bytes[OFF_VERSION],
        });
    }
    // Fixed-offset reads (the length check above makes these infallible).
    let writer_id = u64::from_le_bytes(bytes[OFF_WRITER_ID..OFF_RUN_ID].try_into().expect("8"));
    let run_id = u128::from_le_bytes(bytes[OFF_RUN_ID..OFF_PID].try_into().expect("16"));
    let supervisor_pid = u32::from_le_bytes(bytes[OFF_PID..OFF_STARTED].try_into().expect("4"));
    let run_started_at_ns =
        u64::from_le_bytes(bytes[OFF_STARTED..OFF_STATE].try_into().expect("8"));
    let Some(state) = RunState::from_wire(bytes[OFF_STATE]) else {
        return Err(RunRecordError::BadState {
            found: bytes[OFF_STATE],
        });
    };
    let name_len =
        u16::from_le_bytes(bytes[OFF_NAME_LEN..OFF_DIR_LEN].try_into().expect("2")) as usize;
    let dir_len =
        u16::from_le_bytes(bytes[OFF_DIR_LEN..RUN_HEADER_LEN].try_into().expect("2")) as usize;
    if name_len > MAX_RUN_GRAPH_NAME_LEN {
        return Err(RunRecordError::GraphNameTooLong { len: name_len });
    }
    if dir_len > MAX_RUN_DIR_LEN {
        return Err(RunRecordError::RunDirTooLong { len: dir_len });
    }
    let actual_payload = bytes.len() - RUN_HEADER_LEN;
    if name_len + dir_len != actual_payload {
        return Err(RunRecordError::LengthMismatch {
            declared: name_len + dir_len,
            actual_payload,
        });
    }
    let name_end = RUN_HEADER_LEN + name_len;
    let graph_name = std::str::from_utf8(&bytes[RUN_HEADER_LEN..name_end])
        .map_err(|_| RunRecordError::InvalidUtf8)?;
    let run_dir =
        std::str::from_utf8(&bytes[name_end..]).map_err(|_| RunRecordError::InvalidUtf8)?;
    if graph_name.is_empty() {
        return Err(RunRecordError::EmptyGraphName);
    }
    if run_dir.is_empty() {
        return Err(RunRecordError::EmptyRunDir);
    }
    Ok((
        writer_id,
        RunRecord {
            run_id,
            supervisor_pid,
            run_started_at_ns,
            state,
            graph_name: graph_name.to_string(),
            run_dir: run_dir.to_string(),
        },
    ))
}

/// Mint a practically-unique per-[`RunRegistry`]-instance identity,
/// distinct across processes AND across multiple registries in one process
/// (tests). Mixes the pid, a nanosecond timestamp, and a process-global
/// sequence — no external RNG dependency. A collision (astronomically
/// unlikely) only makes a gather UNDER-count writers heard, which is the SAFE
/// direction: it can never early-exit incomplete, only pay the full ceiling.
fn mint_writer_id() -> u64 {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let pid = std::process::id() as u64;
    pid.wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ nanos.rotate_left(23)
        ^ seq.wrapping_mul(0xD1B5_4A32_D192_ED03).rotate_left(37)
}

/// Mint a 128-bit RUN identity. Distinct per invocation by
/// construction (pid + a nanosecond timestamp + a process-global sequence in
/// each half), which is the whole point: a restarted graph mints a new one, so
/// a recorder bound to the old id terminates rather than splicing two runs into
/// one bag.
#[must_use]
pub fn mint_run_id() -> u128 {
    (u128::from(mint_writer_id()) << 64) | u128::from(mint_writer_id())
}

/// Open the ONE registry pub/sub service (`open_or_create`, so
/// whichever side — writer or reader — arrives first CREATES it and the other
/// OPENS it). Both sides call THIS helper, so the static config is identical
/// and `open_or_create` is always compatible.
fn open_registry_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    let name: ServiceName =
        RUN_REGISTRY_SERVICE_NAME
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "run registry: invalid control service name \
                     '{RUN_REGISTRY_SERVICE_NAME}': {e:?}"
                ),
            })?;
    node.service_builder(&name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(RUN_REGISTRY_QUEUE_DEPTH)
        .max_subscribers(RUN_REGISTRY_MAX_READERS)
        .max_publishers(RUN_REGISTRY_MAX_WRITERS)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "run registry: could not open the control service \
                 '{RUN_REGISTRY_SERVICE_NAME}': {e:?}"
            ),
        })
}

/// Open the ONE doorbell EVENT service. Callers treat a failure as
/// DEGRADED, never fatal: without a doorbell the writer falls back to its
/// republish interval and the reader to catching it, which is slow rather than
/// wrong.
fn open_doorbell_service(
    node: &Node<CerService>,
) -> TransportResult<iceoryx2::service::port_factory::event::PortFactory<CerService>> {
    let name: ServiceName =
        RUN_REGISTRY_DOORBELL_SERVICE_NAME
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "run registry: invalid doorbell service name \
                 '{RUN_REGISTRY_DOORBELL_SERVICE_NAME}': {e:?}"
                ),
            })?;
    node.service_builder(&name)
        .event()
        .max_listeners(RUN_DOORBELL_MAX_LISTENERS)
        .max_notifiers(RUN_DOORBELL_MAX_NOTIFIERS)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "run registry: could not open the doorbell service \
                 '{RUN_REGISTRY_DOORBELL_SERVICE_NAME}': {e:?}"
            ),
        })
}

/// A writer's doorbell port pair — the listener its republish thread
/// waits on, and the notifier its teardown rings to wake that thread.
struct DoorbellPorts {
    listener: iceoryx2::port::listener::Listener<CerService>,
    notifier: iceoryx2::port::notifier::Notifier<CerService>,
}

/// Mint a WRITER's doorbell ports. BEST-EFFORT: every failure arm
/// returns `None` after ONE loud `warn!` naming what is lost, because failing
/// the run over a latency optimisation would trade a slow answer for no run at
/// all.
fn open_writer_doorbell(node: &Node<CerService>) -> Option<DoorbellPorts> {
    let degraded = |what: &str, reason: String| -> Option<DoorbellPorts> {
        tracing::warn!(
            reason = %reason,
            what = what,
            "run registry: could not create a doorbell port (the `what` field names \
             which) — this run still republishes its record every {RUN_REPUBLISH_INTERVAL:?}, but \
             a gather can no longer ASK it to speak and must CATCH that broadcast inside its \
             window instead",
        );
        None
    };
    let service = match open_doorbell_service(node) {
        Ok(s) => s,
        Err(e) => return degraded("service", format!("{e}")),
    };
    let listener = match service.listener_builder().create() {
        Ok(l) => l,
        Err(e) => return degraded("listener", format!("{e:?}")),
    };
    let notifier = match service.notifier_builder().create() {
        Ok(n) => n,
        Err(e) => return degraded("notifier", format!("{e:?}")),
    };
    Some(DoorbellPorts { listener, notifier })
}

/// The shared, thread-safe core of a [`RunRegistry`].
struct RunRegistryInner {
    publisher: Publisher<CerService, [u8], ()>,
    /// This writer instance's stable identity, stamped into every record so a
    /// gather can count the distinct writers it has heard from.
    writer_id: u64,
    /// THE record. Exactly one, for this writer's whole life — see the module
    /// docs on why no presence frame is needed here.
    record: Mutex<RunRecord>,
    /// First send failure of a run logs `warn!`, repeats `debug!`; a success
    /// re-arms.
    send_warned: AtomicBool,
    /// Doorbell rings this writer's pump has ANSWERED (Principle #3).
    /// The observable that separates "a gather settled because it ASKED" from
    /// "a gather settled because it happened to catch the timer" — a
    /// distinction no wall-clock assertion can make on a loaded runner.
    doorbell_rings: AtomicU64,
    /// First doorbell WAIT failure logs `warn!`, repeats `debug!`.
    doorbell_warned: AtomicBool,
}

impl RunRegistryInner {
    /// Encode + loan + send THE record. Returns whether the send succeeded.
    fn send_record(&self) -> bool {
        let snapshot = self
            .record
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone();
        let bytes = match encode_record(self.writer_id, &snapshot) {
            Ok(b) => b,
            Err(e) => {
                // Unreachable from the public API: `RunRegistry::open` validates
                // the record before the registry exists. We still handle the
                // `Err` (never send garbage) and flag any regression.
                debug_assert!(
                    false,
                    "send_record hit an unencodable record ({e}) — the constructor's \
                     guards were bypassed"
                );
                tracing::warn!(
                    error = %e,
                    "run registry: refusing to send an unencodable record"
                );
                return false;
            }
        };
        match self.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => match sample.write_from_slice(&bytes).send() {
                Ok(_) => {
                    self.record_send_success();
                    true
                }
                Err(e) => {
                    self.record_send_failure(&format!("{e}"));
                    false
                }
            },
            Err(e) => {
                self.record_send_failure(&format!("{e}"));
                false
            }
        }
    }

    fn record_send_failure(&self, reason: &str) {
        if self.send_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                reason = %reason,
                "run registry: run-record send failed (repeat — see the first warning)"
            );
        } else {
            tracing::warn!(
                reason = %reason,
                "run registry: run-record send failed — this run will retry on the next \
                 republish, but until it lands a recorder cannot discover this run (repeats log \
                 at debug)"
            );
        }
    }

    fn record_send_success(&self) {
        if self.send_warned.swap(false, Ordering::Relaxed) {
            tracing::info!("run registry: run-record send recovered — republishing again");
        }
    }

    fn note_doorbell_ring(&self) {
        self.doorbell_rings.fetch_add(1, Ordering::Relaxed);
    }

    fn record_doorbell_failure(&self, reason: &str) {
        if self.doorbell_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                reason = %reason,
                "run registry: doorbell wait failed (repeat — see the first warning)"
            );
        } else {
            tracing::warn!(
                reason = %reason,
                "run registry: doorbell wait failed — this run falls back to republishing \
                 every {RUN_REPUBLISH_INTERVAL:?}, so a gather must CATCH that broadcast instead \
                 of being answered (repeats log at debug)"
            );
        }
    }
}

/// A joined-on-drop handle for the background republish thread.
struct PumpHandle {
    exit: Arc<AtomicBool>,
    /// This writer's OWN doorbell notifier, rung at teardown to wake the pump
    /// out of its wait — which is what lets the pump wait ONCE for a whole
    /// interval instead of polling in short chunks (the timer-coalescing lesson).
    wake: Option<iceoryx2::port::notifier::Notifier<CerService>>,
    join: Option<JoinHandle<()>>,
}

impl Drop for PumpHandle {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        if let Some(wake) = &self.wake {
            let _ = wake.notify();
        }
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// The WRITER side of the run registry — ONE per live `graph run`.
/// Owns the control publisher, this writer's stable identity, THE record, and
/// the background republish thread. When the process exits (this drops),
/// republication stops and the run's record expires: a subsequent gather does
/// not see it.
///
/// `pump` is declared BEFORE `inner` so it drops FIRST, joining the republish
/// thread before the `inner` Arc is released.
pub struct RunRegistry {
    pump: Mutex<Option<PumpHandle>>,
    inner: Arc<RunRegistryInner>,
}

impl RunRegistry {
    /// Open the registry on `node`, publish `record` immediately (the
    /// reader-already-up fast path), and start the background republish thread
    /// (the belt that lets a later gather converge).
    ///
    /// # Errors
    ///
    /// [`TransportError::Internal`] if the record is unencodable (empty or
    /// over-long graph name / run dir — reported with the exact
    /// [`RunRecordError`]), or if the control service / publisher cannot be
    /// created.
    pub fn open(node: &Node<CerService>, record: RunRecord) -> TransportResult<Self> {
        // Validate BEFORE opening anything: a record that cannot be encoded
        // would leave a live publisher that says nothing, which is exactly the
        // silent state this registry exists to prevent.
        let writer_id = mint_writer_id();
        encode_record(writer_id, &record).map_err(|e| TransportError::Internal {
            reason: format!(
                "run registry: refusing to register run '{}' — {e}",
                record.graph_name
            ),
        })?;
        let factory = open_registry_service(node)?;
        let publisher = factory
            .publisher_builder()
            .initial_max_slice_len(MAX_RUN_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "run registry: could not create the control publisher on \
                     '{RUN_REGISTRY_SERVICE_NAME}': {e:?}"
                ),
            })?;
        let doorbell = open_writer_doorbell(node);
        let me = Self {
            pump: Mutex::new(None),
            inner: Arc::new(RunRegistryInner {
                publisher,
                writer_id,
                record: Mutex::new(record),
                send_warned: AtomicBool::new(false),
                doorbell_rings: AtomicU64::new(0),
                doorbell_warned: AtomicBool::new(false),
            }),
        };
        me.inner.send_record();
        me.start_pump(doorbell);
        Ok(me)
    }

    /// Flip the published state and send it IMMEDIATELY (a graceful shutdown
    /// must not wait a republish interval to be heard). Returns whether the
    /// state actually changed — a repeat is a no-op that still re-sends, so a
    /// caller may call it idempotently.
    pub fn set_state(&self, state: RunState) -> bool {
        let changed = {
            let mut record = self.inner.record.lock().unwrap_or_else(|e| e.into_inner());
            let changed = record.state != state;
            record.state = state;
            changed
        };
        self.inner.send_record();
        changed
    }

    /// How many doorbell rings this writer's pump has ANSWERED (Principle #3).
    /// The observable that makes "the gather ASKED and was answered" a
    /// MEASUREMENT rather than an inference — a wall-clock reading cannot
    /// separate a doorbell answer from a lucky timer tick on a loaded runner
    /// (the loaded-runner false-verdict class), and this can.
    #[must_use]
    pub fn doorbell_rings_answered(&self) -> u64 {
        self.inner.doorbell_rings.load(Ordering::Relaxed)
    }

    /// Start the background republish thread. A spawn failure is non-fatal +
    /// loud: the record was ALREADY sent once (a gatherer already up receives
    /// it), but the periodic convergence belt is disabled.
    fn start_pump(&self, doorbell: Option<DoorbellPorts>) {
        let mut pump = self.pump.lock().unwrap_or_else(|e| e.into_inner());
        if pump.is_some() {
            return;
        }
        let exit = Arc::new(AtomicBool::new(false));
        let exit_thread = Arc::clone(&exit);
        let inner = Arc::clone(&self.inner);
        let (listener, wake) = match doorbell {
            Some(DoorbellPorts { listener, notifier }) => (Some(listener), Some(notifier)),
            None => (None, None),
        };
        match std::thread::Builder::new()
            .name("cer-run-republish".to_string())
            .spawn(move || pump_loop(&exit_thread, &inner, listener))
        {
            Ok(join) => {
                *pump = Some(PumpHandle {
                    exit,
                    wake,
                    join: Some(join),
                });
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "run registry: could not spawn the republish thread — the run's \
                     record was still sent once (a gatherer already up receives it), but the \
                     periodic convergence belt is DISABLED: a gather that opens later may not \
                     discover this run"
                );
            }
        }
    }
}

/// The background republish thread body: until asked to exit, WAIT — for a
/// doorbell ring or, failing that, one [`RUN_REPUBLISH_INTERVAL`] — then
/// re-send this run's record.
///
/// ONE wait rather than a chain of short sleeps, deliberately: macOS timer
/// coalescing charges its slack PER WAKEUP under background QoS, which is what
/// turned a nominal 150 ms wait into a measured 1100 to 1696 ms. Teardown
/// promptness comes from the doorbell ring in [`PumpHandle::drop`] instead.
fn pump_loop(
    exit: &AtomicBool,
    inner: &Arc<RunRegistryInner>,
    doorbell: Option<iceoryx2::port::listener::Listener<CerService>>,
) {
    while !exit.load(Ordering::Relaxed) {
        match doorbell.as_ref() {
            Some(listener) => match listener.timed_wait_one(RUN_REPUBLISH_INTERVAL) {
                // A ring: somebody is gathering and wants this run's answer.
                Ok(Some(_)) => inner.note_doorbell_ring(),
                // The fallback timeout — republish on the interval as before.
                Ok(None) => {}
                // A broken listener must degrade to the interval, never spin.
                Err(e) => {
                    inner.record_doorbell_failure(&format!("{e}"));
                    std::thread::sleep(RUN_REPUBLISH_INTERVAL);
                }
            },
            None => std::thread::sleep(RUN_REPUBLISH_INTERVAL),
        }
        if exit.load(Ordering::Relaxed) {
            return;
        }
        inner.send_record();
    }
}

/// The READER side — a subscriber on the control service that ALSO
/// tracks the distinct `writer_id`s it has heard from (the gather-completeness
/// gate).
struct RunRegistryReader {
    subscriber: Subscriber<CerService, [u8], ()>,
    writers_heard: BTreeSet<u64>,
    malformed: u64,
    malformed_warned: bool,
    receive_warned: bool,
}

impl RunRegistryReader {
    fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_registry_service(node)?;
        let subscriber =
            factory
                .subscriber_builder()
                .create()
                .map_err(|e| TransportError::Internal {
                    reason: format!(
                        "run registry: could not create the control subscriber on \
                         '{RUN_REGISTRY_SERVICE_NAME}': {e:?}"
                    ),
                })?;
        Ok(Self {
            subscriber,
            writers_heard: BTreeSet::new(),
            malformed: 0,
            malformed_warned: false,
            receive_warned: false,
        })
    }

    /// Drain up to [`RUN_REGISTRY_QUEUE_DEPTH`] records, decoding each: valid
    /// records push onto `out` AND record their `writer_id`. A malformed record
    /// is counted + warn-once + SKIPPED (never wedges the drain); a mid-drain
    /// receive failure is warn-once + returns the partial fill.
    fn drain(&mut self, out: &mut Vec<RunRecord>) {
        for _ in 0..RUN_REGISTRY_QUEUE_DEPTH {
            match self.subscriber.receive() {
                Ok(Some(sample)) => match decode_record(sample.payload()) {
                    Ok((writer_id, record)) => {
                        self.writers_heard.insert(writer_id);
                        out.push(record);
                    }
                    Err(e) => self.record_malformed(&e),
                },
                Ok(None) => break,
                Err(e) => {
                    self.record_receive_failure(&format!("{e}"));
                    break;
                }
            }
        }
    }

    fn writers_heard_count(&self) -> usize {
        self.writers_heard.len()
    }

    fn record_malformed(&mut self, error: &RunRecordError) {
        self.malformed += 1;
        if self.malformed_warned {
            tracing::debug!(
                error = %error,
                "run registry: dropped a malformed control record (repeat)"
            );
        } else {
            self.malformed_warned = true;
            tracing::warn!(
                error = %error,
                "run registry: dropped a malformed control record on \
                 '{RUN_REGISTRY_SERVICE_NAME}' (counted; the drain continues; repeats at debug)"
            );
        }
    }

    fn record_receive_failure(&mut self, reason: &str) {
        if self.receive_warned {
            tracing::debug!(
                reason = %reason,
                "run registry: control drain receive failed (repeat)"
            );
        } else {
            self.receive_warned = true;
            tracing::warn!(
                reason = %reason,
                "run registry: control drain receive failed — kept the partial fill; the \
                 gather retries (repeats log at debug)"
            );
        }
    }
}

/// The gather EARLY-EXIT predicate — stop once we have heard from
/// every currently-live writer. Pure so it is oracle-testable independent of
/// transport + wall clock.
///
/// Unlike the mirror registry's twin this needs NO "the set is non-empty"
/// conjunct and never did: a run writer holds exactly one record for its whole
/// life, so being heard and having contributed a record are the same event.
fn gather_reached_all_live_writers(writers_heard: usize, live_publishers: usize) -> bool {
    live_publishers > 0 && writers_heard >= live_publishers
}

/// A gather's records PLUS what they are evidence of.
///
/// [`GatherCompleteness`] is REUSED from
/// [`mirror_registry`](super::mirror_registry) rather than duplicated — it
/// encodes "what a windowed LISTEN's answer is evidence of", which is not a
/// mirror-specific idea. Copying it would be the two-copies-of-one-predicate
/// mistake the shared mirror fold exists to prevent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunGather {
    /// Every run record heard during the window, deduped by `run_id`
    /// (last-write-wins, so a `Live` → `Ending` flip converges).
    pub records: Vec<RunRecord>,
    /// Whether this answer settles the question.
    pub completeness: GatherCompleteness,
}

/// Gather the CURRENT live runs over `node`, carrying WHAT THE ANSWER
/// IS EVIDENCE OF.
///
/// FAST PATH: if the registry service reports ZERO publishers (no live run on
/// this machine — the common case), returns empty IMMEDIATELY without paying
/// any window, and that empty is `Settled` because the service itself reports
/// nobody is publishing.
///
/// Otherwise it drains for up to `window`, unioning every republished record,
/// ringing the doorbell before each pass so live runs ANSWER rather than being
/// caught mid-timer, and EARLY-EXITS the instant it has heard from every
/// currently-live writer. A CRASHED writer whose stale port lingers keeps the
/// live count above the writers heard, so the gather pays the full ceiling and
/// reports `Incomplete` — an explicit "I could not establish the picture",
/// never a confident empty.
///
/// # Errors
///
/// [`TransportError`] from opening the control service or building the
/// subscriber (an SHM / config failure).
pub fn gather_from_node_checked(
    node: &Node<CerService>,
    window: Duration,
) -> TransportResult<RunGather> {
    let factory = open_registry_service(node)?;
    if factory.dynamic_config().number_of_publishers() == 0 {
        return Ok(RunGather {
            records: Vec::new(),
            completeness: GatherCompleteness::Settled,
        });
    }
    let mut reader = RunRegistryReader::open(node)?;
    // Mint the doorbell notifier AFTER the zero-publisher fast path, so a
    // machine with no live run never creates the event service just to ask a
    // question it already has the answer to.
    let bell = open_gather_notifier(node);
    let mut set: BTreeMap<u128, RunRecord> = BTreeMap::new();
    let mut scratch: Vec<RunRecord> = Vec::new();
    let deadline = Instant::now() + window;
    const POLL_CHUNK: Duration = Duration::from_millis(20);
    // WHICH break arm the loop takes is the whole verdict, so it is recorded at
    // the break rather than re-derived afterwards — re-deriving from the final
    // counts cannot distinguish "the deadline expired on this state" from "this
    // state satisfied the early exit".
    let completeness;
    loop {
        // ASK, then listen. Ringing every pass (not just the first) covers a
        // writer whose listener did not exist when the previous ring went out.
        if let Some(bell) = &bell {
            let _ = bell.notify();
        }
        scratch.clear();
        reader.drain(&mut scratch);
        for r in scratch.drain(..) {
            set.insert(r.run_id, r);
        }
        let live = factory.dynamic_config().number_of_publishers();
        if live == 0 || gather_reached_all_live_writers(reader.writers_heard_count(), live) {
            completeness = GatherCompleteness::Settled;
            break;
        }
        if Instant::now() >= deadline {
            completeness = GatherCompleteness::Incomplete {
                live_writers: live,
                writers_heard: reader.writers_heard_count(),
            };
            break;
        }
        std::thread::sleep(POLL_CHUNK);
    }
    Ok(RunGather {
        records: set.into_values().collect(),
        completeness,
    })
}

/// Mint the READER's doorbell notifier for one gather. BEST-EFFORT,
/// and the degrade is `debug!` rather than `warn!` on purpose: losing the
/// doorbell costs LATENCY (the writers still broadcast on their interval), and
/// a gather is a read-only question asked by short-lived commands.
fn open_gather_notifier(
    node: &Node<CerService>,
) -> Option<iceoryx2::port::notifier::Notifier<CerService>> {
    let service = match open_doorbell_service(node) {
        Ok(s) => s,
        Err(e) => {
            tracing::debug!(
                reason = %e,
                "run registry: no doorbell service for this gather — falling back to \
                 catching the runs' periodic republish inside the window"
            );
            return None;
        }
    };
    match service.notifier_builder().create() {
        Ok(n) => Some(n),
        Err(e) => {
            tracing::debug!(
                reason = ?e,
                "run registry: could not mint a doorbell notifier for this gather — \
                 falling back to catching the runs' periodic republish inside the window"
            );
            None
        }
    }
}

/// Build a bare, side-effect-minimal iceoryx2 node for the run
/// registry on `config`.
///
/// Auto dead-node cleanup is DISABLED (never trust the global config; a
/// registry participant must never reap anyone) and NO startup
/// dead-node sweep runs.
///
/// `role` names the node so an operator reading `iox2` bookkeeping can tell a
/// run's writer from a one-shot gather.
fn build_registry_node(
    config: &iceoryx2::config::Config,
    role: &str,
) -> TransportResult<Node<CerService>> {
    crate::iceoryx_logger::init_iceoryx_log_level_from_env();
    let config = super::disable_auto_dead_node_cleanup(config.clone());
    let node_name: iceoryx2::prelude::NodeName =
        role.try_into().map_err(|e| TransportError::Internal {
            reason: format!("run registry: invalid node name '{role}': {e:?}"),
        })?;
    iceoryx2::prelude::NodeBuilder::new()
        .name(&node_name)
        .config(&config)
        .create::<CerService>()
        .map_err(|e| TransportError::Internal {
            reason: format!("run registry: could not create the '{role}' node: {e:?}"),
        })
}

/// A live run's registry WRITER, owning the transient iceoryx2 node it
/// publishes on.
///
/// This is what `cerulion graph run` holds for the run's lifetime. It mints its
/// OWN node rather than borrowing a [`TransportManager`](super::TransportManager)'s,
/// for two verified reasons.
///
/// **Namespace.** On the multi-process path — the Unix DEFAULT — the
/// supervisor's first manager lives on a deliberately THROWAWAY planning
/// namespace (`multiprocess::planning_ix_config`, documented inert: the workers
/// run on the DEFAULT data plane as separate processes). Announcing the run
/// there would publish it where nothing gathers.
///
/// **Ordering.** Every manager on every path is built BELOW the point where the
/// run is announced — the supervisor's planning and data-plane managers alike,
/// and the monolith's — so borrowing one would move the announcement later.
///
/// Two plausible claims are WRONG and are recorded here so they are not
/// re-asserted: it does NOT hold that the
/// supervisor "builds no transport manager for itself" (it builds two), and the
/// reason is not that "a recorder needs to find the run before the graph is
/// released to step 0" (that is bagd's own handshake, which this announcement neither
/// uses nor establishes).
///
/// `build_registry_node` mints the same kind of bare, sweep-free node the
/// mirror registry's one-shot reader already uses.
///
/// Dropping this stops republication, so the run's record expires and a
/// subsequent gather does not see it — which is exactly what a crash looks
/// like, and why a GRACEFUL exit calls [`RunHandle::set_ending`] first.
pub struct RunHandle {
    registry: RunRegistry,
    // Declared LAST so it drops after the registry: the publisher's ports must
    // be released before the node they were created on.
    _node: Node<CerService>,
}

impl RunHandle {
    /// Publish `record` on `config`'s namespace and keep republishing it until
    /// this handle drops.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the node, the control service, or the publisher
    /// cannot be created, or if the record is unencodable.
    pub fn publish_on_config(
        config: &iceoryx2::config::Config,
        record: RunRecord,
    ) -> TransportResult<Self> {
        let node = build_registry_node(config, "cerulion-run-writer")?;
        let registry = RunRegistry::open(&node, record)?;
        Ok(Self {
            registry,
            _node: node,
        })
    }

    /// Publish `record` on the PROCESS-GLOBAL iceoryx2 namespace — the
    /// production `cerulion graph run` entry point.
    ///
    /// # Errors
    ///
    /// As [`RunHandle::publish_on_config`].
    pub fn publish(record: RunRecord) -> TransportResult<Self> {
        Self::publish_on_config(iceoryx2::config::Config::global_config(), record)
    }

    /// Announce that this run is shutting down GRACEFULLY, and send it
    /// immediately. Idempotent. Returns whether the state actually changed.
    ///
    /// A consumer bound to this run's lifetime uses it to tell "the operator
    /// stopped it" from "the process vanished" — a crash publishes nothing.
    pub fn set_ending(&self) -> bool {
        self.registry.set_state(RunState::Ending)
    }

    /// How many doorbell rings this run's pump has ANSWERED (Principle #3).
    #[must_use]
    pub fn doorbell_rings_answered(&self) -> u64 {
        self.registry.doorbell_rings_answered()
    }
}

/// Gather the CURRENT live runs on `config`'s namespace.
///
/// # Errors
///
/// [`TransportError`] from building the reader node or opening the control
/// service.
pub fn gather_runs_on_config(
    config: &iceoryx2::config::Config,
    window: Duration,
) -> TransportResult<RunGather> {
    let node = build_registry_node(config, "cerulion-run-reader")?;
    gather_from_node_checked(&node, window)
}

/// Gather the CURRENT live runs on the process-global iceoryx2
/// namespace, under the default [`RUN_GATHER_WINDOW`].
///
/// # Errors
///
/// As [`gather_runs_on_config`].
pub fn gather_current_runs() -> TransportResult<RunGather> {
    gather_runs_on_config(iceoryx2::config::Config::global_config(), RUN_GATHER_WINDOW)
}

// ===========================================================================
// LIFETIME BINDING — watching ONE run for the rest of its life
// ===========================================================================

/// The INSTANTANEOUS reading of one [`RunWatcher`] observation,
/// before any confirmation is applied.
///
/// Separated from [`RunLifetimeTracker`] so the two halves are independently
/// testable: this answers "what does this one look at say?", the tracker
/// answers "how many of those does it take to act?".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunSignal {
    /// A record for the watched run was heard and it still says `Live`.
    Live,
    /// A record for the watched run was heard carrying `Ending` — the run
    /// announced a GRACEFUL shutdown.
    Ending,
    /// Silence that now counts as evidence: either nothing at all is publishing
    /// on the registry, or the watched run has been quiet past
    /// [`RUN_VANISH_GRACE`].
    Absent,
    /// Nothing heard from the watched run, and the silence does not yet mean
    /// anything. Explicitly NOT `Absent`: a reading that makes no claim must be
    /// distinguishable from one that does.
    Waiting,
}

/// How a watched run ENDED — the only two terminal outcomes, and
/// the two the bag stamps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunEnded {
    /// The run published `Ending` before it went. Only a CONTROLLED exit can
    /// do that, so this is positive evidence the operator (or the graph itself)
    /// stopped it.
    Graceful,
    /// The run stopped being published without ever saying `Ending`. A crash, a
    /// SIGKILL, a power cut — and also a graceful exit whose last
    /// word this watcher did not hear (see [`RunWatcher`] on why a long-lived
    /// subscriber makes that last case rare rather than routine).
    Vanished,
}

impl std::fmt::Display for RunEnded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunEnded::Graceful => write!(f, "graceful"),
            RunEnded::Vanished => write!(f, "vanished"),
        }
    }
}

/// What ONE observation saw — the pure input to
/// [`classify_run_signal`], with no clock and no transport in it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunSighting {
    /// The state carried by a record for the WATCHED run in this observation's
    /// batch, if one was heard. `Ending` wins over `Live` within a batch: the
    /// state is monotone per writer, so a batch holding both is the flip
    /// itself.
    pub own_state: Option<RunState>,
    /// Live publishers on the registry service at this observation. `0` means
    /// no run at all is publishing on this machine, which settles the question
    /// for the watched run without waiting out any grace.
    pub live_writers: usize,
    /// Whether this watcher has EVER heard the run it watches.
    ///
    /// Load-bearing: until first contact the watcher cannot tell "the run is
    /// gone" from "the run's writer has not come up yet", so the zero-writer
    /// fast path must not fire. Only the grace can escalate a run that was
    /// never heard.
    pub heard_ever: bool,
    /// A SUCCESSOR run was heard SINCE the watched run last spoke —
    /// a different `run_id`, the same `graph_name`, started no earlier than ours
    /// (see [`RunWatcher::observe`]).
    ///
    /// Scoped to the current silence on purpose. A successor that announced,
    /// failed to build and exited is stale evidence a moment later, and letting
    /// it stand would apply the shorter [`RUN_REPLACED_GRACE`] to a completely
    /// unrelated later silence of a still-live run.
    pub successor_seen: bool,
    /// Time since the watched run was last heard — or since the watcher opened,
    /// when it never has.
    pub since_own_record: Duration,
}

/// The two silence graces [`classify_run_signal`] applies, carried
/// together so the pair cannot be swapped at a call site.
///
/// Two adjacent `Duration` arguments that mean different things is the hazard
/// the CLI engine's `RecordingInputSpec` was extracted for; here a swap would apply the
/// SHORT grace to a plain crash and the LONG one to a restart — inverting the
/// whole point — while compiling clean.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunGraces {
    /// Silence alone, with no successor in sight: [`RUN_VANISH_GRACE`].
    pub vanish: Duration,
    /// Silence WITH a successor announcing: [`RUN_REPLACED_GRACE`].
    pub replaced: Duration,
}

impl Default for RunGraces {
    fn default() -> Self {
        Self {
            vanish: RUN_VANISH_GRACE,
            replaced: RUN_REPLACED_GRACE,
        }
    }
}

impl RunGraces {
    /// The shipped pair.
    #[must_use]
    pub fn shipped() -> Self {
        Self::default()
    }

    /// A pair scaled from a caller-chosen `vanish` grace, PRESERVING the shipped
    /// ratio between the two.
    ///
    /// The one-knob seam every caller with a shorter grace uses (a test that must
    /// reach an absence verdict without spending five seconds; an operator whose
    /// registry is degraded). Scaling rather than clamping is what keeps the
    /// SHRINK observable under an injected grace: a rule that just took
    /// `min(replaced, vanish)` would collapse both to one number below 2 s and
    /// make the replaced route indistinguishable from the silence route in every
    /// test that injects one.
    #[must_use]
    pub fn scaled_from(vanish: Duration) -> Self {
        let ratio = RUN_REPLACED_GRACE.as_secs_f64() / RUN_VANISH_GRACE.as_secs_f64();
        Self {
            vanish,
            replaced: vanish.mul_f64(ratio),
        }
    }
}

/// Fold ONE record's state into the batch's verdict for the watched
/// run. Pure, and EXTRACTED so the rule is oracle-testable.
///
/// **`Ending` wins within a batch.** The state is monotone per writer (`Live` →
/// `Ending`, never back), so a batch carrying both IS the flip, and the flip is
/// what matters — the whole reason C3 holds a long-lived subscriber is to catch
/// that one frame, and a fold that let a `Live` republish sitting in the same
/// queue mask it would throw the catch away and report a graceful exit as a
/// crash.
///
/// This is the NORMAL batch shape on the shipping cadence, not a corner: the
/// registry republishes every [`RUN_REPUBLISH_INTERVAL`] while a consumer looks
/// less often, so a drain routinely carries several of one run's records and the
/// `Ending` is simply the last of them.
#[must_use]
pub fn fold_batch_state(current: Option<RunState>, next: RunState) -> Option<RunState> {
    match (current, next) {
        (Some(RunState::Ending), _) | (_, RunState::Ending) => Some(RunState::Ending),
        _ => Some(RunState::Live),
    }
}

/// The identity a [`RunWatcher`] LEARNS about its own run from the
/// first record it hears for it — what a successor is compared against.
///
/// Learned from the wire rather than supplied by the caller: the watcher is
/// opened with a `run_id` alone, and the comparison is only meaningful once the
/// run has been heard at least once (which is exactly when
/// [`RunSighting::heard_ever`] turns true, so the two gates cannot disagree).
#[derive(Debug, Clone, PartialEq, Eq)]
struct OwnRunIdentity {
    graph_name: String,
    run_started_at_ns: u64,
}

/// Is `record` a SUCCESSOR of the watched run — i.e. does it mean
/// "something is taking over the topics this recording taps"? Pure.
///
/// Three conjuncts, each closing one way a record could look like a successor
/// and not be one:
///
/// 1. **A different `run_id`.** Our own record is not our successor.
/// 2. **The SAME `graph_name`.** Another graph's run publishes to different
///    topics (a graph's prefix is frozen at `graph create`), so it takes
///    nothing over and is ordinary co-tenancy.
/// 3. **Started no EARLIER than ours.** A same-graph run that started BEFORE us
///    cannot be replacing us — we replaced IT — and treating one as a successor
///    would let a stale record still on the wire end a run that just started.
#[must_use]
fn is_successor_record(own: &OwnRunIdentity, own_run_id: u128, record: &RunRecord) -> bool {
    record.run_id != own_run_id
        && record.graph_name == own.graph_name
        && record.run_started_at_ns >= own.run_started_at_ns
}

/// Classify ONE observation. Pure, so every arm is oracle-testable
/// without a clock or a transport.
///
/// Precedence, and each rule closes one way the others would be wrong:
///
/// 1. **A heard `Ending` is terminal.** Positive evidence; nothing observed
///    later can make a graceful shutdown not have happened.
/// 2. **A heard `Live` is `Live`.** It also resets the absence streak at the
///    tracker, which is what keeps one lost observation from ending a run.
/// 3. **Nothing heard, and nobody is publishing at all** ⇒ `Absent`, with NO
///    grace — but only once this watcher has heard the run at least once. A
///    zero-publisher reading before first contact is indistinguishable from a
///    writer that has not started, and treating it as absence would let a
///    recorder armed a moment early terminate itself. NOTE that `live_writers`
///    is a MACHINE-WIDE count, so this route is dead whenever any other run is
///    publishing — see [`RUN_VANISH_GRACE`] for why there is no cheaper exact
///    per-run alternative.
/// 4. **Nothing heard past [`RUN_REPLACED_GRACE`] while a SUCCESSOR is
///    announcing** ⇒ `Absent`. Same `heard_ever` gate, for the same reason.
/// 5. **Nothing heard past [`RUN_VANISH_GRACE`]** ⇒ `Absent`. This is the arm
///    that also covers a run that died before the watcher ever heard it.
/// 6. Otherwise `Waiting` — no claim.
#[must_use]
pub fn classify_run_signal(sighting: &RunSighting, graces: RunGraces) -> RunSignal {
    match sighting.own_state {
        Some(RunState::Ending) => RunSignal::Ending,
        Some(RunState::Live) => RunSignal::Live,
        None => {
            // THREE independent routes to the same verdict, kept NAMED because
            // they are different facts and a reader of a `||` cannot tell which
            // one fired. Each is also independently testable: the pure
            // table below drives a case that only one of them answers.
            let nobody_is_publishing = sighting.heard_ever && sighting.live_writers == 0;
            // Silence PLUS positive evidence that something
            // is taking over this run's topics. Gated on `heard_ever` for the
            // same reason as the route above — before first contact a successor
            // sighting says nothing about a writer that has not come up.
            let replaced_by_a_successor = sighting.heard_ever
                && sighting.successor_seen
                && sighting.since_own_record >= graces.replaced;
            let quiet_past_the_grace = sighting.since_own_record >= graces.vanish;
            if nobody_is_publishing || replaced_by_a_successor || quiet_past_the_grace {
                RunSignal::Absent
            } else {
                RunSignal::Waiting
            }
        }
    }
}

/// The CONFIRMATION half — folds a stream of [`RunSignal`]s into a
/// terminal [`RunEnded`], or nothing.
///
/// Pure and clock-free. Once it reports an outcome it KEEPS reporting the same
/// one: a lifetime verdict is not something a later observation can walk back,
/// and a caller that polls after acting must not see it flip.
#[derive(Debug, Clone, Default)]
pub struct RunLifetimeTracker {
    absent_streak: u32,
    outcome: Option<RunEnded>,
}

impl RunLifetimeTracker {
    /// A fresh tracker for a run believed live.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one signal in and report the terminal outcome, if there now is one.
    pub fn observe(&mut self, signal: RunSignal) -> Option<RunEnded> {
        if self.outcome.is_some() {
            return self.outcome;
        }
        match signal {
            RunSignal::Ending => {
                // Positive evidence — terminal on the FIRST sighting. There is
                // no transient that fabricates an `Ending` record.
                self.outcome = Some(RunEnded::Graceful);
            }
            RunSignal::Absent => {
                self.absent_streak = self.absent_streak.saturating_add(1);
                if self.absent_streak >= RUN_ABSENCE_CONFIRMATIONS {
                    self.outcome = Some(RunEnded::Vanished);
                }
            }
            // A live sighting is proof the run is there — it clears the streak,
            // which is exactly what stops ONE lost observation from ending a
            // recording.
            RunSignal::Live => self.absent_streak = 0,
            // A reading that makes NO claim must not advance the streak, and
            // must not clear it either: it is not evidence in either direction.
            RunSignal::Waiting => {}
        }
        self.outcome
    }

    /// The terminal outcome, if one has been reached (Principle #3).
    #[must_use]
    pub fn outcome(&self) -> Option<RunEnded> {
        self.outcome
    }

    /// Consecutive absent observations banked so far — the evidence the next
    /// one would add to.
    #[must_use]
    pub fn absent_streak(&self) -> u32 {
        self.absent_streak
    }
}

/// What one [`RunWatcher::observe`] call saw and concluded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunObservation {
    /// This observation's instantaneous reading.
    pub signal: RunSignal,
    /// The terminal outcome, once confirmed. `Some` is STICKY.
    pub ended: Option<RunEnded>,
    /// Distinct run ids heard by this watcher so far, the watched one included.
    /// Reported for the LOG only — a second live run on the machine is legal
    /// and changes no decision here.
    pub runs_seen: usize,
    /// A SUCCESSOR of the watched run was heard SINCE the
    /// watched run last spoke — see [`RunSighting::successor_seen`].
    ///
    /// Reported because a CONSUMER needs it for a second purpose the classifier
    /// does not: it is the discriminator between "this recording's tail is
    /// ambiguous" and "this recording's tail is simply its own run's last
    /// frames". A run that crashed with nothing replacing it leaves frames that
    /// can only be its own; a run replaced by a successor leaves frames that may
    /// belong to either, and only the consumer can say what to do about that.
    pub successor_seen: bool,
}

/// A LONG-LIVED subscriber bound to ONE run's lifetime.
///
/// # Why this exists rather than a poll of [`gather_runs_on_config`]
///
/// `Ending` is published ONCE, synchronously, and the writer is released
/// microseconds later; the registry service requests no history. C0 MEASURED a
/// gatherer racing a graceful exit at **7 of 8 at a hot spin and 0 of 8 at a
/// natural 250 ms cadence** — because every gather opens a FRESH subscriber, so
/// a frame sent before that subscriber existed is not merely missed by timing,
/// it is structurally unreachable.
///
/// That makes polling the wrong shape twice over:
///
/// * **It loses data that was delivered.** A `graph run`'s last word is a frame
///   committed to shared memory. Principle #6 says a missing wakeup must not
///   lose data; a reader that only exists between frames guarantees it does.
/// * **The tighter you poll, the worse the trade.** A hot spin still lost 1 in
///   8 while burning a core — an unbounded loop whose budget is never spent is
///   the observer-spin class, and here it does not even buy correctness.
///
/// A subscriber attached ONCE, before the run is watched, has the `Ending` frame
/// sitting in its own SHM queue ([`RUN_REGISTRY_QUEUE_DEPTH`] deep) whenever it
/// next looks. The discrimination becomes a property of DATA the watcher holds
/// (Principle #2) rather than of when it happened to ask.
///
/// # What it costs
///
/// ONE subscriber port on `/__cerulion/runs` for the watcher's life, out of
/// [`RUN_REGISTRY_MAX_READERS`], plus one bare iceoryx2 node. No thread and no
/// timer: [`observe`](Self::observe) runs on the caller's existing loop.
///
/// # It deliberately does NOT ring the doorbell
///
/// A gather is a one-shot QUESTION, so it rings to be answered rather than
/// waiting for a broadcast. A watcher is a standing LISTENER: the republish belt
/// it already receives is exactly what it needs, and ringing on every pass would
/// wake every live run's pump on this machine — a cost paid by processes that
/// are not recording, for an answer this watcher gets anyway.
///
/// # The one thing it cannot hear
///
/// A crash publishes NOTHING, so absence is the only available signal for it —
/// see [`classify_run_signal`] and [`RUN_VANISH_GRACE`] for how that is bounded,
/// and [`RUN_ABSENCE_CONFIRMATIONS`] for why one quiet look is never enough.
///
/// **That window is not free, and a consumer must not assume it is.** A watcher
/// reports a crash some time AFTER it happened, and whatever the consumer does
/// in between it does on behalf of a run that is already over. For the recorder
/// that means frames: bagd's taps are keyed by topic NAME and hold the iceoryx2
/// service open across a producer swap, so a successor publishing on the same
/// topics is drained into the predecessor's bag for the whole detection window.
/// [`RUN_REPLACED_GRACE`] shortens that window on the shape where it matters
/// most; it cannot close it, so the consumer is responsible for saying what its
/// window cost — see `cerulion_bagd`'s `RunBindingCoverage`.
pub struct RunWatcher {
    run_id: u128,
    factory: PortFactory<CerService, [u8], ()>,
    reader: RunRegistryReader,
    /// Anchors the grace before first contact — the watcher's own open instant.
    last_own_record: Instant,
    heard_ever: bool,
    /// LEARNED from the first record heard for the watched run — what a
    /// successor is compared against. `None` until first contact, which is the
    /// same instant `heard_ever` turns true.
    own_identity: Option<OwnRunIdentity>,
    /// A successor was heard since the watched run last spoke. CLEARED whenever
    /// it speaks again — see [`RunSighting::successor_seen`].
    successor_since_own_record: bool,
    runs_seen: BTreeSet<u128>,
    tracker: RunLifetimeTracker,
    graces: RunGraces,
    // Declared LAST so it drops after the subscriber: ports must be released
    // before the node they were created on.
    _node: Node<CerService>,
}

impl RunWatcher {
    /// Watch `run_id` on `config`'s namespace, under [`RUN_VANISH_GRACE`].
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the node, the control service, or the subscriber
    /// cannot be created.
    pub fn open_on_config(
        config: &iceoryx2::config::Config,
        run_id: u128,
    ) -> TransportResult<Self> {
        Self::open_on_config_with_graces(config, run_id, RunGraces::shipped())
    }

    /// [`RunWatcher::open_on_config`] with an explicit silence grace — the seam
    /// a test uses to reach the absence arms without spending the shipped
    /// five seconds.
    ///
    /// The replaced grace is SCALED from it ([`RunGraces::scaled_from`]), so an
    /// injected grace shortens both routes in proportion rather than collapsing
    /// them onto one number.
    ///
    /// # Errors
    ///
    /// As [`RunWatcher::open_on_config`].
    pub fn open_on_config_with_grace(
        config: &iceoryx2::config::Config,
        run_id: u128,
        grace: Duration,
    ) -> TransportResult<Self> {
        Self::open_on_config_with_graces(config, run_id, RunGraces::scaled_from(grace))
    }

    /// [`RunWatcher::open_on_config`] with BOTH graces named explicitly.
    ///
    /// # Errors
    ///
    /// As [`RunWatcher::open_on_config`].
    pub fn open_on_config_with_graces(
        config: &iceoryx2::config::Config,
        run_id: u128,
        graces: RunGraces,
    ) -> TransportResult<Self> {
        let node = build_registry_node(config, "cerulion-run-watcher")?;
        let factory = open_registry_service(&node)?;
        let reader = RunRegistryReader::open(&node)?;
        Ok(Self {
            run_id,
            factory,
            reader,
            last_own_record: Instant::now(),
            heard_ever: false,
            own_identity: None,
            successor_since_own_record: false,
            runs_seen: BTreeSet::new(),
            tracker: RunLifetimeTracker::new(),
            graces,
            _node: node,
        })
    }

    /// Watch `run_id` on the namespace `mgr` runs on.
    ///
    /// The entry point for a consumer that must not name an iceoryx2 type —
    /// `cerulion_bagd` is explicit in its `Cargo.toml` about having no direct
    /// iceoryx2 dependency, and this keeps that true while still binding the
    /// watcher to the SAME namespace the recorder taps on (a watcher on a
    /// different SHM root would hear nothing and call every run vanished).
    ///
    /// # Errors
    ///
    /// As [`RunWatcher::open_on_config`].
    pub fn open_on_manager(mgr: &super::TransportManager, run_id: u128) -> TransportResult<Self> {
        Self::open_on_config(&mgr.iox_config(), run_id)
    }

    /// [`RunWatcher::open_on_manager`] with an explicit silence grace.
    ///
    /// # Errors
    ///
    /// As [`RunWatcher::open_on_config`].
    pub fn open_on_manager_with_grace(
        mgr: &super::TransportManager,
        run_id: u128,
        grace: Duration,
    ) -> TransportResult<Self> {
        Self::open_on_config_with_grace(&mgr.iox_config(), run_id, grace)
    }

    /// The run this watcher is bound to.
    #[must_use]
    pub fn run_id(&self) -> u128 {
        self.run_id
    }

    /// The terminal outcome, once confirmed (Principle #3 — readable without
    /// driving another observation).
    #[must_use]
    pub fn ended(&self) -> Option<RunEnded> {
        self.tracker.outcome()
    }

    /// Drain everything the registry has delivered since the last call and fold
    /// it into this run's lifetime verdict.
    ///
    /// Cheap and non-blocking: a bounded drain of an SHM queue plus one
    /// dynamic-config read on an already-open service handle (the liveness
    /// observer's primitive). Sound here for a reason that does NOT hold
    /// in-process: the watcher is a DIFFERENT process from the run it watches,
    /// so the same-PID liveness defeat does not apply.
    pub fn observe(&mut self) -> RunObservation {
        let mut scratch: Vec<RunRecord> = Vec::new();
        self.reader.drain(&mut scratch);

        let mut own_state: Option<RunState> = None;
        for record in &scratch {
            self.runs_seen.insert(record.run_id);
            if record.run_id == self.run_id {
                own_state = fold_batch_state(own_state, record.state);
                // LEARN our own identity from the first record we hear for the
                // run — the successor comparison has nothing to compare against
                // until then, which is exactly the window `heard_ever` gates.
                if self.own_identity.is_none() {
                    self.own_identity = Some(OwnRunIdentity {
                        graph_name: record.graph_name.clone(),
                        run_started_at_ns: record.run_started_at_ns,
                    });
                }
            }
        }
        // The successor pass runs AFTER the loop above, so a batch that carries
        // our own FIRST record and a successor in the same drain is judged
        // against an identity we already learned rather than against `None`.
        if let Some(own) = &self.own_identity {
            for record in &scratch {
                if is_successor_record(own, self.run_id, record) {
                    self.successor_since_own_record = true;
                }
            }
        }
        if own_state.is_some() {
            self.last_own_record = Instant::now();
            self.heard_ever = true;
            // Our run just spoke, so any successor sighting is now evidence
            // about a silence that has ENDED — it must not carry over and apply
            // the shorter grace to some later, unrelated quiet stretch.
            self.successor_since_own_record = false;
        }

        let sighting = RunSighting {
            own_state,
            live_writers: self.factory.dynamic_config().number_of_publishers(),
            heard_ever: self.heard_ever,
            successor_seen: self.successor_since_own_record,
            since_own_record: self.last_own_record.elapsed(),
        };
        let signal = classify_run_signal(&sighting, self.graces);
        let ended = self.tracker.observe(signal);
        RunObservation {
            signal,
            ended,
            runs_seen: self.runs_seen.len(),
            successor_seen: sighting.successor_seen,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A hand-built record used across the codec oracles.
    fn sample() -> RunRecord {
        RunRecord {
            run_id: 0x0123_4567_89AB_CDEF_FEDC_BA98_7654_3210,
            supervisor_pid: 4711,
            run_started_at_ns: 1_753_000_000_000_000_000,
            state: RunState::Live,
            graph_name: "go2_attach".to_string(),
            run_dir: "/home/op/.cerulion/runs/go2_attach-0123".to_string(),
        }
    }

    /// THE codec oracle: the encoded frame is compared BYTE FOR BYTE against a
    /// hand-assembled expectation, so a field reorder / endianness flip / width
    /// change fails here rather than silently on the wire.
    #[test]
    fn encode_produces_the_hand_built_frame_and_decode_is_its_exact_inverse() {
        let record = sample();
        let writer_id = 0x1122_3344_5566_7788u64;
        let bytes = encode_record(writer_id, &record).expect("encodable");

        let mut want = Vec::new();
        want.extend_from_slice(&[0xCE, 0x81]); // magic
        want.push(1); // version
        want.extend_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());
        want.extend_from_slice(&record.run_id.to_le_bytes());
        want.extend_from_slice(&4711u32.to_le_bytes());
        want.extend_from_slice(&1_753_000_000_000_000_000u64.to_le_bytes());
        want.push(1); // state = live
        want.extend_from_slice(&(record.graph_name.len() as u16).to_le_bytes());
        want.extend_from_slice(&(record.run_dir.len() as u16).to_le_bytes());
        want.extend_from_slice(record.graph_name.as_bytes());
        want.extend_from_slice(record.run_dir.as_bytes());
        assert_eq!(bytes, want, "the wire frame must match the hand oracle");
        assert_eq!(
            bytes.len(),
            RUN_HEADER_LEN + record.graph_name.len() + record.run_dir.len()
        );

        let (got_writer, got) = decode_record(&bytes).expect("decodable");
        assert_eq!(got_writer, writer_id);
        assert_eq!(got, record);
    }

    /// The `Ending` state round-trips as its OWN byte — a state flip must be
    /// visible on the wire, not inferred.
    #[test]
    fn the_ending_state_rides_its_own_wire_byte() {
        let mut record = sample();
        record.state = RunState::Ending;
        let bytes = encode_record(7, &record).expect("encodable");
        assert_eq!(bytes[OFF_STATE], 2, "ending is wire byte 2");
        assert_eq!(
            decode_record(&bytes).expect("decodable").1.state,
            RunState::Ending
        );
        // And the two states are genuinely distinct on the wire.
        let live = encode_record(7, &sample()).expect("encodable");
        assert_eq!(live[OFF_STATE], 1, "live is wire byte 1");
    }

    /// A ZEROED frame of the right length is REFUSED, and specifically on the
    /// state byte having no meaning — `0` is deliberately not `Live`, so an
    /// all-zero buffer can never be read as a healthy run.
    #[test]
    fn a_zeroed_state_byte_is_refused_rather_than_read_as_live() {
        let mut bytes = encode_record(7, &sample()).expect("encodable");
        bytes[OFF_STATE] = 0;
        assert_eq!(
            decode_record(&bytes),
            Err(RunRecordError::BadState { found: 0 })
        );
        bytes[OFF_STATE] = 3;
        assert_eq!(
            decode_record(&bytes),
            Err(RunRecordError::BadState { found: 3 })
        );
    }

    /// Every malformed shape is rejected with its OWN reason — one oracle per
    /// condition, so a widened check cannot pass by collapsing two into one.
    #[test]
    fn every_malformed_frame_is_refused_with_its_own_reason() {
        assert_eq!(
            decode_record(&[]),
            Err(RunRecordError::Truncated { len: 0 })
        );
        assert_eq!(
            decode_record(&[0u8; RUN_HEADER_LEN - 1]),
            Err(RunRecordError::Truncated {
                len: RUN_HEADER_LEN - 1
            })
        );

        let good = encode_record(7, &sample()).expect("encodable");

        let mut bad_magic = good.clone();
        bad_magic[0] = 0x00;
        assert_eq!(decode_record(&bad_magic), Err(RunRecordError::BadMagic));

        let mut bad_version = good.clone();
        bad_version[OFF_VERSION] = RUN_VERSION + 1;
        assert_eq!(
            decode_record(&bad_version),
            Err(RunRecordError::BadVersion {
                found: RUN_VERSION + 1
            })
        );

        // Declared lengths that disagree with the frame.
        let mut short = good.clone();
        short.truncate(good.len() - 1);
        let payload = short.len() - RUN_HEADER_LEN;
        assert_eq!(
            decode_record(&short),
            Err(RunRecordError::LengthMismatch {
                declared: sample().graph_name.len() + sample().run_dir.len(),
                actual_payload: payload
            })
        );

        // A declared name length past the bound is refused BEFORE any slicing.
        let mut huge_name = good.clone();
        huge_name[OFF_NAME_LEN..OFF_DIR_LEN]
            .copy_from_slice(&((MAX_RUN_GRAPH_NAME_LEN + 1) as u16).to_le_bytes());
        assert_eq!(
            decode_record(&huge_name),
            Err(RunRecordError::GraphNameTooLong {
                len: MAX_RUN_GRAPH_NAME_LEN + 1
            })
        );

        // Non-UTF-8 body.
        let mut bad_utf8 = encode_record(
            7,
            &RunRecord {
                graph_name: "ab".to_string(),
                ..sample()
            },
        )
        .expect("encodable");
        bad_utf8[RUN_HEADER_LEN] = 0xFF;
        assert_eq!(decode_record(&bad_utf8), Err(RunRecordError::InvalidUtf8));
    }

    /// The ENCODE-side bounds are refused before a frame is ever built — an
    /// empty or over-long field must not reach the wire.
    #[test]
    fn the_encoder_refuses_empty_and_over_long_fields() {
        assert_eq!(
            encode_record(
                1,
                &RunRecord {
                    graph_name: String::new(),
                    ..sample()
                }
            ),
            Err(RunRecordError::EmptyGraphName)
        );
        assert_eq!(
            encode_record(
                1,
                &RunRecord {
                    run_dir: String::new(),
                    ..sample()
                }
            ),
            Err(RunRecordError::EmptyRunDir)
        );
        assert_eq!(
            encode_record(
                1,
                &RunRecord {
                    graph_name: "g".repeat(MAX_RUN_GRAPH_NAME_LEN + 1),
                    ..sample()
                }
            ),
            Err(RunRecordError::GraphNameTooLong {
                len: MAX_RUN_GRAPH_NAME_LEN + 1
            })
        );
        assert_eq!(
            encode_record(
                1,
                &RunRecord {
                    run_dir: "d".repeat(MAX_RUN_DIR_LEN + 1),
                    ..sample()
                }
            ),
            Err(RunRecordError::RunDirTooLong {
                len: MAX_RUN_DIR_LEN + 1
            })
        );
        // …and both bounds are INCLUSIVE: exactly at the limit encodes.
        let at_limit = RunRecord {
            graph_name: "g".repeat(MAX_RUN_GRAPH_NAME_LEN),
            run_dir: "d".repeat(MAX_RUN_DIR_LEN),
            ..sample()
        };
        let bytes = encode_record(1, &at_limit).expect("at the limit is encodable");
        assert_eq!(bytes.len(), MAX_RUN_RECORD_LEN);
        assert_eq!(decode_record(&bytes).expect("decodable").1, at_limit);
    }

    /// The early-exit predicate, pinned on BOTH sides of its threshold.
    #[test]
    fn the_gather_early_exit_predicate_is_pinned_on_both_sides() {
        assert!(
            !gather_reached_all_live_writers(0, 0),
            "no live writer is handled by the caller's fast path, never by this predicate"
        );
        assert!(!gather_reached_all_live_writers(0, 1));
        assert!(
            !gather_reached_all_live_writers(1, 2),
            "one of two is not all"
        );
        assert!(gather_reached_all_live_writers(1, 1));
        assert!(gather_reached_all_live_writers(2, 2));
        assert!(
            gather_reached_all_live_writers(3, 2),
            "a writer that exited mid-gather leaves more heard than live — still complete"
        );
    }

    /// The service names carry NO `/data` suffix, which is what keeps the
    /// registry invisible to `topic list` and to bagd's live-topic enumeration.
    /// Byte-pinned because the invisibility is a property of the NAME.
    #[test]
    fn run_registry_service_names_have_no_data_suffix() {
        for name in [
            RUN_REGISTRY_SERVICE_NAME,
            RUN_REGISTRY_DOORBELL_SERVICE_NAME,
        ] {
            assert!(
                !name.ends_with("/data"),
                "'{name}' must not end in /data or `topic list` would surface it"
            );
            assert!(
                name.starts_with("/__cerulion/"),
                "'{name}' must live under the reserved prefix"
            );
        }
        assert_eq!(RUN_REGISTRY_SERVICE_NAME, "/__cerulion/runs");
        assert_eq!(RUN_REGISTRY_DOORBELL_SERVICE_NAME, "/__cerulion/runs/event");
        assert_ne!(
            RUN_REGISTRY_SERVICE_NAME, RUN_REGISTRY_DOORBELL_SERVICE_NAME,
            "iceoryx2 keys a service by name; one name under two messaging patterns is an error"
        );
    }

    /// Two run ids minted back to back DIFFER — the property the whole
    /// "one bag = one run" invariant rests on (a restart must not reuse an id).
    #[test]
    fn minted_run_ids_are_distinct() {
        let ids: BTreeSet<u128> = (0..64).map(|_| mint_run_id()).collect();
        assert_eq!(ids.len(), 64, "every minted run id must be distinct");
        assert!(
            ids.iter().all(|id| *id != 0),
            "a zero run id would be indistinguishable from an unset field"
        );
    }

    /// The DESIGN statement, pinned: a run writer is minted FROM its record and
    /// has no way to become record-less, so the presence-frame problem
    /// (a live writer that is silent because it holds nothing) cannot arise
    /// here. The `RunRegistry` API surface is the proof: there is no
    /// `register` / `unregister`, only `set_state`.
    ///
    /// Enforced structurally by a source walk over THIS module rather than by
    /// prose — a future `unregister` would re-open the silence hole and must
    /// fail a test, not merely be remembered.
    ///
    /// The walk covers the PRODUCTION half only: it truncates at this test
    /// module's own header, because the forbidden tokens appear here as string
    /// literals and a walk that reads them would fail on itself. The needle is
    /// ASSEMBLED at runtime for the same reason — spelled literally it would
    /// match its own occurrence and truncate at the wrong place. The split is
    /// asserted to have actually happened, so a rename cannot silently reduce
    /// the walk to the whole file (which fails) or to nothing (which passes
    /// vacuously).
    #[test]
    fn a_run_writer_always_has_exactly_one_record_to_send() {
        let src = include_str!("run_registry.rs");
        let needle = concat!("mod ", "tests {");
        let cut = src.find(needle).expect("this module has a test module");
        let production = &src[..cut];
        // Strip comments so the module's own EXPLANATION of why these methods
        // are absent is not mistaken for them.
        let code: String = production
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<Vec<_>>()
            .join("\n");
        for forbidden in ["fn register(", "fn unregister(", "fn remove("] {
            assert!(
                !code.contains(forbidden),
                "`{forbidden}` would let a run writer become record-less, which is exactly the \
                 silent-writer hole this registry avoids by construction"
            );
        }
        // Anti-tautology: the stripped production view must still contain the
        // code it DOES have, or every assertion above is vacuous.
        assert!(
            code.contains("fn set_state("),
            "the walk lost the production half"
        );
        assert!(
            code.contains("fn send_record("),
            "the walk lost the production half"
        );
        assert!(
            code.contains("pub fn set_ending("),
            "the walk lost the production half"
        );
    }
}

// ===========================================================================
// The LIFETIME rule — pure oracles (no transport, no clock)
// ===========================================================================
#[cfg(test)]
mod lifetime_tests {
    use super::*;

    const GRACE: Duration = Duration::from_secs(5);
    /// The replaced grace used by the pure arms. DELIBERATELY not equal to
    /// `GRACE`: an arm that cannot tell the two routes apart cannot catch code
    /// that applies the wrong one.
    const REPLACED: Duration = Duration::from_secs(2);

    fn graces() -> RunGraces {
        RunGraces {
            vanish: GRACE,
            replaced: REPLACED,
        }
    }

    /// A sighting that heard nothing and saw no successor, with the knobs most
    /// arms vary.
    fn silent(heard_ever: bool, live_writers: usize, since: Duration) -> RunSighting {
        RunSighting {
            own_state: None,
            live_writers,
            heard_ever,
            successor_seen: false,
            since_own_record: since,
        }
    }

    /// A sighting that heard nothing while a SUCCESSOR is announcing.
    fn replaced(heard_ever: bool, live_writers: usize, since: Duration) -> RunSighting {
        RunSighting {
            successor_seen: true,
            ..silent(heard_ever, live_writers, since)
        }
    }

    /// THE precedence oracle: every arm of `classify_run_signal`, driven
    /// against a HAND-WRITTEN expectation rather than against itself.
    #[test]
    fn every_signal_arm_is_pinned_against_a_hand_written_verdict() {
        // (description, sighting, expected)
        let cases: Vec<(&str, RunSighting, RunSignal)> = vec![
            (
                "a heard Live record is Live",
                RunSighting {
                    own_state: Some(RunState::Live),
                    ..silent(true, 1, Duration::ZERO)
                },
                RunSignal::Live,
            ),
            (
                "a heard Ending record is terminal evidence",
                RunSighting {
                    own_state: Some(RunState::Ending),
                    ..silent(true, 1, Duration::ZERO)
                },
                RunSignal::Ending,
            ),
            (
                "an Ending heard while NOBODY is publishing any more is still Ending — the \
                 writer flips the state and is released microseconds later, so this is the \
                 NORMAL shape of a graceful exit, not a contradiction",
                RunSighting {
                    own_state: Some(RunState::Ending),
                    ..silent(true, 0, Duration::ZERO)
                },
                RunSignal::Ending,
            ),
            (
                "a run HEARD in this very batch outranks a successor sighting — the shorter \
                 replaced grace judges a SILENCE, and this run is not silent",
                RunSighting {
                    own_state: Some(RunState::Live),
                    ..replaced(true, 2, GRACE * 10)
                },
                RunSignal::Live,
            ),
            (
                "heard before, now zero publishers: absence is EXACT, no grace needed",
                silent(true, 0, Duration::from_millis(1)),
                RunSignal::Absent,
            ),
            (
                "NEVER heard and zero publishers: the writer may simply not be up yet — this \
                 must NOT be absence, or a recorder armed a moment early ends itself",
                silent(false, 0, Duration::from_millis(1)),
                RunSignal::Waiting,
            ),
            (
                "never heard, past the grace: the run died before we ever heard it",
                silent(false, 0, GRACE),
                RunSignal::Absent,
            ),
            (
                "heard before, other runs still publishing, quiet past the grace",
                silent(true, 3, GRACE),
                RunSignal::Absent,
            ),
            (
                "heard before, other runs publishing, quiet but INSIDE the grace",
                silent(true, 3, GRACE - Duration::from_millis(1)),
                RunSignal::Waiting,
            ),
            (
                "REPLACED: a successor is announcing and we have been quiet past the SHORTER \
                 grace — the restart shape, and the whole reason that grace exists",
                replaced(true, 3, REPLACED),
                RunSignal::Absent,
            ),
            (
                "a successor is announcing but we have been quiet only a moment: the successor \
                 announces BEFORE it builds, so a zero-grace rule would truncate a live run on \
                 an operator's double-launch",
                replaced(true, 3, REPLACED - Duration::from_millis(1)),
                RunSignal::Waiting,
            ),
            (
                "NEVER heard, a successor announcing: with no first contact there is no identity \
                 to have been replaced — only the full grace may escalate this",
                replaced(false, 3, REPLACED),
                RunSignal::Waiting,
            ),
            (
                "no successor, quiet past the SHORTER grace but inside the full one: silence \
                 alone is judged by the full grace, which is what stops the shorter one leaking \
                 onto the plain-crash shape",
                silent(true, 3, REPLACED),
                RunSignal::Waiting,
            ),
        ];
        for (what, sighting, want) in cases {
            assert_eq!(
                classify_run_signal(&sighting, graces()),
                want,
                "{what}: {sighting:?}"
            );
        }
    }

    /// The REPLACED grace is a threshold of its own, pinned on both sides — and
    /// pinned to be SHORTER than the silence grace, which is the entire point.
    #[test]
    fn the_replaced_grace_is_a_shorter_threshold_pinned_on_both_sides() {
        assert_eq!(
            classify_run_signal(
                &replaced(true, 2, REPLACED - Duration::from_millis(1)),
                graces()
            ),
            RunSignal::Waiting,
            "one millisecond under the replaced grace makes no claim"
        );
        assert_eq!(
            classify_run_signal(&replaced(true, 2, REPLACED), graces()),
            RunSignal::Absent,
            "AT the replaced grace, silence plus a successor is evidence"
        );
        // …and at that same instant, WITHOUT a successor, the answer differs.
        // Without this the arm above is satisfied by a rule that ignores the
        // successor flag and simply uses the shorter grace for everything.
        assert_eq!(
            classify_run_signal(&silent(true, 2, REPLACED), graces()),
            RunSignal::Waiting,
            "the SAME silence with no successor is not yet evidence"
        );
        assert!(
            RUN_REPLACED_GRACE <= RUN_VANISH_GRACE,
            "the shipped pair must SHORTEN the wait on stronger evidence, never lengthen it: \
             replaced={RUN_REPLACED_GRACE:?} vanish={RUN_VANISH_GRACE:?}"
        );
    }

    /// `RunGraces::scaled_from` keeps the shipped RATIO, so an injected grace
    /// shortens both routes in proportion instead of collapsing them onto one
    /// number (which would make every test that injects a grace blind to which
    /// route fired).
    #[test]
    fn an_injected_grace_scales_both_routes_and_keeps_them_distinct() {
        let shipped = RunGraces::shipped();
        assert_eq!(shipped.vanish, RUN_VANISH_GRACE);
        assert_eq!(shipped.replaced, RUN_REPLACED_GRACE);

        let scaled = RunGraces::scaled_from(Duration::from_millis(1_500));
        assert_eq!(scaled.vanish, Duration::from_millis(1_500));
        assert_eq!(
            scaled.replaced,
            Duration::from_millis(600),
            "1.5s at the shipped 2:5 ratio"
        );
        assert!(
            scaled.replaced < scaled.vanish,
            "a scaled pair must stay DISTINCT, or an injected-grace test cannot tell the two \
             routes apart: {scaled:?}"
        );
    }

    /// `Ending` wins within a batch, from BOTH orders — the fold that makes the
    /// long-lived subscriber worth having.
    ///
    /// The shipping cadence produces mixed batches as its NORMAL shape (the
    /// registry republishes every `RUN_REPUBLISH_INTERVAL` while a consumer
    /// looks less often), so a fold that let a `Live` republish mask the
    /// `Ending` sitting behind it in the same queue would report a graceful
    /// exit as a crash on the ordinary path.
    #[test]
    fn ending_wins_within_a_batch_whichever_order_it_arrives_in() {
        // A fresh batch takes whatever it is given.
        assert_eq!(fold_batch_state(None, RunState::Live), Some(RunState::Live));
        assert_eq!(
            fold_batch_state(None, RunState::Ending),
            Some(RunState::Ending)
        );
        // Live then Ending — the flip lands last, the ordinary graceful shape.
        assert_eq!(
            fold_batch_state(Some(RunState::Live), RunState::Ending),
            Some(RunState::Ending)
        );
        // Ending then Live — a stale republish behind the flip must NOT undo it.
        assert_eq!(
            fold_batch_state(Some(RunState::Ending), RunState::Live),
            Some(RunState::Ending)
        );
        // Idempotent on both sides.
        assert_eq!(
            fold_batch_state(Some(RunState::Live), RunState::Live),
            Some(RunState::Live)
        );
        assert_eq!(
            fold_batch_state(Some(RunState::Ending), RunState::Ending),
            Some(RunState::Ending)
        );
        // And folding a whole batch is order-independent, which is the property
        // a consumer relies on: an SHM queue hands records over in commit order,
        // but nothing in the rule may DEPEND on that order.
        let both = [RunState::Live, RunState::Ending];
        let forward = both.iter().fold(None, |acc, s| fold_batch_state(acc, *s));
        let backward = both
            .iter()
            .rev()
            .fold(None, |acc, s| fold_batch_state(acc, *s));
        assert_eq!(forward, Some(RunState::Ending));
        assert_eq!(backward, forward);
    }

    /// What counts as a SUCCESSOR — one conjunct flipped per case, so each is
    /// independently testable.
    #[test]
    fn a_successor_is_a_different_run_of_the_same_graph_started_no_earlier() {
        const OWN_ID: u128 = 0x1111;
        let own = OwnRunIdentity {
            graph_name: "go2_attach".into(),
            run_started_at_ns: 1_000,
        };
        let base = RunRecord {
            run_id: 0x2222,
            supervisor_pid: 7,
            run_started_at_ns: 2_000,
            state: RunState::Live,
            graph_name: "go2_attach".into(),
            run_dir: "/tmp/x".into(),
        };
        assert!(
            is_successor_record(&own, OWN_ID, &base),
            "a later run of the same graph is taking over this recording's topics"
        );
        assert!(
            !is_successor_record(
                &own,
                OWN_ID,
                &RunRecord {
                    run_id: OWN_ID,
                    ..base.clone()
                }
            ),
            "our OWN record is not our successor — a republish must never look like a restart"
        );
        assert!(
            !is_successor_record(
                &own,
                OWN_ID,
                &RunRecord {
                    graph_name: "some_other_graph".into(),
                    ..base.clone()
                }
            ),
            "another GRAPH's run publishes to different topics (the prefix is frozen at \
             graph create), so it takes nothing over — ordinary co-tenancy"
        );
        assert!(
            !is_successor_record(
                &own,
                OWN_ID,
                &RunRecord {
                    run_started_at_ns: 999,
                    ..base.clone()
                }
            ),
            "a same-graph run that started BEFORE us cannot be replacing us — we replaced IT, \
             and a stale record still on the wire must not end a run that just started"
        );
        assert!(
            is_successor_record(
                &own,
                OWN_ID,
                &RunRecord {
                    run_started_at_ns: 1_000,
                    ..base
                }
            ),
            "the start-time bound is INCLUSIVE: a clock tie must not open a hole"
        );
    }

    /// The grace is a THRESHOLD, pinned on both sides at one millisecond.
    #[test]
    fn the_vanish_grace_is_a_threshold_pinned_on_both_sides() {
        assert_eq!(
            classify_run_signal(&silent(true, 2, GRACE - Duration::from_millis(1)), graces()),
            RunSignal::Waiting,
            "one millisecond under the grace makes no claim"
        );
        assert_eq!(
            classify_run_signal(&silent(true, 2, GRACE), graces()),
            RunSignal::Absent,
            "AT the grace the silence becomes evidence"
        );
    }

    /// A run that keeps publishing is never called absent, however long the
    /// watcher runs — the anti-tautology for every absence arm above.
    #[test]
    fn a_run_that_keeps_publishing_is_never_absent() {
        let mut tracker = RunLifetimeTracker::new();
        for _ in 0..1_000 {
            let signal = classify_run_signal(
                &RunSighting {
                    own_state: Some(RunState::Live),
                    ..silent(true, 1, Duration::ZERO)
                },
                graces(),
            );
            assert_eq!(signal, RunSignal::Live);
            assert_eq!(tracker.observe(signal), None, "a live run never ends");
        }
        assert_eq!(tracker.absent_streak(), 0);
    }

    /// ONE missed observation must NOT end a recording: the single-miss
    /// case is exactly what this test targets. The streak is asserted at every step so the arm
    /// cannot pass merely because the outcome stayed `None`.
    #[test]
    fn a_single_absent_observation_never_ends_a_recording() {
        let mut tracker = RunLifetimeTracker::new();
        assert_eq!(tracker.observe(RunSignal::Absent), None);
        assert_eq!(tracker.absent_streak(), 1, "the evidence is banked");
        // A live sighting is proof the run is there — it CLEARS the evidence,
        // which is the whole reason one lost look is survivable.
        assert_eq!(tracker.observe(RunSignal::Live), None);
        assert_eq!(
            tracker.absent_streak(),
            0,
            "a live sighting clears the streak"
        );
        assert_eq!(tracker.observe(RunSignal::Absent), None);
        assert_eq!(tracker.absent_streak(), 1, "the streak begins again");
        assert_eq!(
            tracker.observe(RunSignal::Absent),
            Some(RunEnded::Vanished),
            "the SECOND consecutive absence confirms it"
        );
    }

    /// A `Waiting` reading is not evidence in EITHER direction: it must not
    /// advance the streak, and it must not clear it.
    #[test]
    fn a_waiting_reading_neither_advances_nor_clears_the_evidence() {
        let mut tracker = RunLifetimeTracker::new();
        assert_eq!(tracker.observe(RunSignal::Absent), None);
        assert_eq!(tracker.absent_streak(), 1);
        for _ in 0..50 {
            assert_eq!(
                tracker.observe(RunSignal::Waiting),
                None,
                "a no-claim reading can never confirm an absence on its own"
            );
        }
        assert_eq!(
            tracker.absent_streak(),
            1,
            "and it must not have cleared the banked absence either"
        );
        assert_eq!(tracker.observe(RunSignal::Absent), Some(RunEnded::Vanished));
    }

    /// A heard `Ending` is terminal on its FIRST sighting — it needs no
    /// confirmation, because no transient fabricates one.
    #[test]
    fn a_graceful_end_is_terminal_on_the_first_sighting() {
        let mut tracker = RunLifetimeTracker::new();
        assert_eq!(tracker.observe(RunSignal::Ending), Some(RunEnded::Graceful));
        assert_eq!(tracker.outcome(), Some(RunEnded::Graceful));
    }

    /// The outcome is STICKY, and the FIRST one wins. A recorder acts on this
    /// verdict, so a later observation must not walk it back — and a run that
    /// went gracefully must not be re-reported as vanished a moment later when
    /// its writer disappears (which it always does, immediately).
    #[test]
    fn the_first_outcome_is_final_and_a_graceful_end_is_never_downgraded() {
        let mut tracker = RunLifetimeTracker::new();
        assert_eq!(tracker.observe(RunSignal::Ending), Some(RunEnded::Graceful));
        for _ in 0..10 {
            assert_eq!(
                tracker.observe(RunSignal::Absent),
                Some(RunEnded::Graceful),
                "the writer vanishing AFTER a graceful end is expected, not a downgrade"
            );
        }

        let mut vanished = RunLifetimeTracker::new();
        vanished.observe(RunSignal::Absent);
        assert_eq!(
            vanished.observe(RunSignal::Absent),
            Some(RunEnded::Vanished)
        );
        assert_eq!(
            vanished.observe(RunSignal::Ending),
            Some(RunEnded::Vanished),
            "a verdict already acted on must not flip"
        );
    }

    /// The wire spelling of the two outcomes, pinned: they are stamped into a
    /// durable artifact and read by an operator.
    #[test]
    fn the_outcome_labels_are_the_pinned_strings() {
        assert_eq!(RunEnded::Graceful.to_string(), "graceful");
        assert_eq!(RunEnded::Vanished.to_string(), "vanished");
    }

    /// The shipped grace keeps the relationship its doc claims, stated in the
    /// terms a reader would otherwise have to derive.
    ///
    /// The confirmation floor is NOT re-asserted here: it is const-asserted at
    /// the module level, so a violation fails the BUILD rather than a test.
    #[test]
    fn the_shipped_grace_stays_far_above_the_republish_cadence() {
        assert!(
            RUN_VANISH_GRACE >= RUN_REPUBLISH_INTERVAL * 10,
            "a grace within a coalescing-stretched republish interval would call a HEALTHY run \
             vanished: grace={RUN_VANISH_GRACE:?} interval={RUN_REPUBLISH_INTERVAL:?}"
        );
    }
}
