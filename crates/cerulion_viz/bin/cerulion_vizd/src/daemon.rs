// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion-vizd` daemon runtime.
//!
//! One long-lived process, ONE iceoryx2 node (Principle #8), owning:
//!
//! - a [`TapManager`] of listener-less runtime taps,
//! - a POLL THREAD (~16 ms) that drains every tap → per-tick [`InputFrames`] →
//!   the never-block [`VizLogWorker`] (which owns the Rerun `RecordingStream`
//!   off the poll thread), updating per-topic Hz/schema stats from the drained
//!   frames' wire headers,
//! - a UDS CONTROL SERVER accepting MULTIPLE concurrent controllers (Studio +
//!   agent + the `cerulion viz` verb), each a thread reading NDJSON requests +
//!   writing NDJSON responses.
//!
//! The daemon is DEPENDENCY-INJECTED (manager + worker + walker) so tests drive
//! an in-process daemon over an isolated per-test SHM root against a memory Rerun
//! sink; `main.rs` wires the production singleton transport + the real gRPC
//! stream.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io::{self, Read};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::codegen::unknown_hash::{
    diagnose_unknown_hash_with_walker, report_schema_hash_resolved, report_unknown_schema_hash,
    DiagnosisVantage, UnknownHashDiagnosis,
};
use cerulion_core::codegen::FrameWalker;
use cerulion_core::transport::cerulion_q::format_run_id;
use cerulion_core::transport::discovery::{query_robot_catalog, query_robot_schema};
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::transport::liveness::LIVENESS_STREAMING_RECENCY_MS;
use cerulion_core::transport::network::NetworkManager;
use cerulion_core::transport::run_artifacts::collect_run_entries;
use cerulion_core::transport::run_registry::RUN_GATHER_WINDOW;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::WireHeader;
use cerulion_core::{
    write_line_bounded, CatalogEntry, CatalogReply, SchemaDoc, SchemaReply, TopicLiveness,
    TopicRateEstimate, UdsWriteOutcome,
};
use cerulion_netd::protocol::DiscoveryState;
use cerulion_netd::{CatalogGather, ClientError, NetdClient, RunsGather};
use cerulion_viz::schema_registry::walker_from_bridge_config_env_with_docs;

use cerulion_viz::blueprint::{
    archetype_components, archetype_wire_name, compose_layout as compile_layout,
    default_layout_excluding, plan_origins_unmatched, set_attached_entities_snapshot,
    validate_plan_against_attached, views_for_render, AttachedRender, BlueprintPlan, ComposeGroup,
    ComposeInput, ComposeStrategy, ContainerKind, GroupRole, LayoutError, PlanContainer, PlanNode,
    PlanView, ViewKind,
};
use cerulion_viz::monitor::{MonitorSample, MONITOR_SAMPLE_INTERVAL_NS};
use cerulion_viz::pointcloud::{FieldsLogAction, FieldsWarnLatch};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::{
    classify_frame, classify_schema, non_tf_entity_for, reported_entity_for, route_key_for_topic,
    route_key_topic, ArchetypeKind, RenderProof,
};
use cerulion_viz::tap_manager::{AttachOutcome, TapManager, WakeMode};
use cerulion_viz::worker::{InputFrames, VizControl, VizLogWorker, VizWorkerCounters};

use crate::hygiene::{acquire_socket, SocketGuard};
use crate::monitors::{MonitorPlane, WriteStamp};
use crate::protocol::{
    parse_request, AttachResponse, AttachedEntry, ComposeLayoutResponse, ContainerSpec,
    DetachResponse, DiscoverResponse, DiscoveredEntry, DiscoveredRobot, DiscoveryLabel, Hello,
    LayoutIntent, LayoutNode, LayoutSpec, ListResponse, MonitorsResponse, Placement,
    RepresentationResponse, Request, Response, RetryHint, RunsResponse, SetBlueprintResponse,
    SkippedDead, StatusResponse, TopicStatus, UndecodableReport, ViewSpec, WorkerStatus,
};
use crate::runs::{fold_runs, local_reply, LocalArm, RemoteArm};

/// The shared retry clause of every UNKNOWN-verdict message vizd renders for a
/// topic it could not resolve — the two arms of [`resolve_remote_attach_target`] and
/// [`catalog_absence_verdict`].
///
/// ONE spelling, for the reason `cerulion_cli_engine`'s twin gives: three copies of one
/// claim drift, and the convergence-wait rewording was applied to the CLI's copies while these
/// three kept the old text, which is the second item of this rework.
///
/// **What was wrong with the old text.** It read "Retry — discovery often converges
/// within a second or two". Real-LAN convergence was MEASURED at ~3–13 s from a cold
/// desk (the measurement the convergence wait is built on), so the promise was false on its face.
///
/// Every clause has to be true in EVERY rendering state, and this
/// message renders whether or not the CLIENT chooses to keep asking — vizd itself
/// answers in one round trip (see [`Ctx::gather_catalog`]). So it names the SHAPE of the
/// failure rather than a duration, and says "may not be discoverable YET" rather than
/// implying anybody is waiting. The machine-readable half of the same statement is
/// [`RetryHint`], which is what a client acts on. No trailing punctuation: each site
/// appends its own remedy tail.
const RETRY_CLAUSE: &str = "Retry in a few seconds — a robot that is still booting, or \
     joining the network, may not be discoverable yet; if it keeps failing, check the \
     robot is powered on and on this network";

/// An attach failure, plus the structured [`RetryHint`] when the failure is a
/// NOT-FOUND-**YET** that re-asking could change.
///
/// It exists because the retryability verdict is decided DEEP in the resolve chain —
/// `catalog_resolve_type` knows whether THIS robot's catalog named a type for THIS
/// topic — while the `Response` is built at the top, in `attach_remote`. Threading a
/// bare `String` up that chain loses the verdict, and re-deriving it at the top would
/// mean string-matching prose the daemon is free to reword: exactly what the hint
/// exists to stop a CLIENT from having to do.
///
/// [`From<String>`] keeps every unrelated `Err` on the chain compiling unchanged, and
/// makes TERMINAL the default: a failure is retryable only where a seam says so.
#[derive(Debug)]
struct AttachError {
    message: String,
    retry: Option<RetryHint>,
}

impl From<String> for AttachError {
    fn from(message: String) -> Self {
        Self {
            message,
            retry: None,
        }
    }
}

impl AttachError {
    /// A NOT-FOUND-YET: the thing asked for was absent from a discovery answer, and the
    /// answering daemon's state is forwarded so the client can run the shipped policy.
    fn not_found_yet(message: String, gather: &CatalogGather) -> Self {
        Self {
            message,
            retry: Some(retry_hint_for(gather)),
        }
    }
}

/// Build the wire [`RetryHint`] from the gather that produced the failure.
///
/// PURE. `unsettled_for` is forwarded verbatim (including `None`, which is UNKNOWN and
/// caps nothing — the same reading `ConvergenceWait::decide` gives it, so the rule is
/// identical on both sides of the wire).
fn retry_hint_for(gather: &CatalogGather) -> RetryHint {
    RetryHint {
        discovery: DiscoveryLabel::from_state(gather.discovery),
        plane_unsettled_ms: gather
            .unsettled_for
            .map(|d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX)),
    }
}

/// The default poll cadence — drain every tap ~60×/s (the worker's
/// queue is ≈130 ms deep at this rate).
pub const DEFAULT_POLL_INTERVAL: Duration = Duration::from_millis(16);

/// Max frames drained from EACH tap per poll (well above any realistic per-tick
/// burst; a tap that has more is drained fully over the next polls, no loss).
const POLL_MAX: usize = 4096;

/// How many wake sources the frame-drain multiplexer pre-sizes
/// its fired-set buffers for.
///
/// A HINT, not a cap — the real cap is `WAITSET_MAX_ATTACHMENTS` (1024),
/// enforced per wait. Sized at a Go2-class attach set (a `ros2 attach` robot
/// announces ~75 topics and a user watches a handful), so a steady-state wait
/// records into already-reserved space; a set that outgrows it simply grows the
/// vector once.
const WAKE_SET_CAPACITY_HINT: usize = 32;

/// The wall window over which per-topic Hz is measured (wire-sequence delta ÷
/// window). Long enough to smooth jitter, short enough that a stopped
/// topic's rate decays visibly.
const HZ_WINDOW: Duration = Duration::from_secs(5);

/// How recently a frame must have ARRIVED at this daemon's tap for a
/// rate to still be a claim about the present.
///
/// This is [`LIVENESS_STREAMING_RECENCY_MS`] — deliberately the SAME threshold
/// the liveness classifier calls [`cerulion_core::LivenessState::Streaming`], so "shows a rate"
/// and "reads streaming" cannot disagree on one row. A rate outside it is not
/// merely imprecise, it is a claim about a stream that has stopped, and
/// [`TopicStat::hz`] returns `None` there so the row falls through to the
/// liveness verdict instead.
const HZ_STREAMING_RECENCY: Duration = Duration::from_millis(LIVENESS_STREAMING_RECENCY_MS);

/// Best-effort bound for the transient discover/attach schema peek (a live topic
/// yields a frame well within this; a silent one returns "unresolved").
const PEEK_BOUND: Duration = Duration::from_millis(250);

/// TOTAL peek budget for ONE `discover` sweep.
/// Without it, N silent topics cost N × [`PEEK_BOUND`] SEQUENTIALLY — a controller
/// blocked for seconds on a robot with many idle topics. Cached topics resolve
/// instantly (no budget cost); uncached topics peek only while budget remains,
/// else return `schema: null` (no guess — the same contract `attach` gives a silent
/// topic).
const DISCOVER_PEEK_BUDGET: Duration = Duration::from_millis(500);

/// TOTAL archetype-peek budget for ONE `compose_layout` request.
/// `compose_resolve_topic` peeks each freshly-attached silent topic up
/// to `PEEK_BOUND`; over N such topics that would block the control connection
/// N × `PEEK_BOUND` SEQUENTIALLY (a 90-topic robot ≈ 22 s). A shared per-request
/// deadline caps the WHOLE batch: cached topics cost nothing, and once the budget
/// is spent the rest resolve to `None` (still-silent → placed by role or a loud
/// NoRenderableTopics — the same contract a single silent topic gets).
///
/// `pub` so the e2e pin DERIVES its wall ceiling from this value instead of
/// hardcoding a number that silently stops being a ceiling when the budget is
/// retuned (`CONN_READ_TIMEOUT` above is `pub` for the same reason). The pin is
/// `vizd_e2e_test::compose_over_many_silent_topics_is_bounded_by_one_shared_peek_budget`,
/// the compose half of the budget PAIR — `discover`'s half
/// (`DISCOVER_PEEK_BUDGET`) is pinned separately and cannot stand in for it: the
/// two handlers compute their own deadlines and call DIFFERENT helpers
/// (`resolve_for_attach_bounded` vs `resolve_for_discover`), so a variant that
/// removes only this cap leaves the discover pin green.
pub const COMPOSE_PEEK_BUDGET: Duration = Duration::from_millis(750);

/// How far a topic's wire sequence may move BACKWARD before the daemon treats it
/// as a producer restart (sequence reset to 0) rather than in-window reorder, and
/// RESETS the Hz window so the fresh post-restart rate computes promptly instead
/// of being masked by the stale high sequences lingering at the window front.
/// The daemon observes only the newest frame of
/// each poll from an in-order transport, so any backward step this large is a
/// restart; a small tolerance guards against benign jitter (whose ordered delta is
/// handled wrap-safely by [`TopicStat::hz`]'s `saturating_sub`).
const SEQ_RESET_TOLERANCE: u32 = 8;

/// Bounded gather window for ONE remote schema-catalog / schema-closure GET over
/// the daemon's zenoh session (remote arm). Matches the CLI
/// engine's `CATALOG_GATHER_WINDOW` — long enough for a robot's gateway to answer
/// a directed GET, short enough that a schema-less remote attach on an unreachable
/// robot fails fast (a hard, precise error) instead of hanging the controller.
const CATALOG_GATHER_WINDOW: Duration = Duration::from_millis(250);

/// How often the accept loop re-checks the shutdown flag between nonblocking
/// `accept`s (bounds shutdown latency without a busy spin).
const ACCEPT_POLL: Duration = Duration::from_millis(25);

/// Per-connection read timeout — the reader re-checks the shutdown flag this
/// often while idle, so a connected-but-idle controller never wedges teardown, and
/// it is what PACES that loop: an idle controller goes round it once per
/// this interval, never faster. `pub` so the e2e derives its iteration bound from
/// the shipped value rather than a literal.
pub const CONN_READ_TIMEOUT: Duration = Duration::from_millis(200);

/// Per-connection WRITE NO-PROGRESS budget.
/// A controller that stops READING lets the UDS send buffer fill, so a
/// write to it can place no further bytes. [`write_line`] uses the shared bounded
/// resume loop ([`cerulion_core::write_line_bounded`]): a big response RESUMES once
/// the controller drains, and the connection is dropped only when NO forward
/// progress happens within this budget — the genuine "stopped reading" signal that
/// keeps [`RunningDaemon::shutdown`]'s `join` over the connection thread bounded
/// (SIGINT always finishes). A dead/wedged controller trips it.
const CONN_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// Max bytes a single NDJSON request line may accumulate BEFORE its terminating
/// newline. Control requests are tiny JSON
/// lines — the largest, an `attach` with `topic` + `entity` + `robot`, is a few
/// hundred bytes — so 1 MiB is enormously generous while still bounding memory.
/// Without this cap the per-connection read loop's `pending` buffer grows
/// UNBOUNDED on a controller that writes bytes but never a `\n` (a buggy
/// serializer, a partial flush, or a malicious local client) → OOM for a
/// long-lived daemon. When an UNTERMINATED line exceeds this, the daemon
/// best-effort writes ONE structured error response, warns once, and drops THAT
/// connection — it never blocks, loses other controllers, or dies. (A COMPLETE
/// line is always drained before this check, so a legitimately large-but-newline-
/// terminated request is never rejected; only unbounded no-newline growth is.)
pub const MAX_REQUEST_LINE_BYTES: usize = 1024 * 1024;

/// A topic's resolved schema + archetype, computed ONCE from the first decodable
/// frame (a topic's schema is stable). Until that frame, a remote attach may hold a
/// guess made from the pinned schema NAME; [`TopicStat::resolved_from_frame`] says
/// which one this is.
#[derive(Debug, Clone)]
struct Resolved {
    /// The qualified schema name (`pkg/Type`).
    schema: String,
    /// The Rerun archetype family the sink renders it as.
    archetype: ArchetypeKind,
}

/// Per-tapped-topic live stats: the resolved render route, a sliding Hz window,
/// the drained-frame total, and the once-resolved schema/archetype.
#[derive(Debug, Default)]
struct TopicStat {
    /// The render route key resolved at attach (last path segment or override).
    route_key: Option<String>,
    /// Sliding window of `(wire_seq, observed_at)` for the newest frame of each
    /// yielding poll — the Hz basis.
    samples: VecDeque<(u32, Instant)>,
    /// Total frames drained from this tap.
    frames_seen: u64,
    /// The resolved schema + archetype: the verdict on the first decodable frame,
    /// or, before any frame, the remote attach's guess from the pinned schema name.
    resolved: Option<Resolved>,
    /// `resolved` came from a DRAINED FRAME, classified by the sink's whole ladder
    /// (`classify_frame`, content rungs included). False while it is only the
    /// remote attach's schema-NAME guess.
    ///
    /// The name table cannot see content: it maps every
    /// `sensor_msgs/CompressedImage` to `Image`, while the sink draws one whose
    /// `data` is an H.264 access unit as `VideoStream`. So the poll thread keeps
    /// classifying until this is true, and a re-attach never writes the name guess
    /// over a frame verdict. It is never cleared; `detach` drops the whole stats
    /// entry.
    resolved_from_frame: bool,
    /// The ORIGIN ROBOT whose data THIS tap carries.
    ///
    /// Written only through [`DaemonState::set_origin_robot`], which releases the
    /// monitor row the tap's PREVIOUS identity opened — rows are keyed
    /// `(topic, robot)`, so a bare assignment would strand one. Read
    /// [`Self::origin_robot_hinted`] for the two very different kinds of claim
    /// that reach it.
    ///
    /// Deliberately NOT [`DaemonState::remote_provenance`], which answers a
    /// DIFFERENT question, "where did a topic of this NAME last live", the remote-provenance
    /// routing memory, never cleared so a re-check can re-demand from the right
    /// robot. Attribution needs "whose data is this tap carrying", and the two
    /// diverge the moment a name is REUSED: attach a robot's `/tf`, detach it
    /// (netd's refcount-0 teardown genuinely frees the single-writer slot), start a
    /// desk graph publishing its own `/tf`, and attach that. A name-keyed lookup
    /// re-satisfies on the NEW tap and attributes a desk-owned topic to the
    /// historical robot on every surface. This field lives and dies with the tap —
    /// `detach` removes the whole stats entry, so the next attach starts from
    /// `None` — so it cannot outlive the generation that earned it.
    origin_robot: Option<String>,
    /// Is [`Self::origin_robot`] a HINT rather than first-hand knowledge?
    ///
    /// The two kinds of claim are not interchangeable, and conflating them is a
    /// the attribution REGRESSION with the sign flipped, an affirmatively WRONG robot on
    /// a desk-owned topic, which is worse than no robot at all:
    ///
    /// * **`false` — AUTHORITATIVE.** The REMOTE attach path DEMANDED this topic
    ///   from that robot. First-hand, and never revised by anything below: when
    ///   netd's teardown later removes the mirror's provenance under a still-held
    ///   tap, the row must STAY on its robot rather than flickering into the desk's
    ///   own section (`a_torn_down_mirror_keeps_its_attached_row_on_its_robot_e2e`).
    /// * **`true` — a HINT** read out of the `/__cerulion/mirrors` map, which is
    ///   keyed by NAME and therefore describes a topic's name rather than this
    ///   tap's generation. That is exactly the tombstone hazard this field's
    ///   sibling doc describes, one step removed: retire a mirror, let a desk
    ///   producer take the freed name, and a snapshot taken before the retirement
    ///   still names the historical robot for up to [`PROVENANCE_CACHE_TTL`].
    ///
    /// So a hint is never allowed to become sticky. It is REFRESHED — re-pointed
    /// or CLEARED — from the live map on every sampling pass
    /// ([`DaemonState::refresh_hinted_origin_robots`]), which makes it a per-tap
    /// cache of the live answer rather than a record of a past one. See that
    /// method for the bound.
    origin_robot_hinted: bool,
    /// When this tap's DATA-FLOW OBSERVATION began, or `None` when this
    /// entry is not observing anything.
    ///
    /// `None` is a real state, not an oversight: `resolve_for_discover` mints a
    /// stats entry to cache a resolved schema for a topic nobody attached, and
    /// that entry has observed nothing. Only [`TopicStat::begin_observation`] —
    /// called where a tap is actually created — sets it, so an un-attached entry
    /// reports UNKNOWN rather than a fabricated "watched, saw nothing" verdict.
    observed_since: Option<Instant>,
    /// When a frame last ARRIVED at this tap, dated by THIS daemon's own
    /// monotonic clock — never by the publisher's wire stamp.
    ///
    /// The wire stamp is the publisher's clock, and under multi-process execution a
    /// worker's `VirtualClock` restarts at 0 on every run, so comparing it against
    /// the daemon's clock compares two unrelated number lines (the exact hazard
    /// the liveness module's docs record). Arrival at our OWN tap needs no clock
    /// agreement with anyone.
    ///
    /// `None` while only the BASELINE batch has arrived — see
    /// [`TopicStat::observe_frames`].
    last_frame_at: Option<Instant>,
    /// The qualified type name this topic is BELIEVED to carry,
    /// recorded by the REMOTE attach path from the catalog / the attach pin.
    ///
    /// Kept SEPARATELY from [`Self::resolved`] because that one is only written
    /// when the name also maps to an archetype — so on exactly the topics that
    /// fail to decode (an unmapped or unknown type) the name was computed and
    /// then thrown away. It is the ONLY out-of-band name this daemon has, and it
    /// is what turns an unresolvable wire hash from "0x…" into "you hold this
    /// type at a different hash — rebuild" or "you never compiled this type —
    /// acquire it". The wire carries no name, so without it no such verdict is
    /// possible.
    pinned_schema: Option<String>,
    /// The standing verdict for a topic whose frames this build cannot
    /// decode, served on `list` / `status` / `discover`. `None` = decodes fine,
    /// or nothing has been observed yet.
    undecodable: Option<UndecodableReport>,
    /// The flood latch for that condition — an undecodable topic fails
    /// EVERY poll until somebody redeploys or seeds a schema, so the report is
    /// loud once per regime, re-announced at each decade of the running total,
    /// and `debug!` in between.
    unknown_hash: FailureRegimeLatch,
    /// The newest wire sequence the PREVIOUS yielding poll drained.
    ///
    /// Kept OUTSIDE [`Self::samples`], which [`observe_seq`](Self::observe_seq)
    /// prunes to `HZ_WINDOW`: a topic slower than that window would lose its
    /// baseline every poll and could never be gap-checked at all.
    last_newest_seq: Option<u32>,
    /// The running total of frames this tap NEVER delivered — the
    /// pre-decode loss a `drop_oldest` subscriber queue performs SILENTLY.
    ///
    /// UNCONDITIONAL (Principle #3): bumped whatever the log level, never reset
    /// by recovery, and served on `status` so the question "did the desk lose
    /// frames?" is answerable by reading a number instead of running a bespoke
    /// stride-1 `CERULION_H264_LAT_PROBE` capture and joining it by hand.
    frames_missed: u64,
    /// The flood latch for that condition. A tap that has fallen behind
    /// stays behind, so an unlatched report would fire at the poll cadence — the
    /// disk-fill class.
    gap_latch: FailureRegimeLatch,
}

/// Everything ONE `discover` row reads out of `state`, captured
/// under a SINGLE lock acquisition by [`Ctx::discover_row_state`].
///
/// The type exists so the three values cannot be re-read independently: a row that
/// looked each up on its own could pair a representation from one snapshot with a
/// route from another, which is the drift this type prevents. See
/// [`Ctx::discover_row_state`] for what that produced.
struct DiscoverRowState {
    /// The operator's remembered choice for this topic (`Auto` unless set).
    representation: Representation,
    /// `Some(route_key)` iff this desk CURRENTLY holds a tap on the topic — the key
    /// the worker's live-signal mirrors are indexed by. `None` = nothing here has
    /// rendered it, so the default is the accurate report.
    route: Option<String>,
    /// The standing undecodable verdict, absent for an un-tapped topic.
    undecodable: Option<UndecodableReport>,
}

/// How many frames the tap never delivered between two consecutive
/// YIELDING polls — `None` when no claim can be made.
///
/// Within one poll a drained batch is CONTIGUOUS by construction: `drop_oldest`
/// evicts at the queue HEAD, and the drain then reads what remains in order. So
/// the only place a gap can hide is BETWEEN two polls, and it is exactly the
/// amount by which the newest sequence advanced FURTHER than the number of
/// frames handed over.
///
/// `None` is a real answer and not a zero:
/// * no baseline yet — a tap legitimately attaches mid-stream, so a first
///   observation at `seq > 0` is not a loss (the `topic echo` rule);
/// * a sequence REGRESSION — a producer restart re-bases the counter, and a
///   `u32` wrap is treated the same way (once per ~4.5 years at 30 Hz, costing
///   one skipped claim rather than a fabricated 4-billion-frame loss).
///
/// Scope: `WireHeader::sequence` is PER-PUBLISHER. Every topic vizd taps is
/// single-writer (a graph topic by contract, a netd mirror by construction), so
/// a gap is a true loss signal. On an opted-in `multi_publisher_topics:` topic
/// the interleaved counters would make it noise — the same caveat `topic echo`
/// carries, and the same reason it is a counter rather than a refusal.
fn frames_missed_between(prev_newest: Option<u32>, newest: u32, drained: u64) -> Option<u64> {
    let prev = prev_newest?;
    // `checked_sub` → `None` on a regression: re-baseline, claim nothing.
    let advanced = u64::from(newest.checked_sub(prev)?);
    Some(advanced.saturating_sub(drained))
}

impl TopicStat {
    /// Start this tap's data-flow observation at `now` — idempotent, so
    /// re-entering an existing observation never restarts the clock.
    ///
    /// Called where a tap is CREATED. `detach` drops the whole stats entry, so a
    /// re-attach genuinely begins a new observation (and a new baseline).
    fn begin_observation(&mut self, now: Instant) {
        self.observed_since.get_or_insert(now);
    }

    /// Record that `count` frames were drained from this tap at `now`.
    ///
    /// **The FIRST yielding drain is a BASELINE: banked, never dated.** iceoryx2
    /// flushes a publisher's RETAINED HISTORY into a freshly-connected subscriber,
    /// so the first frames a tap ever sees may have been published long before it
    /// attached — a dead route with history is indistinguishable from a live one
    /// at that instant. Dating it would render a dead route `Streaming`. This is
    /// the liveness observer's advancement rule in the form vizd needs it: we bank the frames
    /// (so the topic is never libeled [`cerulion_core::LivenessState::NoData`] — it demonstrably
    /// carries data) and date from the SECOND drain onward, where an arrival
    /// proves the publisher committed a frame while we were already watching.
    ///
    /// Residual, deliberate: a history flush SPLIT across two poll ticks dates on
    /// its second chunk, so a dead-but-latched route can read `Streaming` for up
    /// to [`HZ_STREAMING_RECENCY`] once per attach before settling to `Idle`. It
    /// is bounded and self-correcting; the alternative is a second copy of
    /// the liveness observer's two-guard stamp-advancement machine on this side of the wire.
    fn observe_frames(&mut self, count: u64, now: Instant) {
        if count == 0 {
            return;
        }
        let baseline = self.frames_seen == 0;
        self.frames_seen += count;
        if !baseline {
            self.last_frame_at = Some(now);
        }
    }

    /// This tap's own data-flow observation in the robot-side liveness vocabulary, or
    /// `None` (UNKNOWN) when this entry is not observing.
    ///
    /// Deliberately the SAME [`TopicLiveness`] the robot-side observer reports, so
    /// [`TopicLiveness::state`] classifies BOTH with ONE oracle-tested set of
    /// thresholds and the sidebar never learns a second vocabulary.
    fn liveness(&self, now: Instant) -> Option<TopicLiveness> {
        let since = self.observed_since?;
        Some(TopicLiveness {
            last_frame_age_ms: self
                .last_frame_at
                .map(|at| now.saturating_duration_since(at).as_millis() as u64),
            observed_for_ms: now.saturating_duration_since(since).as_millis() as u64,
            frames_observed: self.frames_seen,
            // The desk's own per-tap rate, in the rate estimate's vocabulary: the same wire-
            // sequence delta `TopicStatus::hz` reports on the `status` surface (see
            // `desk_rate_estimate` for why it is the robot observer's non-floor
            // quantity). Carried here so `discover` — the surface the graph panel
            // rates its nodes and edges from — says the same number about a tapped
            // LOCAL topic that `status` does, instead of leaving it unmeasured.
            // `None` (window not yet primed, or the stream stopped) serializes absent.
            rate_estimate: self.hz(now).map(desk_rate_estimate),
        })
    }

    /// Record the newest frame's wire sequence at `now`, pruning the window to
    /// [`HZ_WINDOW`].
    ///
    /// A producer RESTART resets its wire sequence to 0, so the newest seq drops
    /// well below the last observed one. Left unhandled, the stale high sequences
    /// would linger at the window FRONT until they age out of [`HZ_WINDOW`],
    /// masking the fresh post-restart rate (`saturating_sub` reads newest ≤ oldest
    /// as ~0 Hz). Detect a backward jump beyond [`SEQ_RESET_TOLERANCE`] and RESET
    /// the window (start fresh at the new sequence) so the new rate computes
    /// promptly. (Both halves are wrap-safe: an in-tolerance backward jitter that
    /// is NOT reset is handled by [`hz`](Self::hz)'s `saturating_sub`.)
    fn observe_seq(&mut self, seq: u32, drained: u64, now: Instant, topic: &str) {
        // Pre-decode loss accounting, BEFORE the Hz window is touched.
        //
        // This runs on the newest frame of a YIELDING poll, which is where the
        // evidence lives: `drained` is what the tap handed over this poll, `seq`
        // is where the producer's counter had reached, and any surplus is what
        // the subscriber queue evicted while we were not looking.
        match frames_missed_between(self.last_newest_seq, seq, drained) {
            Some(0) => {
                // A clean poll ENDS the regime — the house contract, so a LATER
                // regime is loud again instead of silent for the rest of the run.
                if let Some(suppressed) = self.gap_latch.on_success() {
                    tracing::info!(
                        topic = %topic,
                        suppressed_count = suppressed,
                        total_failures = self.gap_latch.total_failures(),
                        frames_missed = self.frames_missed,
                        "vizd: tap is keeping up again — the pre-decode frame-loss regime healed \
                         (the running total is NOT reset: it is the run's loss, not the regime's)"
                    );
                }
            }
            Some(missed) => {
                self.frames_missed += missed;
                report_tap_gap(&mut self.gap_latch, topic, missed, self.frames_missed);
            }
            // No baseline / a sequence regression: no claim either way, and the
            // regime is left exactly as it was.
            None => {}
        }
        self.last_newest_seq = Some(seq);
        if let Some(&(last_seq, _)) = self.samples.back() {
            if seq < last_seq && last_seq - seq > SEQ_RESET_TOLERANCE {
                // Producer restart (or a schema/tap re-create): the old high
                // sequences are meaningless against the new low ones — drop them.
                self.samples.clear();
            }
        }
        self.samples.push_back((seq, now));
        while let Some(&(_, t)) = self.samples.front() {
            if now.duration_since(t) > HZ_WINDOW {
                self.samples.pop_front();
            } else {
                break;
            }
        }
    }

    /// Publish rate in Hz = (newest_seq − oldest_seq) ÷ (now − oldest_time), or
    /// `None` until ≥2 samples span positive time.
    ///
    /// **`None` once the newest sample is older than
    /// [`HZ_STREAMING_RECENCY`] — a rate is a claim about NOW.** Using `now` as
    /// the window end means a stopped topic's numerator is fixed while `dt` grows
    /// without bound, so the reported rate decays toward 0 and NEVER reaches it:
    /// the samples are pruned only by [`observe_seq`], which a dead publisher
    /// never calls, so without this gate `hz` serves `0.4`, `0.3`, `0.2`… for as long as the
    /// daemon runs: a topic whose publisher was
    /// killed sits at "0.3 Hz" indefinitely while `cerulion topic hz` correctly says
    /// not-published. That is worse than no number, because a decaying rate
    /// reads as a slow stream rather than a dead one. Past the recency bound the
    /// right answer is not a smaller number, it is the liveness verdict
    /// ([`Self::liveness`]), so this returns `None` and the row falls through to
    /// it.
    ///
    /// The gate is on the newest ARRIVAL, not on `dt`, so the bound is explicit and
    /// worth stating exactly ("a genuinely SLOW
    /// topic is unaffected" is only true up to a point): a topic whose period
    /// is UNDER [`HZ_STREAMING_RECENCY`] keeps reporting a rate indefinitely, and one
    /// whose period is OVER it loses its rate between publishes and regains it on
    /// each arrival. At the shipped 5 s that covers every control and sensor route;
    /// a `/goal_pose`, a replanned `/plan` or a 30 s diagnostics roll-up genuinely
    /// falls outside it and reports no rate for most of its cycle.
    ///
    /// That is the correct answer rather than a defect: the alternative is a rate
    /// computed over a window with no frame in it, which is the decayed number this
    /// gate exists to stop. Such a topic is not left unexplained — it classifies
    /// [`cerulion_core::LivenessState::Idle`] with a DATED age, and the sidebar
    /// renders that as a muted "last seen Ns ago" rather than the red "no data"
    /// reserved for a topic nothing is publishing to.
    ///
    /// [`observe_seq`]: Self::observe_seq
    ///
    /// The seq delta is `saturating_sub`, NOT `wrapping_sub`: [`observe_seq`]
    /// only CLEARS the window on a backward jump BEYOND [`SEQ_RESET_TOLERANCE`],
    /// so a small in-tolerance backward jitter (benign reorder) is admitted and
    /// `back().seq` can sit at or below `front().seq`. A wrapping delta there
    /// wraps to ~2³² → ~4.3 billion Hz for up to [`HZ_WINDOW`]. Saturating to 0
    /// reports the true rate: newest ≤ oldest means no net forward publish
    /// progress across this window, i.e. ~0 Hz — never a wrap-around blow-up.
    ///
    /// [`observe_seq`]: Self::observe_seq
    fn hz(&self, now: Instant) -> Option<f64> {
        if self.samples.len() < 2 {
            return None;
        }
        let &(s1, t1) = self.samples.back()?;
        // The stream has stopped — serve the verdict, never a decayed rate.
        if now.saturating_duration_since(t1) > HZ_STREAMING_RECENCY {
            return None;
        }
        let &(s0, t0) = self.samples.front()?;
        let dt = now.checked_duration_since(t0)?.as_secs_f64();
        if dt <= 0.0 {
            return None;
        }
        Some(s1.saturating_sub(s0) as f64 / dt)
    }
}

/// Monitors: the desk's own per-tap rate, in the substrate's vocabulary.
///
/// `is_floor: false` is a CLAIM and it is the right one. The substrate reserves the floor
/// label for a window counted from FRAMES DRAINED, whose value is clipped by the
/// tap's queue depth and is therefore only a lower bound. [`TopicStat::hz`] is not
/// that: its numerator is the delta of the publisher's own WIRE SEQUENCE
/// (`observe_seq`), which is the commit counter the substrate's own non-floor arm reads,
/// so the two rates are the same quantity measured the same way over different
/// windows.
///
/// SCOPE, stated at the same strength `frames_missed_between` states it:
/// `WireHeader::sequence` is PER-PUBLISHER, and every topic vizd taps is
/// single-writer (a graph topic by contract, a netd mirror by construction). On an
/// opted-in `multi_publisher_topics:` topic the interleaved counters would make the
/// number noise — the same caveat that surface already carries, and the reason the
/// monitor's own `ever_floor` gate exists for the rows the substrate CAN label.
///
/// The cast saturates (Rust float→int casts have since 1.45), and `hz` is finite
/// and non-negative by construction (`saturating_sub` over a positive `dt`), so no
/// value of it can produce a wrapped millihertz.
fn desk_rate_estimate(hz: f64) -> TopicRateEstimate {
    TopicRateEstimate {
        millihertz: (hz * 1_000.0).round() as u64,
        is_floor: false,
    }
}

/// Monitors: ONE [`MonitorSample`] per attached tap, all dated from the
/// SAME instant.
///
/// Modelled on [`DiscoverRowState`]: the point of the type there, and of the
/// signature here, is that a row's values cannot be re-read independently. Which is
/// why this takes the pieces rather than `&self` — a `&self` method would have to
/// acquire the state lock itself, and the sampler needs the snapshot and the
/// engine pass to happen under ONE acquisition or a concurrent `detach` can land
/// between them and resurrect the row it just released.
///
/// Three deliberate choices:
///
/// * **The robot is the TAP's own `origin_robot`, not the attribution map.**
///   [`Ctx::attribution_snapshot`] answers a different question ("where did a topic
///   of this NAME last live") and pays a `/__cerulion/mirrors` gather to do it —
///   150-600 ms, on the poll thread, every 400 ms, which would break the monitors' cost
///   claim outright. The per-tap record is also the CORRECT one here: it is the
///   field whose whole purpose is "whose data is THIS tap carrying", which is
///   exactly the identity a standing per-row baseline must be keyed on, and it is
///   what keeps a desk producer reusing a detached remote topic's NAME from
///   inheriting the historical robot's learned state.
///
///   That is a claim about where the answer is READ, not about which answer it is.
///   Both attach seams now RECORD their attribution onto the tap — the remote path
///   from the robot it demanded from, `attach_local` from the very snapshot its own
///   reply is built from — so this row and the `list`/`status`/`attach` rows are
///   the same answer, which is the whole of the attribution rule, reached without the poll
///   thread ever paying for it. Before that, a netd mirror entering through the
///   LOCAL path (netd registered it before Studio checked it — the live shape) was
///   reported remote by its own reply and rendered LOCAL here.
/// * **`discovery_converged: true`, always.** The engine's second absolute rule
///   suppresses conditions while the DISCOVERY plane has not converged, because a
///   short catalog proves nothing about what exists. This plane reads no catalog:
///   the evidence is this daemon's own tap, first-hand. There is no discovery
///   answer to distrust, so passing `false` here would suppress every condition
///   forever on the one plane that cannot be wrong about it. (The discovery plane's
///   catalog-fed rows are where the marker is real.)
/// * **A tap with no stats entry is SKIPPED.** Every attach seam opens the entry
///   (`stats.entry(..).or_default().begin_observation(..)`), so this is not a
///   reachable production shape; and a row with neither an observation nor an
///   origin has nothing to key on, so minting one would invent a LOCAL row that a
///   later sample carrying a robot could not join.
/// * **The pass stamp is a [`WriteStamp`], not a `u64`.** The discovery plane is a second
///   writer on a controller thread, so the plane's write clock is LATCHED and a
///   builder must not be able to reach around it. Taking the latched type is what
///   makes that structural: `advance_to` is its only constructor, so there is no
///   unlatched instant in scope here to use by mistake.
fn monitor_snapshot(
    topics: Vec<String>,
    stats: &BTreeMap<String, TopicStat>,
    now: Instant,
    at: WriteStamp,
) -> Vec<MonitorSample> {
    topics
        .into_iter()
        .filter_map(|topic| {
            let stat = stats.get(&topic)?;
            // The per-tap rate rides `liveness.rate_estimate` (`TopicStat::liveness`),
            // so the sample reads it from there like every other surface does.
            let liveness = stat.liveness(now);
            Some(MonitorSample::new(
                topic,
                stat.origin_robot.clone(),
                at.as_ns(),
                liveness,
                true,
            ))
        })
        .collect()
}

/// Monitors: the `(robot, topic)` rows an ATTACHED tap owns.
///
/// Exactly the keys [`monitor_snapshot`] will mint with a robot — same tap list,
/// same per-tap `origin_robot` — because "attached" has to mean "the attached
/// sampler feeds this row", not "something like it exists". A tap with no
/// attribution (`origin_robot: None`) is a genuine LOCAL row and can never collide
/// with a catalog entry, which always names its robot.
fn attached_row_keys(
    topics: &[String],
    stats: &BTreeMap<String, TopicStat>,
) -> BTreeSet<(String, String)> {
    topics
        .iter()
        .filter_map(|topic| {
            let robot = stats.get(topic)?.origin_robot.clone()?;
            Some((robot, topic.clone()))
        })
        .collect()
}

/// Monitors: ONE [`MonitorSample`] per CATALOG entry — the required
/// property, a verdict on a topic NOBODY has checked.
///
/// The evidence is the robot's own [`TopicLiveness`], carried on
/// [`CatalogEntry::liveness`], including the rate estimate,
/// which is what makes a rate for an unattached topic possible at all. It survives
/// netd's typed re-encode (pinned in `cerulion_netd`'s
/// `a_liveness_carrying_catalog_survives_the_netd_re_encode`), so this needs no
/// protocol change on either daemon.
///
/// Three deliberate choices:
///
/// * **Rows an attached tap owns are SKIPPED.** The two sample sources are
///   split by attachment, and the split is not bookkeeping: an attached row is fed
///   every 400 ms from the desk's own tap over the desk's rate window, a catalog
///   row every ~2 s from the robot's observer over the ROBOT's window. Feeding one
///   row from both interleaves two cadences into one confirmation streak and one
///   baseline learning run, so the row would be judged on a mixture of two
///   measurements of different things. The attached plane wins because its evidence
///   is first-hand.
/// * **The RATE is the payload's own** — no `with_rate` override. There is no desk
///   tap to measure, which is the whole point; the robot's estimate is the only rate
///   that exists for this row, and its `is_floor` label is what the engine's exact-rate
///   gate reads.
/// * **`discovery_converged` is the GATHER's marker, not `true`.** The attached
///   plane passes `true` because its evidence is first-hand and there is no
///   discovery answer to distrust. Here there is: a short or empty catalog from an
///   unconverged plane proves nothing, and a row built from one
///   must raise no condition. The engine collapses that into
///   [`MonitorSample::is_evidential`] alongside an absent payload and an `Unknown`
///   classification, because all three mean the same thing — nothing was observed
///   that anyone may act on.
fn catalog_monitor_samples(
    catalogs: &[CatalogReply],
    attached: &BTreeSet<(String, String)>,
    discovery_converged: bool,
    at: WriteStamp,
) -> Vec<MonitorSample> {
    catalogs
        .iter()
        .flat_map(|catalog| {
            catalog.entries.iter().filter_map(move |entry| {
                let key = (catalog.robot.clone(), entry.topic.clone());
                if attached.contains(&key) {
                    return None;
                }
                Some(MonitorSample::new(
                    entry.topic.clone(),
                    Some(catalog.robot.clone()),
                    at.as_ns(),
                    entry.liveness,
                    discovery_converged,
                ))
            })
        })
        .collect()
}

/// Whether the DEFAULT consolidated layout ([`default_layout`]) is in effect
/// (re-applied on every attach/detach), or an EXPLICIT agent/user layout has taken
/// over (`set_blueprint`/`compose_layout` — that layout WINS, so attach/detach no
/// longer clobbers it). Reset to the built-in default via `set_blueprint` with no
/// spec, which returns to [`Self::Auto`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum LayoutMode {
    /// vizd owns the layout: on every attach/detach it re-derives + applies the
    /// consolidated [`default_layout`] from the currently-attached topics.
    #[default]
    Auto,
    /// An explicit `set_blueprint`/`compose_layout` layout is active — vizd does NOT
    /// auto-recompose on attach/detach (the explicit layout is the user's/agent's).
    Explicit,
}

/// The daemon's shared control state (poll thread + every controller thread).
#[derive(Default)]
struct DaemonState {
    /// The runtime taps.
    taps: TapManager,
    /// Per-attached-topic stats (keyed by absolute topic). A `detach` removes the
    /// entry, so it never outlives its tap.
    stats: BTreeMap<String, TopicStat>,
    /// Topics whose attach ASKED for a wake listener
    /// ([`WakeMode::Listener`]), whether or not one exists now.
    ///
    /// Held apart from the tap's own `has_wake` because the two answer different
    /// questions, and the DIFFERENCE is the operator-facing one: a topic in this
    /// set WITHOUT a wake is running on the poll cadence — the listener could not
    /// be opened, or it was demoted for firing without delivering — and "why is
    /// this topic slower than that one?" has no answer without it. Cleared by
    /// `detach` alongside the tap.
    ///
    /// **Every attach seam inserts here**,
    /// so on THIS daemon the set equals the tap set and a row is never absent
    /// from `status`. The set is kept rather than collapsed into `has_wake`
    /// precisely because that equality is a decision, not a law: if a future topic
    /// class is ever excluded from the wake, `Some(false)` must keep meaning
    /// "asked and lost" rather than silently starting to mean "never asked".
    wake_requested: std::collections::BTreeSet<String>,
    /// Topics this daemon currently DEMANDS from `cerulion-netd`, mapped
    /// to the ORIGIN ROBOT (so `detach` knows which `(robot, topic)` to release).
    /// The daemon demands each remote topic at most ONCE (idempotent per topic — a
    /// re-attach reuses the live demand), and `detach` RELEASES the demand and clears
    /// the entry (netd's refcount-0 teardown
    /// fires when the last consumer leaves). The demand is reserve-then-committed
    /// under this state lock so concurrent same-topic attaches demand exactly once.
    demanded: BTreeMap<String, String>,
    /// A session-scoped PROVENANCE TOMBSTONE (topic → origin robot) recorded
    /// on every successful remote attach and — unlike [`Self::demanded`] — NEVER
    /// cleared on `detach`. When a remote topic is unchecked in Studio, the shared
    /// mirror is torn down and its provenance dies WITH it, so a re-check sends a
    /// plain `attach` with NO `robot` (the row had been showing as a local mirror). The
    /// tombstone REMEMBERS that the topic is remote, so that no-`robot` re-attach
    /// re-demands from the right robot (the automagic "check a box → it appears" bar)
    /// instead of failing with the nonsensical local graph-YAML "does not exist". It
    /// survives the mirror's whole lifecycle; it is only overwritten by a later remote
    /// attach of the same topic (a different origin robot re-points it).
    remote_provenance: BTreeMap<String, String>,
    /// A short-TTL CACHE of the mirror-provenance snapshot (topic → origin
    /// robot) so `discover` does not pay a 150-600 ms `/__cerulion/mirrors` gather on
    /// EVERY ~2 s sidebar poll while a mirror is live (the cerulion_core registry fast
    /// path only short-circuits a registry-LESS desk — exactly NOT the fixed scenario).
    /// Refreshed on a cache MISS (`None`, or older than [`PROVENANCE_CACHE_TTL`]) and
    /// INVALIDATED on the mutation points vizd itself drives (`attach_remote` / `detach`,
    /// which demand/release the mirror), so the user's own check/uncheck reflects on the
    /// very next discover. A stale entry only mis-groups a row for at most ONE poll cycle
    /// before the TTL forces a re-gather — acceptable (the defect to avoid is a PERSISTENT
    /// mis-group, not a one-cycle transient).
    mirror_provenance: Option<(Instant, BTreeMap<String, String>)>,
    /// The operator's per-TOPIC representation choice, for topics they
    /// overrode. Absent means [`Representation::Auto`].
    ///
    /// Keyed by TOPIC and — like [`Self::remote_provenance`], and unlike
    /// [`Self::stats`] — NEVER cleared on `detach`. A representation is a
    /// preference ABOUT a topic, not state OF an attachment: unchecking a row and
    /// checking it again must not silently revert a choice the operator made, or
    /// the affordance is one the sidebar keeps forgetting. Session-scoped by
    /// design (nothing is written to disk — a preference is not data, and the
    /// instant-only decision is about neither).
    ///
    /// NOT bounded by the robot's topic count: the verb inserts whatever topic
    /// string a client sends, with no existence check — deliberately, since a
    /// choice may be set before the topic is attached — and nothing prunes the
    /// map. The real bound is client behaviour, which is acceptable for a
    /// session-scoped map of `(String, enum)` driven by human clicks, and is
    /// recorded here rather than claimed away. Same one layer down for
    /// `SinkState::representation` / `forced_dump`.
    representation: BTreeMap<String, Representation>,
    /// Monitors: the standing per-topic watchdog — the engine, its one
    /// monotonic origin, the bounded alert ring, and the flood latches.
    ///
    /// It lives HERE, in the state the poll thread already locks every pass, rather
    /// than behind a mutex of its own. The monitors' cost claim includes "no
    /// new lock, no new lock ordering", and this plane must be written by the poll
    /// thread's sampler and read by the `monitors` handler on a controller thread —
    /// so it borrows the one lock that already serialises those two rather than
    /// adding a second one to get the ordering wrong between.
    ///
    /// The cost of that choice, stated: the engine's arithmetic runs INSIDE this
    /// lock. It is `O(rows)` integer work plus one small allocation per row, ONCE
    /// PER [`MONITOR_SAMPLE_INTERVAL_NS`] (400 ms — one pass in 25 at the default
    /// 16 ms interval), against a lock the drain takes on every pass for the whole
    /// of its own work. Every OTHER pass costs one `Instant` compare, taken outside
    /// the lock entirely.
    ///
    /// [`MONITOR_SAMPLE_INTERVAL_NS`]: cerulion_viz::monitor::MONITOR_SAMPLE_INTERVAL_NS
    monitors: MonitorPlane,
}

impl DaemonState {
    /// Record (or CLEAR) the ROBOT whose data `topic`'s tap is carrying, RELEASING
    /// the monitor row its previous identity opened.
    ///
    /// The ONE writer of [`TopicStat::origin_robot`], and it exists because monitor
    /// rows are keyed `(topic, robot)`: a sample naming a different robot for the
    /// same topic name is a different DATA SOURCE, so the engine opens its OWN row
    /// rather than re-attributing one in place. That is the right rule, and it is
    /// exactly why a bare assignment is not allowed here — the next sampling pass
    /// would open the new row while the old one kept being served, with nothing able
    /// to release it (`detach` forgets the key the tap's CURRENT record names, so it
    /// takes the new row and leaves the old). Two rows for one tap, one of them
    /// frozen at whatever it last knew, ageing forever.
    ///
    /// The old row is DROPPED, never migrated. Its baseline is a median of rates
    /// measured on a different producer and its settle window was opened against a
    /// different observation, so carrying either onto a new identity would judge the
    /// new source by the old one's behaviour. A fresh row costs one settle window
    /// and is accurate. Alerts the old row minted stay in the ring — they record
    /// something that really happened.
    ///
    /// An UNCHANGED robot is a no-op on the ROW (only the `hinted` classification is
    /// updated), deliberately: `attach` is idempotent, and a re-attach — or a
    /// sampling pass re-confirming a hint 2.5 times a second — must not throw away a
    /// learned baseline.
    fn set_origin_robot(&mut self, topic: &str, robot: Option<String>, hinted: bool) {
        let Some(stat) = self.stats.get_mut(topic) else {
            return;
        };
        if stat.origin_robot == robot {
            stat.origin_robot_hinted = hinted && robot.is_some();
            return;
        }
        let previous = std::mem::replace(&mut stat.origin_robot, robot);
        stat.origin_robot_hinted = hinted && stat.origin_robot.is_some();
        self.monitors.forget(previous.as_deref(), topic);
    }

    /// AUTHORITATIVE: this daemon DEMANDED `topic` from `robot`, so the record is
    /// first-hand and nothing derived from the name-keyed map may revise it.
    fn retarget_origin_robot(&mut self, topic: &str, robot: String) {
        self.set_origin_robot(topic, Some(robot), false);
    }

    /// Re-point every HINTED tap record at the live `/__cerulion/mirrors` map, and
    /// CLEAR the ones the map no longer names.
    ///
    /// # Why a hint is refreshed instead of recorded
    ///
    /// The provenance map is keyed by TOPIC NAME, so it describes a name and not a
    /// tap's GENERATION. Writing one into
    /// the tap at attach time turns
    /// a 3-second staleness into a permanent, affirmatively-WRONG claim, and the
    /// window is ordinary: retire a mirror, let a desk producer take the freed name,
    /// and compose or attach it before [`PROVENANCE_CACHE_TTL`] expires. MEASURED on
    /// that shape — the row read `robot: "ubuntu"` for a topic this desk was
    /// publishing itself, and it never healed, because once the record exists the
    /// HELD half of `attribution_snapshot` re-asserts it to `list`/`status`/`discover`
    /// forever. Worse than `None`, and on every surface at once.
    ///
    /// There is no cheap way to CORROBORATE the map at stamp time: the registry's
    /// entire read surface is `gather_mirror_provenance{,_checked}`, both of which
    /// LISTEN for a window, and neither the sampler (400 ms cadence, poll thread) nor
    /// the compose handler (already spending `COMPOSE_PEEK_BUDGET`) may pay one. So
    /// the hint is not made authoritative — it is made FALSIFIABLE. Every sampling
    /// pass re-reads the cache and tracks it in both directions, which makes the
    /// record a per-tap CACHE of the live answer rather than a memory of a past one.
    ///
    /// # The bound, stated exactly
    ///
    /// A wrong hint survives until the cached snapshot is REPLACED, which needs a
    /// verb that gathers (`discover` / `list` / `status` / `attach_local`) to run past
    /// the TTL — then at most one sampling interval more. On a live Studio, which
    /// polls the sidebar every ~2 s, that is a few seconds. With NOTHING polling those
    /// verbs it does not converge at all; the accurate reading is that the attribution
    /// is only as fresh as the last question anyone asked, which is the same contract
    /// `list` and `status` already serve under.
    ///
    /// A STALE snapshot is HELD, never treated as an absence: expiry means "nobody has
    /// asked recently", not "the mirror is gone", and clearing on it would flap every
    /// tap's row between polls. AUTHORITATIVE records are skipped entirely — a torn-
    /// down mirror under a still-held remote tap keeps its robot.
    fn refresh_hinted_origin_robots(&mut self) {
        // MOVED OUT rather than cloned: the loop below mutates `self.stats` and
        // `self.monitors`, so it cannot hold a borrow of `self.mirror_provenance`.
        // Put back before returning, so this is a read of the cache and nothing more.
        let Some((at, mirrors)) = self.mirror_provenance.take() else {
            return;
        };
        if at.elapsed() < PROVENANCE_CACHE_TTL {
            for topic in self.taps.list() {
                let Some(stat) = self.stats.get(&topic) else {
                    continue;
                };
                if let Some(target) = hint_refresh_target(
                    stat.origin_robot.as_deref(),
                    stat.origin_robot_hinted,
                    mirrors.get(&topic).map(String::as_str),
                ) {
                    self.set_origin_robot(&topic, target, true);
                }
            }
        }
        self.mirror_provenance = Some((at, mirrors));
    }
}

/// What a refresh should do to ONE tap's record: `Some(target)` = write it (and
/// `target` may itself be `None` — a WITHDRAWAL), `None` = leave it alone.
///
/// Pure, so the policy is oracle-testable without a transport, and split out
/// because it is the whole of the tombstone rule in three lines: a record
/// backed by first-hand knowledge is never revised from a name-keyed map, and one
/// that is not tracks that map EXACTLY — including when the map stops naming the
/// topic, which is the case a write-only stamp can never express.
fn hint_refresh_target(
    origin_robot: Option<&str>,
    hinted: bool,
    live: Option<&str>,
) -> Option<Option<String>> {
    if origin_robot.is_some() && !hinted {
        return None; // AUTHORITATIVE — this daemon demanded it.
    }
    Some(live.map(str::to_string))
}

/// How long a cached mirror-provenance snapshot stays fresh before
/// `discover` re-gathers it. 3 s comfortably spans the ~2 s sidebar poll (so alternate
/// polls are O(1)) while bounding staleness to one poll cycle: a topic that just became
/// a mirror (or a torn-down one) is correctly grouped within one refresh. External
/// mirror mutations (another desk consumer, `topic echo`) converge within the TTL; the
/// user's OWN attach/detach are reflected immediately via cache invalidation.
///
/// `pub` because the e2e suite must
/// DERIVE its attribution-convergence bound from this number rather than
/// re-spell it. A gather that misses the record is cached exactly like a real
/// snapshot (see `Ctx::gather_mirror_provenance` — deliberate, so a transient
/// registry error cannot re-storm the gather), so a mirror can legitimately read
/// as a LOCAL topic for up to one whole TTL. Two tests hard-coded budgets that
/// did not account for that — one polled for 4 s against this 3 s, the other did
/// not poll at all — and a contended macOS runner outran both. A test bound
/// written as a multiple of this constant cannot silently fall under it again if
/// the TTL is ever raised.
pub const PROVENANCE_CACHE_TTL: Duration = Duration::from_secs(3);

/// The daemon's growable schema universe (remote arm). A schema-less (or
/// pinned-but-unknown-custom-type) remote attach fetches the type's `.msg` closure
/// from the robot over the daemon's zenoh session and folds it in here, rebuilding
/// the [`FrameWalker`] so the tapped frames decode + render. Guarded by its OWN
/// mutex (NOT the daemon's `state` lock). The poll thread takes this lock BRIEFLY
/// each iteration (to take a pending swap), and [`Ctx::seed_walker_with_docs`]
/// holds it only for the fast doc-append/install — the heavy rebuild runs OFF this
/// lock, so a seed never blocks the poll thread's tap draining.
#[derive(Default)]
struct SeedState {
    /// Every robot-served schema-closure doc fetched so far (accumulated across
    /// remote attaches). The walker is rebuilt from builtins ∪ store ∪ THESE, so
    /// growing it is monotonic and a re-attach never loses an earlier type.
    docs: Vec<SchemaDoc>,
    /// The OFFERED docs the WALKER refused, kept VERBATIM so an identical
    /// re-offer costs nothing.
    ///
    /// **Only ever a doc that was REFUSED — never an incumbent that survived one.**
    /// The distinction is the whole value of this field: a re-offer matching an
    /// entry here is reported `rejected`, so an incumbent landing in it would make
    /// the daemon permanently disown a definition it is successfully resolving.
    /// [`rollback`](Self::rollback) is what keeps that true, and the reason it
    /// takes a SET (see its docs).
    ///
    /// [`rollback`](Self::rollback) exists so a refused doc does not sit in `docs`
    /// swallowing a later corrected offer of the same type — but on its own it
    /// re-opens the cost [`absorb`](Self::absorb)'s idempotence short-circuit was
    /// written to prevent: `bag play` re-offers the SAME set every 2 s, so a doc
    /// the walker cannot use looped absorb(added=1) → full rebuild (a re-parse of
    /// the whole ~254-message built-in corpus) + a `pending_worker_swap` → refuse
    /// → roll back, for the entire playback.
    ///
    /// Remembering the refused TEXT closes that without weakening recovery: the
    /// short-circuit fires only for a byte-identical re-offer, which by
    /// construction would be refused again, while a CORRECTED definition differs
    /// and is absorbed normally. The doc is MOVED here out of `docs` rather than
    /// cloned, so the memory costs one allocation the daemon already held.
    refused: Vec<SchemaDoc>,
    /// A freshly-rebuilt walker the poll thread must hand to the render worker
    /// (via `swap_walker`, on the worker's own thread — the FIFO-with-data seam).
    /// `take`n by the poll thread each iteration; `None` = nothing to swap.
    pending_worker_swap: Option<FrameWalker>,
}

/// What one [`SeedState::absorb`] changed.
#[derive(Debug, Default, PartialEq, Eq)]
struct Absorbed {
    /// Names the daemon had never held.
    added: usize,
    /// Names it HELD under a different definition, now replaced. Reported apart
    /// from `added` because a replacement is destructive: the walker keys layouts
    /// by qualified name, so the previous definition — and therefore its wire
    /// hash — stops resolving the instant this lands. Whatever was decoding the
    /// old version goes dark, and an operator is owed that fact.
    replaced: Vec<String>,
    /// The definition each REPLACED name held BEFORE this absorb, so
    /// [`SeedState::rollback`] can put it back when the rebuilt walker refuses the
    /// replacement. Without it a merge is irreversible and a bad re-offer takes a
    /// working type dark daemon-wide.
    previous: Vec<SchemaDoc>,
    /// Names this offer did NOT install: either the rebuilt walker refused them
    /// (recorded by [`SeedState::rollback`]) or they are byte-identical to a doc
    /// already refused, so `absorb` short-circuited them.
    ///
    /// AUTHORITATIVE for reporting. A caller cannot recover this by asking the
    /// walker `knows(name)`: a refusal whose INCUMBENT survived still answers
    /// `true`, which is exactly the outcome this field exists to describe.
    refused: Vec<String>,
}

impl Absorbed {
    /// Whether anything changed at all (`false` = the caller can skip the
    /// rebuild entirely).
    ///
    /// Deliberately independent of [`refused`](Self::refused): a doc short-
    /// circuited by the refusal memory is REPORTED but installs nothing, so it
    /// must not buy a rebuild.
    fn changed(&self) -> bool {
        self.added > 0 || !self.replaced.is_empty()
    }
}

impl SeedState {
    /// Fold `incoming` into [`docs`](Self::docs), keeping only what is genuinely
    /// NEW, and report what changed.
    ///
    /// "New" is by QUALIFIED NAME, with a byte-different text for a name already
    /// held counting as new and REPLACING it. Replacing rather than ignoring is
    /// deliberate: an identical re-offer is the idempotent case and must cost
    /// nothing, but a DIFFERENT definition for the same name is a real update
    /// (a robot redeployed with a changed message, a bag recorded against
    /// another version), and silently keeping the stale one would decode frames
    /// against a definition their producer no longer uses.
    ///
    /// Pure — no locks, no rebuild — so the policy is oracle-testable on its own.
    /// A doc BYTE-IDENTICAL to one the walker already refused changes nothing:
    /// it would be rebuilt, refused and rolled back again, so it is skipped before
    /// it can make `changed()` true — but it IS reported in
    /// [`Absorbed::refused`], because a caller told "nothing changed" about a doc
    /// that is still unusable learns the wrong thing.
    ///
    /// A merge is REVERSIBLE: every replacement records the incumbent in
    /// [`Absorbed::previous`] so [`rollback`](Self::rollback) can restore it when
    /// the rebuilt walker refuses the replacement.
    fn absorb(&mut self, incoming: Vec<SchemaDoc>) -> Absorbed {
        let mut out = Absorbed::default();
        for doc in incoming {
            // A CORRECTED definition for a refused name differs from the
            // remembered one, so it falls through to the normal path below and
            // the memory of the bad one is dropped — recovery is unaffected.
            if let Some(pos) = self
                .refused
                .iter()
                .position(|r| r.qualified == doc.qualified)
            {
                if self.refused[pos] == doc {
                    // Report each name ONCE even if the offer carries it twice —
                    // `refused` is a report, and a duplicated entry would make an
                    // operator read one bad doc as two.
                    if !out.refused.contains(&doc.qualified) {
                        out.refused.push(doc.qualified);
                    }
                    continue;
                }
                self.refused.swap_remove(pos);
            }
            match self.docs.iter_mut().find(|d| d.qualified == doc.qualified) {
                Some(held) if *held == doc => {}
                Some(held) => {
                    // ONCE per name, even when the offer carries it twice: an
                    // offer that replaces one type is ONE replacement, and the
                    // field's own doc says a replacement is a distinct outcome
                    // from a gain — listing it twice makes one type read as two.
                    let first_time = !out.replaced.contains(&doc.qualified);
                    if first_time {
                        out.replaced.push(doc.qualified.clone());
                    }
                    // The incumbent, BEFORE it is overwritten — the whole of what
                    // makes this merge undoable. Only the FIRST is kept: on a
                    // second replacement of the same name within one batch the
                    // displaced value is this batch's own earlier doc, and rolling
                    // back to THAT would restore something the daemon never
                    // served. `rollback` must land on the incumbent the offer
                    // found.
                    let displaced = std::mem::replace(held, doc);
                    if first_time {
                        out.previous.push(displaced);
                    }
                }
                None => {
                    self.docs.push(doc);
                    out.added += 1;
                }
            }
        }
        out
    }

    /// Undo the part of an [`absorb`](Self::absorb) the rebuilt walker REFUSED
    /// (validate-then-commit), leaving the daemon exactly as it was for those names.
    ///
    /// For each refused name: the offered doc leaves `docs` and is remembered
    /// VERBATIM in [`refused`](Self::refused) (so an identical re-offer costs no
    /// rebuild), and the incumbent this offer displaced — if there was one — is
    /// restored to the SLOT it held, so nothing else about the set moves.
    ///
    /// This is what makes a bad offer NON-DESTRUCTIVE. `absorb` commits the
    /// replacement before the walker has had a chance to parse it, and the walker
    /// warn-SKIPS an unparseable doc — so without the restore, one malformed
    /// re-offer of a held name left the type undecodable for EVERY consumer of
    /// this daemon, including live robot taps unrelated to the caller.
    ///
    /// A refused name with no incumbent simply leaves the set (the old `forget`
    /// behaviour, which this subsumes — it was the no-incumbent case of exactly
    /// this operation).
    ///
    /// `refused` is a SET, and that is load-bearing rather than tidy.
    /// This operation is NOT idempotent per name: a repeat visit to the
    /// same name finds the RESTORED INCUMBENT where the bad doc was, evicts the
    /// real refusal from the memory and records the GOOD doc as the refused one —
    /// after which every later byte-identical offer of that good definition is
    /// reported `rejected` with a "stays UNDECODABLE" warning, about a type the
    /// walker resolves perfectly. The data plane survives (the incumbent is
    /// re-inserted) but the REPORT inverts permanently, which is a
    /// success-shaped lie pointed the other way. A `Vec` makes that
    /// reachable from any offer carrying one qualified name twice — the `schemas`
    /// verb takes docs straight off the control seam, and `classify_schema_replies`
    /// does not dedup a robot's served closure. A set makes it unrepresentable.
    fn rollback(&mut self, refused: &BTreeSet<String>, previous: &[SchemaDoc]) {
        for name in refused {
            let Some(pos) = self.docs.iter().position(|d| &d.qualified == name) else {
                continue;
            };
            let bad = self.docs.remove(pos);
            // Keep ONE memory per name — the most recent refusal is the one a
            // re-offer will match.
            self.refused.retain(|r| &r.qualified != name);
            self.refused.push(bad);
            if let Some(prev) = previous.iter().find(|d| &d.qualified == name) {
                self.docs.insert(pos, prev.clone());
            }
        }
    }
}

/// The shared, cloneable daemon context (all `Arc`s) passed to the poll thread +
/// every controller thread. Its methods implement the request handlers.
#[derive(Clone)]
struct Ctx {
    state: Arc<Mutex<DaemonState>>,
    manager: Arc<TransportManager>,
    /// The netd DEMAND plane — `attach_remote` demands a remote topic's
    /// shared mirror from the one-per-computer `cerulion-netd` daemon (instead of
    /// opening vizd's own ingress on the zenoh session), and `detach` releases it.
    /// A DI seam ([`DemandPlane`]): production is [`NetdDemandPlane`] (one persistent
    /// UDS connection for the daemon's lifetime); tests inject a counting spy.
    demand_plane: Arc<dyn DemandPlane>,
    /// The schema-resolution walker, SWAPPABLE (remote arm): a schema-less
    /// remote attach grows it with a robot-served type's closure. Reads take a
    /// cheap read lock + clone the inner `Arc` (`walker_snapshot`); a seed takes
    /// the write lock to install a rebuilt walker. Distinct from the render
    /// worker's own walker (grown in lockstep via `SeedState::pending_worker_swap`).
    walker: Arc<RwLock<Arc<FrameWalker>>>,
    /// The growable schema universe + the poll-thread→worker walker-swap handoff
    /// (remote arm). The poll thread takes this lock BRIEFLY every
    /// iteration (to take a pending swap), so it is held only for the fast
    /// append/clone/install — never across the heavy walker rebuild.
    seed: Arc<Mutex<SeedState>>,
    /// Serializes concurrent seeds so two remote attaches can't
    /// interleave a doc-append + rebuild and install an older walker over a newer
    /// one (losing a type). The POLL THREAD never takes this, so the heavy rebuild
    /// it guards never blocks the poll thread.
    rebuild_lock: Arc<Mutex<()>>,
    counters: Arc<VizWorkerCounters>,
    /// The control handle to the never-block worker (the ONE thread that owns the
    /// `RecordingStream`), shared across every controller thread — used by the
    /// `set_blueprint` verb to push a runtime layout to the worker. `Arc` so the
    /// daemon can `close()` the shared sender at shutdown (the layout verb).
    worker_control: Arc<VizControl>,
    /// TOCTOU fix: serializes "decide layout mode + enqueue the blueprint"
    /// so a concurrent attach cannot enqueue the DEFAULT after an explicit apply's plan
    /// but before its mode flip (leaving the viewer on the default with mode Explicit —
    /// stuck). The mode read/write AND the worker send happen under THIS lock in
    /// [`Ctx::maybe_apply_default_layout`] and [`Ctx::apply_layout_atomic`], so the FIFO
    /// send order always matches the recorded mode. Held only briefly (a bounded worker
    /// send); never acquired while holding [`Ctx::state`] (no lock inversion —
    /// `attached_render_infos` re-locks `state` on its own, inside this lock).
    layout_lock: Arc<Mutex<LayoutMode>>,
    /// Every render ENTITY this daemon has attached SINCE BOOT,
    /// never pruned. `default_layout_excluding` excludes a topic's attached descendants from
    /// its view, but nothing in vizd clears a DETACHED topic's data from the rerun
    /// store — so recomposing from the attached set alone re-admitted a detached
    /// descendant's last frame into its ancestor topic's view (and into the Scene).
    /// Accumulated + read at the single choke point every attach/detach already
    /// funnels through, [`Ctx::consolidated_default_plan`].
    logged_entities: Arc<Mutex<std::collections::BTreeSet<String>>>,
    /// The Rerun endpoint the daemon logs to + every viewer connects to,
    /// advertised in the [`Hello`] banner. `None` = viz
    /// disabled. Cheap `Option<String>` clone per connection (banner-only).
    rerun_url: Option<String>,
    /// The catalog-change EVENT hub — the per-controller push slots the
    /// forwarder thread fills from `cerulion-netd`'s subscription, so a Studio sidebar
    /// updates itself instead of waiting for a manual refresh.
    events: Arc<crate::events::EventHub>,
    /// Principle #3 observable: how many times ANY controller handler has gone round
    /// its read loop since this daemon started. On an idle connection the loop is
    /// paced by `CONN_READ_TIMEOUT`, so this grows at ~5 Hz per connection; a
    /// regression that leaves the accepted socket nonblocking turns the same loop
    /// into a spin and this counter into millions per second. Read via
    /// [`RunningDaemon::conn_read_loop_iterations`].
    conn_loop_iterations: Arc<AtomicU64>,
    /// Principle #3 observable: the frame-drain loop's LAST measured
    /// inter-iteration period, in nanoseconds. `0` until the loop has completed a
    /// second iteration (one sample needs two marks).
    ///
    /// It exists because the loop's REAL period is otherwise unobservable
    /// from outside the process, and a measurement of the desk chain
    /// (11.8 ms p50 / 19.4 ms p90 of SHM residency against 78 µs of work) can
    /// only be explained by INFERENCE. Two order statistics of a uniform
    /// residency give two independent estimates of the period (`p50 = 0.5T`,
    /// `p90 = 0.9T` ⇒ `T ≈ 23.6` / `21.6` ms), which is *consistent* with a
    /// fixed sleep taken AFTER the work but does not distinguish it from
    /// contention on [`Ctx::state`] with the control plane. This measures the
    /// quantity directly instead. Read via
    /// [`RunningDaemon::poll_loop_period_ns`].
    poll_period_ns: Arc<AtomicU64>,
    /// Principle #3 observable: total frame-drain loop iterations since
    /// this daemon started — the COUNTED twin of [`Ctx::poll_period_ns`], and the
    /// direct analogue of [`Ctx::conn_loop_iterations`] for the sibling loop.
    ///
    /// A last-value register needs a sampler and a median to see a spin, and the
    /// sampler under-counts precisely when the loop outruns its own poll rate —
    /// i.e. the instrument degrades where the defect lives. A counter is a rate
    /// outright. An unspent-budget spin bound has to be stated on this
    /// observable (only a counted bound sees it), so the
    /// counted form is wired here.
    poll_iterations: Arc<AtomicU64>,
    /// Principle #3 observable: passes on which the loop took
    /// NO wait at all. Structurally 0 while [`plan_tick_sleep`] floors its
    /// already-late arm — which is exactly what makes it an oracle: removing the
    /// floor turns it into a per-pass count under sustained overrun.
    poll_sleepless_passes: Arc<AtomicU64>,
    /// Principle #3 observable: passes that reached the already-late arm
    /// — the STIMULUS observable.
    ///
    /// It exists because the overrun e2e could not see the loss of its own
    /// stimulus: `poll_sleepless_passes == 0` holds on ANY loop while the floor
    /// is in place, and an iteration FLOOR is satisfied more easily by a healthy
    /// loop than by an overrunning one, so an inert injector went green under a
    /// "sustained overrun" label. This counter is strictly one-sided —
    /// contention can only ADD late passes — so a floor on it proves the regime
    /// was entered without a bound load can invert.
    poll_late_passes: Arc<AtomicU64>,
    /// TEST-ONLY per-pass overrun injection, in
    /// nanoseconds (`0` = off, the production value — nothing in the daemon ever
    /// writes it). See [`RunningDaemon::inject_poll_work_for_test`].
    poll_injected_work_ns: Arc<AtomicU64>,
    /// TEST-ONLY delay injected into the `runs` verb's LOCAL arm, in
    /// nanoseconds (`0` = off, the production value — nothing in the daemon ever
    /// writes it). See [`RunningDaemon::inject_runs_local_delay_for_test`].
    runs_local_delay_ns: Arc<AtomicU64>,
    /// Principle #3 observable: passes that spent their wait BLOCKED ON A
    /// WAKE SET rather than on `thread::sleep`.
    ///
    /// This is the no-inert-shipping observable. Every behavioural assertion
    /// about the wake path is otherwise satisfiable by the timer path running
    /// slightly early, so without a counter that says WHICH wait a pass took,
    /// reverting `WakeMode::Listener` to `Timer` at the attach seam would leave a
    /// latency test passing on a fast runner and the whole wake path shipped inert.
    poll_wake_waits: Arc<AtomicU64>,
    /// Principle #3 observable: passes whose wake set reported at least
    /// one FIRED source — i.e. the wait ended because a producer notified, not
    /// because the timer expired.
    ///
    /// Strictly one-sided in the load-safe direction: contention can only make a
    /// wake arrive LATER (and, past the timeout, not be counted at all), so a
    /// FLOOR on this is a claim a loaded runner cannot manufacture.
    poll_wake_fires: Arc<AtomicU64>,
    /// Principle #3 observable: default-layout reflows the drain loop RE-DERIVED
    /// because a topic's live LAYOUT SIGNAL changed — the dump companion
    /// appearing or going, on evidence only the worker can produce.
    ///
    /// It is the observable that makes the feature falsifiable from outside the
    /// process. The layout decision itself is pure and pinned in
    /// `dump_companion_test.rs`; the reflow is the wiring, and without it
    /// NOTHING watches either live signal (a rendition set going non-empty reaches
    /// the layout only if some UNRELATED attach/detach/late-resolve happens to
    /// reflow afterwards), so a change that never re-applies is exactly the
    /// inert-shipping shape a green pure suite cannot see.
    ///
    /// **RE-DERIVED, not called**: gated on
    /// [`Ctx::maybe_apply_default_layout`]'s return, so a pass under an operator's
    /// EXPLICIT layout — where that function returns at its first line and no plan
    /// is built — does not climb it.
    ///
    /// BOUNDED, not a per-frame counter, and the bound is enforced at the WRITE
    /// site rather than asserted here: the worker bumps its layout-signal
    /// generation only when a mirror value really changed, the proof flags are
    /// sticky and rendition sets only grow, so a topic contributes at most its
    /// first sighting plus its own transitions — the same at-most-once-per-topic
    /// class as the late-resolve reflow beside it. A steady stream of identical
    /// frames contributes NOTHING, which is pinned as a CEILING by
    /// `vizd_e2e_test::a_steady_stream_of_identical_frames_stops_reflowing`.
    ///
    /// **Reporting lag, stated at its real bound**: every report is served off the
    /// worker's mirrors by whichever control call asks, while the APPLIED blueprint
    /// is re-derived on this loop's next pass — so for ≈ ONE poll interval a report
    /// is ahead of what is installed, and for TWO when the wake
    /// pacing engages, since a paced pass spends a second sleep before it reaches
    /// the watcher.
    poll_layout_signal_reflows: Arc<AtomicU64>,
    /// Principle #3 observable: total FRAMES the drain loop has handed on
    /// since this daemon started — the daemon-wide counted twin of the per-topic
    /// `status.frames`.
    ///
    /// It is an ATOMIC rather than a control-plane read because the question it
    /// answers is a LATENCY one: "when did the loop see this frame?" A `status`
    /// round trip takes the daemon's state lock and costs milliseconds, so an
    /// observer using it would be measuring its own instrument against a
    /// millisecond-scale quantity. A relaxed load contends with nothing.
    poll_frames_drained: Arc<AtomicU64>,
    /// Principle #3 observable: passes that PACED themselves after a
    /// wake — the wake had been firing without delivering, so the loop spent the
    /// budget the wake was returning instantly from.
    ///
    /// A nonzero value on a healthy desk is a report about a PRODUCER (something
    /// is ringing an event service it is not publishing to), not about this loop.
    poll_wake_paced_passes: Arc<AtomicU64>,
    /// Principle #3 observable: topics whose wake was DROPPED because it
    /// kept firing without delivering, reverting them to the poll cadence.
    ///
    /// Never reset: it is a running total of producers this daemon stopped
    /// billing, and an operator asking "why did that topic get slower?" needs it
    /// to be true for the whole run, not since the last quiet moment.
    poll_wake_demotions: Arc<AtomicU64>,
    /// Principle #3 observable: passes that skipped the wait entirely
    /// because the drain yielded frames (the backlog-aware arm).
    ///
    /// Counted rather than left implicit because a zero-wait pass is exactly the
    /// shape a spin takes. Its bound is the pairing with the NEXT pass: a pass
    /// that drained frames is followed by one that drains none and DOES wait, so
    /// the skip costs at most one extra pass per burst — a claim the e2e's
    /// iteration ceiling checks rather than assumes.
    poll_backlog_passes: Arc<AtomicU64>,
    /// Monitors (Principle #3): samples the drain loop's sampler has HANDED to
    /// the monitor engine since this daemon started.
    ///
    /// The no-inert-shipping observable for the drain-loop sampler. Every behavioural claim about
    /// a monitor is otherwise a claim about the ENGINE, which its own pure suite
    /// proves and which stays true whether or not anything calls it — so without a
    /// counter that says the SAMPLER ran, deleting its call site from `poll_loop`
    /// leaves the whole feature shipped dead with a green pure suite.
    ///
    /// It counts samples handed over, INCLUDING ones the engine discarded (a row's
    /// settle window, a blind sample). That is deliberate: the question is "is the
    /// sampler running?", and a counter that only moved on admitted evidence would
    /// read zero for the first [`cerulion_viz::monitor::MONITOR_SETTLE_NS`] of
    /// every row — i.e. it would be silent for exactly the window in which someone
    /// would first look. The evidence count is per row, on
    /// [`cerulion_viz::monitor::MonitorRow::samples`].
    monitor_samples_taken: Arc<AtomicU64>,
    /// Monitors (Principle #3): conditions RAISED since this daemon started.
    ///
    /// Bumped where the raise HAPPENED (inside the sampling pass that observed it),
    /// not where a raise was expected — the same rule as
    /// [`Ctx::poll_layout_signal_reflows`]: a bump at the detection site leaves a counter
    /// climbing on a daemon that never does the work.
    monitor_alerts_raised: Arc<AtomicU64>,
    /// Monitors (Principle #3): conditions CLEARED since this daemon started.
    ///
    /// Separate from the raise count rather than a net gauge: a row that raised and
    /// cleared ten times is a FLAPPER, and a net figure of zero describes it
    /// identically to a desk on which nothing ever happened.
    monitor_alerts_cleared: Arc<AtomicU64>,
    /// Monitors (Principle #3): how many watched rows are currently withholding
    /// at least one condition — a GAUGE, overwritten by each sampling pass.
    ///
    /// Not a running total, and the difference is the point: "12 rows are being
    /// judged on nothing right now" is actionable, while a sum over passes would
    /// climb forever on a healthy desk carrying one permanently-ineligible `/tf`.
    /// It is the aggregate twin of the per-row `ineligible` list, so an operator can
    /// see the size of what the agent is NOT being told without polling the verb.
    monitor_rows_ineligible: Arc<AtomicU64>,
    /// Monitors (Principle #3): samples the DISCOVERY plane has handed
    /// to the engine — catalog rows, i.e. topics nobody attached.
    ///
    /// The discovery plane's no-inert-shipping observable, and it is deliberately NOT
    /// [`Self::monitor_samples_taken`]: that one is the drain-loop sampler's, and
    /// sharing it would let a daemon that deleted either call site keep the other's
    /// number climbing. Two planes, two liveness signals.
    monitor_catalog_samples_taken: Arc<AtomicU64>,
    /// Monitors: the ticket every catalog gather takes BEFORE it runs,
    /// so a slow gather that hands over LAST cannot apply an OLDER answer — see
    /// [`Ctx::begin_catalog_gather`].
    catalog_gather_seq: Arc<AtomicU64>,
    /// Monitors (Principle #3): discovery-plane rows RETIRED because a
    /// SETTLED gather stopped naming them.
    ///
    /// A running total, not a gauge — it records events, and an operator watching a
    /// churning LAN wants the rate of churn rather than a snapshot of it. Counted
    /// at all because a retirement is otherwise observable only by ABSENCE, and "the
    /// row was retired" and "it was never watched" render identically on the verb.
    monitor_catalog_rows_retired: Arc<AtomicU64>,
}

impl Ctx {
    /// Monitors: run ONE sampling pass — snapshot every attached tap,
    /// feed the engine, bank the transitions, and fold the pass's deltas into the
    /// daemon's counters.
    ///
    /// Called from `poll_loop` once per
    /// [`MONITOR_SAMPLE_INTERVAL_NS`](cerulion_viz::monitor::MONITOR_SAMPLE_INTERVAL_NS),
    /// never from a control handler: a handler-driven sampler would make the
    /// watchdog's cadence a function of who is polling it, and a row's confirmation
    /// span is measured in samples.
    ///
    /// ONE state-lock acquisition covers the snapshot AND the engine pass. Two
    /// acquisitions would leave a window in which a concurrent `detach` released a
    /// row between them, and the engine would then RESURRECT it from the already-
    /// captured sample — a row nothing can ever update again, reporting UNKNOWN
    /// with a forever-growing age.
    ///
    /// The DELTA counters are bumped OUTSIDE the lock, from the outcome the pass
    /// returned, so a relaxed `fetch_add` never runs inside it. **The GAUGE is
    /// published INSIDE it**, and the split is not stylistic: a `fetch_add` is
    /// COMMUTATIVE, so two passes may land in either order and the total is the
    /// same, while a `store` is a LATEST-WINS publication and "latest" has to be
    /// decided by the same lock that decided the value. Computing under the lock
    /// and storing after it lets an older pass's `store` land behind a newer
    /// pass's and leave the gauge describing a row set that is no longer current
    /// (MEASURED: two passes computed 1 and 2, and the 2 landed last).
    fn sample_monitors(&self, now: Instant) {
        let outcome = {
            let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            // The LATCHED write clock, not the read one: the discovery plane puts a second
            // writer on a controller thread, so `now` — captured before this lock —
            // can already be behind a stamp that thread banked while this pass was
            // waiting for the lock. See `WriteStamp`.
            let at = st.monitors.advance_to(now);
            // Track the live attribution BEFORE snapshotting, so a row is keyed on
            // the freshest answer this daemon holds — and so a HINT that has gone
            // wrong is corrected here rather than becoming permanent. Reads the
            // CACHE (a map lookup per tap, under the lock already held); it never
            // gathers, which is what keeps this pass free of the 150-600 ms
            // `/__cerulion/mirrors` listen. See `refresh_hinted_origin_robots` for
            // why a name-keyed claim may not be recorded once and left alone.
            st.refresh_hinted_origin_robots();
            let samples = monitor_snapshot(st.taps.list(), &st.stats, now, at);
            let outcome = st.monitors.observe_pass(&samples, at);
            // The GAUGE, published while the lock that produced it is still held —
            // see this function's doc for why a latest-wins store cannot be
            // ordered outside it.
            self.monitor_rows_ineligible
                .store(outcome.rows_ineligible, Ordering::Relaxed);
            outcome
        };
        self.monitor_samples_taken
            .fetch_add(outcome.observed, Ordering::Relaxed);
        self.monitor_alerts_raised
            .fetch_add(outcome.raised, Ordering::Relaxed);
        self.monitor_alerts_cleared
            .fetch_add(outcome.cleared, Ordering::Relaxed);
    }

    /// Monitors: start a catalog gather, and take the GENERATION that
    /// orders its feed against every other gather in flight.
    ///
    /// Taken BEFORE the gather, so it orders by when a gather STARTED rather than
    /// by when it finished — which is the whole point. Two `discover` handlers can
    /// be in flight at once (a Studio sidebar and a `cerulion viz`, or one
    /// controller refreshing while another attaches), netd gather latency is
    /// wildly variable (the convergence wait states the ceiling in SECONDS), and the slow one
    /// hands over LAST while carrying the OLDER answer. Ordering by hand-over
    /// instant — which is all [`WriteStamp`] can see, since it is captured after
    /// the gather returns — cannot distinguish the two.
    ///
    /// `Relaxed` is sufficient: this only has to be UNIQUE and monotone on the
    /// atomic itself, and the comparison that uses it happens under the state
    /// lock, whose acquire/release supplies the ordering with everything else.
    fn begin_catalog_gather(&self) -> u64 {
        self.catalog_gather_seq.fetch_add(1, Ordering::Relaxed)
    }

    /// Monitors: feed ONE `discover` gather's catalog rows to the
    /// engine — the required property, a verdict on a topic NOBODY checked.
    ///
    /// # Where this runs, and why that is the whole of the discovery plane's risk
    ///
    /// This is the SECOND writer, and it runs on a CONTROLLER thread — the plane's
    /// own doc names exactly that as the reason it refuses a leaf mutex for the plane
    /// (the `discover` handler must reach the same engine from a controller
    /// thread anyway). Three things follow, and each is closed by a seam rather
    /// than by care:
    ///
    /// 1. **The gather must already be DONE.** This takes the catalogs by
    ///    reference; it holds no `demand_plane`, opens nothing, and asks netd
    ///    nothing, so the state lock CANNOT be held across a round trip. That
    ///    matters more here than anywhere else in `discover`: the poll thread takes
    ///    the same lock on every 16 ms pass, so a lock held across a 150–600 ms
    ///    gather would stall the whole daemon's drain, not merely the monitor.
    ///    Pinned by `monitors_v1_the_catalog_feed_pays_no_gather_and_opens_no_connection`.
    /// 2. **The write clock is LATCHED.** `now` is captured before the lock and
    ///    this thread may have been waiting behind several sampling passes, so the
    ///    stamp comes from `MonitorPlane::advance_to`, whose `WriteStamp` is the
    ///    only thing the write entry points accept. See that type for why a stale
    ///    anchor is an unsafe-direction bug rather than a cosmetic one.
    /// 3. **ONE acquisition covers the attached-key read, the sample build and the
    ///    engine pass**, for the reason `sample_monitors` states and one more of its
    ///    own: the attached set is what protects a tapped row from retirement, so
    ///    reading it in a separate acquisition would let an `attach` land in between
    ///    and have the very next gather retire the row it just opened.
    ///
    /// 4. **The pass is ORDERED BY ITS GATHER, not by when it arrives.** The
    ///    latched clock fixes the plane's number line; it cannot make a STALE
    ///    ANSWER inert, because `now` is captured after the gather returns, so a
    ///    slow gather started first hands over LAST with a LATER instant. Its
    ///    liveness would overwrite newer evidence, and — worse — its SETTLED
    ///    verdict would retire rows the newer catalog still contains, destroying
    ///    their learned baselines. `gather_gen` (see [`Self::begin_catalog_gather`])
    ///    is taken before the gather, and the plane rejects any pass whose
    ///    generation is below the newest it has already applied.
    ///
    /// The DELTA counters are bumped OUTSIDE the lock; the GAUGE is published
    /// INSIDE it — see [`Self::sample_monitors`] for why a latest-wins store
    /// cannot be ordered outside the lock that produced its value.
    fn observe_catalog_monitors(
        &self,
        catalogs: &[CatalogReply],
        discovery: DiscoveryState,
        gather_gen: u64,
    ) {
        let now = Instant::now();
        let settled = discovery == DiscoveryState::Settled;
        let outcome = {
            let mut st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            let at = st.monitors.advance_to(now);
            let attached = attached_row_keys(&st.taps.list(), &st.stats);
            let samples = catalog_monitor_samples(catalogs, &attached, settled, at);
            let outcome = st
                .monitors
                .observe_catalog_pass(&samples, &attached, settled, at, gather_gen);
            // The GAUGE, published while the lock that produced it is still held.
            if let Some(outcome) = &outcome {
                self.monitor_rows_ineligible
                    .store(outcome.rows_ineligible, Ordering::Relaxed);
            }
            outcome
        };
        // A pass the plane DROPPED — throttled, or superseded by a newer gather —
        // reports nothing at all, and folding it would be wrong rather than merely
        // pointless: `monitor_rows_ineligible` is a GAUGE, so storing a dropped
        // pass's zero is a positive claim that nothing is being withheld from the
        // agent, made by a pass that looked at nothing.
        let Some(outcome) = outcome else {
            return;
        };
        self.monitor_catalog_samples_taken
            .fetch_add(outcome.observed, Ordering::Relaxed);
        self.monitor_catalog_rows_retired
            .fetch_add(outcome.retired, Ordering::Relaxed);
        // The ALERT counters are shared with the attached plane, deliberately: an
        // alert is an alert whichever plane observed it, and an agent asking "how
        // much has this desk raised?" wants one number. The SAMPLE counter is not
        // shared, and that is not symmetry-breaking for its own sake —
        // `monitor_samples_taken` is the no-inert-shipping observable for the
        // DRAIN-LOOP sampler, so a catalog pass bumping it would let a daemon that
        // deleted the sampler's call site keep the number climbing.
        self.monitor_alerts_raised
            .fetch_add(outcome.raised, Ordering::Relaxed);
        self.monitor_alerts_cleared
            .fetch_add(outcome.cleared, Ordering::Relaxed);
    }

    /// The `monitors` verb: every watched row's CURRENT state plus the recent
    /// alert ring.
    ///
    /// Read-only and prompt — one state-lock acquisition, no gather, no wait, no
    /// network (the contract every vizd handler is held to). It samples
    /// NOTHING: the sampler runs on the drain loop, so what this verb answers does
    /// not depend on how often anyone asks.
    ///
    /// Every served alert is stamped with its SERVE-TIME age
    /// ([`cerulion_viz::monitor::Alert::with_age`]) inside the same lock scope the
    /// rows are read under, so a response's rows and its alert ages are dated from
    /// ONE instant.
    fn monitors(&self, id: u64) -> Response {
        let now = Instant::now();
        let st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let now_ns = st.monitors.now_ns(now);
        Response::Monitors(MonitorsResponse {
            id,
            ok: true,
            monitors: st.monitors.rows(now_ns),
            alerts: st.monitors.alerts_at(now_ns / 1_000_000),
            seq: st.monitors.next_seq(),
        })
    }

    /// `runs`: the live `graph run`s this desk can see, this machine's
    /// own registry folded with every remote robot's.
    ///
    /// # It takes NO state lock, and that is a property rather than an accident
    ///
    /// Both arms are gathers: the local one listens on this machine's run registry
    /// (bounded by `RUN_GATHER_WINDOW`, with a zero-publisher fast path that returns
    /// instantly on a desk running nothing), and the remote one is one netd round
    /// trip. The poll thread needs the state lock every `DEFAULT_POLL_INTERVAL` to
    /// drain every tap, so holding it across either gather would stall the daemon's
    /// whole data path to answer a control question — the hazard
    /// [`Self::observe_catalog_monitors`] documents. This handler needs nothing out
    /// of [`DaemonState`], so it never takes the lock at all.
    ///
    /// # It GATHERS, but it never WAITS
    ///
    /// The contract is that no handler runs a multi-second CONVERGENCE wait
    /// (`viz_client` arms a 5 s reply deadline, so such a wait does not make the verb
    /// slower — it kills it). One bounded gather per arm is exactly what `discover`
    /// already does. What this verb owes a client that finds nothing is the FACTS —
    /// `completeness`, `discovery`, `unusable`, `degraded` — not a longer wait.
    ///
    /// # Cost, and the caching contract that makes it affordable
    ///
    /// A remote serve builds a fresh iceoryx2 reader node per call (~620 ms) plus a
    /// gather window. A run's DAG is immutable for the life of its `run_id`, so a
    /// controller fetches once per `run_id` and refreshes liveness off the
    /// `discover` poll it already makes. **This verb must never join a poll loop.**
    fn runs(&self, id: u64, robot: Option<String>) -> Response {
        // The two arms run CONCURRENTLY, and that is a DEADLINE fix rather than a
        // speed one. They are independent — one listens on this machine's run
        // registry, the other is a netd round trip — so run sequentially the wall
        // is their SUM, and `viz_client` arms a 5 s `SO_RCVTIMEO` on every reply
        // read: past it the client does not get a slow answer, it gets an ERRNO
        // and the verb dies. A LOCAL gather that spends its whole window (600 ms)
        // plus a remote arm at netd's own measured worst case is enough to cross
        // it, and the local half is the part a robot-scoped request already skips
        // — so a desk asking the ordinary unscoped question was the one shape that
        // could time out. Concurrent, the wall is their MAX.
        //
        // `std::thread::scope` keeps the handler SYNCHRONOUS from the protocol's
        // point of view (one request, one reply, no wait ADDED — this only ever
        // SHRINKS the worst case), and needs no `'static` bound, so neither arm's
        // borrow of `self` has to be cloned.
        let (local, remote) = std::thread::scope(|s| {
            // The LOCAL arm. Skipped when the request scoped itself to one robot: a
            // local run is attributed `robot: None` and no robot NAME can
            // select one, so running the gather could only ever produce rows the
            // request asked not to see.
            let local_arm = s.spawn(|| {
        // TEST-ONLY (`0` in production — nothing in the daemon writes it). The
        // local arm is the one that cannot be made slow on demand, and a slow
        // REMOTE arm alone cannot tell a sum from a max.
        let injected = self.runs_local_delay_ns.load(Ordering::Relaxed);
        if injected > 0 {
            std::thread::sleep(Duration::from_nanos(injected));
        }
        match &robot {
            Some(_) => LocalArm::NotAsked,
            None => match self.manager.gather_live_runs(RUN_GATHER_WINDOW) {
                // `gather_live_runs` asks THIS manager's namespace, never the
                // process-global one — on any isolated SHM root (every test, every
                // multi-process deployment) the global config is a different
                // machine.
                Ok(gather) => {
                    let fold = collect_run_entries(&gather.records);
                    for skip in &fold.skipped {
                        tracing::warn!(
                            run_id = %format_run_id(skip.run_id),
                            run_dir = %skip.run_dir,
                            reason = %skip.reason,
                            "cerulion-vizd: skipping a LOCAL run whose directory could not be \
                             read — it is reported as undescribable rather than dropped, because \
                             an entry carrying an empty document would render as a graph with no \
                             nodes"
                        );
                    }
                    // The gather's OWN verdict, whole — its `live_writers` /
                    // `writers_heard` counts are the diagnostic for the one
                    // machine an operator can go and look at, and a boolean
                    // would replace them with `not_established`'s zeroes.
                    local_reply(fold, gather.completeness.into())
                }
                Err(e) => {
                    // A gather that could not RUN is a fault worth seeing: the
                    // registry is on this very machine.
                    tracing::warn!(
                        error = %e,
                        "cerulion-vizd: the LOCAL run registry could not be gathered — reporting \
                         'could not establish', never 'this machine is running nothing'"
                    );
                    LocalArm::Failed(e.to_string())
                }
            },
        }
        });

            // The REMOTE arm — ONE netd round trip, on THIS thread (spawning one
            // helper is enough for two arms).
            let remote = match self.demand_plane.query_runs(robot.as_deref()) {
                Ok(gather) => RemoteArm::Answered {
                    replies: gather.replies,
                    unusable: gather.unusable,
                    silent: gather.silent,
                    discovery: DiscoveryLabel::from_state(gather.discovery),
                },
                Err(e) => {
                    tracing::warn!(
                        error = %e,
                        "cerulion-vizd: cerulion-netd unreachable for the runs gather — the LOCAL \
                         half of this answer still stands; the remote half is UNKNOWN (start \
                         cerulion-netd, or check the robot is reachable, then retry)"
                    );
                    RemoteArm::Degraded(e)
                }
            };

            // A PANICKING local arm becomes a reported failure rather than an unwind
            // through the control connection: the remote half of this answer is
            // already gathered and still true, and `LocalArm::Failed` is exactly the
            // "could not establish" non-claim that state deserves.
            let local = local_arm.join().unwrap_or_else(|_| {
                tracing::error!(
                    "cerulion-vizd: the LOCAL run gather PANICKED — reporting 'could not \
                 establish' for this machine; the remote half of the answer stands"
                );
                LocalArm::Failed("the local run gather panicked".to_string())
            });
            (local, remote)
        });

        let fold = fold_runs(local, remote);
        // A robot that answered UNUSABLY will re-serve the same bytes forever, so
        // its remedy is a REDEPLOY and never a wait. Loud, once per gather, because
        // nothing else on this desk will ever notice.
        for bad in &fold.unusable {
            tracing::warn!(
                robot = %bad.robot,
                reason = %bad.reason,
                "cerulion-vizd: a robot answered the runs GET with something this binary could \
                 not use — REDEPLOY that robot to a matching build; it is not 'still \
                 discovering' and re-asking cannot change the answer"
            );
        }
        // A robot that was ASKED and said NOTHING is the coverage shortfall: the
        // query ran and it was in it, so this is neither the redeploy above nor a
        // discovery gap. `debug!` rather than `warn!` because it is REACHABLE ON A
        // HEALTHY LAN — a robot whose binary predates the verb, or a Strict
        // ingress-only gateway that declares no query surface, answers nothing
        // forever and would otherwise warn on every poll (the flood
        // class). The response carries it, which is where a renderer reads it.
        for quiet in &fold.silent {
            tracing::debug!(
                robot = %quiet,
                "cerulion-vizd: a robot was asked for its runs and did not answer — nothing is \
                 claimed about what it is running; it may answer on a later poll, or it may \
                 predate the runs verb / serve no query surface at all"
            );
        }
        Response::Runs(RunsResponse {
            id,
            ok: true,
            runs: fold.runs,
            completeness: fold.completeness,
            sources: fold.sources,
            unusable: fold.unusable,
            silent: fold.silent,
            discovery: fold.discovery,
            degraded: fold.degraded,
        })
    }

    /// Gather a catalog from `cerulion-netd` — ONE round trip, always.
    ///
    /// `robot: Some` scopes it to one robot, `None` harvests every announcing robot.
    ///
    /// **The daemon never waits for discovery to converge, and that is the design.**
    /// vizd answers the CLOSED-WORLD question ("does this LAN serve it *now*?"); the
    /// OPEN-WORLD one ("*will* it?") has no bounded answer, and the party that knows
    /// how long a human will wait is the client. It also cannot wait: the only
    /// in-repo vizd control client arms a 5 s `SO_RCVTIMEO` on every reply read
    /// (`cerulion_cli_engine::viz_client`), so a reply past ~5 s does not arrive
    /// late — it never arrives, and `cerulion viz` aborts on an errno. What the
    /// daemon owes the client instead is the FACTS, which ride out on
    /// [`RetryHint`]; the client runs the shipped `ConvergenceWait` policy.
    fn gather_catalog(&self, robot: Option<&str>) -> Result<CatalogGather, String> {
        match robot {
            Some(robot) => self.demand_plane.query_catalog_with_discovery(robot),
            None => self.demand_plane.query_catalog_all_with_discovery(),
        }
    }

    /// Handle one parsed request → its response. Never panics.
    ///
    /// `conn` is the connection's own id — only [`Request::SubscribeEvents`] uses it
    /// (a subscription is scoped to the connection that asked for it).
    fn dispatch_for_conn(&self, req: Request, conn: u64) -> Response {
        if let Request::SubscribeEvents { id } = req {
            let (version, robots, connected) = self.events.subscribe(conn);
            return Response::SubscribeEvents(crate::protocol::SubscribeEventsResponse {
                id,
                ok: true,
                version,
                robots,
                connected,
            });
        }
        self.dispatch(req)
    }

    /// Handle one parsed request → its response. Never panics.
    fn dispatch(&self, req: Request) -> Response {
        match req {
            Request::Discover { id } => self.discover(id),
            Request::Attach {
                id,
                topic,
                entity,
                robot,
                schema,
            } => match robot {
                // The REMOTE arm: DEMAND the topic's
                // shared mirror from cerulion-netd then tap the re-injected local
                // mirror (remote = local).
                Some(robot) => self.attach_remote(id, topic, entity, robot, schema),
                // Local attach: the network is never touched.
                None => self.attach(id, topic, entity),
            },
            Request::Detach { id, topic } => self.detach(id, topic),
            Request::Representation {
                id,
                topic,
                representation,
            } => self.set_representation(id, topic, representation),
            Request::List { id } => self.list(id),
            Request::Status { id } => self.status(id),
            Request::SetBlueprint { id, layout } => self.set_blueprint(id, layout),
            Request::ComposeLayout { id, intent } => self.compose_layout(id, intent),
            // Side-load definitions (a bag player handing over the
            // types its recording carries). The network is never touched.
            Request::Schemas { id, docs } => self.schemas(id, docs),
            // Monitors: read-only, one lock, no gather (see `Ctx::monitors`).
            Request::Monitors { id } => self.monitors(id),
            // The local run registry folded with every robot's. Two
            // bounded gathers, NO state lock, no wait (see `Ctx::runs`).
            Request::Runs { id, robot } => self.runs(id, robot),
            // A subscription is scoped to the CONNECTION that asked for it, so
            // it is served by `dispatch_for_conn` (which knows the connection id) and
            // can never arrive here. Answering with a structured error rather than
            // panicking keeps the "never panics" contract if a future caller routes a
            // request through the connection-less path by mistake.
            Request::SubscribeEvents { id } => Response::error(
                Some(id),
                "subscribe_events must be dispatched on a connection (internal routing error)",
                None,
            ),
        }
    }

    /// A cheap snapshot of the current schema-resolution walker (a read lock + an
    /// `Arc` clone). Every walker READER goes through this so a concurrent seed
    /// (write lock) never tears a read.
    fn walker_snapshot(&self) -> Arc<FrameWalker> {
        Arc::clone(&self.walker.read().unwrap())
    }

    /// `schemas`: fold caller-supplied definitions into the daemon's
    /// schema universe, so a type NEITHER the built-in corpus NOR the bridge
    /// config's `msg_dirs` covers becomes decodable and renderable.
    ///
    /// This is the LOCAL counterpart to the remote arm's fetch-and-seed: the same
    /// `seed_walker_with_docs` machinery, reached without a robot. A bag player
    /// is the motivating caller — its frames land on local SHM, so no remote
    /// attach ever runs, and without this verb the definitions the bag carries have
    /// nowhere to go.
    ///
    /// Answers with what CHANGED (`offered`/`accepted`/`known`), so a caller can
    /// distinguish "you learned something" from "you already had all of this"
    /// without inferring it. An empty `docs` list is a legal no-op query of that
    /// count, not an error.
    ///
    /// `accepted` counts docs the WALKER took, not docs the list absorbed. Those
    /// differ: a YAML-encoded doc and one whose `.msg` text fails to parse are
    /// both warn-SKIPPED by the walker, and reporting them as accepted would be
    /// the same success-shaped lie this whole feature exists to remove — the
    /// caller would log "those types are now decodable" over a scene that still
    /// renders nothing.
    ///
    /// A doc naming a BUILT-IN is REFUSED outright. Docs are folded in last and
    /// the layout resolver is last-insert-wins, so accepting one would redefine
    /// that type for EVERY topic in the daemon — including live robot taps that
    /// have nothing to do with the caller. The refusal is unconditional because
    /// that blast radius is: one caller's doc must never change what every other
    /// consumer decodes, whatever the caller intended.
    ///
    /// The guard does NOT rest on "the bag recorder never ships built-in
    /// text": that is false and enforced nowhere — a workspace shadowing a built-in is a
    /// supported configuration, and a bag recorded under it legitimately carries
    /// the shadow's text (it is the only definition that explains the recorded
    /// hash). See
    /// `cerulion_viz::schema_registry::is_builtin_schema_name`.
    fn schemas(&self, id: u64, docs: Vec<SchemaDoc>) -> Response {
        let offered = docs.len() as u32;
        let before = self.walker_snapshot();
        let (docs, shadowing): (Vec<_>, Vec<_>) = docs
            .into_iter()
            .partition(|d| !cerulion_viz::schema_registry::is_builtin_schema_name(&d.qualified));
        let mut rejected: Vec<String> = shadowing.iter().map(|d| d.qualified.clone()).collect();
        if !rejected.is_empty() {
            tracing::warn!(
                count = rejected.len(),
                types = ?rejected,
                "cerulion-vizd: REFUSED side-loaded definitions that would redefine a BUILT-IN \
                 type — a side-load applies daemon-wide, so accepting one would change how every \
                 topic of that type is decoded, including live taps unrelated to the caller"
            );
        }

        // A SET: `accepted` counts TYPES that became decodable, and one offer may
        // carry the same qualified name twice (nothing upstream dedups a
        // hand-built request or a robot's served closure). As a list it reported
        // `accepted: 2` beside `known: 1` for ONE type — a count the response's
        // own fields contradict.
        let wanted: BTreeSet<String> = docs.iter().map(|d| d.qualified.clone()).collect();
        let (walker, absorbed) = self.seed_walker_with_docs(docs);

        // Which docs did NOT install? `seed_walker_with_docs` is authoritative
        // here: it VALIDATES each offered name against the rebuilt walker
        // and rolls the refused ones back to their incumbents, so it is the only
        // place that knows the difference. A walker-based partition here CANNOT
        // recover that — a refusal whose incumbent survived still answers
        // `knows(name) == true`, so this verb would report a refused doc as
        // learned: a success-shaped failure.
        let unusable = absorbed.refused.clone();
        if !unusable.is_empty() {
            tracing::warn!(
                count = unusable.len(),
                types = ?unusable,
                "cerulion-vizd: side-loaded definitions the schema walker could not use (a \
                 workspace-YAML doc, which this daemon has no parser for, or a .msg it could not \
                 parse) — the OFFERED definition is not installed and a corrected offer will be \
                 taken. Any definition this daemon ALREADY held for these names is UNTOUCHED and \
                 still resolves, so a refusal here does not by itself mean the type is \
                 undecodable"
            );
            rejected.extend(unusable.iter().cloned());
        }
        // What the caller can now decode BECAUSE of this offer: known to the
        // walker AND not refused. The second half matters — a refused doc whose
        // incumbent survived is still `knows`-true, and calling that "learned"
        // would credit the offer with a type it did not supply.
        let learned: Vec<String> = wanted
            .into_iter()
            .filter(|q| walker.knows(q) && !unusable.contains(q))
            .collect();

        // `accepted` = became decodable and was NOT before. A doc that UPDATED an
        // already-known type is reported separately (`replaced`), because that is
        // a destructive event for whatever was decoding the previous definition,
        // not a gain.
        let accepted = learned.iter().filter(|q| !before.knows(q)).count() as u32;
        let known = self.seed.lock().unwrap().docs.len() as u32;
        // A replacement that was ROLLED BACK did not happen: the
        // incumbent is still installed, nothing went dark, and reporting it as
        // replaced would tell an operator their working type was just overwritten.
        let replaced: Vec<String> = absorbed
            .replaced
            .iter()
            .filter(|q| !unusable.contains(q))
            .cloned()
            .collect();
        if !replaced.is_empty() {
            tracing::warn!(
                types = ?replaced,
                "cerulion-vizd: a side-load REPLACED definitions this daemon already held — the \
                 walker keys layouts by qualified name, so the previous definition's wire hash \
                 stops resolving and anything still producing it goes dark until it is re-seeded"
            );
        }
        if accepted > 0 {
            tracing::info!(
                offered,
                accepted,
                known,
                "cerulion-vizd: side-loaded schema definitions — those types are now decodable"
            );
        } else if offered > 0 {
            tracing::debug!(
                offered,
                known,
                rejected = rejected.len(),
                "cerulion-vizd: a side-load made nothing NEW decodable (already known, refused as \
                 built-in shadowing, or unusable)"
            );
        }
        Response::Schemas(crate::protocol::SchemasResponse {
            id: Some(id),
            ok: true,
            offered,
            accepted,
            known,
            replaced,
            rejected,
        })
    }

    /// Fold a robot-served schema-closure into the daemon's schema universe
    /// (remote arm): accumulate `docs`, rebuild the walker from
    /// builtins ∪ store ∪ all-accumulated-docs, install it as the resolution
    /// walker, and queue the SAME walker for the render worker (the poll thread
    /// hands it over via `try_swap_walker`). Returns the rebuilt walker so the
    /// caller can immediately resolve the just-fetched type's wire hash, plus
    /// what changed (an unchanged [`Absorbed`] = nothing was rebuilt — see
    /// [`SeedState::absorb`]).
    ///
    /// The heavy rebuild (a full builtin-corpus reparse) runs OFF the `seed` lock
    /// the poll thread contends every iteration — the `seed` lock is held only for
    /// the fast doc-append/snapshot and the fast install/queue. `rebuild_lock`
    /// serializes concurrent seeds so two attaches can't install an older walker
    /// over a newer one. Neither lock is the poll thread's hot path, so
    /// a seed never blocks tap draining.
    ///
    /// # VALIDATE, THEN COMMIT
    ///
    /// A replacement becomes visible to the served walker ONLY after a rebuilt
    /// walker has accepted it. `absorb` merges into `seed.docs` first (it must —
    /// the rebuild reads that set), but every replacement it makes is recorded in
    /// [`Absorbed::previous`], so a name the rebuilt walker cannot use is ROLLED
    /// BACK to its incumbent and the offered doc is remembered as refused. Nothing
    /// a walker refused is ever installed and nothing an offer displaced is ever
    /// lost.
    ///
    /// The rollback lives HERE rather than at the `schemas` verb because BOTH callers
    /// reach this seam and only one of them reconciles: the remote-attach path
    /// (`seed_and_resolve_schema`) discards its `Absorbed` entirely, so without it a robot
    /// serving one malformed doc for a held name would poison `seed.docs`
    /// permanently, with no verb-level `forget` to undo it.
    ///
    /// # Lock ordering
    ///
    /// `rebuild_lock` is held across the WHOLE sequence, so validate→rollback→
    /// re-install stays atomic with respect to another seed exactly as
    /// append→rebuild→install was. The `seed` lock is still taken only in short,
    /// allocation-free-ish critical sections (merge+snapshot, rollback+snapshot,
    /// queue-the-swap) and NEVER across a rebuild; both rebuilds run with it
    /// released, so the poll thread's per-iteration `seed` acquisition is
    /// unaffected. The poll thread takes neither lock on its draining path.
    ///
    /// The refusal path pays a SECOND rebuild. That is deliberate: the candidate
    /// walker contains a doc that must not be installed, so the only
    /// alternative to rebuilding from the committed set is installing something
    /// the daemon has decided to reject. It is also the rare path — an identical
    /// re-offer of a refused doc short-circuits in `absorb` and never gets here.
    fn seed_walker_with_docs(&self, docs: Vec<SchemaDoc>) -> (Arc<FrameWalker>, Absorbed) {
        // Serialize seeds (the poll thread never takes this) so the append→rebuild→
        // install sequence is atomic w.r.t. another concurrent seed.
        let _rebuild = self.rebuild_lock.lock().unwrap();
        // Append + snapshot the full accumulated doc set under the FAST seed lock.
        //
        // Only docs that are genuinely NEW are appended. A rebuild
        // re-parses the whole built-in corpus, and the doc list is monotonic, so
        // an unfiltered `extend` made a repeated seed of the SAME closure both
        // grow the list without bound and pay that rebuild every time. It was
        // already reachable (re-attaching a remote topic re-fetches its closure);
        // the bag player makes it routine, since it re-offers its definitions
        // while it plays so a viewer started mid-playback still learns them.
        // A SET, not a list: one offer may legitimately carry the same qualified
        // name twice (a hand-built `schemas` request, a robot's served closure —
        // neither is deduped upstream), and every downstream use here is
        // set-shaped. See `SeedState::rollback` for what a duplicate did.
        let offered: BTreeSet<String> = docs.iter().map(|d| d.qualified.clone()).collect();
        let (all_docs, mut absorbed) = {
            let mut seed = self.seed.lock().unwrap();
            let absorbed = seed.absorb(docs);
            if !absorbed.changed() {
                // Nothing to rebuild — but `absorbed` may still name docs the
                // refusal memory short-circuited, and the caller must hear about
                // those rather than be told the offer was a clean no-op.
                return (self.walker_snapshot(), absorbed);
            }
            (seed.docs.clone(), absorbed)
        };
        // Heavy CANDIDATE rebuild OFF the poll-contended seed lock.
        let candidate = walker_from_bridge_config_env_with_docs(&all_docs);
        // VALIDATE: of the names this offer installed, which can the candidate
        // walker not use? Names the refusal memory already short-circuited are
        // excluded — `absorb` installed nothing for them, so there is nothing to
        // roll back, and they are already reported.
        let refused: BTreeSet<String> = offered
            .into_iter()
            .filter(|q| !absorbed.refused.contains(q) && !candidate.knows(q))
            .collect();

        let arc = if refused.is_empty() {
            Arc::new(candidate)
        } else {
            // ROLL BACK under the FAST seed lock, then rebuild from what was
            // actually COMMITTED — the candidate holds docs the daemon refused,
            // so installing it is exactly the bug this guards.
            let committed = {
                let mut seed = self.seed.lock().unwrap();
                seed.rollback(&refused, &absorbed.previous);
                seed.docs.clone()
            };
            absorbed.refused.extend(refused);
            Arc::new(walker_from_bridge_config_env_with_docs(&committed))
        };
        // Queue a clone for the render worker (poll thread applies it, FIFO with
        // data batches) under the FAST seed lock.
        {
            let mut seed = self.seed.lock().unwrap();
            seed.pending_worker_swap = Some((*arc).clone());
        }
        // Install the resolution walker for status/list/attach + the poll thread.
        *self.walker.write().unwrap() = Arc::clone(&arc);
        (arc, absorbed)
    }

    /// The daemon's [`NetworkManager`]. Since the INGRESS/mirror plane
    /// moved to `cerulion-netd` (one gateway per computer), this session is used
    /// ONLY for the remote arm's schema QUERY surface — the catalog + schema-closure
    /// GETs `resolve_remote_type` needs to learn a topic's type before demanding it
    /// from netd. (Folding those GETs into netd's query surface would leave vizd with NO
    /// network session at all.) A `network: None` daemon
    /// (kill-switched or a local-only test transport) returns the explicit
    /// not-network-configured residual so a remote attach fails loudly, never
    /// silently.
    fn robot_network(&self) -> Result<&NetworkManager, String> {
        self.manager.network().ok_or_else(|| {
            "cerulion-vizd is not network-configured, so it cannot reach a remote robot — run \
             the daemon with its network default (scouting ON), i.e. WITHOUT \
             CERULION_VIZD_NETWORK=off"
                .to_string()
        })
    }

    /// Resolve a remote `topic`'s ROS type to `(qualified_name, schema_hash)` for
    /// `register_ingress_topic` (remote arm). Three paths, in order:
    ///
    /// 1. `schema` PINNED + already known to the walker → resolve locally (no
    ///    network) — the fast path for a built-in / workspace / already-seeded type.
    /// 2. `schema` PINNED but a custom type the walker lacks → fetch its `.msg`
    ///    closure from the robot + seed the walker.
    /// 3. `schema` OMITTED → resolve the type NAME from the robot's served CATALOG
    ///    (the "no schema pin!" product bar), then resolve locally if the desk
    ///    already has that type, else fetch + seed as in (2).
    ///
    /// A hard, precise error is returned ONLY when the robot genuinely does not
    /// serve the topic/type, or the daemon is not network-configured.
    fn resolve_remote_type(
        &self,
        topic: &str,
        robot: &str,
        schema: Option<&str>,
    ) -> Result<(String, u64), AttachError> {
        let name = match schema {
            // (1)+(2): a pinned schema. Resolve locally, or fetch if unknown.
            Some(name) => match resolve_pinned_schema(&self.walker_snapshot(), name)? {
                Some(resolved) => return Ok(resolved),
                None => name.to_string(),
            },
            // (3): schema-less — learn the type from the robot's catalog.
            None => self.catalog_resolve_type(robot, topic)?,
        };
        // The type name is known (pinned-but-unknown, or catalog-resolved) but the
        // walker does not have it. If a CONCURRENT attach already seeded it since
        // the snapshot above, resolve locally; else fetch + seed.
        if let Some(hash) = self.walker_snapshot().schema_hash_for(&name) {
            return Ok((name, hash));
        }
        // NO retry hint on the schema-fetch half — the catalog already named
        // the producing robot, so the robot IS discovered and a schema miss is not a
        // discovery-convergence problem (the convergence wait's own decision for the second,
        // robot-scoped `query_schema` in `netd_resolve_topic_schema`). Re-asking would
        // burn a client's budget on an answer that cannot change.
        self.fetch_and_seed_type(robot, &name)
            .map_err(AttachError::from)
    }

    /// Learn a remote `topic`'s qualified ROS type from `robot`'s served CATALOG
    /// (the catalog verb). The query runs over the shared `cerulion-netd` query
    /// plane (ONE zenoh session per computer) — a netd-unreachable error LOUDLY
    /// degrades to vizd's OWN transient session (never a hard failure). A precise
    /// error when the robot does not answer, or answers without a type name.
    ///
    /// The "no type for this topic" answer is an ABSENCE CLAIM, so it rides
    /// the same gate as `attach_resolving_remote` — one hop over, and the same
    /// defect class. Two things were wrong before: an unconverged gather licensed a
    /// terminal verdict, AND `catalog_resolve_error` asserts the robot "serves a
    /// catalog" even when the gather came back EMPTY (no catalog at all), which is a
    /// claim the caller never had evidence for.
    ///
    /// It answers in ONE round trip and, when the answer does not name a type
    /// for THIS topic, attaches a [`RetryHint`] so the client can decide whether to keep
    /// asking. The predicate is the SEAM's question — "did this robot's catalog name a
    /// type for this topic" — not "was the gather empty": on a two-robot LAN where A has
    /// answered, netd reports `Settled` and a NON-empty gather while B's topics are
    /// still undiscovered, and that answer is a snapshot, not a verdict about the future.
    fn catalog_resolve_type(&self, robot: &str, topic: &str) -> Result<String, AttachError> {
        match self.gather_catalog(Some(robot)) {
            Ok(gather) => {
                let (catalogs, discovery) = (&gather.catalogs, gather.discovery);
                resolve_type_from_catalogs(catalogs, topic).ok_or_else(|| {
                    let message = match catalog_absence_verdict(robot, topic, catalogs, discovery) {
                        Some(msg) => msg,
                        None => catalog_resolve_error(robot, topic),
                    };
                    // EVERY arm here is "no type for this topic, right now" — including
                    // the settled one, which is a snapshot of a LAN a robot can join at
                    // any moment. The hint says re-asking may change it; the LABEL says
                    // how much was actually known.
                    AttachError::not_found_yet(message, &gather)
                })
            }
            Err(e) => {
                tracing::warn!(
                    robot = %robot, topic = %topic, error = %e,
                    "cerulion-vizd: cerulion-netd unreachable for the schema catalog — falling back \
                     to vizd's own zenoh session (the transient path)"
                );
                self.catalog_resolve_type_transient(robot, topic)
                    .map_err(AttachError::from)
            }
        }
    }

    /// The transient path for [`Self::catalog_resolve_type`] — GET the catalog
    /// over vizd's OWN zenoh session (the fallback when netd is unreachable).
    fn catalog_resolve_type_transient(&self, robot: &str, topic: &str) -> Result<String, String> {
        let net = self.robot_network()?;
        let session = net
            .session()
            .map_err(|e| format!("could not open the daemon's zenoh session: {e}"))?;
        let catalog = query_robot_catalog(session, robot, CATALOG_GATHER_WINDOW)
            .ok_or_else(|| catalog_no_answer_error(robot, topic))?;
        resolve_type_from_catalogs(std::slice::from_ref(&catalog), topic)
            .ok_or_else(|| catalog_resolve_error(robot, topic))
    }

    /// Fetch `name`'s `.msg` closure from `robot` (the `schema` verb), seed the
    /// walker with it, and return the now-resolvable `(name, hash)`. The
    /// fetch runs over the shared `cerulion-netd` query plane — a netd-unreachable
    /// error LOUDLY degrades to vizd's OWN transient session. A precise error when the
    /// robot does not serve the type.
    fn fetch_and_seed_type(&self, robot: &str, name: &str) -> Result<(String, u64), String> {
        match self.demand_plane.query_schema(robot, name) {
            // netd answered authoritatively — classify: the first reply carrying docs
            // wins (seed + resolve); an all-empty answer surfaces the robot's OWN
            // refusal/not-found reason (MESSAGE parity with the transient path); no
            // reply at all is reported as "did not answer".
            Ok(replies) => {
                self.seed_and_resolve_schema(robot, name, classify_schema_replies(replies))
            }
            Err(e) => {
                tracing::warn!(
                    robot = %robot, name = %name, error = %e,
                    "cerulion-vizd: cerulion-netd unreachable for the schema fetch — falling back to \
                     vizd's own zenoh session (the transient path)"
                );
                self.fetch_and_seed_type_transient(robot, name)
            }
        }
    }

    /// The transient path for [`Self::fetch_and_seed_type`] — GET the schema
    /// over vizd's OWN zenoh session (the fallback when netd is unreachable).
    fn fetch_and_seed_type_transient(
        &self,
        robot: &str,
        name: &str,
    ) -> Result<(String, u64), String> {
        let net = self.robot_network()?;
        let session = net
            .session()
            .map_err(|e| format!("could not open the daemon's zenoh session: {e}"))?;
        // `query_robot_schema` yields the FIRST decodable reply (which MAY be a
        // docs-empty not-found/refused answer carrying a reason) or `None` (no
        // answer). Classify it the SAME way as the netd path so the error MESSAGE is
        // accurate: a not-found reply surfaces its reason, `None` stays the distinct
        // "did not answer" arm — never the misleading "served closure failed to
        // parse" over an empty reply.
        let replies: Vec<SchemaReply> =
            query_robot_schema(session, robot, name, CATALOG_GATHER_WINDOW)
                .into_iter()
                .collect();
        self.seed_and_resolve_schema(robot, name, classify_schema_replies(replies))
    }

    /// SHARED (netd + transient): given the desk-side [`SchemaFetchOutcome`] of a
    /// schema fetch, seed the walker with the served `.msg`/YAML closure and resolve
    /// `name` to its wire `(name, hash)`, OR surface a PRECISE error.
    /// [`SchemaFetchOutcome::NotServed`] carries the robot's OWN refusal/not-found
    /// reason (never the misleading "served closure failed to parse" claim over an
    /// EMPTY reply), and
    /// [`SchemaFetchOutcome::NoAnswer`] keeps the unreachable-vs-answered distinction.
    /// The parse-failure error fires ONLY when a NON-EMPTY served closure genuinely
    /// fails to resolve a hash.
    fn seed_and_resolve_schema(
        &self,
        robot: &str,
        name: &str,
        outcome: SchemaFetchOutcome,
    ) -> Result<(String, u64), String> {
        let docs = match outcome {
            SchemaFetchOutcome::Served(docs) => docs,
            SchemaFetchOutcome::NotServed(reason) => {
                return Err(schema_not_served_error(robot, name, &reason))
            }
            SchemaFetchOutcome::NoAnswer => return Err(schema_no_answer_error(robot, name)),
        };
        // What changed is mostly the `schemas` verb's business — but a REFUSAL is
        // this caller's business too, see below.
        let (seeded, absorbed) = self.seed_walker_with_docs(docs);
        // The robot served a definition for a name this daemon ALREADY held, and
        // the rebuilt walker refused the new one — so the incumbent survived and
        // the resolve below succeeds on ITS hash rather than the served type's.
        //
        // Attaching on a possibly-STALE definition is the right trade (the
        // alternative, letting one bad served doc take a working type dark
        // daemon-wide, is what the rollback exists to prevent) but it must be
        // LOUD: without the rollback this attach would fail loudly, because
        // the poisoned walker would not know the name. A silent success would make
        // the robot's redeployed message look like it landed. The mismatch, if
        // real, surfaces downstream as frames that do not decode — so say it here,
        // where the robot and the doc are still in hand.
        report_incumbent_fallback(robot, name, &absorbed.refused);
        seeded
            .schema_hash_for(name)
            .map(|hash| (name.to_string(), hash))
            .ok_or_else(|| {
                format!(
                "fetched schema '{name}' from robot '{robot}' but could not resolve its wire hash \
                 (the served `.msg` closure failed to parse, or was YAML-encoded — the viz daemon \
                 seeds only `.msg` types)"
            )
            })
    }

    /// `set_blueprint`: validate the client's rerun-agnostic layout (or reset to
    /// the Go2 default when `layout` is absent/null), soft-warn on view origins
    /// that match no attached topic, and hand the plan to the never-block worker.
    /// Answers with a [`SetBlueprintResponse`] — the blueprint was VALIDATED +
    /// ENQUEUED, NOT a render acknowledgement (the daemon never blocks on
    /// rendering) — or a structured error naming the problem VERBATIM.
    fn set_blueprint(&self, id: u64, layout: Option<LayoutSpec>) -> Response {
        // Reset (absent/null layout) → back to the AUTO consolidated default
        // (Scene + one plot per attached topic, tiled — NOT the bare Scene-only Go2
        // default), re-enabling auto-recompose on attach/detach; a spec → validate it
        // into the inspectable plan (an unknown kind / bad sizing is a loud, actionable
        // error, never a silent drop) and, on a successful apply below, take over as
        // the EXPLICIT layout (attach/detach stops clobbering it).
        let (plan, reset) = match layout {
            None => (self.consolidated_default_plan(), true),
            Some(spec) => match build_plan(&spec) {
                Ok(plan) => (plan, false),
                Err(e) => return Response::error(Some(id), e.to_string(), None),
            },
        };

        // The live render placements (topic + entity + resolved archetype) — the
        // input to BOTH the layout guardrails and the soft origin-grounding
        // hint (derived once so they see the identical attached set).
        let attached = self.attached_render_infos();

        // Layout guardrails: REFUSE a PROVABLY-broken layout, meaning every view
        // ungrounded (the all-guessed-origins failure), a spatial2d view that
        // renders NOTHING (the LaserScan shape), or an ungrounded named view
        // with auto_views off. `set_blueprint` stays the power escape hatch, so an
        // unusual-but-valid layout still applies — and a mixed spatial2d view returns
        // SOFT warnings (its invisible 3D-only siblings) merged into the response. A
        // RESET (Go2 default) is the built-in fallback — never guardrail-checked.
        let mut warnings: Vec<String> = Vec::new();
        if !reset {
            match validate_plan_against_attached(&plan, &attached) {
                Ok(soft) => {
                    for w in &soft {
                        tracing::warn!(
                            warning = %w,
                            "set_blueprint: a view renders a grounded topic invisibly — applied anyway"
                        );
                    }
                    warnings = soft;
                }
                Err(e) => return Response::error(Some(id), e.to_string(), None),
            }
        }

        // Soft-warn: view origins that match NO currently-attached topic. Never an
        // error — the agent may lay out a dashboard BEFORE attaching the topics
        // that populate it (Discover → SetLayout → Attach is a valid order too).
        let entities: Vec<String> = attached.into_iter().map(|a| a.entity).collect();
        let origin_warnings = plan_origins_unmatched(&plan, &entities);
        for origin in &origin_warnings {
            tracing::warn!(
                origin = %origin,
                "set_blueprint: a view origin matches no attached topic — attach it, or ignore if intentional"
            );
        }
        // The response carries BOTH warning classes (the invisible-sibling hints +
        // the ungrounded-origin hints) so Studio can surface every one.
        warnings.extend(origin_warnings);

        let views = plan.view_count();
        // Hand the plan to the worker (the ONE thread that owns + may block on the
        // `RecordingStream`). Bounded → an explicit error if the worker is
        // wedged (viewer stuck), never a hang, never a silent drop.
        // Send + record the mode ATOMICALLY. A RESET returns to Auto (the
        // dynamic default resumes); a spec TAKES OVER as Explicit — and the mode flips
        // ONLY on a successful send (a Busy/WorkerGone failure never leaves the mode
        // changed with a stale layout).
        let new_mode = if reset {
            LayoutMode::Auto
        } else {
            LayoutMode::Explicit
        };
        match self.apply_layout_atomic(plan, new_mode) {
            Ok(()) => Response::SetBlueprint(SetBlueprintResponse {
                id: Some(id),
                ok: true,
                views,
                reset,
                warnings,
            }),
            Err(e) => Response::error(
                Some(id),
                format!("set_blueprint could not reach the viz worker: {e}"),
                None,
            ),
        }
    }

    /// `compose_layout` — compile a SEMANTIC [`LayoutIntent`] (topic
    /// terms) into a grounded blueprint and apply it. The agent never names an
    /// entity path or a rerun view kind: vizd attaches any missing topic, resolves
    /// each, and hands the resolved set to the PURE [`compile_layout`] compiler,
    /// then validates HARD through [`validate_plan_against_attached`] (reuse, never
    /// reimplement) and emits through the SAME worker path `set_blueprint` uses (so
    /// a reconnect re-applies the composed layout). Answers with the placement echo
    /// + the applied arrangement, or a structured error naming the problem VERBATIM.
    fn compose_layout(&self, id: u64, intent: LayoutIntent) -> Response {
        // 1. Parse the strategy FIRST (loud, before any attach side effect).
        let strategy = match ComposeStrategy::from_wire(&intent.strategy) {
            Ok(s) => s,
            Err(e) => return Response::error(Some(id), e.to_string(), None),
        };

        // 2. The intent shape: EXACTLY one of `groups` / `topics`, both non-empty.
        //    Parse each group's role loudly (also before any attach). Yields the
        //    raw (topics, role, title) groups (bare-topics = ONE role-less group).
        let raw_groups = match parse_intent_groups(&intent) {
            Ok(g) => g,
            Err(msg) => return Response::error(Some(id), msg, None),
        };

        // 3. attach-if-missing + resolve each topic → the resolved compiler groups.
        //    Every failure from here on is TRANSACTIONAL: the taps
        //    THIS call opened (`attached_now`) are rolled back via `compose_fail`, so
        //    a compose that fails at ANY step leaves the tap set exactly as it was.
        //    A shared per-request peek budget caps the whole batch's
        //    silent-topic peeks instead of paying PEEK_BOUND per topic serially.
        let peek_deadline = Instant::now() + COMPOSE_PEEK_BUDGET;
        let mut attached_now: Vec<String> = Vec::new();
        let mut compose_groups: Vec<ComposeGroup> = Vec::new();
        for (topics, role, title) in raw_groups {
            let mut resolved = Vec::new();
            for topic in topics {
                match self.compose_resolve_topic(&topic, intent.attach_missing, peek_deadline) {
                    Ok((render, newly)) => {
                        if newly {
                            attached_now.push(topic.clone());
                        }
                        resolved.push(render);
                    }
                    Err(e) => {
                        return self.compose_fail(
                            id,
                            format!("compose_layout: could not attach '{topic}': {e}"),
                            Some(topic),
                            &attached_now,
                        );
                    }
                }
            }
            compose_groups.push(ComposeGroup {
                topics: resolved,
                role,
                title,
            });
        }
        // A new tap changed the attached set → keep the reapply-warn snapshot current.
        self.refresh_attached_snapshot();
        // Monitors: compose does NOT record attribution, and that is what avoids
        // the tombstone hazard rather than an omission. A tap's robot is
        // maintained by the SAMPLER from the live map (see
        // `DaemonState::refresh_hinted_origin_robots`), so a compose-attached mirror
        // is monitored under its robot within one sampling interval WITHOUT this
        // handler recording a name-keyed claim that a retired-then-reused topic
        // would make permanently wrong.

        // 4. Compile (PURE) — a role-incompatible topic or a zero-placement compile
        //    is a loud, actionable refusal here.
        let input = ComposeInput {
            groups: compose_groups,
            strategy,
            include_robot: intent.include_robot,
        };
        let composed = match compile_layout(&input) {
            Ok(c) => c,
            Err(e) => return self.compose_fail(id, e.to_string(), None, &attached_now),
        };

        // 5. Validate HARD through the reused plan guardrails (defense in depth —
        //    the compiler grounds by construction, so this normally passes). Validate
        //    against the entity set the compiler GROUNDED the plan over = the
        //    currently-attached topics UNION the PLACED topics: under
        //    attach_missing:false a placed topic may be deliberately un-attached and
        //    thus absent from attached_render_infos(), so validating against attached
        //    ALONE would falsely refuse a plan the compiler grounded by construction.
        let validation_set = self.compose_validation_set(&input.groups);
        let mut warnings = composed.warnings.clone();
        match validate_plan_against_attached(&composed.plan, &validation_set) {
            Ok(soft) => warnings.extend(soft),
            Err(e) => return self.compose_fail(id, e.to_string(), None, &attached_now),
        }

        // 6. Build the placement echo + emit through the SAME worker path (which
        //    REMEMBERS the plan for the reconnect-reapply contract).
        let views = composed.plan.view_count();
        let arrangement = composed.arrangement.as_str().to_string();
        let placements: Vec<Placement> = composed
            .placements
            .iter()
            .map(|p| Placement {
                topic: p.topic.clone(),
                entity: p.entity.clone(),
                archetype: p.archetype.map(|a| archetype_name(a).to_string()),
                view: p.view.as_str().to_string(),
                components: p.archetype.map(|a| {
                    archetype_components(a)
                        .iter()
                        .map(|s| s.to_string())
                        .collect()
                }),
                // An EXPLICIT (role-tagged) placement of a dead route (user
                // intent wins) is flagged so Studio can tell the user.
                dead: p.dead,
            })
            .collect();

        // The AUTOMAGIC-path topics skipped as dead routes, surfaced
        // EXPLICITLY (never a silent drop) so the agent can mention them to the user.
        let skipped_dead: Vec<SkippedDead> = composed
            .skipped_dead
            .iter()
            .map(|s| SkippedDead {
                topic: s.topic.clone(),
                reason: s.reason.clone(),
            })
            .collect();

        // A successful compose TAKES OVER as the explicit layout
        // (attach/detach stops auto-recomposing over it) — send + mode-flip atomically,
        // flipping ONLY on a successful send.
        match self.apply_layout_atomic(composed.plan, LayoutMode::Explicit) {
            Ok(()) => {
                attached_now.sort();
                Response::ComposeLayout(ComposeLayoutResponse {
                    id: Some(id),
                    ok: true,
                    placements,
                    arrangement,
                    views,
                    attached: attached_now,
                    warnings,
                    skipped_dead,
                })
            }
            Err(e) => self.compose_fail(
                id,
                format!("compose_layout could not reach the viz worker: {e}"),
                None,
                &attached_now,
            ),
        }
    }

    /// Build a `compose_layout` failure response,
    /// TRANSACTIONALLY rolling back every tap THIS call opened (`opened`) so a
    /// compose that fails at any step leaves the tap set exactly as it was — never a
    /// half-attached residue the agent must clean up. The rollback is disclosed in
    /// the message (loud, never silent) when it actually detached something.
    fn compose_fail(
        &self,
        id: u64,
        msg: String,
        topic: Option<String>,
        opened: &[String],
    ) -> Response {
        let msg = if opened.is_empty() {
            msg
        } else {
            format!(
                "{msg} (rolled back {} tap(s) this compose_layout opened)",
                opened.len()
            )
        };
        self.rollback_compose_attaches(opened);
        Response::error(Some(id), msg, topic)
    }

    /// Detach every tap `opened` by the current `compose_layout` call and refresh the
    /// reapply snapshot once. Best-effort per topic.
    ///
    /// **This ALSO releases the netd demand.** `attach_for_compose` routes
    /// through the demand plane, so a plain detach + stats removal is NOT the
    /// exact inverse of it. A
    /// compose that demanded a remote mirror and then failed (no renderable
    /// archetype, a refused layout) would otherwise drop the tap while LEAVING the
    /// demand held: netd keeps re-injecting, frames keep crossing the network for
    /// the life of this daemon, and there is no row and no `detach` target to stop
    /// it. The rollback must be the inverse of what the attach ACTUALLY did.
    ///
    /// Ordering mirrors the `detach` verb: taps + stats under the lock, then the
    /// release OUTSIDE it (a network round-trip, and `perform_remote_release` takes
    /// the lock itself). A release Err is LOGGED, never propagated — the rollback's
    /// job is to undo as much as it can, and failing it would strand the tap too.
    /// For a topic that was never demanded (a genuine local compose) the release is
    /// a documented no-op, so this is safe for every compose.
    fn rollback_compose_attaches(&self, opened: &[String]) {
        if opened.is_empty() {
            return;
        }
        {
            let mut st = self.state.lock().unwrap();
            for topic in opened {
                st.taps.detach(topic);
                let removed = st.stats.remove(topic);
                // Monitors: and the MONITOR ROW, by the same rule the
                // two lines around it follow — the rollback is the inverse of what
                // the attach did, and a sampling pass can land inside the window
                // this is undoing. `compose_layout` holds the state lock only for
                // each individual attach, so between the tap opening and a later
                // step failing (a peek-bounded resolve, the compile, the
                // validation, the worker send) the drain loop is free to run
                // `sample_monitors` and OPEN a row for a topic this call is about
                // to take away.
                //
                // Leaving it is not a stale-but-harmless entry, it is a PERMANENT
                // one: the only other things that release a row are the `detach`
                // verb and `retarget_origin_robot`, and both need a TAP — which
                // this loop has just taken away, and which no `detach` can be
                // issued against. So `monitors` would serve a row nothing can update
                // (UNKNOWN with a forever-growing age), a later re-attach of the
                // same topic would inherit its learned baseline and settle state
                // rather than starting the fresh observation the new tap is, and a
                // compose loop over robot-announced names would grow the row table
                // without bound — the same three costs the wake-request line below
                // is here for.
                //
                // Keyed off the record being dropped, exactly as `detach` does:
                // rows are `(topic, robot)`, so forgetting the wrong key would
                // leave the real row behind.
                st.monitors
                    .forget(removed.and_then(|s| s.origin_robot).as_deref(), topic);
                // And the WAKE REQUEST. The attach
                // this is rolling back inserted one, so leaving it here breaks
                // the invariant `wake_requested`'s own doc states — "the set
                // equals the tap set", "cleared by `detach` alongside the tap" —
                // and the entry can never be cleared again (the `detach` verb is
                // the only other remover and there is no tap left to detach).
                // The set is keyed by TOPIC NAME, and a compose can name a topic
                // a remote robot announced, so a rollback loop is unbounded
                // growth on a long-lived daemon. Same rule as the two lines
                // above: the rollback is the inverse of what the attach did.
                st.wake_requested.remove(topic);
            }
        }
        for topic in opened {
            if let Err(e) = perform_remote_release(self.demand_plane.as_ref(), &self.state, topic) {
                tracing::warn!(
                    topic = %topic, error = %e,
                    "cerulion-vizd: compose rollback could not release the netd demand — the \
                     shared mirror lingers until this daemon's connection to cerulion-netd drops"
                );
            }
        }
        self.refresh_attached_snapshot();
        tracing::warn!(
            rolled_back = opened.len(),
            "compose_layout: a failure after attaching rolled back the tap(s) this call opened"
        );
    }

    /// The entity set the compose compiler GROUNDED the
    /// plan over — the currently-attached render infos UNION the PLACED topics (which
    /// under attach_missing:false may be deliberately un-attached, so absent from
    /// `attached_render_infos`). Deduped by topic (an already-attached placed topic
    /// appears once). This is what the reused guardrail validates against, so it
    /// never falsely refuses a plan the compiler grounded by construction.
    fn compose_validation_set(&self, groups: &[ComposeGroup]) -> Vec<AttachedRender> {
        let mut set = self.attached_render_infos();
        let mut seen: std::collections::HashSet<String> =
            set.iter().map(|a| a.topic.clone()).collect();
        for group in groups {
            for placed in &group.topics {
                if seen.insert(placed.topic.clone()) {
                    set.push(placed.clone());
                }
            }
        }
        set
    }

    /// Attach-if-missing + resolve ONE topic for `compose_layout`.
    /// Returns the resolved [`AttachedRender`] (topic + render entity + archetype)
    /// and whether this call NEWLY attached it. A topic already tapped (or attached
    /// here) resolves its archetype (cached or a bounded peek); a topic left
    /// un-attached (`attach_missing = false`) keeps its derived entity but an
    /// unresolved archetype (it can still be placed by an explicit role).
    fn compose_resolve_topic(
        &self,
        topic: &str,
        attach_missing: bool,
        peek_deadline: Instant,
    ) -> Result<(AttachedRender, bool), String> {
        let mut attached = self
            .state
            .lock()
            .unwrap()
            .taps
            .list()
            .iter()
            .any(|t| t.as_str() == topic);
        let mut newly = false;
        if !attached && attach_missing {
            newly = self.attach_for_compose(topic)?;
            attached = true;
        }
        // The render entity: the stored route key (if attached) or the derived one
        // — exactly as `list`/`attached_render_infos` derive it.
        let route = {
            let st = self.state.lock().unwrap();
            st.stats.get(topic).and_then(|s| s.route_key.clone())
        }
        .unwrap_or_else(|| route_key_for_topic(topic, None));
        // Resolve the archetype only for an attached topic (a bounded peek caches
        // into its stats entry — never orphan a stats entry for a topic the agent
        // chose NOT to attach). Resolved BEFORE the entity, because the entity a
        // tf-NAMED topic reports depends on whether its payload is a TFMessage.
        let archetype = if attached {
            self.resolve_for_attach_bounded(topic, peek_deadline)
                .map(|r| r.archetype)
        } else {
            None
        };
        let entity = reported_entity_for(&route, archetype);
        // Probe the topic's LIVE producer count so the compiler can PREFER
        // live topics. The probe queries iceoryx2's global publisher count for the
        // `{topic}/data` service (covers all processes, not just our tap), so it is
        // meaningful whether or not WE attached — `Some(0)` = a registered-but-dead
        // route (the live `/uslam/cloud_map` case), `Some(n>0)` = live, `None` =
        // UNKNOWN (probe failed, treated as live-equivalent — never penalized). A
        // cheap no-wait probe (publisher PRESENCE, not a rate).
        let producer_count = self.manager.topic_publisher_count_checked(topic);
        let video_renditions = self.video_renditions_for(&route);
        // The live "provably rendering" signal the dump companion
        // is gated on. Read here for the same reason as the rendition set — the
        // compiler must place the views the RENDER will fill.
        let render_proof = self.render_proof_for(&route);
        // The compiler buckets by the views this TOPIC needs, so it reads
        // the operator's choice exactly as the reflow does.
        let representation = {
            let st = self.state.lock().unwrap();
            st.representation.get(topic).copied().unwrap_or_default()
        };
        Ok((
            AttachedRender {
                topic: topic.to_string(),
                entity,
                archetype,
                producer_count,
                video_renditions,
                representation,
                render_proof,
            },
            newly,
        ))
    }

    /// This route's H.264 rendition segments, read off the worker's
    /// mirror ([`VizWorkerCounters::video_renditions`]).
    ///
    /// Empty for every non-video topic, and for a video topic whose first
    /// parameter set has not arrived yet — in both cases the layout keeps its
    /// single topic-rooted view, which is the correct shape for them.
    ///
    /// The demux that knows this lives on the WORKER thread and the layout is
    /// assembled here on the control thread, so the mirror is the only path
    /// between them. A poisoned lock degrades to "no renditions" (a single view)
    /// rather than propagating — a sub-optimal layout, never a failed control op.
    /// The reported `(components, view_kinds)` for a topic this
    /// daemon HAS a route for, with both live signals folded in.
    ///
    /// The pair exists so the four ATTACHED-topic reporters (both attach replies,
    /// `list`, `status`) cannot drift from the applied blueprint by forgetting a
    /// signal, the exact drift a signal-blind `layout_metadata` shows,
    /// reporting `text_document` for a decoding video topic whose blueprint
    /// has already dropped it. `discover` reaches it through
    /// [`Ctx::layout_metadata_for_discover_row`], which threads the same signals
    /// for a row that IS attached and defaults for one that is not.
    fn layout_metadata_for_route(
        &self,
        archetype: Option<ArchetypeKind>,
        representation: Representation,
        route_key: &str,
    ) -> (Option<Vec<String>>, Option<Vec<String>>) {
        layout_metadata(
            archetype,
            representation,
            self.video_renditions_for(route_key),
            self.render_proof_for(route_key),
        )
    }

    /// The reported `(components, view_kinds)` for a `discover`
    /// row — live signals when this desk is actually rendering the topic, the
    /// default when it is not.
    ///
    /// **A blanket default rests on a premise that is false.** The
    /// premise reads "a `discover` row is a topic this desk has NOT rendered, so
    /// neither live signal exists for it" — but [`Ctx::discover`] enumerates
    /// `list_topics()` with NO attach filter, so every ATTACHED local topic is a
    /// discover row too. The same function proves it: it already reads tap
    /// liveness and the undecodable verdict, both of which exist only for a
    /// topic this daemon taps. So on a desk with a rendering `/plan` or `/markers`,
    /// `discover` would advertise a `text_document` pane the installed blueprint does not
    /// contain while `status`/`list`/both attach replies say it does — one run, two
    /// answers, on the surface the Studio sidebar polls. That is the one-run-two-answers class,
    /// re-created by the very fix that removed it from the other four surfaces.
    ///
    /// The default is KEPT for a row nothing has rendered, because there it is the
    /// correct answer rather than a missing lookup — and it is gated on a LIVE tap,
    /// so a topic detached long ago reports the default rather than the proof its
    /// previous attach earned (proof entries outlive the attach — see
    /// [`cerulion_viz::sink::RenderProof`]).
    ///
    /// **That gate is REDUNDANT, stated rather than implied**: `detach` also
    /// does `st.stats.remove(&topic)`, so the route lookup would come back `None`
    /// for a detached topic on its own — MEASURED: dropping
    /// `taps.contains` survives the whole suite, and
    /// `a_detached_topic_reports_the_default_placement_again_on_discover_e2e`
    /// therefore pins the OUTCOME rather than either mechanism.
    /// It is kept because the claim this function makes is "live
    /// signals only for a topic this desk is CURRENTLY rendering", and stating that
    /// in code is worth one map lookup on a cold path — deriving it instead from
    /// another verb's cleanup order is an inference across modules that a future
    /// change to `detach` would silently invert.
    ///
    /// **Both inputs arrive as ONE snapshot** ([`Ctx::discover_row_state`]) rather
    /// than being looked up here — see that function for why re-reading them
    /// separately is the drift that snapshot prevents. This function takes
    /// no lock at all.
    fn layout_metadata_for_discover_row(
        &self,
        archetype: Option<ArchetypeKind>,
        representation: Representation,
        route: Option<&str>,
    ) -> (Option<Vec<String>>, Option<Vec<String>>) {
        match route {
            Some(route) => self.layout_metadata_for_route(archetype, representation, route),
            None => layout_metadata(
                archetype,
                representation,
                Vec::new(),
                RenderProof::default(),
            ),
        }
    }

    /// Everything a `discover` row reads out
    /// of `state`, captured under ONE lock acquisition.
    ///
    /// `list` and `status` capture a row's representation AND its route inside a
    /// single `st` guard. A `discover` row that took the lock THREE times —
    /// once for the representation, once for the route, once for the
    /// undecodable verdict — would let a `set_representation` or a `detach` interleaved
    /// between them pair a representation from one snapshot with a route from
    /// another. The visible failures are both in the "one run, two answers"
    /// class:
    ///
    /// * `set_representation` re-applies the default layout from the NEW choice, so
    ///   a row could advertise a placement derived from the OLD one while the
    ///   blueprint the viewer already has was built from the new one;
    /// * worse, the row's own `representation` FIELD would be a THIRD read, so it could
    ///   disagree with the `view_kinds` printed beside it, and the whole
    ///   reason for carrying that field is that the view kinds already encode it.
    ///
    /// The window is small and neither surface is load-bearing for control flow, so
    /// this is a consistency guarantee, not a crash guard — but "small" is exactly the
    /// argument that lets `discover` and `status` disagree.
    ///
    /// Taken and RELEASED before the live signals are read: `layout_metadata_for_route`
    /// locks the worker's counter mirrors, and those are a different subsystem the
    /// state lock cannot freeze anything relative to anyway (`list`/`status` nest
    /// the two; this path does not need to, so it does not).
    fn discover_row_state(&self, topic: &str) -> DiscoverRowState {
        let st = self.state.lock().unwrap();
        DiscoverRowState {
            representation: st.representation.get(topic).copied().unwrap_or_default(),
            // The tap's OWN route key, exactly as `list`/`status` resolve it — that
            // is what the worker's mirrors are keyed by.
            route: st.taps.contains(topic).then(|| {
                st.stats
                    .get(topic)
                    .and_then(|s| s.route_key.clone())
                    .unwrap_or_else(|| route_key_for_topic(topic, None))
            }),
            // Only ever READ here — `resolve_for_discover` records nothing
            // (an un-tapped topic must leave no stats entry), so this carries the
            // verdict for a topic this daemon already taps and is absent for one it
            // does not.
            undecodable: st.stats.get(topic).and_then(|s| s.undecodable.clone()),
        }
    }

    /// The worker's LAYOUT-SIGNAL generation — one relaxed load
    /// that answers "has either live layout signal changed since I last looked?".
    ///
    /// The drain loop compares it pass to pass to decide whether the default layout
    /// must reflow. It replaced a whole-map clone-and-compare of the render proofs,
    /// which was wrong in TWO ways at once: it ran on every pass of the loop
    /// the wake rework shrank to tens of microseconds, and it watched only ONE of the two
    /// signals `blueprint::render_is_proven` reads — see
    /// [`cerulion_viz::worker::VizWorkerCounters::layout_signal_generation`] for
    /// why the missing half is exactly the flagship camera path.
    fn layout_signal_generation(&self) -> u64 {
        self.counters
            .layout_signal_generation
            .load(Ordering::Relaxed)
    }

    /// This route's render proof, read off the worker's mirror
    /// ([`cerulion_viz::worker::VizWorkerCounters::render_proofs`]).
    ///
    /// The DEFAULT (nothing observed) for a topic no frame of which has reached a
    /// render arm yet — which is also what a poisoned lock degrades to, and in both
    /// cases it is the right answer: the companion is KEPT.
    /// A refusal must rest on evidence we actually have.
    fn render_proof_for(&self, route_key: &str) -> RenderProof {
        self.counters
            .render_proofs
            .lock()
            .ok()
            .and_then(|m| m.get(route_key).copied())
            .unwrap_or_default()
    }

    fn video_renditions_for(&self, route_key: &str) -> Vec<String> {
        self.counters
            .video_renditions
            .lock()
            .ok()
            .and_then(|m| m.get(route_key).cloned())
            .unwrap_or_default()
    }

    /// Attach a topic for `compose_layout` (no entity override) under
    /// the state lock, mirroring [`Self::attach`]'s tap + route-commit. Returns
    /// whether the tap was NEWLY opened (`false` = already attached, idempotent).
    ///
    /// **This path drives a LOCAL tap directly, which a REMOTE topic cannot
    /// use.** `compose_layout` does not go through [`Self::attach`]'s router,
    /// so a REMOTE topic would reach the tap manager with no demand behind it and die
    /// `DoesNotExist` — an error naming the record/replay tap, which is the
    /// tell that no remote arm was ever consulted. An agent could not bring back
    /// a `/lowstate` it had just detached, from a surface where the sidebar succeeds.
    ///
    /// So it routes through the ONE resolution the `attach` verb uses, in BOTH
    /// directions: BEFORE the local open when the topic is already known not to be
    /// locally visible, and AFTER a local open that failed because it stopped being
    /// visible in between. Two surfaces resolving the same question independently is
    /// how they come to disagree; attach routing follows the same rule as attribution.
    fn attach_for_compose(&self, topic: &str) -> Result<bool, String> {
        // Not locally visible + a network → the demand plane owns this topic. Checked
        // BEFORE taking the lock, since the remote path takes it itself.
        if self.should_resolve_remote(topic) {
            return self.attach_for_compose_via_demand_plane(topic);
        }
        let mut st = self.state.lock().unwrap();
        // This is the LOCAL arm of compose (the remote arm above
        // routes through `attach_remote`), and it takes the same
        // `WakeMode::Listener` as `attach_local` for the same reason — a topic
        // composed into a layout is a topic a human is watching, and the
        // decision is that such a topic is drained within a wake whichever seam
        // attached it. A compose-attached row rendering 16 ms behind an
        // `attach`-ed one would be the same defect with a different door.
        let outcome = match st
            .taps
            .attach(&self.manager, topic, None, WakeMode::Listener)
        {
            Ok(o) => o,
            Err(e) => {
                // The same post-failure fallback `attach_local` performs —
                // the mirror was torn down between the check above and this open.
                drop(st);
                if self.should_resolve_remote(topic) {
                    tracing::info!(
                        topic = %topic,
                        "cerulion-vizd: compose local attach found no service — \
                         resolving '{topic}' through the demand plane"
                    );
                    return self.attach_for_compose_via_demand_plane(topic);
                }
                return Err(e.to_string());
            }
        };
        let already = matches!(outcome, AttachOutcome::AlreadyAttached { .. });
        // Compose's local arm asks for a wake like every other
        // seam, so its rows carry the same latency verdict on `status`.
        st.wake_requested.insert(topic.to_string());
        // The tap is up — start (or continue) its data-flow observation.
        st.stats
            .entry(topic.to_string())
            .or_default()
            .begin_observation(Instant::now());
        let route_key = route_key_for_topic(topic, None);
        match decide_route_commit(
            topic,
            st.stats.get(topic).and_then(|s| s.route_key.as_deref()),
            &route_key,
            already,
        ) {
            RouteCommit::Refuse(msg) => Err(msg),
            RouteCommit::Store => {
                st.stats.entry(topic.to_string()).or_default().route_key = Some(route_key);
                Ok(!already)
            }
            RouteCommit::Keep => Ok(!already),
        }
    }

    /// Drive the ONE remote attach resolution and translate its verb-shaped
    /// answer back into [`Self::attach_for_compose`]'s `Result<newly, message>`.
    ///
    /// An adapter rather than a second implementation, deliberately. The alternative
    /// — teaching compose its own remote path — is exactly the duplication that
    /// loses the routing: a `compose_layout` with its own attach misses
    /// the remote-provenance routing entirely. Every remote-aware error message
    /// (`attach_resolving_remote` names the robot, the candidates, or netd) reaches
    /// the agent unchanged, instead of a local "does not exist".
    ///
    /// The correlation id is irrelevant here — this response is consumed in-process
    /// and never written to the wire — so a fixed 0 is passed and only `ok`,
    /// `already_attached` and `error` are read.
    fn attach_for_compose_via_demand_plane(&self, topic: &str) -> Result<bool, String> {
        match self.attach_resolving_remote(0, topic.to_string(), None) {
            Response::Attach(r) if r.ok => Ok(!r.already_attached),
            Response::Error(e) => Err(e.error),
            // Neither arm is reachable — `attach_resolving_remote` answers only
            // `Attach`/`Error`, and it never builds an `Attach` with `ok: false`.
            // Reported as an error rather than unwrapped, so a future variant surfaces
            // as a message instead of a panic in the layout verb.
            _ => Err(format!(
                "internal: remote attach resolution for '{topic}' answered neither an \
                 attach nor an error — this is a cerulion-vizd bug, please report it"
            )),
        }
    }

    /// The render placements (topic + entity + resolved archetype) of the
    /// currently-attached taps — the input to the `set_blueprint` guardrails AND the
    /// soft origin-grounding hint. The `entity` is derived exactly as
    /// [`Self::list`] derives its `entity` field, so the guardrails compare against
    /// the SAME paths the frames render under; the `archetype` is the once-resolved
    /// render family (`None` for a still-silent topic — never guardrail-checked).
    fn attached_render_infos(&self) -> Vec<AttachedRender> {
        let infos = self.attached_render_infos_inner();
        // REMEMBER every entity the moment it is
        // materialized, not at plan-derivation time. Accumulating
        // inside `consolidated_default_plan` is not enough: `maybe_apply_default_layout`
        // returns before reaching it under `LayoutMode::Explicit` — the NORMAL Studio
        // session, since `compose_layout`/`set_blueprint(spec)` flip the mode. A
        // topic attached AND detached inside that window would never be recorded, so a
        // later reset would have nothing to exclude and the detach zombie would reopen
        // on exactly the path the reset exclusion closes.
        self.note_logged_entities(infos.iter().map(|t| t.entity.clone()));
        infos
    }

    /// The unrecorded half of [`Ctx::attached_render_infos`] — holds the state lock,
    /// so the (separate) `logged_entities` lock is only ever taken AFTER it drops.
    fn attached_render_infos_inner(&self) -> Vec<AttachedRender> {
        let st = self.state.lock().unwrap();
        st.taps
            .list()
            .into_iter()
            .map(|topic| {
                let stat = st.stats.get(&topic);
                let route = stat
                    .and_then(|s| s.route_key.clone())
                    .unwrap_or_else(|| route_key_for_topic(&topic, None));
                let archetype = stat.and_then(|s| s.resolved.as_ref().map(|r| r.archetype));
                let entity = reported_entity_for(&route, archetype);
                // Liveness is `None` here — this feeds the `set_blueprint`
                // guardrails + `default_layout` + the compose VALIDATION set, none of
                // which read `producer_count` (only the compose COMPILER prefers-live,
                // and its inputs come from `compose_resolve_topic`, which DOES probe).
                // Probing on every attach/detach reflow would add a per-topic iceoryx2
                // open to that hot path for a field its consumers ignore.
                let video_renditions = self.video_renditions_for(&route);
                // And the same live rendering signal — this is THE path
                // the applied default layout is built from, so a companion refused
                // (or restored) anywhere is refused here.
                let render_proof = self.render_proof_for(&route);
                // The layout must place the views the RENDER will fill,
                // so it reads the same choice the worker was handed.
                let representation = st.representation.get(&topic).copied().unwrap_or_default();
                AttachedRender {
                    topic,
                    entity,
                    archetype,
                    producer_count: None,
                    video_renditions,
                    representation,
                    render_proof,
                }
            })
            .collect()
    }

    /// Refresh the process-static attached-entities snapshot the
    /// blueprint reapply sites warn against (called after every attach/detach — a
    /// low-frequency control op, never the poll hot path). Keeps the reconnect-
    /// reapply grounding warn current: if every topic detaches, a later reconnect
    /// re-applying the stored layout warns it now grounds nothing.
    fn refresh_attached_snapshot(&self) {
        let entities = self
            .attached_render_infos()
            .into_iter()
            .map(|a| a.entity)
            .collect();
        set_attached_entities_snapshot(entities);
    }

    /// Re-derive + apply the consolidated default layout ([`default_layout`]) from
    /// the currently-attached topics — the "one plot per topic in a grid,
    /// Scene primary, never tabs" default. A NO-OP under [`LayoutMode::Explicit`] (an
    /// explicit `set_blueprint`/`compose_layout` layout WINS — attach/detach must not
    /// clobber it). BEST-EFFORT: a worker-send failure is logged, never fails the
    /// attach/detach the caller is answering (the layout is cosmetic; the tap already
    /// succeeded). Called on every attach/detach (AND on a poll-thread late-resolve)
    /// AFTER the archetype is resolved + the attached snapshot refreshed, so
    /// the derived layout sees the just-changed topic's view kinds. Grounded by
    /// construction, so it bypasses the hard `set_blueprint` guardrails (applied straight
    /// to the worker, which remembers it for reconnect).
    ///
    /// **No TOCTOU:** the mode read AND the worker send happen under the ONE
    /// [`Ctx::layout_lock`] the explicit-apply path also holds, so a concurrent explicit
    /// apply cannot flip the mode + enqueue its plan BETWEEN this read and this send
    /// (which would leave the viewer on the default with mode Explicit — stuck). Whoever
    /// takes the lock last wins BOTH the FIFO send and the mode.
    ///
    /// This does N O(n) full-blueprint sends on rapid sequential attaches
    /// (a known cost): each attach re-derives the whole grid + re-sends it.
    /// The bounded worker queue coalesces nothing (each is a distinct blueprint), but a
    /// blueprint is small and control ops are low-frequency, so the O(n) re-send is
    /// deliberately NOT debounced — a debounce would add a timer thread + a settle delay
    /// (the layout would lag the attach) for a cosmetic flicker that the reflow's whole
    /// point is to make instant. Left as documented behavior.
    /// The consolidated DEFAULT layout, derived from the currently-attached
    /// set MINUS every entity this run has ever logged.
    ///
    /// **Both** producers of the default plan must go through here — the
    /// attach/detach recompose ([`Ctx::maybe_apply_default_layout`]) AND
    /// `set_blueprint`'s RESET arm. A reset arm calling plain `default_layout`,
    /// which knows nothing about detached entities, would
    /// re-open the zombie the exclusion exists to close: a topic detached
    /// earlier reappears inside a still-attached ancestor's view. `default_layout`
    /// is deliberately NOT imported in this module, so that bypass cannot
    /// be introduced by reaching for the shorter name.
    fn consolidated_default_plan(&self) -> BlueprintPlan {
        // `attached_render_infos` has already folded the current set in.
        let attached = self.attached_render_infos();
        default_layout_excluding(&attached, &self.logged_entities_snapshot())
    }

    /// Record entities into the run's EVER-LOGGED set.
    ///
    /// Called from [`Ctx::attached_render_infos`], which every attach, detach,
    /// compose and reset passes through REGARDLESS of [`LayoutMode`] — so an
    /// explicit layout cannot starve the exclusion set.
    fn note_logged_entities(&self, entities: impl IntoIterator<Item = String>) {
        let mut ever = self
            .logged_entities
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        record_logged_entities(&mut ever, entities);
    }

    /// The ever-logged set as a sorted snapshot.
    fn logged_entities_snapshot(&self) -> Vec<String> {
        snapshot_logged_entities(
            &self
                .logged_entities
                .lock()
                .unwrap_or_else(|e| e.into_inner()),
        )
    }

    /// Re-derive and install the consolidated default layout, unless an explicit
    /// layout verb has taken over.
    ///
    /// Returns whether the plan was actually RE-DERIVED — false under
    /// [`LayoutMode::Explicit`], where this is a no-op. The
    /// drain loop's `poll_layout_signal_reflows` counter is gated on this, so the
    /// Principle #3 observable means "the layout was re-derived for that signal"
    /// rather than "the loop called a function that may have returned at its first
    /// line". A FAILED send still counts as re-derived and says so loudly below:
    /// the derivation ran, only the install did not, and conflating the two would
    /// make the counter silent on exactly the failure the warn exists for.
    fn maybe_apply_default_layout(&self) -> bool {
        // Hold `layout_lock` across the mode read + the (bounded) worker send, so the
        // two are atomic. `attached_render_infos()` re-locks `state` on its own INSIDE this
        // lock (no inversion — nothing acquires `layout_lock` while holding `state`).
        let guard = self.layout_lock.lock().unwrap_or_else(|e| e.into_inner());
        if *guard != LayoutMode::Auto {
            return false;
        }
        let plan = self.consolidated_default_plan();
        if let Err(e) = self.worker_control.set_blueprint(plan) {
            tracing::warn!(
                error = %e,
                "cerulion-vizd: could not apply the default layout to the viz worker — the viewer \
                 keeps its current layout (the attach/detach itself succeeded)"
            );
        }
        true
    }

    /// Send an EXPLICIT/RESET layout AND record the resulting
    /// [`LayoutMode`] ATOMICALLY under [`Ctx::layout_lock`] — the mode flips ONLY on a
    /// successful send (a `Busy`/`WorkerGone` failure leaves the mode unchanged,
    /// never Auto-with-a-stale-explicit-layout), and no concurrent
    /// [`Ctx::maybe_apply_default_layout`] can interleave between the send and the flip
    /// (fix 1). Returns the worker-send result verbatim for the caller to map.
    ///   - `set_blueprint`(spec) / `compose_layout` → `LayoutMode::Explicit` (attach/detach
    ///     stops clobbering it);
    ///   - `set_blueprint` RESET → `LayoutMode::Auto` (the dynamic default resumes).
    fn apply_layout_atomic(
        &self,
        plan: BlueprintPlan,
        new_mode: LayoutMode,
    ) -> Result<(), cerulion_viz::worker::VizControlError> {
        let mut guard = self.layout_lock.lock().unwrap_or_else(|e| e.into_inner());
        let result = self.worker_control.set_blueprint(plan);
        if result.is_ok() {
            *guard = new_mode;
        }
        result
    }

    /// `representation`: choose how one topic RENDERS.
    ///
    /// Remembered per topic for the daemon's life (see
    /// [`DaemonState::representation`]) and pushed to the worker straight away
    /// when the topic is attached, so the operator's click lands on the stream
    /// they are already watching rather than on the next one.
    ///
    /// An unrecognized value is REFUSED naming the accepted set. Reading it as
    /// `auto` would answer `ok` for a click that changed nothing — the silent
    /// no-op this whole affordance exists to replace.
    fn set_representation(&self, id: u64, topic: String, representation: String) -> Response {
        let Some(choice) = Representation::from_wire(&representation) else {
            let accepted: Vec<&str> = Representation::ALL.iter().map(|r| r.as_wire()).collect();
            return Response::error(
                Some(id),
                format!(
                    "unknown representation '{representation}' — expected one of {}",
                    accepted.join(" / ")
                ),
                Some(topic),
            );
        };
        // Learn whether the topic is attached — the route key IS the sink's
        // per-input key — WITHOUT committing anything yet.
        let route_key = {
            let st = self.state.lock().unwrap();
            st.stats.get(&topic).and_then(|s| s.route_key.clone())
        };
        // PUSH FIRST, COMMIT ON SUCCESS. The map is what every
        // derived surface reads — `list`, `status`, `layout_metadata`,
        // `attached_render_infos_inner` — so committing before a push that can
        // fail leaves an `ok:false` response coexisting with a `list` affirming the
        // refused value, and the next reflow PLACING a `text_document` view the
        // sink was never told to fill. That is the affirmatively-wrong claim
        // `layout_metadata`'s own comment exists to prevent, and it is worse than
        // the transient it looks like: Studio's corrective `list` reads THIS map,
        // so the designated corrector would re-manufacture the lie every poll.
        //
        // Ordering rather than rollback: `apply_layout_atomic` (just above `set_representation` in this file)
        // commits on success under its lock, but this push can block for
        // `CONTROL_SEND_TIMEOUT`, and holding the state mutex that long would stall
        // the poll thread. Pushing first has no failure window at all — on `Err`
        // the map was never touched — and on success the two converge within
        // microseconds.
        if let Some(route_key) = &route_key {
            if let Err(e) = self.worker_control.set_representation(route_key, choice) {
                return Response::error(Some(id), e.to_string(), Some(topic));
            }
        }
        {
            let mut st = self.state.lock().unwrap();
            if choice.is_auto() {
                st.representation.remove(&topic);
            } else {
                st.representation.insert(topic.clone(), choice);
            }
        }
        // The views a topic needs depend on the choice, so the layout is re-derived
        // exactly as it is on attach/detach (a no-op under an explicit layout verb).
        //
        // Only when the topic is ATTACHED. `consolidated_default_plan`
        // derives from `attached_render_infos()`, which enumerates attached taps
        // only, so on the unattached arm the plan CANNOT differ — and every
        // `set_blueprint` appends a fresh blueprint store to the proxy's retained
        // history, replayed to every new viewer, which is a fast unbounded-growth
        // path in the viewer proxy.
        if route_key.is_some() {
            self.maybe_apply_default_layout();
        }
        Response::Representation(RepresentationResponse {
            id: Some(id),
            ok: true,
            topic,
            representation: choice.as_wire().to_string(),
            applied: route_key.is_some(),
        })
    }

    /// This topic's remembered representation ([`Representation::Auto`]
    /// unless the operator chose otherwise).
    fn representation_for(&self, topic: &str) -> Representation {
        let st = self.state.lock().unwrap();
        st.representation.get(topic).copied().unwrap_or_default()
    }

    /// Hand the worker this topic's remembered representation, now that
    /// `route_key` names it to the sink.
    ///
    /// Called on every successful attach: the choice outlives the attachment, so a
    /// re-checked row renders the way the operator left it. A control-channel
    /// failure is logged rather than failing the attach — the tap is up and the
    /// data is flowing, and refusing the attach would cost the operator the topic
    /// as well as the preference.
    ///
    /// **UNCONDITIONAL, and that is the whole point.** "Cleared"
    /// is encoded here as an ABSENCE in the map, so a reconciler that returned
    /// early on absence could SET but never CLEAR: the sink's map is keyed by ROUTE
    /// KEY and lives for the daemon's life, `detach` erases the `stats` entry the
    /// verb needs to reach it, and nothing else ever pushes. A topic set to `text`,
    /// detached, returned to `auto` and re-attached would render under the stale
    /// override FOREVER — the visual half suppressed entirely — while `list`,
    /// `status`, the layout and the chip all reported `auto`. Pushing
    /// [`Representation::Auto`] IS the clear (`SinkState::set_representation`
    /// removes both the entry and its `ForcedDumpGate`), and on a topic nobody
    /// overrode it is a no-op, so the unconditional push costs one control message
    /// per attach and closes the direction absence could not express.
    fn apply_remembered_representation(&self, topic: &str, route_key: &str) {
        let choice = {
            let st = self.state.lock().unwrap();
            st.representation.get(topic).copied().unwrap_or_default()
        };
        if let Err(e) = self.worker_control.set_representation(route_key, choice) {
            tracing::warn!(
                topic = %topic, error = %e,
                representation = choice.as_wire(),
                "cerulion-vizd: could not hand the worker this topic's representation — the RENDER keeps whatever it had for this route, which after a CLEAR means the old override is still in force; set the representation again once the viewer recovers"
            );
        }
    }

    /// `attach`: open the tap (idempotent), record its route, best-effort resolve
    /// its schema/archetype, and answer with the resolved placement — OR the
    /// tap layer's error surfaced VERBATIM.
    /// The `attach` VERB entry (no `robot`). Routes between the genuine-local tap and
    /// the remote-provenance recovery:
    ///
    /// - a topic that IS locally visible (a genuine local producer OR a live mirror)
    ///   attaches LOCALLY, unchanged;
    /// - a topic that is NOT locally visible on a NETWORK-configured daemon is resolved
    ///   to its origin robot (a session tombstone, else the netd catalog) and re-demanded
    ///   as a REMOTE attach — the "check a box → it appears" bar surviving an uncheck →
    ///   re-check round-trip, instead of the nonsensical local "does not exist";
    /// - a NETWORK-less daemon keeps the plain local behavior (a missing topic errors
    ///   locally — there is no remote to resolve).
    fn attach(&self, id: u64, topic: String, entity_override: Option<String>) -> Response {
        // A locally-visible topic (genuine producer OR live mirror) → local tap. Only a
        // NOT-locally-visible topic on a network-configured daemon takes the
        // remote-resolution path (which never touches the network for a visible topic).
        if self.should_resolve_remote(&topic) {
            return self.attach_resolving_remote(id, topic, entity_override);
        }
        self.attach_local(id, topic, entity_override)
    }

    /// Should this topic be resolved through the DEMAND PLANE rather
    /// than tapped locally? True iff this daemon has a network AND the topic's local
    /// data service is not there.
    ///
    /// Read TWICE per attach, and the second read is load-bearing. As a ROUTING
    /// predicate it answers "is the topic locally visible"; re-asked AFTER a local
    /// attach FAILS, it answers "was it locally OPENABLE", and
    /// those two can disagree across a few milliseconds.
    ///
    /// Detach releases the netd demand, netd's refcount hits 0, and the mirror's local
    /// SHM service is torn down — so a prompt re-attach can read the DYING mirror as
    /// still visible here, route local, and then fail `DoesNotExist` at open time. An
    /// agent that hits it on `/lowstate` retries discovery for its whole step
    /// budget and gives up; the same topic attaches fine once the mirror is long gone,
    /// because by then this predicate is true on the FIRST read. A window, not a
    /// broken route.
    ///
    /// Re-asking is a STRUCTURAL classification, not a parse of the error text: at
    /// that point the service is provably absent, and "not locally openable" deserves
    /// exactly the resolution "not locally visible" gets. It also stays correctly
    /// SILENT for every other local failure — an exhausted introspection slot leaves
    /// the service present, so that error still surfaces verbatim instead of being
    /// laundered into a remote attach that would fail differently.
    fn should_resolve_remote(&self, topic: &str) -> bool {
        self.manager.network().is_some() && self.manager.data_service_missing(topic)
    }

    /// An `attach` with NO `robot` for a topic that is NOT locally visible on a
    /// network-configured daemon. Resolve WHERE the topic lives — a session provenance
    /// TOMBSTONE ([`DaemonState::remote_provenance`]) first (the topic was attached from
    /// a specific robot before its mirror was torn down), else the netd CATALOG
    /// ([`DemandPlane::query_catalog_all_with_discovery`]) — and re-demand it as a REMOTE
    /// attach. Every failure surface is REMOTE-aware (names the robot / the candidates /
    /// netd), NEVER the local graph-YAML "does not exist" for a topic known to be remote.
    ///
    /// The gather's [`DiscoveryState`] rides along, because two of the
    /// resolution's arms make an ABSENCE CLAIM with it ("no reachable robot serves it",
    /// "that robot no longer announces it"). An empty/partial answer from a netd that has
    /// not completed a discovery pass proves NOTHING, so it may not license either claim —
    /// the same gate `fold_streaming_mirrors_into_robots` applies to the leftover-mirror
    /// drop, at the call site the user actually asked a question from.
    fn attach_resolving_remote(
        &self,
        id: u64,
        topic: String,
        entity_override: Option<String>,
    ) -> Response {
        let remembered = self
            .state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remote_provenance
            .get(&topic)
            .cloned();
        // Gather the LAN catalog to learn which robots CURRENTLY serve the topic. A
        // netd-unreachable gather is NOT fatal: fall back to the tombstone if we have
        // one (attach_remote surfaces its own remote-aware error if the robot is truly
        // gone), else a remote-aware "not local + catalog unreachable" error.
        //
        // The DISCOVERY STATE the answer was given under travels with it — see
        // this method's docs. An `Err` is a different condition entirely (netd could not
        // RUN the query), and keeps its own arm below.
        //
        // ONE round trip. The gather also rides out as a [`RetryHint`] on the
        // failure arms below, so a client can keep asking on its own schedule without
        // this handler holding a connection (or netd's one client mutex) open.
        let gather = match self.gather_catalog(None) {
            Ok(gather) => gather,
            Err(e) => match remembered {
                Some(robot) => {
                    tracing::info!(
                        topic = %topic, robot = %robot, error = %e,
                        "cerulion-vizd: re-attaching a remembered remote topic — the netd catalog \
                         gather is unreachable, trusting the session provenance tombstone"
                    );
                    return self.attach_remote(id, topic, entity_override, robot, None);
                }
                None => {
                    return Response::error(
                        Some(id),
                        format!(
                            "'{topic}' is not a local topic and the network catalog is unreachable \
                             ({e}) — is cerulion-netd running? attach '{topic}' from the ROBOTS \
                             list once its robot is discovered"
                        ),
                        Some(topic),
                    )
                }
            },
        };
        let serving = robots_serving_topic(&gather.catalogs, &topic);
        match resolve_remote_attach_target(
            &topic,
            remembered.as_deref(),
            &serving,
            gather.discovery,
        ) {
            RemoteAttachResolution::Robot(robot) => {
                if remembered.as_deref() == Some(robot.as_str()) {
                    tracing::info!(
                        topic = %topic, robot = %robot,
                        "cerulion-vizd: re-attaching '{topic}' from robot '{robot}' (remembered)"
                    );
                } else {
                    tracing::info!(
                        topic = %topic, robot = %robot,
                        "cerulion-vizd: auto-resolving '{topic}' to robot '{robot}' (the only \
                         robot serving it)"
                    );
                }
                self.attach_remote(id, topic, entity_override, robot, None)
            }
            // `retryable` is the SEAM's verdict — "no robot in this answer
            // serves THIS topic", which a later answer can change — as opposed to the
            // AMBIGUITY arm, where re-asking changes nothing because the user must pick.
            RemoteAttachResolution::Error { message, retryable } => Response::error_retryable(
                Some(id),
                message,
                Some(topic),
                retryable.then(|| retry_hint_for(&gather)),
            ),
        }
    }

    /// The genuine-LOCAL tap path (the local half of `attach`). Opens a tap on a
    /// locally-visible `topic` and commits its render route.
    fn attach_local(&self, id: u64, topic: String, entity_override: Option<String>) -> Response {
        let route_key = route_key_for_topic(&topic, entity_override.as_deref());

        // Open the tap + record the route atomically under the lock (fast — no
        // blocking peek here). Guardrail 5: the entity-override stability
        // decision (conflict refusal + the AlreadyAttached-never-overwrites-route_key
        // invariant) runs under the SAME lock as the tap, so a concurrent conflicting
        // override cannot slip through. A conflict is only possible on a topic already
        // known (route_key set ⟺ tap attached), so tap.attach returns AlreadyAttached
        // — no spurious tap is opened.
        let already = {
            let mut st = self.state.lock().unwrap();
            let outcome = match st.taps.attach(
                &self.manager,
                &topic,
                entity_override.as_deref(),
                // A GENUINE LOCAL
                // topic gets the wake too.
                //
                // The cost is real and was never in doubt — this producer IS
                // somebody else's graph publisher, so the listener un-arms its
                // notify elision for as long as the tap is held, and
                // that publisher's own in-graph consumers resume paying a
                // `sendto` per frame. The decision is that a human
                // watching a topic is worth it: without this, `cerulion viz`
                // ON A ROBOT (and every local-topic view) sat on the ~16 ms
                // poll floor while a remote view of the same data ran at
                // ~0.03 ms. One class of user paying 500x for being on the
                // wrong machine is not a defensible default.
                //
                // The bill is bounded and self-clearing: it is one listener per
                // VIEWED topic, it exists only while the tap is held, and the
                // producer's gate re-arms within one publish of the detach.
                //
                // The resweep's debt rule keeps "a `sendto` per frame" TRUE for this class. A
                // listener on a graph publisher's event service also arms
                // the live-loop boundary resweep. A resweep that fired a real notify
                // on every `live_step` pass the producer did NOT publish would set
                // the bill by the graph's LOOP rate (measured 198
                // notifies for 2 frames), and worse, that notify reaches the
                // topic's own in-graph consumer listener on the live WaitSet, so
                // the ROBOT's control loop would wake itself and free-run (measured
                // 53/s → 3425/s for ONE viewed topic). The resweep announces
                // an un-announced FRAME instead of observing a silent pass, so a
                // quiescent producer costs zero.
                WakeMode::Listener,
            ) {
                Ok(o) => o,
                // The local open FAILED. If the topic is not locally
                // openable at all and this daemon has a network, that is the same
                // condition the router checks BEFORE attaching — it just became true
                // in between (a mirror torn down by the detach this re-attach
                // follows). Resolve it through the demand plane rather than handing
                // the user a terminal "does not exist" for a topic a robot is
                // serving. The state lock is DROPPED first: `attach_resolving_remote`
                // takes it, and std mutexes are not reentrant.
                Err(e) => {
                    drop(st);
                    if self.should_resolve_remote(&topic) {
                        tracing::info!(
                            topic = %topic,
                            "cerulion-vizd: local attach found no service — resolving \
                             '{topic}' through the demand plane (a mirror torn \
                             down between the routing check and the open)"
                        );
                        return self.attach_resolving_remote(id, topic, entity_override);
                    }
                    // Any OTHER local failure (an exhausted slot, a bad name) is a
                    // real local error — surface it VERBATIM, unchanged.
                    return Response::error(Some(id), e.to_string(), Some(topic));
                }
            };
            let already = matches!(outcome, AttachOutcome::AlreadyAttached { .. });
            // A LOCAL topic asks for a wake too, so `status` can
            // tell a local topic running on the timer (listener refused, or
            // demoted) from one drained within a wake. Recorded under the SAME
            // lock as the tap, exactly as `attach_remote` does.
            st.wake_requested.insert(topic.clone());
            // The tap is up — start (or continue) its data-flow observation.
            st.stats
                .entry(topic.clone())
                .or_default()
                .begin_observation(Instant::now());
            match decide_route_commit(
                &topic,
                st.stats.get(&topic).and_then(|s| s.route_key.as_deref()),
                &route_key,
                already,
            ) {
                RouteCommit::Refuse(msg) => return Response::error(Some(id), msg, Some(topic)),
                // Fresh attach: record the route. AlreadyAttached (Keep): leave the
                // stored key untouched so the stat always reflects the tap's ACTUAL
                // key (never a diverging re-attach's).
                RouteCommit::Store => {
                    st.stats.entry(topic.clone()).or_default().route_key = Some(route_key.clone());
                }
                RouteCommit::Keep => {}
            }
            already
        };
        // Keep the reapply-warn snapshot current (a new tap changes the attached set).
        self.refresh_attached_snapshot();

        // Best-effort schema resolution OUTSIDE the lock (a bounded peek may wait
        // for a frame; the poll thread must not stall on it). The resolve CACHES the
        // archetype into stats, so the default-layout recompose below sees this
        // topic's view kinds.
        let resolved = self.resolve_for_attach(&topic);
        // Decision: make a checked-but-unmappable topic LOUD — a topic that
        // resolves to NO renderable archetype renders nothing in the Scene or a plot. The
        // attach reply already carries `schema`/`view_kinds` (both null here); this also
        // surfaces it in the daemon log so a silent no-render is never invisible.
        if resolved.is_none() {
            tracing::warn!(
                topic = %topic,
                "cerulion-vizd: attached a topic that resolved NO renderable archetype (no frame \
                 decoded within the peek window, or its schema is not mapped to a viz archetype) — \
                 it will not appear in the Scene or a plot until a frame with a known schema \
                 arrives (see schema/view_kinds in the attach reply)"
            );
        }
        // Re-apply this topic's remembered representation BEFORE the
        // layout is derived, so the views and the render are decided from the same
        // choice on the very first reflow after a re-check.
        self.apply_remembered_representation(&topic, &route_key);
        // Re-derive + apply the consolidated default layout (Scene + one plot per
        // topic, tiled, never tabs) unless an explicit layout verb has taken over.
        self.maybe_apply_default_layout();
        let archetype_kind = resolved.as_ref().map(|r| r.archetype);
        let (components, view_kinds) = self.layout_metadata_for_route(
            archetype_kind,
            self.representation_for(&topic),
            &route_key,
        );
        let (schema, archetype) = split_resolved(resolved);
        let entity = reported_entity_for(&route_key, archetype_kind);
        // Attribute the reply through the SAME map the sidebar surfaces
        // read, so a mirror attached by the local path (netd registered it before
        // Studio checked it — the live shape) still names its robot.
        let robot = self.attribution_snapshot().get(&topic).cloned();
        Response::Attach(AttachResponse {
            id: Some(id),
            ok: true,
            topic,
            schema,
            archetype,
            entity,
            route: route_key_topic(&route_key).to_string(),
            already_attached: already,
            components,
            view_kinds,
            robot,
        })
    }

    /// The REMOTE arm (with the remote-schema completion,
    /// remote = local). Resolve the topic's ROS type, DEMAND the topic's
    /// shared mirror from `cerulion-netd` — netd re-injects the remote
    /// robot's frames into desk-local SHM ONCE, shared by every consumer — then tap
    /// the now-local mirror by the IDENTICAL [`attach`](Self::attach) tap path, so
    /// the tap never knows the frames came over the network.
    ///
    /// `schema` (the topic's ROS type, `pkg/Type`) is OPTIONAL: when omitted,
    /// the daemon resolves it from the robot's served CATALOG, and when
    /// the type is a CUSTOM one the desk never compiled, it fetches the type's
    /// `.msg` closure from the robot and seeds its walker so the tapped frames
    /// decode + render — the "no schema pin!" product bar. See
    /// [`resolve_remote_type`](Self::resolve_remote_type). A hard, precise error is
    /// returned ONLY when the robot genuinely does not serve the topic/type (or the
    /// daemon is not network-configured).
    fn attach_remote(
        &self,
        id: u64,
        topic: String,
        entity_override: Option<String>,
        robot: String,
        schema: Option<String>,
    ) -> Response {
        // Guardrail 5: entity-override stability, checked EARLY (before any
        // network round-trip). A re-attach whose override resolves to a DIFFERENT
        // render entity than the one this topic is already known under is refused —
        // fail fast without demanding, so a conflicting re-point never wastes a netd
        // demand. (A first attach or a same-entity re-attach passes; the demand's
        // one-robot-per-topic rule is a separate guard in `perform_remote_demand`.)
        let route_key = route_key_for_topic(&topic, entity_override.as_deref());
        if let Some(msg) = entity_override_conflict(
            &topic,
            self.state
                .lock()
                .unwrap()
                .stats
                .get(&topic)
                .and_then(|s| s.route_key.as_deref()),
            &route_key,
        ) {
            return Response::error(Some(id), msg, Some(topic));
        }

        // 1. Resolve the topic's ROS type → the wire hash. A pinned schema the
        //    walker already knows is a network-free fast path; otherwise the type
        //    is catalog-resolved (schema-less) and/or its closure fetched + seeded
        //    over the daemon's ONE zenoh session. Precise errors on a
        //    genuinely-unserved topic/type or a network-less daemon.
        let (schema_name, schema_hash) =
            match self.resolve_remote_type(&topic, &robot, schema.as_deref()) {
                Ok(pair) => pair,
                // The resolve chain's verdict rides out on the wire. `retry`
                // is `Some` only for a NOT-FOUND-YET (see `AttachError`); every other
                // failure keeps the earlier terminal shape byte for byte.
                Err(e) => {
                    return Response::error_retryable(Some(id), e.message, Some(topic), e.retry)
                }
            };

        // 2a. Honor the CERULION_VIZD_NETWORK=off kill-switch: a LOCAL-ONLY vizd does
        //     not do remote viz, so refuse the remote attach here (the schema-less
        //     path already fails earlier in `resolve_remote_type` — its catalog fetch
        //     needs the session; this guard covers the pinned-schema path, which
        //     resolves the hash locally without touching the network). The netd demand
        //     plane has its OWN network independent of vizd's, but a user who set vizd
        //     off means "no remote viz from vizd" — keep that contract loud + exact.
        if self.manager.network().is_none() {
            return Response::error(
                Some(id),
                format!(
                    "remote viz for '{topic}' from robot '{robot}' needs a network-configured \
                     daemon, but this cerulion-vizd is LOCAL-ONLY (CERULION_VIZD_NETWORK=off) — \
                     restart it with its network default (scouting ON) to enable the remote arm"
                ),
                Some(topic),
            );
        }

        // 2b. DEMAND the shared mirror from `cerulion-netd` (the ONE
        //     per-computer network gateway) ONCE per topic per daemon lifetime,
        //     instead of opening vizd's OWN ingress on the zenoh session. netd owns
        //     the single mirror publisher + the zenoh demand token + the mirror
        //     PROVENANCE, so two consumers of the same remote topic (vizd + a user
        //     graph) share ONE mirror and the frame crosses the network ONCE.
        //     Each UDS control connection runs on its OWN handler
        //     thread, so RESERVE-THEN-COMMIT closes the concurrent-attach TOCTOU:
        //     atomically CLAIM `(topic → robot)` under ONE state lock, then demand
        //     OUTSIDE the lock (a network call — never held under the state mutex).
        //     Exactly one thread demands; a concurrent same-topic attach sees the
        //     claim and SKIPS (idempotent — the tap step below reports
        //     `already_attached`), and a re-attach after `detach` re-demands cleanly
        //     (netd's teardown freed the slot). A demand FAILURE rolls the claim
        //     back so a retry re-demands with no phantom claim left behind.
        let newly_demanded = match perform_remote_demand(
            self.demand_plane.as_ref(),
            &self.state,
            &robot,
            &topic,
            schema_hash,
        ) {
            Ok(fresh) => fresh,
            Err(e) => {
                return Response::error(
                    Some(id),
                    format!(
                        "remote viz for '{topic}' from robot '{robot}' could not demand the shared \
                         network mirror from cerulion-netd: {e}"
                    ),
                    Some(topic),
                );
            }
        };
        // On a FRESH demand, wait for netd's just-created mirror service to become
        // cross-process-visible before tapping it (netd is a separate process).
        if newly_demanded {
            wait_for_mirror_service(&self.manager, &topic);
        }

        // 3. Tap the now-local mirror by the IDENTICAL local tap path, and report
        //    the controller-provided schema + its archetype (there is no frame yet
        //    to resolve from; once frames flow the poll thread replaces this with
        //    the verdict on the first decodable frame). classify_schema is the
        //    name-based table; a schema not in it stays archetype=null until the
        //    first frame (no fabrication).
        //    `route_key` was resolved at the top (the entity-stability check).
        let archetype = classify_schema(&schema_name);
        // Attach under the lock, but DROP the guard before any rollback (the release
        // re-locks `state`). Guardrail 5: the entity-stability decision RE-runs
        // under this FINAL commit lock (the early check alone cannot close the
        // concurrent-attach_remote race — two racers both saw route_key None; here one
        // has committed its route_key, so the other sees the conflict). A conflict
        // refuses WITHOUT releasing the demand (a Conflict ⟹ AlreadyAttached ⟹ a
        // surviving attach whose `detach` releases the topic-keyed demand); only a TAP
        // error rolls the demand back.
        let commit = {
            let mut st = self.state.lock().unwrap();
            match st.taps.attach(
                &self.manager,
                &topic,
                entity_override.as_deref(),
                // A REMOTE/mirror topic gets the wake. Its
                // publisher is `cerulion-netd`'s OWN desk-side
                // `create_ingress_publisher` — on THIS machine, in a process we
                // own, never elision-armed (arming is graph-runtime-only)
                // — so the wake costs the desk one `sendto` per frame and bills
                // the robot nothing. This is the whole of the measured
                // 11.8 ms p50, and it is the ONLY class that gets a listener.
                WakeMode::Listener,
            ) {
                Err(e) => RemoteCommit::TapErr(e.to_string()),
                Ok(outcome) => {
                    // This topic ASKED for a wake. Recorded
                    // under the same lock as the tap, so `status` can tell a
                    // remote topic running on the timer (listener refused, or
                    // demoted) from a plain local tap that never wanted one.
                    st.wake_requested.insert(topic.clone());
                    let already = matches!(outcome, AttachOutcome::AlreadyAttached { .. });
                    match decide_route_commit(
                        &topic,
                        st.stats.get(&topic).and_then(|s| s.route_key.as_deref()),
                        &route_key,
                        already,
                    ) {
                        RouteCommit::Refuse(msg) => RemoteCommit::Conflict(msg),
                        commit => {
                            let stat = st.stats.entry(topic.clone()).or_default();
                            // The tap is up — start (or continue) its
                            // data-flow observation.
                            stat.begin_observation(Instant::now());
                            // Fresh attach records the route; AlreadyAttached (Keep)
                            // NEVER overwrites it (invariant b — the stat reflects the
                            // tap's actual key).
                            if matches!(commit, RouteCommit::Store) {
                                stat.route_key = Some(route_key.clone());
                            }
                            // Record the resolved type NAME unconditionally.
                            // The `if let Some(arch)` below writes it only when the
                            // name ALSO maps to an archetype, so on exactly the
                            // topics that go on to fail — an unmapped or unknown
                            // type — the name was computed here and thrown away,
                            // leaving the poll thread with a bare hash and no way to
                            // tell a version skew from a type it never compiled.
                            stat.pinned_schema = Some(schema_name.clone());
                            // Seed the resolution from the known type so status/list show
                            // it immediately. The poll thread replaces it with the verdict
                            // on the first decodable frame, and a re-attach must not write
                            // this name guess back over that verdict: the name table says
                            // `Image` for every CompressedImage, which the sink draws as
                            // `VideoStream` when its data is H.264.
                            if let Some(arch) = archetype.filter(|_| !stat.resolved_from_frame) {
                                stat.resolved = Some(Resolved {
                                    schema: schema_name.clone(),
                                    archetype: arch,
                                });
                            }
                            // Stamp the tap's ORIGIN ROBOT. Re-stamped on an
                            // AlreadyAttached re-attach too, so a topic tapped by a
                            // seam that could not attribute it — `compose_layout`'s
                            // local arm, or a demand-plane compose — is re-pointed by
                            // the first `attach` that names its robot.
                            //
                            // Monitors: which is why it goes through
                            // `retarget_origin_robot` rather than assigning the field.
                            // Monitor rows are keyed `(topic, robot)`, so a change of
                            // identity that only WROTE the new value would leave the
                            // old row being served with nothing able to release it.
                            // Last in the block because the helper takes `st` whole.
                            st.retarget_origin_robot(&topic, robot.clone());
                            RemoteCommit::Ok(already)
                        }
                    }
                }
            }
        };
        let already = match commit {
            RemoteCommit::Ok(already) => already,
            RemoteCommit::Conflict(msg) => {
                // Refuse WITHOUT releasing the demand: the surviving attach's `detach`
                // releases the topic-keyed demand; releasing here would tear down a
                // mirror still in use by the winner.
                return Response::error(Some(id), msg, Some(topic));
            }
            RemoteCommit::TapErr(e) => {
                // The tap failed AFTER the demand succeeded. RELEASE the demand +
                // drop the claim so the netd refcount does not leak (the controller
                // got an error, so it never `detach`es). A release failure is logged,
                // NEVER masks the original tap error.
                if let Err(re) =
                    perform_remote_release(self.demand_plane.as_ref(), &self.state, &topic)
                {
                    tracing::warn!(
                        topic = %topic, robot = %robot, error = %re,
                        "cerulion-vizd: remote attach failed at the tap step AND rolling the netd \
                         demand back also failed — the demand may leak until this daemon's \
                         connection to cerulion-netd drops"
                    );
                }
                return Response::error(Some(id), e, Some(topic));
            }
        };
        // Record the topic's origin robot in the session PROVENANCE TOMBSTONE.
        // Unlike `demanded` (released on detach), this SURVIVES the mirror's teardown,
        // so a later no-`robot` re-attach (the Studio sidebar sends a plain `attach`
        // after an uncheck tore the mirror down) re-demands from the right robot instead
        // of failing as a local topic. Idempotent on an AlreadyAttached re-attach.
        self.state
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .remote_provenance
            .insert(topic.clone(), robot.clone());
        // This attach DEMANDED a mirror from netd (which registers its
        // provenance), so the cached snapshot is now stale — drop it so the next
        // discover re-gathers and folds the new mirror to its robot promptly.
        self.invalidate_mirror_provenance_cache();
        // Keep the reapply-warn snapshot current (a new remote tap changes the set).
        self.refresh_attached_snapshot();
        // From here on, report what the stat holds: the name guess on a fresh
        // attach, the frame verdict once a frame has been classified (a re-attach
        // of a streaming topic, or a fresh attach the poll thread already served).
        // The warning below reads it too, so a re-attach of a topic whose frames
        // DO render does not claim that it renders nothing.
        let archetype = self
            .cached_resolution(&topic)
            .map(|r| r.archetype)
            .or(archetype);
        // Decision: a checked-but-unmappable REMOTE topic is LOUD too — its
        // schema resolved (it is served), but it maps to no viz archetype, so it renders
        // nothing. The reply carries `view_kinds: null`; this surfaces it in the log.
        if archetype.is_none() {
            tracing::warn!(
                topic = %topic, robot = %robot,
                "cerulion-vizd: attached a remote topic that maps to NO renderable archetype — it \
                 will not appear in the Scene or a plot (its schema is not mapped to a viz \
                 archetype; see view_kinds in the attach reply)"
            );
        }
        // Same as the local arm — the remembered choice is re-applied
        // before the reflow reads it.
        self.apply_remembered_representation(&topic, &route_key);
        // Re-derive + apply the consolidated default layout unless an explicit
        // layout verb has taken over (the archetype is already cached in stats above).
        self.maybe_apply_default_layout();
        let entity = reported_entity_for(&route_key, archetype);
        let (components, view_kinds) =
            self.layout_metadata_for_route(archetype, self.representation_for(&topic), &route_key);
        Response::Attach(AttachResponse {
            id: Some(id),
            ok: true,
            topic,
            schema: Some(schema_name),
            archetype: archetype.map(|a| archetype_name(a).to_string()),
            entity,
            route: route_key_topic(&route_key).to_string(),
            already_attached: already,
            components,
            view_kinds,
            // This path KNOWS the robot (it just demanded from it) and has
            // already stamped it on the tap's stat, so the reply states it directly
            // rather than paying a gather to re-derive the answer it supplied.
            robot: Some(robot),
        })
    }

    /// `detach`: drop the tap + its stats entry, and
    /// RELEASE the topic's netd DEMAND if it held one. Releasing the demand
    /// (removing the `demanded` entry) lets netd's refcount-0 teardown tear the
    /// shared mirror down when the LAST consumer leaves. No marker is kept
    /// for a re-attach to reuse, because a
    /// re-attach after a release cleanly re-demands (netd freed the slot). The
    /// release is best-effort: a netd release failure is logged, never fails the
    /// detach (the tap is already gone; the demand ages out when vizd's connection
    /// drops at worst).
    fn detach(&self, id: u64, topic: String) -> Response {
        let detached = {
            let mut st = self.state.lock().unwrap();
            let d = st.taps.detach(&topic);
            let removed = st.stats.remove(&topic);
            // Monitors: the monitor row dies with the tap it was
            // derived from, and it is released HERE — inside the same acquisition
            // that removed the stats entry, so the sampler can never observe a
            // half-detached topic and resurrect its row.
            //
            // Not a leak fix but a CORRECTNESS one, and it is the row's own contract:
            // a re-attach genuinely begins a new observation and a new baseline
            // (`TopicStat`'s entry is gone, so its Hz window and frame count start
            // over), so nothing learned about the old producer describes the new
            // one. The alerts the row already minted stay in the ring — they record
            // something that really happened. The ROBOT is read off the record that
            // is about to be dropped, because rows are keyed by `(topic, robot)`
            // and forgetting the wrong key would leave the real row behind.
            let departing_robot = removed.and_then(|s| s.origin_robot);
            st.monitors.forget(departing_robot.as_deref(), &topic);
            // The wake REQUEST dies with the tap, so a later
            // local attach of the same name is not reported as a degraded remote.
            st.wake_requested.remove(&topic);
            d
        };
        // Release the netd demand (outside the tap lock — a network round-trip).
        if let Err(e) = perform_remote_release(self.demand_plane.as_ref(), &self.state, &topic) {
            tracing::warn!(
                topic = %topic, error = %e,
                "cerulion-vizd: could not release the netd demand on detach — the shared mirror \
                 lingers until this daemon's connection to cerulion-netd drops"
            );
        }
        // The release tears the mirror down (netd refcount-0), so its
        // provenance is about to disappear — drop the cached snapshot so the next
        // discover re-gathers and the topic returns to its robot row promptly.
        self.invalidate_mirror_provenance_cache();
        // Keep the reapply-warn snapshot current (the detached topic left the set —
        // a reconnect re-applying a layout rooted only at it now warns it's empty).
        self.refresh_attached_snapshot();
        // Re-derive + apply the consolidated default layout (the detached topic's
        // view drops out) unless an explicit layout verb has taken over.
        self.maybe_apply_default_layout();
        Response::Detach(DetachResponse {
            id: Some(id),
            ok: true,
            topic,
            detached,
        })
    }

    /// `list`: the attached taps with their resolved route/entity/schema.
    fn list(&self, id: u64) -> Response {
        // The SAME attribution map `discover` folds with, so an attached
        // mirror's row and its robot's row are one row. Built BEFORE the state lock
        // (it takes that lock itself; std mutexes are not reentrant).
        let attribution = self.attribution_snapshot();
        // Probed BEFORE the lock — see `registration_snapshot` for why the
        // probes must not run inside it.
        let registration = self.registration_snapshot();
        // Dated OUTSIDE the lock so a cache-miss gather above cannot age
        // every row's observation by the gather duration (the same reason `status`
        // captures `now` after its own snapshot).
        let now = Instant::now();
        let st = self.state.lock().unwrap();
        let attached = st
            .taps
            .list()
            .into_iter()
            .map(|topic| {
                let robot = attribution.get(&topic).cloned();
                let stat = st.stats.get(&topic);
                // The registration + data-flow truth this row renders from.
                let producer_count = registration.get(&topic).copied().flatten();
                let liveness = stat.and_then(|s| s.liveness(now));
                let liveness_state = liveness.and_then(|l| l.wire_state());
                let route = stat
                    .and_then(|s| s.route_key.clone())
                    .unwrap_or_else(|| route_key_for_topic(&topic, None));
                let resolved = stat.and_then(|s| s.resolved.clone());
                let undecodable = stat.and_then(|s| s.undecodable.clone());
                let entity = reported_entity_for(&route, resolved.as_ref().map(|r| r.archetype));
                // Carry the SAME deterministic layout placement the
                // attach/status/discover responses expose, through the SAME helper.
                let (components, view_kinds) = self.layout_metadata_for_route(
                    resolved.as_ref().map(|r| r.archetype),
                    st.representation.get(&topic).copied().unwrap_or_default(),
                    &route,
                );
                let (schema, archetype) = split_resolved(resolved);
                // ABSENT on `auto`, so an untouched desk's wire is
                // byte-identical to the wire from before the choice existed.
                let representation = st
                    .representation
                    .get(&topic)
                    .map(|r| r.as_wire().to_string());
                AttachedEntry {
                    topic,
                    route: route_key_topic(&route).to_string(),
                    entity,
                    schema,
                    archetype,
                    components,
                    view_kinds,
                    robot,
                    producer_count,
                    liveness,
                    liveness_state,
                    representation,
                    // Without this an undecodable row is byte-identical
                    // to a silent one — `schema: null` while frames climb.
                    undecodable,
                }
            })
            .collect();
        Response::List(ListResponse {
            id: Some(id),
            ok: true,
            attached,
        })
    }

    /// `status`: per-topic Hz + frame counts + the worker's global counters.
    fn status(&self, id: u64) -> Response {
        // The SAME attribution map (see `list`), built before the lock.
        // A cache MISS can block here for the gather window, so `now` is captured
        // AFTER it — timestamping before would age every Hz window by the gather
        // duration and under-report the rate of every attached topic.
        let attribution = self.attribution_snapshot();
        // Probed BEFORE the lock (see `registration_snapshot`), and before
        // `now` for the same reason the attribution gather is.
        let registration = self.registration_snapshot();
        let now = Instant::now();
        let topics = {
            let st = self.state.lock().unwrap();
            st.taps
                .list()
                .into_iter()
                .map(|topic| {
                    let robot = attribution.get(&topic).cloned();
                    // This row's LATENCY CLASS. `Some(true)` = drained
                    // within a wake and billing its producer a notify per frame;
                    // `Some(false)` = a topic that ASKED for a wake and does not
                    // have one (the listener could not be opened, or it was
                    // demoted for firing without delivering).
                    //
                    // EVERY seam asks, local included,
                    // so no row this daemon serves is absent. The absent arm
                    // survives for the WIRE — an older daemon sends no field
                    // — and as the guard described on `wake_requested`.
                    let wake = if st.taps.has_wake(&topic) {
                        Some(true)
                    } else if st.wake_requested.contains(&topic) {
                        Some(false)
                    } else {
                        None
                    };
                    let stat = st.stats.get(&topic);
                    let hz = stat.and_then(|s| s.hz(now));
                    // The verdict the row falls through to when `hz` is
                    // `None`, plus the registration truth that decides whether the
                    // row belongs on screen at all.
                    let producer_count = registration.get(&topic).copied().flatten();
                    let liveness = stat.and_then(|s| s.liveness(now));
                    let liveness_state = liveness.and_then(|l| l.wire_state());
                    let frames = stat.map(|s| s.frames_seen).unwrap_or(0);
                    // `Some(0)` = checked, lost nothing. A row with NO
                    // stats entry has not been checked, so it claims nothing.
                    let frames_missed = stat.map(|s| s.frames_missed);
                    let resolved = stat.and_then(|s| s.resolved.clone());
                    // `frames` climbing beside a `null` schema is the
                    // exact shape a version skew produces; this says which it is.
                    let undecodable = stat.and_then(|s| s.undecodable.clone());
                    let archetype_kind = resolved.as_ref().map(|r| r.archetype);
                    // The render entity path — derived exactly as `list` derives
                    // its entity (the recorded route key, or the topic-derived one).
                    // Resolved BEFORE the layout metadata, which now keys
                    // its live signals on it.
                    let route = stat
                        .and_then(|s| s.route_key.clone())
                        .unwrap_or_else(|| route_key_for_topic(&topic, None));
                    let (components, view_kinds) = self.layout_metadata_for_route(
                        archetype_kind,
                        st.representation.get(&topic).copied().unwrap_or_default(),
                        &route,
                    );
                    let (schema, archetype) = split_resolved(resolved);
                    let entity = reported_entity_for(&route, archetype_kind);
                    // This row's `view_kinds` already carry the choice,
                    // so it carries the choice itself too.
                    let representation = st
                        .representation
                        .get(&topic)
                        .map(|r| r.as_wire().to_string());
                    TopicStatus {
                        topic,
                        hz,
                        frames,
                        frames_missed,
                        schema,
                        archetype,
                        entity,
                        components,
                        view_kinds,
                        robot,
                        producer_count,
                        liveness,
                        liveness_state,
                        representation,
                        undecodable,
                        wake,
                    }
                })
                .collect()
        };
        let worker = WorkerStatus {
            dropped_frames: self.counters.dropped_frames.load(Ordering::Relaxed),
            dropped_batches: self.counters.dropped_batches.load(Ordering::Relaxed),
            coalesced_frames: self.counters.coalesced_frames.load(Ordering::Relaxed),
            reconnects: self.counters.reconnects.load(Ordering::Relaxed),
        };
        Response::Status(StatusResponse {
            id: Some(id),
            ok: true,
            topics,
            worker,
        })
    }

    /// The CURRENT mirror-provenance snapshot as a `topic → origin robot`
    /// map, served from a short-TTL CACHE ([`DaemonState::mirror_provenance`]) so a live
    /// mirror does not cost `discover` a 150-600 ms gather on every poll. On a cache
    /// MISS (`None` or older than [`PROVENANCE_CACHE_TTL`]) it re-gathers from the desk's
    /// `/__cerulion/mirrors` registry over THIS daemon's transport node (the parity of
    /// the CLI's `topic_cmd::gather_mirror_provenance`) and stores the fresh snapshot.
    /// The gather runs OUTSIDE the state lock (it can block up to the window); a
    /// concurrent double-gather just overwrites (harmless, last-write-wins). BEST-EFFORT
    /// and network-free: a registry-less desk returns empty INSTANTLY (the no-publisher
    /// fast path in cerulion_core), and ANY gather error is treated as "no provenance" (a
    /// `debug!` breadcrumb, cached like a real snapshot so a transient registry error
    /// does not re-storm the gather) so `discover` never fails on the registry — the
    /// stated fallback is that a mirrored topic shows as LOCAL (pre-fold).
    fn gather_mirror_provenance(&self) -> std::collections::BTreeMap<String, String> {
        // Fast path: a fresh cached snapshot (the common steady-state poll).
        {
            let st = self.state.lock().unwrap();
            if let Some((at, snapshot)) = st.mirror_provenance.as_ref() {
                if at.elapsed() < PROVENANCE_CACHE_TTL {
                    return snapshot.clone();
                }
            }
        }
        // Cache miss — gather OUTSIDE the lock (the drain can block up to the window).
        let fresh: std::collections::BTreeMap<String, String> =
            match self.manager.gather_mirror_provenance(
                cerulion_core::transport::mirror_registry::MIRROR_GATHER_WINDOW,
            ) {
                Ok(records) => records
                    .into_iter()
                    .map(|r| (r.topic, r.origin_robot))
                    .collect(),
                Err(e) => {
                    tracing::debug!(
                        error = %e,
                        "cerulion-vizd discover: mirror-provenance gather failed — treating as no \
                         mirrors (any mirrored topics show as LOCAL this sweep)"
                    );
                    std::collections::BTreeMap::new()
                }
            };
        let mut st = self.state.lock().unwrap();
        st.mirror_provenance = Some((Instant::now(), fresh.clone()));
        fresh
    }

    /// The attribution map EVERY sidebar surface reads — `discover`'s
    /// enumeration fold and the `list`/`status` attach-state rows.
    ///
    /// It is the live `/__cerulion/mirrors` snapshot merged with the ORIGIN ROBOT
    /// each currently-held tap recorded for ITSELF at attach ([`TopicStat`]'s
    /// `origin_robot`). One map for all three surfaces is what makes "one data
    /// source = one topic" structural: they cannot place one topic in two sections,
    /// because they are reading the same answer. Live provenance wins (see
    /// [`cerulion_core::transport::mirror_registry::merge_attribution`]).
    ///
    /// The per-tap half matters because the live gather is BEST-EFFORT and cached:
    /// it can momentarily fail to answer for a topic this daemon knowingly attached
    /// from a robot (a gather error, or netd tearing the mirror down under a
    /// still-held tap), and without it the row would flicker into the desk's own
    /// section — re-creating the phantom row, just transiently.
    ///
    /// It reads the TAP's own record and NOT the session tombstone
    /// ([`DaemonState::remote_provenance`]) — see [`TopicStat::origin_robot`]. The
    /// tombstone is never cleared and is keyed by NAME, so a topic whose name was
    /// once remote would keep being attributed to that robot across a detach and a
    /// genuine desk producer taking the name; the tap's own record dies with the
    /// tap, so it can only ever describe the generation that earned it.
    ///
    /// COST: a cache MISS pays the `/__cerulion/mirrors` gather (up to
    /// [`MIRROR_GATHER_WINDOW`](cerulion_core::transport::mirror_registry::MIRROR_GATHER_WINDOW),
    /// ~600 ms, and instant on a desk with no live mirror), so `list`/`status` are
    /// not unconditionally O(1). The [`PROVENANCE_CACHE_TTL`] cache means at most
    /// one such gather per 3 s across all three verbs.
    ///
    /// Takes the state lock, so callers must NOT already hold it.
    fn attribution_snapshot(&self) -> std::collections::BTreeMap<String, String> {
        // Gather BEFORE the lock — `gather_mirror_provenance` takes it for the cache
        // and std mutexes are not reentrant.
        let mirrors = self.gather_mirror_provenance();
        // AUTHORITATIVE records only. A HINTED record is a per-tap cache of the very
        // map being merged here, so including it would add nothing on a hit — and on
        // a MISS it would do real harm, re-asserting a name-keyed claim the live
        // answer has just withdrawn. That is the fallback wanted for a tap
        // this daemon knowingly DEMANDED from a robot, and precisely what it does
        // NOT want for a guess about a name (see `TopicStat::origin_robot_hinted`).
        let held = {
            let st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            st.taps
                .list()
                .into_iter()
                .filter_map(|topic| {
                    let stat = st.stats.get(&topic)?;
                    let robot = stat.origin_robot.clone()?;
                    (!stat.origin_robot_hinted).then_some((topic, robot))
                })
                .collect()
        };
        cerulion_core::transport::mirror_registry::merge_attribution(&mirrors, &held)
    }

    /// Every held tap's OWN data-flow observation, as one snapshot taken
    /// at ONE instant.
    ///
    /// The desk runs no standing `TopicLivenessObserver` — but a tap IS a data-flow
    /// observer, and vizd holds one per attached topic. That makes a first-hand
    /// verdict available for exactly the rows that need one
    /// (checked topics whose producers have been killed), on ANY robot, with no
    /// robot-side support: a robot that predates robot-side liveness entirely still gets its
    /// attached rows classified.
    ///
    /// Snapshotted rather than queried per row so every entry of one response is
    /// dated from the same instant, and so `discover` reads the same numbers
    /// `list`/`status` do (two surfaces computing the same
    /// answer separately is how they come to disagree).
    ///
    /// Takes the state lock, so callers must NOT already hold it.
    fn tap_liveness_snapshot(&self) -> BTreeMap<String, TopicLiveness> {
        let now = Instant::now();
        let st = self.state.lock().unwrap_or_else(|e| e.into_inner());
        st.taps
            .list()
            .into_iter()
            .filter_map(|topic| {
                let liveness = st.stats.get(&topic)?.liveness(now)?;
                Some((topic, liveness))
            })
            .collect()
    }

    /// The registration probe for every held tap — `topic → producer
    /// count` — taken WITHOUT the state lock held.
    ///
    /// The lock discipline is the point. Each probe opens an iceoryx2 service, and
    /// the attach-state verbs run on a controller thread while the POLL thread needs
    /// the same state lock every [`DEFAULT_POLL_INTERVAL`] to drain every tap. Probing
    /// inside the `list`/`status` map would hold that lock across N service opens —
    /// on a 75-topic robot, long enough to stall the drain and lose frames to keep a
    /// sidebar row accurate, which is a bad trade. So the lock is taken only to read the
    /// tap NAMES, released, and the probes run on the names.
    ///
    /// A tap detached between the two is simply probed and discarded; a tap attached
    /// in between is absent from this map and reports UNKNOWN for one poll, which is
    /// the correct answer for a topic this snapshot never looked at.
    fn registration_snapshot(&self) -> BTreeMap<String, Option<u32>> {
        let topics = {
            let st = self.state.lock().unwrap_or_else(|e| e.into_inner());
            st.taps.list()
        };
        topics
            .into_iter()
            .map(|topic| {
                let count = self.manager.topic_publisher_count_checked(&topic);
                (topic, count)
            })
            .collect()
    }

    /// Drop the cached mirror-provenance snapshot so the NEXT `discover`
    /// re-gathers fresh. Called on the mutation points vizd itself drives (`attach_remote`
    /// demands a mirror, `detach` releases one), so the user's own check/uncheck is
    /// reflected on the very next discover instead of waiting out the TTL.
    fn invalidate_mirror_provenance_cache(&self) {
        self.state.lock().unwrap().mirror_provenance = None;
    }

    /// `discover`: every attachable local topic + a best-effort resolved schema.
    fn discover(&self, id: u64) -> Response {
        let topics = match self.manager.list_topics() {
            Ok(t) => t,
            // A discovery failure must be LOUD, never a silently-empty list.
            Err(e) => {
                return Response::error(Some(id), format!("topic discovery failed: {e}"), None)
            }
        };
        // The mirror fold (Studio parity with `cerulion topic list`): a
        // locally-visible `{topic}/data` service that is really a re-injected MIRROR
        // of a remote robot's topic must NOT present as a second LOCAL topic — it
        // folds OUT of LOCAL and INTO its origin robot's section (the
        // "one data source = one topic" rule). Without this, checking a robot topic
        // in Studio makes cerulion-netd register a desk-local mirror, and `discover`
        // would report it LOCAL — moving it from the robot group to THIS MACHINE (and,
        // if the mirror lingers after detach, keeping it there). Provenance is read
        // from local SHM (network-free, best-effort), so the fold holds even under a
        // netd-unreachable catalog gather; a registry-less desk yields an empty map →
        // every topic stays LOCAL (as if there were no fold).
        let mirrors = self.attribution_snapshot();
        // This daemon's OWN tap observations, snapshotted once so every row
        // of this response reads the SAME truth `list`/`status` are reporting. The
        // desk runs no standing liveness observer, but a TAP IS one — so a topic
        // Studio has checked carries a first-hand verdict here too, instead of a
        // blanket UNKNOWN that makes the sidebar fall back to a rate.
        let tap_liveness = self.tap_liveness_snapshot();

        // Bound the TOTAL time spent peeking silent topics across the WHOLE sweep
        // with a shared deadline, not a per-topic one, so N idle topics
        // never cost N × PEEK_BOUND sequentially. Cached topics cost nothing;
        // once the budget is spent, the rest return `schema: null`.
        let peek_deadline = Instant::now() + DISCOVER_PEEK_BUDGET;

        // Partition the local enumeration by provenance, through the SHARED core
        // predicate `cerulion topic list` folds with (it was hoisted out of
        // `cerulion_cli_engine` for exactly this reason — one fold, every surface).
        // `local_names` (the ROBOTS-section local-present filter) is built from ONLY
        // the genuine-local topics, so a mirror is NOT filtered out of its robot's
        // catalog as "already local" — it stays attributed to its robot.
        let (genuine_local, streaming) =
            cerulion_core::transport::mirror_registry::partition_local_topics(topics, &mirrors);
        let local_names: std::collections::BTreeSet<String> =
            genuine_local.iter().cloned().collect();

        let entries = genuine_local
            .into_iter()
            .map(|topic| {
                let resolved = self.resolve_for_discover(&topic, peek_deadline);
                // ONE state read per row (representation + route +
                // undecodable), so this row's placement, its `representation` field
                // and its verdict cannot come from three different snapshots.
                let row_state = self.discover_row_state(&topic);
                // A discover row for a topic this desk IS
                // rendering reports the LIVE placement, so the sidebar cannot
                // disagree with `status` about the same topic; one nothing has
                // rendered keeps the default. See the helper for why a
                // blanket default is false on its own premise.
                let (components, view_kinds) = self.layout_metadata_for_discover_row(
                    resolved.as_ref().map(|r| r.archetype),
                    row_state.representation,
                    row_state.route.as_deref(),
                );
                let archetype_kind = resolved.as_ref().map(|r| r.archetype);
                let (schema, archetype) = split_resolved(resolved);
                // The entity this topic WOULD render under if attached (derived
                // deterministically from the name — no override at discover time),
                // so an agent can lay out before attaching. Through
                // `reported_entity_for` like every OTHER surface: a tf-NAMED topic
                // whose frames are not a TFMessage is reconciled off the viz root by
                // the sink, and a discover row that still said `world` would name a
                // location nothing is ever logged at.
                let entity =
                    reported_entity_for(&route_key_for_topic(&topic, None), archetype_kind);
                // The desk's own live producer count for this LOCAL topic —
                // the per-row liveness affordance (`Some(0)` = a registered-but-dead
                // route, `Some(n)` = live). A cheap no-wait probe of local SHM.
                // `_checked` reports `None` (UNKNOWN) on a transient probe
                // FAILURE rather than a fabricated `Some(0)` that would dim a healthy row.
                let producer_count = self.manager.topic_publisher_count_checked(&topic);
                let liveness = tap_liveness.get(&topic).copied();
                let liveness_state = liveness.and_then(|l| l.wire_state());
                // The SAME snapshot the placement above was
                // computed from — the reason for carrying this field is
                // that the view kinds already encode it, which a second read could
                // make false.
                let representation = row_state.representation;
                let undecodable = row_state.undecodable;
                DiscoveredEntry {
                    topic,
                    schema,
                    archetype,
                    entity,
                    components,
                    view_kinds,
                    // A genuine-local topic carries no robot attribution.
                    robot: None,
                    producer_count,
                    // The desk runs no STANDING liveness observer,
                    // so an un-tapped local topic still reports UNKNOWN — a
                    // serve-time peek provably drains nothing (see
                    // `cerulion_core::transport::liveness`), so there is no grounded
                    // verdict to synthesize here. But a topic this daemon has a TAP
                    // on has been observed first-hand, and this daemon carries that
                    // observation onto this row so `discover`, `list` and `status`
                    // cannot disagree about the same topic.
                    liveness,
                    liveness_state,
                    representation: (!representation.is_auto())
                        .then(|| representation.as_wire().to_string()),
                    undecodable,
                }
            })
            .collect();

        // The streaming mirrors, each resolved LOCALLY (the re-injected frames are
        // tappable) and self-attributed to its origin robot — folded into the ROBOTS
        // section below so the topic is attributed to its robot EVEN when netd's
        // catalog gather is unreachable / does not list it (the network-free
        // guarantee mirroring the CLI's `partition_local_topics`).
        let streaming_entries: Vec<(String, DiscoveredEntry)> = streaming
            .into_iter()
            .map(|row| {
                let (topic, robot) = (row.topic, row.robot);
                let resolved = self.resolve_for_discover(&topic, peek_deadline);
                // ONE state read per row, exactly as the
                // genuine-local arm above.
                let row_state = self.discover_row_state(&topic);
                // Same rule as the genuine-local rows above — a
                // MIRROR this desk has attached and is rendering is exactly the
                // shape that drifts, since attaching it is what Studio does.
                let (components, view_kinds) = self.layout_metadata_for_discover_row(
                    resolved.as_ref().map(|r| r.archetype),
                    row_state.representation,
                    row_state.route.as_deref(),
                );
                let archetype_kind = resolved.as_ref().map(|r| r.archetype);
                let (schema, archetype) = split_resolved(resolved);
                let entity =
                    reported_entity_for(&route_key_for_topic(&topic, None), archetype_kind);
                // A live mirror is a re-injected producer in local SHM, so its
                // producer count reflects the desk-side re-inject publisher (a streaming
                // mirror is live by definition). The same cheap no-wait local probe;
                // `None` (UNKNOWN) on a transient probe failure, never a
                // fabricated `Some(0)`.
                let producer_count = self.manager.topic_publisher_count_checked(&topic);
                let liveness = tap_liveness.get(&topic).copied();
                let liveness_state = liveness.and_then(|l| l.wire_state());
                // The SAME snapshot the placement was computed
                // from (see the genuine-local arm).
                let representation = row_state.representation;
                // A MIRROR is re-injected verbatim, so its frames carry
                // the ROBOT's schema hash and this desk is exactly where a skew
                // shows up. Read from the tap's own stat — absent when this daemon
                // holds no tap on the mirror.
                let undecodable = row_state.undecodable;
                (
                    robot.clone(),
                    DiscoveredEntry {
                        topic,
                        schema,
                        archetype,
                        entity,
                        components,
                        view_kinds,
                        // A mirror self-attributes its origin robot (so attach
                        // `demand`s it — the origin logic).
                        robot: Some(robot),
                        producer_count,
                        // A streaming mirror's local publisher is
                        // the re-injector, so `producer_count` says nothing about
                        // whether the REMOTE source is alive. The authoritative
                        // answer is the ROBOT's `liveness`, which reaches the sidebar
                        // through the catalog: `fold_streaming_mirrors_into_robots`
                        // lets a catalog entry WIN over this local row, so a mirrored
                        // topic normally renders the robot's own observation.
                        //
                        // This improves the FALLBACK for a robot netd could not
                        // reach: if this daemon holds a tap on the mirror, the frames
                        // it re-injects (or stops re-injecting) ARE a first-hand
                        // observation of the remote stream, so we report that instead
                        // of UNKNOWN. Un-tapped, UNKNOWN is still the only correct
                        // answer.
                        liveness,
                        liveness_state,
                        representation: (!representation.is_auto())
                            .then(|| representation.as_wire().to_string()),
                        undecodable,
                    },
                )
            })
            .collect();

        // The REMOTE gather — every announcing robot's catalog over the
        // shared `cerulion-netd` query plane (ONE zenoh session per computer). A
        // netd-unreachable / not-network-configured error LOUDLY degrades to a
        // LOCAL-ONLY discover (never breaks a previously-working local listing); an
        // `Ok` (even empty) is authoritative. A REFUSED catalog surfaces explicitly.
        // The gather's DISCOVERY STATE rides along, because the fold
        // below makes an ABSENCE claim with it (a mirror the catalog omits is dropped,
        // which hides the row). An unconverged gather may not license that.
        //
        // This is a REFRESHING surface (Studio re-asks, and vizd now
        // PUSHES netd's catalog changes to subscribed controllers), so it answers in
        // one round trip and carries the MARKER instead — a cold-start empty sidebar
        // becomes distinguishable on the wire from a settled one, and a controller
        // renders "discovering…" rather than a terminal "no robots".
        let mut catalog_discovery = DiscoveryState::NotConverged;
        // `None` = netd could not be reached at all, which is UNKNOWN — not a claim
        // that discovery is or is not converged.
        let mut reported_discovery = None;
        // Monitors: the ticket that orders this gather's feed against
        // every other one in flight — taken BEFORE the gather, because the hazard
        // is a slow gather that STARTED first and hands over LAST carrying the
        // older answer. See `begin_catalog_gather`.
        let gather_gen = self.begin_catalog_gather();
        let robots = match self.gather_catalog(None) {
            Ok(gather) => {
                catalog_discovery = gather.discovery;
                reported_discovery = Some(DiscoveryLabel::from_state(catalog_discovery));
                // Monitors: the DISCOVERY plane's sample source — the
                // required property, a verdict on a topic NOBODY checked.
                //
                // It PIGGYBACKS on this gather and adds none of its own:
                // netd is stateless and uncached, so every `query_catalog`
                // costs a real LAN harvest on the one per-machine daemon, and a
                // standing self-refresh would double that to learn nothing this
                // call has not already fetched. The cost of the choice is that an
                // unattached row is only as current as the last `discover` — which
                // is why every row carries `last_sample_age_ms` and `samples`, so
                // the agent is told how fresh the evidence is and can never read
                // silence as health.
                //
                // BEFORE `build_discovered_robots`, which consumes the catalogs;
                // AFTER the gather, which is the point — the feed takes the
                // catalogs by reference and holds no query plane, so the state
                // lock it takes cannot span a round trip. The poll thread takes
                // that same lock every 16 ms.
                self.observe_catalog_monitors(&gather.catalogs, catalog_discovery, gather_gen);
                build_discovered_robots(gather.catalogs, &local_names)
            }
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "cerulion-vizd: cerulion-netd unreachable for the discover catalog gather — \
                     showing LOCAL topics only (start cerulion-netd, or check the robot is \
                     reachable, then Refresh)"
                );
                Vec::new()
            }
        };
        // Fold the desk-local streaming mirrors into the ROBOTS section (network-free)
        // — a catalog that already lists the topic WINS (no duplicate); a robot netd
        // could not reach still shows its live mirror.
        let robots =
            fold_streaming_mirrors_into_robots(robots, streaming_entries, catalog_discovery);

        Response::Discover(DiscoverResponse {
            id: Some(id),
            ok: true,
            topics: entries,
            robots,
            discovery: reported_discovery,
        })
    }

    /// The cached resolution for a topic, if the poll thread has already resolved
    /// it (from a drained frame).
    fn cached_resolution(&self, topic: &str) -> Option<Resolved> {
        let st = self.state.lock().unwrap();
        st.stats.get(topic).and_then(|s| s.resolved.clone())
    }

    /// Resolve a TAPPED topic's schema for the attach response: cached if the
    /// poll thread already saw a frame, else a bounded transient peek (CACHED
    /// back so `status`/`list` show it immediately).
    fn resolve_for_attach(&self, topic: &str) -> Option<Resolved> {
        self.resolve_for_attach_bounded(topic, Instant::now() + PEEK_BOUND)
    }

    /// Resolve a TAPPED topic's archetype like [`Self::resolve_for_attach`], but with
    /// the transient peek bounded by BOTH [`PEEK_BOUND`] and a caller-supplied
    /// `deadline` — the per-REQUEST budget `compose_layout` threads across ITS whole
    /// batch, so N freshly-attached silent topics never cost
    /// N × [`PEEK_BOUND`] serially. Cached topics resolve for free (no budget cost);
    /// once the budget is spent, an uncached topic resolves to `None` (still-silent).
    fn resolve_for_attach_bounded(&self, topic: &str, deadline: Instant) -> Option<Resolved> {
        if let Some(r) = self.cached_resolution(topic) {
            return Some(r);
        }
        let remaining = deadline.checked_duration_since(Instant::now())?; // budget spent → None
                                                                          // The candidate NAME for this topic, if the remote attach path
                                                                          // pinned one. Read before the peek so the diagnosis below can name a
                                                                          // remedy rather than a bare hash.
        let candidate = {
            let st = self.state.lock().unwrap();
            st.stats.get(topic).and_then(|s| s.pinned_schema.clone())
        };
        let outcome = peek_resolve(
            &self.manager,
            &self.walker_snapshot(),
            topic,
            remaining.min(PEEK_BOUND),
            candidate.as_deref(),
        );
        // This peek ALREADY caches its result into `stats`, so recording
        // an undecodable verdict here creates no orphan entry — it fills in the
        // one this call was going to write anyway. That matters at attach time:
        // a topic whose frames cannot be read is the case the user is most
        // likely to be staring at.
        let mut st = self.state.lock().unwrap();
        let stat = st.stats.entry(topic.to_string()).or_default();
        record_resolution(stat, topic, &outcome);
        outcome.ok()
    }

    /// Resolve a topic's schema for `discover`: cached if tapped (free, always),
    /// else a transient peek bounded by BOTH [`PEEK_BOUND`] and the sweep-wide
    /// `deadline` — but NOT cached (an un-tapped topic must leave no
    /// orphan stats entry). Once `deadline` passes, uncached topics resolve to
    /// `None` (`schema: null`) with no peek at all.
    fn resolve_for_discover(&self, topic: &str, deadline: Instant) -> Option<Resolved> {
        if let Some(r) = self.cached_resolution(topic) {
            return Some(r);
        }
        let now = Instant::now();
        let remaining = deadline.checked_duration_since(now)?; // budget spent → null
                                                               // Deliberately NOT recorded. This path must leave no stats entry
                                                               // for an un-tapped topic (the invariant above), and recording an
                                                               // undecodable verdict would mint exactly such an orphan. A `discover` row
                                                               // for a topic this daemon HAS tapped still carries the verdict — it is
                                                               // read from the cached stat by the response builder.
        peek_resolve(
            &self.manager,
            &self.walker_snapshot(),
            topic,
            remaining.min(PEEK_BOUND),
            None,
        )
        .ok()
    }
}

/// The layout enrichment `(components, view_kinds)` for a resolved
/// archetype — the deterministic placement the layout agent is HANDED instead of
/// guessing. `components` is the rerun families the topic renders as;
/// `view_kinds` the resolved view(s) it belongs in. An unresolved archetype
/// (silent topic) yields `(None, None)` — serialized as `null`, matching the
/// `schema:null`/`archetype:null` contract those same topics already carry, so
/// `null` (NOT yet resolved: no view answer) is distinct from a hypothetical `[]`
/// ("resolved: renders nothing"). A resolved archetype always yields non-empty
/// lists (every variant produces ≥1 component and ≥1 view).
///
/// **The live signals are ARGUMENTS, not blanks.** This builds a synthetic
/// [`AttachedRender`] purely to ask [`views_for_render`]. Hardcoding
/// an empty rendition set there would make the `view_kinds` an `attach` reply and every
/// `status` row reports still list `text_document` for a decoding video topic
/// whose APPLIED blueprint (built from [`Ctx::attached_render_infos`], which does
/// read the mirror) has already dropped it. One run, two answers, and the one a
/// client can read would be the wrong one. Threading both signals through is what keeps
/// the reported placement the placement.
fn layout_metadata(
    archetype: Option<ArchetypeKind>,
    representation: Representation,
    video_renditions: Vec<String>,
    render_proof: RenderProof,
) -> (Option<Vec<String>>, Option<Vec<String>>) {
    match archetype {
        Some(kind) => (
            Some(
                archetype_components(kind)
                    .iter()
                    .map(|s| s.to_string())
                    .collect(),
            ),
            // The view kinds this TOPIC renders in, choice applied — the
            // reply tells the operator (and the agent laying topics out) where the
            // topic actually lands, not where its archetype alone would. Reporting
            // the archetype's own list on an overridden topic would be an
            // affirmatively WRONG claim about a row the daemon itself is placing
            // elsewhere.
            Some(
                views_for_render(&AttachedRender {
                    topic: String::new(),
                    entity: String::new(),
                    archetype: Some(kind),
                    producer_count: None,
                    video_renditions,
                    representation,
                    render_proof,
                })
                .iter()
                .map(|v| v.as_str().to_string())
                .collect(),
            ),
        ),
        None => (None, None),
    }
}

/// Split a resolution into the response's `(schema, archetype)` string pair.
fn split_resolved(resolved: Option<Resolved>) -> (Option<String>, Option<String>) {
    match resolved {
        Some(r) => (
            Some(r.schema),
            Some(archetype_name(r.archetype).to_string()),
        ),
        None => (None, None),
    }
}

/// The stable wire name for an [`ArchetypeKind`] (the protocol string). Delegates
/// to the single home in `cerulion_viz::blueprint` so the response strings and the
/// guardrail-2 error can never drift apart.
fn archetype_name(kind: ArchetypeKind) -> &'static str {
    archetype_wire_name(kind)
}

/// Guardrail 5 (the zombie-view root cause): the PURE entity-override
/// stability check. Given a `topic`, the route key it is CURRENTLY known under
/// (`existing_route`, `None` on a first attach), and the `requested_route` a new
/// attach derives from its override, return `Some(refusal message)` when the
/// requested route resolves to a DIFFERENT render ENTITY than the known one — else
/// `None` (a first attach, or a same-entity re-attach, both free / idempotent).
///
/// The comparison is on the resolved ENTITY, NOT the raw route key, so two keys
/// that map to the SAME entity never conflict. Note what that
/// means: entity segments preserve CASE, so `cloud` vs `CLOUD` IS a different
/// entity and therefore IS a conflict — asserted by
/// `entity_override_conflict_arms_are_honest` below. Only the
/// tf/odom KNOBS still match case-insensitively. Refusing an inconsistent re-point stops
/// data accumulating under a NEW entity root while the shell's uncheck-tombstone
/// (DropEntity) still targets the OLD one — the zombie a detached topic left
/// rendering. Pure — oracle-tested.
fn entity_override_conflict(
    topic: &str,
    existing_route: Option<&str>,
    requested_route: &str,
) -> Option<String> {
    let existing = existing_route?;
    // Compare the entities the daemon would REPORT for a non-TFMessage payload, not
    // `route_for_input`'s raw answer: the tf arm collapses EVERY override to the viz
    // root, so two different overrides on a `/robot1/tf` compared equal and a
    // conflicting re-point was silently accepted (the response then reported the new
    // override while the sink kept rendering at the stored one). The kind is not
    // known here, so the comparison uses the reconciled form — which is an identity
    // for every topic that is not tf-named.
    let live = non_tf_entity_for(existing);
    let requested = non_tf_entity_for(requested_route);
    if live == requested {
        return None;
    }
    Some(format!(
        "topic '{topic}' is already attached under entity path '{live}', but this attach requests a \
         different entity '{requested}' — detach '{topic}' first to re-point it, or reuse '{live}'"
    ))
}

/// Fold `entities` into the run's EVER-LOGGED set. Pure apart
/// from the `&mut` — oracle-tested.
///
/// The set only ever GROWS: a detached topic's data stays in the rerun store (vizd
/// issues no tombstone — the uncheck one belongs to the Studio shell, and there are
/// shell-free detach paths), so its entity must stay excludable from any ancestor
/// topic's view for the rest of the run. Additive by construction, so a re-attach
/// is a no-op and a stale entry is harmless: `default_layout_excluding` only ever
/// excludes STRICT descendants.
fn record_logged_entities(
    ever: &mut std::collections::BTreeSet<String>,
    entities: impl IntoIterator<Item = String>,
) {
    ever.extend(entities);
}

/// The ever-logged set as a sorted snapshot — the exclusion list
/// `default_layout_excluding` consumes.
fn snapshot_logged_entities(ever: &std::collections::BTreeSet<String>) -> Vec<String> {
    ever.iter().cloned().collect()
}

/// Guardrail 5: the route-commit decision made under the tap-attach lock
/// (shared by local `attach` + remote `attach_remote`, so the conflict re-check +
/// the invariant are identical on both paths). Given the tap outcome
/// (`already_attached`) + the stability check against the CURRENTLY-stored route,
/// decide to:
/// - [`RouteCommit::Refuse`] — a conflicting override (a different resolved entity);
/// - [`RouteCommit::Store`] — a FRESH attach: record the requested route;
/// - [`RouteCommit::Keep`] — an AlreadyAttached re-attach: NEVER overwrite the
///   stored route (the stat must reflect the tap's ACTUAL key, set once at the first
///   attach — a diverging re-attach that reached AlreadyAttached raced past the early
///   check, and its key must not clobber the live one). Pure — oracle-tested.
enum RouteCommit {
    /// A conflicting override — refuse with this message.
    Refuse(String),
    /// A fresh attach — store the requested route key.
    Store,
    /// An AlreadyAttached re-attach — keep the existing stored route unchanged.
    Keep,
}

fn decide_route_commit(
    topic: &str,
    existing_route: Option<&str>,
    requested_route: &str,
    already_attached: bool,
) -> RouteCommit {
    if let Some(msg) = entity_override_conflict(topic, existing_route, requested_route) {
        return RouteCommit::Refuse(msg);
    }
    if already_attached {
        RouteCommit::Keep
    } else {
        RouteCommit::Store
    }
}

/// Guardrail 5: the `attach_remote` commit outcome under the tap lock —
/// distinguishes a CONFLICT refusal (refuse WITHOUT releasing the netd demand: the
/// demand is keyed by `(topic → robot)` and released by the SURVIVING attach's
/// `detach`, so releasing here would tear a still-used mirror down) from a TAP error
/// (refuse AND roll the demand back, since no tap survives).
enum RemoteCommit {
    /// The tap committed — `already_attached`.
    Ok(bool),
    /// A conflicting override — refuse, do NOT release the demand.
    Conflict(String),
    /// The tap open failed — refuse AND release the demand (rollback).
    TapErr(String),
}

/// A raw compose group before topic resolution — the topics, an
/// optional parsed role, and an optional group title.
type RawComposeGroup = (Vec<String>, Option<GroupRole>, Option<String>);

/// Validate a [`LayoutIntent`]'s SHAPE into the raw
/// `(topics, role, title)` groups the compiler consumes. EXACTLY one of `groups` /
/// `topics` must be set (a loud refusal on both-set / neither-set / any empty
/// list), and each group's `role` string is parsed loudly ([`GroupRole::from_wire`]).
/// Bare `topics` become ONE role-less group. PURE — oracle-tested.
fn parse_intent_groups(intent: &LayoutIntent) -> Result<Vec<RawComposeGroup>, String> {
    let out = match (&intent.groups, &intent.topics) {
        (Some(_), Some(_)) => {
            return Err(
                "compose_layout: provide EITHER 'groups' (role-tagged) OR 'topics' (bare \
                 automagic), not both"
                    .to_string(),
            )
        }
        (None, None) => {
            return Err(
                "compose_layout: provide 'groups' (role-tagged) or 'topics' (a bare topic list \
                 for full automagic)"
                    .to_string(),
            )
        }
        (Some(groups), None) => {
            if groups.is_empty() {
                return Err("compose_layout: 'groups' is empty — list at least one group".into());
            }
            let mut out = Vec::new();
            for g in groups {
                if g.topics.is_empty() {
                    return Err(
                        "compose_layout: a group has no topics — every group must list at \
                                least one topic"
                            .into(),
                    );
                }
                let role = match &g.role {
                    Some(r) => Some(GroupRole::from_wire(r).map_err(|e| e.to_string())?),
                    None => None,
                };
                out.push((g.topics.clone(), role, g.title.clone()));
            }
            out
        }
        (None, Some(topics)) => {
            if topics.is_empty() {
                return Err("compose_layout: 'topics' is empty — list at least one topic".into());
            }
            vec![(topics.clone(), None, None)]
        }
    };
    // Reject a topic listed more than ONCE across the whole
    // intent (a bare list OR across groups) — LOUD, before any attach side effect.
    // Each topic may appear at most once; a duplicate placement echo would confuse
    // the agent, and a topic in two role groups is an ambiguous placement.
    if let Some(dup) = first_duplicate_topic(&out) {
        return Err(format!(
            "compose_layout: topic '{dup}' is listed more than once — each topic may appear at \
             most once across the intent's groups/topics"
        ));
    }
    Ok(out)
}

/// The FIRST topic that appears more than once across all
/// of `groups` (in declaration order), or `None` when every topic is unique. Pure.
fn first_duplicate_topic(groups: &[RawComposeGroup]) -> Option<String> {
    let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for (topics, _, _) in groups {
        for t in topics {
            if !seen.insert(t.as_str()) {
                return Some(t.clone());
            }
        }
    }
    None
}

/// Validate a rerun-agnostic [`LayoutSpec`] into the inspectable
/// [`BlueprintPlan`] (the layout verb) — the PURE `spec → plan` half (the
/// `plan → rerun` half is [`cerulion_viz::blueprint::build_blueprint_msgs`], run
/// on the worker). Every failure is a loud, actionable [`LayoutError`]: an unknown
/// view/container kind names the supported set, and sizing knobs are checked
/// against the kind — `horizontal`/`vertical` shares are one-per-child, a `grid`'s
/// shares are one-per-COLUMN (so it requires `columns`), `tabs` take none, shares
/// must be finite > 0, and a grid's `columns` is bounded `1..=children.len()`.
/// Oracle-tested — never a silent drop.
fn build_plan(spec: &LayoutSpec) -> Result<BlueprintPlan, LayoutError> {
    Ok(BlueprintPlan {
        root: build_node(&spec.root)?,
        auto_views: spec.auto_views,
        // `set_blueprint` is a HAND-AUTHORED (power-user) layout — never
        // auto-decorated (it stays theirs). The `compose_layout` compiler AND the
        // built-in go2 default emit DECORATED plans; only a hand-authored
        // set_blueprint plan carries `decorate: false`.
        decorate: false,
    })
}

fn build_node(node: &LayoutNode) -> Result<PlanNode, LayoutError> {
    match node {
        LayoutNode::Container(c) => build_container(c),
        LayoutNode::View(v) => build_view(v),
    }
}

fn build_view(v: &ViewSpec) -> Result<PlanNode, LayoutError> {
    Ok(PlanNode::View(PlanView {
        kind: ViewKind::from_wire(&v.kind)?,
        name: v.name.clone(),
        origin: v.origin.clone(),
        // `set_blueprint` carries no per-view contents — keep rerun's default
        // `$origin/**`; `compose_layout` sets globs instead.
        contents: None,
    }))
}

fn build_container(c: &ContainerSpec) -> Result<PlanNode, LayoutError> {
    let kind = ContainerKind::from_wire(&c.kind)?;
    if c.children.is_empty() {
        return Err(LayoutError::EmptyContainer(kind.as_str()));
    }
    // `columns` is grid-only, and on a grid it must be a sane bound: 0 collapses
    // the grid to one column (the viewer silently clamps 0→1) and an unbounded
    // value OOMs the viewer's per-column `col_shares.resize`, so require
    // `1..=children.len()` (extra columns are pointless).
    if let Some(columns) = c.columns {
        if !matches!(kind, ContainerKind::Grid) {
            return Err(LayoutError::ColumnsOnNonGrid(kind.as_str()));
        }
        if columns == 0 {
            return Err(LayoutError::ZeroGridColumns);
        }
        if columns as usize > c.children.len() {
            return Err(LayoutError::TooManyGridColumns {
                columns,
                children: c.children.len(),
            });
        }
    }
    // `shares` size the container's SIZED AXIS. The count is validated PER KIND so
    // validation, the rerun `with_column_shares`/`with_row_shares` call, and the
    // docs all agree: a horizontal child IS a column and a vertical child IS a
    // row (one share per child), but a grid's shares are per-COLUMN, so they are
    // checked against `columns` (which must therefore be present). `tabs` stack —
    // they are unsized — so shares on tabs is a mistake.
    if let Some(shares) = &c.shares {
        match kind {
            ContainerKind::Tabs => return Err(LayoutError::SharesOnTabs),
            ContainerKind::Horizontal | ContainerKind::Vertical => {
                if shares.len() != c.children.len() {
                    return Err(LayoutError::SharesLenMismatch {
                        container: kind.as_str(),
                        shares: shares.len(),
                        children: c.children.len(),
                    });
                }
            }
            ContainerKind::Grid => {
                let Some(columns) = c.columns else {
                    return Err(LayoutError::GridSharesRequireColumns);
                };
                if shares.len() != columns as usize {
                    return Err(LayoutError::GridSharesLenMismatch {
                        shares: shares.len(),
                        columns,
                    });
                }
            }
        }
        if shares.iter().any(|&s| !s.is_finite() || s <= 0.0) {
            return Err(LayoutError::NonPositiveShare(kind.as_str()));
        }
    }
    let children = c
        .children
        .iter()
        .map(build_node)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(PlanNode::Container(PlanContainer {
        kind,
        children,
        name: c.name.clone(),
        shares: c.shares.clone(),
        columns: c.columns,
    }))
}

/// The netd DEMAND plane — the seam between vizd's remote arm and the
/// one-per-computer `cerulion-netd` daemon. `attach_remote` `demand`s a remote
/// topic's shared mirror; `detach` `release`s it. A DI trait so the daemon's
/// demand/release wiring is oracle-testable with a counting spy (no netd process);
/// production is [`NetdDemandPlane`]. `Send + Sync`: shared across every controller
/// thread via the daemon's `Ctx`.
pub trait DemandPlane: Send + Sync {
    /// Demand the shared mirror for `(robot, topic)`, validating inbound frames
    /// against `schema_hash`. Called ONCE per topic (the reserve-then-commit
    /// guarantees it). A `String` error is surfaced to the controller verbatim.
    fn demand(&self, robot: &str, topic: &str, schema_hash: u64) -> Result<(), String>;
    /// Release a previously-demanded `(robot, topic)` (the `detach` early release).
    fn release(&self, robot: &str, topic: &str) -> Result<(), String>;
    /// Query `robot`'s topic CATALOG over netd's shared session (the
    /// schema-less attach's type resolve). An `Err` means netd could not RUN the query
    /// (unreachable / not network-configured), so the caller LOUDLY degrades to its OWN
    /// transient zenoh session. Folds vizd's per-daemon catalog GET onto netd's ONE
    /// session.
    ///
    /// **An `Ok` is NOT on its own authoritative.** Reading
    /// "an `Ok` (possibly empty) is AUTHORITATIVE — netd reached the LAN" into it
    /// is exactly the claim the discovery marker exists to refuse: netd answers `Ok(vec![])`
    /// the moment its session finds nothing, INCLUDING before it has completed a
    /// discovery pass, and that empty is indistinguishable on the wire from a settled
    /// one. A caller that turns an empty answer into a user-visible ABSENCE must use
    /// [`Self::query_catalog_with_discovery`] and check the marker; this verb is the
    /// right call only when the caller has no absence claim to make (the same split
    /// `cerulion_netd::NetdClient::query_catalog` documents).
    fn query_catalog(&self, robot: &str) -> Result<Vec<CatalogReply>, String>;
    /// [`Self::query_catalog`] plus the [`DiscoveryState`] netd
    /// answered under — the single-robot twin of
    /// [`Self::query_catalog_all_with_discovery`], for the schema-less attach's type
    /// resolve (`catalog_resolve_type`), which renders an empty answer as the terminal
    /// "robot serves a catalog but names no ROS type for topic".
    ///
    /// Same fail-closed DEFAULT and same reason: [`DiscoveryState::NotConverged`] means
    /// "this plane cannot tell you", so every double and every plane predating this
    /// method withholds the absence claim rather than silently licensing it.
    ///
    /// **It returns netd's whole [`CatalogGather`]**, because the
    /// first-contact wait needs a third fact the pair `(catalogs, discovery)` cannot
    /// carry — `unsettled_for`, how long the ANSWERING daemon's plane has been running
    /// without ever settling. Without it a robot-less desk would pay the full
    /// convergence ceiling on every attach, forever (the cap, which is forwarded to the CLIENT on `RetryHint`).
    fn query_catalog_with_discovery(&self, robot: &str) -> Result<CatalogGather, String> {
        Ok(CatalogGather {
            catalogs: self.query_catalog(robot)?,
            discovery: DiscoveryState::NotConverged,
            // UNKNOWN, never a positive claim — it caps no wait.
            unsettled_for: None,
        })
    }
    /// Harvest EVERY announcing robot's catalog over netd's shared session
    /// (the `discover` verb's remote gather — `query_catalog(None)`). An `Ok`
    /// (possibly empty) is AUTHORITATIVE (netd reached the LAN; an empty vec = nobody
    /// answered, the desk shows a robots-less discover); an `Err` means netd could not
    /// run the query (unreachable / not network-configured), so `discover` LOUDLY
    /// degrades to a LOCAL-ONLY listing (never a hard failure). A REFUSED robot rides
    /// through as a [`CatalogReply`] carrying an `error` (surfaced explicitly).
    fn query_catalog_all(&self) -> Result<Vec<CatalogReply>, String>;
    /// [`Self::query_catalog_all`] plus the
    /// [`DiscoveryState`] netd answered under — the discriminator between "the LAN
    /// was searched and this robot does not serve that topic" and "netd has not
    /// completed a discovery pass, so an empty/partial answer proves NOTHING".
    ///
    /// Needed because `discover` makes an ABSENCE CLAIM with this gather: a mirror
    /// the catalog omits is DROPPED, which hides the row. An unconverged gather must
    /// never license that.
    ///
    /// The DEFAULT is deliberately [`DiscoveryState::NotConverged`], i.e. "this plane
    /// cannot tell you". Every test double and every plane that predates this method
    /// therefore FORBIDS the drop rather than silently authorising it — a double that
    /// wants to exercise the drop has to SAY so by overriding this. Fail-closed on the
    /// axis where a wrong answer deletes a row the user is looking at.
    ///
    /// Returns netd's whole [`CatalogGather`] — see the single-robot twin,
    /// [`Self::query_catalog_with_discovery`], for why the plane age rides along.
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        Ok(CatalogGather {
            catalogs: self.query_catalog_all()?,
            discovery: DiscoveryState::NotConverged,
            unsettled_for: None,
        })
    }
    /// Fetch `requested`'s `.msg`/YAML closure from `robot` over netd's
    /// shared session (the schema fetch + seed). `Ok` (possibly empty) is
    /// authoritative; an `Err` triggers the transient-session fallback.
    fn query_schema(&self, robot: &str, requested: &str) -> Result<Vec<SchemaReply>, String>;
    /// Ask which runs are LIVE on `robot` (or on every announcing
    /// robot, `None`) over netd's shared session — the REMOTE arm of the `runs`
    /// fold.
    ///
    /// Returns netd's whole [`RunsGather`] rather than a bare list, because the
    /// three facts it carries answer three different questions and the fold needs
    /// all of them: `replies` is who answered USABLY, `unusable` is who answered
    /// with bytes this binary could not decode (a REDEPLOY signal, never a
    /// silence), and `discovery` is whether netd's LAN discovery had converged —
    /// without which an empty answer proves nothing.
    ///
    /// # The DEFAULT is an `Err`, and that is the fail-closed direction
    ///
    /// A plane that cannot query runs has learned NOTHING about any robot, so the
    /// correct answer is "the remote arm could not be run" — which the fold surfaces
    /// as [`RunsResponse::degraded`](crate::protocol::RunsResponse::degraded), an
    /// explicit non-claim. An `Ok(empty)` default would instead let every test
    /// double and every plane predating this method assert "no robot is running
    /// anything", which is the confident lie this whole verb is shaped to refuse.
    fn query_runs(&self, _robot: Option<&str>) -> Result<RunsGather, String> {
        Err("this demand plane cannot query runs (it predates the `runs` verb)".to_string())
    }
}

/// The production demand plane — ONE persistent [`NetdClient`]
/// connection to `cerulion-netd` for the daemon's lifetime (spawning netd detached
/// on the first demand if it is not running). Every vizd demand rides this ONE
/// connection, so netd sees vizd as ONE consumer; dropping the plane (at daemon
/// shutdown) closes the connection, releasing every remaining demand (the crash-safe
/// refcount). The client is behind a `Mutex` (its `demand`/`release` take `&mut
/// self`, and controller threads share it) and opened LAZILY on the first demand, so
/// a viz daemon that only ever serves LOCAL attaches never connects to netd.
pub struct NetdDemandPlane {
    client: Mutex<Option<NetdClient>>,
    /// `Some(socket)` targets a SPECIFIC control socket (the e2e's in-process netd);
    /// `None` uses the well-known default path (production, first-consumer-spawns).
    socket: Option<PathBuf>,
}

impl NetdDemandPlane {
    /// A demand plane targeting the DEFAULT well-known netd socket (opened lazily on
    /// the first demand, spawning netd if it is not running). The production plane.
    pub fn new() -> Self {
        Self {
            client: Mutex::new(None),
            socket: None,
        }
    }

    /// A demand plane targeting a SPECIFIC netd control `socket` — the e2e seam
    /// (point vizd at an in-process netd). Still lazy: connects on the first demand.
    pub fn with_socket(socket: PathBuf) -> Self {
        Self {
            client: Mutex::new(None),
            socket: Some(socket),
        }
    }

    /// Run `f` against the lazily-connected client. Connects (spawning netd if
    /// needed) on the first call; a connect failure is surfaced (never silently
    /// dropped). Poison-tolerant (a panicked controller must not wedge the plane).
    fn with_client<R>(
        &self,
        f: impl FnOnce(&mut NetdClient) -> Result<R, ClientError>,
    ) -> Result<R, String> {
        let mut guard = self.client.lock().unwrap_or_else(|e| e.into_inner());
        if guard.is_none() {
            let client = match &self.socket {
                Some(sock) => NetdClient::connect_or_spawn_at(sock.clone()),
                None => NetdClient::connect_or_spawn(),
            };
            *guard = Some(client.map_err(|e| e.to_string())?);
        }
        let client = guard.as_mut().expect("just connected");
        match f(client) {
            Ok(r) => Ok(r),
            Err(e) => {
                // A CONNECTION-level error (broken pipe / a protocol desync — e.g.
                // netd exited or was killed) leaves the client unusable, so DROP it:
                // the NEXT demand reconnects (re-spawning netd if needed), instead of
                // this long-lived daemon's demand plane staying permanently wedged on
                // a dead socket (the crash-recovery model — netd is
                // recovery-stateless). A `Netd(response)` error is a VALID refusal
                // (schema conflict, …) — the connection is fine, so keep it.
                if should_drop_client_on_error(&e) {
                    *guard = None;
                }
                Err(e.to_string())
            }
        }
    }
}

impl Default for NetdDemandPlane {
    fn default() -> Self {
        Self::new()
    }
}

impl DemandPlane for NetdDemandPlane {
    fn demand(&self, robot: &str, topic: &str, schema_hash: u64) -> Result<(), String> {
        self.with_client(|c| c.demand(robot, topic, schema_hash).map(|_| ()))
    }
    fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
        self.with_client(|c| c.release(robot, topic).map(|_| ()))
    }
    fn query_catalog(&self, robot: &str) -> Result<Vec<CatalogReply>, String> {
        // Route the catalog GET through the ONE persistent netd connection (netd runs
        // it over the machine's ONE zenoh session). A connection-level error drops the
        // client so the next call reconnects (`with_client`); the caller falls back.
        self.with_client(|c| c.query_catalog(Some(robot)))
    }

    /// The REAL discovery state for a SINGLE robot's catalog, from
    /// netd's discovery-aware verb — the one plane that can actually answer it.
    fn query_catalog_with_discovery(&self, robot: &str) -> Result<CatalogGather, String> {
        // ONE round trip, and the client mutex is taken + DROPPED inside
        // `with_client`. The first-contact wait loop lives above this trait, so a wait
        // that polls N times acquires the lock N times and holds it across NOTHING —
        // see `crate::convergence`. Calling `NetdClient::query_catalog_converged` here
        // would run the whole wait under one guard and stall the demand plane.
        self.with_client(|c| c.query_catalog_with_discovery(Some(robot)))
    }
    fn query_catalog_all(&self) -> Result<Vec<CatalogReply>, String> {
        // The LAN-wide gather — `query_catalog(None)` harvests every
        // announcing robot over the SAME one persistent netd connection. A
        // connection-level error drops the client (`with_client`) so the next call
        // reconnects; `discover` degrades to local-only on the returned `Err`.
        self.with_client(|c| c.query_catalog(None))
    }

    /// The REAL discovery state, from netd's
    /// discovery-aware verb — the one plane that can actually answer it.
    fn query_catalog_all_with_discovery(&self) -> Result<CatalogGather, String> {
        // See the single-robot twin — one round trip, one lock acquisition.
        self.with_client(|c| c.query_catalog_with_discovery(None))
    }
    fn query_schema(&self, robot: &str, requested: &str) -> Result<Vec<SchemaReply>, String> {
        self.with_client(|c| c.query_schema(Some(robot), requested))
    }

    /// The REAL runs gather, over the ONE persistent netd connection.
    ///
    /// `query_runs_with_discovery` is netd's NON-looping verb — one round trip, and
    /// the client mutex is taken and DROPPED inside `with_client`. There is
    /// deliberately no `_converged` sibling to reach for on this verb (netd does not
    /// ship one), which is the same conclusion the catalog seams reached the hard
    /// way: a multi-second wait behind `&mut self` would hold this plane's one lock
    /// for its whole ceiling and push the reply past `viz_client`'s 5 s read
    /// deadline.
    fn query_runs(&self, robot: Option<&str>) -> Result<RunsGather, String> {
        self.with_client(|c| c.query_runs_with_discovery(robot))
    }
}

/// Whether a demand/release [`ClientError`] should DROP the persistent
/// netd client so the next call reconnects (the crash-recovery reset). A
/// CONNECTION-level error ([`ClientError::Io`] broken pipe / [`ClientError::Protocol`]
/// desync — netd exited/killed) leaves the client unusable → drop + reconnect; a
/// [`ClientError::Netd`] response is a VALID refusal (schema conflict, …) → the
/// connection is fine, keep it. (`Connect`/`Spawn` arise before `f(client)` runs, so
/// they never reach here.) PURE — oracle-tested.
fn should_drop_client_on_error(e: &ClientError) -> bool {
    matches!(e, ClientError::Io(_) | ClientError::Protocol(_))
}

/// PURE extraction of `topic`'s qualified ROS type from gathered
/// catalogs — the FIRST catalog entry matching `topic` that carries a schema name
/// (shared by the netd + transient resolve paths). `None` when no catalog names a
/// type for `topic`. Oracle-tested.
fn resolve_type_from_catalogs(catalogs: &[CatalogReply], topic: &str) -> Option<String> {
    catalogs.iter().find_map(|cat| {
        cat.entries.iter().find_map(|e| {
            if e.topic == topic {
                e.schema_name.clone()
            } else {
                None
            }
        })
    })
}

/// PURE — the SET of robots whose gathered catalog CURRENTLY names `topic`
/// (sorted, de-duplicated; a REFUSED catalog serves nothing, so it never contributes).
/// The oracle for [`resolve_remote_attach_target`]'s reachability decision. Oracle-tested.
fn robots_serving_topic(catalogs: &[CatalogReply], topic: &str) -> Vec<String> {
    let mut robots: Vec<String> = catalogs
        .iter()
        .filter(|c| c.error.is_none())
        .filter(|c| c.entries.iter().any(|e| e.topic == topic))
        .map(|c| c.robot.clone())
        .collect();
    robots.sort();
    robots.dedup();
    robots
}

/// The resolution of a no-`robot` attach on a topic that is NOT locally
/// visible — either the robot to re-demand from, or a REMOTE-aware error (NEVER the
/// local graph-YAML "does not exist" for a topic known to be remote).
#[derive(Debug, Clone, PartialEq, Eq)]
enum RemoteAttachResolution {
    /// Re-demand the topic's mirror from this robot (a tombstone hit, or the unique
    /// robot the catalog says serves it).
    Robot(String),
    /// A remote-aware, actionable error message.
    Error {
        /// The message rendered to the human.
        message: String,
        /// Is this a NOT-FOUND-**YET** that re-asking could change?
        ///
        /// TRUE for every arm whose verdict is "no robot in THIS answer serves THIS
        /// topic" — including the SETTLED one. A settled gather is authoritative about
        /// the LAN *right now*, and discovery is open-world: a robot can join at any
        /// moment, which is exactly what that arm's own message tells the user to wait
        /// for ("once its robot is discovered"). FALSE for the AMBIGUITY arm, where
        /// several robots serve the topic and re-asking changes nothing — the user has
        /// to pick.
        ///
        /// This is the SEAM's question, deliberately not the gather's emptiness: on a
        /// two-robot LAN where A answered and B is booting, the gather is non-empty and
        /// netd reports `Settled` (its `ever_settled` bit latches permanently on the
        /// first non-empty gather) while B's topic is genuinely not there yet.
        retryable: bool,
    },
}

/// PURE — decide where a no-`robot` attach of a not-locally-visible `topic`
/// should re-demand from, given the session provenance TOMBSTONE (`remembered`, if the
/// topic was attached from a robot before) and the `serving` set the catalog gather
/// reports. Precedence + arms (each oracle-tested):
///
/// - TOMBSTONE first: the desk attached this topic from a specific robot before.
///   - the remembered robot still serves it → re-demand from it (the round-trip heal);
///   - the remembered robot no longer serves it → a REMOTE-aware "previously served
///     but unreachable" error (never the local "does not exist"), hinting any robots
///     that DO serve it now so the user can re-attach from one.
/// - no tombstone → CATALOG auto-resolve:
///   - exactly ONE robot serves it → re-demand from it (full automagic);
///   - MULTIPLE robots serve it → an ambiguity error NAMING the candidates;
///   - NONE serves it → an explicit "not local + no reachable robot" error.
///
/// Two of those arms — the tombstone MISS and the empty `serving` set — are
/// ABSENCE CLAIMS about the LAN, so they are gated on the `discovery` marker
/// exactly like `fold_streaming_mirrors_into_robots`'s leftover-mirror drop. A
/// `NotConverged` gather means netd has not completed a discovery pass: an empty or
/// partial answer is indistinguishable on the wire from a settled one, so it is NOT
/// evidence of absence. Those two arms then report the answer as UNKNOWN instead of
/// TERMINAL ("no reachable robot" / "unreachable"), which is the whole point: the desk
/// must never render a confident "topic not found" at the exact moment the user asked
/// for a topic that exists.
///
/// **The wording states UNKNOWN and hedges the retry, because `NotConverged` is not
/// necessarily transient.** netd's `DiscoveryMaturity` latches `settled` only on a
/// NON-EMPTY gather and never resets, so a desk that never hears a robot at all — a
/// robot-less dev machine, a robot on the wrong VLAN, blocked multicast — is
/// `NotConverged` for the daemon's whole lifetime. A message promising "retry in a
/// moment" would be permanently unactionable there, converting a correct reachability
/// diagnosis into wrong timing advice. So both arms follow the shipped
/// `cerulion_cli_engine::topic_cmd::discovery_not_converged_message` pattern: name only
/// what is actually known, say UNKNOWN explicitly, and pair the retry with the checks
/// that matter if it keeps failing. Neither claims WHY nothing answered (powered off,
/// wrong network, still booting) — the desk cannot tell those apart, and netd's own
/// cold-start `warn!` carries the discriminator it does have.
///
/// The other arms are untouched, and deliberately: a gather that FOUND a robot serving
/// the topic is POSITIVE evidence, and an unconverged gather can only ever under-report,
/// so a cold-start desk that already sees the robot still attaches immediately. The
/// multi-robot ambiguity error likewise claims nothing about absence — it names what was
/// found and asks the user to pick.
///
/// **The retry clause names no duration.** None of the three UNKNOWN-verdict messages (the two
/// here and `catalog_absence_verdict`'s) promises that "discovery often converges
/// within a second or two": real-LAN
/// convergence was MEASURED at ~3–13 s, and vizd would contradict the claim twice over:
/// the attach seams WAIT through convergence before rendering any of these, so by the
/// time a user reads it the daemon has already spent seconds it was told not to need.
/// The advice to retry STAYS — the rule is to claim no duration, not to stop
/// telling the user to try again. The shared head is [`RETRY_CLAUSE`]; each site keeps
/// its own remedy tail.
fn resolve_remote_attach_target(
    topic: &str,
    remembered: Option<&str>,
    serving: &[String],
    discovery: DiscoveryState,
) -> RemoteAttachResolution {
    // Only a SETTLED gather may license an absence claim (the `may_drop` gate
    // in `fold_streaming_mirrors_into_robots`, at a second call site).
    let settled = discovery == DiscoveryState::Settled;
    if let Some(robot) = remembered {
        if serving.iter().any(|r| r == robot) {
            return RemoteAttachResolution::Robot(robot.to_string());
        }
        // Known-remote, but the remembered robot is not in the gather. NEVER the local
        // "does not exist" — a remote-aware error that keeps the provenance accurate.
        // Whatever the gather DID find is positive evidence, so it is hinted either way.
        let alt = if serving.is_empty() {
            String::new()
        } else {
            format!(" (other robots serving it now: {})", serving.join(", "))
        };
        if !settled {
            // The "nothing answered" clause is TRUE only of an EMPTY
            // gather. Interpolating `alt` into it produced a sentence that
            // contradicted itself in one breath — "nothing on the network answered
            // … (other robots serving it now: go2)". A partial gather gets the
            // weaker, true claim instead: discovery is still running, so this view
            // may be incomplete.
            let unconverged = if serving.is_empty() {
                "nothing on the network answered cerulion-netd (no robot was discovered, or \
                 none served its catalog in time)"
                    .to_string()
            } else {
                format!(
                    "cerulion-netd's discovery has not converged, so this view of the \
                     network may be incomplete{alt}"
                )
            };
            return RemoteAttachResolution::Error {
                message: format!(
                    "robot '{robot}' previously served '{topic}', and {unconverged} — so whether \
                     '{robot}' still serves '{topic}' is UNKNOWN (this is not a claim that it \
                     stopped). {RETRY_CLAUSE}, or re-attach '{topic}' from a robot in the ROBOTS \
                     list."
                ),
                retryable: true,
            };
        }
        return RemoteAttachResolution::Error {
            message: format!(
                "robot '{robot}' previously served '{topic}' but is unreachable (or no longer \
                 announces it){alt} — reconnect the robot, or re-attach '{topic}' from a robot in \
                 the ROBOTS list"
            ),
            retryable: true,
        };
    }
    match serving {
        [] if !settled => RemoteAttachResolution::Error {
            message: format!(
                "'{topic}' is not a local topic, and nothing on the network answered \
                 cerulion-netd (no robot was discovered, or none served its catalog in time) — so \
                 whether '{topic}' exists is UNKNOWN (this is not a claim that it is missing). \
                 {RETRY_CLAUSE}, and use the ROBOTS list to see which robots are discoverable from \
                 here."
            ),
            retryable: true,
        },
        [] => RemoteAttachResolution::Error {
            message: format!(
                "'{topic}' is not a local topic and no reachable robot serves it — attach it from \
                 the ROBOTS list once its robot is discovered"
            ),
            // The message itself says "once its robot is discovered" — that IS a
            // re-ask instruction, so the wire must say so too.
            retryable: true,
        },
        [one] => RemoteAttachResolution::Robot(one.clone()),
        many => RemoteAttachResolution::Error {
            message: format!(
                "topic '{topic}' is served by robots: {} — specify which by attaching '{topic}' from \
                 that robot's group in the ROBOTS list",
                many.join(", ")
            ),
            // NOT retryable: several robots serve it and re-asking changes nothing —
            // the user has to pick one.
            retryable: false,
        },
    }
}

/// PURE — build the `discover` ROBOTS section from the gathered catalogs.
/// One [`DiscoveredRobot`] per catalog reply, the robots sorted by identity
/// (deterministic; netd returns one reply per distinct announcing robot). A REFUSED
/// catalog (`error: Some`) rows with its reason + NO topics — surfaced
/// explicitly, never a silent empty. A served catalog's topics are filtered against
/// `local_names` (a topic already listed/attachable locally — a genuine local
/// producer OR an attached mirror — is not re-listed as remote: the "one data source
/// = one topic" surface), then each remaining entry is mapped to a remote
/// [`DiscoveredEntry`]. Oracle-tested.
fn build_discovered_robots(
    catalogs: Vec<CatalogReply>,
    local_names: &std::collections::BTreeSet<String>,
) -> Vec<DiscoveredRobot> {
    let mut robots: Vec<DiscoveredRobot> = catalogs
        .into_iter()
        .map(|cat| {
            // A refusal serves nothing (mutually exclusive with entries).
            if let Some(reason) = cat.error {
                return DiscoveredRobot {
                    robot: cat.robot,
                    topics: Vec::new(),
                    error: Some(reason),
                };
            }
            let mut topics: Vec<DiscoveredEntry> = cat
                .entries
                .into_iter()
                .filter(|e| !local_names.contains(&e.topic))
                .map(|e| remote_discovered_entry(&cat.robot, e))
                .collect();
            topics.sort_by(|a, b| a.topic.cmp(&b.topic));
            DiscoveredRobot {
                robot: cat.robot,
                topics,
                error: None,
            }
        })
        .collect();
    robots.sort_by(|a, b| a.robot.cmp(&b.robot));
    robots
}

/// Studio parity, PURE: fold the desk-local STREAMING MIRRORS into the
/// `discover` ROBOTS section. Each `(robot, entry)` is a re-injected mirror of a
/// remote robot's topic (already tapped + resolved locally) that MUST be attributed
/// to its origin robot rather than presenting as a LOCAL topic. A robot row already
/// carrying the topic (from netd's catalog gather) WINS — the streaming entry is a
/// no-op there (no duplicate); otherwise the entry is appended to its robot's row,
/// CREATING a served row for a robot netd's catalog gather never listed (the
/// network-free guarantee: a mirror stays attributed to its robot even when the LAN
/// catalog is unreachable).
///
/// A row's catalog was REFUSED (`error.is_some()`) is the one subtlety: the
/// sidebar renders an error row's reason INSTEAD of its topics, so appending a mirror
/// onto a refused row would (a) break `build_discovered_robots`'s "refusal serves
/// nothing" invariant and (b) make the live mirror INVISIBLE. A flowing mirror is
/// GROUND TRUTH — proof of a working data path from that robot right now — so it
/// SUPERSEDES the (necessarily stale) catalog refusal: the row is converted to SERVED
/// carrying the mirror, so no row ever renders with BOTH a reason and topics. This
/// state is essentially unreachable with the current netd (the mirror demand and the
/// catalog query share ONE `DemandAuthorizer`, so a robot that authorized the mirror
/// also authorizes its catalog); the guard keeps the invariant coherent if it ever
/// arises (e.g. auth revoked after the mirror was established, before netd's teardown).
///
/// Robots + each augmented row's topics are re-sorted for a deterministic response.
/// Oracle-tested.
fn fold_streaming_mirrors_into_robots(
    mut robots: Vec<DiscoveredRobot>,
    streaming: Vec<(String, DiscoveredEntry)>,
    discovery: DiscoveryState,
) -> Vec<DiscoveredRobot> {
    // **The drop gate reads a SNAPSHOT, never live row state.**
    //
    // The gate means "the CATALOG listed other topics but not this one". It must not
    // ask that question of `row.topics` and `row.error` — both of which THIS LOOP
    // MUTATES (it pushes each folded mirror into `row.topics` and clears `row.error`
    // on a refusal). `streaming` carries one tuple per mirrored topic with the robot
    // REPEATED, so from the second mirror of a robot onward the gate would read the
    // fold's own earlier iteration and every carve-out would collapse:
    //
    //   3 tapped go2 mirrors, one of them Idle, catalog gather fails ⇒ `robots` is
    //   empty ⇒ iteration 1 CREATES the row via the `None` arm with `topics = [m1]`
    //   ⇒ iteration 2 sees a non-refused, NON-EMPTY row ⇒ drops the Idle mirror.
    //
    // A row whose tap is attached and whose view is live would vanish — precisely the
    // "worse error" this fold must never commit. All three carve-outs
    // (unreachable robot, cold-start empty catalog, refused catalog) degrade the same
    // way at N >= 2, with exactly one mirror surviving and the survivor decided by
    // iteration order.
    //
    // So the catalog's own answer is captured BEFORE the loop and the gate reads only
    // that. Refusal is folded INTO the snapshot as an EMPTY topic set rather than a
    // second flag: a refusal serves nothing by construction (`CatalogReply::error` is
    // mutually exclusive with non-empty entries), so "empty" already forbids the drop
    // and a separate `!refused` term would be redundant — encoding it structurally
    // means it cannot drift.
    let catalog_topics: BTreeMap<String, std::collections::BTreeSet<String>> = robots
        .iter()
        .map(|r| {
            let topics = if r.error.is_some() {
                std::collections::BTreeSet::new()
            } else {
                r.topics.iter().map(|e| e.topic.clone()).collect()
            };
            (r.robot.clone(), topics)
        })
        .collect();

    // Dropping a mirror is an ABSENCE CLAIM, and the discovery marker exists
    // because an empty/partial gather can mean "netd has not finished looking". Only
    // a SETTLED gather may license it. Without this gate the safety is incidental — an
    // unconverged gather usually yields no robots at all, so the `None` arm happens
    // to keep the mirror — which is not the same as being correct.
    let may_drop = discovery == DiscoveryState::Settled;

    for (robot, entry) in streaming {
        match robots.iter_mut().find(|r| r.robot == robot) {
            Some(row) => {
                // A live mirror supersedes a stale catalog REFUSAL for the row: convert
                // to served (drop the reason + any [] topics) before attributing the
                // mirror, so the invariant "a row is EITHER refused (no topics) OR
                // served" holds and the flowing mirror is never hidden. The snapshot
                // above already recorded the refusal, so this mutation cannot affect
                // the gate.
                if row.error.is_some() {
                    row.error = None;
                    row.topics.clear();
                }
                // Already listed for this robot (by the catalog, or by an earlier
                // mirror of the same robot) ⇒ no duplicate row.
                //
                // The catalog's row WINS, but the mirror's LIVE
                // LAYOUT METADATA is carried onto it. Dropping the mirror entry
                // whole would discard the one thing only THIS DESK knows — whether it is
                // rendering the topic — so on the Studio flagship flow (attach a
                // robot topic → netd mirrors it → the desk renders it → the sidebar
                // re-polls `discover`) the surviving row would report the PRE-DROP
                // companion while `status`/`list`/both attach replies report the
                // applied set. One run, two answers, on the surface the sidebar polls.
                //
                // The catalog row's own archetype is inferred from the SCHEMA NAME
                // alone (`remote_discovered_entry` → `classify_schema`), while the
                // mirror's comes from a real local frame — so the pair is carried
                // only when the two AGREE. Splicing one archetype's view kinds onto
                // a row naming another is a NEW inconsistency, worse than the stale
                // one it would fix.
                //
                // **The DOMINANT refuse class is `None` vs `Some(kind)`, not two
                // different kinds** — and it includes the flagship camera.
                // `classify_schema` maps a fixed table of ROS types, so EVERY
                // unmapped or vendor-custom schema yields `None` there while the
                // mirror side classifies from real bytes: the Go2's
                // `unitree_go/Go2FrontVideoData` is not in that table, and its
                // frames classify `VideoStream` by CONTENT. Stated exactly, the
                // consequence for that row is ABSENCE rather than wrong data — the
                // catalog row carries `null` components/view_kinds (the same
                // null-not-empty convention a silent local topic uses), so
                // `discover` reports "not yet known" for a topic this desk is
                // rendering. That is the accepted residual, and it is strictly the
                // safer of the two failures: a row claiming an archetype's
                // placement while naming another archetype cannot be reconciled by
                // a client at all.
                //
                // SCOPE, so the boundary is visible to the next reader: the layout
                // pair moves WITH its cause. Carrying `view_kinds` while
                // leaving `representation` absent would make the row report the
                // CONSEQUENCE without the CAUSE — exactly what
                // `DiscoveredEntry::representation`'s own contract exists to
                // prevent, since these view kinds are computed THROUGH the choice.
                // The mirror entry derived it from the same `representation_for`
                // lookup, and `None` on `auto` keeps the wire byte-identical for a
                // desk where nobody has chosen anything.
                if let Some(existing) = row.topics.iter_mut().find(|e| e.topic == entry.topic) {
                    if existing.archetype == entry.archetype {
                        existing.components = entry.components;
                        existing.view_kinds = entry.view_kinds;
                        existing.representation = entry.representation;
                    }
                    continue;
                }
                if may_drop && should_drop_leftover_mirror(&catalog_topics, &robot, &entry) {
                    continue;
                }
                row.topics.push(entry);
                row.topics.sort_by(|a, b| a.topic.cmp(&b.topic));
            }
            // A robot netd's catalog gather never listed (unreachable / not
            // announcing) still surfaces its live mirror — a served row. The catalog
            // said NOTHING about this robot, so there is no absence to claim; the
            // snapshot lookup in `should_drop_leftover_mirror` reaches the same verdict
            // for the SECOND mirror of such a robot, the arm a gate reading live row state gets wrong.
            None => robots.push(DiscoveredRobot {
                robot,
                topics: vec![entry],
                error: None,
            }),
        }
    }
    robots.sort_by(|a, b| a.robot.cmp(&b.robot));
    robots
}

/// Is this streaming mirror a LEFTOVER the sidebar should stop showing?
///
/// PURE, and reading only the pre-loop catalog SNAPSHOT — see the snapshot note in
/// [`fold_streaming_mirrors_into_robots`] for what happens when the equivalent
/// question is asked of live row state.
///
/// **The robot's catalog is the registration authority for its own topics — not this
/// desk's mirror slot.** The motivating case is a SERVE SWAP: the operator changes the
/// robot's graph, so the old topic's publisher dies. But `cerulion-netd` holds the
/// desk-local mirror for as long as anything demands it, and the mirror's re-injector
/// owns the local publisher slot — so every desk-side signal (`producer_count`, local
/// visibility) keeps reading "present" for a topic the robot stopped serving. A topic
/// accounted for nowhere is what hides the row.
///
/// The drop needs POSITIVE evidence on BOTH axes, because either alone is wrong:
///
/// * the CATALOG must have listed OTHER topics for this robot and not this one. A
///   robot absent from the gather said nothing; a REFUSED catalog serves nothing by
///   definition (folded in as an empty set above); an EMPTY served catalog is
///   deliberately not authoritative, since the discovery marker exists because a cold-start empty
///   is indistinguishable on the wire from a settled one.
/// * this desk's OWN tap must say the mirror is not delivering (`liveness_state`
///   present and not `Streaming`). A
///   gather can be STALE while the mirror is genuinely flowing, and hiding a row that
///   is carrying live data is the worse error. **Data beats a catalog.** UNKNOWN
///   liveness (nobody tapped it) is not evidence and keeps the mirror.
fn should_drop_leftover_mirror(
    catalog_topics: &BTreeMap<String, std::collections::BTreeSet<String>>,
    robot: &str,
    entry: &DiscoveredEntry,
) -> bool {
    let Some(served) = catalog_topics.get(robot) else {
        // The robot never answered — no absence to claim.
        return false;
    };
    if served.is_empty() || served.contains(&entry.topic) {
        return false;
    }
    entry
        .liveness_state
        .is_some_and(|state| state != cerulion_core::LivenessState::Streaming)
}

/// PURE — map one served [`CatalogEntry`] to a REMOTE [`DiscoveredEntry`].
/// `schema` is the catalog's qualified type NAME (`None` when the robot named none);
/// the archetype hint is inferred from that NAME ALONE (no frame — [`classify_schema`]),
/// so a topic renders a known family before any data crosses the network (an unmapped
/// / unnamed type yields `None` archetype + `null` components/view_kinds, the same
/// null-not-empty convention a silent local topic uses); `entity` is derived
/// deterministically from the topic name (as for a local topic); `robot` self-attributes
/// so attach `demand`s the mirror. Oracle-tested via [`build_discovered_robots`].
fn remote_discovered_entry(robot: &str, entry: CatalogEntry) -> DiscoveredEntry {
    let archetype_kind = entry.schema_name.as_deref().and_then(classify_schema);
    // A known residual, stated: this is a REMOTE catalog row built by a pure
    // function with no access to the choice map, so a remembered override on a
    // remote topic reads its AUTOMAGIC view kinds here. The attach reply and
    // `list` are correct from the moment it is attached, which is the only state
    // in which the choice changes what renders.
    //
    // **These blanks are the correct answer HERE and are
    // OVERWRITTEN downstream when they are not.** A catalog row describes a topic
    // as the ROBOT reports it, so no live rendering signal exists for it — but if
    // this desk is MIRRORING and rendering that same topic,
    // `fold_streaming_mirrors_into_robots` carries the mirror's live
    // `(components, view_kinds)` onto this row, TOGETHER WITH the `representation`
    // they were computed through (so the row never reports the consequence without
    // its cause). So the residual above is scoped to a row whose live metadata the
    // fold did NOT carry — one this desk is not mirroring, OR one the archetype
    // gate refused (the dominant refuse class below: catalog `None` vs mirror
    // `Some(kind)`, where the row keeps the catalog's view_kinds too and stays
    // internally consistent). See that fold for why the carry is gated on the two
    // archetypes agreeing, and for what that gate costs on an unmapped schema.
    let (components, view_kinds) = layout_metadata(
        archetype_kind,
        Representation::Auto,
        Vec::new(),
        RenderProof::default(),
    );
    let archetype = archetype_kind.map(|k| archetype_name(k).to_string());
    let entity = reported_entity_for(&route_key_for_topic(&entry.topic, None), archetype_kind);
    DiscoveredEntry {
        // Absent for the same reason `view_kinds` reads automagic here
        // — this pure fn has no access to the choice map (see the note above).
        // Like those view kinds, this is OVERWRITTEN by
        // `fold_streaming_mirrors_into_robots` when this desk mirrors the topic
        // AND the archetype gate lets the carry fire — cause and consequence
        // travel together or not at all — so that residual still applies to any row the carry
        // did not reach: one this desk is not rendering, or one the gate refused
        // (the flagship camera's shape; neither half moves there, so the row
        // stays internally consistent).
        representation: None,
        topic: entry.topic,
        schema: entry.schema_name,
        archetype,
        entity,
        components,
        view_kinds,
        robot: Some(robot.to_string()),
        // The robot's publisher-PRESENCE count, carried verbatim. Kept
        // because it still answers a real question ("is a port registered") —
        // but see `liveness` below: on a `ros2 attach` robot this reads `Some(1)`
        // for a dead route too, which is why it cannot be the liveness
        // affordance on its own.
        producer_count: entry.producer_count,
        // The robot's DATA-FLOW observation, carried verbatim. `None`
        // (an old robot, observation disabled, a WAN-served catalog, or a tap
        // that never attached) is UNKNOWN and renders as it did before, never dimmed.
        liveness: entry.liveness,
        // Classify here, once, from the shared oracle-tested thresholds, so the
        // sidebar never re-derives them. `wire_state` (not `state`) so an UNKNOWN
        // classification becomes ABSENCE — the ONE wire encoding of unknown, see
        // `DiscoveredEntry::liveness_state`.
        liveness_state: entry.liveness.and_then(|l| l.wire_state()),
        // ALWAYS absent here, and that is the correct answer. This row
        // describes a topic on a ROBOT that this desk has not attached, so no
        // frame of it has ever reached this build's walker — "I cannot decode it"
        // is a claim only a received frame can support. The verdict appears the
        // moment the topic is attached and its frames are polled.
        undecodable: None,
    }
}

/// PURE — should `catalog_resolve_type`'s "no type for this topic" answer be
/// DOWNGRADED from the terminal [`catalog_resolve_error`], and to what?
///
/// `Some(msg)` = do not make the terminal claim, say this instead. `None` = the
/// terminal claim is earned; the caller uses [`catalog_resolve_error`] verbatim.
///
/// The decision turns on TWO axes — the marker AND whether the gather came
/// back empty — because each wording is only true in its own quadrant:
///
/// | gather | `NotConverged` | `Settled` |
/// |---|---|---|
/// | **empty** | the robot did not answer AND discovery has not converged ⇒ UNKNOWN | the robot did not answer ⇒ [`catalog_no_answer_error`] |
/// | **non-empty** | discovery has not converged, so this view may be incomplete ⇒ UNKNOWN | `None` — the terminal claim is EARNED |
///
/// # What the single-robot verb may NOT say
///
/// Reusing the FAN-OUT path's wording — "nothing on the network answered
/// cerulion-netd (no robot was discovered, or none served its catalog in time)" — in
/// both `NotConverged` quadrants is wrong. It is false in both:
///
/// * **Non-empty gather.** The robot demonstrably ANSWERED; saying nothing did
///   contradicts the caller's own input. Reachable permanently, not just at cold
///   start: `NetdClient::trust_reported_discovery` downgrades an earlier daemon's
///   `Settled` to `NotConverged` while its catalogs come back full, and netd is
///   spawn-once.
/// * **"no robot was discovered" — on ANY gather.** This verdict is fed exclusively by
///   netd's SINGLE-ROBOT GET, which runs no announce harvest at all (see
///   `cerulion_netd::query`: the announce discriminator there is UNOBSERVED, never a
///   claimed "no robot was discovered", oracle-pinned by
///   `announce_observation_label_never_claims_what_was_not_observed`). The disjunct is
///   unknowable by construction here, and the `topic_cmd` wording this follows avoids
///   a flat "no robot was discovered … check the power" for the same reason:
///   it is FALSE in the partial case.
///
/// The direction matches the fan-out path — every `NotConverged` quadrant errs UNKNOWN and
/// hedges the retry (a desk that never hears a robot is `NotConverged` for the
/// daemon's whole lifetime). Only the CLAIM differs. Oracle-tested.
fn catalog_absence_verdict(
    robot: &str,
    topic: &str,
    catalogs: &[CatalogReply],
    discovery: DiscoveryState,
) -> Option<String> {
    let unconverged_tail = format!(
        "so whether '{robot}' serves '{topic}' is UNKNOWN (this is not a claim that it does \
         not). {RETRY_CLAUSE}."
    );
    match (discovery == DiscoveryState::Settled, catalogs.is_empty()) {
        // Discovery still running AND nothing came back: state BOTH facts, and only
        // those two — this verb never looked at the announce space.
        (false, true) => Some(format!(
            "could not learn the type of '{topic}': robot '{robot}' did not answer a schema \
             catalog, and cerulion-netd's discovery has not converged — {unconverged_tail}"
        )),
        // Discovery still running but the robot DID answer: the only weakness
        // is that the view may be incomplete.
        (false, false) => Some(format!(
            "could not learn the type of '{topic}' from robot '{robot}': cerulion-netd's \
             discovery has not converged, so this view of the network may be incomplete — \
             {unconverged_tail}"
        )),
        // Discovery ran and this robot answered nothing. A genuine absence — but NOT
        // the one `catalog_resolve_error` describes ("serves a catalog but names no ROS
        // type" is false when no catalog came back). Reuse the
        // condition's EXISTING message rather than inventing a second one, so the netd
        // and transient paths cannot diverge on one user-visible condition — the whole
        // point of message parity.
        (true, true) => Some(catalog_no_answer_error(robot, topic)),
        // The robot answered and its catalog simply names no type: terminal, earned.
        (true, false) => None,
    }
}

/// The precise "robot serves a catalog but names no type for this
/// topic" error (a catalog answered, but `topic` has no schema name). Shared by the
/// netd + transient paths. PURE.
///
/// Only correct when a catalog really DID come back from a SETTLED gather —
/// [`catalog_absence_verdict`] is the gate that decides.
fn catalog_resolve_error(robot: &str, topic: &str) -> String {
    format!(
        "robot '{robot}' serves a catalog but names no ROS type for topic '{topic}' (is the topic \
         produced on the robot? name the type explicitly as '{topic}=pkg/Type' if you know it)"
    )
}

/// The precise "robot did not answer a catalog" error. PURE.
///
/// Shared by BOTH paths now. It was the transient fallback's alone, while
/// netd's own no-answer was reported through [`catalog_resolve_error`] over an empty
/// catalog list — i.e. as "serves a catalog but names no ROS type", which asserts an
/// answer that never came. [`catalog_absence_verdict`] routes the netd path here
/// instead, so one condition has ONE message (and the `--connect` / gateway hints
/// reach both paths, which is exactly what message parity exists to guarantee).
fn catalog_no_answer_error(robot: &str, topic: &str) -> String {
    format!(
        "robot '{robot}' did not answer a schema catalog (is it reachable, and is its gateway \
         serving schemas? try naming the type explicitly as '{topic}=pkg/Type', or add --connect \
         <locator> if scouting can't reach it)"
    )
}

/// Say so when a remote attach falls back to the definition
/// this daemon ALREADY held, because the one the robot served was refused.
///
/// Extracted from the call site purely so the LEVEL is pinnable — it is `warn!`,
/// not `debug!`, because the attach then SUCCEEDS on a definition that is not the
/// one the robot just served.
///
/// The behaviour it reports is deliberate. Without the validate-then-commit
/// rollback, a refused doc for a held name would poison `seed.docs` and the attach
/// would fail LOUDLY (the walker would not know the name). With it the incumbent survives —
/// strictly better for every other consumer of that type — but it makes this
/// caller's success quiet, and a robot whose message really did
/// change would look like it landed. A real mismatch surfaces downstream as frames
/// that do not decode; this line is what connects that to its cause.
fn report_incumbent_fallback(robot: &str, name: &str, refused: &[String]) {
    if !refused.iter().any(|q| q == name) {
        return;
    }
    tracing::warn!(
        robot = %robot,
        schema = %name,
        "cerulion-vizd: the schema served by this robot could not be used (it failed to parse, \
         or is YAML-encoded — this daemon seeds only `.msg`), so the definition this daemon \
         ALREADY held is being used instead. The attach proceeds on that INCUMBENT definition: \
         if the robot's type has changed, its frames will not decode until a usable definition \
         is served"
    );
}

/// The desk-side outcome of a schema fetch (netd OR
/// transient), classified from the gathered [`SchemaReply`]s so BOTH paths produce
/// the SAME, accurate error. A docs-carrying reply is `Served` (seed + resolve); a
/// docs-EMPTY reply carrying a served `error` reason is `NotServed` (the robot
/// ANSWERED "I don't have / won't serve that"); NO usable reply is `NoAnswer`
/// (unreachable / no query surface / an undecodable reply). The PURE input to
/// [`Ctx::seed_and_resolve_schema`].
#[derive(Debug, Clone, PartialEq, Eq)]
enum SchemaFetchOutcome {
    /// A reply carrying the requested type's `.msg`/YAML closure.
    Served(Vec<SchemaDoc>),
    /// The robot ANSWERED but does not serve the type (docs-empty, `error: Some`) —
    /// carries the served refusal/not-found reason verbatim.
    NotServed(String),
    /// No usable answer at all.
    NoAnswer,
}

/// PURE: classify gathered schema `replies` into a
/// [`SchemaFetchOutcome`]. The FIRST reply carrying docs wins (`Served`); else the
/// FIRST docs-empty reply carrying an `error` reason is surfaced (`NotServed`); an
/// empty set — or replies with neither docs nor a reason — is `NoAnswer`. Shared by
/// the netd + transient fetch paths so their error MESSAGE quality is identical (a
/// not-found reply is never reported as a parse failure on either). Oracle-tested.
fn classify_schema_replies(replies: Vec<SchemaReply>) -> SchemaFetchOutcome {
    let mut first_reason: Option<String> = None;
    for reply in replies {
        if !reply.docs.is_empty() {
            return SchemaFetchOutcome::Served(reply.docs);
        }
        if first_reason.is_none() {
            first_reason = reply.error;
        }
    }
    match first_reason {
        Some(reason) => SchemaFetchOutcome::NotServed(reason),
        None => SchemaFetchOutcome::NoAnswer,
    }
}

/// The precise "the robot answered but does not serve this schema"
/// error — carries the robot's OWN served `reason` (a not-found OR an authorization
/// refusal). Shared by the netd + transient fetch paths. PURE.
fn schema_not_served_error(robot: &str, name: &str, reason: &str) -> String {
    format!("robot '{robot}' does not serve schema '{name}': {reason}")
}

/// The precise "the robot did not answer a schema fetch at all" error
/// (unreachable / no query surface / an undecodable reply) — kept DISTINCT from
/// [`schema_not_served_error`] so the unreachable-vs-answered distinction survives.
/// PURE.
fn schema_no_answer_error(robot: &str, name: &str) -> String {
    format!(
        "robot '{robot}' did not answer a schema fetch for '{name}' (unreachable, or its gateway \
         is not serving schemas)"
    )
}

/// RESERVE-THEN-COMMIT demand — the production seam that closes the
/// concurrent-controller TOCTOU (each UDS control connection runs on its OWN handler
/// thread). Atomically CLAIMs `(topic → robot)` in `demanded` under ONE state lock
/// (so exactly one of two concurrent same-topic attaches proceeds), then `demand`s
/// OUTSIDE the lock (a network round-trip — never held under the state mutex, which
/// would serialize every other controller). A demand FAILURE rolls the claim back so
/// a retry re-demands with no phantom claim. An already-claimed topic FROM THE SAME
/// robot is an idempotent no-op (the tap step reports `already_attached`); a claim
/// from a DIFFERENT robot is a LOUD refusal (one data source = one topic — see the
/// entry-API body), never a silent binding flip.
///
/// Returns whether this call FRESHLY demanded the topic (`true` = netd just created
/// the mirror → the caller waits for its cross-process-visible service before
/// tapping; `false` = an idempotent same-robot re-demand, the mirror already exists).
fn perform_remote_demand(
    plane: &dyn DemandPlane,
    state: &Mutex<DaemonState>,
    robot: &str,
    topic: &str,
    schema_hash: u64,
) -> Result<bool, String> {
    use std::collections::btree_map::Entry;
    // Decide under the lock via the ENTRY API (one topic = ONE origin robot — the
    // decision). A plain `insert` would OVERWRITE the stored robot on a
    // different-robot re-claim while reporting "already claimed", so a later `detach`
    // would release the WRONG (robot, topic) and leak the real one. The entry API
    // makes each case explicit:
    // - Vacant → claim it here, demand OUTSIDE the lock (rolled back on failure).
    // - Occupied, SAME robot → idempotent no-op (keep the value, never re-insert).
    // - Occupied, DIFFERENT robot → LOUD refusal: no claim change, no demand, no
    //   release (a pure refusal that leaks nothing).
    {
        let mut st = state.lock().unwrap_or_else(|e| e.into_inner());
        match st.demanded.entry(topic.to_string()) {
            Entry::Vacant(v) => {
                v.insert(robot.to_string()); // claimed; demand below (outside the lock).
            }
            Entry::Occupied(o) => {
                if o.get() == robot {
                    return Ok(false); // same robot — idempotent (re-attach / concurrent).
                }
                return Err(format!(
                    "topic '{topic}' is already mirrored from robot '{}' — one data source = one \
                     topic, so it cannot ALSO be demanded from robot '{robot}'; detach '{topic}' \
                     first if you meant to re-point it to a different robot",
                    o.get()
                ));
            }
        }
    }
    // Demand OUTSIDE the lock. On failure roll the just-made claim back so a retry
    // re-demands (self-healing — no phantom claim left behind).
    if let Err(e) = plane.demand(robot, topic, schema_hash) {
        let mut st = state.lock().unwrap_or_else(|e| e.into_inner());
        st.demanded.remove(topic);
        return Err(e);
    }
    Ok(true)
}

/// RELEASE a topic's netd demand (the `detach` early release).
/// Removes the `(topic → robot)` claim under the state lock and, if
/// it held one, `release`s it from netd OUTSIDE the lock. Returns whether a demand
/// was released (`false` = the topic was not demanded, a no-op). A netd release
/// error is propagated for the caller to log (never fails the detach).
fn perform_remote_release(
    plane: &dyn DemandPlane,
    state: &Mutex<DaemonState>,
    topic: &str,
) -> Result<bool, String> {
    let robot = {
        let mut st = state.lock().unwrap_or_else(|e| e.into_inner());
        st.demanded.remove(topic)
    };
    match robot {
        Some(robot) => plane.release(&robot, topic).map(|_| true),
        None => Ok(false),
    }
}

/// How long to wait for a freshly-demanded mirror's `{topic}/data`
/// service to become cross-process-visible before tapping it (~0.5 s total). In
/// production `cerulion-netd` is a SEPARATE process, so the mirror it just created
/// may not be visible on this daemon's first tap-open — mirrors `topic_cmd`'s
/// `open_mirror_after_demand` bounded retry. Best-effort: if the service never
/// appears the tap step attempts the open anyway and its Err arm rolls the demand
/// back (never a silent leak).
const MIRROR_VISIBLE_ATTEMPTS: u32 = 20;
const MIRROR_VISIBLE_DELAY: Duration = Duration::from_millis(25);

/// Bounded-wait until `topic`'s mirror data service is visible on `manager` (or the
/// budget elapses). Returns immediately when the service already exists, so a
/// re-attach pays nothing. NOT under any lock — a pure transport probe.
fn wait_for_mirror_service(manager: &TransportManager, topic: &str) {
    for _ in 0..MIRROR_VISIBLE_ATTEMPTS {
        if !manager.data_service_missing(topic) {
            return; // the service is visible — tap it.
        }
        std::thread::sleep(MIRROR_VISIBLE_DELAY);
    }
    // Fell through: the tap step still tries the open; its Err arm rolls the demand
    // back. Loud so a persistent cross-process visibility failure is diagnosable.
    tracing::warn!(
        topic = %topic,
        "cerulion-vizd: netd's mirror service for '{topic}' did not become visible within the \
         wait budget — the tap open will decide (its failure rolls the demand back)"
    );
}

/// The network-free half of remote schema resolution (remote arm): decide
/// what a PINNED `schema` name needs, WITHOUT touching the network. Pure over the
/// walker so the decision is oracle-testable:
///
/// - a BARE (unqualified, no `/`) name → `Err` with the "qualify it as `pkg/Type`"
///   hint (a bare built-in name is a fixable typo, NOT a custom type).
/// - a QUALIFIED name the walker ALREADY knows (built-in ∪ workspace `.msg` ∪
///   already-seeded) → `Ok(Some((name, hash)))` — the fast path, no fetch needed.
/// - a QUALIFIED name the walker does NOT know → `Ok(None)`: the caller must fetch
///   its `.msg` closure from the robot and seed the walker.
fn resolve_pinned_schema(
    walker: &FrameWalker,
    name: &str,
) -> Result<Option<(String, u64)>, String> {
    // A bare, unqualified type name is a fixable typo (name it `pkg/Type`), NOT a
    // custom type — do NOT send it down the remote-fetch path. Types
    // resolve ONLY by their fully-qualified `pkg/Type` name.
    if !name.contains('/') {
        return Err(format!(
            "schema '{name}' is not fully qualified — name the ROS type as `pkg/Type` \
             (e.g. `sensor_msgs/PointCloud2`, `geometry_msgs/Vector3`); a bare type name \
             cannot be resolved to a wire hash"
        ));
    }
    Ok(walker
        .schema_hash_for(name)
        .map(|hash| (name.to_string(), hash)))
}

/// Resolve a frame's `(schema, archetype)` via the walker: header → schema
/// qualified name, then the sink's WHOLE classification ladder, so the reported
/// archetype EXACTLY matches what the sink will render.
///
/// **It must be [`classify_frame`], never the name-table-then-shape pair.** This
/// value drives what `discover` / `list` / `status` report, which views
/// [`maybe_apply_default_layout`] composes, and what guardrail 2 accepts — so any
/// divergence from the sink is a layout built for a topic that renders something
/// else. The H.264 route made that concrete: a decoded H.264 message
/// (`{time_frame, video_height, video_data}`) infers `Scalars` from its field
/// shape (the two integers are plottable) while the sink renders `VideoStream`,
/// so the daemon composed a time-series plot and NO `spatial2d` view — an empty
/// plot pane, video logged where nothing displayed it, and a hand-written
/// `spatial2d` view refused as "renders nothing".
fn resolve_from_frame(
    walker: &FrameWalker,
    frame: &[u8],
    candidate_name: Option<&str>,
) -> Result<Resolved, ResolveFailure> {
    let header = WireHeader::read_from_buf(frame).ok_or(ResolveFailure::ShortHeader)?;
    let qname = match walker.schema_name_for_hash(header.schema_hash) {
        Some(q) => q.to_string(),
        // The hash matches NO schema this build holds. A bare `None`
        // with no log at all would be indistinguishable from
        // a silent topic, and the topic would die silently. Classify
        // it against whatever out-of-band name the caller has (the catalog /
        // attach pin) so the verdict names a REMEDY, not just a number.
        None => {
            return Err(ResolveFailure::UnknownHash(
                diagnose_unknown_hash_with_walker(walker, header.schema_hash, candidate_name),
            ))
        }
    };
    let fv = walker
        .walk(&qname, frame)
        .map_err(|_| ResolveFailure::Decode(qname.clone()))?;
    // Deliberately the UNMEMOIZED ladder, not `SinkState::archetype_for`
    // here: this is a ONE-SHOT attach/discover peek over a single drained
    // frame, not the per-frame render loop, so there is no repeat to amortize and
    // no `SinkState` here to hold a memo. The memo lives where frames repeat.
    //
    // But it MUST be the WHOLE ladder — `classify_frame`, not
    // `classify_schema`-then-shape. This value drives what discover/list/status
    // report, which views the default layout composes, and guardrail 2's
    // accept/refuse, so any divergence from what the sink RENDERS builds a layout
    // for a topic that draws something else. A decoded H.264 message infers
    // `Scalars` from its integers while the sink renders `VideoStream`: the peek
    // would compose a time-series plot and NO spatial2d view.
    let archetype = classify_frame(&fv);
    Ok(Resolved {
        schema: qname,
        archetype,
    })
}

/// Why a frame did not resolve to a `(schema, archetype)`.
///
/// Without this enum, every one of these is a bare `None`, so "the topic is silent",
/// "these bytes are not a frame", "I hold no schema for this hash" and "the
/// frame is corrupt" are ONE indistinguishable answer — reported to the client
/// as `schema: null` and to the log as nothing at all. They have different
/// remedies, so they are different values.
#[derive(Debug, Clone, PartialEq, Eq)]
enum ResolveFailure {
    /// No frame was available to resolve — the topic is silent within the peek
    /// bound, or no tap could be opened. NOT a decode problem, and deliberately
    /// NOT reported as one.
    NoFrame,
    /// Shorter than a `WireHeader`: not a Cerulion frame at all.
    ShortHeader,
    /// No schema in this build matches the wire `schema_hash` — the known
    /// condition, already classified into a remedy.
    UnknownHash(UnknownHashDiagnosis),
    /// The hash resolved to this schema, but the frame did not decode as it (a
    /// truncated or corrupt frame — a FRAMING disagreement, not a type one).
    Decode(String),
}

/// A bounded, best-effort schema peek: open a TRANSIENT data-only tap (the
/// invisible observer class), drain one frame within `bound`, and resolve it.
/// The transient tap is dropped immediately (≤1 extra introspection slot,
/// briefly).
///
/// Returns the classified [`ResolveFailure`] rather than a bare `None`,
/// so a caller can tell a SILENT topic from one whose frames arrived and could
/// not be read.
fn peek_resolve(
    manager: &TransportManager,
    walker: &FrameWalker,
    topic: &str,
    bound: Duration,
    candidate_name: Option<&str>,
) -> Result<Resolved, ResolveFailure> {
    let mut sub = manager
        .create_data_only_subscriber(topic)
        .map_err(|_| ResolveFailure::NoFrame)?;
    let deadline = Instant::now() + bound;
    let mut scratch = Vec::new();
    loop {
        scratch.clear();
        if let Ok(n) = sub.drain_owned(1, &mut scratch) {
            if n > 0 {
                return resolve_from_frame(walker, scratch[0].payload(), candidate_name);
            }
        }
        if Instant::now() >= deadline {
            return Err(ResolveFailure::NoFrame);
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The wire report for an unresolvable hash — the explicit per-topic
/// state a Studio sidebar renders instead of a blank row.
fn undecodable_report(diagnosis: &UnknownHashDiagnosis) -> UndecodableReport {
    UndecodableReport {
        reason: diagnosis.reason_code().to_string(),
        schema_hash: format!("0x{:016X}", diagnosis.wire_hash()),
        schema: diagnosis.schema_name().map(str::to_string),
        local_schema_hash: diagnosis.local_hash().map(|h| format!("0x{h:016X}")),
        detail: diagnosis.detail(),
    }
}

/// Fold one resolution attempt into a topic's stats — the ONE place
/// the verdict, the latch and the log stay in agreement.
///
/// A resolution CLOSES any open regime (recovery is real: a walker swap can seed
/// the missing type mid-run). An unresolvable hash opens or sustains one. The
/// other failures deliberately do NOT touch the latch: `NoFrame` is a silent
/// topic and no failure at all, while `ShortHeader`/`Decode` are framing
/// conditions with their own remedies, and folding them into the type-identity
/// regime would let one swallow the other's loud head.
fn record_resolution(
    stat: &mut TopicStat,
    topic: &str,
    outcome: &Result<Resolved, ResolveFailure>,
) {
    match outcome {
        Ok(r) => {
            stat.undecodable = None;
            stat.resolved = Some(r.clone());
            stat.resolved_from_frame = true;
            report_schema_hash_resolved(&mut stat.unknown_hash, DiagnosisVantage::Resolver, topic);
        }
        Err(ResolveFailure::UnknownHash(diagnosis)) => {
            stat.undecodable = Some(undecodable_report(diagnosis));
            report_unknown_schema_hash(
                &mut stat.unknown_hash,
                DiagnosisVantage::Resolver,
                topic,
                diagnosis,
            );
        }
        Err(ResolveFailure::NoFrame)
        | Err(ResolveFailure::ShortHeader)
        | Err(ResolveFailure::Decode(_)) => {}
    }
}

/// Report `missed` frames this tap never delivered, under the shared
/// [`FailureRegimeLatch`] contract (loud head, `debug!` repeats carrying the
/// suppressed count, a loud re-announcement at each decade of the running total).
///
/// `warn!` and not `error!`: the desk has lost data and the operator must be told,
/// but nothing is broken that a slower producer or a deeper queue would not fix —
/// the same level `cerulion topic echo`'s gap marker has carried since it shipped.
///
/// The message names the CONSEQUENCE for video deliberately. A dropped scalar
/// sample costs one point on a plot; a dropped H.264 access unit costs every
/// frame that references it, so a 1.6 % pre-decode loss renders as ~22 % of the
/// stream missing. An operator reading "3 frames missed" has no way to know that
/// without being told.
fn report_tap_gap(latch: &mut FailureRegimeLatch, topic: &str, missed: u64, total_missed: u64) {
    match latch.on_failure() {
        RegimeDecision::Loud => tracing::warn!(
            topic = %topic,
            missed,
            frames_missed = total_missed,
            total_failures = latch.total_failures(),
            "vizd: frames were LOST before decoding — the tap's subscriber queue overflowed \
             between two polls (drop_oldest evicts silently). On an encoded video topic every \
             frame that references a lost one also fails, so the visible gap is far larger than \
             the loss; repeats log at debug until the tap keeps up again"
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            topic = %topic,
            missed,
            suppressed,
            frames_missed = total_missed,
            total_failures = latch.total_failures(),
            "vizd: still losing frames before decode (warn suppressed)"
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            topic = %topic,
            missed,
            suppressed,
            frames_missed = total_missed,
            total_failures = total,
            "vizd: STILL losing frames before decode — the regime has not closed"
        ),
    }
}

/// A running daemon: the shutdown flag + the poll/accept/connection threads + the
/// socket cleanup guard. Dropping it (or [`shutdown`](Self::shutdown)) stops
/// every thread, drops the taps + worker, and removes the socket + pidfile.
#[derive(Debug)]
pub struct RunningDaemon {
    shutdown: Arc<AtomicBool>,
    accept_handle: Option<JoinHandle<()>>,
    poll_handle: Option<JoinHandle<()>>,
    conns: Arc<Mutex<Vec<JoinHandle<()>>>>,
    /// The shared worker control sender. Held so [`shutdown`] can
    /// `close()` it FIRST, releasing the worker's `Disconnected` exit even while
    /// `Ctx` clones on other threads still hold an `Arc<VizControl>`.
    ///
    /// [`shutdown`]: RunningDaemon::shutdown
    worker_control: Arc<VizControl>,
    guard: Option<SocketGuard>,
    socket_path: PathBuf,
    /// The catalog-change forwarder thread (`None` when it could not be
    /// spawned — the daemon runs without pushes rather than failing to start).
    events_handle: Option<JoinHandle<()>>,
    /// The event hub, for observability (Principle #3) and the e2e.
    events: Arc<crate::events::EventHub>,
    /// The controller read-loop iteration counter (Principle #3) — see
    /// [`Ctx::conn_loop_iterations`].
    conn_loop_iterations: Arc<AtomicU64>,
    /// The frame-drain loop's last measured period (Principle #3) — see
    /// [`Ctx::poll_period_ns`].
    poll_period_ns: Arc<AtomicU64>,
    /// The frame-drain loop's iteration counter — see
    /// [`Ctx::poll_iterations`].
    poll_iterations: Arc<AtomicU64>,
    /// Passes that took no wait at all — see
    /// [`Ctx::poll_sleepless_passes`].
    poll_sleepless_passes: Arc<AtomicU64>,
    /// Passes that reached the already-late arm — see
    /// [`Ctx::poll_late_passes`].
    poll_late_passes: Arc<AtomicU64>,
    /// Passes that blocked on a WAKE SET — see
    /// [`Ctx::poll_wake_waits`].
    poll_wake_waits: Arc<AtomicU64>,
    /// Passes whose wake set reported a fired source — see
    /// [`Ctx::poll_wake_fires`].
    poll_wake_fires: Arc<AtomicU64>,
    /// Default-layout reflows driven by a layout-signal change — see
    /// [`Ctx::poll_layout_signal_reflows`].
    poll_layout_signal_reflows: Arc<AtomicU64>,
    /// Total frames the drain loop handed on — see
    /// [`Ctx::poll_frames_drained`].
    poll_frames_drained: Arc<AtomicU64>,
    /// Passes that paced themselves after a wake — see
    /// [`Ctx::poll_wake_paced_passes`].
    poll_wake_paced_passes: Arc<AtomicU64>,
    /// Topics whose wake was dropped — see
    /// [`Ctx::poll_wake_demotions`].
    poll_wake_demotions: Arc<AtomicU64>,
    /// Passes that skipped the wait on a drained backlog — see
    /// [`Ctx::poll_backlog_passes`].
    poll_backlog_passes: Arc<AtomicU64>,
    /// Monitors: samples handed to the monitor engine — see
    /// [`Ctx::monitor_samples_taken`].
    monitor_samples_taken: Arc<AtomicU64>,
    /// Monitors: conditions raised — see [`Ctx::monitor_alerts_raised`].
    monitor_alerts_raised: Arc<AtomicU64>,
    /// Monitors: conditions cleared — see [`Ctx::monitor_alerts_cleared`].
    monitor_alerts_cleared: Arc<AtomicU64>,
    /// Monitors: rows currently withholding a condition (a GAUGE) — see
    /// [`Ctx::monitor_rows_ineligible`].
    monitor_rows_ineligible: Arc<AtomicU64>,
    /// Monitors: samples the DISCOVERY plane handed over — see
    /// [`Ctx::monitor_catalog_samples_taken`].
    monitor_catalog_samples_taken: Arc<AtomicU64>,
    /// Monitors: discovery rows retired — see
    /// [`Ctx::monitor_catalog_rows_retired`].
    monitor_catalog_rows_retired: Arc<AtomicU64>,
    /// The test-only overrun injector — see
    /// [`Ctx::poll_injected_work_ns`].
    ///
    /// GATED with the accessor that reads it. Its ONLY reader is
    /// [`inject_poll_work_for_test`](Self::inject_poll_work_for_test), so
    /// leaving the field unconditional makes a NO-feature build of the lib carry
    /// a never-read field — `error: field is never read` under this repo's
    /// `dead_code = "deny"`.
    ///
    /// That is not hypothetical and it is not visible to the obvious local
    /// gates: the `test-helpers` self-reference in `[dev-dependencies]`
    /// FEATURE-UNIFIES the feature ON for `cargo test -p cerulion_vizd` and for
    /// `cargo clippy --workspace --all-targets`, so both pass while CI's plain
    /// `cargo build`/`cargo check` of the lib fails. `cargo check -p
    /// cerulion_vizd` on its own is the gate that reproduces CI here.
    ///
    /// The `Ctx` copy is deliberately NOT gated: the drive loop reads it once
    /// per pass, which is production code.
    #[cfg(any(test, feature = "test-helpers"))]
    poll_injected_work_ns: Arc<AtomicU64>,
    /// See [`Self::inject_runs_local_delay_for_test`]. Gated for the
    /// same `dead_code = "deny"` reason as the field above.
    #[cfg(any(test, feature = "test-helpers"))]
    runs_local_delay_ns: Arc<AtomicU64>,
}

impl RunningDaemon {
    /// Principle #3 observable: total controller read-loop iterations across every
    /// handler since this daemon started. An IDLE controller contributes one per
    /// `CONN_READ_TIMEOUT` (~5/s) because its `read` blocks for that long; if the
    /// accepted socket were left nonblocking the same loop would spin and this would
    /// climb by millions per second. The oracle for that difference is an UPPER
    /// bound over a held-idle window — see `catalog_events_e2e_test` (the arm lives
    /// beside the push tests, because it is the same read loop they ride).
    pub fn conn_read_loop_iterations(&self) -> u64 {
        self.conn_loop_iterations.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: the frame-drain loop's LAST measured
    /// inter-iteration period, in nanoseconds — `0` until the loop reaches the
    /// TOP of its second iteration, i.e. once one full iteration has elapsed.
    ///
    /// This is the loop that owns the desk's frame latency: a frame committed to
    /// SHM waits here, on average, half a period. The number is the loop's OWN
    /// measurement between two consecutive tops of its body, so it includes the
    /// drain work, the lock acquisition, the worker hand-off AND the wait — i.e.
    /// exactly the quantity `DEFAULT_POLL_INTERVAL` is supposed to name.
    ///
    /// It is a LAST value, not an average: an assertion over it must therefore
    /// be a CEILING (contention can only push a period UP, so a loaded runner
    /// cannot fake a small one), never a tight band.
    pub fn poll_loop_period_ns(&self) -> u64 {
        self.poll_period_ns.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: total frame-drain loop iterations since
    /// this daemon started — the COUNTED twin of
    /// [`poll_loop_period_ns`](Self::poll_loop_period_ns), and the drain-loop
    /// analogue of
    /// [`conn_read_loop_iterations`](Self::conn_read_loop_iterations).
    ///
    /// A rate over a held window is the direct instrument for a loop that is
    /// running too fast; the period register needs a sampler and a median to say
    /// the same thing, and the sampler under-counts exactly when the loop
    /// outruns it. A spin bound is stated on a counted rate
    /// for that reason.
    pub fn poll_loop_iterations(&self) -> u64 {
        self.poll_iterations.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes that took NO wait at
    /// all.
    ///
    /// **Zero is the contract, not merely the observed value.** Waiting to a
    /// deadline means an already-late pass has no remaining time to wait, so a
    /// loop that simply slept the remainder would take zero waits for as long as
    /// an overrun lasted — running back-to-back and re-acquiring the daemon's
    /// state lock microseconds after releasing it, where a fixed
    /// post-work sleep would yield unconditionally. `plan_tick_sleep` floors that arm
    /// at `LATE_PASS_YIELD` (200 µs), so this counter stays 0 under any overrun;
    /// it is the observable that says so rather than assuming it.
    pub fn poll_loop_sleepless_passes(&self) -> u64 {
        self.poll_sleepless_passes.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes that reached the
    /// ALREADY-LATE arm, i.e. passes whose own wall consumed the whole interval.
    ///
    /// This is the loop's STIMULUS observable, and it is strictly one-sided:
    /// contention can only ADD late passes, never remove them. A test proving it
    /// drove the loop into overrun therefore asserts a FLOOR here — a bound load
    /// cannot invert — where an iteration floor would be satisfied *more* easily
    /// by the healthy loop it is trying to exclude, and
    /// [`poll_loop_sleepless_passes`](Self::poll_loop_sleepless_passes) is 0 on
    /// every loop by construction.
    ///
    /// On a healthy daemon this stays at or near 0: reaching the arm needs a
    /// pass to outrun its own interval, and a real drain pass measures ~0.009 ms
    /// (75 empty taps) to ~0.230 ms (75 saturated) against 16 ms.
    pub fn poll_loop_late_passes(&self) -> u64 {
        self.poll_late_passes.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes that spent their wait
    /// BLOCKED ON A WAKE SET rather than on `thread::sleep`.
    ///
    /// This is the chunk's no-inert-shipping observable. A latency assertion
    /// alone cannot distinguish "the wake fired" from "the timer happened to
    /// expire just then on a fast runner", so a test that proves the wake path is
    /// live asserts a FLOOR here as well — and reverting the attach seam's
    /// `WakeMode::Listener` to `Timer` drops it to 0 while every frame oracle
    /// stays green.
    pub fn poll_loop_wake_waits(&self) -> u64 {
        self.poll_wake_waits.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes whose wake set reported
    /// at least one FIRED source — the wait ended because a producer notified,
    /// not because the timeout expired.
    ///
    /// One-sided in the load-safe direction: contention makes a wake arrive
    /// LATER (and past the timeout it is not counted at all), so a FLOOR is a
    /// claim a loaded runner cannot manufacture — only suppress. A CEILING on it
    /// would be the inverted, unsafe form.
    pub fn poll_loop_wake_fires(&self) -> u64 {
        self.poll_wake_fires.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: default-layout reflows the drain loop RE-DERIVED
    /// because a topic's live layout signal — its render proof, or its H.264
    /// rendition set — changed, the dump companion appearing or going on
    /// live evidence.
    ///
    /// A FLOOR on this is the load-safe claim in the TIME direction: contention can
    /// only DELAY the worker's mirror refresh and therefore the reflow, never
    /// manufacture one, so a test may wait for the count to reach N but must not
    /// assert it stays below one derived from an INTERVAL.
    ///
    /// A STRUCTURAL ceiling is load-safe too and is what pins the boundedness this
    /// counter's doc claims: the worker bumps its generation only on a real change,
    /// so a burst of identical frames adds nothing however slowly it is drained,
    /// and a test may assert the count does not grow across one (see
    /// `a_steady_stream_of_identical_frames_stops_reflowing`). What remains
    /// forbidden is a ceiling derived from how many PASSES a runner got through.
    pub fn poll_loop_layout_signal_reflows(&self) -> u64 {
        self.poll_layout_signal_reflows.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes that skipped the wait
    /// because the drain yielded frames (the backlog-aware arm).
    ///
    /// A zero-wait pass is the shape a spin takes, so it is COUNTED rather than
    /// left implicit. It is bounded by pairing: a pass that drains frames is
    /// followed by one that drains none and DOES wait, so the skip costs at most
    /// one extra pass per burst.
    pub fn poll_loop_backlog_passes(&self) -> u64 {
        self.poll_backlog_passes.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): samples the drain loop's sampler has
    /// HANDED to the monitor engine since this daemon started.
    ///
    /// The sampler's no-inert-shipping observable. The engine is pure and its
    /// whole suite stays green whether or not anything ever calls it, so this is
    /// the number that says the sampler is WIRED: a floor on it cannot be
    /// satisfied by a daemon that deleted the call site. Strictly one-sided in the
    /// load-safe direction — contention can only make the sampler run LESS often —
    /// so a FLOOR is a claim a loaded runner cannot manufacture, and a CEILING on
    /// it would be the class a loaded runner can invert.
    ///
    /// Counts samples handed over, settle-window and blind ones INCLUDED; see
    /// `Ctx::monitor_samples_taken` for why, and read
    /// [`cerulion_viz::monitor::MonitorRow::samples`] for the evidence count.
    pub fn monitor_samples_taken(&self) -> u64 {
        self.monitor_samples_taken.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): conditions RAISED since this daemon
    /// started, across every watched row.
    ///
    /// Bumped inside the sampling pass that observed the raise, so it cannot be
    /// true unless the engine really transitioned (the counter rule
    /// stated on `Ctx::poll_layout_signal_reflows`).
    pub fn monitor_alerts_raised(&self) -> u64 {
        self.monitor_alerts_raised.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): conditions CLEARED since this daemon
    /// started.
    ///
    /// Kept apart from the raise count rather than netted: a row that raised and
    /// cleared ten times is a FLAPPER, and a net zero describes it identically to a
    /// desk on which nothing happened at all.
    pub fn monitor_alerts_cleared(&self) -> u64 {
        self.monitor_alerts_cleared.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): how many watched rows are currently
    /// withholding at least one condition — a GAUGE as of the last sampling pass,
    /// not a running total.
    ///
    /// It is the size of what the agent is NOT being told, readable without
    /// polling the verb. It goes DOWN when rows recover, which is the property a
    /// running total could not have.
    ///
    /// Written by BOTH planes — it is a gauge over the whole row
    /// table, so whichever pass ran last states the current size of it.
    pub fn monitor_rows_ineligible(&self) -> u64 {
        self.monitor_rows_ineligible.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): samples the DISCOVERY plane has handed
    /// to the engine — the rows for topics NOBODY has attached.
    ///
    /// The discovery plane's no-inert-shipping observable. Everything that plane does
    /// is otherwise indistinguishable from the attached plane on the `monitors`
    /// verb (both mint the same rows into the same table), so this is the number
    /// that says the catalog feed is WIRED — a floor on it cannot be satisfied by a
    /// daemon that deleted the call site in `discover`.
    ///
    /// Load-safe in one direction only, like its attached twin: this plane samples
    /// when a controller calls `discover`, so a FLOOR is a claim a loaded runner
    /// cannot manufacture and a CEILING would be the class a loaded runner can invert.
    pub fn monitor_catalog_samples_taken(&self) -> u64 {
        self.monitor_catalog_samples_taken.load(Ordering::Relaxed)
    }

    /// Monitors (Principle #3): discovery-plane rows retired because a
    /// SETTLED gather stopped naming them.
    ///
    /// A catalog row has no lifecycle of its own — a tap's row dies with its tap,
    /// but a row minted from whatever a robot announced would otherwise outlive the
    /// robot and report UNKNOWN with a forever-growing age. This is the number that
    /// says the retirement really runs, and it is a total rather than a gauge
    /// because it records events.
    pub fn monitor_catalog_rows_retired(&self) -> u64 {
        self.monitor_catalog_rows_retired.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: total FRAMES the drain loop has handed on
    /// since this daemon started, across every tap.
    ///
    /// This is the instrument a latency measurement needs. `status.frames`
    /// answers the same question per topic, but through a control round trip that
    /// takes the daemon's state lock — milliseconds, against a quantity measured
    /// in microseconds, i.e. an instrument coarser than the thing it measures. A
    /// relaxed atomic load contends with nothing and can be spun on.
    pub fn poll_loop_frames_drained(&self) -> u64 {
        self.poll_frames_drained.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: frame-drain passes that PACED themselves
    /// after a wake that kept firing without delivering.
    ///
    /// A FLOOR on this is the load-safe oracle that a notifier storm was
    /// bounded: the pacing can only be entered by genuinely churning wakes, and
    /// contention cannot manufacture one. Zero on every healthy desk.
    pub fn poll_loop_wake_paced_passes(&self) -> u64 {
        self.poll_wake_paced_passes.load(Ordering::Relaxed)
    }

    /// Principle #3 observable: topics whose wake listener was DROPPED
    /// because it fired `WAKE_DEMOTE_STREAK` times running without delivering.
    ///
    /// Running total, never reset — it is a count of producers this daemon
    /// stopped billing, and the answer to "why did that topic get slower?".
    pub fn poll_loop_wake_demotions(&self) -> u64 {
        self.poll_wake_demotions.load(Ordering::Relaxed)
    }

    /// Test seam: make every frame-drain pass burn
    /// `nanos` of CPU, so the loop's per-pass wall exceeds its interval and the
    /// already-late arm is reached. `0` restores production behaviour.
    ///
    /// There is no other way to drive that arm: a REAL drain pass was measured
    /// (over real iceoryx2, through the exact `drain_owned` + copy + header-parse
    /// path) at 0.009 ms for 75 empty taps and 0.230 ms for 75 fully saturated
    /// ones, against a 16 ms interval — so no arrangement of real topics gets
    /// within 50× of the regime, and a test that tried would be asserting on the
    /// runner rather than on the loop.
    ///
    /// CFG-GATED behind `test-helpers`, matching the repo-wide
    /// convention for `*_for_test` seams: this one takes a caller-supplied span
    /// and busy-spins the drain thread for it, so it must not be reachable from
    /// a crate that merely links the vizd lib. `#[cfg(test)]` alone cannot serve
    /// — the consumers are integration tests, which link the non-test rlib — so
    /// the feature is enabled for them through the dev-dependencies
    /// self-reference in `Cargo.toml` (the `cerulion_core` idiom).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn inject_poll_work_for_test(&self, nanos: u64) {
        self.poll_injected_work_ns.store(nanos, Ordering::Relaxed);
    }

    /// Make the `runs` verb's LOCAL arm take `nanos`, the only way
    /// to observe that the two arms run CONCURRENTLY.
    ///
    /// Both real arms resist being made slow on demand: the local registry gather
    /// takes iceoryx2's zero-publisher fast path on a desk running nothing (and
    /// exits early the moment it has heard from every live writer), so it is
    /// naturally near-instant, and a slow REMOTE arm alone cannot discriminate —
    /// `max(0, D)` and `0 + D` are the same number. Only a slow LOCAL arm makes
    /// the sum and the max differ, which is exactly the shape that crosses
    /// `viz_client`'s 5 s reply deadline.
    ///
    /// CFG-GATED for the same reason as [`Self::inject_poll_work_for_test`]: it
    /// sleeps a handler thread for a caller-supplied span, so it must not be
    /// reachable from a crate that merely links this lib.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn inject_runs_local_delay_for_test(&self, nanos: u64) {
        self.runs_local_delay_ns.store(nanos, Ordering::Relaxed);
    }

    /// The bound control-socket path (for logging / the verb to connect to).
    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// Test seam: release the worker's control sender so the NEXT
    /// control push fails with [`VizControlError::WorkerGone`].
    ///
    /// The failure modes this stands in for — a wedged viewer (`Busy`) and a dead
    /// worker (`WorkerGone`) — are real and documented, and both reach the verb
    /// through the same `Result`. There is no other way to drive that arm from
    /// outside: the queue is bounded at 8 with a 1 s timeout, so provoking `Busy`
    /// for real would mean wedging the render thread on a timer.
    ///
    /// Test-only by construction (nothing in the daemon calls it), and DISTINCT
    /// from [`shutdown`](Self::shutdown), which also stops the threads — here the
    /// control server keeps serving, which is the whole point.
    ///
    /// CFG-GATED behind `test-helpers` (same-class sweep with
    /// `inject_poll_work_for_test`): it was `#[doc(hidden)] pub` with no gate,
    /// which is the shape the repo's `*_for_test` convention replaced. Once the
    /// feature exists, gating this too costs nothing and removes a way to
    /// silently break a live daemon's control plane from outside.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn close_worker_control_for_test(&self) {
        self.worker_control.close();
    }

    /// How many controllers are subscribed to catalog-change events.
    pub fn event_subscriber_count(&self) -> usize {
        self.events.subscriber_count()
    }

    /// Whether the upstream catalog-change subscription is live. `false`
    /// means no event can currently arrive (no source, an unreachable/too-old netd).
    pub fn is_catalog_stream_connected(&self) -> bool {
        self.events.is_connected()
    }

    /// The catalog generation this daemon has relayed up to.
    pub fn catalog_event_version(&self) -> u64 {
        self.events.version()
    }

    /// The number of live (not-yet-reaped) controller-connection handles the
    /// daemon is tracking. Diagnostics / observability (Principle #3): the accept
    /// loop prunes finished handles (`retain(is_finished)`) on every accept, so
    /// this stays bounded on a long-lived daemon even as controllers connect and
    /// disconnect. Used by the e2e suite to prove the reaping actually shrinks
    /// the handle set.
    pub fn live_connection_handle_count(&self) -> usize {
        self.conns.lock().unwrap().len()
    }

    /// Stop the daemon: flip the shutdown flag, join the accept + poll threads
    /// (the poll thread drops the worker → its bounded teardown flush), join
    /// every controller connection, then remove the socket + pidfile. Idempotent.
    pub fn shutdown(&mut self) {
        // Release the shared worker control sender FIRST: the worker's
        // `recv` can then reach `Disconnected` — and run its bounded teardown
        // flush — once the poll thread drops the worker, EVEN THOUGH `Ctx` clones
        // on the poll/accept/connection threads still hold an `Arc<VizControl>`
        // (the sender inside is now `None`). Without this the poll thread's
        // worker-drop join would spin to its 6 s detach on every shutdown.
        self.worker_control.close();
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(h) = self.accept_handle.take() {
            let _ = h.join();
        }
        if let Some(h) = self.poll_handle.take() {
            let _ = h.join();
        }
        // The forwarder observes the same shutdown flag (and sleeps in short
        // slices), so joining it here leaves no thread holding a netd subscription.
        if let Some(h) = self.events_handle.take() {
            let _ = h.join();
        }
        let handles = std::mem::take(&mut *self.conns.lock().unwrap());
        for h in handles {
            let _ = h.join();
        }
        // Drop the guard LAST → removes the socket + pidfile after the listener
        // thread is joined.
        self.guard.take();
    }
}

impl Drop for RunningDaemon {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// [`start_with_planes`] with an INJECTED [`DemandPlane`] — the test
/// seam. The e2e injects a plane pointed at an IN-PROCESS `cerulion-netd` (sharing
/// the isolated transport) so the full `attach_remote` → demand → netd-mirror → tap
/// composition runs in-process; production (`main.rs`, via [`start_on_socket`])
/// injects [`NetdDemandPlane`].
pub fn start_with_demand_plane(
    socket_path: PathBuf,
    poll_interval: Duration,
    manager: Arc<TransportManager>,
    worker: VizLogWorker,
    walker: FrameWalker,
    rerun_url: Option<String>,
    demand_plane: Arc<dyn DemandPlane>,
) -> io::Result<RunningDaemon> {
    // No catalog-event source, the pre-push behaviour exactly. Every existing
    // caller (the hermetic e2e included) keeps a daemon that opens no second netd
    // connection and pushes nothing; `subscribe_events` still succeeds and
    // reports `connected: false`.
    start_with_planes(
        socket_path,
        poll_interval,
        manager,
        worker,
        walker,
        rerun_url,
        demand_plane,
        Arc::new(crate::events::NoCatalogEventSource),
    )
}

/// The control socket, ALREADY acquired as THE vizd for its path (single-daemon
/// flock + stale recovery — [`ControlSocket::acquire`]).
///
/// Acquiring is the daemon's single-instance election, and it has to happen
/// BEFORE anything reaches a viewer — or touches the shared transport. The Rerun
/// stream is built and the worker spawned ahead of the daemon (they are injected
/// into [`start_on_socket`]), and the
/// worker's first act is to send the boot blueprint — so a second `cerulion-vizd`
/// pointed at a live daemon's proxy (`$CERULION_RERUN_URL`) that acquired late would push its
/// Scene-only default over the viewer's current layout and only THEN lose the
/// election and exit. A supervisor respawning that loser every few seconds would keep
/// the viewer snapping back to the default: a representation change or an attach
/// reflows for a moment, then vanishes. `main.rs` acquires first — before even
/// `TransportManager::init` (ONE iceoryx2 node + the startup dead-node sweep) — so
/// a loser exits before it owns a stream or a transport node.
#[derive(Debug)]
pub struct ControlSocket {
    listener: UnixListener,
    guard: SocketGuard,
    path: PathBuf,
}

impl ControlSocket {
    /// Acquire `socket_path` as THE vizd for that path — the `AddrInUse` refusal
    /// names the live holder (see [`cerulion_hygiene::DaemonSocket::acquire_socket`]).
    pub fn acquire(socket_path: PathBuf) -> io::Result<Self> {
        let (listener, guard) = acquire_socket(socket_path.clone())?;
        listener.set_nonblocking(true)?;
        Ok(Self {
            listener,
            guard,
            path: socket_path,
        })
    }

    /// The bound socket path.
    pub fn path(&self) -> &Path {
        &self.path
    }
}

/// [`start_with_demand_plane`] plus an INJECTED [`crate::events::CatalogEventSource`] — the
/// full-DI entry. Production (`main.rs`, via [`start_on_socket`]) injects the real
/// `cerulion-netd` subscription;
/// the e2e injects a scripted source so the relay + push contracts run hermetically.
#[allow(clippy::too_many_arguments)]
pub fn start_with_planes(
    socket_path: PathBuf,
    poll_interval: Duration,
    manager: Arc<TransportManager>,
    worker: VizLogWorker,
    walker: FrameWalker,
    rerun_url: Option<String>,
    demand_plane: Arc<dyn DemandPlane>,
    event_source: Arc<dyn crate::events::CatalogEventSource>,
) -> io::Result<RunningDaemon> {
    start_on_socket(
        ControlSocket::acquire(socket_path)?,
        poll_interval,
        manager,
        worker,
        walker,
        rerun_url,
        demand_plane,
        event_source,
    )
}

/// [`start_with_planes`] on a [`ControlSocket`] the caller ALREADY acquired — the
/// production entry, so the single-instance election precedes the viz stream.
#[allow(clippy::too_many_arguments)]
pub fn start_on_socket(
    socket: ControlSocket,
    poll_interval: Duration,
    manager: Arc<TransportManager>,
    worker: VizLogWorker,
    walker: FrameWalker,
    rerun_url: Option<String>,
    demand_plane: Arc<dyn DemandPlane>,
    event_source: Arc<dyn crate::events::CatalogEventSource>,
) -> io::Result<RunningDaemon> {
    let ControlSocket {
        listener,
        guard,
        path: socket_path,
    } = socket;

    let counters = worker.counters();
    // Take the worker's control handle BEFORE moving the worker onto the poll
    // thread (the `set_blueprint` verb). Shared across controller threads (via
    // `Ctx`) + held by `RunningDaemon` so `shutdown` can `close()` it first.
    let worker_control = worker.control();
    let shutdown = Arc::new(AtomicBool::new(false));
    let ctx = Ctx {
        state: Arc::new(Mutex::new(DaemonState::default())),
        manager,
        demand_plane,
        walker: Arc::new(RwLock::new(Arc::new(walker))),
        seed: Arc::new(Mutex::new(SeedState::default())),
        rebuild_lock: Arc::new(Mutex::new(())),
        counters,
        worker_control: Arc::clone(&worker_control),
        layout_lock: Arc::new(Mutex::new(LayoutMode::default())),
        logged_entities: Arc::new(Mutex::new(std::collections::BTreeSet::new())),
        rerun_url,
        events: Arc::new(crate::events::EventHub::new()),
        conn_loop_iterations: Arc::new(AtomicU64::new(0)),
        poll_period_ns: Arc::new(AtomicU64::new(0)),
        poll_iterations: Arc::new(AtomicU64::new(0)),
        poll_sleepless_passes: Arc::new(AtomicU64::new(0)),
        poll_late_passes: Arc::new(AtomicU64::new(0)),
        poll_injected_work_ns: Arc::new(AtomicU64::new(0)),
        runs_local_delay_ns: Arc::new(AtomicU64::new(0)),
        poll_wake_waits: Arc::new(AtomicU64::new(0)),
        poll_wake_fires: Arc::new(AtomicU64::new(0)),
        poll_layout_signal_reflows: Arc::new(AtomicU64::new(0)),
        poll_frames_drained: Arc::new(AtomicU64::new(0)),
        poll_wake_paced_passes: Arc::new(AtomicU64::new(0)),
        poll_wake_demotions: Arc::new(AtomicU64::new(0)),
        poll_backlog_passes: Arc::new(AtomicU64::new(0)),
        monitor_samples_taken: Arc::new(AtomicU64::new(0)),
        catalog_gather_seq: Arc::new(AtomicU64::new(0)),
        monitor_catalog_samples_taken: Arc::new(AtomicU64::new(0)),
        monitor_catalog_rows_retired: Arc::new(AtomicU64::new(0)),
        monitor_alerts_raised: Arc::new(AtomicU64::new(0)),
        monitor_alerts_cleared: Arc::new(AtomicU64::new(0)),
        monitor_rows_ineligible: Arc::new(AtomicU64::new(0)),
    };
    let conns: Arc<Mutex<Vec<JoinHandle<()>>>> = Arc::new(Mutex::new(Vec::new()));

    // Poll thread: drains taps → stats + worker. OWNS the worker (dropped on
    // shutdown → the worker's own bounded teardown flush).
    let poll_handle = {
        let ctx = ctx.clone();
        let shutdown = Arc::clone(&shutdown);
        std::thread::Builder::new()
            .name("vizd-poll".to_string())
            .spawn(move || poll_loop(ctx, worker, shutdown, poll_interval))?
    };

    // Accept loop: nonblocking accept + a thread per connection. The write-fail
    // latch is SHARED across every controller thread so a fleet of wedged
    // controllers (each contributing one write-timeout) never storms stderr.
    let write_fail_latch = Arc::new(Mutex::new(FieldsWarnLatch::new()));
    let accept_handle = {
        let ctx = ctx.clone();
        let sd = Arc::clone(&shutdown);
        let conns = Arc::clone(&conns);
        match std::thread::Builder::new()
            .name("vizd-accept".to_string())
            .spawn(move || accept_loop(listener, ctx, sd, conns, write_fail_latch))
        {
            Ok(h) => h,
            Err(e) => {
                // The poll thread is ALREADY running. A bare `?` here would drop
                // its JoinHandle (detaching it) and return — orphaning the poll
                // thread FOREVER: it spins on `shutdown` (never flipped), pinning
                // the worker + taps + transport alive and never running its own
                // bounded teardown flush. Flip the shutdown flag and JOIN it
                // before propagating, so a failed accept-thread spawn tears the
                // daemon down cleanly. `close()` the worker control FIRST so the
                // worker's `recv` reaches `Disconnected` promptly once the poll
                // thread drops it (else its drop-join spins to the 6 s detach).
                // `guard` (socket + pidfile + flock) drops on this early return.
                worker_control.close();
                shutdown.store(true, Ordering::SeqCst);
                let _ = poll_handle.join();
                return Err(e);
            }
        }
    };

    // The catalog-change forwarder — holds ONE upstream subscription and
    // relays every change into the hub. A spawn failure is NOT fatal: the daemon runs
    // exactly as it does without the forwarder, with `subscribe_events` reporting
    // `connected: false`, rather than failing to start over an enhancement.
    let events_handle = {
        let hub = Arc::clone(&ctx.events);
        let sd = Arc::clone(&shutdown);
        match crate::events::spawn_forwarder(event_source, hub, sd) {
            Ok(h) => Some(h),
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "cerulion-vizd: could not spawn the catalog-change forwarder — controllers \
                     keep their own refresh path"
                );
                None
            }
        }
    };

    tracing::info!(socket = %socket_path.display(), "cerulion-vizd listening (NDJSON control protocol)");
    Ok(RunningDaemon {
        shutdown,
        accept_handle: Some(accept_handle),
        poll_handle: Some(poll_handle),
        events_handle,
        events: Arc::clone(&ctx.events),
        conn_loop_iterations: Arc::clone(&ctx.conn_loop_iterations),
        poll_period_ns: Arc::clone(&ctx.poll_period_ns),
        poll_iterations: Arc::clone(&ctx.poll_iterations),
        poll_sleepless_passes: Arc::clone(&ctx.poll_sleepless_passes),
        poll_late_passes: Arc::clone(&ctx.poll_late_passes),
        poll_wake_waits: Arc::clone(&ctx.poll_wake_waits),
        poll_wake_fires: Arc::clone(&ctx.poll_wake_fires),
        poll_layout_signal_reflows: Arc::clone(&ctx.poll_layout_signal_reflows),
        poll_frames_drained: Arc::clone(&ctx.poll_frames_drained),
        poll_wake_paced_passes: Arc::clone(&ctx.poll_wake_paced_passes),
        poll_wake_demotions: Arc::clone(&ctx.poll_wake_demotions),
        poll_backlog_passes: Arc::clone(&ctx.poll_backlog_passes),
        monitor_samples_taken: Arc::clone(&ctx.monitor_samples_taken),
        monitor_catalog_samples_taken: Arc::clone(&ctx.monitor_catalog_samples_taken),
        monitor_catalog_rows_retired: Arc::clone(&ctx.monitor_catalog_rows_retired),
        monitor_alerts_raised: Arc::clone(&ctx.monitor_alerts_raised),
        monitor_alerts_cleared: Arc::clone(&ctx.monitor_alerts_cleared),
        monitor_rows_ineligible: Arc::clone(&ctx.monitor_rows_ineligible),
        // Gated with the field (see `RunningDaemon::poll_injected_work_ns`).
        #[cfg(any(test, feature = "test-helpers"))]
        poll_injected_work_ns: Arc::clone(&ctx.poll_injected_work_ns),
        #[cfg(any(test, feature = "test-helpers"))]
        runs_local_delay_ns: Arc::clone(&ctx.runs_local_delay_ns),
        conns,
        worker_control,
        guard: Some(guard),
        socket_path,
    })
}

/// The drain loop's DEADLINE TICKER — where the next wake lands,
/// given the deadline just served, the time it is now, and the configured
/// interval. All three in nanoseconds against ONE monotonic origin.
///
/// PURE (no clock read, no sleep) so the arithmetic is oracle-testable: an
/// assertion about a 10-interval stall cannot be written against a loop that has
/// to actually stall for it.
///
/// **Why a ticker at all.** A loop that does `sleep(poll_interval)` AFTER its
/// work has an effective period of `interval + work + wake lag`, never
/// `interval` — MEASURED at a **20.3 ms median against a
/// configured 16 ms** (see `poll_period_test.rs`, which prints both numbers).
/// Waiting until the next GRID POINT instead of for a fixed span after the work
/// puts the lag inside the interval rather than on top of it.
///
/// The rules, each load-bearing:
///
/// * **On time** (`deadline + interval` is still in the future) — exactly one
///   grid step.
/// * **Late** — the missed ticks are DROPPED and the next wake is the first grid
///   point STRICTLY AFTER `now`. Two consequences: the returned deadline is
///   never in the past, so a stall can never be followed by a BURST of
///   zero-length sleeps "catching up" (the catch-up is clamped to at most one
///   interval of waiting); and the cadence stays ON GRID, i.e. phase-preserving,
///   so the loop after a stall ticks on the same instants it would have.
/// * **A clock regression** (`now` at or before `deadline`, which `Instant`
///   forbids but a caller could still hand in) takes the on-time arm — one grid
///   step forward, never a jump back.
/// * **A zero interval** collapses the grid onto `now`, so every subsequent pass
///   takes the already-late arm of [`plan_tick_sleep`] and is paced by
///   [`LATE_PASS_YIELD`] instead of dividing by zero — a bounded degradation,
///   not an unbounded hot spin.
///
/// **What this does NOT decide** is how long to wait — see [`plan_tick_sleep`],
/// which is where the duty-cycle floor lives. Splitting the two is deliberate:
/// this function's job is *which grid point is next*, and it can return a
/// deadline already behind `now` (the `interval == 0` and saturation cases), so
/// a caller reading it as "the time to sleep to" would wait zero.
///
/// This changes only WHEN the loop wakes — never which frames it drains, in what
/// order, or what it does with them.
fn next_tick_deadline_ns(deadline_ns: u64, now_ns: u64, interval_ns: u64) -> u64 {
    if interval_ns == 0 {
        return now_ns;
    }
    let next = deadline_ns.saturating_add(interval_ns);
    if next > now_ns {
        return next;
    }
    // Late by at least one whole interval: skip the missed grid points in one
    // step so the next deadline is strictly in the future.
    let behind = now_ns - deadline_ns; // >= interval_ns (the arm above)
    let skips = behind / interval_ns + 1; // >= 2
    deadline_ns.saturating_add(skips.saturating_mul(interval_ns))
}

/// The drain loop's DUTY-CYCLE FLOOR: the
/// wait an ALREADY-LATE pass still takes, so the loop can never run back-to-back
/// with no wait at all.
///
/// **Why a floor exists.** Waiting to a deadline means that once a pass's own
/// wall exceeds the interval it exceeds it on every pass thereafter — the
/// remaining time is negative every time, so a bare "sleep the remainder" loop
/// takes ZERO waits for as long as the overrun lasts. The old fixed post-work
/// `sleep(poll_interval)` yielded unconditionally, so removing the wait outright
/// would have made this loop *less* like its predecessor than a small floor
/// does; the floor is what keeps "record-only" true of duty cycle as well
/// as about drain semantics.
///
/// **Why 200 µs.** The cost of the no-wait regime is that the drain re-acquires
/// [`Ctx::state`] microseconds after releasing it, and every controller verb
/// (`list`/`status`/`attach` — ~25 `state.lock()` sites) contends for that same
/// lock with no guaranteed gap. Both ends of this were measured on
/// one machine: with a ~2 µs window between release and re-acquire a verb waited a
/// median of 1 and a maximum of 11 drain passes, and a paired run of the two
/// loop shapes under a 20 ms-per-pass overrun put the verb's lock-acquire p50 at
/// 0 µs (fixed sleep) against 22 ms (deadline ticker, no floor), with 3.9× fewer verbs served; with
/// a **200 µs** window the same overrun measured 0/0/0 passes waited — identical
/// to the fixed-sleep shape. So the floor is sized at the smallest measured window
/// that matches the fixed-sleep behaviour, not at a round number.
///
/// **Why it is small — stated at both the requested and the REALIZED size.**
/// `thread::sleep` does not return at the requested instant, so the constant is
/// not the cost. MEASURED on macOS (200 samples per point):
/// `sleep` dilates by **~1.5×** across the whole 50 µs – 2 ms band
/// (`50→77 µs`, `100→153`, `200→303`, `400→636`, `1000→1508`, `2000→3004`,
/// p50), so:
///
/// | | requested | realized p50 | of a 16 ms interval | vs the ~4.5 ms drift |
/// |---|---|---|---|---|
/// | this floor | 200 µs | ~303 µs | 1.25 % → **1.89 %** | 22× → **~15× smaller** |
///
/// Both numbers are far below the drift the deadline wait exists to remove, and the
/// floor is taken ONLY on a pass that already missed its deadline, so a healthy
/// loop never pays it at all. The dilation is modelled explicitly by
/// [`SLEEP_GRANULARITY_FACTOR`] so the drift-guard test bounds the REALIZED
/// floor rather than the constant.
///
/// It also does not contradict the backlog-aware arm, which drops the
/// wait *while a backlog exists* — a condition on the DATA, decided per pass,
/// not a standing property of the thread.
const LATE_PASS_YIELD: Duration = Duration::from_micros(200);

/// The modelled ratio of a realized `thread::sleep` to its
/// requested duration, used to bound [`LATE_PASS_YIELD`]'s REAL cost.
///
/// MEASURED at 1.50–1.63× across 50 µs – 2 ms (see
/// [`LATE_PASS_YIELD`]); modelled at **2×**, conservatively above the worst
/// measured point, because the guard must hold on runners nobody has measured.
/// A multiplier is the right shape: the overhead grows WITH the request
/// (+27 µs at 50 µs, +103 at 200, +1004 at 2000), it is not a fixed offset.
const SLEEP_GRANULARITY_FACTOR: u32 = 2;

/// The REALIZED floor — the constant times the modelled
/// dilation — must stay under ~3 % of the poll interval. Const-asserted rather
/// than only tested, because the quantity being bounded is a pair of constants
/// and a compile error is the right place to catch a bump to either.
///
/// Bounding the REQUESTED constant alone bounds the wrong thing: at the shipped
/// 200 µs the realized p50 is ~303 µs (1.89 % of a 16 ms interval), and a bump
/// to 300 µs realizes ~3.1 % — which a guard stated on the
/// constant alone, with a 2 % message, would accept.
const _: () = assert!(
    LATE_PASS_YIELD.as_nanos() * SLEEP_GRANULARITY_FACTOR as u128 * 34
        <= DEFAULT_POLL_INTERVAL.as_nanos(),
    "the realized late-pass floor (LATE_PASS_YIELD x SLEEP_GRANULARITY_FACTOR) \
     must stay under ~3% of DEFAULT_POLL_INTERVAL"
);

/// The modelled dilation must remain a genuine OVER-estimate of what was
/// measured (1.50–1.63× across 50 µs – 2 ms), or the guard above is
/// bounding an optimistic fiction rather than the floor a robot really pays.
const _: () = assert!(
    SLEEP_GRANULARITY_FACTOR >= 2,
    "SLEEP_GRANULARITY_FACTOR must stay at or above the measured worst-case \
     sleep dilation (1.63x), rounded up"
);

/// How long the drain loop waits at the end of a pass, given
/// the deadline it is aiming at and the time it is now.
///
/// PURE, and split out from [`next_tick_deadline_ns`] so the floor is testable
/// without a loop that has to genuinely overrun for it.
///
/// * **Ahead of the deadline** — the remaining time, exactly. This is the whole
///   of the deadline fix: the wait shrinks by however long the pass took, so the
///   period is the interval rather than `interval + work`.
/// * **At or past the deadline** — [`LATE_PASS_YIELD`], never zero.
///
/// The returned duration is therefore ALWAYS nonzero, which is the property the
/// sleepless-pass counter pins. (`Duration::ZERO` would not even be a yield:
/// std's unix `sleep` never enters its retry loop for a zero duration, so it
/// costs ~1 ns and issues no syscall.)
fn plan_tick_sleep(deadline_ns: u64, now_ns: u64) -> Duration {
    if is_late_pass(deadline_ns, now_ns) {
        return LATE_PASS_YIELD;
    }
    Duration::from_nanos(deadline_ns - now_ns)
}

/// Has this pass already reached or passed its deadline?
///
/// ONE predicate, shared by [`plan_tick_sleep`] (which arm to take) and by the
/// loop (which counter to bump), so the `poll_late_passes` observable cannot
/// drift out of agreement with the arm it claims to be counting — a drift that
/// would let the stimulus assertion in `poll_period_test.rs` pass while the
/// regime it names was never entered.
fn is_late_pass(deadline_ns: u64, now_ns: u64) -> bool {
    deadline_ns <= now_ns
}

/// Nanoseconds since `origin`, saturating (u64 ns is ~584 years).
fn elapsed_ns(origin: Instant) -> u64 {
    u64::try_from(origin.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

/// The most consecutive passes that may SKIP their wait on a
/// drained backlog before one is forced to wait anyway.
///
/// **This bound is the notify storm's structural defence, not a tuning knob.**
/// The listener's event queue is drained by the multiplexer's callback, which
/// only runs inside a wait — so a pass that skips its wait leaves the listener
/// UNDRAINED. A topic publishing faster than the loop drains would take the
/// backlog arm on every pass forever, its `AF_UNIX SOCK_DGRAM` socket would
/// fill, and then EVERY publisher notify pays `FailedToDeliverSignal` plus a
/// ~2 KB iceoryx2 warn, the exact failure listener-less taps removed, re-created on the
/// desk's own netd.
///
/// Eight keeps the skip useful (a `POLL_MAX`-capped drain gets eight immediate
/// re-drains, i.e. up to 32 768 frames per tap with no wait) while the undrained
/// window stays a handful of passes rather than unbounded. The forced wait costs
/// `LATE_PASS_YIELD` (200 µs) at worst, since a loop in that regime is already
/// past its deadline.
const WAKE_BACKLOG_MAX_CONSECUTIVE: u32 = 8;

/// Consecutive FIRED-BUT-EMPTY waits tolerated before the loop
/// starts pacing itself: the observer-pacing shape, applied to this loop.
///
/// A wake that returns instantly and yields no frames costs a whole pass. A
/// producer ringing the event service without publishing (the observer-pacing fix measured a
/// real one at ~3 000 000 events over 2.3 s) would therefore spin this loop at
/// 100 % of a core, because the wait's budget is never spent. Past this
/// tolerance the loop sleeps its remaining budget AFTER the wake, which bounds
/// the rate without ever skipping the drain.
///
/// Deliberately > 1: a single fired-but-empty wait is NORMAL (notifications
/// coalesce, and a connection-lifecycle event is not a frame), so pacing on the
/// first one would add latency to a healthy topic for nothing.
const WAKE_IDLE_CHURN_TOLERANCE: u32 = 16;

/// How many consecutive times a topic's wake may fire without
/// that topic yielding a frame before its listener is DROPPED and the topic
/// reverts to the timer path.
///
/// A wake is a bill: the producer pays one `sendto` per notify. A topic that
/// keeps ringing and never delivers is billing its producer for nothing, and it
/// is exactly the notify-flood signature (a notifier with no publisher behind it).
/// Demotion stops paying; the tap, the SHM queue and every frame are untouched,
/// so the cost is latency only, and the producer's own notify-elision gate
/// re-arms within one publish.
///
/// Sized well above any coalescing artefact: a genuine stream cannot fire 64
/// consecutive times while delivering nothing, because a fire means a notify and
/// a notify means a publish reached the topic.
const WAKE_DEMOTE_STREAK: u32 = 64;

/// What the drain loop does at the END of a pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PassWait {
    /// Skip the wait entirely and re-drain now (a backlog was found).
    Skip,
    /// Block on the wake set for the pass's budget. `pace` additionally sleeps
    /// the budget AFTER the wake returns, bounding a wake flood.
    Wake { pace: bool },
    /// `thread::sleep` the budget — the deadline-timer loop, for a daemon with no wake.
    Timer,
}

/// The drain loop's PURE wait policy.
///
/// Split out from the loop so both bounds it enforces are testable against hand
/// oracles at their exact thresholds. Neither is reachable from an ordinary
/// arrangement of topics (one needs a producer that outruns the drain
/// indefinitely, the other a notifier storm), and both guard a failure that is
/// loud, remote and expensive when it does happen.
#[derive(Debug, Default)]
struct WakeGovernor {
    /// Consecutive passes that took [`PassWait::Skip`].
    consecutive_skips: u32,
    /// Consecutive passes whose PREVIOUS wait fired and which then drained
    /// nothing.
    idle_churn: u32,
}

impl WakeGovernor {
    /// Decide this pass's wait.
    ///
    /// * `drained_any` — did THIS pass's drain yield frames?
    /// * `wake_armed` — is there at least one wake source to block on?
    /// * `last_wait_fired` — did the PREVIOUS wait end on a fired source?
    fn decide(&mut self, drained_any: bool, wake_armed: bool, last_wait_fired: bool) -> PassWait {
        // Churn is "a wake fired and there was nothing to show for it". Any
        // productive pass clears it, so a healthy topic never paces.
        if drained_any {
            self.idle_churn = 0;
        } else if last_wait_fired {
            self.idle_churn = self.idle_churn.saturating_add(1);
        }
        if !wake_armed {
            self.consecutive_skips = 0;
            return PassWait::Timer;
        }
        if drained_any && self.consecutive_skips < WAKE_BACKLOG_MAX_CONSECUTIVE {
            self.consecutive_skips += 1;
            return PassWait::Skip;
        }
        self.consecutive_skips = 0;
        PassWait::Wake {
            pace: self.idle_churn >= WAKE_IDLE_CHURN_TOLERANCE,
        }
    }
}

/// Per-topic wake demotion bookkeeping — which topics' wakes
/// FIRED on the previous wait, and how long each has been firing without
/// delivering.
#[derive(Debug, Default)]
struct WakeDemoter {
    /// Topics whose source fired on the most recent wait, awaiting this pass's
    /// drain to say whether that fire was productive.
    fired_last_wait: Vec<String>,
    /// Per-topic consecutive fired-but-undelivered count.
    streaks: std::collections::BTreeMap<String, u32>,
}

impl WakeDemoter {
    /// Settle the previous wait's fires against this pass's drained topics and
    /// return every topic whose streak has reached [`WAKE_DEMOTE_STREAK`].
    ///
    /// PURE (no transport) so the threshold is testable at its exact boundary. A
    /// returned topic is REMOVED from the bookkeeping — a demotion spends its
    /// evidence, so a topic re-attached later starts clean rather than being
    /// demoted again on its first quiet wake.
    fn settle(&mut self, drained_topics: &[String]) -> Vec<String> {
        let mut demote = Vec::new();
        for topic in std::mem::take(&mut self.fired_last_wait) {
            if drained_topics.contains(&topic) {
                self.streaks.remove(&topic);
                continue;
            }
            let streak = self.streaks.entry(topic.clone()).or_insert(0);
            *streak += 1;
            if *streak >= WAKE_DEMOTE_STREAK {
                demote.push(topic);
            }
        }
        for topic in &demote {
            self.streaks.remove(topic);
        }
        demote
    }

    /// Record the topics whose sources fired on the wait just completed.
    fn note_fired(&mut self, topics: impl IntoIterator<Item = String>) {
        self.fired_last_wait.clear();
        self.fired_last_wait.extend(topics);
    }
}

/// The poll thread body: every `poll_interval`, drain each tap, fold per-topic
/// Hz/schema stats from the frames' wire headers, and hand the batch to the
/// never-block worker. Exits when the shutdown flag flips (dropping `worker` →
/// its bounded teardown flush).
///
/// The wait is a DEADLINE, not a fixed post-work sleep — see
/// [`next_tick_deadline_ns`] and [`plan_tick_sleep`]. Record-only in the sense
/// that matters: the same taps are drained, in the same order, under the same
/// locks, and every frame is handed on unchanged.
///
/// Two second-order consequences of restoring the cadence, stated because
/// "record-only" does not cover them:
///
/// * **More, smaller batches.** MEASURED on a desk machine the period went 20.3-21.9
///   ms → 15.8-16.2 ms, so the loop enqueues ~30 % more batches per second, each
///   carrying proportionally fewer frames. The worker's queue is bounded in
///   SLOTS, so its depth in WALL time shrinks by the same ~24 % — against the
///   ≈130 ms [`DEFAULT_POLL_INTERVAL`] derives at the nominal rate. The
///   direction is toward the designed behaviour (every constant here, that
///   derivation included, was sized against 16 ms), but a wedged viewer now
///   starts dropping frames ~24 % sooner in wall time than it did, which a
///   future look at `dropped_frames` timing should know.
/// * **Duty cycle.** See [`plan_tick_sleep`] — the unconditional post-work sleep
///   became a conditional one, and the floor is what keeps a guaranteed
///   lock-free window per pass.
fn poll_loop(
    ctx: Ctx,
    mut worker: VizLogWorker,
    shutdown: Arc<AtomicBool>,
    poll_interval: Duration,
) {
    // Flood-latch for the walker-swap-deferred debug: loud-once per
    // wedged-viewer regime, re-armed when a swap finally lands, so a sustained
    // wedge never storms the log at the poll cadence.
    let mut swap_retry_latch = FieldsWarnLatch::new();
    // The previous iteration's start mark. The period is measured
    // between two consecutive tops of the body, so ONE sample covers the whole
    // iteration — drain work, lock, hand-off AND wait — which is the quantity the
    // configured interval claims to be. `None` until the second iteration.
    let mut last_iteration_start: Option<Instant> = None;
    // The tick grid. ONE monotonic origin, and every deadline is
    // an offset from it, so the pacing decision is the pure
    // `next_tick_deadline_ns` and nothing else.
    let origin = Instant::now();
    let interval_ns = u64::try_from(poll_interval.as_nanos()).unwrap_or(u64::MAX);
    let mut deadline_ns = interval_ns;
    // The event multiplexer this loop blocks on when any tap
    // carries a wake listener. Built ONCE (it owns only reusable buffers; the
    // sources are passed per wait, so an attach/detach needs no rebuild).
    //
    // A failure to build it is NOT fatal: the loop falls back to the deadline timer
    // for the daemon's whole life, loudly. Latency is what is lost, never frames
    // — the timeout was always the fallback, so this is the same degraded state
    // a topic in `WakeMode::Timer` is in.
    let mut wake_set = match cerulion_core::wake::WakeSet::new(WAKE_SET_CAPACITY_HINT) {
        Ok(set) => Some(set),
        Err(e) => {
            tracing::warn!(
                error = %e,
                "cerulion-vizd: could not build the frame-drain wake set — every topic \
                 falls back to the {poll_interval:?} poll cadence for this daemon's \
                 lifetime (frames still arrive, one interval later)"
            );
            None
        }
    };
    // Flood-latch for an over-capacity wake-set refusal: it can only fire with
    // >1024 attached taps, and if it ever does it would fire on EVERY pass.
    let mut wake_refusal_latch = FieldsWarnLatch::new();
    // The two bounds (backlog-skip cap + wake-flood pacing) and
    // the per-topic demotion bookkeeping. Both are PURE decisions — see
    // `WakeGovernor` / `WakeDemoter`.
    let mut governor = WakeGovernor::default();
    let mut demoter = WakeDemoter::default();
    // Did the PREVIOUS wait end on a fired source? The churn signal is
    // "a wake fired and the next drain had nothing to show for it".
    let mut last_wait_fired = false;
    // The worker's layout-signal generation as of the previous pass. Zero
    // to start, which is what a worker that has published nothing reads — so the
    // first signal any topic produces is itself a change and reflows.
    let mut last_layout_generation: u64 = 0;
    // Monitors: when the standing watchdog last sampled. PASS-LOCAL,
    // like the layout generation above and for the same reason — the cadence
    // belongs to this loop, and a shared cell would let a control handler change
    // when the watchdog observes.
    //
    // `None` samples on the FIRST pass, deliberately: a row's settle window and its
    // confirmation span are both measured from when the engine first SAW it, so
    // starting immediately is what makes those windows describe the daemon's uptime
    // rather than the sampler's phase.
    let mut last_monitor_sample: Option<Instant> = None;
    while !shutdown.load(Ordering::Acquire) {
        let iteration_start = Instant::now();
        ctx.poll_iterations.fetch_add(1, Ordering::Relaxed);
        if let Some(prev) = last_iteration_start {
            ctx.poll_period_ns.store(
                iteration_start.saturating_duration_since(prev).as_nanos() as u64,
                Ordering::Relaxed,
            );
        }
        last_iteration_start = Some(iteration_start);
        // Remote arm: a schema-less remote attach grew the daemon's walker
        // with a robot-served type's closure and queued the SAME walker here. Hand
        // it to the render worker on THIS (the worker's) thread, so it is FIFO with
        // the data batches enqueued below (frames after the swap decode against the
        // new schema set — deterministic). `take` so it applies exactly once.
        //
        // The hand-off is NON-BLOCKING (`try_swap_walker`). A
        // wedged viewer keeps the worker's bounded queue FULL, and a blocking swap
        // would park THIS poll thread forever → every tap stops draining, the whole
        // viz plane hangs. On a full queue the walker is RETAINED (put back) and
        // retried next poll (loud-once debug on sustained retry), never dropped —
        // so the swap lands the moment the queue drains.
        let pending_swap = ctx.seed.lock().unwrap().pending_worker_swap.take();
        if let Some(new_walker) = pending_swap {
            if let Some(retained) = worker.try_swap_walker(new_walker) {
                // Queue full (viewer wedged) — retain for the next iteration. Do
                // NOT clobber a NEWER swap another attach queued meanwhile
                // (`get_or_insert` keeps the newer, monotone-superset walker).
                let mut seed = ctx.seed.lock().unwrap();
                if seed.pending_worker_swap.is_none() {
                    seed.pending_worker_swap = Some(retained);
                    if swap_retry_latch.on_inferred() == FieldsLogAction::WarnFirst {
                        tracing::debug!(
                            "vizd: viz worker queue full — deferring the walker swap to the next \
                             poll (a wedged viewer); the new schema set installs once it drains"
                        );
                    }
                }
            } else {
                // Applied — re-arm the retry latch so a future full-queue regime
                // logs its first deferral again.
                swap_retry_latch.on_decoded();
            }
        }
        // Snapshot the resolution walker ONCE per poll (a read lock + Arc clone) so
        // the drain loop below sees a consistent schema set even if a seed swaps it
        // mid-poll.
        let walker = ctx.walker_snapshot();
        // Did ANY attached topic transition resolved None→Some THIS poll?
        // A topic attached while momentarily silent resolves NO archetype, so the
        // dynamic default layout (auto_views:false) placed it in NO view. When its
        // first decodable frame arrives here, re-derive + re-apply the default so the
        // late-resolving topic reflows into its view (below, OUTSIDE the state lock).
        let mut newly_resolved = false;
        // The wake sources are collected inside the SAME lock
        // scope as the drain (it is already held) and used strictly OUTSIDE it.
        // A wait taken under `ctx.state` would stall every control verb for its
        // whole duration — the very problem this chunk removes, re-created.
        let wake_sources: Vec<Arc<cerulion_core::wake::WakeSource>>;
        // The topics this pass drained, for the demoter to
        // settle the PREVIOUS wait's fires against.
        let mut drained_topics: Vec<String> = Vec::new();
        let batch = {
            let mut st = ctx.state.lock().unwrap();
            let polled = st.taps.poll_detailed(POLL_MAX);
            // Settle + demote BEFORE the wake sources are read,
            // so a demoted topic's listener is gone from THIS pass's wait rather
            // than one pass later.
            drained_topics.extend(polled.iter().map(|p| p.topic.clone()));
            for topic in demoter.settle(&drained_topics) {
                if st.taps.demote_to_timer(&topic) {
                    ctx.poll_wake_demotions.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!(
                        topic = %topic,
                        streak = WAKE_DEMOTE_STREAK,
                        "cerulion-vizd: this topic's wake fired {WAKE_DEMOTE_STREAK} times \
                         running without delivering a frame — dropping its listener and \
                         reverting it to the {poll_interval:?} poll cadence. The topic keeps \
                         delivering; its producer stops paying a notify for nothing. \
                         Re-attach the topic to try the wake again."
                    );
                }
            }
            wake_sources = st.taps.wake_sources();
            let now = Instant::now();
            let mut input_frames = Vec::with_capacity(polled.len());
            for p in polled {
                let stat = st.stats.entry(p.topic.clone()).or_default();
                // Bank the arrival against this tap's own observation.
                // A tap that yields frames IS observing, so a stats entry the
                // attach path somehow did not open (a topic re-entering the poll
                // set) still starts its observation here rather than reporting
                // UNKNOWN forever.
                stat.begin_observation(now);
                stat.observe_frames(p.frames.len() as u64, now);
                // The daemon-wide counted twin (Principle #3).
                // Bumped HERE, beside the per-topic count it aggregates, so the
                // two can never disagree about what the loop saw.
                ctx.poll_frames_drained
                    .fetch_add(p.frames.len() as u64, Ordering::Relaxed);
                // Newest frame (highest seq) drives the Hz window — and also
                // the pre-decode loss check: the batch LENGTH is what turns a
                // sequence jump into a verdict (a jump equal to the frames handed
                // over is a healthy multi-frame poll, not a gap).
                //
                // Composition with the wait arms: this arithmetic sits in the
                // DRAIN, at the top of the pass, and the three wait arms
                // (backlog-skip / wake / timer) are all at the BOTTOM — they
                // change only WHEN the next drain happens, never what it reads.
                // A tap that yielded nothing is not in `polled` at all, so
                // `last_newest_seq` is untouched on an empty pass and the
                // per-poll deltas still telescope. Waking more often produces
                // MORE, SMALLER batches, which moves the distribution of
                // `frames_missed` DOWN (fewer evictions) without touching the
                // identity `delivered + missed == published`.
                if let Some(header) = p.frames.last().and_then(|f| WireHeader::read_from_buf(f)) {
                    stat.observe_seq(header.sequence, p.frames.len() as u64, now, &p.topic);
                }
                // Resolve the schema ONCE, from the first decodable frame. A
                // remote attach's name guess does not count as resolved: the
                // frame verdict replaces it, and only a CHANGED archetype counts
                // as a new resolution for the layout reflow below.
                if !stat.resolved_from_frame {
                    if let Some(first) = p.frames.first() {
                        // Classified, not a bare `Option`. A topic whose
                        // frames this build cannot decode never resolves, so it
                        // re-enters this arm on EVERY poll — which is exactly the
                        // flood `record_resolution`'s latch exists to bound, and
                        // exactly the state a Studio row must be able to show.
                        // The pinned type name (set by the remote attach path) is
                        // what lets the verdict say "rebuild" or "acquire"
                        // instead of just printing a hash.
                        let candidate = stat.pinned_schema.clone();
                        let outcome = resolve_from_frame(&walker, first, candidate.as_deref());
                        let before = stat.resolved.as_ref().map(|r| r.archetype);
                        newly_resolved |=
                            outcome.as_ref().is_ok_and(|r| Some(r.archetype) != before);
                        record_resolution(stat, &p.topic, &outcome);
                    }
                }
                // The vizd-drain stage: the frame's SHM residency ends
                // here. `t_drain_ns` minus netd's `t_shm_ns` for the SAME wire
                // sequence IS the netd→vizd gap (bounded below by this loop's poll
                // cadence), which is the single largest un-instrumented leg of the
                // desk chain. The whole per-frame header scan is behind the gate, so
                // a disabled probe costs one `OnceLock` load per poll, not per frame.
                if cerulion_core::lat_probe::probe_enabled() {
                    for f in &p.frames {
                        let Some(header) = WireHeader::read_from_buf(f) else {
                            continue;
                        };
                        if cerulion_core::lat_probe::should_sample(header.sequence) {
                            tracing::info!(
                                stage = "s2_vizd_drain",
                                topic = %p.topic,
                                seq = header.sequence,
                                bytes = f.len(),
                                batch = p.frames.len(),
                                t_drain_ns = cerulion_core::lat_probe::wall_ns() as u64,
                                "latency probe stage"
                            );
                        }
                    }
                }
                input_frames.push(InputFrames {
                    name: p.route_key,
                    frames: p.frames,
                });
            }
            input_frames
        };
        // OUTSIDE the state lock (maybe_apply_default_layout re-locks `state` under
        // `layout_lock`): a late-resolve reflows the default layout. A NO-OP under an
        // explicit layout; bounded (a rare, at-most-once-per-topic event). Fix 1's
        // `layout_lock` serializes it against concurrent controller-driven applies.
        // ...and so does a change in any topic's LIVE LAYOUT SIGNAL. The
        // layout refuses the dump companion on live evidence, and that
        // evidence is produced by the WORKER (its render arms and its H.264 demux)
        // while the layout is built here — so without a watcher the decision would
        // only ever reach the viewer if some unrelated attach/detach happened to
        // reflow afterwards. Nothing else
        // detects a rendition-set change either, so without it the residual (a dump
        // raised after a rendition is observed lands with no pane) has no bound.
        //
        // ONE generation, covering BOTH signals. Watching the render-proof map
        // alone would close the general half and leave
        // VIDEO unwatched, because `render_is_proven` reads the
        // RENDITION SET for `VideoStream` and the proof map for everything else,
        // and on the ordinary mid-GOP camera attach the proof map reaches its
        // terminal value seconds before the first rendition appears. The bump lives
        // at the WRITE site (see
        // `cerulion_viz::worker::VizWorkerCounters::layout_signal_generation`),
        // which is also what keeps this loop's per-pass cost at one relaxed load
        // instead of a whole-map clone on a loop whose pass costs microseconds.
        //
        // Read OUTSIDE the state lock, like the reflow itself:
        // `maybe_apply_default_layout` re-locks `state`.
        let layout_generation = ctx.layout_signal_generation();
        let signal_changed = layout_generation != last_layout_generation;
        if signal_changed {
            last_layout_generation = layout_generation;
        }
        if newly_resolved || signal_changed {
            // Counted where the layout was RE-DERIVED, not where the change was
            // detected: `maybe_apply_default_layout` returns whether it got past
            // the `LayoutMode::Auto` gate and rebuilt the plan, so a pass under an
            // operator's explicit layout re-derives nothing and is not counted.
            // If it were counted at the detection site instead, dropping
            // `signal_changed` from the condition above would leave the counter climbing
            // on a daemon that never reflows, and no test would notice.
            // The observable cannot be true unless the reflow ran.
            let re_derived = ctx.maybe_apply_default_layout();
            if signal_changed && re_derived {
                ctx.poll_layout_signal_reflows
                    .fetch_add(1, Ordering::Relaxed);
            }
        }
        // Monitors: the standing per-topic watchdog's SAMPLER.
        //
        // OUTSIDE the state lock, beside the reflow above and for the same
        // reason: `sample_monitors` takes that lock itself, and std mutexes are not
        // reentrant.
        //
        // The gate is the whole cost story on a non-sampling pass — ONE `Instant`
        // compare, no lock, no allocation — which is what makes the monitors' claim
        // ("monitors add zero ports, zero subscribers, zero threads, zero netd
        // round trips") a property of the shape rather than a promise. Sampling at
        // the 16 ms poll cadence instead would re-read one robot-side observation
        // ~12 times (the observer's own sweep grid is 200 ms), counting the same
        // evidence over and over and confirming a fault in a fraction of the real
        // time; `MONITOR_SAMPLE_INTERVAL_NS` is twice that grid so every sample is
        // a fresh observation even when the two grids beat.
        //
        // The mark advances HERE, at the gate, while every counter is bumped inside
        // `sample_monitors` where the pass really ran — the same rule as
        // the reflow above (a bump at the DETECTION site leaves the
        // observable climbing on a daemon that does no work).
        //
        // A late pass simply samples late: the engine derives every duration from
        // the timestamps it is handed, so a stretched cadence delays a confirmation
        // rather than corrupting one.
        let monitor_due = last_monitor_sample.is_none_or(|prev| {
            iteration_start.saturating_duration_since(prev)
                >= Duration::from_nanos(MONITOR_SAMPLE_INTERVAL_NS)
        });
        if monitor_due {
            last_monitor_sample = Some(iteration_start);
            ctx.sample_monitors(iteration_start);
        }
        // Did this pass find anything? Read BEFORE the hand-off
        // moves the batch — it decides the backlog-aware arm below.
        let drained_any = !batch.is_empty();
        worker.try_enqueue(batch);
        // A test-only per-pass overrun injector.
        // The duty-cycle floor is only observable on a loop whose pass genuinely
        // outruns its interval, and the MEASURED cost of a real drain is
        // 0.009-0.30 ms against a 16 ms interval — 50x short — so no arrangement
        // of real topics reaches the regime. One relaxed load off an `Arc` per
        // pass when unset. Deliberately OUTSIDE the state lock: what is under
        // test is the WAIT, and holding `ctx.state` for the injected span would
        // stall every controller verb in the harness for reasons that are the
        // motivation, not the property.
        let injected_ns = ctx.poll_injected_work_ns.load(Ordering::Relaxed);
        if injected_ns > 0 {
            let until = Instant::now() + Duration::from_nanos(injected_ns);
            while Instant::now() < until {
                std::hint::spin_loop();
            }
        }
        // Wait until the next GRID POINT, not for a fixed span
        // after the work — and never for nothing at all (`plan_tick_sleep`'s
        // already-late arm yields `LATE_PASS_YIELD`). The ticker then drops the
        // missed ticks rather than bursting through them.
        let now_ns = elapsed_ns(origin);
        if is_late_pass(deadline_ns, now_ns) {
            // The STIMULUS observable. Strictly one-sided —
            // contention can only ADD late passes, never remove them — so a
            // FLOOR on it is the load-safe way for a test to prove the overrun
            // regime was actually entered rather than assuming it.
            ctx.poll_late_passes.fetch_add(1, Ordering::Relaxed);
        }
        let waited = plan_tick_sleep(deadline_ns, now_ns);
        if waited.is_zero() {
            // Unreachable while `plan_tick_sleep` floors its late arm — counted
            // rather than asserted, so removing the floor shows up as a NUMBER
            // (Principle #3) instead of as a duty-cycle change nothing reports.
            ctx.poll_sleepless_passes.fetch_add(1, Ordering::Relaxed);
        }
        // HOW this pass waits. `waited` is the deadline
        // budget either way — the wake only decides whether the budget can end
        // EARLY, never how long it is, so the timer contract is untouched and a
        // daemon with no wake source is byte-identical to the timer-only loop.
        //
        // Drain-before-wait is structural here: the drain is at the TOP of this
        // body and every wait is at the bottom, so a frame committed between two
        // waits is always seen by the drain that follows, notify or no notify
        // (Principle #6 — the send→notify race `wait_for_message` names).
        let armed = wake_set.as_mut().filter(|_| !wake_sources.is_empty());
        let plan = governor.decide(drained_any, armed.is_some(), last_wait_fired);
        last_wait_fired = false;
        match (armed, plan) {
            // BACKLOG-AWARE: this pass yielded frames, so the SHM queue may still
            // hold more (the drain is capped at `POLL_MAX`) and a coalesced
            // notify may already be spent. Re-drain now rather than wait.
            //
            // CAPPED at `WAKE_BACKLOG_MAX_CONSECUTIVE`, and the cap is the
            // storm defence: a skipped wait is a pass on which the listener's
            // event queue is NOT drained (only the multiplexer's callback drains
            // it), so an uncapped skip on a producer that outruns the loop would
            // fill that socket and put every notify on the failure path. Counted
            // because a zero-wait pass is the shape a spin takes.
            (Some(_), PassWait::Skip) => {
                ctx.poll_backlog_passes.fetch_add(1, Ordering::Relaxed);
            }
            (Some(set), PassWait::Wake { pace }) => {
                ctx.poll_wake_waits.fetch_add(1, Ordering::Relaxed);
                let refs: Vec<&cerulion_core::wake::WakeSource> =
                    wake_sources.iter().map(|s| s.as_ref()).collect();
                match set.wait(&refs, waited) {
                    Ok(fired) => {
                        if !fired.is_empty() {
                            ctx.poll_wake_fires.fetch_add(1, Ordering::Relaxed);
                            last_wait_fired = true;
                        }
                        // Remember WHICH topics rang, so the next pass's drain
                        // can say whether each fire was productive.
                        demoter.note_fired(
                            fired
                                .iter()
                                .filter_map(|&i| wake_sources.get(i))
                                .map(|s| s.topic().to_string()),
                        );
                    }
                    Err(e) => {
                        // The only refusal is an over-capacity source slice, which
                        // needs >1024 attached taps. Loud, and then FALL BACK to
                        // the timer for this pass — a latency optimisation must
                        // never be able to stop the loop.
                        if wake_refusal_latch.on_inferred() == FieldsLogAction::WarnFirst {
                            tracing::warn!(
                                error = %e,
                                sources = wake_sources.len(),
                                "cerulion-vizd: the frame-drain wake set refused its source \
                                 set — this pass falls back to the poll cadence (frames still \
                                 arrive, one interval later)"
                            );
                        }
                        std::thread::sleep(waited);
                    }
                }
                if pace {
                    // Observer pacing: the wake has been
                    // firing without delivering, so it is returning instantly and
                    // the budget is never spent. Spend it here. The DRAIN is
                    // untouched — pacing costs latency on a topic that is not
                    // delivering anything anyway, and it is the only thing
                    // between a notifier storm and 100 % of a core.
                    ctx.poll_wake_paced_passes.fetch_add(1, Ordering::Relaxed);
                    std::thread::sleep(waited);
                }
            }
            // No wake source (every tap is `WakeMode::Timer`, or none is
            // attached): the deadline-timer loop, unchanged.
            _ => std::thread::sleep(waited),
        }
        deadline_ns = next_tick_deadline_ns(deadline_ns, elapsed_ns(origin), interval_ns);
    }
    // `worker` drops here → its Drop drains the backlog + bounded final flush.
}

/// Hand-written oracle vectors for the deadline ticker.
///
/// Every expectation below is computed by hand from the grid (`I`, `2I`, `3I`, …)
/// and written as a literal — never as the function's own output. The properties
/// that matter are the ones a loop-level test cannot reach: what happens after a
/// TEN-interval stall, and whether the returned deadline can ever land in the
/// past (which is what turns a stall into a burst of zero-length sleeps).
#[cfg(test)]
mod tick_deadline_tests {
    use super::{next_tick_deadline_ns, plan_tick_sleep, LATE_PASS_YIELD};

    /// 16 ms, in nanoseconds — the shipped [`super::DEFAULT_POLL_INTERVAL`].
    const I: u64 = 16_000_000;

    #[test]
    fn steady_state_advances_by_exactly_one_interval() {
        // Served the deadline at `I`, woke 1.2 ms late (the real shape: `sleep`
        // returns at or after its deadline, never before). The next wake is the
        // NEXT GRID POINT `2I` — NOT `now + I`, which is what a fixed post-work
        // sleep does and is precisely where the measured 20.3 ms median came from.
        assert_eq!(next_tick_deadline_ns(I, I + 1_200_000, I), 2 * I);
        assert_eq!(next_tick_deadline_ns(2 * I, 2 * I + 900_000, I), 3 * I);
        // Woke EARLY relative to the next grid point (the loop's work finished
        // fast): still exactly one step.
        assert_eq!(next_tick_deadline_ns(3 * I, 3 * I + 1, I), 4 * I);
    }

    #[test]
    fn one_late_iteration_resumes_on_the_next_grid_point() {
        // Overran the next grid point by 3 ms: `2I` is already gone, so that tick
        // is DROPPED and the next wake is `3I` — still on the original phase.
        let now = 2 * I + 3_000_000;
        assert_eq!(next_tick_deadline_ns(I, now, I), 3 * I);
        // And it is strictly in the future, so the loop sleeps rather than
        // spinning through a missed tick.
        assert!(next_tick_deadline_ns(I, now, I) > now);
    }

    #[test]
    fn a_ten_interval_stall_resumes_on_grid_without_a_burst() {
        // THE clamp pin. A 10-interval stall from the deadline at `I` lands at
        // `11I + 4ms`; the next wake must be `12I` — one grid point, on the
        // original phase, strictly in the future.
        let now = 11 * I + 4_000_000;
        let next = next_tick_deadline_ns(I, now, I);
        assert_eq!(next, 12 * I, "the stall must resume on the ORIGINAL grid");
        assert!(next > now, "a deadline in the past is a zero-length sleep");

        // The burst an UNCLAMPED `deadline += interval` produces is a RUN of
        // deadlines still behind `now`, each yielding a zero-length wait. Drive
        // it from the SAME pre-stall deadline with `now` HELD at the stalled
        // value — which is what the loop sees while it is catching up — and
        // require every step to stay strictly ahead of `now`.
        //
        // `now` held is the whole point: a version that
        // advances `now` alongside the deadline puts every call on the
        // ON-TIME arm where the clamped and un-clamped functions AGREE, so it
        // re-asserts what the line above already proved. Held, the
        // un-clamped variant walks 2I, 3I, 4I … — every one behind `now`.
        //
        // Scope: this is the PROPERTY form of the assertion above, not a
        // second independent kill. Under the un-clamped variant it fails on its
        // first step, at the same place.
        let mut d = I;
        for expected in 12..=16u64 {
            d = next_tick_deadline_ns(d, now, I);
            assert!(
                d > now,
                "step {expected} landed at {d} — behind `now` ({now}), i.e. a \
                 zero-length wait: the catch-up is bursting"
            );
            assert_eq!(
                d,
                expected * I,
                "with `now` held at the stall, the grid advances one step per call"
            );
        }
    }

    #[test]
    fn a_deadline_exactly_reached_still_moves_strictly_forward() {
        // The boundary: `now` lands EXACTLY on `deadline + interval`. A `>=`
        // slip here would return a deadline equal to `now` → a zero-length sleep
        // every iteration, i.e. a spin at the wake rate.
        let now = 2 * I;
        let next = next_tick_deadline_ns(I, now, I);
        assert_eq!(next, 3 * I);
        assert!(next > now);
    }

    #[test]
    fn a_clock_regression_never_moves_the_deadline_backwards() {
        // `Instant` is monotonic, so this is a caller-contract guard rather than
        // an observed condition: a `now` BEFORE the served deadline must take the
        // on-time arm (one step), never underflow and never rewind.
        assert_eq!(next_tick_deadline_ns(5 * I, 3 * I, I), 6 * I);
        assert_eq!(next_tick_deadline_ns(5 * I, 0, I), 6 * I);
    }

    #[test]
    fn a_zero_interval_collapses_the_grid_onto_now_instead_of_dividing_by_zero() {
        assert_eq!(next_tick_deadline_ns(I, 7 * I, 0), 7 * I);
        assert_eq!(next_tick_deadline_ns(0, 0, 0), 0);
        // …and the wait that deadline produces is the FLOOR, not zero: the
        // returned deadline IS `now`, so the loop takes the already-late arm on
        // every subsequent pass. That is the difference between a bounded
        // ~200 µs cadence and an unbounded hot spin, and it is why the doc says
        // "collapses the grid" rather than "disables pacing".
        assert_eq!(plan_tick_sleep(7 * I, 7 * I), LATE_PASS_YIELD);
    }

    #[test]
    fn a_saturating_now_never_wraps_the_grid() {
        // Defensive: `u64::MAX` ns is ~584 years of uptime, so this is
        // unreachable in practice. What saturation buys is precisely "no wrap,
        // no wrong-grid deadline" — NOT "a deadline it can sleep to": the
        // returned value EQUALS `now`, so the wait comes from the already-late
        // arm. Before the floor that arm waited nothing at all
        // and the loop span here; the assertion below is what closes it.
        let next = next_tick_deadline_ns(I, u64::MAX, I);
        assert_eq!(next, u64::MAX, "saturate, never wrap");
        assert_eq!(plan_tick_sleep(next, u64::MAX), LATE_PASS_YIELD);
    }
}

/// Oracle vectors for the duty-cycle floor.
///
/// The floor's whole job is that the returned wait is NEVER zero, so every arm
/// below asserts a literal duration rather than a comparison against the
/// function's own output.
#[cfg(test)]
mod tick_sleep_tests {
    use super::{plan_tick_sleep, LATE_PASS_YIELD};
    use std::time::Duration;

    const I: u64 = 16_000_000;

    #[test]
    fn an_on_time_pass_waits_exactly_the_remaining_time() {
        // The deadline fix itself: the wait shrinks by however long the pass
        // took, so the PERIOD is the interval rather than `interval + work`.
        assert_eq!(plan_tick_sleep(2 * I, I), Duration::from_nanos(I));
        assert_eq!(
            plan_tick_sleep(2 * I, 2 * I - 1_000_000),
            Duration::from_millis(1)
        );
        assert_eq!(plan_tick_sleep(2 * I, 2 * I - 1), Duration::from_nanos(1));
    }

    #[test]
    fn an_already_late_pass_still_yields_the_floor() {
        // THE headline pin. Deadline exactly reached, missed by a hair, and missed by
        // ten whole intervals all take the floor — never `Duration::ZERO`, which
        // std's unix `sleep` does not even turn into a syscall (~1 ns, no yield).
        assert_eq!(plan_tick_sleep(2 * I, 2 * I), LATE_PASS_YIELD);
        assert_eq!(plan_tick_sleep(2 * I, 2 * I + 1), LATE_PASS_YIELD);
        assert_eq!(plan_tick_sleep(2 * I, 12 * I), LATE_PASS_YIELD);
        assert_eq!(plan_tick_sleep(0, u64::MAX), LATE_PASS_YIELD);
    }

    #[test]
    fn the_planned_wait_is_never_zero() {
        // The property the `poll_sleepless_passes` observable counts, asserted
        // directly over the boundary and over a spread of shapes: whatever the
        // loop is handed, it waits for SOMETHING.
        for deadline in [0, 1, I, 2 * I, u64::MAX - 1, u64::MAX] {
            for now in [0, 1, I - 1, I, I + 1, 2 * I, u64::MAX] {
                assert!(
                    !plan_tick_sleep(deadline, now).is_zero(),
                    "deadline={deadline} now={now} planned a zero wait — the loop \
                     would run back-to-back with no yield"
                );
            }
        }
    }

    #[test]
    fn the_late_arm_and_the_late_predicate_agree_everywhere() {
        // The `poll_late_passes` observable is the STIMULUS oracle for the e2e
        // overrun arm, and the loop computes it from `is_late_pass` while the
        // WAIT comes from `plan_tick_sleep`. If those two ever disagreed, the
        // test could report a regime the loop never entered — or miss one it
        // did. They share the predicate; this pins that they still do.
        for deadline in [0, 1, I, 2 * I, u64::MAX - 1, u64::MAX] {
            for now in [0, 1, I - 1, I, I + 1, 2 * I, u64::MAX] {
                let late = super::is_late_pass(deadline, now);
                let floored = plan_tick_sleep(deadline, now) == LATE_PASS_YIELD;
                assert_eq!(
                    late,
                    floored,
                    "deadline={deadline} now={now}: is_late_pass said {late} while \
                     plan_tick_sleep {} the floor",
                    if floored { "took" } else { "did not take" }
                );
            }
        }
    }
}

/// The accept loop: nonblocking `accept` polled against the shutdown flag, one
/// controller thread per connection. Finished connection handles are pruned each
/// iteration so the tracking Vec stays bounded on a long-lived daemon.
///
/// Both transient-error paths — a failed `accept` (e.g. a persistent `EMFILE`
/// fd-exhaustion, ~40 warns/s at [`ACCEPT_POLL`]) and a failed thread `spawn` —
/// are FLOOD-LATCHED (the stderr-storm
/// class): loud first, `debug!`-suppressed repeats with a running count, one
/// recovery `info!` when it heals. `WouldBlock` is the normal nonblocking idle,
/// not an error, so it never trips the latch.
fn accept_loop(
    listener: UnixListener,
    ctx: Ctx,
    shutdown: Arc<AtomicBool>,
    conns: Arc<Mutex<Vec<JoinHandle<()>>>>,
    write_fail_latch: Arc<Mutex<FieldsWarnLatch>>,
) {
    let mut accept_err_latch = FieldsWarnLatch::new();
    let mut spawn_err_latch = FieldsWarnLatch::new();
    while !shutdown.load(Ordering::Acquire) {
        match listener.accept() {
            Ok((stream, _addr)) => {
                // A successful accept HEALS an accept-error regime.
                if let Some(suppressed) = on_transient_recovered(&mut accept_err_latch) {
                    tracing::info!(suppressed, "vizd: control-socket accept recovered");
                }
                let ctx = ctx.clone();
                let sd = Arc::clone(&shutdown);
                let wfl = Arc::clone(&write_fail_latch);
                let handle = std::thread::Builder::new()
                    .name("vizd-conn".to_string())
                    .spawn(move || handle_connection(stream, ctx, sd, wfl));
                match handle {
                    Ok(h) => {
                        if let Some(suppressed) = on_transient_recovered(&mut spawn_err_latch) {
                            tracing::info!(suppressed, "vizd: controller-thread spawn recovered");
                        }
                        let mut guard = conns.lock().unwrap();
                        guard.retain(|h| !h.is_finished()); // reap finished handlers
                        guard.push(h);
                    }
                    Err(e) => match on_transient_failure(&mut spawn_err_latch) {
                        FieldsLogAction::WarnFirst => {
                            tracing::warn!(error = %e, "vizd: failed to spawn a controller thread")
                        }
                        FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                            error = %e,
                            suppressed,
                            "vizd: still failing to spawn a controller thread (warn suppressed)"
                        ),
                    },
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                std::thread::sleep(ACCEPT_POLL);
            }
            Err(e) => {
                match on_transient_failure(&mut accept_err_latch) {
                    FieldsLogAction::WarnFirst => {
                        tracing::warn!(error = %e, "vizd: control-socket accept error")
                    }
                    FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                        error = %e,
                        suppressed,
                        "vizd: control-socket accept still failing (warn suppressed)"
                    ),
                }
                std::thread::sleep(ACCEPT_POLL);
            }
        }
    }
}

/// Route a transient accept-loop failure through its flood latch: loud on the
/// first of a regime, `debug!`-suppressed (running count) on repeats. Pure — the
/// caller maps the returned action to a log level. Load-bearing (`dead_code =
/// deny`): the accept loop's single call site is the only user, so reverting it
/// to an un-latched `warn!` would leave this dead.
fn on_transient_failure(latch: &mut FieldsWarnLatch) -> FieldsLogAction {
    latch.on_inferred()
}

/// Route a transient accept-loop RECOVERY (a success after a failing regime)
/// through the same latch: `Some(suppressed)` → log one recovery `info!` + re-arm;
/// `None` → nothing to say. Pure — see [`on_transient_failure`].
fn on_transient_recovered(latch: &mut FieldsWarnLatch) -> Option<u64> {
    latch.on_decoded()
}

/// One controller connection: send the [`Hello`] banner, then loop reading NDJSON
/// request lines and writing NDJSON responses. A read timeout re-checks the
/// shutdown flag so an idle-but-connected controller never wedges teardown, and the
/// WRITE no-progress budget ([`CONN_WRITE_TIMEOUT`], via [`write_line`]'s shared
/// bounded resume loop) stops a controller that has stopped reading from wedging
/// teardown (a full send buffer does not truncate a big reply — the write RESUMES,
/// and is dropped only after the budget of no progress). A malformed line
/// yields a structured error (never a disconnect); an UNTERMINATED line exceeding
/// [`MAX_REQUEST_LINE_BYTES`] yields ONE structured error then a disconnect (the
/// `pending` buffer is bounded, never an OOM); EOF or a write failure (controller
/// gone / wedged) ends the connection.
fn handle_connection(
    mut stream: UnixStream,
    ctx: Ctx,
    shutdown: Arc<AtomicBool>,
    write_fail_latch: Arc<Mutex<FieldsWarnLatch>>,
) {
    // Bound BOTH directions BEFORE the first write: the banner itself must not be
    // able to block a never-reading controller (a full send buffer).
    //
    // The same call also clears the accept-inherited O_NONBLOCK. The read
    // loop below is PACED by `read` blocking for `CONN_READ_TIMEOUT`; on a
    // nonblocking socket that read returns EAGAIN in microseconds and the
    // `WouldBlock => continue` arm becomes an unbounded tight loop (MEASURED at
    // 99.4 % CPU on a vizd rendering nothing). One shared seam with netd — see
    // `cerulion_core::prepare_accepted_stream`. If any step fails the socket is
    // unusable → bail.
    if cerulion_core::prepare_accepted_stream(&stream, CONN_READ_TIMEOUT, CONN_WRITE_TIMEOUT)
        .is_err()
    {
        return;
    }

    // Banner FIRST (version handshake + the advertised `rerun_url` every viewer
    // connects to). A write failure = the controller already left (or wedged) →
    // nothing to serve.
    let banner = Hello::new(ctx.rerun_url.clone()).to_json_line();
    if !write_line(&mut stream, &banner, &write_fail_latch) {
        return;
    }
    // A successful banner write HEALS a wedged-controller regime + re-arms the
    // shared latch (recovery `info!` once if a fleet of wedges just cleared).
    if let Some(suppressed) = write_fail_latch.lock().unwrap().on_decoded() {
        tracing::info!(suppressed, "vizd: controller writes recovered");
    }

    // This connection's id — the key for its catalog-change push slot.
    let conn = ctx.events.mint_conn();
    let _unsubscribe = UnsubscribeOnDrop {
        events: Arc::clone(&ctx.events),
        conn,
    };

    let mut pending: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 4096];
    loop {
        // Principle #3: one bump per trip round the loop. Cheap (a relaxed
        // fetch_add against a loop whose other arm is a syscall) and it is the ONLY
        // way to tell "paced by the read timeout" from "spinning" without a profiler.
        ctx.conn_loop_iterations.fetch_add(1, Ordering::Relaxed);
        if shutdown.load(Ordering::Acquire) {
            return;
        }
        // Drain this connection's catalog-change notification, if it has one.
        //
        // Written HERE — by the connection's own handler thread, on the read loop it
        // already runs — and never by the forwarder. That is the no-wedge story:
        // exactly one thread ever writes a given stream (so no interleaved half-lines),
        // the forwarder does a mutex store and no I/O (so a controller that stopped
        // reading cannot stall it or a healthy sibling), and a stalled controller's own
        // `write_line` times out and drops its OWN connection exactly as for any
        // response. The loop reaches here at least every `CONN_READ_TIMEOUT`.
        // That is the REAL cadence on an idle controller, not just a
        // ceiling. If the accepted socket kept its inherited O_NONBLOCK, the
        // read would return EAGAIN instantly and this drain would run in a microsecond-scale
        // spin — a push latency nobody asked for, bought with a whole CPU core.
        if let Some(event) = ctx.events.take_pending(conn) {
            if !write_line(&mut stream, &event.to_json_line(), &write_fail_latch) {
                return; // controller gone / wedged — only this connection dies.
            }
        }
        match stream.read(&mut tmp) {
            Ok(0) => return, // EOF: controller closed the connection.
            Ok(n) => {
                pending.extend_from_slice(&tmp[..n]);
                // Extract + serve every COMPLETE line in the buffer.
                while let Some(pos) = pending.iter().position(|&b| b == b'\n') {
                    let line: Vec<u8> = pending.drain(..=pos).collect();
                    let text = String::from_utf8_lossy(&line[..line.len() - 1]);
                    let text = text.trim_end_matches('\r'); // tolerate CRLF
                    if text.trim().is_empty() {
                        continue;
                    }
                    let response = dispatch_line(text, &ctx, conn);
                    if !write_line(&mut stream, &response.to_json_line(), &write_fail_latch) {
                        return; // controller gone / wedged mid-exchange.
                    }
                }
                // Every COMPLETE line has been drained above; whatever remains in
                // `pending` is an IN-PROGRESS line with no newline yet. If it has
                // grown past the cap, the controller is building an unbounded line
                // (buggy serializer / partial flush / malicious client) — refuse
                // it (best-effort structured error, id None) and drop THIS
                // connection so the buffer can never OOM the daemon. A
                // one-shot warn per offending connection (immediately before the
                // disconnect) is fine — this is NOT the sustained-write flood the
                // shared `write_fail_latch` guards.
                if pending.len() > MAX_REQUEST_LINE_BYTES {
                    tracing::warn!(
                        pending_bytes = pending.len(),
                        cap = MAX_REQUEST_LINE_BYTES,
                        "vizd: controller request line exceeded the cap without a newline — dropping the connection"
                    );
                    let response = Response::error(
                        None,
                        format!(
                            "request line exceeded {MAX_REQUEST_LINE_BYTES} bytes without a \
                             newline — dropping the connection"
                        ),
                        None,
                    );
                    let _ = write_line(&mut stream, &response.to_json_line(), &write_fail_latch);
                    return;
                }
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                continue; // idle: re-check the shutdown flag.
            }
            Err(_) => return, // a real read error ends the connection.
        }
    }
}

/// Write one NDJSON line + newline to `stream`, returning whether the write
/// SUCCEEDED (so the caller can end the connection on failure). A write TIMEOUT
/// ([`CONN_WRITE_TIMEOUT`]) means the controller stopped reading and its send
/// buffer filled — a dead/wedged controller — so it is FLOOD-LATCHED (loud first,
/// `debug!`-suppressed repeats across the whole controller fleet) then the
/// connection is dropped. Any other write error is a normal disconnect (the
/// controller left) — silent.
fn write_line(
    stream: &mut UnixStream,
    line: &str,
    write_fail_latch: &Mutex<FieldsWarnLatch>,
) -> bool {
    // A big control response (the 75-topic discover/list reply) exceeds the
    // ~8 KiB UDS send buffer; on macOS the accepted socket is nonblocking, so the old
    // `writeln!` (== `write_all`) aborted on the FIRST `WouldBlock` and dropped the
    // connection after ~one send buffer — the viewer read a truncated line + EOF
    // ("Can't reach vizd"). The shared bounded resume loop RESUMES the partial write
    // and only gives up when the controller makes NO forward progress within
    // `CONN_WRITE_TIMEOUT` (the genuine "stopped reading" signal). That no-progress
    // timeout still rides the once-per-regime flood latch (loud first, `debug!`-
    // suppressed repeats across the controller fleet); a plain disconnect stays
    // silent, exactly as before.
    match write_line_bounded(stream, line, CONN_WRITE_TIMEOUT) {
        UdsWriteOutcome::Complete => true,
        UdsWriteOutcome::NoProgressTimeout { .. } => {
            match write_fail_latch.lock().unwrap().on_inferred() {
                FieldsLogAction::WarnFirst => tracing::warn!(
                    no_progress_ms = CONN_WRITE_TIMEOUT.as_millis() as u64,
                    "vizd: controller write timed out (not reading) — dropping the connection"
                ),
                FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                    suppressed,
                    "vizd: controller write still timing out (warn suppressed)"
                ),
            }
            false
        }
        // Peer closed mid-write / a normal disconnect (BrokenPipe/etc.) — silent, as
        // before (only the sustained "not reading" wedge rides the flood latch).
        UdsWriteOutcome::PeerClosed { .. } | UdsWriteOutcome::IoError => false,
    }
}

/// Parse + dispatch one request line into its response — malformed lines become
/// a structured error carrying the best-effort correlation id (never a panic).
/// RAII — the connection close IS the unsubscribe (the crash-safe rule the
/// netd demand plane uses). A guard rather than a call at the end of
/// `handle_connection` because that function returns from a dozen places (EOF, every
/// write failure, the over-cap refusal, shutdown); a missed one would leak a push slot
/// nobody drains for the daemon's lifetime.
struct UnsubscribeOnDrop {
    events: Arc<crate::events::EventHub>,
    conn: u64,
}

impl Drop for UnsubscribeOnDrop {
    fn drop(&mut self) {
        self.events.unsubscribe(self.conn);
    }
}

fn dispatch_line(line: &str, ctx: &Ctx, conn: u64) -> Response {
    match parse_request(line) {
        Ok(req) => ctx.dispatch_for_conn(req, conn),
        Err(e) => Response::error(e.id, format!("malformed request: {}", e.message), None),
    }
}

#[cfg(test)]
mod absorb_tests {
    use super::{Absorbed, SeedState};
    use cerulion_core::{SchemaDoc, SchemaEncoding};
    use std::collections::BTreeSet;

    /// `rollback` takes a SET (see its docs — a repeated name inverts the refusal
    /// memory), so tests build one rather than a list.
    fn names(qs: &[&str]) -> BTreeSet<String> {
        qs.iter().map(|q| (*q).to_string()).collect()
    }

    fn doc(q: &str, text: &str) -> SchemaDoc {
        SchemaDoc {
            qualified: q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: text.to_string(),
            deps: vec![],
        }
    }

    /// The seed's idempotence policy, against a hand oracle.
    ///
    /// It is load-bearing rather than an optimisation: every accepted doc costs a
    /// FULL walker rebuild (a re-parse of all ~254 built-ins), and the bag
    /// player re-offers its definitions on a timer for the whole playback. Without
    /// this, a 10-minute playback would pay ~300 rebuilds and grow the doc list
    /// unboundedly with duplicates.
    #[test]
    fn an_identical_re_offer_changes_nothing_while_a_changed_one_updates() {
        let mut seed = SeedState::default();

        // First sight of two types: both new, nothing replaced.
        let a = seed.absorb(vec![doc("a/A", "int32 x\n"), doc("a/B", "int32 y\n")]);
        assert_eq!(a.added, 2);
        assert!(a.replaced.is_empty());
        assert!(a.changed());
        assert_eq!(seed.docs.len(), 2);

        // The SAME two, byte-identical: nothing changed, nothing to rebuild.
        let a = seed.absorb(vec![doc("a/A", "int32 x\n"), doc("a/B", "int32 y\n")]);
        assert_eq!(a, Absorbed::default());
        assert!(!a.changed(), "an identical re-offer must skip the rebuild");
        assert_eq!(
            seed.docs.len(),
            2,
            "an identical re-offer never grows the set"
        );

        // A CHANGED definition for a held name is a REPLACEMENT, reported apart
        // from an addition: the walker keys layouts by qualified name, so the
        // previous definition stops resolving and an operator is owed that fact.
        let a = seed.absorb(vec![doc("a/A", "int32 x\nint32 z\n")]);
        assert_eq!(a.added, 0);
        assert_eq!(a.replaced, vec!["a/A".to_string()]);
        assert!(a.changed());
        assert_eq!(seed.docs.len(), 2, "a replacement does not grow the set");
        let held = seed.docs.iter().find(|d| d.qualified == "a/A").unwrap();
        assert_eq!(held.text, "int32 x\nint32 z\n");

        // A mixed batch separates the two outcomes.
        let a = seed.absorb(vec![
            doc("a/A", "int32 x\nint32 z\n"), // unchanged
            doc("a/B", "int32 y\nint32 w\n"), // replaced
            doc("a/C", "int32 c\n"),          // added
        ]);
        assert_eq!(a.added, 1);
        assert_eq!(a.replaced, vec!["a/B".to_string()]);
        assert_eq!(seed.docs.len(), 3);

        // Two docs for the SAME name in ONE batch: the second replaces the first,
        // so the set grows by one and the replacement is reported — `added` alone
        // could otherwise exceed the set size.
        let mut fresh_seed = SeedState::default();
        let a = fresh_seed.absorb(vec![doc("z/Z", "int32 a\n"), doc("z/Z", "int32 b\n")]);
        assert_eq!(a.added, 1);
        assert_eq!(a.replaced, vec!["z/Z".to_string()]);
        assert_eq!(fresh_seed.docs.len(), 1);
        assert_eq!(fresh_seed.docs[0].text, "int32 b\n");

        // An empty offer is a legal no-op, not an error.
        assert_eq!(seed.absorb(vec![]), Absorbed::default());
        assert_eq!(seed.docs.len(), 3);
    }

    /// A definition the WALKER refused must not stay in the accumulated set: the
    /// idempotence short-circuit reads that set, so retaining it would swallow a
    /// later corrected offer of the same type forever.
    ///
    /// This is `rollback`'s NO-INCUMBENT case (there is no separate `forget`:
    /// a refused doc the daemon never held before simply leaves).
    #[test]
    fn a_rolled_back_doc_can_be_offered_again() {
        let mut seed = SeedState::default();
        let a = seed.absorb(vec![doc("a/A", "bad\n"), doc("a/B", "int32 y\n")]);
        assert!(a.changed());
        assert!(
            a.previous.is_empty(),
            "neither name was held, so there is no incumbent to restore"
        );
        seed.rollback(&names(&["a/A"]), &a.previous);
        assert_eq!(seed.docs.len(), 1, "only the refused doc is dropped");
        assert_eq!(seed.docs[0].qualified, "a/B");

        // The SAME name offered again is genuinely new once more — which is the
        // whole point: one bad definition must not be terminal.
        let a = seed.absorb(vec![doc("a/A", "int32 x\n")]);
        assert_eq!(a.added, 1);
        assert!(a.replaced.is_empty());

        // Rolling back a name never held is a no-op.
        seed.rollback(&names(&["nope/Nope"]), &[]);
        assert_eq!(seed.docs.len(), 2);
    }

    /// Forgetting a refused doc must not re-open the rebuild cost the
    /// idempotence short-circuit exists to prevent.
    ///
    /// `bag play` re-offers the SAME set every 2 s. With `forget` dropping the doc
    /// outright, a definition the walker cannot use looped absorb(added=1) → full
    /// rebuild (a re-parse of the whole built-in corpus) + a render-worker walker
    /// swap → refuse → forget, for the whole playback — exactly the cost
    /// `an_identical_re_offer_changes_nothing_...` calls "load-bearing rather than
    /// an optimisation", paid on every cycle for the one doc that can never help.
    ///
    /// Hand oracle: the FIRST refusal is a real change (the daemon had to try it
    /// to learn it was unusable), every byte-identical re-offer after it is not,
    /// and a CORRECTED offer of the same name still lands.
    #[test]
    fn an_identical_re_offer_of_a_refused_doc_costs_no_rebuild() {
        let mut seed = SeedState::default();

        // The first sight of a bad doc IS a change — it has to be tried.
        assert!(seed.absorb(vec![doc("a/Bad", "@@@\n")]).changed());
        seed.rollback(&names(&["a/Bad"]), &[]);
        assert!(
            seed.docs.is_empty(),
            "a refused doc leaves the walker's set"
        );

        // THE PIN: re-offering it byte-identically, as the 2 s player does,
        // INSTALLS nothing — no rebuild, no worker swap, and it does not sneak
        // back into the set the walker is built from — while STILL being reported
        // as refused. Suppressing the rebuild must not also suppress the truth: a
        // caller told "nothing changed" about a doc that is still unusable would
        // read it as "you already had this".
        for cycle in 0..5 {
            let a = seed.absorb(vec![doc("a/Bad", "@@@\n")]);
            assert_eq!(
                a,
                Absorbed {
                    refused: vec!["a/Bad".to_string()],
                    ..Absorbed::default()
                },
                "cycle {cycle}: an identical re-offer of a REFUSED doc must not \
                 trigger a rebuild, and must still be reported"
            );
            assert!(!a.changed(), "cycle {cycle}");
            assert!(seed.docs.is_empty(), "cycle {cycle}");
        }

        // ANTI-TAUTOLOGY / RECOVERY: a CORRECTED definition for the same name is
        // still absorbed, so the memory suppresses repeats and nothing else.
        let a = seed.absorb(vec![doc("a/Bad", "int32 x\n")]);
        assert_eq!(a.added, 1, "a corrected definition must still land");
        assert!(a.replaced.is_empty());
        assert_eq!(seed.docs.len(), 1);
        assert_eq!(seed.docs[0].text, "int32 x\n");

        // …and once corrected, the ORIGINAL bad text is no longer remembered, so
        // re-offering it is a genuine REPLACEMENT the operator is told about
        // (it destroys what the good one was decoding).
        let a = seed.absorb(vec![doc("a/Bad", "@@@\n")]);
        assert_eq!(a.replaced, vec!["a/Bad".to_string()]);

        // A doc refused in a batch does not suppress its HEALTHY siblings.
        let mut seed = SeedState::default();
        assert_eq!(
            seed.absorb(vec![doc("b/Bad", "@@@\n"), doc("b/Good", "int32 y\n")])
                .added,
            2
        );
        seed.rollback(&names(&["b/Bad"]), &[]);
        let a = seed.absorb(vec![doc("b/Bad", "@@@\n"), doc("b/Good", "int32 y\n")]);
        assert_eq!(
            a,
            Absorbed {
                refused: vec!["b/Bad".to_string()],
                ..Absorbed::default()
            },
            "the good sibling is idempotent by the ordinary rule, the bad one by \
             the refusal memory — together nothing is INSTALLED, and only the bad \
             one is reported"
        );
        assert!(!a.changed(), "so the batch buys no rebuild");
        assert_eq!(seed.docs.len(), 1);
        assert_eq!(seed.docs[0].qualified, "b/Good");
    }

    /// The PURE half: a REFUSED replacement restores the
    /// incumbent, in its own slot, and remembers the refused text.
    ///
    /// `absorb` has to commit into `docs` before the walker can parse anything (the
    /// rebuild reads that set), so a merge must be UNDOABLE — otherwise one
    /// malformed re-offer of a held name overwrites a working definition, the
    /// rebuilt walker warn-SKIPS it, and that walker is installed daemon-wide.
    ///
    /// Hand oracle throughout; the ORDER assertion is deliberate — the restore
    /// puts the incumbent back where it was, so a rollback leaves the doc set
    /// byte-identical to what it was before the offer.
    #[test]
    fn a_rolled_back_replacement_restores_the_incumbent_in_its_own_slot() {
        let mut seed = SeedState::default();
        let good = doc("a/A", "int32 x\n");
        assert_eq!(
            seed.absorb(vec![
                doc("a/First", "int32 f\n"),
                good.clone(),
                doc("a/Last", "int32 l\n"),
            ])
            .added,
            3
        );
        let before: Vec<SchemaDoc> = seed.docs.clone();

        // A malformed replacement lands FIRST (that is what makes it dangerous)…
        let a = seed.absorb(vec![doc("a/A", "@@@\n")]);
        assert_eq!(a.replaced, vec!["a/A".to_string()]);
        assert_eq!(
            a.previous,
            vec![good.clone()],
            "the incumbent must be captured, or the merge is irreversible"
        );
        assert_eq!(
            seed.docs[1].text, "@@@\n",
            "PRECONDITION: absorb really did overwrite it — the rollback below is \
             undoing something"
        );

        // …and the walker refuses it, so it is rolled back.
        seed.rollback(&names(&["a/A"]), &a.previous);
        assert_eq!(
            seed.docs, before,
            "THE PIN: a refused replacement leaves the doc set exactly as it was — \
             same definitions, same order"
        );

        // The refused TEXT is remembered, so an identical re-offer is free…
        let a = seed.absorb(vec![doc("a/A", "@@@\n")]);
        assert_eq!(
            a,
            Absorbed {
                refused: vec!["a/A".to_string()],
                ..Absorbed::default()
            },
            "an identical re-offer of a refused doc is REPORTED but rebuilds nothing"
        );
        assert!(!a.changed(), "…so it must not buy a rebuild");
        assert_eq!(seed.docs, before, "and it does not touch the held set");

        // …while a CORRECTED definition still replaces the incumbent normally.
        let fixed = doc("a/A", "int32 x\nint32 y\n");
        let a = seed.absorb(vec![fixed.clone()]);
        assert_eq!(a.replaced, vec!["a/A".to_string()]);
        assert_eq!(
            a.previous,
            vec![good],
            "it displaced the incumbent, not the bad text"
        );
        assert!(a.refused.is_empty());
        assert_eq!(
            seed.docs[1], fixed,
            "the correction landed, in the same slot"
        );
    }

    /// ONE offer carrying the same qualified name TWICE must
    /// not put the surviving INCUMBENT into the refusal memory.
    ///
    /// `rollback` is not idempotent per name — a repeat visit finds the RESTORED
    /// incumbent where the bad doc was, evicts the real refusal from the memory
    /// and records the good doc as the refused one. The data plane survives (the
    /// incumbent is re-inserted either way), so this is invisible to a docs-only
    /// assertion; what breaks is the REPORT, permanently: every later
    /// byte-identical offer of that good definition comes back `rejected` with a
    /// "could not use" warning, about a type the walker resolves perfectly. That
    /// is a success-shaped lie pointed the other way.
    ///
    /// Reachable from both production seams — the `schemas` verb takes docs
    /// straight off the control seam, and `classify_schema_replies` does not dedup
    /// a robot's served closure. (`bag play` is safe: the catalog normalises.)
    ///
    /// `rollback` takes a SET, so this is unrepresentable; the
    /// test drives the composed absorb→rollback sequence the seam performs.
    #[test]
    fn a_duplicated_name_in_one_offer_never_puts_the_incumbent_in_the_refusal_memory() {
        let mut seed = SeedState::default();
        let good = doc("a/A", "int32 x\n");
        assert_eq!(seed.absorb(vec![good.clone()]).added, 1);

        // ONE offer, the SAME name twice, the LAST one unusable — which is what
        // `absorb` leaves installed, so the walker refuses `a/A`.
        let a = seed.absorb(vec![doc("a/A", "int32 y\n"), doc("a/A", "@@@\n")]);
        // ONE replacement is reported, even though BOTH docs replaced — the offer
        // changed one type. (Listing the name twice would contradict the
        // field's own doc, which calls a replacement a distinct outcome from a gain.)
        assert_eq!(
            a.replaced,
            vec!["a/A".to_string()],
            "one type replaced, one report line"
        );
        assert_eq!(
            a.previous,
            vec![good.clone()],
            "and the retained incumbent is the one the OFFER found — not this \
             batch's own earlier doc, which the daemon never served"
        );
        assert_eq!(
            seed.docs[0].text, "@@@\n",
            "PRECONDITION: the bad one is what ended up installed"
        );

        // The seam derives the refusal set from the offered names; a SET collapses
        // the duplicate, which is exactly what stops the destructive repeat visit.
        seed.rollback(&names(&["a/A"]), &a.previous);

        // Data plane: the incumbent is back. (True under the defect too — this is
        // the assertion that CANNOT see it, kept so the next reader knows why the
        // one below is the real pin.)
        assert_eq!(seed.docs, vec![good.clone()], "the incumbent survives");

        // THE PIN: the refusal memory holds the doc that was REFUSED, never the
        // incumbent that survived it.
        assert_eq!(
            seed.refused,
            vec![doc("a/A", "@@@\n")],
            "the refusal memory must hold the REFUSED doc — holding the incumbent \
             makes the daemon permanently disown a definition it resolves"
        );

        // …and the consequence that makes it matter: re-offering the GOOD
        // definition is a clean no-op, NOT a reported refusal.
        let a = seed.absorb(vec![good.clone()]);
        assert_eq!(
            a,
            Absorbed::default(),
            "a byte-identical re-offer of the INCUMBENT is idempotent — under the \
             defect it comes back refused, forever"
        );
        assert_eq!(seed.docs, vec![good], "and it stays installed");
        // That re-offer also CLEARS the stale memory, by design: a text differing
        // from the remembered refusal drops it (`absorb`'s corrected-definition
        // arm), and the incumbent's own text differs from the doc that was
        // refused. So the next offer of the bad text is a fresh replacement the
        // seam refuses again — the memory is a repeat-suppressor, not a verdict.
        assert!(
            seed.refused.is_empty(),
            "the memory is dropped once a different \
             definition for that name is absorbed"
        );
    }

    /// A name repeated in ONE offer is reported ONCE. `refused` is a report, and a
    /// duplicated entry reads as two separate bad definitions.
    #[test]
    fn a_name_repeated_in_one_offer_is_reported_once() {
        let mut seed = SeedState::default();
        assert!(seed.absorb(vec![doc("a/Bad", "@@@\n")]).changed());
        seed.rollback(&names(&["a/Bad"]), &[]);

        let a = seed.absorb(vec![doc("a/Bad", "@@@\n"), doc("a/Bad", "@@@\n")]);
        assert_eq!(
            a.refused,
            vec!["a/Bad".to_string()],
            "one bad definition, one report line"
        );
        assert!(!a.changed());
    }
}

#[cfg(test)]
mod tests {
    // Test-only: the RAW derivation, kept out of production scope on purpose —
    // `discover_reports_the_same_entity_as_attach_for_a_tf_named_non_tf_topic`
    // asserts no production site calls it.
    // The classifier the tap-liveness oracles assert against. Imported
    // HERE (not at module scope) because production classifies through
    // `TopicLiveness::wire_state`, which names no variant.
    use cerulion_core::LivenessState;
    use cerulion_viz::sink::route_for_input;

    /// What the daemon REPORTS for a frame must equal what the sink
    /// RENDERS for it.
    ///
    /// A `resolve_from_frame` that ran `classify_schema(...).unwrap_or_else(shape
    /// inference)` — the expression `classify_frame` stands in for in the two sink
    /// paths — would disagree with the sink's content-gated classification.
    /// Everything downstream reads this value: `discover` / `list` /
    /// `status`, the views `maybe_apply_default_layout` composes, and guardrail 2's
    /// accept/refuse. A disagreement therefore builds a layout for a topic that
    /// renders something else, and a hand-written correct view gets refused as
    /// "renders nothing".
    ///
    /// The discriminating input is a `sensor_msgs/CompressedImage` whose `data` is
    /// an H.264 Annex-B access unit: the NAME table answers `Image` with full
    /// confidence, and only the CONTENT gate reaches `VideoStream`. So this fails
    /// if `resolve_from_frame` consults the name table first — which is the
    /// regression — while a JPEG-carrying twin pins that the ordinary image path is
    /// untouched.
    #[test]
    fn resolve_from_frame_reports_what_the_sink_renders_not_the_name_table() {
        use cerulion_core::codegen::layout::LayoutResolver;
        use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
        use cerulion_core::message::ShmMessage;
        use cerulion_core::shm_runtime::write_offset_entry;
        use cerulion_core::wire::WireHeader;
        use cerulion_viz::schema_registry::builtin_walker;

        /// SPS + PPS + IDR, Annex-B framed — real bytes from the committed Go2
        /// capture (see `cerulion_viz/lib/cerulion_viz/tests/video_h264_test.rs`
        /// for the full provenance note).
        fn go2_access_unit() -> Vec<u8> {
            const SPS: &[u8] = &[
                0x67, 0x64, 0x10, 0x28, 0xac, 0x1b, 0x1a, 0xa0, 0xa0, 0x2f, 0xf9, 0x61, 0x00, 0x00,
                0x03, 0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x3c, 0x8f, 0x08, 0x84, 0x6a,
            ];
            const PPS: &[u8] = &[0x68, 0xee, 0x31, 0xb2, 0x1b];
            const IDR: &[u8] = &[0x65, 0xb8, 0x00, 0x01, 0x40, 0x00, 0x01, 0x3f];
            let mut out = Vec::new();
            for nal in [SPS, PPS, IDR] {
                out.extend_from_slice(&[0, 0, 0, 1]);
                out.extend_from_slice(nal);
            }
            out
        }

        /// A real `sensor_msgs/CompressedImage` wire frame carrying `blob`.
        fn compressed_image_frame(format: &[u8], blob: &[u8]) -> Vec<u8> {
            let mut schemas: Vec<MessageSchema> = Vec::new();
            for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
                if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                    schemas.push(s);
                }
            }
            let (mut resolver, _) = LayoutResolver::new(schemas);
            let layout = resolver
                .layout_of("sensor_msgs/CompressedImage")
                .expect("built-in schema");
            let fixed = layout.fixed_size;
            let table = layout.offset_table_bytes();
            let mut payload = vec![0u8; fixed + table];
            let format_off = (fixed + table) as u32;
            let data_off = format_off + format.len() as u32;
            write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
            write_offset_entry(&mut payload, fixed, 1, format_off, format.len() as u32);
            write_offset_entry(&mut payload, fixed, 2, data_off, blob.len() as u32);
            payload.extend_from_slice(format);
            payload.extend_from_slice(blob);

            let mut frame = vec![0u8; WireHeader::SIZE];
            WireHeader {
                schema_hash:
                    <native_ros2_messages::sensor_msgs::CompressedImage as ShmMessage>::SCHEMA_HASH,
                total_size: (WireHeader::SIZE + payload.len()) as u32,
                offset_table_offset: (WireHeader::SIZE + fixed) as u32,
                offset_table_count: 3,
                sequence: 0,
                timestamp_ns: 42_000,
            }
            .write_to_buf(&mut frame);
            frame.extend_from_slice(&payload);
            frame
        }

        let walker = builtin_walker();

        // The name table says Image with certainty; the CONTENT says video.
        let video = compressed_image_frame(b"h264", &go2_access_unit());
        let resolved = resolve_from_frame(&walker, &video, None).expect("resolves");
        assert_eq!(resolved.schema, "sensor_msgs/CompressedImage");
        assert_eq!(
            resolved.archetype,
            ArchetypeKind::VideoStream,
            "the daemon must report what the sink RENDERS — reporting the name \
             table's answer composes a plot/2D view the video never lands in"
        );
        // The report is what the layout is built from, so pin the consequence too.
        // The layout appends the dump pane a VideoStream's rescan-disagreement arm
        // degrades into; the picture still lands in spatial2d, which is what this
        // arm is about.
        assert_eq!(
            cerulion_viz::blueprint::views_for_archetype(resolved.archetype),
            &[
                cerulion_viz::blueprint::ViewKind::Spatial2d,
                cerulion_viz::blueprint::ViewKind::TextDocument
            ]
        );

        // ANTI-TAUTOLOGY: an ordinary JPEG on the same schema stays Image.
        let jpeg = compressed_image_frame(b"jpeg", &[0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10]);
        assert_eq!(
            resolve_from_frame(&walker, &jpeg, None)
                .expect("resolves")
                .archetype,
            ArchetypeKind::Image,
            "the JPEG path must be untouched"
        );
    }

    /// The detach-zombie fix's PRODUCTION CARRIER.
    ///
    /// `default_layout_excluding` was pinned by `topic_view_identity_test`, but the
    /// thing that FEEDS it — the daemon's ever-logged accumulation — had none, and a
    /// carrier that silently returned only the attached set would have left every one
    /// of those pins green while the zombie was fully back.
    ///
    /// Drives the real gesture sequence against a hand oracle: attach both topics,
    /// detach the descendant, then re-attach it.
    #[test]
    fn logged_entities_accumulate_across_attach_detach_and_re_attach() {
        let att = |topic: &str| AttachedRender {
            topic: topic.to_string(),
            entity: route_for_input(&route_key_for_topic(topic, None)).entity,
            archetype: Some(cerulion_viz::sink::ArchetypeKind::Image),
            producer_count: None,
            video_renditions: Vec::new(),
            representation: Representation::Auto,
            render_proof: RenderProof::default(),
        };
        let parent = att("/camera/image_raw");
        let child = att("/camera/image_raw/compressed");
        let mut ever = std::collections::BTreeSet::new();

        // 1. both attached — both known.
        record_logged_entities(&mut ever, [parent.entity.clone(), child.entity.clone()]);
        let snap = snapshot_logged_entities(&ever);
        assert_eq!(
            snap,
            vec![
                "world/camera/image_raw".to_string(),
                "world/camera/image_raw/compressed".to_string(),
            ]
        );

        // 2. the CHILD detaches — it is gone from `attached` but its data is still
        // in the store, so it must SURVIVE in the snapshot. This is the assert the
        // whole fix rests on.
        record_logged_entities(&mut ever, [parent.entity.clone()]);
        let snap = snapshot_logged_entities(&ever);
        assert_eq!(
            snap,
            vec![
                "world/camera/image_raw".to_string(),
                "world/camera/image_raw/compressed".to_string(),
            ],
            "a detached entity must stay excludable — its frames are still logged"
        );
        // …and it really does keep the exclusion in the plan the daemon applies.
        let plan = default_layout_excluding(std::slice::from_ref(&parent), &snap);
        let views = plan_views(&plan);
        let parent_view = views
            .iter()
            .find(|v| v.origin.as_deref() == Some("world/camera/image_raw"))
            .expect("the ancestor keeps its view");
        assert_eq!(
            parent_view.contents,
            Some(vec![
                "/world/camera/image_raw/**".to_string(),
                "-/world/camera/image_raw/compressed/**".to_string(),
            ]),
            "the detached descendant stays excluded"
        );

        // 3. RE-attaching it is a no-op on the set (no duplicate, no growth).
        record_logged_entities(&mut ever, [parent.entity.clone(), child.entity.clone()]);
        let snap = snapshot_logged_entities(&ever);
        assert_eq!(snap.len(), 2, "re-attach must not duplicate: {snap:?}");

        // 4. Everything detached: the history still holds both.
        record_logged_entities(&mut ever, []);
        let snap = snapshot_logged_entities(&ever);
        assert_eq!(snap.len(), 2, "the set never shrinks: {snap:?}");
    }

    /// A source view with `//` line comments and `/* … */` blocks removed, so a
    /// structural walk cannot be satisfied — or tripped — by prose. Block comments
    /// NEST in Rust, so the scan is depth-tracked. (Crib:
    /// `cerulion_core::tests::cdylib_iox2_log_level_test::code_only`.)
    fn code_only(src: &str) -> String {
        let b = src.as_bytes();
        let (mut out, mut i, mut depth) = (String::with_capacity(src.len()), 0usize, 0usize);
        while i < b.len() {
            if depth == 0 && b[i..].starts_with(b"//") {
                while i < b.len() && b[i] != b'\n' {
                    i += 1;
                }
            } else if b[i..].starts_with(b"/*") {
                depth += 1;
                i += 2;
            } else if depth > 0 && b[i..].starts_with(b"*/") {
                depth -= 1;
                i += 2;
            } else {
                if depth == 0 {
                    out.push(b[i] as char);
                }
                i += 1;
            }
        }
        out
    }

    /// EVERY surface that reports an entity must agree
    /// with the sink, `discover` included.
    ///
    /// attach/list/status report through `reported_entity_for`. If
    /// the three `discover` sites derived from `route_for_input` instead, a
    /// tf-NAMED topic whose frames are not a `tf2_msgs/TFMessage` would be advertised at
    /// the bare viz root `world` while the sink (via `reconcile_tf_route`) and every
    /// other surface use `world/<topic>`. A layout agent laying out pre-attach
    /// would target the scene root, and an `entity` override fed back from such a row
    /// could not round-trip.
    ///
    /// Structural PLUS behavioural: the pure agreement is asserted directly, and the
    /// walk pins that no `discover` site regresses to the raw derivation.
    #[test]
    fn discover_reports_the_same_entity_as_attach_for_a_tf_named_non_tf_topic() {
        let key = route_key_for_topic("/robot1/tf", None);
        // The DIVERGENCE to avoid: the raw derivation says the viz root…
        assert_eq!(route_for_input(&key).entity, "world");
        // …while every reporting surface, given the resolved kind, says the entity
        // the sink actually logs at. All four surfaces derive through this one fn.
        let discover = reported_entity_for(&key, Some(ArchetypeKind::Odometry));
        let attach = reported_entity_for(&key, Some(ArchetypeKind::Odometry));
        assert_eq!(discover, "world/robot1/tf");
        assert_eq!(discover, attach, "discover must agree with attach");
        // A REAL TFMessage keeps the root answer everywhere (anti-tautology: the
        // reconcile is schema-gated, not unconditional).
        assert_eq!(
            reported_entity_for(&key, Some(ArchetypeKind::Transforms)),
            "world"
        );
        // …and an unresolved topic falls back to the name-derived answer on both.
        assert_eq!(reported_entity_for(&key, None), "world");

        // No production site may derive a REPORTED entity from `route_for_input`
        // directly — that is what left `discover` behind.
        let whole = code_only(include_str!("daemon.rs"));
        let src = whole
            .split_once("#[cfg(test)]")
            .map(|(before, _)| before)
            .expect("this module has a test cfg gate");
        assert_eq!(
            src.matches("route_for_input(").count(),
            0,
            "every reported entity must go through `reported_entity_for` / \
             `non_tf_entity_for`; a raw `route_for_input` call is the divergence"
        );
    }

    /// The ever-logged recorder must be fed from a seam
    /// that runs under EVERY [`LayoutMode`], not only `Auto`.
    ///
    /// Fed inside `consolidated_default_plan`, which
    /// `maybe_apply_default_layout` only reaches when the mode is `Auto`,
    /// it would record nothing for the whole of a normal Studio session, because
    /// `compose_layout` and `set_blueprint(spec)` flip the mode to `Explicit`. A
    /// topic attached and detached inside that window would be missing from the exclusion
    /// set when the user later resets the layout, so its still-stored frames would be
    /// swept back into a still-attached ancestor's view: the detach zombie, reopened
    /// on the very path the reset exclusion closes.
    ///
    /// Structural, for the same reason as the single-producer guard: the gate is in
    /// the CALLER, so no test of the recorder itself can see it. Pins that the write
    /// happens in `attached_render_infos` (which every attach, detach, compose and
    /// reset passes through) and NOT inside `consolidated_default_plan` or
    /// `maybe_apply_default_layout`, both of which sit behind the mode gate.
    #[test]
    fn the_logged_entity_recorder_is_fed_from_a_mode_independent_seam() {
        let whole = code_only(include_str!("daemon.rs"));
        let src = whole
            .split_once("#[cfg(test)]")
            .map(|(before, _)| before)
            .expect("this module has a test cfg gate");
        // Exactly one production call site, and it is the ungated one.
        assert_eq!(
            src.matches("self.note_logged_entities(").count(),
            1,
            "the recorder must have ONE production feeder"
        );
        let fed_in = |method: &str| -> bool {
            let Some(at) = src.find(&format!("fn {method}(")) else {
                panic!("{method} not found");
            };
            // The method body ends at the next top-level `\n    fn ` (4-space indent).
            let rest = &src[at..];
            let end = rest[1..].find("\n    fn ").map_or(rest.len(), |i| i + 1);
            rest[..end].contains("self.note_logged_entities(")
        };
        assert!(
            fed_in("attached_render_infos"),
            "the recorder must be fed from `attached_render_infos`, which runs under \
             every LayoutMode"
        );
        assert!(
            !fed_in("consolidated_default_plan"),
            "feeding the recorder here starves it under LayoutMode::Explicit"
        );
        assert!(
            !fed_in("maybe_apply_default_layout"),
            "feeding the recorder here starves it under LayoutMode::Explicit"
        );
        // …and the gate this guards really is in that caller (anti-tautology: if the
        // early return went away the guard would be pinning nothing).
        assert!(
            src.contains("if *guard != LayoutMode::Auto {"),
            "the mode gate this guard exists for must still be in `maybe_apply_default_layout`"
        );
    }

    /// There must be exactly ONE producer of the consolidated
    /// default plan in this module.
    ///
    /// The exclusion of detached-but-still-logged entities lives in
    /// [`Ctx::consolidated_default_plan`]. A `set_blueprint` RESET arm that
    /// derived its own plan with plain `default_layout` would make resetting the layout
    /// silently drop every exclusion and re-open the detach zombie — a bypass
    /// no test of `default_layout_excluding` itself could ever see, because the
    /// bypass is in the CALLER.
    ///
    /// A structural guard rather than a behavioural one: the zombie is a rendering
    /// artifact of the rerun store, which a unit test cannot observe, and a
    /// hand-maintained list of call sites reproduces the very failure it guards (a
    /// reset arm is easy to miss precisely because nothing enumerates the producers).
    /// Adding any second call — including the exact bypass, `default_layout_excluding
    /// (&self.attached_render_infos(), &[])` — fails HERE.
    #[test]
    fn the_consolidated_default_plan_has_exactly_one_producer() {
        let whole = code_only(include_str!("daemon.rs"));
        // PRODUCTION code only — the test module below legitimately calls the
        // layout builder directly to assert what the daemon would apply.
        let src = whole
            .split_once("#[cfg(test)]")
            .map(|(before, _)| before)
            .expect("this module has a test cfg gate");
        let calls = src.matches("default_layout_excluding(").count();
        assert_eq!(
            calls, 1,
            "every default plan must be derived by `Ctx::consolidated_default_plan`, \
             which is what carries the detached-entity exclusions; found {calls} call sites"
        );
        // …and it really is the one inside that method.
        assert!(
            src.contains("fn consolidated_default_plan"),
            "the single producer must be `consolidated_default_plan`"
        );
    }

    /// Flatten a plan to its leaf views (the daemon-side twin of the helper in
    /// `topic_view_identity_test`).
    fn plan_views(plan: &BlueprintPlan) -> Vec<cerulion_viz::blueprint::PlanView> {
        fn walk(
            node: &cerulion_viz::blueprint::PlanNode,
            out: &mut Vec<cerulion_viz::blueprint::PlanView>,
        ) {
            match node {
                cerulion_viz::blueprint::PlanNode::View(v) => out.push(v.clone()),
                cerulion_viz::blueprint::PlanNode::Container(c) => {
                    c.children.iter().for_each(|ch| walk(ch, out))
                }
            }
        }
        let mut out = Vec::new();
        walk(&plan.root, &mut out);
        out
    }
    use super::*;

    /// A counting SPY [`DemandPlane`] (a DI test double — Principle #13,
    /// not fake data): records every `(robot, topic, schema_hash)` demanded and
    /// every `(robot, topic)` released, and can be armed to FAIL the next demand
    /// (the rollback arm). No netd process, no network — the tests assert against
    /// HAND oracles (exact keys, exact call counts).
    #[derive(Default)]
    struct SpyDemandPlane {
        demands: Mutex<Vec<(String, String, u64)>>,
        releases: Mutex<Vec<(String, String)>>,
        fail_demand: std::sync::atomic::AtomicBool,
        /// The canned catalog replies `query_catalog` returns (default
        /// empty — "netd reached the LAN, nobody catalogs it").
        catalogs: Mutex<Vec<CatalogReply>>,
        /// The canned schema replies `query_schema` returns.
        schemas: Mutex<Vec<SchemaReply>>,
        /// When set, the query methods return `Err` (netd-unreachable) so
        /// the caller's transient-session fallback path is exercised.
        fail_query: std::sync::atomic::AtomicBool,
    }
    impl SpyDemandPlane {
        fn set_fail_demand(&self, fail: bool) {
            self.fail_demand.store(fail, Ordering::SeqCst);
        }
        fn demand_count(&self) -> usize {
            self.demands.lock().unwrap().len()
        }
        fn release_keys(&self) -> Vec<(String, String)> {
            self.releases.lock().unwrap().clone()
        }
        fn set_catalogs(&self, catalogs: Vec<CatalogReply>) {
            *self.catalogs.lock().unwrap() = catalogs;
        }
        fn set_schemas(&self, schemas: Vec<SchemaReply>) {
            *self.schemas.lock().unwrap() = schemas;
        }
        fn set_fail_query(&self, fail: bool) {
            self.fail_query.store(fail, Ordering::SeqCst);
        }
    }
    impl DemandPlane for SpyDemandPlane {
        fn demand(&self, robot: &str, topic: &str, schema_hash: u64) -> Result<(), String> {
            if self.fail_demand.load(Ordering::SeqCst) {
                return Err("spy: forced demand failure".to_string());
            }
            self.demands
                .lock()
                .unwrap()
                .push((robot.to_string(), topic.to_string(), schema_hash));
            Ok(())
        }
        fn release(&self, robot: &str, topic: &str) -> Result<(), String> {
            self.releases
                .lock()
                .unwrap()
                .push((robot.to_string(), topic.to_string()));
            Ok(())
        }
        fn query_catalog(&self, _robot: &str) -> Result<Vec<CatalogReply>, String> {
            if self.fail_query.load(Ordering::SeqCst) {
                return Err("spy: netd unreachable".to_string());
            }
            Ok(self.catalogs.lock().unwrap().clone())
        }
        fn query_catalog_all(&self) -> Result<Vec<CatalogReply>, String> {
            // The LAN gather returns the SAME canned catalogs (the
            // fail_query gate models a netd-unreachable degrade).
            if self.fail_query.load(Ordering::SeqCst) {
                return Err("spy: netd unreachable".to_string());
            }
            Ok(self.catalogs.lock().unwrap().clone())
        }
        fn query_schema(&self, _robot: &str, _requested: &str) -> Result<Vec<SchemaReply>, String> {
            if self.fail_query.load(Ordering::SeqCst) {
                return Err("spy: netd unreachable".to_string());
            }
            Ok(self.schemas.lock().unwrap().clone())
        }
    }

    /// Build a one-robot [`CatalogReply`] naming `topic`'s type (a test-oracle
    /// helper — NOT fake data, a hand-built catalog).
    fn catalog(robot: &str, topic: &str, schema_name: Option<&str>) -> CatalogReply {
        use cerulion_core::{CatalogEntry, CatalogProvenance};
        CatalogReply {
            version: 1,
            robot: robot.to_string(),
            entries: vec![CatalogEntry {
                topic: topic.to_string(),
                schema_hash: Some(7),
                schema_name: schema_name.map(str::to_string),
                provenance: CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            }],
            error: None,
        }
    }

    #[test]
    fn resolve_type_from_catalogs_extracts_the_first_named_type() {
        // The PURE topic→type extraction the netd + transient paths share.
        // Hand oracle: a catalog naming the topic's type resolves; a topic with no
        // name (a boot-plan hashless announce) → None; an empty gather → None.
        let cats = vec![
            catalog("go2", "/scan", Some("sensor_msgs/LaserScan")),
            catalog("ubuntu", "/tf", Some("tf2_msgs/TFMessage")),
        ];
        assert_eq!(
            resolve_type_from_catalogs(&cats, "/tf"),
            Some("tf2_msgs/TFMessage".to_string())
        );
        assert_eq!(
            resolve_type_from_catalogs(&cats, "/scan"),
            Some("sensor_msgs/LaserScan".to_string())
        );
        // Catalogued topic but NO schema name → None (the "names no type" case).
        let unnamed = vec![catalog("go2", "/odom", None)];
        assert_eq!(resolve_type_from_catalogs(&unnamed, "/odom"), None);
        // Topic in no catalog → None.
        assert_eq!(resolve_type_from_catalogs(&cats, "/missing"), None);
        // Empty gather → None.
        assert_eq!(resolve_type_from_catalogs(&[], "/tf"), None);
    }

    /// A multi-entry [`CatalogReply`] oracle helper (a hand-built catalog,
    /// NOT fake data).
    fn multi_catalog(robot: &str, entries: &[(&str, Option<&str>)]) -> CatalogReply {
        use cerulion_core::CatalogProvenance;
        CatalogReply {
            version: 1,
            robot: robot.to_string(),
            entries: entries
                .iter()
                .map(|(topic, schema_name)| CatalogEntry {
                    topic: topic.to_string(),
                    schema_hash: Some(7),
                    schema_name: schema_name.map(str::to_string),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None,
                    liveness: None,
                })
                .collect(),
            error: None,
        }
    }

    #[test]
    fn build_discovered_robots_groups_attributes_infers_and_filters_local() {
        // The PURE ROBOTS-section builder — grouping + robot attribution +
        // name-inferred archetype hint + local-topic filter + refusal surfacing +
        // deterministic sort. Hand oracle throughout (never a self-compare).
        use std::collections::BTreeSet;
        // Two robots (given OUT of sorted order → must sort to [ubuntu, zzz]); ubuntu
        // serves a mapped type (/scan), an UNNAMED type (/odom), and a topic already
        // present LOCALLY (/cloud → filtered out). The catalog gives /scan AFTER /odom
        // so the topic sort is exercised too.
        let local: BTreeSet<String> = ["/cloud".to_string()].into_iter().collect();
        let cats = vec![
            multi_catalog(
                "ubuntu",
                &[
                    ("/cloud", Some("sensor_msgs/PointCloud2")), // local-present → filtered
                    ("/odom", None),                             // unnamed → schema/archetype null
                    ("/scan", Some("sensor_msgs/LaserScan")),    // mapped → archetype hint
                ],
            ),
            multi_catalog("zzz", &[("/tf", Some("tf2_msgs/TFMessage"))]),
        ];
        let robots = build_discovered_robots(cats, &local);

        // Sorted by robot identity: ubuntu before zzz.
        assert_eq!(robots.len(), 2);
        assert_eq!(robots[0].robot, "ubuntu");
        assert_eq!(robots[1].robot, "zzz");

        // ubuntu: /cloud filtered (local-present); /odom + /scan remain, sorted.
        let u = &robots[0];
        assert!(u.error.is_none());
        assert_eq!(
            u.topics
                .iter()
                .map(|e| e.topic.as_str())
                .collect::<Vec<_>>(),
            vec!["/odom", "/scan"]
        );
        // Every remote entry self-attributes its origin robot (for attach).
        assert!(u
            .topics
            .iter()
            .all(|e| e.robot.as_deref() == Some("ubuntu")));

        // /odom — unnamed type → schema/archetype null, components/view_kinds null.
        let odom = &u.topics[0];
        assert_eq!(odom.schema, None);
        assert_eq!(odom.archetype, None);
        assert_eq!(odom.components, None);
        assert_eq!(odom.view_kinds, None);

        // /scan — mapped LaserScan → schema name + a NAME-inferred archetype hint
        // (no frame) + a deterministic render entity.
        let scan = &u.topics[1];
        assert_eq!(scan.schema.as_deref(), Some("sensor_msgs/LaserScan"));
        assert_eq!(
            scan.archetype.as_deref(),
            Some(archetype_name(
                classify_schema("sensor_msgs/LaserScan").unwrap()
            ))
        );
        assert!(
            scan.components.is_some(),
            "a mapped archetype fills components"
        );
        assert_eq!(
            scan.entity,
            route_for_input(&route_key_for_topic("/scan", None)).entity
        );

        // zzz: its /tf survives (not local-present).
        assert_eq!(
            robots[1]
                .topics
                .iter()
                .map(|e| e.topic.as_str())
                .collect::<Vec<_>>(),
            vec!["/tf"]
        );
    }

    #[test]
    fn remote_discovered_entry_carries_producer_count_liveness() {
        // THE reported scenario end-to-end through the PURE catalog→discover
        // mapping. A robot's catalog names a LIVE topic (`producer_count: Some(1)`) and
        // a registered-but-DEAD route (`Some(0)`, the `/uslam/cloud_map` case) plus an
        // old-robot un-probed topic (`None`). `build_discovered_robots` must carry each
        // through to the DiscoveredEntry verbatim so the sidebar can dim the dead route
        // + label it "no data yet" while the live one renders normally. Hand oracle.
        use cerulion_core::CatalogProvenance;
        use std::collections::BTreeSet;
        let cat = CatalogReply {
            version: 1,
            robot: "go2".to_string(),
            entries: vec![
                CatalogEntry {
                    topic: "/utlidar/cloud".to_string(),
                    schema_hash: Some(1),
                    schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                    provenance: CatalogProvenance::Runtime,
                    producer_count: Some(1), // a live 20 Hz topic
                    liveness: None,
                },
                CatalogEntry {
                    topic: "/uslam/cloud_map".to_string(),
                    schema_hash: None,
                    schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                    provenance: CatalogProvenance::Boot,
                    producer_count: Some(0), // a registered-but-dead DDS route
                    liveness: None,
                },
                CatalogEntry {
                    topic: "/legacy".to_string(),
                    schema_hash: None,
                    schema_name: None,
                    provenance: CatalogProvenance::Runtime,
                    producer_count: None, // an old robot that never stamped it
                    liveness: None,
                },
            ],
            error: None,
        };
        let robots = build_discovered_robots(vec![cat], &BTreeSet::new());
        assert_eq!(robots.len(), 1);
        let entries = &robots[0].topics; // sorted by topic
                                         // Sorted: /legacy, /uslam/cloud_map, /utlidar/cloud.
        let by_topic = |t: &str| {
            entries
                .iter()
                .find(|e| e.topic == t)
                .unwrap_or_else(|| panic!("entry {t} missing"))
        };
        assert_eq!(
            by_topic("/utlidar/cloud").producer_count,
            Some(1),
            "a live topic carries its producer count"
        );
        assert_eq!(
            by_topic("/uslam/cloud_map").producer_count,
            Some(0),
            "the dead route carries Some(0) — the sidebar dims it + 'no data yet'"
        );
        assert_eq!(
            by_topic("/legacy").producer_count,
            None,
            "an un-probed topic stays None (degrades to pre-876 rendering)"
        );
    }

    /// The catalog → discover mapping carries the robot's DATA-FLOW
    /// liveness through and classifies it ONCE, here, from the shared thresholds.
    ///
    /// This hand oracle models three rows that ALL
    /// report `producer_count: Some(1)` — because the `ros2 attach` bridge graph
    /// registered a publisher for every discovered DDS topic at build time — so
    /// `producer_count` cannot separate them, and only the observation can. The
    /// assertions are written against hand-chosen expected states, never read back
    /// from the values under test.
    #[test]
    fn remote_entries_carry_data_flow_liveness_and_its_classification() {
        use cerulion_core::{CatalogProvenance, LivenessState, TopicLiveness, TopicRateEstimate};
        use std::collections::BTreeSet;
        let entry = |topic: &str, liveness: Option<TopicLiveness>| CatalogEntry {
            topic: topic.to_string(),
            schema_hash: Some(1),
            schema_name: Some("sensor_msgs/PointCloud2".to_string()),
            provenance: CatalogProvenance::Runtime,
            // IDENTICAL on every row — the defect in one line.
            producer_count: Some(1),
            liveness,
        };
        let cat = CatalogReply {
            version: 1,
            robot: "go2".to_string(),
            entries: vec![
                // Streaming: a frame 48 ms ago, with an EXACT measured rate.
                entry(
                    "/utlidar/cloud_deskewed",
                    Some(TopicLiveness {
                        last_frame_age_ms: Some(48),
                        observed_for_ms: 30_000,
                        frames_observed: 600,
                        rate_estimate: Some(TopicRateEstimate {
                            millihertz: 20_000,
                            is_floor: false,
                        }),
                    }),
                ),
                // Streaming with a LABELLED FLOOR. The two
                // `is_floor` values must BOTH survive the hop — a mapping that
                // dropped the field, or hardcoded either flag, would still satisfy
                // a single-row fixture.
                entry(
                    "/lowstate",
                    Some(TopicLiveness {
                        last_frame_age_ms: Some(2),
                        observed_for_ms: 30_000,
                        frames_observed: 300,
                        rate_estimate: Some(TopicRateEstimate {
                            millihertz: 10_000,
                            is_floor: true,
                        }),
                    }),
                ),
                // The dead DDS route: watched 30 s, never a frame — and therefore
                // NO rate (the absent-value negative for the arm above).
                entry(
                    "/uslam/cloud_map",
                    Some(TopicLiveness {
                        last_frame_age_ms: None,
                        observed_for_ms: 30_000,
                        frames_observed: 0,
                        rate_estimate: None,
                    }),
                ),
                // An older robot / disabled observer: UNKNOWN, never "dead".
                entry("/legacy", None),
            ],
            error: None,
        };
        let robots = build_discovered_robots(vec![cat], &BTreeSet::new());
        let entries = &robots[0].topics;
        let by_topic = |t: &str| {
            entries
                .iter()
                .find(|e| e.topic == t)
                .unwrap_or_else(|| panic!("entry {t} missing"))
        };

        // Every row's publisher count is the SAME: the presence signal is blind here.
        for e in entries {
            assert_eq!(
                e.producer_count,
                Some(1),
                "{}: the bridge registers a publisher per DDS topic",
                e.topic
            );
        }

        let streaming = by_topic("/utlidar/cloud_deskewed");
        assert_eq!(streaming.liveness_state, Some(LivenessState::Streaming));
        assert_eq!(
            streaming.liveness.expect("carried").last_frame_age_ms,
            Some(48),
            "the raw age is carried verbatim for a row that wants to show it"
        );
        assert_eq!(streaming.liveness.expect("carried").frames_observed, 600);

        // The rate estimate survives this hop. Before
        // this, EVERY fixture reaching the mapping passed `rate_estimate: None`, so
        // the only test touching the field built a `DiscoveredEntry` by hand and
        // serialized it — which pins the WIRE shape, not the catalog→discover
        // mapping. A mapping that dropped the field was green.
        assert_eq!(
            streaming.liveness.expect("carried").rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 20_000,
                is_floor: false,
            }),
            "an exactly-measured rate arrives verbatim, floor flag included"
        );

        let floored = by_topic("/lowstate");
        assert_eq!(floored.liveness_state, Some(LivenessState::Streaming));
        assert_eq!(
            floored.liveness.expect("carried").rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 10_000,
                is_floor: true,
            }),
            "a LABELLED floor arrives with its label intact — the sidebar renders \
             `>= N Hz` off exactly this bit"
        );

        let dead = by_topic("/uslam/cloud_map");
        assert_eq!(
            dead.liveness_state,
            Some(LivenessState::NoData),
            "the dead route is the ONE row the sidebar dims — and it is dimmed on \
             the observation, not on the (identical) producer count"
        );
        assert_eq!(dead.liveness.expect("carried").last_frame_age_ms, None);
        assert_eq!(
            dead.liveness.expect("carried").rate_estimate,
            None,
            "a row the robot measured no rate for arrives with none — the negative \
             that stops the arms above passing on a hardcoded estimate"
        );

        let legacy = by_topic("/legacy");
        assert_eq!(legacy.liveness, None);
        assert_eq!(
            legacy.liveness_state, None,
            "no observation ⇒ UNKNOWN, which must NOT render as dead"
        );
    }

    #[test]
    fn build_discovered_robots_surfaces_a_refusal_and_empty_gather() {
        // A refused catalog rows EXPLICITLY (reason + no
        // topics), never a silent empty; an empty gather → no robots.
        use std::collections::BTreeSet;
        let refused = CatalogReply::refused("locked", "pairing required");
        let robots = build_discovered_robots(vec![refused], &BTreeSet::new());
        assert_eq!(robots.len(), 1);
        assert_eq!(robots[0].robot, "locked");
        assert!(robots[0].topics.is_empty());
        assert_eq!(robots[0].error.as_deref(), Some("pairing required"));

        assert!(build_discovered_robots(vec![], &BTreeSet::new()).is_empty());
    }

    #[test]
    fn build_discovered_robots_rows_a_fully_attached_robot_with_no_extra_topics() {
        // A NON-refused robot whose EVERY served topic is
        // ALREADY visible locally (the desk attached all of its mirrors) still ROWS —
        // `topics == []`, `error == None`. The robot IS present (the "one
        // data source = one topic" surface: its topics are already listed + attachable
        // under the LOCAL section), just fully-attached, so the sidebar renders its
        // "no additional topics" branch rather than dropping the robot. Without this
        // arm, a mutation that DROPPED such rows (or an
        // `error`-vs-empty conflation) would pass green. Hand oracle throughout.
        use std::collections::BTreeSet;
        let local: BTreeSet<String> = ["/scan".to_string(), "/tf".to_string()]
            .into_iter()
            .collect();
        // Every catalogued topic is local-present → all filtered out.
        let cats = vec![multi_catalog(
            "ubuntu",
            &[
                ("/scan", Some("sensor_msgs/LaserScan")),
                ("/tf", Some("tf2_msgs/TFMessage")),
            ],
        )];
        let robots = build_discovered_robots(cats, &local);
        assert_eq!(robots.len(), 1, "the fully-attached robot still rows");
        assert_eq!(robots[0].robot, "ubuntu");
        assert!(
            robots[0].topics.is_empty(),
            "every served topic was local-present → filtered to an empty topic list"
        );
        // The DISCRIMINATOR vs a refusal: `error` is None (present-but-attached), NOT a
        // reason string. A refused robot ALSO has empty topics, so this pins that the
        // two reachable empty-topics states stay distinct.
        assert_eq!(
            robots[0].error, None,
            "a fully-attached robot is present, NOT refused"
        );
    }

    /// A hand-built streaming-mirror [`DiscoveredEntry`] (a mirror the desk taps
    /// locally), self-attributing its origin robot — a test oracle (NOT fake data).
    fn streaming_entry(topic: &str, robot: &str, schema: Option<&str>) -> DiscoveredEntry {
        DiscoveredEntry {
            topic: topic.to_string(),
            schema: schema.map(str::to_string),
            archetype: None,
            entity: route_for_input(&route_key_for_topic(topic, None)).entity,
            components: None,
            view_kinds: None,
            robot: Some(robot.to_string()),
            // A streaming mirror is live by definition; the pure fold tests do not
            // assert on the count, so `None` (un-probed) keeps them count-agnostic.
            producer_count: None,
            liveness: None,
            liveness_state: None,
            representation: None,
            undecodable: None,
        }
    }

    /// A catalog-won row carries the MIRROR's live layout
    /// metadata — the half the `discover` fix built and this fold threw away.
    ///
    /// `discover` computes a mirror row's `(components, view_kinds)` from the live
    /// signals when this desk is rendering the topic; the fold then hits its
    /// duplicate guard, `continue`s, and the row a client reads is
    /// `remote_discovered_entry`'s — built from BLANK signals. So on the Studio
    /// flagship flow (attach a robot topic → netd mirrors it → the desk renders it
    /// → the sidebar re-polls `discover`) the sidebar advertised the PRE-DROP
    /// companion while `status`/`list`/both attach replies reported the applied
    /// set. One run, two answers, on the surface the sidebar polls.
    #[test]
    fn a_catalog_won_row_carries_the_mirrors_live_layout_metadata() {
        let topic = "/go2/markers";
        // The catalog's own row: archetype inferred from the SCHEMA NAME, blank
        // signals, so the companion is KEPT.
        let robots = build_discovered_robots(
            vec![multi_catalog(
                "go2",
                &[(topic, Some("visualization_msgs/MarkerArray"))],
            )],
            &Default::default(),
        );
        let catalog_views = robots[0].topics[0].view_kinds.clone();
        assert_eq!(
            catalog_views,
            Some(vec!["spatial3d".to_string(), "text_document".to_string()]),
            "PRECONDITION: the catalog row keeps the companion — if it did not, the \
             carry below would be indistinguishable from doing nothing"
        );

        // The MIRROR row this desk built: same archetype, and the live signals say
        // the topic is provably rendering, so the companion is refused.
        let mut mirror = streaming_entry(topic, "go2", Some("visualization_msgs/MarkerArray"));
        mirror.archetype = Some("MarkerArray".to_string());
        mirror.components = Some(vec!["Points3D".to_string()]);
        mirror.view_kinds = Some(vec!["spatial3d".to_string()]);

        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![("go2".to_string(), mirror)],
            DiscoveryState::Settled,
        );
        assert_eq!(folded.len(), 1, "one robot: {folded:?}");
        assert_eq!(
            folded[0].topics.len(),
            1,
            "still ONE row — the catalog's row wins, it is not duplicated: {:?}",
            folded[0].topics
        );
        let row = &folded[0].topics[0];
        assert_eq!(
            row.view_kinds,
            Some(vec!["spatial3d".to_string()]),
            "THE PIN: the surviving row must report the placement this desk is \
             actually rendering: {row:?}"
        );
        // SCOPE of this one: in PRODUCTION `components` is a pure function
        // of the archetype, and the gate above forces the two archetypes equal — so
        // the assignment is an IDENTITY on every reachable path, and this arm
        // reaches it only because the mirror is hand-built with a different list.
        // Kept deliberately: it guards the refactor that breaks that identity (a
        // components list that starts depending on the representation or on live
        // signals, as `view_kinds` already does), which would otherwise ship with
        // the pair silently half-carried.
        assert_eq!(
            row.components,
            Some(vec!["Points3D".to_string()]),
            "…and its components travel with it — the two are one answer: {row:?}"
        );
    }

    /// The CAUSE travels with the CONSEQUENCE: a catalog-won row carries the
    /// mirror's `representation` beside its view kinds.
    ///
    /// `DiscoveredEntry::representation`'s own contract says this row's view kinds
    /// are ALREADY computed through the choice, so "a row that reported the
    /// consequence without the cause would show view kinds its archetype alone does
    /// not imply and nothing to explain why". Before the carry the pair was
    /// CONSISTENT — `remote_discovered_entry` computed view kinds under a hardcoded
    /// `Auto`, so an absent `representation` was correct — and carrying only the
    /// layout half is what would break it.
    #[test]
    fn a_catalog_won_row_carries_the_mirrors_representation_too() {
        let topic = "/go2/plan";
        let robots = build_discovered_robots(
            vec![multi_catalog("go2", &[(topic, Some("nav_msgs/Path"))])],
            &Default::default(),
        );
        let catalog_row = robots[0].topics[0].clone();
        assert_eq!(
            catalog_row.representation, None,
            "PRECONDITION: the catalog row names no choice — a pure fn cannot see \
             one: {catalog_row:?}"
        );

        // The MIRROR row `discover` built: the operator FORCED both halves, so its
        // view kinds carry the companion BECAUSE of that choice, not despite it.
        let mut mirror = streaming_entry(topic, "go2", Some("nav_msgs/Path"));
        mirror.archetype = catalog_row.archetype.clone();
        mirror.components = catalog_row.components.clone();
        mirror.view_kinds = Some(vec!["spatial3d".to_string(), "text_document".to_string()]);
        mirror.representation = Some("both".to_string());

        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![("go2".to_string(), mirror)],
            DiscoveryState::Settled,
        );
        let row = &folded[0].topics[0];
        assert_eq!(
            row.view_kinds,
            Some(vec!["spatial3d".to_string(), "text_document".to_string()]),
            "the consequence is carried: {row:?}"
        );
        assert_eq!(
            row.representation.as_deref(),
            Some("both"),
            "THE PIN: …and so is the CAUSE — a row showing view kinds its archetype \
             alone does not imply must say WHY: {row:?}"
        );
    }

    /// The carry is GATED on the two archetypes agreeing — driven on the shape that
    /// is actually DOMINANT, so the gate's real cost is visible rather than
    /// implied.
    ///
    /// The refuse class is not two different kinds; it is **`None` vs
    /// `Some(kind)`**. `classify_schema` maps a fixed table of ROS types, so every
    /// unmapped or vendor-custom schema yields `None` on the catalog side while the
    /// mirror side classifies from real bytes — and the FLAGSHIP camera is exactly
    /// that: `unitree_go/Go2FrontVideoData` is in no table, and its frames classify
    /// `VideoStream` by content.
    ///
    /// The consequence, stated exactly: the surviving row carries `null`
    /// components/view_kinds — ABSENCE under the null-not-empty convention, not
    /// wrong data — so `discover` reports "not yet known" for a topic this desk is
    /// rendering. That is the accepted residual, and it is the safer failure: a row
    /// naming one archetype while carrying another's placement cannot be reconciled
    /// by a client at all.
    #[test]
    fn a_mirror_that_classifies_differently_does_not_splice_its_views() {
        let topic = "/go2/camera/h264";
        // The Go2's own camera type — deliberately NOT a ROS built-in, which is why
        // the catalog side has nothing to infer from.
        let robots = build_discovered_robots(
            vec![multi_catalog(
                "go2",
                &[(topic, Some("unitree_go/Go2FrontVideoData"))],
            )],
            &Default::default(),
        );
        let catalog_row = robots[0].topics[0].clone();
        assert_eq!(
            catalog_row.archetype, None,
            "PRECONDITION: an unmapped schema NAME yields no archetype — the \
             dominant refuse shape: {catalog_row:?}"
        );
        assert_eq!(
            catalog_row.view_kinds, None,
            "…and therefore no placement at all (null-not-empty): {catalog_row:?}"
        );

        // The MIRROR row: this desk decoded the frames and knows they are video.
        let mut mirror = streaming_entry(topic, "go2", Some("unitree_go/Go2FrontVideoData"));
        mirror.archetype = Some("VideoStream".to_string());
        mirror.components = Some(vec!["VideoStream".to_string()]);
        mirror.view_kinds = Some(vec!["spatial2d".to_string()]);

        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![("go2".to_string(), mirror)],
            DiscoveryState::Settled,
        );
        let row = &folded[0].topics[0];
        assert_eq!(
            row.archetype, None,
            "the row still names the catalog's (absent) archetype: {row:?}"
        );
        assert_eq!(
            row.view_kinds, None,
            "so it reports NO placement rather than one its own archetype field \
             does not imply — the accepted residual, absence not wrong data: {row:?}"
        );

        // The two-different-KINDS shape refuses identically, and is kept beside it
        // so the gate is pinned on both arms rather than on the common one only.
        let robots = build_discovered_robots(
            vec![multi_catalog(
                "go2",
                &[("/go2/cam/jpeg", Some("sensor_msgs/CompressedImage"))],
            )],
            &Default::default(),
        );
        let named = robots[0].topics[0].clone();
        assert_eq!(named.archetype.as_deref(), Some("Image"));
        let mut mirror =
            streaming_entry("/go2/cam/jpeg", "go2", Some("sensor_msgs/CompressedImage"));
        mirror.archetype = Some("VideoStream".to_string());
        mirror.view_kinds = Some(vec!["spatial2d".to_string()]);
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![("go2".to_string(), mirror)],
            DiscoveryState::Settled,
        );
        assert_eq!(
            folded[0].topics[0].view_kinds, named.view_kinds,
            "a row naming one archetype and placing another is a new lie: {:?}",
            folded[0].topics[0]
        );
    }

    /// **The serve-swap observed on the Go2, both stages,
    /// and every carve-out driven with MORE THAN ONE mirror.**
    ///
    /// The operator changed the robot's graph, which killed `/go2/camera/jpeg`'s
    /// publisher. The sidebar rule has two states for that row, and the row can only ever
    /// reach the SECOND one ("not found locally or on any robot ⇒ it disappears"),
    /// because every desk-side signal is structurally blind to a dead remote
    /// publisher: `cerulion-netd` holds the mirror for as long as anything demands it
    /// and its re-injector owns the local publisher slot, so `producer_count` reads
    /// `Some(1)` for a topic the robot stopped serving. The registration authority
    /// for a robot's topic is the ROBOT'S CATALOG.
    ///
    /// **Every arm below folds at least TWO mirrors of one robot.**
    /// N=1 is exactly the case
    /// that cannot catch a gate reading its own mutations: with one tuple there is no
    /// earlier iteration to have pushed into `row.topics` or cleared `row.error`. At
    /// N >= 2 a gate reading live row state collapses all three carve-outs and drops
    /// a live mirror, with the survivor decided by iteration order. A real robot has
    /// many mirrors; N=1 is never the shipping shape.
    #[test]
    fn a_swap_killed_mirror_stays_while_the_catalog_lists_it_then_leaves_when_it_does_not() {
        use cerulion_core::LivenessState;
        let jpeg = "/go2/camera/jpeg";
        // A tap that saw frames once and none recently — what a swap-killed topic
        // looks like from here (dated Idle, never NoData, because it demonstrably
        // produced). This is the ONLY shape the gate may act on.
        let quiet = |topic: &str| {
            let mut e = streaming_entry(topic, "go2", Some("sensor_msgs/CompressedImage"));
            e.liveness = Some(TopicLiveness {
                last_frame_age_ms: Some(120_000),
                observed_for_ms: 600_000,
                frames_observed: 4200,
                rate_estimate: None,
            });
            e.liveness_state = Some(LivenessState::Idle);
            e
        };
        let flowing = |topic: &str| {
            let mut e = quiet(topic);
            e.liveness_state = Some(LivenessState::Streaming);
            e
        };
        let topics_of = |folded: &[DiscoveredRobot]| -> Vec<String> {
            folded
                .iter()
                .flat_map(|r| r.topics.iter().map(|e| e.topic.clone()))
                .collect()
        };
        // THREE mirrors of ONE robot — the shipping shape, and the one that exposed
        // the snapshot defect. Two are flowing, one is quiet.
        let three_mirrors = || {
            vec![
                ("go2".to_string(), flowing("/go2/lowstate")),
                ("go2".to_string(), quiet(jpeg)),
                ("go2".to_string(), flowing("/go2/utlidar/cloud")),
            ]
        };

        // ── STAGE 1: the catalog STILL lists it → the row stays. ──────────────────
        let robots = build_discovered_robots(
            vec![multi_catalog(
                "go2",
                &[(jpeg, Some("sensor_msgs/CompressedImage")), ("/tf", None)],
            )],
            &Default::default(),
        );
        let folded =
            fold_streaming_mirrors_into_robots(robots, three_mirrors(), DiscoveryState::Settled);
        assert!(
            topics_of(&folded).contains(&jpeg.to_string()),
            "stage 1 — while the robot still serves it the row STAYS (and renders as \
             stale, not hidden): {:?}",
            topics_of(&folded)
        );

        // ── STAGE 2: the swap landed — the catalog serves OTHER topics, not this one. ──
        let robots = build_discovered_robots(
            vec![multi_catalog("go2", &[("/tf", None), ("/odom", None)])],
            &Default::default(),
        );
        let folded =
            fold_streaming_mirrors_into_robots(robots, three_mirrors(), DiscoveryState::Settled);
        let after = topics_of(&folded);
        assert!(
            !after.contains(&jpeg.to_string()),
            "stage 2: once the robot no longer serves it, the \
             leftover mirror is accounted for NOWHERE, so the sidebar hides the row: \
             {after:?}"
        );
        // The OTHER two mirrors of the same robot are untouched. A gate that
        // reads its own pushes lets a sibling mirror take a later iteration
        // down the drop path purely because an earlier one had been folded in.
        assert!(
            after.contains(&"/go2/lowstate".to_string())
                && after.contains(&"/go2/utlidar/cloud".to_string()),
            "the robot's OTHER mirrors survive — the gate must not read the fold's own \
             earlier iterations: {after:?}"
        );
        assert!(
            after.contains(&"/tf".to_string()) && after.contains(&"/odom".to_string()),
            "and the catalog's own topics are untouched: {after:?}"
        );

        // ── The shapes that must NEVER hide a row, each folded with 3 mirrors. ────
        // (a) UNREACHABLE robot — absent from the gather entirely. NOTE: the FIRST
        //     mirror takes the `None` arm (which never reaches the gate at all), so
        //     only mirrors 2..N exercise it — which is precisely why this arm must
        //     fold more than one.
        let folded = fold_streaming_mirrors_into_robots(
            Vec::new(),
            three_mirrors(),
            DiscoveryState::Settled,
        );
        let after = topics_of(&folded);
        assert_eq!(
            after.len(),
            3,
            "a robot that did not answer tells us nothing about what it serves — ALL \
             its mirrors survive (a live-state gate keeps exactly one): {after:?}"
        );

        // (b) An EMPTY served catalog is NOT authoritative (cold start).
        let robots = build_discovered_robots(vec![multi_catalog("go2", &[])], &Default::default());
        let folded =
            fold_streaming_mirrors_into_robots(robots, three_mirrors(), DiscoveryState::Settled);
        assert_eq!(
            topics_of(&folded).len(),
            3,
            "an empty catalog could be a gather that has not converged — never a reason \
             to hide a row, and never for the 2nd mirror either"
        );

        // (c) A REFUSED catalog serves nothing by definition, so its emptiness says
        //     nothing about any topic.
        let refused = cerulion_core::CatalogReply {
            version: 1,
            robot: "go2".to_string(),
            entries: Vec::new(),
            error: Some("not authorized (account/pairing)".to_string()),
        };
        let robots = build_discovered_robots(vec![refused], &Default::default());
        let folded =
            fold_streaming_mirrors_into_robots(robots, three_mirrors(), DiscoveryState::Settled);
        assert_eq!(
            topics_of(&folded).len(),
            3,
            "a refusal is not an absence claim — every mirror survives it"
        );

        // (d) A mirror we can SEE FRAMES ON is never dropped, whatever the catalog
        //     says — data beats a stale gather.
        let robots = build_discovered_robots(
            vec![multi_catalog("go2", &[("/tf", None)])],
            &Default::default(),
        );
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![("go2".to_string(), flowing(jpeg))],
            DiscoveryState::Settled,
        );
        assert!(
            topics_of(&folded).contains(&jpeg.to_string()),
            "hiding a row with live data on it is the worse error"
        );

        // (e) UNKNOWN liveness (nobody tapped it) is not evidence.
        let robots = build_discovered_robots(
            vec![multi_catalog("go2", &[("/tf", None)])],
            &Default::default(),
        );
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![(
                "go2".to_string(),
                streaming_entry(jpeg, "go2", Some("sensor_msgs/CompressedImage")),
            )],
            DiscoveryState::Settled,
        );
        assert!(
            topics_of(&folded).contains(&jpeg.to_string()),
            "an un-observed mirror stays — unknown is not absence"
        );
    }

    /// An UNCONVERGED gather licenses no absence claim at all.
    ///
    /// The discovery marker exists because an empty/partial answer from netd can mean "discovery
    /// has not finished looking". Dropping a mirror HIDES A ROW, so it is exactly the
    /// kind of claim that state forbids. Without an explicit gate the safety here is incidental —
    /// an unconverged gather usually returns no robots, so the `None` arm happens to
    /// keep the mirror — which is not the same as being correct, and stops being
    /// true the moment a catalog came back non-empty while discovery was still cold.
    ///
    /// Same inputs as stage 2 above (where the drop DOES fire), so the discovery
    /// state is the only variable.
    #[test]
    fn an_unconverged_catalog_gather_never_hides_a_mirror() {
        use cerulion_core::LivenessState;
        let jpeg = "/go2/camera/jpeg";
        let mut quiet = streaming_entry(jpeg, "go2", Some("sensor_msgs/CompressedImage"));
        quiet.liveness = Some(TopicLiveness {
            last_frame_age_ms: Some(120_000),
            observed_for_ms: 600_000,
            frames_observed: 4200,
            rate_estimate: None,
        });
        quiet.liveness_state = Some(LivenessState::Idle);

        let robots = || {
            build_discovered_robots(
                vec![multi_catalog("go2", &[("/tf", None), ("/odom", None)])],
                &Default::default(),
            )
        };
        let listed = |folded: Vec<DiscoveredRobot>| -> bool {
            folded
                .iter()
                .any(|r| r.topics.iter().any(|e| e.topic == jpeg))
        };

        assert!(
            listed(fold_streaming_mirrors_into_robots(
                robots(),
                vec![("go2".to_string(), quiet.clone())],
                DiscoveryState::NotConverged,
            )),
            "an unconverged gather proves nothing — the mirror stays"
        );
        // The anti-tautology half: the SAME inputs under a settled gather DO drop it,
        // so this test cannot pass by the gate being dead.
        assert!(
            !listed(fold_streaming_mirrors_into_robots(
                robots(),
                vec![("go2".to_string(), quiet)],
                DiscoveryState::Settled,
            )),
            "and a settled gather still drops it — the discovery state is the only \
             variable between these two"
        );
    }

    #[test]
    fn fold_streaming_mirrors_attributes_to_robot_dedups_and_survives_no_catalog() {
        // Studio parity: the PURE fold that keeps a re-injected MIRROR
        // attributed to its ORIGIN ROBOT instead of presenting it as a LOCAL topic —
        // THE fix for "checking a robot topic moves it to THIS MACHINE (and stays
        // there after unchecking)". Hand oracle throughout (never a self-compare).

        // ── Arm 1: fold into an EXISTING served robot row (append + sort) ──────────
        // ubuntu's catalog lists /scan; a mirror of /cloud (attributed to ubuntu) folds
        // in and must sort BEFORE /scan.
        let robots = build_discovered_robots(
            vec![multi_catalog(
                "ubuntu",
                &[("/scan", Some("sensor_msgs/LaserScan"))],
            )],
            &std::collections::BTreeSet::new(),
        );
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![(
                "ubuntu".to_string(),
                streaming_entry("/cloud", "ubuntu", Some("sensor_msgs/PointCloud2")),
            )],
            DiscoveryState::Settled,
        );
        assert_eq!(folded.len(), 1);
        assert_eq!(folded[0].robot, "ubuntu");
        assert_eq!(
            folded[0]
                .topics
                .iter()
                .map(|e| e.topic.as_str())
                .collect::<Vec<_>>(),
            vec!["/cloud", "/scan"],
            "the streaming mirror folds in, sorted before the catalog topic"
        );
        // The mirror self-attributes ubuntu (attach demands it — the origin logic).
        let cloud = folded[0]
            .topics
            .iter()
            .find(|e| e.topic == "/cloud")
            .unwrap();
        assert_eq!(cloud.robot.as_deref(), Some("ubuntu"));

        // ── Arm 2: a catalog that ALREADY lists the topic WINS (no duplicate) ──────
        // ubuntu's catalog lists /scan; a streaming mirror of /scan folds in but must
        // NOT double-list it (the catalog entry is kept).
        let robots = build_discovered_robots(
            vec![multi_catalog("ubuntu", &[("/scan", None)])],
            &Default::default(),
        );
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![(
                "ubuntu".to_string(),
                streaming_entry("/scan", "ubuntu", Some("sensor_msgs/LaserScan")),
            )],
            DiscoveryState::Settled,
        );
        assert_eq!(folded.len(), 1);
        assert_eq!(
            folded[0]
                .topics
                .iter()
                .filter(|e| e.topic == "/scan")
                .count(),
            1,
            "the catalog entry wins — the streaming mirror does not double-list it"
        );

        // ── Arm 3: netd-unreachable (NO catalog) still attributes the mirror ────────
        // The network-free guarantee: with an EMPTY robots list (catalog gather
        // degraded), a streaming mirror still CREATES a served row for its robot.
        let folded = fold_streaming_mirrors_into_robots(
            Vec::new(),
            vec![
                (
                    "ubuntu".to_string(),
                    streaming_entry("/scan", "ubuntu", None),
                ),
                (
                    "ubuntu".to_string(),
                    streaming_entry("/cloud", "ubuntu", None),
                ),
            ],
            DiscoveryState::Settled,
        );
        assert_eq!(folded.len(), 1, "one row created for ubuntu");
        assert_eq!(folded[0].robot, "ubuntu");
        assert!(
            folded[0].error.is_none(),
            "a created row is served, not refused"
        );
        assert_eq!(
            folded[0]
                .topics
                .iter()
                .map(|e| e.topic.as_str())
                .collect::<Vec<_>>(),
            vec!["/cloud", "/scan"],
            "both mirrors of the same robot land on ONE row, sorted"
        );

        // ── Arm 4: robots re-sorted; a refused robot with no mirror is untouched ────
        let robots = vec![CatalogReply::refused("locked", "pairing required")];
        let robots = build_discovered_robots(robots, &Default::default());
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![(
                "ubuntu".to_string(),
                streaming_entry("/scan", "ubuntu", None),
            )],
            DiscoveryState::Settled,
        );
        // Sorted: locked before ubuntu.
        assert_eq!(
            folded.iter().map(|r| r.robot.as_str()).collect::<Vec<_>>(),
            vec!["locked", "ubuntu"]
        );
        // locked stays a refusal with no topics (no mirror flowed for it).
        assert_eq!(folded[0].error.as_deref(), Some("pairing required"));
        assert!(folded[0].topics.is_empty());

        // ── Arm 5: the identity fold — no streaming mirrors is a no-op ──────────────
        let robots = build_discovered_robots(
            vec![multi_catalog("ubuntu", &[("/scan", None)])],
            &Default::default(),
        );
        let expected = robots.clone();
        assert_eq!(
            fold_streaming_mirrors_into_robots(robots, Vec::new(), DiscoveryState::Settled),
            expected,
            "an empty streaming set leaves the catalog robots byte-identical"
        );

        // ── Arm 6: a live mirror SUPERSEDES its robot's stale catalog REFUSAL ───────
        // A refused "locked" row + a live mirror attributed to "locked": the flowing
        // mirror is ground truth, so the row becomes SERVED carrying it — never a row
        // with BOTH a reason AND topics (which the sidebar would render as the reason,
        // hiding the mirror). Guards the "refusal serves nothing" invariant.
        let robots = build_discovered_robots(
            vec![CatalogReply::refused("locked", "pairing required")],
            &Default::default(),
        );
        let folded = fold_streaming_mirrors_into_robots(
            robots,
            vec![(
                "locked".to_string(),
                streaming_entry("/scan", "locked", Some("sensor_msgs/LaserScan")),
            )],
            DiscoveryState::Settled,
        );
        assert_eq!(folded.len(), 1);
        assert_eq!(folded[0].robot, "locked");
        assert_eq!(
            folded[0].error, None,
            "the live mirror supersedes the refusal — the row is now SERVED"
        );
        assert_eq!(
            folded[0]
                .topics
                .iter()
                .map(|e| e.topic.as_str())
                .collect::<Vec<_>>(),
            vec!["/scan"],
            "the flowing mirror is attributed, never hidden under an error row"
        );
        assert_eq!(folded[0].topics[0].robot.as_deref(), Some("locked"));
    }

    #[test]
    fn robots_serving_topic_collects_sorts_dedups_and_skips_refusals() {
        // The PURE reachability oracle for the no-`robot` re-attach. Hand oracle.
        let cats = vec![
            multi_catalog("zzz", &[("/cloud", Some("sensor_msgs/PointCloud2"))]),
            multi_catalog(
                "ubuntu",
                &[("/scan", Some("sensor_msgs/LaserScan")), ("/cloud", None)],
            ),
            // A REFUSED catalog serves NOTHING — even if it "named" the topic it never
            // contributes (it carries an error, not entries).
            CatalogReply::refused("locked", "pairing required"),
        ];
        // Two robots serve /cloud, returned SORTED (ubuntu < zzz), deduped.
        assert_eq!(
            robots_serving_topic(&cats, "/cloud"),
            vec!["ubuntu".to_string(), "zzz".to_string()]
        );
        // One robot serves /scan.
        assert_eq!(
            robots_serving_topic(&cats, "/scan"),
            vec!["ubuntu".to_string()]
        );
        // Nobody serves a phantom topic.
        assert!(robots_serving_topic(&cats, "/missing").is_empty());
        assert!(robots_serving_topic(&[], "/cloud").is_empty());
    }

    #[test]
    fn resolve_remote_attach_target_covers_every_outcome() {
        // The PURE decision tree for a no-`robot` re-attach of a
        // not-locally-visible topic — every reachable outcome, hand oracle (never a
        // self-compare). The KEY invariant across ALL error arms: the message is
        // REMOTE-aware — it names the robot / the candidates / the ROBOTS list — and
        // NEVER contains the local graph-YAML "does not exist" for a known-remote topic.
        //
        // Every arm here declares a SETTLED gather, because two of them assert
        // a TERMINAL absence ("no reachable robot", "unreachable") that only a completed
        // discovery pass may license. The unconverged half of each is the sibling test
        // `an_unconverged_gather_never_makes_a_terminal_absence_claim_at_attach`; this
        // test is its anti-tautology partner — it is what fails if the carve-out
        // ever swallows the settled path too.
        let t = "/utlidar/cloud";
        let one = vec!["ubuntu".to_string()];
        let two = vec!["go2".to_string(), "ubuntu".to_string()];

        // TOMBSTONE hit: the remembered robot still serves it → re-demand from it (the
        // detach → re-check ROUND-TRIP heal).
        assert_eq!(
            resolve_remote_attach_target(t, Some("ubuntu"), &one, DiscoveryState::Settled),
            RemoteAttachResolution::Robot("ubuntu".to_string())
        );
        // TOMBSTONE hit even when OTHER robots also serve it — the remembered robot wins
        // (provenance integrity; "one data source = one topic").
        assert_eq!(
            resolve_remote_attach_target(t, Some("ubuntu"), &two, DiscoveryState::Settled),
            RemoteAttachResolution::Robot("ubuntu".to_string())
        );
        // TOMBSTONE but the remembered robot is GONE (empty serving) → the specific
        // "previously served but unreachable" error, remote-aware, never "does not exist".
        match resolve_remote_attach_target(t, Some("ubuntu"), &[], DiscoveryState::Settled) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(msg.contains("ubuntu") && msg.contains(t));
                assert!(msg.contains("previously served") && msg.contains("unreachable"));
                assert!(
                    !msg.contains("does not exist"),
                    "REMOTE-aware, not the local error"
                );
            }
            other => panic!("expected a tombstone-unreachable error, got {other:?}"),
        }
        // TOMBSTONE gone BUT a DIFFERENT robot serves it now → still the specific error,
        // and it HINTS the alternative (so the user can re-attach from it) — the
        // remembered robot never silently flips to another.
        match resolve_remote_attach_target(
            t,
            Some("ubuntu"),
            &["go2".to_string()],
            DiscoveryState::Settled,
        ) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(msg.contains("ubuntu") && msg.contains("previously served"));
                assert!(
                    msg.contains("go2"),
                    "hints the robot that DOES serve it now"
                );
                assert!(!msg.contains("does not exist"));
            }
            other => panic!("expected a tombstone-unreachable error, got {other:?}"),
        }

        // NO tombstone, exactly ONE robot serves it → auto-resolve (full automagic).
        assert_eq!(
            resolve_remote_attach_target(t, None, &one, DiscoveryState::Settled),
            RemoteAttachResolution::Robot("ubuntu".to_string())
        );
        // NO tombstone, MULTIPLE robots → ambiguity error NAMING every candidate.
        match resolve_remote_attach_target(t, None, &two, DiscoveryState::Settled) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(
                    msg.contains("go2") && msg.contains("ubuntu"),
                    "names the candidates"
                );
                assert!(msg.contains("served by robots"));
                assert!(!msg.contains("does not exist"));
            }
            other => panic!("expected a multi-robot ambiguity error, got {other:?}"),
        }
        // NO tombstone, NONE serves it → an explicit remote-aware "no reachable robot".
        match resolve_remote_attach_target(t, None, &[], DiscoveryState::Settled) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(msg.contains(t) && msg.contains("no reachable robot"));
                assert!(msg.contains("ROBOTS list"));
                assert!(
                    !msg.contains("does not exist"),
                    "REMOTE-aware, not the local error"
                );
            }
            other => panic!("expected a no-robot error, got {other:?}"),
        }
    }

    #[test]
    fn an_unconverged_gather_never_makes_a_terminal_absence_claim_at_attach() {
        // The sidebar fold's sibling defect, at the ATTACH call
        // site. An `attach_resolving_remote` that concludes "no reachable robot serves it" (or
        // "that robot no longer announces it") from a gather that may simply not have
        // FINISHED, the cold-start class, renders a terminal "not found"
        // for a topic that exists, at exactly the moment the user asked for it.
        //
        // Hand oracles throughout: each arm asserts the RETRY-WORTHY wording is present
        // AND the TERMINAL phrase is absent, and every claim is paired in-body with its
        // `Settled` twin so the carve-out cannot silently swallow the settled path.
        let t = "/utlidar/cloud";
        let one = vec!["ubuntu".to_string()];
        let two = vec!["go2".to_string(), "ubuntu".to_string()];

        // ── NO tombstone, NOTHING serving, gather NOT CONVERGED ────────────────────
        // The headline: an empty answer from a netd still looking proves nothing.
        match resolve_remote_attach_target(t, None, &[], DiscoveryState::NotConverged) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(msg.contains(t), "still names the topic: {msg}");
                assert!(
                    msg.contains("nothing on the network answered cerulion-netd"),
                    "says WHY the answer is not an absence: {msg}"
                );
                assert!(
                    msg.contains("is UNKNOWN (this is not a claim that it is missing)"),
                    "states the verdict as UNKNOWN, not absence: {msg}"
                );
                assert!(
                    // The head was reworded (see `RETRY_CLAUSE`); the advice stays.
                    msg.contains("Retry in a few seconds")
                        && msg.contains("check the robot is powered on and on this network"),
                    "retry-worthy AND hedged — a desk that never hears a robot is \
                     NotConverged forever, so an unqualified 'retry in a moment' would \
                     be permanently unactionable: {msg}"
                );
                assert!(
                    !msg.contains("no reachable robot"),
                    "NEVER the terminal absence claim off an unconverged gather: {msg}"
                );
                assert!(
                    !msg.contains("does not exist"),
                    "and never the local error either: {msg}"
                );
            }
            other => panic!("expected a retry-worthy unconverged error, got {other:?}"),
        }
        // ANTI-TAUTOLOGY (the arm that dies if the carve-out ever returns): the SAME
        // inputs under a SETTLED gather still make the terminal absence claim, because
        // there netd really did look and really found nothing.
        match resolve_remote_attach_target(t, None, &[], DiscoveryState::Settled) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(
                    msg.contains("no reachable robot"),
                    "a SETTLED empty gather is genuine absence: {msg}"
                );
                assert!(
                    !msg.contains("nothing on the network answered cerulion-netd")
                        && !msg.contains("UNKNOWN"),
                    "and must not hedge: {msg}"
                );
            }
            other => panic!("expected the terminal no-robot error, got {other:?}"),
        }

        // ── TOMBSTONE MISS, gather NOT CONVERGED ───────────────────────────────────
        // The second absence claim: "that robot stopped announcing it" is equally
        // unlicensed when discovery has not finished.
        match resolve_remote_attach_target(t, Some("ubuntu"), &[], DiscoveryState::NotConverged) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(
                    msg.contains("ubuntu") && msg.contains(t),
                    "still names the robot + topic: {msg}"
                );
                assert!(
                    msg.contains("nothing on the network answered cerulion-netd")
                        && msg.contains("is UNKNOWN (this is not a claim that it stopped)"),
                    "states the verdict as UNKNOWN, not a stopped robot: {msg}"
                );
                assert!(
                    // The head was reworded (see `RETRY_CLAUSE`); the advice stays.
                    msg.contains("Retry in a few seconds")
                        && msg.contains("check the robot is powered on and on this network"),
                    "retry-worthy AND hedged (see the sibling arm): {msg}"
                );
                assert!(
                    !msg.contains("unreachable"),
                    "NEVER claims the robot is unreachable off an unconverged gather: {msg}"
                );
                assert!(!msg.contains("does not exist"));
            }
            other => panic!("expected a retry-worthy unconverged tombstone error, got {other:?}"),
        }
        // ANTI-TAUTOLOGY twin: SETTLED keeps the terminal "unreachable" wording.
        match resolve_remote_attach_target(t, Some("ubuntu"), &[], DiscoveryState::Settled) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(
                    msg.contains("previously served") && msg.contains("unreachable"),
                    "a SETTLED gather that omits the robot IS evidence: {msg}"
                );
                assert!(!msg.contains("nothing on the network answered cerulion-netd"));
            }
            other => panic!("expected the terminal tombstone error, got {other:?}"),
        }

        // A PARTIAL unconverged gather still hints whatever it DID find — that half is
        // positive evidence and stays useful; only the absence claim is withheld.
        match resolve_remote_attach_target(
            t,
            Some("ubuntu"),
            &["go2".to_string()],
            DiscoveryState::NotConverged,
        ) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(
                    msg.contains("go2"),
                    "hints the robot the partial gather DID find: {msg}"
                );
                // A PARTIAL gather may not claim nothing answered — go2
                // demonstrably did, and one sentence must not say both.
                assert!(
                    !msg.contains("nothing on the network answered"),
                    "must not contradict its own hint: {msg}"
                );
                assert!(
                    msg.contains("discovery has not converged")
                        && msg.contains("may be incomplete"),
                    "the true weaker claim for a partial gather: {msg}"
                );
                assert!(
                    msg.contains("is UNKNOWN (this is not a claim that it stopped)"),
                    "direction is unchanged — still UNKNOWN, not absence: {msg}"
                );
                assert!(!msg.contains("unreachable"));
            }
            other => panic!("expected a retry-worthy unconverged tombstone error, got {other:?}"),
        }

        // ── POSITIVE evidence is NEVER withheld ────────────────────────────────────
        // An unconverged gather can only ever UNDER-report, so a robot it already found
        // resolves immediately: a cold-start desk attaches at once instead of being told
        // to retry. (These are the arms that would break if the gate were applied to the
        // whole function rather than to the two absence arms.)
        assert_eq!(
            resolve_remote_attach_target(t, None, &one, DiscoveryState::NotConverged),
            RemoteAttachResolution::Robot("ubuntu".to_string()),
            "a found robot resolves even mid-discovery"
        );
        assert_eq!(
            resolve_remote_attach_target(t, Some("ubuntu"), &one, DiscoveryState::NotConverged),
            RemoteAttachResolution::Robot("ubuntu".to_string()),
            "a tombstone HIT resolves even mid-discovery"
        );
        // Ambiguity claims nothing about absence — it names what was found, unchanged.
        match resolve_remote_attach_target(t, None, &two, DiscoveryState::NotConverged) {
            RemoteAttachResolution::Error { message: msg, .. } => {
                assert!(msg.contains("served by robots"));
                assert!(msg.contains("go2") && msg.contains("ubuntu"));
                assert!(
                    !msg.contains("nothing on the network answered cerulion-netd")
                        && !msg.contains("discovery has not converged"),
                    "the ambiguity arm is not an absence claim, so it is not degraded: {msg}"
                );
            }
            other => panic!("expected the multi-robot ambiguity error, got {other:?}"),
        }
    }

    #[test]
    fn catalog_error_messages_are_loud_and_actionable() {
        let no_type = catalog_resolve_error("go2", "/tf");
        assert!(no_type.contains("go2") && no_type.contains("/tf"));
        assert!(
            no_type.contains("=pkg/Type"),
            "names the explicit-type escape hatch"
        );
        let no_answer = catalog_no_answer_error("ubuntu", "/scan");
        assert!(no_answer.contains("ubuntu") && no_answer.contains("/scan"));
        assert!(
            no_answer.contains("--connect"),
            "names the locator escape hatch"
        );
    }

    #[test]
    fn catalog_absence_verdict_withholds_the_terminal_claim_it_has_no_evidence_for() {
        // `catalog_resolve_type`'s "no type for this topic" is
        // an ABSENCE CLAIM — the same defect class as `attach_resolving_remote`'s, one
        // hop over on the SINGLE-robot query verb, which dropped the marker
        // entirely. Hand oracles; every downgrade arm is paired IN BODY with the
        // `Settled` case that must still make the terminal claim.
        use cerulion_core::{CatalogEntry, CatalogProvenance};
        let served = |topic: &str| CatalogReply {
            version: 1,
            robot: "go2".to_string(),
            entries: vec![CatalogEntry {
                topic: topic.to_string(),
                schema_hash: Some(1),
                schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: None,
            }],
            error: None,
        };

        // (1) UNCONVERGED with a NON-EMPTY gather. The robot ANSWERED,
        // so "nothing on the network answered" would contradict this very input —
        // and it is reachable permanently, not just at cold start (the
        // version-skew downgrade hands back a full catalog marked NotConverged).
        // The only weakness is that the view may be incomplete.
        let cats = vec![served("/other")];
        let msg = catalog_absence_verdict("go2", "/tf", &cats, DiscoveryState::NotConverged)
            .expect("an unconverged gather may not license the terminal claim");
        assert!(msg.contains("go2") && msg.contains("/tf"), "{msg}");
        assert!(
            msg.contains("discovery has not converged")
                && msg.contains("this view of the network may be incomplete"),
            "the true weaker claim when the robot DID answer: {msg}"
        );
        assert!(
            !msg.contains("nothing on the network answered"),
            "must not contradict its own input — this gather returned a catalog: {msg}"
        );
        assert!(
            !msg.contains("no robot was discovered"),
            "and must never claim an announce verdict: the SINGLE-robot GET runs no \
             announce harvest, so that disjunct is unknowable here: {msg}"
        );
        assert!(
            msg.contains("is UNKNOWN (this is not a claim that it does not)"),
            "direction unchanged — still UNKNOWN, never absence: {msg}"
        );
        assert!(
            // The head was reworded (see `RETRY_CLAUSE`); the ADVICE survives.
            msg.contains("Retry in a few seconds")
                && msg.contains("check the robot is powered on and on this network"),
            "retry-worthy AND hedged — NotConverged is not necessarily transient: {msg}"
        );
        assert!(
            !msg.contains("serves a catalog"),
            "must not make the terminal claim: {msg}"
        );

        // ANTI-TAUTOLOGY: identical inputs, SETTLED ⇒ the terminal claim is earned
        // (the robot answered, its catalog just has no type for this topic).
        assert_eq!(
            catalog_absence_verdict("go2", "/tf", &cats, DiscoveryState::Settled),
            None,
            "a settled gather that DID return this robot's catalog earns the \
             terminal 'serves a catalog but names no ROS type' error"
        );

        // (2) SETTLED but EMPTY — a real absence, but not the one the terminal
        // message describes: it asserts the robot "serves a catalog", false here.
        // This is `catalog_no_answer_error`'s existing condition, so
        // it reuses that message rather than inventing a second one — otherwise the
        // netd and transient paths would diverge on ONE user-visible condition, which
        // is exactly what message parity exists to prevent.
        let msg = catalog_absence_verdict("go2", "/tf", &[], DiscoveryState::Settled)
            .expect("an empty gather may not claim the robot served a catalog");
        assert_eq!(
            msg,
            catalog_no_answer_error("go2", "/tf"),
            "the netd path reuses the transient path's message for this condition"
        );
        assert!(
            msg.contains("did not answer a schema catalog"),
            "names what actually happened: {msg}"
        );
        assert!(
            !msg.contains("serves a catalog"),
            "and never the false claim: {msg}"
        );
        assert!(
            msg.contains("=pkg/Type") && msg.contains("--connect"),
            "and inherits BOTH hints the invented message had dropped: {msg}"
        );

        // (3) EMPTY and UNCONVERGED — state BOTH facts, and only those two. The
        // announce disjunct the fan-out path uses is deliberately ABSENT: this
        // verdict is fed exclusively by netd's single-robot GET, which never looks
        // at the announce space.
        let msg = catalog_absence_verdict("go2", "/tf", &[], DiscoveryState::NotConverged)
            .expect("still withheld");
        assert!(
            msg.contains("did not answer a schema catalog")
                && msg.contains("discovery has not converged"),
            "states both facts: {msg}"
        );
        assert!(
            !msg.contains("no robot was discovered"),
            "never an announce verdict this query shape cannot observe: {msg}"
        );
        assert!(
            msg.contains("is UNKNOWN (this is not a claim that it does not)"),
            "and still errs UNKNOWN: {msg}"
        );
    }

    /// **No UNKNOWN-verdict message promises a duration this attach
    /// has already out-waited.**
    ///
    /// The clause "discovery often converges within a second or two" is
    /// false: real-LAN convergence was MEASURED at
    /// ~3 to 13 s from a cold desk (the measurement the convergence wait is built on), and
    /// the first-contact wait makes it self-contradicting: the attach seams WAIT through
    /// convergence before rendering any of these, so the user has just watched the
    /// daemon spend seconds the sentence claimed it would not need. All THREE vizd
    /// messages share the clause, so all three are driven here.
    ///
    /// The test drives every message that carries the clause AND asserts, in the same
    /// body, that the retry ADVICE survives — the rule is to claim no duration,
    /// not to stop telling the user to try again. Without that half a message that
    /// dropped the whole paragraph would pass.
    #[test]
    fn no_honest_unknown_message_promises_a_convergence_time_nobody_can_keep() {
        let mut msgs = Vec::new();
        // (1) `resolve_remote_attach_target`, remembered-robot arm.
        match resolve_remote_attach_target(
            "/go2/vel",
            Some("go2"),
            &[],
            DiscoveryState::NotConverged,
        ) {
            RemoteAttachResolution::Error { message: m, .. } => msgs.push(m),
            other => panic!("expected the UNKNOWN-verdict error, got {other:?}"),
        }
        // (2) `resolve_remote_attach_target`, no-tombstone empty-gather arm.
        match resolve_remote_attach_target("/go2/vel", None, &[], DiscoveryState::NotConverged) {
            RemoteAttachResolution::Error { message: m, .. } => msgs.push(m),
            other => panic!("expected the UNKNOWN-verdict error, got {other:?}"),
        }
        // (3) `catalog_absence_verdict` — both of ITS unconverged quadrants share one
        // tail, and both are driven so a rewording of either cannot slip through.
        for cats in [Vec::new(), vec![catalog_reply_for_test("go2", "/other")]] {
            msgs.push(
                catalog_absence_verdict("go2", "/tf", &cats, DiscoveryState::NotConverged)
                    .expect("an unconverged gather never licenses the terminal claim"),
            );
        }

        assert_eq!(msgs.len(), 4, "every clause-carrying message is driven");
        for m in &msgs {
            assert!(
                !m.contains("within a second or two"),
                "promises a convergence time the attach already out-waited: {m}"
            );
            assert!(
                !m.contains("often converges"),
                "and makes no timing promise at all about convergence: {m}"
            );
            // ANTI-TAUTOLOGY: deleting the paragraph would satisfy both negatives.
            assert!(
                m.contains(RETRY_CLAUSE),
                "the retry advice SURVIVES the rewording, verbatim from the ONE \
                 shared const: {m}"
            );
            assert!(
                m.contains("check the robot is powered on and on this network"),
                "and still names the checks for the case where it never converges: {m}"
            );
        }

        // The shared const itself makes no timing claim, and says WHAT it is bounded
        // by instead — a shape that stays true whether the seam waited the full
        // ceiling or (plane-age cap / an older daemon) ran a single round trip.
        assert!(!RETRY_CLAUSE.contains("second or two"));
        assert!(
            RETRY_CLAUSE.contains("may not be discoverable yet"),
            "names the SHAPE of the failure, not a duration: {RETRY_CLAUSE}"
        );
        // It must not imply that anything is WAITING either: vizd answers in one round
        // trip, and whether a client keeps asking is the client's business.
        assert!(
            !RETRY_CLAUSE.contains("waits") && !RETRY_CLAUSE.contains("waiting"),
            "the daemon does not wait, so the message must not say it does: \
             {RETRY_CLAUSE}"
        );
    }

    /// A minimal served [`CatalogReply`] for the prose oracle above.
    fn catalog_reply_for_test(robot: &str, topic: &str) -> CatalogReply {
        use cerulion_core::{CatalogEntry, CatalogProvenance};
        CatalogReply {
            version: 1,
            robot: robot.to_string(),
            entries: vec![CatalogEntry {
                topic: topic.to_string(),
                schema_hash: Some(1),
                schema_name: Some("sensor_msgs/PointCloud2".to_string()),
                provenance: CatalogProvenance::Runtime,
                producer_count: Some(1),
                liveness: None,
            }],
            error: None,
        }
    }

    /// The classifier that gives the netd + transient
    /// schema-fetch paths IDENTICAL, accurate error semantics. Served / NotServed
    /// (docs-empty, reason-carrying) / NoAnswer (no reply) — oracle vectors, never a
    /// self-compare.
    #[test]
    fn classify_schema_replies_distinguishes_served_notserved_and_noanswer() {
        use cerulion_core::SchemaEncoding;
        let doc = SchemaDoc {
            qualified: "pkg/T".to_string(),
            encoding: SchemaEncoding::Msg,
            text: "float64 x\n".to_string(),
            deps: vec![],
        };

        // Served: a reply carrying docs.
        assert_eq!(
            classify_schema_replies(vec![SchemaReply::found("go2", "pkg/T", vec![doc.clone()])]),
            SchemaFetchOutcome::Served(vec![doc.clone()])
        );

        // NotServed: a docs-EMPTY not-found reply carries the robot's OWN reason.
        assert_eq!(
            classify_schema_replies(vec![SchemaReply::not_found(
                "go2",
                "pkg/T",
                "robot has no schema 'pkg/T'"
            )]),
            SchemaFetchOutcome::NotServed("robot has no schema 'pkg/T'".to_string())
        );

        // A REFUSAL (account/pairing) is wire-identical to a not-found → its reason
        // surfaces too (docs-empty + `error: Some`).
        assert_eq!(
            classify_schema_replies(vec![SchemaReply::refused(
                "go2",
                "pkg/T",
                "not authorized (account/pairing)"
            )]),
            SchemaFetchOutcome::NotServed("not authorized (account/pairing)".to_string())
        );

        // NoAnswer: an EMPTY reply set (the transient `None` / netd `Ok(vec![])`).
        assert_eq!(
            classify_schema_replies(vec![]),
            SchemaFetchOutcome::NoAnswer
        );

        // First-docs-WINS even when an earlier reply is a docs-empty not-found.
        assert_eq!(
            classify_schema_replies(vec![
                SchemaReply::not_found("r1", "pkg/T", "r1 has none"),
                SchemaReply::found("r2", "pkg/T", vec![doc.clone()]),
            ]),
            SchemaFetchOutcome::Served(vec![doc])
        );

        // No docs anywhere → the FIRST served reason wins.
        assert_eq!(
            classify_schema_replies(vec![
                SchemaReply::not_found("r1", "pkg/T", "first reason"),
                SchemaReply::refused("r2", "pkg/T", "second reason"),
            ]),
            SchemaFetchOutcome::NotServed("first reason".to_string())
        );
    }

    /// The not-served error carries the robot's OWN
    /// reason VERBATIM (a not-found is never reported as "the
    /// served closure failed to parse"), and the no-answer error stays DISTINCT.
    #[test]
    fn schema_fetch_error_messages_surface_reason_and_distinguish_no_answer() {
        let not_served =
            schema_not_served_error("go2", "pkg/T", "not authorized (account/pairing)");
        assert_eq!(
            not_served,
            "robot 'go2' does not serve schema 'pkg/T': not authorized (account/pairing)"
        );
        assert!(
            !not_served.contains("failed to parse") && !not_served.contains("wire hash"),
            "a not-served answer must NOT claim a parse failure: {not_served}"
        );

        let no_answer = schema_no_answer_error("go2", "pkg/T");
        assert!(no_answer.contains("go2") && no_answer.contains("pkg/T"));
        assert!(
            no_answer.contains("did not answer") && no_answer.contains("unreachable"),
            "the no-answer error keeps the unreachable distinction: {no_answer}"
        );
        assert_ne!(no_answer, not_served);
    }

    #[test]
    fn spy_demand_plane_query_surface_returns_canned_and_fails() {
        // Proves the DI query seam: canned catalogs/schemas come back verbatim; the
        // fail flag surfaces the netd-unreachable Err the daemon degrades on.
        let spy = SpyDemandPlane::default();
        // Default: an empty (authoritative) answer.
        assert!(spy.query_catalog("go2").unwrap().is_empty());
        assert!(spy.query_schema("go2", "pkg/T").unwrap().is_empty());
        // Canned data flows through.
        spy.set_catalogs(vec![catalog("go2", "/tf", Some("tf2_msgs/TFMessage"))]);
        spy.set_schemas(vec![SchemaReply::found(
            "go2",
            "tf2_msgs/TFMessage",
            vec![],
        )]);
        let cats = spy.query_catalog("go2").unwrap();
        assert_eq!(
            resolve_type_from_catalogs(&cats, "/tf").as_deref(),
            Some("tf2_msgs/TFMessage")
        );
        assert_eq!(
            spy.query_schema("go2", "tf2_msgs/TFMessage").unwrap().len(),
            1
        );
        // The fail flag is the netd-unreachable fallback trigger (a loud Err).
        spy.set_fail_query(true);
        assert!(spy.query_catalog("go2").is_err());
        assert!(spy.query_schema("go2", "pkg/T").is_err());
    }

    /// A rate is a claim about NOW, so a stopped stream loses it.
    ///
    /// Nothing prunes the sample window without a fresh `observe_seq`, so
    /// without the recency bound a dead publisher leaves `hz` dividing a fixed
    /// sequence delta by an ever-growing
    /// `dt`: 0.4, 0.3, 0.2 … forever, never `None`. A topic whose publisher
    /// has been killed would sit at "0.3 Hz" indefinitely.
    ///
    /// Hand oracles on BOTH sides of the recency bound, so a widened OR narrowed
    /// gate fails: at the bound the rate still reports; one millisecond past it the
    /// answer is `None`, not a smaller number.
    #[test]
    fn hz_reports_while_streaming_and_refuses_a_rate_once_the_stream_stops() {
        let mut stat = TopicStat::default();
        let base = Instant::now();
        // One sample → not enough to span time.
        observe_seq_gap_free(&mut stat, 0, base);
        assert_eq!(stat.hz(base + Duration::from_millis(10)), None);
        // Two samples spanning 1s with 30 seq → ~30 Hz.
        let last = base + Duration::from_secs(1);
        observe_seq_gap_free(&mut stat, 30, last);
        let hz = stat.hz(last).expect("hz");
        assert!((hz - 30.0).abs() < 1.0, "≈30 Hz, got {hz}");

        // AT the recency bound the newest sample is still fresh — a genuinely SLOW
        // topic must not lose its rate.
        assert!(
            stat.hz(last + HZ_STREAMING_RECENCY).is_some(),
            "a rate at exactly the recency bound is still a claim about now"
        );
        // One millisecond PAST it: the stream has stopped.
        assert_eq!(
            stat.hz(last + HZ_STREAMING_RECENCY + Duration::from_millis(1)),
            None,
            "past the bound the correct answer is the liveness verdict, not a rate"
        );
        // The unbounded signature: far in the future it would serve a small number.
        assert_eq!(
            stat.hz(base + Duration::from_secs(100)),
            None,
            "a dead publisher must never keep a decaying rate (the 0.3 Hz shape)"
        );
    }

    /// Monitors: a sampling instant taken BEFORE the drain that produced the
    /// stats still yields a rate — the claim that it does not is REFUTED here.
    ///
    /// `poll_loop` captures `iteration_start` at the top of the pass and hands THAT
    /// to `sample_monitors`, which runs after the drain has banked its samples at a
    /// slightly later `Instant`. That can be read as fatal: "`hz`
    /// (`iteration_start`) sees its newest timestamp in the future and returns
    /// `None`, so attached rows never learn a baseline or rate". It does not, and
    /// this arm drives the exact two-pass shape the loop produces to say so.
    ///
    /// [`TopicStat::hz`] has two `now`-sensitive guards and NEITHER trips:
    ///
    /// * the recency gate is `saturating_duration_since(newest)`, so a `now` at or
    ///   BEFORE the newest sample reads 0 — inside the bound, never past it;
    /// * `dt` is `checked_duration_since(OLDEST)`, which is the only arm that can
    ///   answer `None`, and it needs `now` to precede the oldest RETAINED sample.
    ///   `observe_seq` runs at most once per topic per pass (`poll_detailed` yields
    ///   one entry per tap), so two samples span two passes and the oldest is
    ///   always from a pass that STARTED before this one.
    ///
    /// What the ordering really costs is `dt` short by ONE PASS DURATION, so the
    /// rate reads high by `pass / span` — a RELATIVE error, and that is the form
    /// the second half pins, because it is the one that bounds the consequence. At
    /// the loop's measured 0.009-0.30 ms pass against a window seconds wide it is
    /// under 0.05 %, which is three and a half orders of magnitude inside
    /// [`MONITOR_RATE_BAND`]'s octave — so the skew cannot move a `rate_deviation`
    /// verdict, which is the only thing on this plane that reads the number. It is
    /// NOT nothing: `desk_rate_estimate` rounds to millihertz, so the two instants
    /// can serve different mHz digits, and a doc claiming byte-equality would be
    /// wrong. The oracle is hand-computed from the sequence delta and the pass span,
    /// never from a second call to the code under test.
    #[test]
    fn monitors_v1_a_sampling_instant_captured_before_the_drain_still_rates_the_tap() {
        let mut stat = TopicStat::default();
        // Pass 1: the loop's `iteration_start`, then the drain a beat later.
        let iteration_start_1 = Instant::now();
        let drain_1 = iteration_start_1 + Duration::from_micros(300);
        observe_seq_gap_free(&mut stat, 0, drain_1);
        assert_eq!(
            stat.hz(iteration_start_1),
            None,
            "one sample spans no time — `None` here is the sample count, not the instant"
        );

        // Pass 2, one second later: SAME ordering, and now there are two samples.
        let iteration_start_2 = iteration_start_1 + Duration::from_secs(1);
        let drain_2 = iteration_start_2 + Duration::from_micros(300);
        observe_seq_gap_free(&mut stat, 30, drain_2);

        // THE REFUTATION: the pre-drain instant rates the tap.
        let sampled = stat
            .hz(iteration_start_2)
            .expect("a sampling instant before the drain must still yield a rate");
        // Hand oracle: 30 sequences over `iteration_start_2 - drain_1`, which is
        // 1 s minus the 300 µs the first drain lagged its own start.
        let span = (Duration::from_secs(1) - Duration::from_micros(300)).as_secs_f64();
        let expected = 30.0 / span;
        assert!(
            (sampled - expected).abs() < 1e-6,
            "hand oracle {expected} Hz, got {sampled}"
        );
        // The COST, bounded in the form that bounds the consequence: `dt` is short
        // by exactly one pass duration, so the rate reads high by `pass / span`.
        let at_drain = stat.hz(drain_2).expect("the drain instant rates it too");
        let relative = (sampled - at_drain).abs() / at_drain;
        let pass_over_span = Duration::from_micros(300).as_secs_f64() / span;
        assert!(
            relative <= pass_over_span * 1.01,
            "the skew must be the pass/span ratio ({pass_over_span}), got {relative} \
             ({sampled} vs {at_drain})"
        );
        // …and that ratio is orders of magnitude inside the octave the only reader
        // of this number judges on, so the ordering can never move a verdict. Both
        // directions, against the SHIPPED band rather than a literal.
        assert!(
            !cerulion_viz::monitor::rate_deviates(
                desk_rate_estimate(at_drain).millihertz,
                desk_rate_estimate(sampled).millihertz,
            ) && !cerulion_viz::monitor::rate_deviates(
                desk_rate_estimate(sampled).millihertz,
                desk_rate_estimate(at_drain).millihertz,
            ),
            "the two instants must never disagree by a rate_deviation: {sampled} vs {at_drain}"
        );
    }

    /// Monitors: re-pointing a tap at a different robot RELEASES the row its old
    /// identity opened, and the new row starts from nothing.
    ///
    /// Monitor rows are keyed `(topic, robot)`, so a tap whose attribution changes
    /// gets a NEW row on the next sampling pass while the old one keeps being
    /// served — and nothing can ever release it, because `detach` forgets the key
    /// the tap's CURRENT record names. Two rows for one tap, one frozen at whatever
    /// it last knew and ageing forever.
    ///
    /// Reachable without any exotic step: a topic tapped by a seam that could not
    /// attribute it opens a row under `None`, and the next `attach` naming its robot
    /// re-points the tap.
    ///
    /// Hand oracles throughout — the row COUNT and the surviving row's `robot` after
    /// each transition, never a comparison of the code against itself. Three
    /// distinct claims: the old row goes, the new row is FRESH (a baseline learned
    /// from one producer must not judge another), and an UNCHANGED identity is a
    /// NO-OP (a re-attach from the same robot must not throw away what was learned).
    #[test]
    fn monitors_v1_retargeting_a_taps_robot_releases_the_row_its_old_identity_opened() {
        use cerulion_viz::monitor::{MonitorSample, MonitorState, MONITOR_SETTLE_NS};

        let topic = "/lowstate";
        // A payload that classifies `Streaming` and carries a trustworthy rate, so
        // the row can admit EVIDENCE and its freshness is observable.
        let live = || TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: 60_000,
            frames_observed: 100,
            rate_estimate: Some(TopicRateEstimate {
                millihertz: 20_000,
                is_floor: false,
            }),
        };
        let sample = |robot: Option<&str>, at_ns: u64| {
            MonitorSample::new(topic, robot.map(str::to_string), at_ns, Some(live()), true)
        };

        let mut st = DaemonState::default();
        st.stats.insert(topic.to_string(), TopicStat::default());

        // A tap nothing could attribute yet: its rows open under `None`, and past
        // the settle window they start LEARNING.
        let mut t = 0u64;
        st.monitors
            .observe_pass(&[sample(None, t)], WriteStamp::for_test(t));
        t = MONITOR_SETTLE_NS + 1;
        st.monitors
            .observe_pass(&[sample(None, t)], WriteStamp::for_test(t));
        let rows = st.monitors.rows(t);
        assert_eq!(rows.len(), 1, "one tap, one row");
        assert_eq!(rows[0].robot, None, "…and it is the LOCAL row");
        assert!(
            rows[0].samples > 0,
            "precondition: the row has admitted evidence, so a carried-over one \
             would be visible as such: {rows:?}"
        );

        // THE PIN: the identity changes, and the old row goes with it.
        st.retarget_origin_robot(topic, "go2".to_string());
        assert_eq!(
            st.stats[topic].origin_robot.as_deref(),
            Some("go2"),
            "the tap really was re-pointed"
        );
        assert_eq!(
            st.monitors.watched(),
            0,
            "the row its OLD identity opened must be released — `detach` forgets \
             the key the tap's CURRENT record names, so nothing else ever can"
        );

        // The next pass opens exactly ONE row, under the new key, and it is FRESH:
        // a baseline median-ed from another producer's rates, and a settle window
        // opened against another observation, would judge this source by that one.
        t += 1;
        st.monitors
            .observe_pass(&[sample(Some("go2"), t)], WriteStamp::for_test(t));
        let rows = st.monitors.rows(t);
        assert_eq!(rows.len(), 1, "no duplicate: {rows:?}");
        assert_eq!(rows[0].robot.as_deref(), Some("go2"));
        assert_eq!(
            (rows[0].samples, rows[0].state),
            (0, MonitorState::Unknown),
            "the new identity starts inside a NEW settle window, learning nothing \
             from the old producer: {rows:?}"
        );

        // AND AN UNCHANGED IDENTITY IS A NO-OP. `attach` is idempotent, so a
        // re-attach from the SAME robot must not throw away what the row learned —
        // a `forget` on every stamp would reset the baseline on every re-check.
        t += MONITOR_SETTLE_NS + 1;
        st.monitors
            .observe_pass(&[sample(Some("go2"), t)], WriteStamp::for_test(t));
        let learned = st.monitors.rows(t)[0].samples;
        assert!(learned > 0, "precondition: it learned something: {learned}");
        st.retarget_origin_robot(topic, "go2".to_string());
        let rows = st.monitors.rows(t);
        assert_eq!(
            rows.len(),
            1,
            "the row survives an unchanged stamp: {rows:?}"
        );
        assert_eq!(
            rows[0].samples, learned,
            "…with everything it had learned: {rows:?}"
        );
    }

    /// Monitors: a HINTED record TRACKS the live map — set, re-pointed and
    /// CLEARED — while an AUTHORITATIVE one is never revised.
    ///
    /// The distinction is the whole tombstone fix. A hint comes from a NAME-keyed
    /// map, so it describes a name rather than this tap's generation and must stay
    /// falsifiable; an authoritative record is the robot this daemon DEMANDED from,
    /// and it is required to survive netd tearing the mirror down under a
    /// still-held tap.
    ///
    /// The POLICY is driven through the pure `hint_refresh_target` and the WRITE
    /// through `set_origin_robot`, which is every part of the refresher except its
    /// walk over the tap set; the walk is what the e2e arms exercise.
    #[test]
    fn monitors_v1_a_hinted_robot_tracks_the_live_map_while_an_authoritative_one_never_moves() {
        // ── the POLICY, against hand-written vectors ────────────────────────────
        // Unattributed: adopt whatever the map says, including nothing.
        assert_eq!(
            hint_refresh_target(None, false, Some("ubuntu")),
            Some(Some("ubuntu".to_string()))
        );
        assert_eq!(hint_refresh_target(None, false, None), Some(None));
        // Hinted: TRACK the map — re-point, and WITHDRAW when it stops naming the
        // topic. The withdrawal is the case a write-only stamp cannot express, and
        // it is how the historical robot became permanent.
        assert_eq!(
            hint_refresh_target(Some("ubuntu"), true, Some("orin")),
            Some(Some("orin".to_string()))
        );
        assert_eq!(
            hint_refresh_target(Some("ubuntu"), true, None),
            Some(None),
            "a hint the live map has withdrawn must be TAKEN BACK"
        );
        // Authoritative: never revised, in either direction — not re-pointed by a
        // map that disagrees, and not cleared by one that has gone quiet (the attribution fix's
        // torn-down-mirror fallback).
        assert_eq!(hint_refresh_target(Some("go2"), false, Some("spot")), None);
        assert_eq!(hint_refresh_target(Some("go2"), false, None), None);

        // ── the WRITE, including that a withdrawal releases the row ─────────────
        let topic = "/hinted";
        let mut st = DaemonState::default();
        st.stats.insert(topic.to_string(), TopicStat::default());

        st.set_origin_robot(topic, Some("ubuntu".to_string()), true);
        assert_eq!(st.stats[topic].origin_robot.as_deref(), Some("ubuntu"));
        assert!(st.stats[topic].origin_robot_hinted, "…recorded AS a hint");

        // A sampling pass re-confirming the same hint must not disturb the row —
        // this runs 2.5 times a second, so a needless `forget` would reset every
        // baseline continuously.
        let sample = |robot: Option<&str>, at_ns: u64| {
            cerulion_viz::monitor::MonitorSample::new(
                topic,
                robot.map(str::to_string),
                at_ns,
                None,
                true,
            )
        };
        st.monitors
            .observe_pass(&[sample(Some("ubuntu"), 0)], WriteStamp::for_test(0));
        assert_eq!(st.monitors.watched(), 1);
        st.set_origin_robot(topic, Some("ubuntu".to_string()), true);
        assert_eq!(
            st.monitors.watched(),
            1,
            "re-confirming an unchanged hint must leave the row alone"
        );

        // THE WITHDRAWAL: the record clears AND the row it was keyed on goes.
        st.set_origin_robot(topic, None, true);
        assert_eq!(st.stats[topic].origin_robot, None);
        assert!(
            !st.stats[topic].origin_robot_hinted,
            "an absent robot is not a hint — the flag tracks the value it qualifies"
        );
        assert_eq!(
            st.monitors.watched(),
            0,
            "the row the withdrawn identity opened must go with it, or it is served forever with \
             nothing able to release it"
        );
    }

    /// The tap's own observation, in the robot-side liveness vocabulary.
    ///
    /// Hand oracles for every arm, including the two that are easy to get wrong:
    /// an entry that is not observing reports UNKNOWN (never "watched, saw
    /// nothing"), and the FIRST yielding drain is a BASELINE — banked into
    /// `frames_observed`, never dated — because iceoryx2 flushes a publisher's
    /// retained history into a freshly-connected tap and a dead route with history
    /// would otherwise read `Streaming`.
    #[test]
    fn tap_liveness_banks_the_baseline_and_classifies_with_the_shared_thresholds() {
        let base = Instant::now();

        // Not observing → UNKNOWN, whatever else the entry holds.
        let idle_entry = TopicStat {
            route_key: Some("cache".to_string()),
            ..Default::default()
        };
        assert_eq!(idle_entry.liveness(base), None);

        let mut stat = TopicStat::default();
        stat.begin_observation(base);
        // Observing, nothing seen yet → too early to be confident.
        let fresh = stat
            .liveness(base + Duration::from_secs(1))
            .expect("observing");
        assert_eq!(fresh.frames_observed, 0);
        assert_eq!(fresh.last_frame_age_ms, None);
        assert_eq!(fresh.state(), LivenessState::Unknown);
        // Watched long enough having seen NOTHING → the dimmed, dead route.
        assert_eq!(
            stat.liveness(base + Duration::from_secs(30))
                .expect("observing")
                .state(),
            LivenessState::NoData
        );

        // BASELINE batch: banked, NOT dated (it could be a retained-history flush).
        stat.observe_frames(4, base + Duration::from_secs(30));
        let banked = stat
            .liveness(base + Duration::from_secs(30))
            .expect("observing");
        assert_eq!(banked.frames_observed, 4);
        assert_eq!(
            banked.last_frame_age_ms, None,
            "the first drain is a baseline — a flushed backlog must not date the topic"
        );
        assert_eq!(
            banked.state(),
            LivenessState::Idle,
            "produced but undatable is Idle — a topic carrying frames is never NoData"
        );
        // A produced topic NEVER returns to the dimmed row, however long we watch.
        assert_eq!(
            stat.liveness(base + Duration::from_secs(3_600))
                .expect("observing")
                .state(),
            LivenessState::Idle
        );

        // The boundary: the streaming/stale edge, pinned on BOTH sides at the seam
        // vizd reads it through. This is the threshold that decides whether a row
        // keeps its rate or falls through to the liveness verdict, and it is also
        // why a topic with a period ABOVE it spends most of its cycle non-streaming
        // — the fact the sidebar must render muted rather than red. Anchored to the
        // shipped constant, not a literal, so a threshold change moves both sides.
        let boundary = |age_ms: u64| TopicLiveness {
            last_frame_age_ms: Some(age_ms),
            observed_for_ms: 60_000,
            frames_observed: 10,
            rate_estimate: None,
        };
        assert_eq!(
            boundary(LIVENESS_STREAMING_RECENCY_MS).state(),
            LivenessState::Streaming,
            "AT the recency bound a topic is still streaming (inclusive)"
        );
        assert_eq!(
            boundary(LIVENESS_STREAMING_RECENCY_MS + 1).state(),
            LivenessState::Idle,
            "one millisecond past it, the row falls through to the verdict"
        );

        // SECOND drain: an arrival while we were already watching — dated.
        let arrival = base + Duration::from_secs(31);
        stat.observe_frames(2, arrival);
        let dated = stat
            .liveness(arrival + Duration::from_millis(250))
            .expect("observing");
        assert_eq!(dated.frames_observed, 6);
        assert_eq!(dated.last_frame_age_ms, Some(250));
        assert_eq!(dated.state(), LivenessState::Streaming);
        // Then the publisher dies: the age grows and the row goes stale — the
        // reported jpeg topic, which carries frames so it is dated-Idle, never NoData.
        let stale = stat
            .liveness(arrival + Duration::from_secs(120))
            .expect("observing");
        assert_eq!(stale.last_frame_age_ms, Some(120_000));
        assert_eq!(stale.state(), LivenessState::Idle);

        // A zero-frame drain changes nothing (the poll loop can yield an empty batch).
        let before = stat.liveness(arrival).expect("observing");
        stat.observe_frames(0, arrival + Duration::from_secs(1));
        assert_eq!(stat.liveness(arrival).expect("observing"), before);

        // begin_observation is idempotent — a re-attach onto a live tap must not
        // restart the clock (which would re-open the "too early to say" window).
        stat.begin_observation(base + Duration::from_secs(500));
        assert_eq!(
            stat.liveness(base + Duration::from_secs(500))
                .expect("observing")
                .observed_for_ms,
            500_000
        );
    }

    /// A tapped topic's `liveness` carries the desk's own rate — the SAME number
    /// `status` reports as `hz`, in millihertz, so `discover`/`list` say
    /// what `status` says about it; and it carries none while the window is not yet
    /// primed or the stream has stopped (the verdict, never a decayed rate).
    #[test]
    fn tap_liveness_carries_the_desk_rate_status_measures() {
        let mut stat = TopicStat::default();
        let base = Instant::now();
        stat.begin_observation(base);
        observe_seq_gap_free(&mut stat, 100, base);
        let primed = stat.liveness(base).expect("observing");
        assert_eq!(primed.rate_estimate, None, "one sample is no rate");

        let at = base + Duration::from_secs(2);
        observe_seq_gap_free(&mut stat, 200, at);
        let streaming = stat.liveness(at).expect("observing");
        // Synthetic `Instant`s: 100 sequence steps over exactly 2 s, no scheduler jitter.
        let hz = stat.hz(at).expect("hz");
        assert_eq!(hz, 50.0, "100 frames over 2 s");
        assert_eq!(
            streaming.rate_estimate,
            Some(TopicRateEstimate {
                millihertz: 50_000,
                is_floor: false,
            })
        );
        assert_eq!(
            streaming.rate_estimate,
            Some(desk_rate_estimate(hz)),
            "the same quantity `status` reports"
        );

        let stopped = stat
            .liveness(at + HZ_STREAMING_RECENCY + Duration::from_millis(1))
            .expect("observing");
        assert_eq!(
            stopped.rate_estimate, None,
            "a stopped stream claims no rate"
        );
    }

    #[test]
    fn hz_resets_window_on_producer_sequence_reset() {
        // A producer RESTART resets its wire sequence to 0. A wrapping_sub in
        // hz() reads the drop as a ~2³² delta → an absurd Hz.
        let mut stat = TopicStat::default();
        let base = Instant::now();
        // Build a healthy ~30 Hz window at HIGH sequences.
        observe_seq_gap_free(&mut stat, 1000, base);
        observe_seq_gap_free(&mut stat, 1030, base + Duration::from_secs(1));
        let hz = stat.hz(base + Duration::from_secs(1)).expect("hz");
        assert!((hz - 30.0).abs() < 1.0, "≈30 Hz before reset, got {hz}");

        // Producer restarts → sequence drops to 0 (a backward jump ≫ tolerance).
        // The window RESETS (old high seqs dropped), so there is not yet enough
        // data for a rate — NOT a ~2³² wrap-around.
        observe_seq_gap_free(&mut stat, 0, base + Duration::from_secs(2));
        assert_eq!(
            stat.hz(base + Duration::from_secs(2)),
            None,
            "window resets on producer restart (no wrapping_sub blow-up)"
        );

        // Fresh post-reset samples compute a sane rate again.
        observe_seq_gap_free(&mut stat, 30, base + Duration::from_secs(3));
        let hz2 = stat.hz(base + Duration::from_secs(3)).expect("hz");
        assert!((hz2 - 30.0).abs() < 1.0, "≈30 Hz after reset, got {hz2}");
    }

    #[test]
    fn small_backward_jitter_within_tolerance_does_not_reset() {
        // A backward step WITHIN SEQ_RESET_TOLERANCE is benign reorder, not a
        // restart — the window is NOT cleared (control for the reset threshold).
        let mut stat = TopicStat::default();
        let base = Instant::now();
        observe_seq_gap_free(&mut stat, 1000, base);
        observe_seq_gap_free(
            &mut stat,
            1000 - SEQ_RESET_TOLERANCE,
            base + Duration::from_millis(1),
        );
        assert_eq!(
            stat.samples.len(),
            2,
            "in-tolerance reorder keeps the window"
        );
    }

    #[test]
    fn hz_is_wrap_safe_under_within_tolerance_backward_jitter() {
        // The reset guard admits a backward step at or below SEQ_RESET_TOLERANCE
        // (benign reorder), so the window can end with back().seq < front().seq.
        // hz() must compute the ordered delta with `saturating_sub`, NOT
        // `wrapping_sub`: a wrapping delta there wraps to ~2³² → ~4.3 billion Hz.
        let mut stat = TopicStat::default();
        let base = Instant::now();
        // Front at a high seq, then a max-in-tolerance backward step (== 8, which
        // is NOT `> SEQ_RESET_TOLERANCE` → admitted, window not cleared).
        observe_seq_gap_free(&mut stat, 1000, base);
        observe_seq_gap_free(
            &mut stat,
            1000 - SEQ_RESET_TOLERANCE,
            base + Duration::from_secs(1),
        );
        assert_eq!(stat.samples.len(), 2, "in-tolerance jitter kept the window");

        let hz = stat
            .hz(base + Duration::from_secs(1))
            .expect("two samples span 1s");
        // Hand oracle: net forward progress is ≤ 0 (newest 992 ≤ oldest 1000), so
        // the true rate is 0 Hz. saturating_sub(1000) = 0 → 0.0 / 1.0s = 0.0.
        // The `wrapping_sub` mutation yields 992.wrapping_sub(1000) = 4_294_967_288
        // → ~4.29e9 Hz, which BLOWS this bound.
        assert!(
            hz.is_finite() && (0.0..1.0).contains(&hz),
            "in-tolerance backward jitter is ~0 Hz, never a wrap-around blow-up, got {hz}"
        );
    }

    #[test]
    fn hz_stays_sane_when_admitted_jitter_still_nets_forward() {
        // A forward burst then a small in-tolerance backward step that leaves
        // back().seq still ABOVE front().seq: saturating_sub == the true forward
        // delta here (it only clamps the newest-≤-oldest case), so the rate must
        // stay the true forward value — proving the saturating delta does not distort a
        // genuinely-advancing window.
        let mut stat = TopicStat::default();
        let base = Instant::now();
        observe_seq_gap_free(&mut stat, 1000, base);
        observe_seq_gap_free(&mut stat, 1030, base + Duration::from_secs(1));
        // Backward step of 2 (≤ tolerance) → admitted; back()=1028 > front()=1000.
        observe_seq_gap_free(&mut stat, 1028, base + Duration::from_secs(1));
        let hz = stat.hz(base + Duration::from_secs(1)).expect("hz");
        // Hand oracle: (1028 − 1000) / 1.0s = 28 Hz (both `saturating_sub` and a
        // hypothetical un-jittered `wrapping_sub` agree in the net-forward case,
        // so this pins that the clamp is inert when the window nets forward).
        assert!((hz - 28.0).abs() < 1.0, "≈28 Hz net-forward, got {hz}");
    }

    /// Drive `observe_seq` the way the Hz-window oracles want it: they set wire
    /// sequences DIRECTLY (`0` then `30`) to model a RATE, not a poll, so this
    /// reports a `drained` count equal to the advance and every step is gap-free.
    ///
    /// Without it a `0 → 30` step would also assert 29 lost frames, and the
    /// loss oracles below — which own that claim — would stop being the only
    /// thing that can fail on it.
    ///
    /// A free fn inside `mod tests` rather than a `#[cfg(test)]` method on
    /// `TopicStat`, and that is load-bearing: `the_consolidated_default_plan_has_
    /// exactly_one_producer` and `the_logged_entity_recorder_is_fed_from_a_mode_
    /// independent_seam` both take "production code" to be everything BEFORE the
    /// FIRST `#[cfg(test)]` in this file. A gated method up in the impl block
    /// moves that boundary to line ~450 and silently truncates what those guards
    /// scan — they failed with "found 0 call sites" until this moved down here.
    fn observe_seq_gap_free(stat: &mut TopicStat, seq: u32, now: Instant) {
        let drained = frames_missed_between(stat.last_newest_seq, seq, 0).unwrap_or(1);
        stat.observe_seq(seq, drained, now, "test");
    }

    // ---- Pre-decode loss accounting -------------------------------
    //
    // These own the claim that a sequence jump means what it says. Every oracle
    // is hand-written; none compares the implementation against itself.

    #[test]
    fn a_jump_equal_to_the_frames_drained_is_a_healthy_poll_not_a_gap() {
        // THE anti-tautology arm, and the one a naive "seq jumped ⇒ loss"
        // implementation fails: a poll that drains 5 frames advances the newest
        // sequence by 5. Reporting 4 lost frames here would make every
        // multi-frame poll — the normal shape under bursty WiFi arrival — look
        // like loss, which is worse than no counter at all.
        assert_eq!(frames_missed_between(Some(100), 105, 5), Some(0));
        // The single-frame steady state.
        assert_eq!(frames_missed_between(Some(100), 101, 1), Some(0));
    }

    #[test]
    fn the_surplus_over_the_frames_drained_is_the_loss() {
        // 3 frames handed over but the producer advanced 5 ⇒ exactly 2 evicted.
        assert_eq!(frames_missed_between(Some(100), 105, 3), Some(2));
        // A whole poll's worth lost: nothing drained is not reachable here (this
        // runs only on a YIELDING poll), but one frame arriving after a jump of
        // 60 is, and it is the shape a 16-deep queue produces under a stall.
        assert_eq!(frames_missed_between(Some(100), 160, 1), Some(59));
    }

    #[test]
    fn a_first_observation_and_a_regression_make_no_claim() {
        // A tap legitimately attaches MID-STREAM: seq 45185 at attach is not a
        // 45185-frame loss. `None`, never `Some(0)` — the two are different
        // answers and only one of them is correct here.
        assert_eq!(frames_missed_between(None, 45185, 1), None);
        // A producer restart re-bases the counter downward. Claiming anything
        // across that boundary compares two different epochs — the same rule
        // `observe_seq`'s Hz window and `topic echo`'s gap marker already follow.
        assert_eq!(frames_missed_between(Some(45185), 3, 1), None);
        // A `u32` wrap presents as a regression and is treated identically:
        // one skipped claim, never a fabricated ~4-billion-frame loss.
        assert_eq!(frames_missed_between(Some(u32::MAX - 1), 2, 1), None);
    }

    #[test]
    fn a_duplicate_or_stalled_sequence_is_never_reported_as_loss() {
        // `publish_raw` writes the caller's header verbatim, so a producer whose
        // sequence does not advance is a real shape (the rate estimate hit it). It is not
        // loss, and an underflow here would be a spectacular false positive.
        assert_eq!(frames_missed_between(Some(100), 100, 1), Some(0));
        assert_eq!(frames_missed_between(Some(100), 100, 4), Some(0));
    }

    #[test]
    fn the_running_total_accumulates_and_survives_recovery() {
        let mut stat = TopicStat::default();
        let base = Instant::now();
        // Baseline — no claim, nothing counted.
        stat.observe_seq(1000, 1, base, "/t");
        assert_eq!(stat.frames_missed, 0);
        assert_eq!(stat.gap_latch.total_failures(), 0);

        // Advance 5, hand over 3 ⇒ 2 lost.
        stat.observe_seq(1005, 3, base + Duration::from_millis(16), "/t");
        assert_eq!(stat.frames_missed, 2);

        // A clean poll heals the REGIME but must NOT reset the run's total —
        // Principle #3: the number is what the run lost, not what the current
        // regime lost. A reset here is how a bug hides behind a healthy row.
        stat.observe_seq(1006, 1, base + Duration::from_millis(32), "/t");
        assert_eq!(stat.frames_missed, 2, "recovery must not reset the total");

        // A second regime accumulates on top of the first.
        stat.observe_seq(1016, 1, base + Duration::from_millis(48), "/t");
        assert_eq!(stat.frames_missed, 11);
    }

    #[test]
    fn a_healthy_tap_counts_zero_and_stays_silent() {
        // The promise the guard makes to a working desk: 100 clean polls report
        // nothing and count nothing. Without this the "exactly N" oracles above
        // would pass an implementation that also fired on healthy polls.
        let mut stat = TopicStat::default();
        let base = Instant::now();
        for i in 0..100u32 {
            stat.observe_seq(
                1000 + i,
                1,
                base + Duration::from_millis(16 * u64::from(i)),
                "/t",
            );
        }
        assert_eq!(stat.frames_missed, 0);
        assert_eq!(stat.gap_latch.total_failures(), 0);
        assert!(!stat.gap_latch.is_failing());
    }

    #[test]
    fn status_claims_zero_when_checked_and_nothing_when_not() {
        // `Some(0)` and absent are DIFFERENT answers on the wire: one is "this
        // daemon checked and lost nothing", the other is "this daemon does not
        // check" (an earlier vizd). A reader that collapses them re-creates
        // exactly the silence this guard removes.
        let stat = TopicStat::default();
        assert_eq!(Some(stat.frames_missed), Some(0));
        let absent: Option<&TopicStat> = None;
        assert_eq!(absent.map(|s| s.frames_missed), None);
    }

    #[test]
    fn accept_loop_transient_failure_latch_is_loud_first_then_suppressed() {
        // Pin the flood-suppression contract the accept/spawn error
        // paths rely on (through the SAME helpers the loop calls — load-bearing
        // under dead_code=deny): loud first, debug-suppressed repeats with a
        // running count, one recovery on the first success, then re-arm.
        let mut latch = FieldsWarnLatch::new();
        assert_eq!(on_transient_failure(&mut latch), FieldsLogAction::WarnFirst);
        assert_eq!(
            on_transient_failure(&mut latch),
            FieldsLogAction::DebugSuppressed { suppressed: 1 }
        );
        assert_eq!(
            on_transient_failure(&mut latch),
            FieldsLogAction::DebugSuppressed { suppressed: 2 }
        );
        // First success after a suppressed regime → recovery carrying the count.
        assert_eq!(on_transient_recovered(&mut latch), Some(2));
        // Re-armed: the next failure is loud again.
        assert_eq!(on_transient_failure(&mut latch), FieldsLogAction::WarnFirst);
        // A success with no prior suppression re-arms SILENTLY (no doubled volume).
        assert_eq!(on_transient_recovered(&mut latch), None);
    }

    #[test]
    fn archetype_names_are_stable_strings() {
        // Pin the wire strings (a variant rename must not silently change them).
        assert_eq!(archetype_name(ArchetypeKind::Points3D), "Points3D");
        assert_eq!(archetype_name(ArchetypeKind::Scalars), "Scalars");
        assert_eq!(archetype_name(ArchetypeKind::Transforms), "Transforms");
        assert_eq!(archetype_name(ArchetypeKind::AnyValues), "AnyValues");
    }

    #[test]
    fn resolve_pinned_schema_arms_are_honest() {
        // Remote arm: pin the network-FREE half of schema resolution — the
        // decision `resolve_remote_type` makes before any fetch.
        let walker = cerulion_viz::schema_registry::builtin_walker();

        // Known builtin → Ok(Some((name, non-zero hash))) — the network-free fast
        // path (no fetch needed).
        let (name, hash) = resolve_pinned_schema(&walker, "geometry_msgs/Vector3")
            .expect("known type resolves")
            .expect("a known type is Some (no fetch)");
        assert_eq!(name, "geometry_msgs/Vector3");
        assert_ne!(hash, 0, "a resolved schema carries a real wire hash");

        // Unknown QUALIFIED custom type → Ok(None): NOT an error (the
        // caller fetches its closure from the robot), but definitely not resolvable
        // locally.
        assert_eq!(
            resolve_pinned_schema(&walker, "robot_msgs/SecretSauce").expect("qualified is ok"),
            None,
            "an unknown qualified type is a fetch trigger (None), not a local error"
        );

        // A BARE (unqualified) name is a fixable typo — the "name it
        // `pkg/Type`" hint, a HARD error (never a phantom fetch of a bare name).
        let err =
            resolve_pinned_schema(&walker, "PointCloud2").expect_err("a bare name is a hard error");
        assert!(
            err.contains("not fully qualified") && err.contains("pkg/Type"),
            "a bare name gets the qualify-it hint: {err}"
        );
    }

    #[test]
    fn should_drop_client_only_on_connection_level_errors() {
        // The crash-recovery reset: a connection-level error drops the client
        // (reconnect next call); a valid Netd refusal keeps it.
        assert!(should_drop_client_on_error(&ClientError::Io(
            std::io::Error::from(std::io::ErrorKind::BrokenPipe)
        )));
        assert!(should_drop_client_on_error(&ClientError::Protocol(
            "bad banner".to_string()
        )));
        assert!(
            !should_drop_client_on_error(&ClientError::Netd {
                error: "schema conflict".to_string(),
                robot: Some("go2".to_string()),
                topic: Some("/tf".to_string()),
            }),
            "a valid Netd refusal keeps the connection"
        );
    }

    #[test]
    fn perform_remote_demand_release_is_idempotent_self_healing_and_pairs() {
        // The reserve-then-commit demand + release, driven through a spy
        // plane (a HAND oracle, not a self-compare). A demand on a never-seen topic
        // demands ONCE; a repeat/re-attach is an idempotent no-op (does NOT re-demand);
        // a DIFFERENT topic demands its own; a demand FAILURE rolls the claim back so
        // a retry re-demands (self-healing — no phantom claim); a detach RELEASES the
        // right (robot, topic) and clears the claim; a re-demand after release
        // re-demands cleanly; releasing an unheld topic is a no-op.
        let spy = SpyDemandPlane::default();
        let state = Mutex::new(DaemonState::default());

        // First demand of /go2/cloud → exactly one demand call with the right key,
        // reported FRESH (true → the caller waits for the new mirror service).
        assert!(
            perform_remote_demand(&spy, &state, "go2", "/go2/cloud", 0xAA).expect("first demand"),
            "the first demand is fresh"
        );
        assert_eq!(spy.demand_count(), 1);
        assert_eq!(
            spy.demands.lock().unwrap()[0],
            ("go2".to_string(), "/go2/cloud".to_string(), 0xAA)
        );
        // Repeat / re-attach → idempotent, NO second demand, reported NOT fresh.
        assert!(
            !perform_remote_demand(&spy, &state, "go2", "/go2/cloud", 0xAA).expect("repeat ok"),
            "a re-attach is not fresh (the mirror already exists)"
        );
        assert_eq!(spy.demand_count(), 1, "re-attach does not re-demand");
        // A DIFFERENT topic demands its own.
        perform_remote_demand(&spy, &state, "go2", "/go2/tf", 0xBB).expect("other topic");
        assert_eq!(spy.demand_count(), 2);

        // A demand FAILURE rolls the claim back (self-healing).
        spy.set_fail_demand(true);
        let err = perform_remote_demand(&spy, &state, "ubuntu", "/utlidar/odom", 0xCC)
            .expect_err("forced failure");
        assert!(err.contains("forced demand failure"), "{err}");
        assert!(
            !state.lock().unwrap().demanded.contains_key("/utlidar/odom"),
            "a failed demand leaves no phantom claim"
        );
        spy.set_fail_demand(false);
        perform_remote_demand(&spy, &state, "ubuntu", "/utlidar/odom", 0xCC).expect("retry ok");
        assert_eq!(
            spy.demand_count(),
            3,
            "the retry re-demands after the rollback"
        );

        // Detach RELEASES the demand with the right (robot, topic) + clears the claim.
        assert!(
            perform_remote_release(&spy, &state, "/go2/cloud").expect("release ok"),
            "a held topic reports released == true"
        );
        assert_eq!(
            spy.release_keys(),
            vec![("go2".to_string(), "/go2/cloud".to_string())]
        );
        assert!(!state.lock().unwrap().demanded.contains_key("/go2/cloud"));
        // A re-demand after release re-demands cleanly (netd freed the slot).
        perform_remote_demand(&spy, &state, "go2", "/go2/cloud", 0xAA).expect("re-demand");
        assert_eq!(spy.demand_count(), 4, "re-attach after detach re-demands");
        // Releasing an unheld topic is a no-op (false, no release call).
        assert!(
            !perform_remote_release(&spy, &state, "/never/demanded").expect("noop release"),
            "releasing an unheld topic is a no-op"
        );
        assert_eq!(spy.release_keys().len(), 1, "no spurious release call");
    }

    #[test]
    fn perform_remote_demand_refuses_a_different_robot_and_preserves_the_binding() {
        // One data source = one topic ⇒ a topic binds to ONE
        // origin robot. A bare `insert` would OVERWRITE the stored robot on a
        // different-robot re-claim (silently flipping the release target). The entry
        // API must REFUSE the different-robot demand LOUDLY and PRESERVE the original
        // (topic → robot) binding — so a later detach releases the RIGHT robot.
        let spy = SpyDemandPlane::default();
        let state = Mutex::new(DaemonState::default());
        let topic = "/utlidar/robot_odom";

        // A demands (topic, R1) → fresh demand, one call.
        assert!(
            perform_remote_demand(&spy, &state, "R1", topic, 0xAA).expect("R1 fresh"),
            "R1 freshly demands"
        );
        assert_eq!(spy.demand_count(), 1);

        // B demands (topic, R2) → LOUD refusal naming BOTH robots + the topic; NO
        // demand, NO release, NO claim change (a pure refusal that leaks nothing).
        let err = perform_remote_demand(&spy, &state, "R2", topic, 0xBB)
            .expect_err("a different robot for a bound topic is refused");
        assert!(
            err.contains("R1") && err.contains("R2") && err.contains(topic),
            "the refusal names both robots + the topic: {err}"
        );
        assert!(
            err.contains("already mirrored") && err.contains("detach"),
            "the refusal names the cause + the remedy: {err}"
        );
        assert_eq!(spy.demand_count(), 1, "the refusal made NO second demand");
        assert!(
            spy.release_keys().is_empty(),
            "the refusal made NO release (pure refusal — nothing leaked)"
        );

        // The SAME robot re-demand stays the idempotent no-op (existing behavior).
        assert!(
            !perform_remote_demand(&spy, &state, "R1", topic, 0xAA).expect("R1 re-demand"),
            "same-robot re-demand is an idempotent no-op"
        );
        assert_eq!(spy.demand_count(), 1, "no re-demand on the same robot");

        // The original binding survived: a detach releases (R1, topic), NOT R2.
        assert!(
            perform_remote_release(&spy, &state, topic).expect("release"),
            "the bound topic releases"
        );
        assert_eq!(
            spy.release_keys(),
            vec![("R1".to_string(), topic.to_string())],
            "the release target is the ORIGINAL robot R1, never the refused R2"
        );
    }

    #[test]
    fn concurrent_demand_of_same_topic_demands_exactly_once() {
        // THE TOCTOU pin: N controller
        // threads race to attach the SAME remote topic, each on its own handler
        // thread. `perform_remote_demand`
        // CLAIMs `(topic → robot)` as ONE `BTreeMap::insert` under ONE state lock, so
        // EXACTLY ONE thread proceeds to `demand` and the rest skip (idempotent). The
        // spy counts every demand call: exactly ONE regardless of scheduling.
        // Reverting to a non-atomic check-then-insert makes >1 thread demand under
        // the barrier (the double-demand the reserve-then-commit removes).
        use std::sync::Barrier;

        const THREADS: usize = 32;
        const ROUNDS: usize = 50;
        let topic = "/go2/cloud";

        for _ in 0..ROUNDS {
            let state = Arc::new(Mutex::new(DaemonState::default()));
            let spy = Arc::new(SpyDemandPlane::default());
            let barrier = Arc::new(Barrier::new(THREADS));

            let handles: Vec<_> = (0..THREADS)
                .map(|_| {
                    let state = Arc::clone(&state);
                    let spy = Arc::clone(&spy);
                    let barrier = Arc::clone(&barrier);
                    std::thread::spawn(move || {
                        barrier.wait(); // maximize the race window
                        perform_remote_demand(spy.as_ref(), &state, "go2", topic, 0xAA)
                            .expect("demand ok");
                    })
                })
                .collect();
            for h in handles {
                h.join().expect("demand thread panicked");
            }

            assert_eq!(
                spy.demand_count(),
                1,
                "exactly one concurrent attach demands the same topic (a second demand \
                 is the double-crossing the reserve-then-commit removes)"
            );
            assert_eq!(
                state
                    .lock()
                    .unwrap()
                    .demanded
                    .get(topic)
                    .map(String::as_str),
                Some("go2"),
                "the winning claim records the (topic → robot) demand"
            );
        }
    }

    #[test]
    fn split_resolved_maps_none_to_null_pair() {
        assert_eq!(split_resolved(None), (None, None));
        assert_eq!(
            split_resolved(Some(Resolved {
                schema: "geometry_msgs/Vector3".to_string(),
                archetype: ArchetypeKind::Scalars,
            })),
            (
                Some("geometry_msgs/Vector3".to_string()),
                Some("Scalars".to_string())
            )
        );
    }

    // ── Guardrail 5: entity_override stability (hand oracles) ───────

    #[test]
    fn entity_override_conflict_arms_are_honest() {
        // First attach (no existing route) → free (None), whatever the override.
        assert_eq!(entity_override_conflict("/x/cloud", None, "wrist"), None);

        // Same override re-attach → idempotent (None).
        assert_eq!(
            entity_override_conflict("/x/cloud", Some("cloud"), "cloud"),
            None
        );

        // Two DIFFERENT route keys that resolve to the SAME entity → NOT a conflict
        // (the comparison is on the RESOLVED entity, never the raw key). The entity rule
        // decides WHICH keys alias: the entity is `world/<sanitized segments>`
        // and preserves case, so `cloud`/`CLOUD` are genuinely DIFFERENT
        // entities (nothing folds both onto one lidar entity),
        // while spellings differing only in surrounding slashes still alias.
        assert_eq!(
            route_for_input("cloud").entity,
            route_for_input("/cloud/").entity,
            "the fixture keys genuinely resolve to one entity"
        );
        assert_eq!(
            entity_override_conflict("/x/cloud", Some("cloud"), "/cloud/"),
            None,
            "same resolved entity is not a conflict"
        );
        // …and a case-only difference IS now a conflict, because it IS a different
        // entity path (entity segments preserve the user's casing).
        assert!(
            entity_override_conflict("/x/cloud", Some("cloud"), "CLOUD").is_some(),
            "a case-differing entity path is a genuinely different entity"
        );

        // A DIFFERENT resolved entity → REFUSED, naming the topic, the live path,
        // the requested path, and the fix (detach first / reuse the live path).
        let msg = entity_override_conflict("/x/cloud", Some("cloud"), "wrist")
            .expect("a different entity conflicts");
        let live = route_for_input("cloud").entity;
        let requested = route_for_input("wrist").entity;
        assert_ne!(live, requested, "the fixture entities genuinely differ");
        assert!(msg.contains("/x/cloud"), "names the topic: {msg}");
        assert!(msg.contains(&live), "names the LIVE entity path: {msg}");
        assert!(
            msg.contains(&requested),
            "names the REQUESTED entity path: {msg}"
        );
        assert!(
            msg.contains("detach") && msg.contains("reuse"),
            "names the fix: {msg}"
        );
    }

    #[test]
    fn decide_route_commit_arms_are_honest() {
        // Guardrail 5 commit-seam pin (the deterministic replacement for a
        // racy two-thread test — drive the commit seam directly).
        // The decision BOTH attach paths make under their tap lock:

        // Fresh attach (not already), no existing route → STORE the requested route.
        assert!(matches!(
            decide_route_commit("/x/cloud", None, "wrist", false),
            RouteCommit::Store
        ));

        // AlreadyAttached + a SAME-entity re-attach → KEEP (never overwrite the
        // stored route_key — the invariant: the stat reflects the tap's ACTUAL key).
        assert!(matches!(
            decide_route_commit("/x/cloud", Some("cloud"), "cloud", true),
            RouteCommit::Keep
        ));
        // Even a DIFFERENT route key that resolves to the SAME entity is Keep, never
        // an overwrite (`cloud` and `/cloud/` → the same `world/cloud`).
        assert!(matches!(
            decide_route_commit("/x/cloud", Some("cloud"), "/cloud/", true),
            RouteCommit::Keep
        ));

        // AlreadyAttached + a DIFFERENT resolved entity → REFUSE (the concurrent-race
        // close: the racer that reached AlreadyAttached under a diverging key is
        // refused rather than silently overwriting the live mapping).
        assert!(matches!(
            decide_route_commit("/x/cloud", Some("cloud"), "wrist", true),
            RouteCommit::Refuse(_)
        ));

        // Defensive: a first attach can never conflict (no existing route) regardless
        // of the already flag → never Refuse.
        assert!(!matches!(
            decide_route_commit("/x/cloud", None, "wrist", true),
            RouteCommit::Refuse(_)
        ));
    }

    // ── set_blueprint: build_plan (spec → plan) validation oracles ──

    fn mk_view(kind: &str, name: Option<&str>, origin: Option<&str>) -> LayoutNode {
        LayoutNode::View(ViewSpec {
            kind: kind.to_string(),
            name: name.map(str::to_string),
            origin: origin.map(str::to_string),
        })
    }

    fn mk_container(
        kind: &str,
        children: Vec<LayoutNode>,
        shares: Option<Vec<f32>>,
        columns: Option<u32>,
    ) -> LayoutNode {
        LayoutNode::Container(ContainerSpec {
            kind: kind.to_string(),
            children,
            name: None,
            shares,
            columns,
        })
    }

    fn mk_spec(root: LayoutNode) -> LayoutSpec {
        LayoutSpec {
            root,
            auto_views: true,
        }
    }

    #[test]
    fn build_plan_translates_a_nested_layout_to_the_hand_oracle() {
        let spec = mk_spec(mk_container(
            "horizontal",
            vec![
                mk_view("spatial3d", Some("Scene"), Some("world")),
                mk_container(
                    "vertical",
                    vec![
                        mk_view("time_series", None, None),
                        mk_view("text_document", None, None),
                    ],
                    None,
                    None,
                ),
            ],
            Some(vec![3.0, 1.0]),
            None,
        ));
        let plan = build_plan(&spec).expect("valid nested layout");
        let expected = BlueprintPlan {
            auto_views: true,
            decorate: false,
            root: PlanNode::Container(PlanContainer {
                kind: ContainerKind::Horizontal,
                shares: Some(vec![3.0, 1.0]),
                columns: None,
                name: None,
                children: vec![
                    PlanNode::View(PlanView {
                        kind: ViewKind::Spatial3d,
                        name: Some("Scene".to_string()),
                        origin: Some("world".to_string()),
                        contents: None,
                    }),
                    PlanNode::Container(PlanContainer {
                        kind: ContainerKind::Vertical,
                        shares: None,
                        columns: None,
                        name: None,
                        children: vec![
                            PlanNode::View(PlanView {
                                kind: ViewKind::TimeSeries,
                                name: None,
                                origin: None,
                                contents: None,
                            }),
                            PlanNode::View(PlanView {
                                kind: ViewKind::TextDocument,
                                name: None,
                                origin: None,
                                contents: None,
                            }),
                        ],
                    }),
                ],
            }),
        };
        assert_eq!(plan, expected);
        assert_eq!(plan.view_count(), 3);
    }

    #[test]
    fn build_plan_rejects_unknown_view_kind_naming_the_set() {
        let err =
            build_plan(&mk_spec(mk_view("spatial9d", None, None))).expect_err("unknown view kind");
        assert_eq!(err, LayoutError::UnknownViewKind("spatial9d".to_string()));
        let msg = err.to_string();
        assert!(msg.contains("spatial9d"), "names the offender: {msg}");
        assert!(msg.contains("spatial3d"), "lists the supported set: {msg}");
    }

    #[test]
    fn build_plan_rejects_unknown_container_kind() {
        let err = build_plan(&mk_spec(mk_container(
            "stack",
            vec![mk_view("spatial3d", None, None)],
            None,
            None,
        )))
        .expect_err("unknown container kind");
        assert_eq!(err, LayoutError::UnknownContainerKind("stack".to_string()));
        assert!(err.to_string().contains("horizontal"), "lists the set");
    }

    #[test]
    fn build_plan_rejects_empty_container() {
        let err = build_plan(&mk_spec(mk_container("horizontal", vec![], None, None)))
            .expect_err("empty container");
        assert_eq!(err, LayoutError::EmptyContainer("horizontal"));
    }

    #[test]
    fn build_plan_rejects_shares_len_mismatch() {
        let err = build_plan(&mk_spec(mk_container(
            "horizontal",
            vec![
                mk_view("spatial3d", None, None),
                mk_view("time_series", None, None),
            ],
            Some(vec![1.0]),
            None,
        )))
        .expect_err("shares len mismatch");
        assert_eq!(
            err,
            LayoutError::SharesLenMismatch {
                container: "horizontal",
                shares: 1,
                children: 2,
            }
        );
    }

    #[test]
    fn build_plan_rejects_non_positive_or_nonfinite_share() {
        for bad in [0.0_f32, -1.0, f32::NAN, f32::INFINITY] {
            let err = build_plan(&mk_spec(mk_container(
                "vertical",
                vec![
                    mk_view("spatial3d", None, None),
                    mk_view("time_series", None, None),
                ],
                Some(vec![1.0, bad]),
                None,
            )))
            .unwrap_err();
            assert_eq!(
                err,
                LayoutError::NonPositiveShare("vertical"),
                "share {bad} must be rejected"
            );
        }
    }

    #[test]
    fn build_plan_rejects_shares_on_tabs() {
        let err = build_plan(&mk_spec(mk_container(
            "tabs",
            vec![
                mk_view("spatial3d", None, None),
                mk_view("time_series", None, None),
            ],
            Some(vec![1.0, 1.0]),
            None,
        )))
        .expect_err("tabs take no shares");
        assert_eq!(err, LayoutError::SharesOnTabs);
    }

    #[test]
    fn build_plan_rejects_columns_on_non_grid() {
        let err = build_plan(&mk_spec(mk_container(
            "horizontal",
            vec![mk_view("spatial3d", None, None)],
            None,
            Some(2),
        )))
        .expect_err("columns are grid-only");
        assert_eq!(err, LayoutError::ColumnsOnNonGrid("horizontal"));
    }

    #[test]
    fn build_plan_grid_shares_are_per_column_and_bind_to_columns_not_children() {
        // A MULTI-ROW grid (4 children, 2 columns → 2 rows): shares are
        // per-COLUMN, so exactly `columns` (2) entries — NOT one per child. The
        // valid form applies; the per-child form (4 shares) is rejected.
        let four = || {
            vec![
                mk_view("spatial2d", None, None),
                mk_view("time_series", None, None),
                mk_view("text_document", None, None),
                mk_view("spatial3d", None, None),
            ]
        };
        let ok = build_plan(&mk_spec(mk_container(
            "grid",
            four(),
            Some(vec![3.0, 1.0]), // one per COLUMN
            Some(2),
        )))
        .expect("a grid with one share per column is valid");
        match ok.root {
            PlanNode::Container(c) => {
                assert_eq!(c.kind, ContainerKind::Grid);
                assert_eq!(c.columns, Some(2));
                assert_eq!(c.shares, Some(vec![3.0, 1.0]));
            }
            PlanNode::View(_) => panic!("expected a grid container root"),
        }
        // The per-CHILD form (4 shares, 2 columns) — which rerun silently
        // truncates if validation accepts it — is rejected loudly.
        let err = build_plan(&mk_spec(mk_container(
            "grid",
            four(),
            Some(vec![3.0, 1.0, 3.0, 1.0]),
            Some(2),
        )))
        .expect_err("per-child grid shares are rejected");
        assert_eq!(
            err,
            LayoutError::GridSharesLenMismatch {
                shares: 4,
                columns: 2,
            }
        );
    }

    #[test]
    fn build_plan_grid_shares_without_columns_rejected() {
        // A grid's shares are per-column, so the column count must be known.
        let err = build_plan(&mk_spec(mk_container(
            "grid",
            vec![
                mk_view("spatial2d", None, None),
                mk_view("time_series", None, None),
            ],
            Some(vec![1.0, 1.0]),
            None,
        )))
        .expect_err("grid shares require columns");
        assert_eq!(err, LayoutError::GridSharesRequireColumns);
    }

    #[test]
    fn build_plan_rejects_zero_grid_columns() {
        let err = build_plan(&mk_spec(mk_container(
            "grid",
            vec![mk_view("spatial2d", None, None)],
            None,
            Some(0),
        )))
        .expect_err("columns:0 is rejected");
        assert_eq!(err, LayoutError::ZeroGridColumns);
    }

    #[test]
    fn build_plan_rejects_grid_columns_over_child_count() {
        // A huge/over-cap `columns` (> children) would OOM the viewer's per-column
        // resize; reject it (the 4-billion-columns scenario, and the
        // pointless-extra-columns case in one bound).
        let err = build_plan(&mk_spec(mk_container(
            "grid",
            vec![
                mk_view("spatial2d", None, None),
                mk_view("time_series", None, None),
            ],
            None,
            Some(4_000_000_000),
        )))
        .expect_err("columns over the child count is rejected");
        assert_eq!(
            err,
            LayoutError::TooManyGridColumns {
                columns: 4_000_000_000,
                children: 2,
            }
        );
    }

    #[test]
    fn build_plan_propagates_a_nested_error() {
        // A bad kind DEEP in the tree surfaces (validation recurses).
        let err = build_plan(&mk_spec(mk_container(
            "horizontal",
            vec![mk_container(
                "vertical",
                vec![mk_view("bogus_view", None, None)],
                None,
                None,
            )],
            None,
            None,
        )))
        .expect_err("nested bad view kind");
        assert_eq!(err, LayoutError::UnknownViewKind("bogus_view".to_string()));
    }

    // ── Compose_layout intent-shape validation (oracle vectors) ──

    fn mk_intent(
        groups: Option<Vec<crate::protocol::LayoutGroup>>,
        topics: Option<Vec<String>>,
    ) -> LayoutIntent {
        LayoutIntent {
            groups,
            topics,
            strategy: "auto".to_string(),
            attach_missing: true,
            include_robot: true,
        }
    }

    #[test]
    fn parse_intent_groups_bare_topics_is_one_role_less_group() {
        let intent = mk_intent(None, Some(vec!["/a".to_string(), "/b".to_string()]));
        assert_eq!(
            parse_intent_groups(&intent).unwrap(),
            vec![(vec!["/a".to_string(), "/b".to_string()], None, None)]
        );
    }

    #[test]
    fn parse_intent_groups_parses_roles_and_titles() {
        let intent = mk_intent(
            Some(vec![
                crate::protocol::LayoutGroup {
                    topics: vec!["/cloud".to_string()],
                    role: Some("hero".to_string()),
                    title: Some("Scene".to_string()),
                },
                crate::protocol::LayoutGroup {
                    topics: vec!["/vel".to_string()],
                    role: None,
                    title: None,
                },
            ]),
            None,
        );
        assert_eq!(
            parse_intent_groups(&intent).unwrap(),
            vec![
                (
                    vec!["/cloud".to_string()],
                    Some(GroupRole::Hero),
                    Some("Scene".to_string())
                ),
                (vec!["/vel".to_string()], None, None),
            ]
        );
    }

    #[test]
    fn parse_intent_groups_rejects_both_neither_and_empties() {
        // Both set → refuse.
        assert!(parse_intent_groups(&mk_intent(Some(vec![]), Some(vec![])))
            .unwrap_err()
            .contains("not both"));
        // Neither set → refuse.
        assert!(parse_intent_groups(&mk_intent(None, None))
            .unwrap_err()
            .contains("provide 'groups'"));
        // Empty topics → refuse.
        assert!(parse_intent_groups(&mk_intent(None, Some(vec![])))
            .unwrap_err()
            .contains("'topics' is empty"));
        // Empty groups → refuse.
        assert!(parse_intent_groups(&mk_intent(Some(vec![]), None))
            .unwrap_err()
            .contains("'groups' is empty"));
        // A group with no topics → refuse.
        let empty_group = mk_intent(
            Some(vec![crate::protocol::LayoutGroup {
                topics: vec![],
                role: None,
                title: None,
            }]),
            None,
        );
        assert!(parse_intent_groups(&empty_group)
            .unwrap_err()
            .contains("no topics"));
    }

    #[test]
    fn parse_intent_groups_rejects_an_unknown_role_naming_the_set() {
        let bad = mk_intent(
            Some(vec![crate::protocol::LayoutGroup {
                topics: vec!["/x".to_string()],
                role: Some("sidekick".to_string()),
                title: None,
            }]),
            None,
        );
        let err = parse_intent_groups(&bad).unwrap_err();
        assert!(
            err.contains("sidekick") && err.contains("hero"),
            "the error names the bad role + the supported set: {err}"
        );
    }

    #[test]
    fn parse_intent_groups_rejects_a_duplicate_topic_naming_it() {
        // A topic listed twice in a BARE list → loud refusal naming it.
        let bare_dup = mk_intent(
            None,
            Some(vec![
                "/cloud".to_string(),
                "/vel".to_string(),
                "/cloud".to_string(),
            ]),
        );
        let err = parse_intent_groups(&bare_dup).unwrap_err();
        assert!(
            err.contains("/cloud") && err.contains("more than once"),
            "the refusal names the duplicated topic: {err}"
        );

        // A topic repeated ACROSS two groups is also a duplicate → refuse.
        let cross_group_dup = mk_intent(
            Some(vec![
                crate::protocol::LayoutGroup {
                    topics: vec!["/cloud".to_string()],
                    role: Some("hero".to_string()),
                    title: None,
                },
                crate::protocol::LayoutGroup {
                    topics: vec!["/cloud".to_string()],
                    role: Some("plots".to_string()),
                    title: None,
                },
            ]),
            None,
        );
        let err = parse_intent_groups(&cross_group_dup).unwrap_err();
        assert!(
            err.contains("/cloud") && err.contains("at most once"),
            "a cross-group duplicate is rejected: {err}"
        );

        // Anti-tautology: DISTINCT topics across groups parse fine.
        let ok = mk_intent(
            Some(vec![
                crate::protocol::LayoutGroup {
                    topics: vec!["/cloud".to_string()],
                    role: Some("hero".to_string()),
                    title: None,
                },
                crate::protocol::LayoutGroup {
                    topics: vec!["/vel".to_string()],
                    role: Some("plots".to_string()),
                    title: None,
                },
            ]),
            None,
        );
        assert!(
            parse_intent_groups(&ok).is_ok(),
            "distinct topics are accepted"
        );
    }

    #[test]
    fn first_duplicate_topic_oracle() {
        // Pure oracle: first repeat in declaration order, or None.
        assert_eq!(
            first_duplicate_topic(&[(vec!["/a".into(), "/b".into(), "/a".into()], None, None)]),
            Some("/a".to_string())
        );
        assert_eq!(
            first_duplicate_topic(&[
                (vec!["/a".into()], None, None),
                (vec!["/b".into(), "/a".into()], None, None),
            ]),
            Some("/a".to_string()),
            "duplicates are detected ACROSS groups"
        );
        assert_eq!(
            first_duplicate_topic(&[(vec!["/a".into(), "/b".into()], None, None)]),
            None
        );
    }
}

#[cfg(test)]
mod incumbent_fallback_tests {
    use super::report_incumbent_fallback;
    use tracing_test::traced_test;

    /// The LEVEL is the contract: a remote attach that proceeds on an INCUMBENT
    /// definition rather than the one the robot served must say so LOUDLY.
    ///
    /// Without the validate-then-commit rollback this case would fail the
    /// attach outright (the poisoned walker would not know the name), so the
    /// rollback turns a loud failure into a success. `warn!` is what keeps it
    /// from succeeding SILENTLY.
    #[traced_test]
    #[test]
    fn an_incumbent_fallback_is_reported_at_warn_naming_the_robot_and_schema() {
        report_incumbent_fallback(
            "go2",
            "unitree_go/LowState",
            &["unitree_go/LowState".into()],
        );
        logs_assert(|lines: &[&str]| {
            let hits: Vec<&&str> = lines
                .iter()
                .filter(|l| l.contains("ALREADY held is being used instead"))
                .collect();
            if hits.len() != 1 {
                return Err(format!(
                    "expected exactly 1 fallback line, got {}",
                    hits.len()
                ));
            }
            let line = hits[0];
            // The LEVEL token, read as a whole whitespace token out of the header
            // (a bare substring would also match a field value).
            if !line.split_whitespace().any(|t| t == "WARN") {
                return Err(format!("the fallback must be WARN, got: {line}"));
            }
            // The two fields an operator greps by.
            for field in ["robot=go2", "schema=unitree_go/LowState"] {
                if !line.split_whitespace().any(|t| t == field) {
                    return Err(format!("missing field {field} in: {line}"));
                }
            }
            Ok(())
        });
    }

    /// ANTI-TAUTOLOGY: the ordinary path — a served schema that was NOT refused —
    /// says nothing. Without this, the arm above would pass a reporter that fired
    /// on every remote attach.
    #[traced_test]
    #[test]
    fn a_served_schema_that_was_accepted_reports_nothing() {
        report_incumbent_fallback("go2", "unitree_go/LowState", &[]);
        // A DIFFERENT name being refused is also not this schema's fallback.
        report_incumbent_fallback("go2", "unitree_go/LowState", &["other/Type".into()]);
        logs_assert(|lines: &[&str]| {
            let hits = lines
                .iter()
                .filter(|l| l.contains("ALREADY held is being used instead"))
                .count();
            if hits != 0 {
                return Err(format!(
                    "an accepted schema must report nothing, got {hits}"
                ));
            }
            Ok(())
        });
    }
}

#[cfg(test)]
/// The unresolvable-`schema_hash` verdict at the daemon's
/// resolution seam.
///
/// SCOPE, stated exactly. These pin `resolve_from_frame` and
/// `record_resolution` — the two functions the poll thread calls, in the
/// order it calls them — including the NAMED verdicts (version skew vs never
/// compiled), which the local end-to-end arm in `vizd_e2e_test.rs` cannot
/// reach: a name is only ever pinned by the REMOTE attach path, and the poll
/// loop's use of it is the one-line `stat.pinned_schema.clone()` read beside
/// those calls. What the e2e arm covers instead is that the poll loop really
/// does drive them, over real transport, onto the wire.
mod unknown_hash {
    use super::*;
    use cerulion_core::message::ShmMessage;
    use cerulion_core::wire::WireHeader;
    use cerulion_viz::schema_registry::builtin_walker;
    use native_ros2_messages::geometry_msgs::Vector3;
    use tracing_test::traced_test;

    /// A real `geometry_msgs/Vector3` wire frame stamped with a caller-chosen
    /// `schema_hash`. A FOREIGN hash over real Vector3 bytes is exactly the
    /// shape a version skew puts on the wire — the bytes ARE the message and
    /// the hash says otherwise, which is what the 48 corpus hash bumps
    /// produce across a half-deployed pair.
    fn vector3_frame(schema_hash: u64) -> Vec<u8> {
        let payload: Vec<u8> = [1.0f64, 2.0, 3.0]
            .iter()
            .flat_map(|x| x.to_le_bytes())
            .collect();
        let header = WireHeader {
            schema_hash,
            total_size: (WireHeader::SIZE + payload.len()) as u32,
            offset_table_offset: (WireHeader::SIZE + payload.len()) as u32,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns: 1_000_000,
        };
        let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        frame[WireHeader::SIZE..].copy_from_slice(&payload);
        frame
    }

    /// THE headline verdict: a robot publishing a type this desk holds under
    /// a DIFFERENT definition. The desk knows `geometry_msgs/Vector3` by name
    /// (the catalog said so) but hashes it differently from the wire, so the
    /// answer is not "unknown type" — it is "your two builds disagree", with
    /// BOTH hashes, and the remedy is to rebuild the stale side.
    #[test]
    fn a_known_name_at_a_foreign_hash_is_reported_as_a_version_skew() {
        let walker = builtin_walker();
        let local = <Vector3 as ShmMessage>::SCHEMA_HASH;
        let foreign = local ^ 0xFFFF_0000;
        let err = resolve_from_frame(
            &walker,
            &vector3_frame(foreign),
            Some("geometry_msgs/Vector3"),
        )
        .expect_err("a foreign hash cannot resolve");

        let ResolveFailure::UnknownHash(diagnosis) = err else {
            panic!("expected an unknown-hash verdict, got {err:?}");
        };
        let report = undecodable_report(&diagnosis);
        assert_eq!(report.reason, "schema_version_skew");
        assert_eq!(report.schema.as_deref(), Some("geometry_msgs/Vector3"));
        assert_eq!(report.schema_hash, format!("0x{foreign:016X}"));
        assert_eq!(
            report.local_schema_hash,
            Some(format!("0x{local:016X}")),
            "the PAIR is the diagnosis — one hash alone names no stale end"
        );
        assert!(
            report.detail.contains("Rebuild and redeploy"),
            "a skew's remedy is a rebuild: {}",
            report.detail
        );
    }

    /// The other named verdict, whose remedy is the OPPOSITE: a type this
    /// build never compiled. Telling this user to rebuild would send them to
    /// the wrong end of their own problem — rebuilding a type you do not have
    /// cannot help.
    #[test]
    fn a_type_this_build_never_compiled_is_reported_as_not_compiled() {
        let walker = builtin_walker();
        let err = resolve_from_frame(&walker, &vector3_frame(0xDEAD_BEEF), Some("robot_msgs/Odd"))
            .expect_err("an unknown type cannot resolve");
        let ResolveFailure::UnknownHash(diagnosis) = err else {
            panic!("expected an unknown-hash verdict, got {err:?}");
        };
        let report = undecodable_report(&diagnosis);
        assert_eq!(report.reason, "schema_not_compiled");
        assert_eq!(report.schema.as_deref(), Some("robot_msgs/Odd"));
        assert_eq!(
            report.local_schema_hash, None,
            "no local hash exists for a type we do not hold, so none is claimed"
        );
        assert!(
            report.detail.contains("cerulion ros2 attach") && !report.detail.contains("Rebuild"),
            "acquire, never rebuild: {}",
            report.detail
        );
    }

    /// With NO out-of-band name the daemon reports the hash and says so — it
    /// does not invent a type. This is the LOCAL-attach shape (nothing pins a
    /// name for a desk-owned topic).
    #[test]
    fn an_unnamed_topic_reports_the_hash_and_claims_no_schema() {
        let walker = builtin_walker();
        let err = resolve_from_frame(&walker, &vector3_frame(0xDEAD_BEEF), None)
            .expect_err("an unknown hash cannot resolve");
        let ResolveFailure::UnknownHash(diagnosis) = err else {
            panic!("expected an unknown-hash verdict, got {err:?}");
        };
        let report = undecodable_report(&diagnosis);
        assert_eq!(report.reason, "schema_unidentified");
        assert_eq!(report.schema, None);
        assert_eq!(report.local_schema_hash, None);
        assert_eq!(report.schema_hash, "0x00000000DEADBEEF");
    }

    /// The four resolution shapes stay DISTINCT. Collapsed into
    /// one `None`, "silent", "not a frame", "unknown type" and
    /// "corrupt frame" would reach the client as the same blank row.
    #[test]
    fn the_failure_shapes_are_distinguished_not_collapsed() {
        let walker = builtin_walker();
        // Not a frame at all.
        assert_eq!(
            resolve_from_frame(&walker, &[0u8; 4], None).err(),
            Some(ResolveFailure::ShortHeader)
        );
        // A hash we DO hold, over bytes that are not that message.
        let truncated = {
            let mut f = vector3_frame(<Vector3 as ShmMessage>::SCHEMA_HASH);
            f.truncate(WireHeader::SIZE + 4);
            f
        };
        assert_eq!(
            resolve_from_frame(&walker, &truncated, None).err(),
            Some(ResolveFailure::Decode("geometry_msgs/Vector3".to_string())),
            "a FRAMING disagreement on a known type is not a type disagreement"
        );
        // The healthy control, so the failures above are not vacuous.
        let ok = resolve_from_frame(
            &walker,
            &vector3_frame(<Vector3 as ShmMessage>::SCHEMA_HASH),
            None,
        )
        .expect("a well-formed frame of a known type resolves");
        assert_eq!(ok.schema, "geometry_msgs/Vector3");
    }

    /// `record_resolution` is the ONE place the served verdict, the latch and
    /// the log agree. Drives a full regime through it: the verdict appears,
    /// the log is loud once then quiet, a resolution CLEARS the verdict and
    /// reports recovery, and the unconditional total survives.
    #[traced_test]
    #[test]
    fn a_regime_sets_the_served_verdict_then_a_resolution_clears_it() {
        let walker = builtin_walker();
        let local = <Vector3 as ShmMessage>::SCHEMA_HASH;
        let skewed = resolve_from_frame(
            &walker,
            &vector3_frame(local ^ 0xFFFF_0000),
            Some("geometry_msgs/Vector3"),
        );
        let mut stat = TopicStat::default();

        for _ in 0..4 {
            record_resolution(&mut stat, "/go2/marker", &skewed);
        }
        let verdict = stat.undecodable.clone().expect("a verdict is served");
        assert_eq!(verdict.reason, "schema_version_skew");
        assert_eq!(stat.unknown_hash.total_failures(), 4);
        assert!(
            stat.resolved.is_none(),
            "an undecodable topic must not carry a resolved schema"
        );

        // A frame the walker CAN read — the seed-and-swap recovery.
        let healed = resolve_from_frame(&walker, &vector3_frame(local), None);
        record_resolution(&mut stat, "/go2/marker", &healed);
        assert_eq!(
            stat.undecodable, None,
            "the row must stop claiming it cannot decode once it can"
        );
        assert_eq!(
            stat.resolved.as_ref().map(|r| r.schema.as_str()),
            Some("geometry_msgs/Vector3")
        );
        assert_eq!(
            stat.unknown_hash.total_failures(),
            4,
            "recovery reports; it does not erase the history"
        );

        logs_assert(|logs: &[&str]| {
            let loud = logs
                .iter()
                .filter(|l| l.contains("dropping a frame"))
                .count();
            let recovered = logs
                .iter()
                .filter(|l| l.contains("frames on this topic decode again"))
                .count();
            if (loud, recovered) != (1, 1) {
                return Err(format!(
                    "expected 1 loud head + 1 recovery, got ({loud}, {recovered})"
                ));
            }
            if !logs
                .iter()
                .any(|l| l.contains("topic=/go2/marker") && l.contains("dropping a frame"))
            {
                return Err("no undecodable-frame line named the topic".to_string());
            }
            Ok(())
        });
    }

    /// The three NON-type outcomes must not touch the type-identity regime. A
    /// silent topic is not a failure at all, and a framing problem has its own
    /// remedy — folding either in would let one condition swallow the other's
    /// loud head and put a wrong verdict on a Studio row.
    #[traced_test]
    #[test]
    fn a_silent_or_malformed_topic_opens_no_undecodable_regime() {
        let mut stat = TopicStat::default();
        for outcome in [
            Err(ResolveFailure::NoFrame),
            Err(ResolveFailure::ShortHeader),
            Err(ResolveFailure::Decode("geometry_msgs/Vector3".to_string())),
        ] {
            record_resolution(&mut stat, "/go2/quiet", &outcome);
        }
        assert_eq!(stat.undecodable, None);
        assert_eq!(stat.unknown_hash.total_failures(), 0);
        logs_assert(|logs: &[&str]| {
            let noisy: Vec<_> = logs
                .iter()
                .filter(|l| l.contains("dropping a frame"))
                .collect();
            if !noisy.is_empty() {
                return Err(format!("a non-type failure logged a frame drop: {noisy:?}"));
            }
            Ok(())
        });
    }
}

#[cfg(test)]
/// Hand-written oracle vectors for the drain loop's two BOUNDS
/// and its per-topic wake demotion.
///
/// Every expectation is written as a literal computed by hand from the
/// thresholds — never as the function's own output. Both bounds guard failures
/// no arrangement of real topics reaches (a producer that outruns the drain
/// indefinitely; a notifier storm), and both are cheap to get subtly wrong, so
/// each threshold is pinned on BOTH sides.
mod wake_policy_oracle {
    use super::{
        PassWait, WakeDemoter, WakeGovernor, WAKE_BACKLOG_MAX_CONSECUTIVE, WAKE_DEMOTE_STREAK,
        WAKE_IDLE_CHURN_TOLERANCE,
    };

    /// With NO wake source the policy is the deadline timer's, on every input — the
    /// byte-identical-to-the-timer-only-loop claim, stated as a decision.
    #[test]
    fn an_unarmed_loop_always_takes_the_timer() {
        let mut g = WakeGovernor::default();
        for drained in [false, true, true, false] {
            assert_eq!(
                g.decide(drained, false, true),
                PassWait::Timer,
                "no wake source means the timer, whatever the drain did"
            );
        }
    }

    /// The backlog cap is a THRESHOLD, pinned on both sides: exactly
    /// `WAKE_BACKLOG_MAX_CONSECUTIVE` skips are allowed, then a wait is FORCED
    /// even though the drain is still yielding frames.
    ///
    /// The forced wait is what drains the listener's event queue, so this bound
    /// is the notify storm's structural defence: an uncapped skip on a
    /// producer that outruns the loop fills the listener socket forever.
    #[test]
    fn the_backlog_skip_is_capped_and_the_cap_forces_a_wait() {
        let mut g = WakeGovernor::default();
        for i in 0..WAKE_BACKLOG_MAX_CONSECUTIVE {
            assert_eq!(
                g.decide(true, true, false),
                PassWait::Skip,
                "skip {i} of {WAKE_BACKLOG_MAX_CONSECUTIVE} must still be a skip"
            );
        }
        assert_eq!(
            g.decide(true, true, false),
            PassWait::Wake { pace: false },
            "past the cap a still-draining pass must WAIT anyway — that wait is \
             the only thing that drains the listener's event queue"
        );
        // And the counter RESETS, so the next burst gets a fresh allowance
        // rather than one skip forever.
        assert_eq!(g.decide(true, true, false), PassWait::Skip);
    }

    /// A pass that drained NOTHING never skips, whatever the skip budget says.
    #[test]
    fn an_empty_pass_never_skips_its_wait() {
        let mut g = WakeGovernor::default();
        assert_eq!(g.decide(false, true, false), PassWait::Wake { pace: false });
        assert_eq!(g.decide(false, true, false), PassWait::Wake { pace: false });
    }

    /// The churn tolerance is a THRESHOLD too: the loop paces on the pass that
    /// REACHES `WAKE_IDLE_CHURN_TOLERANCE` fired-but-empty waits, not before.
    #[test]
    fn pacing_starts_exactly_at_the_churn_tolerance() {
        let mut g = WakeGovernor::default();
        // Churn is counted on THIS pass and read on the SAME pass, so the
        // tolerance-th fired-but-empty pass is the first paced one.
        for i in 1..WAKE_IDLE_CHURN_TOLERANCE {
            assert_eq!(
                g.decide(false, true, true),
                PassWait::Wake { pace: false },
                "churn {i} is under the tolerance and must not pace"
            );
        }
        assert_eq!(
            g.decide(false, true, true),
            PassWait::Wake { pace: true },
            "at the tolerance the loop must pace — a wake that fires and \
             delivers nothing spends no budget, which is the spin this guards"
        );
    }

    /// One productive pass CLEARS the churn — a healthy topic never paces, and a
    /// topic that recovers stops paying for its bad patch immediately.
    #[test]
    fn a_single_delivered_frame_clears_the_churn() {
        let mut g = WakeGovernor::default();
        for _ in 0..WAKE_IDLE_CHURN_TOLERANCE * 2 {
            g.decide(false, true, true);
        }
        assert_eq!(
            g.decide(false, true, true),
            PassWait::Wake { pace: true },
            "precondition: the loop is pacing"
        );
        // A pass that drained frames: it skips (backlog arm) AND clears churn.
        assert_eq!(g.decide(true, true, true), PassWait::Skip);
        assert_eq!(
            g.decide(false, true, true),
            PassWait::Wake { pace: false },
            "one delivered frame must clear the churn outright"
        );
    }

    /// A wait that did NOT fire is not churn: a plain timeout on a quiet topic
    /// already spent its budget, so counting it would pace a healthy idle
    /// daemon for no reason.
    #[test]
    fn a_timed_out_wait_is_not_churn() {
        let mut g = WakeGovernor::default();
        for _ in 0..WAKE_IDLE_CHURN_TOLERANCE * 3 {
            assert_eq!(
                g.decide(false, true, false),
                PassWait::Wake { pace: false },
                "a wait that timed out spent its budget — never pace on it"
            );
        }
    }

    /// The demotion streak is a THRESHOLD, pinned on both sides.
    ///
    /// It pins the THRESHOLD only, not the per-topic property:
    /// a single-topic drive cannot observe that, since `settle`'s returned
    /// vector holds `/noisy` alone because nothing else was ever in a fired set,
    /// whatever the implementation does. The sibling
    /// `a_storming_topic_is_demoted_while_its_healthy_sibling_keeps_its_wake`
    /// owns that half.
    #[test]
    fn the_demotion_streak_is_a_threshold_pinned_on_both_sides() {
        let mut d = WakeDemoter::default();
        // `/noisy` fires every wait and never delivers.
        for i in 1..WAKE_DEMOTE_STREAK {
            d.note_fired(["/noisy".to_string()]);
            assert!(
                d.settle(&[]).is_empty(),
                "streak {i} of {WAKE_DEMOTE_STREAK} must not demote yet"
            );
        }
        d.note_fired(["/noisy".to_string()]);
        assert_eq!(
            d.settle(&[]),
            vec!["/noisy".to_string()],
            "at the streak the topic is demoted"
        );
        // The demotion SPENT its evidence: the streak restarts from zero.
        d.note_fired(["/noisy".to_string()]);
        assert!(
            d.settle(&[]).is_empty(),
            "a demotion spends its evidence — the next fire starts a fresh streak"
        );
    }

    /// **The streak is PER TOPIC — pinned against a SIBLING, which is the only
    /// shape that can see it.**
    ///
    /// `the_demotion_streak_is_per_topic_and_pinned_on_both_sides` drives ONE
    /// topic, and under a single-topic drive a per-topic map and one global
    /// counter are indistinguishable: its `assert_eq!(settle(&[]),
    /// vec!["/noisy"])` — "and ONLY that topic" — is satisfied by CONSTRUCTION,
    /// because nothing else was ever in a fired set. A variant
    /// PROVES that: a `WakeDemoter::settle` rewritten to keep ONE shared streak
    /// across all wake sources survives the rest of the vizd suite, all three
    /// pure oracles and both e2e storm arms included.
    ///
    /// The property is load-bearing rather than tidy. The wake path puts
    /// producers we do NOT own behind these bounds, so a shared streak breaks
    /// both ways: a healthy sibling's delivery clears the counter, so a STORMING
    /// topic is never demoted and keeps billing a foreign graph publisher a
    /// notify per ring (measured under that variant: 0 demotions across 3x the
    /// streak); or a quiet stretch crosses the threshold for both and an
    /// INNOCENT topic silently loses its wake and reverts to the ~16 ms poll
    /// floor — the exact regression the per-topic streak exists to prevent.
    ///
    /// So this drives two topics whose fates must diverge: `/noisy` fires every
    /// wait and never delivers; `/live` fires every wait and delivers every
    /// time. The shared-streak rewrite fails here with
    /// `/noisy` never demoted, while every other demoter arm stays green.
    #[test]
    fn a_storming_topic_is_demoted_while_its_healthy_sibling_keeps_its_wake() {
        let mut d = WakeDemoter::default();
        let noisy = "/noisy".to_string();
        let live = "/live".to_string();

        let mut noisy_demotions = 0_u32;
        // Three full streaks' worth of waits: long enough that a SHARED counter
        // reset by `/live`'s delivery can never reach the threshold, and long
        // enough that `/live` would be swept up by one if it ever did.
        for _ in 0..(WAKE_DEMOTE_STREAK * 3) {
            d.note_fired([noisy.clone(), live.clone()]);
            // `/live` delivered on this wait; `/noisy` did not.
            let demoted = d.settle(std::slice::from_ref(&live));
            assert!(
                !demoted.contains(&live),
                "a topic that DELIVERS on every wait must never be demoted — it is \
                 paying for its wake"
            );
            noisy_demotions += demoted.len() as u32;
        }

        assert!(
            noisy_demotions >= 1,
            "the STORMING topic must be demoted despite a healthy sibling clearing \
             its own streak on every single wait — the streak is per topic, so the \
             sibling's success cannot un-bill the storm"
        );
    }

    /// A fire that DELIVERED resets the streak, so a live topic can never be
    /// demoted however long it runs.
    #[test]
    fn a_delivering_topic_is_never_demoted() {
        let mut d = WakeDemoter::default();
        for _ in 0..WAKE_DEMOTE_STREAK * 4 {
            d.note_fired(["/live".to_string()]);
            // Two empty settles, then one that delivered — a coalescing pattern
            // no threshold should mistake for a dead wake.
            assert!(d.settle(&[]).is_empty());
            d.note_fired(["/live".to_string()]);
            assert!(d.settle(&[]).is_empty());
            d.note_fired(["/live".to_string()]);
            assert!(d.settle(&["/live".to_string()]).is_empty());
        }
    }

    /// A topic that never FIRED is never counted against — the streak is about
    /// wakes that rang, not about topics that were quiet.
    #[test]
    fn a_topic_that_never_fires_is_never_demoted() {
        let mut d = WakeDemoter::default();
        for _ in 0..WAKE_DEMOTE_STREAK * 2 {
            d.note_fired(["/other".to_string()]);
            let out = d.settle(&["/other".to_string()]);
            assert!(out.is_empty());
        }
        // `/silent` was never in a fired set, so it can never appear here.
        assert!(d.settle(&[]).is_empty());
    }
}

/// Monitors: the ATTACHED-plane sample builder.
///
/// [`monitor_snapshot`] is where the daemon's own tap statistics become the
/// engine's input vocabulary, so it is where the attached plane can silently lie —
/// by keying a row on the wrong robot, by labelling the desk's rate a FLOOR, or by
/// suppressing every condition with a `discovery_converged: false` that has no
/// meaning on a first-hand observation. Each of those is a one-token slip that
/// changes nothing about whether the daemon compiles, runs, or serves a plausible
/// answer, and none is reachable from the pure engine's own suite.
///
/// Driven over hand-built [`TopicStat`]s rather than real transport: the function
/// takes the pieces precisely so it can be, and a real tap would make the rate a
/// measurement of the RUNNER rather than an oracle.
#[cfg(test)]
mod monitor_snapshot_tests {
    use super::*;

    /// A tap that has drained `frames` frames, the newest at `newest_seq`, with
    /// its observation open since `since`.
    fn tap(since: Instant, robot: Option<&str>) -> TopicStat {
        let mut stat = TopicStat {
            origin_robot: robot.map(str::to_string),
            ..TopicStat::default()
        };
        stat.begin_observation(since);
        stat
    }

    /// Drive a tap's Hz window by hand: `count` frames arriving one per `period`,
    /// ending at `end`. The wire sequence advances by one per frame, so the rate
    /// the window reports is `1 / period` — a HAND-COMPUTED oracle, not whatever
    /// the machine managed.
    fn drive(stat: &mut TopicStat, count: u32, period: Duration, end: Instant) {
        for i in 0..count {
            let at = end - period * (count - 1 - i);
            stat.observe_frames(1, at);
            stat.observe_seq(i, 1, at, "/fixture");
        }
    }

    /// A steady 20 Hz tap becomes a sample carrying the desk's own rate, labelled
    /// as a MEASUREMENT.
    ///
    /// The two claims that matter are asserted separately, because a slip in
    /// either is invisible to the other: the millihertz value (hand-computed from
    /// the frame spacing the fixture drove) and `is_floor == false` (which is what
    /// lets the engine learn a baseline at all; the rate estimate's floor label makes a row
    /// permanently ineligible, so mislabelling here would leave every attached row
    /// judged on nothing while every test about VALUES still passed).
    #[test]
    fn a_steady_tap_becomes_a_sample_carrying_the_desks_own_rate_as_a_measurement() {
        let now = Instant::now();
        let mut stat = tap(now - Duration::from_secs(30), None);
        // 5 frames, 50 ms apart, the newest arriving NOW: 4 sequence steps over
        // 200 ms => 20 Hz => 20_000 mHz.
        drive(&mut stat, 5, Duration::from_millis(50), now);
        let stats = BTreeMap::from([("/imu".to_string(), stat)]);

        let samples = monitor_snapshot(
            vec!["/imu".to_string()],
            &stats,
            now,
            WriteStamp::for_test(7_000),
        );
        assert_eq!(samples.len(), 1);
        let s = &samples[0];
        assert_eq!(s.topic, "/imu");
        assert_eq!(s.observed_at_ns, 7_000, "dated by the CALLER's one instant");
        let rate = s.rate.expect("a driven tap has a rate");
        assert_eq!(rate.millihertz, 20_000, "4 steps over 200 ms");
        assert!(
            !rate.is_floor,
            "the desk's Hz reads the publisher's own commit SEQUENCE, so it is a \
             measurement — a floor label would make the row permanently ineligible"
        );
    }

    /// The row is keyed on the TAP's own origin, and a LOCAL producer is a
    /// DISTINCT row from a remote topic of the same name.
    ///
    /// This is the identity the engine's whole per-row state hangs off: merged
    /// under the topic name alone, two robots' samples interleave into one
    /// baseline and one alert history. Asserted with all three shapes present at
    /// once so a builder that dropped the robot entirely (the easy slip — the
    /// field is `Option<String>` and `None` compiles) collapses them into one.
    #[test]
    fn each_tap_is_keyed_on_its_own_origin_robot_and_a_local_producer_is_its_own_row() {
        let now = Instant::now();
        let stats = BTreeMap::from([
            ("/a".to_string(), tap(now, Some("go2"))),
            ("/b".to_string(), tap(now, Some("spot"))),
            ("/c".to_string(), tap(now, None)),
        ]);
        let samples = monitor_snapshot(
            vec!["/a".to_string(), "/b".to_string(), "/c".to_string()],
            &stats,
            now,
            WriteStamp::for_test(0),
        );
        let keyed: Vec<(&str, Option<&str>)> = samples
            .iter()
            .map(|s| (s.topic.as_str(), s.robot.as_deref()))
            .collect();
        assert_eq!(
            keyed,
            vec![("/a", Some("go2")), ("/b", Some("spot")), ("/c", None)]
        );
    }

    /// The attached plane always reports discovery CONVERGED, and a tap with no
    /// rate yet carries NO rate rather than a fabricated zero.
    ///
    /// Both halves are absolute rules the engine enforces but cannot check the
    /// INPUT of. `discovery_converged: false` makes every sample non-evidential,
    /// so a slip there ships the whole feature inert while every pure test stays
    /// green — and there is nothing here to distrust: the evidence is this
    /// daemon's own tap, not a catalog answer. A fabricated `0 mHz`, meanwhile,
    /// would read as a topic running at zero and score against its own baseline.
    #[test]
    fn the_attached_plane_claims_converged_discovery_and_never_fabricates_a_rate() {
        let now = Instant::now();
        // Attached, observing, but not a single frame drained yet — `hz` needs two
        // samples spanning positive time.
        let stats = BTreeMap::from([("/quiet".to_string(), tap(now, None))]);
        let samples = monitor_snapshot(
            vec!["/quiet".to_string()],
            &stats,
            now,
            WriteStamp::for_test(0),
        );
        assert_eq!(samples.len(), 1);
        assert!(
            samples[0].discovery_converged,
            "a first-hand tap observation has no discovery answer to distrust"
        );
        assert_eq!(
            samples[0].rate, None,
            "no rate yet means NO rate — never a fabricated 0 mHz"
        );
        // The liveness payload IS present: the tap is observing, it has simply
        // seen nothing. That is what lets the row classify at all.
        assert!(samples[0].liveness.is_some());
    }

    /// An attached topic with no stats entry is SKIPPED rather than minted as a
    /// phantom LOCAL row.
    ///
    /// Unreachable in production (every attach seam opens the entry), and the
    /// alternative is worse than a gap: a row with neither an observation nor an
    /// origin would key as LOCAL, and a later sample carrying the real robot could
    /// not join it — it would open a SECOND row and the phantom would sit at
    /// UNKNOWN forever. The anti-tautology half is in the same body: the sibling
    /// that DOES have an entry is still sampled.
    #[test]
    fn an_attached_topic_with_no_stats_entry_is_skipped_not_minted_as_a_phantom_row() {
        let now = Instant::now();
        let stats = BTreeMap::from([("/real".to_string(), tap(now, Some("go2")))]);
        let samples = monitor_snapshot(
            vec!["/ghost".to_string(), "/real".to_string()],
            &stats,
            now,
            WriteStamp::for_test(0),
        );
        assert_eq!(samples.len(), 1, "the ghost minted no row: {samples:?}");
        assert_eq!(samples[0].topic, "/real");
    }

    // ── the CATALOG sample builder ────────────────────────────────────

    /// A catalog reply carrying `entries`, each with the liveness handed in.
    fn catalog(robot: &str, entries: &[(&str, Option<TopicLiveness>)]) -> CatalogReply {
        CatalogReply {
            version: cerulion_core::transport::cerulion_q::CATALOG_WIRE_VERSION,
            robot: robot.to_string(),
            entries: entries
                .iter()
                .map(|(topic, liveness)| CatalogEntry {
                    topic: (*topic).to_string(),
                    schema_hash: None,
                    schema_name: None,
                    provenance: cerulion_core::transport::cerulion_q::CatalogProvenance::Runtime,
                    producer_count: Some(1),
                    liveness: *liveness,
                })
                .collect(),
            error: None,
        }
    }

    /// The robot's own observation of a streaming topic, rate included: the robot-side
    /// estimate that makes a rate for an UNATTACHED topic possible at all.
    fn robot_streaming(mhz: u64, is_floor: bool) -> TopicLiveness {
        TopicLiveness {
            last_frame_age_ms: Some(0),
            observed_for_ms: 60_000,
            frames_observed: 500,
            rate_estimate: Some(TopicRateEstimate {
                millihertz: mhz,
                is_floor,
            }),
        }
    }

    /// The required property: a catalog row nobody attached becomes a monitor sample
    /// carrying the ROBOT's own liveness and its rate.
    ///
    /// The rate is the payload's own — there is no desk tap to measure, which is the
    /// entire point of this plane — so a builder that dropped it would leave every
    /// unattached row unable to learn a baseline and therefore permanently
    /// `Learning`, with no alert oracle able to see it.
    #[test]
    fn a_catalog_row_becomes_a_sample_carrying_the_robots_own_liveness_and_rate() {
        let catalogs = [catalog(
            "go2",
            &[
                ("/lowstate", Some(robot_streaming(500_000, false))),
                ("/uslam/cloud_map", None),
            ],
        )];
        let samples = catalog_monitor_samples(
            &catalogs,
            &BTreeSet::new(),
            true,
            WriteStamp::for_test(7_000),
        );

        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0].topic, "/lowstate");
        assert_eq!(
            samples[0].robot.as_deref(),
            Some("go2"),
            "the row is keyed on the robot the catalog self-attributed to"
        );
        assert_eq!(
            samples[0].rate.map(|r| (r.millihertz, r.is_floor)),
            Some((500_000, false)),
            "the ROBOT's rate rides through — nothing else can measure this topic"
        );
        assert_eq!(
            samples[0].observed_at_ns, 7_000,
            "every sample of a pass is dated from the pass's ONE latched stamp"
        );

        // A row the robot could not observe carries NO liveness and NO rate. That
        // is one of the four causes of an absent payload (an earlier robot,
        // `CERULION_TOPIC_LIVENESS=off`, an exhausted tap budget, a WAN-served
        // catalog), and the engine treats all four as UNKNOWN — never as trouble.
        assert_eq!(samples[1].topic, "/uslam/cloud_map");
        assert_eq!(samples[1].liveness, None);
        assert_eq!(samples[1].rate, None, "absent is not zero");
    }

    /// A row an ATTACHED tap owns is SKIPPED, and the skip is keyed on
    /// `(robot, topic)` — not on the topic name.
    ///
    /// The keying is the sharp half. One desk watches many robots, so `/lowstate`
    /// attached from go2 and `/lowstate` in SPOT's catalog are different data
    /// sources with different baselines; skipping by NAME would leave spot's row
    /// permanently unfed while go2's tap fed a row spot never appears in. And a
    /// GENUINE LOCAL tap (`origin_robot: None`) can never collide with a catalog
    /// row at all, so it must not suppress one either.
    #[test]
    fn the_catalog_builder_skips_only_the_exact_row_an_attached_tap_owns() {
        let catalogs = [
            catalog(
                "go2",
                &[("/lowstate", Some(robot_streaming(500_000, false)))],
            ),
            catalog(
                "spot",
                &[("/lowstate", Some(robot_streaming(20_000, false)))],
            ),
        ];
        let attached = BTreeSet::from([("go2".to_string(), "/lowstate".to_string())]);
        let samples = catalog_monitor_samples(&catalogs, &attached, true, WriteStamp::for_test(0));

        let keyed: Vec<(&str, Option<&str>)> = samples
            .iter()
            .map(|s| (s.topic.as_str(), s.robot.as_deref()))
            .collect();
        assert_eq!(
            keyed,
            vec![("/lowstate", Some("spot"))],
            "go2's row is fed by its tap; spot's is a DIFFERENT data source and \
             must still be watched: {samples:?}"
        );
    }

    /// The attached-key set is exactly the keys the ATTACHED sampler mints with a
    /// robot — same tap list, same per-tap `origin_robot`.
    ///
    /// An unattributed tap yields NO key, and that is not an omission: a
    /// `robot: None` row is a genuine LOCAL producer, which no catalog can describe
    /// (a `CatalogReply` self-attributes), so admitting one would let a local tap
    /// on `/imu` suppress a robot's `/imu` catalog row — a topic silently unwatched
    /// because something of the same name happens to be attached here.
    #[test]
    fn the_attached_key_set_is_exactly_the_attributed_taps() {
        let now = Instant::now();
        let stats = BTreeMap::from([
            ("/a".to_string(), tap(now, Some("go2"))),
            ("/b".to_string(), tap(now, None)),
            ("/gone".to_string(), tap(now, Some("spot"))),
        ]);
        // `/gone` has a stats entry but is NOT in the tap list, and `/ghost` is in
        // the list with no entry — neither can be an attached row.
        let keys = attached_row_keys(
            &["/a".to_string(), "/b".to_string(), "/ghost".to_string()],
            &stats,
        );
        assert_eq!(
            keys,
            BTreeSet::from([("go2".to_string(), "/a".to_string())]),
            "only an ATTRIBUTED, currently-tapped row can own a catalog key: {keys:?}"
        );
    }

    /// The DISCOVERY marker rides onto every sample, and it is the GATHER's — not a
    /// hardcoded `true`.
    ///
    /// The attached plane passes `true` because its evidence is first-hand and there
    /// is no discovery answer to distrust. Here there is: an unconverged catalog is
    /// short or empty for reasons that have nothing to do with any topic,
    /// so a row built from one must raise nothing. The failure mode of
    /// hardcoding `true` is the worst kind — every cold-start `discover`, before
    /// netd has converged, would feed rows whose evidence is an artefact of the
    /// gather and start confirming `silent` on them.
    #[test]
    fn the_discovery_marker_rides_the_gather_and_is_not_hardcoded_true() {
        let catalogs = [catalog(
            "go2",
            &[("/lowstate", Some(robot_streaming(500_000, false)))],
        )];
        let settled =
            catalog_monitor_samples(&catalogs, &BTreeSet::new(), true, WriteStamp::for_test(0));
        let unconverged =
            catalog_monitor_samples(&catalogs, &BTreeSet::new(), false, WriteStamp::for_test(0));

        assert!(settled[0].discovery_converged);
        assert!(!unconverged[0].discovery_converged);
        // …and the marker really is what makes the sample non-evidential, so the
        // engine's rule-2 gate is reachable from this plane. Same payload, same
        // classification available, opposite verdicts.
        assert!(settled[0].is_evidential().is_some());
        assert_eq!(
            unconverged[0].is_evidential(),
            None,
            "an unconverged catalog proves nothing about any topic"
        );
    }

    /// A tap whose stream STOPPED carries no rate and a liveness the substrate
    /// classifies as stale — which is the whole stimulus the `stalled` condition
    /// rests on, produced here by the production code path rather than by hand.
    ///
    /// `hz` refuses a rate past the recency window (a decaying rate reads
    /// as a slow stream rather than a dead one), so this pins that the snapshot
    /// forwards the refusal instead of reaching for the last known number.
    #[test]
    fn a_stopped_tap_carries_no_rate_and_a_stale_liveness() {
        let now = Instant::now();
        let stopped = now - (HZ_STREAMING_RECENCY + Duration::from_secs(1));
        let mut stat = tap(now - Duration::from_secs(60), None);
        drive(&mut stat, 5, Duration::from_millis(50), stopped);
        let stats = BTreeMap::from([("/dead".to_string(), stat)]);

        let samples = monitor_snapshot(
            vec!["/dead".to_string()],
            &stats,
            now,
            WriteStamp::for_test(0),
        );
        assert_eq!(samples[0].rate, None, "past the recency window: no rate");
        let liveness = samples[0].liveness.expect("still observing");
        assert_eq!(
            liveness.state(),
            cerulion_core::LivenessState::Idle,
            "the substrate classifies the stop; the monitor never re-thresholds it"
        );
        // Anti-tautology: the SAME tap read at the moment it stopped does carry a
        // rate, so the `None` above is the recency gate and not a broken fixture.
        let live = monitor_snapshot(
            vec!["/dead".to_string()],
            &stats,
            stopped,
            WriteStamp::for_test(0),
        );
        assert_eq!(
            live[0].rate.map(|r| r.millihertz),
            Some(20_000),
            "the fixture really did drive a measurable rate"
        );
    }
}
