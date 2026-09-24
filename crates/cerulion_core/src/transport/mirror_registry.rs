// SPDX-License-Identifier: AGPL-3.0-only
//! The MIRROR-PROVENANCE registry `/__cerulion/mirrors`.
//!
//! # Why this exists (the design decision)
//!
//! *One data source = one topic.* A remote robot's topic must NEVER present as a
//! second LOCAL topic just because the desk mirrors it into local SHM. When a
//! desk process re-injects a remote topic (vizd's `attach_remote`, `topic echo`'s
//! auto-ingress, connectd's iroh re-inject engine), the mirror lands as a
//! plain canonical-named iceoryx2 `{topic}/data` service — indistinguishable from
//! a genuinely-local topic by `cerulion topic list`, which enumerates `*/data`
//! services. This registry is the OUT-OF-BAND provenance channel that lets
//! `topic list` (and Studio) fold each mirrored topic back into the REMOTE
//! section, attributed to its origin robot, with a streaming marker — WITHOUT
//! touching the wire name (the frame stays byte-identical, Principle #7;
//! a consumer subscribing to `/utlidar/robot_odom` still finds the mirror at that
//! exact name — "remote = local" for zero-copy transparency).
//!
//! # The control service
//!
//! One iceoryx2 pub/sub service at the fixed name
//! [`MIRROR_REGISTRY_SERVICE_NAME`] (`/__cerulion/mirrors`, under
//! [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX)). It is a
//! FRAMEWORK control service, NOT a graph topic, so — exactly like
//! [`reg_channel`](super::reg_channel) — it deliberately does NOT ride the
//! graph-topic conventions:
//!
//! - provisioned for MANY writer processes ([`MIRROR_REGISTRY_MAX_WRITERS`]) —
//!   every desk re-injector registers its own provenance — plus reader headroom
//!   ([`MIRROR_REGISTRY_MAX_READERS`]) for concurrent `topic list` / Studio reads;
//! - a bounded record size ([`MAX_MIRROR_RECORD_LEN`]);
//! - a companion EVENT service — the doorbell, see below. (A reader
//!   gathers a one-shot snapshot rather than
//!   subscribing long-term, so a listener/notifier looks like dead weight. It
//!   is not: without it a gather can only CATCH a periodic broadcast, and
//!   catching it is a bet on the writer's timer — see "The doorbell".)
//! - NO `/data` suffix on the service name, so it is invisible to
//!   `cerulion topic list` (which enumerates `*/data` services) — pinned by
//!   `tests::mirror_registry_service_name_has_no_data_suffix`.
//!
//! # Late-join, per-writer identity, and expiry
//!
//! A `topic list` reader is a SHORT-LIVED process that opens, gathers a snapshot,
//! renders, and exits — it must see the CURRENT set of live mirrors regardless of
//! when it started relative to the mirror processes, AND regardless of the phase
//! offset between INDEPENDENT writer processes. The same bring-up-race +
//! quiet-publisher trap [`reg_channel`](super::reg_channel) documents applies
//! (iceoryx2 native late-joiner history is delivered only inside a `send`-driven
//! `update_connections`, so a fire-and-forget publisher that went quiet would
//! never deliver to a later reader). The solution: each writer keeps its
//! authoritative provenance set in a process-local map and RE-PUBLISHES the whole
//! set periodically ([`MIRROR_REPUBLISH_INTERVAL`]) from a background thread, and
//! every record carries a per-writer IDENTITY (`writer_id`, minted once per
//! writer instance). A reader gather (`gather_from_node`) unions the records AND
//! tracks the distinct `writer_id`s it has heard from, comparing that against the
//! LIVE publisher count re-read each pass; it stops EARLY only once it has heard
//! from every currently-live writer. This is what closes the
//! independent-writer race: a set-stability heuristic (2 quiet passes) would
//! early-exit after hearing ONE writer if a second writer's republish lands on a
//! different 150 ms phase, silently returning an incomplete snapshot and rendering
//! a live mirror as a phantom LOCAL topic.
//!
//! EXPIRY falls out for free: a writer that EXITS (drops its `MirrorRegistry`)
//! stops republishing and its publisher port is released, so its records are never
//! re-sent — a subsequent reader gather does not see them. The empty-desk case is
//! instant (no publishers ⇒ return immediately). A CRASHED writer whose stale
//! iceoryx2 port lingers un-swept keeps the live count above the writers heard, so
//! the gather cannot early-exit and pays the full `window` ceiling (the documented
//! bound — the per-pass count re-read still catches a MID-gather sweep). This is
//! why the registry works against per-process mirrors, with no daemon.
//!
//! # The doorbell: the gather ASKS, live writers ANSWER
//!
//! Periodic republication alone makes a gather's verdict a BET ON A TIMER. The
//! reader can only CATCH a broadcast, so "did I hear every live writer?" reduces
//! to "did a republish land inside my window?" — and the republish interval is
//! not a duration a process controls. MEASURED on macOS under background QoS
//! (reproduced with `taskpolicy -b`, which is the regime a GitHub macOS runner
//! puts the job in): a pump that sleeps in six 25 ms chunks to keep
//! teardown prompt pays for it, because macOS timer coalescing charges its slack PER WAKEUP:
//! a nominal 150 ms interval measured **1100–1696 ms** — longer than the whole
//! 600 ms [`MIRROR_GATHER_WINDOW`]. Every gather on such a desk therefore
//! reported `Incomplete` with `writers_heard: 0` while a perfectly healthy
//! writer sat there publishing on schedule, which is a FALSE
//! `mirrors_established: false` in every bag that desk records — the exact
//! defect the presence frame closed, re-opened by the operating system rather than by the code.
//!
//! Widening the window or shortening the interval would only move the bet. The
//! fix is to stop betting: [`MIRROR_REGISTRY_DOORBELL_SERVICE_NAME`] is an
//! iceoryx2 EVENT service on which
//!
//! - every writer's republish thread WAITS (`timed_wait_one`) instead of
//!   sleeping, and
//! - a reader NOTIFIES once per gather pass, before it drains.
//!
//! So a gather's first pass ASKS every live writer to speak and hears the answer
//! on its next pass — one IPC wake, no timer involved. The interval survives as
//! the FALLBACK (an old writer with no listener, a reader that could not mint a
//! notifier, a writer that starts mid-gather and misses a ring), and it is now
//! ONE wait rather than six chained sleeps, so it pays timer slack once:
//! MEASURED 332–407 ms in the same background-QoS regime, against 1100–1696 ms
//! before. Teardown stays prompt WITHOUT chunking because the writer keeps a
//! notifier of its own and rings it on drop.
//!
//! Purely additive and version-free: no wire format changes, so a writer without
//! a listener simply never hears a ring and a reader without a notifier simply
//! never sends one — each falls back to the interval, exactly as before. Both
//! sides degrade LOUDLY but non-fatally if the event service cannot be opened.
//! Documented residuals: a ring reaches only listeners that exist AT NOTIFY TIME
//! (hence the ring-every-pass), and one ring wakes EVERY writer on the machine,
//! each answering with one frame — which is precisely what a gather wants, since
//! it must hear from all of them.
//!
//! # Record/replay inertness
//!
//! `/__cerulion/mirrors` is inert to record + replay BY CONSTRUCTION, exactly as
//! `reg_channel`'s control service is: `bagd` taps only its explicit tap plan
//! (never all iceoryx2 services) and refuses the `__cerulion/` reserved prefix;
//! replay reads bag channels excluding `__cerulion/*`; and `topic list` enumerates
//! `*/data` services, which this name is not.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use iceoryx2::node::Node;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::ServiceName;
// The concrete factory type AND the trait share the name `PortFactory`; bring the
// trait into scope anonymously so `factory.dynamic_config()` resolves without
// shadowing the concrete type used in signatures below.
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;
use iceoryx2::service::port_factory::PortFactory as _;

use super::CerService;
use crate::error::{TransportError, TransportResult};

/// The fixed iceoryx2 service name of the mirror-provenance registry —
/// canonical `/__cerulion/mirrors`, under
/// [`RESERVED_TOPIC_PREFIX`](super::gateway::RESERVED_TOPIC_PREFIX). Deliberately
/// carries NO `/data` suffix (unlike graph topic services `{topic}/data`), so
/// `cerulion topic list` — which enumerates `*/data` services — never surfaces
/// it. Both the writer (`MirrorRegistry`) and the reader (`MirrorRegistryReader`)
/// build their service from this ONE constant, so their static configs cannot
/// drift.
pub const MIRROR_REGISTRY_SERVICE_NAME: &str = "/__cerulion/mirrors";

/// `max_publishers` the registry is provisioned with — one provenance
/// writer per local re-injecting process (vizd, `topic echo`, connectd, the
/// future netd). Sized generously above any realistic count of concurrent
/// re-injectors; nowhere near iceoryx2's port ceilings.
pub const MIRROR_REGISTRY_MAX_WRITERS: usize = 64;

/// `max_subscribers` the registry is provisioned with — concurrent
/// `topic list` / Studio readers, each a short-lived one-shot gather, plus
/// headroom. Larger than [`reg_channel`](super::reg_channel)'s single-gateway
/// reader budget because the mirror registry's readers are user-facing CLIs that
/// can legitimately run in parallel.
pub const MIRROR_REGISTRY_MAX_READERS: usize = 8;

/// `subscriber_max_buffer_size` — the per-pass reader queue. Comfortably
/// exceeds any realistic count of simultaneously-mirrored topics on one desk so
/// the whole set lands in a single drain pass.
pub const MIRROR_REGISTRY_QUEUE_DEPTH: usize = 256;

/// The fixed iceoryx2 EVENT service name of the registry DOORBELL — a
/// reader rings it to ask every live writer to republish NOW, so a gather never
/// has to catch a periodic broadcast (see the module docs). Follows the
/// `{name}/event` convention of graph topics and carries no `/data` suffix, so
/// `cerulion topic list` never surfaces it. A DISTINCT service name from
/// [`MIRROR_REGISTRY_SERVICE_NAME`] is required, not cosmetic: iceoryx2 keys a
/// service by name, and opening one name under two messaging patterns is an
/// incompatibility error.
pub const MIRROR_REGISTRY_DOORBELL_SERVICE_NAME: &str = "/__cerulion/mirrors/event";

/// `max_listeners` on the doorbell — one per live registry WRITER
/// (each writer's republish thread waits on its own listener), so this tracks
/// [`MIRROR_REGISTRY_MAX_WRITERS`] exactly.
pub const MIRROR_DOORBELL_MAX_LISTENERS: usize = MIRROR_REGISTRY_MAX_WRITERS;

/// `max_notifiers` on the doorbell. Every WRITER keeps one (it rings
/// its own pump awake at teardown, which is what lets the fallback wait be ONE
/// long wait instead of six chained sleeps) and every concurrent READER mints
/// one for the duration of a gather — hence writers + readers.
pub const MIRROR_DOORBELL_MAX_NOTIFIERS: usize =
    MIRROR_REGISTRY_MAX_WRITERS + MIRROR_REGISTRY_MAX_READERS;

/// How often the background republish thread re-sends the full
/// provenance set (the belt that lets a fresh reader converge to the live set
/// regardless of start order). 150 ms keeps a reader's bounded gather snappy
/// while adding negligible SHM traffic for a handful of mirrored topics.
///
/// This is the FALLBACK cadence, not the primary path — a gather
/// rings the doorbell and is answered without waiting for it. It is also the
/// bound on how long a `MirrorRegistry` drop can take on the degraded path
/// where no doorbell could be opened (with one, teardown rings the pump awake).
pub const MIRROR_REPUBLISH_INTERVAL: Duration = Duration::from_millis(150);

/// The reader's bounded gather window CEILING (see `gather_from_node`).
/// The gather stops EARLY the instant it has heard from every live writer, so this
/// ceiling is only ever fully paid in the pathological case of a CRASHED writer
/// whose stale iceoryx2 port lingers un-swept (the live count stays above the
/// writers heard). Sized to comfortably span several `MIRROR_REPUBLISH_INTERVAL`s
/// plus in-process connection-establishment latency.
pub const MIRROR_GATHER_WINDOW: Duration = Duration::from_millis(600);

/// The record magic (`0xCE`, `0x37`; the `CE` is for Cerulion). A frame that
/// does not open with this is not one of ours and is rejected as malformed
/// (defense in depth on a dedicated service that should never carry foreign
/// frames).
const MIRROR_MAGIC: [u8; 2] = [0xCE, 0x37];

/// The record wire-format version. Version 2 added the per-writer
/// `writer_id` (the independent-writer gather-completeness fix). Bumped only on an
/// incompatible record layout change; a mismatched version is rejected as
/// malformed.
const MIRROR_VERSION: u8 = 2;

/// The fixed record header length — `magic(2) + version(1) +
/// writer_id(8) + topic_len(2) + robot_len(2)`. The topic bytes then the robot
/// bytes follow.
pub const MIRROR_HEADER_LEN: usize = 2 + 1 + 8 + 2 + 2;

/// The maximum registrable topic length (bytes). Far above any realistic
/// canonical topic name; bounds the record size.
pub const MAX_MIRROR_TOPIC_LEN: usize = 512;

/// The maximum origin-robot identity length (bytes). Far above any
/// realistic hostname / robot identity.
pub const MAX_MIRROR_ROBOT_LEN: usize = 256;

/// The maximum record length — the writer's `initial_max_slice_len`. A
/// record is `MIRROR_HEADER_LEN + topic.len() + robot.len()` bytes.
pub const MAX_MIRROR_RECORD_LEN: usize =
    MIRROR_HEADER_LEN + MAX_MIRROR_TOPIC_LEN + MAX_MIRROR_ROBOT_LEN;

/// A decoded provenance record — a mirrored topic plus the ORIGIN ROBOT
/// identity the re-injecting process attributed it to. This is the CLI-facing
/// payload; the wire ALSO carries a per-writer `writer_id` (see `decode_record`)
/// used only by the gather's completeness gate, deliberately NOT surfaced here.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MirrorRecord {
    /// The (leading-slash) topic the process mirrors into local SHM — stored
    /// verbatim (NOT canonicalized) so it equals `topic list`'s `/data`-stripped
    /// service name.
    pub topic: String,
    /// The origin robot identity the mirror is attributed to (its hostname /
    /// `CERULION_ROBOT_IDENTITY`). Rendered VERBATIM only after
    /// terminal-escape sanitization at the display seam (a hostile LAN peer's
    /// name reaches this via a re-injector) — see `topic_cmd` partition/render.
    pub origin_robot: String,
}

/// A LOCAL topic that is really a live MIRROR of a
/// remote robot's topic (its name appears in the `/__cerulion/mirrors` provenance
/// registry). It belongs in the ORIGIN ROBOT's section, NOT under the desk's own
/// topics — the "one data source = one topic" decision. The `robot` string
/// is externally-sourced (a remote robot's identity, carried by a re-injector), so
/// every surface that RENDERS it must sanitize it at its own display seam (the CLI
/// does so in `topic_cmd`; the vizd wire hands it to Studio as a JSON string).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MirrorStreamRow {
    /// The mirrored topic, exactly as the local enumeration named it.
    pub topic: String,
    /// The origin robot identity the mirror is attributed to.
    pub robot: String,
}

/// Attribute ONE locally-visible topic to its origin robot, or `None`
/// when it is a genuine local producer.
///
/// This is THE fold predicate, in one place. `mirrors` is the live
/// `/__cerulion/mirrors` snapshot (`topic → origin robot`, see
/// [`gather_current_provenance`]); `remembered` is an optional per-session
/// fallback for a topic a consumer itself attached from a known robot earlier
/// (vizd's provenance tombstone), which keeps the attribution STABLE across the
/// windows where the live snapshot cannot answer — a best-effort gather that
/// errored, a cache refresh, or a mirror torn down while the consumer still holds
/// the topic. Live provenance WINS: a topic re-mirrored from a different robot
/// re-attributes immediately rather than being pinned by a stale memory.
///
/// Returning `None` is the "this is really ours" answer, so the caller renders it
/// under the desk's own topics — the behaviour of a registry-less desk.
pub fn attribute_local_topic<'a>(
    topic: &str,
    mirrors: &'a BTreeMap<String, String>,
    remembered: Option<&'a str>,
) -> Option<&'a str> {
    mirrors.get(topic).map(String::as_str).or(remembered)
}

/// Fold the live provenance snapshot together with a consumer's own
/// REMEMBERED attributions into ONE `topic → origin robot` map.
///
/// A consumer that holds topics it demanded from a known robot (vizd's session
/// provenance tombstone) must attribute them the SAME way on every surface it
/// serves — the enumeration and the attach state alike — or one topic occupies two
/// rows. Merging once, here, is what makes that structural: every surface reads
/// the same map, so they cannot disagree even while the best-effort live gather is
/// erroring or serving a stale cache.
///
/// Precedence is [`attribute_local_topic`]'s, expressed in exactly one place: live
/// provenance WINS, so a topic re-mirrored from a different robot re-attributes
/// immediately instead of being pinned by the memory.
pub fn merge_attribution(
    mirrors: &BTreeMap<String, String>,
    remembered: &BTreeMap<String, String>,
) -> BTreeMap<String, String> {
    // DEDUPE the key walk first: a topic present in BOTH sources appears twice in a
    // plain chain, so the predicate would run (and the entry be built) twice for it.
    let topics: BTreeSet<&String> = remembered.keys().chain(mirrors.keys()).collect();
    topics
        .into_iter()
        .filter_map(|topic| {
            attribute_local_topic(topic, mirrors, remembered.get(topic).map(String::as_str))
                .map(|robot| (topic.clone(), robot.to_string()))
        })
        .collect()
}

/// PURE partition of a LOCAL topic enumeration into
/// genuinely-local topics and mirror-streaming rows (folded out of LOCAL,
/// attributed to their origin robot).
///
/// Hoisted into `cerulion_core` so the CLI (`cerulion topic list`) and
/// the Studio daemon (`cerulion-vizd`'s `discover`) fold through ONE predicate.
/// Two copies of this loop is exactly how
/// a surface gets missed: a fix applied to one copy leaves the other
/// listing a robot's topic as a second, desk-local data source.
///
/// With an EMPTY `mirrors` map this is the identity partition (every topic stays
/// local, zero streaming rows) — a registry-less / no-mirror desk behaves
/// as if the fold were absent. Order is PRESERVED within each half, so a sorted
/// enumeration yields two sorted halves. Oracle-tested below.
pub fn partition_local_topics<I>(
    local: I,
    mirrors: &BTreeMap<String, String>,
) -> (Vec<String>, Vec<MirrorStreamRow>)
where
    I: IntoIterator<Item = String>,
{
    let mut genuine_local = Vec::new();
    let mut streaming = Vec::new();
    for topic in local {
        match attribute_local_topic(&topic, mirrors, None) {
            Some(robot) => {
                let robot = robot.to_string();
                streaming.push(MirrorStreamRow { topic, robot });
            }
            None => genuine_local.push(topic),
        }
    }
    (genuine_local, streaming)
}

/// Why a provenance record failed to encode or decode. Every variant is
/// a distinct, testable reason (oracle-vector coverage below); the reader drain
/// maps ANY decode error to a counted + warn-once + continue (a poisoned/hostile
/// record must never wedge the drain).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MirrorRecordError {
    /// The topic is empty (no topic name to register).
    EmptyTopic,
    /// The origin robot identity is empty.
    EmptyRobot,
    /// The topic exceeds [`MAX_MIRROR_TOPIC_LEN`] bytes.
    TopicTooLong { len: usize },
    /// The robot identity exceeds [`MAX_MIRROR_ROBOT_LEN`] bytes.
    RobotTooLong { len: usize },
    /// The frame is shorter than [`MIRROR_HEADER_LEN`] (truncated header).
    Truncated { len: usize },
    /// The leading magic bytes are not `MIRROR_MAGIC` (not a Cerulion record).
    BadMagic,
    /// The version byte is not `MIRROR_VERSION`.
    BadVersion { found: u8 },
    /// The declared topic+robot lengths disagree with the frame length.
    LengthMismatch {
        declared: usize,
        actual_payload: usize,
    },
    /// The topic or robot bytes are not valid UTF-8.
    InvalidUtf8,
}

impl std::fmt::Display for MirrorRecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MirrorRecordError::EmptyTopic => write!(f, "empty topic name"),
            MirrorRecordError::EmptyRobot => write!(f, "empty origin robot identity"),
            MirrorRecordError::TopicTooLong { len } => {
                write!(
                    f,
                    "topic length {len} exceeds the {MAX_MIRROR_TOPIC_LEN}-byte limit"
                )
            }
            MirrorRecordError::RobotTooLong { len } => {
                write!(
                    f,
                    "origin robot length {len} exceeds the {MAX_MIRROR_ROBOT_LEN}-byte limit"
                )
            }
            MirrorRecordError::Truncated { len } => {
                write!(
                    f,
                    "record is {len} bytes, shorter than the {MIRROR_HEADER_LEN}-byte header"
                )
            }
            MirrorRecordError::BadMagic => {
                write!(f, "bad record magic (not a Cerulion mirror record)")
            }
            MirrorRecordError::BadVersion { found } => {
                write!(
                    f,
                    "unsupported record version {found} (expected {MIRROR_VERSION})"
                )
            }
            MirrorRecordError::LengthMismatch {
                declared,
                actual_payload,
            } => write!(
                f,
                "record declares {declared} topic+robot bytes but carries {actual_payload}"
            ),
            MirrorRecordError::InvalidUtf8 => write!(f, "topic/robot bytes are not valid UTF-8"),
        }
    }
}

/// Encode a `(writer_id, topic, origin_robot)` provenance record for the
/// control wire — `magic(2) ++ version(1) ++ writer_id(8, LE) ++ topic_len(2, LE)
/// ++ robot_len(2, LE) ++ topic_bytes ++ robot_bytes`. Pure (no transport); the
/// exact inverse of [`decode_record`]. The `writer_id` distinguishes independent
/// writer processes so a gather knows when it has heard from all of them.
///
/// # Errors
///
/// - [`MirrorRecordError::EmptyTopic`] / [`MirrorRecordError::EmptyRobot`] for an
///   empty topic or robot.
/// - [`MirrorRecordError::TopicTooLong`] / [`MirrorRecordError::RobotTooLong`] if
///   either exceeds its byte limit.
pub fn encode_record(
    writer_id: u64,
    topic: &str,
    origin_robot: &str,
) -> Result<Vec<u8>, MirrorRecordError> {
    if topic.is_empty() {
        return Err(MirrorRecordError::EmptyTopic);
    }
    if origin_robot.is_empty() {
        return Err(MirrorRecordError::EmptyRobot);
    }
    if topic.len() > MAX_MIRROR_TOPIC_LEN {
        return Err(MirrorRecordError::TopicTooLong { len: topic.len() });
    }
    if origin_robot.len() > MAX_MIRROR_ROBOT_LEN {
        return Err(MirrorRecordError::RobotTooLong {
            len: origin_robot.len(),
        });
    }
    let mut buf = Vec::with_capacity(MIRROR_HEADER_LEN + topic.len() + origin_robot.len());
    buf.extend_from_slice(&MIRROR_MAGIC);
    buf.push(MIRROR_VERSION);
    buf.extend_from_slice(&writer_id.to_le_bytes());
    buf.extend_from_slice(&(topic.len() as u16).to_le_bytes());
    buf.extend_from_slice(&(origin_robot.len() as u16).to_le_bytes());
    buf.extend_from_slice(topic.as_bytes());
    buf.extend_from_slice(origin_robot.as_bytes());
    Ok(buf)
}

/// Encode a PRESENCE frame — "writer `writer_id` is live and currently
/// mirrors NOTHING".
///
/// # Why an empty writer must still be audible
///
/// `republish` sends one frame per record, so a writer holding an EMPTY record
/// map sent nothing at all — and a reader can only learn a writer's identity
/// from a frame. That made a LIVE writer with zero records byte-identical on the
/// wire to a writer the reader simply never heard, which is not a corner case:
/// it is `cerulion-netd`'s NORMAL state after its last mirror retires
/// (`unregister_mirror_provenance` empties the map and deliberately keeps the
/// registry — and so the iceoryx2 publisher — alive for the process lifetime).
/// Every gather on such a desk was permanently `Incomplete`, which through the checked gather
/// stamps a FALSE `mirrors_established: false` into every bag's durable manifest
/// and makes a healthy machine warn on every recording.
///
/// # Wire shape, and what an OLD reader does with it
///
/// Same `MIRROR_MAGIC` and same `MIRROR_VERSION` — deliberately NOT a
/// version bump, because a bump would make every OLD reader reject every NEW
/// reader's *records*, which is catastrophic where this is merely noisy. The
/// frame is the bare header with `topic_len == 0 && robot_len == 0`, a shape
/// [`encode_record`] can never produce (it refuses an empty topic or robot).
///
/// A reader predating the presence frame decodes it with `decode_record`, which rejects an empty
/// topic, so every presence frame it receives is counted malformed.
///
/// STATED AT ITS REAL RATE: "one
/// latched warn" would be wrong in both halves. `MirrorRegistryReader::open`
/// starts with `malformed_warned: false` and `gather_from_node_inner` opens a
/// FRESH reader on every gather, so the latch is per-GATHER, not per-process:
/// the true figure is one `warn!` (plus `debug!` repeats for the rest of that
/// window's frames) PER GATHER. An old reader also cannot settle against a
/// presence-only writer — its `set_nonempty` gate can never fire — so it pays
/// the full `MIRROR_GATHER_WINDOW` and receives every presence frame in it. A
/// polling consumer scales accordingly: an old vizd re-gathering at its 3 s
/// provenance TTL logs on the order of twenty warns a minute, sustained.
///
/// The cost is nonetheless bounded and deliberately SCOPED: presence frames are
/// sent ONLY while the record map is empty, so a writer with mirrors stays
/// byte-identical on the wire to before presence frames existed, and the one state that gains frames is
/// the state where an old reader was already getting the wrong answer.
///
/// SCOPE OF THE TEST BELOW: `a_presence_frame_is_a_frame_new_readers_understand_and_old_ones_reject`
/// pins the DECODE outcome (`decode_record` rejects it as `EmptyTopic`) — a pure
/// function. It is NOT a bound on the flood: nothing drains a presence frame
/// through a real older reader, so the rate above is reasoned from the code,
/// not measured.
pub fn encode_presence(writer_id: u64) -> Vec<u8> {
    let mut buf = Vec::with_capacity(MIRROR_HEADER_LEN);
    buf.extend_from_slice(&MIRROR_MAGIC);
    buf.push(MIRROR_VERSION);
    buf.extend_from_slice(&writer_id.to_le_bytes());
    buf.extend_from_slice(&0u16.to_le_bytes()); // topic_len
    buf.extend_from_slice(&0u16.to_le_bytes()); // robot_len
    buf
}

/// One decoded control frame — a provenance RECORD, or a bare PRESENCE
/// announcement from a writer that currently mirrors nothing.
///
/// Both carry the writer identity, because both are evidence of the same thing:
/// this writer has been HEARD, and the gather's completeness gate can count it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MirrorFrame {
    /// A provenance registration: this writer mirrors `topic` from `robot`.
    Record(u64, MirrorRecord),
    /// A live writer announcing that it currently mirrors nothing.
    Presence(u64),
}

impl MirrorFrame {
    /// The writer identity this frame was heard from — the only thing the
    /// completeness gate needs, and the one field both arms share.
    #[must_use]
    pub fn writer_id(&self) -> u64 {
        match self {
            Self::Record(id, _) => *id,
            Self::Presence(id) => *id,
        }
    }
}

/// Decode any control frame — the reader-side entry point.
///
/// Recognises a PRESENCE frame (see [`encode_presence`]) before delegating to
/// [`decode_record`], so the two cannot be confused: a presence frame is a
/// well-formed statement that this writer has no mirrors, NOT a malformed
/// record. Every other malformed shape still reaches `decode_record`'s existing
/// rejections unchanged.
///
/// # Errors
///
/// [`MirrorRecordError`] for any frame that is neither a valid presence
/// announcement nor a valid record.
pub fn decode_frame(bytes: &[u8]) -> Result<MirrorFrame, MirrorRecordError> {
    if bytes.len() == MIRROR_HEADER_LEN
        && bytes[0..2] == MIRROR_MAGIC
        && bytes[2] == MIRROR_VERSION
        && u16::from_le_bytes(bytes[11..13].try_into().expect("2 bytes")) == 0
        && u16::from_le_bytes(bytes[13..15].try_into().expect("2 bytes")) == 0
    {
        let writer_id = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
        return Ok(MirrorFrame::Presence(writer_id));
    }
    decode_record(bytes).map(|(id, r)| MirrorFrame::Record(id, r))
}

/// Decode a control-wire provenance record — the exact inverse of
/// [`encode_record`]. Returns the `(writer_id, MirrorRecord)` pair. Pure (no
/// transport). Validates the magic, version, the declared-vs-actual payload
/// length, and UTF-8 so a truncated / hostile / foreign frame is rejected with a
/// precise reason rather than silently misread. (Terminal-escape sanitization of a
/// valid-UTF-8-but-hostile robot name is a DISPLAY concern, applied at the
/// `topic list` render seam — a valid record with a control-char-bearing name
/// decodes fine here.)
///
/// # Errors
///
/// Any [`MirrorRecordError`] variant — each on its own condition (see the variant
/// docs): [`Truncated`](MirrorRecordError::Truncated),
/// [`BadMagic`](MirrorRecordError::BadMagic),
/// [`BadVersion`](MirrorRecordError::BadVersion),
/// [`TopicTooLong`](MirrorRecordError::TopicTooLong) /
/// [`RobotTooLong`](MirrorRecordError::RobotTooLong) (declared),
/// [`LengthMismatch`](MirrorRecordError::LengthMismatch),
/// [`InvalidUtf8`](MirrorRecordError::InvalidUtf8),
/// [`EmptyTopic`](MirrorRecordError::EmptyTopic) /
/// [`EmptyRobot`](MirrorRecordError::EmptyRobot).
pub fn decode_record(bytes: &[u8]) -> Result<(u64, MirrorRecord), MirrorRecordError> {
    if bytes.len() < MIRROR_HEADER_LEN {
        return Err(MirrorRecordError::Truncated { len: bytes.len() });
    }
    if bytes[0..2] != MIRROR_MAGIC {
        return Err(MirrorRecordError::BadMagic);
    }
    if bytes[2] != MIRROR_VERSION {
        return Err(MirrorRecordError::BadVersion { found: bytes[2] });
    }
    // Fixed-offset reads (the length check above makes these infallible).
    let writer_id = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
    let topic_len = u16::from_le_bytes(bytes[11..13].try_into().expect("2 bytes")) as usize;
    let robot_len = u16::from_le_bytes(bytes[13..15].try_into().expect("2 bytes")) as usize;
    if topic_len > MAX_MIRROR_TOPIC_LEN {
        return Err(MirrorRecordError::TopicTooLong { len: topic_len });
    }
    if robot_len > MAX_MIRROR_ROBOT_LEN {
        return Err(MirrorRecordError::RobotTooLong { len: robot_len });
    }
    let actual_payload = bytes.len() - MIRROR_HEADER_LEN;
    if topic_len + robot_len != actual_payload {
        return Err(MirrorRecordError::LengthMismatch {
            declared: topic_len + robot_len,
            actual_payload,
        });
    }
    let topic_end = MIRROR_HEADER_LEN + topic_len;
    let topic = std::str::from_utf8(&bytes[MIRROR_HEADER_LEN..topic_end])
        .map_err(|_| MirrorRecordError::InvalidUtf8)?;
    let origin_robot =
        std::str::from_utf8(&bytes[topic_end..]).map_err(|_| MirrorRecordError::InvalidUtf8)?;
    if topic.is_empty() {
        return Err(MirrorRecordError::EmptyTopic);
    }
    if origin_robot.is_empty() {
        return Err(MirrorRecordError::EmptyRobot);
    }
    Ok((
        writer_id,
        MirrorRecord {
            topic: topic.to_string(),
            origin_robot: origin_robot.to_string(),
        },
    ))
}

/// Mint a practically-unique per-`MirrorRegistry`-instance identity,
/// distinct across processes AND across multiple registries in one process
/// (tests). Mixes the pid, a nanosecond timestamp, and a process-global sequence —
/// no external RNG dependency. A collision (astronomically unlikely) only makes a
/// gather UNDER-count writers heard (two writers read as one), which is the SAFE
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
        ^ nanos.rotate_left(17)
        ^ seq.wrapping_mul(0xD1B5_4A32_D192_ED03).rotate_left(31)
}

/// Open the ONE registry pub/sub service (`open_or_create`, so whichever
/// side — writer or reader — arrives first CREATES it and the other OPENS it).
/// Both sides call THIS helper, so the static config (buffer / writer slots /
/// reader slots; NO event service, NO history) is identical and `open_or_create`
/// is always compatible.
fn open_registry_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    let name: ServiceName =
        MIRROR_REGISTRY_SERVICE_NAME
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "mirror registry: invalid control service name \
                     '{MIRROR_REGISTRY_SERVICE_NAME}': {e:?}"
                ),
            })?;
    node.service_builder(&name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(MIRROR_REGISTRY_QUEUE_DEPTH)
        .max_subscribers(MIRROR_REGISTRY_MAX_READERS)
        .max_publishers(MIRROR_REGISTRY_MAX_WRITERS)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "mirror registry: could not open the control service \
                 '{MIRROR_REGISTRY_SERVICE_NAME}': {e:?}"
            ),
        })
}

/// Open the ONE doorbell EVENT service (`open_or_create`, like its
/// pub/sub sibling, so whichever side arrives first creates it and both agree on
/// the static config). Callers treat a failure as DEGRADED, never fatal: without
/// a doorbell the writer falls back to its republish interval and the reader to
/// catching it: the pre-doorbell behaviour, which is slow rather than wrong.
fn open_doorbell_service(
    node: &Node<CerService>,
) -> TransportResult<iceoryx2::service::port_factory::event::PortFactory<CerService>> {
    let name: ServiceName = MIRROR_REGISTRY_DOORBELL_SERVICE_NAME
        .try_into()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "mirror registry: invalid doorbell service name \
                 '{MIRROR_REGISTRY_DOORBELL_SERVICE_NAME}': {e:?}"
            ),
        })?;
    node.service_builder(&name)
        .event()
        .max_listeners(MIRROR_DOORBELL_MAX_LISTENERS)
        .max_notifiers(MIRROR_DOORBELL_MAX_NOTIFIERS)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "mirror registry: could not open the doorbell service \
                 '{MIRROR_REGISTRY_DOORBELL_SERVICE_NAME}': {e:?}"
            ),
        })
}

/// Mint a WRITER's doorbell ports — the listener its republish thread
/// waits on, and the notifier its teardown rings to wake that thread.
///
/// BEST-EFFORT by construction. Every failure arm returns `None` after ONE loud
/// `warn!` naming what is lost, because the alternative — failing
/// `register_mirror_provenance` — would take a desk from "provenance converges
/// on a timer" to "provenance does not exist", trading a latency problem for a
/// correctness one. The warn states the consequence in the operator's terms:
/// gathers must now CATCH the broadcast.
fn open_writer_doorbell(node: &Node<CerService>) -> Option<DoorbellPorts> {
    let degraded = |what: &str, reason: String| -> Option<DoorbellPorts> {
        tracing::warn!(
            reason = %reason,
            what = %what,
            "mirror registry: could not create a doorbell port (the `what` field names \
             which) — this writer still republishes every {MIRROR_REPUBLISH_INTERVAL:?}, but a \
             reader's gather can no longer ASK it to speak and must CATCH that broadcast inside \
             its window instead"
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

/// The shared, thread-safe core of a [`MirrorRegistry`] — the control publisher
/// (Send+Sync via `ipc_threadsafe`, shared by `Arc` between the register callsite
/// AND the background republish thread with no extra mutex), this writer's stable
/// `writer_id`, the authoritative topic→robot map, and the send-failure flood
/// latch. `BTreeMap` gives a deterministic republish order for reproducible tests.
struct MirrorRegistryInner {
    publisher: Publisher<CerService, [u8], ()>,
    /// This writer instance's stable identity, stamped into every record so a
    /// gather can count the distinct writers it has heard from.
    writer_id: u64,
    /// Topic → origin robot identity. Every registration is retained and re-sent;
    /// dropping the whole [`MirrorRegistry`] (process/writer exit) is what makes a
    /// mirror's provenance expire (it stops being republished).
    records: Mutex<BTreeMap<String, String>>,
    /// First send failure of a run logs `warn!`, repeats `debug!`; a success
    /// re-arms (the flood-suppression discipline, mirroring `reg_channel`).
    send_warned: AtomicBool,
    /// Doorbell rings this writer's pump has ANSWERED (Principle #3).
    /// The observable that separates "a gather settled because it ASKED" from
    /// "a gather settled because it happened to catch the timer" — a
    /// distinction no wall-clock assertion can make on a loaded runner.
    doorbell_rings: AtomicU64,
    /// First doorbell WAIT failure logs `warn!`, repeats `debug!` — a broken
    /// listener degrades to the interval, it never wedges or spins the pump.
    doorbell_warned: AtomicBool,
}

impl MirrorRegistryInner {
    /// Insert (or update) a provenance registration. Returns `true` iff the
    /// topic was not already present. LAST-WRITE-WINS on the robot: a re-register
    /// with a DIFFERENT origin overwrites (a topic re-attributed to a new robot
    /// converges to the latest attribution).
    fn insert(&self, topic: &str, origin_robot: &str) -> bool {
        let mut records = self.records.lock().unwrap_or_else(|e| e.into_inner());
        records
            .insert(topic.to_string(), origin_robot.to_string())
            .is_none()
    }

    /// Remove a provenance registration. Returns `true` iff the topic
    /// was present. The wire carries no delete tombstone — a removed record simply
    /// stops being republished, so a `topic list` reader (which gathers only the
    /// records republished during its window) stops folding it into REMOTE. The
    /// remaining records keep republishing unchanged.
    fn remove(&self, topic: &str) -> bool {
        let mut records = self.records.lock().unwrap_or_else(|e| e.into_inner());
        records.remove(topic).is_some()
    }

    /// Encode + loan + send ONE record on the control publisher. Returns whether
    /// the send succeeded. A send failure is flood-latched and non-fatal — the
    /// periodic republish re-attempts it.
    fn send_record(&self, topic: &str, origin_robot: &str) -> bool {
        let bytes = match encode_record(self.writer_id, topic, origin_robot) {
            Ok(b) => b,
            Err(e) => {
                // Unreachable from the public API: `register_mirror_provenance`
                // validates the topic + robot (non-empty, within bounds) before
                // the record ever enters the authoritative map. We still handle
                // the `Err` (never send garbage) and flag any regression.
                debug_assert!(
                    false,
                    "send_record hit an unencodable record ({e}) — the public API \
                     guards were bypassed (a map invariant broke)"
                );
                tracing::warn!(
                    topic = %topic,
                    error = %e,
                    "mirror registry: refusing to send an unencodable record"
                );
                return false;
            }
        };
        match self.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => {
                let sample = sample.write_from_slice(&bytes);
                match sample.send() {
                    Ok(_) => {
                        self.record_send_success();
                        true
                    }
                    Err(e) => {
                        self.record_send_failure(topic, &format!("{e}"));
                        false
                    }
                }
            }
            Err(e) => {
                self.record_send_failure(topic, &format!("{e}"));
                false
            }
        }
    }

    /// Re-send EVERY retained provenance record (the background thread body + the
    /// deterministic test seam). Returns the number successfully sent. Snapshots
    /// the map under the lock, then sends OUTSIDE the lock (a slow send must not
    /// block a concurrent register). The realistic mirror count fits comfortably
    /// under the reader queue, so no send-order rotation is needed (unlike
    /// `reg_channel`'s ~90-route burst class).
    /// An EMPTY record map still sends — a bare PRESENCE frame (see
    /// [`encode_presence`]). Otherwise a live writer holding no records is
    /// SILENT, and silence is exactly what a reader cannot distinguish from a
    /// writer it failed to hear: `cerulion-netd` after its last mirror retires
    /// keeps its registry (and publisher) alive for the process lifetime, so
    /// every gather on such a desk timed out and reported itself Incomplete
    /// forever. Returns the number of frames successfully sent (a presence frame
    /// counts as one), so a caller can still tell "sent something" from "the
    /// send failed".
    fn republish(&self) -> usize {
        let snapshot: Vec<(String, String)> = {
            let records = self.records.lock().unwrap_or_else(|e| e.into_inner());
            records
                .iter()
                .map(|(t, r)| (t.clone(), r.clone()))
                .collect()
        };
        if snapshot.is_empty() {
            return usize::from(self.send_presence());
        }
        let mut sent = 0usize;
        for (topic, robot) in snapshot {
            if self.send_record(&topic, &robot) {
                sent += 1;
            }
        }
        sent
    }

    /// Send this writer's PRESENCE frame — "I am live and mirror
    /// nothing". Shares `send_record`'s failure reporting (one latch, one
    /// counter); the topic field reads `<presence>` since there is none.
    fn send_presence(&self) -> bool {
        let bytes = encode_presence(self.writer_id);
        match self.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => {
                let sample = sample.write_from_slice(&bytes);
                match sample.send() {
                    Ok(_) => {
                        self.record_send_success();
                        true
                    }
                    Err(e) => {
                        self.record_send_failure("<presence>", &format!("{e}"));
                        false
                    }
                }
            }
            Err(e) => {
                self.record_send_failure("<presence>", &format!("{e}"));
                false
            }
        }
    }

    fn record_send_failure(&self, topic: &str, reason: &str) {
        if self.send_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                topic = %topic,
                reason = %reason,
                "mirror registry: provenance send failed (repeat — see the first warning)"
            );
        } else {
            tracing::warn!(
                topic = %topic,
                reason = %reason,
                "mirror registry: provenance send failed — this record will retry on the \
                 next republish (repeats log at debug)"
            );
        }
    }

    fn record_send_success(&self) {
        if self.send_warned.swap(false, Ordering::Relaxed) {
            tracing::info!("mirror registry: provenance send recovered — republishing again");
        }
    }

    #[cfg(test)]
    fn record_count(&self) -> usize {
        self.records.lock().unwrap_or_else(|e| e.into_inner()).len()
    }

    /// Record that the pump was woken by a doorbell ring rather than by
    /// its fallback timeout.
    fn note_doorbell_ring(&self) {
        self.doorbell_rings.fetch_add(1, Ordering::Relaxed);
    }

    /// A doorbell WAIT failed. Flood-latched and non-fatal — the caller
    /// falls back to sleeping the interval, so a broken listener costs latency,
    /// never correctness, and never a spin.
    fn record_doorbell_failure(&self, reason: &str) {
        if self.doorbell_warned.swap(true, Ordering::Relaxed) {
            tracing::debug!(
                reason = %reason,
                "mirror registry: doorbell wait failed (repeat — see the first warning)"
            );
        } else {
            tracing::warn!(
                reason = %reason,
                "mirror registry: doorbell wait failed — this writer falls back to \
                 republishing every {MIRROR_REPUBLISH_INTERVAL:?}, so a reader's gather must \
                 CATCH that broadcast instead of being answered (repeats log at debug)"
            );
        }
    }
}

/// A joined-on-drop handle for the background republish thread.
struct PumpHandle {
    exit: Arc<AtomicBool>,
    /// This writer's OWN doorbell notifier, rung at teardown to wake
    /// the pump out of its wait. This is what lets the pump wait ONCE for a
    /// whole interval instead of polling `exit` in short chunks — and the
    /// chunking is precisely what macOS timer coalescing multiplied into a
    /// missed gather window (module docs). `None` on the degraded path, where
    /// a drop instead waits out at most one [`MIRROR_REPUBLISH_INTERVAL`].
    wake: Option<iceoryx2::port::notifier::Notifier<CerService>>,
    join: Option<JoinHandle<()>>,
}

impl Drop for PumpHandle {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        // Ring BEFORE joining: the pump re-reads `exit` the moment it wakes.
        // This also wakes every OTHER writer's pump on this machine, each of
        // which answers with one presence/record frame — the same bounded cost
        // a reader's gather ring pays, and harmless.
        if let Some(wake) = &self.wake {
            let _ = wake.notify();
        }
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// The WRITER side of the mirror-provenance registry — one per
/// re-injecting process, held lazily by its
/// [`TransportManager`](super::TransportManager). Owns the control publisher +
/// this writer's stable identity + the authoritative topic→robot set + the
/// background republish thread. When the process exits (this drops), republication
/// stops and the mirror's provenance expires (see the module docs).
///
/// `pump` is declared BEFORE `inner` so it drops FIRST: joining the republish
/// thread before the `inner` Arc is released.
pub(crate) struct MirrorRegistry {
    pump: Mutex<Option<PumpHandle>>,
    /// The doorbell ports, minted at `open` and CONSUMED when the pump
    /// first starts — the listener moves into the pump thread (that is what it
    /// waits on), the notifier stays in the [`PumpHandle`] to wake it at
    /// teardown. `None` means the doorbell could not be opened: a loud,
    /// non-fatal degrade to the pre-doorbell interval cadence.
    doorbell: Mutex<Option<DoorbellPorts>>,
    inner: Arc<MirrorRegistryInner>,
    /// The pump's fallback republish belt. Production is always
    /// [`MIRROR_REPUBLISH_INTERVAL`]; threading it instead of reading the const
    /// inside [`pump_loop`] lets ONE test QUIESCE the belt, so a gather settled
    /// inside its ceiling can only have been settled by a doorbell ring — a
    /// structural attribution, where a free-running belt would make the arm bet on
    /// winning a race against a 150 ms timer of unknown phase.
    republish_interval: Duration,
}

/// A writer's doorbell port pair (see [`MirrorRegistry::doorbell`]).
struct DoorbellPorts {
    listener: iceoryx2::port::listener::Listener<CerService>,
    notifier: iceoryx2::port::notifier::Notifier<CerService>,
}

impl MirrorRegistry {
    /// Open the registry service on `node` and build a fresh writer with a newly
    /// minted `writer_id` (no records, no pump thread yet — the pump starts lazily
    /// on the first `register`).
    pub(crate) fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_registry_service(node)?;
        let publisher = factory
            .publisher_builder()
            .initial_max_slice_len(MAX_MIRROR_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!(
                    "mirror registry: could not create the control publisher on \
                     '{MIRROR_REGISTRY_SERVICE_NAME}': {e:?}"
                ),
            })?;
        Ok(Self {
            pump: Mutex::new(None),
            doorbell: Mutex::new(open_writer_doorbell(node)),
            inner: Arc::new(MirrorRegistryInner {
                publisher,
                writer_id: mint_writer_id(),
                records: Mutex::new(BTreeMap::new()),
                send_warned: AtomicBool::new(false),
                doorbell_rings: AtomicU64::new(0),
                doorbell_warned: AtomicBool::new(false),
            }),
            republish_interval: MIRROR_REPUBLISH_INTERVAL,
        })
    }

    /// Register `topic` as a mirror of `origin_robot`: record it, send it once
    /// immediately (the reader-already-up fast path), and ensure the background
    /// republish thread is running (the belt that lets a later reader converge).
    /// Idempotent — a repeat returns `false` and re-sends the record. Returns
    /// whether the topic was genuinely NEW.
    pub(crate) fn register(&self, topic: &str, origin_robot: &str) -> bool {
        let newly = self.inner.insert(topic, origin_robot);
        self.inner.send_record(topic, origin_robot);
        self.ensure_pump_running();
        newly
    }

    /// Unregister `topic` (refcount-0 teardown): drop it from the
    /// authoritative provenance set so the background pump stops republishing it and
    /// `topic list` stops folding it into REMOTE. Returns whether the topic was
    /// present. The pump is left running for the remaining records (it self-idles
    /// when the whole registry drops); no wire tombstone is sent — expiry is by
    /// ceasing republication (see [`MirrorRegistryInner::remove`]).
    pub(crate) fn unregister(&self, topic: &str) -> bool {
        self.inner.remove(topic)
    }

    /// This writer instance's stable identity. Test seam (the two-writer gather
    /// race asserts distinct writers were heard).
    #[cfg(test)]
    pub(crate) fn writer_id(&self) -> u64 {
        self.inner.writer_id
    }

    /// Insert a record WITHOUT sending or starting the pump — the deterministic
    /// test seam (drives republish inline, no background thread races).
    #[cfg(test)]
    pub(crate) fn insert_for_test(&self, topic: &str, origin_robot: &str) -> bool {
        self.inner.insert(topic, origin_robot)
    }

    /// Re-send the entire provenance set inline (no thread). The deterministic
    /// test seam — production drives the SAME [`MirrorRegistryInner::republish`]
    /// body from the background thread ([`pump_loop`]).
    #[cfg(test)]
    pub(crate) fn republish(&self) -> usize {
        self.inner.republish()
    }

    /// Send arbitrary raw bytes on the control publisher WITHOUT recording them —
    /// the seam tests use to craft hostile / malformed frames on the wire.
    /// Returns whether the send succeeded.
    #[cfg(test)]
    pub(crate) fn send_raw_for_test(&self, bytes: &[u8]) -> bool {
        match self.inner.publisher.loan_slice_uninit(bytes.len()) {
            Ok(sample) => sample.write_from_slice(bytes).send().is_ok(),
            Err(_) => false,
        }
    }

    /// The number of distinct topics in the authoritative provenance set
    /// (Principle #3). Test seam.
    #[cfg(test)]
    pub(crate) fn record_count(&self) -> usize {
        self.inner.record_count()
    }

    /// How many doorbell rings this writer's pump has ANSWERED
    /// (Principle #3). The observable that makes "the gather ASKED and was
    /// answered" a MEASUREMENT rather than an inference: a wall-clock reading
    /// cannot separate a doorbell answer from a lucky timer tick on a loaded
    /// runner (the load-sensitive timing class), and this can.
    #[cfg(test)]
    pub(crate) fn doorbell_rings_answered(&self) -> u64 {
        self.inner.doorbell_rings.load(Ordering::Relaxed)
    }

    /// QUIESCE this writer's fallback republish belt so a gather's
    /// settle is attributable to the doorbell BY CONSTRUCTION rather than by
    /// winning a wall-clock race. Returns `false` — refusing to change anything
    /// — on either shape that would make the quiesce unsound:
    ///
    /// * NO DOORBELL: the pump would take the `None => sleep(belt)` arm and
    ///   [`PumpHandle`]'s drop `wake` would be `None`, so teardown could not ring
    ///   it awake and would join a thread parked for the whole belt.
    /// * PUMP ALREADY RUNNING: the belt is read at spawn, so a late change is a
    ///   silent no-op and every assertion built on it would be vacuous.
    ///
    /// SCOPE, measured rather than asserted: today the second conjunct is
    /// SUBSUMED by the first, so removing it changes nothing observable. `ensure_pump_running`
    /// CONSUMES the doorbell ports (`self.doorbell.lock().take()`), so a live pump
    /// implies an empty `doorbell` and `!has_bell` already refuses — dropping
    /// `pump_running` alone leaves all 28 arms of this module GREEN (RUN, not
    /// reasoned). It is kept as DECLARED INTENT, not defence that currently fires:
    /// the two premises are independent claims, and the day the ports stop being
    /// taken at spawn this conjunct is the one still standing. Do not read its
    /// presence as evidence that a test exercises it.
    ///
    /// `#[cfg(test)]` is COMPILE-TIME confinement: the seam does not exist in a
    /// shipping build, so the "production silently built with the test seam"
    /// case cannot compile and needs no source walk (contrast the mDNS beacon, whose
    /// `_for_test` constructors WERE compiled in).
    #[cfg(test)]
    pub(crate) fn quiesce_republish_belt_for_test(&mut self, belt: Duration) -> bool {
        let has_bell = self
            .doorbell
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some();
        let pump_running = self
            .pump
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some();
        if !has_bell || pump_running {
            return false;
        }
        self.republish_interval = belt;
        true
    }

    /// Whether the background republish thread is LIVE (Principle #3). Test seam.
    #[cfg(test)]
    pub(crate) fn republish_pump_active(&self) -> bool {
        self.pump
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .is_some()
    }

    /// Start the background republish thread exactly once (latched-idempotent). A
    /// spawn failure is non-fatal + loud: every `register` STILL sends its record
    /// immediately (a reader already up receives it), but the periodic convergence
    /// belt is disabled — a reader that opens later may miss the mirror until the
    /// next register re-sends it.
    fn ensure_pump_running(&self) {
        let mut pump = self.pump.lock().unwrap_or_else(|e| e.into_inner());
        if pump.is_some() {
            return;
        }
        let exit = Arc::new(AtomicBool::new(false));
        let exit_thread = Arc::clone(&exit);
        let inner = Arc::clone(&self.inner);
        // The belt is read HERE, at spawn — which is why
        // `quiesce_republish_belt_for_test` refuses once a pump exists.
        let interval = self.republish_interval;
        // Hand the LISTENER to the pump (its wait) and keep the
        // NOTIFIER here (its teardown wake). Taken, not cloned — a second pump
        // is impossible, and the ports must not outlive one owner.
        let ports = self
            .doorbell
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .take();
        let (listener, wake) = match ports {
            Some(DoorbellPorts { listener, notifier }) => (Some(listener), Some(notifier)),
            None => (None, None),
        };
        let spawned = std::thread::Builder::new()
            .name("cer-mirror-republish".to_string())
            .spawn(move || pump_loop(&exit_thread, &inner, listener, interval));
        match spawned {
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
                    "mirror registry: could not spawn the republish thread — each register \
                     still sends its record immediately (a reader already up receives it), but the \
                     periodic convergence belt is DISABLED: a reader that opens later may miss this \
                     mirror's provenance until the next register re-sends it"
                );
            }
        }
    }
}

/// The background republish thread body: until asked to exit, WAIT — for a
/// doorbell ring or, failing that, one CONFIGURED republish belt (production:
/// [`MIRROR_REPUBLISH_INTERVAL`]; see [`MirrorRegistry::republish_interval`]) —
/// then re-send the whole provenance set. Record-only: it publishes into SHM a
/// reader drains; it never touches the network or the graph's execution
/// (Principle #7 is unaffected).
///
/// The loop makes ONE wait rather than a chain of six 25 ms sleeps, and that shape is
/// load-bearing rather than a tidy-up. Chunking would only serve to keep teardown
/// prompt, and it charges macOS timer coalescing SIX TIMES per interval —
/// MEASURED 1100–1696 ms for a nominal 150 ms under background QoS, i.e. longer
/// than the entire gather window (module docs). Teardown promptness comes
/// from the doorbell ring in [`PumpHandle::drop`] instead, so waiting once is
/// free. On the degraded path (no listener) this is a single sleep, which pays
/// that slack once — measured 332–407 ms in the same regime.
fn pump_loop(
    exit: &AtomicBool,
    inner: &Arc<MirrorRegistryInner>,
    doorbell: Option<iceoryx2::port::listener::Listener<CerService>>,
    interval: Duration,
) {
    while !exit.load(Ordering::Relaxed) {
        match doorbell.as_ref() {
            // iceoryx2 0.10: `timed_wait_one` is gone — a ring is `Ok(n > 0)`,
            // the timeout `Ok(0)` (see `run_registry::pump_loop`).
            Some(listener) => match listener.timed_wait(|_a| {}, interval) {
                // A ring: somebody is gathering and wants this writer's answer.
                Ok(n) if n > 0 => inner.note_doorbell_ring(),
                // The fallback timeout — republish on the interval as before.
                Ok(_) => {}
                // A broken listener must degrade to the interval, never spin:
                // without this sleep a persistently-failing wait would return
                // instantly and republish in a tight loop.
                Err(e) => {
                    inner.record_doorbell_failure(&format!("{e}"));
                    std::thread::sleep(interval);
                }
            },
            None => std::thread::sleep(interval),
        }
        if exit.load(Ordering::Relaxed) {
            return;
        }
        inner.republish();
    }
}

/// The READER side of the mirror-provenance registry — a listener-less
/// subscriber on the control service that ALSO tracks the distinct `writer_id`s it
/// has heard from (the gather-completeness gate). Short-lived: a `topic list` /
/// Studio reader opens it, gathers one snapshot (`gather_from_node`), and drops
/// it. Opened `open_or_create`, so a reader on a desk with no mirror ever CREATES
/// the service (and sees zero publishers → an empty snapshot).
pub(crate) struct MirrorRegistryReader {
    subscriber: Subscriber<CerService, [u8], ()>,
    /// Distinct writer identities observed across every drain of this reader's
    /// life — the numerator of the gather's "heard from every live writer" gate.
    writers_heard: BTreeSet<u64>,
    /// Cumulative malformed records dropped at the drain (Principle #3).
    malformed: u64,
    /// First malformed record of the reader's life logs `warn!`; repeats `debug!`.
    malformed_warned: bool,
    /// First drain receive failure logs `warn!`; repeats `debug!`.
    receive_warned: bool,
}

impl MirrorRegistryReader {
    /// Open the registry service on `node` and build the reader.
    pub(crate) fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let factory = open_registry_service(node)?;
        let subscriber =
            factory
                .subscriber_builder()
                .create()
                .map_err(|e| TransportError::Internal {
                    reason: format!(
                        "mirror registry: could not create the control subscriber on \
                     '{MIRROR_REGISTRY_SERVICE_NAME}': {e:?}"
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

    /// Drain up to [`MIRROR_REGISTRY_QUEUE_DEPTH`] records off the control queue,
    /// decoding each: valid records push onto `out` AND record their `writer_id`
    /// into `writers_heard`. A malformed record is counted + warn-once + SKIPPED
    /// (never wedges the drain); a mid-drain receive failure is warn-once + returns
    /// the partial fill (non-fatal). Never holds more than one sample borrowed.
    /// Returns the number of valid records pushed.
    pub(crate) fn drain(&mut self, out: &mut Vec<MirrorRecord>) -> usize {
        let mut pushed = 0usize;
        for _ in 0..MIRROR_REGISTRY_QUEUE_DEPTH {
            match self.subscriber.receive() {
                // `decode_frame`, not `decode_record` — a PRESENCE
                // frame is a well-formed statement ("this writer mirrors
                // nothing"), so it marks the writer HEARD and pushes no record.
                // Treating it as malformed would leave the writer unheard,
                // which is the very state this frame exists to end.
                Ok(Some(sample)) => match decode_frame(sample.payload()) {
                    Ok(frame) => {
                        self.writers_heard.insert(frame.writer_id());
                        if let MirrorFrame::Record(_, record) = frame {
                            out.push(record);
                            pushed += 1;
                        }
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
        pushed
    }

    /// The count of DISTINCT live writers this reader has heard from so far — the
    /// numerator of `gather_from_node`'s completeness gate.
    fn writers_heard_count(&self) -> usize {
        self.writers_heard.len()
    }

    /// Cumulative malformed records dropped at the drain (Principle #3).
    #[cfg(test)]
    pub(crate) fn malformed_count(&self) -> u64 {
        self.malformed
    }

    fn record_malformed(&mut self, error: &MirrorRecordError) {
        self.malformed += 1;
        if self.malformed_warned {
            tracing::debug!(
                error = %error,
                "mirror registry: dropped a malformed control record (repeat)"
            );
        } else {
            self.malformed_warned = true;
            tracing::warn!(
                error = %error,
                "mirror registry: dropped a malformed control record on \
                 '{MIRROR_REGISTRY_SERVICE_NAME}' (counted; the drain continues; repeats at debug)"
            );
        }
    }

    fn record_receive_failure(&mut self, reason: &str) {
        if self.receive_warned {
            tracing::debug!(
                reason = %reason,
                "mirror registry: control drain receive failed (repeat)"
            );
        } else {
            self.receive_warned = true;
            tracing::warn!(
                reason = %reason,
                "mirror registry: control drain receive failed — kept the partial fill; \
                 the gather retries (repeats log at debug)"
            );
        }
    }
}

/// The gather EARLY-EXIT predicate — stop the gather once we have heard
/// from every currently-live writer. Pure so it is oracle-testable independent of
/// transport + wall-clock. Requires `writers_heard >= live_publishers` with
/// `live_publishers > 0` (the caller returns early when the live count is zero).
/// The key correctness property over a set-stability heuristic: with two live
/// writers on offset republish phases, `writers_heard == 1 < live == 2` keeps the
/// gather running until the second is heard, so it never returns an incomplete
/// snapshot.
///
/// **The `set_nonempty` conjunct is gone**, and that removal is half the
/// fix rather than a tidy-up. It was a PROXY for "heard from somebody" back when
/// a record was the only frame a writer could send — so a writer with an empty
/// record map could never satisfy it, and `netd`-after-retire (a live publisher
/// holding zero records, the normal desk state) could never reach `Settled` no
/// matter how many times it was heard. With [`encode_presence`] such a writer is
/// audible, and `writers_heard` is the direct measurement the proxy stood in for.
/// A SETTLED EMPTY answer is now reachable and means what it says: every live
/// writer was heard, and none of them mirrors anything.
fn gather_reached_all_live_writers(writers_heard: usize, live_publishers: usize) -> bool {
    live_publishers > 0 && writers_heard >= live_publishers
}

/// What a gather's answer is EVIDENCE of — the discriminator that makes
/// an EMPTY snapshot readable.
///
/// A gather is a windowed LISTEN, so "I heard no records" and "there are no
/// records" are different statements, and only one of them is a fact about the
/// desk. Carrying which one the caller got is what lets a durable consumer (the
/// recorder's coverage manifest) decline to claim what it did not establish.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatherCompleteness {
    /// The answer is EVIDENCE. Either no writer was live at all (the instant
    /// fast path — a desk with no re-injector), or every live writer was heard
    /// from before the window expired. An empty record set here genuinely means
    /// "nothing is mirrored".
    Settled,
    /// The window expired with live writers this gather never heard from. The
    /// record set is whatever arrived, which may be EMPTY and may be a strict
    /// subset of the truth — it is NOT an absence claim.
    ///
    /// Reached in two shipping shapes: a CRASHED writer whose stale iceoryx2
    /// port lingers un-swept (the documented ceiling case — the records that DID
    /// arrive are still correct), and a live writer whose republish never
    /// reached this reader inside the window (scheduling starvation under load —
    /// the unheard-writer case, where the set is empty and nothing was established).
    Incomplete {
        /// Publishers the registry service reported live on the final pass.
        live_writers: usize,
        /// Distinct writer identities this reader actually heard from.
        writers_heard: usize,
    },
}

impl GatherCompleteness {
    /// Whether an EMPTY record set from this gather may be read as "no mirrors".
    ///
    /// The one question a consumer actually asks. A non-empty set is useful
    /// under either verdict (the records that arrived are real); it is the
    /// empty case whose meaning depends on this.
    #[must_use]
    pub fn is_settled(&self) -> bool {
        matches!(self, Self::Settled)
    }
}

/// A gather's records PLUS what they are evidence of.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MirrorGather {
    /// Every provenance record heard during the window, deduped by topic.
    pub records: Vec<MirrorRecord>,
    /// Whether this answer settles the question — see [`GatherCompleteness`].
    pub completeness: GatherCompleteness,
}

/// Gather the CURRENT live mirror-provenance snapshot over `node`, deduped
/// by topic (last-write-wins on a topic re-attributed to a new robot). A fresh
/// bounded snapshot each call — the reader does NOT accumulate across calls, so a
/// dropped writer's provenance naturally disappears (expiry, see the module docs).
///
/// FAST PATH: if the registry service reports ZERO publishers (no live mirror
/// writer on the desk — the common headless / local-only case), returns empty
/// IMMEDIATELY without paying any window.
///
/// Otherwise it drains for up to `window`, unioning every republished record, and
/// EARLY-EXITS the instant it has heard from every currently-live writer
/// (`gather_reached_all_live_writers`), re-reading the live publisher count each
/// pass so a mid-gather stale-port sweep is honored. A CRASHED writer whose stale
/// port lingers keeps the live count above the writers heard, so the gather pays
/// the full `window` ceiling (documented) but still returns every LIVE writer's
/// records correctly.
///
/// # Errors
///
/// [`TransportError`] from opening the control service or building the subscriber
/// (an SHM / config failure). The CLI treats any error as "no provenance"
/// (best-effort) so `topic list` never fails on the registry.
pub(crate) fn gather_from_node(
    node: &Node<CerService>,
    window: Duration,
) -> TransportResult<Vec<MirrorRecord>> {
    gather_from_node_inner(node, window, GatherDoorbell::Ring, |_pass| {}).map(|g| g.records)
}

/// Whether a gather RINGS the doorbell (asking every live writer to
/// speak now) or listens SILENTLY for the writers' own republish cadence.
///
/// Production always rings — the silent arm exists so a test can drive the
/// FALLBACK deliberately and prove it is still real rather than inert. Without
/// that arm the interval cadence would be reachable by no assertion the moment
/// the doorbell works, which is how a belt quietly rots into dead code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GatherDoorbell {
    /// Ring before every drain pass — the production posture.
    Ring,
    /// Never ring; hear only what the writers broadcast on their own timer.
    #[cfg_attr(not(test), allow(dead_code))]
    Silent,
}

/// [`gather_from_node`], but carrying WHAT THE ANSWER IS EVIDENCE OF.
///
/// The plain gather collapses three different situations into one empty `Vec`:
/// no writer exists (a settled absence), every live writer was heard and none
/// registered anything (also settled), and **the window expired having heard
/// nothing at all while a writer was live** (not an absence — an unknown). A
/// caller that treats the third as "no mirrors" makes a confident claim on no
/// evidence, which is the class this repo has closed twice before (the query
/// plane's `DiscoveryState`, the recorder's `enumerated`).
///
/// It is not hypothetical: `cerulion_bagd`'s arm-time gather hit exactly the
/// third case on macOS CI — the recorder paid the full 600 ms window, heard
/// nothing, and recorded another robot's mirrored stream as this machine's data
/// while its coverage manifest called the recording complete.
pub(crate) fn gather_from_node_checked(
    node: &Node<CerService>,
    window: Duration,
) -> TransportResult<MirrorGather> {
    gather_from_node_inner(node, window, GatherDoorbell::Ring, |_pass| {})
}

/// The gather body, parameterized by a per-pass `hook(pass_index)` called BEFORE
/// each drain. Production passes a no-op; the deterministic two-writer race test
/// passes a hook that republishes specific writers at controlled passes so it can
/// exercise the phase-offset case without wall-clock flakiness.
fn gather_from_node_inner(
    node: &Node<CerService>,
    window: Duration,
    doorbell: GatherDoorbell,
    mut hook: impl FnMut(usize),
) -> TransportResult<MirrorGather> {
    let factory = open_registry_service(node)?;
    // No live writer ⇒ no mirrors ⇒ instant empty (the common desk). This empty
    // is EVIDENCE: the service itself reports nobody is publishing provenance.
    if factory.dynamic_config().number_of_publishers() == 0 {
        return Ok(MirrorGather {
            records: Vec::new(),
            completeness: GatherCompleteness::Settled,
        });
    }
    let mut reader = MirrorRegistryReader::open(node)?;
    // Mint the doorbell notifier AFTER the zero-publisher fast path, so
    // a desk with no re-injector never creates the event service just to ask a
    // question it already has the answer to. Best-effort: no notifier means
    // this gather listens for the writers' own cadence, as every gather did before the doorbell.
    let bell = match doorbell {
        GatherDoorbell::Ring => open_gather_notifier(node),
        GatherDoorbell::Silent => None,
    };
    // Union across republishes; dedup by topic (last-write-wins).
    let mut set: BTreeMap<String, String> = BTreeMap::new();
    let mut scratch: Vec<MirrorRecord> = Vec::new();
    let deadline = Instant::now() + window;
    const POLL_CHUNK: Duration = Duration::from_millis(20);
    let mut pass = 0usize;
    // WHICH break arm the loop takes is the whole verdict, so it is
    // recorded at the break rather than re-derived afterwards — re-deriving from
    // the final counts cannot distinguish "the deadline expired on this state"
    // from "this state satisfied the early exit".
    let completeness;
    loop {
        // ASK, then listen. Ringing every pass (not just the first) is
        // what covers a writer whose listener did not exist when the previous
        // ring went out — a notification reaches only the listeners alive at
        // notify time, and a writer can start mid-gather. The loop exits as
        // soon as every live writer has been heard, so a healthy desk rings
        // once or twice; only the pathological stale-port case rings for the
        // whole window, and a ring is a semaphore post answered by one frame.
        if let Some(bell) = &bell {
            let _ = bell.notify();
        }
        hook(pass);
        scratch.clear();
        reader.drain(&mut scratch);
        for r in scratch.drain(..) {
            set.insert(r.topic, r.origin_robot);
        }
        // Re-read the LIVE publisher count each pass (drops when a stale port is
        // swept mid-gather). Zero ⇒ every writer gone; return what we have.
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
        pass += 1;
        std::thread::sleep(POLL_CHUNK);
    }
    Ok(MirrorGather {
        records: set
            .into_iter()
            .map(|(topic, origin_robot)| MirrorRecord {
                topic,
                origin_robot,
            })
            .collect(),
        completeness,
    })
}

/// Mint the READER's doorbell notifier for one gather.
///
/// BEST-EFFORT, and the degrade is `debug!` rather than `warn!` on purpose: a
/// gather is a read-only question asked by short-lived, user-facing commands
/// (`topic list`, a Studio refresh, bagd's arm-time snapshot), and losing the
/// doorbell costs LATENCY — the writers still broadcast on their interval — so
/// warning on every `topic list` would be the flood this repo suppresses
/// elsewhere. The WRITER's inability to answer is the loud one (it is durable
/// and affects every reader on the machine); a single reader's inability to ask
/// is not.
fn open_gather_notifier(
    node: &Node<CerService>,
) -> Option<iceoryx2::port::notifier::Notifier<CerService>> {
    let service = match open_doorbell_service(node) {
        Ok(s) => s,
        Err(e) => {
            tracing::debug!(
                reason = %e,
                "mirror registry: no doorbell service for this gather — falling back to \
                 catching the writers' periodic republish inside the window"
            );
            return None;
        }
    };
    match service.notifier_builder().create() {
        Ok(n) => Some(n),
        Err(e) => {
            tracing::debug!(
                reason = ?e,
                "mirror registry: could not mint a doorbell notifier for this gather — \
                 falling back to catching the writers' periodic republish inside the window"
            );
            None
        }
    }
}

/// Build a bare, side-effect-minimal iceoryx2 node on the process-global
/// config for a one-shot mirror-provenance gather — the `cerulion topic list`
/// entry. Auto dead-node cleanup is DISABLED (never trust the global config; a
/// read-only `topic list` must never reap anyone), and NO startup
/// dead-node sweep runs (this is a transient reader, not a transport init).
fn build_reader_node() -> TransportResult<Node<CerService>> {
    crate::iceoryx_logger::init_iceoryx_log_level_from_env();
    let config =
        super::disable_auto_dead_node_cleanup(iceoryx2::config::Config::global_config().clone());
    let node_name: iceoryx2::prelude::NodeName =
        "cerulion-mirror-reader"
            .try_into()
            .map_err(|e| TransportError::Internal {
                reason: format!("mirror registry: invalid reader node name: {e:?}"),
            })?;
    iceoryx2::prelude::NodeBuilder::new()
        .name(&node_name)
        .config(&config)
        .create::<CerService>()
        .map_err(|e| TransportError::Internal {
            reason: format!("mirror registry: could not create the reader node: {e:?}"),
        })
}

/// Gather the CURRENT live mirror-provenance snapshot on the
/// process-global iceoryx2 namespace — the production `cerulion topic list` /
/// Studio entry point. Mints a transient reader node (see `build_reader_node`) and
/// delegates to `gather_from_node` with the default [`MIRROR_GATHER_WINDOW`].
///
/// # Errors
///
/// [`TransportError`] from building the reader node or opening the control service
/// — the CLI treats any error as "no provenance" (best-effort), so `topic list`
/// never fails on the registry.
pub fn gather_current_provenance() -> TransportResult<Vec<MirrorRecord>> {
    let node = build_reader_node()?;
    gather_from_node(&node, MIRROR_GATHER_WINDOW)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a `topic → robot` provenance snapshot from pairs (test helper).
    fn mirrors(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
        pairs
            .iter()
            .map(|(t, r)| ((*t).to_string(), (*r).to_string()))
            .collect()
    }

    /// Enumeration helper: a local `list_topics()`-shaped vector.
    fn enumerated(topics: &[&str]) -> Vec<String> {
        topics.iter().map(|t| (*t).to_string()).collect()
    }

    /// THE fold oracle: a mixed enumeration — two mirrors of DIFFERENT
    /// robots interleaved with genuine desk producers — splits into exactly the
    /// hand-written halves, each attributed to the RIGHT robot, with input order
    /// preserved on both sides. Hand oracle (never a self-compare).
    #[test]
    fn partition_splits_genuine_local_from_mirrors_of_each_robot() {
        let local = enumerated(&[
            "/demo/ticker/out",
            "/tf",
            "/utlidar/cloud",
            "/demo/sink/in",
            "/lf/lowstate",
        ]);
        let map = mirrors(&[
            ("/tf", "ubuntu"),
            ("/utlidar/cloud", "ubuntu"),
            ("/lf/lowstate", "go2"),
        ]);

        let (genuine, streaming) = partition_local_topics(local, &map);

        assert_eq!(
            genuine,
            vec!["/demo/ticker/out".to_string(), "/demo/sink/in".to_string()],
            "only the desk's OWN producers stay local, in enumeration order"
        );
        assert_eq!(
            streaming,
            vec![
                MirrorStreamRow {
                    topic: "/tf".to_string(),
                    robot: "ubuntu".to_string(),
                },
                MirrorStreamRow {
                    topic: "/utlidar/cloud".to_string(),
                    robot: "ubuntu".to_string(),
                },
                MirrorStreamRow {
                    topic: "/lf/lowstate".to_string(),
                    robot: "go2".to_string(),
                },
            ],
            "every mirror folds out of LOCAL attributed to ITS OWN origin robot"
        );
    }

    /// A registry-less / no-mirror desk is the IDENTITY partition — the unfolded
    /// behaviour, byte-for-byte. This is what makes the fold safe to run
    /// unconditionally on every surface.
    #[test]
    fn partition_with_no_mirrors_is_the_identity() {
        let local = enumerated(&["/a", "/b", "/c"]);
        let (genuine, streaming) = partition_local_topics(local.clone(), &BTreeMap::new());
        assert_eq!(genuine, local);
        assert!(streaming.is_empty());
    }

    /// A provenance entry for a topic that is NOT locally enumerated (a mirror
    /// torn down between the gather and the enumeration, or another process's
    /// registration) contributes NOTHING — the fold never invents a row.
    #[test]
    fn partition_ignores_provenance_for_an_unenumerated_topic() {
        let (genuine, streaming) = partition_local_topics(
            enumerated(&["/a"]),
            &mirrors(&[("/gone", "ubuntu"), ("/also_gone", "go2")]),
        );
        assert_eq!(genuine, vec!["/a".to_string()]);
        assert!(
            streaming.is_empty(),
            "provenance without a live local service must not fabricate a streaming row"
        );
    }

    /// The whole enumeration being mirrors ⇒ the desk owns NOTHING (the common
    /// live shape: a desk with no graph of its own, viewing a robot).
    #[test]
    fn partition_of_an_all_mirror_desk_leaves_no_local_topics() {
        let (genuine, streaming) = partition_local_topics(
            enumerated(&["/tf", "/tf_static"]),
            &mirrors(&[("/tf", "ubuntu"), ("/tf_static", "ubuntu")]),
        );
        assert!(
            genuine.is_empty(),
            "a desk whose every topic is a mirror must show NO topics of its own"
        );
        assert_eq!(streaming.len(), 2);
    }

    /// The name match is EXACT — a same-suffix / prefix-shaped neighbour of a
    /// mirrored topic is NOT folded (`/tf` mirrored must not drag `/tf_static`
    /// or `/robot/tf` out of LOCAL).
    #[test]
    fn attribution_matches_the_topic_name_exactly() {
        let map = mirrors(&[("/tf", "ubuntu")]);
        assert_eq!(attribute_local_topic("/tf", &map, None), Some("ubuntu"));
        for neighbour in ["/tf_static", "/robot/tf", "tf", "/TF"] {
            assert_eq!(
                attribute_local_topic(neighbour, &map, None),
                None,
                "'{neighbour}' is a DIFFERENT topic from the mirrored '/tf'"
            );
        }
    }

    /// Attribution precedence, all four cells against hand answers: live
    /// provenance WINS over a remembered robot (a re-mirror re-attributes at
    /// once), the memory carries the topic when the snapshot cannot answer, and
    /// with neither the topic is genuinely the desk's own.
    #[test]
    fn attribution_prefers_live_provenance_over_the_remembered_robot() {
        let map = mirrors(&[("/tf", "ubuntu")]);
        let empty = BTreeMap::new();

        assert_eq!(
            attribute_local_topic("/tf", &map, Some("stale-robot")),
            Some("ubuntu"),
            "live provenance must win — a topic re-mirrored from another robot re-attributes"
        );
        assert_eq!(
            attribute_local_topic("/tf", &map, None),
            Some("ubuntu"),
            "live provenance alone attributes"
        );
        assert_eq!(
            attribute_local_topic("/tf", &empty, Some("ubuntu")),
            Some("ubuntu"),
            "with no live snapshot the remembered robot keeps the row attributed"
        );
        assert_eq!(
            attribute_local_topic("/demo/out", &empty, None),
            None,
            "neither ⇒ a genuine local producer"
        );
    }

    /// The merge is ONE map every surface reads, so an enumeration fold
    /// and an attach-state row can never disagree. Hand oracle over all four
    /// cells: live-only, remembered-only, BOTH (live wins), and neither.
    #[test]
    fn merge_attribution_unions_both_sources_with_live_provenance_winning() {
        let live = mirrors(&[("/tf", "ubuntu"), ("/moved", "go2")]);
        let remembered = mirrors(&[("/moved", "ubuntu"), ("/remembered", "orin")]);

        let merged = merge_attribution(&live, &remembered);

        assert_eq!(
            merged,
            mirrors(&[
                ("/tf", "ubuntu"),
                ("/moved", "go2"),
                ("/remembered", "orin"),
            ]),
            "the union carries both sources, and the LIVE robot wins on /moved"
        );
        // A topic in neither source is absent (so it folds as genuinely local).
        assert!(!merged.contains_key("/demo/out"));
        // Degenerate inputs.
        assert!(merge_attribution(&BTreeMap::new(), &BTreeMap::new()).is_empty());
        assert_eq!(merge_attribution(&live, &BTreeMap::new()), live);
        assert_eq!(merge_attribution(&BTreeMap::new(), &remembered), remembered);
    }

    /// Partitioning the MERGED map produces the hand-written sectioning both
    /// halves of a consumer must agree on — the enumeration fold (`discover`) and
    /// the per-topic attribution (`list`/`status`) — so "one topic, one row" is
    /// structural. Hand oracle, NOT a comparison of the two functions against each
    /// other (which would pass with both broken the same way).
    #[test]
    fn partitioning_the_merged_map_yields_the_hand_written_sectioning() {
        let live = mirrors(&[("/tf", "ubuntu"), ("/moved", "go2")]);
        let remembered = mirrors(&[("/moved", "ubuntu"), ("/held", "orin")]);
        let merged = merge_attribution(&live, &remembered);

        let (genuine, streaming) = partition_local_topics(
            enumerated(&["/tf", "/held", "/moved", "/desk/own"]),
            &merged,
        );

        assert_eq!(
            genuine,
            vec!["/desk/own".to_string()],
            "only the desk's own producer stays local"
        );
        assert_eq!(
            streaming,
            vec![
                MirrorStreamRow {
                    topic: "/tf".to_string(),
                    robot: "ubuntu".to_string(),
                },
                MirrorStreamRow {
                    topic: "/held".to_string(),
                    robot: "orin".to_string(),
                },
                MirrorStreamRow {
                    topic: "/moved".to_string(),
                    robot: "go2".to_string(),
                },
            ],
            "live-only, remembered-only and BOTH (live wins) each land on the right robot"
        );
        // And the per-topic predicate the attach-state rows use gives the SAME four
        // answers, stated as hand values rather than by re-deriving them.
        let attribute = |t: &str| {
            attribute_local_topic(t, &live, remembered.get(t).map(String::as_str))
                .map(str::to_string)
        };
        assert_eq!(attribute("/tf"), Some("ubuntu".to_string()));
        assert_eq!(attribute("/held"), Some("orin".to_string()));
        assert_eq!(attribute("/moved"), Some("go2".to_string()));
        assert_eq!(attribute("/desk/own"), None);
    }

    /// The service name has no `/data` suffix (so `cerulion topic list` never
    /// surfaces it) and lives under the reserved control-plane namespace. Hand
    /// oracle — the integration twin against `data_topic_of_service` lives in
    /// `topic_cmd`.
    #[test]
    fn mirror_registry_service_name_has_no_data_suffix() {
        assert!(
            !MIRROR_REGISTRY_SERVICE_NAME.ends_with("/data"),
            "the mirror registry must NOT end in /data or `topic list` would surface it"
        );
        assert!(super::super::gateway::is_reserved_topic(
            MIRROR_REGISTRY_SERVICE_NAME
        ));
    }

    /// Round-trip: encode then decode reproduces the input EXACTLY (writer_id +
    /// record), across a plain pair, a max-length topic + max-length robot,
    /// single-char values, and a non-ASCII robot name. Hand oracle (field-by-field,
    /// not a self-compare).
    #[test]
    fn encode_decode_round_trips() {
        let cases: &[(u64, &str, &str)] = &[
            (0x1234_5678_9abc_def0, "/utlidar/robot_odom", "ubuntu"),
            (0, "/a", "r"),
            (u64::MAX, "/lf/lowstate", "go2-α"),
            (42, "/tf", "orin-nano.local"), // leak-scan: allow mdns-local an invented fixture name
        ];
        for &(writer_id, topic, robot) in cases {
            let bytes = encode_record(writer_id, topic, robot).expect("encode");
            assert_eq!(bytes.len(), MIRROR_HEADER_LEN + topic.len() + robot.len());
            assert_eq!(
                decode_record(&bytes).expect("decode"),
                (
                    writer_id,
                    MirrorRecord {
                        topic: topic.to_string(),
                        origin_robot: robot.to_string(),
                    }
                )
            );
        }
        // Genuinely MAX-length topic + robot round-trip; encodes to exactly the
        // max record length (pins the writer's initial_max_slice_len sizing).
        let max_topic = format!("/{}", "x".repeat(MAX_MIRROR_TOPIC_LEN - 1));
        let max_robot = "r".repeat(MAX_MIRROR_ROBOT_LEN);
        assert_eq!(max_topic.len(), MAX_MIRROR_TOPIC_LEN);
        let bytes = encode_record(7, &max_topic, &max_robot).expect("encode max");
        assert_eq!(bytes.len(), MAX_MIRROR_RECORD_LEN);
        let (id, decoded) = decode_record(&bytes).expect("decode max");
        assert_eq!(id, 7);
        assert_eq!(decoded.topic, max_topic);
        assert_eq!(decoded.origin_robot, max_robot);
    }

    /// Encode rejects empty topic, empty robot, over-long topic, over-long robot.
    /// Hand oracle.
    #[test]
    fn encode_rejects_empty_and_over_long() {
        assert_eq!(
            encode_record(1, "", "r"),
            Err(MirrorRecordError::EmptyTopic)
        );
        assert_eq!(
            encode_record(1, "/t", ""),
            Err(MirrorRecordError::EmptyRobot)
        );
        let long_topic = "x".repeat(MAX_MIRROR_TOPIC_LEN + 1);
        assert_eq!(
            encode_record(1, &long_topic, "r"),
            Err(MirrorRecordError::TopicTooLong {
                len: MAX_MIRROR_TOPIC_LEN + 1
            })
        );
        let long_robot = "r".repeat(MAX_MIRROR_ROBOT_LEN + 1);
        assert_eq!(
            encode_record(1, "/t", &long_robot),
            Err(MirrorRecordError::RobotTooLong {
                len: MAX_MIRROR_ROBOT_LEN + 1
            })
        );
    }

    /// Decode rejects every malformed shape with the PRECISE reason. Hand oracles
    /// per variant.
    #[test]
    fn decode_rejects_each_malformed_shape() {
        // Truncated (shorter than the header).
        assert_eq!(
            decode_record(&[0xCE, 0x37, 0x02]),
            Err(MirrorRecordError::Truncated { len: 3 })
        );
        let good = encode_record(9, "/t", "ro").expect("encode");
        // Bad magic.
        let mut bad_magic = good.clone();
        bad_magic[0] = 0x00;
        assert_eq!(decode_record(&bad_magic), Err(MirrorRecordError::BadMagic));
        // Bad version.
        let mut bad_ver = good.clone();
        bad_ver[2] = 0xFF;
        assert_eq!(
            decode_record(&bad_ver),
            Err(MirrorRecordError::BadVersion { found: 0xFF })
        );
        // Length mismatch: declare a 100-byte topic on a 2-byte-topic frame
        // (topic_len is at bytes[11..13] in the v2 header).
        let mut bad_len = good.clone();
        bad_len[11..13].copy_from_slice(&100u16.to_le_bytes());
        assert_eq!(
            decode_record(&bad_len),
            Err(MirrorRecordError::LengthMismatch {
                declared: 100 + 2, // topic 100 + robot 2
                actual_payload: 4, // 2 topic + 2 robot bytes carried
            })
        );
        // Over-long DECLARED topic length (> MAX) rejected before the match check.
        let mut over = good.clone();
        over[11..13].copy_from_slice(&((MAX_MIRROR_TOPIC_LEN + 1) as u16).to_le_bytes());
        assert_eq!(
            decode_record(&over),
            Err(MirrorRecordError::TopicTooLong {
                len: MAX_MIRROR_TOPIC_LEN + 1
            })
        );
        // Invalid UTF-8 in the robot bytes: plant non-UTF-8 at the tail.
        let mut bad_utf8 = encode_record(9, "/t", "ro").expect("encode");
        let n = bad_utf8.len();
        bad_utf8[n - 2] = 0xFF;
        bad_utf8[n - 1] = 0xFE;
        assert_eq!(
            decode_record(&bad_utf8),
            Err(MirrorRecordError::InvalidUtf8)
        );
    }

    /// A control-char-bearing robot name (a hostile LAN identity) decodes FINE —
    /// it is valid UTF-8. Terminal-escape sanitization is a DISPLAY concern
    /// (applied at the `topic list` render seam, pinned in `topic_cmd`), NOT a
    /// decode-time rejection. Hand oracle: the bytes round-trip verbatim.
    #[test]
    fn hostile_but_valid_utf8_robot_name_decodes_verbatim() {
        let hostile = "evil\u{1b}[2Krobot"; // ESC [ 2K — an ANSI erase-line escape
        let bytes = encode_record(3, "/t", hostile).expect("encode");
        assert_eq!(decode_record(&bytes).unwrap().1.origin_robot, hostile);
    }

    /// The gather early-exit predicate: stop ONLY once heard from every live
    /// writer. Hand-oracle vector. Mutation pin for ROOT 1: a regression to
    /// set-stability-only (ignore the writer/live counts, e.g. return
    /// `set_nonempty`) makes `(true, 1, 2)` true — this test then fails, because
    /// one writer heard of two live must KEEP GATHERING.
    #[test]
    fn gather_predicate_gates_on_all_live_writers_heard() {
        // Not yet heard from any writer.
        assert!(!gather_reached_all_live_writers(0, 1));
        // Heard from one of two live writers — MUST keep gathering (the ROOT-1 fix).
        assert!(!gather_reached_all_live_writers(1, 2));
        // Heard from both live writers — done.
        assert!(gather_reached_all_live_writers(2, 2));
        // A single live writer, heard — done.
        assert!(gather_reached_all_live_writers(1, 1));
        // Live count zero is the caller's early-return, but the predicate is
        // vacuously false regardless of writers_heard (belt-and-suspenders).
        assert!(!gather_reached_all_live_writers(3, 0));
        // Heard from MORE distinct writers than the current live count (a writer
        // dropped mid-gather) — still complete.
        assert!(gather_reached_all_live_writers(3, 2));
        // The predicate does not consult the RECORD SET. One live
        // writer, heard, contributing nothing is COMPLETE — that is netd after
        // its last mirror retires, and a `set_nonempty` conjunct would make it
        // permanently un-settleable. The two arms above that pass `1, 1` and
        // `2, 2` say nothing about a set this arm proves is irrelevant.
        assert!(
            gather_reached_all_live_writers(1, 1),
            "a heard writer with NO records settles the question: nothing is mirrored"
        );
    }

    /// SHM round-trip + EXPIRE-ON-DROP over a per-test root, driven
    /// DETERMINISTICALLY (inline `insert_for_test` + `republish` — NO background
    /// pump thread races). Register a mirror, prove the reader sees it (hand
    /// oracle), then DROP the writer and prove a fresh drain no longer delivers it
    /// (expiry). Cribs `reg_channel`'s
    /// `production_republish_rotation_converges_over_real_queue` two-manager shape.
    #[test]
    fn shm_round_trip_then_expire_on_writer_drop() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let mut reader = MirrorRegistryReader::open(&reader_mgr.node).expect("reader");
        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        // Inline seam: no pump thread ⇒ delivery happens ONLY on our explicit
        // `republish()`, so the expire assertion below is race-free.
        assert!(writer.insert_for_test("/utlidar/robot_odom", "ubuntu"));

        // Warm up the in-process pub↔sub connection (takes a pass or two).
        let mut scratch: Vec<MirrorRecord> = Vec::new();
        let mut connected = false;
        for _ in 0..40 {
            writer.republish();
            scratch.clear();
            if reader.drain(&mut scratch) > 0 {
                connected = true;
                break;
            }
        }
        assert!(
            connected,
            "the control pub↔sub connection never established"
        );

        // Fully drain, then one clean republish → the reader observes EXACTLY the
        // registered record (hand oracle, not a self-compare).
        loop {
            scratch.clear();
            if reader.drain(&mut scratch) == 0 {
                break;
            }
        }
        writer.republish();
        scratch.clear();
        reader.drain(&mut scratch);
        assert_eq!(
            scratch,
            vec![MirrorRecord {
                topic: "/utlidar/robot_odom".to_string(),
                origin_robot: "ubuntu".to_string(),
            }],
            "the reader observes exactly the registered provenance"
        );
        // The reader heard from exactly this ONE writer.
        assert_eq!(reader.writers_heard_count(), 1);
        assert!(reader.writers_heard.contains(&writer.writer_id()));

        // EXPIRE: drop the writer (process/writer exit). Its publisher is released
        // and nothing republishes, so a fresh drain delivers NOTHING — the mirror's
        // provenance has expired.
        drop(writer);
        for _ in 0..5 {
            scratch.clear();
            assert_eq!(
                reader.drain(&mut scratch),
                0,
                "after the writer drops, no provenance is delivered — it has expired"
            );
        }
    }

    /// The production `register` path records the topic AND starts the background
    /// republish pump (Principle #3 observability seams), and mints a nonzero
    /// stable writer id. Hand oracle on the accessors — no reader needed; the
    /// writer drops (joining the pump) at end.
    #[test]
    fn register_records_topic_and_starts_pump() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_reg".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");
        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        assert!(
            !writer.republish_pump_active(),
            "pump is lazy until register"
        );
        assert_eq!(writer.record_count(), 0);
        assert!(
            writer.register("/utlidar/robot_odom", "ubuntu"),
            "first register is genuinely new"
        );
        assert!(
            !writer.register("/utlidar/robot_odom", "ubuntu"),
            "an idempotent repeat is not new"
        );
        assert_eq!(writer.record_count(), 1, "one distinct topic recorded");
        assert!(
            writer.republish_pump_active(),
            "register started the background republish pump"
        );
    }

    /// `unregister` drops the topic from the authoritative set (returns
    /// whether it was present), and a subsequent republish no longer sends it — so a
    /// reader draining a FRESH window after the removal sees only the survivor (the
    /// removed mirror expires from `topic list`). Deterministic inline seams
    /// (`insert_for_test`/`republish`) over a per-test root — no background thread.
    #[test]
    fn unregister_drops_a_topic_and_stops_republishing_it() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_unreg".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_unreg".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let mut reader = MirrorRegistryReader::open(&reader_mgr.node).expect("reader");
        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        writer.insert_for_test("/gone", "ubuntu");
        writer.insert_for_test("/stays", "go2");
        assert_eq!(writer.record_count(), 2);

        // Warm the control connection until BOTH records are delivered in one window.
        let mut scratch: Vec<MirrorRecord> = Vec::new();
        let mut connected = false;
        for _ in 0..60 {
            writer.republish();
            scratch.clear();
            reader.drain(&mut scratch);
            if scratch.len() >= 2 {
                connected = true;
                break;
            }
        }
        assert!(connected, "both records never delivered");

        // Remove /gone. `unregister` reports it was present; a second call is a
        // no-op false (idempotent, loud-error-free at this layer).
        assert!(writer.unregister("/gone"), "/gone was present");
        assert!(!writer.unregister("/gone"), "already removed");
        assert!(!writer.unregister("/never"), "never present");
        assert_eq!(writer.record_count(), 1, "only /stays remains");

        // Drain any in-flight (pre-removal) samples empty, THEN republish + drain a
        // fresh window: only /stays is sent now — /gone has expired.
        loop {
            scratch.clear();
            if reader.drain(&mut scratch) == 0 {
                break;
            }
        }
        let mut seen: BTreeMap<String, String> = BTreeMap::new();
        for _ in 0..40 {
            writer.republish();
            scratch.clear();
            reader.drain(&mut scratch);
            for r in &scratch {
                seen.insert(r.topic.clone(), r.origin_robot.clone());
            }
        }
        assert_eq!(
            seen.get("/stays").map(String::as_str),
            Some("go2"),
            "the survivor still republishes"
        );
        assert!(
            !seen.contains_key("/gone"),
            "the unregistered mirror is no longer republished (expired from topic list)"
        );
    }

    /// The reader drain COUNTS + SKIPS a malformed on-wire frame (never wedges),
    /// and still delivers a valid one sent after it. Anti-tautology control on the
    /// drain's fault path. Deterministic inline seams over a per-test root.
    #[test]
    fn drain_skips_malformed_and_keeps_valid() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_mal".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_mal".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let mut reader = MirrorRegistryReader::open(&reader_mgr.node).expect("reader");
        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        writer.insert_for_test("/t", "ubuntu");

        // Warm up the connection with valid republishes.
        let mut scratch: Vec<MirrorRecord> = Vec::new();
        let mut connected = false;
        for _ in 0..40 {
            writer.republish();
            scratch.clear();
            if reader.drain(&mut scratch) > 0 {
                connected = true;
                break;
            }
        }
        assert!(connected, "the control connection never established");
        loop {
            scratch.clear();
            if reader.drain(&mut scratch) == 0 {
                break;
            }
        }

        // Send a hostile/garbage frame, then a valid one.
        assert!(writer.send_raw_for_test(&[0xDE, 0xAD, 0xBE, 0xEF]));
        writer.republish();
        // The malformed frame is counted + skipped; the valid record still lands.
        let mut got: Vec<MirrorRecord> = Vec::new();
        for _ in 0..10 {
            scratch.clear();
            reader.drain(&mut scratch);
            got.append(&mut scratch);
            if !got.is_empty() {
                break;
            }
        }
        assert_eq!(
            got,
            vec![MirrorRecord {
                topic: "/t".to_string(),
                origin_robot: "ubuntu".to_string(),
            }]
        );
        assert!(
            reader.malformed_count() >= 1,
            "the malformed frame was counted (Principle #3)"
        );
    }

    /// ROOT-1 e2e: TWO independent writer processes (two `MirrorRegistry`
    /// instances with distinct `writer_id`s) whose republishes are STAGGERED far
    /// apart in time. The gather must include BOTH — driven deterministically via
    /// the per-pass hook so writer B only republishes AFTER a set-stability
    /// heuristic would already have early-exited on writer A alone.
    /// Reverting `gather_reached_all_live_writers` to a set-stability-only exit
    /// returns just A's record, failing the both-present assertion.
    #[test]
    fn gather_includes_both_of_two_staggered_writers() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_two".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_a_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_a".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("writer a manager");
        let writer_b_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_b".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer b manager");

        // Two writers, distinct ids (minted per instance), inline seam (no pumps).
        let writer_a = MirrorRegistry::open(&writer_a_mgr.node).expect("writer a");
        let writer_b = MirrorRegistry::open(&writer_b_mgr.node).expect("writer b");
        assert_ne!(
            writer_a.writer_id(),
            writer_b.writer_id(),
            "independent writer instances have distinct ids"
        );
        writer_a.insert_for_test("/from_a", "robot_a");
        writer_b.insert_for_test("/from_b", "robot_b");

        // STAGGER via the gather hook: writer A republishes on early passes
        // (0..=3), writer B only from pass 8 onward. A set-stability heuristic
        // would exit ~pass 3-5 having heard ONLY A; the writer-count gate keeps
        // gathering until B is heard at pass 8+. Generous window so the deadline
        // never cuts B off (both writers' publishers are counted from the start,
        // so `live == 2` throughout).
        const B_STARTS_AT: usize = 8;
        let got = gather_from_node_inner(
            &reader_mgr.node,
            Duration::from_secs(5),
            // Production posture. Neither writer runs a pump (both are driven
            // by the hook), so nothing answers the ring — the stagger under
            // test is entirely the hook's.
            GatherDoorbell::Ring,
            |pass| {
                if pass <= 3 || pass >= B_STARTS_AT {
                    writer_a.republish();
                }
                if pass >= B_STARTS_AT {
                    writer_b.republish();
                }
            },
        )
        .expect("gather");

        // Hand oracle: BOTH mirrors present, deduped + sorted by topic. Missing
        // `/from_b` is exactly the phantom-LOCAL bug ROOT 1 fixes.
        assert_eq!(
            got.records,
            vec![
                MirrorRecord {
                    topic: "/from_a".to_string(),
                    origin_robot: "robot_a".to_string(),
                },
                MirrorRecord {
                    topic: "/from_b".to_string(),
                    origin_robot: "robot_b".to_string(),
                },
            ],
            "the gather must include BOTH staggered writers' mirrors"
        );
        // Having heard from every live writer, this answer is EVIDENCE.
        // The ANTI-TAUTOLOGY half of the silent-writer arm below, which drives
        // the same code to the opposite verdict — without this pair a
        // `completeness` hardcoded either way would pass one of them.
        assert_eq!(
            got.completeness,
            GatherCompleteness::Settled,
            "a gather that heard from both live writers settles the question"
        );
    }

    /// ROOT-1 crashed/silent-writer arm: a SILENT writer (its publisher port is
    /// counted but it never sends — models a crashed writer whose stale iceoryx2
    /// port lingers) alongside a LIVE writer. The gather returns the LIVE writer's
    /// record correctly (a stale sibling never hides a live mirror) but, unable to
    /// hear from every "live" publisher, pays the full `window` ceiling — the
    /// DOCUMENTED bound. Short window so the test stays fast; asserts both the
    /// correctness and the bound.
    #[test]
    fn silent_writer_gather_returns_live_record_and_pays_the_ceiling() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_silent".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let live_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_live".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("live writer manager");
        let silent_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_silent".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("silent writer manager");

        let live = MirrorRegistry::open(&live_mgr.node).expect("live writer");
        // The silent writer opens its publisher (so number_of_publishers counts it)
        // but NEVER registers/republishes — the stale-port model.
        let _silent = MirrorRegistry::open(&silent_mgr.node).expect("silent writer");
        live.insert_for_test("/live_topic", "live_robot");

        const WINDOW: Duration = Duration::from_millis(300);
        let started = Instant::now();
        let got = gather_from_node_inner(
            &reader_mgr.node,
            WINDOW,
            // Production posture. The silent writer holds a doorbell listener
            // (every registry does) but runs NO pump, so nothing answers the
            // ring on its behalf — which is the point: a stale port stays
            // unheard however loudly the reader asks.
            GatherDoorbell::Ring,
            |_pass| {
                // Only the live writer republishes; the silent one never does.
                live.republish();
            },
        )
        .expect("gather");
        let elapsed = started.elapsed();

        // Correctness: the live mirror surfaces despite the silent sibling.
        assert_eq!(
            got.records,
            vec![MirrorRecord {
                topic: "/live_topic".to_string(),
                origin_robot: "live_robot".to_string(),
            }],
            "the live writer's mirror is returned even with a silent sibling publisher"
        );
        // Documented bound: never hearing from the silent publisher, the gather
        // cannot early-exit and pays (most of) the window ceiling.
        assert!(
            elapsed >= WINDOW.mul_f64(0.8),
            "a silent/stale sibling publisher forces the full-window ceiling (elapsed {elapsed:?} \
             vs window {WINDOW:?})"
        );
        // And it says so. The records that DID arrive are correct — the
        // assertion above — but this gather cannot speak for the writer it never
        // heard, so it must not be read as a settled picture of the desk. The
        // counts are exact: two publishers live, one heard.
        assert_eq!(
            got.completeness,
            GatherCompleteness::Incomplete {
                live_writers: 2,
                writers_heard: 1,
            },
            "a gather that timed out on an unheard live writer reports itself INCOMPLETE, \
             naming both counts"
        );
    }

    /// THE shape every healthy desk is in most of the time: a
    /// LIVE registry writer holding ZERO records must be HEARD, so the gather
    /// SETTLES on an empty answer.
    ///
    /// This is `cerulion-netd` after its last mirror retires —
    /// `unregister_mirror_provenance` empties the record map and deliberately
    /// KEEPS the registry (hence the iceoryx2 publisher) alive for the process
    /// lifetime. Before the presence frame, `republish` sent nothing for an
    /// empty map, so such a writer was byte-identical on the wire to one the
    /// reader failed to hear: MEASURED through this exact public API,
    /// `Incomplete { live_writers: 1, writers_heard: 0 }` after the full window,
    /// permanently. Through the checked gather that stamps a FALSE `mirrors_established:
    /// false` into every bag recorded on that desk, warns on every recording,
    /// and pays the whole retry budget — a wrong verdict in a durable artifact,
    /// on a machine with nothing wrong with it.
    ///
    /// Driven through `register_mirror_provenance` + `unregister_mirror_provenance`
    /// rather than the test seams, because the SHAPE is the point: it is
    /// reached by ordinary use, not by a contrived state.
    #[test]
    fn a_live_writer_that_mirrors_nothing_is_heard_and_settles_the_gather() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_retired".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_retired".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        // Register then RETIRE — netd's refcount-0 teardown. The publisher stays.
        writer_mgr
            .register_mirror_provenance("/utlidar/cloud", "go2")
            .expect("register");
        assert!(
            writer_mgr
                .unregister_mirror_provenance("/utlidar/cloud")
                .expect("unregister"),
            "precondition: the mirror really was registered and is now retired"
        );

        let got = gather_from_node_checked(&reader_mgr.node, MIRROR_GATHER_WINDOW).expect("gather");

        assert!(
            got.records.is_empty(),
            "the retired mirror must not still be reported (got {:?})",
            got.records
        );
        assert_eq!(
            got.completeness,
            GatherCompleteness::Settled,
            "a live writer that mirrors NOTHING is still a writer that can be HEARD — its empty \
             answer is evidence, not a timeout. Incomplete here is the original defect: a \
             false `mirrors_established: false` in every bag this desk records"
        );
        assert!(got.completeness.is_settled());
    }

    /// THE mechanism pin: a gather ASKS, and is answered WITHOUT waiting
    /// for the writer's republish belt.
    ///
    /// # How this arm establishes that, and why a short window could not
    ///
    /// Running the gather under a window of `2/3` of
    /// [`MIRROR_REPUBLISH_INTERVAL`] and arguing that "the timer's first tick
    /// cannot land inside this window" is **FALSE reasoning**:
    /// it would hold only if the
    /// writer's belt started when the GATHER starts, and it does not. The pump is
    /// started by `register`, several statements earlier, so by the time the
    /// gather opens its subscriber the belt is already free-running with UNKNOWN
    /// PHASE and a tick can land anywhere in the window — ~67 % of phases on
    /// arithmetic alone. Such an arm normally passes because the doorbell
    /// round trip is microseconds and reliably WINS THAT RACE; under CI load the
    /// pump thread is not scheduled in time, the ring goes unanswered inside the
    /// window, and a belt tick settles the gather. Correct code, red gate.
    ///
    /// This arm removes the contender instead of out-running it. The belt is
    /// QUIESCED to 30 s before the pump exists, so inside the 5 s ceiling the
    /// writer's ONLY route to `inner.republish()` is a doorbell ring: both
    /// non-ring routes in `pump_loop` are gated on that belt (`Ok(None)` returns
    /// only after `timed_wait_one(30s)`; the `Err` arm sleeps 30 s first), and
    /// `register`'s inline `send_record` fires before the reader's subscriber
    /// exists on a service that requests no history. `Settled` is therefore
    /// ring-attributable BY CONSTRUCTION rather than by winning a race — the
    /// `wake_drain_e2e_test` pattern, where the load-bearing claim moved off a
    /// wall and onto a fact load cannot fake.
    ///
    /// Load cannot invert it in the surviving direction either: `timed_wait_one`
    /// is a LOWER bound, so coalescing and scheduler delay push the belt's first
    /// tick FURTHER outside the ceiling, never into it. The one premise load
    /// could break — setup taking so long that the belt could still fire — is
    /// ASSERTED, so the worst case is a loud self-explaining precondition
    /// failure, never an inverted verdict. No bound here is stated in units of
    /// [`MIRROR_REPUBLISH_INTERVAL`]; the ceilings are seconds-scale liveness
    /// bounds.
    ///
    /// Why this arm exists at all, when
    /// `a_live_writer_that_mirrors_nothing_is_heard_and_settles_the_gather`
    /// already asserts `Settled` over the production window: that test passes on
    /// a healthy desk EITHER WAY — the 150 ms timer comfortably fits a 600 ms
    /// window — so it cannot see the doorbell. It only fails where the timer is
    /// not 150 ms, which is the whole point: MEASURED under macOS
    /// background QoS (`taskpolicy -b`), a pump built from six chained 25 ms
    /// sleeps took 1100-1696 ms for a nominal 150 ms interval, so every gather
    /// on that desk timed out and stamped a false `mirrors_established: false`
    /// into its bag. A wall-clock assertion cannot pin the doorbell either (on a
    /// loaded runner a doorbell answer and a timer tick are not separable by
    /// duration: the load-sensitive timing class), which is why the oracle is the writer's
    /// OWN ring counter: a MEASUREMENT of "it spoke because it was asked".
    #[test]
    fn a_gather_is_answered_by_the_doorbell_not_by_the_republish_belt() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_doorbell".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_doorbell".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        // Quiesce the belt BEFORE the pump exists: the belt is read at spawn.
        let mut writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        const QUIESCED_BELT: Duration = Duration::from_secs(30);
        const GATHER_CEILING: Duration = Duration::from_secs(5);
        const _: () = assert!(
            QUIESCED_BELT.as_millis() >= GATHER_CEILING.as_millis() * 6,
            "the quiesced belt must dwarf the gather ceiling, or a timer tick could still answer \
             this gather and the arm would pin nothing"
        );
        assert!(
            writer.quiesce_republish_belt_for_test(QUIESCED_BELT),
            "this arm REQUIRES a doorbell and a not-yet-started pump: without the first, teardown \
             joins a thread parked for the belt; without the second the quiesce is a silent no-op"
        );

        let armed_at = Instant::now();
        // The presence-frame shape: a LIVE writer holding ZERO records (netd after its
        // last mirror retires). `register` is what starts the pump — and the
        // pump is what waits on the doorbell — so the production call order is
        // load-bearing, not incidental.
        writer.register("/utlidar/cloud", "go2");
        assert!(writer.unregister("/utlidar/cloud"), "precondition: retired");
        assert_eq!(writer.record_count(), 0, "precondition: nothing to mirror");
        assert!(
            writer.republish_pump_active(),
            "precondition: the pump — the thing that waits on the doorbell — is running"
        );
        let rings_before = writer.doorbell_rings_answered();
        assert_eq!(
            rings_before, 0,
            "precondition: nothing has rung this writer yet"
        );

        // The premise, MEASURED on THIS run rather than argued in a comment —
        // which is exactly what the deleted `const _` assert failed to be.
        assert!(
            armed_at.elapsed() + GATHER_CEILING < QUIESCED_BELT,
            "precondition: setup took {:?}, leaving under the quiesced belt before the gather's \
             deadline — the belt could interfere and this arm would pin nothing",
            armed_at.elapsed()
        );

        let settle_started = Instant::now();
        let got = gather_from_node_checked(&reader_mgr.node, GATHER_CEILING).expect("gather");
        let settle_wall = settle_started.elapsed();

        assert!(got.records.is_empty(), "got {:?}", got.records);
        assert_eq!(
            got.completeness,
            GatherCompleteness::Settled,
            "with the belt quiesced to {QUIESCED_BELT:?} the ONLY frame this writer can emit \
             inside {GATHER_CEILING:?} is a ring answer, so Settled IS the doorbell — and without \
             it this desk's every bag carries a false `mirrors_established: false`"
        );
        assert!(
            writer.doorbell_rings_answered() > rings_before,
            "the writer must have ANSWERED a ring (answered {}, was {rings_before}) — a settle \
             with no ring counted would mean the frame came from somewhere this arm does not model",
            writer.doorbell_rings_answered()
        );
        // Latency is EVIDENCE FOR A HUMAN, never a gate: load can only lengthen
        // a wall, and no wall here is stated in units of MIRROR_REPUBLISH_INTERVAL.
        eprintln!(
            "gather settled in {settle_wall:?} (belt quiesced to {QUIESCED_BELT:?}, \
             production belt {MIRROR_REPUBLISH_INTERVAL:?})"
        );
    }

    /// The seam's ANTI-INERT control: a QUIESCED writer is genuinely
    /// not heard on its belt.
    ///
    /// Without this, `ensure_pump_running` reading the const instead of
    /// `self.republish_interval` — the one change that makes the quiesce a
    /// no-op — restores a 150 ms free-running timer, and the arm
    /// above degenerates into the flaky race it is written to remove
    /// while still passing on an idle desk.
    ///
    /// It is the mirror image of
    /// `a_gather_that_never_rings_still_converges_on_the_writers_own_cadence`
    /// (production belt + silent reader ⇒ `Settled`); here the belt is quiesced
    /// and the same silent reader must get `Incomplete`. Nothing ever rings, so
    /// there is no residual ring race to lose.
    #[test]
    fn a_quiesced_writer_is_not_heard_on_its_belt() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_quiesced_belt".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_quiesced_belt".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let mut writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        const QUIESCED_BELT: Duration = Duration::from_secs(30);
        const WINDOW: Duration = Duration::from_secs(3);
        const _: () = assert!(
            QUIESCED_BELT.as_millis() >= WINDOW.as_millis() * 6,
            "the quiesced belt must dwarf this window, or the belt could speak and the control \
             would be measuring nothing"
        );
        assert!(
            writer.quiesce_republish_belt_for_test(QUIESCED_BELT),
            "precondition: a doorbell exists and no pump has started yet"
        );
        writer.register("/utlidar/cloud", "go2");
        assert!(writer.unregister("/utlidar/cloud"), "precondition: retired");
        assert!(
            writer.republish_pump_active(),
            "precondition: the pump is running"
        );

        // SILENT: nothing rings, so the belt is the only thing that could speak.
        let got =
            gather_from_node_inner(&reader_mgr.node, WINDOW, GatherDoorbell::Silent, |_pass| {})
                .expect("gather");

        assert!(
            matches!(
                got.completeness,
                GatherCompleteness::Incomplete {
                    live_writers,
                    writers_heard: 0
                } if live_writers >= 1
            ),
            "a writer whose belt is quiesced to {QUIESCED_BELT:?} must NOT be heard inside \
             {WINDOW:?} — if it is, the quiesce is inert and its sibling arm pins nothing (got \
             {:?})",
            got.completeness
        );
        assert_eq!(
            writer.doorbell_rings_answered(),
            0,
            "nothing rang: this arm's verdict is about the BELT and nothing else"
        );
    }

    /// The FALLBACK, kept real: with the reader SILENT — never ringing
    /// — a gather still converges on the writer's own republish cadence.
    ///
    /// The anti-inert half of the arm above. The doorbell makes the interval belt
    /// unnecessary in the common case, and an unexercised belt is how a fallback
    /// quietly rots into dead code — while it is exactly what carries a mixed
    /// deployment (a pre-doorbell writer has no listener; a reader that cannot
    /// mint a notifier sends no ring). Asserting the counter is ZERO is what
    /// makes this arm's `Settled` attributable to the timer and nothing else.
    ///
    /// The window is deliberately GENEROUS (many intervals). The property under
    /// test is "the belt still converges", not "within 600 ms" — and a ceiling
    /// tight enough to time the belt is exactly the bet the doorbell removed from
    /// production; it has no business being re-introduced in a test.
    #[test]
    fn a_gather_that_never_rings_still_converges_on_the_writers_own_cadence() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_silent_bell".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_silent_bell".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        writer.register("/utlidar/cloud", "go2");
        assert!(writer.unregister("/utlidar/cloud"), "precondition: retired");

        const WINDOW: Duration = Duration::from_secs(5);
        let got =
            gather_from_node_inner(&reader_mgr.node, WINDOW, GatherDoorbell::Silent, |_pass| {})
                .expect("gather");

        assert!(got.records.is_empty(), "got {:?}", got.records);
        assert_eq!(
            got.completeness,
            GatherCompleteness::Settled,
            "the periodic republish belt must still carry a gather that never rang — this is the \
             mixed-deployment path (an old writer has no listener) and the degrade path (a reader \
             that could not mint a notifier)"
        );
        assert_eq!(
            writer.doorbell_rings_answered(),
            0,
            "precondition + attribution: this gather never rang, so its Settled verdict is the \
             TIMER's work and nothing else"
        );
    }

    /// The doorbell's service name is the EVENT sibling of the registry
    /// — a DISTINCT name (iceoryx2 keys a service by name, so one name cannot
    /// carry two messaging patterns) that still ends in neither `/data` nor a
    /// shape `cerulion topic list` enumerates, and port budgets that follow the
    /// writer/reader counts rather than a hand-picked number.
    #[test]
    fn the_doorbell_is_an_event_sibling_that_topic_list_can_never_surface() {
        assert_ne!(
            MIRROR_REGISTRY_DOORBELL_SERVICE_NAME, MIRROR_REGISTRY_SERVICE_NAME,
            "the doorbell needs its OWN name: iceoryx2 refuses one name under two messaging \
             patterns"
        );
        assert!(
            !MIRROR_REGISTRY_DOORBELL_SERVICE_NAME.ends_with("/data"),
            "`topic list` enumerates `*/data` services; the doorbell must stay invisible to it"
        );
        assert!(
            MIRROR_REGISTRY_DOORBELL_SERVICE_NAME.ends_with("/event"),
            "follow the `{{name}}/event` convention graph topics use"
        );
        assert!(
            MIRROR_REGISTRY_DOORBELL_SERVICE_NAME.starts_with("/__cerulion/"),
            "the doorbell is framework control surface, under the reserved prefix"
        );
        // One listener per live WRITER (each pump waits on its own).
        assert_eq!(MIRROR_DOORBELL_MAX_LISTENERS, MIRROR_REGISTRY_MAX_WRITERS);
        // Every writer keeps a notifier (teardown wake) and every concurrent
        // reader mints one for its gather.
        assert_eq!(
            MIRROR_DOORBELL_MAX_NOTIFIERS,
            MIRROR_REGISTRY_MAX_WRITERS + MIRROR_REGISTRY_MAX_READERS
        );
    }

    /// The ZERO-PUBLISHER fast path settles — pinned
    /// DIRECTLY, on the verdict.
    ///
    /// `mirror_provenance_e2e_test::no_mirror_desk_gathers_empty_instantly`
    /// asserts the records and the latency but predates `GatherCompleteness`, so
    /// it asserts NO verdict: the whole `Settled` arm of the fast path was
    /// reachable by no assertion anywhere. It is the majority shipping path —
    /// every desk with no re-injector — and the one place an empty answer is
    /// authoritative for free.
    ///
    /// Its PAIR is `an_unheard_live_writer_...` below: same empty `records`, the
    /// opposite verdict, and the difference is the entire point of the type.
    #[test]
    fn a_desk_with_no_registry_writer_at_all_settles_immediately() {
        use crate::transport::{TransportConfig, TransportManager};

        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_empty_desk".to_string(),
                ..Default::default()
            },
            crate::testing::iceoryx_test_config(),
        )
        .expect("reader manager");

        // A deliberately LONG window, so any wall assertion here has an order
        // of magnitude of headroom rather than a hair's breadth.
        //
        // A ceiling of `MIRROR_GATHER_WINDOW / 2`
        // (300 ms) FLAKES under a parallel module run — a ceiling tight
        // enough to separate 0 ms from 600 ms is also tight enough for a loaded
        // runner to cross, which is the load-sensitive timing class. Load can only make this
        // path SLOWER, never faster, so the direction is safe (no false pass);
        // widening the gap removes the false FAILURE.
        //
        // What the wall assertion is worth: it does NOT
        // catch deletion of the zero-publisher fast path. With that
        // block deleted this test stays GREEN in 0.04 s, because the loop's own
        // `live == 0` arm breaks on the first pass and neither route contains a
        // wall sleep — the fast path saves syscalls, not seconds. The VERDICT
        // assertion below is what carries the load here; the wall is a cheap
        // guard against a future change that makes an empty desk wait.
        const LONG_WINDOW: Duration = Duration::from_secs(5);
        const FAST_PATH_CEILING: Duration = Duration::from_secs(1);
        let started = Instant::now();
        let got = gather_from_node_checked(&reader_mgr.node, LONG_WINDOW).expect("gather");
        let elapsed = started.elapsed();

        assert!(got.records.is_empty());
        // The load-INSENSITIVE assertion, and the one that matters.
        assert_eq!(
            got.completeness,
            GatherCompleteness::Settled,
            "no publisher at all is the one empty the service itself vouches for"
        );
        assert!(
            elapsed < FAST_PATH_CEILING,
            "an empty desk must not WAIT: this answer is available without listening, and this \
             run took {elapsed:?} of a {LONG_WINDOW:?} window"
        );
    }

    /// The presence frame's WIRE shape, and what an OLD reader
    /// does with it — stated as a test rather than as a promise in a comment.
    ///
    /// An older reader has only `decode_record`, which rejects an empty topic,
    /// so it counts one malformed frame per republish while the writer's map is
    /// empty. That cost is real and bounded, and it is the reason presence
    /// frames are sent ONLY in that state: a writer WITH records stays
    /// byte-identical on the wire to before presence frames existed.
    #[test]
    fn a_presence_frame_is_a_frame_new_readers_understand_and_old_ones_reject() {
        let bytes = encode_presence(0xABCD_EF01_2345_6789);
        assert_eq!(
            bytes.len(),
            MIRROR_HEADER_LEN,
            "a presence frame is the bare header — no topic, no robot"
        );
        assert_eq!(
            decode_frame(&bytes).expect("a new reader understands it"),
            MirrorFrame::Presence(0xABCD_EF01_2345_6789),
            "the writer identity is what the frame exists to carry"
        );
        // An OLD reader's decoder: rejected, and specifically as an empty topic.
        assert_eq!(
            decode_record(&bytes),
            Err(MirrorRecordError::EmptyTopic),
            "an older reader counts this malformed — the stated, bounded rollout cost"
        );
        // A real record still decodes as a RECORD through the new entry point,
        // so the presence check cannot have swallowed the ordinary path.
        let record = encode_record(7, "/tf", "go2").expect("encode");
        assert_eq!(
            decode_frame(&record).expect("record decodes"),
            MirrorFrame::Record(
                7,
                MirrorRecord {
                    topic: "/tf".to_string(),
                    origin_robot: "go2".to_string(),
                }
            )
        );
        // And `encode_record` can never MINT a presence-shaped frame, so the two
        // shapes cannot collide by accident.
        assert_eq!(
            encode_record(7, "", "go2"),
            Err(MirrorRecordError::EmptyTopic)
        );
        assert_eq!(
            encode_record(7, "/tf", ""),
            Err(MirrorRecordError::EmptyRobot)
        );
    }

    /// THE regression shape, over real transport: a live writer this
    /// reader never hears from at all. The gather comes back EMPTY — and the
    /// whole point is that this empty must NOT read as "no mirrors on this
    /// desk".
    ///
    /// This is the macOS-CI failure reproduced deterministically. There the
    /// writer WAS republishing and the reader simply lost the race under load
    /// (unreproducible on demand); here the writer holds a registered record and
    /// never republishes during the window, which drives the identical code path
    /// to the identical state — live publisher, nothing heard, deadline reached
    /// — without depending on scheduler starvation.
    ///
    /// Its PAIRS are the two arms ABOVE — `a_desk_with_no_registry_writer_at_all_settles_immediately`
    /// (no writer) and `a_live_writer_that_mirrors_nothing_is_heard_and_settles_the_gather`
    /// (a writer with nothing to say). All three produce an EMPTY `records`;
    /// only this one is an unknown, and that difference is the whole reason
    /// `GatherCompleteness` exists.
    ///
    /// The pairing deliberately names IN-MODULE arms rather than
    /// `mirror_provenance_e2e_test::no_mirror_desk_gathers_empty_instantly`,
    /// which predates the verdict and
    /// asserts only the records and the latency, so it pins no `Settled` at all
    /// and could not serve as the opposite half of anything.
    #[test]
    fn an_unheard_live_writer_yields_an_empty_gather_that_is_not_an_absence_claim() {
        use crate::transport::{TransportConfig, TransportManager};

        let root = crate::testing::iceoryx_test_config();
        let reader_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_reader_unheard".to_string(),
                ..Default::default()
            },
            root.clone(),
        )
        .expect("reader manager");
        let writer_mgr = TransportManager::init_for_test(
            TransportConfig {
                node_name: "mirror_writer_unheard".to_string(),
                ..Default::default()
            },
            root,
        )
        .expect("writer manager");

        // A writer holding a REAL registration that never reaches this reader.
        let writer = MirrorRegistry::open(&writer_mgr.node).expect("writer");
        writer.insert_for_test("/utlidar/cloud", "go2");
        assert_eq!(
            writer.record_count(),
            1,
            "precondition: the desk really does have a mirror to find"
        );

        const WINDOW: Duration = Duration::from_millis(200);
        // Production posture: the reader ASKS. The writer holds a listener but
        // runs no pump, so nobody answers — an unheard writer stays unheard
        // whether or not the reader has a doorbell, which is what makes this
        // a genuine UNKNOWN rather than an absence.
        let got =
            gather_from_node_inner(&reader_mgr.node, WINDOW, GatherDoorbell::Ring, |_pass| {})
                .expect("gather");

        assert!(
            got.records.is_empty(),
            "precondition: this gather heard nothing (got {:?})",
            got.records
        );
        assert_eq!(
            got.completeness,
            GatherCompleteness::Incomplete {
                live_writers: 1,
                writers_heard: 0,
            },
            "an empty gather taken while a writer was live is an UNKNOWN, not an absence — \
             reading it as 'no mirrors' is what recorded another robot's stream as local"
        );
        assert!(
            !got.completeness.is_settled(),
            "is_settled() is the question consumers ask; it must be false here"
        );
    }
}
