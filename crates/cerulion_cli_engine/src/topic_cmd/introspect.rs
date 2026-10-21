// SPDX-License-Identifier: AGPL-3.0-only
//! Topic discovery verbs: `topic type`, `topic find`, `topic bw` and the
//! typed `topic list -t` listing.
//!
//! The decision functions here are pure (oracle-tested below); the observing
//! entry points reuse the same subscriber, scope and schema-naming ladder
//! `topic info` uses.

use std::collections::{BTreeMap, VecDeque};
use std::io::Write;
use std::num::NonZeroUsize;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use super::{TopicScope, TopicSource};
use crate::error::{CliError, CliResult};

/// One received frame as `topic bw` sees it: the publisher's wire stamp and
/// the payload size in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BwFrame {
    /// The frame's wire `timestamp_ns` (the publisher's stamp, so a replayed
    /// stream measures the same bandwidth as the live one).
    pub timestamp_ns: u64,
    /// The payload size in bytes.
    pub size_bytes: u64,
}

/// What [`bandwidth`] computes over one window of frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BwStats {
    /// Bytes per second, rounded to the nearest byte.
    pub bytes_per_sec: u64,
    /// How many frames the window held.
    pub frames: usize,
    /// The mean payload size in bytes (rounded down).
    pub mean_size: u64,
    /// The smallest payload size in bytes.
    pub min_size: u64,
    /// The largest payload size in bytes.
    pub max_size: u64,
}

/// The bandwidth of a window of frames.
///
/// The rate is the bytes that arrived AFTER the earliest stamp divided by the
/// time from the earliest stamp to the latest one: `n` frames span `n - 1`
/// intervals, so a steady stream of `S` bytes every `T` seconds measures
/// exactly `S / T`. Every frame ON the earliest stamp opens the span and is
/// left out of the rate (two frames stamped together at the start are one
/// instant, not one interval). `None` when the window cannot give a rate: no
/// frame, one frame, or every frame on one stamp (no time span). Integer
/// arithmetic in `u128`, so the largest sizes and spans never overflow; a
/// rate above `u64::MAX` saturates.
pub fn bandwidth(window: &[BwFrame]) -> Option<BwStats> {
    let first_ns = window.iter().map(|f| f.timestamp_ns).min()?;
    let last_ns = window.iter().map(|f| f.timestamp_ns).max()?;
    let span_ns = u128::from(last_ns - first_ns);
    if span_ns == 0 {
        return None;
    }
    let total: u128 = window.iter().map(|f| u128::from(f.size_bytes)).sum();
    let after_first: u128 = window
        .iter()
        .filter(|f| f.timestamp_ns != first_ns)
        .map(|f| u128::from(f.size_bytes))
        .sum();
    let rate = (after_first * 1_000_000_000 + span_ns / 2) / span_ns;
    let frames = window.len();
    Some(BwStats {
        bytes_per_sec: u64::try_from(rate).unwrap_or(u64::MAX),
        frames,
        mean_size: u64::try_from(total / frames as u128).unwrap_or(u64::MAX),
        min_size: window.iter().map(|f| f.size_bytes).min().unwrap_or(0),
        max_size: window.iter().map(|f| f.size_bytes).max().unwrap_or(0),
    })
}

/// The decimal unit prefixes `topic bw` scales by (1 KB = 1000 B, as
/// `ros2 topic bw` prints them).
const UNIT_PREFIXES: [&str; 5] = ["", "K", "M", "G", "T"];

/// Format `value` bytes with `suffix` (`B` or `B/s`) on the largest decimal
/// unit that keeps the figure under 1000: `999 B/s`, `1.00 KB/s`,
/// `1.50 MB/s`. Below 1000 the figure is an exact integer; above, two
/// decimals rounded half up, and a value that rounds to 1000.00 moves up a
/// unit (`999999 B/s` is `1.00 MB/s`, never `1000.00 KB/s`). Terabytes is
/// the top unit.
pub fn format_bytes(value: u64, suffix: &str) -> String {
    if value < 1000 {
        return format!("{value} {suffix}");
    }
    let value = u128::from(value);
    let mut scale: u128 = 1;
    for (i, prefix) in UNIT_PREFIXES.iter().enumerate().skip(1) {
        scale *= 1000;
        let hundredths = (value * 100 + scale / 2) / scale;
        if hundredths < 100_000 || i == UNIT_PREFIXES.len() - 1 {
            return format!(
                "{}.{:02} {prefix}{suffix}",
                hundredths / 100,
                hundredths % 100
            );
        }
    }
    unreachable!("the loop returns on its last unit")
}

/// The report line `topic bw` prints for one window.
pub fn render_bw_line(stats: &BwStats) -> String {
    format!(
        "bandwidth: {}, mean: {}, min: {}, max: {}, window: {}",
        format_bytes(stats.bytes_per_sec, "B/s"),
        format_bytes(stats.mean_size, "B"),
        format_bytes(stats.min_size, "B"),
        format_bytes(stats.max_size, "B"),
        stats.frames
    )
}

/// A message type name as `topic find` takes it: the names a typed listing
/// may print for the type the user means.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeQuery {
    /// The name exactly as typed, which a workspace schema's YAML key or a
    /// robot's catalog may hold verbatim.
    as_written: String,
    /// For the ROS 2 form `package/msg/Type`, the Cerulion name
    /// `package/Type` it aliases.
    ros_alias: Option<String>,
}

impl TypeQuery {
    /// Does a typed listing that prints `name` show the type this query
    /// means?
    pub fn matches(&self, name: &str) -> bool {
        name == self.as_written || self.ros_alias.as_deref() == Some(name)
    }
}

/// Parse a message type name as `topic find` takes it.
///
/// A name matches the type a typed listing prints with exactly that text,
/// so the Cerulion form `geometry_msgs/Vector3` and a bare workspace `Type`
/// are taken as written. The ROS 2 form `package/msg/Type` also matches the
/// Cerulion name `package/Type`.
///
/// A workspace schema is named by its YAML key and a robot's catalog by the
/// text the robot sends, and neither is checked against a grammar:
/// `schema create` writes keys such as `1scan` or `Scan-2`, and a hand
/// written key may hold `/`. So every slash-separated part may be any
/// non-empty text with no control character, and only an empty part or a
/// control character, which no printed name holds, is a validation error
/// naming the input and the expected shape. A well-formed name no topic
/// carries is not an error: it finds nothing.
pub fn parse_type_name(input: &str) -> CliResult<TypeQuery> {
    if !input.split('/').all(is_name_part) {
        return Err(malformed_type(input));
    }
    let ros_alias = match input.split('/').collect::<Vec<_>>().as_slice() {
        [package, "msg", ty] => Some(format!("{package}/{ty}")),
        _ => None,
    };
    Ok(TypeQuery {
        as_written: input.to_string(),
        ros_alias,
    })
}

/// One part of a type name: non-empty, with no character that
/// [`super::sanitize_display`] would replace, since a printed name never
/// holds one.
fn is_name_part(part: &str) -> bool {
    !part.is_empty() && !part.chars().any(char::is_control)
}

fn malformed_type(input: &str) -> CliError {
    CliError::Validation(format!(
        "malformed type name '{}': expected <package>/<Type>, <package>/msg/<Type> or a \
         workspace <Type>, for example geometry_msgs/Vector3",
        super::sanitize_display(input)
    ))
}

/// What a typed listing knows about one topic's message type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopicType {
    /// The type's name (`package/Type`, or a package-less workspace
    /// schema's bare key), already safe to print.
    Named(String),
    /// The topic's schema hash names no built-in or workspace type.
    Unnamed(u64),
    /// The type is unknown: a local topic sent no frame in the wait, or a
    /// remote robot's catalog holds neither a name nor a hash for it.
    Unknown,
}

impl TopicType {
    /// The type `hash` names: the first walker in `walkers` that knows the
    /// hash gives the name, else [`TopicType::Unnamed`]. The name is
    /// sanitized for the terminal, since a walker also reads workspace and
    /// robot-served `.msg` text.
    pub fn of_hash(hash: u64, walkers: &[&cerulion_core::codegen::FrameWalker]) -> Self {
        walkers
            .iter()
            .find_map(|w| w.schema_name_for_hash(hash))
            .map_or(TopicType::Unnamed(hash), |name| {
                TopicType::Named(super::sanitize_display(name))
            })
    }

    /// How much this answer tells: a name beats a bare hash beats nothing.
    fn rank(&self) -> u8 {
        match self {
            TopicType::Named(_) => 2,
            TopicType::Unnamed(_) => 1,
            TopicType::Unknown => 0,
        }
    }
}

impl std::fmt::Display for TopicType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TopicType::Named(name) => write!(f, "[{name}]"),
            TopicType::Unnamed(hash) => write!(f, "[0x{hash:016x}]"),
            TopicType::Unknown => write!(f, "[unknown]"),
        }
    }
}

/// One topic with its type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypedTopic {
    /// The topic name.
    pub name: String,
    /// What is known about its type.
    pub ty: TopicType,
}

/// Render the LOCAL section of `topic list -t`: the section
/// `render_local_topics_section` prints (one renderer serves both), with
/// each row's type in brackets after the topic name
/// (`/robot/cmd  [geometry_msgs/Vector3]`). The topic name stays the first
/// token of the row; the `internal` marker of `--all` follows the type.
pub fn render_local_typed_topics_section(local: &[TypedTopic], show_all: bool) -> String {
    super::render_local_topic_rows(
        local
            .iter()
            .map(|t| (t.name.as_str(), Some(t.ty.to_string()))),
        show_all,
    )
}

/// Render the REMOTE section of `topic list -t`: the section `topic list`
/// prints (one renderer serves both), with each row's type in brackets after
/// the topic name and before any `● streaming` marker. A row `types` does not
/// name shows `[unknown]`.
pub fn render_remote_typed_topics_section(
    disc: &super::RemoteDiscovery,
    had_endpoints: bool,
    streaming: &[super::MirrorStreamRow],
    types: &BTreeMap<String, TopicType>,
) -> String {
    super::render_remote_topic_rows(disc, had_endpoints, streaming, &|topic| {
        Some(types.get(topic).unwrap_or(&TopicType::Unknown).to_string())
    })
}

/// The type of every topic the remote `catalogs` list, keyed by topic.
///
/// An entry's type is its catalog schema name when the robot sent one, else
/// its schema hash named by `walker` (or [`TopicType::Unnamed`]), else
/// [`TopicType::Unknown`]. When several robots list one topic, the most
/// informative answer wins (a name over a hash over nothing) and, between
/// equal answers, the robot whose name sorts first, so the result does not
/// depend on the order the replies arrived in.
///
/// A `streaming` mirror row is the one exception: it carries the messages of
/// its ORIGIN robot, so its topic takes that robot's catalog answer only, and
/// [`TopicType::Unknown`] when that catalog does not list it. Another robot
/// that publishes the same topic name says nothing about the mirror's type.
pub fn remote_topic_types(
    catalogs: &[cerulion_core::CatalogReply],
    streaming: &[super::MirrorStreamRow],
    walker: &cerulion_core::codegen::FrameWalker,
) -> BTreeMap<String, TopicType> {
    let mut by_robot: Vec<&cerulion_core::CatalogReply> = catalogs.iter().collect();
    by_robot.sort_by(|a, b| a.robot.cmp(&b.robot));
    let mut types: BTreeMap<String, TopicType> = BTreeMap::new();
    for entry in by_robot.iter().flat_map(|c| &c.entries) {
        let ty = catalog_entry_type(entry, walker);
        match types.get(&entry.topic) {
            Some(known) if known.rank() >= ty.rank() => {}
            _ => {
                types.insert(entry.topic.clone(), ty);
            }
        }
    }
    // In row order, so a topic two mirror rows share takes the robot of the
    // row the REMOTE section prints for it (the last).
    for row in streaming {
        let ty = catalogs
            .iter()
            .filter(|c| c.robot == row.robot)
            .flat_map(|c| &c.entries)
            .filter(|e| e.topic == row.topic)
            .map(|e| catalog_entry_type(e, walker))
            .fold(TopicType::Unknown, |best, ty| {
                if ty.rank() > best.rank() {
                    ty
                } else {
                    best
                }
            });
        types.insert(row.topic.clone(), ty);
    }
    types
}

/// The type one catalog entry names: its schema name when the robot sent
/// one, else its schema hash named by `walker` (or [`TopicType::Unnamed`]),
/// else [`TopicType::Unknown`].
fn catalog_entry_type(
    entry: &cerulion_core::CatalogEntry,
    walker: &cerulion_core::codegen::FrameWalker,
) -> TopicType {
    match (&entry.schema_name, entry.schema_hash) {
        (Some(name), _) => TopicType::Named(super::sanitize_display(name)),
        (None, Some(hash)) => TopicType::of_hash(hash, &[walker]),
        (None, None) => TopicType::Unknown,
    }
}

/// Every remote topic whose type `wanted` matches ([`TypeQuery::matches`]),
/// sorted once each. Each robot's entry is matched on its own, so a topic
/// two robots publish with different types is found by either type (the
/// one-type-per-topic map of [`remote_topic_types`] serves the listing only).
///
/// A topic a `streaming` mirror row carries is matched only through its
/// origin robot's catalog, as the listing types it: another robot that
/// publishes the same topic name says nothing about the mirror's type.
pub fn remote_topics_of_type(
    catalogs: &[cerulion_core::CatalogReply],
    streaming: &[super::MirrorStreamRow],
    walker: &cerulion_core::codegen::FrameWalker,
    wanted: &TypeQuery,
) -> Vec<String> {
    let counts = |robot: &str, topic: &str| {
        let mut origins = streaming.iter().filter(|m| m.topic == topic).peekable();
        origins.peek().is_none() || origins.any(|m| m.robot == robot)
    };
    let mut found: Vec<String> = catalogs
        .iter()
        .flat_map(|c| c.entries.iter().map(move |e| (c.robot.as_str(), e)))
        .filter(|(robot, e)| counts(robot, &e.topic))
        .filter(
            |(_, e)| matches!(catalog_entry_type(e, walker), TopicType::Named(n) if wanted.matches(&n)),
        )
        .map(|(_, e)| e.topic.clone())
        .collect();
    found.sort_unstable();
    found.dedup();
    found
}

/// The stderr note `topic find` gives for the mirrored topics it could not
/// search, else `None`.
///
/// A mirror (a remote robot's topic re-injected into local shared memory) is
/// a REMOTE topic: `topic list` files it under its origin robot, and its type
/// comes from that robot's catalog, never from a local read (that read would
/// treat it as a local producer, and under a local scope it is refused). A
/// mirror is searched only when its ORIGIN robot's catalog lists its topic:
/// another robot that publishes the same topic name says nothing about the
/// mirror's type. A mirror with no such catalog entry (the scope is local,
/// the remote query failed, or its robot did not answer) is not searched, and
/// the note names how many, so the result is never silently short.
pub fn unsearched_mirrors_note(
    streaming: &[super::MirrorStreamRow],
    catalogs: &[cerulion_core::CatalogReply],
) -> Option<String> {
    let unsearched = streaming
        .iter()
        .filter(|m| {
            !catalogs
                .iter()
                .filter(|c| c.robot == m.robot)
                .flat_map(|c| &c.entries)
                .any(|e| e.topic == m.topic)
        })
        .count();
    (unsearched > 0).then(|| {
        format!(
            "remote: {unsearched} mirrored topic(s) not searched: their type comes from their \
             robot's catalog, which this search did not get (`topic list` shows them)\n"
        )
    })
}

/// The topics whose type `wanted` matches ([`TypeQuery::matches`]), sorted
/// by name with duplicates removed. A topic with an unknown or unnamed type
/// never matches.
pub fn topics_of_type<'a>(topics: &'a [TypedTopic], wanted: &TypeQuery) -> Vec<&'a str> {
    let mut found: Vec<&str> = topics
        .iter()
        .filter(|t| matches!(&t.ty, TopicType::Named(name) if wanted.matches(name)))
        .map(|t| t.name.as_str())
        .collect();
    found.sort_unstable();
    found.dedup();
    found
}

/// The error `topic type` gives when the topic sent no frame within `wait`.
pub fn silent_topic_error(topic: &str, wait: Duration) -> CliError {
    CliError::Validation(format!(
        "topic '{}' sent no message within {}s, so its type is unknown; retry while its \
         publisher is sending",
        super::sanitize_display(topic),
        wait.as_secs()
    ))
}

/// The error `topic type` gives when no known type names the topic's hash.
pub fn unnamed_hash_error(topic: &str, hash: u64) -> CliError {
    CliError::Validation(format!(
        "topic '{}' carries schema hash 0x{hash:016x}, which no built-in or workspace type \
         names; add its `.msg` under `schemas/` to name it",
        super::sanitize_display(topic)
    ))
}

/// The error a receive that keeps failing gives (the `topic hz` text).
pub fn receive_failed_error(topic: &str, error: &dyn std::fmt::Display) -> CliError {
    CliError::Validation(format!(
        "receive failed on topic '{}': {error}",
        super::sanitize_display(topic)
    ))
}

/// How long `topic type` waits for a LOCAL topic's first frame (the
/// `topic info` wait).
const LOCAL_FRAME_WAIT: Duration = Duration::from_secs(1);

/// How long `topic type` waits for a REMOTE topic's first frame: the demand
/// handshake comes first (the `topic info` wait).
const REMOTE_FRAME_WAIT: Duration = Duration::from_secs(3);

/// The one shared wait a typed listing gives every local topic for its first
/// frame. Every subscriber opens before the wait starts, so a listing of many
/// topics costs about one wait, not one per topic.
pub const TYPED_LISTING_WAIT: Duration = Duration::from_secs(1);

/// The schema hash of the first frame `subscriber` delivers within `wait`,
/// `Ok(None)` when none arrived.
///
/// One `wait_for_message` can return early with no frame (an event that
/// carried no deliverable sample), so the wait repeats until a frame or the
/// deadline, paced by the observer starvation floor so an event flood cannot
/// spin a core. A failed wait is retried within the observers' consecutive
/// error budget; a failure that outlasts the budget, or that ends the wait,
/// is returned, never read as silence.
fn first_schema_hash(
    subscriber: &cerulion_core::CerulionSubscriber,
    wait: Duration,
) -> Result<Option<u64>, cerulion_core::TransportError> {
    let deadline = Instant::now() + wait;
    let mut hash = None;
    let mut idle_churn: u32 = 0;
    let mut consecutive_wait_errors: u32 = 0;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let started = Instant::now();
        let received = subscriber.wait_for_message(remaining, |msg| {
            hash.get_or_insert(msg.header().schema_hash);
        });
        if hash.is_some() {
            return Ok(hash);
        }
        let action =
            super::decide_wait_outcome(received.is_ok(), true, &mut consecutive_wait_errors);
        let delivered = match received {
            Ok(delivered) => delivered,
            Err(e) if remaining.is_zero() || action == super::WaitOutcomeAction::Fail => {
                return Err(e)
            }
            Err(_) => continue,
        };
        if remaining.is_zero() {
            return Ok(None);
        }
        if let Some(pause) =
            super::decide_observer_pacing(delivered, started.elapsed(), remaining, &mut idle_churn)
        {
            super::note_pacing_engaged();
            std::thread::sleep(pause.min(remaining)); // ALLOW: starvation floor, engaged ONLY on a progress-free early-returning wait (the `topic hz` rule), so it bounds CPU on event churn and is not a poll.
        }
    }
}

/// Open `topic` for a typed listing the way `topic info` opens a genuine
/// local producer: under a local scope the local-observation lease is taken
/// first, and a topic that turns out to be a network mirror after the open
/// is dropped (it is not a local topic). `None` when the topic cannot be read
/// as a genuine local one; the reason is logged at `debug`.
fn open_genuine_local(
    transport: &cerulion_core::TransportManager,
    topic: &str,
    local_only: bool,
) -> Option<super::ScopedSubscriber> {
    let guard = if local_only {
        match transport.acquire_local_topic_lease(topic) {
            Ok(lease) => super::ObservationGuard::LocalLease {
                _lease: Box::new(lease),
            },
            Err(e) => {
                tracing::debug!(topic = %topic, error = %e, "typed listing: no local lease for the topic");
                return None;
            }
        }
    } else {
        super::ObservationGuard::None
    };
    let subscriber = match transport.create_subscriber_open_only(topic) {
        Ok(subscriber) => super::ScopedSubscriber {
            subscriber,
            _guard: guard,
        },
        Err(e) => {
            tracing::debug!(topic = %topic, error = %e, "typed listing: the topic did not open");
            return None;
        }
    };
    match transport.is_network_mirror(topic) {
        Ok(false) => Some(subscriber),
        Ok(true) => {
            tracing::debug!(topic = %topic, "typed listing: the topic became a network mirror");
            None
        }
        Err(e) => {
            tracing::debug!(topic = %topic, error = %e, "typed listing: the mirror check failed");
            None
        }
    }
}

/// Which internal topics a typed listing reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InternalTopics {
    /// Read them too (`topic find`, `topic list -t --all`).
    Read,
    /// Skip them: they are hidden from the output, so reading them is wasted
    /// work (`topic list -t` without `--all`). A skipped topic is
    /// [`TopicType::Unknown`].
    Skip,
}

/// The type of every genuine LOCAL topic in `local` (the listing after the
/// mirror fold), in the order given. Every subscriber opens first; then each
/// topic gets what is left of one shared [`TYPED_LISTING_WAIT`] for a frame.
/// A frame's schema hash is named by `walker`; this listing opens no network.
///
/// A topic is opened as a genuine local producer (see `local_only` and the
/// mirror recheck of `topic info`). A topic that cannot be opened that way,
/// that sends no frame in the wait, or whose receive fails is
/// [`TopicType::Unknown`] (a failed receive is logged at `warn`); a hash
/// `walker` does not name is [`TopicType::Unnamed`].
pub fn type_local_topics(
    local: &[super::TopicInfo],
    walker: &cerulion_core::codegen::FrameWalker,
    local_only: bool,
    internal: InternalTopics,
) -> CliResult<Vec<TypedTopic>> {
    let transport = cerulion_core::TransportManager::get_or_init()?;
    let subscribers: Vec<Option<super::ScopedSubscriber>> = local
        .iter()
        .map(|t| {
            let skip = internal == InternalTopics::Skip && super::is_internal_topic(&t.name);
            if skip {
                None
            } else {
                open_genuine_local(&transport, &t.name, local_only)
            }
        })
        .collect();
    let deadline = Instant::now() + TYPED_LISTING_WAIT;
    Ok(local
        .iter()
        .zip(&subscribers)
        .map(|(topic, subscriber)| {
            let hash = subscriber.as_ref().and_then(|s| {
                match first_schema_hash(s, deadline.saturating_duration_since(Instant::now())) {
                    Ok(hash) => hash,
                    Err(e) => {
                        tracing::warn!(topic = %topic.name, error = %e, "typed listing: receive failed, so the type is unknown");
                        None
                    }
                }
            });
            TypedTopic {
                name: topic.name.clone(),
                ty: hash.map_or(TopicType::Unknown, |h| TopicType::of_hash(h, &[walker])),
            }
        })
        .collect())
}

/// Whether `topic list` prints each row's type (`-t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypeColumn {
    /// Plain `topic list`: the topic name alone (and its markers).
    Hidden,
    /// `topic list -t`: each row's type after its name.
    Shown,
}

/// `cerulion topic list [-t]`: print the listing to `out`, and the one
/// remote-failure line to `err`. One flow serves the plain and the typed
/// listing, so the two differ only in the type column.
///
/// LOCAL topics print FIRST, before the remote query starts, so a slow or
/// failing remote query never delays or hides them. A local service that is
/// a MIRROR of a remote robot (its name is in the `/__cerulion/mirrors`
/// provenance registry) is not a second LOCAL topic: it folds into the
/// REMOTE section as a `● streaming` row attributed to its origin robot.
/// Internal topics are hidden from the LOCAL section and counted unless
/// `show_all`; REMOTE rows are never filtered.
///
/// `remote` is `None` under local scope (`--local`, `CERULION_NETWORK=off`),
/// which skips the query. A failed remote query is ONE loud stderr line and
/// the verb still succeeds (the local list already printed): never a
/// silently empty section, never a fake success. The mirror rows are local
/// knowledge, so they print even with no query or a failed one.
///
/// With [`TypeColumn::Shown`], a LOCAL row is typed from one frame (see
/// [`type_local_topics`]; hidden internal topics are not read) and a REMOTE
/// row from the robots' catalogs, a mirror row from its origin robot's only
/// ([`remote_topic_types`]); a row with no remote answer shows `[unknown]`.
pub fn print_topic_list(
    show_all: bool,
    types: TypeColumn,
    remote: Option<super::RemoteTopicsOptions>,
    scan: bool,
    schemas_dir: Option<&Path>,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> CliResult<()> {
    let mirrors = super::gather_mirror_identities()?;
    let (local, streaming) = super::partition_local_topics(super::topic_list()?, &mirrors);
    let shown = types == TypeColumn::Shown;
    let walker = shown.then(|| super::local_walker_from_workspace(schemas_dir));
    let local_section = match &walker {
        Some(walker) => {
            let internal = if show_all {
                InternalTopics::Read
            } else {
                InternalTopics::Skip
            };
            let typed = type_local_topics(&local, walker, remote.is_none(), internal)?;
            render_local_typed_topics_section(&typed, show_all)
        }
        None => super::render_local_topics_section(&local, show_all),
    };
    write!(out, "{local_section}")?;
    out.flush()?;
    let remote_section = |disc: &super::RemoteDiscovery,
                          had_endpoints: bool,
                          catalogs: &[cerulion_core::CatalogReply]| {
        match &walker {
            Some(walker) => render_remote_typed_topics_section(
                disc,
                had_endpoints,
                &streaming,
                &remote_topic_types(catalogs, &streaming, walker),
            ),
            None => {
                super::render_remote_topics_section_with_mirrors(disc, had_endpoints, &streaming)
            }
        }
    };
    if let Some(opts) = remote {
        // `--scan` is the sole producer of a `true` scan, so the subnet
        // sweep never runs on a default listing.
        match super::query_remote_topics_and_catalogs(&opts, scan) {
            Ok((disc, catalogs)) => {
                write!(
                    out,
                    "{}",
                    remote_section(&disc, opts.has_endpoints(), &catalogs.replies)
                )?;
                return Ok(());
            }
            Err(e) => write!(err, "{}", super::render_remote_discovery_unavailable(&e))?,
        }
    }
    if !streaming.is_empty() {
        write!(
            out,
            "{}",
            remote_section(&super::RemoteDiscovery::empty(), false, &[])
        )?;
    }
    Ok(())
}

/// What `topic find` found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FoundTopics {
    /// The topics of the type, local and remote together, sorted once each.
    pub topics: Vec<String>,
    /// The `topic list` stderr line when the remote query failed (the local
    /// result still stands), else `None`.
    pub remote_unavailable: Option<String>,
    /// The [`unsearched_mirrors_note`] when a mirrored topic could not be
    /// searched, else `None`.
    pub unsearched_mirrors: Option<String>,
    /// The [`unsearched_robots_note`] when a robot's catalog could not be
    /// read, else `None`.
    pub unsearched_robots: Option<String>,
}

/// Every robot `disc` found: the announce identities the catalog gather
/// asks, and the ROBOTS rows, which also hold an mDNS-only peer the gather
/// never asks.
pub fn discovered_robots(disc: &super::RemoteDiscovery) -> Vec<String> {
    disc.announce_entries
        .iter()
        .map(|(robot, _)| robot.clone())
        .chain(disc.robots.iter().map(|row| row.robot.clone()))
        .collect()
}

/// The stderr note `topic find` gives for the robots it could not search,
/// else `None`.
///
/// Each robot in `discovered` (every robot discovery found, an mDNS-only
/// peer the catalog gather does not ask included) must have served a
/// catalog in `gather`: a robot never asked, one whose request timed out or
/// got no decodable answer, one that refused to serve its catalog (a reply
/// carrying an error, which holds no entries), and one whose worker panicked
/// in this binary say nothing about their topics. The note names each one so
/// the result is never silently short.
pub fn unsearched_robots_note<'a>(
    discovered: impl IntoIterator<Item = &'a str>,
    gather: &'a super::CatalogGather,
) -> Option<String> {
    let (served, refused): (Vec<_>, Vec<_>) =
        gather.replies.iter().partition(|c| c.error.is_none());
    let answered: std::collections::BTreeSet<&str> =
        served.iter().map(|c| c.robot.as_str()).collect();
    let unsearched: std::collections::BTreeSet<String> = discovered
        .into_iter()
        .chain(gather.panicked.iter().map(String::as_str))
        .chain(refused.iter().map(|c| c.robot.as_str()))
        .filter(|robot| !answered.contains(robot))
        .map(super::sanitize_display)
        .collect();
    (!unsearched.is_empty()).then(|| {
        format!(
            "remote: {} robot(s) not searched: no catalog from {}, so their topics of this \
             type are not listed\n",
            unsearched.len(),
            unsearched.into_iter().collect::<Vec<_>>().join(", ")
        )
    })
}

/// `cerulion topic find <type>`: every topic whose type is `type_name`
/// (`package/Type`, `package/msg/Type` or a bare workspace `Type`), sorted.
///
/// Genuine LOCAL topics are typed from one frame each; unless the scope is
/// local, the remote robots `topic list` discovers are asked too and each
/// robot's topics typed from its own catalog ([`remote_topics_of_type`]). A
/// mirrored topic is searched through its origin robot's catalog only; one
/// that catalog does not list is counted in [`FoundTopics::unsearched_mirrors`]. An unknown type
/// is an empty list; a malformed name is an error before any topic is read.
/// `connect` and `listen` add locators to the discovery session, as `topic
/// list --connect` and `--listen` do, for a robot scouting cannot find.
pub fn topic_find(
    type_name: &str,
    schemas_dir: Option<&Path>,
    scope: TopicScope,
    connect: Vec<String>,
    listen: Vec<String>,
) -> CliResult<FoundTopics> {
    let wanted = parse_type_name(type_name)?;
    let mirrors = super::gather_mirror_identities()?;
    let (local, streaming) = super::partition_local_topics(super::topic_list()?, &mirrors);
    let walker = super::local_walker_from_workspace(schemas_dir);
    let remote = super::remote_discovery_options(scope == TopicScope::Local, connect, listen);
    let typed = type_local_topics(&local, &walker, remote.is_none(), InternalTopics::Read)?;
    let mut topics: Vec<String> = topics_of_type(&typed, &wanted)
        .into_iter()
        .map(str::to_string)
        .collect();
    let mut remote_unavailable = None;
    let mut gather = super::CatalogGather::default();
    let mut discovered: Vec<String> = Vec::new();
    if let Some(opts) = remote {
        match super::query_remote_topics_and_catalogs(&opts, false) {
            Ok((disc, gathered)) => {
                discovered = discovered_robots(&disc);
                gather = gathered;
            }
            Err(e) => {
                remote_unavailable = Some(super::render_remote_discovery_unavailable(&e));
            }
        }
    }
    topics.extend(remote_topics_of_type(
        &gather.replies,
        &streaming,
        &walker,
        &wanted,
    ));
    topics.sort_unstable();
    topics.dedup();
    Ok(FoundTopics {
        topics: topics.iter().map(|t| super::sanitize_display(t)).collect(),
        remote_unavailable,
        unsearched_mirrors: unsearched_mirrors_note(&streaming, &gather.replies),
        unsearched_robots: unsearched_robots_note(discovered.iter().map(String::as_str), &gather),
    })
}

/// `cerulion topic type <topic>`: the name of the topic's message type,
/// `package/Type` or a package-less workspace schema's bare key.
///
/// The topic is opened exactly as `topic info` opens it (a local producer
/// directly; a remote robot's topic through the network daemon unless the
/// scope is local) and its first frame's schema hash is named by the same
/// ladder: the local walker, then the remote one. For a local topic the remote
/// step is the `topic echo` schema lookup ([`super::resolve_remote_walker_for_topic`]),
/// run only when the local walker cannot name the hash and the scope is not
/// local, so it can add that lookup's round trip (a few seconds against a cold
/// daemon) after the one-second frame wait. A missing topic is the
/// `topic info` not-found error. A topic that sends no frame in the wait
/// ([`silent_topic_error`]), whose receive keeps failing
/// ([`receive_failed_error`]) or whose hash no known type names
/// ([`unnamed_hash_error`]) is an error that names the topic.
pub fn topic_type_with_scope(
    topic: &str,
    schemas_dir: Option<&Path>,
    scope: TopicScope,
) -> CliResult<String> {
    let (_transport, subscriber, source) =
        super::ensure_topic_available(topic, schemas_dir, None, scope)?;
    let wait = match &source {
        TopicSource::Remote { .. } => REMOTE_FRAME_WAIT,
        TopicSource::Local => LOCAL_FRAME_WAIT,
    };
    let hash = match first_schema_hash(&subscriber, wait) {
        Ok(Some(hash)) => hash,
        Ok(None) => return Err(silent_topic_error(topic, wait)),
        Err(e) => return Err(receive_failed_error(topic, &e)),
    };
    let local_walker = super::local_walker_from_workspace(schemas_dir);
    let mut ty = TopicType::of_hash(hash, &[&local_walker]);
    if matches!(ty, TopicType::Unnamed(_)) {
        ty = match &source {
            TopicSource::Remote { walker, .. } => TopicType::of_hash(hash, &[walker]),
            TopicSource::Local if scope == TopicScope::Local => ty,
            TopicSource::Local => super::resolve_remote_walker_for_topic(topic)
                .map_or(ty, |(_robot, walker)| TopicType::of_hash(hash, &[&walker])),
        };
    }
    match ty {
        TopicType::Named(name) => Ok(name),
        TopicType::Unnamed(_) | TopicType::Unknown => Err(unnamed_hash_error(topic, hash)),
    }
}

/// The line `topic bw` prints at the end of one report interval, given the
/// frames in `ring` and how many of them arrived in this interval.
///
/// An interval with no new frame prints the silence line and KEEPS the ring:
/// the figure always covers the newest frames of the window, as
/// `ros2 topic bw` measures it, so a publisher slower than one report
/// interval still collects the two frames a rate needs. Otherwise the line is
/// [`render_bw_line`] over the ring, or a waiting line while the ring holds
/// too few frames for a rate.
pub fn bw_report_line(
    ring: &mut VecDeque<BwFrame>,
    arrived_this_report: usize,
    silent_for: Duration,
    source_desc: &str,
) -> String {
    if arrived_this_report == 0 {
        return format!(
            "no messages received: attached to {source_desc}; 0 frames in {:.0}s",
            silent_for.as_secs_f64()
        );
    }
    match bandwidth(ring.make_contiguous()) {
        Some(stats) => render_bw_line(&stats),
        None => format!("waiting for more messages... (received {})", ring.len()),
    }
}

/// `cerulion topic bw <topic>`: print the topic's bandwidth once a second
/// over the last `window` frames until `running` clears.
///
/// The topic is opened as `topic hz` opens it, and the loop shares its wait,
/// retry and pacing rules. Each frame adds its wire stamp and payload size to
/// a ring allocated once before the loop, so the receive callback never
/// allocates. A second with no new frame prints a silence line instead of
/// repeating the last figure, and keeps the window (see [`bw_report_line`]).
/// A closed output (a reader that left the pipe) ends the command cleanly;
/// any other write failure is returned.
pub fn topic_bw_with_scope(
    topic: &str,
    schemas_dir: Option<&Path>,
    running: Arc<AtomicBool>,
    writer: &mut dyn Write,
    scope: TopicScope,
    window: NonZeroUsize,
) -> CliResult<()> {
    let (_transport, subscriber, source) =
        match super::ensure_topic_available(topic, schemas_dir, Some(running.as_ref()), scope) {
            Ok(v) => v,
            Err(_) if super::interrupted_before_observing(Some(running.as_ref())) => return Ok(()),
            Err(e) => return Err(e),
        };
    let source_desc = super::observer_source_description(&source);
    let window = window.get();
    // The one allocation of the loop: the full window, before any frame.
    let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(window);
    let mut arrived_this_report = 0usize;
    let mut last_report = Instant::now();
    let mut silence_since = Instant::now();
    let mut consecutive_wait_errors: u32 = 0;
    let mut idle_churn: u32 = 0;

    while running.load(Ordering::Relaxed) {
        let until_report = super::HZ_REPORT_INTERVAL.saturating_sub(last_report.elapsed());
        let wait = until_report.min(super::OBSERVER_HEARTBEAT);
        let wait_started = Instant::now();
        let mut delivered = 0usize;
        let received = subscriber.wait_for_message(wait, |msg| {
            if ring.len() == window {
                ring.pop_front();
            }
            ring.push_back(BwFrame {
                timestamp_ns: msg.header().timestamp_ns,
                size_bytes: msg.payload().len() as u64,
            });
            delivered += 1;
        });
        // Count what reached the ring before any error path: a receive can
        // deliver frames and then fail, and those frames are in the window.
        if delivered > 0 {
            arrived_this_report += delivered;
            silence_since = Instant::now();
        }
        let action = super::decide_wait_outcome(
            received.is_ok(),
            running.load(Ordering::Relaxed),
            &mut consecutive_wait_errors,
        );
        if let Err(e) = received {
            match action {
                super::WaitOutcomeAction::ExitCleanly => break,
                super::WaitOutcomeAction::Continue => continue,
                super::WaitOutcomeAction::Fail => {
                    return Err(receive_failed_error(topic, &e));
                }
            }
        }
        if let Some(pause) =
            super::decide_observer_pacing(delivered, wait_started.elapsed(), wait, &mut idle_churn)
        {
            super::note_pacing_engaged();
            std::thread::sleep(pause); // ALLOW: starvation floor, engaged ONLY on a progress-free early-returning wait (the `topic hz` rule), so it bounds CPU on event churn and is not a poll.
        }
        if last_report.elapsed() >= super::HZ_REPORT_INTERVAL {
            let line = bw_report_line(
                &mut ring,
                arrived_this_report,
                silence_since.elapsed(),
                &source_desc,
            );
            if write_report_line(writer, &line)? == Output::Closed {
                break;
            }
            arrived_this_report = 0;
            last_report = Instant::now();
        }
    }
    Ok(())
}

/// Whether an observer's output can still take lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Output {
    /// The line was written.
    Open,
    /// The reader closed the pipe, so the observer stops.
    Closed,
}

/// Write one report line and flush it, so a reader sees each report as it
/// is made. A broken pipe is [`Output::Closed`]; any other failure is
/// returned.
fn write_report_line(writer: &mut dyn Write, line: &str) -> CliResult<Output> {
    match writeln!(writer, "{line}").and_then(|()| writer.flush()) {
        Ok(()) => Ok(Output::Open),
        Err(e) if e.kind() == std::io::ErrorKind::BrokenPipe => Ok(Output::Closed),
        Err(e) => Err(e.into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(timestamp_ns: u64, size_bytes: u64) -> BwFrame {
        BwFrame {
            timestamp_ns,
            size_bytes,
        }
    }

    const SEC: u64 = 1_000_000_000;

    #[test]
    fn an_empty_window_has_no_bandwidth() {
        assert_eq!(bandwidth(&[]), None);
    }

    #[test]
    fn one_frame_has_no_bandwidth() {
        assert_eq!(bandwidth(&[frame(5, 100)]), None);
    }

    #[test]
    fn frames_on_one_stamp_have_no_bandwidth() {
        assert_eq!(bandwidth(&[frame(7, 10), frame(7, 20)]), None);
    }

    #[test]
    fn a_steady_rate_measures_size_over_period() {
        // 24 bytes every 50 ms is 480 B/s.
        let window: Vec<BwFrame> = (0..10).map(|i| frame(i * 50_000_000, 24)).collect();
        assert_eq!(
            bandwidth(&window),
            Some(BwStats {
                bytes_per_sec: 480,
                frames: 10,
                mean_size: 24,
                min_size: 24,
                max_size: 24,
            })
        );
    }

    #[test]
    fn a_burst_counts_every_byte_after_the_first_frame() {
        // Three frames in the first 2 ms, then one more at 1 s:
        // 200 + 300 + 400 bytes after the first over 1 s.
        let window = [
            frame(0, 100),
            frame(1_000_000, 200),
            frame(2_000_000, 300),
            frame(SEC, 400),
        ];
        let stats = bandwidth(&window).unwrap();
        assert_eq!(stats.bytes_per_sec, 900);
        assert_eq!(stats.mean_size, 250);
        assert_eq!(stats.min_size, 100);
        assert_eq!(stats.max_size, 400);
    }

    #[test]
    fn the_earliest_stamp_starts_the_span_whatever_the_arrival_order() {
        let window = [frame(SEC, 400), frame(0, 100), frame(2 * SEC, 600)];
        assert_eq!(bandwidth(&window).unwrap().bytes_per_sec, 500);
    }

    #[test]
    fn a_rate_rounds_to_the_nearest_byte() {
        // 2 bytes over 3 s is 0.67 B/s, which rounds to 1.
        assert_eq!(
            bandwidth(&[frame(0, 1), frame(3 * SEC, 2)])
                .unwrap()
                .bytes_per_sec,
            1
        );
        // 1 byte over 3 s is 0.33 B/s, which rounds to 0.
        assert_eq!(
            bandwidth(&[frame(0, 1), frame(3 * SEC, 1)])
                .unwrap()
                .bytes_per_sec,
            0
        );
    }

    #[test]
    fn the_largest_sizes_and_spans_do_not_overflow() {
        // u64::MAX bytes in 1 ns saturates instead of wrapping.
        let huge = bandwidth(&[frame(0, u64::MAX), frame(1, u64::MAX)]).unwrap();
        assert_eq!(huge.bytes_per_sec, u64::MAX);
        assert_eq!(huge.mean_size, u64::MAX);
        // u64::MAX bytes over u64::MAX ns is 1e9 B/s.
        let long = bandwidth(&[frame(0, 0), frame(u64::MAX, u64::MAX)]).unwrap();
        assert_eq!(long.bytes_per_sec, 1_000_000_000);
    }

    #[test]
    fn bytes_below_one_thousand_print_as_an_integer() {
        assert_eq!(format_bytes(0, "B/s"), "0 B/s");
        assert_eq!(format_bytes(999, "B/s"), "999 B/s");
    }

    #[test]
    fn bytes_scale_by_one_thousand_with_two_decimals() {
        assert_eq!(format_bytes(1000, "B/s"), "1.00 KB/s");
        assert_eq!(format_bytes(1500, "B"), "1.50 KB");
        assert_eq!(format_bytes(1_234_567, "B/s"), "1.23 MB/s");
        assert_eq!(format_bytes(1_235_000, "B/s"), "1.24 MB/s");
        assert_eq!(format_bytes(2_000_000_000, "B/s"), "2.00 GB/s");
    }

    #[test]
    fn a_figure_that_rounds_to_one_thousand_moves_up_a_unit() {
        assert_eq!(format_bytes(999_994, "B/s"), "999.99 KB/s");
        assert_eq!(format_bytes(999_995, "B/s"), "1.00 MB/s");
        assert_eq!(format_bytes(999_999, "B/s"), "1.00 MB/s");
    }

    #[test]
    fn terabytes_is_the_top_unit() {
        assert_eq!(format_bytes(u64::MAX, "B/s"), "18446744.07 TB/s");
    }

    #[test]
    fn the_bw_line_names_rate_sizes_and_window() {
        let stats = BwStats {
            bytes_per_sec: 480,
            frames: 10,
            mean_size: 24,
            min_size: 20,
            max_size: 1500,
        };
        assert_eq!(
            render_bw_line(&stats),
            "bandwidth: 480 B/s, mean: 24 B, min: 20 B, max: 1.50 KB, window: 10"
        );
    }

    /// The query `topic find` builds from `input`.
    fn q(input: &str) -> TypeQuery {
        parse_type_name(input).expect(input)
    }

    #[test]
    fn a_type_name_matches_the_name_printed_with_its_text() {
        assert!(q("geometry_msgs/Vector3").matches("geometry_msgs/Vector3"));
        assert!(q("_p/T_2").matches("_p/T_2"));
        assert!(!q("geometry_msgs/Vector3").matches("geometry_msgs/Vector"));
        assert!(!q("geometry_msgs/Vector3").matches("Vector3"));
    }

    #[test]
    fn the_ros_2_msg_form_names_the_same_type() {
        assert!(q("geometry_msgs/msg/Vector3").matches("geometry_msgs/Vector3"));
        // Only the `msg` middle part is the ROS 2 alias.
        assert!(!q("geometry_msgs/srv/Vector3").matches("geometry_msgs/Vector3"));
        assert!(!q("geometry_msgs/Vector3").matches("geometry_msgs/msg/Vector3"));
    }

    #[test]
    fn a_bare_workspace_type_name_is_kept_bare() {
        // A workspace schema with no package prints bare in `topic type` and
        // `topic list -t`, so `topic find` must match it bare.
        assert!(q("DetectionArray").matches("DetectionArray"));
        assert!(q("_T2").matches("_T2"));
        let rows = [
            typed("/det", TopicType::Named("DetectionArray".to_string())),
            typed("/odom", TopicType::Named("nav_msgs/Odometry".to_string())),
        ];
        assert_eq!(topics_of_type(&rows, &q("DetectionArray")), vec!["/det"]);
    }

    #[test]
    fn a_malformed_type_name_is_refused_with_its_shape() {
        for bad in [
            "",
            "/",
            "msg/",
            "geometry_msgs/",
            "/Vector3",
            "pkg//Type",
            "a/msg/",
            "pkg/Ty\u{7f}pe",
            "Ty\u{85}pe",
        ] {
            let err = parse_type_name(bad).expect_err(bad).to_string();
            let shown = crate::topic_cmd::sanitize_display(bad);
            assert!(
                err.contains(&format!("malformed type name '{shown}'"))
                    && err.contains(
                        "expected <package>/<Type>, <package>/msg/<Type> or a workspace \
                         <Type>, for example geometry_msgs/Vector3"
                    ),
                "{bad:?}: {err}"
            );
        }
    }

    /// The name the workspace walker prints for the schema `key` that
    /// `yaml` declares in `schemas/<file>.yaml`.
    fn printed_workspace_name(file: &str, yaml: &str, key: &str) -> TopicType {
        let ws = tempfile::tempdir().unwrap();
        let schemas = ws.path().join("schemas");
        std::fs::create_dir(&schemas).unwrap();
        std::fs::write(schemas.join(format!("{file}.yaml")), yaml).unwrap();
        let walker = crate::topic_cmd::local_walker_from_workspace(Some(&schemas));
        let hash = walker.schema_hash_for(key).expect(key);
        TopicType::of_hash(hash, &[&walker])
    }

    #[test]
    fn every_name_schema_create_writes_is_a_type_name_topic_find_takes() {
        // `schema create` PascalCases the name it is given into the YAML key
        // the walker names the type by, and validates nothing else: a word
        // that starts with a digit keeps its case. Each key is written out
        // by hand here.
        for (created, key) in [
            ("lidar_scan", "LidarScan"),
            ("1scan", "1scan"),
            ("scan-2", "Scan-2"),
            ("pose2D", "Pose2d"),
        ] {
            let ws = tempfile::tempdir().unwrap();
            let schemas = ws.path().join("schemas");
            std::fs::create_dir(&schemas).unwrap();
            crate::schema_cmd::schema_create(&schemas, created).unwrap();
            let yaml = std::fs::read_to_string(schemas.join(format!("{created}.yaml")))
                .unwrap()
                .replace("      # Add fields: type name\n", "      uint32 a:\n");

            let printed = printed_workspace_name(created, &yaml, key);
            assert_eq!(printed, TopicType::Named(key.to_string()), "{created}");
            let rows = [typed("/scan", printed)];
            assert_eq!(topics_of_type(&rows, &q(key)), vec!["/scan"], "{created}");
        }
    }

    #[test]
    fn a_workspace_key_with_slashes_is_found_by_the_name_it_prints() {
        // A package-less key may itself read `pkg/msg/Type` or `a/b/c`; the
        // listing prints it verbatim, so `topic find` must find it with that
        // text, while `pkg/msg/Type` still aliases `pkg/Type` too.
        for key in ["pkg/msg/Scan", "a/b/c", "a/srv/B"] {
            let yaml = format!("schemas:\n  {key}:\n    fields:\n      uint32 a:\n");
            let printed = printed_workspace_name("slashed", &yaml, key);
            assert_eq!(printed, TopicType::Named(key.to_string()), "{key}");
            let rows = [
                typed("/slashed", printed),
                typed("/aliased", TopicType::Named("pkg/Scan".to_string())),
            ];
            let expected = if key == "pkg/msg/Scan" {
                vec!["/aliased", "/slashed"]
            } else {
                vec!["/slashed"]
            };
            assert_eq!(topics_of_type(&rows, &q(key)), expected, "{key}");
        }
    }

    #[test]
    fn a_part_may_start_with_a_digit_or_hold_a_dash_or_a_space() {
        for name in ["1Scan", "Scan-2", "Vec tor3", "1pkg/Ty-pe"] {
            assert!(q(name).matches(name), "{name}");
        }
        assert!(q("1pkg/msg/2Type").matches("1pkg/2Type"));
    }

    #[test]
    fn a_malformed_type_name_cannot_inject_terminal_escapes() {
        let err = parse_type_name("a\u{1b}[2J").unwrap_err().to_string();
        assert!(!err.contains('\u{1b}'), "{err:?}");
        assert!(err.contains("'a\u{FFFD}[2J'"), "{err:?}");
    }

    fn typed(name: &str, ty: TopicType) -> TypedTopic {
        TypedTopic {
            name: name.to_string(),
            ty,
        }
    }

    fn named(name: &str) -> TopicType {
        TopicType::Named(name.to_string())
    }

    #[test]
    fn a_typed_listing_puts_the_type_after_the_topic() {
        let rows = [
            typed("/demo/a/cmd", named("geometry_msgs/Vector3")),
            typed("/demo/b/raw", TopicType::Unnamed(0xab)),
            typed("/demo/c/idle", TopicType::Unknown),
        ];
        assert_eq!(
            render_local_typed_topics_section(&rows, false),
            "TOPIC\n\
             /demo/a/cmd  [geometry_msgs/Vector3]\n\
             /demo/b/raw  [0x00000000000000ab]\n\
             /demo/c/idle  [unknown]\n"
        );
    }

    #[test]
    fn a_typed_listing_hides_and_counts_internal_topics_like_the_plain_one() {
        let rows = [
            typed("/bagd/status", named("std_msgs/String")),
            typed("/demo/a/cmd", named("geometry_msgs/Vector3")),
        ];
        assert_eq!(
            render_local_typed_topics_section(&rows, false),
            "TOPIC\n/demo/a/cmd  [geometry_msgs/Vector3]\n\
             1 internal topic hidden (--all shows it)\n"
        );
        assert_eq!(
            render_local_typed_topics_section(&rows, true),
            "TOPIC\n/bagd/status  [std_msgs/String]  internal\n\
             /demo/a/cmd  [geometry_msgs/Vector3]\n"
        );
    }

    #[test]
    fn an_empty_typed_listing_says_so() {
        assert_eq!(
            render_local_typed_topics_section(&[], false),
            "No active local topics.\n"
        );
        assert_eq!(
            render_local_typed_topics_section(&[typed("/bagd/status", TopicType::Unknown)], false),
            "No active local topics.\n1 internal topic hidden (--all shows it)\n"
        );
    }

    #[test]
    fn find_lists_only_topics_of_that_type_sorted_once_each() {
        let rows = [
            typed("/z/cmd", named("geometry_msgs/Vector3")),
            typed("/a/name", named("std_msgs/String")),
            typed("/a/cmd", named("geometry_msgs/Vector3")),
            typed("/a/cmd", named("geometry_msgs/Vector3")),
            typed("/m/raw", TopicType::Unnamed(1)),
            typed("/m/idle", TopicType::Unknown),
        ];
        assert_eq!(
            topics_of_type(&rows, &q("geometry_msgs/Vector3")),
            vec!["/a/cmd", "/z/cmd"]
        );
        assert_eq!(
            topics_of_type(&rows, &q("std_msgs/String")),
            vec!["/a/name"]
        );
    }

    #[test]
    fn find_refuses_a_malformed_type_before_reading_any_topic() {
        let err = topic_find("not//a/type", None, TopicScope::Local, vec![], vec![])
            .unwrap_err()
            .to_string();
        assert!(err.contains("malformed type name 'not//a/type'"), "{err}");
    }

    #[test]
    fn find_of_an_unknown_type_is_empty() {
        let rows = [typed("/a/cmd", named("geometry_msgs/Vector3"))];
        assert!(topics_of_type(&rows, &q("geometry_msgs/Twist")).is_empty());
        assert!(topics_of_type(&[], &q("geometry_msgs/Vector3")).is_empty());
    }

    #[test]
    fn frames_sharing_the_earliest_stamp_all_open_the_span() {
        // Two frames at 0 are one instant: only the 100 bytes at 1 s count.
        let window = [frame(0, 100), frame(0, 100), frame(SEC, 100)];
        let stats = bandwidth(&window).unwrap();
        assert_eq!(stats.bytes_per_sec, 100);
        assert_eq!(stats.frames, 3);
        assert_eq!(stats.mean_size, 100);
    }

    #[test]
    fn a_silent_report_says_so_and_keeps_the_window() {
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(8);
        ring.extend([frame(0, 10), frame(SEC, 10)]);
        assert_eq!(
            bw_report_line(&mut ring, 0, Duration::from_secs(3), "local SHM"),
            "no messages received: attached to local SHM; 0 frames in 3s"
        );
        assert_eq!(ring, [frame(0, 10), frame(SEC, 10)]);
    }

    #[test]
    fn a_publisher_slower_than_one_report_still_measures_a_rate() {
        // 500 bytes every 3 s: one frame, two silent reports, the next frame.
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(100);
        ring.push_back(frame(0, 500));
        assert_eq!(
            bw_report_line(&mut ring, 1, Duration::ZERO, "local SHM"),
            "waiting for more messages... (received 1)"
        );
        for silent in [1, 2] {
            bw_report_line(&mut ring, 0, Duration::from_secs(silent), "local SHM");
        }
        ring.push_back(frame(3 * SEC, 500));
        // 500 bytes over 3 s is 166.67 B/s, rounded to 167.
        assert_eq!(
            bw_report_line(&mut ring, 1, Duration::ZERO, "local SHM"),
            "bandwidth: 167 B/s, mean: 500 B, min: 500 B, max: 500 B, window: 2"
        );
    }

    #[test]
    fn a_window_that_spans_a_pause_measures_over_the_pause() {
        // 10 frames of 10 bytes 100 ms apart, a 60 s pause, 10 more: the
        // window of 100 holds all 20, so the figure is 190 bytes after the
        // first frame over the 61.9 s span, 3.07 B/s rounded to 3.
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(100);
        ring.extend((0..10).map(|i| frame(i * SEC / 10, 10)));
        bw_report_line(&mut ring, 0, Duration::from_secs(60), "local SHM");
        ring.extend((0..10).map(|i| frame(61 * SEC + i * SEC / 10, 10)));
        assert_eq!(
            bw_report_line(&mut ring, 10, Duration::ZERO, "local SHM"),
            "bandwidth: 3 B/s, mean: 10 B, min: 10 B, max: 10 B, window: 20"
        );
    }

    /// A writer that fails every write with `kind`.
    struct Failing(std::io::ErrorKind);
    impl Write for Failing {
        fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
            Err(std::io::Error::from(self.0))
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_report_line_is_written_and_flushed() {
        let mut out: Vec<u8> = Vec::new();
        assert_eq!(
            write_report_line(&mut out, "bandwidth: 1 B/s").unwrap(),
            Output::Open
        );
        assert_eq!(out, b"bandwidth: 1 B/s\n");
    }

    #[test]
    fn a_closed_pipe_ends_the_report_and_another_failure_is_returned() {
        let mut closed = Failing(std::io::ErrorKind::BrokenPipe);
        assert_eq!(write_report_line(&mut closed, "x").unwrap(), Output::Closed);
        let mut full = Failing(std::io::ErrorKind::PermissionDenied);
        let err = write_report_line(&mut full, "x").unwrap_err();
        assert!(matches!(err, CliError::Io(_)), "{err:?}");
    }

    #[test]
    fn a_report_with_one_frame_waits_for_more() {
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(4);
        ring.push_back(frame(5, 10));
        assert_eq!(
            bw_report_line(&mut ring, 1, Duration::ZERO, "local SHM"),
            "waiting for more messages... (received 1)"
        );
        assert_eq!(ring.len(), 1);
    }

    fn empty_walker() -> cerulion_core::codegen::FrameWalker {
        cerulion_core::codegen::FrameWalker::new(vec![]).0
    }

    #[test]
    fn a_hash_is_named_by_the_first_walker_that_knows_it() {
        let builtins = super::super::local_walker_from_workspace(None);
        let empty = empty_walker();
        let string = super::super::STD_MSGS_STRING_SCHEMA_HASH;
        assert_eq!(
            TopicType::of_hash(string, &[&builtins]),
            named("std_msgs/String")
        );
        // The local walker misses, the remote one names it.
        assert_eq!(
            TopicType::of_hash(string, &[&empty, &builtins]),
            named("std_msgs/String")
        );
        assert_eq!(
            TopicType::of_hash(string, &[&empty]),
            TopicType::Unnamed(string)
        );
        assert_eq!(
            TopicType::of_hash(0x1234, &[&builtins]),
            TopicType::Unnamed(0x1234)
        );
        assert_eq!(TopicType::of_hash(0x1234, &[]), TopicType::Unnamed(0x1234));
    }

    fn entry(topic: &str, name: Option<&str>, hash: Option<u64>) -> cerulion_core::CatalogEntry {
        cerulion_core::CatalogEntry {
            topic: topic.to_string(),
            schema_hash: hash,
            schema_name: name.map(str::to_string),
            provenance: cerulion_core::CatalogProvenance::Runtime,
            producer_count: None,
            liveness: None,
        }
    }

    fn catalog(
        robot: &str,
        entries: Vec<cerulion_core::CatalogEntry>,
    ) -> cerulion_core::CatalogReply {
        cerulion_core::transport::cerulion_q::build_catalog_reply(robot, entries)
    }

    #[test]
    fn find_matches_each_robot_on_a_shared_topic_by_its_own_type() {
        let walker = super::super::local_walker_from_workspace(None);
        let string = super::super::STD_MSGS_STRING_SCHEMA_HASH;
        let replies = [
            catalog("alpha", vec![entry("/tf", Some("pkg/A"), None)]),
            catalog(
                "bravo",
                vec![
                    entry("/tf", Some("pkg/B"), None),
                    entry("/b/chatter", None, Some(string)),
                ],
            ),
        ];
        assert_eq!(
            remote_topics_of_type(&replies, &[], &walker, &q("pkg/A")),
            ["/tf"]
        );
        assert_eq!(
            remote_topics_of_type(&replies, &[], &walker, &q("pkg/B")),
            ["/tf"]
        );
        assert_eq!(
            remote_topics_of_type(&replies, &[], &walker, &q("std_msgs/String")),
            ["/b/chatter"]
        );
        assert!(remote_topics_of_type(&replies, &[], &walker, &q("pkg/C")).is_empty());
        // Two robots of one type list the topic once.
        let same = [
            catalog("alpha", vec![entry("/tf", Some("pkg/A"), None)]),
            catalog("bravo", vec![entry("/tf", Some("pkg/A"), None)]),
        ];
        assert_eq!(
            remote_topics_of_type(&same, &[], &walker, &q("pkg/A")),
            ["/tf"]
        );
    }

    fn mirror(topic: &str) -> super::super::MirrorStreamRow {
        super::super::MirrorStreamRow {
            topic: topic.to_string(),
            robot: "go2".to_string(),
        }
    }

    #[test]
    fn a_mirror_no_catalog_types_is_counted_as_not_searched() {
        let mirrors = [mirror("/go2/odom"), mirror("/go2/cmd")];
        // No catalog at all: the local scope, or a failed remote query.
        assert_eq!(
            unsearched_mirrors_note(&mirrors, &[]).as_deref(),
            Some(
                "remote: 2 mirrored topic(s) not searched: their type comes from their \
                 robot's catalog, which this search did not get (`topic list` shows them)\n"
            )
        );
        // A catalog that lists one of them leaves the other.
        let replies = [catalog(
            "go2",
            vec![entry("/go2/odom", Some("pkg/A"), None)],
        )];
        assert!(unsearched_mirrors_note(&mirrors, &replies)
            .expect("one is not searched")
            .starts_with("remote: 1 mirrored topic(s) not searched"));
        // Every mirror in a catalog, or no mirror: no note.
        let all = [catalog(
            "go2",
            vec![
                entry("/go2/odom", Some("pkg/A"), None),
                entry("/go2/cmd", None, None),
            ],
        )];
        assert_eq!(unsearched_mirrors_note(&mirrors, &all), None);
        assert_eq!(unsearched_mirrors_note(&[], &[]), None);
    }

    #[test]
    fn a_mirror_is_searched_only_through_its_origin_robots_catalog() {
        let mirrors = [mirror("/odom")];
        // Another robot lists the same topic name: that says nothing about
        // the go2 mirror's type, so the mirror is still not searched.
        let other = [catalog(
            "spot",
            vec![entry("/odom", Some("nav_msgs/Odometry"), None)],
        )];
        assert!(unsearched_mirrors_note(&mirrors, &other)
            .expect("the origin robot sent no catalog")
            .starts_with("remote: 1 mirrored topic(s) not searched"));
        // The origin robot's catalog lists it: searched, with or without the
        // other robot.
        let both = [
            other[0].clone(),
            catalog("go2", vec![entry("/odom", Some("nav_msgs/Odometry"), None)]),
        ];
        assert_eq!(unsearched_mirrors_note(&mirrors, &both), None);
    }

    #[test]
    fn remote_rows_take_the_catalog_name_then_the_hash_then_unknown() {
        let walker = super::super::local_walker_from_workspace(None);
        let string = super::super::STD_MSGS_STRING_SCHEMA_HASH;
        let types = remote_topic_types(
            &[catalog(
                "go2",
                vec![
                    entry("/go2/named", Some("probe_msgs/Widget"), Some(0x99)),
                    entry("/go2/hashed", None, Some(string)),
                    entry("/go2/raw", None, Some(0xab)),
                    entry("/go2/bare", None, None),
                ],
            )],
            &[],
            &walker,
        );
        let expected: BTreeMap<String, TopicType> = [
            ("/go2/bare", TopicType::Unknown),
            ("/go2/hashed", named("std_msgs/String")),
            ("/go2/named", named("probe_msgs/Widget")),
            ("/go2/raw", TopicType::Unnamed(0xab)),
        ]
        .into_iter()
        .map(|(t, ty)| (t.to_string(), ty))
        .collect();
        assert_eq!(types, expected);
    }

    #[test]
    fn two_robots_on_one_topic_give_the_same_answer_in_any_reply_order() {
        let walker = empty_walker();
        let a = catalog(
            "alpha",
            vec![
                entry("/tf", None, Some(0x1)),
                entry("/cmd", Some("geometry_msgs/Twist"), None),
            ],
        );
        let b = catalog(
            "bravo",
            vec![
                entry("/tf", Some("tf2_msgs/TFMessage"), None),
                entry("/cmd", Some("geometry_msgs/Vector3"), None),
            ],
        );
        for replies in [[a.clone(), b.clone()], [b, a]] {
            let types = remote_topic_types(&replies, &[], &walker);
            // A name beats a bare hash, whichever robot sent it.
            assert_eq!(types["/tf"], named("tf2_msgs/TFMessage"));
            // Equal answers: the robot that sorts first.
            assert_eq!(types["/cmd"], named("geometry_msgs/Twist"));
        }
    }

    #[test]
    fn an_mdns_only_peer_is_a_discovered_robot_find_must_account_for() {
        let disc = super::super::RemoteDiscovery {
            announce_entries: vec![
                ("go2".to_string(), Some("/go2/cmd".to_string())),
                ("go2".to_string(), None),
            ],
            robots: vec![super::super::RobotRow {
                robot: "spot".to_string(),
                locator: Some("tcp/10.0.0.7:7683".to_string()),
                provenance: super::super::RobotProvenance::Rung(super::super::DiscoveryRung::Mdns),
                topic_count: 0,
            }],
            ..super::super::RemoteDiscovery::empty()
        };
        let robots = discovered_robots(&disc);
        // go2 answered its catalog; spot was found by mDNS alone, never
        // asked, and is named.
        let gather = super::super::CatalogGather {
            replies: vec![catalog("go2", vec![])],
            panicked: Vec::new(),
        };
        assert_eq!(
            unsearched_robots_note(robots.iter().map(String::as_str), &gather).as_deref(),
            Some(
                "remote: 1 robot(s) not searched: no catalog from spot, so their topics of \
                 this type are not listed\n"
            )
        );
    }

    #[test]
    fn a_robot_with_no_catalog_answer_is_named_as_not_searched() {
        let answered = |robots: &[&str]| super::super::CatalogGather {
            replies: robots.iter().map(|r| catalog(r, vec![])).collect(),
            panicked: Vec::new(),
        };
        // Every discovered robot answered: nothing to say.
        assert_eq!(unsearched_robots_note(["go2"], &answered(&["go2"])), None);
        assert_eq!(unsearched_robots_note([], &answered(&[])), None);
        // A robot that timed out and one whose worker panicked are both
        // named, once each and sorted.
        let mut gather = answered(&["go2"]);
        gather.panicked = vec!["orin".to_string()];
        assert_eq!(
            unsearched_robots_note(["spot", "go2", "spot", "orin"], &gather).as_deref(),
            Some(
                "remote: 2 robot(s) not searched: no catalog from orin, spot, so their topics \
                 of this type are not listed\n"
            )
        );
        let note = unsearched_robots_note(["a\u{1b}[2J"], &answered(&[])).unwrap();
        assert!(!note.contains('\u{1b}'), "{note:?}");
    }

    #[test]
    fn a_robot_that_refused_its_catalog_is_named_as_not_searched() {
        let mut refused = catalog("spot", vec![]);
        refused.error = Some("not paired".to_string());
        let gather = super::super::CatalogGather {
            replies: vec![catalog("go2", vec![]), refused],
            panicked: Vec::new(),
        };
        let expected = Some(
            "remote: 1 robot(s) not searched: no catalog from spot, so their topics of this \
             type are not listed\n",
        );
        // Discovered or not, a refusal is no catalog: its topics were not searched.
        assert_eq!(
            unsearched_robots_note(["go2", "spot"], &gather).as_deref(),
            expected
        );
        assert_eq!(
            unsearched_robots_note(["go2"], &gather).as_deref(),
            expected
        );
    }

    #[test]
    fn find_matches_a_mirror_only_through_its_origin_robots_catalog() {
        let walker = empty_walker();
        let from = |robot: &str| super::super::MirrorStreamRow {
            topic: "/odom".to_string(),
            robot: robot.to_string(),
        };
        let spot_only = [catalog(
            "spot",
            vec![entry("/odom", Some("nav_msgs/Odometry"), None)],
        )];
        let odom = q("nav_msgs/Odometry");
        // Not a mirror: any robot's entry counts.
        assert_eq!(
            remote_topics_of_type(&spot_only, &[], &walker, &odom),
            ["/odom"]
        );
        // Mirrored from go2, whose catalog does not list it: spot's entry
        // does not count, and the mirror note says it was not searched.
        assert!(remote_topics_of_type(&spot_only, &[from("go2")], &walker, &odom).is_empty());
        assert!(unsearched_mirrors_note(&[from("go2")], &spot_only).is_some());
        // go2 lists it with another type: still not found by spot's type.
        let both = [
            spot_only[0].clone(),
            catalog(
                "go2",
                vec![entry("/odom", Some("geometry_msgs/Twist"), None)],
            ),
        ];
        assert!(remote_topics_of_type(&both, &[from("go2")], &walker, &odom).is_empty());
        assert_eq!(
            remote_topics_of_type(&both, &[from("go2")], &walker, &q("geometry_msgs/Twist")),
            ["/odom"]
        );
        // Mirrored from spot: found through spot's catalog.
        assert_eq!(
            remote_topics_of_type(&both, &[from("spot")], &walker, &odom),
            ["/odom"]
        );
    }

    #[test]
    fn a_mirror_row_takes_its_origin_robots_type_only() {
        let walker = empty_walker();
        let replies = [
            catalog(
                "alpha",
                vec![entry("/cmd", Some("geometry_msgs/Twist"), None)],
            ),
            catalog(
                "bravo",
                vec![entry("/cmd", Some("geometry_msgs/Vector3"), None)],
            ),
        ];
        let from = |robot: &str| super::super::MirrorStreamRow {
            topic: "/cmd".to_string(),
            robot: robot.to_string(),
        };
        // Without a mirror the robot that sorts first answers for the topic.
        assert_eq!(
            remote_topic_types(&replies, &[], &walker)["/cmd"],
            named("geometry_msgs/Twist")
        );
        // A mirror from bravo carries bravo's messages, in any reply order.
        for replies in [replies.clone(), [replies[1].clone(), replies[0].clone()]] {
            let types = remote_topic_types(&replies, &[from("bravo")], &walker);
            assert_eq!(types["/cmd"], named("geometry_msgs/Vector3"));
            assert_eq!(
                render_remote_typed_topics_section(
                    &disc(&["/cmd"]),
                    true,
                    &[from("bravo")],
                    &types
                ),
                "\nREMOTE TOPICS\n/cmd  [geometry_msgs/Vector3]  ● streaming  bravo\n"
            );
        }
        // An origin robot whose catalog does not list the topic: unknown,
        // never another robot's answer.
        let types = remote_topic_types(&replies, &[from("charlie")], &walker);
        assert_eq!(types["/cmd"], TopicType::Unknown);
    }

    #[test]
    fn a_catalog_name_cannot_inject_terminal_escapes() {
        let types = remote_topic_types(
            &[catalog("r", vec![entry("/t", Some("a\u{1b}[2J/B"), None)])],
            &[],
            &empty_walker(),
        );
        assert_eq!(types["/t"], named("a\u{FFFD}[2J/B"));
    }

    fn disc(topics: &[&str]) -> super::super::RemoteDiscovery {
        super::super::RemoteDiscovery {
            topics: topics.iter().map(|t| t.to_string()).collect(),
            ..super::super::RemoteDiscovery::empty()
        }
    }

    #[test]
    fn a_typed_remote_section_puts_the_type_after_the_topic() {
        let types: BTreeMap<String, TopicType> =
            [("/go2/cmd".to_string(), named("geometry_msgs/Twist"))].into();
        assert_eq!(
            render_remote_typed_topics_section(
                &disc(&["/go2/cmd", "/go2/odom"]),
                true,
                &[],
                &types
            ),
            "\nREMOTE TOPICS\n/go2/cmd  [geometry_msgs/Twist]\n/go2/odom  [unknown]\n"
        );
        // A mirror row: the type sits between the topic and the marker.
        let streaming = [super::super::MirrorStreamRow {
            topic: "/go2/odom".to_string(),
            robot: "go2".to_string(),
        }];
        assert_eq!(
            render_remote_typed_topics_section(&disc(&["/go2/cmd"]), true, &streaming, &types),
            "\nREMOTE TOPICS\n/go2/cmd  [geometry_msgs/Twist]\n\
             /go2/odom  [unknown]  ● streaming  go2\n"
        );
    }

    #[test]
    fn a_typed_remote_section_with_no_topic_is_the_plain_one_line() {
        let typed = render_remote_typed_topics_section(&disc(&[]), true, &[], &BTreeMap::new());
        assert_eq!(
            typed,
            super::super::render_remote_topics_section(&disc(&[]), true)
        );
        assert!(typed.starts_with("remote: none discovered"), "{typed}");
    }

    #[test]
    fn the_type_error_arms_name_the_topic() {
        assert_eq!(
            silent_topic_error("/a/cmd", LOCAL_FRAME_WAIT).to_string(),
            "topic '/a/cmd' sent no message within 1s, so its type is unknown; retry while its \
             publisher is sending"
        );
        assert_eq!(
            silent_topic_error("/a/cmd", REMOTE_FRAME_WAIT).to_string(),
            "topic '/a/cmd' sent no message within 3s, so its type is unknown; retry while its \
             publisher is sending"
        );
        assert_eq!(
            unnamed_hash_error("/a/raw", 0xab).to_string(),
            "topic '/a/raw' carries schema hash 0x00000000000000ab, which no built-in or \
             workspace type names; add its `.msg` under `schemas/` to name it"
        );
        assert_eq!(
            receive_failed_error("/a/cmd", &"listener gone").to_string(),
            "receive failed on topic '/a/cmd': listener gone"
        );
        assert!(!unnamed_hash_error("/a\u{1b}", 1)
            .to_string()
            .contains('\u{1b}'));
    }
}
