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

/// Parse a message type name as `topic find` takes it and return its
/// canonical `package/Type` form.
///
/// Both the Cerulion form `geometry_msgs/Vector3` and the ROS 2 form
/// `geometry_msgs/msg/Vector3` are accepted. Each part is a non-empty run of
/// ASCII letters, digits and `_` that does not start with a digit. Anything
/// else is a validation error naming the input and the expected shape.
pub fn parse_type_name(input: &str) -> CliResult<String> {
    let parts: Vec<&str> = input.split('/').collect();
    let (package, ty) = match parts.as_slice() {
        [package, ty] | [package, "msg", ty] => (*package, *ty),
        _ => return Err(malformed_type(input)),
    };
    if !is_identifier(package) || !is_identifier(ty) {
        return Err(malformed_type(input));
    }
    Ok(format!("{package}/{ty}"))
}

fn is_identifier(part: &str) -> bool {
    let mut chars = part.chars();
    matches!(chars.next(), Some(c) if c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn malformed_type(input: &str) -> CliError {
    CliError::Validation(format!(
        "malformed type name '{}': expected <package>/<Type>, for example \
         geometry_msgs/Vector3",
        super::sanitize_display(input)
    ))
}

/// What a typed listing knows about one topic's message type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopicType {
    /// The type's `package/Type` name, already safe to print.
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
pub fn remote_topic_types(
    catalogs: &[cerulion_core::CatalogReply],
    walker: &cerulion_core::codegen::FrameWalker,
) -> BTreeMap<String, TopicType> {
    let mut by_robot: Vec<&cerulion_core::CatalogReply> = catalogs.iter().collect();
    by_robot.sort_by(|a, b| a.robot.cmp(&b.robot));
    let mut types: BTreeMap<String, TopicType> = BTreeMap::new();
    for entry in by_robot.iter().flat_map(|c| &c.entries) {
        let ty = match (&entry.schema_name, entry.schema_hash) {
            (Some(name), _) => TopicType::Named(super::sanitize_display(name)),
            (None, Some(hash)) => TopicType::of_hash(hash, &[walker]),
            (None, None) => TopicType::Unknown,
        };
        match types.get(&entry.topic) {
            Some(known) if known.rank() >= ty.rank() => {}
            _ => {
                types.insert(entry.topic.clone(), ty);
            }
        }
    }
    types
}

/// The topics whose type is `type_name` (canonical `package/Type`), sorted
/// by name with duplicates removed. A topic with an unknown or unnamed type
/// never matches.
pub fn topics_of_type<'a>(topics: &'a [TypedTopic], type_name: &str) -> Vec<&'a str> {
    let mut found: Vec<&str> = topics
        .iter()
        .filter(|t| matches!(&t.ty, TopicType::Named(name) if name == type_name))
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

/// `cerulion topic list -t`: the `topic list` listing with each topic's type.
///
/// LOCAL rows are typed from one frame each (see [`type_local_topics`]); the
/// section prints before the remote query starts, as in `topic list`.
/// REMOTE rows, network and mirror alike, are typed from the robots' catalogs
/// ([`remote_topic_types`]); with no remote query (`remote` is `None`) or a
/// failed one, a mirror row shows `[unknown]`. A failed remote query is the
/// one `topic list` stderr line, and the verb still succeeds.
pub fn topic_list_typed(
    show_all: bool,
    remote: Option<super::RemoteTopicsOptions>,
    scan: bool,
    schemas_dir: Option<&Path>,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> CliResult<()> {
    let mirrors = super::gather_mirror_identities()?;
    let (local, streaming) = super::partition_local_topics(super::topic_list()?, &mirrors);
    let walker = super::local_walker_from_workspace(schemas_dir);
    let internal = if show_all {
        InternalTopics::Read
    } else {
        InternalTopics::Skip
    };
    let typed = type_local_topics(&local, &walker, remote.is_none(), internal)?;
    write!(
        out,
        "{}",
        render_local_typed_topics_section(&typed, show_all)
    )?;
    out.flush()?;
    if let Some(opts) = remote {
        match super::query_remote_topics_and_catalogs(&opts, scan) {
            Ok((disc, catalogs)) => {
                let types = remote_topic_types(&catalogs, &walker);
                write!(
                    out,
                    "{}",
                    render_remote_typed_topics_section(
                        &disc,
                        opts.has_endpoints(),
                        &streaming,
                        &types
                    )
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
            render_remote_typed_topics_section(
                &super::RemoteDiscovery::empty(),
                false,
                &streaming,
                &BTreeMap::new()
            )
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
}

/// `cerulion topic find <type>`: every topic whose type is `type_name`
/// (either `package/Type` or `package/msg/Type`), sorted.
///
/// Genuine LOCAL topics are typed from one frame each; unless the scope is
/// local, the remote robots `topic list` discovers are asked too and their
/// topics typed from their catalogs. An unknown type is an empty list; a
/// malformed name is an error before any topic is read.
pub fn topic_find(
    type_name: &str,
    schemas_dir: Option<&Path>,
    scope: TopicScope,
) -> CliResult<FoundTopics> {
    let wanted = parse_type_name(type_name)?;
    let mirrors = super::gather_mirror_identities()?;
    let (local, _streaming) = super::partition_local_topics(super::topic_list()?, &mirrors);
    let walker = super::local_walker_from_workspace(schemas_dir);
    let remote = super::remote_discovery_options(scope == TopicScope::Local, vec![], vec![]);
    let mut typed = type_local_topics(&local, &walker, remote.is_none(), InternalTopics::Read)?;
    let mut remote_unavailable = None;
    if let Some(opts) = remote {
        match super::query_remote_topics_and_catalogs(&opts, false) {
            Ok((_disc, catalogs)) => typed.extend(
                remote_topic_types(&catalogs, &walker)
                    .into_iter()
                    .map(|(name, ty)| TypedTopic { name, ty }),
            ),
            Err(e) => {
                remote_unavailable = Some(super::render_remote_discovery_unavailable(&e));
            }
        }
    }
    Ok(FoundTopics {
        topics: topics_of_type(&typed, &wanted)
            .into_iter()
            .map(super::sanitize_display)
            .collect(),
        remote_unavailable,
    })
}

/// `cerulion topic type <topic>`: the `package/Type` name of the topic's
/// message type.
///
/// The topic is opened exactly as `topic info` opens it (a local producer
/// directly; a remote robot's topic through the network daemon unless the
/// scope is local) and its first frame's schema hash is named by the same
/// ladder: the local walker, then the remote one. A missing topic is the
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
/// An interval with no new frame prints the silence line and EMPTIES the
/// ring (keeping its capacity, so this never allocates a frame slot): the
/// window then starts again with the next frame, so a rate never spans a
/// pause, as `topic hz` measures each second afresh. Otherwise the line is
/// [`render_bw_line`] over the ring, or a waiting line while the ring holds
/// too few frames for a rate.
pub fn bw_report_line(
    ring: &mut VecDeque<BwFrame>,
    arrived_this_report: usize,
    silent_for: Duration,
    source_desc: &str,
) -> String {
    if arrived_this_report == 0 {
        ring.clear();
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
/// repeating the last figure, and starts the window again (see
/// [`bw_report_line`]).
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
        if delivered > 0 {
            arrived_this_report += delivered;
            silence_since = Instant::now();
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
            writeln!(writer, "{line}").ok();
            arrived_this_report = 0;
            last_report = Instant::now();
        }
    }
    Ok(())
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

    #[test]
    fn a_type_name_is_package_slash_type() {
        assert_eq!(
            parse_type_name("geometry_msgs/Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
        assert_eq!(parse_type_name("_p/T_2").unwrap(), "_p/T_2");
    }

    #[test]
    fn the_ros_2_msg_form_names_the_same_type() {
        assert_eq!(
            parse_type_name("geometry_msgs/msg/Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
    }

    #[test]
    fn a_malformed_type_name_is_refused_with_its_shape() {
        for bad in [
            "",
            "Vector3",
            "geometry_msgs/",
            "/Vector3",
            "a/b/c",
            "a/srv/B",
            "geometry_msgs/Vec tor3",
            "1pkg/Type",
            "pkg/Ty-pe",
            "pkg//Type",
        ] {
            let err = parse_type_name(bad).expect_err(bad).to_string();
            assert!(
                err.contains(&format!("malformed type name '{bad}'"))
                    && err.contains("expected <package>/<Type>, for example geometry_msgs/Vector3"),
                "{bad:?}: {err}"
            );
        }
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
            topics_of_type(&rows, "geometry_msgs/Vector3"),
            vec!["/a/cmd", "/z/cmd"]
        );
        assert_eq!(topics_of_type(&rows, "std_msgs/String"), vec!["/a/name"]);
    }

    #[test]
    fn find_refuses_a_malformed_type_before_reading_any_topic() {
        let err = topic_find("not a type", None, TopicScope::Local)
            .unwrap_err()
            .to_string();
        assert!(err.contains("malformed type name 'not a type'"), "{err}");
    }

    #[test]
    fn find_of_an_unknown_type_is_empty() {
        let rows = [typed("/a/cmd", named("geometry_msgs/Vector3"))];
        assert!(topics_of_type(&rows, "geometry_msgs/Twist").is_empty());
        assert!(topics_of_type(&[], "geometry_msgs/Vector3").is_empty());
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
    fn a_silent_report_says_so_and_empties_the_window() {
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(8);
        ring.extend([frame(0, 10), frame(SEC, 10)]);
        let capacity = ring.capacity();
        assert_eq!(
            bw_report_line(&mut ring, 0, Duration::from_secs(3), "local SHM"),
            "no messages received: attached to local SHM; 0 frames in 3s"
        );
        assert!(ring.is_empty());
        assert_eq!(ring.capacity(), capacity, "clearing must keep the slots");
    }

    #[test]
    fn a_stream_that_resumes_after_a_pause_measures_only_the_new_frames() {
        // 10 B every 100 ms, a 60 s pause, then 10 B every 100 ms again.
        let mut ring: VecDeque<BwFrame> = VecDeque::with_capacity(100);
        ring.extend((0..10).map(|i| frame(i * SEC / 10, 10)));
        assert_eq!(
            bw_report_line(&mut ring, 10, Duration::ZERO, "local SHM"),
            "bandwidth: 100 B/s, mean: 10 B, min: 10 B, max: 10 B, window: 10"
        );
        bw_report_line(&mut ring, 0, Duration::from_secs(60), "local SHM");
        ring.extend((0..10).map(|i| frame(61 * SEC + i * SEC / 10, 10)));
        assert_eq!(
            bw_report_line(&mut ring, 10, Duration::ZERO, "local SHM"),
            "bandwidth: 100 B/s, mean: 10 B, min: 10 B, max: 10 B, window: 10"
        );
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
            let types = remote_topic_types(&replies, &walker);
            // A name beats a bare hash, whichever robot sent it.
            assert_eq!(types["/tf"], named("tf2_msgs/TFMessage"));
            // Equal answers: the robot that sorts first.
            assert_eq!(types["/cmd"], named("geometry_msgs/Twist"));
        }
    }

    #[test]
    fn a_catalog_name_cannot_inject_terminal_escapes() {
        let types = remote_topic_types(
            &[catalog("r", vec![entry("/t", Some("a\u{1b}[2J/B"), None)])],
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
