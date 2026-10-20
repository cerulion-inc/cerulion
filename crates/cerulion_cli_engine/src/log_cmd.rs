// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion log echo`: the records on the host's one log topic, printed one
//! record per line, filtered by node and by minimum level.
//!
//! [`LOG_TOPIC`] is `/rosout`, the topic ROS 2 nodes publish their logs on:
//! `rmw_cerulion` keeps a ROS name verbatim, so a ROS 2 node started with
//! `cerulion ros2 run` publishes on this very topic. A record on it is an
//! `rcl_interfaces/Log` message, read under Cerulion names: `name` is the
//! node, `level` the `Log.msg` severity constant (`DEBUG=10`, `INFO=20`,
//! `WARN=30`, `ERROR=40`), `stamp` the record time, `function` the `tracing`
//! target (the module path the record came from), `file` and `line` the
//! source location and `msg` the text. Cerulion node `tracing` records are
//! not published to the topic yet: a Cerulion node's logs go to its stderr.
//!
//! The verb reads the topic the way `topic echo --local` does (a live local
//! shared-memory topic, no remote rung) and renders each record with
//! [`render_line`]. The filters, the line format, the frame decode and the
//! silence notice are pure functions with hand-written oracles below; the loop
//! itself blocks on the topic's event listener and never polls.

use std::fmt;
use std::fmt::Write as _;
use std::io;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::shm_runtime::read_offset_entry;
use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_core::wire::WireError;
use cerulion_core::ShmMessage;
use native_ros2_messages::rcl_interfaces::{Log, LogFixedSection, LogShm};

use crate::error::{CliError, CliResult};
use crate::topic_cmd::{self, TopicScope};

/// The host's one log topic.
///
/// An absolute name, like `/tf`: one topic for the whole host, shared by every
/// process running on it. It is ROS 2 `/rosout` under its own name, because
/// `rmw_cerulion` maps a ROS topic name to the Cerulion name verbatim, with no
/// alias: a different name here would never see a ROS 2 node's records.
pub const LOG_TOPIC: &str = "/rosout";

/// A log level `--level` can name, lowest first. The order is the severity
/// order, so `Debug < Info < Warn < Error`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    /// Every level, lowest first.
    pub const ALL: [LogLevel; 4] = [
        LogLevel::Debug,
        LogLevel::Info,
        LogLevel::Warn,
        LogLevel::Error,
    ];

    /// The spelling `--level` accepts for this level.
    pub fn name(self) -> &'static str {
        match self {
            LogLevel::Debug => "debug",
            LogLevel::Info => "info",
            LogLevel::Warn => "warn",
            LogLevel::Error => "error",
        }
    }

    /// The `rcl_interfaces/Log` severity constant this level is published as.
    pub fn wire(self) -> u8 {
        match self {
            LogLevel::Debug => 10,
            LogLevel::Info => 20,
            LogLevel::Warn => 30,
            LogLevel::Error => 40,
        }
    }

    /// Parse a `--level` value. The match is ASCII case-insensitive, so
    /// `INFO` and `info` both select [`LogLevel::Info`]; anything else is
    /// refused with the accepted set named.
    pub fn parse(spec: &str) -> Result<Self, LogFilterError> {
        LogLevel::ALL
            .into_iter()
            .find(|level| level.name().eq_ignore_ascii_case(spec))
            .ok_or_else(|| LogFilterError::UnknownLevel {
                given: spec.to_string(),
            })
    }
}

/// Why a `log echo` filter was refused. Every arm names the bad value and
/// what would have been accepted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogFilterError {
    /// `--level` named something that is not a level.
    UnknownLevel { given: String },
}

impl fmt::Display for LogFilterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LogFilterError::UnknownLevel { given } => write!(
                f,
                "unknown log level '{given}'; accepted levels: {}",
                LogLevel::ALL.map(LogLevel::name).join(", ")
            ),
        }
    }
}

impl std::error::Error for LogFilterError {}

impl From<LogFilterError> for CliError {
    fn from(e: LogFilterError) -> Self {
        CliError::Validation(e.to_string())
    }
}

/// What `log echo` lets through.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LogFilter {
    /// Logger names to show, matched as given. Empty shows every node.
    pub nodes: Vec<String>,
    /// The lowest level to show. `None` shows every level.
    pub min_level: Option<LogLevel>,
}

impl LogFilter {
    /// Whether `record` passes this filter: its node is listed (or no node is
    /// listed) and its level is at least `min_level` (or no level is set).
    /// The level compare is numeric over the wire value, so a `FATAL=50`
    /// record from another publisher passes `--level error`.
    pub fn passes(&self, record: &LogRecord) -> bool {
        let node_ok = self.nodes.is_empty() || self.nodes.contains(&record.node);
        let level_ok = self.min_level.is_none_or(|min| record.level >= min.wire());
        node_ok && level_ok
    }

    /// The flags that select this filter: `--node <id>` per node in the
    /// order given, then `--level <level>`. Empty for the empty filter.
    /// Each node name is escaped like a fixed column of a record line, so a
    /// name with a space or a newline cannot read as several flags or break
    /// the line.
    pub fn spelling(&self) -> String {
        let mut out = String::new();
        for node in &self.nodes {
            if !out.is_empty() {
                out.push(' ');
            }
            out.push_str("--node ");
            push_escaped(&mut out, node, Column::Fixed);
        }
        if let Some(level) = self.min_level {
            if !out.is_empty() {
                out.push(' ');
            }
            let _ = write!(out, "--level {}", level.name());
        }
        out
    }
}

/// One decoded log record, the `rcl_interfaces/Log` fields under their
/// Cerulion names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogRecord {
    /// `stamp.sec`: whole seconds of the record time.
    pub stamp_sec: i32,
    /// `stamp.nanosec`: the nanoseconds within that second.
    pub stamp_nanosec: u32,
    /// `level`: the severity constant as published.
    pub level: u8,
    /// `name`: the logger name, which a ROS 2 node sets to its node name.
    pub node: String,
    /// `function`: the `tracing` target.
    pub target: String,
    /// `file`: the source file.
    pub file: String,
    /// `line`: the source line.
    pub line: u32,
    /// `msg`: the text.
    pub msg: String,
}

/// The label a wire level prints as: the `Log.msg` constant names for the
/// five standard values, `L<n>` for any other value.
pub fn level_label(level: u8) -> String {
    match level {
        10 => "DEBUG".to_string(),
        20 => "INFO".to_string(),
        30 => "WARN".to_string(),
        40 => "ERROR".to_string(),
        50 => "FATAL".to_string(),
        other => format!("L{other}"),
    }
}

/// Which column of a line a field prints in. It decides whether a space is
/// escaped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Column {
    /// A column before the message. A space in it would move every later
    /// column, so a space prints as `\u{20}`.
    Fixed,
    /// The message, the last column. It keeps its spaces.
    Message,
    /// One item of a `, ` separated list. Escaped like [`Column::Fixed`],
    /// and a comma prints as `\u{2c}`, so an item never looks like several.
    ListItem,
}

/// Append `text` to `line` so the record stays one line of fixed columns: a
/// backslash prints as `\\`; a newline, carriage return or tab as `\n`, `\r`
/// or `\t`; every other control character (C0, DEL, C1) and the line
/// separators U+2028 and U+2029 as `\u{..}` with the hexadecimal code point;
/// in a [`Column::Fixed`] or [`Column::ListItem`] a space as `\u{20}`; and
/// in a [`Column::ListItem`] a comma as `\u{2c}`. Everything else is copied
/// as is.
fn push_escaped(line: &mut String, text: &str, column: Column) {
    for c in text.chars() {
        match c {
            '\\' => line.push_str("\\\\"),
            '\n' => line.push_str("\\n"),
            '\r' => line.push_str("\\r"),
            '\t' => line.push_str("\\t"),
            ' ' if column != Column::Message => line.push_str("\\u{20}"),
            ',' if column == Column::ListItem => line.push_str("\\u{2c}"),
            c if c.is_control() || c == '\u{2028}' || c == '\u{2029}' => {
                let _ = write!(line, "\\u{{{:x}}}", c as u32);
            }
            c => line.push(c),
        }
    }
}

/// Render one record as the one line `log echo` prints:
///
/// `<sec>.<nanosec> <LEVEL> <node> <target> <file>:<line> <msg>`
///
/// The columns are single-space separated and the message comes last, so
/// splitting a line on single spaces gives five fixed columns and then the
/// message (`cut -d' ' -f6-`). The space before the message is always
/// printed, so an empty message is an empty sixth column, never a missing
/// one. An empty node, target or file prints as `-` (a missing file drops the
/// line number with it). Every field is escaped the
/// same way: a backslash prints as `\\`; a newline, carriage return or tab as
/// `\n`, `\r` or `\t`; every other control character and the line separators
/// U+2028 and U+2029 as `\u{..}` with the hexadecimal code point; and in the
/// five columns before the message a space as `\u{20}`. So one record is
/// always one line and those five columns never contain a space. Nothing is
/// truncated.
pub fn render_line(record: &LogRecord) -> String {
    fn or_dash(field: &str) -> &str {
        if field.is_empty() {
            "-"
        } else {
            field
        }
    }
    let mut line = format!(
        "{}.{:09} {} ",
        record.stamp_sec,
        record.stamp_nanosec,
        level_label(record.level),
    );
    push_escaped(&mut line, or_dash(&record.node), Column::Fixed);
    line.push(' ');
    push_escaped(&mut line, or_dash(&record.target), Column::Fixed);
    line.push(' ');
    if record.file.is_empty() {
        line.push('-');
    } else {
        push_escaped(&mut line, &record.file, Column::Fixed);
        let _ = write!(line, ":{}", record.line);
    }
    line.push(' ');
    push_escaped(&mut line, &record.msg, Column::Message);
    line
}

/// Decode one frame of the log topic: the frame has to carry the
/// `rcl_interfaces/Log` schema hash, then its payload (the bytes after the
/// wire header) has to lay out as one record. `None` otherwise.
///
/// The hash is checked first, the way `topic echo` dispatches on it. The
/// topic name is host-wide, so a frame of another type can arrive on it, and
/// one that happens to be long enough and aligned would otherwise decode
/// into a garbage record.
pub fn decode_log_frame(schema_hash: u64, payload: &[u8]) -> Option<LogRecord> {
    if schema_hash != <Log as ShmMessage>::SCHEMA_HASH {
        return None;
    }
    record_from_payload(payload)
}

/// Decode one frame payload (the bytes after the wire header) as an
/// `rcl_interfaces/Log` record, trusting the caller that the frame carries
/// the `Log` schema hash ([`decode_log_frame`] checks it).
///
/// `None` when the payload cannot be a `Log` frame: too short for the fixed
/// section and offset table, misaligned, or an offset table that points past
/// the payload. A string field that is not UTF-8 decodes lossily (invalid
/// bytes become `U+FFFD`) rather than failing the record.
pub fn record_from_payload(payload: &[u8]) -> Option<LogRecord> {
    let table_end = LogShm::OFFSET_TABLE_OFFSET + LogShm::OFFSET_TABLE_BYTES;
    let aligned = payload
        .as_ptr()
        .align_offset(std::mem::align_of::<LogFixedSection>())
        == 0;
    if payload.len() < table_end || !aligned {
        return None;
    }
    let shm = LogShm::from_bytes(payload);
    Some(LogRecord {
        stamp_sec: shm.stamp.sec,
        stamp_nanosec: shm.stamp.nanosec,
        level: shm.level,
        line: shm.line,
        node: string_field(payload, 0, shm.name())?,
        msg: string_field(payload, 1, shm.msg())?,
        file: string_field(payload, 2, shm.file())?,
        target: string_field(payload, 3, shm.function())?,
    })
}

/// One string field of a `Log` payload: the strict UTF-8 read when it holds,
/// a lossy decode of the same bytes when it does not, `None` when the offset
/// table points outside the payload.
fn string_field(payload: &[u8], index: usize, strict: Result<&str, WireError>) -> Option<String> {
    match strict {
        Ok(text) => Some(text.to_string()),
        Err(WireError::InvalidUtf8 { .. }) => {
            let (offset, length) = read_offset_entry(payload, LogShm::WIRE_FIXED_SIZE, index);
            let start = offset as usize;
            let end = start.checked_add(length as usize)?;
            payload
                .get(start..end)
                .map(|bytes| String::from_utf8_lossy(bytes).into_owned())
        }
        Err(_) => None,
    }
}

/// Resolve the `--node` and `--level` values into a [`LogFilter`].
///
/// A `--node` value is the logger name a record carries, matched as given.
/// It is not checked before the stream starts: the names on the topic are
/// the ones its publishers choose (a ROS 2 node's own name, whatever its
/// graph entry is called), so no graph file can list them. A name that
/// matches nothing is answered by the silence notice instead, which lists
/// the names the topic does carry.
pub fn resolve_filter(nodes: &[String], level: Option<&str>) -> Result<LogFilter, LogFilterError> {
    let min_level = level.map(LogLevel::parse).transpose()?;
    Ok(LogFilter {
        nodes: nodes.to_vec(),
        min_level,
    })
}

/// Keep the first write error of one drain for the loop to act on once the
/// drain returns. The loop writes nothing more once one is kept, and only the
/// first is kept if it is called again.
fn note_write_error(result: io::Result<()>, first: &mut Option<io::Error>) {
    if let Err(e) = result {
        if first.is_none() {
            *first = Some(e);
        }
    }
}

/// What a failed line write means for the stream: a closed reader (`head`,
/// `grep -m1`) ends it quietly with exit 0, like every other pipeline source;
/// every other error is reported.
fn end_on_write_error(e: io::Error) -> CliResult<()> {
    if e.kind() == io::ErrorKind::BrokenPipe {
        Ok(())
    } else {
        Err(CliError::Io(e))
    }
}

/// A frame on the log topic that is not an `rcl_interfaces/Log` record: it
/// carries another schema hash, or its payload does not lay out as one.
/// `log echo` skips it ([`FrameSkips`]); the `[skipped]` line it prints names
/// the hash and the payload size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UndecodableFrame {
    /// The frame's wire schema hash.
    pub schema_hash: u64,
    /// The payload size, the bytes after the wire header.
    pub size_bytes: usize,
}

impl fmt::Display for UndecodableFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.schema_hash == <Log as ShmMessage>::SCHEMA_HASH {
            write!(
                f,
                "a frame on {LOG_TOPIC} carries the rcl_interfaces/Log schema but its {} \
                 payload bytes do not lay out as a record",
                self.size_bytes
            )
        } else {
            write!(
                f,
                "a frame on {LOG_TOPIC} is not an rcl_interfaces/Log record: it carries schema \
                 0x{:016x}, not 0x{:016x}, so a publisher puts another type on the topic",
                self.schema_hash,
                <Log as ShmMessage>::SCHEMA_HASH
            )
        }
    }
}

/// How many frames in a row may carry another type's schema hash, before
/// any record has decoded, until `log echo` takes the topic for another
/// type's and ends ([`ForeignTopic`]). Once one record has decoded the topic
/// does carry `Log` records, and no run of foreign frames ends the stream.
pub const CONFLICT_FRAMES: u32 = 8;

/// The topic carries another type: [`CONFLICT_FRAMES`] frames in a row, and
/// no record before them, had a schema hash that is not the `Log` one. The
/// error `log echo` ends with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ForeignTopic {
    /// The schema hash of the last frame of the run.
    pub schema_hash: u64,
}

impl fmt::Display for ForeignTopic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{LOG_TOPIC} carries another type: {CONFLICT_FRAMES} frames in a row before any \
             record are not rcl_interfaces/Log records (the last carries schema 0x{:016x}, \
             not 0x{:016x}), so a publisher puts another type on the topic",
            self.schema_hash,
            <Log as ShmMessage>::SCHEMA_HASH
        )
    }
}

/// What `log echo` does with one frame that does not decode as a record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkipVerdict {
    /// Skip it and print a `[skipped]` line: the first
    /// undecodable frame since the last record, or one that takes the total
    /// across a power of ten (10, 100, 1000 and so on).
    Announce {
        /// Undecodable frames skipped since `log echo` attached, this one
        /// included.
        total: u64,
    },
    /// Skip it with no line: a repeat after an announced one.
    Quiet,
    /// End the stream with [`ForeignTopic`].
    Conflict(ForeignTopic),
}

/// The undecodable frames `log echo` has skipped, and when they amount to a
/// conflict.
///
/// One bad frame, from any publisher on the topic, is skipped and counted:
/// it never ends the stream, so a malformed or hostile publisher cannot stop
/// the operator's log view. The lines it prints go through the shared
/// [`FailureRegimeLatch`], so a flood of bad frames prints a bounded number of
/// lines. The stream ends only on [`ForeignTopic`]. Holds integers and a
/// flag only, so counting a frame costs no allocation; only an announced
/// frame formats a line.
#[derive(Debug, Default)]
pub struct FrameSkips {
    latch: FailureRegimeLatch,
    decoded_any: bool,
    foreign_run: u32,
}

impl FrameSkips {
    /// A record decoded. Returns the frames skipped so far when it closes a
    /// run that had quiet skips in it, for the `records decode again` line;
    /// `None` when there is nothing to report.
    pub fn on_record(&mut self) -> Option<u64> {
        self.decoded_any = true;
        self.foreign_run = 0;
        self.latch.on_success().map(|_| self.latch.total_failures())
    }

    /// A frame did not decode as a record: count it and say what to do.
    pub fn on_undecodable(&mut self, frame: &UndecodableFrame) -> SkipVerdict {
        if frame.schema_hash == <Log as ShmMessage>::SCHEMA_HASH {
            self.foreign_run = 0;
        } else {
            self.foreign_run = self.foreign_run.saturating_add(1);
        }
        let decision = self.latch.on_failure();
        if !self.decoded_any && self.foreign_run >= CONFLICT_FRAMES {
            return SkipVerdict::Conflict(ForeignTopic {
                schema_hash: frame.schema_hash,
            });
        }
        match decision {
            RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => SkipVerdict::Announce {
                total: self.latch.total_failures(),
            },
            RegimeDecision::Suppressed { .. } => SkipVerdict::Quiet,
        }
    }

    /// Undecodable frames skipped since `log echo` attached.
    pub fn skipped(&self) -> u64 {
        self.latch.total_failures()
    }
}

/// `count` frames in words: `1 frame`, `2 frames`.
fn frame_count(count: u64) -> String {
    if count == 1 {
        "1 frame".to_string()
    } else {
        format!("{count} frames")
    }
}

/// The stderr line for an announced skip: what the frame was and how many
/// have been skipped.
fn skip_notice(frame: &UndecodableFrame, total: u64) -> String {
    format!("[skipped] {frame}; {} skipped so far", frame_count(total))
}

/// The stderr line when records decode again after quiet skips.
fn recovery_notice(total: u64) -> String {
    format!(
        "[skipped] {LOG_TOPIC}: records decode again; {} skipped so far",
        frame_count(total)
    )
}

/// The stderr line `log echo` ends with when it skipped any frame.
fn skip_summary(total: u64) -> String {
    format!(
        "[skipped] {LOG_TOPIC}: {} skipped in all",
        frame_count(total)
    )
}

/// The most distinct logger names [`SeenNodes`] keeps. A topic with more
/// publishers than this is listed with `and others` after the first ones in
/// name order, so a flood of distinct names cannot grow the set without end.
pub const SEEN_NODES_CAP: usize = 16;

/// The logger names the records on the topic carried, kept while a `--node`
/// filter is set so the silence notice can list what the topic does carry.
/// Sorted, deduplicated and bounded by [`SEEN_NODES_CAP`]; an empty name is
/// not kept.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SeenNodes {
    names: std::collections::BTreeSet<String>,
    more: bool,
}

impl SeenNodes {
    /// Keep `name`. A name already kept costs a lookup and no allocation;
    /// a new name past the cap is counted as `and others`, not kept.
    pub fn note(&mut self, name: &str) {
        if name.is_empty() || self.names.contains(name) {
            return;
        }
        if self.names.len() < SEEN_NODES_CAP {
            self.names.insert(name.to_string());
        } else {
            self.more = true;
        }
    }

    /// The kept names as the notice lists them: comma separated in name
    /// order, each escaped like a fixed column of a record line with a comma
    /// as `\u{2c}`, then `and others` when names past the cap were seen.
    /// `None` when no name was kept.
    fn listing(&self) -> Option<String> {
        if self.names.is_empty() {
            return None;
        }
        let mut out = String::new();
        for (i, name) in self.names.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            push_escaped(&mut out, name, Column::ListItem);
        }
        if self.more {
            out.push_str(", and others");
        }
        Some(out)
    }
}

/// The stderr line `log echo` prints while nothing has been shown for a
/// while: how long, what a record had to match and, under a `--node`
/// filter, the logger names the topic carried instead.
fn silence_notice(filter: &LogFilter, silent_for: Duration, seen: &SeenNodes) -> String {
    let secs = silent_for.as_secs_f64();
    if filter.nodes.is_empty() && filter.min_level.is_none() {
        return format!("[waiting] {LOG_TOPIC}: 0 records in {secs:.0}s");
    }
    let mut line = format!(
        "[waiting] {LOG_TOPIC}: 0 records matching {} in {secs:.0}s",
        filter.spelling()
    );
    if !filter.nodes.is_empty() {
        if let Some(names) = seen.listing() {
            let _ = write!(line, "; nodes on the topic: {names}");
        }
    }
    line
}

/// Stream the local log topic to `writer`, one rendered record per line,
/// until `running` clears or the reader of `writer` goes away. The
/// `[waiting]` and `[skipped]` lines go to `notices` (stderr in the CLI), so
/// `writer` carries records only.
///
/// Refuses when nothing on this host publishes [`LOG_TOPIC`]: the topic is
/// read locally only, like `topic echo --local`, so there is no remote rung
/// to wait on. A frame that is not an `rcl_interfaces/Log` record
/// ([`UndecodableFrame`]) is skipped and counted ([`FrameSkips`]), and the
/// stream goes on; it ends with [`ForeignTopic`] only when the topic carries
/// another type.
///
/// One drain empties the subscriber queue before control comes back to the
/// loop. Once `running` clears, the reader goes away or the topic turns out
/// to carry another type, the rest of that drain is popped and dropped
/// without rendering or writing, so a busy topic still ends the stream at
/// once.
pub fn log_echo(
    filter: &LogFilter,
    running: Arc<AtomicBool>,
    writer: &mut dyn Write,
    notices: &mut dyn Write,
) -> CliResult<()> {
    let mut skips = FrameSkips::default();
    let result = echo_until_done(filter, &running, writer, notices, &mut skips);
    if skips.skipped() > 0 {
        let _ = writeln!(notices, "{}", skip_summary(skips.skipped()));
    }
    result
}

fn echo_until_done(
    filter: &LogFilter,
    running: &Arc<AtomicBool>,
    writer: &mut dyn Write,
    notices: &mut dyn Write,
    skips: &mut FrameSkips,
) -> CliResult<()> {
    let listed = topic_cmd::topic_list()?.iter().any(|t| t.name == LOG_TOPIC);
    if !listed {
        return Err(CliError::Validation(format!(
            "no node on this host publishes {LOG_TOPIC}; a ROS 2 node publishes it when \
             started with `cerulion ros2 run <package> <executable>`"
        )));
    }
    let (_transport, subscriber, _source) = match topic_cmd::ensure_topic_available(
        LOG_TOPIC,
        None,
        Some(running.as_ref()),
        TopicScope::Local,
    ) {
        Ok(v) => v,
        Err(_) if topic_cmd::interrupted_before_observing(Some(running.as_ref())) => return Ok(()),
        Err(e) => return Err(e),
    };

    let mut consecutive_wait_errors: u32 = 0;
    let mut idle_churn: u32 = 0;
    // When the current run of "nothing shown" began (attach time, then reset
    // by every shown record) and how many notices it has had.
    let mut silence_since = std::time::Instant::now();
    let mut notices_emitted: u32 = 0;
    // The logger names seen, for the notice to list under a `--node` filter.
    let mut seen = SeenNodes::default();
    while running.load(Ordering::Relaxed) {
        let wait_started = std::time::Instant::now();
        let mut shown: usize = 0;
        let mut write_error: Option<io::Error> = None;
        let mut conflict: Option<ForeignTopic> = None;
        let received =
            subscriber
                .subscriber
                .wait_for_message(topic_cmd::OBSERVER_HEARTBEAT, |msg| {
                    // The stream is ending: drop the rest of this drain unread.
                    if write_error.is_some()
                        || conflict.is_some()
                        || !running.load(Ordering::Relaxed)
                    {
                        return;
                    }
                    let schema_hash = msg.header().schema_hash;
                    let payload = msg.payload();
                    match decode_log_frame(schema_hash, payload) {
                        Some(record) => {
                            if let Some(total) = skips.on_record() {
                                let _ = writeln!(notices, "{}", recovery_notice(total));
                            }
                            if !filter.nodes.is_empty() {
                                seen.note(&record.node);
                            }
                            if filter.passes(&record) {
                                shown += 1;
                                note_write_error(
                                    writeln!(writer, "{}", render_line(&record)),
                                    &mut write_error,
                                );
                            }
                        }
                        None => {
                            let frame = UndecodableFrame {
                                schema_hash,
                                size_bytes: payload.len(),
                            };
                            match skips.on_undecodable(&frame) {
                                SkipVerdict::Announce { total } => {
                                    let _ = writeln!(notices, "{}", skip_notice(&frame, total));
                                }
                                SkipVerdict::Quiet => {}
                                SkipVerdict::Conflict(foreign) => conflict = Some(foreign),
                            }
                        }
                    }
                });
        if let Some(e) = write_error {
            return end_on_write_error(e);
        }
        if let Some(foreign) = conflict {
            return Err(CliError::Validation(foreign.to_string()));
        }
        match received {
            // `received` counts the frames the drain delivered, shown or
            // not: a flowing topic is never paced, whatever the filter keeps.
            Ok(received) => {
                let _ = topic_cmd::decide_wait_outcome(
                    true,
                    running.load(Ordering::Relaxed),
                    &mut consecutive_wait_errors,
                );
                if shown > 0 {
                    silence_since = std::time::Instant::now();
                    notices_emitted = 0;
                } else {
                    let silent_for = silence_since.elapsed();
                    if topic_cmd::silence_notice_due(silent_for, notices_emitted) {
                        let _ = writeln!(notices, "{}", silence_notice(filter, silent_for, &seen));
                        notices_emitted = notices_emitted.saturating_add(1);
                    }
                }
                if let Some(pause) = topic_cmd::decide_observer_pacing(
                    received,
                    wait_started.elapsed(),
                    topic_cmd::OBSERVER_HEARTBEAT,
                    &mut idle_churn,
                ) {
                    topic_cmd::note_pacing_engaged();
                    std::thread::sleep(pause); // ALLOW: starvation floor, engaged only on a progress-free early-returning wait, never on a flowing or normally-blocking topic; a CPU bound on external event churn, not an observer poll.
                }
            }
            Err(e) => match topic_cmd::decide_wait_outcome(
                false,
                running.load(Ordering::Relaxed),
                &mut consecutive_wait_errors,
            ) {
                topic_cmd::WaitOutcomeAction::ExitCleanly => break,
                topic_cmd::WaitOutcomeAction::Continue => continue,
                topic_cmd::WaitOutcomeAction::Fail => {
                    return Err(CliError::Validation(format!(
                        "receive failed on topic '{LOG_TOPIC}': {e}"
                    )));
                }
            },
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(level: u8, node: &str, msg: &str) -> LogRecord {
        LogRecord {
            stamp_sec: 1_700_000_000,
            stamp_nanosec: 123_456_789,
            level,
            node: node.to_string(),
            target: "camera::tick".to_string(),
            file: "src/lib.rs".to_string(),
            line: 42,
            msg: msg.to_string(),
        }
    }

    // ---- --level parsing: happy, edge, every error arm ----

    #[test]
    fn every_accepted_level_spelling_parses_to_its_level() {
        let cases = [
            ("debug", LogLevel::Debug),
            ("info", LogLevel::Info),
            ("warn", LogLevel::Warn),
            ("error", LogLevel::Error),
            // ASCII case-insensitive.
            ("INFO", LogLevel::Info),
            ("Warn", LogLevel::Warn),
        ];
        for (spec, want) in cases {
            assert_eq!(LogLevel::parse(spec), Ok(want), "{spec}");
        }
        for level in LogLevel::ALL {
            assert_eq!(LogLevel::parse(level.name()), Ok(level));
        }
    }

    #[test]
    fn levels_order_by_severity_and_carry_the_log_msg_constants() {
        assert!(LogLevel::Debug < LogLevel::Info);
        assert!(LogLevel::Info < LogLevel::Warn);
        assert!(LogLevel::Warn < LogLevel::Error);
        assert_eq!(
            LogLevel::ALL.map(LogLevel::wire),
            [10, 20, 30, 40],
            "the Log.msg DEBUG/INFO/WARN/ERROR constants"
        );
    }

    #[test]
    fn an_unknown_level_is_refused_naming_the_value_and_the_accepted_set() {
        for bad in ["warning", "trace", "fatal", "", " info", "3"] {
            let err = LogLevel::parse(bad).expect_err(bad);
            assert_eq!(
                err,
                LogFilterError::UnknownLevel {
                    given: bad.to_string()
                }
            );
            assert_eq!(
                err.to_string(),
                format!("unknown log level '{bad}'; accepted levels: debug, info, warn, error")
            );
        }
    }

    // ---- the node and level filters ----

    #[test]
    fn an_empty_filter_passes_every_record() {
        let filter = LogFilter::default();
        assert!(filter.passes(&record(10, "camera", "x")));
        assert!(filter.passes(&record(40, "planner", "")));
        // A non-standard level and an empty node id pass too: an empty
        // filter claims nothing about them.
        assert!(filter.passes(&record(25, "", "")));
    }

    #[test]
    fn the_node_filter_passes_only_the_listed_nodes() {
        let filter = LogFilter {
            nodes: vec!["camera".into(), "planner".into()],
            min_level: None,
        };
        assert!(filter.passes(&record(20, "camera", "x")));
        assert!(filter.passes(&record(20, "planner", "x")));
        assert!(!filter.passes(&record(20, "lidar", "x")));
        // Exact id match: a prefix or a different case is another node.
        assert!(!filter.passes(&record(20, "camera2", "x")));
        assert!(!filter.passes(&record(20, "Camera", "x")));
    }

    #[test]
    fn the_level_filter_is_a_minimum_over_the_wire_value() {
        let at_least_warn = LogFilter {
            nodes: vec![],
            min_level: Some(LogLevel::Warn),
        };
        assert!(!at_least_warn.passes(&record(10, "n", "")));
        assert!(!at_least_warn.passes(&record(20, "n", "")));
        assert!(at_least_warn.passes(&record(30, "n", "")));
        assert!(at_least_warn.passes(&record(40, "n", "")));
        // FATAL=50 from another publisher is above error, so it shows.
        assert!(at_least_warn.passes(&record(50, "n", "")));
        // A non-standard value compares numerically: 25 is above info and
        // below warn.
        let at_least_info = LogFilter {
            nodes: vec![],
            min_level: Some(LogLevel::Info),
        };
        assert!(at_least_info.passes(&record(25, "n", "")));
        assert!(!at_least_warn.passes(&record(25, "n", "")));
        // `--level debug` shows every standard level and hides nothing
        // standard; a value below debug is hidden.
        let at_least_debug = LogFilter {
            nodes: vec![],
            min_level: Some(LogLevel::Debug),
        };
        assert!(at_least_debug.passes(&record(10, "n", "")));
        assert!(!at_least_debug.passes(&record(5, "n", "")));
    }

    #[test]
    fn node_and_level_filters_both_have_to_pass() {
        let filter = LogFilter {
            nodes: vec!["camera".into()],
            min_level: Some(LogLevel::Error),
        };
        assert!(filter.passes(&record(40, "camera", "")));
        assert!(!filter.passes(&record(30, "camera", "")));
        assert!(!filter.passes(&record(40, "planner", "")));
    }

    #[test]
    fn a_flood_from_one_node_does_not_leak_past_a_filter_on_another() {
        // Adversarial: ten thousand records from one chatty node around a
        // single record from the node asked for.
        let filter = LogFilter {
            nodes: vec!["planner".into()],
            min_level: None,
        };
        let mut records: Vec<LogRecord> = (0..10_000)
            .map(|i| record(20, "camera", &format!("frame {i}")))
            .collect();
        records.insert(5_000, record(30, "planner", "replanning"));
        let shown: Vec<String> = records
            .iter()
            .filter(|r| filter.passes(r))
            .map(render_line)
            .collect();
        assert_eq!(
            shown,
            ["1700000000.123456789 WARN planner camera::tick src/lib.rs:42 replanning"]
        );
    }

    #[test]
    fn a_filter_spells_itself_as_the_flags_that_select_it() {
        assert_eq!(LogFilter::default().spelling(), "");
        assert_eq!(
            LogFilter {
                nodes: vec!["camera".into(), "planner".into()],
                min_level: Some(LogLevel::Warn),
            }
            .spelling(),
            "--node camera --node planner --level warn"
        );
        assert_eq!(
            LogFilter {
                nodes: vec![],
                min_level: Some(LogLevel::Error),
            }
            .spelling(),
            "--level error"
        );
        assert_eq!(
            LogFilter {
                nodes: vec!["lidar".into()],
                min_level: None,
            }
            .spelling(),
            "--node lidar"
        );
        // A name with a space or a newline stays one flag on one line.
        assert_eq!(
            LogFilter {
                nodes: vec!["my node".into(), "a\nb".into()],
                min_level: Some(LogLevel::Info),
            }
            .spelling(),
            "--node my\\u{20}node --node a\\nb --level info"
        );
    }

    // ---- the line format ----

    #[test]
    fn a_record_renders_as_one_fixed_column_line_with_the_message_last() {
        let line = render_line(&record(20, "camera", "frame dropped"));
        assert_eq!(
            line,
            "1700000000.123456789 INFO camera camera::tick src/lib.rs:42 frame dropped"
        );
        // Five single-space-separated columns, then the message with its
        // own spaces.
        let columns: Vec<&str> = line.splitn(6, ' ').collect();
        assert_eq!(
            columns,
            [
                "1700000000.123456789",
                "INFO",
                "camera",
                "camera::tick",
                "src/lib.rs:42",
                "frame dropped"
            ]
        );
    }

    /// The extraction the docs give (`awk '{print $3}'` for the node,
    /// `cut -d' ' -f6-` for the message) holds on a line of every level: a
    /// four-letter label and a five-letter one put the same text in the same
    /// field.
    #[test]
    fn the_documented_column_extraction_holds_for_every_level_label() {
        for level in [10, 20, 30, 40, 50, 7] {
            let line = render_line(&record(level, "camera", "no path to goal"));
            // `cut -d' ' -f6-`: split on every single space, fields 6 on.
            let fields: Vec<&str> = line.split(' ').collect();
            assert_eq!(fields[5..].join(" "), "no path to goal", "{line}");
            // `awk '{print $3}'`: split on runs of whitespace, field 3.
            let words: Vec<&str> = line.split_whitespace().collect();
            assert_eq!(words[2], "camera", "{line}");
            assert_eq!(words[1], level_label(level), "{line}");
        }
    }

    #[test]
    fn every_standard_level_has_its_label_and_others_print_their_number() {
        assert_eq!(level_label(10), "DEBUG");
        assert_eq!(level_label(20), "INFO");
        assert_eq!(level_label(30), "WARN");
        assert_eq!(level_label(40), "ERROR");
        assert_eq!(level_label(50), "FATAL");
        assert_eq!(level_label(0), "L0");
        assert_eq!(level_label(25), "L25");
        assert_eq!(level_label(255), "L255");
    }

    #[test]
    fn the_stamp_prints_nine_nanosecond_digits_and_keeps_a_negative_second() {
        let mut r = record(20, "n", "m");
        r.stamp_sec = 7;
        r.stamp_nanosec = 5;
        assert!(render_line(&r).starts_with("7.000000005 "));
        r.stamp_sec = -3;
        r.stamp_nanosec = 999_999_999;
        assert!(render_line(&r).starts_with("-3.999999999 "));
        r.stamp_sec = 0;
        r.stamp_nanosec = 0;
        assert!(render_line(&r).starts_with("0.000000000 "));
    }

    #[test]
    fn empty_fields_print_as_a_dash_and_an_empty_message_is_an_empty_sixth_column() {
        let mut r = record(20, "", "");
        r.target = String::new();
        r.file = String::new();
        r.line = 0;
        assert_eq!(render_line(&r), "1700000000.123456789 INFO - - - ");
        // A line number with no file is dropped with the file.
        r.line = 42;
        assert_eq!(render_line(&r), "1700000000.123456789 INFO - - - ");
        // A file with line 0 keeps the file.
        r.file = "src/lib.rs".into();
        r.line = 0;
        assert_eq!(
            render_line(&r),
            "1700000000.123456789 INFO - - src/lib.rs:0 "
        );
    }

    /// The documented `cut -d' ' -f6-` gives the message, empty included: on
    /// an empty message it is the empty string, never the whole line.
    #[test]
    fn the_message_column_of_an_empty_message_is_empty_not_the_whole_line() {
        let line = render_line(&record(30, "camera", ""));
        assert_eq!(
            line,
            "1700000000.123456789 WARN camera camera::tick src/lib.rs:42 "
        );
        let fields: Vec<&str> = line.split(' ').collect();
        assert_eq!(fields.len(), 6, "{line}");
        assert_eq!(fields[5..].join(" "), "", "{line}");
    }

    #[test]
    fn a_multi_line_message_renders_as_one_line() {
        let r = record(30, "n", "first\nsecond\r\nthird");
        let line = render_line(&r);
        assert!(!line.contains('\n') && !line.contains('\r'), "{line:?}");
        assert!(line.ends_with(" first\\nsecond\\r\\nthird"), "{line:?}");
    }

    /// Every escape the line format promises, one vector each, written out
    /// by hand: a backslash doubles (so a literal `\n` in a message is told
    /// apart from a newline), a tab is `\t`, and every other control
    /// character prints as its code point, so a terminal escape sequence or
    /// a line separator from a publisher cannot move the cursor or split the
    /// line.
    #[test]
    fn backslashes_tabs_and_control_characters_in_the_message_are_escaped() {
        let cases: [(&str, &str); 8] = [
            ("a\\nb", "a\\\\nb"),
            ("a\\b", "a\\\\b"),
            ("a\tb", "a\\tb"),
            ("\u{1b}[31mred\u{1b}[0m", "\\u{1b}[31mred\\u{1b}[0m"),
            ("nul\0here", "nul\\u{0}here"),
            ("del\u{7f}", "del\\u{7f}"),
            ("c1\u{85}", "c1\\u{85}"),
            ("sep\u{2028}line\u{2029}", "sep\\u{2028}line\\u{2029}"),
        ];
        for (msg, want) in cases {
            let line = render_line(&record(20, "n", msg));
            assert_eq!(
                line,
                format!("1700000000.123456789 INFO n camera::tick src/lib.rs:42 {want}"),
                "{msg:?}"
            );
        }
        // A space in the message is kept; a non-ASCII letter is copied as is.
        let line = render_line(&record(20, "n", "zwei Wörter"));
        assert!(line.ends_with(" zwei Wörter"), "{line}");
    }

    /// The columns before the message never contain a space or a line
    /// break, whatever a publisher puts in the node id, the target or the
    /// file path, so the documented column extraction holds.
    #[test]
    fn a_space_or_control_character_in_a_fixed_column_is_escaped() {
        let mut r = record(20, "my node", "m");
        r.target = "a\tb".into();
        r.file = "/home/me/my project/src/lib.rs".into();
        let line = render_line(&r);
        assert_eq!(
            line,
            "1700000000.123456789 INFO my\\u{20}node a\\tb /home/me/my\\u{20}project/src/lib.rs:42 m"
        );
        let fields: Vec<&str> = line.split(' ').collect();
        assert_eq!(fields.len(), 6, "{line}");
        assert_eq!(fields[2], "my\\u{20}node");
        assert_eq!(fields[5], "m");
        // A newline in a node id cannot split the line either.
        let line = render_line(&record(20, "bad\nnode", "m"));
        assert!(!line.contains('\n'), "{line:?}");
        assert!(line.contains(" bad\\nnode "), "{line}");
    }

    #[test]
    fn a_very_long_message_is_not_truncated() {
        let long: String = "x".repeat(100_000);
        let line = render_line(&record(20, "n", &long));
        assert!(line.ends_with(&long));
        assert_eq!(
            line.len(),
            "1700000000.123456789 INFO n camera::tick src/lib.rs:42 ".len() + long.len()
        );
    }

    #[test]
    fn rendering_the_same_records_twice_is_bit_identical() {
        let records: Vec<LogRecord> = (0..500u32)
            .map(|i| {
                let mut r = record(
                    10 + (i % 5) as u8 * 10,
                    &format!("node{}", i % 7),
                    &format!("m{i}\t\\"),
                );
                r.stamp_nanosec = i * 1_000_003;
                r.line = i;
                r
            })
            .collect();
        let run = || {
            records
                .iter()
                .map(|r| render_line(r) + "\n")
                .collect::<String>()
        };
        let first = run();
        let second = run();
        assert_eq!(first, second);
        assert_eq!(first.lines().count(), 500);
    }

    // ---- decoding a Log frame ----

    /// The fixed section and offset table sizes the hand-built payloads below
    /// assume, pinned against the generated type so a layout change here is a
    /// loud failure, not a silently wrong oracle.
    #[test]
    fn the_hand_built_payload_layout_matches_the_generated_type() {
        assert_eq!(
            LogShm::WIRE_FIXED_SIZE,
            16,
            "stamp (8) + level (1 + 3 pad) + line (4)"
        );
        assert_eq!(LogShm::VARIABLE_FIELD_COUNT, 4, "name, msg, file, function");
        assert_eq!(LogShm::OFFSET_TABLE_OFFSET, 16);
        assert_eq!(LogShm::OFFSET_TABLE_BYTES, 32);
    }

    /// A `Log` payload laid out by hand: the 16-byte fixed section, the
    /// 4-entry offset table and the four strings in declaration order
    /// (`name`, `msg`, `file`, `function`). Returned 8-aligned inside a
    /// buffer, the way the transport hands a payload over.
    fn log_payload(
        sec: i32,
        nanosec: u32,
        level: u8,
        line: u32,
        strings: [&[u8]; 4],
    ) -> (Vec<u8>, usize, usize) {
        let mut body = Vec::new();
        body.extend_from_slice(&sec.to_le_bytes());
        body.extend_from_slice(&nanosec.to_le_bytes());
        body.push(level);
        body.extend_from_slice(&[0, 0, 0]);
        body.extend_from_slice(&line.to_le_bytes());
        let mut cursor = 16 + 8 * 4;
        for s in strings {
            body.extend_from_slice(&(cursor as u32).to_le_bytes());
            body.extend_from_slice(&(s.len() as u32).to_le_bytes());
            cursor += s.len();
        }
        for s in strings {
            body.extend_from_slice(s);
        }
        aligned(&body)
    }

    /// Copy `body` into a buffer at an 8-aligned offset; returns the buffer
    /// and the `[start, end)` range of the copy.
    fn aligned(body: &[u8]) -> (Vec<u8>, usize, usize) {
        let mut buf = vec![0u8; body.len() + 8];
        let start = buf.as_ptr().align_offset(8);
        buf[start..start + body.len()].copy_from_slice(body);
        (buf, start, start + body.len())
    }

    #[test]
    fn a_log_frame_decodes_to_its_record() {
        let (buf, s, e) = log_payload(
            1_700_000_000,
            123_456_789,
            30,
            42,
            [b"camera", b"frame dropped", b"src/lib.rs", b"camera::tick"],
        );
        let got = record_from_payload(&buf[s..e]).expect("decodes");
        assert_eq!(got, record(30, "camera", "frame dropped"));
        assert_eq!(
            decode_log_frame(<Log as ShmMessage>::SCHEMA_HASH, &buf[s..e]),
            Some(got.clone())
        );
        assert_eq!(
            render_line(&got),
            "1700000000.123456789 WARN camera camera::tick src/lib.rs:42 frame dropped"
        );
    }

    /// A frame of another type on the log topic is skipped on its schema
    /// hash, even when its payload is long enough and aligned to lay out as
    /// a record: here a well-formed `Log` payload under `std_msgs/String`'s
    /// hash, the worst case for a layout-only check.
    #[test]
    fn a_frame_with_another_schema_hash_is_not_decoded_as_a_record() {
        use native_ros2_messages::std_msgs::String as RosString;
        let (buf, s, e) = log_payload(1, 2, 20, 3, [b"camera", b"x", b"f", b"t"]);
        assert!(
            record_from_payload(&buf[s..e]).is_some(),
            "the payload itself decodes"
        );
        let other = <RosString as ShmMessage>::SCHEMA_HASH;
        assert_ne!(other, <Log as ShmMessage>::SCHEMA_HASH);
        assert_eq!(decode_log_frame(other, &buf[s..e]), None);
        assert_eq!(decode_log_frame(0, &buf[s..e]), None);
        assert_eq!(decode_log_frame(u64::MAX, &buf[s..e]), None);
    }

    #[test]
    fn empty_strings_decode_to_empty_fields() {
        let (buf, s, e) = log_payload(1, 2, 20, 0, [b"", b"", b"", b""]);
        let got = record_from_payload(&buf[s..e]).expect("decodes");
        assert_eq!(got.node, "");
        assert_eq!(got.msg, "");
        assert_eq!(got.file, "");
        assert_eq!(got.target, "");
        assert_eq!(
            (got.stamp_sec, got.stamp_nanosec, got.level, got.line),
            (1, 2, 20, 0)
        );
    }

    #[test]
    fn a_non_utf8_message_decodes_lossily_instead_of_dropping_the_record() {
        let (buf, s, e) = log_payload(0, 0, 40, 1, [b"n", b"bad \xff byte", b"f", b"t"]);
        let got = record_from_payload(&buf[s..e]).expect("decodes");
        assert_eq!(got.msg, "bad \u{FFFD} byte");
        assert_eq!(got.node, "n");
    }

    #[test]
    fn a_short_or_misaligned_or_corrupt_payload_is_none_and_never_panics() {
        let hash = <Log as ShmMessage>::SCHEMA_HASH;
        // Too short for the fixed section and table.
        let (buf, s, _) = aligned(&[0u8; 47]);
        assert_eq!(decode_log_frame(hash, &buf[s..s + 47]), None);
        assert_eq!(decode_log_frame(hash, &[]), None);
        // Misaligned by one byte.
        let (buf, s, e) = log_payload(1, 2, 20, 3, [b"a", b"b", b"c", b"d"]);
        assert_eq!(decode_log_frame(hash, &buf[s + 1..e]), None);
        // An offset table pointing past the payload.
        let (mut buf, s, e) = log_payload(1, 2, 20, 3, [b"a", b"b", b"c", b"d"]);
        buf[s + 16..s + 20].copy_from_slice(&u32::MAX.to_le_bytes());
        assert_eq!(decode_log_frame(hash, &buf[s..e]), None);
    }

    // ---- the write error and the silence notice ----

    /// A writer whose reader has gone away: every write is `BrokenPipe`.
    struct ClosedPipe;

    impl Write for ClosedPipe {
        fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
            Err(io::Error::from(io::ErrorKind::BrokenPipe))
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn a_closed_reader_ends_the_stream_quietly_and_any_other_write_error_is_reported() {
        let mut first = None;
        note_write_error(writeln!(ClosedPipe, "line"), &mut first);
        let e = first.as_ref().expect("the write failed");
        assert_eq!(e.kind(), io::ErrorKind::BrokenPipe);
        assert!(end_on_write_error(first.take().unwrap()).is_ok());

        let denied = io::Error::from(io::ErrorKind::PermissionDenied);
        let err = end_on_write_error(denied).expect_err("reported");
        assert!(
            matches!(err, CliError::Io(ref e) if e.kind() == io::ErrorKind::PermissionDenied),
            "{err:?}"
        );
    }

    #[test]
    fn the_first_write_error_of_a_drain_is_the_one_kept() {
        let mut first = None;
        note_write_error(Ok(()), &mut first);
        assert!(first.is_none());
        note_write_error(Err(io::Error::from(io::ErrorKind::BrokenPipe)), &mut first);
        note_write_error(Err(io::Error::from(io::ErrorKind::Other)), &mut first);
        assert_eq!(first.unwrap().kind(), io::ErrorKind::BrokenPipe);
    }

    #[test]
    fn the_silence_notice_names_the_topic_the_elapsed_time_and_the_filter() {
        assert_eq!(
            silence_notice(
                &LogFilter::default(),
                Duration::from_secs(3),
                &SeenNodes::default()
            ),
            "[waiting] /rosout: 0 records in 3s"
        );
        let filter = LogFilter {
            nodes: vec!["camera".into()],
            min_level: Some(LogLevel::Warn),
        };
        assert_eq!(
            silence_notice(&filter, Duration::from_millis(8_400), &SeenNodes::default()),
            "[waiting] /rosout: 0 records matching --node camera --level warn in 8s"
        );
    }

    /// The case a graph id cannot answer: a `ros2:` entry `sidecar` runs a
    /// node that names itself `talker`. `--node sidecar` matches nothing, and
    /// the notice lists the name the records do carry.
    #[test]
    fn under_a_node_filter_the_silence_notice_lists_the_names_on_the_topic() {
        let filter = LogFilter {
            nodes: vec!["sidecar".into()],
            min_level: None,
        };
        let mut seen = SeenNodes::default();
        for name in ["talker", "listener", "talker", ""] {
            seen.note(name);
        }
        assert_eq!(
            silence_notice(&filter, Duration::from_secs(3), &seen),
            "[waiting] /rosout: 0 records matching --node sidecar in 3s; nodes on the topic: \
             listener, talker"
        );
        // Nothing seen yet: no list.
        assert_eq!(
            silence_notice(&filter, Duration::from_secs(3), &SeenNodes::default()),
            "[waiting] /rosout: 0 records matching --node sidecar in 3s"
        );
        // A level filter alone never lists names.
        let level_only = LogFilter {
            nodes: vec![],
            min_level: Some(LogLevel::Error),
        };
        assert_eq!(
            silence_notice(&level_only, Duration::from_secs(3), &seen),
            "[waiting] /rosout: 0 records matching --level error in 3s"
        );
    }

    /// The record's own name is what `--node` matches: the real logger name
    /// passes, and a name that is only a graph entry id does not.
    #[test]
    fn the_node_filter_matches_the_logger_name_the_record_carries() {
        let by_logger = LogFilter {
            nodes: vec!["talker".into()],
            min_level: None,
        };
        assert!(by_logger.passes(&record(20, "talker", "hello")));
        let by_entry_id = LogFilter {
            nodes: vec!["sidecar".into()],
            min_level: None,
        };
        assert!(!by_entry_id.passes(&record(20, "talker", "hello")));
        assert_eq!(
            resolve_filter(&["talker".to_string()], None).expect("no graph is needed"),
            by_logger
        );
    }

    /// Adversarial: a flood of distinct names keeps the first ones in name
    /// order up to the cap and says `and others`; a name in a fixed column
    /// is escaped like a record line.
    #[test]
    fn the_seen_names_are_bounded_sorted_and_escaped() {
        let mut seen = SeenNodes::default();
        for i in (0..1000).rev() {
            seen.note(&format!("n{i:04}"));
        }
        let listing = seen.listing().expect("names were seen");
        let names: Vec<&str> = listing.split(", ").collect();
        assert_eq!(names.len(), SEEN_NODES_CAP + 1, "{listing}");
        assert_eq!(names.last(), Some(&"and others"));
        // The cap keeps the first names noted, listed in name order.
        assert_eq!(names[0], "n0984");
        assert_eq!(names[SEEN_NODES_CAP - 1], "n0999");
        let mut odd = SeenNodes::default();
        odd.note("my node");
        odd.note("bad\nname");
        odd.note("a, b");
        odd.note("c,d\\e");
        // Hand-written: a comma in a name never reads as a separator, and a
        // backslash in a name is still doubled.
        assert_eq!(
            odd.listing().as_deref(),
            Some("a\\u{2c}\\u{20}b, bad\\nname, c\\u{2c}d\\\\e, my\\u{20}node")
        );
        assert_eq!(
            odd.listing().expect("names were seen").split(", ").count(),
            4
        );
        assert_eq!(SeenNodes::default().listing(), None);
    }

    // ---- the wrong-schema outcome ----

    #[test]
    fn an_undecodable_frame_of_another_type_names_both_schema_hashes() {
        let log = <Log as ShmMessage>::SCHEMA_HASH;
        let frame = UndecodableFrame {
            schema_hash: 0x0123_4567_89ab_cdef,
            size_bytes: 12,
        };
        assert_eq!(
            frame.to_string(),
            format!(
                "a frame on /rosout is not an rcl_interfaces/Log record: it carries schema \
                 0x0123456789abcdef, not 0x{log:016x}, so a publisher puts another type on \
                 the topic"
            )
        );
    }

    #[test]
    fn an_undecodable_frame_with_the_log_schema_names_its_payload_size() {
        let frame = UndecodableFrame {
            schema_hash: <Log as ShmMessage>::SCHEMA_HASH,
            size_bytes: 7,
        };
        assert_eq!(
            frame.to_string(),
            "a frame on /rosout carries the rcl_interfaces/Log schema but its 7 payload bytes \
             do not lay out as a record"
        );
    }

    // ---- skipping undecodable frames ----

    const OTHER_HASH: u64 = 0x0123_4567_89ab_cdef;

    fn foreign() -> UndecodableFrame {
        UndecodableFrame {
            schema_hash: OTHER_HASH,
            size_bytes: 12,
        }
    }

    fn malformed() -> UndecodableFrame {
        UndecodableFrame {
            schema_hash: <Log as ShmMessage>::SCHEMA_HASH,
            size_bytes: 7,
        }
    }

    /// Adversarial: one bad frame between good records is skipped, announced
    /// and counted, and the next record re-arms the announcement.
    #[test]
    fn a_bad_frame_between_records_is_announced_counted_and_never_ends_the_stream() {
        let mut skips = FrameSkips::default();
        assert_eq!(skips.on_record(), None);
        assert_eq!(
            skips.on_undecodable(&malformed()),
            SkipVerdict::Announce { total: 1 }
        );
        assert_eq!(
            skips.on_record(),
            None,
            "a lone skip needs no recovery line"
        );
        assert_eq!(
            skips.on_undecodable(&foreign()),
            SkipVerdict::Announce { total: 2 }
        );
        assert_eq!(skips.on_record(), None);
        assert_eq!(skips.skipped(), 2);
    }

    /// A flood of bad frames prints a bounded number of lines: the first, then
    /// one at 10, 100 and 1000 skipped. It never ends the stream once a record
    /// has decoded, whatever type the frames carry.
    #[test]
    fn a_flood_of_bad_frames_after_a_record_is_skipped_with_a_bounded_number_of_lines() {
        let mut skips = FrameSkips::default();
        skips.on_record();
        let mut announced = Vec::new();
        for i in 0..2_000u32 {
            let frame = if i % 2 == 0 { foreign() } else { malformed() };
            match skips.on_undecodable(&frame) {
                SkipVerdict::Announce { total } => announced.push(total),
                SkipVerdict::Quiet => {}
                SkipVerdict::Conflict(c) => panic!("ended after a record decoded: {c}"),
            }
        }
        assert_eq!(announced, vec![1, 10, 100, 1000]);
        assert_eq!(skips.skipped(), 2_000);
        assert_eq!(
            skips.on_record(),
            Some(2_000),
            "quiet skips earn a recovery line"
        );
        let mut all_foreign = FrameSkips::default();
        all_foreign.on_record();
        for _ in 0..(CONFLICT_FRAMES * 4) {
            assert!(!matches!(
                all_foreign.on_undecodable(&foreign()),
                SkipVerdict::Conflict(_)
            ));
        }
    }

    /// Before any record, `CONFLICT_FRAMES` foreign frames in a row are a
    /// topic of another type; one fewer is not, and a `Log` frame that fails
    /// to lay out breaks the run.
    #[test]
    fn a_run_of_foreign_frames_before_any_record_is_a_conflict() {
        let mut skips = FrameSkips::default();
        for i in 1..CONFLICT_FRAMES {
            let verdict = skips.on_undecodable(&foreign());
            let expected = if i == 1 {
                SkipVerdict::Announce { total: 1 }
            } else {
                SkipVerdict::Quiet
            };
            assert_eq!(verdict, expected, "frame {i}");
        }
        assert_eq!(
            skips.on_undecodable(&foreign()),
            SkipVerdict::Conflict(ForeignTopic {
                schema_hash: OTHER_HASH
            })
        );
        assert_eq!(skips.skipped(), u64::from(CONFLICT_FRAMES));

        let mut broken = FrameSkips::default();
        for _ in 1..CONFLICT_FRAMES {
            broken.on_undecodable(&foreign());
        }
        broken.on_undecodable(&malformed());
        assert!(!matches!(
            broken.on_undecodable(&foreign()),
            SkipVerdict::Conflict(_)
        ));
    }

    #[test]
    fn the_skip_lines_name_the_frame_and_the_count() {
        assert_eq!(
            skip_notice(&malformed(), 1),
            "[skipped] a frame on /rosout carries the rcl_interfaces/Log schema but its 7 \
             payload bytes do not lay out as a record; 1 frame skipped so far"
        );
        assert_eq!(
            recovery_notice(12),
            "[skipped] /rosout: records decode again; 12 frames skipped so far"
        );
        assert_eq!(skip_summary(1), "[skipped] /rosout: 1 frame skipped in all");
        assert_eq!(
            skip_summary(3),
            "[skipped] /rosout: 3 frames skipped in all"
        );
        let log = <Log as ShmMessage>::SCHEMA_HASH;
        assert_eq!(
            ForeignTopic {
                schema_hash: OTHER_HASH
            }
            .to_string(),
            format!(
                "/rosout carries another type: 8 frames in a row before any record are not \
                 rcl_interfaces/Log records (the last carries schema 0x0123456789abcdef, not 0x{log:016x}), so a \
                 publisher puts another type on the topic"
            )
        );
    }

    // ---- the loop stays event-driven ----

    const LOG_CMD_SRC: &str = include_str!("log_cmd.rs");

    #[test]
    fn the_echo_loop_blocks_on_the_listener_and_sleeps_only_on_the_pacing_floor() {
        let production: String = LOG_CMD_SRC
            .split("#[cfg(test)]")
            .next()
            .expect("production region")
            .to_string();
        assert!(production.contains("wait_for_message"));
        let sleeps: Vec<&str> = production
            .lines()
            .filter(|l| l.contains("thread::sleep("))
            .collect();
        assert_eq!(sleeps.len(), 1, "one sleep: the pacing floor");
        assert!(sleeps[0].contains("// ALLOW:"), "{}", sleeps[0]);
        assert!(!production.contains("try_receive"));
        // Every frame goes through the schema-hash check before the layout
        // decode: the loop never calls the layout decode directly.
        let loop_body = production
            .split("pub fn log_echo(")
            .nth(1)
            .expect("the echo loop");
        assert!(loop_body.contains("decode_log_frame("));
        assert!(!loop_body.contains("record_from_payload("));
    }
}
