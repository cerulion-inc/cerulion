// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion log echo`: the records on the host's one log topic, printed one
//! record per line, filtered by node and by minimum level.
//!
//! [`LOG_TOPIC`] is the Cerulion name for ROS 2 `/rosout`. A record on it is
//! an `rcl_interfaces/Log` message, read under Cerulion names: `name` is the
//! node id, `level` the `Log.msg` severity constant (`DEBUG=10`, `INFO=20`,
//! `WARN=30`, `ERROR=40`), `stamp` the record time, `function` the `tracing`
//! target (the module path the record came from), `file` and `line` the
//! source location and `msg` the text. Node `tracing` records are not
//! published to the topic yet: a node's logs go to its stderr.
//!
//! The verb reads the topic the way `topic echo --local` does (a live local
//! shared-memory topic, no remote rung) and renders each record with
//! [`render_line`]. The filters, the line format, the frame decode and the
//! node check are pure functions with hand-written oracles below; the loop
//! itself blocks on the topic's event listener and never polls.

use std::fmt;
use std::fmt::Write as _;
use std::io;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::graph::config::GraphConfig;
use cerulion_core::shm_runtime::read_offset_entry;
use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{gather_current_runs, RunGather, RunRecord, RunState};
use cerulion_core::wire::WireError;
use cerulion_core::ShmMessage;
use native_ros2_messages::rcl_interfaces::{Log, LogFixedSection, LogShm};

use crate::error::{CliError, CliResult};
use crate::topic_cmd::{self, TopicScope};

/// The host's one log topic.
///
/// An absolute name, like `/tf`: one topic for the whole host, shared by every
/// graph process running on it. It is the Cerulion spelling of ROS 2 `/rosout`.
pub const LOG_TOPIC: &str = "/log";

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
    /// `--node` named a node id no running graph has. `known` is the sorted
    /// id set the error lists.
    UnknownNode {
        given: String,
        known: Vec<String>,
        /// `Some` when the check could not see every run: the live-run
        /// registry did not hear every live writer, a run's graph could not
        /// be read, or a run is shutting down. The id may belong to a run
        /// the check could not see.
        note: Option<String>,
    },
    /// `--node` was given and no graph is running on this host to check it
    /// against: the registry settled on an empty answer.
    NoRunningGraph { given: String },
    /// `--node` was given and the running graphs could not be checked: the
    /// live-run registry did not settle and no run was heard, or every run
    /// heard is shutting down. `why` says which.
    CouldNotCheck { given: String, why: String },
    /// The live-run registry could not be read, or no live run's graph file
    /// could be.
    Unreadable(String),
}

impl fmt::Display for LogFilterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LogFilterError::UnknownLevel { given } => write!(
                f,
                "unknown log level '{given}'; accepted levels: {}",
                LogLevel::ALL.map(LogLevel::name).join(", ")
            ),
            LogFilterError::UnknownNode { given, known, note } => {
                write!(
                    f,
                    "node '{given}' is not in the running graph; nodes: {}",
                    known.join(", ")
                )?;
                if let Some(note) = note {
                    write!(f, " ({note})")?;
                }
                Ok(())
            }
            LogFilterError::NoRunningGraph { given } => write!(
                f,
                "--node {given} cannot be checked: no graph is running on this host; start one \
                 with `cerulion graph run <graph>`"
            ),
            LogFilterError::CouldNotCheck { given, why } => {
                write!(f, "--node {given} cannot be checked: {why}")
            }
            LogFilterError::Unreadable(why) => f.write_str(why),
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
    /// Node ids to show. Empty shows every node.
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
    pub fn spelling(&self) -> String {
        let mut flags: Vec<String> = self.nodes.iter().map(|n| format!("--node {n}")).collect();
        if let Some(level) = self.min_level {
            flags.push(format!("--level {}", level.name()));
        }
        flags.join(" ")
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
    /// `name`: the node id.
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
}

/// Append `text` to `line` so the record stays one line of fixed columns: a
/// backslash prints as `\\`; a newline, carriage return or tab as `\n`, `\r`
/// or `\t`; every other control character (C0, DEL, C1) and the line
/// separators U+2028 and U+2029 as `\u{..}` with the hexadecimal code point;
/// and in a [`Column::Fixed`] a space as `\u{20}`. Everything else is copied
/// as is.
fn push_escaped(line: &mut String, text: &str, column: Column) {
    for c in text.chars() {
        match c {
            '\\' => line.push_str("\\\\"),
            '\n' => line.push_str("\\n"),
            '\r' => line.push_str("\\r"),
            '\t' => line.push_str("\\t"),
            ' ' if column == Column::Fixed => line.push_str("\\u{20}"),
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
/// message (`cut -d' ' -f6-`). An empty node, target or file prints as `-`
/// (a missing file drops the line number with it). Every field is escaped the
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
    if !record.msg.is_empty() {
        line.push(' ');
        push_escaped(&mut line, &record.msg, Column::Message);
    }
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

/// Check every requested node id against `known` (the running graphs' node
/// ids). `note` is carried into the error when the check could not see every
/// run.
pub fn validate_nodes(
    requested: &[String],
    known: &[String],
    note: Option<&str>,
) -> Result<(), LogFilterError> {
    match requested.iter().find(|id| !known.contains(id)) {
        None => Ok(()),
        Some(given) => Err(LogFilterError::UnknownNode {
            given: given.clone(),
            known: known.to_vec(),
            note: note.map(str::to_string),
        }),
    }
}

/// The node ids a run's effective `graph.yaml` declares, sorted.
pub fn node_ids_from_graph_yaml(yaml: &str) -> Result<Vec<String>, LogFilterError> {
    let config: GraphConfig = serde_yaml::from_str(yaml).map_err(|e| {
        LogFilterError::Unreadable(format!("could not parse the running graph: {e}"))
    })?;
    let mut ids: Vec<String> = config.nodes.iter().map(|node| node.id.clone()).collect();
    ids.sort();
    ids.dedup();
    Ok(ids)
}

/// Reduce one live-run gather to the node ids `--node` is checked against,
/// plus a note on what the check could not see.
///
/// `read_ids` reads one run's node ids (its effective `graph.yaml`). It is a
/// parameter so the decision is a pure function over the gather, with a
/// hand-written vector below for every arm:
///
/// - a settled gather with no run: [`LogFilterError::NoRunningGraph`];
/// - no live run, and a gather that did not settle or runs that are only
///   shutting down: [`LogFilterError::CouldNotCheck`], never a confident
///   "no graph is running" (the registry did not hear every writer, or heard
///   only last words);
/// - a live run whose graph cannot be read is skipped and named in the note,
///   so a node of a readable run still passes; when no live run can be read,
///   the reasons come back as [`LogFilterError::Unreadable`];
/// - otherwise the ids are the union over the live runs, sorted and
///   deduplicated, and the note carries the unsettled registry, the skipped
///   runs and the runs that are shutting down, to ride into an
///   [`LogFilterError::UnknownNode`].
pub(crate) fn node_ids_from_gather(
    given: &str,
    gather: &RunGather,
    mut read_ids: impl FnMut(&RunRecord) -> Result<Vec<String>, LogFilterError>,
) -> Result<(Vec<String>, Option<String>), LogFilterError> {
    let mut notes: Vec<String> = Vec::new();
    if let GatherCompleteness::Incomplete {
        live_writers,
        writers_heard,
    } = gather.completeness
    {
        notes.push(format!(
            "the live-run registry did not settle ({writers_heard} of {live_writers} live \
             writers answered), so a node of an unheard run cannot be seen; try again"
        ));
    }
    for record in &gather.records {
        if record.state == RunState::Ending {
            notes.push(format!("run {} is shutting down", record.graph_name));
        }
    }
    let live: Vec<&RunRecord> = gather
        .records
        .iter()
        .filter(|record| record.state == RunState::Live)
        .collect();
    if live.is_empty() {
        return Err(if notes.is_empty() {
            LogFilterError::NoRunningGraph {
                given: given.to_string(),
            }
        } else {
            LogFilterError::CouldNotCheck {
                given: given.to_string(),
                why: notes.join("; "),
            }
        });
    }
    let mut ids = Vec::new();
    let mut readable = 0usize;
    let mut unreadable = Vec::new();
    for record in live {
        match read_ids(record) {
            Ok(run_ids) => {
                readable += 1;
                ids.extend(run_ids);
            }
            Err(e) => unreadable.push(e.to_string()),
        }
    }
    if readable == 0 {
        return Err(LogFilterError::Unreadable(unreadable.join("; ")));
    }
    notes.extend(unreadable);
    ids.sort();
    ids.dedup();
    let note = if notes.is_empty() {
        None
    } else {
        Some(notes.join("; "))
    };
    Ok((ids, note))
}

/// The node ids of every live run on this host (the union over runs, sorted),
/// and a note on what the check could not see.
fn live_graph_node_ids(given: &str) -> Result<(Vec<String>, Option<String>), LogFilterError> {
    let gather = gather_current_runs()
        .map_err(|e| LogFilterError::Unreadable(format!("could not gather live runs: {e}")))?;
    node_ids_from_gather(given, &gather, node_ids_of_run)
}

/// The node ids one live run's `graph.yaml` declares.
fn node_ids_of_run(record: &RunRecord) -> Result<Vec<String>, LogFilterError> {
    let path = std::path::Path::new(&record.run_dir).join(crate::run_dir::RUN_GRAPH_FILE);
    let yaml = std::fs::read_to_string(&path).map_err(|e| {
        LogFilterError::Unreadable(format!(
            "could not read the graph of run {} ({}): {e}",
            record.graph_name,
            path.display()
        ))
    })?;
    node_ids_from_graph_yaml(&yaml)
}

/// Resolve the `--node` and `--level` values into a [`LogFilter`], checking
/// node ids against the running graphs.
pub fn resolve_filter(nodes: &[String], level: Option<&str>) -> Result<LogFilter, LogFilterError> {
    let min_level = level.map(LogLevel::parse).transpose()?;
    if !nodes.is_empty() {
        let (known, note) = live_graph_node_ids(&nodes[0])?;
        validate_nodes(nodes, &known, note.as_deref())?;
    }
    Ok(LogFilter {
        nodes: nodes.to_vec(),
        min_level,
    })
}

/// Keep the first write error of one drain for the loop to act on once the
/// drain returns. Every later write of the same drain fails the same way, so
/// only the first is kept.
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

/// The stderr line `log echo` prints while nothing has been shown for a
/// while: how long, and what a record had to match.
fn silence_notice(filter: &LogFilter, silent_for: Duration) -> String {
    let secs = silent_for.as_secs_f64();
    if filter.nodes.is_empty() && filter.min_level.is_none() {
        format!("[waiting] {LOG_TOPIC}: 0 records in {secs:.0}s")
    } else {
        format!(
            "[waiting] {LOG_TOPIC}: 0 records matching {} in {secs:.0}s",
            filter.spelling()
        )
    }
}

/// Stream the log topic of the running local graph to `writer`, one rendered
/// record per line, until `running` clears or the reader of `writer` goes
/// away.
///
/// Refuses when no local graph publishes [`LOG_TOPIC`]: the topic is read
/// locally only, like `topic echo --local`, so there is no remote rung to
/// wait on.
// The silence notice is CLI output on stderr; stdout carries the record
// stream a user pipes. It must appear independently of the configured tracing
// filter.
#[allow(clippy::print_stderr)]
pub fn log_echo(
    filter: &LogFilter,
    running: Arc<AtomicBool>,
    writer: &mut dyn Write,
) -> CliResult<()> {
    let listed = topic_cmd::topic_list()?.iter().any(|t| t.name == LOG_TOPIC);
    if !listed {
        return Err(CliError::Validation(format!(
            "no running graph publishes {LOG_TOPIC} on this host; start one with \
             `cerulion graph run <graph>`"
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
    let mut undecodable_reported = false;
    // When the current run of "nothing shown" began (attach time, then reset
    // by every shown record) and how many notices it has had.
    let mut silence_since = std::time::Instant::now();
    let mut notices_emitted: u32 = 0;
    while running.load(Ordering::Relaxed) {
        let wait_started = std::time::Instant::now();
        let mut shown: usize = 0;
        let mut write_error: Option<io::Error> = None;
        let received = subscriber
            .subscriber
            .wait_for_message(topic_cmd::OBSERVER_HEARTBEAT, |msg| {
                let payload = msg.payload();
                match decode_log_frame(msg.header().schema_hash, payload) {
                    Some(record) => {
                        if filter.passes(&record) {
                            shown += 1;
                            note_write_error(
                                writeln!(writer, "{}", render_line(&record)),
                                &mut write_error,
                            );
                        }
                    }
                    None => {
                        // Loud once: a frame on the log topic that does not
                        // carry the `Log` schema hash, or whose payload does
                        // not lay out as a record, means another publisher
                        // put a different type on the name. Every later one
                        // is the same finding, so it is not repeated.
                        if !undecodable_reported {
                            undecodable_reported = true;
                            tracing::warn!(
                                topic = LOG_TOPIC,
                                size_bytes = payload.len(),
                                schema = %format!("0x{:016x}", msg.header().schema_hash),
                                "a frame on the log topic is not an rcl_interfaces/Log record; skipped"
                            );
                        }
                    }
                }
            });
        if let Some(e) = write_error {
            return end_on_write_error(e);
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
                        eprintln!("{}", silence_notice(filter, silent_for));
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
    fn empty_fields_print_as_a_dash_and_an_empty_message_leaves_no_trailing_space() {
        let mut r = record(20, "", "");
        r.target = String::new();
        r.file = String::new();
        r.line = 0;
        assert_eq!(render_line(&r), "1700000000.123456789 INFO - - -");
        // A line number with no file is dropped with the file.
        r.line = 42;
        assert_eq!(render_line(&r), "1700000000.123456789 INFO - - -");
        // A file with line 0 keeps the file.
        r.file = "src/lib.rs".into();
        r.line = 0;
        assert_eq!(
            render_line(&r),
            "1700000000.123456789 INFO - - src/lib.rs:0"
        );
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

    // ---- node ids of the running graphs ----

    #[test]
    fn requested_nodes_that_exist_pass_and_the_first_unknown_is_refused() {
        let known: Vec<String> = ["camera", "lidar", "planner"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        assert_eq!(
            validate_nodes(&["camera".to_string()], &known, None),
            Ok(())
        );
        assert_eq!(
            validate_nodes(&["planner".to_string(), "camera".to_string()], &known, None),
            Ok(())
        );
        assert_eq!(validate_nodes(&[], &known, None), Ok(()));
        let err = validate_nodes(
            &[
                "camera".to_string(),
                "camra".to_string(),
                "nope".to_string(),
            ],
            &known,
            None,
        )
        .expect_err("unknown");
        assert_eq!(
            err,
            LogFilterError::UnknownNode {
                given: "camra".into(),
                known: known.clone(),
                note: None,
            }
        );
        assert_eq!(
            err.to_string(),
            "node 'camra' is not in the running graph; nodes: camera, lidar, planner"
        );
    }

    #[test]
    fn the_check_note_is_named_in_the_refusal() {
        let known = vec!["camera".to_string()];
        let err = validate_nodes(
            &["lidar".to_string()],
            &known,
            Some("the live-run registry did not settle"),
        )
        .expect_err("unknown");
        assert_eq!(
            err.to_string(),
            "node 'lidar' is not in the running graph; nodes: camera (the live-run registry \
             did not settle)"
        );
    }

    #[test]
    fn every_could_not_check_arm_names_the_flag_and_the_reason() {
        let err = LogFilterError::NoRunningGraph {
            given: "camera".into(),
        };
        assert_eq!(
            err.to_string(),
            "--node camera cannot be checked: no graph is running on this host; start one \
             with `cerulion graph run <graph>`"
        );
        let err = LogFilterError::CouldNotCheck {
            given: "camera".into(),
            why: "run demo is shutting down".into(),
        };
        assert_eq!(
            err.to_string(),
            "--node camera cannot be checked: run demo is shutting down"
        );
        let err = LogFilterError::Unreadable("could not gather live runs: boom".into());
        assert_eq!(err.to_string(), "could not gather live runs: boom");
    }

    #[test]
    fn node_ids_come_from_the_effective_graph_yaml_sorted() {
        let yaml = r#"
prefix: robo
nodes:
  - id: planner
    type: planner
  - id: camera
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
"#;
        assert_eq!(
            node_ids_from_graph_yaml(yaml).unwrap(),
            ["camera".to_string(), "planner".to_string()]
        );
        let err = node_ids_from_graph_yaml("nodes: [").expect_err("bad yaml");
        assert!(
            matches!(err, LogFilterError::Unreadable(ref m) if m.starts_with("could not parse the running graph: ")),
            "{err:?}"
        );
    }

    // ---- the gather decision, one vector per arm ----

    fn run(id: u128, graph: &str, state: RunState) -> RunRecord {
        RunRecord {
            run_id: id,
            supervisor_pid: 4242,
            run_started_at_ns: 1_700_000_000_000_000_000,
            state,
            graph_name: graph.to_string(),
            run_dir: format!("/runs/{graph}"),
        }
    }

    fn settled(records: Vec<RunRecord>) -> RunGather {
        RunGather {
            records,
            completeness: GatherCompleteness::Settled,
        }
    }

    fn unsettled(records: Vec<RunRecord>, live_writers: usize, writers_heard: usize) -> RunGather {
        RunGather {
            records,
            completeness: GatherCompleteness::Incomplete {
                live_writers,
                writers_heard,
            },
        }
    }

    /// Reads the ids a test hands it per graph name; a graph not in the
    /// table is unreadable.
    fn reader<'t>(
        table: &'t [(&'t str, Vec<&'t str>)],
    ) -> impl FnMut(&RunRecord) -> Result<Vec<String>, LogFilterError> + 't {
        move |record| {
            table
                .iter()
                .find(|(graph, _)| *graph == record.graph_name)
                .map(|(_, ids)| ids.iter().map(|s| s.to_string()).collect())
                .ok_or_else(|| {
                    LogFilterError::Unreadable(format!(
                        "could not read the graph of run {} ({}/graph.yaml): gone",
                        record.graph_name, record.run_dir
                    ))
                })
        }
    }

    const UNSETTLED_NOTE: &str = "the live-run registry did not settle (1 of 2 live writers \
                                  answered), so a node of an unheard run cannot be seen; try again";

    #[test]
    fn a_settled_empty_gather_is_no_running_graph() {
        let err =
            node_ids_from_gather("camera", &settled(vec![]), reader(&[])).expect_err("no run");
        assert_eq!(
            err,
            LogFilterError::NoRunningGraph {
                given: "camera".into()
            }
        );
    }

    #[test]
    fn an_unsettled_empty_gather_is_could_not_check_never_a_confident_no_graph() {
        let err = node_ids_from_gather("camera", &unsettled(vec![], 2, 1), reader(&[]))
            .expect_err("unsettled");
        assert_eq!(
            err,
            LogFilterError::CouldNotCheck {
                given: "camera".into(),
                why: UNSETTLED_NOTE.into(),
            }
        );
        assert_eq!(
            err.to_string(),
            format!("--node camera cannot be checked: {UNSETTLED_NOTE}")
        );
    }

    #[test]
    fn a_gather_with_only_ending_runs_is_could_not_check_naming_them() {
        let gather = settled(vec![
            run(1, "demo", RunState::Ending),
            run(2, "nav", RunState::Ending),
        ]);
        let err = node_ids_from_gather(
            "camera",
            &gather,
            reader(&[("demo", vec!["camera"]), ("nav", vec!["planner"])]),
        )
        .expect_err("ending only");
        assert_eq!(
            err,
            LogFilterError::CouldNotCheck {
                given: "camera".into(),
                why: "run demo is shutting down; run nav is shutting down".into(),
            }
        );
    }

    #[test]
    fn live_runs_give_the_sorted_union_of_their_node_ids_and_no_note() {
        let gather = settled(vec![
            run(1, "demo", RunState::Live),
            run(2, "nav", RunState::Live),
        ]);
        let got = node_ids_from_gather(
            "camera",
            &gather,
            reader(&[
                ("demo", vec!["planner", "camera"]),
                ("nav", vec!["lidar", "camera"]),
            ]),
        )
        .expect("readable");
        assert_eq!(
            got,
            (
                vec!["camera".to_string(), "lidar".into(), "planner".into()],
                None
            )
        );
    }

    #[test]
    fn an_unsettled_gather_with_live_runs_passes_the_ids_and_carries_the_note() {
        let gather = unsettled(vec![run(1, "demo", RunState::Live)], 2, 1);
        let got = node_ids_from_gather("camera", &gather, reader(&[("demo", vec!["camera"])]))
            .expect("readable");
        assert_eq!(
            got,
            (vec!["camera".to_string()], Some(UNSETTLED_NOTE.into()))
        );
    }

    #[test]
    fn an_unreadable_run_is_skipped_and_named_while_a_readable_one_still_answers() {
        let gather = settled(vec![
            run(1, "demo", RunState::Live),
            run(2, "nav", RunState::Live),
        ]);
        let got = node_ids_from_gather("camera", &gather, reader(&[("demo", vec!["camera"])]))
            .expect("one readable run");
        assert_eq!(
            got,
            (
                vec!["camera".to_string()],
                Some("could not read the graph of run nav (/runs/nav/graph.yaml): gone".into())
            )
        );
    }

    #[test]
    fn when_no_live_run_can_be_read_every_reason_comes_back_as_unreadable() {
        let gather = settled(vec![
            run(1, "demo", RunState::Live),
            run(2, "nav", RunState::Live),
        ]);
        let err = node_ids_from_gather("camera", &gather, reader(&[])).expect_err("none readable");
        assert_eq!(
            err,
            LogFilterError::Unreadable(
                "could not read the graph of run demo (/runs/demo/graph.yaml): gone; could not \
                 read the graph of run nav (/runs/nav/graph.yaml): gone"
                    .into()
            )
        );
    }

    #[test]
    fn an_ending_run_beside_a_live_one_is_left_out_of_the_ids_and_named_in_the_note() {
        let gather = unsettled(
            vec![
                run(1, "demo", RunState::Live),
                run(2, "old", RunState::Ending),
            ],
            2,
            1,
        );
        let got = node_ids_from_gather(
            "camera",
            &gather,
            reader(&[("demo", vec!["camera"]), ("old", vec!["lidar"])]),
        )
        .expect("readable");
        assert_eq!(
            got,
            (
                vec!["camera".to_string()],
                Some(format!("{UNSETTLED_NOTE}; run old is shutting down"))
            )
        );
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
            silence_notice(&LogFilter::default(), Duration::from_secs(3)),
            "[waiting] /log: 0 records in 3s"
        );
        let filter = LogFilter {
            nodes: vec!["camera".into()],
            min_level: Some(LogLevel::Warn),
        };
        assert_eq!(
            silence_notice(&filter, Duration::from_millis(8_400)),
            "[waiting] /log: 0 records matching --node camera --level warn in 8s"
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
