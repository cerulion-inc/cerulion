// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph live` and `cerulion graph fires`: what a remote robot runs,
//! and which of its nodes fire.
//!
//! `graph live` asks the robot's `runs` query once and prints each run's nodes
//! with their triggers and topics, the run's endpoints, and whether the run
//! serves fire telemetry. `graph fires` reads the topology from one `runs`
//! answer, then asks the run's `fires` query every 250 ms with the cursor of
//! the previous answer and prints one row per fire.
//!
//! A run's `run.json` is read through [`RunView`], which ignores keys it does
//! not know, so this desk reads the manifest of a robot newer than itself.
//! Every string a robot sends is neutralized before it is printed: names,
//! ids and topics through `sanitize_display`, cut at [`DISPLAY_LIMIT`]
//! characters, and free text (a refusal, or why an answer cannot be read)
//! through `sanitize_refusal_reason`, which also bounds it.
//! `--json` output carries the whole values; each character a terminal acts
//! on is written as a `\u` escape, so a line reads the same value back.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::fire_tap::{FireCause, FireFlags, FireRecord};
use cerulion_core::graph::RUN_TOPOLOGY_VERSION;
use cerulion_core::transport::cerulion_q::{
    decode_fires_reply, parse_run_id, FiresOutcome, FiresReply, RunEntry, RunsCompleteness,
    RunsReply, FIRES_WIRE_VERSION, MAX_FIRES_PER_REPLY,
};
use cerulion_core::transport::discovery::RunsHarvest;
use serde::Deserialize;

use crate::error::{CliError, CliResult};
use crate::topic_cmd::{is_terminal_hostile, sanitize_display, sanitize_refusal_reason};

/// How often `graph fires` asks the run for new fires.
pub const FIRES_POLL_INTERVAL: Duration = Duration::from_millis(250);
/// How long one `fires` request waits for the robot's answer.
pub const FIRES_GET_WINDOW: Duration = Duration::from_millis(300);
/// How long `graph fires` reads when neither `--follow` nor `--for` is given.
pub const DEFAULT_FIRES_SECS: u64 = 5;
/// How long `graph fires` keeps asking a robot that does not answer.
pub const FIRES_SILENCE_LIMIT: Duration = Duration::from_secs(3);
/// How long one `runs` request waits for the robot's answer.
const RUNS_GET_WINDOW: Duration = Duration::from_secs(3);
/// How long a just-opened session listens for robots announcing themselves.
const ANNOUNCE_WINDOW: Duration = Duration::from_secs(1);
/// How long a just-opened session keeps asking before it reports that no
/// robot answered: a robot's query surface can reach a new session a moment
/// after the session opens.
const FIRST_CONTACT_BUDGET: Duration = Duration::from_secs(5);
const FIRST_CONTACT_RETRY: Duration = Duration::from_millis(250);

/// Where `graph live` and `graph fires` look for robots.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RemoteTarget {
    /// Zenoh locators to connect a session of this command's own to. Empty:
    /// ask through this machine's `cerulion-netd`.
    pub connect: Vec<String>,
    /// The robot to ask. `None`: every robot that announces itself.
    pub robot: Option<String>,
}

// ---------------------------------------------------------------------------
// The desk view of a run's `run.json`.
// ---------------------------------------------------------------------------

/// The parts of a run's `run.json` this desk reads. Unknown keys are ignored
/// and absent keys read as unknown, so a newer robot's manifest still reads.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct RunView {
    /// `embedded` for a run hosted inside an app; absent for `graph run`.
    #[serde(default)]
    pub host: Option<String>,
    /// Non-node participants: the app code around an embedded graph.
    #[serde(default)]
    pub endpoints: Vec<EndpointView>,
    /// The nodes, in fire-record index order. Absent when the host does not
    /// report it.
    #[serde(default)]
    pub topology: Option<TopologyView>,
    /// What telemetry the run serves.
    #[serde(default)]
    pub telemetry: TelemetryView,
}

/// One endpoint of a run.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct EndpointView {
    /// The endpoint's name.
    #[serde(default)]
    pub id: String,
    /// Topics the endpoint publishes into the graph.
    #[serde(default)]
    pub publishes: Vec<EndpointPortView>,
    /// Topics the endpoint subscribes to.
    #[serde(default)]
    pub subscribes: Vec<EndpointPortView>,
}

/// One topic of an endpoint.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct EndpointPortView {
    /// The resolved topic.
    #[serde(default)]
    pub topic: String,
}

/// A run's nodes. `nodes[i]` is fire record node `i`.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct TopologyView {
    /// The topology format version.
    #[serde(default)]
    pub version: u32,
    /// The nodes in graph order.
    #[serde(default)]
    pub nodes: Vec<NodeView>,
}

/// One node of a run's topology.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct NodeView {
    /// The node's id in the graph.
    #[serde(default)]
    pub id: String,
    /// The node's type.
    #[serde(rename = "type", default)]
    pub node_type: String,
    /// The node's resolved trigger.
    #[serde(default)]
    pub trigger: TriggerView,
    /// Inputs in graph order.
    #[serde(default)]
    pub inputs: Vec<PortView>,
    /// Outputs in graph order: output `k` is bit `k` of a fire's write mask.
    #[serde(default)]
    pub outputs: Vec<PortView>,
    /// Whether the node's fires say which outputs it wrote.
    #[serde(default)]
    pub reports_writes: bool,
}

/// One port of a node.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct PortView {
    /// The port's name.
    #[serde(default)]
    pub name: String,
    /// The port's resolved topic.
    #[serde(default)]
    pub topic: String,
}

/// A node's resolved trigger.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TriggerView {
    /// Fires on a timer.
    Period {
        /// The timer interval.
        interval_ns: u64,
    },
    /// Fires when an input receives data.
    Data {
        /// The topics that can trigger it.
        #[serde(default)]
        topics: Vec<String>,
        /// More than one input can trigger it.
        #[serde(default)]
        ambiguous: bool,
    },
    /// Fires when every listed input has a matching message.
    Sync {
        /// The synchronized topics.
        #[serde(default)]
        topics: Vec<String>,
        /// The match window; absent for an unbounded one.
        #[serde(default)]
        window_ns: Option<u64>,
    },
    /// Fires when the code around the graph triggers it.
    External,
    /// A trigger kind this desk does not know, or none at all.
    #[default]
    #[serde(other)]
    Unknown,
}

/// What telemetry a run serves.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct TelemetryView {
    /// Node fire telemetry; absent when the run serves none.
    #[serde(default)]
    pub fires: Option<FiresDeclView>,
}

/// A run's node fire telemetry.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
pub struct FiresDeclView {
    /// The `fires` wire version the run answers.
    #[serde(default)]
    pub version: u32,
    /// Fire records the run keeps.
    #[serde(default)]
    pub capacity: u64,
    /// How long one `fires` request keeps recording on, in milliseconds.
    #[serde(default)]
    pub lease_ms: u64,
}

/// Read a run's `run.json` into its desk view.
pub fn parse_run_json(run_json: &str) -> Result<RunView, String> {
    serde_json::from_str(run_json).map_err(|e| e.to_string())
}

// ---------------------------------------------------------------------------
// Rendering.
// ---------------------------------------------------------------------------

/// The most characters of one robot name, id or topic a terminal shows.
pub const DISPLAY_LIMIT: usize = 120;

/// A name, id or topic a robot sent, safe to print on a terminal: cut at
/// [`DISPLAY_LIMIT`] characters with a `…` marker.
fn clean(text: &str) -> String {
    let mut chars = text.chars();
    let mut shown = sanitize_display(&chars.by_ref().take(DISPLAY_LIMIT).collect::<String>());
    if chars.next().is_some() {
        shown.push('\u{2026}');
    }
    shown
}

/// One `--json` line: `value` as JSON, with each character a terminal acts
/// on written as a `\u` escape. Those characters appear only inside JSON
/// strings, so the line parses to the same value.
fn json_line(value: &serde_json::Value) -> String {
    let encoded = value.to_string();
    let mut line = String::with_capacity(encoded.len());
    for c in encoded.chars() {
        if is_terminal_hostile(c) {
            for unit in c.encode_utf16(&mut [0; 2]) {
                let _ = write!(line, "\\u{unit:04x}");
            }
        } else {
            line.push(c);
        }
    }
    line
}

fn width<I: IntoIterator<Item = usize>>(lengths: I) -> usize {
    lengths.into_iter().max().unwrap_or(0)
}

fn join_topics(topics: &[String]) -> String {
    topics
        .iter()
        .map(|topic| clean(topic))
        .collect::<Vec<_>>()
        .join(", ")
}

/// A duration in the largest whole unit that states it exactly.
fn interval_label(ns: u64) -> String {
    const S: u64 = 1_000_000_000;
    const MS: u64 = 1_000_000;
    const US: u64 = 1_000;
    match ns {
        0 => "0 ns".to_string(),
        ns if ns % S == 0 => format!("{} s", ns / S),
        ns if ns % MS == 0 => format!("{} ms", ns / MS),
        ns if ns % US == 0 => format!("{} µs", ns / US),
        ns => format!("{ns} ns"),
    }
}

fn data_label(topics: &[String], ambiguous: bool) -> String {
    match topics {
        [] => "on data".to_string(),
        [one] if !ambiguous => format!("on {}", clean(one)),
        many => format!("on one of: {}", join_topics(many)),
    }
}

fn trigger_label(trigger: &TriggerView) -> String {
    match trigger {
        TriggerView::Period { interval_ns } => format!("every {}", interval_label(*interval_ns)),
        TriggerView::Data { topics, ambiguous } => data_label(topics, *ambiguous),
        TriggerView::Sync { topics, window_ns } => match window_ns {
            Some(window) => format!(
                "on all of: {} within {}",
                join_topics(topics),
                interval_label(*window)
            ),
            None => format!("on all of: {}", join_topics(topics)),
        },
        TriggerView::External => "external".to_string(),
        TriggerView::Unknown => "unknown trigger".to_string(),
    }
}

/// The topics a trigger names: a data trigger's candidates or a sync
/// trigger's inputs.
fn trigger_topics(trigger: &TriggerView) -> &[String] {
    match trigger {
        TriggerView::Data { topics, .. } | TriggerView::Sync { topics, .. } => topics,
        TriggerView::Period { .. } | TriggerView::External | TriggerView::Unknown => &[],
    }
}

fn telemetry_label(telemetry: &TelemetryView) -> String {
    match &telemetry.fires {
        None => "not served by this run".to_string(),
        Some(decl) if decl.version == FIRES_WIRE_VERSION => format!(
            "available (v{}, ring {}, lease {})",
            decl.version,
            decl.capacity,
            interval_label(decl.lease_ms.saturating_mul(1_000_000))
        ),
        Some(decl) => format!(
            "version {} (this cerulion reads version {FIRES_WIRE_VERSION})",
            decl.version
        ),
    }
}

fn incomplete_note(live_writers: u32, writers_heard: u32) -> String {
    if live_writers == 0 && writers_heard == 0 {
        "the answer is incomplete".to_string()
    } else {
        format!("the answer is incomplete: {writers_heard} of {live_writers} runs answered in time")
    }
}

fn describe_run(robot: &str, entry: &RunEntry) -> String {
    format!(
        "{} (graph {} on robot {})",
        clean(&entry.run_id),
        clean(&entry.graph_name),
        clean(robot)
    )
}

/// Render one robot's `runs` answer as text, one block per run.
pub fn render_live(reply: &RunsReply) -> String {
    let robot = clean(&reply.robot);
    if let Some(reason) = &reply.error {
        return format!(
            "robot {robot}: refused the runs query ({})\n",
            sanitize_refusal_reason(reason)
        );
    }
    let mut out = reply
        .runs
        .iter()
        .map(|entry| render_run(&robot, entry))
        .collect::<Vec<_>>()
        .join("\n");
    match (reply.completeness, reply.runs.is_empty()) {
        (RunsCompleteness::Settled, true) => out.push_str(&format!("robot {robot}: no runs\n")),
        (RunsCompleteness::Settled, false) => {}
        (
            RunsCompleteness::Incomplete {
                live_writers,
                writers_heard,
            },
            empty,
        ) => out.push_str(&format!(
            "robot {robot}: {} ({})\n",
            if empty {
                "no runs heard"
            } else {
                "more runs may be live"
            },
            incomplete_note(live_writers, writers_heard)
        )),
    }
    for run in &reply.undescribable {
        out.push_str(&format!(
            "robot {robot} · run {}: cannot be described ({})\n",
            clean(&run.run_id),
            sanitize_refusal_reason(&run.reason)
        ));
    }
    out
}

fn render_run(robot: &str, entry: &RunEntry) -> String {
    let view = parse_run_json(&entry.run_json);
    let mut out = format!(
        "robot {robot} · run {} · graph {} · {}",
        clean(&entry.run_id),
        clean(&entry.graph_name),
        entry.state
    );
    if let Some(host) = view.as_ref().ok().and_then(|v| v.host.as_deref()) {
        out.push_str(&format!(" · host {}", clean(host)));
    }
    out.push_str(&format!(
        " · started {}\n",
        utc_time(entry.run_started_at_ns)
    ));
    let view = match view {
        Ok(view) => view,
        Err(reason) => {
            out.push_str(&format!(
                "  run.json does not read ({})\n",
                sanitize_refusal_reason(&reason)
            ));
            return out;
        }
    };
    out.push_str(&render_nodes(view.topology.as_ref(), &view.endpoints));
    if let Some(version) = unknown_topology_version(&view) {
        out.push_str(&format!(
            "  the node rows follow topology version {version}, and this cerulion reads version \
             {RUN_TOPOLOGY_VERSION}, so they can be wrong; update cerulion on this machine\n"
        ));
    }
    out.push_str(&format!(
        "  fire telemetry: {}\n",
        telemetry_label(&view.telemetry)
    ));
    out
}

/// The run's topology version when this desk does not read it. A new version
/// can change what a node index or a write bit means.
fn unknown_topology_version(view: &RunView) -> Option<u32> {
    view.topology
        .as_ref()
        .map(|t| t.version)
        .filter(|version| *version != RUN_TOPOLOGY_VERSION)
}

fn render_nodes(topology: Option<&TopologyView>, endpoints: &[EndpointView]) -> String {
    const ENDPOINT: &str = "endpoint";
    let nodes = topology.map_or(&[][..], |t| t.nodes.as_slice());
    let ids: Vec<String> = nodes.iter().map(|n| clean(&n.id)).collect();
    let types: Vec<String> = nodes.iter().map(|n| clean(&n.node_type)).collect();
    let labels: Vec<String> = nodes.iter().map(|n| trigger_label(&n.trigger)).collect();
    let endpoint_ids: Vec<String> = endpoints.iter().map(|e| clean(&e.id)).collect();
    let iw = width(ids.iter().chain(&endpoint_ids).map(|id| id.chars().count()));
    let tw = width(
        types
            .iter()
            .map(|t| t.chars().count())
            .chain(endpoints.first().map(|_| ENDPOINT.len())),
    );
    let gw = width(labels.iter().map(|l| l.chars().count()));

    let mut out = String::new();
    if topology.is_none() {
        out.push_str("  this run's host does not report its nodes\n");
    }
    for (i, node) in nodes.iter().enumerate() {
        let outputs: Vec<String> = node.outputs.iter().map(|p| p.topic.clone()).collect();
        let outputs = if outputs.is_empty() {
            "(no outputs)".to_string()
        } else {
            join_topics(&outputs)
        };
        let mut line = format!(
            "  {:<iw$}  {:<tw$}  {:<gw$}  → {outputs}",
            ids[i], types[i], labels[i]
        );
        if !node.reports_writes {
            line.push_str("  (writes not reported)");
        }
        let triggering = trigger_topics(&node.trigger);
        let others: Vec<String> = node
            .inputs
            .iter()
            .filter(|p| !triggering.contains(&p.topic))
            .map(|p| p.topic.clone())
            .collect();
        if !others.is_empty() {
            line.push_str(&format!("  (also reads {})", join_topics(&others)));
        }
        out.push_str(&line);
        out.push('\n');
    }
    for (endpoint, id) in endpoints.iter().zip(&endpoint_ids) {
        let topics = |ports: &[EndpointPortView]| {
            if ports.is_empty() {
                "(none)".to_string()
            } else {
                join_topics(&ports.iter().map(|p| p.topic.clone()).collect::<Vec<_>>())
            }
        };
        out.push_str(&format!(
            "  {id:<iw$}  {ENDPOINT:<tw$}  publishes  {}\n",
            topics(&endpoint.publishes)
        ));
        out.push_str(&format!(
            "  {:<iw$}  {:<tw$}  subscribes {}\n",
            "",
            "",
            topics(&endpoint.subscribes)
        ));
    }
    out
}

/// Render the robots of a harvest that gave no reply, one line each.
pub fn render_unanswered(harvest: &RunsHarvest) -> String {
    let mut out = String::new();
    for answer in &harvest.unusable {
        out.push_str(&format!(
            "robot {}: answered the runs query with something this desk cannot read ({}); \
             update cerulion on one side\n",
            clean(&answer.robot),
            sanitize_refusal_reason(&answer.reason)
        ));
    }
    for robot in &harvest.silent {
        out.push_str(&format!(
            "robot {}: did not answer the runs query (a robot older than the runs query, or one \
             with no query surface)\n",
            clean(robot)
        ));
    }
    out
}

fn has_flag(rec: &FireRecord, flag: u8) -> bool {
    rec.flags.0 & flag != 0
}

fn age_label(robot_now_ns: u64, at_ns: u64) -> String {
    format!("{:.3} s", robot_now_ns.saturating_sub(at_ns) as f64 / 1e9)
}

fn duration_label(rec: &FireRecord) -> String {
    if has_flag(rec, FireFlags::DURATION_SATURATED) {
        return format!(">{:.2} s", f64::from(u32::MAX) / 1e9);
    }
    match u64::from(rec.dur_ns) {
        ns if ns < 1_000 => format!("{ns} ns"),
        ns if ns < 1_000_000 => format!("{} µs", ns / 1_000),
        ns if ns < 1_000_000_000 => format!("{:.1} ms", ns as f64 / 1e6),
        ns => format!("{:.2} s", ns as f64 / 1e9),
    }
}

fn cause_label(node: Option<&NodeView>, rec: &FireRecord) -> String {
    let trigger = node.map(|n| &n.trigger);
    match (rec.cause, trigger) {
        (FireCause::Period, Some(t @ TriggerView::Period { .. }))
        | (FireCause::Sync, Some(t @ TriggerView::Sync { .. })) => trigger_label(t),
        (FireCause::Data, Some(TriggerView::Data { topics, ambiguous })) => data_label(
            topics,
            *ambiguous || has_flag(rec, FireFlags::TRIGGER_AMBIGUOUS),
        ),
        (FireCause::Period, _) => "period".to_string(),
        (FireCause::Data, _) => "on data".to_string(),
        (FireCause::Sync, _) => "sync".to_string(),
        (FireCause::External, _) => "external".to_string(),
        (FireCause::Unknown, _) => "unknown cause".to_string(),
    }
}

/// The outputs a fire wrote, as `(bit, topic)` with `None` for a bit past the
/// node's outputs; `None` when the fire cannot say what it wrote.
fn written_outputs<'a>(
    node: Option<&'a NodeView>,
    rec: &FireRecord,
) -> Option<Vec<(usize, Option<&'a str>)>> {
    let node = node?;
    if has_flag(rec, FireFlags::WROTE_UNKNOWN) || !node.reports_writes {
        return None;
    }
    Some(
        (0..64usize)
            .filter(|bit| rec.wrote >> bit & 1 == 1)
            .map(|bit| (bit, node.outputs.get(bit).map(|p| p.topic.as_str())))
            .collect(),
    )
}

fn wrote_label(node: Option<&NodeView>, rec: &FireRecord) -> String {
    let Some(outputs) = written_outputs(node, rec) else {
        return "not reported".to_string();
    };
    let mut names: Vec<String> = outputs
        .iter()
        .map(|(bit, topic)| topic.map_or_else(|| format!("output {bit}"), clean))
        .collect();
    if has_flag(rec, FireFlags::WROTE_TRUNCATED) {
        names.push(if names.is_empty() {
            "outputs past 64".to_string()
        } else {
            "and outputs past 64".to_string()
        });
    }
    if names.is_empty() {
        "nothing".to_string()
    } else {
        names.join(", ")
    }
}

/// Render one fire as a text row. `robot_now_ns` is the robot's clock when it
/// served the fire, so the row shows the fire's age without a clock sync.
pub fn render_fire_row(topology: &TopologyView, robot_now_ns: u64, rec: &FireRecord) -> String {
    FireRows::new(topology).render(robot_now_ns, rec)
}

/// The fire rows of one topology: the node ids and the column widths,
/// computed once for a whole reading.
struct FireRows<'a> {
    topology: &'a TopologyView,
    ids: Vec<String>,
    id_width: usize,
    cause_width: usize,
}

impl<'a> FireRows<'a> {
    fn new(topology: &'a TopologyView) -> Self {
        let ids: Vec<String> = topology.nodes.iter().map(|n| clean(&n.id)).collect();
        Self {
            topology,
            id_width: width(ids.iter().map(|id| id.chars().count())),
            cause_width: width(
                topology
                    .nodes
                    .iter()
                    .map(|n| trigger_label(&n.trigger).chars().count()),
            ),
            ids,
        }
    }

    fn render(&self, robot_now_ns: u64, rec: &FireRecord) -> String {
        let index = usize::from(rec.node);
        let node = self.topology.nodes.get(index);
        let id = self
            .ids
            .get(index)
            .cloned()
            .unwrap_or_else(|| format!("node {}", rec.node));
        let (iw, cw) = (self.id_width, self.cause_width);
        let mut row = format!(
            "{:>9} ago  {id:<iw$}  {:<cw$}  {:>9}  wrote {}",
            age_label(robot_now_ns, rec.at_ns),
            cause_label(node, rec),
            duration_label(rec),
            wrote_label(node, rec),
        );
        if has_flag(rec, FireFlags::PANICKED) {
            row.push_str("  panicked");
        }
        if has_flag(rec, FireFlags::TICK_ERROR) {
            row.push_str("  tick error");
        }
        row
    }
}

fn trigger_json(trigger: &TriggerView) -> serde_json::Value {
    match trigger {
        TriggerView::Period { interval_ns } => {
            serde_json::json!({"kind": "period", "interval_ns": interval_ns})
        }
        TriggerView::Data { topics, ambiguous } => {
            serde_json::json!({"kind": "data", "topics": topics, "ambiguous": ambiguous})
        }
        TriggerView::Sync { topics, window_ns } => {
            serde_json::json!({"kind": "sync", "topics": topics, "window_ns": window_ns})
        }
        TriggerView::External => serde_json::json!({"kind": "external"}),
        TriggerView::Unknown => serde_json::json!({"kind": "unknown"}),
    }
}

fn ports_json(ports: &[PortView]) -> serde_json::Value {
    ports
        .iter()
        .map(|p| serde_json::json!({"name": p.name, "topic": p.topic}))
        .collect()
}

/// A run object. A `run.json` that does not read gives `run_json_readable:
/// false` and nulls, so it is not taken for a run whose host reports no nodes.
fn run_object(robot: &str, entry: &RunEntry) -> serde_json::Value {
    let view = parse_run_json(&entry.run_json);
    let readable = view.is_ok();
    let view = view.unwrap_or_default();
    let nodes = view.topology.as_ref().map(|t| {
        t.nodes
            .iter()
            .enumerate()
            .map(|(index, n)| {
                serde_json::json!({
                    "index": index,
                    "id": n.id,
                    "type": n.node_type,
                    "trigger": trigger_json(&n.trigger),
                    "inputs": ports_json(&n.inputs),
                    "outputs": ports_json(&n.outputs),
                    "reports_writes": n.reports_writes,
                })
            })
            .collect::<Vec<_>>()
    });
    let endpoints: Option<Vec<serde_json::Value>> = readable.then(|| {
        view.endpoints
            .iter()
            .map(|e| {
                let topics = |ports: &[EndpointPortView]| {
                    ports.iter().map(|p| p.topic.clone()).collect::<Vec<_>>()
                };
                serde_json::json!({
                    "id": e.id,
                    "publishes": topics(&e.publishes),
                    "subscribes": topics(&e.subscribes),
                })
            })
            .collect()
    });
    let fires = view.telemetry.fires.as_ref().map(|f| {
        serde_json::json!({"version": f.version, "capacity": f.capacity, "lease_ms": f.lease_ms})
    });
    serde_json::json!({
        "kind": "run",
        "robot": robot,
        "run_id": entry.run_id,
        "graph_name": entry.graph_name,
        "state": entry.state,
        "run_started_at_ns": entry.run_started_at_ns,
        "run_json_readable": readable,
        "host": view.host,
        "topology_version": view.topology.as_ref().map(|t| t.version),
        "nodes": nodes,
        "endpoints": endpoints,
        "fires": fires,
    })
}

fn robot_object(robot: &str, status: &str) -> serde_json::Value {
    serde_json::json!({"kind": "robot", "robot": robot, "status": status})
}

/// One `graph live --json` object per run, one per robot with no readable
/// run, and an `incomplete` one after the runs of a robot whose answer may
/// not name every run. Only ids, names, topics and numbers.
pub fn live_json(harvest: &RunsHarvest) -> Vec<serde_json::Value> {
    let mut runs = Vec::new();
    let mut robots = Vec::new();
    for reply in &harvest.replies {
        if reply.error.is_some() {
            robots.push(robot_object(&reply.robot, "refused"));
            continue;
        }
        runs.extend(reply.runs.iter().map(|e| run_object(&reply.robot, e)));
        if names_every_run(reply) {
            if reply.runs.is_empty() {
                robots.push(robot_object(&reply.robot, "no_runs"));
            }
        } else {
            let mut robot = robot_object(&reply.robot, "incomplete");
            robot["undescribable"] = reply
                .undescribable
                .iter()
                .map(|run| serde_json::Value::from(run.run_id.as_str()))
                .collect();
            robots.push(robot);
        }
    }
    robots.extend(
        harvest
            .unusable
            .iter()
            .map(|u| robot_object(&u.robot, "unusable")),
    );
    robots.extend(harvest.silent.iter().map(|r| robot_object(r, "silent")));
    runs.extend(robots);
    runs
}

/// One `graph fires --json` object for a fire. Only ids, names, topics and
/// numbers: no payload and no free text.
pub fn fire_json(run: &SelectedRun, robot_now_ns: u64, rec: &FireRecord) -> serde_json::Value {
    let node = run
        .view
        .topology
        .as_ref()
        .and_then(|t| t.nodes.get(usize::from(rec.node)));
    let trigger_topics: &[String] = match (rec.cause, node.map(|n| &n.trigger)) {
        (FireCause::Data, Some(t @ TriggerView::Data { .. }))
        | (FireCause::Sync, Some(t @ TriggerView::Sync { .. })) => trigger_topics(t),
        _ => &[],
    };
    let wrote_topics: Option<Vec<String>> = written_outputs(node, rec).map(|outputs| {
        outputs
            .into_iter()
            .filter_map(|(_, t)| t.map(str::to_string))
            .collect()
    });
    serde_json::json!({
        "kind": "fire",
        "robot": run.robot,
        "run_id": run.run_id,
        "seq": rec.seq,
        "node": rec.node,
        "node_id": node.map(|n| n.id.clone()),
        "node_type": node.map(|n| n.node_type.clone()),
        "cause": rec.cause as u8,
        "trigger_topics": trigger_topics,
        "at_ns": rec.at_ns,
        "age_ns": robot_now_ns.saturating_sub(rec.at_ns),
        "dur_ns": rec.dur_ns,
        "flags": rec.flags.0,
        "wrote": rec.wrote,
        "wrote_topics": wrote_topics,
    })
}

// ---------------------------------------------------------------------------
// Choosing a run and following its fires.
// ---------------------------------------------------------------------------

/// The run `graph fires` reads.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SelectedRun {
    /// The robot that hosts the run, as the desk asked it.
    pub robot: String,
    /// The run id in its canonical text.
    pub run_id: String,
    /// The run's graph name.
    pub graph_name: String,
    /// The run's `run.json`.
    pub view: RunView,
}

fn list_runs(runs: &[(&str, &RunEntry)]) -> String {
    runs.iter()
        .map(|(robot, entry)| describe_run(robot, entry))
        .collect::<Vec<_>>()
        .join(", ")
}

fn serves_fires(entry: &RunEntry) -> bool {
    parse_run_json(&entry.run_json)
        .map(|view| view.telemetry.fires.is_some())
        .unwrap_or(false)
}

fn readable_fires_run(robot: &str, entry: &RunEntry) -> CliResult<SelectedRun> {
    let refuse = |text: String| Err(CliError::Validation(text));
    let view = match parse_run_json(&entry.run_json) {
        Ok(view) => view,
        Err(reason) => {
            return refuse(format!(
                "run {}: its run.json does not read ({})",
                describe_run(robot, entry),
                sanitize_refusal_reason(&reason)
            ))
        }
    };
    match (&view.telemetry.fires, unknown_topology_version(&view)) {
        (None, _) => refuse(format!(
            "run {} serves no fire telemetry",
            describe_run(robot, entry)
        )),
        (Some(decl), _) if decl.version != FIRES_WIRE_VERSION => refuse(format!(
            "run {} serves fire telemetry version {}, and this cerulion reads version \
             {FIRES_WIRE_VERSION}; update cerulion on this machine",
            describe_run(robot, entry),
            decl.version
        )),
        (Some(_), Some(version)) => refuse(format!(
            "run {} describes its nodes in topology version {version}, and this cerulion \
             reads version {RUN_TOPOLOGY_VERSION}; update cerulion on this machine",
            describe_run(robot, entry)
        )),
        (Some(_), None) => Ok(SelectedRun {
            robot: robot.to_string(),
            run_id: entry.run_id.clone(),
            graph_name: entry.graph_name.clone(),
            view,
        }),
    }
}

/// Why one robot's answer gave no run: it refused, its answer is
/// incomplete, or it has no live run. Each run it cannot describe is named.
fn no_run_reason(reply: &RunsReply) -> String {
    let robot = clean(&reply.robot);
    let mut reason = match (&reply.error, reply.completeness) {
        (Some(refusal), _) => format!(
            "robot {robot} refused the runs query ({})",
            sanitize_refusal_reason(refusal)
        ),
        (
            None,
            RunsCompleteness::Incomplete {
                live_writers,
                writers_heard,
            },
        ) => format!(
            "robot {robot} named no runs ({})",
            incomplete_note(live_writers, writers_heard)
        ),
        (None, RunsCompleteness::Settled) => format!("no run is live on robot {robot}"),
    };
    reason.push_str(&undescribable_note(&robot, reply));
    reason
}

fn undescribable_note(robot: &str, reply: &RunsReply) -> String {
    reply
        .undescribable
        .iter()
        .map(|run| {
            format!(
                "; robot {robot} cannot describe run {} ({})",
                clean(&run.run_id),
                sanitize_refusal_reason(&run.reason)
            )
        })
        .collect()
}

/// A served, settled answer with every run described: the robot hosts no
/// run it does not name.
fn names_every_run(reply: &RunsReply) -> bool {
    reply.error.is_none() && reply.completeness.is_settled() && reply.undescribable.is_empty()
}

/// Why the harvest may not name every run of the robots asked: one entry
/// per robot that refused, gave an incomplete answer, gave an answer this
/// desk cannot read, or did not answer.
fn unnamed_runs(harvest: &RunsHarvest) -> Vec<String> {
    let mut gaps: Vec<String> = harvest
        .replies
        .iter()
        .filter(|reply| !names_every_run(reply))
        .map(|reply| {
            if reply.error.is_some() || reply.runs.is_empty() {
                return no_run_reason(reply);
            }
            let robot = clean(&reply.robot);
            let (live_writers, writers_heard) = match reply.completeness {
                RunsCompleteness::Incomplete {
                    live_writers,
                    writers_heard,
                } => (live_writers, writers_heard),
                RunsCompleteness::Settled => (0, 0),
            };
            format!(
                "robot {robot} may have more runs ({}){}",
                incomplete_note(live_writers, writers_heard),
                undescribable_note(&robot, reply)
            )
        })
        .collect();
    gaps.extend(unanswered(harvest));
    gaps
}

/// The robots asked that gave no reply: one entry per robot whose answer
/// this desk cannot read, then one per robot that did not answer.
fn unanswered(harvest: &RunsHarvest) -> Vec<String> {
    let unusable = harvest.unusable.iter().map(|answer| {
        format!(
            "robot {} answered the runs query with something this desk cannot read ({})",
            clean(&answer.robot),
            sanitize_refusal_reason(&answer.reason)
        )
    });
    let silent = harvest
        .silent
        .iter()
        .map(|robot| format!("robot {} did not answer the runs query", clean(robot)));
    unusable.chain(silent).collect()
}

fn gaps_note(gaps: &[String]) -> String {
    gaps.iter().map(|gap| format!("; {gap}")).collect()
}

/// Choose the run `graph fires` reads. `run` is a run id or a graph name;
/// without it, the one run that serves fire telemetry. Only a run id
/// chooses while a robot asked may host a run the harvest does not name.
pub fn select_run(harvest: &RunsHarvest, run: Option<&str>) -> CliResult<SelectedRun> {
    let refuse = |text: String| Err(CliError::Validation(text));
    let gaps = unnamed_runs(harvest);
    let heard = if gaps.is_empty() { "" } else { " heard" };
    let all: Vec<(&str, &RunEntry)> = harvest
        .replies
        .iter()
        .filter(|reply| reply.error.is_none())
        .flat_map(|reply| reply.runs.iter().map(move |e| (reply.robot.as_str(), e)))
        .collect();
    if all.is_empty() {
        if harvest.replies.is_empty() {
            let text = if harvest.unusable.is_empty() && harvest.silent.is_empty() {
                "no robot answered the runs query, so there is no run to read"
            } else {
                "no robot gave a readable runs answer, so there is no run to read"
            };
            return refuse(text.to_string());
        }
        let mut reasons: Vec<String> = harvest.replies.iter().map(no_run_reason).collect();
        reasons.extend(unanswered(harvest));
        return refuse(format!("no run to read: {}", reasons.join("; ")));
    }
    let Some(text) = run else {
        let capable: Vec<(&str, &RunEntry)> = all
            .iter()
            .copied()
            .filter(|(_, entry)| serves_fires(entry))
            .collect();
        return match capable.as_slice() {
            [] => refuse(format!(
                "no run{heard} serves fire telemetry; the runs{heard} are: {}{}",
                list_runs(&all),
                gaps_note(&gaps)
            )),
            [(robot, entry)] if !gaps.is_empty() => refuse(format!(
                "run {} is the only run heard that serves fire telemetry, and the answer may \
                 not name every run; pass --run with its id to read it{}",
                describe_run(robot, entry),
                gaps_note(&gaps)
            )),
            [(robot, entry)] => readable_fires_run(robot, entry),
            many => refuse(format!(
                "{} runs serve fire telemetry; pass --run with one of: {}",
                many.len(),
                list_runs(many)
            )),
        };
    };
    let id = parse_run_id(text);
    let matches: Vec<(&str, &RunEntry)> = all
        .iter()
        .copied()
        .filter(|(_, entry)| {
            entry.graph_name == text || (id.is_some() && parse_run_id(&entry.run_id) == id)
        })
        .collect();
    match matches.as_slice() {
        [] => refuse(format!(
            "no run{heard} matches --run {}; the runs{heard} are: {}{}",
            clean(text),
            list_runs(&all),
            gaps_note(&gaps)
        )),
        [(robot, entry)]
            if !gaps.is_empty() && (id.is_none() || parse_run_id(&entry.run_id) != id) =>
        {
            refuse(format!(
                "--run {} names run {} by its graph name, and the answer may not name every \
                 run; pass --run with its id to read it{}",
                clean(text),
                describe_run(robot, entry),
                gaps_note(&gaps)
            ))
        }
        [(robot, entry)] => readable_fires_run(robot, entry),
        many => refuse(format!(
            "--run {} matches {} runs; pass a run id: {}",
            clean(text),
            many.len(),
            list_runs(many)
        )),
    }
}

/// The cursor and totals of one `graph fires` reading.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct FireStream {
    cursor: Option<u64>,
    /// Fires read so far.
    pub fires: u64,
    /// Fires the run lost before this desk read them.
    pub lost: u64,
}

impl FireStream {
    /// The cursor the next request passes: the `next` of the last served
    /// page, unchanged, or `None` before the first.
    pub fn cursor(&self) -> Option<u64> {
        self.cursor
    }

    /// Take one served page. Returns the fires not read before, and whether
    /// the page was full, so the caller asks again at once.
    ///
    /// `next` is inclusive: the last sequence the page accounts for, read or
    /// lost. A page that ends before the cursor is stale and adds nothing.
    pub fn absorb(
        &mut self,
        next: u64,
        lost: u64,
        fires: &[FireRecord],
    ) -> (Vec<FireRecord>, bool) {
        let after = self.cursor;
        if after.is_some_and(|cursor| next < cursor) {
            return (Vec::new(), false);
        }
        let fresh: Vec<FireRecord> = fires
            .iter()
            .filter(|rec| after.is_none_or(|cursor| rec.seq > cursor))
            .copied()
            .collect();
        self.fires = self.fires.saturating_add(fresh.len() as u64);
        self.lost = self.lost.saturating_add(lost);
        self.cursor = Some(next);
        (fresh, fires.len() >= MAX_FIRES_PER_REPLY)
    }
}

/// One answer to a `fires` request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FiresPoll {
    /// The robot answered.
    Reply(FiresReply),
    /// Nothing came back.
    Silent,
    /// Something came back that this desk cannot use.
    Unusable {
        /// Why, in operator-readable text.
        reason: String,
    },
}

/// Classify the payloads one `fires` request gathered: the first that
/// decodes wins; else the last decode error; else silence.
pub fn classify_fires(payloads: &[Vec<u8>]) -> FiresPoll {
    let mut last_error = None;
    for payload in payloads {
        match decode_fires_reply(payload) {
            Ok(reply) => return FiresPoll::Reply(reply),
            Err(e) => last_error = Some(e),
        }
    }
    match last_error {
        Some(e) => FiresPoll::Unusable {
            reason: e.to_string(),
        },
        None => FiresPoll::Silent,
    }
}

/// Where `runs` answers come from.
pub trait RunsSource {
    /// Ask `robot`, or every robot that announces itself, for its runs. A
    /// source that waits for a first answer stops when `running` clears.
    fn runs(&mut self, robot: Option<&str>, running: &AtomicBool) -> CliResult<RunsHarvest>;
}

/// Where `fires` answers come from.
pub trait FiresSource {
    /// Ask `robot` for the fires of `run_id` after `after`.
    fn fires(&mut self, robot: &str, run_id: &str, after: Option<u64>) -> FiresPoll;
}

/// Time as `graph fires` sees it, so a test can drive the loop.
pub trait Pacer {
    /// Time since the reading began.
    fn elapsed(&self) -> Duration;
    /// Wait `d`.
    fn sleep(&mut self, d: Duration);
}

/// The wall clock.
pub struct WallPacer {
    started: Instant,
}

impl WallPacer {
    /// A pacer that starts now.
    pub fn start() -> Self {
        Self {
            started: Instant::now(),
        }
    }
}

impl Pacer for WallPacer {
    fn elapsed(&self) -> Duration {
        self.started.elapsed()
    }
    fn sleep(&mut self, d: Duration) {
        std::thread::sleep(d);
    }
}

/// Why a fires reading ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FiresEnd {
    /// The `--for` window elapsed.
    Window,
    /// Ctrl+C.
    Interrupted,
    /// The robot no longer hosts the run.
    RunEnded,
    /// The reading failed.
    Failed(String),
}

impl FiresEnd {
    fn name(&self) -> &'static str {
        match self {
            FiresEnd::Window => "window",
            FiresEnd::Interrupted => "interrupted",
            FiresEnd::RunEnded => "run_ended",
            FiresEnd::Failed(_) => "failed",
        }
    }
}

fn plural(n: u64, one: &str, many: &str) -> String {
    if n == 1 {
        format!("{n} {one}")
    } else {
        format!("{n} {many}")
    }
}

fn write_fires_summary(
    out: &mut dyn Write,
    run: &SelectedRun,
    stream: &FireStream,
    end: &FiresEnd,
    elapsed: Duration,
    json: bool,
) -> std::io::Result<()> {
    if json {
        let summary = serde_json::json!({
            "kind": "summary",
            "robot": run.robot,
            "run_id": run.run_id,
            "fires": stream.fires,
            "lost": stream.lost,
            "elapsed_ms": u64::try_from(elapsed.as_millis()).unwrap_or(u64::MAX),
            "end": end.name(),
        });
        return writeln!(out, "{}", json_line(&summary));
    }
    let tail = match end {
        FiresEnd::RunEnded => "the run ended".to_string(),
        FiresEnd::Window | FiresEnd::Interrupted | FiresEnd::Failed(_) => {
            match &run.view.telemetry.fires {
                Some(decl) => format!(
                    "the robot stops recording {} after the last ask",
                    interval_label(decl.lease_ms.saturating_mul(1_000_000))
                ),
                None => "the robot stops recording when its lease ends".to_string(),
            }
        }
    };
    writeln!(
        out,
        "{} in {:.1} s · {} lost · {tail}",
        plural(stream.fires, "fire", "fires"),
        elapsed.as_secs_f64(),
        stream.lost
    )
}

/// Read the fires of `run` from `source` until `window` elapses (`None`:
/// until `running` clears), writing one row per fire and then the totals.
///
/// The window ends the reading only after an answered ask. A robot that is
/// not answering when the window ends gets the rest of its silence limit, so
/// a short window never reports a silent robot as a clean reading.
pub fn follow_fires(
    source: &mut dyn FiresSource,
    run: &SelectedRun,
    window: Option<Duration>,
    json: bool,
    running: &AtomicBool,
    pacer: &mut dyn Pacer,
    out: &mut dyn Write,
) -> CliResult<(FireStream, FiresEnd)> {
    let topology = run.view.topology.clone().unwrap_or_default();
    let rows = FireRows::new(&topology);
    let mut stream = FireStream::default();
    let mut silent_since: Option<Duration> = None;
    let mut answered = false;
    if !json {
        writeln!(
            out,
            "fires of run {} (graph {}) on robot {}; the robot records while this desk asks",
            clean(&run.run_id),
            clean(&run.graph_name),
            clean(&run.robot)
        )?;
    }
    let end = loop {
        if !running.load(Ordering::Relaxed) {
            break FiresEnd::Interrupted;
        }
        if silent_since
            .is_some_and(|since| pacer.elapsed().saturating_sub(since) >= FIRES_SILENCE_LIMIT)
        {
            break FiresEnd::Failed(format!(
                "the robot {} the fires query for {} s",
                if answered {
                    "stopped answering"
                } else {
                    "did not answer"
                },
                FIRES_SILENCE_LIMIT.as_secs()
            ));
        }
        if silent_since.is_none() && window.is_some_and(|w| pacer.elapsed() >= w) {
            break FiresEnd::Window;
        }
        let mut read_on = false;
        let asked_at = pacer.elapsed();
        match source.fires(&run.robot, &run.run_id, stream.cursor()) {
            FiresPoll::Reply(reply) if reply.run_id != run.run_id => {
                break FiresEnd::Failed(format!(
                    "the robot answered about run {}, not {}",
                    clean(&reply.run_id),
                    clean(&run.run_id)
                ));
            }
            FiresPoll::Reply(reply) => {
                silent_since = None;
                answered = true;
                match reply.outcome {
                    FiresOutcome::Served {
                        now_ns,
                        next,
                        lost,
                        fires,
                        ..
                    } => {
                        let (fresh, full) = stream.absorb(next, lost, &fires);
                        for rec in &fresh {
                            if json {
                                writeln!(out, "{}", json_line(&fire_json(run, now_ns, rec)))?;
                            } else {
                                writeln!(out, "{}", rows.render(now_ns, rec))?;
                            }
                        }
                        out.flush()?;
                        read_on = full;
                    }
                    FiresOutcome::UnknownRun => break FiresEnd::RunEnded,
                    FiresOutcome::NotServed => {
                        break FiresEnd::Failed(format!(
                            "the robot serves no fire telemetry for run {}",
                            clean(&run.run_id)
                        ));
                    }
                    FiresOutcome::Refused { reason } => {
                        break FiresEnd::Failed(format!(
                            "the robot refused the fires query ({})",
                            sanitize_refusal_reason(&reason)
                        ));
                    }
                }
            }
            FiresPoll::Silent => {
                silent_since.get_or_insert(pacer.elapsed());
            }
            FiresPoll::Unusable { reason } => {
                break FiresEnd::Failed(format!(
                    "the robot's fires answer cannot be read ({})",
                    sanitize_refusal_reason(&reason)
                ));
            }
        }
        if !read_on {
            // The ask's own gather window counts toward the interval, so asks
            // start every FIRES_POLL_INTERVAL rather than that long apart.
            let asking = pacer.elapsed().saturating_sub(asked_at);
            pacer.sleep(FIRES_POLL_INTERVAL.saturating_sub(asking));
        }
    };
    write_fires_summary(out, run, &stream, &end, pacer.elapsed(), json)?;
    out.flush()?;
    match end {
        FiresEnd::Failed(reason) => Err(CliError::Validation(reason)),
        end => Ok((stream, end)),
    }
}

// ---------------------------------------------------------------------------
// Where the answers come from.
// ---------------------------------------------------------------------------

/// The `--connect` locators a session dials: those in `reachable`, in the
/// order given. zenoh tries the locators one by one under a 1 s bound for
/// all of them, so one that does not accept a connection would make the
/// open fail and lose every other robot. A malformed locator, or a list with
/// no reachable locator, is refused with the remedy.
fn connect_set(connect: &[String], reachable: &HashSet<String>) -> CliResult<Vec<String>> {
    use crate::discovery_ladder::{locator_is_syntactically_valid, EXPLICIT_PROBE_TIMEOUT};
    let malformed: Vec<String> = connect
        .iter()
        .filter(|locator| !locator_is_syntactically_valid(locator))
        .map(|locator| clean(locator))
        .collect();
    if !malformed.is_empty() {
        return Err(CliError::Validation(format!(
            "malformed --connect locator {}; a locator has the form tcp/<host>:<port>, for \
             example tcp/127.0.0.1:7683",
            malformed.join(", ")
        )));
    }
    let (kept, skipped): (Vec<String>, Vec<String>) = connect
        .iter()
        .cloned()
        .partition(|locator| reachable.contains(locator));
    let wait = interval_label(u64::try_from(EXPLICIT_PROBE_TIMEOUT.as_nanos()).unwrap_or(u64::MAX));
    if kept.is_empty() {
        let named: Vec<String> = connect.iter().map(|locator| clean(locator)).collect();
        return Err(CliError::Validation(format!(
            "nothing accepted a connection at {} within {wait}; check that the robot listens \
             there (a robot reached through adb needs its port forwarded, for example adb \
             forward tcp:17683 tcp:7683)",
            named.join(", ")
        )));
    }
    for locator in &skipped {
        tracing::warn!(
            locator = %clean(locator),
            wait = %wait,
            "--connect locator did not accept a connection in time; asking the other \
             locators only"
        );
    }
    Ok(kept)
}

/// Ask until a robot answers, an answer is final, or the first-contact
/// budget is spent. A cleared `running` ends the wait with an error; an
/// error from `ask` is returned at once, never retried.
fn first_contact(
    mut ask: impl FnMut() -> CliResult<RunsHarvest>,
    running: &AtomicBool,
    pacer: &mut dyn Pacer,
) -> CliResult<RunsHarvest> {
    let interrupted =
        || CliError::Validation("interrupted while asking for runs; nothing was read".to_string());
    loop {
        if !running.load(Ordering::Relaxed) {
            return Err(interrupted());
        }
        let harvest = ask()?;
        if !harvest.replies.is_empty()
            || harvest.is_terminal()
            || pacer.elapsed() >= FIRST_CONTACT_BUDGET
        {
            return Ok(harvest);
        }
        if !running.load(Ordering::Relaxed) {
            return Err(interrupted());
        }
        pacer.sleep(FIRST_CONTACT_RETRY);
    }
}

/// A zenoh session of this command's own, connected to the given locators
/// only: no scouting, so it reaches exactly the robots named.
pub struct ZenohPlane {
    manager: cerulion_core::transport::network::NetworkManager,
}

impl ZenohPlane {
    /// Open the session to the locators that accept a connection. The
    /// connect phase is bounded.
    pub fn open(connect: &[String]) -> CliResult<Self> {
        use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
        let reachable = crate::discovery_ladder::probe_reachable_locators(
            connect,
            crate::discovery_ladder::EXPLICIT_PROBE_TIMEOUT,
        );
        let connect = connect_set(connect, &reachable)?;
        let manager = NetworkManager::new(NetworkConfig {
            connect_endpoints: connect.clone(),
            listen_endpoints: Vec::new(),
            multicast_scouting: false,
            gossip_scouting: false,
            bounded_connect: true,
            ..NetworkConfig::default()
        });
        manager.session().map_err(|e| {
            CliError::Validation(format!(
                "could not open a session to {}: {e}",
                connect.join(", ")
            ))
        })?;
        Ok(Self { manager })
    }
}

impl RunsSource for ZenohPlane {
    fn runs(&mut self, robot: Option<&str>, running: &AtomicBool) -> CliResult<RunsHarvest> {
        use cerulion_core::transport::discovery::{query_announce_entries, query_robot_runs_all};
        let session = self
            .manager
            .session()
            .map_err(|e| CliError::Validation(format!("the session closed: {e}")))?;
        let ask = || -> CliResult<RunsHarvest> {
            let robots: Vec<String> = match robot {
                Some(robot) => vec![robot.to_string()],
                None => query_announce_entries(session, ANNOUNCE_WINDOW)
                    .map_err(|e| {
                        CliError::Validation(format!(
                            "could not ask which robots announce themselves: {e}"
                        ))
                    })?
                    .into_iter()
                    .map(|(robot, _)| robot)
                    .collect::<std::collections::BTreeSet<_>>()
                    .into_iter()
                    .collect(),
            };
            Ok(if robots.is_empty() {
                RunsHarvest::default()
            } else {
                query_robot_runs_all(session, &robots, RUNS_GET_WINDOW)
            })
        };
        first_contact(ask, running, &mut WallPacer::start())
    }
}

impl FiresSource for ZenohPlane {
    fn fires(&mut self, robot: &str, run_id: &str, after: Option<u64>) -> FiresPoll {
        let session = match self.manager.session() {
            Ok(session) => session,
            Err(e) => {
                return FiresPoll::Unusable {
                    reason: format!("the session closed: {e}"),
                }
            }
        };
        classify_fires(&cerulion_core::transport::discovery::query_robot_fires(
            session,
            robot,
            run_id,
            after,
            FIRES_GET_WINDOW,
        ))
    }
}

/// `runs` answers through this machine's `cerulion-netd`.
pub struct NetdRuns;

impl RunsSource for NetdRuns {
    #[cfg(unix)]
    fn runs(&mut self, robot: Option<&str>, _running: &AtomicBool) -> CliResult<RunsHarvest> {
        let mut client = cerulion_netd::NetdClient::connect_or_spawn().map_err(|e| {
            CliError::Validation(format!(
                "could not reach cerulion-netd to ask for runs: {e}; pass --connect LOCATOR \
                 to ask the robot directly"
            ))
        })?;
        netd_harvest(&mut client, robot)
    }

    #[cfg(not(unix))]
    fn runs(&mut self, _robot: Option<&str>, _running: &AtomicBool) -> CliResult<RunsHarvest> {
        Err(CliError::Validation(
            "asking for runs without --connect needs cerulion-netd, which runs on Unix only; \
             pass --connect LOCATOR to ask the robot directly"
                .to_string(),
        ))
    }
}

/// One `runs` query through a connected `cerulion-netd`.
#[cfg(unix)]
fn netd_harvest(
    client: &mut cerulion_netd::NetdClient,
    robot: Option<&str>,
) -> CliResult<RunsHarvest> {
    let answer = client.query_runs(robot).map_err(|e| {
        CliError::Validation(format!(
            "cerulion-netd could not answer the runs query: {e}"
        ))
    })?;
    Ok(RunsHarvest {
        replies: answer.replies,
        unusable: answer.unusable,
        silent: answer.silent,
    })
}

// ---------------------------------------------------------------------------
// The commands.
// ---------------------------------------------------------------------------

/// The refusal `verb` (`graph live` or `graph fires`) gives while a
/// query-side kill switch ([`crate::graph_cmd::query_network_kill`]) keeps this
/// machine local-only. `None` when neither switch is engaged. Neither
/// `--robot` nor `--connect` overrides an engaged switch: both verbs always
/// ask robots. The CLI gives this refusal before its login gate, exit 1.
pub fn network_refusal(verb: &str, kill: &crate::graph_cmd::QueryNetworkKill) -> Option<String> {
    let mut labels: Vec<String> = kill.network.refusal_label().into_iter().collect();
    if kill.netd_off {
        labels.push(format!("{}=off", crate::graph_cmd::NETD_NETWORK_ENV));
    }
    if labels.is_empty() {
        return None;
    }
    Some(format!(
        "cerulion {verb}: asking robots is disabled by {}, so this command stays local-only and \
         contacts no robot; {} to ask robots through cerulion-netd or --connect",
        labels.join(" and "),
        kill.unset_remedy()
    ))
}

/// Refuse `verb` before it opens `cerulion-netd` or a session of its own
/// while a kill switch is engaged: a runs or fires query is network traffic,
/// and a fires query arms recording on the robot. The CLI refuses first,
/// before its login gate ([`network_refusal`]); this check covers a call that
/// does not come through it.
fn refuse_under_kill_switch(verb: &str) -> CliResult<()> {
    match network_refusal(verb, &crate::graph_cmd::query_network_kill()) {
        Some(refusal) => Err(CliError::Validation(refusal)),
        None => Ok(()),
    }
}

fn runs_source(target: &RemoteTarget) -> CliResult<Box<dyn RunsSource>> {
    refuse_under_kill_switch("graph live")?;
    if target.connect.is_empty() {
        Ok(Box::new(NetdRuns))
    } else {
        Ok(Box::new(ZenohPlane::open(&target.connect)?))
    }
}

/// `cerulion graph live`.
pub fn graph_live(target: &RemoteTarget, json: bool) -> CliResult<()> {
    live_on_terminal(runs_source(target)?.as_mut(), target, json)
}

/// [`show_live`] on this process's stdout and stderr. Neither is locked
/// across the ask: a source logs to stderr from the threads it joins.
fn live_on_terminal(
    source: &mut dyn RunsSource,
    target: &RemoteTarget,
    json: bool,
) -> CliResult<()> {
    show_live(
        source,
        target,
        json,
        &AtomicBool::new(true),
        &mut std::io::stdout(),
        &mut std::io::stderr(),
    )
}

/// Ask `source` once and write what `graph live` shows: the runs to `out`,
/// and in `--json` mode the robots with no reply to `err`.
pub fn show_live(
    source: &mut dyn RunsSource,
    target: &RemoteTarget,
    json: bool,
    running: &AtomicBool,
    out: &mut dyn Write,
    err: &mut dyn Write,
) -> CliResult<()> {
    let harvest = source.runs(target.robot.as_deref(), running)?;
    let unanswered = render_unanswered(&harvest);
    if json {
        for object in live_json(&harvest) {
            writeln!(out, "{}", json_line(&object))?;
        }
        write!(err, "{unanswered}")?;
    } else {
        let blocks: Vec<String> = harvest.replies.iter().map(render_live).collect();
        write!(out, "{}{unanswered}", blocks.join("\n"))?;
    }
    out.flush()?;
    err.flush()?;
    live_outcome(&harvest, target)
}

/// Whether `graph live` found anything to show.
fn live_outcome(harvest: &RunsHarvest, target: &RemoteTarget) -> CliResult<()> {
    if harvest.replies.iter().any(|reply| reply.error.is_none()) {
        return Ok(());
    }
    if harvest.replies.is_empty() && harvest.unusable.is_empty() && harvest.silent.is_empty() {
        let via = if target.connect.is_empty() {
            "through cerulion-netd".to_string()
        } else {
            format!("on {}", target.connect.join(", "))
        };
        let remedy = if target.robot.is_some() {
            ""
        } else {
            "; pass --robot NAME if the robot does not announce itself"
        };
        return Err(CliError::Validation(format!(
            "no robot answered the runs query {via}{remedy}"
        )));
    }
    Err(CliError::Validation(
        "no robot gave a readable runs answer".to_string(),
    ))
}

fn fires_window(follow: bool, for_secs: Option<u64>) -> CliResult<Option<Duration>> {
    match (follow, for_secs) {
        (true, Some(_)) => Err(CliError::Validation(
            "--follow and --for cannot be used together".to_string(),
        )),
        (_, Some(0)) => Err(CliError::Validation(
            "--for takes a number of seconds above 0".to_string(),
        )),
        (true, None) => Ok(None),
        (false, secs) => Ok(Some(Duration::from_secs(
            secs.unwrap_or(DEFAULT_FIRES_SECS),
        ))),
    }
}

/// `cerulion graph fires`.
pub fn graph_fires(
    target: &RemoteTarget,
    run: Option<&str>,
    follow: bool,
    for_secs: Option<u64>,
    json: bool,
    running: Arc<AtomicBool>,
) -> CliResult<()> {
    let window = fires_window(follow, for_secs)?;
    refuse_under_kill_switch("graph fires")?;
    if target.connect.is_empty() {
        return Err(fires_needs_netd_protocol_10());
    }
    let mut plane = ZenohPlane::open(&target.connect)?;
    let harvest = plane.runs(target.robot.as_deref(), &running)?;
    write!(std::io::stderr(), "{}", render_unanswered(&harvest))?;
    let selected = select_run(&harvest, run)?;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    follow_fires(
        &mut plane,
        &selected,
        window,
        json,
        &running,
        &mut WallPacer::start(),
        &mut out,
    )
    .map(|_| ())
}

/// `ns` since the Unix epoch as a UTC calendar time, to the second.
fn utc_time(ns: u64) -> String {
    let secs = ns / 1_000_000_000;
    let (days, rem) = ((secs / 86_400) as i64, secs % 86_400);
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02}:{:02} UTC",
        rem / 3_600,
        rem % 3_600 / 60,
        rem % 60
    )
}

/// The error `graph fires` gives without `--connect`.
pub fn fires_needs_netd_protocol_10() -> CliError {
    CliError::Validation(
        "graph fires without --connect reads through cerulion-netd and needs cerulion-netd \
         protocol 10 (its fires query), which this build does not include; pass --connect \
         LOCATOR (for example tcp/127.0.0.1:7683) to ask the robot directly"
            .to_string(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{
        format_run_id, RunEntryState, UndescribableRun, UnusableRunsAnswer,
    };

    const TEMI_RUN_ID: u128 = 0x5d1c;
    const TEMI_STARTED_NS: u64 = 1_762_264_931_000_000_000;

    /// The temi run's `run.json`, as the embedded host renders it.
    const TEMI_RUN_JSON: &str = r#"{
  "endpoints": [
    {
      "id": "app",
      "publishes": [
        {"schema": "std_msgs/String", "topic": "/temi/in"},
        {"schema": "std_msgs/String", "topic": "/temi/chunks"}
      ],
      "subscribes": [
        {"schema": "std_msgs/String", "topic": "/temi/logic/edge"},
        {"schema": "std_msgs/String", "topic": "/temi/hearing/heard"}
      ]
    }
  ],
  "graph_file": "temi",
  "graph_name": "temi",
  "host": "embedded",
  "lan_access": "open",
  "run_id": "0x00000000000000000000000000005d1c",
  "run_started_at_ns": 1762264931000000000,
  "supervisor_pid": 4711,
  "telemetry": {"fires": {"capacity": 4096, "lease_ms": 5000, "version": 1}},
  "topology": {
    "nodes": [
      {"id": "beat", "type": "temi_beat",
       "trigger": {"kind": "period", "interval_ns": 1000000000},
       "inputs": [], "outputs": [{"name": "count", "topic": "/temi/beat/count"}],
       "reports_writes": true},
      {"id": "logic", "type": "temi_logic",
       "trigger": {"kind": "data", "topics": ["/temi/in"], "ambiguous": false},
       "inputs": [{"name": "in", "topic": "/temi/in"}],
       "outputs": [{"name": "edge", "topic": "/temi/logic/edge"},
                   {"name": "pulse", "topic": "/temi/logic/pulse"}],
       "reports_writes": true},
      {"id": "hearing", "type": "temi_hearing",
       "trigger": {"kind": "data", "topics": ["/temi/chunks"], "ambiguous": false},
       "inputs": [{"name": "chunks", "topic": "/temi/chunks"}],
       "outputs": [{"name": "heard", "topic": "/temi/hearing/heard"}],
       "reports_writes": true},
      {"id": "peer", "type": "temi_peer",
       "trigger": {"kind": "data", "topics": ["/temi/logic/edge"], "ambiguous": false},
       "inputs": [{"name": "edge", "topic": "/temi/logic/edge"}],
       "outputs": [], "reports_writes": true}
    ],
    "version": 1
  },
  "version": 1
}"#;

    fn temi_entry() -> RunEntry {
        RunEntry {
            run_id: format_run_id(TEMI_RUN_ID),
            graph_name: "temi".to_string(),
            run_started_at_ns: TEMI_STARTED_NS,
            state: RunEntryState::Live,
            graph_yaml: "name: temi\nprefix: temi\n".to_string(),
            run_json: TEMI_RUN_JSON.to_string(),
        }
    }

    fn reply(robot: &str, runs: Vec<RunEntry>) -> RunsReply {
        RunsReply {
            version: 1,
            robot: robot.to_string(),
            runs,
            completeness: RunsCompleteness::Settled,
            undescribable: Vec::new(),
            error: None,
        }
    }

    fn harvest(replies: Vec<RunsReply>) -> RunsHarvest {
        RunsHarvest {
            replies,
            ..RunsHarvest::default()
        }
    }

    fn temi_view() -> RunView {
        parse_run_json(TEMI_RUN_JSON).expect("the temi run.json reads")
    }

    fn temi_run() -> SelectedRun {
        SelectedRun {
            robot: "temi".to_string(),
            run_id: format_run_id(TEMI_RUN_ID),
            graph_name: "temi".to_string(),
            view: temi_view(),
        }
    }

    fn fire(
        seq: u64,
        at_ns: u64,
        dur_ns: u32,
        node: u16,
        cause: FireCause,
        flags: u8,
        wrote: u64,
    ) -> FireRecord {
        FireRecord {
            seq,
            at_ns,
            dur_ns,
            node,
            cause,
            flags: FireFlags(flags),
            wrote,
        }
    }

    // ---- run.json desk view ------------------------------------------------

    /// The desk reads every key the embedded host writes.
    #[test]
    fn the_temi_run_json_reads_into_the_desk_view() {
        let view = temi_view();
        assert_eq!(view.host.as_deref(), Some("embedded"));
        assert_eq!(view.endpoints.len(), 1);
        assert_eq!(
            view.endpoints[0]
                .publishes
                .iter()
                .map(|p| p.topic.as_str())
                .collect::<Vec<_>>(),
            ["/temi/in", "/temi/chunks"]
        );
        let topology = view.topology.expect("a topology");
        assert_eq!(
            topology
                .nodes
                .iter()
                .map(|n| n.id.as_str())
                .collect::<Vec<_>>(),
            ["beat", "logic", "hearing", "peer"]
        );
        assert_eq!(
            topology.nodes[0].trigger,
            TriggerView::Period {
                interval_ns: 1_000_000_000
            }
        );
        assert_eq!(
            view.telemetry.fires,
            Some(FiresDeclView {
                version: 1,
                capacity: 4096,
                lease_ms: 5000
            })
        );
    }

    /// A newer robot adds keys at every level of its `run.json`. The desk
    /// view reads past each one, where the robot's own topology type refuses
    /// the same document: that refusal is why the desk keeps its own view.
    #[test]
    fn a_newer_robots_run_json_with_extra_keys_still_reads() {
        let mut doc: serde_json::Value = serde_json::from_str(TEMI_RUN_JSON).unwrap();
        doc["added_at_top"] = serde_json::json!({"anything": [1, 2]});
        doc["endpoints"][0]["qos"] = serde_json::json!("reliable");
        doc["endpoints"][0]["publishes"][0]["depth"] = serde_json::json!(4);
        doc["topology"]["added"] = serde_json::json!(true);
        doc["topology"]["nodes"][0]["added"] = serde_json::json!("x");
        doc["topology"]["nodes"][0]["trigger"]["max_catchup"] = serde_json::json!(3);
        doc["topology"]["nodes"][1]["outputs"][0]["schema"] = serde_json::json!("std_msgs/String");
        doc["telemetry"]["fires"]["added"] = serde_json::json!(1);
        doc["telemetry"]["added"] = serde_json::json!({});
        let text = doc.to_string();

        let view = parse_run_json(&text).expect("a newer run.json must read");
        assert_eq!(
            view,
            temi_view(),
            "the extra keys change nothing the desk reads"
        );

        let refused =
            serde_json::from_value::<cerulion_core::graph::RunTopology>(doc["topology"].clone());
        assert!(
            refused.is_err(),
            "the robot's own topology type refuses unknown keys, so the desk must not decode with it"
        );
    }

    /// A trigger kind this desk does not know reads as unknown, not as an
    /// error that would hide the whole run.
    #[test]
    fn an_unknown_trigger_kind_reads_as_unknown() {
        let mut doc: serde_json::Value = serde_json::from_str(TEMI_RUN_JSON).unwrap();
        doc["topology"]["nodes"][0]["trigger"] =
            serde_json::json!({"kind": "on_battery_low", "threshold": 20});
        let view = parse_run_json(&doc.to_string()).expect("reads");
        assert_eq!(
            view.topology.unwrap().nodes[0].trigger,
            TriggerView::Unknown
        );
    }

    /// A desktop `graph run` manifest has no topology and no telemetry.
    #[test]
    fn a_graph_run_manifest_reads_with_no_topology_and_no_fires() {
        let manifest = r#"{"version":1,"run_id":"0x00000000000000000000000000000001",
            "graph_name":"perception","graph_file":"perception","supervisor_pid":7,
            "run_started_at_ns":5,"run_dir":"/r","network":{"mode":"peer"}}"#;
        let view = parse_run_json(manifest).expect("reads");
        assert_eq!(view, RunView::default());
    }

    // ---- render_live -------------------------------------------------------

    /// THE `graph live` oracle for the temi run.
    #[test]
    fn render_live_lists_the_temi_run() {
        let text = render_live(&reply("temi", vec![temi_entry()]));
        let expected = "\
robot temi · run 0x00000000000000000000000000005d1c · graph temi · live · host embedded · started 2025-11-04 14:02:11 UTC
  beat     temi_beat     every 1 s            → /temi/beat/count
  logic    temi_logic    on /temi/in          → /temi/logic/edge, /temi/logic/pulse
  hearing  temi_hearing  on /temi/chunks      → /temi/hearing/heard
  peer     temi_peer     on /temi/logic/edge  → (no outputs)
  app      endpoint      publishes  /temi/in, /temi/chunks
                         subscribes /temi/logic/edge, /temi/hearing/heard
  fire telemetry: available (v1, ring 4096, lease 5 s)
";
        assert_eq!(text, expected);
    }

    /// Non-trigger inputs, writes the node cannot report, sync and external
    /// triggers, an ending run and a run with no telemetry.
    #[test]
    fn render_live_names_every_trigger_kind_and_the_inputs_beside_the_trigger() {
        let run_json = serde_json::json!({
            "host": "embedded",
            "topology": {"version": 1, "nodes": [
                {"id": "src", "type": "clock", "trigger": {"kind": "period", "interval_ns": 250_000_000u64},
                 "inputs": [], "outputs": [{"name": "t", "topic": "/r/src/t"}], "reports_writes": false},
                {"id": "mix", "type": "mixer",
                 "trigger": {"kind": "data", "topics": ["/a", "/b"], "ambiguous": true},
                 "inputs": [{"name": "a", "topic": "/a"}, {"name": "b", "topic": "/b"}],
                 "outputs": [{"name": "o", "topic": "/r/mix/o"}], "reports_writes": true},
                {"id": "join", "type": "joiner",
                 "trigger": {"kind": "sync", "topics": ["/a", "/r/mix/o"], "window_ns": 10_000_000u64},
                 "inputs": [{"name": "ctx", "topic": "/ctx"}, {"name": "a", "topic": "/a"},
                            {"name": "o", "topic": "/r/mix/o"}],
                 "outputs": [], "reports_writes": true},
                {"id": "ext", "type": "button", "trigger": {"kind": "external"},
                 "inputs": [], "outputs": [], "reports_writes": true}
            ]}
        })
        .to_string();
        let entry = RunEntry {
            run_id: format_run_id(7),
            graph_name: "mix".to_string(),
            run_started_at_ns: 0,
            state: RunEntryState::Ending,
            graph_yaml: "nodes: []\n".to_string(),
            run_json,
        };
        let text = render_live(&reply("bench", vec![entry]));
        let expected = "\
robot bench · run 0x00000000000000000000000000000007 · graph mix · ending · host embedded · started 1970-01-01 00:00:00 UTC
  src   clock   every 250 ms                          → /r/src/t  (writes not reported)
  mix   mixer   on one of: /a, /b                     → /r/mix/o
  join  joiner  on all of: /a, /r/mix/o within 10 ms  → (no outputs)  (also reads /ctx)
  ext   button  external                              → (no outputs)
  fire telemetry: not served by this run
";
        assert_eq!(text, expected);
    }

    /// A refused answer, an empty settled one, an incomplete one, a run the
    /// robot cannot describe, a run with no topology and one whose
    /// `run.json` does not parse.
    #[test]
    fn render_live_says_what_each_kind_of_answer_means() {
        let mut refused = reply("a", Vec::new());
        refused.error = Some("not authorized".to_string());
        refused.completeness = RunsCompleteness::not_established();
        assert_eq!(
            render_live(&refused),
            "robot a: refused the runs query (not authorized)\n"
        );

        assert_eq!(render_live(&reply("b", Vec::new())), "robot b: no runs\n");

        let mut incomplete = reply("c", Vec::new());
        incomplete.completeness = RunsCompleteness::Incomplete {
            live_writers: 2,
            writers_heard: 1,
        };
        assert_eq!(
            render_live(&incomplete),
            "robot c: no runs heard (the answer is incomplete: 1 of 2 runs answered in time)\n"
        );

        let mut undescribable = reply("d", Vec::new());
        undescribable.completeness = RunsCompleteness::not_established();
        undescribable.undescribable = vec![UndescribableRun {
            run_id: format_run_id(9),
            reason: "graph.yaml unreadable".to_string(),
        }];
        assert_eq!(
            render_live(&undescribable),
            "robot d: no runs heard (the answer is incomplete)\n\
             robot d · run 0x00000000000000000000000000000009: cannot be described (graph.yaml unreadable)\n"
        );

        let desktop = RunEntry {
            run_id: format_run_id(1),
            graph_name: "perception".to_string(),
            run_started_at_ns: 0,
            state: RunEntryState::Live,
            graph_yaml: "nodes: []\n".to_string(),
            run_json: "{\"version\":1,\"run_dir\":\"/r\"}".to_string(),
        };
        let mut partial = reply("g", vec![desktop.clone()]);
        partial.completeness = RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 1,
        };
        let text = render_live(&partial);
        assert!(
            text.ends_with(
                "robot g: more runs may be live (the answer is incomplete: 1 of 3 runs answered in time)\n"
            ),
            "{text}"
        );
        assert_eq!(
            render_live(&reply("e", vec![desktop])),
            "robot e · run 0x00000000000000000000000000000001 · graph perception · live · started 1970-01-01 00:00:00 UTC\n\
             \x20 this run's host does not report its nodes\n\
             \x20 fire telemetry: not served by this run\n"
        );

        let broken = RunEntry {
            run_id: format_run_id(2),
            graph_name: "x".to_string(),
            run_started_at_ns: 0,
            state: RunEntryState::Live,
            graph_yaml: "nodes: []\n".to_string(),
            run_json: "not json".to_string(),
        };
        let text = render_live(&reply("f", vec![broken]));
        assert!(
            text.starts_with(
                "robot f · run 0x00000000000000000000000000000002 · graph x · live · started 1970-01-01 00:00:00 UTC\n\
                 \x20 run.json does not read ("
            ),
            "{text}"
        );
    }

    /// Two runs on one robot render as two blocks, a blank line apart.
    #[test]
    fn render_live_separates_runs_with_a_blank_line() {
        let mut second = temi_entry();
        second.run_id = format_run_id(0x5d1d);
        let text = render_live(&reply("temi", vec![temi_entry(), second]));
        assert_eq!(text.matches("robot temi · run ").count(), 2);
        assert!(text.contains("lease 5 s)\n\nrobot temi · run 0x00000000000000000000000000005d1d"));
    }

    /// Text a robot controls cannot move the cursor or clear the screen.
    #[test]
    fn render_live_neutralizes_terminal_control_characters() {
        let mut entry = temi_entry();
        entry.graph_name = "evil\u{1b}[2J".to_string();
        let text = render_live(&reply("r\u{7}", vec![entry]));
        assert!(!text.contains('\u{1b}') && !text.contains('\u{7}'));
        assert!(text.starts_with("robot r\u{FFFD} · "));
    }

    /// Robots that answered nothing usable each get one line.
    #[test]
    fn render_unanswered_names_silent_and_unusable_robots() {
        let h = RunsHarvest {
            replies: Vec::new(),
            unusable: vec![UnusableRunsAnswer {
                robot: "old".to_string(),
                reason: "unknown runs wire version 2".to_string(),
            }],
            silent: vec!["quiet".to_string()],
        };
        assert_eq!(
            render_unanswered(&h),
            "robot old: answered the runs query with something this desk cannot read \
             (unknown runs wire version 2); update cerulion on one side\n\
             robot quiet: did not answer the runs query (a robot older than the runs query, \
             or one with no query surface)\n"
        );
    }

    /// `graph live` succeeds when one robot gave a readable answer, and
    /// otherwise names what it asked and what to try.
    #[test]
    fn live_outcome_fails_only_when_no_robot_gave_a_readable_answer() {
        let connect = RemoteTarget {
            connect: vec!["tcp/127.0.0.1:17683".to_string()],
            robot: None,
        };
        let message = |h: &RunsHarvest, t: &RemoteTarget| match live_outcome(h, t) {
            Err(CliError::Validation(text)) => text,
            other => panic!("expected a failure, got {other:?}"),
        };
        assert!(live_outcome(&harvest(vec![reply("idle", Vec::new())]), &connect).is_ok());
        assert_eq!(
            message(&RunsHarvest::default(), &connect),
            "no robot answered the runs query on tcp/127.0.0.1:17683; pass --robot NAME if \
             the robot does not announce itself"
        );
        assert_eq!(
            message(
                &RunsHarvest::default(),
                &RemoteTarget {
                    connect: Vec::new(),
                    robot: Some("temi".to_string()),
                }
            ),
            "no robot answered the runs query through cerulion-netd"
        );
        let mut refused = reply("a", Vec::new());
        refused.error = Some("not authorized".to_string());
        assert_eq!(
            message(&harvest(vec![refused]), &connect),
            "no robot gave a readable runs answer"
        );
        let silent = RunsHarvest {
            silent: vec!["quiet".to_string()],
            ..RunsHarvest::default()
        };
        assert_eq!(
            message(&silent, &connect),
            "no robot gave a readable runs answer"
        );
    }

    // ---- render_fire_row ---------------------------------------------------

    fn temi_topology() -> TopologyView {
        temi_view().topology.unwrap()
    }

    /// THE fire row oracle: one row per node kind of the temi run.
    #[test]
    fn render_fire_row_shows_age_node_cause_duration_and_writes() {
        let t = temi_topology();
        let now = 88_123_456_789;
        let rows = [
            render_fire_row(
                &t,
                now,
                &fire(5120, 88_011_456_789, 38_211, 0, FireCause::Period, 0, 0b1),
            ),
            render_fire_row(
                &t,
                now,
                &fire(5121, 88_123_000_000, 412_003, 1, FireCause::Data, 0, 0b01),
            ),
            render_fire_row(
                &t,
                now,
                &fire(5122, 88_123_456_789, 21_007, 3, FireCause::Data, 0, 0),
            ),
            render_fire_row(
                &t,
                now,
                &fire(5123, 88_100_000_000, 95_001, 2, FireCause::Data, 0, 0b1),
            ),
            render_fire_row(
                &t,
                now,
                &fire(5124, 88_100_000_000, 1_500_000, 1, FireCause::Data, 0, 0b11),
            ),
        ];
        assert_eq!(
            rows,
            [
                "  0.112 s ago  beat     every 1 s                38 µs  wrote /temi/beat/count",
                "  0.000 s ago  logic    on /temi/in             412 µs  wrote /temi/logic/edge",
                "  0.000 s ago  peer     on /temi/logic/edge      21 µs  wrote nothing",
                "  0.023 s ago  hearing  on /temi/chunks          95 µs  wrote /temi/hearing/heard",
                "  0.023 s ago  logic    on /temi/in             1.5 ms  wrote /temi/logic/edge, /temi/logic/pulse",
            ]
        );
    }

    /// Flags, unreported writes, a saturated duration, a cause that differs
    /// from the topology, and a node index the topology does not have.
    #[test]
    fn render_fire_row_reports_flags_and_what_it_cannot_name() {
        let mut t = temi_topology();
        t.nodes[2].reports_writes = false;
        let now = 10_000_000_000;
        let at = 9_000_000_000;
        assert_eq!(
            render_fire_row(
                &t,
                now,
                &fire(1, at, 800, 0, FireCause::Period, FireFlags::PANICKED, 0)
            ),
            "  1.000 s ago  beat     every 1 s               800 ns  wrote nothing  panicked"
        );
        assert_eq!(
            render_fire_row(
                &t,
                now,
                &fire(
                    2,
                    at,
                    3_000_000_000,
                    1,
                    FireCause::Data,
                    FireFlags::TICK_ERROR,
                    0
                )
            ),
            "  1.000 s ago  logic    on /temi/in             3.00 s  wrote nothing  tick error"
        );
        assert_eq!(
            render_fire_row(&t, now, &fire(3, at, 95, 2, FireCause::Data, 0, 0)),
            "  1.000 s ago  hearing  on /temi/chunks          95 ns  wrote not reported"
        );
        assert_eq!(
            render_fire_row(
                &t,
                now,
                &fire(4, at, 95, 1, FireCause::Data, FireFlags::WROTE_UNKNOWN, 0)
            ),
            "  1.000 s ago  logic    on /temi/in              95 ns  wrote not reported"
        );
        assert_eq!(
            render_fire_row(
                &t,
                now,
                &fire(
                    5,
                    at,
                    u32::MAX,
                    1,
                    FireCause::Data,
                    FireFlags::DURATION_SATURATED,
                    0b100
                )
            ),
            "  1.000 s ago  logic    on /temi/in            >4.29 s  wrote output 2"
        );
        assert_eq!(
            render_fire_row(
                &t,
                now,
                &fire(6, at, 10, 0, FireCause::Period, FireFlags::WROTE_TRUNCATED, 0b1)
            ),
            "  1.000 s ago  beat     every 1 s                10 ns  wrote /temi/beat/count, and outputs past 64"
        );
        assert_eq!(
            render_fire_row(&t, now, &fire(7, at, 10, 0, FireCause::Data, 0, 0)),
            "  1.000 s ago  beat     on data                  10 ns  wrote nothing"
        );
        assert_eq!(
            render_fire_row(&t, now, &fire(8, at, 10, 9, FireCause::Unknown, 0, 0b1)),
            "  1.000 s ago  node 9   unknown cause            10 ns  wrote not reported"
        );
        assert_eq!(
            render_fire_row(&t, at, &fire(9, now, 10, 0, FireCause::Period, 0, 0)),
            "  0.000 s ago  beat     every 1 s                10 ns  wrote nothing",
            "a fire stamped after the serve time reads as age zero"
        );
    }

    /// A fire the robot flags as ambiguous names every topic that could have
    /// triggered it.
    #[test]
    fn render_fire_row_names_every_candidate_of_an_ambiguous_trigger() {
        let mut t = temi_topology();
        t.nodes[1].trigger = TriggerView::Data {
            topics: vec!["/temi/in".to_string(), "/temi/aux".to_string()],
            ambiguous: true,
        };
        let row = render_fire_row(
            &t,
            1_000,
            &fire(
                1,
                1_000,
                10,
                1,
                FireCause::Data,
                FireFlags::TRIGGER_AMBIGUOUS,
                0,
            ),
        );
        assert_eq!(
            row,
            "  0.000 s ago  logic    on one of: /temi/in, /temi/aux      10 ns  wrote nothing"
        );
    }

    // ---- JSON --------------------------------------------------------------

    fn assert_only_ids_names_topics_and_numbers(value: &serde_json::Value, path: &str) {
        match value {
            serde_json::Value::Number(_) | serde_json::Value::Bool(_) | serde_json::Value::Null => {
            }
            serde_json::Value::String(s) => assert!(
                !s.contains('\n') && !s.contains(' ') && s.len() <= 128,
                "{path} carries text, not an id, name or topic: {s:?}"
            ),
            serde_json::Value::Array(items) => {
                for (i, item) in items.iter().enumerate() {
                    assert_only_ids_names_topics_and_numbers(item, &format!("{path}[{i}]"));
                }
            }
            serde_json::Value::Object(map) => {
                for (k, v) in map {
                    assert_only_ids_names_topics_and_numbers(v, &format!("{path}.{k}"));
                }
            }
        }
    }

    fn keys(value: &serde_json::Value) -> Vec<&str> {
        let mut keys: Vec<&str> = value
            .as_object()
            .expect("an object")
            .keys()
            .map(String::as_str)
            .collect();
        keys.sort_unstable();
        keys
    }

    /// THE `graph fires --json` row: the exact keys, every value a number,
    /// an id, a name or a topic.
    #[test]
    fn a_fire_json_row_carries_only_ids_names_topics_and_numbers() {
        let run = temi_run();
        let row = fire_json(
            &run,
            88_123_456_789,
            &fire(5121, 88_112_000_000, 412_003, 1, FireCause::Data, 0, 0b11),
        );
        assert_eq!(
            row,
            serde_json::json!({
                "kind": "fire",
                "robot": "temi",
                "run_id": "0x00000000000000000000000000005d1c",
                "seq": 5121,
                "node": 1,
                "node_id": "logic",
                "node_type": "temi_logic",
                "cause": 3,
                "trigger_topics": ["/temi/in"],
                "at_ns": 88_112_000_000u64,
                "age_ns": 11_456_789u64,
                "dur_ns": 412_003,
                "flags": 0,
                "wrote": 3,
                "wrote_topics": ["/temi/logic/edge", "/temi/logic/pulse"],
            })
        );
        assert_only_ids_names_topics_and_numbers(&row, "fire");
    }

    /// A fire the desk cannot name keeps its numbers and names nothing.
    #[test]
    fn a_fire_json_row_for_an_unknown_node_or_unreported_writes_names_nothing() {
        let run = temi_run();
        let row = fire_json(
            &run,
            10,
            &fire(1, 5, 1, 9, FireCause::Unknown, FireFlags::WROTE_UNKNOWN, 0),
        );
        assert_eq!(row["node_id"], serde_json::Value::Null);
        assert_eq!(row["node_type"], serde_json::Value::Null);
        assert_eq!(row["trigger_topics"], serde_json::json!([]));
        assert_eq!(row["wrote_topics"], serde_json::Value::Null);
        assert_eq!(row["flags"], serde_json::json!(FireFlags::WROTE_UNKNOWN));
        assert_only_ids_names_topics_and_numbers(&row, "fire");
    }

    /// THE `graph live --json` run object, and the robot objects for the
    /// robots that gave no readable run. No `graph_yaml`, no `run_json`, no
    /// refusal text.
    #[test]
    fn live_json_carries_only_ids_names_topics_and_numbers() {
        let mut refused = reply("guarded", Vec::new());
        refused.error = Some("not authorized; ask the owner".to_string());
        refused.completeness = RunsCompleteness::not_established();
        let h = RunsHarvest {
            replies: vec![
                reply("temi", vec![temi_entry()]),
                refused,
                reply("idle", Vec::new()),
            ],
            unusable: vec![UnusableRunsAnswer {
                robot: "old".to_string(),
                reason: "unknown runs wire version 2".to_string(),
            }],
            silent: vec!["quiet".to_string()],
        };
        let objects = live_json(&h);
        assert_eq!(objects.len(), 5);
        let run = &objects[0];
        assert_eq!(
            keys(run),
            [
                "endpoints",
                "fires",
                "graph_name",
                "host",
                "kind",
                "nodes",
                "robot",
                "run_id",
                "run_json_readable",
                "run_started_at_ns",
                "state",
                "topology_version"
            ]
        );
        assert_eq!(run["run_json_readable"], true);
        assert_eq!(run["topology_version"], 1);
        assert_eq!(
            run["nodes"][1],
            serde_json::json!({
                "index": 1,
                "id": "logic",
                "type": "temi_logic",
                "trigger": {"kind": "data", "topics": ["/temi/in"], "ambiguous": false},
                "inputs": [{"name": "in", "topic": "/temi/in"}],
                "outputs": [{"name": "edge", "topic": "/temi/logic/edge"},
                            {"name": "pulse", "topic": "/temi/logic/pulse"}],
                "reports_writes": true,
            })
        );
        assert_eq!(
            run["nodes"][0]["trigger"],
            serde_json::json!({"kind": "period", "interval_ns": 1_000_000_000u64})
        );
        assert_eq!(
            run["endpoints"],
            serde_json::json!([{"id": "app",
                "publishes": ["/temi/in", "/temi/chunks"],
                "subscribes": ["/temi/logic/edge", "/temi/hearing/heard"]}])
        );
        assert_eq!(
            run["fires"],
            serde_json::json!({"version": 1, "capacity": 4096, "lease_ms": 5000})
        );
        assert_eq!(
            objects[1..],
            [
                serde_json::json!({"kind": "robot", "robot": "guarded", "status": "refused"}),
                serde_json::json!({"kind": "robot", "robot": "idle", "status": "no_runs"}),
                serde_json::json!({"kind": "robot", "robot": "old", "status": "unusable"}),
                serde_json::json!({"kind": "robot", "robot": "quiet", "status": "silent"}),
            ]
        );
        for (i, object) in objects.iter().enumerate() {
            assert_only_ids_names_topics_and_numbers(object, &format!("object {i}"));
        }
    }

    /// A robot that named some runs and may have more gets an `incomplete`
    /// object after its runs, so a partial answer does not read as a full
    /// one. `undescribable` holds the ids of the runs it could not describe.
    #[test]
    fn live_json_marks_an_answer_that_may_not_name_every_run() {
        let mut partial = reply("temi", vec![temi_entry()]);
        partial.completeness = RunsCompleteness::Incomplete {
            live_writers: 3,
            writers_heard: 2,
        };
        partial.undescribable = vec![UndescribableRun {
            run_id: format_run_id(9),
            reason: "graph.yaml unreadable".to_string(),
        }];
        let mut quiet = reply("c", Vec::new());
        quiet.completeness = RunsCompleteness::Incomplete {
            live_writers: 2,
            writers_heard: 1,
        };
        let objects = live_json(&harvest(vec![partial, quiet]));
        assert_eq!(objects.len(), 3);
        assert_eq!(objects[0]["kind"], "run");
        assert_eq!(objects[0]["run_id"], format_run_id(TEMI_RUN_ID));
        assert_eq!(
            objects[1..],
            [
                serde_json::json!({"kind": "robot", "robot": "temi", "status": "incomplete",
                    "undescribable": [format_run_id(9)]}),
                serde_json::json!({"kind": "robot", "robot": "c", "status": "incomplete",
                    "undescribable": []}),
            ]
        );
        for (i, object) in objects.iter().enumerate() {
            assert_only_ids_names_topics_and_numbers(object, &format!("object {i}"));
        }
    }

    // ---- select_run --------------------------------------------------------

    fn run_entry(run_id: u128, graph: &str, run_json: &str) -> RunEntry {
        RunEntry {
            run_id: format_run_id(run_id),
            graph_name: graph.to_string(),
            run_started_at_ns: 0,
            state: RunEntryState::Live,
            graph_yaml: "nodes: []\n".to_string(),
            run_json: run_json.to_string(),
        }
    }

    /// Without `--run`, the one run that serves fire telemetry, even beside
    /// runs that serve none.
    #[test]
    fn select_run_picks_the_one_run_that_serves_fires() {
        let h = harvest(vec![
            reply("desk", vec![run_entry(1, "perception", "{\"version\":1}")]),
            reply("temi", vec![temi_entry()]),
        ]);
        assert_eq!(select_run(&h, None).expect("one candidate"), temi_run());
    }

    /// `--run` takes the id in any of the spellings `parse_run_id` reads, or
    /// the graph name.
    #[test]
    fn select_run_matches_the_run_id_or_the_graph_name() {
        let h = harvest(vec![reply("temi", vec![temi_entry()])]);
        for spelling in [
            "0x00000000000000000000000000005d1c",
            "5d1c",
            "0X5D1C",
            "temi",
        ] {
            assert_eq!(
                select_run(&h, Some(spelling)).expect(spelling),
                temi_run(),
                "{spelling}"
            );
        }
    }

    /// Every way the choice can fail names what to do.
    #[test]
    fn select_run_refuses_with_the_reason_and_the_remedy() {
        let err = |h: &RunsHarvest, run: Option<&str>| match select_run(h, run) {
            Err(CliError::Validation(text)) => text,
            other => panic!("expected a refusal, got {other:?}"),
        };
        assert_eq!(
            err(&RunsHarvest::default(), None),
            "no robot answered the runs query, so there is no run to read"
        );
        let unread = RunsHarvest {
            unusable: vec![UnusableRunsAnswer {
                robot: "hostile".to_string(),
                reason: "unknown runs wire version 2".to_string(),
            }],
            ..RunsHarvest::default()
        };
        let quiet = RunsHarvest {
            silent: vec!["quiet".to_string()],
            ..RunsHarvest::default()
        };
        for no_reply in [&unread, &quiet] {
            assert_eq!(
                err(no_reply, None),
                "no robot gave a readable runs answer, so there is no run to read"
            );
        }
        let two = harvest(vec![reply(
            "temi",
            vec![temi_entry(), run_entry(0xbeef, "other", TEMI_RUN_JSON)],
        )]);
        assert_eq!(
            err(&two, None),
            "2 runs serve fire telemetry; pass --run with one of: \
             0x00000000000000000000000000005d1c (graph temi on robot temi), \
             0x0000000000000000000000000000beef (graph other on robot temi)"
        );
        let plain = harvest(vec![reply(
            "desk",
            vec![run_entry(1, "perception", "{\"version\":1}")],
        )]);
        assert_eq!(
            err(&plain, None),
            "no run serves fire telemetry; the runs are: \
             0x00000000000000000000000000000001 (graph perception on robot desk)"
        );
        assert_eq!(
            err(&plain, Some("perception")),
            "run 0x00000000000000000000000000000001 (graph perception on robot desk) \
             serves no fire telemetry"
        );
        assert_eq!(
            err(&plain, Some("nope")),
            "no run matches --run nope; the runs are: \
             0x00000000000000000000000000000001 (graph perception on robot desk)"
        );
        let newer = TEMI_RUN_JSON.replace("\"version\": 1}}", "\"version\": 2}}");
        let v2 = harvest(vec![reply(
            "temi",
            vec![run_entry(TEMI_RUN_ID, "temi", &newer)],
        )]);
        assert_eq!(
            err(&v2, Some("temi")),
            "run 0x00000000000000000000000000005d1c (graph temi on robot temi) serves fire \
             telemetry version 2, and this cerulion reads version 1; update cerulion on this machine"
        );
        let same_name = harvest(vec![
            reply("a", vec![run_entry(1, "temi", TEMI_RUN_JSON)]),
            reply("b", vec![run_entry(2, "temi", TEMI_RUN_JSON)]),
        ]);
        assert_eq!(
            err(&same_name, Some("temi")),
            "--run temi matches 2 runs; pass a run id: \
             0x00000000000000000000000000000001 (graph temi on robot a), \
             0x00000000000000000000000000000002 (graph temi on robot b)"
        );
    }

    // ---- FireStream --------------------------------------------------------

    /// The cursor is the `next` of the last page, unchanged: `next` is the
    /// last sequence the page accounts for.
    #[test]
    fn the_cursor_is_the_inclusive_next_of_the_last_page() {
        let mut stream = FireStream::default();
        assert_eq!(stream.cursor(), None);
        let (rows, read_on) = stream.absorb(5119, 0, &[]);
        assert!(rows.is_empty() && !read_on);
        assert_eq!(stream.cursor(), Some(5119));
        let page = [
            fire(5120, 1, 1, 0, FireCause::Period, 0, 1),
            fire(5121, 2, 1, 1, FireCause::Data, 0, 1),
            fire(5123, 3, 1, 2, FireCause::Data, 0, 1),
        ];
        let (rows, read_on) = stream.absorb(5123, 1, &page);
        assert_eq!(rows, page);
        assert!(!read_on);
        assert_eq!(stream.cursor(), Some(5123));
        assert_eq!((stream.fires, stream.lost), (3, 1));
    }

    /// A page that repeats fires already read, or that ends before the
    /// cursor, adds nothing and never moves the cursor back.
    #[test]
    fn a_stale_or_repeated_page_adds_nothing() {
        let mut stream = FireStream::default();
        stream.absorb(10, 0, &[fire(10, 1, 1, 0, FireCause::Period, 0, 1)]);
        let (rows, _) = stream.absorb(
            11,
            0,
            &[
                fire(10, 1, 1, 0, FireCause::Period, 0, 1),
                fire(11, 2, 1, 0, FireCause::Period, 0, 1),
            ],
        );
        assert_eq!(rows.iter().map(|r| r.seq).collect::<Vec<_>>(), [11]);
        let (rows, _) = stream.absorb(9, 4, &[fire(9, 1, 1, 0, FireCause::Period, 0, 1)]);
        assert!(rows.is_empty());
        assert_eq!(stream.cursor(), Some(11));
        assert_eq!((stream.fires, stream.lost), (2, 0));
    }

    /// A full page means more are waiting: the caller asks again at once.
    #[test]
    fn a_full_page_asks_again_at_once() {
        let mut stream = FireStream::default();
        stream.absorb(0, 0, &[]);
        let page: Vec<FireRecord> = (1..=MAX_FIRES_PER_REPLY as u64)
            .map(|seq| fire(seq, seq, 1, 0, FireCause::Period, 0, 1))
            .collect();
        let (rows, read_on) = stream.absorb(MAX_FIRES_PER_REPLY as u64, 0, &page);
        assert_eq!(rows.len(), MAX_FIRES_PER_REPLY);
        assert!(read_on);
    }

    // ---- classify_fires ----------------------------------------------------

    /// The first answer that decodes wins; an answer that does not decode is
    /// unusable; no answer is silence.
    #[test]
    fn classify_fires_takes_the_first_readable_answer() {
        use cerulion_core::transport::cerulion_q::encode_fires_reply;
        let good = FiresReply::new("temi", TEMI_RUN_ID, FiresOutcome::NotServed);
        let skewed = br#"{"version":2,"robot":"temi","run_id":"0x00000000000000000000000000005d1c","outcome":{"kind":"not_served"}}"#.to_vec();
        assert_eq!(
            classify_fires(&[skewed.clone(), encode_fires_reply(&good)]),
            FiresPoll::Reply(good)
        );
        assert_eq!(
            classify_fires(&[skewed]),
            FiresPoll::Unusable {
                reason: "unknown fires wire version 2 (this binary supports 1)".to_string()
            }
        );
        assert_eq!(classify_fires(&[]), FiresPoll::Silent);
    }

    // ---- follow_fires ------------------------------------------------------

    struct FakePacer {
        now: Duration,
        sleeps: u32,
    }

    impl Pacer for FakePacer {
        fn elapsed(&self) -> Duration {
            self.now
        }
        fn sleep(&mut self, d: Duration) {
            self.now += d;
            self.sleeps += 1;
        }
    }

    /// Answers each request from a script; records the cursors it was asked.
    struct ScriptedFires {
        answers: std::collections::VecDeque<FiresPoll>,
        asked: Vec<Option<u64>>,
        then: FiresPoll,
    }

    impl FiresSource for ScriptedFires {
        fn fires(&mut self, robot: &str, run_id: &str, after: Option<u64>) -> FiresPoll {
            assert_eq!(
                (robot, run_id),
                ("temi", "0x00000000000000000000000000005d1c")
            );
            self.asked.push(after);
            self.answers
                .pop_front()
                .unwrap_or_else(|| self.then.clone())
        }
    }

    fn served(now_ns: u64, next: u64, lost: u64, fires: Vec<FireRecord>) -> FiresPoll {
        FiresPoll::Reply(FiresReply::new(
            "temi",
            TEMI_RUN_ID,
            FiresOutcome::Served {
                now_ns,
                armed_since_ns: 1_000,
                next,
                lost,
                in_tick: Vec::new(),
                fires,
            },
        ))
    }

    fn scripted(answers: Vec<FiresPoll>, then: FiresPoll) -> ScriptedFires {
        ScriptedFires {
            answers: answers.into(),
            asked: Vec::new(),
            then,
        }
    }

    fn follow(
        source: &mut ScriptedFires,
        window: Option<Duration>,
        json: bool,
    ) -> (CliResult<(FireStream, FiresEnd)>, String, FakePacer) {
        let mut pacer = FakePacer {
            now: Duration::ZERO,
            sleeps: 0,
        };
        let mut out = Vec::new();
        let running = AtomicBool::new(true);
        let result = follow_fires(
            source,
            &temi_run(),
            window,
            json,
            &running,
            &mut pacer,
            &mut out,
        );
        (result, String::from_utf8(out).unwrap(), pacer)
    }

    /// THE `graph fires` transcript: the first request arms from now, each
    /// later one passes the previous `next`, rows print as they come, and the
    /// totals close the reading when the window ends.
    #[test]
    fn follow_fires_prints_each_fire_then_the_totals() {
        let mut source = scripted(
            vec![
                served(2_000_000_000, 5119, 0, Vec::new()),
                served(
                    2_250_000_000,
                    5121,
                    0,
                    vec![
                        fire(5120, 2_138_000_000, 38_211, 0, FireCause::Period, 0, 1),
                        fire(5121, 2_200_000_000, 412_003, 1, FireCause::Data, 0, 1),
                    ],
                ),
                served(
                    2_500_000_000,
                    5124,
                    1,
                    vec![fire(5124, 2_400_000_000, 95_001, 2, FireCause::Data, 0, 1)],
                ),
            ],
            served(2_750_000_000, 5124, 0, Vec::new()),
        );
        let (result, out, _) = follow(&mut source, Some(Duration::from_secs(1)), false);
        let (stream, end) = result.expect("a clean reading");
        assert_eq!(end, FiresEnd::Window);
        assert_eq!((stream.fires, stream.lost), (3, 1));
        assert_eq!(source.asked, [None, Some(5119), Some(5121), Some(5124)]);
        assert_eq!(
            out,
            "\
fires of run 0x00000000000000000000000000005d1c (graph temi) on robot temi; the robot records while this desk asks
  0.112 s ago  beat     every 1 s                38 µs  wrote /temi/beat/count
  0.050 s ago  logic    on /temi/in             412 µs  wrote /temi/logic/edge
  0.100 s ago  hearing  on /temi/chunks          95 µs  wrote /temi/hearing/heard
3 fires in 1.0 s · 1 lost · the robot stops recording 5 s after the last ask
"
        );
    }

    /// `--json` prints one fire object per line and a summary object last.
    #[test]
    fn follow_fires_json_prints_one_object_per_line() {
        let mut source = scripted(
            vec![
                served(2_000_000_000, 0, 0, Vec::new()),
                served(
                    2_250_000_000,
                    1,
                    0,
                    vec![fire(1, 2_200_000_000, 10, 0, FireCause::Period, 0, 1)],
                ),
            ],
            served(2_500_000_000, 1, 0, Vec::new()),
        );
        let (result, out, _) = follow(&mut source, Some(Duration::from_millis(500)), true);
        result.expect("clean");
        let lines: Vec<serde_json::Value> = out
            .lines()
            .map(|l| serde_json::from_str(l).expect("each line is one JSON object"))
            .collect();
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0]["kind"], "fire");
        assert_eq!(lines[0]["seq"], 1);
        assert_eq!(
            lines[1],
            serde_json::json!({
                "kind": "summary", "robot": "temi", "run_id": "0x00000000000000000000000000005d1c",
                "fires": 1, "lost": 0, "elapsed_ms": 500, "end": "window",
            })
        );
    }

    /// A full page is followed by a request at once, with no wait.
    #[test]
    fn follow_fires_reads_on_without_waiting_after_a_full_page() {
        let page: Vec<FireRecord> = (1..=MAX_FIRES_PER_REPLY as u64)
            .map(|seq| fire(seq, 1_000, 1, 0, FireCause::Period, 0, 1))
            .collect();
        let mut source = scripted(
            vec![
                served(2_000, 0, 0, Vec::new()),
                served(2_000, MAX_FIRES_PER_REPLY as u64, 0, page),
            ],
            served(2_000, MAX_FIRES_PER_REPLY as u64, 0, Vec::new()),
        );
        let (result, _, pacer) = follow(&mut source, Some(Duration::from_millis(300)), true);
        result.expect("clean");
        assert_eq!(
            source.asked,
            [None, Some(0), Some(MAX_FIRES_PER_REPLY as u64)],
            "the full page is followed by a third request inside one interval"
        );
        assert_eq!(pacer.sleeps, 2);
    }

    /// The run ends: the reading stops with the totals and no error.
    #[test]
    fn follow_fires_stops_when_the_run_ends() {
        let ended = FiresPoll::Reply(FiresReply::new(
            "temi",
            TEMI_RUN_ID,
            FiresOutcome::UnknownRun,
        ));
        let mut source = scripted(
            vec![served(1, 0, 0, Vec::new()), served(1, 0, 0, Vec::new())],
            ended,
        );
        let (result, out, _) = follow(&mut source, None, false);
        assert_eq!(
            result.expect("an ended run is not an error").1,
            FiresEnd::RunEnded
        );
        assert!(
            out.ends_with("0 fires in 0.5 s · 0 lost · the run ended\n"),
            "{out}"
        );
    }

    /// Every way the reading fails: the totals still print, then the error.
    #[test]
    fn follow_fires_fails_with_the_reason_after_the_totals() {
        let cases: Vec<(FiresPoll, &str)> = vec![
            (
                FiresPoll::Reply(FiresReply::new(
                    "temi",
                    TEMI_RUN_ID,
                    FiresOutcome::NotServed,
                )),
                "the robot serves no fire telemetry for run 0x00000000000000000000000000005d1c",
            ),
            (
                FiresPoll::Reply(FiresReply::new(
                    "temi",
                    TEMI_RUN_ID,
                    FiresOutcome::Refused {
                        reason: "paired\u{1b}[2J".to_string(),
                    },
                )),
                "the robot refused the fires query (paired?[2J)",
            ),
            (
                FiresPoll::Unusable {
                    reason: "unknown fires wire version 2".to_string(),
                },
                "the robot's fires answer cannot be read (unknown fires wire version 2)",
            ),
            (
                FiresPoll::Reply(FiresReply::new("temi", 0xbeef, FiresOutcome::NotServed)),
                "the robot answered about run 0x0000000000000000000000000000beef, not \
                 0x00000000000000000000000000005d1c",
            ),
        ];
        for (answer, want) in cases {
            let mut source = scripted(vec![served(1, 0, 0, Vec::new())], answer);
            let (result, out, _) = follow(&mut source, None, false);
            match result {
                Err(CliError::Validation(text)) => assert_eq!(text, want),
                other => panic!("expected {want:?}, got {other:?}"),
            }
            assert!(out.contains("0 fires in "), "the totals print first: {out}");
        }
    }

    /// A robot that stops answering is given up on after the silence limit.
    #[test]
    fn follow_fires_gives_up_on_a_silent_robot() {
        let mut source = scripted(vec![served(1, 0, 0, Vec::new())], FiresPoll::Silent);
        let (result, _, pacer) = follow(&mut source, None, false);
        match result {
            Err(CliError::Validation(text)) => {
                assert_eq!(text, "the robot stopped answering the fires query for 3 s")
            }
            other => panic!("expected the silence error, got {other:?}"),
        }
        assert!(
            pacer.now >= FIRES_SILENCE_LIMIT
                && pacer.now < FIRES_SILENCE_LIMIT + Duration::from_secs(1)
        );
    }

    /// Ctrl+C ends the reading with the totals.
    #[test]
    fn follow_fires_stops_on_ctrl_c() {
        let mut source = scripted(Vec::new(), served(1, 0, 0, Vec::new()));
        let mut pacer = FakePacer {
            now: Duration::ZERO,
            sleeps: 0,
        };
        let running = AtomicBool::new(false);
        let mut out = Vec::new();
        let (_, end) = follow_fires(
            &mut source,
            &temi_run(),
            None,
            false,
            &running,
            &mut pacer,
            &mut out,
        )
        .expect("clean");
        assert_eq!(end, FiresEnd::Interrupted);
        assert!(source.asked.is_empty(), "a cleared flag asks nothing");
    }

    // ---- argument errors ---------------------------------------------------

    /// Without `--connect`, `graph fires` names the daemon version it needs
    /// and how to read the robot directly, before it opens anything.
    #[test]
    fn graph_fires_without_connect_names_the_daemon_it_needs() {
        let running = Arc::new(AtomicBool::new(true));
        match graph_fires(&RemoteTarget::default(), None, false, None, false, running) {
            Err(CliError::Validation(text)) => {
                assert!(text.contains("needs cerulion-netd protocol 10"), "{text}");
                assert!(text.contains("--connect"), "{text}");
            }
            other => panic!("expected the netd refusal, got {other:?}"),
        }
    }

    /// Either query-side kill switch refuses both verbs, naming every engaged
    /// variable and the unset remedy; with neither engaged they proceed.
    #[test]
    fn a_kill_switch_refuses_asking_robots() {
        use crate::graph_cmd::query_network_kill_from;
        assert_eq!(
            network_refusal("graph live", &query_network_kill_from(None, None)),
            None
        );
        assert_eq!(
            network_refusal(
                "graph fires",
                &query_network_kill_from(Some(" ".into()), Some("OFF"))
            ),
            None
        );
        assert_eq!(
            network_refusal(
                "graph live",
                &query_network_kill_from(Some(" Off ".into()), None)
            )
            .as_deref(),
            Some(
                "cerulion graph live: asking robots is disabled by CERULION_NETWORK=off, so this \
                 command stays local-only and contacts no robot; unset CERULION_NETWORK to ask \
                 robots through cerulion-netd or --connect"
            )
        );
        assert_eq!(
            network_refusal("graph fires", &query_network_kill_from(None, Some("off"))).as_deref(),
            Some(
                "cerulion graph fires: asking robots is disabled by CERULION_NETD_NETWORK=off, so \
                 this command stays local-only and contacts no robot; unset CERULION_NETD_NETWORK \
                 to ask robots through cerulion-netd or --connect"
            )
        );
        assert_eq!(
            network_refusal(
                "graph fires",
                &query_network_kill_from(Some("on\u{1b}[2J".into()), Some("off"))
            )
            .as_deref(),
            Some(
                "cerulion graph fires: asking robots is disabled by CERULION_NETWORK=on\u{fffd}[2J \
                 (an unrecognized value, which fails closed to local-only) and \
                 CERULION_NETD_NETWORK=off, so this command stays local-only and contacts no \
                 robot; unset CERULION_NETWORK and CERULION_NETD_NETWORK to ask robots through \
                 cerulion-netd or --connect"
            )
        );
    }

    /// `--follow` with `--for`, and a zero window, are refused before any
    /// network is touched.
    #[test]
    fn graph_fires_refuses_contradictory_windows() {
        let target = RemoteTarget {
            connect: vec!["tcp/127.0.0.1:1".to_string()],
            robot: None,
        };
        let running = Arc::new(AtomicBool::new(true));
        for (follow, for_secs, want) in [
            (true, Some(3), "--follow and --for cannot be used together"),
            (false, Some(0), "--for takes a number of seconds above 0"),
        ] {
            match graph_fires(&target, None, follow, for_secs, false, Arc::clone(&running)) {
                Err(CliError::Validation(text)) => assert_eq!(text, want),
                other => panic!("expected {want:?}, got {other:?}"),
            }
        }
    }

    /// The start time renders as a UTC calendar time.
    #[test]
    fn a_start_time_renders_as_utc() {
        assert_eq!(utc_time(0), "1970-01-01 00:00:00 UTC");
        assert_eq!(utc_time(TEMI_STARTED_NS), "2025-11-04 14:02:11 UTC");
        assert_eq!(utc_time(951_782_400_000_000_000), "2000-02-29 00:00:00 UTC");
        assert_eq!(utc_time(u64::MAX), "2554-07-21 23:34:33 UTC");
    }

    // ---- why no run, robot text, versions ---------------------------------

    /// A robot that refused, or whose answer is incomplete, is named with
    /// what it said, not as a robot with no live run.
    #[test]
    fn select_run_names_why_each_robot_gave_no_run() {
        let err = |h: &RunsHarvest, run: Option<&str>| match select_run(h, run) {
            Err(CliError::Validation(text)) => text,
            other => panic!("expected a refusal, got {other:?}"),
        };
        let mut refused = reply("temi", Vec::new());
        refused.error = Some("not authorized".to_string());
        refused.completeness = RunsCompleteness::not_established();
        let mut incomplete = reply("c", Vec::new());
        incomplete.completeness = RunsCompleteness::Incomplete {
            live_writers: 2,
            writers_heard: 1,
        };
        let mut undescribable = reply("d", Vec::new());
        undescribable.completeness = RunsCompleteness::not_established();
        undescribable.undescribable = vec![UndescribableRun {
            run_id: format_run_id(9),
            reason: "graph.yaml unreadable".to_string(),
        }];
        assert_eq!(
            err(&harvest(vec![refused.clone()]), None),
            "no run to read: robot temi refused the runs query (not authorized)"
        );
        assert_eq!(
            err(
                &harvest(vec![incomplete, undescribable, reply("idle", Vec::new())]),
                None
            ),
            "no run to read: robot c named no runs (the answer is incomplete: 1 of 2 runs \
             answered in time); robot d named no runs (the answer is incomplete); robot d cannot \
             describe run 0x00000000000000000000000000000009 (graph.yaml unreadable); no run is \
             live on robot idle"
        );
        let beside = harvest(vec![
            reply("desk", vec![run_entry(1, "perception", "{\"version\":1}")]),
            refused,
        ]);
        assert_eq!(
            err(&beside, None),
            "no run heard serves fire telemetry; the runs heard are: \
             0x00000000000000000000000000000001 (graph perception on robot desk); \
             robot temi refused the runs query (not authorized)"
        );
        assert_eq!(
            err(&beside, Some("temi")),
            "no run heard matches --run temi; the runs heard are: \
             0x00000000000000000000000000000001 (graph perception on robot desk); \
             robot temi refused the runs query (not authorized)"
        );
    }

    /// When a robot asked may host a run this answer does not name, neither
    /// the one run heard that serves fires nor a graph name is a unique
    /// choice: the refusal names the run, its id and every gap. A run id
    /// still reads the run it names.
    #[test]
    fn select_run_needs_a_run_id_when_the_answer_may_not_name_every_run() {
        let err = |h: &RunsHarvest, run: Option<&str>| match select_run(h, run) {
            Err(CliError::Validation(text)) => text,
            other => panic!("expected a refusal, got {other:?}"),
        };
        let temi = "0x00000000000000000000000000005d1c (graph temi on robot temi)";
        let mut partial = reply("temi", vec![temi_entry()]);
        partial.completeness = RunsCompleteness::Incomplete {
            live_writers: 2,
            writers_heard: 1,
        };
        let partial = harvest(vec![partial]);
        let gap = "robot temi may have more runs (the answer is incomplete: 1 of 2 runs \
                   answered in time)";
        assert_eq!(
            err(&partial, None),
            format!(
                "run {temi} is the only run heard that serves fire telemetry, and the answer \
                 may not name every run; pass --run with its id to read it; {gap}"
            )
        );
        assert_eq!(
            err(&partial, Some("temi")),
            format!(
                "--run temi names run {temi} by its graph name, and the answer may not name \
                 every run; pass --run with its id to read it; {gap}"
            )
        );
        for id in ["0x00000000000000000000000000005d1c", "5d1c"] {
            assert_eq!(
                select_run(&partial, Some(id)).expect(id),
                temi_run(),
                "{id}"
            );
        }

        let mut undescribable = reply("temi", vec![temi_entry()]);
        undescribable.completeness = RunsCompleteness::not_established();
        undescribable.undescribable = vec![UndescribableRun {
            run_id: format_run_id(9),
            reason: "graph.yaml unreadable".to_string(),
        }];
        let mut refused = reply("guarded", Vec::new());
        refused.error = Some("not authorized".to_string());
        refused.completeness = RunsCompleteness::not_established();
        let full = || reply("temi", vec![temi_entry()]);
        for (h, gaps) in [
            (
                harvest(vec![undescribable]),
                "robot temi may have more runs (the answer is incomplete); robot temi cannot \
                 describe run 0x00000000000000000000000000000009 (graph.yaml unreadable)",
            ),
            (
                harvest(vec![full(), refused]),
                "robot guarded refused the runs query (not authorized)",
            ),
            (
                RunsHarvest {
                    replies: vec![full()],
                    unusable: vec![UnusableRunsAnswer {
                        robot: "old".to_string(),
                        reason: "unknown runs wire version 2".to_string(),
                    }],
                    silent: vec!["quiet".to_string()],
                },
                "robot old answered the runs query with something this desk cannot read \
                 (unknown runs wire version 2); robot quiet did not answer the runs query",
            ),
        ] {
            assert_eq!(
                err(&h, None),
                format!(
                    "run {temi} is the only run heard that serves fire telemetry, and the \
                     answer may not name every run; pass --run with its id to read it; {gaps}"
                )
            );
            assert_eq!(select_run(&h, Some("5d1c")).expect("by id"), temi_run());
        }

        let mut quiet = reply("c", Vec::new());
        quiet.completeness = RunsCompleteness::Incomplete {
            live_writers: 2,
            writers_heard: 1,
        };
        assert_eq!(
            err(&harvest(vec![full(), quiet]), Some("nope")),
            format!(
                "no run heard matches --run nope; the runs heard are: {temi}; robot c named no \
                 runs (the answer is incomplete: 1 of 2 runs answered in time)"
            )
        );
        let idle_and_quiet = RunsHarvest {
            replies: vec![reply("idle", Vec::new())],
            silent: vec!["quiet".to_string()],
            ..RunsHarvest::default()
        };
        assert_eq!(
            err(&idle_and_quiet, None),
            "no run to read: no run is live on robot idle; robot quiet did not answer the runs \
             query"
        );
    }

    /// Free text a robot sends (a refusal, a reason it cannot describe a run)
    /// cannot start a new line or reorder the terminal's text, and is cut at
    /// 200 characters.
    #[test]
    fn a_robots_free_text_cannot_forge_a_line_or_reorder_the_text() {
        let forged = "denied\u{2028}error: forged \u{202E}txet";
        let shown = "denied?error: forged ?txet";
        let mut refused = reply("a", Vec::new());
        refused.error = Some(forged.to_string());
        refused.completeness = RunsCompleteness::not_established();
        assert_eq!(
            render_live(&refused),
            format!("robot a: refused the runs query ({shown})\n")
        );
        match select_run(&harvest(vec![refused.clone()]), None) {
            Err(CliError::Validation(text)) => assert_eq!(
                text,
                format!("no run to read: robot a refused the runs query ({shown})")
            ),
            other => panic!("expected a refusal, got {other:?}"),
        }
        refused.error = Some("x".repeat(300));
        assert_eq!(
            render_live(&refused),
            format!(
                "robot a: refused the runs query ({}\u{2026})\n",
                "x".repeat(200)
            )
        );
        let mut undescribable = reply("d", Vec::new());
        undescribable.completeness = RunsCompleteness::not_established();
        undescribable.undescribable = vec![UndescribableRun {
            run_id: format_run_id(9),
            reason: forged.to_string(),
        }];
        assert!(render_live(&undescribable).ends_with(&format!("cannot be described ({shown})\n")));
        let mut source = scripted(
            vec![served(1, 0, 0, Vec::new())],
            FiresPoll::Reply(FiresReply::new(
                "temi",
                TEMI_RUN_ID,
                FiresOutcome::Refused {
                    reason: forged.to_string(),
                },
            )),
        );
        match follow(&mut source, None, false).0 {
            Err(CliError::Validation(text)) => {
                assert_eq!(text, format!("the robot refused the fires query ({shown})"))
            }
            other => panic!("expected the refusal, got {other:?}"),
        }
    }

    /// The characters a hostile robot puts in every string it sends: an
    /// escape, NUL, BEL, DEL and the C1 controls that open a terminal
    /// sequence, the bidi and format controls that reorder, hide or join
    /// text, and the Unicode line and paragraph separators.
    const HOSTILE_CHARS: [char; 24] = [
        '\u{1b}',
        '\u{0}',
        '\u{7}',
        '\u{7f}',
        '\u{85}',
        '\u{9b}',
        '\u{9d}',
        '\u{ad}',
        '\u{061C}',
        '\u{200B}',
        '\u{200D}',
        '\u{200E}',
        '\u{200F}',
        '\u{2028}',
        '\u{2029}',
        '\u{202A}',
        '\u{202D}',
        '\u{202E}',
        '\u{2060}',
        '\u{2066}',
        '\u{2068}',
        '\u{2069}',
        '\u{FEFF}',
        '\u{E0041}',
    ];
    /// Every [`HOSTILE_CHARS`] character once: the escape opens a screen
    /// clear, and the others follow it.
    const HOSTILE: &str =
        "x\u{1b}[2J\u{0}\u{7}\u{7f}\u{85}\u{9b}\u{9d}\u{ad}\u{061C}\u{200B}\u{200D}\
                           \u{200E}\u{200F}\u{2028}\u{2029}\u{202A}\u{202D}\u{202E}\u{2060}\u{2066}\
                           \u{2068}\u{2069}\u{FEFF}\u{E0041}y";

    /// [`HOSTILE`] as a JSON string body.
    fn hostile_json() -> String {
        let quoted = serde_json::to_string(HOSTILE).expect("a string serializes");
        quoted[1..quoted.len() - 1].to_string()
    }

    /// The temi `run.json` with a sync node that also reads an input outside
    /// its trigger, and [`HOSTILE`] in every node id and type, every topic,
    /// the endpoint id and the host.
    fn hostile_run_json() -> String {
        let hostile = hostile_json();
        TEMI_RUN_JSON
            .replace(
                r#""outputs": [], "reports_writes": true}"#,
                r#""outputs": [], "reports_writes": true},
      {"id": "fuse", "type": "temi_fuse",
       "trigger": {"kind": "sync", "topics": ["/temi/in", "/temi/chunks"], "window_ns": 5000000},
       "inputs": [{"name": "in", "topic": "/temi/in"}, {"name": "chunks", "topic": "/temi/chunks"},
                  {"name": "mood", "topic": "/temi/mood"}],
       "outputs": [], "reports_writes": true}"#,
            )
            .replace("\"/temi/", &format!("\"/{hostile}/"))
            .replace("\"id\": \"", &format!("\"id\": \"{hostile}"))
            .replace("\"type\": \"", &format!("\"type\": \"{hostile}"))
            .replace("\"embedded\"", &format!("\"{hostile}\""))
    }

    fn assert_neutralized(text: &str) {
        let raw: Vec<char> = text.chars().filter(|c| HOSTILE_CHARS.contains(c)).collect();
        assert!(raw.is_empty(), "{raw:?} reached the terminal in {text:?}");
    }

    /// Answers each `fires` request from a list.
    struct Replay(std::collections::VecDeque<FiresPoll>);

    impl FiresSource for Replay {
        fn fires(&mut self, _robot: &str, _run_id: &str, _after: Option<u64>) -> FiresPoll {
            self.0.pop_front().unwrap_or(FiresPoll::Silent)
        }
    }

    fn follow_replay(
        run: &SelectedRun,
        answers: Vec<FiresPoll>,
        json: bool,
    ) -> (CliResult<(FireStream, FiresEnd)>, String) {
        let mut pacer = FakePacer {
            now: Duration::ZERO,
            sleeps: 0,
        };
        let mut out = Vec::new();
        let result = follow_fires(
            &mut Replay(answers.into()),
            run,
            None,
            json,
            &AtomicBool::new(true),
            &mut pacer,
            &mut out,
        );
        (result, String::from_utf8(out).unwrap())
    }

    /// Every string a robot sends (its robot id, run ids, graph names, node
    /// ids and types, trigger and input topics, host and endpoint ids, its
    /// refusal and undescribable reasons, and why its answers cannot be read)
    /// reaches the terminal with no control, line separator, bidi or format
    /// character left in it. `--json` keeps the value, and writes each of
    /// those characters as a `\u` escape.
    #[test]
    fn no_robot_string_reaches_the_terminal_with_a_control_left_in_it() {
        assert_eq!(
            HOSTILE
                .chars()
                .filter(|c| HOSTILE_CHARS.contains(c))
                .count(),
            HOSTILE_CHARS.len()
        );
        let tail = HOSTILE_CHARS.len() - 1;
        let as_name = format!("x\u{FFFD}[2J{}y", "\u{FFFD}".repeat(tail));
        let as_reason = format!("x?[2J{}y", "?".repeat(tail));
        let run_json = hostile_run_json();
        let view = parse_run_json(&run_json).expect("the hostile run.json reads");
        let nodes = &view.topology.as_ref().expect("a topology").nodes;
        assert_eq!(nodes[1].id, format!("{HOSTILE}logic"));
        assert_eq!(nodes[1].outputs[0].topic, format!("/{HOSTILE}/logic/edge"));

        let entry = RunEntry {
            run_id: format!("0x5d1c{HOSTILE}"),
            graph_name: HOSTILE.to_string(),
            run_json,
            ..temi_entry()
        };
        let mut answered = reply(HOSTILE, vec![entry.clone()]);
        answered.undescribable = vec![UndescribableRun {
            run_id: HOSTILE.to_string(),
            reason: HOSTILE.to_string(),
        }];
        let mut refused = reply(HOSTILE, Vec::new());
        refused.error = Some(HOSTILE.to_string());
        let h = RunsHarvest {
            replies: vec![answered.clone(), refused],
            unusable: vec![UnusableRunsAnswer {
                robot: HOSTILE.to_string(),
                reason: HOSTILE.to_string(),
            }],
            silent: vec![HOSTILE.to_string()],
        };

        let mut shown: Vec<String> = h.replies.iter().map(render_live).collect();
        assert_eq!(
            render_unanswered(&h),
            format!(
                "robot {as_name}: answered the runs query with something this desk cannot read \
                 ({as_reason}); update cerulion on one side\n\
                 robot {as_name}: did not answer the runs query (a robot older than the runs \
                 query, or one with no query surface)\n"
            )
        );
        let mut twice = answered.clone();
        twice.runs.push(entry.clone());
        for (asked, run) in [
            (&h, Some("none")),
            (&h, None),
            (&harvest(vec![twice]), None),
        ] {
            match select_run(asked, run) {
                Err(CliError::Validation(text)) => shown.push(text),
                other => panic!("expected a refusal, got {other:?}"),
            }
        }

        let run = select_run(&harvest(vec![reply(HOSTILE, vec![entry.clone()])]), None)
            .expect("the one run that serves fires");
        let page = |fires: Vec<FireRecord>| {
            FiresPoll::Reply(FiresReply {
                version: FIRES_WIRE_VERSION,
                robot: HOSTILE.to_string(),
                run_id: run.run_id.clone(),
                outcome: FiresOutcome::Served {
                    now_ns: 9_000_000,
                    armed_since_ns: 0,
                    next: 5,
                    lost: 0,
                    in_tick: Vec::new(),
                    fires,
                },
            })
        };
        let rows = vec![
            fire(1, 1_000_000, 38_000, 0, FireCause::Period, 0, 0b1),
            fire(2, 2_000_000, 412_000, 1, FireCause::Data, 0, 0b11),
            fire(3, 3_000_000, 95_000, 2, FireCause::Data, 0, 0b1),
            fire(4, 4_000_000, 21_000, 3, FireCause::Data, 0, 0),
            fire(5, 5_000_000, 10_000, 4, FireCause::Sync, 0, 0),
        ];
        let hostile_reply = |outcome| {
            FiresPoll::Reply(FiresReply {
                version: FIRES_WIRE_VERSION,
                robot: HOSTILE.to_string(),
                run_id: run.run_id.clone(),
                outcome,
            })
        };
        let endings = [
            hostile_reply(FiresOutcome::Refused {
                reason: HOSTILE.to_string(),
            }),
            FiresPoll::Unusable {
                reason: HOSTILE.to_string(),
            },
            FiresPoll::Reply(FiresReply {
                run_id: format!("0x9{HOSTILE}"),
                ..FiresReply::new(HOSTILE, 9, FiresOutcome::UnknownRun)
            }),
        ];
        for ending in endings {
            let (result, text) = follow_replay(&run, vec![page(rows.clone()), ending], false);
            assert_eq!(text.lines().count(), 7, "{text}");
            shown.push(text);
            match result {
                Err(CliError::Validation(reason)) => shown.push(reason),
                other => panic!("expected the reading to fail, got {other:?}"),
            }
        }
        for text in &shown {
            assert_neutralized(text);
        }
        assert!(shown[0].starts_with(&format!(
            "robot {as_name} · run 0x5d1c{as_name} · graph {as_name} · live · host {as_name}"
        )));
        assert!(shown[0].contains(&format!("  {as_name}logic ")));
        assert!(shown[0].contains(&format!("→ /{as_name}/logic/edge, /{as_name}/logic/pulse")));
        assert!(shown[0].contains(&format!(
            "  on all of: /{as_name}/in, /{as_name}/chunks within 5 ms  → (no outputs)  \
             (also reads /{as_name}/mood)\n"
        )));
        assert!(shown.iter().any(|text| text.contains(&format!(
            " ago  {as_name}fuse     on all of: /{as_name}/in, /{as_name}/chunks within 5 ms  "
        ))));
        assert!(shown[0].ends_with(&format!(
            "robot {as_name} · run {as_name}: cannot be described ({as_reason})\n"
        )));
        assert_eq!(
            shown[1],
            format!("robot {as_name}: refused the runs query ({as_reason})\n")
        );
        assert!(shown.iter().any(|text| text.ends_with(&format!(
            "the robot's fires answer cannot be read ({as_reason})"
        ))));

        let target = RemoteTarget {
            connect: Vec::new(),
            robot: Some("temi".to_string()),
        };
        let (mut out, mut err) = (Vec::new(), Vec::new());
        show_live(
            &mut CannedRuns(h.clone()),
            &target,
            true,
            &AtomicBool::new(true),
            &mut out,
            &mut err,
        )
        .expect("a readable answer");
        let out = String::from_utf8(out).unwrap();
        let objects: Vec<serde_json::Value> = out
            .lines()
            .map(|line| {
                assert_neutralized(line);
                serde_json::from_str(line).expect("valid JSON")
            })
            .collect();
        assert_eq!(objects, live_json(&h));
        assert_neutralized(&String::from_utf8(err).unwrap());
        let first = &objects[0];
        assert_eq!(first["robot"], HOSTILE);
        assert_eq!(first["graph_name"], HOSTILE);
        assert_eq!(first["nodes"][1]["id"], format!("{HOSTILE}logic"));
        assert_eq!(
            first["nodes"][1]["outputs"][0]["topic"],
            format!("/{HOSTILE}/logic/edge")
        );
        let (_, text) = follow_replay(&run, vec![page(rows)], true);
        let fires: Vec<serde_json::Value> = text
            .lines()
            .map(|line| {
                assert_neutralized(line);
                serde_json::from_str(line).expect("valid JSON")
            })
            .collect();
        assert_eq!(fires.len(), 6);
        assert_eq!(fires[5]["robot"], HOSTILE);
        assert_eq!(fires[1]["node_id"], format!("{HOSTILE}logic"));
        assert_eq!(
            fires[1]["wrote_topics"][1],
            format!("/{HOSTILE}/logic/pulse")
        );
        assert_eq!(fires[1]["trigger_topics"][0], format!("/{HOSTILE}/in"));
    }

    /// Why a run's `run.json` does not read can quote the robot's own text,
    /// so it is cut at 200 characters, with no control left in it, in
    /// `graph live` and in the refusal of `graph fires --run`.
    #[test]
    fn a_run_json_that_does_not_read_is_named_in_bounded_text() {
        let quoted = format!("{}{HOSTILE}", "z".repeat(300));
        let entry = RunEntry {
            run_json: format!(
                "{{\"topology\": {}}}",
                serde_json::to_string(&quoted).unwrap()
            ),
            ..temi_entry()
        };
        let shown = render_live(&reply("temi", vec![entry.clone()]));
        assert_neutralized(&shown);
        let prefix = "  run.json does not read (";
        let line = shown
            .lines()
            .find(|line| line.starts_with(prefix))
            .unwrap_or_else(|| panic!("no run.json line in {shown}"));
        assert_eq!(line.chars().count(), prefix.len() + 200 + 2, "{line}");
        assert!(line.ends_with("z\u{2026})"), "{line}");

        match select_run(&harvest(vec![reply("temi", vec![entry])]), Some("temi")) {
            Err(CliError::Validation(text)) => {
                assert_neutralized(&text);
                let prefix = "run 0x00000000000000000000000000005d1c (graph temi on robot temi): \
                              its run.json does not read (";
                assert!(text.starts_with(prefix), "{text}");
                assert_eq!(text.chars().count(), prefix.len() + 200 + 2, "{text}");
                assert!(text.ends_with("z\u{2026})"), "{text}");
            }
            other => panic!("expected a refusal, got {other:?}"),
        }
    }

    /// A robot name, id or topic longer than the display limit is cut with a
    /// marker on the terminal, so one long string cannot flood it or widen
    /// every fire row. `--json` keeps the whole value.
    #[test]
    fn a_long_robot_string_is_cut_on_the_terminal() {
        let cut = |c: &str| format!("{}\u{2026}", c.repeat(DISPLAY_LIMIT));
        let mut topology = temi_topology();
        topology.nodes[0].id = "n".repeat(20_000);
        let now = 88_123_456_789;
        let beat = render_fire_row(
            &topology,
            now,
            &fire(5120, 88_011_456_789, 38_211, 0, FireCause::Period, 0, 0b1),
        );
        assert!(
            beat.starts_with(&format!("  0.112 s ago  {}  every 1 s  ", cut("n"))),
            "{beat}"
        );
        let logic = render_fire_row(
            &topology,
            now,
            &fire(5121, 88_123_000_000, 412_003, 1, FireCause::Data, 0, 0b01),
        );
        assert_eq!(
            logic,
            format!(
                "  0.000 s ago  {:<w$}  on /temi/in             412 µs  wrote /temi/logic/edge",
                "logic",
                w = DISPLAY_LIMIT + 1
            )
        );

        let entry = RunEntry {
            graph_name: "g".repeat(300_000),
            ..temi_entry()
        };
        let answered = reply("temi", vec![entry]);
        let shown = render_live(&answered);
        assert!(
            shown.starts_with(&format!(
                "robot temi · run 0x00000000000000000000000000005d1c · graph {} · live",
                cut("g")
            )),
            "{}",
            shown.chars().take(200).collect::<String>()
        );
        let longest = shown.lines().map(|l| l.chars().count()).max();
        assert!(longest < Some(2 * DISPLAY_LIMIT), "{longest:?}");
        let objects = live_json(&harvest(vec![answered.clone()]));
        assert_eq!(
            objects[0]["graph_name"].as_str().map(str::len),
            Some(300_000)
        );

        let run = select_run(&harvest(vec![answered]), None).expect("the temi run");
        let (_, text) = follow_replay(&run, Vec::new(), false);
        let header = text.lines().next().unwrap_or_default();
        assert_eq!(
            header,
            format!(
                "fires of run 0x00000000000000000000000000005d1c (graph {}) on robot temi; the \
                 robot records while this desk asks",
                cut("g")
            )
        );
    }

    /// A run whose fire telemetry or topology is a version this desk does
    /// not read says so, and is not read as version 1.
    #[test]
    fn a_newer_telemetry_or_topology_version_is_named_not_read_as_version_1() {
        let fires_v2 = TEMI_RUN_JSON.replace("\"version\": 1}}", "\"version\": 2}}");
        let text = render_live(&reply(
            "temi",
            vec![run_entry(TEMI_RUN_ID, "temi", &fires_v2)],
        ));
        assert!(
            text.ends_with("  fire telemetry: version 2 (this cerulion reads version 1)\n"),
            "{text}"
        );

        let mut doc: serde_json::Value = serde_json::from_str(TEMI_RUN_JSON).unwrap();
        doc["topology"]["version"] = serde_json::json!(2);
        let entry = run_entry(TEMI_RUN_ID, "temi", &doc.to_string());
        let text = render_live(&reply("temi", vec![entry.clone()]));
        assert!(
            text.ends_with(
                "  the node rows follow topology version 2, and this cerulion reads version 1, \
                 so they can be wrong; update cerulion on this machine\n\
                 \x20 fire telemetry: available (v1, ring 4096, lease 5 s)\n"
            ),
            "{text}"
        );
        match select_run(&harvest(vec![reply("temi", vec![entry.clone()])]), None) {
            Err(CliError::Validation(text)) => assert_eq!(
                text,
                "run 0x00000000000000000000000000005d1c (graph temi on robot temi) describes \
                 its nodes in topology version 2, and this cerulion reads version 1; update \
                 cerulion on this machine"
            ),
            other => panic!("expected a refusal, got {other:?}"),
        }
        let objects = live_json(&harvest(vec![reply("temi", vec![entry])]));
        assert_eq!(objects[0]["topology_version"], 2);
    }

    /// `graph live --json` marks a run whose `run.json` does not read, so it
    /// is not taken for a run whose host reports no nodes.
    #[test]
    fn live_json_marks_a_run_json_that_does_not_read() {
        let broken = live_json(&harvest(vec![reply(
            "f",
            vec![run_entry(2, "x", "not json")],
        )]));
        assert_eq!(broken[0]["kind"], "run");
        assert_eq!(broken[0]["run_json_readable"], false);
        for key in ["host", "topology_version", "nodes", "endpoints", "fires"] {
            assert_eq!(broken[0][key], serde_json::Value::Null, "{key}");
        }
        let desktop = live_json(&harvest(vec![reply(
            "e",
            vec![run_entry(1, "perception", "{\"version\":1}")],
        )]));
        assert_eq!(desktop[0]["run_json_readable"], true);
        assert_eq!(desktop[0]["nodes"], serde_json::Value::Null);
        assert_eq!(desktop[0]["endpoints"], serde_json::json!([]));
    }

    // ---- the reading window and silence ----------------------------------

    /// `--for N` reads N seconds, no flag reads the default, `--follow`
    /// reads until Ctrl+C.
    #[test]
    fn the_reading_window_is_the_given_seconds_the_default_or_until_ctrl_c() {
        assert!(matches!(fires_window(true, None), Ok(None)));
        assert!(matches!(
            fires_window(false, Some(3)),
            Ok(Some(w)) if w == Duration::from_secs(3)
        ));
        assert!(matches!(
            fires_window(false, None),
            Ok(Some(w)) if w == Duration::from_secs(DEFAULT_FIRES_SECS)
        ));
    }

    /// A window of 3 s or less never ends as a clean reading while the robot
    /// is not answering: the robot gets the rest of its silence limit.
    #[test]
    fn a_window_ends_only_after_an_answered_ask() {
        for window in [Duration::from_secs(1), Duration::from_secs(3)] {
            let mut source = scripted(Vec::new(), FiresPoll::Silent);
            let (result, out, pacer) = follow(&mut source, Some(window), false);
            match result {
                Err(CliError::Validation(text)) => {
                    assert_eq!(text, "the robot did not answer the fires query for 3 s")
                }
                other => panic!("{window:?}: expected the silence error, got {other:?}"),
            }
            assert_eq!(pacer.now, FIRES_SILENCE_LIMIT);
            assert!(out.ends_with("0 fires in 3.0 s · 0 lost · the robot stops recording 5 s after the last ask\n"), "{out}");
        }

        let mut source = scripted(vec![served(1, 0, 0, Vec::new())], FiresPoll::Silent);
        match follow(&mut source, Some(Duration::from_secs(3)), true).0 {
            Err(CliError::Validation(text)) => {
                assert_eq!(text, "the robot stopped answering the fires query for 3 s")
            }
            other => panic!("expected the silence error, got {other:?}"),
        }

        let quiet = || served(1, 0, 0, Vec::new());
        let mut source = scripted(
            vec![quiet(), quiet(), quiet(), FiresPoll::Silent, quiet()],
            quiet(),
        );
        let (result, _, pacer) = follow(&mut source, Some(Duration::from_secs(1)), false);
        assert_eq!(
            result.expect("a missed ask, then an answer").1,
            FiresEnd::Window
        );
        assert_eq!(source.asked.len(), 5, "one ask past the window, answered");
        assert_eq!(pacer.now, Duration::from_millis(1250));
    }

    /// An answer resets the silence limit: a robot that misses asks, but
    /// never for 3 s in a row, is read until the window ends.
    #[test]
    fn an_answer_resets_the_silence_limit() {
        let mut answers = Vec::new();
        for _ in 0..3 {
            answers.extend(std::iter::repeat_with(|| FiresPoll::Silent).take(10));
            answers.push(served(1, 0, 0, Vec::new()));
        }
        let mut source = scripted(answers, served(1, 0, 0, Vec::new()));
        let (result, _, _) = follow(&mut source, Some(Duration::from_secs(8)), false);
        assert_eq!(
            result.expect("never 3 s without an answer").1,
            FiresEnd::Window
        );
    }

    /// A page that repeats a fire already shown prints only the new fires.
    #[test]
    fn follow_fires_prints_only_the_fires_not_shown_before() {
        let first = fire(1, 1_000, 10, 0, FireCause::Period, 0, 1);
        let second = fire(2, 2_000, 10, 0, FireCause::Period, 0, 1);
        let mut source = scripted(
            vec![
                served(3_000, 1, 0, vec![first]),
                served(3_000, 2, 0, vec![first, second]),
            ],
            served(3_000, 2, 0, Vec::new()),
        );
        let (result, out, _) = follow(&mut source, Some(Duration::from_millis(500)), true);
        assert_eq!(result.expect("clean").0.fires, 2);
        let seqs: Vec<u64> = out
            .lines()
            .map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
            .filter(|object| object["kind"] == "fire")
            .map(|object| object["seq"].as_u64().unwrap())
            .collect();
        assert_eq!(seqs, [1, 2]);
    }

    // ---- reaching the robot ----------------------------------------------

    /// A loopback locator where nothing listens.
    fn closed_locator() -> String {
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .and_then(|l| l.local_addr())
            .expect("an ephemeral loopback port")
            .port();
        format!("tcp/127.0.0.1:{port}")
    }

    /// A locator where nothing listens is skipped beside one that answers,
    /// and named at once when it is the only one; a malformed locator is
    /// refused before any connection.
    #[test]
    fn the_session_dials_only_locators_that_accept_a_connection() {
        let closed = closed_locator();
        let started = Instant::now();
        match ZenohPlane::open(std::slice::from_ref(&closed)) {
            Err(CliError::Validation(text)) => assert_eq!(
                text,
                format!(
                    "nothing accepted a connection at {closed} within 1 s; check that the robot \
                     listens there (a robot reached through adb needs its port forwarded, for \
                     example adb forward tcp:17683 tcp:7683)"
                )
            ),
            Err(other) => panic!("expected the locator error, got {other}"),
            Ok(_) => panic!("a closed port opens no session"),
        }
        assert!(started.elapsed() < Duration::from_secs(2));

        let open = "tcp/127.0.0.1:7683".to_string();
        let reachable: HashSet<String> = [open.clone()].into();
        for order in [
            [closed.clone(), open.clone()],
            [open.clone(), closed.clone()],
        ] {
            assert_eq!(
                connect_set(&order, &reachable).expect("one reachable"),
                [open.as_str()]
            );
        }
        match connect_set(&["127.0.0.1:7683".to_string(), open], &reachable) {
            Err(CliError::Validation(text)) => assert_eq!(
                text,
                "malformed --connect locator 127.0.0.1:7683; a locator has the form \
                 tcp/<host>:<port>, for example tcp/127.0.0.1:7683"
            ),
            other => panic!("expected the syntax error, got {other:?}"),
        }
    }

    /// The first-contact wait stops when Ctrl+C clears the flag, returns an
    /// error at once, and asks again only while nobody answered.
    #[test]
    fn the_first_contact_wait_stops_on_ctrl_c_and_never_retries_an_error() {
        let pacer = || FakePacer {
            now: Duration::ZERO,
            sleeps: 0,
        };
        let message = |result: CliResult<RunsHarvest>| match result {
            Err(CliError::Validation(text)) => text,
            other => panic!("expected a failure, got {other:?}"),
        };
        const INTERRUPTED: &str = "interrupted while asking for runs; nothing was read";

        let stopped = AtomicBool::new(false);
        let mut asked = 0;
        let result = first_contact(
            || {
                asked += 1;
                Ok(RunsHarvest::default())
            },
            &stopped,
            &mut pacer(),
        );
        assert_eq!((message(result).as_str(), asked), (INTERRUPTED, 0));

        let running = AtomicBool::new(true);
        let mut asked = 0;
        let mut p = pacer();
        let result = first_contact(
            || {
                asked += 1;
                Err(CliError::Validation("announce failed".to_string()))
            },
            &running,
            &mut p,
        );
        assert_eq!(
            (message(result).as_str(), asked, p.sleeps),
            ("announce failed", 1, 0)
        );

        let mut asked = 0;
        let result = first_contact(
            || {
                asked += 1;
                running.store(false, Ordering::Relaxed);
                Ok(RunsHarvest::default())
            },
            &running,
            &mut pacer(),
        );
        assert_eq!((message(result).as_str(), asked), (INTERRUPTED, 1));

        let running = AtomicBool::new(true);
        let mut asked = 0;
        let mut p = pacer();
        let result = first_contact(
            || {
                asked += 1;
                Ok(RunsHarvest::default())
            },
            &running,
            &mut p,
        );
        assert!(result.expect("an empty harvest").replies.is_empty());
        assert_eq!((asked, p.now), (21, FIRST_CONTACT_BUDGET));

        let mut asked = 0;
        let mut p = pacer();
        let result = first_contact(
            || {
                asked += 1;
                Ok(if asked < 3 {
                    RunsHarvest::default()
                } else {
                    harvest(vec![reply("temi", Vec::new())])
                })
            },
            &running,
            &mut p,
        );
        assert_eq!(
            (result.expect("a reply").replies.len(), asked, p.sleeps),
            (1, 3, 2)
        );
    }

    /// A `runs` source that gives one canned harvest.
    struct CannedRuns(RunsHarvest);

    impl RunsSource for CannedRuns {
        fn runs(&mut self, robot: Option<&str>, _running: &AtomicBool) -> CliResult<RunsHarvest> {
            assert_eq!(robot, Some("temi"));
            Ok(self.0.clone())
        }
    }

    /// `graph live` writes the runs to stdout, and in `--json` mode the
    /// robots that gave no reply to stderr.
    #[test]
    fn graph_live_writes_the_runs_and_the_robots_that_did_not_answer() {
        let target = RemoteTarget {
            connect: Vec::new(),
            robot: Some("temi".to_string()),
        };
        let canned = RunsHarvest {
            replies: vec![reply("temi", vec![temi_entry()])],
            unusable: Vec::new(),
            silent: vec!["quiet".to_string()],
        };
        let running = AtomicBool::new(true);
        let show = |harvest: &RunsHarvest, json: bool| {
            let (mut out, mut err) = (Vec::new(), Vec::new());
            let result = show_live(
                &mut CannedRuns(harvest.clone()),
                &target,
                json,
                &running,
                &mut out,
                &mut err,
            );
            (
                result,
                String::from_utf8(out).unwrap(),
                String::from_utf8(err).unwrap(),
            )
        };

        let (result, out, err) = show(&canned, false);
        result.expect("a readable answer");
        assert_eq!(
            out,
            format!(
                "{}{}",
                render_live(&canned.replies[0]),
                render_unanswered(&canned)
            )
        );
        assert_eq!(err, "");

        let (result, out, err) = show(&canned, true);
        result.expect("a readable answer");
        let objects: Vec<serde_json::Value> = out
            .lines()
            .map(|line| serde_json::from_str(line).expect("one JSON object per line"))
            .collect();
        assert_eq!(objects, live_json(&canned));
        assert_eq!(err, render_unanswered(&canned));

        match show(&RunsHarvest::default(), false).0 {
            Err(CliError::Validation(text)) => {
                assert_eq!(
                    text,
                    "no robot answered the runs query through cerulion-netd"
                )
            }
            other => panic!("expected a failure, got {other:?}"),
        }
    }

    /// A runs source that asks on a worker thread and joins it, as the zenoh
    /// plane asks each robot. The worker writes to stderr the way a log line
    /// does, which takes the process's stderr lock.
    struct LoggingRuns;

    impl RunsSource for LoggingRuns {
        fn runs(&mut self, _robot: Option<&str>, _running: &AtomicBool) -> CliResult<RunsHarvest> {
            std::thread::scope(|s| s.spawn(|| std::io::stderr().write_all(b"")).join())
                .expect("the worker finishes")?;
            Ok(RunsHarvest::default())
        }
    }

    /// `graph live` holds no lock on stdout or stderr while it asks, so a
    /// source that logs from a thread it joins cannot hang the command.
    #[test]
    fn graph_live_holds_no_terminal_lock_while_it_asks() {
        let (done, finished) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let target = RemoteTarget {
                connect: Vec::new(),
                robot: Some("temi".to_string()),
            };
            let _ = done.send(live_on_terminal(&mut LoggingRuns, &target, false).is_err());
        });
        assert_eq!(
            finished.recv_timeout(Duration::from_secs(10)),
            Ok(true),
            "graph live did not finish: it waits for a worker that waits for stderr"
        );
    }

    /// Without `--connect`, `graph live` reads one runs answer from
    /// `cerulion-netd`: here a fake daemon on a socket of its own.
    #[cfg(unix)]
    #[test]
    fn graph_live_reads_the_runs_through_cerulion_netd() {
        use cerulion_netd::protocol::{DiscoveryState, Hello, Response, RunsQueryResponse};
        use std::io::{BufRead, BufReader};

        let dir = tempfile::Builder::new()
            .prefix("graph-live")
            .tempdir_in("/tmp")
            .expect("a socket directory");
        let socket = dir.path().join("netd.sock");
        let listener = std::os::unix::net::UnixListener::bind(&socket).expect("bind");
        let daemon = std::thread::spawn(move || {
            let (stream, _) = listener.accept().expect("the client connects");
            let mut writer = stream.try_clone().expect("a second handle");
            writeln!(writer, "{}", serde_json::to_string(&Hello::new()).unwrap()).unwrap();
            let mut line = String::new();
            BufReader::new(stream)
                .read_line(&mut line)
                .expect("one request");
            let request: serde_json::Value = serde_json::from_str(&line).expect("JSON");
            let response = Response::RunsQuery(RunsQueryResponse {
                id: request["id"].as_u64().expect("an id"),
                run_replies: vec![reply("temi", vec![temi_entry()])],
                unusable: vec![UnusableRunsAnswer {
                    robot: "old".to_string(),
                    reason: "unknown runs wire version 2".to_string(),
                }],
                silent: vec!["quiet".to_string()],
                discovery: DiscoveryState::Settled,
                plane_unsettled_ms: None,
            });
            writeln!(writer, "{}", serde_json::to_string(&response).unwrap()).unwrap();
            request
        });
        let mut client =
            cerulion_netd::NetdClient::connect_existing_at(socket).expect("the daemon greets");
        let answer = netd_harvest(&mut client, Some("temi")).expect("one answer");
        let request = daemon.join().expect("the daemon answers");
        assert_eq!(request["method"], "query_runs");
        assert_eq!(request["robot"], "temi");
        assert_eq!(answer.replies, [reply("temi", vec![temi_entry()])]);
        assert_eq!(answer.unusable[0].robot, "old");
        assert_eq!(answer.silent, ["quiet"]);
    }

    // ---- over the wire -----------------------------------------------------

    /// A robot gateway on loopback hosts a built graph's run and its fire
    /// tap; the desk side of these commands finds the robot by its announce,
    /// lists the run, and reads the fires the graph records while it asks.
    mod over_the_wire {
        use super::*;
        use crate::run_dir::{render_hosted_run_manifest, FireTelemetryDecl, HostedRunSpec};
        use cerulion_core::clock::VirtualClock;
        use cerulion_core::fire_tap::FireTapConfig;
        use cerulion_core::graph::node::{
            BackpressurePolicy, InputMeta, NodeContext, NodeEntry, OutputMeta,
        };
        use cerulion_core::graph::{parse_graph, GraphRuntime};
        use cerulion_core::transport::demand_authorizer::LanAccess;
        use cerulion_core::transport::gateway::{GatewayEgressPolicy, GatewayPlan, GatewayRuntime};
        use cerulion_core::transport::hosted_runs::HostedRun;
        use cerulion_core::transport::network::NetworkConfig;
        use cerulion_core::transport::run_registry::RunState;
        use cerulion_core::transport::{TransportConfig, TransportManager};
        use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo, TransportResult};
        use indexmap::IndexMap;
        use native_ros2_messages::geometry_msgs::Vector3;

        const ROBOT: &str = "wirebot";
        const RUN_ID: u128 = 0x00f8_0000_0000_0000_0000_0000_0000_0001;
        const GRAPH: &str = r#"
name: wire
prefix: fires
nodes:
  - id: source
    type: alternating_source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
      - {name: b, schema: geometry_msgs/Vector3}
  - id: relay
    type: relay
    inputs:
      - {name: a, source: source/a}
    outputs:
      - {name: out, schema: geometry_msgs/Vector3}
"#;

        fn publish(ctx: &mut NodeContext, output: &str, x: f64) -> TransportResult<()> {
            let mut proxy = ctx
                .publisher_mut(output)
                .expect("output is wired")
                .loan_proxy::<Vector3>()?;
            proxy.x = x;
            Ok(())
        }

        /// Tick n (from 1) publishes `a` when n is odd and `b` when n is even.
        fn alternating_source() -> ClosureNodeEntry {
            let mut n = 0_u64;
            let info = NodeInfo::from_names(vec![], vec!["a".to_string(), "b".to_string()])
                .with_policy(MacroPolicy::Period { period_ms: 1 });
            ClosureNodeEntry::new(info, move |ctx| {
                n += 1;
                publish(ctx, if n % 2 == 1 { "a" } else { "b" }, n as f64)
            })
        }

        /// Republishes each `a` frame on `out`.
        fn relay() -> ClosureNodeEntry {
            let info = NodeInfo::with_meta(
                vec![InputMeta {
                    name: "a".into(),
                    schema_hash: 0,
                    trigger: true,
                    depth: 4,
                    backpressure: BackpressurePolicy::DropOldest,
                    expect_within_ms: None,
                    consume: Default::default(),
                }],
                vec![OutputMeta::new("out".to_string(), 0, None)],
            )
            .with_policy(MacroPolicy::DataTrigger {
                input_name: "a".into(),
            });
            ClosureNodeEntry::new(info, move |ctx| {
                let x = ctx
                    .subscriber_mut("a")
                    .expect("input is wired")
                    .try_view::<Vector3, _>(|view| view.x)?;
                match x {
                    Some(x) => publish(ctx, "out", x),
                    None => Ok(()),
                }
            })
        }

        fn graph() -> GraphRuntime {
            let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
            factories.insert("source".to_string(), Box::new(alternating_source()));
            factories.insert("relay".to_string(), Box::new(relay()));
            match GraphRuntime::build_for_test(
                parse_graph(GRAPH).expect("graph parses"),
                factories,
                Arc::new(VirtualClock::new()),
                8,
            ) {
                Ok(runtime) => runtime,
                Err(error) => panic!("graph builds: {error}"),
            }
        }

        /// A gateway listening on a probed loopback port. Retried, because
        /// another process can take the probed port first.
        fn listening_gateway() -> (Arc<TransportManager>, GatewayRuntime, u16) {
            for attempt in 0..3 {
                let port = std::net::TcpListener::bind("127.0.0.1:0")
                    .and_then(|l| l.local_addr())
                    .expect("an ephemeral loopback port")
                    .port();
                let manager = TransportManager::init_for_test(
                    TransportConfig {
                        node_name: format!("graph_live_wire_{}_{attempt}", std::process::id()),
                        network: Some(NetworkConfig {
                            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                            multicast_scouting: false,
                            gossip_scouting: false,
                            robot_identity: Some(ROBOT.to_string()),
                            ..NetworkConfig::default()
                        }),
                        ..Default::default()
                    },
                    cerulion_core::testing::iceoryx_test_config(),
                )
                .expect("init gateway manager");
                let plan = GatewayPlan {
                    egress_policy: GatewayEgressPolicy::AllowAll,
                    announce: Vec::new(),
                    ingress: Vec::new(),
                };
                match GatewayRuntime::new(Arc::clone(&manager), plan) {
                    Ok(gateway) => return (manager, gateway, port),
                    Err(e) => {
                        eprintln!("attempt {attempt}: gateway boot failed on port {port}: {e}")
                    }
                }
            }
            panic!("could not boot a listening gateway in 3 attempts");
        }

        struct DeskReport {
            live: String,
            selected: SelectedRun,
            text: CliResult<(FireStream, FiresEnd)>,
            text_out: String,
            json: CliResult<(FireStream, FiresEnd)>,
            json_out: String,
        }

        /// The desk names a closed port before the robot's locator: the
        /// session skips the closed one and still reaches the robot.
        fn desk(locator: String) -> DeskReport {
            let mut plane = ZenohPlane::open(&[closed_locator(), locator])
                .expect("the desk session opens to the reachable locator");
            let running = AtomicBool::new(true);
            let harvest = plane.runs(None, &running).expect("the runs query runs");
            let live: String = harvest.replies.iter().map(render_live).collect();
            let selected = select_run(&harvest, None).expect("the hosted run serves fires");
            let mut text_out = Vec::new();
            let text = follow_fires(
                &mut plane,
                &selected,
                Some(Duration::from_secs(2)),
                false,
                &running,
                &mut WallPacer::start(),
                &mut text_out,
            );
            let mut json_out = Vec::new();
            let json = follow_fires(
                &mut plane,
                &selected,
                Some(Duration::from_secs(1)),
                true,
                &running,
                &mut WallPacer::start(),
                &mut json_out,
            );
            DeskReport {
                live,
                selected,
                text,
                text_out: String::from_utf8(text_out).unwrap(),
                json,
                json_out: String::from_utf8(json_out).unwrap(),
            }
        }

        #[test]
        fn the_desk_lists_a_hosted_run_and_reads_its_fires_over_loopback() {
            let mut runtime = graph();
            let tap = runtime.install_fire_tap(FireTapConfig {
                capacity: 4096,
                lease: Duration::from_secs(5),
            });
            let topology = runtime.run_topology();
            let run_json = String::from_utf8(render_hosted_run_manifest(&HostedRunSpec {
                run_id: RUN_ID,
                graph_name: "wire",
                run_started_at_ns: 0,
                pid: std::process::id(),
                lan_access: "open",
                endpoints: &[],
                topology: &topology,
                fires: Some(FireTelemetryDecl {
                    version: 1,
                    capacity: 4096,
                    lease_ms: 5000,
                }),
            }))
            .unwrap();
            let entry = RunEntry::new(RUN_ID, "wire", 0, RunState::Live, GRAPH, run_json)
                .expect("a well-formed entry");
            let (manager, _gateway, port) = listening_gateway();
            let network = manager.network().expect("a configured network");
            network.set_lan_access(LanAccess::Open);
            let _hosted = network.hosted_runs().host(
                RUN_ID,
                HostedRun {
                    entry,
                    live: Arc::new(AtomicBool::new(true)),
                    fires: Some(tap),
                },
            );

            let locator = format!("tcp/127.0.0.1:{port}");
            let desk = std::thread::spawn(move || desk(locator));
            while !desk.is_finished() {
                runtime.step(Duration::from_millis(1));
                std::thread::sleep(Duration::from_millis(2));
            }
            let report = desk.join().expect("the desk thread finishes");

            assert_eq!(
                report.live,
                "robot wirebot · run 0x00f80000000000000000000000000001 · graph wire · live · \
                 host embedded · started 1970-01-01 00:00:00 UTC\n\
                 \x20 source  alternating_source  every 1 ms          → /fires/source/a, /fires/source/b\n\
                 \x20 relay   relay               on /fires/source/a  → /fires/relay/out\n\
                 \x20 fire telemetry: available (v1, ring 4096, lease 5 s)\n"
            );
            assert_eq!(report.selected.robot, ROBOT);

            let (stream, end) = report.text.expect("the text reading ends cleanly");
            assert_eq!(end, FiresEnd::Window);
            assert_eq!(
                stream.lost, 0,
                "a 4096-slot ring read every 250 ms loses nothing"
            );
            let rows: Vec<&str> = report
                .text_out
                .lines()
                .filter(|line| line.contains(" ago  "))
                .collect();
            assert_eq!(rows.len() as u64, stream.fires);
            let relay_rows = rows
                .iter()
                .filter(|row| {
                    row.contains(" ago  relay   on /fires/source/a  ")
                        && row.ends_with("  wrote /fires/relay/out")
                })
                .count();
            let source_a = rows
                .iter()
                .filter(|row| {
                    row.contains(" ago  source  every 1 ms          ")
                        && row.ends_with("  wrote /fires/source/a")
                })
                .count();
            let source_b = rows
                .iter()
                .filter(|row| {
                    row.contains(" ago  source  every 1 ms          ")
                        && row.ends_with("  wrote /fires/source/b")
                })
                .count();
            assert!(
                relay_rows > 0 && source_a > 0 && source_b > 0,
                "relay {relay_rows}, source a {source_a}, source b {source_b}:\n{}",
                report.text_out
            );
            assert_eq!(
                relay_rows + source_a + source_b,
                rows.len(),
                "every row is a relay write or a source write:\n{}",
                report.text_out
            );
            assert!(
                report.text_out.lines().last().is_some_and(|line| line
                    .ends_with(" · 0 lost · the robot stops recording 5 s after the last ask")),
                "{}",
                report.text_out
            );

            let (stream, _) = report.json.expect("the JSON reading ends cleanly");
            let lines: Vec<serde_json::Value> = report
                .json_out
                .lines()
                .map(|line| serde_json::from_str(line).expect("one JSON object per line"))
                .collect();
            assert!(stream.fires > 0);
            assert_eq!(lines.len() as u64, stream.fires + 1);
            for (i, line) in lines.iter().enumerate() {
                assert_only_ids_names_topics_and_numbers(line, &format!("line {i}"));
            }
            assert_eq!(lines.last().unwrap()["kind"], "summary");
        }
    }
}
