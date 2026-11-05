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
//! Every string a robot sends passes through the terminal sanitizer before
//! it is printed.

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::fire_tap::{FireCause, FireFlags, FireRecord};
use cerulion_core::transport::cerulion_q::{
    decode_fires_reply, parse_run_id, FiresOutcome, FiresReply, RunEntry, RunsCompleteness,
    RunsReply, FIRES_WIRE_VERSION, MAX_FIRES_PER_REPLY,
};
use cerulion_core::transport::discovery::RunsHarvest;
use serde::Deserialize;

use crate::error::{CliError, CliResult};
use crate::topic_cmd::sanitize_display;

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

fn clean(text: &str) -> String {
    sanitize_display(text)
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
            clean(reason)
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
            clean(&run.reason)
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
            out.push_str(&format!("  run.json does not read ({})\n", clean(&reason)));
            return out;
        }
    };
    out.push_str(&render_nodes(view.topology.as_ref(), &view.endpoints));
    out.push_str(&format!(
        "  fire telemetry: {}\n",
        telemetry_label(&view.telemetry)
    ));
    out
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
            clean(&answer.reason)
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
    let node = topology.nodes.get(usize::from(rec.node));
    let iw = width(topology.nodes.iter().map(|n| clean(&n.id).chars().count()));
    let cw = width(
        topology
            .nodes
            .iter()
            .map(|n| trigger_label(&n.trigger).chars().count()),
    );
    let id = node.map_or_else(|| format!("node {}", rec.node), |n| clean(&n.id));
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

fn run_object(robot: &str, entry: &RunEntry) -> serde_json::Value {
    let view = parse_run_json(&entry.run_json).unwrap_or_default();
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
    let endpoints: Vec<serde_json::Value> = view
        .endpoints
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
        .collect();
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
        "host": view.host,
        "nodes": nodes,
        "endpoints": endpoints,
        "fires": fires,
    })
}

fn robot_object(robot: &str, status: &str) -> serde_json::Value {
    serde_json::json!({"kind": "robot", "robot": robot, "status": status})
}

/// One `graph live --json` object per run, and one per robot with no
/// readable run. Only ids, names, topics and numbers.
pub fn live_json(harvest: &RunsHarvest) -> Vec<serde_json::Value> {
    let mut runs = Vec::new();
    let mut robots = Vec::new();
    for reply in &harvest.replies {
        if reply.error.is_some() {
            robots.push(robot_object(&reply.robot, "refused"));
            continue;
        }
        runs.extend(reply.runs.iter().map(|e| run_object(&reply.robot, e)));
        if reply.runs.is_empty() {
            let status = if reply.completeness.is_settled() {
                "no_runs"
            } else {
                "incomplete"
            };
            robots.push(robot_object(&reply.robot, status));
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
                clean(&reason)
            ))
        }
    };
    match &view.telemetry.fires {
        None => refuse(format!(
            "run {} serves no fire telemetry",
            describe_run(robot, entry)
        )),
        Some(decl) if decl.version != FIRES_WIRE_VERSION => refuse(format!(
            "run {} serves fire telemetry version {}, and this cerulion reads version \
             {FIRES_WIRE_VERSION}; update cerulion on this machine",
            describe_run(robot, entry),
            decl.version
        )),
        Some(_) => Ok(SelectedRun {
            robot: robot.to_string(),
            run_id: entry.run_id.clone(),
            graph_name: entry.graph_name.clone(),
            view,
        }),
    }
}

/// Choose the run `graph fires` reads. `run` is a run id or a graph name;
/// without it, the one run that serves fire telemetry.
pub fn select_run(harvest: &RunsHarvest, run: Option<&str>) -> CliResult<SelectedRun> {
    let refuse = |text: String| Err(CliError::Validation(text));
    let all: Vec<(&str, &RunEntry)> = harvest
        .replies
        .iter()
        .filter(|reply| reply.error.is_none())
        .flat_map(|reply| reply.runs.iter().map(move |e| (reply.robot.as_str(), e)))
        .collect();
    if all.is_empty() {
        if harvest.replies.is_empty() {
            return refuse(
                "no robot answered the runs query, so there is no run to read".to_string(),
            );
        }
        let robots: Vec<String> = harvest.replies.iter().map(|r| clean(&r.robot)).collect();
        return refuse(format!(
            "no run to read: no run is live on {}",
            robots.join(", ")
        ));
    }
    let Some(text) = run else {
        let capable: Vec<(&str, &RunEntry)> = all
            .iter()
            .copied()
            .filter(|(_, entry)| serves_fires(entry))
            .collect();
        return match capable.as_slice() {
            [] => refuse(format!(
                "no run serves fire telemetry; the runs are: {}",
                list_runs(&all)
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
            "no run matches --run {}; the runs are: {}",
            clean(text),
            list_runs(&all)
        )),
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
    /// Ask `robot`, or every robot that announces itself, for its runs.
    fn runs(&mut self, robot: Option<&str>) -> CliResult<RunsHarvest>;
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
        return writeln!(out, "{summary}");
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
    let mut stream = FireStream::default();
    let mut silent_since: Option<Duration> = None;
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
        if window.is_some_and(|w| pacer.elapsed() >= w) {
            break FiresEnd::Window;
        }
        let mut read_on = false;
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
                                writeln!(out, "{}", fire_json(run, now_ns, rec))?;
                            } else {
                                writeln!(out, "{}", render_fire_row(&topology, now_ns, rec))?;
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
                            clean(&reason)
                        ));
                    }
                }
            }
            FiresPoll::Silent => {
                let since = *silent_since.get_or_insert(pacer.elapsed());
                if pacer.elapsed().saturating_sub(since) >= FIRES_SILENCE_LIMIT {
                    break FiresEnd::Failed(format!(
                        "the robot stopped answering the fires query for {} s",
                        FIRES_SILENCE_LIMIT.as_secs()
                    ));
                }
            }
            FiresPoll::Unusable { reason } => {
                break FiresEnd::Failed(format!(
                    "the robot's fires answer cannot be read ({})",
                    clean(&reason)
                ));
            }
        }
        if !read_on {
            pacer.sleep(FIRES_POLL_INTERVAL);
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

/// A zenoh session of this command's own, connected to the given locators
/// only: no scouting, so it reaches exactly the robots named.
pub struct ZenohPlane {
    manager: cerulion_core::transport::network::NetworkManager,
}

impl ZenohPlane {
    /// Open the session. The connect phase is bounded.
    pub fn open(connect: &[String]) -> CliResult<Self> {
        use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
        let manager = NetworkManager::new(NetworkConfig {
            connect_endpoints: connect.to_vec(),
            listen_endpoints: Vec::new(),
            multicast_scouting: false,
            gossip_scouting: false,
            bounded_connect: true,
            ..NetworkConfig::default()
        });
        manager.session().map_err(|e| {
            CliError::Validation(format!(
                "could not open a session to {}: {e}; a locator has the form \
                 tcp/<host>:<port>, for example tcp/127.0.0.1:7683",
                connect.join(", ")
            ))
        })?;
        Ok(Self { manager })
    }
}

impl RunsSource for ZenohPlane {
    fn runs(&mut self, robot: Option<&str>) -> CliResult<RunsHarvest> {
        use cerulion_core::transport::discovery::{query_announce_entries, query_robot_runs_all};
        let session = self
            .manager
            .session()
            .map_err(|e| CliError::Validation(format!("the session closed: {e}")))?;
        let started = Instant::now();
        loop {
            let robots: Vec<String> = match robot {
                Some(robot) => vec![robot.to_string()],
                None => match query_announce_entries(session, ANNOUNCE_WINDOW) {
                    Ok(entries) => entries
                        .into_iter()
                        .map(|(robot, _)| robot)
                        .collect::<std::collections::BTreeSet<_>>()
                        .into_iter()
                        .collect(),
                    Err(e) => {
                        tracing::debug!(error = %e, "announce harvest failed; retrying");
                        Vec::new()
                    }
                },
            };
            let harvest = if robots.is_empty() {
                RunsHarvest::default()
            } else {
                query_robot_runs_all(session, &robots, RUNS_GET_WINDOW)
            };
            if !harvest.replies.is_empty()
                || harvest.is_terminal()
                || started.elapsed() >= FIRST_CONTACT_BUDGET
            {
                return Ok(harvest);
            }
            std::thread::sleep(FIRST_CONTACT_RETRY);
        }
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
    fn runs(&mut self, robot: Option<&str>) -> CliResult<RunsHarvest> {
        let mut client = cerulion_netd::NetdClient::connect_or_spawn().map_err(|e| {
            CliError::Validation(format!(
                "could not reach cerulion-netd to ask for runs: {e}; pass --connect LOCATOR \
                 to ask the robot directly"
            ))
        })?;
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

    #[cfg(not(unix))]
    fn runs(&mut self, _robot: Option<&str>) -> CliResult<RunsHarvest> {
        Err(CliError::Validation(
            "asking for runs without --connect needs cerulion-netd, which runs on Unix only; \
             pass --connect LOCATOR to ask the robot directly"
                .to_string(),
        ))
    }
}

// ---------------------------------------------------------------------------
// The commands.
// ---------------------------------------------------------------------------

fn runs_source(target: &RemoteTarget) -> CliResult<Box<dyn RunsSource>> {
    if target.connect.is_empty() {
        Ok(Box::new(NetdRuns))
    } else {
        Ok(Box::new(ZenohPlane::open(&target.connect)?))
    }
}

/// `cerulion graph live`.
pub fn graph_live(target: &RemoteTarget, json: bool) -> CliResult<()> {
    let harvest = runs_source(target)?.runs(target.robot.as_deref())?;
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    let unanswered = render_unanswered(&harvest);
    if json {
        for object in live_json(&harvest) {
            writeln!(out, "{object}")?;
        }
        write!(std::io::stderr(), "{unanswered}")?;
    } else {
        let blocks: Vec<String> = harvest.replies.iter().map(render_live).collect();
        write!(out, "{}{unanswered}", blocks.join("\n"))?;
    }
    out.flush()?;
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
    if target.connect.is_empty() {
        return Err(fires_needs_netd_protocol_10());
    }
    let mut plane = ZenohPlane::open(&target.connect)?;
    let harvest = plane.runs(target.robot.as_deref())?;
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
                "run_started_at_ns",
                "state"
            ]
        );
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
                "the robot refused the fires query (paired\u{FFFD}[2J)",
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

        fn desk(locator: String) -> DeskReport {
            let mut plane = ZenohPlane::open(&[locator]).expect("the desk session opens");
            let harvest = plane.runs(None).expect("the runs query runs");
            let live: String = harvest.replies.iter().map(render_live).collect();
            let selected = select_run(&harvest, None).expect("the hosted run serves fires");
            let running = AtomicBool::new(true);
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
