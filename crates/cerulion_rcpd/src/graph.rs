// SPDX-License-Identifier: AGPL-3.0-only
//! The `robot://graph` document and its pure composer.
//!
//! The topic catalog reuses the LAN catalog vocabulary
//! ([`CatalogReply`] / [`CatalogEntry`]), so a client that already reads
//! `cerulion_q/{robot}/catalog` reads this one unchanged. Rates travel as
//! integral `millihertz` plus `is_floor`, never as a float.

use cerulion_core::monitor::MonitorRow;
use cerulion_core::transport::cerulion_q::{
    build_catalog_reply, format_run_id, CatalogEntry, CatalogProvenance, CatalogReply,
};
use cerulion_core::transport::liveness::TopicLiveness;
use cerulion_core::transport::mirror_registry::GatherCompleteness;
use cerulion_core::transport::run_registry::{RunGather, RunRecord, RunState};
use serde::{Deserialize, Serialize};

use crate::manifest::{DeclaredNode, RobotIdentity};

/// The `robot://graph` document version. Bumped on any breaking change.
pub const GRAPH_DOCUMENT_VERSION: u32 = 1;

/// The `robot://graph` document.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RobotGraph {
    /// [`GRAPH_DOCUMENT_VERSION`].
    pub version: u32,
    /// The robot identity from the manifest.
    pub robot: String,
    /// The transport clock reading this document was composed at.
    pub refreshed_at_ns: u64,
    /// The runs on this robot.
    pub runs: RunsView,
    /// The nodes the manifest's graph declares.
    pub nodes: Vec<NodeView>,
    /// The live topic catalog.
    pub catalog: CatalogReply,
    /// The topic monitor's rows (state, raised conditions, baseline and
    /// observed rates in millihertz).
    pub monitors: Vec<MonitorRow>,
}

/// The runs section.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunsView {
    /// Whether `records` settles the question; an incomplete gather is not an
    /// absence claim.
    pub completeness: RunsCompleteness,
    /// Every run heard, ordered by run id.
    pub records: Vec<RunView>,
}

/// What a runs gather is evidence of.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RunsCompleteness {
    /// Every live writer was heard: `records` is the whole answer.
    Settled,
    /// Some live writers were not heard before the window closed.
    Incomplete {
        /// Writers live on the registry service.
        live_writers: u64,
        /// Writers this gather heard from.
        writers_heard: u64,
    },
    /// The gather itself failed.
    Unavailable {
        /// The transport's explanation.
        error: String,
    },
}

/// One run.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunView {
    /// The run id, as `0x` plus 32 hex digits.
    pub run_id: String,
    /// The graph the run executes.
    pub graph_name: String,
    /// The run's lifecycle state.
    pub state: RunStateView,
    /// The pid owning the run.
    pub supervisor_pid: u32,
    /// Wall-clock ns since the Unix epoch at which the run started.
    pub run_started_at_ns: u64,
}

/// A run's lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunStateView {
    /// Executing.
    Live,
    /// Shutting down gracefully.
    Ending,
}

/// One declared node.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NodeView {
    /// The node's `id` in the graph YAML.
    pub node_id: String,
    /// The node's `type` in the graph YAML.
    pub node_type: String,
    /// The node's scheduler counters. Always `null` in this version: the
    /// runtime does not export per-node counters yet.
    pub counters: Option<NodeCounters>,
}

/// Per-node scheduler counters. Uninhabited until the runtime exports them,
/// so `counters` can only be `null`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NodeCounters {}

/// One live topic as the transport reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicObservation {
    /// The topic name.
    pub topic: String,
    /// Live publishers; `None` when the probe failed (unknown, not dead).
    pub producer_count: Option<u32>,
    /// The topic's liveness; `None` when not observed.
    pub liveness: Option<TopicLiveness>,
}

/// Everything one refresh read from the transport.
#[derive(Debug, Clone)]
pub struct GraphInputs {
    /// The transport clock reading.
    pub now_ns: u64,
    /// The runs gather, or the transport's error text.
    pub runs: Result<RunGather, String>,
    /// The live topics, or the transport's error text.
    pub topics: Result<Vec<TopicObservation>, String>,
    /// The monitor's rows.
    pub monitors: Vec<MonitorRow>,
}

/// Compose the document. Pure: equal inputs give byte-identical JSON.
pub fn compose(identity: &RobotIdentity, inputs: GraphInputs) -> RobotGraph {
    RobotGraph {
        version: GRAPH_DOCUMENT_VERSION,
        robot: identity.robot.clone(),
        refreshed_at_ns: inputs.now_ns,
        runs: runs_view(inputs.runs),
        nodes: identity.nodes.iter().map(node_view).collect(),
        catalog: catalog(&identity.robot, inputs.topics),
        monitors: inputs.monitors,
    }
}

fn runs_view(runs: Result<RunGather, String>) -> RunsView {
    match runs {
        Ok(gather) => {
            let mut records: Vec<RunView> = gather.records.iter().map(run_view).collect();
            records.sort_by(|a, b| a.run_id.cmp(&b.run_id));
            let completeness = match gather.completeness {
                GatherCompleteness::Settled => RunsCompleteness::Settled,
                GatherCompleteness::Incomplete {
                    live_writers,
                    writers_heard,
                } => RunsCompleteness::Incomplete {
                    live_writers: live_writers as u64,
                    writers_heard: writers_heard as u64,
                },
            };
            RunsView {
                completeness,
                records,
            }
        }
        Err(error) => RunsView {
            completeness: RunsCompleteness::Unavailable { error },
            records: Vec::new(),
        },
    }
}

fn run_view(record: &RunRecord) -> RunView {
    RunView {
        run_id: format_run_id(record.run_id),
        graph_name: record.graph_name.clone(),
        state: match record.state {
            RunState::Live => RunStateView::Live,
            RunState::Ending => RunStateView::Ending,
        },
        supervisor_pid: record.supervisor_pid,
        run_started_at_ns: record.run_started_at_ns,
    }
}

fn node_view(node: &DeclaredNode) -> NodeView {
    NodeView {
        node_id: node.node_id.clone(),
        node_type: node.node_type.clone(),
        counters: None,
    }
}

fn catalog(robot: &str, topics: Result<Vec<TopicObservation>, String>) -> CatalogReply {
    match topics {
        Ok(topics) => build_catalog_reply(
            robot,
            topics.into_iter().map(|t| CatalogEntry {
                topic: t.topic,
                schema_hash: None,
                schema_name: None,
                provenance: CatalogProvenance::Runtime,
                producer_count: t.producer_count,
                liveness: t.liveness,
            }),
        ),
        Err(error) => CatalogReply {
            error: Some(error),
            ..build_catalog_reply(robot, std::iter::empty())
        },
    }
}
