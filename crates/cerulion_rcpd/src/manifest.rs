// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp.yaml` manifest.
//!
//! ```yaml
//! robot: go2            # optional; defaults to the graph prefix, then the host name
//! graph: graphs/go2.yaml # optional; relative to the manifest's directory
//! journal:              # optional; these are the defaults
//!   calls_topic: /rcp/calls
//!   results_topic: /rcp/results
//! state:                # optional; each entry is served as robot://state/{name}
//!   - name: arm
//!     topic: /arm/state
//! tools:                # optional; only listed tools exist (estop always does)
//!   - name: arm.step
//!     description: Advance the arm one control period toward a joint target.
//!     safety_class: reversible
//!     timeout_ms: 2000
//!     input_schema: { type: object, properties: { ... }, required: [ ... ] }
//! ```
//!
//! Unknown keys are refused, so a typo never silently drops a setting.

use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::RcpdError;

/// The manifest as written.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RcpManifest {
    /// The robot identity served in the graph document.
    #[serde(default)]
    pub robot: Option<String>,
    /// The graph YAML whose nodes the graph document lists.
    #[serde(default)]
    pub graph: Option<PathBuf>,
    /// Where tool calls are journaled and answered.
    #[serde(default)]
    pub journal: JournalConfig,
    /// String topics served as `robot://state/{name}` resources.
    #[serde(default)]
    pub state: Vec<StateResource>,
    /// The tools an agent may call. Anything not listed does not exist.
    #[serde(default)]
    pub tools: Vec<ToolSpec>,
}

/// The topic pair a tool call travels on.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JournalConfig {
    /// Every tool call is published here as one `std_msgs/String` JSON frame.
    #[serde(default = "default_calls_topic")]
    pub calls_topic: String,
    /// The graph answers each call here, echoing its `call_id`.
    #[serde(default = "default_results_topic")]
    pub results_topic: String,
}

impl Default for JournalConfig {
    fn default() -> Self {
        Self {
            calls_topic: default_calls_topic(),
            results_topic: default_results_topic(),
        }
    }
}

fn default_calls_topic() -> String {
    String::from("/rcp/calls")
}

fn default_results_topic() -> String {
    String::from("/rcp/results")
}

/// One `robot://state/{name}` resource: the newest frame on a String topic.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StateResource {
    /// The `{name}` in the resource URI.
    pub name: String,
    /// The `std_msgs/String` topic whose newest frame is served.
    pub topic: String,
}

/// How much a tool can change the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SafetyClass {
    /// Reads only: no intent receipt, allowed while e-stopped.
    Read,
    /// Moves the robot in a way a later call can undo.
    Reversible,
    /// Needs a human's confirmation. Refused at load: confirmation is not
    /// implemented, and a tool that cannot be confirmed is not served.
    Irreversible,
}

impl SafetyClass {
    /// Whether calls write an intent receipt and are refused while e-stopped.
    pub fn is_mutating(self) -> bool {
        !matches!(self, Self::Read)
    }
}

/// One tool as the manifest declares it.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ToolSpec {
    /// The MCP tool name, and the `tool` field of the journaled call.
    pub name: String,
    /// What the tool does, shown to the agent.
    pub description: String,
    /// How much the tool can change the robot.
    pub safety_class: SafetyClass,
    /// How long to wait for the graph's answer, milliseconds.
    #[serde(default = "default_timeout_ms")]
    pub timeout_ms: u64,
    /// The arguments' JSON Schema, in the subset [`crate::schema`] checks.
    pub input_schema: serde_json::Value,
}

fn default_timeout_ms() -> u64 {
    2000
}

/// The manifest resolved against the filesystem, with its tool surface.
#[derive(Debug, Clone, PartialEq)]
pub struct LoadedManifest {
    /// What the graph document serves.
    pub identity: RobotIdentity,
    /// The call and result topics.
    pub journal: JournalConfig,
    /// The state resources.
    pub state: Vec<StateResource>,
    /// The declared tools, in manifest order.
    pub tools: Vec<ToolSpec>,
}

/// A node the manifest's graph declares.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeclaredNode {
    /// The node's `id` in the graph YAML.
    pub node_id: String,
    /// The node's `type` in the graph YAML (empty when the YAML omits it).
    pub node_type: String,
}

/// The manifest resolved against the filesystem: what the daemon serves.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RobotIdentity {
    /// The robot identity.
    pub robot: String,
    /// The declared nodes, in graph YAML order.
    pub nodes: Vec<DeclaredNode>,
}

impl RcpManifest {
    /// Parse manifest text.
    ///
    /// # Errors
    ///
    /// The parser's message when the text is not a valid manifest.
    pub fn parse(yaml: &str) -> Result<Self, String> {
        let parsed: Option<Self> = serde_yaml::from_str(yaml).map_err(|e| e.to_string())?;
        let manifest = parsed.unwrap_or_default();
        if manifest
            .robot
            .as_deref()
            .is_some_and(|r| r.trim().is_empty())
        {
            return Err(String::from("`robot` must not be blank"));
        }
        manifest.check_surface()?;
        Ok(manifest)
    }

    /// Read and parse the manifest at `path`, then resolve its graph.
    ///
    /// # Errors
    ///
    /// [`RcpdError::ManifestRead`] / [`RcpdError::ManifestParse`] for the
    /// manifest, [`RcpdError::GraphRead`] / [`RcpdError::GraphParse`] for the
    /// graph YAML it names.
    pub fn load(path: &Path) -> Result<RobotIdentity, RcpdError> {
        Self::load_full(path).map(|loaded| loaded.identity)
    }

    /// [`Self::load`], keeping the journal, state and tool sections.
    ///
    /// # Errors
    ///
    /// As [`Self::load`].
    pub fn load_full(path: &Path) -> Result<LoadedManifest, RcpdError> {
        let text = std::fs::read_to_string(path).map_err(|source| RcpdError::ManifestRead {
            path: path.to_path_buf(),
            source,
        })?;
        let manifest = Self::parse(&text).map_err(|reason| RcpdError::ManifestParse {
            path: path.to_path_buf(),
            reason,
        })?;
        let journal = manifest.journal.clone();
        let state = manifest.state.clone();
        let tools = manifest.tools.clone();
        let identity = manifest.resolve(path.parent().unwrap_or_else(|| Path::new("")))?;
        Ok(LoadedManifest {
            identity,
            journal,
            state,
            tools,
        })
    }

    /// Refuse a tool or state section the daemon could not serve as written.
    fn check_surface(&self) -> Result<(), String> {
        for topic in [&self.journal.calls_topic, &self.journal.results_topic] {
            if !topic.starts_with('/') {
                return Err(format!("journal topic {topic:?} must be absolute"));
            }
        }
        // An answer read from the calls topic would be the daemon's own call.
        if self.journal.calls_topic == self.journal.results_topic {
            return Err(String::from(
                "journal calls_topic and results_topic must differ",
            ));
        }
        let mut names = std::collections::BTreeSet::new();
        for tool in &self.tools {
            if !crate::tools::valid_tool_name(&tool.name) {
                return Err(format!(
                    "tool name {:?} must be 1 to 128 characters from [A-Za-z0-9_.-]",
                    tool.name
                ));
            }
            if tool.name == crate::tools::ESTOP_TOOL {
                return Err(format!(
                    "`{}` is built in and cannot be declared",
                    crate::tools::ESTOP_TOOL
                ));
            }
            if !names.insert(tool.name.as_str()) {
                return Err(format!("tool {:?} is declared twice", tool.name));
            }
            if tool.safety_class == SafetyClass::Irreversible {
                return Err(format!(
                    "tool {:?} is irreversible: human confirmation is not implemented, \
                     so irreversible tools are refused",
                    tool.name
                ));
            }
            if tool.timeout_ms == 0 || tool.timeout_ms > 60_000 {
                return Err(format!(
                    "tool {:?}: timeout_ms must lie in 1..=60000",
                    tool.name
                ));
            }
            crate::schema::check_schema(&tool.input_schema)
                .map_err(|e| format!("tool {:?}: input_schema: {e}", tool.name))?;
        }
        let mut state_names = std::collections::BTreeSet::new();
        for entry in &self.state {
            if !crate::tools::valid_tool_name(&entry.name) {
                return Err(format!(
                    "state name {:?} must be 1 to 128 characters from [A-Za-z0-9_.-]",
                    entry.name
                ));
            }
            if !entry.topic.starts_with('/') {
                return Err(format!("state topic {:?} must be absolute", entry.topic));
            }
            if entry.topic == self.journal.calls_topic || entry.topic == self.journal.results_topic
            {
                return Err(format!("state topic {:?} is a journal topic", entry.topic));
            }
            if !state_names.insert(entry.name.as_str()) {
                return Err(format!("state {:?} is declared twice", entry.name));
            }
        }
        Ok(())
    }

    /// Resolve the graph path against `base_dir` and build the identity.
    fn resolve(self, base_dir: &Path) -> Result<RobotIdentity, RcpdError> {
        let Some(graph) = self.graph else {
            let robot = self
                .robot
                .unwrap_or_else(|| cerulion_core::graph::default_prefix(""));
            return Ok(RobotIdentity {
                robot,
                nodes: Vec::new(),
            });
        };
        let path = base_dir.join(graph);
        let yaml = std::fs::read_to_string(&path).map_err(|source| RcpdError::GraphRead {
            path: path.clone(),
            source,
        })?;
        let config =
            cerulion_core::graph::parse_graph(&yaml).map_err(|e| RcpdError::GraphParse {
                path: path.clone(),
                reason: e.to_string(),
            })?;
        let nodes = config
            .nodes
            .into_iter()
            .map(|node| DeclaredNode {
                node_id: node.id,
                node_type: node.node_type,
            })
            .collect();
        Ok(RobotIdentity {
            robot: self.robot.unwrap_or(config.prefix),
            nodes,
        })
    }
}
