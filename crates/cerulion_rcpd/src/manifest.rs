// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp.yaml` manifest.
//!
//! ```yaml
//! robot: go2            # optional; defaults to the graph prefix, then the host name
//! graph: graphs/go2.yaml # optional; relative to the manifest's directory
//! ```
//!
//! Unknown keys are refused, so a typo never silently drops a setting.

use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::RcpdError;

/// The manifest as written.
#[derive(Debug, Clone, Default, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RcpManifest {
    /// The robot identity served in the graph document.
    #[serde(default)]
    pub robot: Option<String>,
    /// The graph YAML whose nodes the graph document lists.
    #[serde(default)]
    pub graph: Option<PathBuf>,
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
        let text = std::fs::read_to_string(path).map_err(|source| RcpdError::ManifestRead {
            path: path.to_path_buf(),
            source,
        })?;
        let manifest = Self::parse(&text).map_err(|reason| RcpdError::ManifestParse {
            path: path.to_path_buf(),
            reason,
        })?;
        manifest.resolve(path.parent().unwrap_or_else(|| Path::new("")))
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
