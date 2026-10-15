// SPDX-License-Identifier: AGPL-3.0-only
//! JSON configuration parsing and validation.
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use cerulion_core::LanAccess;
use serde::Deserialize;

use crate::error::EmbedError;

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration accepted by [`crate::EmbeddedRuntime::start`].
pub struct EmbeddedConfig {
    /// Optional graph YAML document.
    pub graph_yaml: Option<String>,
    /// Directory containing native node shared libraries.
    pub node_lib_dir: Option<PathBuf>,
    /// Network listen endpoints; an empty list keeps transport local-only.
    #[serde(default)]
    pub listen: Vec<String>,
    /// Optional MCAP recording configuration.
    pub record: Option<RecordConfig>,
    /// App-created publishers.
    #[serde(default)]
    pub publish: Vec<PublishConfig>,
    /// Who may reach the app's topics over the LAN; `paired` unless set.
    #[serde(default)]
    pub lan_access: LanAccessConfig,
    /// Raise the `_cerulion._tcp` mDNS beacon while the gateway listens on TCP.
    #[serde(default = "default_mdns")]
    pub mdns: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
/// The `lan_access` posture of the app gateway.
pub enum LanAccessConfig {
    /// Refuse every LAN peer: no topic, catalog or schema is served over the LAN.
    #[default]
    Paired,
    /// Serve any LAN peer, for local development on a trusted network.
    Open,
}

impl From<LanAccessConfig> for LanAccess {
    fn from(value: LanAccessConfig) -> Self {
        match value {
            LanAccessConfig::Paired => LanAccess::Paired,
            LanAccessConfig::Open => LanAccess::Open,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
/// Recording file and optional exact topic selection.
pub struct RecordConfig {
    /// Plain `.mcap` file name relative to the app files directory.
    pub file: String,
    /// Topics to record; omitted selects graph outputs and app publishers.
    pub topics: Option<Vec<String>>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for one app-created publisher.
pub struct PublishConfig {
    /// Canonical absolute topic name.
    pub topic: String,
    /// Qualified message schema name.
    pub schema: String,
    /// Maximum encoded body size; defaults to 4096 bytes.
    #[serde(default = "default_max_payload_bytes")]
    pub max_payload_bytes: usize,
}

fn default_mdns() -> bool {
    true
}

fn default_max_payload_bytes() -> usize {
    4096
}

impl EmbeddedConfig {
    /// Parse JSON while rejecting unknown fields.
    pub fn parse(json: &str) -> Result<Self, EmbedError> {
        serde_json::from_str(json)
            .map_err(|error| EmbedError::InvalidArgument(format!("invalid config JSON: {error}")))
    }

    /// Validate filesystem paths, topics, payload limits, and uniqueness.
    pub fn validate_paths(&self, files_dir: &Path) -> Result<(), EmbedError> {
        if !files_dir.is_absolute() || !files_dir.is_dir() {
            return Err(EmbedError::InvalidArgument(
                "files_dir must be an existing absolute directory".into(),
            ));
        }
        if self
            .node_lib_dir
            .as_ref()
            .is_some_and(|path| !path.is_absolute() || !path.is_dir())
        {
            return Err(EmbedError::InvalidArgument(
                "node_lib_dir must be an existing absolute directory".into(),
            ));
        }
        if let Some(record) = &self.record {
            if record.file.is_empty()
                || record.file.contains('/')
                || record.file.contains('\\')
                || !record.file.ends_with(".mcap")
                || Path::new(&record.file).file_name().and_then(|n| n.to_str())
                    != Some(record.file.as_str())
            {
                return Err(EmbedError::InvalidArgument(
                    "record.file must be a plain .mcap file name".into(),
                ));
            }
            if let Some(topics) = &record.topics {
                validate_topics(topics, "record.topics")?;
            }
        }
        validate_topics(
            &self
                .publish
                .iter()
                .map(|item| item.topic.clone())
                .collect::<Vec<_>>(),
            "publish.topic",
        )?;
        let mut unique = BTreeSet::new();
        for publish in &self.publish {
            if !unique.insert(publish.topic.as_str()) {
                return Err(EmbedError::InvalidArgument(format!(
                    "duplicate publish topic '{}'",
                    publish.topic
                )));
            }
            if !(1..=1_048_576).contains(&publish.max_payload_bytes) {
                return Err(EmbedError::InvalidArgument(format!(
                    "max_payload_bytes for '{}' must be 1..=1048576",
                    publish.topic
                )));
            }
        }
        Ok(())
    }
}

/// Validate the robot name used for the process transport node.
pub fn validate_robot_name(robot_name: &str) -> Result<(), EmbedError> {
    if !(1..=63).contains(&robot_name.len())
        || !robot_name
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-')
    {
        return Err(EmbedError::InvalidArgument(
            "robot name must be 1..=63 ASCII letters, digits, '_' or '-'".into(),
        ));
    }
    Ok(())
}

fn validate_topics(topics: &[String], field: &str) -> Result<(), EmbedError> {
    for topic in topics {
        if !topic.starts_with('/') || cerulion_core::graph::malformed_absolute_name(topic).is_some()
        {
            return Err(EmbedError::InvalidArgument(format!(
                "{field} topic '{topic}' must be canonical and start with '/'"
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_happy_config_parse() {
        let parsed = EmbeddedConfig::parse(
            r#"{"publish":[{"topic":"/app/heartbeat","schema":"std_msgs/String"}]}"#,
        )
        .unwrap();
        assert!(parsed.graph_yaml.is_none());
        assert!(parsed.listen.is_empty());
        assert_eq!(parsed.publish[0].max_payload_bytes, 4096);
    }

    #[test]
    fn lan_access_defaults_to_paired_and_mdns_to_on() {
        let parsed = EmbeddedConfig::parse("{}").unwrap();
        assert_eq!(parsed.lan_access, LanAccessConfig::Paired);
        assert_eq!(LanAccess::from(parsed.lan_access), LanAccess::Paired);
        assert!(parsed.mdns);
        let open = EmbeddedConfig::parse(r#"{"lan_access":"open","mdns":false}"#).unwrap();
        assert_eq!(LanAccess::from(open.lan_access), LanAccess::Open);
        assert!(!open.mdns);
        let paired = EmbeddedConfig::parse(r#"{"lan_access":"paired"}"#).unwrap();
        assert_eq!(paired.lan_access, LanAccessConfig::Paired);
    }

    #[test]
    fn rejects_an_unknown_lan_access_posture() {
        for raw in [
            r#"{"lan_access":"Open"}"#,
            r#"{"lan_access":"off"}"#,
            r#"{"lan_access":""}"#,
            r#"{"lan_access":true}"#,
            r#"{"mdns":"yes"}"#,
        ] {
            let error = EmbeddedConfig::parse(raw).unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
        }
    }

    #[test]
    fn rejects_unknown_config_fields() {
        assert!(EmbeddedConfig::parse(r#"{"unknown":true}"#).is_err());
    }

    #[test]
    fn validates_robot_name_and_record_name() {
        assert!(validate_robot_name("robot_1").is_ok());
        assert!(validate_robot_name("bad/name").is_err());
        assert!(validate_robot_name(&"a".repeat(64)).is_err());
        let parsed = EmbeddedConfig::parse(r#"{"record":{"file":"../bad.mcap"}}"#).unwrap();
        assert!(parsed.validate_paths(Path::new("/tmp")).is_err());
        assert!(EmbeddedConfig::parse(r#"{}"#)
            .unwrap()
            .validate_paths(Path::new("relative"))
            .is_err());
        assert!(EmbeddedConfig::parse(r#"{"node_lib_dir":"relative"}"#)
            .unwrap()
            .validate_paths(Path::new("/tmp"))
            .is_err());
    }

    #[test]
    fn validates_record_file_names_without_rejecting_innocuous_dot_pairs() {
        let config = EmbeddedConfig::parse(r#"{"record":{"file":"capture..backup.mcap"}}"#)
            .expect("parse recording config");
        config
            .validate_paths(Path::new("/tmp"))
            .expect("dot pairs inside a plain filename are allowed");

        // An extension-only name is still a plain file name.
        let config = EmbeddedConfig::parse(r#"{"record":{"file":".mcap"}}"#)
            .expect("parse extension-only recording config");
        config
            .validate_paths(Path::new("/tmp"))
            .expect("extension-only filename remains valid");

        for file in ["..", "../x.mcap", "a/b.mcap", r"a\b.mcap"] {
            let json = serde_json::json!({"record": {"file": file}}).to_string();
            let config = EmbeddedConfig::parse(&json).expect("parse recording config");
            assert!(
                config.validate_paths(Path::new("/tmp")).is_err(),
                "accepted invalid recording name {file:?}"
            );
        }
    }

    #[test]
    fn rejects_noncanonical_and_duplicate_publish_topics() {
        let config = EmbeddedConfig::parse(
            r#"{"publish":[{"topic":"relative","schema":"std_msgs/String"}]}"#,
        )
        .unwrap();
        assert!(config.validate_paths(Path::new("/tmp")).is_err());
        for topic in ["/", "/bad//topic", "/bad?topic"] {
            let config = EmbeddedConfig::parse(&format!(
                r#"{{"publish":[{{"topic":"{topic}","schema":"std_msgs/String"}}]}}"#
            ))
            .unwrap();
            assert!(config.validate_paths(Path::new("/tmp")).is_err(), "{topic}");
        }
        let config = EmbeddedConfig::parse(
            r#"{"publish":[{"topic":"/same","schema":"std_msgs/String"},{"topic":"/same","schema":"std_msgs/String"}]}"#,
        )
        .unwrap();
        assert!(config.validate_paths(Path::new("/tmp")).is_err());
    }

    #[test]
    fn rejects_payload_limit_out_of_range() {
        let config = EmbeddedConfig::parse(
            r#"{"publish":[{"topic":"/x","schema":"std_msgs/String","max_payload_bytes":1048577}]}"#,
        )
        .unwrap();
        assert!(config.validate_paths(Path::new("/tmp")).is_err());
    }
}
