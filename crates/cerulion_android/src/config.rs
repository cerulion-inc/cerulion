// SPDX-License-Identifier: AGPL-3.0-only
//! JSON configuration parsing and validation.
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use cerulion_core::transport::carrier::classify_locator;
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
    /// Network listen endpoints; with an empty `connect` as well, the
    /// transport stays local-only.
    #[serde(default)]
    pub listen: Vec<String>,
    /// `tcp/` locators of peers the app dials; an empty list dials none.
    #[serde(default)]
    pub connect: Vec<String>,
    /// Optional MCAP recording configuration.
    pub record: Option<RecordConfig>,
    /// App-created publishers.
    #[serde(default)]
    pub publish: Vec<PublishConfig>,
    /// App subscriptions, read with `poll`.
    #[serde(default)]
    pub subscribe: Vec<SubscribeConfig>,
    /// Who may reach the app's topics over the LAN; `paired` unless set.
    #[serde(default)]
    pub lan_access: LanAccessConfig,
    /// The only topics the gateway sends to a LAN peer; absent sends every
    /// topic once `lan_access` is `open`, an empty list sends none.
    #[serde(default, deserialize_with = "deserialize_egress")]
    pub egress: Option<Vec<String>>,
    /// Graph input topics a LAN peer may publish into once `lan_access` is
    /// `open`; each frame must carry the schema hash of the inputs that read
    /// the topic. Empty unless set.
    #[serde(default)]
    pub ingress: Vec<String>,
    /// Raise the `_cerulion._tcp` mDNS beacon while the gateway listens on TCP.
    #[serde(default = "default_mdns")]
    pub mdns: bool,
    /// Lowest level of the runtime's and the nodes' `tracing` lines that
    /// reach logcat; `info` unless set.
    #[serde(default)]
    pub log_level: LogLevelConfig,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
/// The `log_level` of the logcat output: lines below it are dropped.
///
/// A release build of the app library compiles out `debug` and `trace`
/// lines, so there [`Debug`](Self::Debug) and [`Trace`](Self::Trace) write
/// what [`Info`](Self::Info) writes.
pub enum LogLevelConfig {
    /// Errors only.
    Error,
    /// Warnings and errors.
    Warn,
    /// Lifecycle lines, warnings and errors.
    #[default]
    Info,
    /// Operational detail and everything above it.
    Debug,
    /// Every line, including the hot path.
    Trace,
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

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
/// Configuration for one app subscription.
pub struct SubscribeConfig {
    /// Canonical absolute topic name.
    pub topic: String,
    /// Qualified message schema name.
    pub schema: String,
    /// Messages the subscription queue holds before it drops the oldest;
    /// defaults to 10, the default depth of a node input.
    #[serde(default = "default_subscribe_depth")]
    pub depth: usize,
}

/// A present `egress` key must hold a list: `null` is refused, never read
/// as the absent key that sends every topic.
fn deserialize_egress<'de, D>(deserializer: D) -> Result<Option<Vec<String>>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    Vec::<String>::deserialize(deserializer).map(Some)
}

fn default_subscribe_depth() -> usize {
    cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH
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
        validate_topics(
            &self
                .subscribe
                .iter()
                .map(|item| item.topic.clone())
                .collect::<Vec<_>>(),
            "subscribe.topic",
        )?;
        let mut unique = BTreeSet::new();
        for subscribe in &self.subscribe {
            if !unique.insert(subscribe.topic.as_str()) {
                return Err(EmbedError::InvalidArgument(format!(
                    "duplicate subscribe topic '{}'",
                    subscribe.topic
                )));
            }
            // The upper bound is the transport's queue ceiling, checked
            // where the subscription is created.
            if subscribe.depth == 0 {
                return Err(EmbedError::InvalidArgument(format!(
                    "depth for '{}' must be at least 1",
                    subscribe.topic
                )));
            }
        }
        let mut unique = BTreeSet::new();
        for locator in &self.connect {
            validate_connect_locator(locator)?;
            if !unique.insert(locator.as_str()) {
                return Err(EmbedError::InvalidArgument(format!(
                    "duplicate connect locator '{locator}'"
                )));
            }
        }
        if let Some(egress) = &self.egress {
            validate_topics(egress, "egress")?;
            let mut unique = BTreeSet::new();
            for topic in egress {
                if !unique.insert(topic.as_str()) {
                    return Err(EmbedError::InvalidArgument(format!(
                        "duplicate egress topic '{topic}'"
                    )));
                }
            }
        }
        validate_topics(&self.ingress, "ingress")?;
        let mut unique = BTreeSet::new();
        for topic in &self.ingress {
            if !unique.insert(topic.as_str()) {
                return Err(EmbedError::InvalidArgument(format!(
                    "duplicate ingress topic '{topic}'"
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

/// A `connect` entry must be a well-formed `tcp/<host>:<port>` locator: a
/// `quic/` or `tls/` peer needs certificate files the config does not carry,
/// and a typo in the host or port would otherwise be dialed in the background
/// forever without a sign.
fn validate_connect_locator(locator: &str) -> Result<(), EmbedError> {
    let invalid =
        |reason: &str| EmbedError::InvalidArgument(format!("connect locator '{locator}' {reason}"));
    let Some((carrier, rest)) = locator.split_once('/') else {
        return Err(invalid("has no carrier: write it as 'tcp/<host>:<port>'"));
    };
    if carrier != "tcp" {
        return Err(invalid("is not a tcp/ locator; the app dials tcp/ only"));
    }
    classify_locator(locator)
        .map_err(|reason| EmbedError::InvalidArgument(format!("connect {reason}")))?;
    // The address ends where zenoh's metadata (`?`) or config (`#`) begins.
    let address = rest.split(['?', '#']).next().unwrap_or_default();
    let (host, port) = match address.strip_prefix('[') {
        Some(bracketed) => {
            let Some((host, tail)) = bracketed.split_once(']') else {
                return Err(invalid("has no port: write it as 'tcp/<host>:<port>'"));
            };
            let port = match tail.strip_prefix(':') {
                Some(port) => port,
                None if tail.is_empty() => {
                    return Err(invalid("has no port: write it as 'tcp/<host>:<port>'"))
                }
                None => {
                    return Err(invalid(&format!(
                        "has '{tail}' after ']'; write it as 'tcp/[<address>]:<port>'"
                    )))
                }
            };
            // The dial resolves a link-local scope by number only, never by
            // name.
            let (address, scope) = host.split_once('%').unwrap_or((host, "0"));
            if address.parse::<std::net::Ipv6Addr>().is_err() {
                return Err(invalid(&format!(
                    "has '{host}' in brackets, which is not an IPv6 address"
                )));
            }
            if scope.is_empty() || !scope.bytes().all(|byte| byte.is_ascii_digit()) {
                return Err(invalid(&format!(
                    "has IPv6 scope '{scope}'; the scope is the interface number"
                )));
            }
            (host, port)
        }
        None => {
            let Some((host, port)) = address.rsplit_once(':') else {
                return Err(invalid("has no port: write it as 'tcp/<host>:<port>'"));
            };
            if host.contains(':') {
                return Err(invalid(
                    "has a ':' in its host; an IPv6 host goes in brackets, as \
                     'tcp/[<address>]:<port>'",
                ));
            }
            if !host
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'-' | b'_'))
            {
                return Err(invalid(&format!(
                    "has host '{host}'; a host name holds only letters, digits, '.', '-' and '_'"
                )));
            }
            (host, port)
        }
    };
    if host.is_empty() {
        return Err(invalid("has no host: write it as 'tcp/<host>:<port>'"));
    }
    if !port.parse::<u16>().is_ok_and(|port| port != 0) {
        return Err(invalid(&format!(
            "has port '{port}'; a port is a number from 1 to 65535"
        )));
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
        if topic == "/__cerulion" || topic.starts_with("/__cerulion/") {
            return Err(EmbedError::InvalidArgument(format!(
                "{field} topic '{topic}' is under the reserved __cerulion/ prefix"
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

    /// A config that only names a LAN-facing listen endpoint must still get
    /// the paired posture: opening the listener never opens the topics.
    #[test]
    fn listen_only_config_defaults_to_paired() {
        let parsed = EmbeddedConfig::parse(r#"{"listen": ["tcp/0.0.0.0:7683"]}"#).unwrap();
        assert_eq!(parsed.listen, vec!["tcp/0.0.0.0:7683".to_string()]);
        assert_eq!(parsed.lan_access, LanAccessConfig::Paired);
        assert_eq!(LanAccess::from(parsed.lan_access), LanAccess::Paired);
        assert!(parsed.mdns);
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
    fn log_level_defaults_to_info_and_parses_every_level() {
        assert_eq!(
            EmbeddedConfig::parse("{}").unwrap().log_level,
            LogLevelConfig::Info
        );
        for (raw, level) in [
            ("error", LogLevelConfig::Error),
            ("warn", LogLevelConfig::Warn),
            ("info", LogLevelConfig::Info),
            ("debug", LogLevelConfig::Debug),
            ("trace", LogLevelConfig::Trace),
        ] {
            let json = serde_json::json!({ "log_level": raw }).to_string();
            assert_eq!(
                EmbeddedConfig::parse(&json).unwrap().log_level,
                level,
                "{raw}"
            );
        }
    }

    #[test]
    fn rejects_an_unknown_log_level_and_a_misspelled_key() {
        for raw in [
            r#"{"log_level":"Info"}"#,
            r#"{"log_level":"verbose"}"#,
            r#"{"log_level":"off"}"#,
            r#"{"log_level":""}"#,
            r#"{"log_level":3}"#,
            r#"{"loglevel":"info"}"#,
            r#"{"log_levels":"info"}"#,
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

    #[test]
    fn egress_is_absent_unless_set_and_an_empty_list_stays_empty() {
        assert_eq!(EmbeddedConfig::parse("{}").unwrap().egress, None);
        assert_eq!(
            EmbeddedConfig::parse(r#"{"egress":[]}"#).unwrap().egress,
            Some(Vec::new())
        );
        let parsed = EmbeddedConfig::parse(r#"{"egress":["/app/state","/app/pose"]}"#).unwrap();
        assert_eq!(
            parsed.egress,
            Some(vec!["/app/state".to_string(), "/app/pose".to_string()])
        );
    }

    /// An `egress` value that is not a list of strings is a start error,
    /// never the allow-all of an absent key.
    #[test]
    fn rejects_an_egress_value_that_is_not_a_list_of_topics() {
        for raw in [
            r#"{"egress":null}"#,
            r#"{"egress":"/app/state"}"#,
            r#"{"egress":[1]}"#,
            r#"{"egress":[null]}"#,
            r#"{"egress":{"topics":[]}}"#,
            r#"{"egress":true}"#,
            r#"{"egres":[]}"#,
        ] {
            let error = EmbeddedConfig::parse(raw).unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
        }
    }

    #[test]
    fn rejects_noncanonical_reserved_and_duplicate_egress_topics() {
        for (raw, message) in [
            (
                r#"{"egress":["relative"]}"#,
                "egress topic 'relative' must be canonical and start with '/'",
            ),
            (
                r#"{"egress":["/bad//topic"]}"#,
                "egress topic '/bad//topic' must be canonical and start with '/'",
            ),
            (
                r#"{"egress":["/__cerulion/x"]}"#,
                "egress topic '/__cerulion/x' is under the reserved __cerulion/ prefix",
            ),
            (
                r#"{"egress":["/same","/same"]}"#,
                "duplicate egress topic '/same'",
            ),
        ] {
            let error = EmbeddedConfig::parse(raw)
                .unwrap()
                .validate_paths(Path::new("/tmp"))
                .unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
            assert_eq!(error.to_string(), message, "{raw}");
        }
    }

    #[test]
    fn ingress_is_empty_unless_set_and_keeps_its_order() {
        assert!(EmbeddedConfig::parse("{}").unwrap().ingress.is_empty());
        let parsed = EmbeddedConfig::parse(r#"{"ingress":["/desk/cmd","/desk/goal"]}"#).unwrap();
        assert_eq!(
            parsed.ingress,
            vec!["/desk/cmd".to_string(), "/desk/goal".to_string()]
        );
        parsed.validate_paths(Path::new("/tmp")).unwrap();
    }

    /// An `ingress` value that is not a list of strings is a start error.
    #[test]
    fn rejects_an_ingress_value_that_is_not_a_list_of_topics() {
        for raw in [
            r#"{"ingress":null}"#,
            r#"{"ingress":"/desk/cmd"}"#,
            r#"{"ingress":[1]}"#,
            r#"{"ingress":[null]}"#,
            r#"{"ingress":{"topics":[]}}"#,
            r#"{"ingres":[]}"#,
        ] {
            let error = EmbeddedConfig::parse(raw).unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
        }
    }

    #[test]
    fn rejects_noncanonical_reserved_and_duplicate_ingress_topics() {
        for (raw, message) in [
            (
                r#"{"ingress":["relative"]}"#,
                "ingress topic 'relative' must be canonical and start with '/'",
            ),
            (
                r#"{"ingress":["/bad//topic"]}"#,
                "ingress topic '/bad//topic' must be canonical and start with '/'",
            ),
            (
                r#"{"ingress":["/__cerulion/x"]}"#,
                "ingress topic '/__cerulion/x' is under the reserved __cerulion/ prefix",
            ),
            (
                r#"{"ingress":["/same","/same"]}"#,
                "duplicate ingress topic '/same'",
            ),
        ] {
            let error = EmbeddedConfig::parse(raw)
                .unwrap()
                .validate_paths(Path::new("/tmp"))
                .unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
            assert_eq!(error.to_string(), message, "{raw}");
        }
    }

    #[test]
    fn connect_is_empty_unless_set_and_keeps_its_order() {
        assert!(EmbeddedConfig::parse("{}").unwrap().connect.is_empty());
        let parsed =
            EmbeddedConfig::parse(r#"{"connect":["tcp/127.0.0.1:7690","tcp/10.0.0.2:7683"]}"#)
                .unwrap();
        assert_eq!(
            parsed.connect,
            vec![
                "tcp/127.0.0.1:7690".to_string(),
                "tcp/10.0.0.2:7683".to_string()
            ]
        );
        parsed.validate_paths(Path::new("/tmp")).unwrap();
        // A host name, a bracketed IPv6 address and the end ports of the
        // range are well-formed too.
        for locator in [
            "tcp/desk.local:1",
            "tcp/[::1]:65535",
            "tcp/[fe80::1%2]:7690",
        ] {
            let json = serde_json::json!({ "connect": [locator] }).to_string();
            EmbeddedConfig::parse(&json)
                .unwrap()
                .validate_paths(Path::new("/tmp"))
                .unwrap_or_else(|error| panic!("{locator}: {error}"));
        }
    }

    #[test]
    fn rejects_a_connect_value_that_is_not_a_list_of_locators() {
        for raw in [
            r#"{"connect":null}"#,
            r#"{"connect":"tcp/127.0.0.1:7690"}"#,
            r#"{"connect":[7690]}"#,
            r#"{"connect":[null]}"#,
            r#"{"connects":[]}"#,
        ] {
            let error = EmbeddedConfig::parse(raw).unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
        }
    }

    /// The app dials over TCP only: a `quic/` or `tls/` peer needs
    /// certificate files the config does not carry, so such a locator, a
    /// malformed one or a duplicate is a start error before the process
    /// transport is built.
    #[test]
    fn rejects_malformed_non_tcp_and_duplicate_connect_locators() {
        for (raw, message) in [
            (
                r#"{"connect":["127.0.0.1:7690"]}"#,
                "connect locator '127.0.0.1:7690' has no carrier: write it as 'tcp/<host>:<port>'",
            ),
            (
                r#"{"connect":["tcp/"]}"#,
                "connect locator 'tcp/' has no address after 'tcp/'",
            ),
            (
                r#"{"connect":["quic/10.0.0.2:7683"]}"#,
                "connect locator 'quic/10.0.0.2:7683' is not a tcp/ locator; the app dials tcp/ only",
            ),
            (
                r#"{"connect":["tls/10.0.0.2:7683"]}"#,
                "connect locator 'tls/10.0.0.2:7683' is not a tcp/ locator; the app dials tcp/ only",
            ),
            (
                r#"{"connect":["udp/10.0.0.2:7683"]}"#,
                "connect locator 'udp/10.0.0.2:7683' is not a tcp/ locator; the app dials tcp/ only",
            ),
            (
                r#"{"connect":["tcp/10.0.0.2"]}"#,
                "connect locator 'tcp/10.0.0.2' has no port: write it as 'tcp/<host>:<port>'",
            ),
            (
                r#"{"connect":["tcp/:7690"]}"#,
                "connect locator 'tcp/:7690' has no host: write it as 'tcp/<host>:<port>'",
            ),
            (
                r#"{"connect":["tcp/desk.local:abc"]}"#,
                "connect locator 'tcp/desk.local:abc' has port 'abc'; a port is a number from 1 to 65535",
            ),
            (
                r#"{"connect":["tcp/desk.local:0"]}"#,
                "connect locator 'tcp/desk.local:0' has port '0'; a port is a number from 1 to 65535",
            ),
            (
                r#"{"connect":["tcp/desk.local:65536"]}"#,
                "connect locator 'tcp/desk.local:65536' has port '65536'; a port is a number from 1 to 65535",
            ),
            (
                r#"{"connect":["tcp/desk.local:7690:123"]}"#,
                "connect locator 'tcp/desk.local:7690:123' has a ':' in its host; an IPv6 host goes in brackets, as 'tcp/[<address>]:<port>'",
            ),
            (
                r#"{"connect":["tcp/::1:7690"]}"#,
                "connect locator 'tcp/::1:7690' has a ':' in its host; an IPv6 host goes in brackets, as 'tcp/[<address>]:<port>'",
            ),
            (
                r#"{"connect":["tcp/not a host:7690"]}"#,
                "connect locator 'tcp/not a host:7690' has host 'not a host'; a host name holds only letters, digits, '.', '-' and '_'",
            ),
            (
                r#"{"connect":["tcp/[desk.local]:7690"]}"#,
                "connect locator 'tcp/[desk.local]:7690' has 'desk.local' in brackets, which is not an IPv6 address",
            ),
            (
                r#"{"connect":["tcp/[::1]x:7690"]}"#,
                "connect locator 'tcp/[::1]x:7690' has 'x:7690' after ']'; write it as 'tcp/[<address>]:<port>'",
            ),
            (
                r#"{"connect":["tcp/[fe80::1%en0]:7690"]}"#,
                "connect locator 'tcp/[fe80::1%en0]:7690' has IPv6 scope 'en0'; the scope is the interface number",
            ),
            (
                r#"{"connect":["tcp/10.0.0.2:7683?mixed_rel=1"]}"#,
                "connect locator 'tcp/10.0.0.2:7683?mixed_rel=1' sets mixed_rel on a tcp/ carrier; mixed reliability exists only on quic/ and udp/...?rel=1",
            ),
            (
                r#"{"connect":["tcp/127.0.0.1:7690","tcp/127.0.0.1:7690"]}"#,
                "duplicate connect locator 'tcp/127.0.0.1:7690'",
            ),
        ] {
            let error = EmbeddedConfig::parse(raw)
                .unwrap()
                .validate_paths(Path::new("/tmp"))
                .unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
            assert_eq!(error.to_string(), message, "{raw}");
        }
    }

    #[test]
    fn subscribe_entries_parse_with_a_default_depth_of_10() {
        let parsed = EmbeddedConfig::parse(
            r#"{"subscribe":[{"topic":"/app/state","schema":"std_msgs/String"},{"topic":"/app/pose","schema":"geometry_msgs/Vector3","depth":3}]}"#,
        )
        .unwrap();
        assert_eq!(parsed.subscribe.len(), 2);
        assert_eq!(parsed.subscribe[0].topic, "/app/state");
        assert_eq!(parsed.subscribe[0].schema, "std_msgs/String");
        assert_eq!(parsed.subscribe[0].depth, 10);
        assert_eq!(parsed.subscribe[1].depth, 3);
        assert!(EmbeddedConfig::parse("{}").unwrap().subscribe.is_empty());
        parsed.validate_paths(Path::new("/tmp")).unwrap();
    }

    #[test]
    fn rejects_a_misspelled_subscribe_key() {
        for raw in [
            r#"{"subscribe":[{"topic":"/a","schema":"std_msgs/String","queue_depth":3}]}"#,
            r#"{"subscribe":[{"topic":"/a"}]}"#,
            r#"{"subscribe":[{"topic":"/a","schema":"std_msgs/String","depth":-1}]}"#,
            r#"{"subscriptions":[]}"#,
        ] {
            let error = EmbeddedConfig::parse(raw).unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
        }
    }

    #[test]
    fn rejects_noncanonical_duplicate_and_zero_depth_subscriptions() {
        for (raw, message) in [
            (
                r#"{"subscribe":[{"topic":"relative","schema":"std_msgs/String"}]}"#,
                "subscribe.topic topic 'relative' must be canonical and start with '/'",
            ),
            (
                r#"{"subscribe":[{"topic":"/__cerulion/x","schema":"std_msgs/String"}]}"#,
                "subscribe.topic topic '/__cerulion/x' is under the reserved __cerulion/ prefix",
            ),
            (
                r#"{"subscribe":[{"topic":"/same","schema":"std_msgs/String"},{"topic":"/same","schema":"std_msgs/String"}]}"#,
                "duplicate subscribe topic '/same'",
            ),
            (
                r#"{"subscribe":[{"topic":"/x","schema":"std_msgs/String","depth":0}]}"#,
                "depth for '/x' must be at least 1",
            ),
        ] {
            let error = EmbeddedConfig::parse(raw)
                .unwrap()
                .validate_paths(Path::new("/tmp"))
                .unwrap_err();
            assert_eq!(error.code(), crate::codes::INVALID_ARGUMENT, "{raw}");
            assert_eq!(error.to_string(), message, "{raw}");
        }
    }
}
