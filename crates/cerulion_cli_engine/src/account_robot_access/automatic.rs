// SPDX-License-Identifier: AGPL-3.0-only
//! Account and explicit pairing catalogs through the shared network daemon.

use std::time::{Duration, Instant};

use cerulion_netd::account_access::{
    AccountAccessReply, AccountAccessRequest, PairedRobot, RobotCatalog, RobotPath, RobotPresence,
    RobotTrustSource,
};
use cerulion_netd::NetdClient;

use crate::error::{CliError, CliResult};
use crate::{account_robots, auth, connect_cmd};

/// A complete directory with independently bounded metadata and visible failures.
#[derive(Default)]
pub struct RemoteCatalogs {
    /// Every robot, including unreachable and unattempted ones.
    pub robots: Vec<RobotCatalog>,
    /// Directory, daemon and revocation failures that must remain visible.
    pub diagnostics: Vec<String>,
    // Keeps the daemon alive between metadata lookup and a subsequent demand.
    _hold: Option<NetdClient>,
    _account: Option<account_robots::AccountRobotDirectory>,
    _paired: Vec<PairedRobot>,
}

impl RemoteCatalogs {
    /// Select an exact name or route, preserving a durable pairing alias when its
    /// endpoint is also registered under this account.
    pub fn select(&self, selector: &str) -> CliResult<Option<&RobotCatalog>> {
        if let Some(robot) = select(&self.robots, selector)? {
            return Ok(Some(robot));
        }
        let Some(pin) = self._paired.iter().find(|pin| pin.name == selector) else {
            return Ok(None);
        };
        let mut matching = self
            .robots
            .iter()
            .filter(|robot| robot.endpoint_key == Some(pin.endpoint_key));
        let first = matching.next();
        if matching.next().is_some() {
            return Err(CliError::Validation(
                "paired endpoint has multiple account registrations; select a full robot route"
                    .into(),
            ));
        }
        Ok(first)
    }

    /// Carry the selected owned robot's current revocation epoch before a read.
    /// Paired guest routes do not use the owner-only account endpoint.
    pub fn sync_selected_epoch(&self, route: &str) -> CliResult<()> {
        let Some(id) = cerulion_netd::account_access::parse_robot_route(route)
            .map_err(CliError::Validation)?
        else {
            return Ok(());
        };
        let directory = self._account.as_ref().ok_or_else(|| {
            CliError::Validation("revocation refresh has no current account directory".into())
        })?;
        let robot = directory
            .robots
            .iter()
            .find(|robot| robot.robot_id.0 == id)
            .ok_or_else(|| {
                CliError::Validation("revocation refresh robot is absent from the directory".into())
            })?;
        directory
            .sync_robot_epoch(robot, super::EPOCH_SYNC_BUDGET)
            .map(|_| ())
    }

    /// Whether account metadata came from the saved directory after a service failure.
    pub fn serves_cached_directory(&self) -> bool {
        self._account
            .as_ref()
            .is_some_and(|directory| directory.is_cached())
    }

    /// Keep cache-backed failures available to listings without warning on each read.
    pub fn log_diagnostics(&self) {
        for diagnostic in &self.diagnostics {
            if self.serves_cached_directory() {
                tracing::debug!(error = %diagnostic, "remote robot directory could not be fully checked");
            } else {
                tracing::warn!(error = %diagnostic, "remote robot directory could not be fully checked");
            }
        }
    }

    pub(super) fn take_hold(&mut self) -> Option<NetdClient> {
        self._hold.take()
    }
}

/// Fetch only when remote traffic is permitted; an unsigned desk with no pairings
/// opens no account request or daemon. Directory failures preserve paired access.
pub fn discover() -> CliResult<RemoteCatalogs> {
    if crate::graph_cmd::remote_network_suppressed() {
        return Ok(RemoteCatalogs::default());
    }
    let (pins, pairing_error) = match paired_robots() {
        Ok(pins) => (pins, None),
        Err(error) => (Vec::new(), Some(super::display_text(&error.to_string()))),
    };
    let signed_in = auth::load()
        .state()
        .is_some_and(|state| state.logged_in_ever);
    let mut result = RemoteCatalogs {
        _paired: pins.clone(),
        ..Default::default()
    };
    if let Some(error) = pairing_error {
        result.diagnostics.push(error);
    }
    if !signed_in && pins.is_empty() {
        return Ok(result);
    }
    let directory = if signed_in {
        // Finish single-use token rotation on this thread before the read-only
        // directory worker starts. A failed refresh falls back to the saved directory.
        let fetched =
            crate::login_cmd::refresh_session_if_stale_within(account_robots::LISTING_FETCH_BUDGET)
                .and_then(|_| account_robots::fetch_within(account_robots::LISTING_FETCH_BUDGET));
        match fetched {
            Ok(directory) => {
                if let Err(error) = directory.cache_for_offline() {
                    result
                        .diagnostics
                        .push(super::display_text(&error.to_string()));
                }
                Some(directory)
            }
            Err(error) => {
                result
                    .diagnostics
                    .push(super::display_text(&error.to_string()));
                match account_robots::cached_directory() {
                    Ok(cached) => cached,
                    Err(error) => {
                        result
                            .diagnostics
                            .push(super::display_text(&error.to_string()));
                        None
                    }
                }
            }
        }
    } else {
        None
    };
    if let Some(directory) = &directory {
        let epoch_rows: Vec<_> = directory
            .robots
            .iter()
            .cloned()
            .map(|robot| super::AccountRobotStatus {
                robot,
                presence: RobotPresence::Unknown {
                    reason: "metadata not checked".into(),
                },
            })
            .collect();
        if let Some(diagnostic) = super::sync_epochs(directory, &epoch_rows) {
            result.diagnostics.push(diagnostic);
        }
        for robot in &directory.robots {
            result.robots.push(RobotCatalog {
                route: cerulion_netd::account_access::robot_route(&robot.robot_id.0),
                name: robot.hostname.clone(),
                endpoint_key: match robot.endpoint {
                    account_robots::RobotEndpoint::Known(key) => Some(key.0),
                    account_robots::RobotEndpoint::Unknown { .. } => None,
                },
                trust: RobotTrustSource::Account,
                presence: RobotPresence::Unknown {
                    reason: "metadata not checked".into(),
                },
                path: RobotPath::Unknown,
                catalog: None,
            });
        }
    }
    for pin in &pins {
        if !result
            .robots
            .iter()
            .any(|robot| robot.endpoint_key == Some(pin.endpoint_key))
        {
            result.robots.push(RobotCatalog {
                route: pin.name.clone(),
                name: pin.name.clone(),
                endpoint_key: Some(pin.endpoint_key),
                trust: RobotTrustSource::Paired,
                presence: RobotPresence::Unknown {
                    reason: "metadata not checked".into(),
                },
                path: RobotPath::Unknown,
                catalog: None,
            });
        }
    }
    if directory.is_none() && pins.is_empty() {
        return Ok(result);
    }
    let attempt = (|| {
        if let Some(directory) = &directory {
            directory.validate_current_login()?;
        }
        let mut client = NetdClient::connect_or_spawn().map_err(super::access_error)?;
        // Installations and every robot share one deadline after bounded daemon startup.
        let deadline = Instant::now() + Duration::from_secs(4);
        if let Some(directory) = &directory {
            super::install_directory_until(directory, &mut client, deadline)?;
        }
        client
            .account_access_until(
                AccountAccessRequest::InstallPaired { robots: pins },
                deadline,
            )
            .map_err(super::access_error)?;
        let remaining = deadline.saturating_duration_since(Instant::now());
        let budget_ms = remaining.as_millis().saturating_sub(50).min(3900) as u64;
        if budget_ms == 0 {
            return Err(CliError::Validation(
                "robot discovery exceeded its shared time budget".into(),
            ));
        }
        let AccountAccessReply::Discovery { robots } = client
            .account_access_until(AccountAccessRequest::Discover { budget_ms }, deadline)
            .map_err(super::access_error)?
        else {
            return Err(CliError::Validation(
                "robot discovery response did not match its request".into(),
            ));
        };
        if let Some(directory) = &directory {
            directory.validate_current_login()?;
        }
        Ok((client, robots))
    })();
    match attempt {
        Ok((client, robots)) => {
            result.robots = robots;
            result._hold = Some(client);
        }
        Err(error) => result
            .diagnostics
            .push(super::display_text(&error.to_string())),
    }
    result
        .robots
        .sort_by(|a, b| (&a.name, &a.route).cmp(&(&b.name, &b.route)));
    result._account = directory;
    Ok(result)
}

/// Validate durable pairing pins and the existing machine key without login,
/// account requests or daemon startup. May reconcile the legacy key alias.
pub fn paired_robots() -> CliResult<Vec<PairedRobot>> {
    let Some(path) = connect_cmd::robots_toml_path() else {
        return Ok(Vec::new());
    };
    let mut options = std::fs::OpenOptions::new();
    options.read(true).custom_flags(libc::O_NONBLOCK);
    use std::os::unix::fs::OpenOptionsExt;
    let file = match options.open(path) {
        Ok(file) => file,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error.into()),
    };
    let metadata = file.metadata()?;
    if !metadata.is_file() || metadata.len() > 1024 * 1024 {
        return Err(CliError::Validation(
            "paired robot directory must be a bounded regular file".into(),
        ));
    }
    let mut text = String::new();
    use std::io::Read;
    file.take(1024 * 1024 + 1).read_to_string(&mut text)?;
    if text.len() > 1024 * 1024 {
        return Err(CliError::Validation(
            "paired robot directory exceeds its size limit".into(),
        ));
    }
    let robots = connect_cmd::parse_robots_toml(&text)
        .into_iter()
        .map(|(name, key)| {
            let endpoint_key = hex::decode(key)
                .ok()
                .and_then(|bytes| bytes.try_into().ok())
                .ok_or_else(|| {
                    CliError::Validation(
                        "paired robot directory contains an invalid endpoint key".into(),
                    )
                })?;
            Ok(PairedRobot { name, endpoint_key })
        })
        .collect::<CliResult<Vec<_>>>()?;
    AccountAccessRequest::InstallPaired {
        robots: robots.clone(),
    }
    .validate()
    .map_err(CliError::Validation)?;
    if !robots.is_empty() {
        let layout = auth::identity_layout()
            .ok_or_else(|| CliError::Validation("paired device identity is unavailable".into()))?;
        layout
            .load_machine_key()
            .map_err(|error| CliError::Validation(error.to_string()))?
            .ok_or_else(|| {
                CliError::Validation(
                    "paired device identity is missing; pair the robot again".into(),
                )
            })?;
    }
    Ok(robots)
}

/// Render directory robots without promoting unknown reachability to online.
pub fn render_robots(robots: &[RobotCatalog], all: bool) -> String {
    let mut out = String::new();
    for robot in robots {
        let presence = match &robot.presence {
            RobotPresence::Online => match robot.path {
                RobotPath::Direct => "online, direct".into(),
                RobotPath::Relayed => "online, relayed".into(),
                RobotPath::Unknown => "online, path unknown".into(),
            },
            RobotPresence::NotReached { reason } => {
                format!("offline (not reached: {})", super::display_text(reason))
            }
            RobotPresence::Unknown { reason } => {
                format!("unknown ({})", super::display_text(reason))
            }
        };
        out.push_str(&format!(
            "  {}  {}  {}",
            super::display_text(&robot.name),
            presence,
            super::display_text(&robot.route)
        ));
        if let Some(error) = robot
            .catalog
            .as_ref()
            .and_then(|catalog| catalog.error.as_ref())
        {
            out.push_str(&format!(
                "  topics unavailable: {}",
                super::display_text(error)
            ));
        }
        if all {
            out.push_str(match robot.trust {
                RobotTrustSource::Account => "  trust=account",
                RobotTrustSource::Paired => "  trust=paired",
            });
        }
        out.push('\n');
    }
    out
}

/// Select an exact robot name or stable route; duplicate names require a route.
pub fn select<'a>(
    robots: &'a [RobotCatalog],
    selector: &str,
) -> CliResult<Option<&'a RobotCatalog>> {
    if cerulion_netd::account_access::parse_robot_route(selector)
        .map_err(CliError::Validation)?
        .is_some()
    {
        return Ok(robots.iter().find(|robot| robot.route == selector));
    }
    let matching: Vec<_> = robots
        .iter()
        .filter(|robot| robot.name == selector || robot.route == selector)
        .collect();
    match matching.as_slice() {
        [] => Ok(None),
        [robot] => Ok(Some(*robot)),
        _ => Err(CliError::Validation(format!(
            "robot '{}' is ambiguous: {}; use --robot with a full robot route",
            super::display_text(selector),
            matching
                .iter()
                .map(|robot| robot.route.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        ))),
    }
}

/// Reject ambiguous names before decoding or demanding any topic.
/// `None` means no remote selection; a genuine local source remains local.
pub fn topic_candidates(
    topic: &str,
    local: bool,
    catalogs: &[cerulion_core::CatalogReply],
    selector: Option<&str>,
) -> CliResult<Option<String>> {
    let mut candidates = std::collections::BTreeSet::new();
    for catalog in catalogs {
        if catalog.error.is_none() && catalog.entries.iter().any(|entry| entry.topic == topic) {
            candidates.insert(catalog.robot.clone());
        }
    }
    if let Some(selector) = selector {
        return if candidates.contains(selector) {
            Ok(Some(selector.into()))
        } else if (local && selector == "local")
            || !catalogs.iter().any(|catalog| catalog.robot == selector)
        {
            Ok(None) // The selected LAN catalog may still resolve an unknown robot.
        } else {
            Err(CliError::Validation(format!(
                "topic '{}' is not available on robot '{}'",
                super::display_text(topic),
                super::display_text(selector)
            )))
        };
    }
    if candidates.len() + usize::from(local) > 1 {
        let mut labels: std::collections::BTreeSet<_> = candidates
            .iter()
            .map(|name| {
                if name == "local" && local {
                    "local (robot)".into()
                } else {
                    super::display_text(name)
                }
            })
            .collect();
        if local {
            labels.insert(if candidates.contains("local") {
                "local (this machine)".into()
            } else {
                "local".into()
            });
        }
        return Err(CliError::AmbiguousTopic(format!(
            "topic '{}' is ambiguous: {}; select one with --robot NAME",
            super::display_text(topic),
            labels.into_iter().collect::<Vec<_>>().join(", ")
        )));
    }
    Ok(candidates.into_iter().next())
}

/// Render every remote topic with its source robot, including identical topic names.
pub fn render_listing(
    robots: &[RobotCatalog],
    lan: &crate::topic_cmd::RemoteDiscovery,
    catalogs: &[cerulion_core::CatalogReply],
    streaming: &[crate::topic_cmd::MirrorStreamRow],
    all: bool,
    show_types: bool,
) -> String {
    let mut out = crate::topic_cmd::render_robots_section(&lan.robots, &lan.peers);
    if out.is_empty() {
        out.push_str("\nROBOTS\n");
    }
    out.push_str(&render_robots(robots, all));
    let mut rows = std::collections::BTreeMap::new();
    for catalog in catalogs
        .iter()
        .chain(robots.iter().filter_map(|robot| robot.catalog.as_ref()))
    {
        for entry in &catalog.entries {
            rows.insert(
                (entry.topic.clone(), catalog.robot.clone()),
                (entry.schema_name.clone(), false),
            );
        }
    }
    for (robot, topic) in &lan.announce_entries {
        if let Some(topic) = topic {
            rows.entry((topic.clone(), robot.clone()))
                .or_insert((None, false));
        }
    }
    for topic in &lan.topics {
        if !rows.keys().any(|(name, _)| name == topic) {
            rows.insert((topic.clone(), "origin unavailable".into()), (None, false));
        }
    }
    for stream in streaming {
        if !all && stream.topic.starts_with("/__cerulion/observers/") {
            continue;
        }
        let schema = catalogs
            .iter()
            .chain(robots.iter().filter_map(|robot| robot.catalog.as_ref()))
            .filter(|catalog| catalog.robot == stream.robot)
            .flat_map(|catalog| &catalog.entries)
            .find(|entry| {
                entry.topic == stream.topic
                    || cerulion_netd::TopicKey::observed(&stream.robot, &entry.topic).local_topic()
                        == stream.topic
            })
            .and_then(|entry| entry.schema_name.clone());
        let row = rows
            .entry((stream.topic.clone(), stream.robot.clone()))
            .or_insert((schema.clone(), false));
        if row.0.is_none() {
            row.0 = schema;
        }
        row.1 = true;
    }
    rows.retain(|(topic, _), _| all || !topic.starts_with("/__cerulion/observers/"));
    if rows.is_empty() {
        out.push_str("\nremote: no topics available; robot reachability is listed above\n");
        return out;
    }
    out.push_str("\nREMOTE TOPICS\n");
    for ((topic, route), (schema, streaming)) in rows {
        let robot = robots.iter().find(|robot| robot.route == route);
        let name = robot.map_or(route.as_str(), |robot| robot.name.as_str());
        out.push_str(&super::display_text(&topic));
        if crate::topic_cmd::is_internal_topic(&topic) {
            out.push_str("  [internal]");
        }
        if show_types {
            out.push_str(&format!(
                "  [{}]",
                super::display_text(schema.as_deref().unwrap_or("unknown"))
            ));
        }
        out.push_str(&format!("  robot={}", super::display_text(name)));
        if name != route {
            out.push_str(&format!("  {}", super::display_text(&route)));
        }
        if streaming {
            out.push_str("  ● streaming");
        }
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::transport::cerulion_q::{CatalogEntry, CatalogReply};

    fn catalog(robot: &str, topic: &str) -> CatalogReply {
        cerulion_core::transport::cerulion_q::build_catalog_reply(
            robot,
            [CatalogEntry {
                egress_qos: None,
                topic: topic.into(),
                schema_name: Some("std_msgs/String".into()),
                schema_hash: Some(1),
                provenance: cerulion_core::transport::cerulion_q::CatalogProvenance::Runtime,
                producer_count: None,
                liveness: None,
            }],
        )
    }

    #[test]
    fn owned_robot_coalescing_preserves_its_durable_pairing_alias() {
        let route = cerulion_netd::account_access::robot_route(&[1; 32]);
        let robot = RobotCatalog {
            route: route.clone(),
            name: "owned".into(),
            endpoint_key: Some([7; 32]),
            trust: RobotTrustSource::Account,
            presence: RobotPresence::Online,
            path: RobotPath::Direct,
            catalog: Some(catalog(&route, "/state")),
        };
        let mut directory = RemoteCatalogs {
            robots: vec![robot],
            _paired: vec![PairedRobot {
                name: "alias".into(),
                endpoint_key: [7; 32],
            }],
            ..Default::default()
        };
        assert_eq!(directory.select("alias").unwrap().unwrap().route, route);
        assert!(directory.select("missing").unwrap().is_none());
        let mut duplicate = directory.robots[0].clone();
        duplicate.route = cerulion_netd::account_access::robot_route(&[2; 32]);
        duplicate.name = route.clone();
        directory.robots.push(duplicate);
        assert_eq!(directory.select(&route).unwrap().unwrap().route, route);
        assert!(directory.select("account:malformed").is_err());
        assert!(directory.select("alias").is_err());
    }

    #[test]
    fn catalog_observer_channels_are_hidden_and_all_rows_keep_their_robot() {
        let catalogs = [
            catalog("a", "/state"),
            catalog("b", "/state"),
            catalog("b", "/__cerulion/observers/robot/topic"),
        ];
        let lan = crate::topic_cmd::RemoteDiscovery::empty();
        let normal = render_listing(&[], &lan, &catalogs, &[], false, true);
        assert!(!normal.contains("/__cerulion/"));
        assert!(normal.contains("/state  [std_msgs/String]  robot=a"));
        assert!(normal.contains("/state  [std_msgs/String]  robot=b"));
        let all = render_listing(&[], &lan, &catalogs, &[], true, true);
        assert!(all
            .contains("/__cerulion/observers/robot/topic  [internal]  [std_msgs/String]  robot=b"));
    }

    #[test]
    fn topic_selection_refuses_every_ambiguous_source_and_obeys_exact_selectors() {
        let catalogs = [catalog("b", "/state"), catalog("a", "/state")];
        assert_eq!(
            topic_candidates("/missing", false, &catalogs, None).unwrap(),
            None
        );
        assert_eq!(
            topic_candidates("/state", false, &catalogs[..1], None).unwrap(),
            Some("b".into())
        );
        assert_eq!(
            topic_candidates("/state", true, &catalogs, Some("local")).unwrap(),
            None
        );
        assert_eq!(
            topic_candidates("/state", true, &catalogs, Some("a")).unwrap(),
            Some("a".into())
        );
        assert_eq!(
            topic_candidates("/state", false, &catalogs, None)
                .unwrap_err()
                .to_string(),
            "topic '/state' is ambiguous: a, b; select one with --robot NAME"
        );
        assert!(topic_candidates("/state", true, &catalogs[..1], None)
            .unwrap_err()
            .to_string()
            .contains("b, local"));
        assert_eq!(
            topic_candidates("/state", false, &catalogs, Some("A")).unwrap(),
            None
        );
        assert!(
            topic_candidates("/state", true, &[catalog("local", "/state")], None)
                .unwrap_err()
                .to_string()
                .contains("local (robot), local (this machine)")
        );
        assert_eq!(
            topic_candidates(
                "/state",
                false,
                &[catalog("local", "/state")],
                Some("local")
            )
            .unwrap(),
            Some("local".into())
        );
        assert_eq!(topic_candidates("/state", true, &[], None).unwrap(), None);
        let reversed = [catalog("a", "/state"), catalog("b", "/state")];
        assert_eq!(
            topic_candidates("/state", false, &catalogs, None)
                .unwrap_err()
                .to_string(),
            topic_candidates("/state", false, &reversed, None)
                .unwrap_err()
                .to_string()
        );
    }

    #[test]
    fn robot_rows_preserve_reachability_and_hide_trust_unless_requested() {
        let robots = [
            RobotCatalog {
                route: "paired".into(),
                name: "paired".into(),
                endpoint_key: Some([7; 32]),
                trust: RobotTrustSource::Paired,
                presence: RobotPresence::Online,
                path: RobotPath::Direct,
                catalog: Some(catalog("paired", "/state")),
            },
            RobotCatalog {
                route: "other".into(),
                name: "other".into(),
                endpoint_key: Some([8; 32]),
                trust: RobotTrustSource::Account,
                presence: RobotPresence::NotReached {
                    reason: "deadline\nforged".into(),
                },
                path: RobotPath::Unknown,
                catalog: None,
            },
        ];
        assert_eq!(render_robots(&robots, false), "  paired  online, direct  paired\n  other  offline (not reached: deadline\u{fffd}forged)  other\n");
        assert!(render_robots(&robots, true).contains("trust=paired"));
        assert!(render_robots(&robots, true).contains("trust=account"));
        let listing = render_listing(
            &robots,
            &crate::topic_cmd::RemoteDiscovery::empty(),
            &[],
            &[],
            false,
            true,
        );
        assert_eq!(listing.matches("REMOTE TOPICS").count(), 1);
        assert!(listing.contains("/state  [std_msgs/String]  robot=paired\n"));
        assert!(!listing.contains("--connect"));
    }
}
