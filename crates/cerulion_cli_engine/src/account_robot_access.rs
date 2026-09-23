// SPDX-License-Identifier: AGPL-3.0-only
//! Account robot selection and bounded presence; the daemon owns every WAN socket.

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use cerulion_netd::account_access::{
    parse_robot_route, robot_route, AccountAccessReply, AccountAccessRequest, RobotPresence,
};
use cerulion_netd::NetdClient;

use crate::account_robots::{self, AccountRobot, AccountRobotDirectory, RobotEndpoint};
use crate::error::{CliError, CliResult};

// A directory can contain thousands of robots: per-robot timeouts alone would
// make listing take hours. Unattempted rows stay explicitly unknown.
const LIST_PROBE_BUDGET: Duration = Duration::from_secs(4);
const ROBOT_PROBE_BUDGET: Duration = Duration::from_millis(500);
// Leave the local daemon time to report a transport timeout within the IPC cap.
const PROBE_REPLY_ALLOWANCE: Duration = Duration::from_millis(50);

/// An account directory entry plus independently obtained reachability evidence.
#[derive(Debug, Clone)]
pub struct AccountRobotStatus {
    /// Public account-service identity; the hostname is only a display label.
    pub robot: AccountRobot,
    /// A TLS-key-confirmed observation, a bounded failure, or an explicit unknown.
    pub presence: RobotPresence,
}

/// Account-only evidence. LAN rows remain independent unless both IDs and keys match.
#[derive(Debug)]
pub struct AccountListing {
    /// Owned robots and their individually classified presence.
    pub rows: Vec<AccountRobotStatus>,
    /// One local account-access failure, kept separate from the LAN discovery result.
    pub diagnostic: Option<String>,
}

/// Select by exact case-sensitive name or canonical account ID. A shared name is
/// never sufficient evidence to merge two identities; explicit IDs disambiguate.
pub fn select_account_robot<'a>(
    robots: &'a [AccountRobot],
    selector: &str,
    lan_names: &BTreeSet<String>,
) -> CliResult<Option<&'a AccountRobot>> {
    if let Some(id) = parse_robot_route(selector).map_err(CliError::Validation)? {
        return robots
            .iter()
            .find(|robot| robot.robot_id.0 == id)
            .map(Some)
            .ok_or_else(|| {
                CliError::Validation(
                    "the selected account robot is not in the current account directory".into(),
                )
            });
    }
    let matching: Vec<_> = robots
        .iter()
        .filter(|robot| robot.hostname == selector)
        .collect();
    if matching.is_empty() {
        return Ok(None);
    }
    if matching.len() > 1 || lan_names.contains(selector) {
        let mut choices = matching
            .iter()
            .map(|robot| robot_route(&robot.robot_id.0))
            .collect::<Vec<_>>();
        choices.sort();
        let more = choices.len().saturating_sub(8);
        choices.truncate(8);
        let mut candidates = choices.join(", ");
        if more > 0 {
            candidates.push_str(&format!(" (and {more} more account robots)"));
        }
        if lan_names.contains(selector) {
            candidates.push_str("; a LAN robot has the same display name");
        }
        return Err(CliError::Validation(format!(
            "robot name '{}' is ambiguous: {candidates}; select an account robot by its full account ID",
            display_text(selector)
        )));
    }
    Ok(matching.first().copied())
}

/// Install only public identity and membership. Netd independently loads the key
/// and checks the snapshot against one locked local login transaction.
pub fn install_directory(directory: &AccountRobotDirectory) -> CliResult<NetdClient> {
    let snapshot = directory.daemon_snapshot()?;
    let mut client = NetdClient::connect_or_spawn().map_err(access_error)?;
    // Connecting can wait for startup; refuse stale HTTP results before IPC.
    directory.validate_current_login()?;
    client
        .account_access_once(AccountAccessRequest::Install {
            snapshot: Box::new(snapshot),
        })
        .map_err(access_error)?;
    Ok(client)
}

/// Fetch owned robots, then probe under one listing budget. Presence never pairs
/// or demands. An unavailable daemon leaves the fetched rows explicitly unknown.
pub fn list() -> CliResult<AccountListing> {
    let directory = account_robots::fetch()?;
    let mut rows = directory
        .robots
        .iter()
        .cloned()
        .map(|robot| {
            let reason = match &robot.endpoint {
                RobotEndpoint::Known(_) => "not checked within the listing budget",
                RobotEndpoint::Unknown { reason } => reason,
            };
            AccountRobotStatus {
                presence: RobotPresence::Unknown {
                    reason: reason.to_string(),
                },
                robot,
            }
        })
        .collect::<Vec<_>>();
    let mut client = match install_directory(&directory) {
        Ok(client) => client,
        Err(error) => {
            directory.validate_current_login()?;
            for row in &mut rows {
                if matches!(row.robot.endpoint, RobotEndpoint::Known(_)) {
                    row.presence = RobotPresence::Unknown {
                        reason: "account access could not be initialized".into(),
                    };
                }
            }
            return Ok(AccountListing {
                rows,
                diagnostic: Some(error.to_string()),
            });
        }
    };
    let deadline = Instant::now() + LIST_PROBE_BUDGET;
    let mut diagnostic = None;
    for row in &mut rows {
        if matches!(row.robot.endpoint, RobotEndpoint::Unknown { .. }) {
            continue;
        }
        let Some(budget) = probe_budget(deadline, Instant::now()) else {
            break;
        };
        let request = AccountAccessRequest::Probe {
            robot_id: row.robot.robot_id.0,
            budget_ms: budget
                .saturating_sub(PROBE_REPLY_ALLOWANCE)
                .as_millis()
                .max(1) as u64,
        };
        let attempt_deadline = (Instant::now() + budget).min(deadline);
        match client.account_access_until(request, attempt_deadline) {
            Ok(AccountAccessReply::Presence { presence, .. }) => row.presence = presence,
            Ok(_) => {
                diagnostic = Some("account presence response did not match its request".into());
                break;
            }
            Err(error) => {
                row.presence = RobotPresence::Unknown {
                    reason: "presence query could not be completed".into(),
                };
                diagnostic = Some(access_error(error).to_string());
                break;
            }
        }
    }
    directory.validate_current_login()?;
    Ok(AccountListing { rows, diagnostic })
}

fn probe_budget(deadline: Instant, now: Instant) -> Option<Duration> {
    let remaining = deadline.checked_duration_since(now)?;
    let millis = remaining.min(ROBOT_PROBE_BUDGET).as_millis();
    (millis > 0).then(|| Duration::from_millis(millis as u64))
}

/// Render account rows inside ROBOTS without changing or merging the LAN rows.
pub fn render_rows(rows: &[AccountRobotStatus]) -> String {
    let mut out = String::new();
    for row in rows {
        let presence = match &row.presence {
            RobotPresence::Online => "online".to_owned(),
            RobotPresence::NotReached { reason } => {
                format!("not reached: {}", display_text(reason))
            }
            RobotPresence::Unknown { reason } => format!("unknown: {}", display_text(reason)),
        };
        let endpoint = match row.robot.endpoint {
            RobotEndpoint::Known(key) => hex::encode(key.0),
            RobotEndpoint::Unknown { .. } => "unknown".into(),
        };
        out.push_str(&format!(
            "  {}  (account directory)  {}  {}  endpoint={}\n",
            display_text(&row.robot.hostname),
            presence,
            robot_route(&row.robot.robot_id.0),
            endpoint
        ));
    }
    out
}

fn access_error(error: impl std::fmt::Display) -> CliError {
    CliError::Validation(format!(
        "account robot access unavailable: {}",
        display_text(&error.to_string())
    ))
}

fn display_text(text: &str) -> String {
    text.chars()
        .take(1024)
        .map(|ch| if ch.is_control() { '\u{fffd}' } else { ch })
        .collect()
}

/// A resolved viewer target and diagnostics for independently unavailable sources.
pub struct VizRobotTarget {
    /// Internal stable route or the original LAN selector.
    pub route: String,
    /// Visible diagnostics; these never turn a selected account route into LAN.
    pub diagnostics: Vec<String>,
    // Prevent idle daemon exit while the viewer is starting its first demand.
    _daemon_hold: Option<NetdClient>,
}

/// Resolve an existing `viz --robot` selector before starting the viewer. Account
/// lookup failure preserves the old LAN path with a diagnostic; an explicit account
/// route and any already-selected account target always fail closed.
pub fn prepare_viz_target(
    selector: &str,
    connect: &[String],
    listen: &[String],
) -> CliResult<VizRobotTarget> {
    let explicit = parse_robot_route(selector)
        .map_err(CliError::Validation)?
        .is_some();
    let directory = match account_robots::fetch() {
        Ok(directory) => directory,
        Err(error) if explicit => return Err(error),
        Err(error) => {
            return Ok(VizRobotTarget {
                route: selector.into(),
                diagnostics: vec![display_text(&error.to_string())],
                _daemon_hold: None,
            })
        }
    };
    let mut lan_names = BTreeSet::new();
    let mut diagnostics = Vec::new();
    if !explicit
        && directory
            .robots
            .iter()
            .any(|robot| robot.hostname == selector)
    {
        let options = crate::topic_cmd::RemoteTopicsOptions {
            connect: connect.to_vec(),
            listen: listen.to_vec(),
            scouting: true,
        };
        match crate::topic_cmd::query_remote_topics(&options) {
            Ok(discovery) => {
                lan_names.extend(discovery.robots.into_iter().map(|robot| robot.robot))
            }
            Err(error) => diagnostics.push(format!(
                "LAN discovery unavailable while checking robot names: {}",
                display_text(&error.to_string())
            )),
        }
    }
    let selected = select_account_robot(&directory.robots, selector, &lan_names)?;
    directory.validate_current_login()?;
    let mut daemon_hold = None;
    let route = match selected {
        Some(robot) => {
            let route = robot_route(&robot.robot_id.0);
            daemon_hold = Some(install_directory(&directory)?);
            route
        }
        None => selector.into(),
    };
    Ok(VizRobotTarget {
        route,
        diagnostics,
        _daemon_hold: daemon_hold,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::{PublicKey, RobotId};

    fn robot(id: u8, name: &str) -> AccountRobot {
        AccountRobot {
            robot_id: RobotId([id; 32]),
            hostname: name.into(),
            endpoint: RobotEndpoint::Known(PublicKey([7; 32])),
        }
    }

    #[test]
    fn exact_selection_refuses_duplicate_and_lan_collisions_but_ids_disambiguate() {
        let robots = vec![robot(1, "robot"), robot(2, "robot"), robot(3, "Robot")];
        let none = BTreeSet::new();
        assert!(select_account_robot(&robots, "robot", &none)
            .unwrap_err()
            .to_string()
            .contains("ambiguous"));
        assert_eq!(
            select_account_robot(&robots, "Robot", &none)
                .unwrap()
                .unwrap()
                .robot_id
                .0,
            [3; 32]
        );
        assert!(select_account_robot(&robots, "ROBOT", &none)
            .unwrap()
            .is_none());
        assert!(select_account_robot(&robots, "rob", &none)
            .unwrap()
            .is_none());
        let lan = BTreeSet::from(["Robot".into()]);
        let error = select_account_robot(&robots, "Robot", &lan)
            .unwrap_err()
            .to_string();
        assert!(error.contains("a LAN robot has the same display name"));
        let explicit = robot_route(&[3; 32]);
        assert_eq!(
            select_account_robot(&robots, &explicit, &lan)
                .unwrap()
                .unwrap()
                .robot_id
                .0,
            [3; 32]
        );
        for invalid in [
            "account:bad".to_string(),
            format!(" {explicit} "),
            robot_route(&[9; 32]),
        ] {
            assert!(select_account_robot(&robots, &invalid, &lan).is_err());
        }
    }

    #[test]
    fn listing_probe_budget_is_shared_and_never_rounds_up_past_the_deadline() {
        let now = Instant::now();
        assert_eq!(
            probe_budget(now + LIST_PROBE_BUDGET, now),
            Some(Duration::from_millis(500))
        );
        assert_eq!(
            probe_budget(now + Duration::from_millis(73), now),
            Some(Duration::from_millis(73))
        );
        assert_eq!(probe_budget(now + Duration::from_micros(999), now), None);
        assert_eq!(probe_budget(now, now), None);
        assert_eq!(probe_budget(now, now + Duration::from_secs(1)), None);
    }

    #[test]
    fn account_rows_label_provenance_and_do_not_promote_unknown_to_online() {
        let mut identity = robot(1, "robot");
        identity.endpoint = RobotEndpoint::Unknown {
            reason: "no endpoint key",
        };
        let rows = [AccountRobotStatus {
            robot: identity,
            presence: RobotPresence::Unknown {
                reason: "not checked\nforged".into(),
            },
        }];
        let rendered = render_rows(&rows);
        assert_eq!(rendered, "  robot  (account directory)  unknown: not checked\u{fffd}forged  account:0101010101010101010101010101010101010101010101010101010101010101  endpoint=unknown\n");
        assert!(!rendered.contains("online"));
    }
}
