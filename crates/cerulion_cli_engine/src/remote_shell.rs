// SPDX-License-Identifier: AGPL-3.0-only
//! Paired remote-session planning, account selection, and helper invocation.

use std::path::Path;
use std::sync::atomic::AtomicBool;
#[cfg(unix)]
use std::sync::atomic::Ordering;
use std::sync::Arc;

use crate::connect_cmd::ConnectPlan;
#[cfg(unix)]
use crate::connect_cmd::{self, ConnectArgs};
use crate::error::{CliError, CliResult};
use crate::graph_cmd::{query_network_kill, query_network_kill_from, QueryNetworkKill};
#[cfg(unix)]
use crate::{account_robot_access, account_robots, auth, peer_cache};

/// Exit code when the network kill switch refuses a remote shell operation.
pub const EXIT_NETWORK_REFUSED: u8 = crate::message_text::EXIT_USAGE;

/// The remote operation selected by the user.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Operation {
    /// An interactive login shell.
    Shell,
    /// Exact command arguments, without local shell interpretation.
    Exec(Vec<String>),
    /// Inspect or change the owner switch.
    Policy {
        /// Change the owner switch, or leave it unchanged.
        enabled: Option<bool>,
    },
}

/// Turn a paired [`crate::connect_cmd::plan`] result into a session plan.
/// This never creates an identity, signs in, or pairs a robot. The connection
/// options stay intact; command arguments remain literal after `--`.
pub fn session_plan(
    mut connection: ConnectPlan,
    operation: &Operation,
    json: bool,
    witness_dir: &Path,
) -> CliResult<ConnectPlan> {
    if connection.argv.first().map(String::as_str) != Some("connect") {
        return Err(CliError::Validation(
            "remote session requires a connect plan".into(),
        ));
    }
    let key = connection
        .argv
        .iter()
        .position(|arg| arg == "--key-file")
        .and_then(|index| connection.argv.get(index + 1));
    if key.is_none_or(|value| value.is_empty() || value.starts_with("--") || value.contains('\0')) {
        return Err(CliError::Validation(
            "remote shell requires this machine's paired identity; sign in and pair the robot first"
                .into(),
        ));
    }
    let command = match operation {
        Operation::Shell => "remote-shell",
        Operation::Exec(argv) => {
            if argv.is_empty() || argv[0].is_empty() || argv.iter().any(|arg| arg.contains('\0')) {
                return Err(CliError::Validation(
                    "exec requires a nonempty command after --; arguments cannot contain NUL"
                        .into(),
                ));
            }
            "remote-exec"
        }
        Operation::Policy { .. } => "shell-access",
    };
    connection.argv[0] = command.into();
    if json {
        connection.argv.push("--json".into());
    }
    match operation {
        Operation::Policy { enabled } => {
            if let Some(value) = enabled {
                connection
                    .argv
                    .extend(["--enabled".into(), if *value { "on" } else { "off" }.into()]);
            }
        }
        Operation::Shell | Operation::Exec(_) => {
            let witness = witness_dir
                .to_str()
                .filter(|value| !value.is_empty() && !value.contains('\0'))
                .ok_or_else(|| {
                    CliError::Validation(
                        "the witness directory must be a nonempty UTF-8 path without NUL".into(),
                    )
                })?;
            connection
                .argv
                .extend(["--witness-dir".into(), witness.to_owned()]);
            if let Operation::Exec(argv) = operation {
                connection.argv.push("--".into());
                connection.argv.extend(argv.iter().cloned());
            }
        }
    }
    Ok(connection)
}

/// Refuse disabled networking before login, robot lookup, or helper resolution.
/// Call this again at the engine boundary before resolving or spawning a helper.
pub fn require_network(operation: &Operation) -> CliResult<()> {
    require_network_with(operation, query_network_kill())
}

/// The pure refusal for one invocation's `CERULION_NETWORK` and
/// `CERULION_NETD_NETWORK` snapshot, using the query-side parse. Dispatch
/// prints this before the login gate; [`require_network`] uses the same text.
pub fn network_refusal(
    operation: &Operation,
    network: Option<String>,
    netd: Option<&str>,
) -> Option<String> {
    network_refusal_with(operation, query_network_kill_from(network, netd))
}

fn require_network_with(operation: &Operation, kill: QueryNetworkKill) -> CliResult<()> {
    match network_refusal_with(operation, kill) {
        Some(reason) => Err(CliError::RemoteShellNetworkRefused(reason)),
        None => Ok(()),
    }
}

fn network_refusal_with(operation: &Operation, kill: QueryNetworkKill) -> Option<String> {
    if !kill.is_off() {
        return None;
    }
    let (verb, action) = match operation {
        Operation::Shell => ("shell", "open a shell on the robot"),
        Operation::Exec(_) => ("exec", "run the command on the robot"),
        Operation::Policy { .. } => ("shell-access", "inspect or change the robot's shell access"),
    };
    let mut reason = kill.network.refusal_label().unwrap_or_default();
    if kill.netd_off {
        if !reason.is_empty() {
            reason.push_str(" and ");
        }
        reason.push_str("CERULION_NETD_NETWORK=off");
    }
    Some(format!(
        "cerulion {verb} reaches the robot over the network, which {reason} disables; {} to {action}",
        kill.unset_remedy()
    ))
}

#[cfg(unix)]
struct AccountSession {
    directory: account_robots::AccountRobotDirectory,
    robot: account_robots::AccountRobot,
}

#[cfg(unix)]
struct PreparedSession {
    plan: ConnectPlan,
    account: Option<AccountSession>,
}

#[cfg(unix)]
impl PreparedSession {
    fn spawn(self, running: Arc<AtomicBool>) -> CliResult<i32> {
        // Cancel before epoch HTTP as well as before spawning the helper.
        if !running.load(Ordering::SeqCst) {
            return Ok(130);
        }
        if let Some(account) = &self.account {
            account.directory.validate_current_login()?;
            if let Err(error) = account
                .directory
                .sync_robot_epoch(&account.robot, account_robot_access::EPOCH_SYNC_BUDGET)
            {
                tracing::warn!(
                    robot = %cerulion_netd::account_access::robot_route(&account.robot.robot_id.0),
                    error = %crate::topic_cmd::sanitize_refusal_reason(&error.to_string()),
                    "remote shell revocation refresh did not complete"
                );
            }
            // HTTP uses the saved session; re-read identity after every result.
            account.directory.validate_current_login()?;
        }
        if !running.load(Ordering::SeqCst) {
            return Ok(130);
        }
        connect_cmd::spawn_and_wait(&self.plan, running)
    }
}

#[cfg(unix)]
#[derive(Debug)]
struct SelectedAccountRobot {
    robot: account_robots::AccountRobot,
    eid: String,
}

#[cfg(unix)]
fn set_account_epoch_route(
    connection: &mut ConnectPlan,
    robot: &account_robots::AccountRobot,
) -> CliResult<()> {
    let route = cerulion_netd::account_access::robot_route(&robot.robot_id.0);
    if let Some(index) = connection.argv.iter().position(|arg| arg == "--robot-name") {
        let value = connection.argv.get_mut(index + 1).ok_or_else(|| {
            CliError::Validation("the connection plan has no value for its robot name".into())
        })?;
        *value = route;
    } else {
        connection.argv.extend(["--robot-name".into(), route]);
    }
    Ok(())
}

#[cfg(unix)]
fn local_selector(robot: &str) -> CliResult<bool> {
    if connect_cmd::is_hex_eid(robot) {
        return Ok(true);
    }
    if robot.starts_with("account:") {
        return Ok(false);
    }
    let Some(path) = connect_cmd::robots_toml_path() else {
        return Ok(false);
    };
    match std::fs::read_to_string(path) {
        Ok(text) => Ok(connect_cmd::parse_robots_toml(&text).contains_key(robot)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(false),
        Err(error) => Err(error.into()),
    }
}

#[cfg(unix)]
fn account_endpoint(
    robots: &[account_robots::AccountRobot],
    robot: &str,
    cached_names: &std::collections::BTreeSet<String>,
) -> CliResult<SelectedAccountRobot> {
    let selected = account_robot_access::select_account_robot(robots, robot, cached_names)?
        .ok_or_else(|| {
            CliError::Validation(format!(
                "robot '{}' is neither a pinned alias nor an owned account robot; use a paired alias, endpoint ID, or account:<robot-id>",
                crate::topic_cmd::sanitize_display(robot)
            ))
        })?;
    match &selected.endpoint {
        account_robots::RobotEndpoint::Known(key) => Ok(SelectedAccountRobot {
            robot: selected.clone(),
            eid: hex::encode(key.0),
        }),
        account_robots::RobotEndpoint::Unknown { reason } => Err(CliError::Validation(format!(
            "cannot open the selected account robot: {reason}"
        ))),
    }
}

#[cfg(unix)]
fn prepare(robot: &str, operation: &Operation, json: bool) -> CliResult<PreparedSession> {
    require_network(operation)?;
    let config = auth::cerulion_config_dir().ok_or_else(|| {
        CliError::Validation("cannot resolve the Cerulion directory for recording witnesses".into())
    })?;
    let (connection, account) = if local_selector(robot)? {
        // Raw endpoint IDs and explicit pins never need the account service.
        (
            connect_cmd::plan(&ConnectArgs {
                robot: Some(robot.to_owned()),
                ..ConnectArgs::default()
            })?,
            None,
        )
    } else {
        let directory = account_robots::fetch()?;
        let names = peer_cache::default_cache_path()
            .map(|path| peer_cache::load_peers(&path, peer_cache::current_unix_secs()))
            .unwrap_or_default()
            .into_iter()
            .map(|peer| peer.robot)
            .collect();
        let selected = account_endpoint(&directory.robots, robot, &names)?;
        let mut connection = connect_cmd::plan(&ConnectArgs {
            eid: Some(selected.eid),
            ..ConnectArgs::default()
        })?;
        // Replace reverse-pin names before session_plan appends the command tail.
        set_account_epoch_route(&mut connection, &selected.robot)?;
        let auth_path = auth::auth_json_path().ok_or_else(|| {
            CliError::Validation("cannot resolve the current account store".into())
        })?;
        connection.argv.extend([
            "--owner-account".into(),
            hex::encode(directory.account_id.0),
            "--owner-desk-key".into(),
            hex::encode(directory.desk_key.0),
            "--owner-config-dir".into(),
            context_path(&config)?,
            "--owner-store-lock".into(),
            context_path(&auth::store_lock_path(&auth_path))?,
        ]);
        (
            connection,
            Some(AccountSession {
                directory,
                robot: selected.robot,
            }),
        )
    };
    Ok(PreparedSession {
        plan: session_plan(connection, operation, json, &config.join("shell-witnesses"))?,
        account,
    })
}

#[cfg(unix)]
fn context_path(path: &Path) -> CliResult<String> {
    path.to_str()
        .map(str::to_owned)
        .ok_or_else(|| CliError::Validation("the account directory path is not valid UTF-8".into()))
}

/// Resolve a pinned or account-owned robot, start connectd, and preserve its exit status.
///
/// Explicit pins and endpoint IDs skip account lookup. Account selections retain
/// their login and desk-key identity, rechecked around the bounded epoch-refresh
/// wait. The helper receives public account context, never a session token.
#[cfg(unix)]
pub fn run(
    robot: String,
    operation: Operation,
    json: bool,
    running: Arc<AtomicBool>,
) -> CliResult<i32> {
    prepare(&robot, &operation, json)?.spawn(running)
}

/// Refuse unsupported platforms before account lookup or helper creation.
#[cfg(not(unix))]
pub fn run(
    _robot: String,
    operation: Operation,
    _json: bool,
    _running: Arc<AtomicBool>,
) -> CliResult<i32> {
    require_network(&operation)?;
    Err(CliError::Validation("remote shell requires Unix".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_and_engine_refusal_share_the_diagnostic_and_typed_error() {
        for (operation, verb, action) in [
            (Operation::Shell, "shell", "open a shell on the robot"),
            (
                Operation::Exec(vec!["printf".into(), "%s".into(), "--no-network".into()]),
                "exec",
                "run the command on the robot",
            ),
            (
                Operation::Policy { enabled: None },
                "shell-access",
                "inspect or change the robot's shell access",
            ),
        ] {
            for (network, netd, expected) in [
                (
                    Some("off"),
                    None,
                    "CERULION_NETWORK=off disables; unset CERULION_NETWORK",
                ),
                (
                    None,
                    Some("off"),
                    "CERULION_NETD_NETWORK=off disables; unset CERULION_NETD_NETWORK",
                ),
                (
                    Some("off"),
                    Some("off"),
                    "CERULION_NETWORK=off and CERULION_NETD_NETWORK=off disables; \
                     unset CERULION_NETWORK and CERULION_NETD_NETWORK",
                ),
            ] {
                let expected = format!(
                    "cerulion {verb} reaches the robot over the network, which {expected} to {action}"
                );
                assert_eq!(
                    network_refusal(&operation, network.map(str::to_string), netd).as_deref(),
                    Some(expected.as_str())
                );
                let error = require_network_with(
                    &operation,
                    query_network_kill_from(network.map(str::to_string), netd),
                )
                .expect_err("disabled network");
                assert!(matches!(error, CliError::RemoteShellNetworkRefused(_)));
                assert_eq!(error.to_string(), expected);
            }
        }
    }

    #[test]
    fn network_policy_reuses_fail_closed_and_exact_netd_semantics() {
        for (network, netd, blocked) in [
            (None, None, false),
            (Some("  "), None, false),
            (None, Some("OFF"), false),
            (None, Some(" off "), false),
            (Some("off"), None, true),
            (Some(" OFF "), None, true),
            (Some("on"), None, true),
            (None, Some("off"), true),
            (Some("off"), Some("off"), true),
        ] {
            let kill = crate::graph_cmd::query_network_kill_from(network.map(str::to_owned), netd);
            assert_eq!(
                require_network_with(&Operation::Shell, kill).is_err(),
                blocked,
                "{network:?}/{netd:?}"
            );
        }
        let error = require_network_with(
            &Operation::Shell,
            crate::graph_cmd::query_network_kill_from(
                Some("bad\u{1b}[31m\nvalue".into()),
                Some("off"),
            ),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("unrecognized value"));
        assert!(error.contains("unset CERULION_NETWORK and CERULION_NETD_NETWORK"));
        assert!(!error.contains(['\u{1b}', '\n']));
    }

    fn paired() -> ConnectPlan {
        ConnectPlan {
            bin: "cerulion-connectd".into(),
            argv: [
                "connect",
                "--eid",
                "robot-key",
                "--robot-name",
                "robot",
                "--key-file",
                "desk.key",
            ]
            .map(str::to_owned)
            .to_vec(),
        }
    }

    #[test]
    fn exec_preserves_every_argument_and_the_existing_identity() {
        let arguments = ["printf", "%s", "a b", "", "$(touch /tmp/no)", "--json"]
            .map(str::to_owned)
            .to_vec();
        let plan = session_plan(
            paired(),
            &Operation::Exec(arguments),
            true,
            Path::new("witnesses"),
        )
        .unwrap();
        assert_eq!(
            plan.argv,
            [
                "remote-exec",
                "--eid",
                "robot-key",
                "--robot-name",
                "robot",
                "--key-file",
                "desk.key",
                "--json",
                "--witness-dir",
                "witnesses",
                "--",
                "printf",
                "%s",
                "a b",
                "",
                "$(touch /tmp/no)",
                "--json"
            ]
        );
    }

    #[test]
    fn missing_identity_and_invalid_commands_fail_before_spawn() {
        let mut no_key = paired();
        no_key.argv.truncate(5);
        assert!(session_plan(no_key, &Operation::Shell, false, Path::new("w")).is_err());
        for argv in [
            vec![],
            vec![String::new()],
            vec!["sh\0".into()],
            vec!["printf".into(), "x\0y".into()],
        ] {
            assert!(session_plan(paired(), &Operation::Exec(argv), false, Path::new("w")).is_err());
        }
    }

    #[test]
    fn malformed_connection_plans_are_refused_without_an_index_panic() {
        for argv in [
            vec![],
            vec!["--key-file", "desk.key"],
            vec!["pair", "--key-file", "desk.key"],
            vec!["connect", "--key-file"],
            vec!["connect", "--key-file", ""],
            vec!["connect", "--key-file", "--json"],
            vec!["connect", "--key-file", "bad\0key"],
        ] {
            let mut connection = paired();
            connection.argv = argv.into_iter().map(str::to_owned).collect();
            for operation in [Operation::Shell, Operation::Policy { enabled: None }] {
                assert!(matches!(
                    session_plan(connection.clone(), &operation, false, Path::new("w")),
                    Err(CliError::Validation(_))
                ));
            }
        }
    }

    #[test]
    fn shell_plans_match_the_same_literal_oracle_on_repeated_calls() {
        for _ in 0..2 {
            let plan = session_plan(paired(), &Operation::Shell, false, Path::new("a b")).unwrap();
            assert_eq!(plan.bin, Path::new("cerulion-connectd"));
            assert_eq!(
                plan.argv,
                [
                    "remote-shell",
                    "--eid",
                    "robot-key",
                    "--robot-name",
                    "robot",
                    "--key-file",
                    "desk.key",
                    "--witness-dir",
                    "a b"
                ]
            );
        }
    }

    #[test]
    fn empty_and_nul_witness_paths_are_refused_only_for_recorded_operations() {
        for path in [Path::new(""), Path::new("bad\0path")] {
            for operation in [Operation::Shell, Operation::Exec(vec!["true".into()])] {
                assert!(matches!(
                    session_plan(paired(), &operation, false, path),
                    Err(CliError::Validation(_))
                ));
            }
            assert!(
                session_plan(paired(), &Operation::Policy { enabled: None }, false, path).is_ok()
            );
        }
    }

    #[cfg(unix)]
    #[test]
    fn witness_paths_never_replace_non_utf8_bytes() {
        use std::os::unix::ffi::OsStringExt;
        let path = std::path::PathBuf::from(std::ffi::OsString::from_vec(vec![b'w', 0xff]));
        assert!(matches!(
            session_plan(paired(), &Operation::Shell, false, &path),
            Err(CliError::Validation(_))
        ));
        assert!(session_plan(paired(), &Operation::Policy { enabled: None }, false, &path).is_ok());
    }

    #[cfg(unix)]
    #[test]
    fn account_epoch_route_is_set_before_literal_command_flags() {
        use cerulion_pairing::format::{PublicKey, RobotId};
        let robot = account_robots::AccountRobot {
            robot_id: RobotId(std::array::from_fn(|index| index as u8)),
            hostname: "account-name".into(),
            endpoint: account_robots::RobotEndpoint::Known(PublicKey([0x42; 32])),
        };
        let route = "account:000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
        let command = vec![
            "printf".to_owned(),
            "--robot-name".into(),
            "literal-route".into(),
            String::new(),
            "--owner-account".into(),
        ];
        for existing_pin in [false, true] {
            let mut connection = paired();
            if !existing_pin {
                connection.argv.drain(3..5);
            }
            set_account_epoch_route(&mut connection, &robot).unwrap();
            let plan = session_plan(
                connection,
                &Operation::Exec(command.clone()),
                false,
                Path::new("w"),
            )
            .unwrap();
            let separator = plan.argv.iter().position(|arg| arg == "--").unwrap();
            let options = &plan.argv[..separator];
            assert_eq!(
                options
                    .iter()
                    .filter(|arg| arg.as_str() == "--robot-name")
                    .count(),
                1
            );
            let index = options
                .iter()
                .position(|arg| arg == "--robot-name")
                .unwrap();
            assert_eq!(options[index + 1], route);
            assert_eq!(&plan.argv[separator + 1..], command);
        }
        let mut malformed = paired();
        malformed.argv = vec!["connect".into(), "--robot-name".into()];
        assert!(set_account_epoch_route(&mut malformed, &robot).is_err());
    }

    #[test]
    fn policy_change_and_inspection_use_the_owner_switch() {
        for (enabled, expected) in [(false, "off"), (true, "on")] {
            let plan = session_plan(
                paired(),
                &Operation::Policy {
                    enabled: Some(enabled),
                },
                true,
                Path::new("w"),
            )
            .unwrap();
            assert_eq!(plan.argv[0], "shell-access");
            assert_eq!(&plan.argv[7..], ["--json", "--enabled", expected]);
        }
        let inspect = session_plan(
            paired(),
            &Operation::Policy { enabled: None },
            false,
            Path::new("w"),
        )
        .unwrap();
        assert_eq!(inspect.argv[0], "shell-access");
        assert_eq!(inspect.argv.len(), 7);
    }
}

#[cfg(test)]
mod account_tests;
