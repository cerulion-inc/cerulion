// SPDX-License-Identifier: AGPL-3.0-only
//! Pure argument planning and network refusals for paired remote sessions.

use std::path::Path;

use crate::connect_cmd::ConnectPlan;
use crate::error::{CliError, CliResult};
use crate::graph_cmd::{query_network_kill, query_network_kill_from, QueryNetworkKill};

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
