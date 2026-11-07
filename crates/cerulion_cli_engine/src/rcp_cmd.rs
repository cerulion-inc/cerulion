// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion rcp` verb's PURE resolution logic.
//!
//! `cerulion rcp <robot>` serves a robot's `cerulion-rcpd` as a local
//! stdio MCP server. Like `cerulion connect`, the CLI stays iroh-free: it
//! resolves the robot (a 64-char-hex eid, `--eid`, or a `~/.cerulion/robots.toml`
//! pin) and this machine's device key, builds a `cerulion-connectd rcp` argv,
//! and spawns that binary with inherited stdio
//! ([`crate::connect_cmd::spawn_and_wait`]), so the child's stdin and stdout
//! are the MCP stream. Under the network kill-switch it is refused before any
//! of that ([`network_refusal`]).

use std::path::PathBuf;

use crate::connect_cmd::{
    default_connect_key_file, load_robots, resolve_connectd_bin, resolve_eid_with, ConnectPlan,
};
use crate::error::CliResult;
use crate::graph_cmd::{network_env_kill_from, NetworkEnvKill};

/// The exit code of a `cerulion rcp` the kill-switch refuses: the usage
/// class, the code `--robot` on `topic pub` exits with under the switch.
pub const EXIT_NETWORK_REFUSED: u8 = crate::message_text::EXIT_USAGE;

/// The raw `cerulion rcp` flags.
#[derive(Debug, Clone, Default)]
pub struct RcpArgs {
    /// The positional ROBOT: a 64-char-hex eid OR a `robots.toml` name.
    pub robot: Option<String>,
    /// The explicit `--eid`.
    pub eid: Option<String>,
    /// Direct `ip:port` addresses (`--addr`, repeatable).
    pub addrs: Vec<String>,
    /// The desk device key file (`--key-file`).
    pub key_file: Option<PathBuf>,
    /// Self-hosted relay URL (`--relay-url`).
    pub relay_url: Option<String>,
    /// Disable relays (`--relay-disabled`).
    pub relay_disabled: bool,
    /// The `off` kill-switch (`--network`).
    pub network: Option<String>,
}

/// The refusal of `cerulion rcp` while the kill-switch is engaged, `None`
/// when it is not. `flag` is the `--network` value and `env` the
/// `CERULION_NETWORK` value (`None` = not given), each read under the parse
/// `cerulion pair` and `--robot` use ([`network_env_kill_from`]): `off` after
/// trimming, ASCII case-insensitively, engages it, any other non-empty value
/// fails closed and engages it too, and an empty value is unset. The refusal
/// names the value; it comes before the robot is resolved, the device key is
/// read or `cerulion-connectd` is spawned. PURE.
pub fn network_refusal(flag: Option<&str>, env: Option<String>) -> Option<String> {
    let reached = "cerulion rcp reaches the robot over the network, which";
    let flag_label = match network_env_kill_from(flag.map(str::to_string)) {
        NetworkEnvKill::Unset => None,
        NetworkEnvKill::Off => Some("--network off".to_string()),
        NetworkEnvKill::Unrecognized(value) => Some(format!(
            "--network {} (an unrecognized value, which fails closed to local-only)",
            crate::topic_cmd::sanitize_display(value.trim())
        )),
    };
    if let Some(label) = flag_label {
        return Some(format!(
            "{reached} {label} disables; drop --network to reach the robot's rcpd"
        ));
    }
    network_env_kill_from(env).refusal_label().map(|label| {
        format!("{reached} {label} disables; unset CERULION_NETWORK to reach the robot's rcpd")
    })
}

/// The `cerulion-connectd rcp` argv for a resolved eid. PURE.
pub fn build_rcp_argv(eid: &str, args: &RcpArgs) -> Vec<String> {
    let mut v = vec!["rcp".to_string(), "--eid".to_string(), eid.to_string()];
    for a in &args.addrs {
        v.push("--addr".to_string());
        v.push(a.clone());
    }
    if let Some(k) = &args.key_file {
        v.push("--key-file".to_string());
        v.push(k.display().to_string());
    }
    if let Some(u) = &args.relay_url {
        v.push("--relay-url".to_string());
        v.push(u.clone());
    }
    if args.relay_disabled {
        v.push("--relay-disabled".to_string());
    }
    if let Some(n) = &args.network {
        v.push("--network".to_string());
        v.push(n.clone());
    }
    v
}

/// Resolve the spawn plan: the robot eid, this machine's key when none was
/// passed and one exists, the argv and the `cerulion-connectd` path.
///
/// # Errors
///
/// An unresolvable robot, an unreadable machine key, or a missing
/// `cerulion-connectd`.
pub fn plan(args: &RcpArgs) -> CliResult<ConnectPlan> {
    let robots = load_robots();
    let eid = resolve_eid_with(args.robot.as_deref(), args.eid.as_deref(), &robots)?;
    let mut args = args.clone();
    if args.key_file.is_none() {
        args.key_file = default_connect_key_file()?;
    }
    let argv = build_rcp_argv(&eid, &args);
    let bin = resolve_connectd_bin()?;
    Ok(ConnectPlan { bin, argv })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_kill_switch_refuses_on_the_flag_or_the_variable_and_names_the_value() {
        let unset = [None, Some(String::new()), Some("  ".to_string())];
        for env in unset.clone() {
            assert_eq!(network_refusal(None, env.clone()), None, "{env:?}");
            assert_eq!(network_refusal(Some(""), env.clone()), None, "{env:?}");
        }
        for env in ["off", "OFF", " off "] {
            assert_eq!(
                network_refusal(None, Some(env.into())).as_deref(),
                Some(
                    "cerulion rcp reaches the robot over the network, which \
                     CERULION_NETWORK=off disables; unset CERULION_NETWORK to reach the \
                     robot's rcpd"
                ),
                "{env:?}"
            );
        }
        assert_eq!(
            network_refusal(None, Some("typo".into())).as_deref(),
            Some(
                "cerulion rcp reaches the robot over the network, which CERULION_NETWORK=typo \
                 (an unrecognized value, which fails closed to local-only) disables; unset \
                 CERULION_NETWORK to reach the robot's rcpd"
            )
        );
        for env in unset.into_iter().chain([Some("off".to_string())]) {
            assert_eq!(
                network_refusal(Some(" Off "), env.clone()).as_deref(),
                Some(
                    "cerulion rcp reaches the robot over the network, which --network off \
                     disables; drop --network to reach the robot's rcpd"
                ),
                "{env:?}"
            );
        }
        assert_eq!(
            network_refusal(Some("of"), None).as_deref(),
            Some(
                "cerulion rcp reaches the robot over the network, which --network of (an \
                 unrecognized value, which fails closed to local-only) disables; drop \
                 --network to reach the robot's rcpd"
            )
        );
        assert_eq!(EXIT_NETWORK_REFUSED, 2);
    }

    #[test]
    fn argv_is_the_rcp_subcommand_with_every_flag_in_order() {
        let args = RcpArgs {
            robot: Some("go2".into()),
            eid: None,
            addrs: vec!["10.0.0.2:4433".into(), "[::1]:9".into()],
            key_file: Some(PathBuf::from("/k/desk.key")),
            relay_url: Some("https://relay.example".into()),
            relay_disabled: true,
            network: Some("off".into()),
        };
        assert_eq!(
            build_rcp_argv("ab", &args),
            [
                "rcp",
                "--eid",
                "ab",
                "--addr",
                "10.0.0.2:4433",
                "--addr",
                "[::1]:9",
                "--key-file",
                "/k/desk.key",
                "--relay-url",
                "https://relay.example",
                "--relay-disabled",
                "--network",
                "off"
            ]
        );
        assert_eq!(
            build_rcp_argv("cd", &RcpArgs::default()),
            ["rcp", "--eid", "cd"]
        );
    }
}
