// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion rcp` verb's PURE resolution logic.
//!
//! `cerulion rcp <robot>` serves a robot's `cerulion-rcpd` as a local
//! stdio MCP server. Like `cerulion connect`, the CLI stays iroh-free: it
//! resolves the robot (a 64-char-hex eid, `--eid`, or a `~/.cerulion/robots.toml`
//! pin) and this machine's device key, builds a `cerulion-connectd rcp` argv,
//! and spawns that binary with inherited stdio
//! ([`crate::connect_cmd::spawn_and_wait`]), so the child's stdin and stdout
//! are the MCP stream.

use std::path::PathBuf;

use crate::connect_cmd::{
    default_connect_key_file, load_robots, resolve_connectd_bin, resolve_eid_with, ConnectPlan,
};
use crate::error::CliResult;

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
