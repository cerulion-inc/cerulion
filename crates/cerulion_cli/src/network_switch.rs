// SPDX-License-Identifier: AGPL-3.0-only
//! The network kill switch, decided before the login gate.
//!
//! `CERULION_NETWORK` (and a verb's own `--network off`) is the network kill
//! switch. A verb that reaches a robot or a daemon over the network refuses
//! while it is engaged, BEFORE the login gate, so a machine that never signed
//! in meets the refusal and never the gate's device-code sign-in against the
//! account service.
//!
//! Every top-level verb and verb set states its refusal: a
//! [`crate::families::VerbFamily`] and a nested-verb set through their
//! `network_refusal` method, a `Commands` verb through the exhaustive match in
//! `main.rs`, and a subverb of a `Commands` verb through its parent's arm, held
//! by its row in `cases.rs`. Both dispatch paths hand that
//! refusal to [`refuse_or_gate`], the one function the login gate is called
//! from, so a refused verb never reaches the gate.

use std::fmt;
use std::process::ExitCode;

use clap::ArgMatches;

use cerulion_cli_engine::{graph_cmd, robot_act, viz_cmd};

/// `CERULION_NETWORK`.
const NETWORK_ENV: &str = "CERULION_NETWORK";

/// The kill-switch variables of one invocation, read once.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct SwitchEnv {
    /// `CERULION_NETWORK` (`None` = not set).
    pub(crate) network: Option<String>,
    /// `CERULION_NETD_NETWORK`, which `viz --robot` also reads.
    pub(crate) netd: Option<String>,
    /// `CERULION_VIZD_NETWORK`, which `viz --robot` also reads.
    pub(crate) vizd: Option<String>,
}

impl SwitchEnv {
    /// The variables of this process.
    pub(crate) fn from_process() -> Self {
        Self {
            network: std::env::var(NETWORK_ENV).ok(),
            netd: std::env::var(graph_cmd::NETD_NETWORK_ENV).ok(),
            vizd: std::env::var(viz_cmd::VIZD_NETWORK_ENV).ok(),
        }
    }

    /// `CERULION_NETWORK` with only `network` set.
    #[cfg(test)]
    pub(crate) fn network(value: Option<&str>) -> Self {
        Self {
            network: value.map(str::to_string),
            ..Self::default()
        }
    }

    /// Whether `CERULION_NETWORK` engages the switch under the run and query
    /// parse ([`graph_cmd::network_env_kill_from`]): `off` after trimming in
    /// any case, and any other non-empty value, which fails closed.
    pub(crate) fn network_engaged(&self) -> bool {
        graph_cmd::network_env_kill_from(self.network.clone()).is_off()
    }
}

/// What a refused invocation writes to stderr, and the code it exits with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct NetworkRefusal {
    line: String,
    code: u8,
}

impl NetworkRefusal {
    /// `Error: <message>` and exit `code`, the frame every verb error prints.
    pub(crate) fn error(message: impl fmt::Display, code: u8) -> Self {
        Self {
            line: format!("Error: {message}"),
            code,
        }
    }

    /// `warning: <message>` and exit `code`.
    pub(crate) fn warning(message: impl fmt::Display, code: u8) -> Self {
        Self {
            line: format!("warning: {message}"),
            code,
        }
    }

    /// The stderr line.
    #[cfg(test)]
    pub(crate) fn line(&self) -> &str {
        &self.line
    }

    /// The exit code.
    #[cfg(test)]
    pub(crate) fn code(&self) -> u8 {
        self.code
    }

    /// Print the line and return the exit code.
    fn exit(&self) -> ExitCode {
        eprintln!("{}", self.line);
        ExitCode::from(self.code)
    }
}

/// Refuse with `refusal`, else run `gate` when `needs_identity`. `Some` is the
/// exit code of a refusal or of a gate that refused; `None` lets the verb run.
pub(crate) fn refuse_or_gate(
    refusal: Option<NetworkRefusal>,
    needs_identity: bool,
    gate: impl FnOnce() -> Option<ExitCode>,
) -> Option<ExitCode> {
    if let Some(refusal) = refusal {
        return Some(refusal.exit());
    }
    if needs_identity {
        gate()
    } else {
        None
    }
}

/// The refusal of `--robot robot` on an act verb that calls `verb` on the
/// robot ([`robot_act::network_refusal`], exit 2). `None` without `--robot`:
/// the command runs on this machine.
pub(crate) fn robot_refusal(
    robot: Option<&str>,
    verb: &str,
    env: &SwitchEnv,
) -> Option<NetworkRefusal> {
    let refusal = robot_act::network_refusal(robot?, verb, env.network.as_deref())?;
    Some(NetworkRefusal::error(&refusal, refusal.exit_code()))
}

/// Whether the switch is engaged for the invocation `matches` (the root
/// matches): `CERULION_NETWORK`, or the invocation's own `--network` value,
/// under the run and query parse ([`SwitchEnv::network_engaged`]), or `--local`
/// on `graph run` and `node run`, where it is the run's `--network off`. On
/// every other verb `--local` selects that verb's local scope and is not the
/// switch.
pub(crate) fn engaged(matches: &ArgMatches, env: &SwitchEnv) -> bool {
    if env.network_engaged() {
        return true;
    }
    let mut path: Vec<&str> = Vec::new();
    let mut at = matches;
    loop {
        // `--network <value>` on connect, rcp, graph run and node run. An id
        // the level does not define reads as no value in every build profile.
        let value = at
            .try_get_raw("network")
            .ok()
            .flatten()
            .and_then(|mut values| values.next())
            .map(|value| value.to_string_lossy().into_owned());
        if graph_cmd::network_env_kill_from(value).is_off() {
            return true;
        }
        let Some((name, sub)) = at.subcommand() else {
            break;
        };
        path.push(name);
        at = sub;
    }
    // The verb is decided by its path, not by whether its level defines
    // `local`: clap answers an undefined id with an error only in a debug
    // build.
    matches!(path.as_slice(), ["graph", "run"] | ["node", "run"])
        && matches!(at.try_get_one::<bool>("local"), Ok(Some(true)))
}

#[cfg(test)]
mod cases;
#[cfg(test)]
mod tests;
