// SPDX-License-Identifier: AGPL-3.0-only
//! The `restart` verb: restart a named Cerulion graph unit via the systemd
//! transient-unit pattern (`systemd-run --unit=cerulion-graph-<name>`).
//!
//! The graph side exits 0 on a clean SIGTERM, so a stop is a
//! graceful shutdown. On a non-Linux host the verb returns a structured
//! [`CerudError::UnsupportedHost`] — never a panic.
//!
//! The command construction is PURE ([`build_stop_command`] /
//! [`build_start_command`], oracle-tested) and the graph name is strictly
//! validated before it is interpolated into a unit name (defense against
//! injection into the transient-unit command). Execution goes through the
//! injectable [`RestartRunner`] seam, so dispatch + validation are testable
//! without invoking systemd.

use serde::Deserialize;

use crate::error::{CerudError, CerudResult};
use crate::verbs::VerbHandler;

/// The `restart` verb.
pub struct RestartVerb {
    runner: Box<dyn RestartRunner>,
}

impl RestartVerb {
    /// The production verb: a systemd runner on Linux, an
    /// unsupported-host runner everywhere else.
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        RestartVerb {
            runner: default_runner(),
        }
    }

    /// A verb with an injected runner (used by tests to capture the argv or
    /// force the unsupported-host arm).
    pub fn with_runner(runner: Box<dyn RestartRunner>) -> Self {
        RestartVerb { runner }
    }
}

#[derive(Debug, Deserialize)]
struct RestartArgs {
    graph: String,
}

impl VerbHandler for RestartVerb {
    fn name(&self) -> &'static str {
        "restart"
    }

    fn is_mutating(&self) -> bool {
        true
    }

    fn execute(&self, args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        let args: RestartArgs = serde_json::from_value(args.clone())
            .map_err(|e| CerudError::Verb(format!("invalid restart args: {e}")))?;
        validate_graph_name(&args.graph)?;

        if !self.runner.supported() {
            return Err(CerudError::UnsupportedHost(
                "restart requires a Linux host with systemd (systemd-run)".to_string(),
            ));
        }

        // Stop is best-effort (the unit may not be running yet); start must
        // succeed. Both commands are built from the validated graph name.
        let stop = build_stop_command(&args.graph)?;
        let _ = self.runner.run(&stop);
        let start = build_start_command(&args.graph)?;
        self.runner.run(&start)?;

        Ok(serde_json::json!({
            "graph": args.graph,
            "unit": unit_name(&args.graph),
            "restarted": true,
        }))
    }
}

/// The transient unit name for a graph.
pub fn unit_name(graph: &str) -> String {
    format!("cerulion-graph-{graph}")
}

/// Validate a graph name before it is baked into a systemd unit / command.
///
/// Pure. Accepts only non-empty ASCII alphanumerics plus `-`/`_`, and bounds
/// the length. This rejects path separators, whitespace, `..`, and shell
/// metacharacters outright. A LEADING `-` is additionally rejected so the name
/// can never be mistaken for a command-line flag when it reaches
/// `systemd-run`/`systemctl` (argument-injection defense-in-depth).
pub fn validate_graph_name(graph: &str) -> CerudResult<()> {
    if graph.is_empty() {
        return Err(CerudError::Verb("restart: graph name is empty".to_string()));
    }
    if graph.len() > 128 {
        return Err(CerudError::Verb(
            "restart: graph name exceeds 128 characters".to_string(),
        ));
    }
    if graph.starts_with('-') {
        return Err(CerudError::Verb(format!(
            "restart: graph name '{graph}' may not start with '-' (would be read as a flag)"
        )));
    }
    if !graph
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_')
    {
        return Err(CerudError::Verb(format!(
            "restart: graph name '{graph}' has invalid characters (allowed: a-z A-Z 0-9 - _)"
        )));
    }
    Ok(())
}

/// The best-effort stop command for a graph's transient unit.
pub fn build_stop_command(graph: &str) -> CerudResult<Vec<String>> {
    validate_graph_name(graph)?;
    Ok(vec![
        "systemctl".to_string(),
        "stop".to_string(),
        unit_name(graph),
    ])
}

/// The start command: a fresh transient unit running `cerulion graph run`.
pub fn build_start_command(graph: &str) -> CerudResult<Vec<String>> {
    validate_graph_name(graph)?;
    Ok(vec![
        "systemd-run".to_string(),
        format!("--unit={}", unit_name(graph)),
        // Reap the unit after it exits so a re-run can reuse the unit name.
        "--collect".to_string(),
        "cerulion".to_string(),
        "graph".to_string(),
        "run".to_string(),
        graph.to_string(),
    ])
}

/// The seam that actually runs a command. Injectable so verb dispatch is
/// testable without invoking systemd.
pub trait RestartRunner: Send + Sync {
    /// Whether restart is supported on this host (false → the verb returns
    /// [`CerudError::UnsupportedHost`] before running anything).
    fn supported(&self) -> bool;

    /// Run `argv` to completion.
    fn run(&self, argv: &[String]) -> CerudResult<()>;
}

/// The production runner for a non-Linux host: restart is unsupported.
#[derive(Debug, Default, Clone, Copy)]
pub struct UnsupportedHostRunner;

impl RestartRunner for UnsupportedHostRunner {
    fn supported(&self) -> bool {
        false
    }

    fn run(&self, _argv: &[String]) -> CerudResult<()> {
        Err(CerudError::UnsupportedHost(
            "restart requires a Linux host with systemd".to_string(),
        ))
    }
}

/// The production runner on Linux: spawn the command and wait.
#[cfg(target_os = "linux")]
#[derive(Debug, Default, Clone, Copy)]
pub struct SystemdRunner;

#[cfg(target_os = "linux")]
impl RestartRunner for SystemdRunner {
    fn supported(&self) -> bool {
        true
    }

    fn run(&self, argv: &[String]) -> CerudResult<()> {
        let (program, rest) = argv
            .split_first()
            .ok_or_else(|| CerudError::Verb("restart: empty command".to_string()))?;
        let status = std::process::Command::new(program).args(rest).status()?;
        if !status.success() {
            return Err(CerudError::Verb(format!(
                "restart: command '{}' exited with {status}",
                argv.join(" ")
            )));
        }
        Ok(())
    }
}

#[cfg(target_os = "linux")]
fn default_runner() -> Box<dyn RestartRunner> {
    Box::new(SystemdRunner)
}

#[cfg(not(target_os = "linux"))]
fn default_runner() -> Box<dyn RestartRunner> {
    Box::new(UnsupportedHostRunner)
}
