// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion doctor`: read-only checks of this host, each with a verdict and
//! the exact fix.
//!
//! A verb family (see [`super`]): parsing, dispatch and output only. The
//! checks live in `cerulion_cli_engine::doctor`.

use std::process::ExitCode;
use std::time::Duration;

use clap::builder::PossibleValuesParser;
use clap::{ArgMatches, Args, Command, FromArgMatches, Subcommand};

use super::VerbFamily;
use crate::cli::VerbLogClass;

// The family's one verb, declared as a one-variant `Subcommand` whose
// arguments are a separate `Args` struct, so `run` parses the family's own
// sub-matches straight into it.
#[derive(Subcommand)]
enum DoctorCmd {
    /// Check this host for problems that slow or stop Cerulion, with the fix for each.
    ///
    /// Every check reads one fact about this machine and reports pass, info,
    /// warn, fail or skip; each warn and fail says why it matters and the exact
    /// command or edit that fixes it. No check needs root, and none writes a
    /// file or changes a sysctl or a network setting; the open-file check
    /// raises this process's own limit the way `graph run` does. Exit 0 when
    /// no check failed (warnings included), 1 when any check failed, 2 for a
    /// usage error.
    Doctor(DoctorArgs),
}

#[derive(Args)]
struct DoctorArgs {
    /// Print the report as one JSON object on stdout instead of the table.
    #[arg(long)]
    json: bool,
    /// Skip the checks that use the network (`net.mdns`, `net.udp`).
    /// `CERULION_NETWORK=off` does the same.
    #[arg(long)]
    local: bool,
    /// Run only this check (repeatable). Default: every check.
    #[arg(
        long = "check",
        value_name = "ID",
        value_parser = PossibleValuesParser::new(cerulion_cli_engine::doctor::check_ids())
    )]
    checks: Vec<String>,
    /// A QUIC endpoint (`host:port`, `[v6]:port`) for the UDP check to probe,
    /// beside the quic/ and udp/...?rel=1 connect locators of the workspace's
    /// graphs (repeatable). Ignored when `net.udp` does not run.
    #[arg(long = "udp-target", value_name = "HOST:PORT", value_parser = parse_host_port)]
    udp_targets: Vec<String>,
    /// How long to wait for the checks, in milliseconds. A check that has not
    /// answered by then is reported as a warning. Default: twice the bounded
    /// scan of the stale-node check.
    #[arg(
        long = "timeout-ms",
        value_name = "MS",
        value_parser = clap::value_parser!(u64).range(1..)
    )]
    timeout_ms: Option<u64>,
}

/// A `--udp-target` value: `HOST:PORT` with a non-empty host and a port from
/// 1 to 65535, an IPv6 address written `[ADDR]:PORT`.
fn parse_host_port(value: &str) -> Result<String, String> {
    let (host, port) = value
        .rsplit_once(':')
        .ok_or_else(|| format!("`{value}` has no port: write it as HOST:PORT"))?;
    if port.is_empty() {
        return Err(format!("`{value}` has no port: write it as HOST:PORT"));
    }
    match port.parse::<u16>() {
        Ok(0) | Err(_) => {
            return Err(format!(
                "`{port}` is not a port from 1 to 65535 in `{value}`"
            ))
        }
        Ok(_) => {}
    }
    let bracketed = host.strip_prefix('[').and_then(|h| h.strip_suffix(']'));
    match bracketed {
        Some(inner) if inner.parse::<std::net::Ipv6Addr>().is_ok() => Ok(value.to_string()),
        Some(_) => Err(format!("`{host}` is not an IPv6 address in `{value}`")),
        None if host.is_empty() => Err(format!("`{value}` has no host: write it as HOST:PORT")),
        None if host.contains(':') || host.contains('[') || host.contains(']') => {
            Err(format!("`{value}`: write an IPv6 address as [ADDR]:PORT"))
        }
        None => Ok(value.to_string()),
    }
}

/// The `doctor` verb family.
pub(crate) struct Doctor;

impl VerbFamily for Doctor {
    fn name(&self) -> &'static str {
        "doctor"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("clean")
    }

    fn command(&self) -> Command {
        super::derived_verb::<DoctorCmd>()
    }

    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    /// The verb diagnoses a machine that has never signed in, among others,
    /// and reaches no account, so the login gate does not run before it.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        false
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let args = DoctorArgs::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        run(&args)
    }
}

fn run(args: &DoctorArgs) -> ExitCode {
    use cerulion_cli_engine::doctor;

    let selected = match doctor::select(&args.checks) {
        Ok(selected) => selected,
        Err(unknown) => {
            eprintln!("Error: {unknown}");
            return ExitCode::from(2);
        }
    };
    let cwd = match std::env::current_dir() {
        Ok(cwd) => cwd,
        Err(e) => {
            eprintln!("Error: cannot read the current directory: {e}");
            return ExitCode::FAILURE;
        }
    };
    // The workspace is discovered inside the checks that read it, under the
    // deadline: a stalled filesystem above `cwd` cannot hold the report.
    let ctx = doctor::Context {
        cwd,
        udp_targets: args.udp_targets.clone(),
    };
    let timeout = args
        .timeout_ms
        .map_or(doctor::DEFAULT_TIMEOUT, Duration::from_millis);
    let report = doctor::run(&selected, &ctx, doctor::network_gate(args.local), timeout);
    if args.json {
        println!(
            "{}",
            serde_json::to_string_pretty(&report.to_json())
                .unwrap_or_else(|_| report.to_json().to_string())
        );
    } else {
        print!("{}", report.render_human());
    }
    ExitCode::from(report.exit_code())
}

#[cfg(test)]
mod tests {
    use super::parse_host_port;

    #[test]
    fn udp_targets_need_a_host_and_a_port_and_brackets_around_ipv6() {
        for ok in [
            "robot.local:7447",
            "10.0.0.2:7447",
            "[::1]:7447",
            "[fd00::2]:1",
        ] {
            assert_eq!(parse_host_port(ok).as_deref(), Ok(ok), "{ok}");
        }
        for bad in [
            "robot.local",
            "robot.local:",
            ":7447",
            "host:0",
            "host:99999",
            "fd00::1",
            "fd00::1:7447",
            "fd00::1]:7447",
            "[fd00::1:7447",
            "[nope]:7447",
        ] {
            assert!(parse_host_port(bad).is_err(), "{bad} was accepted");
        }
    }
}
