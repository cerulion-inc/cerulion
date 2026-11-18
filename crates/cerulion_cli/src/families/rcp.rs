// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion rcp`: serve a robot's `cerulion-rcpd` as a local stdio MCP
//! server, or, with `--sim`, the Cerulion harness's tools with a simulated
//! robot.
//!
//! A verb family (see [`super`]): parsing and dispatch. The refusal and the
//! helper plan live in `cerulion_cli_engine::rcp_cmd`; the session itself runs
//! in the spawned `cerulion-connectd rcp`. `--sim` finds the harness and plans
//! its launcher in `cerulion_cli_engine::harness_cmd`, and `cerulion rcp bench`
//! is a nested verb (`super::nested::rcp_bench`).

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use clap::{ArgMatches, Args, Command, FromArgMatches, Subcommand};
use clap_complete::engine::ArgValueCandidates;

use cerulion_cli_engine::{connect_cmd, harness_cmd, rcp_cmd};

use super::VerbFamily;
use crate::cli::VerbLogClass;
use crate::network_switch::{NetworkRefusal, SwitchEnv};

// The family's one verb, declared as a one-variant `Subcommand` whose
// arguments are a separate `Args` struct, so `run` parses the family's own
// sub-matches straight into it.
#[derive(Subcommand)]
enum RcpCmd {
    /// Serve a robot, or a simulated one, as an MCP server.
    ///
    /// `cerulion rcp <robot>` dials the robot's rcp plane with this machine's
    /// device key and bridges one MCP session over stdin and stdout
    /// (newline-delimited JSON-RPC), so an MCP client such as Claude Desktop
    /// can run it as a stdio server and use the robot's rcpd resources and
    /// tools directly. STDOUT carries only MCP messages; logs go to STDERR.
    ///
    /// The robot admits only a desk on its access list, and checks every
    /// message: reading resources needs the observe capability, calling a
    /// tool needs the teleop capability and the operator role, and any desk
    /// on the access list may call the `estop` tool.
    ///
    /// Runs the `cerulion-connectd` helper binary. The release packages place
    /// it beside `cerulion`; a source build gets it from
    /// `cargo build -p cerulion_connectd`, and `CERULION_CONNECTD_BIN` points
    /// at one anywhere else.
    ///
    /// `cerulion rcp --sim` serves the Cerulion harness's tools with a
    /// simulated robot on this machine instead: over stdio, or over streamable
    /// HTTP at `--http ADDR` (path `/mcp`). The first run installs the
    /// simulator and the benchmark scenes once, with its progress on STDERR.
    /// `cerulion rcp bench` reruns a benchmark task with and without the
    /// harness against your own model. Both run under the free Cerulion
    /// account, as `cerulion rcp <robot>` does; neither reaches a robot.
    #[command(args_conflicts_with_subcommands = true)]
    Rcp(RcpCliArgs),
}

#[derive(Args)]
struct RcpCliArgs {
    /// The robot: a 64-char-hex endpoint id, OR a name pinned in
    /// `~/.cerulion/robots.toml`. Omit when using `--eid`.
    #[arg(value_name = "ROBOT", add = ArgValueCandidates::new(crate::completion::robot_names))]
    robot: Option<String>,
    /// The robot's iroh endpoint id (64-char hex). Use instead of ROBOT;
    /// passing both is refused.
    #[arg(long, value_name = "HEX")]
    eid: Option<String>,
    /// A direct `ip:port` for the robot (LAN direct-dial). Repeatable.
    #[arg(long = "addr", value_name = "IP:PORT")]
    addrs: Vec<String>,
    /// The desk device key file (32 raw bytes). Default: this machine's
    /// key `~/.cerulion/identity/device.key` when it exists.
    #[arg(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    key_file: Option<PathBuf>,
    /// Self-hosted relay URL. Default: n0 public relays.
    #[arg(long, value_name = "URL")]
    relay_url: Option<String>,
    /// Disable all relays (LAN / direct-dial only).
    #[arg(long)]
    relay_disabled: bool,
    /// Network kill switch: `off` refuses to dial the robot. The switch keeps
    /// Cerulion service traffic off (robots, relays, hosted Cerulion services
    /// and telemetry); signing in to your account still works.
    #[arg(long)]
    network: Option<String>,
    /// Serve the Cerulion harness's tools with a simulated robot on this
    /// machine instead of a robot. The first run installs the simulator and
    /// the benchmark scenes once.
    #[arg(long, conflicts_with_all = ["robot", "eid", "addrs", "key_file", "relay_url", "relay_disabled", "network"])]
    sim: bool,
    /// With `--sim`: serve streamable HTTP at ADDR (path `/mcp`) instead of
    /// stdio, for example `127.0.0.1:8765`.
    #[arg(long, value_name = "ADDR", requires = "sim")]
    http: Option<String>,
    /// With `--sim`: the task the simulated robot is set up for:
    /// `bowl-on-plate` (the default), `open-drawer` or `soup-in-basket`.
    #[arg(long, value_name = "NAME", requires = "sim")]
    task: Option<String>,
}

/// The `rcp` verb family.
pub(crate) struct Rcp;

impl VerbFamily for Rcp {
    fn name(&self) -> &'static str {
        "rcp"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("pair")
    }

    fn command(&self) -> Command {
        super::derived_verb::<RcpCmd>()
    }

    /// `rcp` stays attached to the helper for the MCP session.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::LongRunning
    }

    /// `cerulion rcp` reaches the robot, so `CERULION_NETWORK` or its own
    /// `--network` refuses it with exit 2 ([`rcp_cmd::network_refusal`]). The
    /// registry gives the refusal before the login gate and before
    /// [`VerbFamily::run`], so a refused `cerulion rcp` never contacts the
    /// account service, asks for a sign-in, resolves the robot, reads the key
    /// or spawns the helper.
    ///
    /// `--sim` reaches no robot and no Cerulion service (its first run
    /// downloads the simulator from the benchmark's and the Python packages'
    /// own hosts), so the switch does not refuse it.
    fn network_refusal(&self, matches: &ArgMatches, env: &SwitchEnv) -> Option<NetworkRefusal> {
        if matches.get_flag("sim") {
            return None;
        }
        rcp_cmd::network_refusal(
            matches.get_one::<String>("network").map(String::as_str),
            env.network.clone(),
        )
        .map(|refusal| NetworkRefusal::error(refusal, rcp_cmd::EXIT_NETWORK_REFUSED))
    }

    /// The same rule as `connect`, `--sim` included: every form runs under
    /// the free Cerulion account.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    /// Spawns `cerulion-connectd rcp` with inherited stdio (the MCP stream)
    /// and forwards its exit code; the kill switch has not refused the
    /// invocation by now ([`VerbFamily::network_refusal`]). Errors go to
    /// stderr; stdout stays MCP-only.
    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let cli = RcpCliArgs::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        if cli.sim {
            return run_harness(&harness_cmd::HarnessVerb::Sim {
                http: cli.http,
                task: cli.task,
            });
        }
        let args = rcp_cmd::RcpArgs {
            robot: cli.robot,
            eid: cli.eid,
            addrs: cli.addrs,
            key_file: cli.key_file,
            relay_url: cli.relay_url,
            relay_disabled: cli.relay_disabled,
            network: cli.network,
        };
        let plan = match rcp_cmd::plan(&args) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Error: {e}");
                return ExitCode::FAILURE;
            }
        };
        let running = match crate::setup_ctrlc_handler() {
            Ok(r) => r,
            Err(e) => {
                eprintln!("warning: could not install a signal handler: {e}");
                Arc::new(AtomicBool::new(true))
            }
        };
        match connect_cmd::spawn_and_wait(&plan, running) {
            Ok(code) => ExitCode::from(code as u8),
            Err(e) => {
                eprintln!("Error: {e}");
                ExitCode::FAILURE
            }
        }
    }
}

/// Find the harness, plan its launcher for `verb` and run it with inherited
/// stdio (the MCP stream for `--sim`), returning its exit code. Shared by
/// `--sim` and the nested `cerulion rcp bench`.
pub(crate) fn run_harness(verb: &harness_cmd::HarnessVerb) -> ExitCode {
    let plan = match harness_plan(verb) {
        Ok(plan) => plan,
        Err(e) => return super::exit_code_of(Err(e)),
    };
    let running = match crate::setup_ctrlc_handler() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("warning: could not install a signal handler: {e}");
            Arc::new(AtomicBool::new(true))
        }
    };
    match connect_cmd::spawn_child_and_wait(&plan, running, &harness_cmd::HARNESS_CHILD) {
        Ok(code) => ExitCode::from(code as u8),
        Err(e) => super::exit_code_of(Err(e)),
    }
}

/// The launcher plan for `verb`: the harness folder this process finds and
/// the copy home under this machine's Cerulion config directory.
fn harness_plan(
    verb: &harness_cmd::HarnessVerb,
) -> cerulion_cli_engine::error::CliResult<connect_cmd::ConnectPlan> {
    let dir = harness_cmd::find_harness_dir_here()?;
    let config = cerulion_cli_engine::auth::cerulion_config_dir().ok_or_else(|| {
        cerulion_cli_engine::error::CliError::Validation(
            "no home directory to keep the harness in; set CERULION_HOME".to_string(),
        )
    })?;
    harness_cmd::plan(verb, &dir, &harness_cmd::copy_home(&config))
}
