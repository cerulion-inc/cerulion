// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion service`: call a service and print its response.
//!
//! A verb family (see [`super`]): this module is the whole of the verb, from
//! its clap definition to its dispatch. The command logic lives in
//! `cerulion_cli_engine::service_cmd`.

use std::process::ExitCode;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};

use cerulion_cli_engine::message_text::VerbError;
use cerulion_cli_engine::service_cmd;

use super::nested::topic_message::{seconds, verb_exit_code};
use super::VerbFamily;
use crate::cli::VerbLogClass;

// The family's top-level verb, declared as a one-variant `Subcommand` so the
// derive generates the `service` command.
#[derive(Subcommand)]
enum ServiceCmd {
    /// Call services
    Service {
        #[command(subcommand)]
        action: ServiceAction,
    },
}

#[derive(Subcommand)]
enum ServiceAction {
    /// Call a service once and print its response as JSON
    ///
    /// Sends REQUEST (a JSON or YAML mapping, default `{}`; a field it leaves
    /// out takes its `.msg` default or zero) to SERVICE as TYPE and prints the
    /// response as one JSON document. Speaks the protocol ROS 2 nodes on
    /// rmw_cerulion serve, so `/controller_manager/list_controllers` answers
    /// with no ROS tools installed. TYPE's `.srv` file is read from the
    /// workspace's `schemas/<pkg>/srv/<Type>.srv`. The server must run on
    /// this machine, or with `--robot` on that paired robot. Exit 3 when no server or no response arrives within
    /// `--timeout` (one deadline for both), 2 for a usage error or an unknown type, 1 for a value
    /// that does not fit its field.
    Call {
        /// Call the service on the paired robot NAME (a `~/.cerulion/robots.toml`
        /// name or endpoint id) through its ops plane instead of on this
        /// machine.
        #[arg(long, value_name = "NAME",
              add = clap_complete::engine::ArgValueCandidates::new(crate::completion::robot_names))]
        robot: Option<String>,
        /// Seconds to wait in total for a server and its response
        #[arg(long, value_name = "SECS",
              default_value_t = service_cmd::DEFAULT_CALL_TIMEOUT.as_secs_f64())]
        timeout: f64,
        /// Service name, for example `/controller_manager/list_controllers`
        service: String,
        /// Service type: `package/srv/Type` or `package/Type`
        #[arg(value_name = "TYPE")]
        type_name: String,
        /// The request as a JSON or YAML mapping
        #[arg(default_value = "{}")]
        request: String,
    },
}

/// The `service` verb family.
pub(crate) struct Service;

impl VerbFamily for Service {
    fn name(&self) -> &'static str {
        "service"
    }

    fn listed_after(&self) -> Option<&'static str> {
        Some("topic")
    }

    fn command(&self) -> Command {
        super::derived_verb::<ServiceCmd>()
    }

    /// `service call` runs and exits, so it defaults quiet.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    /// The same rule as every `topic` verb.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = ServiceAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        verb_exit_code(run(action))
    }
}

fn run(action: ServiceAction) -> Result<String, VerbError> {
    match action {
        ServiceAction::Call {
            robot,
            timeout,
            service,
            type_name,
            request,
        } => {
            let ws = crate::discover_optional_workspace()?;
            let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
            let running = crate::setup_ctrlc_handler()?;
            let req = service_cmd::CallRequest {
                service: &service,
                type_name: &type_name,
                request: &request,
                timeout: seconds("--timeout", timeout)?,
            };
            match robot {
                Some(robot) => cerulion_cli_engine::robot_act::robot_service_call(&robot, &req),
                None => service_cmd::service_call(&req, schemas_dir, &running),
            }
        }
    }
}
