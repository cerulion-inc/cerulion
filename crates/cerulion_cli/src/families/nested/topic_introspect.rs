// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic type`, `topic find` and `topic bw`: the discovery verbs a
//! ROS 2 user knows as `ros2 topic type`, `ros2 topic find` and
//! `ros2 topic bw`, added under the built-in `topic` verb (see [`super`]).
//! The command logic lives in `cerulion_cli_engine::topic_cmd::introspect`.

use std::num::NonZeroUsize;
use std::process::ExitCode;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};
use clap_complete::engine::ArgValueCandidates;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::topic_cmd::{introspect, TopicScope};

use super::NestedVerbs;
use crate::cli::VerbLogClass;
use crate::completion;

/// The verb these verbs are added under (read by the docs gate too).
const PARENT: &str = "topic";

/// The default `topic bw --window`, the `ros2 topic bw` default.
const DEFAULT_BW_WINDOW: u64 = 100;

/// The largest `topic bw --window`: the window is allocated once up front.
const MAX_BW_WINDOW: u64 = 100_000;

// The verbs, as the `topic` verb's subcommands.
#[derive(Subcommand)]
enum TopicIntrospectAction {
    /// Print the message type of a topic
    ///
    /// Reads one message from a LOCAL topic or, if the name is not a local
    /// topic, a REMOTE robot's topic discovered on the LAN, and prints its
    /// type as `package/Type` (a workspace schema with no package prints its
    /// bare name). Pass `--local` to read genuine local producers
    /// without remote discovery or demand.
    Type {
        /// Read genuine local shared-memory producers only.
        #[arg(long)]
        local: bool,
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
    },
    /// List the topics that carry a message type
    ///
    /// Takes `package/Type`, the ROS 2 form `package/msg/Type` or a bare
    /// workspace `Type` and prints, one per line and sorted, the topics of
    /// that type: every LOCAL topic
    /// (typed from one message) and every REMOTE robot's topic `topic list`
    /// discovers (typed from its catalog). A type no topic carries prints
    /// nothing and exits 0. Pass `--connect` or `--listen` locators to reach
    /// robots discovery cannot find, as `topic list` takes them; pass
    /// `--local` to read genuine local producers only, without remote
    /// discovery.
    Find {
        /// Read genuine local shared-memory producers only.
        #[arg(long)]
        local: bool,
        /// Additional zenoh locator to connect to for discovery (repeatable),
        /// for example `tcp/192.0.2.10:7683` (7683 is the default gateway
        /// port). Adds to the discovered peers.
        #[arg(long, value_name = "LOCATOR")]
        connect: Vec<String>,
        /// Additional zenoh locator to listen on for discovery (repeatable),
        /// for example `tcp/0.0.0.0:7447`. Adds to the default discovery session.
        #[arg(long, value_name = "LOCATOR")]
        listen: Vec<String>,
        /// Message type, for example `geometry_msgs/Vector3`
        #[arg(value_name = "TYPE", add = ArgValueCandidates::new(completion::schema_names))]
        type_name: String,
    },
    /// Measure the bandwidth of a topic
    ///
    /// Prints, once a second, the bytes per second of the topic's message
    /// payloads over the last `--window` messages, with their mean, smallest
    /// and largest size. Reads a LOCAL topic or a REMOTE robot's topic like
    /// `topic hz`; pass `--local` to read genuine local producers only.
    Bw {
        /// Read genuine local shared-memory producers only.
        #[arg(long)]
        local: bool,
        /// How many of the newest messages the figure covers (2 to 100000).
        #[arg(
            long,
            value_name = "N",
            default_value_t = DEFAULT_BW_WINDOW,
            value_parser = clap::value_parser!(u64).range(2..=MAX_BW_WINDOW)
        )]
        window: u64,
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
    },
}

/// The `topic type` / `find` / `bw` verbs.
pub(crate) struct TopicIntrospect;

impl NestedVerbs for TopicIntrospect {
    fn parent(&self) -> &'static str {
        PARENT
    }

    fn commands(&self) -> Vec<Command> {
        super::derived_verbs::<TopicIntrospectAction>()
    }

    /// Quiet like every `topic` verb: the output is the data.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    /// The same rule as every `topic` verb.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = TopicIntrospectAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        super::super::exit_code_of(run(action))
    }
}

fn run(action: TopicIntrospectAction) -> CliResult<()> {
    // Workspace-optional, like `topic info`: the workspace `schemas/` name
    // workspace types, and outside one the built-ins still resolve. A
    // workspace that cannot be read is an error, never a quiet fallback.
    let ws = crate::discover_optional_workspace()?;
    let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
    match action {
        TopicIntrospectAction::Type { local, topic } => {
            let name = introspect::topic_type_with_scope(
                &topic,
                schemas_dir,
                TopicScope::from_local(local),
            )?;
            introspect::print_lines(&mut std::io::stdout().lock(), [name.as_str()])
        }
        TopicIntrospectAction::Find {
            local,
            connect,
            listen,
            type_name,
        } => {
            let found = introspect::topic_find(
                &type_name,
                schemas_dir,
                TopicScope::from_local(local),
                connect,
                listen,
            )?;
            // `topic find ... | head -1` closes the pipe early: that ends
            // the list, it is not a failure.
            introspect::print_lines(
                &mut std::io::stdout().lock(),
                found.topics.iter().map(String::as_str),
            )?;
            // The `topic list` rule: a failed remote query is one loud
            // stderr line, and the local result still stands. Each topic or
            // robot the search could not read is named the same way.
            for note in [
                &found.remote_unavailable,
                &found.unread_local,
                &found.silent_local,
                &found.unsearched_mirrors,
                &found.unsearched_robots,
            ]
            .into_iter()
            .flatten()
            {
                eprint!("{note}");
            }
            Ok(())
        }
        TopicIntrospectAction::Bw {
            local,
            window,
            topic,
        } => {
            let running = crate::setup_ctrlc_handler()?;
            // clap holds the window to 2..=MAX_BW_WINDOW, so both conversions hold.
            let window = usize::try_from(window)
                .ok()
                .and_then(NonZeroUsize::new)
                .unwrap_or(NonZeroUsize::MIN);
            introspect::topic_bw_with_scope(
                &topic,
                schemas_dir,
                running,
                &mut std::io::stdout(),
                TopicScope::from_local(local),
                window,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(argv: &[&str]) -> Result<TopicIntrospectAction, clap::Error> {
        let matches = crate::families::cli_command().try_get_matches_from(argv)?;
        let (_, topic) = matches.subcommand().expect("a verb");
        TopicIntrospectAction::from_arg_matches(topic)
    }

    #[test]
    fn topic_type_takes_a_topic_and_local() {
        match parse(&["cerulion", "topic", "type", "/a/cmd", "--local"]).unwrap() {
            TopicIntrospectAction::Type { local, topic } => {
                assert!(local);
                assert_eq!(topic, "/a/cmd");
            }
            _ => panic!("expected type"),
        }
    }

    #[test]
    fn topic_find_takes_a_type_name() {
        match parse(&["cerulion", "topic", "find", "geometry_msgs/msg/Vector3"]).unwrap() {
            TopicIntrospectAction::Find {
                local,
                connect,
                listen,
                type_name,
            } => {
                assert!(!local);
                assert!(connect.is_empty() && listen.is_empty());
                assert_eq!(type_name, "geometry_msgs/msg/Vector3")
            }
            _ => panic!("expected find"),
        }
        // `--connect` and `--listen` accumulate in order, as on `topic list`.
        match parse(&[
            "cerulion",
            "topic",
            "find",
            "std_msgs/String",
            "--connect",
            "tcp/192.0.2.10:7683",
            "--listen",
            "tcp/0.0.0.0:7447",
            "--connect",
            "tcp/192.0.2.11:7683",
        ])
        .unwrap()
        {
            TopicIntrospectAction::Find {
                connect, listen, ..
            } => {
                assert_eq!(connect, ["tcp/192.0.2.10:7683", "tcp/192.0.2.11:7683"]);
                assert_eq!(listen, ["tcp/0.0.0.0:7447"]);
            }
            _ => panic!("expected find"),
        }
        match parse(&["cerulion", "topic", "find", "std_msgs/String", "--local"]).unwrap() {
            TopicIntrospectAction::Find { local, .. } => assert!(local),
            _ => panic!("expected find"),
        }
    }

    #[test]
    fn topic_bw_defaults_to_a_window_of_one_hundred() {
        match parse(&["cerulion", "topic", "bw", "/a/cmd"]).unwrap() {
            TopicIntrospectAction::Bw {
                local,
                window,
                topic,
            } => {
                assert!(!local);
                assert_eq!(window, 100);
                assert_eq!(topic, "/a/cmd");
            }
            _ => panic!("expected bw"),
        }
        match parse(&["cerulion", "topic", "bw", "/a/cmd", "--window", "2"]).unwrap() {
            TopicIntrospectAction::Bw { window, .. } => assert_eq!(window, 2),
            _ => panic!("expected bw"),
        }
    }

    #[test]
    fn topic_bw_refuses_a_window_that_cannot_give_a_rate_or_is_too_large() {
        for bad in ["0", "1", "100001", "many"] {
            let err = parse(&["cerulion", "topic", "bw", "/a/cmd", "--window", bad])
                .err()
                .unwrap_or_else(|| panic!("--window {bad} was accepted"));
            assert_eq!(err.kind(), clap::error::ErrorKind::ValueValidation, "{bad}");
        }
    }

    #[test]
    fn the_new_topic_verbs_are_quiet_and_gated_like_the_built_in_ones() {
        for argv in [
            &["cerulion", "topic", "type", "/t"][..],
            &["cerulion", "topic", "find", "std_msgs/String"],
            &["cerulion", "topic", "bw", "/t"],
        ] {
            let matches = crate::families::cli_command()
                .try_get_matches_from(argv)
                .expect("parse");
            let (verbs, sub) = super::super::find(super::super::NESTED, &matches)
                .unwrap_or_else(|| panic!("{argv:?} is not a nested verb"));
            assert_eq!(verbs.log_class(sub), VerbLogClass::OneShot, "{argv:?}");
            assert!(verbs.needs_identity(sub), "{argv:?}");
        }
    }
}
