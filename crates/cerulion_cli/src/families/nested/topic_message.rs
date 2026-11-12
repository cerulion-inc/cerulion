// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion topic pub` and `topic snapshot`: publish a message given as
//! JSON or YAML text, and save the newest frame of a camera topic as a
//! JPEG, added under the built-in `topic` verb (see [`super`]). The command
//! logic lives in `cerulion_cli_engine::topic_cmd::{publish, snapshot}`.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};
use clap_complete::engine::ArgValueCandidates;

use cerulion_cli_engine::message_text::VerbError;
use cerulion_cli_engine::topic_cmd::{publish, snapshot, TopicScope};

use super::NestedVerbs;
use crate::cli::VerbLogClass;
use crate::completion;
use crate::network_switch::{self, NetworkRefusal, SwitchEnv};

/// The verb these verbs are added under (read by the docs gate too).
const PARENT: &str = "topic";

// The verbs, as the `topic` verb's subcommands.
#[derive(Subcommand)]
enum TopicMessageAction {
    /// Publish a message given as JSON or YAML
    ///
    /// Encodes MESSAGE (a JSON or YAML mapping, `ros2 topic pub` style) as
    /// TYPE, waits up to `--wait-subscribers` seconds for a subscriber,
    /// publishes, and prints `published: topic=.. type=.. frames=..
    /// subscribers=..`. A field the message leaves out takes its `.msg`
    /// default or zero. Unless `--local`, a `/`-rooted topic is also offered on the
    /// LAN through cerulion-netd, so a remote subscriber can demand it.
    /// Ctrl+C before the first frame exits 1.
    /// Exit 1 for a value that does not fit its field, 2 for a usage error
    /// or an unknown type.
    Pub {
        /// Publish on local shared memory only: no remote type lookup and
        /// no LAN offer.
        #[arg(long)]
        local: bool,
        /// Run on the paired robot NAME (a `~/.cerulion/robots.toml` name or
        /// endpoint id) through its ops plane instead of on this machine.
        #[arg(long, value_name = "NAME", conflicts_with = "local",
              add = ArgValueCandidates::new(completion::robot_names))]
        robot: Option<String>,
        /// How many frames to publish
        #[arg(long, value_name = "N", default_value_t = 1,
              value_parser = clap::value_parser!(u64).range(1..))]
        times: u64,
        /// Publish rate in Hz when `--times` is above 1 (default 1)
        #[arg(long, value_name = "HZ")]
        rate: Option<f64>,
        /// Seconds to wait for a first subscriber before publishing
        #[arg(long, value_name = "SECS", default_value_t = publish::DEFAULT_WAIT_SUBSCRIBERS.as_secs_f64())]
        wait_subscribers: f64,
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
        /// Message type: `package/Type` or `package/msg/Type`
        #[arg(value_name = "TYPE", add = ArgValueCandidates::new(completion::schema_names))]
        type_name: String,
        /// The message as a JSON or YAML mapping, for example
        /// `{"linear": {"x": 0.2}}`
        message: String,
    },
    /// Save the newest frame of a camera topic as a JPEG
    ///
    /// Takes the newest `sensor_msgs/Image` frame (yuv422_yuy2 or yuyv,
    /// rgb8, bgr8, rgba8, bgra8, mono8) or `sensor_msgs/CompressedImage`
    /// JPEG on TOPIC, writes a JPEG and prints one JSON line with its path,
    /// width, height, encoding, stamp_ns and bytes. Reads a LOCAL topic or
    /// a REMOTE robot's topic like `topic echo`. Exit 3 when the topic does
    /// not open or sends no frame within `--timeout`, 1 for an unsupported
    /// encoding.
    Snapshot {
        /// Read genuine local shared-memory producers only.
        #[arg(long)]
        local: bool,
        /// Run on the paired robot NAME (a `~/.cerulion/robots.toml` name or
        /// endpoint id) through its ops plane instead of on this machine.
        #[arg(long, value_name = "NAME", conflicts_with = "local",
              add = ArgValueCandidates::new(completion::robot_names))]
        robot: Option<String>,
        /// Where to write the JPEG (default: a new file in a new private
        /// directory under the system temp directory)
        #[arg(short = 'o', long = "output", value_name = "PATH")]
        output: Option<PathBuf>,
        /// Seconds to wait for a frame
        #[arg(long, value_name = "SECS", default_value_t = snapshot::DEFAULT_SNAPSHOT_TIMEOUT.as_secs_f64())]
        timeout: f64,
        /// JPEG quality, 1 to 100
        #[arg(long, value_name = "Q", default_value_t = snapshot::DEFAULT_JPEG_QUALITY,
              value_parser = clap::value_parser!(u8).range(1..=100))]
        quality: u8,
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
    },
}

/// The `topic pub` / `snapshot` verbs.
pub(crate) struct TopicMessage;

impl NestedVerbs for TopicMessage {
    fn parent(&self) -> &'static str {
        PARENT
    }

    fn commands(&self) -> Vec<Command> {
        super::derived_verbs::<TopicMessageAction>()
    }

    /// Quiet like every `topic` verb: the output is the data.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::OneShot
    }

    /// `--robot` reaches the robot, so the switch refuses it; without
    /// `--robot` the verb is local and runs.
    fn network_refusal(&self, matches: &ArgMatches, env: &SwitchEnv) -> Option<NetworkRefusal> {
        use cerulion_cli_engine::robot_act::{TOPIC_PUBLISH_VERB, TOPIC_SNAPSHOT_VERB};
        let (robot, verb) = match TopicMessageAction::from_arg_matches(matches).ok()? {
            TopicMessageAction::Pub { robot, .. } => (robot, TOPIC_PUBLISH_VERB),
            TopicMessageAction::Snapshot { robot, .. } => (robot, TOPIC_SNAPSHOT_VERB),
        };
        network_switch::robot_refusal(robot.as_deref(), verb, env)
    }

    /// The same rule as every `topic` verb.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        true
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = TopicMessageAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        verb_exit_code(run(action))
    }
}

/// Print a verb's one output line and exit 0, or its error and the error's
/// exit code.
pub(crate) fn verb_exit_code(result: Result<String, VerbError>) -> ExitCode {
    match result {
        Ok(line) => {
            println!("{line}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::from(e.exit_code())
        }
    }
}

/// A seconds flag as a duration; a negative, NaN or infinite value is a
/// usage error naming the flag.
pub(crate) fn seconds(flag: &str, secs: f64) -> Result<Duration, VerbError> {
    Duration::try_from_secs_f64(secs).map_err(|_| {
        VerbError::Usage(format!(
            "{flag} must be a number of seconds, 0 or more, got {secs}"
        ))
    })
}

fn run(action: TopicMessageAction) -> Result<String, VerbError> {
    let ws = crate::discover_optional_workspace()?;
    let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
    let running = crate::setup_ctrlc_handler()?;
    match action {
        TopicMessageAction::Pub {
            local,
            robot,
            times,
            rate,
            wait_subscribers,
            topic,
            type_name,
            message,
        } => {
            let req = publish::PubRequest {
                topic: &topic,
                type_name: &type_name,
                message: &message,
                times,
                rate_hz: rate,
                wait_subscribers: seconds("--wait-subscribers", wait_subscribers)?,
                scope: TopicScope::from_local(local),
            };
            match robot {
                Some(robot) => cerulion_cli_engine::robot_act::robot_pub(&robot, &req),
                None => publish::topic_pub(&req, schemas_dir, &running).map(|r| r.line()),
            }
        }
        TopicMessageAction::Snapshot {
            local,
            robot,
            output,
            timeout,
            quality,
            topic,
        } => {
            let req = snapshot::SnapshotRequest {
                topic: &topic,
                output: output.as_deref(),
                timeout: seconds("--timeout", timeout)?,
                quality,
                scope: TopicScope::from_local(local),
            };
            match robot {
                Some(robot) => cerulion_cli_engine::robot_act::robot_snapshot(&robot, &req),
                None => {
                    snapshot::topic_snapshot(&req, schemas_dir, &running).map(|r| r.json_line())
                }
            }
        }
    }
}
