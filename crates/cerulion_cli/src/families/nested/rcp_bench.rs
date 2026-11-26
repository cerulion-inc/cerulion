// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion rcp bench`: rerun one benchmark task with and without the
//! Cerulion harness against the user's own model, or (`--verify`) check that
//! the harness on disk is the measured code. Added under the `rcp` family (see
//! [`super`]); the harness folder and the launcher plan live in
//! `cerulion_cli_engine::harness_cmd`, and the run is the family's own
//! [`crate::families::rcp::run_harness`].

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{ArgMatches, Command, FromArgMatches, Subcommand};

use cerulion_cli_engine::harness_cmd::{BenchArgs, HarnessVerb};

use super::NestedVerbs;
use crate::cli::VerbLogClass;
use crate::network_switch::{NetworkRefusal, SwitchEnv};

/// The verb these verbs are added under (read by the docs gate too).
const PARENT: &str = "rcp";

// The verb, as one of the `rcp` verb's subcommands.
#[derive(Subcommand)]
enum RcpBenchAction {
    /// Rerun a benchmark task with and without the Cerulion harness
    ///
    /// Runs one LIBERO-PRO cell (one task at one start state) twice against
    /// your model: once through the benchmark's published three-tool
    /// interface and once through the Cerulion harness's tools, and prints the
    /// benchmark's own verdict for each. The endpoint is any OpenAI-compatible
    /// chat completions URL that takes tool calls and images; the key is read
    /// from your environment and sent only to that URL. The first run installs
    /// the simulator and the benchmark scenes once. One episode is one draw,
    /// so a single pair can go either way.
    ///
    /// `--verify` checks every file of the harness's measured code against the
    /// manifests of the code the published benchmark ran, and exits 0 only when
    /// all of them match.
    Bench {
        /// Check that the harness code on disk is the measured code, and run
        /// nothing else.
        #[arg(long, conflicts_with_all = ["model", "base_url", "key_env", "cell", "out"])]
        verify: bool,
        /// The model name your endpoint serves.
        #[arg(long, value_name = "MODEL", required_unless_present = "verify")]
        model: Option<String>,
        /// The endpoint's OpenAI-compatible API root. Default: `OPENAI_BASE_URL`.
        #[arg(long, value_name = "URL", value_hint = clap::ValueHint::Url)]
        base_url: Option<String>,
        /// The environment variable that holds your key. Default:
        /// `OPENAI_API_KEY`.
        #[arg(long, value_name = "NAME")]
        key_env: Option<String>,
        /// The benchmark cell to run, as the results package names it.
        /// Default: the harness's development cell.
        #[arg(long, value_name = "ID")]
        cell: Option<String>,
        /// The folder the two episodes' results are written to. Default: a
        /// new `cerulion-bench-<UTC time>` folder in the current directory.
        #[arg(long, value_name = "DIR", value_hint = clap::ValueHint::DirPath)]
        out: Option<PathBuf>,
    },
}

/// The `rcp bench` verb.
pub(crate) struct RcpBench;

impl NestedVerbs for RcpBench {
    fn parent(&self) -> &'static str {
        PARENT
    }

    fn commands(&self) -> Vec<Command> {
        super::derived_verbs::<RcpBenchAction>()
    }

    /// The run prints its own progress and lasts minutes.
    fn log_class(&self, _matches: &ArgMatches) -> VerbLogClass {
        VerbLogClass::LongRunning
    }

    /// The bench reaches the user's own model endpoint and, on its first run,
    /// the benchmark's and the Python packages' own hosts: no robot and no
    /// Cerulion service, so the switch does not refuse it.
    fn network_refusal(&self, _matches: &ArgMatches, _env: &SwitchEnv) -> Option<NetworkRefusal> {
        None
    }

    /// Local benchmark / verification reaches no Cerulion service or robot.
    fn needs_identity(&self, _matches: &ArgMatches) -> bool {
        false
    }

    fn run(&self, matches: &ArgMatches) -> ExitCode {
        let action = RcpBenchAction::from_arg_matches(matches).unwrap_or_else(|e| e.exit());
        crate::families::rcp::run_harness(&verb_of(action))
    }
}

/// The harness verb a parsed `rcp bench` runs.
fn verb_of(action: RcpBenchAction) -> HarnessVerb {
    let RcpBenchAction::Bench {
        verify,
        model,
        base_url,
        key_env,
        cell,
        out,
    } = action;
    if verify {
        return HarnessVerb::Verify;
    }
    HarnessVerb::Bench(BenchArgs {
        // clap requires `--model` unless `--verify` is present.
        model: model.unwrap_or_default(),
        base_url,
        key_env,
        cell,
        out,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `argv` parsed by the production command, converted the way
    /// [`RcpBench::run`] converts the `rcp` sub-matches it is handed.
    fn parsed(argv: &[&str]) -> Result<HarnessVerb, clap::Error> {
        let matches = crate::families::cli_command().try_get_matches_from(argv)?;
        let (_, rcp) = matches.subcommand().expect("rcp");
        Ok(verb_of(RcpBenchAction::from_arg_matches(rcp)?))
    }

    #[test]
    fn bench_parses_every_flag_into_the_harness_verb() {
        let verb = parsed(&[
            "cerulion",
            "rcp",
            "bench",
            "--model",
            "m",
            "--base-url",
            "https://example.test/v1",
            "--key-env",
            "MY_KEY",
            "--cell",
            "pro_spatial_swap_t00_i00",
            "--out",
            "results",
        ])
        .expect("parses");
        assert_eq!(
            verb,
            HarnessVerb::Bench(BenchArgs {
                model: "m".to_string(),
                base_url: Some("https://example.test/v1".to_string()),
                key_env: Some("MY_KEY".to_string()),
                cell: Some("pro_spatial_swap_t00_i00".to_string()),
                out: Some(PathBuf::from("results")),
            })
        );
    }

    #[test]
    fn verify_alone_is_the_verify_verb_and_needs_no_model() {
        assert_eq!(
            parsed(&["cerulion", "rcp", "bench", "--verify"]).expect("parses"),
            HarnessVerb::Verify
        );
    }

    #[test]
    fn bench_without_a_model_or_verify_with_a_run_flag_is_a_usage_error() {
        for argv in [
            &["cerulion", "rcp", "bench"][..],
            &["cerulion", "rcp", "bench", "--verify", "--model", "m"][..],
            &["cerulion", "rcp", "bench", "--verify", "--out", "o"][..],
        ] {
            let err = parsed(argv).expect_err("refused");
            assert!(
                matches!(
                    err.kind(),
                    clap::error::ErrorKind::MissingRequiredArgument
                        | clap::error::ErrorKind::ArgumentConflict
                ),
                "{argv:?}: {err}"
            );
        }
    }
}
