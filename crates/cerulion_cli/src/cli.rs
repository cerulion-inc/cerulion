// SPDX-License-Identifier: AGPL-3.0-only
//! Clap CLI definitions.

use std::path::PathBuf;

use cerulion_cli_engine::graph_cmd::PeerLossPolicy as EnginePeerLossPolicy;
use cerulion_cli_engine::graph_cmd::TimeSource as EngineTimeSource;
use clap::{Parser, Subcommand, ValueEnum};
// `add = ArgValueCandidates::new(..)` attaches a live value source to
// an arg; `ArgValueCompleter` + `PathCompleter` do the same for paths where the
// extension is a hard contract. Both lower to clap's `Arg::add` extension seam
// (`clap/unstable-ext`).
use clap_complete::engine::{ArgValueCandidates, ArgValueCompleter, PathCompleter};

use crate::completion;

/// Clap-parseable mirror of the engine's [`EngineTimeSource`].
/// The engine crate does not depend on `clap`, so the
/// `ValueEnum` lives here and maps 1:1 to the canonical engine enum.
/// Value names are lowercased by clap's default `ValueEnum` renaming:
/// `real`, `external`, `virtual`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum TimeSource {
    /// Live, event-driven loop on the real clock (the default). Wakes within
    /// microseconds of a publish.
    Real,
    /// An external time master. Not wired yet: the clock starts at 0 and never
    /// advances, so time-based triggers never fire.
    External,
    /// Deterministic virtual clock driven by a 1 ms poll loop, for
    /// deterministic runs and benchmarking.
    Virtual,
}

impl From<TimeSource> for EngineTimeSource {
    fn from(ts: TimeSource) -> Self {
        match ts {
            TimeSource::Real => EngineTimeSource::Real,
            TimeSource::External => EngineTimeSource::External,
            TimeSource::Virtual => EngineTimeSource::Virtual,
        }
    }
}

/// The `--record-env` capture policy — the clap
/// `ValueEnum` mirror of the canonical engine enum
/// (`cerulion_cli_engine::graph_cmd::RecordEnvMode`), mapped 1:1 like
/// [`TimeSource`]. Value names lowercase: `allowlist` / `all` / `none`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum RecordEnv {
    /// Record every variable NAME, and VALUES only for CERULION_*, RUST_LOG
    /// and IOX2_*. Every other value is stored as a hash, so a re-execution
    /// can detect that it changed without the bag carrying your secrets. The
    /// default.
    Allowlist,
    /// Record values verbatim. Full fidelity, but the bag embeds whatever
    /// secrets were in your environment (warned loudly at record time).
    All,
    /// Record names only. A re-execution cannot compare environment values.
    None,
}

impl From<RecordEnv> for cerulion_cli_engine::graph_cmd::RecordEnvMode {
    fn from(re: RecordEnv) -> Self {
        use cerulion_cli_engine::graph_cmd::RecordEnvMode;
        match re {
            RecordEnv::Allowlist => RecordEnvMode::Allowlist,
            RecordEnv::All => RecordEnvMode::All,
            RecordEnv::None => RecordEnvMode::None,
        }
    }
}

/// The explicit `--record-cpu` VALUE: a core id or
/// the literal `none` (float). Flag ABSENT = the engine's `RecordCpu::Auto`
/// (see [`record_cpu_mode`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordCpuArg {
    /// Pin bagd to this core id.
    Core(u32),
    /// Do not pin — bagd floats.
    None,
}

/// clap value parser for `--record-cpu`: a `u32` core id or the literal
/// `none`. Anything else is a loud parse error naming both accepted forms.
fn parse_record_cpu(s: &str) -> Result<RecordCpuArg, String> {
    if s.eq_ignore_ascii_case("none") {
        return Ok(RecordCpuArg::None);
    }
    s.parse::<u32>()
        .map(RecordCpuArg::Core)
        .map_err(|_| format!("expected a core id (e.g. `--record-cpu=7`) or `none`, got `{s}`"))
}

/// Map the optional CLI value to the engine's [`RecordCpu`] — flag absent is
/// AUTO (the measured default: pin bagd to the highest core on >=4-core
/// Linux machines; float on capacity-bound small-core machines).
///
/// [`RecordCpu`]: cerulion_cli_engine::graph_cmd::RecordCpu
pub fn record_cpu_mode(arg: Option<RecordCpuArg>) -> cerulion_cli_engine::graph_cmd::RecordCpu {
    use cerulion_cli_engine::graph_cmd::RecordCpu;
    match arg {
        Option::None => RecordCpu::Auto,
        Some(RecordCpuArg::Core(n)) => RecordCpu::Core(n),
        Some(RecordCpuArg::None) => RecordCpu::None,
    }
}

/// Clap-parseable mirror of the engine's [`EnginePeerLossPolicy`] —
/// the multi-process worker-death policy. Same mirror pattern as
/// [`TimeSource`]: the engine crate does not depend on `clap`, so the
/// `ValueEnum` lives here and maps 1:1. Value names lowercase: `continue`,
/// `fail`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum PeerLoss {
    /// A crashed worker is dropped and the surviving workers keep running
    /// degraded, with a loud error naming the lost group. The run still exits
    /// 0 unless every worker crashed. The default.
    Continue,
    /// Any worker death stops the whole deployment. For CI and replay.
    Fail,
}

impl From<PeerLoss> for EnginePeerLossPolicy {
    fn from(p: PeerLoss) -> Self {
        match p {
            PeerLoss::Continue => EnginePeerLossPolicy::Continue,
            PeerLoss::Fail => EnginePeerLossPolicy::Fail,
        }
    }
}

#[derive(Parser)]
#[command(name = "cerulion", about = "Cerulion robotics middleware CLI", version)]
pub struct Cli {
    /// Enable verbose (debug-level) logging (`-v` or `--verbose`)
    //
    // `display_order` only moves where the flag is LISTED: a global flag is
    // otherwise printed second on every verb's page, between that verb's own
    // options.
    #[arg(global = true, short = 'v', long, display_order = 900)]
    pub verbose: bool,

    #[command(subcommand)]
    pub command: Commands,
}

/// The `cerulion completions` long help, verbatim.
///
/// A const rather than a doc comment because the zsh install line contains
/// `$+functions[compdef]` — see the variant's doc comment for why every other
/// spelling either breaks the docs gate or corrupts the text a user copies.
/// Written as a literal so what `--help` prints is exactly what is here, with
/// no clap re-wrapping of the install table.
const COMPLETIONS_LONG_ABOUT: &str = "\
Print the shell code that enables `cerulion` tab-completion.

Completes everything: subcommands, flags, and enum values from the command
tree, plus LIVE names: topic names for `topic echo/info/hz`, node types
for `node build/run/info/...`, graph names for `graph run/levels/partition/...`,
workspace + built-in ROS 2 schemas for `schema info`, and robot names for
`viz --robot`.

Install it once, per shell:

  zsh         echo 'autoload -Uz compinit && (( $+functions[compdef] )) || compinit' >> ~/.zshrc
              echo 'source <(COMPLETE=zsh cerulion)' >> ~/.zshrc
  bash        echo 'source <(COMPLETE=bash cerulion)' >> ~/.bashrc
  fish        echo 'COMPLETE=fish cerulion | source' > ~/.config/fish/completions/cerulion.fish
  elvish      echo 'eval (E:COMPLETE=elvish cerulion | slurp)' >> ~/.elvish/rc.elv
  powershell  echo '$env:COMPLETE = \"powershell\"; cerulion | Out-String | Invoke-Expression; Remove-Item Env:\\COMPLETE' >> $PROFILE

zsh needs BOTH lines: the completion script ends in `compdef`, which is a
function `compinit` defines, and macOS's system zshrc never calls it.
Without the guard a stock ~/.zshrc prints `command not found: compdef` at every
shell start and nothing completes. The guard is a no-op if a framework
(oh-my-zsh, prezto) already ran compinit.

These forms re-generate the shell code on demand, so they self-correct when
`cerulion` is upgraded or moved, which is why they are recommended.
`cerulion completions <shell> > FILE` writes the same code to a file instead;
re-run it after an upgrade.

Live names are read from LOCAL sources only (the workspace, the iceoryx2
service directory, ~/.cerulion) under a hard 150 ms budget, so a TAB press
never opens the network and never hangs.

A REMOTE robot's topics complete only while something HOLDS a mirror of them on
this desk. `cerulion viz --robot NAME` leaves one standing (the vizd daemon
keeps the demand); `topic echo/info/hz` holds one only for as long as that
command runs; `topic list` demands nothing at all. With no such attach, remote
topics do not complete: learning their names needs a network round-trip,
which a TAB press must not do.";

#[derive(Subcommand)]
pub enum Commands {
    /// Workspace management
    Workspace {
        #[command(subcommand)]
        action: WorkspaceAction,
    },
    /// Node management
    Node {
        #[command(subcommand)]
        action: NodeAction,
    },
    /// Graph management
    Graph {
        #[command(subcommand)]
        action: GraphAction,
    },
    /// Topic introspection
    Topic {
        #[command(subcommand)]
        action: TopicAction,
    },
    /// Schema management
    Schema {
        #[command(subcommand)]
        action: SchemaAction,
    },
    /// Removed: `cerulion ros attach` is now `cerulion ros2 attach`.
    ///
    /// The `ros` family folded into `ros2` (run, launch, attach, migrate). The
    /// old spelling fails with a message that names the new one.
    //
    // Attach was the `ros` family's only verb, so the whole family folded into
    // `ros2`. This stub exists so the OLD spelling fails LOUDLY with the
    // migration message (`main` intercepts it above the login gate) instead of
    // clap's generic "unrecognized subcommand". Hidden from help and
    // completions; no alias.
    //
    // `disable_help_flag` is load-bearing: clap answers `--help`/`-h` BEFORE
    // returning a parsed command, so without it `cerulion ros --help` served
    // clap's help page for the stub (exit 0) instead of the migration error.
    // With the auto flag gone, `--help` is just another hyphen value the
    // trailing positional swallows, and EVERY old spelling, help flags
    // included, reaches the exit-2 migration message. (`cerulion help ros`
    // still renders the doc comment above, which names the new spelling.)
    #[command(hide = true, disable_help_flag = true)]
    Ros {
        /// Swallows whatever followed (`attach --iface …`, `--help` included)
        /// so the user sees the migration error, never a clap parse error
        /// about the old verb's own flags.
        #[arg(
            trailing_var_arg = true,
            allow_hyphen_values = true,
            hide = true,
            value_name = "ARGS"
        )]
        args: Vec<String>,
    },
    /// ROS 2 interop: run, launch, attach to and migrate a ROS 2 stack.
    ///
    /// `ros2 run` and `ros2 launch` run stock ROS 2 entry points on Cerulion
    /// transport (`RMW_IMPLEMENTATION=rmw_cerulion`): swap the command, not the
    /// stack. `ros2 attach` bridges a live ROS 2 / DDS robot into a Cerulion
    /// graph. `ros2 migrate` rewrites a colcon workspace's publish sites to the
    /// loaned-message API.
    Ros2 {
        #[command(subcommand)]
        action: Ros2Action,
    },
    /// Visualize live topics in Cerulion Studio.
    ///
    /// `cerulion viz` is a thin client of the long-lived `cerulion-vizd` daemon.
    /// It makes sure the daemon is running (starting it in the background if
    /// absent) and asks the daemon to attach each topic. The daemon taps the
    /// topics, decodes them from their schemas and renders them. No graph YAML,
    /// node or script is needed.
    ///
    /// Each `TOPIC` is an absolute Cerulion topic (a missing leading `/` is
    /// added). Optionally pin a schema with `TOPIC=SCHEMA` (for example
    /// `/go2/cloud=sensor_msgs/PointCloud2`); a local topic's schema is
    /// otherwise learned from its first frame.
    ///
    /// Cerulion Studio is the viewer. The daemon hosts a Rerun gRPC proxy by
    /// default and advertises its URL; Studio connects to that same daemon and
    /// renders whatever is attached, so this verb starts no viewer itself.
    /// `$CERULION_RERUN_URL` is read once, when the daemon starts, and makes the
    /// daemon a client of an external Rerun endpoint instead of hosting one; it
    /// has no effect on a daemon that is already running.
    ///
    /// Without `--detach` the verb stays attached until Ctrl+C, then detaches
    /// only the topics it added (the shared daemon keeps running). Run it with
    /// no arguments to attach every decodable local topic the daemon discovers
    /// (silent and undecodable topics are skipped).
    Viz {
        /// Cerulion topics to visualize, each `TOPIC` or `TOPIC=SCHEMA`.
        // Completes the bare `TOPIC` form. The `TOPIC=SCHEMA` pin is
        // not completed past the `=`: a schema source here would have to know
        // which topic precedes it, and the pin exists precisely for the case
        // where the type is NOT locally resolvable.
        #[arg(value_name = "TOPIC", add = ArgValueCandidates::new(completion::topics))]
        topics: Vec<String>,
        /// Visualize a REMOTE robot's topics over the network.
        ///
        /// The daemon demands each topic from the robot named NAME; the robot's
        /// frames are mirrored into this machine's shared memory and rendered
        /// from there, so the robot serves raw bytes and nothing else. On a
        /// typical LAN the robot is found by scouting with no configuration, and
        /// no schema pin is needed: the daemon resolves each topic's ROS type
        /// from the robot's catalog and fetches any custom schema over the
        /// network. You may still pin `TOPIC=SCHEMA` (for example
        /// `/utlidar/cloud=sensor_msgs/PointCloud2`) to skip the catalog lookup.
        /// A topic the robot does not serve is an error, not a silent skip.
        #[arg(long, value_name = "NAME", add = ArgValueCandidates::new(completion::robot_names))]
        robot: Option<String>,
        /// Additional zenoh gateway locator to CONNECT to, for a robot scouting
        /// cannot reach, for example `tcp/192.0.2.10:7683`. Repeatable. Requires
        /// `--robot`. Used when this command STARTS the daemon; a daemon that is
        /// already running keeps the locators it started with (scouting covers
        /// the LAN regardless, and the verb says so).
        #[arg(long, value_name = "LOCATOR", requires = "robot")]
        connect: Vec<String>,
        /// Local zenoh locator to LISTEN on for remote discovery and ingress, for
        /// example `tcp/0.0.0.0:7447`. Repeatable. Requires `--robot`. Used when
        /// this command STARTS the daemon; a daemon that is already running keeps
        /// the locators it started with.
        #[arg(long, value_name = "LOCATOR", requires = "robot")]
        listen: Vec<String>,
        /// Attach the topics to the long-lived `cerulion-vizd` daemon and RETURN
        /// immediately, leaving the daemon running. Without it, `cerulion viz`
        /// stays attached until Ctrl+C, then detaches the topics it added (the
        /// daemon keeps running for the next run).
        #[arg(long)]
        detach: bool,
    },
    /// Connect to a remote robot and mirror its topics into local shared memory.
    ///
    /// Dials the robot, demands the topics you name and re-injects each into
    /// this machine's shared memory, so `topic echo` and `viz` see them as
    /// local topics. Runs until Ctrl+C.
    ///
    /// Runs the `cerulion-connectd` helper binary. The release packages place
    /// it beside `cerulion`; a source build gets it from
    /// `cargo build -p cerulion_connectd`, and `CERULION_CONNECTD_BIN` points
    /// at one anywhere else.
    Connect {
        /// The robot: a 64-char-hex endpoint id, OR a name pinned in
        /// `~/.cerulion/robots.toml` (`[robots]` table). `cerulion pair <robot>`
        /// writes that pin, so after pairing this name just works. Omit when
        /// using `--eid`.
        // The pins in `robots.toml` are one of `robot_names`' three
        // sources, so a paired robot completes here by construction.
        #[arg(value_name = "ROBOT", add = ArgValueCandidates::new(completion::robot_names))]
        robot: Option<String>,
        /// The robot's iroh endpoint id (64-char hex). Overrides a positional
        /// name. From `cerulion pair`, the robot's mDNS TXT `eid=`, or the operator.
        #[arg(long, value_name = "HEX")]
        eid: Option<String>,
        /// A direct `ip:port` for the robot (LAN direct-dial). Repeatable. When
        /// omitted, the eid is resolved via relay/discovery.
        #[arg(long = "addr", value_name = "IP:PORT")]
        addrs: Vec<String>,
        /// A topic to demand + re-inject, e.g. `/utlidar/cloud`. Repeatable. With
        /// no `--topic` and no `--all`, the catalog is printed and NOTHING is
        /// demanded (the discoverable default).
        #[arg(long = "topic", value_name = "TOPIC")]
        topics: Vec<String>,
        /// Demand + re-inject EVERY topic the robot's catalog lists.
        #[arg(long)]
        all: bool,
        /// The desk device key file (32 raw bytes: an ed25519 seed). When
        /// omitted, the paired desk key `~/.cerulion/desk.key` is used if it
        /// exists (written by `cerulion pair`), else an EPHEMERAL key, which an
        /// unpaired robot REFUSES, so pair the desk first with `cerulion pair`.
        #[arg(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        key_file: Option<PathBuf>,
        /// Materialize fetched `.msg`/YAML schemas into this dir so `topic echo` /
        /// `viz` can decode a never-seen type. When omitted, materialization is
        /// skipped (built-in types still decode).
        #[arg(long, value_name = "DIR", value_hint = clap::ValueHint::DirPath)]
        schemas_dir: Option<PathBuf>,
        /// Self-hosted relay URL. Default: n0 public relays.
        #[arg(long, value_name = "URL")]
        relay_url: Option<String>,
        /// Disable all relays (LAN / direct-dial only).
        #[arg(long)]
        relay_disabled: bool,
        /// Kill-switch: pass `off` to refuse to connect.
        #[arg(long)]
        network: Option<String>,
    },
    /// Pair this desk with a robot.
    ///
    /// Runs the code-pairing ceremony with the robot, so the robot adds this
    /// desk's device key to its access list, then records the robot's name and
    /// endpoint id in `~/.cerulion/robots.toml` so `cerulion connect <robot>`
    /// works afterwards with no flags.
    ///
    /// Runs the `cerulion-connectd` helper binary. The release packages place
    /// it beside `cerulion`; a source build gets it from
    /// `cargo build -p cerulion_connectd`, and `CERULION_CONNECTD_BIN` points
    /// at one anywhere else.
    ///
    /// The robot owner starts code pairing on the robot and shares a short code;
    /// you supply it here (`--code`, else typed at the prompt or piped on stdin).
    /// The desk device key is created at `~/.cerulion/desk.key` on the first pair
    /// (0600 permissions) and reused afterwards. An existing key is never
    /// overwritten.
    ///
    /// STDOUT carries deterministic, machine-parseable state lines:
    /// `pairing: robot=<name> eid=<hex>`, then on success
    /// `paired: robot=<name> eid=<hex> account=<hex>` and
    /// `pinned: name=<name> eid=<hex>`. STDERR carries the logs and the
    /// interactive code prompt.
    ///
    /// The exit code is the primary signal: 0 = paired, 1 = usage or config
    /// error, 2 = the robot refused (no armed code, denied, or the window
    /// expired), 3 = unreachable (dial failure or timeout), 4 = wrong pairing
    /// code or attempts exhausted, 130 = interrupted (Ctrl-C or SIGTERM during
    /// the ceremony; no pairing occurred).
    Pair {
        /// The robot: a 64-char-hex endpoint id, OR a name resolved via the
        /// robot's mDNS TXT `eid=` record, else `~/.cerulion/robots.toml`. Omit
        /// when using `--eid`.
        // A robot seen on this LAN lands in the peer cache, so the
        // FIRST pairing completes too, not only a re-pair of a pinned robot.
        #[arg(value_name = "ROBOT", add = ArgValueCandidates::new(completion::robot_names))]
        robot: Option<String>,
        /// The robot's iroh endpoint id (64-char hex). Overrides a positional
        /// name (no name is pinned). From the robot operator or its mDNS `eid=`.
        #[arg(long, value_name = "HEX")]
        eid: Option<String>,
        /// A direct `ip:port` for the robot (LAN direct-dial). Repeatable. When
        /// omitted, the eid is resolved via relay/discovery.
        #[arg(long = "addr", value_name = "IP:PORT")]
        addrs: Vec<String>,
        /// The short pairing code the robot owner shared. PREFER omitting this
        /// and typing the code at the prompt (or piping it on stdin): a code
        /// passed via `--code` is VISIBLE in process listings (`ps`) for the
        /// ceremony window, and it authorizes durable enrollment.
        #[arg(long, value_name = "CODE")]
        code: Option<String>,
        /// The account (64-char hex) this pairing is FOR. When omitted, a
        /// self-account is derived from the desk device key (a desk with a
        /// keypair and no Cerulion account).
        #[arg(long, value_name = "HEX")]
        account: Option<String>,
        /// The access-list label the robot stores on its pairing row (so the
        /// owner recognizes this desk). Default: this machine's hostname.
        #[arg(long, value_name = "NAME")]
        label: Option<String>,
        /// The desk device key file (32 raw bytes: an ed25519 seed). Default:
        /// `~/.cerulion/desk.key` (created 0600 on the first pair, reused after).
        #[arg(long, value_name = "PATH", value_hint = clap::ValueHint::AnyPath)]
        key_file: Option<PathBuf>,
        /// Self-hosted relay URL. Default: n0 public relays.
        #[arg(long, value_name = "URL")]
        relay_url: Option<String>,
        /// Disable all relays (LAN / direct-dial only).
        #[arg(long)]
        relay_disabled: bool,
    },
    /// Sign in to a Cerulion account.
    ///
    /// Runs the device-code flow (RFC 8628): prints a short code and a
    /// verification URL to open in a browser on any device, then waits for you
    /// to authorize, so it works on a headless machine. Use it any time to sign
    /// in, re-authenticate or switch accounts.
    Login,
    /// Manage your Cerulion account.
    ///
    /// Currently: your devices (`cerulion account devices list` and
    /// `cerulion account devices revoke <device_id>`), for managing your own
    /// desks. Robot access is managed in Studio and on the web account page,
    /// not in the CLI.
    Account {
        #[command(subcommand)]
        action: AccountAction,
    },
    /// Interactive terminal dashboard
    Tui,
    /// Inspect legacy publish-trace files (`trace_*.jsonl`).
    ///
    /// Reads the JSON Lines records from a directory and prints a
    /// human-readable timeline. These are not recordings: a `graph run --record`
    /// recording is an MCAP bag, read with `cerulion bag info` and
    /// `cerulion bag play`.
    Trace {
        #[command(subcommand)]
        action: TraceAction,
    },
    // There is no `Replay` variant, not even a hidden one that parses the
    // removed verb in order to diagnose it:
    // `cerulion replay` is clap's plain `unrecognized subcommand`
    // (exit 2, the usage code). Pinned by
    // `removed_verb_tests::removed_replay_verb_is_an_unknown_subcommand`.
    /// Record topics into a bag, inspect one, and play one back.
    ///
    /// `bag play` executes NOTHING: it re-publishes the recorded frames verbatim
    /// under their recorded topic names, wall-paced, so the desk-side tools
    /// (`topic list`/`echo`/`hz`, `cerulion viz`, Studio) see what a live robot
    /// would publish. To RE-EXECUTE a bagged graph against your workspace's
    /// current node builds, add `--resim all`; add `--verify` on top of that to
    /// byte-compare the result against the recording.
    ///
    /// `bag record` produces such a bag from LOCAL topics only: run it on the
    /// machine that produces the data (the robot), then transfer the file.
    /// Together: record on the robot, play on your desk.
    Bag {
        #[command(subcommand)]
        action: BagAction,
    },
    /// Print the shell code that enables `cerulion` tab-completion.
    ///
    /// The full help (per-shell install lines + the guarantees) is
    /// [`COMPLETIONS_LONG_ABOUT`], kept OUT of this doc comment on purpose:
    /// the zsh line contains `$+functions[compdef]`, and rustdoc reads
    /// `[compdef]` as an intra-doc link (the `RUSTDOCFLAGS=-D warnings` gate
    /// fails). Escaping the brackets would satisfy rustdoc while putting
    /// literal backslashes into the `--help` text a user copies, and indenting
    /// it into a code block makes rustdoc try to compile it as Rust. An
    /// explicit `long_about` is the only form that keeps the help EXACT.
    #[command(long_about = COMPLETIONS_LONG_ABOUT)]
    Completions {
        /// Shell to emit completion code for.
        shell: completion::CompletionShell,
    },
    /// Capture the moment: save the last ~30 seconds plus the next ~15 as a bag.
    ///
    /// The capture (a Flashback) comes from the rolling window every serving
    /// graph holds. Nobody has to arm anything in advance: the window is always
    /// on. The capture lands in `recordings/flashbacks/` and this command waits
    /// for it, so you are handed a file rather than a promise. Turn the whole
    /// plane off with `CERULION_FLASHBACK=off`.
    #[cfg(unix)]
    Flashback {
        /// A note recorded into the capture, so a bag found three weeks later
        /// says what it was about.
        #[arg(long, value_name = "TEXT")]
        note: Option<String>,
        /// Exclude this capture from retention eviction, so it cannot rotate
        /// away.
        #[arg(long)]
        pin: bool,
        /// Return as soon as the capture is accepted, without waiting for the
        /// bag. The accepted line already carries the path it will have.
        #[arg(long)]
        no_wait: bool,
    },
    /// Capture the moment (unsupported on this platform: the recorder that
    /// holds the rolling window is Unix-only).
    #[cfg(not(unix))]
    Flashback,
    /// Remove the shared-memory bookkeeping that dead Cerulion processes left behind.
    ///
    /// Sweeps iceoryx2's bookkeeping under `/tmp/iceoryx2/` for DEAD nodes only;
    /// the state of every live node is preserved. Useful after a crash or a
    /// `kill -9`: what a dead process left behind can make a graph fail to
    /// publish, or make topics look missing.
    ///
    /// On macOS and FreeBSD it also reports the `/tmp/*.shm_state` population (files
    /// iceoryx2 leaves behind when a process is killed, which slow every later
    /// sweep) and reclaims the ones whose creating process is provably gone.
    Clean {
        /// Report the `.shm_state` population and what is reclaimable, but
        /// delete nothing. The dead-node sweep still runs.
        #[arg(long)]
        report_only: bool,
    },
    /// The recorder daemon: taps live topics into a standard MCAP bag.
    ///
    /// `cerulion graph run --record` and `cerulion bag record` start this
    /// recorder for you. Run it by hand only to attach a recorder to something
    /// already running with settings neither of those verbs exposes.
    ///
    /// Run by hand it attaches to publishers that are already live and learns
    /// each channel's wire schema hash from the first message; configured
    /// schema resolution can add names and definitions. That is enough to
    /// inspect and play back the bag. Re-executing a bag (`bag play --resim`)
    /// also needs the run's graph, environment and scheduler trace, which
    /// `graph run --record` supplies to this same recorder.
    //
    // BOXED: `BagdArgs` is by far the widest variant here (it is the recorder's
    // whole argv surface), and growing it by one `Option<String>` (`--run-id`)
    // pushed the gap past `large_enum_variant`'s 200-byte threshold. Boxing is
    // what clippy asks for and what the shape wants anyway: this enum is parsed
    // ONCE per process and `Commands` is matched by every other verb, so making
    // all of them carry the recorder's argv inline is the cost, and one
    // allocation on the `bagd` path alone is the fix. `clap` implements
    // `Args for Box<T>`, so the derive is unchanged.
    #[cfg(unix)]
    Bagd(Box<cerulion_bagd::BagdArgs>),
    /// The recorder daemon (unsupported on this platform: bagd is Unix-only,
    /// because it is driven by SIGTERM lifecycle signals).
    #[cfg(not(unix))]
    Bagd,
}

/// The tracing-verbosity class of a verb, used to pick the default
/// `cerulion` log filter when `RUST_LOG` is unset.
///
/// - `OneShot` — run-and-exit introspection / management verbs (`topic *`,
///   `schema *`, `node list`/`info`/`create`/…, `graph levels`/`list`/…,
///   `workspace *`, `trace inspect`, `clean`). These default QUIET
///   (`cerulion=warn`) so their command output isn't interleaved with
///   lifecycle breadcrumbs.
/// - `LongRunning` — verbs that run a runtime loop / daemon (`graph run`,
///   `graph profile`, `node run`, `bag play`, `ros2 attach`, `viz`, `connect`,
///   and the hidden multi-process `run-worker`/`run-gateway` workers). These
///   keep the legacy `cerulion=info` default.
///
/// `-v/--verbose` raises EITHER class to `cerulion=debug`; an explicit
/// `RUST_LOG` always wins over the default (see `cerulion_core::init_logging`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerbLogClass {
    /// Run-and-exit verb — quiet (`warn`) default.
    OneShot,
    /// Runtime / daemon verb — `info` default.
    LongRunning,
}

impl VerbLogClass {
    /// True when this class defaults to the quiet `warn` filter — the
    /// `quiet_default` argument threaded into `cerulion_core::init_logging`.
    pub fn is_quiet_default(self) -> bool {
        matches!(self, VerbLogClass::OneShot)
    }
}

impl Commands {
    /// Classify this verb for the default tracing filter level.
    ///
    /// One-shot verbs run and exit without a runtime loop, so their lifecycle
    /// breadcrumbs are noise on top of the command's real output — they default
    /// QUIET. Long-running / runtime verbs (and the daemons) keep `info`.
    ///
    /// `Tui` and `Bagd` never drive this default (`main` skips `init_logging`
    /// for the TUI and dispatches `bagd` to its own logging init before
    /// `init_logging` runs), but they are classified here for exhaustiveness.
    pub fn log_verb_class(&self) -> VerbLogClass {
        use VerbLogClass::{LongRunning, OneShot};
        match self {
            // ── Long-running / runtime verbs: keep the `info` default ──
            Commands::Graph { action } => match action {
                // Runtime loops: the live run, the live profiler, and the
                // hidden multi-process worker / network-gateway processes.
                GraphAction::Run { .. }
                | GraphAction::Profile { .. }
                | GraphAction::RunWorker { .. }
                | GraphAction::RunGateway { .. } => LongRunning,
                // create / validate / list / levels / partition all run-and-exit.
                GraphAction::Create { .. }
                | GraphAction::Validate { .. }
                | GraphAction::List
                | GraphAction::Levels { .. }
                | GraphAction::Partition { .. } => OneShot,
            },
            // `node run` is a runtime loop; every other node verb runs-and-exits.
            Commands::Node { action } => match action {
                NodeAction::Run { .. } => LongRunning,
                NodeAction::Create { .. }
                | NodeAction::Delete { .. }
                | NodeAction::Modify { .. }
                | NodeAction::Build { .. }
                | NodeAction::Stage { .. }
                | NodeAction::List
                | NodeAction::Info { .. } => OneShot,
            },
            // `viz` spawns/attaches the vizd daemon; `connect` spawns
            // connectd and stays attached; `pair` drives the interactive
            // pairing ceremony via connectd — all runtime verbs.
            // `login` waits interactively for browser authorization
            // — keep `info` so any device-registration / auth warns are visible.
            Commands::Viz { .. }
            | Commands::Connect { .. }
            | Commands::Pair { .. }
            | Commands::Login => LongRunning,
            // `ros2 attach` runs a bridge graph; `ros2 run`/`launch` exec()
            // the real ros2 (runtime processes — the classification covers
            // only the pre-exec window, since init_logging runs before
            // `main`'s intercept, where any staging warn should stay visible
            // — `info` default like the other runtime verbs). `ros2 migrate`
            // is a run-and-exit rewrite whose PRODUCT is the
            // report/diff it prints — quiet `OneShot` default, so lifecycle
            // breadcrumbs cannot interleave with the report.
            Commands::Ros2 { action } => match action {
                Ros2Action::Attach { .. }
                | Ros2Action::Run { .. }
                | Ros2Action::Launch { .. } => LongRunning,
                Ros2Action::Migrate { .. } => OneShot,
            },
            // The removed-`ros`-family stub never runs anything: `main`
            // intercepts it above the login gate and prints the migration
            // error before `init_logging` matters. Classified for
            // exhaustiveness only.
            Commands::Ros { .. } => OneShot,
            // `bag play` and `bag record` run a loop until the bag
            // ends / the duration elapses / Ctrl-C, and their lifecycle
            // breadcrumbs (topics refused, taps attached, schemas learned) are
            // the operator's only window into a run — `info` default. `bag
            // info` is a pure run-and-exit render, so it stays quiet.
            // `bag play --resim` RE-EXECUTES nodes, which is the same
            // runtime-verb argument the removed `cerulion replay` made, and it
            // arrives through this same arm.
            // `bag migrate` is a run-and-exit rewrite whose product
            // is the preview it prints and the file it writes — `info` default,
            // like `bag info`.
            Commands::Bag { action } => match action {
                BagAction::Play { .. } | BagAction::Record { .. } => LongRunning,
                BagAction::Info { .. } | BagAction::Migrate { .. } => OneShot,
            },
            // ── One-shot verbs: QUIET (`warn`) default ──
            // `account devices list/revoke` run-and-exit.
            Commands::Account { .. }
            | Commands::Workspace { .. }
            | Commands::Topic { .. }
            | Commands::Schema { .. }
            | Commands::Trace { .. }
            // Prints a shell script to stdout and exits. It is also
            // the one verb whose stdout is EVALUATED by a shell, so a stray
            // lifecycle breadcrumb on stderr would at best confuse and at
            // worst be captured into a completion file.
            | Commands::Completions { .. }
            | Commands::Clean { .. } => OneShot,
            // `flashback` publishes ONE request and then WAITS —
            // up to the post window plus the finalize grace — while a recorder
            // records and writes a bag. Its breadcrumbs (which recorders
            // answered, what they refused and why) are the operator's only
            // window into that wait, so it keeps the `info` default like every
            // other verb that stays attached to a running robot.
            #[cfg(unix)]
            Commands::Flashback { .. } => LongRunning,
            #[cfg(not(unix))]
            Commands::Flashback => OneShot,
            // Never drives the default (see the doc comment) — classified for
            // exhaustiveness so a new verb forces an explicit decision here.
            Commands::Tui => OneShot,
            #[cfg(unix)]
            Commands::Bagd(..) => OneShot,
            #[cfg(not(unix))]
            Commands::Bagd => OneShot,
        }
    }
}

/// `cerulion account <action>`.
#[derive(Subcommand)]
pub enum AccountAction {
    /// Manage the devices (desks) registered to your account.
    Devices {
        #[command(subcommand)]
        action: DevicesAction,
    },
}

/// `cerulion account devices <action>`.
#[derive(Subcommand)]
pub enum DevicesAction {
    /// List the devices registered to your account (id, kind, revoked state).
    List,
    /// Revoke one of your own devices, such as a lost or decommissioned desk.
    ///
    /// The device id comes from `cerulion account devices list`.
    Revoke {
        /// The device id to revoke.
        device_id: String,
    },
}

#[derive(Subcommand)]
pub enum TraceAction {
    /// Print a human-readable timeline of the publish-trace records in `dir`.
    ///
    /// Reads every `trace_*.jsonl` file in lexicographic order (the order the
    /// files were written in) and prints one line per record, formatted as
    /// `<topic> seq=<N> t=<TIMESTAMP_NS> schema=<HASH>`. Use `--filter <topic>`
    /// to restrict the output to a single topic.
    Inspect {
        /// Directory containing `trace_*.jsonl` files.
        // Same `String`-typed path class as `graph run --record`.
        #[arg(value_hint = clap::ValueHint::DirPath)]
        dir: String,
        /// Only show entries whose `topic` matches this string exactly.
        #[arg(short = 't', long)]
        filter: Option<String>,
        /// Limit the number of records printed (most recent first
        /// if `--reverse`, otherwise oldest-first).
        #[arg(short = 'n', long)]
        limit: Option<usize>,
        /// Print most-recent records first instead of oldest-first.
        #[arg(short = 'r', long)]
        reverse: bool,
    },
}

/// `cerulion ros2 <action>`: the ROS 2 interop family. `Run` and `Launch`
/// are VERBATIM pass-through wrappers of the native `ros2` verbs; `main`
/// intercepts THOSE TWO before clap parses (so every native token, hyphenated
/// flags included, is forwarded by construction), and their variants exist for
/// the family listing, `cerulion help ros2 run|launch` and shell completion.
/// `Attach` is a NATIVE clap verb (formerly `cerulion ros attach`; the `ros`
/// family folded into `ros2`): clap parses it normally and `run()` dispatches
/// it. `Migrate` is likewise a normal clap verb (it owns its own argument
/// surface and nothing is forwarded).
///
/// The FIRST paragraph of `Run` and of `Launch` is what `cerulion ros2 --help`
/// lists, and it is the first place a user looks after an exit-69 refusal, so
/// each carries its own `--adopt-take` wording there (pinned per verb by
/// `ros2_run_e2e_test`).
#[derive(Subcommand)]
pub enum Ros2Action {
    /// Run a ROS 2 executable on Cerulion transport. Arguments go to `ros2 run`
    /// verbatim, except a leading `--adopt-take`, which this verb REFUSES (exit 69).
    ///
    /// Stages the child environment (`RMW_IMPLEMENTATION=rmw_cerulion`, an
    /// `LD_LIBRARY_PATH` prepend of the Cerulion library directory, an
    /// `AMENT_PREFIX_PATH` prepend of a minimal ament prefix so ROS 2 can load
    /// `librmw_cerulion.so`, and, when it ships beside the binary, an
    /// `LD_PRELOAD` of the Cerulion heap hook), then exec()s
    /// `ros2 run <ARGS...>`, replacing this process so stdio, signals and the
    /// exit code pass through untouched.
    ///
    /// ALL arguments are forwarded to `ros2 run` verbatim, a `--help` after
    /// the verb included: read THIS page with `cerulion help ros2 run`, and
    /// `ros2 run --help` for the native surface.
    ///
    /// The one exception is `--adopt-take` given right after `run`, which is
    /// refused. `ros2` is a Python CLI that starts the node as a further
    /// subprocess with close_fds=True, so the heap hook this launcher hands
    /// over as an inherited descriptor never reaches the node and every take
    /// would be served by copy. For zero-copy plain takes, launch the node
    /// executable directly with the hook preloaded and
    /// CERULION_RMW_ADOPT_TAKE=1 (Linux/GNU only: the hook interposes glibc's
    /// malloc/free).
    ///
    /// Exit codes (pre-exec failures only; a successful exec inherits ros2's
    /// own): 69 = `librmw_cerulion.so` missing or `--adopt-take` given,
    /// 127 = `ros2` not on PATH, 1 = other. Unix-only.
    Run {
        /// Forwarded verbatim to `ros2 run` (e.g. `demo_nodes_cpp talker`);
        /// a LEADING `--adopt-take` is Cerulion's own flag, refused here
        /// (see above).
        #[arg(
            trailing_var_arg = true,
            allow_hyphen_values = true,
            value_name = "ARGS"
        )]
        args: Vec<String>,
    },
    /// Launch a ROS 2 launch file on Cerulion transport. Arguments go to
    /// `ros2 launch` verbatim, except a leading `--adopt-take`, refused here for
    /// the same reason as on `ros2 run`.
    ///
    /// Stages the same child environment as `cerulion ros2 run`, then exec()s
    /// `ros2 launch <ARGS...>`, replacing this process so stdio, signals and
    /// the exit code pass through untouched.
    ///
    /// ALL arguments are forwarded to `ros2 launch` verbatim, a `--help` after
    /// the verb included: read THIS page with `cerulion help ros2 launch`, and
    /// `ros2 launch --help` for the native surface.
    ///
    /// Exit codes (pre-exec failures only; a successful exec inherits ros2's
    /// own): 69 = `librmw_cerulion.so` missing or `--adopt-take` given,
    /// 127 = `ros2` not on PATH, 1 = other. Unix-only.
    Launch {
        /// Forwarded verbatim to `ros2 launch` (e.g.
        /// `demo.launch.py use_rviz:=false`, or the `pkg file.launch.py`
        /// package form); a LEADING `--adopt-take` is Cerulion's own flag,
        /// refused here (see `cerulion ros2 --help`). A `--help` AFTER the verb
        /// is forwarded to the real `ros2`, like every other token.
        #[arg(
            trailing_var_arg = true,
            allow_hyphen_values = true,
            value_name = "ARGS"
        )]
        args: Vec<String>,
    },
    /// Migrate a colcon workspace's C++ publish sites to the loaned-message API.
    ///
    /// Rewrites `publish` call sites to `borrow_loaned_message()`, fill, then
    /// `publish(std::move(loaned))` wherever a clang AST prover shows the
    /// rewrite preserves behavior. It is an AST transform over the workspace's
    /// compile_commands.json, never a regex. Everything it cannot prove lands
    /// in a manual-candidates report with its reason; rclpy nodes are
    /// REPORT-ONLY (rclpy has no loaned-message API upstream). Migrating does
    /// not couple the workspace to Cerulion: rclcpp falls back to
    /// allocate-and-copy when the rmw cannot loan.
    ///
    /// INITIALIZATION IS NOT IDENTICAL ON EVERY RMW, so check this before
    /// migrating. `make_unique<T>()` value-initializes, so every field your
    /// code does not write is a declared default (a Quaternion's `w=1`,
    /// zeros elsewhere). A LOANED message is only initialized if the rmw
    /// does it: `rclcpp::LoanedMessage` does NOT construct on the
    /// can-loan path, and rmw_fastrtps (the ROS 2 default) calls Fast DDS
    /// `loan_sample()` without a LoanInitializationKind, whose default is
    /// NO_LOAN_INITIALIZATION: recycled buffer memory. So the rewrite
    /// VALUE-INITIALIZES the loaned message itself, emitting
    /// `<loaned>.get() = <MessageT>();` right after the borrow: every field
    /// the code does not write keeps the declared default `make_unique`
    /// gave it, on every rmw, for one value-init store.
    ///
    /// DRY-RUN BY DEFAULT: prints the full unified diff and the candidates
    /// report, and refreshes the machine-readable manifest at
    /// `.cerulion/ros2-migrate-manifest.json` (its only write). `--write`
    /// applies behind a consent gate: it REFUSES a dirty git tree, writes
    /// ONE commit plus `cerulion-ros2-migration.patch` (undo with
    /// `git revert <sha>` or `git apply -R`), then automatically runs
    /// `colcon build --packages-select <affected>`. A build failure is loud
    /// and names the revert path.
    ///
    /// The one-commit promise is QUALIFIED in one case: a `pre-commit` hook
    /// stages inside `git commit`, after this verb's index check, so a hook
    /// that stages puts its paths in the migration commit. Hooks are your
    /// responsibility: the run commits and WARNS, naming those paths, and
    /// when that warning appears the safe undo is
    /// `git apply -R cerulion-ros2-migration.patch` (it reverses only the
    /// migration's edits); `git revert <sha>` would undo the hook's paths
    /// too.
    ///
    /// Requires a compile database (`colcon build --cmake-args
    /// -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`) and the migration engine binary
    /// `cerulion-ros2-migrate-clang`, looked for beside the `cerulion` binary,
    /// on PATH, and at `CERULION_ROS2_MIGRATE_TOOL`. The engine is built from
    /// the Cerulion source tree (`tools/ros2_migrate/README.md` there); when it
    /// is absent the verb exits 69 and says how to build it.
    Migrate {
        /// The colcon workspace root (holds src/ and build/).
        #[arg(
            long,
            value_name = "DIR",
            default_value = ".",
            value_hint = clap::ValueHint::DirPath
        )]
        workspace: std::path::PathBuf,
        /// Apply the migration (default is dry-run): consent gate, clean
        /// git tree required, ONE commit + patch file, then an automatic
        /// `colcon build --packages-select` of the affected packages.
        #[arg(long)]
        write: bool,
        /// Skip the interactive confirm and apply (scripts / non-TTY).
        /// Only meaningful with --write.
        #[arg(long, requires = "write")]
        yes: bool,
    },
    /// Attach to a live ROS 2 / DDS robot and bridge its topics into a graph.
    ///
    /// Discovers the robot's DDS topics on `--iface`, generates the
    /// `dds_bridge` mapping config and a one-node bridge graph for the topics
    /// whose types resolve, and (after consent) runs it. `--dry-run` prints the
    /// discovery report and stops.
    ///
    /// The generated graph bridges the robot's topics and nothing else: no
    /// visualization node is staged on the robot. To SEE the data, run
    /// `cerulion viz --robot <name>` (or Cerulion Studio) on YOUR machine: the
    /// desk demands each topic, `cerulion-netd` mirrors it into desk-local
    /// shared memory, and `cerulion-vizd` decodes and renders it there. The
    /// robot only ships raw frames.
    Attach {
        /// The IP of this machine's interface on the robot's LAN, to run DDS
        /// discovery on (for example `192.0.2.18`). REQUIRED: it restricts DDS
        /// to that interface, so fragmented discovery data is not dropped on a
        /// host with more than one network interface.
        #[arg(long, value_name = "IP")]
        iface: std::net::IpAddr,
        /// DDS domain id. Must match the robot's `ROS_DOMAIN_ID`.
        #[arg(long, default_value_t = 0, value_name = "N")]
        domain: u16,
        /// Discovery collection window in seconds. Must be a finite number
        /// greater than 0 and at most 3600 (one hour); anything else is
        /// rejected at parse time.
        #[arg(
            long,
            default_value_t = 5.0,
            value_name = "SECS",
            value_parser = parse_attach_timeout
        )]
        timeout: f64,
        /// Print the discovery report and stop: write nothing, run nothing.
        /// Wins over `--yes`.
        #[arg(long)]
        dry_run: bool,
        /// Skip the interactive confirm and write + run non-interactively
        /// (scripts / CI / non-TTY).
        #[arg(long)]
        yes: bool,
        /// The generated graph name: writes `graphs/<name>.yaml` +
        /// `graphs/<name>.bridge.yaml`.
        #[arg(long, default_value = "attach", value_name = "NAME")]
        graph_name: String,
        /// Optional prefix prepended to each generated Cerulion topic name
        /// (default: mirror the ROS topic name).
        #[arg(long, value_name = "PREFIX")]
        topic_prefix: Option<String>,
        /// The robot-name namespace label written as the attach graph's
        /// `prefix:`. This is NOT the network announce identity: the name
        /// shown under ROBOTS in `topic list` (and advertised over mDNS as
        /// `_cerulion._tcp`) comes from this machine's hostname, or from the
        /// CERULION_ROBOT_IDENTITY environment variable, independent of this
        /// flag. Default prefix: the robot's shared ROS namespace if its topics
        /// have one, else this machine's hostname. Set CERULION_ROBOT_IDENTITY
        /// to control the announced identity (for example for a remote attach
        /// or a fleet running a stock image).
        #[arg(long, value_name = "NAME")]
        robot_name: Option<String>,
    },
}

/// The maximum `ros2 attach --timeout`, in
/// seconds (one hour — already absurdly generous for a discovery window; the
/// measured default is 5 s). The bound is sensible UX AND the overflow
/// guard: `Duration::from_secs_f64` PANICS above ~5.8e11 s, so an unbounded
/// "finite positive" check (the first parser) still let `--timeout 2e19`
/// through to a runtime panic.
const MAX_ATTACH_TIMEOUT_SECS: f64 = 3600.0;

/// `ros2 attach --timeout` must be a FINITE
/// number of seconds in `(0, 3600]`, rejected AT PARSE TIME with a clear
/// message. (`Duration::from_secs_f64` panics on nan/inf/negative AND on
/// overflow-large values; a 0-second or multi-hour discovery window is never
/// what a user meant — without this check non-finite shapes are silently
/// coerced to the default and overflow-large values panic at runtime.)
fn parse_attach_timeout(s: &str) -> Result<f64, String> {
    let v: f64 = s
        .parse()
        .map_err(|e| format!("'{s}' is not a number: {e}"))?;
    if !v.is_finite() || v <= 0.0 || v > MAX_ATTACH_TIMEOUT_SECS {
        return Err(format!(
            "'{s}' is not a valid discovery window: --timeout must be a finite number of \
             seconds > 0 and <= {MAX_ATTACH_TIMEOUT_SECS} (one hour; e.g. 5)"
        ));
    }
    Ok(v)
}

/// clap value parser for the counts and durations that must be at least 1.
/// It accepts exactly what `value_parser!(u64).range(1..)` accepts; the only
/// difference is an error a person can read, in place of clap's raw
/// `0 is not in 1..18446744073709551615`.
fn parse_at_least_one(s: &str) -> Result<u64, String> {
    match s.parse::<u64>() {
        Ok(0) => Err("must be at least 1".to_string()),
        Ok(v) => Ok(v),
        Err(_) => Err(format!("`{s}` is not a whole number of at least 1")),
    }
}

#[derive(Subcommand)]
pub enum WorkspaceAction {
    /// Create a new workspace
    Create {
        /// Workspace name
        name: String,
    },
    /// Initialize a workspace at the current directory
    Init {
        /// Location (default: current directory)
        // `String`-typed AND named LOCATION, so a `PathBuf` sweep and a walk
        // keyed on the value names PATH/DIR/FILE both miss it: the same class
        // twice. The walk is therefore an inverted declared-inventory guard,
        // so a name cannot hide an arg.
        #[arg(default_value = ".", value_hint = clap::ValueHint::DirPath)]
        location: String,
    },
}

#[derive(Subcommand)]
pub enum NodeAction {
    /// Create a new node type
    Create {
        /// Node type name
        // Deliberately NO completer. A create argument names
        // something that does not exist yet, so the existing node types are
        // exactly the set this verb REJECTS (`NodeExists`). Offering them
        // completes a guaranteed error. The correct candidate set is empty.
        node_type: String,
        /// Add an output port: SCHEMA NAME (both required). At most one `-o`
        /// per `node create`; add more with `cerulion node modify`.
        #[arg(short = 'o', long = "output", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        output: Vec<String>,
        /// Add a non-trigger input port: SCHEMA NAME (both required). At most
        /// one `-i` per `node create`; add more with `cerulion node modify`.
        /// For the input that should fire the node, use `-T` instead.
        #[arg(short = 'i', long = "input", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        input: Vec<String>,
        /// Add the TRIGGER input: SCHEMA NAME (both required). The input is
        /// declared `#[input(trigger)]`, so the node fires whenever it receives
        /// a message. At most one `-T` per node.
        #[arg(short = 'T', long = "trigger-input", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        trigger_input: Vec<String>,
        /// Trigger policy. SPEC is one of `period_ms=N`, `sync_window_ms=N`,
        /// `external`, `data_trigger=NAME` (or `trigger=NAME`). N must be
        /// greater than 0, and a bare `period_ms` (no `=N`) is rejected.
        ///
        /// `period_ms=N` fires the node every N milliseconds.
        ///
        /// `data_trigger=NAME` fires the node when input NAME receives a
        /// message. It is equivalent to `-T SCHEMA NAME`: supply one of the two,
        /// or both only when they name the same input.
        ///
        /// `external` makes a self-triggering driver node: it watches a signal
        /// from outside Cerulion (a device file descriptor, a blocking SDK
        /// call) and fires itself. The scaffold includes an `external_source()`
        /// stub returning `HostDriven`, which compiles as-is but which
        /// `cerulion graph run` refuses to run: replace it with a device `Fd`
        /// or a `Blocking` source first.
        ///
        /// `sync_window_ms=N` fires the node when every `#[input(trigger)]`
        /// input has received a message within an N ms window. It needs two or
        /// more trigger inputs, which one `node create` call cannot write:
        /// after creating the node, add the inputs with
        /// `cerulion node modify <TYPE> -i SCHEMA NAME` and mark each input to
        /// align as `#[input(trigger)]` in `nodes/<TYPE>/src/lib.rs`.
        ///
        /// When `--policy` is omitted: with `-T`, the node is data-triggered on
        /// that input. With no `-T` and no `-i`, the command is refused, because
        /// a node with no inputs needs `--policy period_ms=N` or
        /// `--policy external`. With `-i` and no `-T`, NO trigger policy is
        /// written and the node does not build until it has one: pass `-T`
        /// instead of `-i` for the input that should fire the node, or set a
        /// `--policy`.
        #[arg(long, value_name = "SPEC")]
        policy: Option<String>,
        /// Use the legacy raw FFI template instead of the `#[cerulion_node]` macro
        #[arg(long)]
        raw_ffi: bool,
    },
    /// Delete a node type
    Delete {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
    },
    /// Modify a node (add ports, change trigger policy)
    Modify {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
        /// Add input port: SCHEMA NAME (both required).
        #[arg(short = 'i', long = "input", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        input: Vec<String>,
        /// Add a TRIGGER input: SCHEMA NAME (both required). Adds the input
        /// with `#[input(trigger)]` AND sets the node's policy to
        /// `data_trigger=<NAME>`, clearing any conflicting `period_ms`,
        /// `sync_window_ms` or `external` macro argument. At most one `-T` per
        /// call.
        #[arg(short = 'T', long = "trigger-input", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        trigger_input: Vec<String>,
        /// Add output port: SCHEMA NAME (both required).
        #[arg(short = 'o', long = "output", num_args = 2, value_names = ["SCHEMA", "NAME"])]
        output: Vec<String>,
        /// Set the trigger policy. SPEC is one of `period_ms=N`,
        /// `sync_window_ms=N`, `external`, `data_trigger=NAME` (or
        /// `trigger=NAME`). N must be greater than 0, and a bare `period_ms`
        /// (no `=N`) is rejected.
        ///
        /// `external` turns the node into a self-triggering driver. Unlike
        /// `node create`, setting it here does NOT add the required
        /// `external_source()` method: the macro's compile error tells you
        /// exactly what to add (return `ExternalSource::Fd` or
        /// `ExternalSource::Blocking`).
        ///
        /// `data_trigger=NAME` requires NAME to match an input, either an
        /// existing one or one added in the same `node modify` call with `-i`
        /// or `-T`.
        #[arg(long, value_name = "SPEC")]
        policy: Option<String>,
    },
    /// Build a node crate
    Build {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
        /// Build in release mode
        #[arg(long)]
        release: bool,
    },
    /// Stage a node into a graph
    Stage {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
        /// Node instance ID
        #[arg(short = 'i', long)]
        id: Option<String>,
        /// Target graph name
        #[arg(short = 'g', long, add = ArgValueCandidates::new(completion::graph_names))]
        graph: Option<String>,
        // NO `-p/--prefix`. It was declared and help-texted here, accepted by
        // clap, and then destructured into `_` in `main`'s arm (no warn, no
        // error, no effect) while the sibling `node run -p` honours its own.
        // It cannot be "honoured" instead of removed: a staged instance is a
        // `NodeDef`, which has no prefix field, and `prefix:` is a GRAPH-level
        // key, so the only available behaviours were the no-op it already was
        // or a silent graph-wide rewrite of every other instance's topics.
        // Inert after the deletion of its last consumer.
        /// Bind an input: NAME SOURCE, where SOURCE is `<node_id>/<output>` of
        /// a node in the graph (for example `-I image camera/image`) or an
        /// absolute topic starting with `/`. Repeat `-I` for each input.
        #[arg(short = 'I', num_args = 2, value_names = ["NAME", "SOURCE"])]
        input_binding: Vec<String>,
    },
    /// Run a single node
    Run {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
        /// Topic prefix (default: `standalone`)
        #[arg(short = 'p', long)]
        prefix: Option<String>,
        /// Node instance ID
        #[arg(short = 'i', long)]
        id: Option<String>,
        /// Force the release-profile cdylib (`target/release`) when
        /// loading the node, matching `cerulion node build --release`.
        /// Without this flag the freshest built cdylib is auto-selected
        /// between `target/debug` and `target/release` (the profile you
        /// most recently built wins); the other profile stays a
        /// fallback when only one is built. Mirrors `graph run
        /// --release`.
        #[arg(long)]
        release: bool,
        /// Disable the CPU C-state cap (Linux PM_QoS through
        /// /dev/cpu_dma_latency). On the live (real or external) clock the cap
        /// is DERIVED from the graph's own tightest timing: a latency-sensitive
        /// graph caps idle at the shallow C1 state, which recovers most of the
        /// roughly 13 µs cold-wake cost at low power (all of it only with a C0
        /// pin, `CERULION_CPU_DMA_LOCK=1`); a latency-tolerant graph takes no
        /// lock and idles deep to save power. Pass this to disable the cap
        /// entirely (laptops, power-sensitive development). Force a hard C0
        /// pin with `CERULION_CPU_DMA_LOCK=1`, or an explicit cap in
        /// microseconds with `CERULION_CPU_DMA_LOCK_US=N`. Linux-only.
        #[arg(long = "no-cpu-dma-lock")]
        no_cpu_dma_lock: bool,
        /// Hard opt-out from the live-loop CPU monitor-wait park; no
        /// environment variable can re-enable it. Without this flag, on the
        /// live (real or external) clock the park turns on automatically when
        /// the CPU has a monitor-wait primitive (Linux x86_64 WAITPKG, Linux
        /// aarch64 WFE): it replaces the blocking wait with a shallow per-core
        /// park (no deep idle state), cutting the wake latency after an idle
        /// period at a small per-core power cost (and it suppresses the global
        /// C-state cap). That automatic choice is tunable with
        /// `CERULION_MONITOR_WAIT` and `CERULION_DOORBELL` (`"1"` forces it on,
        /// `"0"` disables it), but only when this flag is absent. No effect on
        /// other targets, where the loop blocks normally.
        #[arg(long = "no-monitor-wait")]
        no_monitor_wait: bool,
        /// Network kill-switch, the same as `graph run --network off`. The only
        /// accepted value is `off`: run LOCAL-ONLY, with no network gateway
        /// and no topic visible on the network. When the flag is absent, a
        /// real-clock `node run` starts the network gateway and every produced
        /// topic is announced on the LAN, viewable by any peer without pairing.
        #[arg(long = "network", value_name = "MODE", value_parser = ["off"])]
        network: Option<String>,
    },
    /// List all node types
    List,
    /// Show info about a node type
    Info {
        /// Node type name
        #[arg(add = ArgValueCandidates::new(completion::node_types))]
        node_type: String,
    },
}

#[derive(Subcommand)]
pub enum GraphAction {
    /// Create a new graph
    Create {
        /// Graph name
        // No completer: see `NodeAction::Create`. The existing
        // graphs are exactly what this verb rejects (`GraphExists`).
        name: String,
        /// Namespace prefix for the graph's topics, which are named
        /// `/<prefix>/<node_id>/<port>`. Default: this machine's hostname,
        /// written into the file when the graph is created.
        #[arg(short = 'n', long)]
        prefix: Option<String>,
    },
    /// Run a graph
    Run {
        /// Graph name
        #[arg(add = ArgValueCandidates::new(completion::graph_names))]
        name: String,
        /// Clock that drives the run
        #[arg(long = "time-source", value_enum, default_value_t = TimeSource::Real)]
        time_source: TimeSource,
        /// Skip the pre-flight validation REPORT (not validation as such).
        ///
        /// The graph topology, the macro trigger wiring and the node libraries
        /// are re-checked unconditionally when the graph is built, so this flag
        /// cannot get a run past those: it moves you from one refusal to an
        /// identical one.
        ///
        /// What it does still skip is the `schema:` family: a NON-EMPTY value
        /// that names no resolvable schema, contradicts the producing node's
        /// own declaration, or disagrees with the consumer's. Those graphs run,
        /// because the wire layout comes from the node's Rust type, not the
        /// label. The bill arrives later, as a bag channel whose label resolves
        /// to nothing or to the wrong definition, a Studio decoder that renders
        /// nothing, or a replay that refuses a healthy bag. How much of that
        /// channel is wrong depends on the producing node: a `#[cerulion_node]`
        /// one supplies both the wire hash and the fixed size, so the frames
        /// are described correctly and only the NAME is wrong; a node whose
        /// port metadata carries no size falls back to the workspace schema's
        /// size, or records 0 when neither source can size the channel.
        ///
        /// An ABSENT or empty `schema:` is NOT in that set: it is a topology
        /// refusal, so this flag cannot skip it either. Nor is an AMBIGUOUS
        /// spelling, one the workspace defines more than once (two files
        /// declaring it, a file stem beside an entry of that name, a YAML
        /// definition the `.msg` store also spells, a bare name the store
        /// defines in two packages): the workspace lookup refuses it, naming
        /// every source, before the graph is built, report or no report. A
        /// schema file it cannot read or parse stops the run the same way,
        /// naming that file.
        ///
        /// Conflicts with `--record`: those later bills are exactly what a
        /// RECORDING makes permanent. A bag written with the schema checks off
        /// carries channel labels nothing ever checked, so it cannot be
        /// reliably replay-verified: `bag play --resim` either refuses a
        /// healthy bag or, worse, replays one under a wrong label. Refused at
        /// PARSE, so it costs no shared memory, no workers and no bag.
        #[arg(long, conflicts_with = "record")]
        no_validate: bool,
        /// Force release-profile cdylibs (`target/release`) when
        /// loading node libraries, matching `cerulion node build
        /// --release`. Without this flag the freshest built cdylib is
        /// auto-selected between `target/debug` and `target/release`
        /// (the profile you most recently built wins); the other
        /// profile stays a fallback when only one is built. The pre-run
        /// validation uses the same mode.
        #[arg(long)]
        release: bool,
        /// Disable the CPU C-state cap (Linux PM_QoS through
        /// /dev/cpu_dma_latency). On the live (real or external) clock the cap
        /// is DERIVED from the graph's own tightest timing: a latency-sensitive
        /// graph caps idle at the shallow C1 state, which recovers most of the
        /// roughly 13 µs cold-wake cost at low power (all of it only with a C0
        /// pin, `CERULION_CPU_DMA_LOCK=1`); a latency-tolerant graph takes no
        /// lock. No effect under `--time-source virtual` (the poll loop never
        /// idles, so no lock is taken there). Force a hard C0 pin with
        /// `CERULION_CPU_DMA_LOCK=1`, or an explicit cap in microseconds with
        /// `CERULION_CPU_DMA_LOCK_US=N`. Linux-only.
        #[arg(long = "no-cpu-dma-lock")]
        no_cpu_dma_lock: bool,
        /// Hard opt-out from the live-loop CPU monitor-wait park; no
        /// environment variable can re-enable it. Without this flag, on the
        /// live (real or external) clock the park turns on automatically when
        /// the CPU has a monitor-wait primitive (Linux x86_64 WAITPKG, Linux
        /// aarch64 WFE): it replaces the blocking wait with a shallow per-core
        /// park (no deep idle state), cutting the wake latency after an idle
        /// period at a small per-core power cost (and it suppresses the global
        /// C-state cap). That automatic choice is tunable with
        /// `CERULION_MONITOR_WAIT` and `CERULION_DOORBELL` (`"1"` forces it on,
        /// `"0"` disables it), but only when this flag is absent. No effect
        /// under `--time-source virtual` (the poll loop never parks).
        #[arg(long = "no-monitor-wait")]
        no_monitor_wait: bool,
        /// Record the run to an MCAP bag.
        ///
        /// `--record` alone writes to `./recordings/`; `--record=DIR` writes to
        /// `DIR` (created if missing). The `=` is REQUIRED for a custom
        /// directory, so the optional value can never swallow the graph name.
        ///
        /// The recorder (`cerulion bagd`) taps every topic the graph declares,
        /// with its exact schema, so the graph's own topics can be re-executed
        /// and verified with `cerulion bag play --resim`. It also DISCOVERS
        /// live producers the graph never declared and records them too. Those
        /// channels carry only the wire schema hash, so `bag play --resim`
        /// SKIPS them rather than verifying them, and the recorder says so
        /// when it creates the bag. Every live producer the recorder still
        /// could not record is named in the bag's
        /// `__cerulion/record_coverage.json`, which `cerulion bag info` prints.
        ///
        /// This command builds the recorder's command line, so the recorder's
        /// discovery flags are not reachable from here: set
        /// `CERULION_RECORD_DISCOVERY=off` to record only the graph's declared
        /// topics, or `CERULION_RECORD_DISCOVERY_SETTLE_MS` to change how long
        /// bag creation waits for late producers.
        ///
        /// A single-process run records real cumulative fire times, so the
        /// graph's own topics replay identically. A multi-process run is
        /// recorded into ONE bag covering every worker, and its fire times are
        /// lockstep logical time rather than wall time; `--single-process`
        /// forces the single-process recording instead. Live clock only:
        /// combining `--record` with `--time-source virtual|external` is
        /// rejected. Unix-only.
        // `String`-typed, so the original `PathBuf` sweep missed
        // it and it would have completed NOTHING. Found by the structural
        // walk in `completion_wiring_tests`.
        #[arg(
            long,
            value_name = "DIR",
            num_args = 0..=1,
            require_equals = true,
            default_missing_value = "recordings",
            value_hint = clap::ValueHint::DirPath
        )]
        record: Option<String>,
        /// How `--record` captures the environment into the bag's `env.json`
        /// attachment. Requires `--record` (a loud parse error otherwise).
        #[arg(
            long = "record-env",
            value_enum,
            default_value_t = RecordEnv::Allowlist,
            requires = "record"
        )]
        record_env: RecordEnv,
        /// Where the recorder that `--record` starts runs: a core id to PIN it
        /// to, or `none` to let it float. When absent the choice is automatic:
        /// on Linux with at least 4 PERMITTED cores (the process's CPU affinity
        /// set, not the machine's core count) the recorder is pinned to the
        /// highest-numbered permitted core and says so in the log; with fewer,
        /// it shares cores with the graph. An explicit core must be in the
        /// permitted set. Pinning is Linux-only: elsewhere an explicit core
        /// warns loudly and the recorder floats. Requires `--record`.
        #[arg(
            long = "record-cpu",
            value_name = "CORE|none",
            value_parser = parse_record_cpu,
            requires = "record"
        )]
        record_cpu: Option<RecordCpuArg>,
        /// What a multi-process run does when a worker process dies. Only
        /// meaningful on a multi-process run: on a single-process run it warns
        /// and is ignored. Default: `continue`.
        #[arg(long = "peer-loss", value_enum)]
        peer_loss: Option<PeerLoss>,
        /// Force a single-process run, even when the graph declares
        /// `process_groups:`. The graph produces identical results, just
        /// without process isolation. On a graph WITHOUT `process_groups:`
        /// this also skips the multi-process default entirely: no partition is
        /// derived and nothing is asked. Conflicts with `--auto-partition`.
        #[arg(long = "single-process")]
        single_process: bool,
        /// Network kill-switch. The only accepted value is `off`: run
        /// LOCAL-ONLY. No network gateway is started, no network session opens
        /// and nothing crosses the machine boundary (a loud notice says so).
        /// When the flag is absent, a graph with an enabled `network:` block
        /// runs STRICT (exactly the declared egress and ingress), and a graph
        /// with no block, or a disabled one, runs PERMISSIVE: a gateway process
        /// announces every produced topic on the LAN, viewable by any peer
        /// without pairing. Restrict exposure with a `network:` block, or pass
        /// `off` for local-only.
        #[arg(long = "network", value_name = "MODE", value_parser = ["off"])]
        network: Option<String>,
        /// Cap, in entries, on the in-memory fire trace. The trace is a BOUNDED
        /// window for introspection, NOT the recording (that is the bag's job).
        /// Unbounded growth would cost about 2 GB per hour at 1 kHz with 10
        /// nodes, so `0` (unbounded) is rejected. A multi-process run applies
        /// this cap in every worker.
        #[arg(
            long = "trace-limit",
            default_value_t = cerulion_cli_engine::graph_cmd::PRODUCTION_TRACE_LIMIT as u64,
            value_parser = parse_at_least_one
        )]
        trace_limit: u64,
        /// Decline this run's per-rank SCHEDULER-TRACE rings.
        ///
        /// By default every multi-process `graph run` provisions one
        /// shared-memory scheduler-trace ring per rank (~40 MiB apparent
        /// each, converging to resident only as it fills) so a Flashback
        /// capture of the run can be RE-EXECUTED with
        /// `cerulion bag play --resim`. This declines them, for a
        /// memory-tight robot that would rather not spend it.
        ///
        /// It also stops the Flashback WINDOW RECORDER being started for
        /// this run: with no trace rings nothing captured could be
        /// re-executed, so the run takes no captures at all rather than
        /// frames-only ones. `CERULION_FLASHBACK=off` is the SEPARATE
        /// switch for the state plane + anchors; the two are orthogonal.
        ///
        /// NOT `--trace-limit`, which caps the IN-MEMORY fire-trace deque a
        /// running graph keeps for `cerulion graph` introspection. These are
        /// the SHARED-MEMORY rings a recorder drains.
        ///
        /// Conflicts with `--record`: a recording without a scheduler trace
        /// is not a recording.
        ///
        /// On the shapes that mint no ring anyway it is NOT a no-op: a
        /// `--single-process` or `--time-source external` run still has its
        /// window recorder stopped by it, so the run takes no captures. Each
        /// says at launch which it was.
        #[arg(long = "no-rings", conflicts_with = "record")]
        no_rings: bool,
        /// Re-derive the multi-process partition even when the graph already
        /// declares `process_groups:`.
        ///
        /// Shows the derived partition (from the cost snapshot at
        /// `graphs/<NAME>.costs.yaml` when present, else one process per node)
        /// and the diff against the existing block. On a terminal it then asks
        /// whether to keep yours or apply the new one; with `--yes` it applies;
        /// with no terminal and no `--yes` the run uses the re-derived groups
        /// IN MEMORY (file untouched) and says so loudly. An unpartitioned
        /// graph on Unix under the real clock derives a partition BY DEFAULT,
        /// so this flag matters only for re-deriving over an existing block.
        /// Conflicts with `--single-process`.
        #[arg(long = "auto-partition", conflicts_with = "single_process")]
        auto_partition: bool,
        /// Write the derived partition into the graph YAML without asking (a
        /// surgical rewrite with a `.bak` backup). Without it, a run on a
        /// terminal asks y/N, and a run with no terminal NEVER changes the
        /// file: it runs the derived groups in memory and says so loudly. Only
        /// meaningful when a partition is being derived (an unpartitioned graph
        /// by default, or `--auto-partition`).
        #[arg(long)]
        yes: bool,
    },
    /// Validate a graph against the workspace
    Validate {
        /// Graph name
        #[arg(add = ArgValueCandidates::new(completion::graph_names))]
        name: String,
        /// Force release-profile cdylibs (`target/release`) when
        /// resolving node libraries, matching `cerulion node build
        /// --release`. Without this flag the freshest built cdylib is
        /// auto-selected between `target/debug` and `target/release`
        /// (the profile you most recently built wins); the other
        /// profile stays a fallback when only one is built. The report
        /// shows which profile was found.
        #[arg(long)]
        release: bool,
    },
    /// List all graphs
    List,
    /// Show the derived DAG levels of a graph (read-only).
    ///
    /// Prints one row per level (nodes in graph order with each node's trigger
    /// policy), the triggering topic edges leaving each level and a summary
    /// line. When the graph declares `process_groups:` it also prints which
    /// levels each group owns and whether the partition can run as written
    /// (an invalid partition is printed in full and the command exits
    /// nonzero). Uses the same levelization the runtime uses.
    Levels {
        /// Graph name
        #[arg(add = ArgValueCandidates::new(completion::graph_names))]
        name: String,
    },
    /// Profile a graph LIVE and write its cost snapshot.
    ///
    /// Runs the graph on the real clock until every node reaches its fire
    /// target (or the duration cap or Ctrl+C stops it early), measures each
    /// node's median tick duration and each edge's fire rate, and writes them
    /// to `graphs/<NAME>.costs.yaml`, the input of the cost-aware partitioner
    /// (`cerulion graph partition`). By default each node's fire target is
    /// derived automatically from a short warm-up observation of its own rate,
    /// so a low-rate graph profiles at the defaults; `--fires N` overrides
    /// that with one uniform target for every node. A node that fires too
    /// rarely to measure is ISOLATED (no cost is recorded and it stays in its
    /// own process group) with a loud warning. Isolation is a valid outcome,
    /// not an error (exit 0).
    Profile {
        /// Graph name
        #[arg(add = ArgValueCandidates::new(completion::graph_names))]
        name: String,
        /// Observation-window cap in seconds: the run stops here even if some
        /// node has not reached its fire target yet (that node is isolated).
        #[arg(long, default_value_t = 30, value_parser = parse_at_least_one)]
        duration: u64,
        /// One fire target for EVERY node: the run stops early once every node
        /// has fired this many times. Omit it (the default) to derive each
        /// node's target from a warm-up observation of its own rate: the
        /// warm-up lasts a tenth of the duration cap, clamped to between 1 and
        /// 3 seconds, and each target is scaled to the node's projected fires
        /// over the cap, clamped into [20, 1000]. Use this override for a node
        /// whose FIRST fire lands after the warm-up (for example a period
        /// longer than 3 s): a node that stays silent through the warm-up gets
        /// no target and is isolated.
        #[arg(long, value_parser = parse_at_least_one)]
        fires: Option<u64>,
        /// Artifact output path (default: `graphs/<NAME>.costs.yaml` in the
        /// workspace).
        #[arg(short = 'o', long, value_hint = clap::ValueHint::AnyPath)]
        out: Option<PathBuf>,
    },
    /// Derive and write the graph's multi-process partition.
    ///
    /// Writes the `process_groups:` block and, when it changes anything, the
    /// `level_assignments:` block of refined levels.
    ///
    /// With a cost snapshot (from `cerulion graph profile`) the levels are
    /// first REFINED: expensive low-rate nodes move later within their
    /// topological slack, and a dense slow chain with no slack moves later as
    /// a whole (which grows the level count), so the levels that gate cheap
    /// fast chains stay cheap; the nodes of the highest-rate chain never move.
    /// The partition is then the cost-aware optimum over the refined levels.
    /// Without a snapshot the partition is one process per node over the
    /// derived levels.
    ///
    /// `level_assignments:` is written ONLY when refinement moves at least one
    /// node, so its presence always means "these levels differ from the
    /// derived ones". When refinement changes nothing (every run without a
    /// snapshot included) the block is OMITTED and a stale existing one is
    /// REMOVED (a hand-written block equal to the derived levels changes
    /// nothing and is removed too; the preview names it). The runtime and bag
    /// replays read the written block, so refined levels only take effect once
    /// WRITTEN: a plain `graph run` never refines in memory.
    ///
    /// The graph file is rewritten SURGICALLY: only the `process_groups:` and
    /// `level_assignments:` blocks change (a stale `process_group_order:`
    /// block is also removed); every comment and all other formatting are
    /// preserved byte for byte, and the previous file is backed up to
    /// `<file>.bak`. It never writes without consent: an interactive y/N
    /// confirm shows the proposed grouping, the level changes and the YAML
    /// diff, and a run with no terminal requires `--yes`. A stale or invalid
    /// existing block of either kind is REPLACED, not fatal: this verb is the
    /// recovery tool.
    Partition {
        /// Graph name
        #[arg(add = ArgValueCandidates::new(completion::graph_names))]
        name: String,
        /// Cost-snapshot artifact path (default: `graphs/<NAME>.costs.yaml`).
        /// An explicitly named file must exist and parse: it never silently
        /// falls back to the baseline; only the ABSENT default path does.
        #[arg(long, value_hint = clap::ValueHint::FilePath)]
        costs: Option<PathBuf>,
        /// Per-group compute budget in nanoseconds: nodes are fused into one
        /// group only while the sum of their median tick times stays within
        /// the budget. Omit it (the default) to use the budget the cost
        /// snapshot froze at profile time (total median tick time divided by
        /// the profiling machine's permitted core count). An older snapshot, or
        /// one with no frozen value, falls back to unbounded fusion and
        /// suggests a re-profile. An explicit value always overrides the frozen
        /// one. Zero is rejected.
        #[arg(long = "budget-ns", value_parser = parse_at_least_one)]
        budget_ns: Option<u64>,
        /// Print the derived partition + the YAML diff and exit WITHOUT
        /// writing. Wins over --yes when both are passed.
        #[arg(long = "dry-run")]
        dry_run: bool,
        /// Skip the interactive confirmation and write (scripts / non-TTY
        /// runs). Overridden by --dry-run.
        #[arg(long)]
        yes: bool,
    },
    /// HIDDEN: run ONE worker process of a multi-process deployment.
    ///
    /// NOT a user-facing verb. The multi-process supervisor behind
    /// `cerulion graph run` starts this once per process group, handing each
    /// worker its own plan file. Users always run `cerulion graph run`.
    #[command(hide = true)]
    RunWorker {
        /// Path to the serialized `WorkerPlan` JSON the supervisor wrote.
        #[arg(long)]
        #[arg(value_hint = clap::ValueHint::FilePath)]
        plan: PathBuf,
    },
    /// HIDDEN: run the network gateway process of a `graph run`.
    ///
    /// NOT a user-facing verb. `cerulion graph run` starts this once per
    /// running graph, handing it a handoff file (the network plan and the
    /// run's shared-memory namespace). The gateway owns the robot's whole
    /// network plane; graph and worker processes stay network-free. Users
    /// always run `cerulion graph run`.
    #[command(hide = true)]
    RunGateway {
        /// Path to the serialized `GatewayHandoff` JSON the parent wrote.
        #[arg(long)]
        #[arg(value_hint = clap::ValueHint::FilePath)]
        handoff: PathBuf,
    },
}

#[derive(Subcommand)]
pub enum TopicAction {
    /// List active topics: local shared-memory topics, then remote ones.
    ///
    /// Genuinely LOCAL shared-memory topics are listed first, then REMOTE
    /// topics. Remote discovery runs BY DEFAULT (multicast and gossip scouting
    /// on), so robots on the LAN show up with no flags and no pairing; when it
    /// finds nothing the listing ends with one `remote: none discovered` line.
    /// A local topic that is really a MIRROR of a remote robot's topic is
    /// listed under REMOTE as a `● streaming` row attributed to its robot,
    /// never under LOCAL. The framework's own channels (the recorder's
    /// `/bagd/status`, anything under `/__cerulion/`) are hidden from the
    /// LOCAL section by default; a count line says when any were, and `--all`
    /// lists them with an `internal` marker. REMOTE rows are not filtered.
    /// Pass `--connect` or `--listen` locators to reach
    /// peers scouting cannot find; pass `--no-network` to skip the remote
    /// network query (scripts, CI).
    List {
        /// Also list the framework's own internal topics (the recorder's
        /// `/bagd/status`, anything under `/__cerulion/`) in the LOCAL
        /// section, each with an `internal` marker after its path. Without it
        /// they are hidden there and a count line names how many were. REMOTE
        /// rows are the same with or without this flag.
        #[arg(long)]
        all: bool,
        /// Skip the REMOTE network discovery query (scripts, CI, offline).
        /// Without it, remote discovery runs automatically. Locally mirrored
        /// remote topics are still listed as REMOTE streaming rows: that
        /// information is read from local shared memory, not the network, so
        /// this flag skips only the network query.
        #[arg(long = "no-network")]
        no_network: bool,
        /// Additional zenoh locator to connect to for discovery (repeatable),
        /// for example `tcp/192.0.2.10:7683` (7683 is the default gateway
        /// port). Adds to the default scouting session.
        #[arg(long, value_name = "LOCATOR")]
        connect: Vec<String>,
        /// Additional zenoh locator to listen on for discovery (repeatable),
        /// for example `tcp/0.0.0.0:7447`. Adds to the default scouting session.
        #[arg(long, value_name = "LOCATOR")]
        listen: Vec<String>,
        /// OPT-IN: also sweep the local /24 subnets, one address at a time, on
        /// the default gateway port, for robots that neither multicast nor
        /// mDNS reveals. OFF by default: a horizontal connect sweep looks like
        /// port-scan reconnaissance to a corporate intrusion-detection system,
        /// so it never runs without this flag. An address that answers on that
        /// port is listed under ROBOTS as an unverified candidate, and becomes
        /// a robot row once its gateway announces itself (with or without
        /// topics) or answers an mDNS browse; an unrelated open port never
        /// appears as a robot.
        #[arg(long)]
        scan: bool,
    },
    /// Show info about a topic (schema name + last sequence/timestamp)
    ///
    /// Reads a LOCAL topic or, if the name is not a local topic, a REMOTE
    /// robot's topic discovered on the LAN: the topic is demanded
    /// automatically and released when the command exits. Set
    /// CERULION_NETWORK=off to disable remote discovery (local-only).
    Info {
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
    },
    /// Echo messages from a topic (decoded fields)
    ///
    /// Reads a LOCAL topic or, if the name is not a local topic, a REMOTE
    /// robot's topic discovered on the LAN: the topic is demanded
    /// automatically and released when the command exits. Set
    /// CERULION_NETWORK=off to disable remote discovery (local-only).
    Echo {
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
        /// Max array elements to render before truncating with `...` + a
        /// `(N elements)` tail (default 128; matches `ros2 topic echo
        /// --truncate-length`). Must be a positive integer: 0 and negative
        /// values are rejected at parse.
        #[arg(
            long = "truncate-length",
            value_name = "N",
            default_value_t = cerulion_cli_engine::topic_cmd::DEFAULT_ECHO_TRUNCATE_LENGTH as u64,
            value_parser = parse_at_least_one
        )]
        truncate_length: u64,
    },
    /// Measure publish rate on a topic
    ///
    /// Reads a LOCAL topic or, if the name is not a local topic, a REMOTE
    /// robot's topic discovered on the LAN: the topic is demanded
    /// automatically and released when the command exits. Set
    /// CERULION_NETWORK=off to disable remote discovery (local-only).
    Hz {
        /// Topic name
        #[arg(add = ArgValueCandidates::new(completion::topics))]
        topic: String,
    },
}

/// The `cerulion bag` verb family: bag as a data source.
#[derive(Subcommand)]
pub enum BagAction {
    /// Play a bag's recorded frames back onto local shared memory, wall-paced.
    ///
    /// Every frame is re-published BYTE-VERBATIM (its wire header's `sequence`
    /// and `timestamp_ns` intact) under its RECORDED topic name, in the order
    /// the recorder saw them, paced from the bag's own log times. Nothing is
    /// re-executed and no node is loaded: to anything attached to local shared
    /// memory this is indistinguishable from a robot publishing. Local only:
    /// the frames land in THIS machine's shared memory.
    ///
    /// A topic whose publisher slot is already held by a live producer is
    /// refused BY NAME and skipped; the remaining topics still play.
    ///
    /// `--resim` switches the verb into RE-EXECUTION: the bag's graph
    /// runs again against your workspace's current node builds instead of its
    /// frames being republished. Bare `--resim all` makes no claim about whether
    /// the result matches the recording (divergence is the product, and a
    /// completed re-execution exits 0); `--verify` adds the byte-comparison and
    /// its exit codes 0 to 6. This is where the removed `cerulion replay` verb
    /// went; that spelling no longer exists.
    Play {
        /// Path to the `.mcap` bag: a `cerulion graph run --record` recording,
        /// a Flashback capture, a `cerulion bag record --run` mid-run attach, or
        /// a standalone `cerulion bag record`. `--resim` additionally needs a
        /// SCHEDULER TRACE, which the first three carry (every
        /// multi-process run provisions the rings, recording or not) and a
        /// standalone `bag record` does not.
        // `.mcap` is a hard contract here (`cerulion_bag` writes
        // standard MCAP and nothing else), so this narrows to bags rather than
        // offering every file. Directories still complete (`complete_path`
        // keeps a filtered-out directory as a traversal candidate), so a bag in
        // a subdirectory stays reachable.
        #[arg(add = ArgValueCompleter::new(
            PathCompleter::any().filter(|p| {
                p.is_dir() || p.extension().is_some_and(|e| e.eq_ignore_ascii_case("mcap"))
            })
        ))]
        bag: PathBuf,
        /// RE-EXECUTE the bag's graph instead of republishing its frames.
        /// `all` re-executes every node; a node subset is not built
        /// yet and is refused by name. Without `--verify` this makes NO claim
        /// about matching the recording and exits 0 on any completed run.
        //
        // Deliberately a free-form `String` validated in the engine,
        // NOT a clap `value_parser` possible-values list. A clap rejection would
        // frame `--resim planner` as a bad value; the engine's refusal
        // says a node subset is not supported, and points at the spelling
        // that works. The completer still offers the one accepted value.
        #[arg(long, value_name = "NODES|all", add = ArgValueCandidates::new(completion::resim_selections))]
        resim: Option<String>,
        /// With `--resim`: byte-compare every re-executed frame against the
        /// recording and apply the stable exit contract: 0 = identical,
        /// 1 = data violation, 2 = bag I/O or not-replay-grade, 3 = node cdylib
        /// load error or a node that PANICKED, 4 = tolerance-YAML validation
        /// error, 5 = internal error, 6 = structural trace divergence.
        #[arg(long)]
        verify: bool,
        /// Playback rate multiplier: 1.0 = the recorded pace (default), 2.0 =
        /// twice as fast, 0.5 = half speed. Must be finite and greater than 0.
        /// Playback only: a resim runs on the recording's own clock.
        //
        // An `Option` rather than `default_value_t = 1.0`, because
        // clap's default erases the difference between "the user asked for the
        // recorded pace" and "the user asked for nothing", which is exactly
        // what the `--resim` refusal has to tell apart. The default is applied
        // by `resim_cmd::resolve_play_mode` instead.
        #[arg(long, short = 'r', value_name = "N")]
        rate: Option<f64>,
        /// Restart from the beginning when the bag ends, until interrupted.
        /// NOTE: the wire timestamps REGRESS on each wrap (the frames are
        /// verbatim), which consumers read as a publisher clock-epoch reset.
        /// Playback only.
        #[arg(long = "loop")]
        repeat: bool,
        /// Play only these topics (repeatable). Default: every topic in the bag.
        /// A name the bag does not carry is a loud error listing what it does.
        /// Playback only: a resim's topic set follows the nodes that execute.
        #[arg(long, value_name = "TOPIC")]
        topics: Vec<String>,
        /// Stop after `D` SECONDS of BAG TIME (fractional accepted, e.g. 12.5).
        /// Playback stops republishing there; a resim re-executes each rank up
        /// to `D` of recorded time measured from the run's shared bag-time
        /// origin. Omit to cover the whole bag.
        //
        // This REPLACES `--max-ticks`,
        // which is deleted outright (no alias, no shim: the removed `replay` verb's precedent,
        // so a stale `--max-ticks` is clap's unknown-argument error). A step cap
        // cannot bound a per-rank resim: there are k step axes and no shared step
        // number. Time is better because it has a shared ORIGIN (the minimum
        // first-boundary target across the ranks), NOT because it is uniform:
        // each rank's clock starts at its own live-loop entry, so a bound names
        // one wall interval for all of them and a later-booting rank loses more
        // of its own tail.
        #[arg(long, short = 'u', value_name = "D")]
        duration: Option<f64>,
        /// Skip the first `S` SECONDS of BAG TIME (fractional accepted).
        /// PLAYBACK ONLY: a resim refuses it by name, because re-executing from
        /// the middle of a recording is not supported yet.
        #[arg(long, short = 's', value_name = "S")]
        start_offset: Option<f64>,
        /// With `--resim --verify`: write the verdict as a JSON report to this
        /// path. The same verdict is always printed to stderr; this persists the
        /// machine-readable verdict for CI. Needs `--verify`: a bare `--resim`
        /// produces no verdict for it to persist.
        #[arg(long, value_name = "PATH", value_hint = clap::ValueHint::AnyPath)]
        report: Option<PathBuf>,
        /// With `--resim --verify`: path to a tolerance YAML. RELAXES the
        /// byte-exact diff by a `max_abs` / `max_rel` / `rmse` / `bbox_iou` /
        /// `set_equal` / `set_subset` / `ordered_list_equal` metric (a divergence
        /// past the bound is an exit-1 `tolerance-exceeded` violation), resolved
        /// per field by the precedence `fields[path]` > topic-wide `metric:` >
        /// `default_metric`, so a bare topic `metric:` covers every field of
        /// that topic and a `default_metric` covers every field of every topic.
        /// Every field that resolves to `bit_exact` (and every untargeted
        /// topic) stays BYTE-EXACT, so a drifting sibling field is still caught.
        /// Validated STRICTLY as a PRE-FLIGHT gate: an unknown key, out-of-range
        /// threshold, unresolvable topic/field name, or a non-`bit_exact` metric
        /// on a publisher-opaque field (per-field OR via a topic-wide/default
        /// metric expanded over the schema) is a hard exit-4 error (with
        /// suggestions) BEFORE any node loads.
        #[arg(long, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
        tolerance: Option<PathBuf>,
        /// With `--resim`: refuse the re-execution unless EVERY executed node's
        /// state was restored.
        ///
        /// Only a recording that begins MID-RUN restores anything. One that
        /// begins at step 0 starts every node from its constructor, which IS
        /// the recorded state, so this flag is inert on a from-start bag.
        ///
        /// It governs COVERAGE, not drift: a recorded state whose SHAPE differs
        /// from this build's is already terminal, so there is no looser
        /// behaviour for it to tighten. What it refuses is the reading a
        /// default re-execution takes silently, "this node declares no
        /// restorable state, so it starts fresh", which is right for a
        /// stateless node and a gap for one that simply has no state derive
        /// yet. Only you know which, so the default reports and this flag
        /// refuses.
        ///
        /// Valid in BOTH resim modes: it is a PRECONDITION on the run, not a
        /// claim about the recording, so a plain `--resim all` honours it too.
        #[arg(long)]
        strict_state: bool,
    },
    /// Show what a bag holds (topics, frame counts, schemas, time span) without
    /// publishing anything.
    ///
    /// This is the same summary `bag play` prints before it starts.
    Info {
        /// Path to the `.mcap` bag.
        #[arg(add = ArgValueCompleter::new(
            PathCompleter::any().filter(|p| {
                p.is_dir() || p.extension().is_some_and(|e| e.eq_ignore_ascii_case("mcap"))
            })
        ))]
        bag: PathBuf,
    },
    /// Rewrite a bag whose embedded graph no longer parses into a NEW bag.
    ///
    /// For a bag whose embedded graph carries keys the graph format no longer
    /// defines: the new bag is one `bag play --resim` reads.
    ///
    /// A legacy bag can embed your on-disk `graphs/<name>.yaml`
    /// rather than the config the run actually executed, so a since-removed
    /// setting (a legacy `policy:` block is the one that really happens)
    /// travels into the bag and now refuses to resim. An MCAP attachment is
    /// sealed, so there is no way to edit it in place; this writes a corrected
    /// copy instead.
    ///
    /// It lists every key it would remove, with the path and line it sits on in
    /// the embedded document, and asks before writing. The input bag is NEVER
    /// modified: the migrated bag is a second file, `<name>.migrated.mcap`
    /// beside the original unless `-o` says otherwise. Every recorded frame,
    /// the scheduler trace and every other attachment are copied through
    /// unchanged.
    ///
    /// A bag whose graph already parses is refused: there is nothing to
    /// migrate and no copy worth making.
    Migrate {
        /// Path to the `.mcap` bag to migrate. It is read, never written.
        // Same `.mcap` filter + directory traversal as `bag play`.
        #[arg(add = ArgValueCompleter::new(
            PathCompleter::any().filter(|p| {
                p.is_dir() || p.extension().is_some_and(|e| e.eq_ignore_ascii_case("mcap"))
            })
        ))]
        bag: PathBuf,
        /// Where to write the migrated bag. Defaults to `<name>.migrated.mcap`
        /// beside the input. A path that already exists is refused rather than
        /// overwritten.
        #[arg(
            short = 'o',
            long,
            value_name = "PATH",
            add = ArgValueCompleter::new(
                PathCompleter::any().filter(|p| {
                    p.is_dir() || p.extension().is_some_and(|e| e.eq_ignore_ascii_case("mcap"))
                })
            )
        )]
        out: Option<PathBuf>,
        /// Show exactly which keys would be removed and write nothing. Wins
        /// over --yes.
        #[arg(long)]
        dry_run: bool,
        /// Write the migrated bag without asking. Required when stdin is not a
        /// terminal (scripts / CI).
        #[arg(long)]
        yes: bool,
    },
    /// Record LOCAL topics into a bag (the `ros2 bag record` shape).
    ///
    /// Drives Cerulion's production recorder (`cerulion bagd`). Name topics
    /// positionally, take everything with `--all`, or select with `--regex`
    /// and prune with `--exclude`.
    ///
    /// LOCAL PAYLOAD CAPTURE: this taps THIS machine's shared memory and never
    /// pulls a topic's frames across the network. Recording a robot's topics
    /// means running this verb ON the robot and transferring the file
    /// afterwards. A named topic that is not live here is refused by name.
    /// Schema lookup is the one thing that may use the network: resolving a
    /// type this machine does not know can query network peers within a time
    /// budget, and `CERULION_RECORD_SCHEMA_DEMAND_MS=0` disables that.
    ///
    /// Each channel carries the real wire schema hash learned from its first
    /// frame. Schema resolution adds the qualified name and embeds custom
    /// definitions when it can; a type it cannot resolve stays explicitly
    /// unknown. That is everything `cerulion bag play` needs. A bare topic
    /// recording carries no graph context, so it cannot be re-executed with
    /// `bag play --resim`; `--run` attaches to a live `cerulion graph run` and
    /// records that context too.
    Record {
        /// Topics to record. Omit when using --all or --regex.
        #[arg(value_name = "TOPIC", add = ArgValueCandidates::new(completion::topics))]
        topics: Vec<String>,
        /// Record every live local topic (`ros2 bag record -a`). Cerulion's own
        /// internal channels (`__cerulion/*`, `/bagd/*`) are never auto-selected.
        #[arg(short = 'a', long)]
        all: bool,
        /// Record live local topics matching this regex (`ros2 bag record -e`).
        #[arg(short = 'e', long, value_name = "PATTERN")]
        regex: Option<String>,
        /// Drop selected topics matching this regex (repeatable). Narrows any
        /// selection, including an explicit list.
        #[arg(short = 'x', long, value_name = "PATTERN")]
        exclude: Vec<String>,
        /// Where to write the bag. One recording is one file.
        #[arg(
            short = 'o',
            long,
            value_name = "PATH",
            default_value = "recording.mcap",
            value_hint = clap::ValueHint::AnyPath
        )]
        out: PathBuf,
        // `-b/--max-bag-size`, the
        // size-cap rotation flag, is DE-PRODUCTIZED before launch: one
        // recording is one artifact. The recorder's roll path is intact and
        // library-reachable but no CLI flag drives it; see
        // `cerulion_bagd::BagdConfig::size_cap_bytes` for why re-adding a flag
        // needs the post-launch artifact-shape issue first.
        /// Stop recording after this many seconds. Omit to record until Ctrl-C.
        /// (ros2's `--max-bag-duration` SPLITS files; this one just stops.)
        #[arg(long, value_name = "SECS", value_parser = parse_at_least_one)]
        duration: Option<u64>,
        /// Milliseconds to wait for each topic's first frame before creating the
        /// bag, so its channel can carry the real wire schema hash. Frames seen
        /// during the wait are recorded, not dropped. A topic still silent when
        /// it elapses gets a hash-0 placeholder, permanently for that bag.
        #[arg(long = "schema-wait-ms", value_name = "MS", default_value_t = 2000)]
        schema_wait_ms: u64,
        /// ATTACH to a live `cerulion graph run`.
        ///
        /// The bag then describes a RUN, not just a set of topics: it carries
        /// the run's effective graph, its env snapshot, its host identity and
        /// its run identity, and it records the topics the run DECLARES rather
        /// than everything live on this machine.
        ///
        /// Pass the flag alone when one run is live. Name a run with
        /// `--run=<RUN>`, where RUN is a run id (as printed in `run.json`) or a
        /// graph name, to pick among several; several live runs and no id is a
        /// refusal, never a guess.
        ///
        /// The recording begins where it attaches: frames and scheduler trace
        /// start at the attach point, every channel is marked `attached_late`,
        /// and `__cerulion/run.json` records `attached_mid_run`. Nothing before
        /// the attach is recoverable, and nothing implies otherwise.
        // No completer. Run ids live only in shared memory, and a TAB
        // press must never open a transport (`completions_test.rs` walks for
        // exactly that); this arg is classified free-form in the inventory.
        //
        // `require_equals` is LOAD-BEARING, not style. With a bare
        // `num_args = 0..=1` the flag GREEDILY swallows the next word, so
        // `bag record --run /topic` bound the RUN NAME to `/topic` and
        // recorded no topics, i.e. the one composition the verb explicitly
        // supports (attach to a run, record a hand-named subset of it) was
        // unspellable in its natural form, and failed by binding rather than
        // by complaining. Requiring `=` splits the two cleanly: bare `--run`
        // takes `default_missing_value` and every following word stays a
        // positional topic, while `--run=NAME` names a run. The cost, stated:
        // `--run NAME` does not name a run: it attaches to the sole run and
        // records the topic `NAME`, which is a well-formed request rather than
        // a silent misparse, and clap's own help shows the `=` form.
        #[arg(
            long,
            value_name = "RUN",
            num_args = 0..=1,
            require_equals = true,
            default_missing_value = ""
        )]
        run: Option<String>,
    },
}

#[derive(Subcommand)]
pub enum SchemaAction {
    /// Create a new schema
    Create {
        /// Schema name
        // No completer, and this arm was the worst of the three.
        // The workspace half is what the verb rejects (already-exists), and
        // the 254 built-in `pkg/Type` names are worse than useless here: a
        // `sensor_msgs/Image` either fails on the `/` or silently creates a
        // workspace schema that SHADOWS the built-in for the whole workspace.
        name: String,
    },
    /// Delete a schema
    Delete {
        /// Schema name
        #[arg(add = ArgValueCandidates::new(completion::schema_names))]
        name: String,
    },
    /// Show info about a schema
    Info {
        /// Schema name
        #[arg(add = ArgValueCandidates::new(completion::schema_names))]
        name: String,
    },
    /// List all schemas: workspace-local (schemas/*.yaml) and built-in ROS 2 types by package
    List,
}

#[cfg(test)]
mod graph_levels_dispatch_tests {
    use super::*;

    /// `graph levels <name>` parses to `GraphAction::Levels`
    /// with the positional graph name.
    #[test]
    fn graph_levels_parses_name() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "levels", "perception"])
            .expect("`graph levels perception` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Levels { name },
            } => assert_eq!(name, "perception"),
            _ => panic!("expected Graph::Levels"),
        }
    }

    /// `graph levels` without a name is a loud parse error (the name is a
    /// required positional), never a silent default.
    #[test]
    fn graph_levels_requires_name() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "levels"]).is_err(),
            "`graph levels` without a graph name must be rejected"
        );
    }
}

#[cfg(test)]
mod pair_dispatch_tests {
    use super::*;

    /// `cerulion pair go2` parses to `Commands::Pair` with the positional
    /// robot name and every optional flag defaulting to absent.
    #[test]
    fn pair_parses_positional_name_with_defaults() {
        let cli = Cli::try_parse_from(["cerulion", "pair", "go2"])
            .expect("`cerulion pair go2` must parse");
        match cli.command {
            Commands::Pair {
                robot,
                eid,
                addrs,
                code,
                account,
                label,
                key_file,
                relay_url,
                relay_disabled,
            } => {
                assert_eq!(robot.as_deref(), Some("go2"));
                assert!(eid.is_none());
                assert!(addrs.is_empty());
                assert!(code.is_none(), "the pairing code defaults to stdin");
                assert!(account.is_none(), "the account defaults to the desk key");
                assert!(label.is_none(), "the label defaults to the desk hostname");
                assert!(
                    key_file.is_none(),
                    "the key file defaults to ~/.cerulion/desk.key"
                );
                assert!(relay_url.is_none());
                assert!(!relay_disabled);
            }
            _ => panic!("expected Commands::Pair"),
        }
    }

    /// All flags together: `--eid`, `--addr` (repeatable), `--code`, `--account`,
    /// `--label`, `--key-file`, `--relay-url`, `--relay-disabled`.
    #[test]
    fn pair_parses_all_flags() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "pair",
            "--eid",
            "deadbeef",
            "--addr",
            "192.168.1.20:7842",
            "--addr",
            "[::1]:9000",
            "--code",
            "SWAN-42",
            "--account",
            "aabb",
            "--label",
            "my-desk",
            "--key-file",
            "/keys/desk.key",
            "--relay-url",
            "https://relay.example",
            "--relay-disabled",
        ])
        .expect("all pair flags must parse");
        match cli.command {
            Commands::Pair {
                robot,
                eid,
                addrs,
                code,
                account,
                label,
                key_file,
                relay_url,
                relay_disabled,
            } => {
                assert!(robot.is_none(), "--eid replaces the positional");
                assert_eq!(eid.as_deref(), Some("deadbeef"));
                assert_eq!(addrs, vec!["192.168.1.20:7842", "[::1]:9000"]);
                assert_eq!(code.as_deref(), Some("SWAN-42"));
                assert_eq!(account.as_deref(), Some("aabb"));
                assert_eq!(label.as_deref(), Some("my-desk"));
                assert_eq!(key_file, Some(PathBuf::from("/keys/desk.key")));
                assert_eq!(relay_url.as_deref(), Some("https://relay.example"));
                assert!(relay_disabled);
            }
            _ => panic!("expected Commands::Pair"),
        }
    }

    /// `cerulion pair` with no robot AND no `--eid` still PARSES (both are
    /// optional at the clap layer) — the engine's `resolve_pair_target` produces
    /// the "specify a robot" error, matching the `connect` verb's shape.
    #[test]
    fn pair_with_no_target_parses_then_engine_errors() {
        let cli = Cli::try_parse_from(["cerulion", "pair"]).expect("`cerulion pair` parses");
        match cli.command {
            Commands::Pair { robot, eid, .. } => {
                assert!(robot.is_none());
                assert!(eid.is_none());
            }
            _ => panic!("expected Commands::Pair"),
        }
    }
}

#[cfg(test)]
mod graph_profile_dispatch_tests {
    use super::*;

    /// `graph profile <name>` (no flags) parses
    /// with the documented defaults — 30 s cap, AUTO-DERIVED per-node fire
    /// targets (`fires: None`), default artifact location (`out: None`).
    #[test]
    fn graph_profile_parses_name_with_defaults() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "profile", "perception"])
            .expect("`graph profile perception` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Profile {
                        name,
                        duration,
                        fires,
                        out,
                    },
            } => {
                assert_eq!(name, "perception");
                assert_eq!(duration, 30, "default duration cap is 30 s");
                assert_eq!(
                    fires, None,
                    "default = AUTO-DERIVE per-node targets, not a scalar"
                );
                assert!(out.is_none(), "default artifact location when -o absent");
            }
            _ => panic!("expected Graph::Profile"),
        }
    }

    /// The graph name is a required positional — a loud parse error, never a
    /// silent default.
    #[test]
    fn graph_profile_requires_name() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "profile"]).is_err(),
            "`graph profile` without a graph name must be rejected"
        );
    }

    /// All flags together: `--duration`, `--fires`, and the short `-o`.
    #[test]
    fn graph_profile_parses_all_flags() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "profile",
            "perception",
            "--duration",
            "120",
            "--fires",
            "50",
            "-o",
            "custom/costs.yaml",
        ])
        .expect("all profile flags must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Profile {
                        name,
                        duration,
                        fires,
                        out,
                    },
            } => {
                assert_eq!(name, "perception");
                assert_eq!(duration, 120);
                assert_eq!(fires, Some(50), "--fires N = the uniform-target override");
                assert_eq!(out, Some(PathBuf::from("custom/costs.yaml")));
            }
            _ => panic!("expected Graph::Profile"),
        }
    }

    /// The long `--out` form parses to the same field as `-o`.
    #[test]
    fn graph_profile_parses_long_out() {
        let cli =
            Cli::try_parse_from(["cerulion", "graph", "profile", "g", "--out", "x.costs.yaml"])
                .expect("--out must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Profile { out, .. },
            } => assert_eq!(out, Some(PathBuf::from("x.costs.yaml"))),
            _ => panic!("expected Graph::Profile"),
        }
    }

    /// Zero is rejected at PARSE for both knobs (`parse_at_least_one`): a 0-second
    /// cap or a 0-fire override would profile nothing; the engine
    /// re-validates, but the CLI surface enforces the same invariant
    /// (contract alignment). `--fires` is an OPTIONAL override —
    /// an explicit `--fires 0` still fails at parse (omitting the flag, not
    /// zeroing it, is how you select auto-derive).
    #[test]
    fn graph_profile_rejects_zero_duration_and_fires() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "profile", "g", "--duration", "0"]).is_err(),
            "--duration 0 must be a parse error"
        );
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "profile", "g", "--fires", "0"]).is_err(),
            "--fires 0 must be a parse error (auto-derive is selected by OMITTING --fires)"
        );
    }
}

#[cfg(test)]
mod graph_partition_dispatch_tests {
    use super::*;

    /// `graph partition <name>` (no flags) parses
    /// with the documented defaults — default artifact location
    /// (`costs: None`), the artifact's FROZEN budget (`budget_ns: None`),
    /// interactive (no dry-run, no yes).
    #[test]
    fn graph_partition_parses_name_with_defaults() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "partition", "perception"])
            .expect("`graph partition perception` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Partition {
                        name,
                        costs,
                        budget_ns,
                        dry_run,
                        yes,
                    },
            } => {
                assert_eq!(name, "perception");
                assert!(
                    costs.is_none(),
                    "default artifact location when --costs absent"
                );
                assert_eq!(
                    budget_ns, None,
                    "default = the artifact's frozen core-count budget, \
                     not a scalar"
                );
                assert!(!dry_run, "not a dry run by default");
                assert!(!yes, "interactive by default — never auto-consent");
            }
            _ => panic!("expected Graph::Partition"),
        }
    }

    /// The graph name is a required positional — a loud parse error, never a
    /// silent default.
    #[test]
    fn graph_partition_requires_name() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "partition"]).is_err(),
            "`graph partition` without a graph name must be rejected"
        );
    }

    /// All flags together: `--costs`, `--budget-ns`, `--dry-run`, `--yes`.
    #[test]
    fn graph_partition_parses_all_flags() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "partition",
            "perception",
            "--costs",
            "custom/costs.yaml",
            "--budget-ns",
            "5000000",
            "--dry-run",
            "--yes",
        ])
        .expect("all partition flags must parse (dry-run + yes coexist; dry-run wins)");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Partition {
                        name,
                        costs,
                        budget_ns,
                        dry_run,
                        yes,
                    },
            } => {
                assert_eq!(name, "perception");
                assert_eq!(costs, Some(PathBuf::from("custom/costs.yaml")));
                assert_eq!(
                    budget_ns,
                    Some(5_000_000),
                    "--budget-ns N = the explicit override"
                );
                assert!(dry_run);
                assert!(yes);
            }
            _ => panic!("expected Graph::Partition"),
        }
    }

    /// Zero is rejected at PARSE (`parse_at_least_one`): a 0 budget rejects every
    /// fusion; the engine re-validates, but the CLI surface enforces the same
    /// invariant (contract alignment).
    #[test]
    fn graph_partition_rejects_zero_budget() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "partition", "g", "--budget-ns", "0"])
                .is_err(),
            "--budget-ns 0 must be a parse error"
        );
    }

    /// An unknown flag is a loud parse error (no silent typo absorption).
    #[test]
    fn graph_partition_rejects_unknown_flag() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "partition", "g", "--force"]).is_err(),
            "an unknown --force flag must be rejected"
        );
    }
}

#[cfg(test)]
mod graph_run_auto_partition_flag_tests {
    use super::*;

    /// `graph run <name>` (no flags) defaults BOTH new
    /// knobs off — no re-derive request, no auto-consent (the engine's
    /// intent matrix + consent ladder own the rest).
    #[test]
    fn graph_run_defaults_auto_partition_and_yes_off() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        auto_partition,
                        yes,
                        ..
                    },
            } => {
                assert!(!auto_partition, "no re-derive by default");
                assert!(!yes, "never auto-consent by default");
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// Both flags parse (together — a scripted re-derive-and-persist run).
    #[test]
    fn graph_run_parses_auto_partition_with_yes() {
        let cli =
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--auto-partition", "--yes"])
                .expect("--auto-partition --yes must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        auto_partition,
                        yes,
                        ..
                    },
            } => {
                assert!(auto_partition);
                assert!(yes);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--single-process` + `--auto-partition` is CONTRADICTORY intent —
    /// rejected at PARSE (`conflicts_with`); the engine's intent matrix
    /// re-rejects it (defense in depth / CLI-engine contract alignment).
    #[test]
    fn graph_run_rejects_single_process_plus_auto_partition() {
        assert!(
            Cli::try_parse_from([
                "cerulion",
                "graph",
                "run",
                "g",
                "--single-process",
                "--auto-partition"
            ])
            .is_err(),
            "--single-process + --auto-partition must be a parse error"
        );
    }

    /// `--yes` composes with `--single-process` (it is simply inert there —
    /// nothing derives, nothing to consent to).
    #[test]
    fn graph_run_yes_with_single_process_parses() {
        let cli =
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--single-process", "--yes"])
                .expect("--single-process --yes must parse (yes is inert on the monolith)");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        single_process,
                        yes,
                        ..
                    },
            } => {
                assert!(single_process);
                assert!(yes);
            }
            _ => panic!("expected Graph::Run"),
        }
    }
}

#[cfg(test)]
mod time_source_flag_tests {
    use super::*;

    /// `graph run <name>` (no flag) defaults to `TimeSource::Real` — the
    /// live-by-default behavior.
    #[test]
    fn graph_run_defaults_to_real() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run { time_source, .. },
            } => assert_eq!(time_source, TimeSource::Real, "default must be Real"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `graph run <name> --time-source virtual` parses to `TimeSource::Virtual`.
    #[test]
    fn graph_run_time_source_virtual() {
        let cli =
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--time-source", "virtual"])
                .expect("`graph run g --time-source virtual` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run { time_source, .. },
            } => assert_eq!(time_source, TimeSource::Virtual),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `graph run <name> --time-source external` parses to `TimeSource::External`.
    #[test]
    fn graph_run_time_source_external() {
        let cli =
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--time-source", "external"])
                .expect("`graph run g --time-source external` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run { time_source, .. },
            } => assert_eq!(time_source, TimeSource::External),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// The removed `--sim-clock` flag is an unknown arg and must fail to
    /// parse (hard-removed, no alias).
    #[test]
    fn graph_run_rejects_removed_sim_clock_flag() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--sim-clock"]).is_err(),
            "--sim-clock was removed and must be rejected"
        );
    }

    /// **`--no-rings` parses, defaults OFF, and is refused beside
    /// `--record`.**
    ///
    /// The default is the whole point: every multi-process `graph run`
    /// provisions per-rank scheduler-trace rings, so a Flashback capture of the
    /// DEFAULT run shape can be re-executed, by design. A flag that
    /// silently defaulted ON would ship that inert.
    ///
    /// The conflict is not tidiness: a recording without a scheduler trace is not
    /// a recording — the bag would finalize looking complete while
    /// `bag play --resim` refused it. Refused at PARSE, so it costs no SHM, no
    /// workers and no bag.
    #[test]
    fn graph_run_no_rings_parses_defaults_off_and_conflicts_with_record() {
        let flagged = Cli::try_parse_from(["cerulion", "graph", "run", "g", "--no-rings"])
            .expect("`graph run g --no-rings` must parse");
        match flagged.command {
            Commands::Graph {
                action: GraphAction::Run { no_rings, .. },
            } => assert!(no_rings, "the flag must set no_rings = true"),
            _ => panic!("expected Graph::Run"),
        }

        let bare = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match bare.command {
            Commands::Graph {
                action: GraphAction::Run { no_rings, .. },
            } => assert!(
                !no_rings,
                "rings are ON by default — a default-off flag would ship the rings inert, which \
                 is exactly the state the feature exists to leave"
            ),
            _ => panic!("expected Graph::Run"),
        }

        let Err(err) = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-rings",
            "--record=recordings",
        ]) else {
            panic!("a recording with no scheduler trace is not a recording — it must be refused");
        };
        let text = err.to_string();
        assert!(
            text.contains("--no-rings") && text.contains("--record"),
            "the refusal names BOTH flags, since either one is the operator's to drop: {text}"
        );
    }

    /// `--no-rings` composes with the shapes it is INERT on rather
    /// than being refused by them.
    ///
    /// The wall-gated shapes stay ringless by design, so on
    /// `--single-process` and the virtual/external clocks the flag changes only
    /// whether captures are taken. Making it a parse error there would mean an
    /// operator could not put one line in a service file and have it mean the
    /// same thing across every graph they run.
    #[test]
    fn graph_run_no_rings_composes_with_the_shapes_it_is_inert_on() {
        for extra in [
            vec!["--single-process"],
            vec!["--time-source", "virtual"],
            vec!["--time-source", "external"],
        ] {
            let mut argv = vec!["cerulion", "graph", "run", "g", "--no-rings"];
            argv.extend_from_slice(&extra);
            let cli =
                Cli::try_parse_from(&argv).unwrap_or_else(|e| panic!("`{argv:?}` must parse: {e}"));
            match cli.command {
                Commands::Graph {
                    action: GraphAction::Run { no_rings, .. },
                } => assert!(no_rings, "…and the flag still reaches the run: {argv:?}"),
                _ => panic!("expected Graph::Run"),
            }
        }
    }

    /// **`--no-validate` is refused beside `--record`, in every spelling.**
    ///
    /// Decision: a recording must never be made with the schema checks
    /// off. `--no-validate` skips exactly the `schema:` family — a label that
    /// resolves to nothing, contradicts the producing node, or disagrees with
    /// the consumer — and a RUN pays for that later and loudly. A BAG pays for
    /// it permanently and silently: the mislabelled channel is written into the
    /// artifact, so `bag play --resim` refuses a healthy bag, or replays one
    /// under a label nothing ever checked. Refused at PARSE, like the
    /// `--no-rings` conflict above: no shared memory, no workers, no bag.
    ///
    /// Driven in EVERY spelling because `--record` takes an optional
    /// `require_equals` value, so `--record` and `--record=DIR` reach clap as
    /// different shapes, and a conflict is order-independent — a pin that drove
    /// one spelling would leave the others resting on that assumption.
    ///
    /// The anti-tautology half is in the same body: each flag ALONE must still
    /// parse. Without it, an arg accidentally made unparseable in every
    /// combination would satisfy every assertion above.
    #[test]
    fn graph_run_no_validate_conflicts_with_record_in_every_spelling() {
        for argv in [
            vec!["cerulion", "graph", "run", "g", "--no-validate", "--record"],
            vec!["cerulion", "graph", "run", "g", "--record", "--no-validate"],
            vec![
                "cerulion",
                "graph",
                "run",
                "g",
                "--no-validate",
                "--record=recordings",
            ],
            vec![
                "cerulion",
                "graph",
                "run",
                "g",
                "--record=recordings",
                "--no-validate",
            ],
        ] {
            let Err(err) = Cli::try_parse_from(&argv) else {
                panic!("a recording with the schema checks off must be refused: {argv:?}");
            };
            let text = err.to_string();
            assert!(
                text.contains("--no-validate") && text.contains("--record"),
                "the refusal names BOTH flags, since either one is the operator's to \
                 drop ({argv:?}): {text}"
            );
        }

        // ANTI-TAUTOLOGY: each flag alone still parses, so the refusals above
        // are attributable to the COMBINATION.
        for argv in [
            vec!["cerulion", "graph", "run", "g", "--no-validate"],
            vec!["cerulion", "graph", "run", "g", "--record=recordings"],
        ] {
            Cli::try_parse_from(&argv).unwrap_or_else(|e| panic!("`{argv:?}` must parse: {e}"));
        }
    }

    /// `graph run g --no-cpu-dma-lock` parses with the flag set.
    #[test]
    fn graph_run_no_cpu_dma_lock_flag_sets_true() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g", "--no-cpu-dma-lock"])
            .expect("`graph run g --no-cpu-dma-lock` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run {
                    no_cpu_dma_lock, ..
                },
            } => assert!(no_cpu_dma_lock, "flag must set no_cpu_dma_lock = true"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// Bare `graph run g` leaves `no_cpu_dma_lock` false (lock
    /// default-on for the live path).
    #[test]
    fn graph_run_no_cpu_dma_lock_defaults_false() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run {
                    no_cpu_dma_lock, ..
                },
            } => assert!(!no_cpu_dma_lock, "default must be no_cpu_dma_lock = false"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--no-cpu-dma-lock` coexists with `--time-source virtual`
    /// (the flag is independent of the clock selection).
    #[test]
    fn graph_run_no_cpu_dma_lock_coexists_with_virtual() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-cpu-dma-lock",
            "--time-source",
            "virtual",
        ])
        .expect("`graph run g --no-cpu-dma-lock --time-source virtual` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        no_cpu_dma_lock,
                        time_source,
                        ..
                    },
            } => {
                assert!(no_cpu_dma_lock);
                assert_eq!(time_source, TimeSource::Virtual);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--no-cpu-dma-lock` coexists with `--time-source external`
    /// — the opt-out is honored even on the live `External` clock (whose
    /// default would otherwise acquire the lock).
    #[test]
    fn graph_run_no_cpu_dma_lock_coexists_with_external() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-cpu-dma-lock",
            "--time-source",
            "external",
        ])
        .expect("`graph run g --no-cpu-dma-lock --time-source external` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        no_cpu_dma_lock,
                        time_source,
                        ..
                    },
            } => {
                assert!(no_cpu_dma_lock);
                assert_eq!(time_source, TimeSource::External);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `graph run g --no-monitor-wait` parses with the flag set.
    #[test]
    fn graph_run_no_monitor_wait_flag_sets_true() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g", "--no-monitor-wait"])
            .expect("`graph run g --no-monitor-wait` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run {
                    no_monitor_wait, ..
                },
            } => assert!(no_monitor_wait, "flag must set no_monitor_wait = true"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// Bare `graph run g` leaves `no_monitor_wait` false (park
    /// auto-on for the live path where the primitive exists).
    #[test]
    fn graph_run_no_monitor_wait_defaults_false() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run {
                    no_monitor_wait, ..
                },
            } => assert!(!no_monitor_wait, "default must be no_monitor_wait = false"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--no-monitor-wait` coexists with `--time-source virtual` (the
    /// flag is independent of the clock selection; the park is live-only so it
    /// is a no-op under virtual, but the flag must still parse).
    #[test]
    fn graph_run_no_monitor_wait_coexists_with_virtual() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-monitor-wait",
            "--time-source",
            "virtual",
        ])
        .expect("`graph run g --no-monitor-wait --time-source virtual` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        no_monitor_wait,
                        time_source,
                        ..
                    },
            } => {
                assert!(no_monitor_wait);
                assert_eq!(time_source, TimeSource::Virtual);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--no-monitor-wait` coexists with `--time-source external` —
    /// the opt-out is honored on the live `External` clock (whose default would
    /// otherwise arm the park where the primitive exists).
    #[test]
    fn graph_run_no_monitor_wait_coexists_with_external() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-monitor-wait",
            "--time-source",
            "external",
        ])
        .expect("`graph run g --no-monitor-wait --time-source external` must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        no_monitor_wait,
                        time_source,
                        ..
                    },
            } => {
                assert!(no_monitor_wait);
                assert_eq!(time_source, TimeSource::External);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// The hidden `graph run-worker --plan <file>` verb parses and
    /// captures the plan path. Clap renames the `RunWorker` variant to the
    /// kebab-case `run-worker` subcommand.
    #[test]
    fn graph_run_worker_parses_plan_path() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run-worker",
            "--plan",
            "/tmp/worker_P0.json",
        ])
        .expect("`graph run-worker --plan <file>` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::RunWorker { plan },
            } => assert_eq!(
                plan,
                std::path::PathBuf::from("/tmp/worker_P0.json"),
                "the --plan path must be captured verbatim"
            ),
            _ => panic!("expected Graph::RunWorker"),
        }
    }

    /// `graph run-worker` WITHOUT `--plan` is rejected by clap (the
    /// plan file is required — a worker cannot run without its plan).
    #[test]
    fn graph_run_worker_requires_plan() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run-worker"]).is_err(),
            "`graph run-worker` must require --plan"
        );
    }

    /// `--no-monitor-wait` and `--no-cpu-dma-lock` coexist (independent
    /// opt-outs; both set true together).
    #[test]
    fn graph_run_no_monitor_wait_and_no_cpu_dma_lock_coexist() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "graph",
            "run",
            "g",
            "--no-monitor-wait",
            "--no-cpu-dma-lock",
        ])
        .expect("both opt-out flags must parse together");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        no_monitor_wait,
                        no_cpu_dma_lock,
                        ..
                    },
            } => {
                assert!(no_monitor_wait);
                assert!(no_cpu_dma_lock);
            }
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--record` clap semantics with
    /// `require_equals` — absent → `None`, `--record` alone → the default dir,
    /// `--record=DIR` → `DIR`, and CRUCIALLY `graph run --record demo` parses
    /// `demo` as the GRAPH NAME (the optional value cannot greedily
    /// swallow the required positional). All four orderings pinned.
    #[test]
    fn graph_run_record_flag_orderings() {
        let parse = |args: &[&str]| -> (String, Option<String>) {
            match Cli::try_parse_from(args).expect("parse").command {
                Commands::Graph {
                    action: GraphAction::Run { name, record, .. },
                } => (name, record),
                _ => panic!("expected Graph::Run"),
            }
        };
        // (1) no --record → None.
        assert_eq!(
            parse(&["cerulion", "graph", "run", "g"]),
            ("g".to_string(), None),
            "no --record → recording off"
        );
        // (2) bare --record after the positional → the default dir.
        assert_eq!(
            parse(&["cerulion", "graph", "run", "g", "--record"]),
            ("g".to_string(), Some("recordings".to_string())),
            "--record alone → the default `recordings` dir"
        );
        // (3) --record=DIR → DIR.
        assert_eq!(
            parse(&["cerulion", "graph", "run", "g", "--record=bags"]),
            ("g".to_string(), Some("bags".to_string())),
            "--record=DIR → DIR"
        );
        // (4) THE footgun ordering: `--record` BEFORE the graph
        // name must leave the positional intact (record → default dir).
        assert_eq!(
            parse(&["cerulion", "graph", "run", "--record", "demo"]),
            ("demo".to_string(), Some("recordings".to_string())),
            "--record before the positional must NOT swallow the graph name"
        );
        // require_equals: there is no space-separated value form — a token
        // after --record is a positional, so `g --record bags2` has TWO
        // positionals and is a loud parse error (not a silent record=bags2).
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--record", "bags2"]).is_err(),
            "space-separated --record value must be rejected (use --record=DIR)"
        );
    }

    /// `--record-env` parses its
    /// three modes, defaults to `allowlist` (the secret-safe default), and
    /// REQUIRES `--record` — a lone `--record-env` is a loud parse error, never
    /// silently accepted-and-ignored.
    #[test]
    fn graph_run_record_env_three_modes_default_allowlist_requires_record() {
        let env_of = |args: &[&str]| -> RecordEnv {
            match Cli::try_parse_from(args).expect("parse").command {
                Commands::Graph {
                    action: GraphAction::Run { record_env, .. },
                } => record_env,
                _ => panic!("expected Graph::Run"),
            }
        };
        // Default (no flags) stays the secret-safe allowlist.
        assert_eq!(
            env_of(&["cerulion", "graph", "run", "g"]),
            RecordEnv::Allowlist,
            "default must be the secret-safe allowlist"
        );
        // With --record, all three modes parse.
        assert_eq!(
            env_of(&[
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-env",
                "all"
            ]),
            RecordEnv::All
        );
        assert_eq!(
            env_of(&[
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-env",
                "none"
            ]),
            RecordEnv::None
        );
        assert_eq!(
            env_of(&[
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-env",
                "allowlist"
            ]),
            RecordEnv::Allowlist
        );
        // --record-env WITHOUT --record is a loud parse
        // error (the loud-over-silent norm; a user typing it expected
        // recording side effects).
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--record-env", "all"]).is_err(),
            "--record-env without --record must be a loud parse error"
        );
        assert!(
            Cli::try_parse_from([
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-env",
                "bogus"
            ])
            .is_err(),
            "unknown mode rejected"
        );
    }

    /// **The `--record-env` flag's only crossing into the engine: a 3-row hand
    /// oracle over `RecordEnvMode::from(RecordEnv)`.**
    ///
    /// `--record-env` is the `env.json` secret boundary, and its default
    /// (`allowlist`) is what keeps a shared bag from carrying verbatim
    /// credentials. The flag reaches the engine through exactly one expression —
    /// `record_env.into()` in `main`'s `graph run` arm — and that `From` impl was
    /// untested from BOTH sides: the clap arm above stops at this crate's
    /// `RecordEnv`, and the engine's own `render_env_json` tests start at
    /// `RecordEnvMode`. A swapped arm (`Allowlist => RecordEnvMode::All`)
    /// compiles, trips no lint, and turns the secret-safe default into full
    /// environment capture — on every `graph run --record` bag AND on the run
    /// directory every serving run writes.
    ///
    /// The oracle is an EXHAUSTIVE `match` written out by hand, not a table:
    ///
    /// * TOTALITY is then real. A fourth `RecordEnv` variant makes `want` fail to
    ///   compile (E0004), and the only way to answer that is to STATE what the
    ///   new flag maps to — there is no `_` arm and no or-pattern to widen. (A
    ///   `[(flag, mode); 3]` array plus a separate exhaustive match would not
    ///   do: the match forces an or-pattern arm rather than a row, and an
    ///   `assert_eq!(oracle.len(), 3)` meant to tie the two together is
    ///   VACUOUS — the length of a fixed-size array is a compile-time constant,
    ///   so that assertion can never fire.)
    /// * The DEFAULT is asserted ACROSS the boundary — a rotation that kept all
    ///   three arms distinct would still satisfy an arm-wise check written
    ///   against a different default.
    ///
    /// Known limit: the three `check(..)` calls below are the INPUT set, and
    /// nothing forces a new variant to be added there. What IS forced is that the
    /// new variant's mapping be written down one line above, which is where a
    /// reader looking at this test would look — so the two sit adjacent
    /// deliberately.
    #[test]
    fn record_env_maps_one_to_one_onto_the_engine_redaction_mode() {
        use cerulion_cli_engine::graph_cmd::RecordEnvMode;

        // The HAND ORACLE: the redaction mode each flag value must reach the
        // engine as. Exhaustive by construction — no `_`, no or-pattern — so a
        // fourth `RecordEnv` variant is a compile error here until someone says
        // what it means. Read each arm as a sentence.
        fn want(flag: RecordEnv) -> RecordEnvMode {
            match flag {
                // `allowlist` (the default): values only for CERULION_*/RUST_LOG/IOX2_*.
                RecordEnv::Allowlist => RecordEnvMode::Allowlist,
                // `all`: verbatim values, secrets included.
                RecordEnv::All => RecordEnvMode::All,
                // `none`: names only, every value null.
                RecordEnv::None => RecordEnvMode::None,
            }
        }

        let check = |flag: RecordEnv| {
            let got: RecordEnvMode = flag.into();
            assert_eq!(
                got,
                want(flag),
                "`--record-env {flag:?}` must reach the engine as {:?}, got {got:?} — this is \
                 the env.json secret boundary",
                want(flag)
            );
        };
        check(RecordEnv::Allowlist);
        check(RecordEnv::All);
        check(RecordEnv::None);

        // The secret-safe default survives the crossing: the mode an engine
        // caller gets with no flag typed is the mode `allowlist` maps to.
        assert_eq!(
            RecordEnvMode::from(RecordEnv::Allowlist),
            RecordEnvMode::default(),
            "the clap default (`allowlist`, pinned above) must land on the engine's own default — \
             otherwise `graph run --record` with no flag captures a different environment than an \
             engine caller with no flag"
        );
    }

    /// **The `--time-source` flag's only crossing into the engine: a 3-row hand
    /// oracle over `EngineTimeSource::from(TimeSource)`.**
    ///
    /// The identically-shaped sibling of the `--record-env` mapping above, and
    /// untested for the same reason: the three clap arms at the top of this
    /// module stop at this crate's `TimeSource`, and every engine test starts at
    /// `EngineTimeSource`. A swap here silently selects a different CLOCK, so
    /// `--time-source virtual` would run the live `RealClock` loop — or, the
    /// other way, a bare `graph run` would step a 1 ms poll loop and look
    /// mysteriously slow.
    #[test]
    fn time_source_maps_one_to_one_onto_the_engine_clock_selection() {
        // The HAND ORACLE, same exhaustive-match shape as the sibling above: the
        // clock each flag value must select. A fourth `TimeSource` variant is a
        // compile error here until someone states which clock it means.
        fn want(flag: TimeSource) -> EngineTimeSource {
            match flag {
                // `real` (the default): the live event-driven WaitSet loop.
                TimeSource::Real => EngineTimeSource::Real,
                // `external`: externally-mastered time (inert today, warns at start).
                TimeSource::External => EngineTimeSource::External,
                // `virtual`: deterministic VirtualClock + 1 ms poll loop.
                TimeSource::Virtual => EngineTimeSource::Virtual,
            }
        }

        let check = |flag: TimeSource| {
            let got: EngineTimeSource = flag.into();
            assert_eq!(
                got,
                want(flag),
                "`--time-source {flag:?}` must reach the engine as {:?}, got {got:?}",
                want(flag)
            );
        };
        check(TimeSource::Real);
        check(TimeSource::External);
        check(TimeSource::Virtual);

        // The clap default (`Real`, pinned by `graph_run_defaults_to_real`
        // above) must land on the clock a bare `graph run` is documented to use.
        assert_eq!(
            EngineTimeSource::from(TimeSource::Real),
            EngineTimeSource::Real,
            "a bare `graph run` is the LIVE path"
        );
    }

    /// `--record-cpu` clap arms: a core id, the
    /// literal `none`, requires-`--record` rejection, and bad-value rejection.
    /// Flag absent maps to the engine's AUTO.
    #[test]
    fn graph_run_record_cpu_arms() {
        use cerulion_cli_engine::graph_cmd::RecordCpu;
        let cpu_of = |args: &[&str]| -> RecordCpu {
            match Cli::try_parse_from(args).expect("parse").command {
                Commands::Graph {
                    action: GraphAction::Run { record_cpu, .. },
                } => record_cpu_mode(record_cpu),
                _ => panic!("expected Graph::Run"),
            }
        };
        // Absent → AUTO.
        assert_eq!(
            cpu_of(&["cerulion", "graph", "run", "g", "--record"]),
            RecordCpu::Auto,
            "flag absent = auto placement"
        );
        // Explicit core.
        assert_eq!(
            cpu_of(&[
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-cpu",
                "7"
            ]),
            RecordCpu::Core(7)
        );
        // Literal none → float.
        assert_eq!(
            cpu_of(&[
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-cpu",
                "none"
            ]),
            RecordCpu::None
        );
        // requires = "record": a lone --record-cpu is a loud parse error.
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--record-cpu", "3"]).is_err(),
            "--record-cpu without --record must be a loud parse error"
        );
        // Bad value: neither a core id nor `none`.
        assert!(
            Cli::try_parse_from([
                "cerulion",
                "graph",
                "run",
                "g",
                "--record",
                "--record-cpu",
                "fastest"
            ])
            .is_err(),
            "a non-numeric non-`none` value must be rejected"
        );
    }

    /// The folded `cerulion bagd` subcommand parses
    /// (attach-mode flags reach BagdArgs through the embedding).
    #[cfg(unix)]
    #[test]
    fn bagd_subcommand_parses_attach_mode_args() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "bagd",
            "--out",
            "/tmp/x.mcap",
            "--topic",
            "/a",
            "--topic",
            "/b",
        ])
        .expect("`cerulion bagd` must parse");
        match cli.command {
            Commands::Bagd(args) => {
                assert_eq!(args.out, std::path::PathBuf::from("/tmp/x.mcap"));
                assert_eq!(args.topics, vec!["/a".to_string(), "/b".to_string()]);
            }
            _ => panic!("expected Commands::Bagd"),
        }
        // --topic and --topics-json stay mutually exclusive on the subcommand.
        assert!(
            Cli::try_parse_from([
                "cerulion",
                "bagd",
                "--out",
                "/tmp/x.mcap",
                "--topic",
                "/a",
                "--topics-json",
                "/tmp/t.json",
            ])
            .is_err(),
            "--topic + --topics-json must conflict"
        );
    }

    // ─── --peer-loss / --single-process / --trace-limit ───

    /// Helper: parse args and return the `GraphAction::Run` fields under test.
    fn parse_run(args: &[&str]) -> (Option<PeerLoss>, bool, u64, TimeSource) {
        let mut full = vec!["cerulion", "graph", "run", "g"];
        full.extend_from_slice(args);
        let cli = Cli::try_parse_from(full).expect("graph run args must parse");
        match cli.command {
            Commands::Graph {
                action:
                    GraphAction::Run {
                        peer_loss,
                        single_process,
                        trace_limit,
                        time_source,
                        ..
                    },
            } => (peer_loss, single_process, trace_limit, time_source),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `--peer-loss fail` / `--peer-loss continue` parse to the mirror enum.
    #[test]
    fn graph_run_peer_loss_parses_fail_and_continue() {
        let (p, ..) = parse_run(&["--peer-loss", "fail"]);
        assert_eq!(p, Some(PeerLoss::Fail));
        let (p, ..) = parse_run(&["--peer-loss", "continue"]);
        assert_eq!(p, Some(PeerLoss::Continue));
    }

    /// Bare `graph run g` leaves `--peer-loss` at `None` — the engine then
    /// consults the hidden env seam / the Continue default (`resolve_peer_loss`).
    /// The `None`-vs-`Some(Continue)` distinction is the PRECEDENCE contract:
    /// only an EXPLICIT flag beats the env seam.
    #[test]
    fn graph_run_peer_loss_defaults_to_none() {
        let (p, ..) = parse_run(&[]);
        assert_eq!(p, None, "absent flag must be None (env seam stays live)");
    }

    /// A garbage `--peer-loss` value is rejected at parse (value_enum).
    #[test]
    fn graph_run_peer_loss_garbage_rejected() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--peer-loss", "banana"])
                .is_err(),
            "--peer-loss only accepts continue|fail"
        );
    }

    /// `--single-process` parses (default false).
    #[test]
    fn graph_run_single_process_flag() {
        let (_, sp, ..) = parse_run(&["--single-process"]);
        assert!(sp);
        let (_, sp, ..) = parse_run(&[]);
        assert!(!sp, "default must be single_process = false");
    }

    /// `--trace-limit 50000` parses; the default is the engine's
    /// PRODUCTION_TRACE_LIMIT (single source — 100_000 today).
    #[test]
    fn graph_run_trace_limit_parses_and_defaults() {
        let (_, _, tl, _) = parse_run(&["--trace-limit", "50000"]);
        assert_eq!(tl, 50_000);
        let (_, _, tl, _) = parse_run(&[]);
        assert_eq!(
            tl,
            cerulion_cli_engine::graph_cmd::PRODUCTION_TRACE_LIMIT as u64,
            "default must be the engine's production trace cap"
        );
    }

    /// `--trace-limit 0` is REJECTED at parse (value_parser range 1..): the
    /// trace ring is a bounded observability window — no unbounded escape hatch.
    #[test]
    fn graph_run_trace_limit_zero_rejected() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--trace-limit", "0"]).is_err(),
            "--trace-limit 0 (unbounded) must be rejected at parse"
        );
    }

    /// The multi-process run flags compose with `--time-source` (independent knobs;
    /// the External × multi-process rejection is an ENGINE dispatch decision,
    /// not a parse-level conflict).
    #[test]
    fn graph_run_new_flags_combo_with_time_source() {
        let (p, sp, tl, ts) = parse_run(&[
            "--peer-loss",
            "fail",
            "--single-process",
            "--trace-limit",
            "2000",
            "--time-source",
            "virtual",
        ]);
        assert_eq!(p, Some(PeerLoss::Fail));
        assert!(sp);
        assert_eq!(tl, 2_000);
        assert_eq!(ts, TimeSource::Virtual);
    }

    /// The PeerLoss mirror maps 1:1 onto the engine enum (both directions of
    /// the two-variant surface).
    #[test]
    fn peer_loss_mirror_maps_to_engine_enum() {
        assert_eq!(
            EnginePeerLossPolicy::from(PeerLoss::Continue),
            EnginePeerLossPolicy::Continue
        );
        assert_eq!(
            EnginePeerLossPolicy::from(PeerLoss::Fail),
            EnginePeerLossPolicy::Fail
        );
    }
}

#[cfg(test)]
mod topic_list_network_flag_tests {
    use super::*;

    /// Bare `topic list` (no flags) defaults to AUTOMAGIC remote
    /// discovery — `no_network` is false, no explicit locators. The `--network`
    /// flag is GONE (remote discovery is on by default).
    #[test]
    fn topic_list_bare_defaults_to_network_on() {
        let cli = Cli::try_parse_from(["cerulion", "topic", "list"])
            .expect("bare `topic list` must parse");
        match cli.command {
            Commands::Topic {
                action:
                    TopicAction::List {
                        all,
                        no_network,
                        connect,
                        listen,
                        scan,
                    },
            } => {
                assert!(!no_network, "bare list must default to remote-discovery ON");
                assert!(connect.is_empty());
                assert!(listen.is_empty());
                assert!(!scan, "the opt-in subnet sweep must be OFF by default");
                assert!(!all, "internal topics must be HIDDEN on a bare list");
            }
            _ => panic!("expected Topic::List"),
        }
    }

    /// Discovery rung 4: `topic list --scan` opts into the unicast subnet sweep;
    /// the flag is OFF on a bare `topic list` (pinned above). This is the sole
    /// producer of a `true` scan — the structural gate against the sweep firing
    /// unopted.
    #[test]
    fn topic_list_scan_flag_parses() {
        let cli = Cli::try_parse_from(["cerulion", "topic", "list", "--scan"])
            .expect("`topic list --scan` must parse");
        match cli.command {
            Commands::Topic {
                action: TopicAction::List { scan, .. },
            } => assert!(scan, "--scan must set scan=true"),
            _ => panic!("expected Topic::List"),
        }
    }

    /// `topic list --no-network` skips the remote query.
    #[test]
    fn topic_list_no_network_flag_parses() {
        let cli = Cli::try_parse_from(["cerulion", "topic", "list", "--no-network"])
            .expect("`topic list --no-network` must parse");
        match cli.command {
            Commands::Topic {
                action: TopicAction::List { no_network, .. },
            } => assert!(no_network, "--no-network must set no_network=true"),
            _ => panic!("expected Topic::List"),
        }
    }

    /// `topic list --all` lists the framework's own internal topics (hidden on
    /// a bare list, pinned above) with their `internal` marker.
    #[test]
    fn topic_list_all_flag_parses() {
        let cli = Cli::try_parse_from(["cerulion", "topic", "list", "--all"])
            .expect("`topic list --all` must parse");
        match cli.command {
            Commands::Topic {
                action: TopicAction::List { all, .. },
            } => assert!(all, "--all must set all=true"),
            _ => panic!("expected Topic::List"),
        }
    }

    /// The removed `--network` flag is an UNKNOWN argument (killed,
    /// not deprecated).
    #[test]
    fn topic_list_removed_network_flag_is_unknown() {
        assert!(
            Cli::try_parse_from(["cerulion", "topic", "list", "--network"]).is_err(),
            "the `--network` flag was removed — it must be an unknown argument"
        );
    }

    /// `--connect` / `--listen` accumulate in order and NO LONGER
    /// require any gating flag (they add to the default scouting session).
    #[test]
    fn topic_list_repeatable_locators_accumulate_without_gate() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "topic",
            "list",
            "--connect",
            "tcp/192.168.123.99:7447",
            "--connect",
            "tcp/192.168.123.100:7447",
            "--listen",
            "tcp/0.0.0.0:7447",
        ])
        .expect("repeated locators must parse without a gating flag");
        match cli.command {
            Commands::Topic {
                action:
                    TopicAction::List {
                        all,
                        no_network,
                        connect,
                        listen,
                        scan,
                    },
            } => {
                assert!(!no_network);
                assert!(!all, "no --all ⇒ internal topics stay hidden");
                assert!(!scan, "no --scan ⇒ scan stays off");
                assert_eq!(
                    connect,
                    vec![
                        "tcp/192.168.123.99:7447".to_string(),
                        "tcp/192.168.123.100:7447".to_string()
                    ]
                );
                assert_eq!(listen, vec!["tcp/0.0.0.0:7447".to_string()]);
            }
            _ => panic!("expected Topic::List"),
        }
    }

    /// A bare `--connect` (no gating flag) parses fine — there is no
    /// `requires = "network"` coupling.
    #[test]
    fn topic_list_connect_alone_parses() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "topic",
            "list",
            "--connect",
            "tcp/192.168.123.99:7447",
        ])
        .expect("`--connect` alone must parse: no flag gates it");
        match cli.command {
            Commands::Topic {
                action: TopicAction::List { connect, .. },
            } => assert_eq!(connect, vec!["tcp/192.168.123.99:7447".to_string()]),
            _ => panic!("expected Topic::List"),
        }
    }
}

/// Verbs this CLI has REMOVED must not parse at all.
///
/// The precedent is `topic_list_removed_network_flag_is_unknown` above: a
/// killed surface is killed, not deprecated, and the pin that proves it is a
/// parse REFUSAL rather than an assertion about a message. A removed verb that
/// still parses is the failure this module exists to catch — clap answers a
/// declared variant itself, so the surface can go on existing (in `--help`
/// classification, in dispatch, in tab completion) long after the behaviour
/// behind it is gone.
#[cfg(test)]
mod removed_verb_tests {
    use super::*;

    /// `cerulion replay` is an unknown subcommand.
    ///
    /// No variant named `replay` exists, not even a hidden one that parses the
    /// removed verb purely to DIAGNOSE it — such a variant runs nothing
    /// and always exits 2, but it still parses.
    /// Nothing named `replay` may
    /// reach the parser: re-execution is `cerulion bag play <bag> --resim all
    /// [--verify]`, and the old spelling dies as clap's plain
    /// `unrecognized subcommand 'replay'` (exit 2, the usage code
    /// callers already expect).
    ///
    /// BOTH arms are load-bearing. A diagnosing variant has to be declared
    /// `trailing_var_arg` + `allow_hyphen_values` so that every
    /// old invocation shape reaches it, so a bare bag path and a hyphenated
    /// flag shape enter by DIFFERENT clap paths — a half-removed declaration
    /// (variant gone, or an `external_subcommand` catch-all added later) can
    /// leave one still parsing while the other refuses.
    ///
    /// With such a variant declared, each arm FAILS —
    /// `replay bag.mcap` and `replay bag.mcap --tolerance t.yaml` both parse
    /// into it.
    #[test]
    fn removed_replay_verb_is_an_unknown_subcommand() {
        assert!(
            Cli::try_parse_from(["cerulion", "replay", "bag.mcap"]).is_err(),
            "`cerulion replay` was removed — it must be an unrecognized \
             subcommand, not a parse into a hidden diagnosing variant"
        );
        assert!(
            Cli::try_parse_from(["cerulion", "replay", "bag.mcap", "--tolerance", "t.yaml"])
                .is_err(),
            "an old `cerulion replay` flag shape must be unrecognized too — a \
             diagnosing variant swallows trailing hyphenated args, so this arm is \
             what catches a half-removed declaration"
        );
    }

    /// **`node stage -p/--prefix` is gone, and its sibling `node run -p` is
    /// not.**
    ///
    /// The flag was declared, help-texted, accepted by clap, and then
    /// destructured into `_` in `main`'s `node stage` arm — a silent success on
    /// a staging verb. It could not be honoured instead of removed: a staged
    /// instance is a `NodeDef`, which carries no prefix field, and `prefix:` is
    /// a GRAPH-level key, so the only behaviours available were the no-op it
    /// already was or a silent rewrite of every other instance's topics in the
    /// same file.
    ///
    /// BOTH halves are load-bearing, and they are in one body so a revert
    /// cannot satisfy half of it:
    ///
    /// * BOTH SPELLINGS refused. Deleting a `#[arg(short, long)]` removes both
    ///   at once today, but an "alias for compatibility" added later would
    ///   resurrect exactly the accept-and-discard shape, one spelling at a
    ///   time.
    /// * The SIBLINGS still parse. `node run -p` and `graph create -n` are real,
    ///   honoured prefixes, and this is the OVER-deletion half: a sweep over
    ///   `short = 'p'` (or over the name `prefix`) catches all three, compiles,
    ///   satisfies both refusal arms above, and silently breaks two working
    ///   flags. (A deletion that hit the WRONG one instead is caught by the
    ///   refusal arms, since `node stage -p` would still parse.)
    #[test]
    fn removed_node_stage_prefix_flag_is_an_unexpected_argument() {
        for argv in [
            vec!["cerulion", "node", "stage", "t", "-g", "g", "-p", "pfx"],
            vec![
                "cerulion", "node", "stage", "t", "-g", "g", "--prefix", "pfx",
            ],
        ] {
            let Err(err) = Cli::try_parse_from(&argv) else {
                panic!(
                    "`node stage` has no prefix to honour (`NodeDef` has no such field; \
                     `prefix:` is graph-level), so {argv:?} must be REFUSED rather than \
                     accepted and discarded"
                );
            };
            let text = err.to_string();
            assert!(
                text.contains("unexpected argument"),
                "the refusal must be clap's plain unknown-argument error, so a script still \
                 passing the dead flag fails loudly ({argv:?}): {text}"
            );
        }

        // ANTI-REGRESSION: the two REAL prefixes are untouched. `node run -p`
        // is honoured (it is the one-node graph's prefix) and `graph create -n`
        // writes the graph-level `prefix:` this verb's instances resolve
        // against.
        Cli::try_parse_from(["cerulion", "node", "run", "t", "-p", "pfx"])
            .expect("`node run -p` is a REAL, honoured prefix and must still parse");
        Cli::try_parse_from(["cerulion", "graph", "create", "g", "-n", "pfx"])
            .expect("`graph create -n` writes the graph-level prefix and must still parse");

        // ...and `node stage`'s surviving flags still parse, so the arms above
        // are attributable to the deleted flag rather than to a broken verb.
        Cli::try_parse_from([
            "cerulion", "node", "stage", "t", "-g", "g", "-i", "id", "-I", "inp", "src",
        ])
        .expect("`node stage` without the dead flag must still parse");
    }
}

#[cfg(test)]
mod graph_run_network_flag_tests {
    use super::*;

    /// Bare `graph run g` carries no kill-switch — the run takes the
    /// networking default (an enabled `network:` block ⇒ Strict; no block ⇒
    /// Permissive).
    #[test]
    fn graph_run_without_network_flag_parses_none() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g"])
            .expect("bare `graph run g` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run { network, .. },
            } => assert!(network.is_none(), "absent flag must parse to None"),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `graph run g --network off` parses to `Some("off")` — the
    /// kill-switch.
    #[test]
    fn graph_run_network_off_parses() {
        let cli = Cli::try_parse_from(["cerulion", "graph", "run", "g", "--network", "off"])
            .expect("`graph run g --network off` must parse");
        match cli.command {
            Commands::Graph {
                action: GraphAction::Run { network, .. },
            } => assert_eq!(network.as_deref(), Some("off")),
            _ => panic!("expected Graph::Run"),
        }
    }

    /// `off` is the ONLY accepted value: `--network on` is a loud parse
    /// error listing the valid value (there is no `--network on` — an
    /// absent flag already means "honor the YAML").
    #[test]
    fn graph_run_network_on_rejected_at_parse() {
        let Err(err) = Cli::try_parse_from(["cerulion", "graph", "run", "g", "--network", "on"])
        else {
            panic!("--network on must be rejected at parse (only `off` exists)");
        };
        let msg = err.to_string();
        assert!(
            msg.contains("off"),
            "the rejection must list the valid value `off`; got: {msg}"
        );
    }

    /// A bare `--network` without a value is a parse error (the flag
    /// requires its MODE value — clap enforces it).
    #[test]
    fn graph_run_network_without_value_rejected_at_parse() {
        assert!(
            Cli::try_parse_from(["cerulion", "graph", "run", "g", "--network"]).is_err(),
            "bare --network (no value) must be a parse error"
        );
    }

    /// `node run` mirrors `graph run`'s kill-switch — bare parse =
    /// None (the permissive default applies), `--network off` = Some("off"),
    /// any other value rejected at parse.
    #[test]
    fn node_run_network_off_mirrors_graph_run() {
        let cli = Cli::try_parse_from(["cerulion", "node", "run", "n"])
            .expect("bare `node run n` must parse");
        match cli.command {
            Commands::Node {
                action: NodeAction::Run { network, .. },
            } => assert!(
                network.is_none(),
                "absent flag = the permissive default applies"
            ),
            _ => panic!("expected Node::Run"),
        }

        let cli = Cli::try_parse_from(["cerulion", "node", "run", "n", "--network", "off"])
            .expect("`node run n --network off` must parse");
        match cli.command {
            Commands::Node {
                action: NodeAction::Run { network, .. },
            } => assert_eq!(network.as_deref(), Some("off")),
            _ => panic!("expected Node::Run"),
        }

        assert!(
            Cli::try_parse_from(["cerulion", "node", "run", "n", "--network", "on"]).is_err(),
            "`off` is the only accepted value for node run --network"
        );
    }
}

#[cfg(test)]
mod ros_attach_flag_tests {
    use super::*;

    /// The removed `ros` family's stub swallows EVERY old spelling — help
    /// flags included. Note that clap answers `--help`
    /// BEFORE returning a parsed command, so without `disable_help_flag` on
    /// the stub, `cerulion ros --help` served clap's help page (exit 0)
    /// instead of the migration message `main` promises for the removed
    /// family. Dropping `disable_help_flag` makes every arm below
    /// fail with a `DisplayHelp` error instead of a parsed stub.
    #[test]
    fn removed_ros_stub_swallows_help_flags_instead_of_serving_clap_help() {
        for argv in [
            vec!["cerulion", "ros", "--help"],
            vec!["cerulion", "ros", "-h"],
            vec!["cerulion", "ros", "attach", "--help"],
            vec!["cerulion", "ros", "attach", "--iface", "10.0.0.7"],
        ] {
            let expected_args: Vec<String> = argv[2..].iter().map(|s| s.to_string()).collect();
            let cli = match Cli::try_parse_from(&argv) {
                Ok(cli) => cli,
                Err(e) => panic!(
                    "`{}` must parse into the migration stub, never clap's \
                     own help/error (kind {:?})",
                    argv.join(" "),
                    e.kind()
                ),
            };
            match cli.command {
                Commands::Ros { args } => assert_eq!(
                    args,
                    expected_args,
                    "the stub must swallow the full old argv for `{}`",
                    argv.join(" ")
                ),
                _ => panic!("expected the Commands::Ros stub"),
            }
        }
    }

    /// Non-finite / non-positive
    /// `ros2 attach --timeout` values are rejected AT PARSE TIME with a
    /// message naming the constraint — otherwise they would parse fine and be
    /// silently coerced to 5.0 downstream.
    #[test]
    fn ros_attach_rejects_nonpositive_and_nonfinite_timeouts_at_parse() {
        // The set includes the OVERFLOW class ("2e19": finite + positive, but
        // `Duration::from_secs_f64` panics above ~5.8e11 s — unbounded, it would
        // parse fine and panic at runtime) and the bound edge ("3601").
        for bad in ["0", "-3", "nan", "inf", "-inf", "0.0", "2e19", "3601"] {
            let arg = format!("--timeout={bad}");
            // let-else (the `graph_run_network_on_rejected_at_parse` house
            // pattern) — `expect_err` would require `Cli: Debug`, which the
            // clap structs deliberately do not derive.
            let Err(err) = Cli::try_parse_from([
                "cerulion",
                "ros2",
                "attach",
                "--iface",
                "192.168.123.18",
                arg.as_str(),
            ]) else {
                panic!("--timeout={bad} must be rejected at parse");
            };
            let msg = err.to_string();
            assert!(
                msg.contains("finite number of seconds > 0 and <= 3600"),
                "the rejection must name BOTH bounds; got: {msg}"
            );
        }
    }

    /// The happy `ros2 attach` parse: a valid `--timeout` lands verbatim, the
    /// default is 5.0, and `--iface` is REQUIRED.
    #[test]
    fn ros_attach_timeout_parses_and_iface_is_required() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "ros2",
            "attach",
            "--iface",
            "192.168.123.18",
            "--timeout",
            "2.5",
        ])
        .expect("valid ros2 attach must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Attach { timeout, iface, .. },
            } => {
                assert_eq!(timeout, 2.5);
                assert_eq!(iface.to_string(), "192.168.123.18");
            }
            _ => panic!("expected Ros2::Attach"),
        }

        // Default window.
        let cli = Cli::try_parse_from(["cerulion", "ros2", "attach", "--iface", "10.0.0.7"])
            .expect("iface-only ros2 attach must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Attach { timeout, .. },
            } => assert_eq!(timeout, 5.0),
            _ => panic!("expected Ros2::Attach"),
        }

        // The upper bound is INCLUSIVE — exactly 3600 s parses.
        let cli = Cli::try_parse_from([
            "cerulion",
            "ros2",
            "attach",
            "--iface",
            "10.0.0.7",
            "--timeout",
            "3600",
        ])
        .expect("--timeout 3600 (the inclusive bound) must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Attach { timeout, .. },
            } => assert_eq!(timeout, 3600.0),
            _ => panic!("expected Ros2::Attach"),
        }

        // --iface is required (the load-bearing multi-homed-discovery knob).
        assert!(
            Cli::try_parse_from(["cerulion", "ros2", "attach"]).is_err(),
            "ros2 attach without --iface must be a parse error"
        );
    }

    /// `ros2 attach` stages no visualization node on the robot, so the
    /// `--no-viz` opt-out is REMOVED rather than defaulted-on — a flag that
    /// skips staging is a misleading name for a no-op, and a misleading
    /// user-facing surface is removed, never re-documented.
    ///
    /// Removing a CLI flag is a user-visible breaking change, so the contract
    /// is that a stale invocation FAILS LOUDLY (clap's unknown-argument error)
    /// rather than being silently accepted and ignored. Both the removed flag
    /// and its positive twin `--viz` are pinned. Clap-level unit, no
    /// process spawn.
    #[test]
    fn ros_attach_rejects_the_removed_no_viz_flag() {
        // Anti-tautology control: the same invocation WITHOUT the flag parses,
        // so the rejection below is attributable to `--no-viz` alone and not to
        // some unrelated breakage in the `ros2 attach` argument set.
        let cli = Cli::try_parse_from(["cerulion", "ros2", "attach", "--iface", "10.0.0.7"])
            .expect("iface-only ros2 attach must parse");
        assert!(
            matches!(
                cli.command,
                Commands::Ros2 {
                    action: Ros2Action::Attach { .. }
                }
            ),
            "expected Ros2::Attach"
        );

        for removed in ["--no-viz", "--viz"] {
            // `Cli` is not `Debug`, so unwrap the Result by hand rather than
            // with `expect_err`.
            let err = match Cli::try_parse_from([
                "cerulion", "ros2", "attach", "--iface", "10.0.0.7", removed,
            ]) {
                Ok(_) => panic!(
                    "`{removed}` must be a LOUD parse error, never silently accepted \
                     (a removed flag that still parses would be ignored in silence)"
                ),
                Err(e) => e,
            };
            assert_eq!(
                err.kind(),
                clap::error::ErrorKind::UnknownArgument,
                "`{removed}` must fail as an unknown argument, got {:?}",
                err.kind()
            );
        }
    }

    /// The viz flags parse with the daemon-client defaults —
    /// `--detach` defaults OFF. `--release`, `--memory-limit`, `--no-spawn` and
    /// `--serve-web` were REMOVED: the daemon is a prebuilt binary that owns its
    /// own build + the hosted-proxy memory, and the viewer is Cerulion Studio, so
    /// no per-run verb flag starts, shapes or suppresses one.
    #[test]
    fn viz_flags_parse_and_default() {
        // Defaults: topics-only ⇒ detach off.
        let cli = Cli::try_parse_from(["cerulion", "viz", "/tf"]).expect("viz /tf must parse");
        match cli.command {
            Commands::Viz {
                topics,
                robot,
                connect,
                listen,
                detach,
            } => {
                assert_eq!(topics, vec!["/tf".to_string()]);
                // Remote flags default empty/None.
                assert!(robot.is_none(), "--robot defaults None");
                assert!(
                    connect.is_empty() && listen.is_empty(),
                    "--connect/--listen default empty"
                );
                assert!(!detach, "--detach defaults OFF");
            }
            _ => panic!("expected Commands::Viz"),
        }
        // --detach parses (the Studio handoff — attach + return).
        let cli = Cli::try_parse_from(["cerulion", "viz", "/tf", "--detach"])
            .expect("viz --detach parses");
        match cli.command {
            Commands::Viz { detach, .. } => assert!(detach, "--detach sets detach"),
            _ => panic!("expected Commands::Viz"),
        }
        // The REMOVED flags are rejected as UNKNOWN arguments (not merely some
        // other parse error — the flag is gone from the contract, so clap must not
        // recognize it at all). `--no-spawn` and `--serve-web` join the list: the
        // verb never starts a viewer, so a stale invocation must FAIL rather than
        // be a silent no-op.
        // (`Cli` is not `Debug`, so unwrap the ERROR via `.err()` rather than
        // `expect_err`, which would need the Ok type to be `Debug`.)
        for removed in ["--release", "--no-spawn", "--serve-web"] {
            let err = Cli::try_parse_from(["cerulion", "viz", "/tf", removed])
                .err()
                .unwrap_or_else(|| panic!("`{removed}` was removed and must not parse"));
            assert_eq!(
                err.kind(),
                clap::error::ErrorKind::UnknownArgument,
                "`{removed}` rejects as an UNKNOWN flag, got {:?}",
                err.kind()
            );
        }
        let mem = Cli::try_parse_from(["cerulion", "viz", "/tf", "--memory-limit", "256MB"])
            .err()
            .expect("--memory-limit was removed");
        assert_eq!(
            mem.kind(),
            clap::error::ErrorKind::UnknownArgument,
            "--memory-limit rejects as an UNKNOWN flag, got {:?}",
            mem.kind()
        );
    }

    /// The remote-robot flags parse — `--robot NAME` plus
    /// repeatable `--connect`/`--listen` locators (mirroring `topic list`), with a
    /// pinned `TOPIC=SCHEMA` positional.
    #[test]
    fn viz_remote_robot_flags_parse() {
        let cli = Cli::try_parse_from([
            "cerulion",
            "viz",
            "--robot",
            "go2",
            "--connect",
            "tcp/192.168.123.99:7683",
            "--connect",
            "tcp/10.0.0.9:7447",
            "--listen",
            "tcp/0.0.0.0:7447",
            "/utlidar/cloud=sensor_msgs/PointCloud2",
        ])
        .expect("viz --robot with locators must parse");
        match cli.command {
            Commands::Viz {
                topics,
                robot,
                connect,
                listen,
                ..
            } => {
                assert_eq!(robot.as_deref(), Some("go2"));
                assert_eq!(
                    connect,
                    vec!["tcp/192.168.123.99:7683", "tcp/10.0.0.9:7447"],
                    "--connect is repeatable"
                );
                assert_eq!(listen, vec!["tcp/0.0.0.0:7447"]);
                assert_eq!(topics, vec!["/utlidar/cloud=sensor_msgs/PointCloud2"]);
            }
            _ => panic!("expected Commands::Viz"),
        }
    }

    /// `--connect`/`--listen` `requires = "robot"`:
    /// the local viz path never reads these locators, so passing one WITHOUT
    /// `--robot` is a LOUD clap parse error (naming `--robot`), not a silent drop.
    #[test]
    fn viz_connect_or_listen_without_robot_is_a_loud_error() {
        // let-else (the house pattern) — `expect_err` would require `Cli: Debug`,
        // which the clap structs deliberately do not derive.
        // --connect without --robot: rejected.
        let Err(err) = Cli::try_parse_from([
            "cerulion",
            "viz",
            "/tf",
            "--connect",
            "tcp/192.168.123.99:7683",
        ]) else {
            panic!("--connect without --robot must be a parse error");
        };
        let msg = err.to_string();
        assert!(
            msg.contains("--robot"),
            "the error must name the required --robot flag:\n{msg}"
        );

        // --listen without --robot: rejected too.
        let Err(err) = Cli::try_parse_from(["cerulion", "viz", "--listen", "tcp/0.0.0.0:7447"])
        else {
            panic!("--listen without --robot must be a parse error");
        };
        assert!(
            err.to_string().contains("--robot"),
            "the error must name the required --robot flag:\n{err}"
        );

        // Control: WITH --robot both parse fine (the gate is satisfied).
        Cli::try_parse_from([
            "cerulion",
            "viz",
            "--robot",
            "go2",
            "--connect",
            "tcp/192.168.123.99:7683",
            "--listen",
            "tcp/0.0.0.0:7447",
        ])
        .expect("--connect/--listen WITH --robot must parse");
    }

    /// The `--connect`/`--listen` help states the
    /// requirement is ENFORCED (`Requires --robot`) and never the misleading
    /// "Ignored without --robot" (which would describe a silent drop, where the
    /// product raises a hard parse error).
    #[test]
    fn viz_connect_listen_help_states_requires_robot_not_ignored() {
        use clap::CommandFactory;
        let mut cmd = Cli::command();
        let viz = cmd
            .find_subcommand_mut("viz")
            .expect("the viz subcommand exists");
        let help = viz.render_long_help().to_string();
        let flat: String = help.split_whitespace().collect::<Vec<_>>().join(" ");
        assert!(
            flat.contains("Requires `--robot`"),
            "connect/listen help must state the requirement is enforced:\n{help}"
        );
        assert!(
            !flat.contains("Ignored without `--robot`"),
            "the misleading 'Ignored without --robot' footnote must be gone:\n{help}"
        );
    }

    /// `--robot-name` parses to `Some(name)`; its ABSENCE
    /// defaults to `None` (the identity is then derived from the topics'
    /// shared namespace, else the hostname). Clap-level unit, no process spawn.
    #[test]
    fn ros_attach_robot_name_flag_parses_and_defaults_none() {
        // Absent: robot_name == None.
        let cli = Cli::try_parse_from(["cerulion", "ros2", "attach", "--iface", "10.0.0.7"])
            .expect("iface-only ros2 attach must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Attach { robot_name, .. },
            } => assert_eq!(robot_name, None, "robot_name defaults to None"),
            _ => panic!("expected Ros2::Attach"),
        }
        // Present: robot_name == Some(value).
        let cli = Cli::try_parse_from([
            "cerulion",
            "ros2",
            "attach",
            "--iface",
            "10.0.0.7",
            "--robot-name",
            "spot",
        ])
        .expect("ros2 attach --robot-name must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Attach { robot_name, .. },
            } => assert_eq!(robot_name.as_deref(), Some("spot")),
            _ => panic!("expected Ros2::Attach"),
        }
    }

    /// The global `--verbose` flag also accepts the `-v` short alias, and the
    /// two are equivalent; its ABSENCE defaults to `false`. Clap-level unit.
    #[test]
    fn verbose_flag_accepts_short_and_long_alias() {
        // Absent → false.
        let cli = Cli::try_parse_from(["cerulion", "ros2", "attach", "--iface", "10.0.0.7"])
            .expect("bare subcommand must parse");
        assert!(!cli.verbose, "verbose defaults to false");
        // `-v` → true.
        let short =
            Cli::try_parse_from(["cerulion", "-v", "ros2", "attach", "--iface", "10.0.0.7"])
                .expect("-v must parse");
        assert!(short.verbose, "-v sets verbose true");
        // `--verbose` → true (equivalent to `-v`).
        let long = Cli::try_parse_from([
            "cerulion",
            "--verbose",
            "ros2",
            "attach",
            "--iface",
            "10.0.0.7",
        ])
        .expect("--verbose must parse");
        assert!(long.verbose, "--verbose sets verbose true");
        assert_eq!(
            short.verbose, long.verbose,
            "-v and --verbose are equivalent"
        );
    }
}

#[cfg(test)]
mod verb_log_class_tests {
    use super::*;

    /// Parse a full argv and return the classified verb log class — the exact
    /// path `main` takes (`Cli::parse().command.log_verb_class()`).
    fn class_of(argv: &[&str]) -> VerbLogClass {
        Cli::try_parse_from(argv)
            .unwrap_or_else(|e| panic!("argv {argv:?} must parse: {e}"))
            .command
            .log_verb_class()
    }

    /// One-shot introspection / management verbs default QUIET
    /// (`OneShot` → `cerulion=warn`). `topic echo`/`hz` LOOP but are still
    /// one-shot — their OUTPUT is the data, and the
    /// breadcrumbs are noise.
    #[test]
    fn one_shot_verbs_are_quiet() {
        let one_shot: &[&[&str]] = &[
            &["cerulion", "topic", "list"],
            &["cerulion", "topic", "info", "/t"],
            &["cerulion", "topic", "echo", "/t"],
            &["cerulion", "topic", "hz", "/t"],
            &["cerulion", "schema", "list"],
            &["cerulion", "schema", "info", "sensor_msgs/Image"],
            &["cerulion", "schema", "create", "Foo"],
            &["cerulion", "schema", "delete", "Foo"],
            &["cerulion", "node", "list"],
            &["cerulion", "node", "info", "camera"],
            &[
                "cerulion", "node", "create", "camera", "--policy", "external",
            ],
            &["cerulion", "node", "delete", "camera"],
            &["cerulion", "node", "build", "camera"],
            &["cerulion", "node", "stage", "camera"],
            &["cerulion", "graph", "create", "g"],
            &["cerulion", "graph", "validate", "g"],
            &["cerulion", "graph", "list"],
            &["cerulion", "graph", "levels", "g"],
            &["cerulion", "graph", "partition", "g"],
            &["cerulion", "workspace", "create", "ws"],
            &["cerulion", "workspace", "init"],
            &["cerulion", "trace", "inspect", "somedir"],
            &["cerulion", "clean"],
            // `ros2 migrate` is a run-and-exit rewrite — its
            // report/diff is the product; breadcrumbs would interleave.
            &["cerulion", "ros2", "migrate"],
            &["cerulion", "ros2", "migrate", "--write", "--yes"],
        ];
        for argv in one_shot {
            assert_eq!(
                class_of(argv),
                VerbLogClass::OneShot,
                "{argv:?} must be a QUIET one-shot verb"
            );
            assert!(
                class_of(argv).is_quiet_default(),
                "{argv:?} is_quiet_default() must be true"
            );
        }
    }

    /// `--strict-state` crossed the verb rename and
    /// parses on `bag play`, in both resim modes.
    ///
    /// The RESOLVER half (it reaches the engine in both modes) is pinned by
    /// `resim_cmd::tests::run_shaping_flags_are_legal_in_both_resim_modes`; this
    /// arm pins the half that one structurally cannot see — that clap parses the
    /// flag onto the variant at all, so a `main` destructure which dropped it
    /// would fail here rather than silently ignoring the operator's request.
    #[test]
    fn bag_play_parses_strict_state_in_both_resim_modes() {
        for (argv, want_verify) in [
            (
                vec![
                    "cerulion",
                    "bag",
                    "play",
                    "b.mcap",
                    "--resim",
                    "all",
                    "--strict-state",
                ],
                false,
            ),
            (
                vec![
                    "cerulion",
                    "bag",
                    "play",
                    "b.mcap",
                    "--resim",
                    "all",
                    "--verify",
                    "--strict-state",
                ],
                true,
            ),
        ] {
            let cli = Cli::try_parse_from(&argv).expect("must parse");
            let Commands::Bag {
                action:
                    BagAction::Play {
                        strict_state,
                        verify,
                        resim,
                        ..
                    },
            } = cli.command
            else {
                panic!("expected `bag play`");
            };
            assert!(strict_state, "{argv:?} must carry --strict-state");
            assert_eq!(verify, want_verify, "{argv:?}");
            assert_eq!(resim.as_deref(), Some("all"), "{argv:?}");
        }
    }

    /// Runtime / daemon verbs keep the `info` default (`LongRunning`).
    #[test]
    fn long_running_verbs_keep_info() {
        let long_running: &[&[&str]] = &[
            &["cerulion", "graph", "run", "g"],
            &["cerulion", "graph", "profile", "g"],
            &["cerulion", "node", "run", "camera"],
            // `bag play --resim` is where the removed `cerulion
            // replay`'s re-execution went, so it inherits its log class — as
            // does plain playback, which was already `LongRunning`.
            &["cerulion", "bag", "play", "bag.mcap", "--resim", "all"],
            &["cerulion", "ros2", "attach", "--iface", "10.0.0.7"],
            &["cerulion", "viz"],
            &["cerulion", "connect", "robot1"],
            // run/launch stay `info` (runtime processes; the
            // classification covers the pre-exec window) while their
            // sibling `ros2 migrate` is OneShot — the split is the pin.
            &["cerulion", "ros2", "run", "demo_nodes_cpp", "talker"],
            &["cerulion", "ros2", "launch", "pkg", "file.launch.py"],
        ];
        for argv in long_running {
            assert_eq!(
                class_of(argv),
                VerbLogClass::LongRunning,
                "{argv:?} must keep the `info` default"
            );
            assert!(
                !class_of(argv).is_quiet_default(),
                "{argv:?} is_quiet_default() must be false"
            );
        }
    }

    /// The hidden multi-process worker / gateway verbs run a runtime loop, so
    /// they keep `info` (they are spawned by `graph run`, never typed).
    #[test]
    fn hidden_worker_verbs_keep_info() {
        assert_eq!(
            class_of(&["cerulion", "graph", "run-worker", "--plan", "/tmp/p.json"]),
            VerbLogClass::LongRunning,
            "run-worker is a runtime process"
        );
        assert_eq!(
            class_of(&[
                "cerulion",
                "graph",
                "run-gateway",
                "--handoff",
                "/tmp/h.json"
            ]),
            VerbLogClass::LongRunning,
            "run-gateway is a runtime process"
        );
    }

    /// `-v/--verbose` is class-independent (it is applied on top of the class
    /// in `init_logging`), so the classification of a verb is the SAME with or
    /// without `-v` — the flag raises the level, it does not reclassify.
    #[test]
    fn verbose_flag_does_not_change_class() {
        assert_eq!(
            class_of(&["cerulion", "-v", "topic", "list"]),
            VerbLogClass::OneShot,
            "-v does not reclassify a one-shot verb"
        );
        assert_eq!(
            class_of(&["cerulion", "-v", "graph", "run", "g"]),
            VerbLogClass::LongRunning,
            "-v does not reclassify a long-running verb"
        );
    }
}

#[cfg(test)]
mod topic_echo_flag_tests {
    use super::*;

    /// Destructure a parsed `topic echo` command into its `truncate_length`.
    fn parse_echo_truncate(args: &[&str]) -> u64 {
        let mut argv = vec!["cerulion", "topic", "echo", "t"];
        argv.extend_from_slice(args);
        let cli = Cli::try_parse_from(argv).expect("`topic echo` must parse");
        match cli.command {
            Commands::Topic {
                action:
                    TopicAction::Echo {
                        truncate_length, ..
                    },
            } => truncate_length,
            _ => panic!("expected Topic::Echo"),
        }
    }

    /// Bare `topic echo <topic>` defaults `--truncate-length` to the
    /// engine's `DEFAULT_ECHO_TRUNCATE_LENGTH` (128) — one source of truth.
    #[test]
    fn topic_echo_bare_defaults_to_128() {
        assert_eq!(
            parse_echo_truncate(&[]),
            cerulion_cli_engine::topic_cmd::DEFAULT_ECHO_TRUNCATE_LENGTH as u64,
        );
    }

    /// An explicit `--truncate-length N` overrides the default.
    #[test]
    fn topic_echo_truncate_length_parses() {
        assert_eq!(parse_echo_truncate(&["--truncate-length", "4"]), 4);
        assert_eq!(parse_echo_truncate(&["--truncate-length", "1"]), 1);
    }

    /// `--truncate-length 0` is REJECTED at parse by the `parse_at_least_one`
    /// value_parser. Asserting the error KIND is `ValueValidation` pins that the
    /// range check (not some unrelated parse failure) is what rejected it: the
    /// element render bound cannot be zero.
    #[test]
    fn topic_echo_truncate_length_zero_rejected() {
        // `Cli` does not derive `Debug`, so match (not `expect_err`) to reach the
        // error without requiring `T: Debug` on the Ok arm.
        match Cli::try_parse_from(["cerulion", "topic", "echo", "t", "--truncate-length", "0"]) {
            Ok(_) => panic!("--truncate-length 0 must be rejected at parse"),
            Err(err) => assert_eq!(
                err.kind(),
                clap::error::ErrorKind::ValueValidation,
                "parse_at_least_one must be the rejecter (got {:?})",
                err.kind()
            ),
        }
    }

    /// A NEGATIVE `--truncate-length` is REJECTED at parse. clap treats
    /// `-5` as an unexpected hyphen-led argument (it looks like a flag) and
    /// errors BEFORE the u64 value_parser / range check ever runs — a
    /// parse-structure rejection, not the `ValueValidation` the zero case hits.
    /// Either way it must be an error.
    #[test]
    fn topic_echo_truncate_length_negative_rejected() {
        assert!(
            Cli::try_parse_from(["cerulion", "topic", "echo", "t", "--truncate-length", "-5"])
                .is_err(),
            "a negative --truncate-length must be rejected at parse"
        );
    }

    /// The `(run, topics)` a `bag record` argv parses to, or `None` when the
    /// argv is not a `bag record` at all.
    fn parse_bag_record(argv: &[&str]) -> Option<(Option<String>, Vec<String>)> {
        let cli = Cli::try_parse_from(argv).ok()?;
        match cli.command {
            Commands::Bag {
                action: BagAction::Record { run, topics, .. },
            } => Some((run, topics)),
            _ => None,
        }
    }

    /// `--run` must not SWALLOW the first positional topic.
    ///
    /// The verb's whole point is that a run and a topic list COMPOSE — attach to
    /// a run for its graph/env/identity, record a hand-named subset of it — and
    /// with a bare `num_args = 0..=1` that composition was unspellable: clap
    /// bound `/topic` as the RUN NAME, leaving zero topics, so the operator got
    /// a "no such run" refusal naming their own topic. `require_equals` is what
    /// separates the two, and all three shapes are pinned together because the
    /// fix is exactly a re-partition of one argv space.
    #[test]
    fn bare_run_does_not_swallow_the_first_positional_topic() {
        let (run, topics) = parse_bag_record(&["cerulion", "bag", "record", "--run", "/telemetry"])
            .expect("`bag record --run /telemetry` must parse");
        assert_eq!(
            run.as_deref(),
            Some(""),
            "a bare --run must take its default_missing_value (the SOLE-run target), never the \
             next word"
        );
        assert_eq!(
            topics,
            vec!["/telemetry".to_string()],
            "the word after a bare --run is a TOPIC — this composition is the one the verb \
             documents and it must survive parsing"
        );
    }

    /// The other half: `--run=NAME` still NAMES a run, and topics beside it are
    /// still topics. Without this arm, the one above is satisfied by an arg that
    /// takes no value at all.
    #[test]
    fn run_equals_names_a_run_and_still_composes_with_topics() {
        let (run, topics) =
            parse_bag_record(&["cerulion", "bag", "record", "--run=perception", "/a", "/b"])
                .expect("`bag record --run=perception /a /b` must parse");
        assert_eq!(
            run.as_deref(),
            Some("perception"),
            "--run=<RUN> is how a run is named"
        );
        assert_eq!(topics, vec!["/a".to_string(), "/b".to_string()]);
    }

    /// A bare `--run` with NO topics is still the sole-run attach — the shape
    /// `bag record --run` is documented as, and the one `default_missing_value`
    /// exists for.
    #[test]
    fn bare_run_alone_is_the_sole_run_target() {
        let (run, topics) =
            parse_bag_record(&["cerulion", "bag", "record", "--run"]).expect("`--run` must parse");
        assert_eq!(run.as_deref(), Some(""));
        assert!(
            topics.is_empty(),
            "nothing follows the flag, so nothing is a topic"
        );
    }
}

#[cfg(test)]
mod ros2_run_dispatch_tests {
    use super::*;

    /// Parse argv and destructure the `Ros2` action into (is_launch, args).
    /// This pins the clap FALLBACK path only — the primary dispatch is
    /// `main`'s raw-argv intercept, which never parses at all (that is what
    /// makes a LEADING hyphenated token forwardable; the e2e pins it).
    fn parse_ros2(argv: &[&str]) -> (bool, Vec<String>) {
        let cli = Cli::try_parse_from(argv).expect("argv must parse");
        match cli.command {
            Commands::Ros2 {
                action: Ros2Action::Run { args },
            } => (false, args),
            Commands::Ros2 {
                action: Ros2Action::Launch { args },
            } => (true, args),
            _ => panic!("expected Commands::Ros2"),
        }
    }

    /// `ros2 run`: everything after the verb token is collected verbatim.
    #[test]
    fn ros2_run_collects_args_verbatim() {
        let (is_launch, args) = parse_ros2(&[
            "cerulion",
            "ros2",
            "run",
            "demo_nodes_cpp",
            "talker",
            "--ros-args",
            "-r",
            "chatter:=c2",
        ]);
        assert!(!is_launch);
        assert_eq!(
            args,
            [
                "demo_nodes_cpp",
                "talker",
                "--ros-args",
                "-r",
                "chatter:=c2"
            ]
        );
    }

    /// `ros2 launch`: the package form and launch arguments pass through
    /// untouched — nothing here knows what a launch file is.
    #[test]
    fn ros2_launch_collects_package_form_and_launch_args() {
        let (is_launch, args) = parse_ros2(&[
            "cerulion",
            "ros2",
            "launch",
            "moveit2_tutorials",
            "demo.launch.py",
            "use_rviz:=false",
        ]);
        assert!(is_launch);
        assert_eq!(
            args,
            ["moveit2_tutorials", "demo.launch.py", "use_rviz:=false"]
        );
    }

    /// Hyphenated tokens AFTER a positional are collected raw
    /// (trailing_var_arg + allow_hyphen_values) — clap never interprets them.
    #[test]
    fn ros2_post_token_hyphenated_args_are_collected_raw() {
        let (_, args) = parse_ros2(&[
            "cerulion",
            "ros2",
            "launch",
            "demo.launch.py",
            "--show-args",
        ]);
        assert_eq!(args, ["demo.launch.py", "--show-args"]);
    }

    /// Bare `cerulion ros2` requires a subcommand (clap's exit-2 usage error
    /// — the one usage class the wrappers own).
    #[test]
    fn ros2_without_action_is_a_usage_error() {
        assert!(
            Cli::try_parse_from(["cerulion", "ros2"]).is_err(),
            "ros2 requires an action"
        );
    }
}
