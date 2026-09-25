// SPDX-License-Identifier: AGPL-3.0-only
//! Cerulion CLI — thin wrapper around `cerulion_cli_engine`.

mod cli;
mod completion;
// Pins for the shell-facing WIRING (which arg carries
// which completer, the path hints, the create arms completing nothing). A
// binary-crate unit test because `cerulion_cli` has no library target, so an
// integration test could only reach `Cli::command()` by spawning the binary.
#[cfg(test)]
mod completion_wiring_tests;
// Pins that `resim_cmd`'s platform gating and `main`'s references to it
// agree. A Unix host builds only the `cfg(unix)` arm, so the non-Unix build is
// verified by READING both files rather than by compiling one.
#[cfg(test)]
mod cfg_symmetry_tests;
// Pins that `cerulion clean` really runs the `/tmp/*.shm_state`
// diagnostic, and runs it AFTER the dead-node sweep. A source walk because the
// diagnostic reads (and can delete from) the one directory iceoryx2 hardcodes,
// so a behavioural test would have to operate on the real `/tmp`.
#[cfg(test)]
mod clean_diagnostic_tests;

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use clap::{CommandFactory, Parser};

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::workspace::CerulionWorkspace;
use cerulion_cli_engine::{
    account_cmd, connect_cmd, graph_cmd, login_cmd, node_cmd, pair_cmd, partition_emit, ros_cmd,
    schema_cmd, topic_cmd, viz_client, workspace,
};
use cli::{
    AccountAction, BagAction, Cli, Commands, DevicesAction, GraphAction, NodeAction, Ros2Action,
    SchemaAction, TopicAction, TraceAction, WorkspaceAction,
};

/// The migration error for the REMOVED `cerulion ros` family (`ros attach`
/// lives in the `ros2` family as `cerulion ros2 attach` —
/// HARD BREAK, no alias). Printed by the `main` intercept ABOVE the login
/// gate (the rule: you do not have to prove who you are to be told
/// the verb moved) and pinned byte-for-byte by
/// `tests/ros2_attach_verb_test.rs`.
const ROS_VERB_MOVED_MSG: &str = "`cerulion ros attach` has moved: the verb is now `cerulion ros2 attach` (the `ros` family was folded into `ros2`). Re-run the same invocation with `ros2` in place of `ros`.";

fn main() -> ExitCode {
    // The shell completion hook, FIRST — before any other statement.
    //
    // When `COMPLETE=<shell>` is set (only ever by the code
    // `cerulion completions <shell>` installs), this prints candidates or the
    // registration script and EXITS; otherwise it returns immediately and the
    // normal run proceeds. It must precede everything because `CompleteEnv`
    // owns stdout for that invocation — anything written before it becomes
    // garbage in the user's shell — and because a completion should pay none
    // of the startup below (logging, the iceoryx2 log bridge, the login gate).
    //
    // The factory it takes is the SAME `Cli::command` this binary parses with,
    // which is what makes every subcommand, flag and enum value complete for
    // free and keeps completions from drifting out of sync with the CLI.
    clap_complete::CompleteEnv::with_factory(Cli::command).complete();

    // Quiet iceoryx2's own logger by default. It writes a `[W] Notifier { ... }`
    // pretty-print of internal state on every notify path, which dwarfs our
    // tracing output in the demo. Users can still override by exporting
    // `IOX2_LOG_LEVEL` themselves.
    if std::env::var_os("IOX2_LOG_LEVEL").is_none() {
        std::env::set_var("IOX2_LOG_LEVEL", "error");
    }
    // The `set_var` above only INHERITS into child processes (the
    // gateway, `graph run-worker`s, bagd) — it does NOT set THIS process's
    // iceoryx2 level, which lives in an `iceoryx2-log` static. Apply it here,
    // before any iceoryx2 call can emit, rather than relying on whichever
    // `TransportManager::init*` happens to run first (some verbs never init a
    // transport at all). See `init_iceoryx_log_level`'s docs for why every
    // linked copy — including each cdylib node — must do this for itself.
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();

    // Decision (the ros2 verb split): `cerulion ros2 run|launch` is a
    // VERBATIM pass-through of the native verb — everything after the verb
    // token is forwarded to `ros2 <verb>` untouched, hyphenated tokens
    // included. clap would claim a leading `--prefix` / `-s` as its own
    // flag, so the dispatch happens HERE, on the raw argv, before
    // `Cli::parse()`. `cerulion ros2` bare / `--help` / an unknown action
    // still fall through to clap (family help + its usage error).
    #[cfg(unix)]
    if let Some(code) = ros2_passthrough_intercept() {
        return code;
    }

    let cli = Cli::parse();
    // bagd is folded into `cerulion` as a subcommand: dispatch
    // the recorder subcommand BEFORE `init_logging` so `bagd_cli_main` installs
    // its own logging default (the old standalone binary's exact behavior; its
    // `try_init` would silently no-op behind cerulion's subscriber otherwise).
    #[cfg(unix)]
    let cli = match cli {
        Cli {
            command: Commands::Bagd(args),
            ..
        } => return cerulion_bagd::bagd_cli_main(*args),
        other => other,
    };
    // Non-Unix stub (mirrors `graph run --record`'s): bagd is Unix-only — the
    // recorder lifecycle is driven by SIGTERM over POSIX shared memory.
    #[cfg(not(unix))]
    if matches!(cli.command, Commands::Bagd) {
        eprintln!(
            "Error: `cerulion bagd` is only supported on Unix platforms (the recorder is \
             driven by SIGTERM lifecycle signals over POSIX shared memory)"
        );
        return ExitCode::FAILURE;
    }
    // The REMOVED `cerulion ros` family (folded into `ros2`): answer with the
    // migration message HERE — above `init_logging` (a pure eprintln needs no
    // subscriber) and above the login gate (the rule: you do not have
    // to prove who you are to be told the verb moved). Exit 2, clap's own
    // usage-error code: the invocation never ran, and a script must be able
    // to tell "your command line is stale" from a runtime failure (1).
    if matches!(cli.command, Commands::Ros { .. }) {
        eprintln!("Error: {ROS_VERB_MOVED_MSG}");
        return ExitCode::from(2);
    }

    // The TUI takes over the terminal and is fragile to anything writing
    // to stderr/stdout out-of-band — every `tracing::info!("transport
    // manager initialized")` (and friends) shreds the ratatui frame.
    // Skip the tracing setup entirely in TUI mode; users who need debug
    // output can run `RUST_LOG=... cerulion graph run` in a sibling pane.
    if !matches!(cli.command, Commands::Tui) {
        // One-shot verbs (topic/schema/node list/… — run-and-exit)
        // default to a quiet `warn` filter so their output isn't interleaved
        // with lifecycle breadcrumbs; long-running / runtime verbs keep `info`.
        // `-v` raises either to `debug`; an explicit `RUST_LOG` always wins.
        init_logging(cli.verbose, cli.command.log_verb_class().is_quiet_default());
    }
    // Bridge iceoryx2's internal logger to `tracing`
    // so iceoryx2's diagnostics (cleanup failures, lock contention,
    // etc.) flow through Cerulion's standard log pipeline AND can
    // be captured per-call by `cerulion clean` for structured
    // failure reporting. Idempotent: returns false if iceoryx2
    // already has a logger; we don't care which install wins
    // because both bridges have the same destination semantics.
    let _ = cerulion_core::iceoryx_logger::install_iceoryx2_tracing_bridge();

    // The first-need runtime login gate. Every
    // identity-needing command runs under a logged-in-ever account — the gate
    // reads LOCAL state (microseconds; zero network on the proceed path) and, on
    // a machine that has never signed in, either runs the device-code flow
    // inline (at a terminal) or refuses at once (anywhere else). The gate is on
    // in every build, released or built from source; the one escape is this
    // repository's own runs, which set `CERULION_LOGIN_GATE=off` (see
    // `login_cmd` and `docs/internals/cli.md`). Internal
    // subprocess verbs (bagd, already dispatched above; `graph
    // run-worker`/`run-gateway`, spawned by a gated parent) and `login` itself
    // are exempt (`command_needs_identity`). The prompt/refusal rides stderr so
    // it never pollutes a command's stdout.
    //
    // A REMOVED verb needs no intercept here. `cerulion replay` must not be
    // blocked on a device-code prompt before the user learns it is gone, and
    // with no variant declared for it that refusal
    // comes EARLIER (clap rejects it while parsing) and on every
    // platform, so the property holds without any code here.
    //
    // A malformed resim invocation is answered
    // here, ABOVE the login gate, for the same reason the removed `replay` verb
    // is — you do not have to prove who you are to be told your command line is
    // wrong. `bag play` is identity-gated (`command_needs_identity` exempts only
    // `login`, `completions` and the two internal `graph run-worker` /
    // `run-gateway` subprocess verbs), and on a never-signed-in machine
    // `ensure_login_gate` runs the DEVICE-CODE FLOW inline, so `cerulion bag
    // play b.mcap --verify` would open a browser prompt and exit 7 (auth)
    // instead of exit 2 (usage), never reaching the refusal that would have
    // named `--resim`.
    //
    // Only the VALIDATION is hoisted, never the run: `resolve_play_mode` is a
    // pure function of the parsed flags (no I/O, no transport, no bag read), so
    // it is safe above the gate. A WELL-FORMED resim still falls through to the
    // gate and authenticates before a single node executes — re-execution is a
    // runtime verb and stays gated.
    //
    // The cost is that `resolve_play_mode` runs twice on the happy path. It is
    // pure and total, so the second call is the same answer for free; the
    // alternative (threading the resolved mode down) would put the gate INSIDE
    // the resim path and make the ordering an implementation detail rather than
    // a visible property of `main`.
    //
    // The refusal is hoisted for EVERY `bag play`, not
    // only the resim family. `--duration` is legal on BOTH halves, so a
    // malformed one (`--duration -1`) on the playback half fell through to the
    // generic dispatch and exited 1 — while the identical mistake on
    // `--start-offset` exited 2, because that flag puts the invocation in the
    // family. A malformed VALUE is a usage error on both halves, and on this
    // surface 1 means "your code diverged": a CI job cannot be left to tell a
    // typo from a regression by reading stderr. `resolve_play_mode` is pure and
    // total and answers `Ok(Playback)` for every well-formed playback
    // invocation, so widening the gate refuses nothing it did not refuse
    // before — it only changes which CODE the refusal spends.
    if let Commands::Bag { action } = &cli.command {
        if let Some(refusal) = resim_usage_refusal(action) {
            eprintln!("Error: {refusal}");
            return ExitCode::from(cerulion_cli_engine::resim_cmd::EXIT_USAGE);
        }
    }

    if command_needs_identity(&cli.command) {
        if let Err(e) = login_cmd::ensure_login_gate(&mut std::io::stderr()) {
            eprintln!("Error: {e}");
            // Exit 7 (not the generic FAILURE=1): the login gate is a
            // "command did not run" refusal, and 1 collides with `bag play
            // --resim --verify`'s EXIT_VIOLATION. A distinct code lets a CI caller tell
            // auth-refusal apart from a data violation. The constant lives in
            // `login_cmd` (the platform-independent login gate), NOT the
            // `#[cfg(unix)]` `replay_cmd`, so this reference compiles on every
            // target; the full exit-code registry is documented in `replay_cmd`.
            return ExitCode::from(cerulion_cli_engine::login_cmd::EXIT_AUTH_REQUIRED);
        }
    }

    // `cerulion bag play --resim` inherits the stable 0–6
    // exit-code contract, which the generic run() dispatch (it collapses every
    // error to FAILURE) cannot express. Intercept here and map the engine's
    // typed result to an ExitCode. A `bag play` with NO `--resim` is ordinary
    // playback and falls through to run() untouched.
    if let Commands::Bag { action } = &cli.command {
        if is_resim_family(action) {
            return resim_exit_code(cli.command);
        }
    }

    // `cerulion connect` SPAWNS the `cerulion-connectd` sibling
    // binary (it links iroh; the `cerulion` CLI stays iroh-free) and FORWARDS its
    // exit code — the generic run() dispatch (SUCCESS/FAILURE only) cannot express
    // that, so intercept here like `bag play --resim`.
    if matches!(cli.command, Commands::Connect { .. }) {
        return connect_exit_code(cli.command);
    }

    // `cerulion pair` SPAWNS `cerulion-connectd pair` (it links iroh; the
    // `cerulion` CLI stays iroh-free) and forwards its 0–4 pairing exit code — the
    // generic run() dispatch (SUCCESS/FAILURE only) cannot express that, so
    // intercept here like `connect` / `bag play --resim`.
    if matches!(cli.command, Commands::Pair { .. }) {
        return pair_exit_code(cli.command);
    }

    // Belt-and-braces for `cerulion ros2` invocations that reached clap
    // anyway (the raw-argv intercept above `Cli::parse()` handles every
    // direct spelling): forward the parsed action through the same exec
    // dispatch. run/launch deliberately install NO ctrlc handler: after
    // exec() the real ros2 owns the process group and SIGINT. `migrate
    // --write` is the exception (interrupt safety): it arms the
    // flag-flipping handler inside `ros2_migrate_exit_code` so a Ctrl-C
    // mid-write rolls back instead of stranding a partial migration.
    // `ros2 attach` is a NATIVE clap verb (formerly `ros attach`) and falls
    // through to the generic run() dispatch below.
    if matches!(
        cli.command,
        Commands::Ros2 {
            action: Ros2Action::Run { .. } | Ros2Action::Launch { .. } | Ros2Action::Migrate { .. }
        }
    ) {
        let Commands::Ros2 { action } = cli.command else {
            unreachable!("guarded by the matches! above")
        };
        return match action {
            Ros2Action::Run { args } => {
                ros2_exit_code(cerulion_cli_engine::ros2_cmd::Ros2NativeVerb::Run, args)
            }
            Ros2Action::Launch { args } => {
                ros2_exit_code(cerulion_cli_engine::ros2_cmd::Ros2NativeVerb::Launch, args)
            }
            // NOT a pass-through — the migrate verb owns its own
            // argument surface and exit codes (69 = engine not built,
            // mirroring the run/launch missing-library code).
            Ros2Action::Migrate {
                workspace,
                write,
                yes,
            } => ros2_migrate_exit_code(workspace, write, yes),
            Ros2Action::Attach { .. } => unreachable!("guarded by the matches! above"),
        };
    }

    match run(cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {}", e);
            ExitCode::FAILURE
        }
    }
}

/// The usage refusal for a `bag play` resim-family invocation, or
/// `None` if the flags are legal.
///
/// Pure — it clones the parsed flags and asks the engine's
/// [`cerulion_cli_engine::resim_cmd::resolve_play_mode`] oracle, touching no
/// bag, no transport and no network. That purity is what lets `main` call it
/// ABOVE the login gate.
///
/// The `Ok(Playback)` arm IS the ordinary answer, because the caller
/// covers every `bag play` (see `main`): a well-formed playback
/// invocation yields no refusal and falls through to the existing dispatch.
fn resim_usage_refusal(action: &BagAction) -> Option<String> {
    cerulion_cli_engine::resim_cmd::resolve_play_mode(play_flags_of(action)?).err()
}

/// The ONE clap-variant → [`cerulion_cli_engine::resim_cmd::PlayFlags`]
/// mapping.
///
/// Extracted so it is written once and can be TESTED. Everything downstream of
/// it was already pinned — `PlayFlags` → `ResimOptions` by
/// `resim_cmd::tests::run_shaping_flags_are_legal_in_both_resim_modes`,
/// `ResimOptions` → `ReplayOptions` field-by-field by
/// `every_knob_survives_the_conversion_into_the_engines_options`, and the
/// engine's own `--strict-state` semantics twice in `replay_engine_test`. And
/// clap's argv → VARIANT half is pinned by
/// `cli.rs::bag_play_parses_strict_state_in_both_resim_modes`.
///
/// This mapping was the one link between those two chains with no test on it,
/// which matters most for `--strict-state`: that flag is documented INERT on a
/// bag beginning at step 0, so the CLI e2e that exercises it cannot observe it
/// being dropped — a `strict_state: false` written here would have passed the
/// entire suite while silently discarding an operator's explicit refusal
/// request. Pinned field-by-field by `resim_flag_mapping_tests`.
///
/// `None` for a non-`Play` action: `bag info` / `bag record` carry no resim
/// flags at all.
fn play_flags_of(action: &BagAction) -> Option<cerulion_cli_engine::resim_cmd::PlayFlags> {
    let BagAction::Play {
        resim,
        verify,
        rate,
        repeat,
        topics,
        duration,
        start_offset,
        report,
        tolerance,
        strict_state,
        ..
    } = action
    else {
        return None;
    };

    Some(cerulion_cli_engine::resim_cmd::PlayFlags {
        resim: resim.clone(),
        verify: *verify,
        rate: *rate,
        repeat: *repeat,
        topics: topics.clone(),
        duration: *duration,
        start_offset: *start_offset,
        report: report.clone(),
        tolerance: tolerance.clone(),
        strict_state: *strict_state,
    })
}

/// Does this `bag <action>` belong to the resim surface?
///
/// TRUE for `--resim` itself AND for every flag that is RESIM-ONLY — one that
/// means nothing without it. Deliberately not just `resim.is_some()`: a
/// resim-only flag given WITHOUT `--resim` is still a resim-surface misuse, and
/// routing it through the generic dispatch would answer it with the generic
/// failure code 1 — the one code this surface must never spend on a malformed
/// invocation (see `resim_cmd::EXIT_USAGE`). One predicate, one code, for the
/// whole family.
///
/// A PLAYBACK-ONLY flag is NOT a family member: it is legitimate on its own,
/// and its misuse (giving it WITH `--resim`) is already routed here by
/// `--resim` itself. See the body for the defect that reading cost.
///
/// A `bag play` carrying NO family flag is ordinary playback and falls
/// through to `run()`. "Falls through" is about THIS predicate only: the
/// pre-auth usage gate above runs `resolve_play_mode`
/// for EVERY `bag play`, so a malformed playback VALUE (`--duration -1`) is
/// refused there with `EXIT_USAGE` rather than reaching the dispatch. What
/// reaches `run()` unchanged is a WELL-FORMED playback invocation.
fn is_resim_family(action: &BagAction) -> bool {
    match action {
        BagAction::Play {
            resim,
            verify,
            report,
            tolerance,
            strict_state,
            ..
        } => {
            // The two BOTH-HALVES flags are deliberately ABSENT
            // from this predicate, and for the SAME reason. `--duration` is
            // legal in both halves outright, and `--start-offset` is
            // PLAYBACK-ONLY — so neither one, on its own, makes an invocation a
            // resim.
            //
            // `--start-offset` is NOT listed here. The argument for listing it is that
            // giving it WITH `--resim` is a resim-surface misuse owed
            // `EXIT_USAGE`. That is right about the case and wrong
            // about the predicate: `resim.is_some()` ALREADY routes that
            // invocation here, so the extra disjunct could only ever fire when
            // `--resim` is ABSENT — i.e. on exactly the legitimate playback
            // invocation, which it would send to `run_play_resim`, answering a
            // well-formed `bag play <bag> --start-offset 1` with "no `--resim`
            // given" and exit 2 instead of seeking and republishing. Pinned by
            // `is_resim_family_tests` here and behaviourally over the real
            // binary by `replay_cli_test::
            // a_playback_start_offset_reaches_the_player_not_the_resim_surface`.
            //
            // The resim-ONLY flags stay: `resolve_play_mode` refuses each of
            // them without `--resim` at the pre-auth gate above, so they are
            // belt-and-braces rather than the live route, and listing them
            // keeps this predicate's answer independent of that gate's scope.
            resim.is_some() || *verify || report.is_some() || tolerance.is_some() || *strict_state
        }
        // `bag migrate` REWRITES a bag rather than executing one — no
        // resim flags exist on it, so it can never be a resim-surface misuse.
        BagAction::Info { .. } | BagAction::Record { .. } | BagAction::Migrate { .. } => false,
    }
}

/// Run `cerulion bag play --resim` and map its result to a process
/// exit code.
///
/// Two contracts meet here, and which one applies is `--verify`:
///
/// - **NEUTRAL** (`--resim all`) re-executes the bag's graph and reports what
///   happened WITHOUT claiming it matches the recording, so a completed
///   re-execution is exit 0 whatever the bytes did. What it does NOT swallow is
///   failure to re-execute at all — an unreadable or not-replay-grade bag (2),
///   a cdylib that would not load (3), a candidate node that PANICKED (3), an
///   internal error (5) all keep their codes.
/// - **`--verify`** adds the byte-comparison and restores the whole replay
///   taxonomy, precedence 3 > 6 > 1 (root cause over symptoms) included.
///
/// The legality of the flag combination is decided first, by
/// [`cerulion_cli_engine::resim_cmd::resolve_play_mode`]; an illegal one is a
/// loud refusal and never
/// reaches the engine.
#[cfg(unix)]
fn resim_exit_code(command: Commands) -> ExitCode {
    let Commands::Bag { action } = &command else {
        unreachable!("`resim_exit_code` is reached only for a `bag play` resim-family invocation")
    };
    let (BagAction::Play { bag, .. }, Some(flags)) = (action, play_flags_of(action)) else {
        unreachable!("`resim_exit_code` is reached only for a `bag play` resim-family invocation")
    };

    // ONE mapping, shared with the pre-auth `resim_usage_refusal` above — so
    // the flags the usage check validates are, by construction, the flags the
    // run receives. Two hand-written destructures could disagree.
    ExitCode::from(cerulion_cli_engine::resim_cmd::run_play_resim(bag, flags))
}

/// Dispatch `cerulion bag <play|info|record>`.
///
/// `play` and `record` are LOOP verbs driven by the shared Ctrl-C `running`
/// flag; `info` is a pure render. Operator output goes to stdout (it is the
/// command's product), lifecycle detail to `tracing`.
#[cfg(unix)]
fn run_bag(action: BagAction) -> CliResult<()> {
    use cerulion_cli_engine::bag_cmd;
    use std::io::Write as _;

    match action {
        BagAction::Info { bag } => {
            // Workspace-OPTIONAL: inside one, the schemas/ store lets a
            // channel's wire hash resolve to a real name so the banner can say
            // truthfully whether a viewer will render it.
            let ws = discover_workspace().ok();
            let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.clone());
            print!("{}", bag_cmd::bag_info(&bag, schemas_dir.as_deref())?);
            std::io::stdout().flush()?;
            Ok(())
        }
        BagAction::Play {
            bag,
            resim,
            verify,
            rate,
            repeat,
            topics,
            duration,
            start_offset,
            report,
            tolerance,
            strict_state,
        } => {
            // A `--resim` invocation never reaches here — `main`
            // intercepts it for the 0–6 exit contract. What DOES reach here is
            // plain playback, possibly carrying a resim-only flag, which
            // `resolve_play_mode` refuses by name.
            let mode = cerulion_cli_engine::resim_cmd::resolve_play_mode(
                cerulion_cli_engine::resim_cmd::PlayFlags {
                    resim,
                    verify,
                    rate,
                    repeat,
                    topics,
                    duration,
                    start_offset,
                    report,
                    tolerance,
                    strict_state,
                },
            )
            .map_err(cerulion_cli_engine::error::CliError::Validation)?;
            let cerulion_cli_engine::resim_cmd::PlayMode::Playback {
                rate,
                repeat,
                topics,
                duration_bound_ns,
                start_offset_ns,
            } = mode
            else {
                unreachable!("`bag play --resim` is dispatched in main() before run()")
            };
            let running = setup_ctrlc_handler()?;
            let mut out = std::io::stdout();
            let summary = bag_cmd::bag_play(
                &bag,
                bag_cmd::PlayOptions {
                    rate,
                    repeat,
                    topics,
                    start_offset_ns,
                    duration_bound_ns,
                    schemas_dir: discover_workspace().ok().map(|w| w.schemas_dir),
                },
                running,
                &mut out,
            )?;
            print!("{}", bag_cmd::render_play_summary(&summary));
            out.flush()?;
            Ok(())
        }
        BagAction::Record {
            topics,
            all,
            regex,
            exclude,
            out: out_path,
            duration,
            schema_wait_ms,
            run,
        } => {
            let running = setup_ctrlc_handler()?;
            let mut out = std::io::stdout();
            let summary = bag_cmd::bag_record(
                bag_cmd::RecordOptions {
                    topics,
                    all,
                    regex,
                    exclude,
                    out: out_path,
                    duration: duration.map(std::time::Duration::from_secs),
                    // No CLI flag sets this: one recording is one artifact.
                    // The field
                    // stays for library callers; see `BagdConfig::size_cap_bytes`.
                    max_bag_size: None,
                    schema_wait: std::time::Duration::from_millis(schema_wait_ms),
                    // `--run` alone is `Sole` (attach to the one
                    // live run); `--run <ID>` names one. clap's
                    // `default_missing_value = ""` is how the bare form arrives,
                    // so the empty string means "the flag was given without a
                    // value" — never a run whose id is empty, which the registry
                    // codec refuses outright.
                    run: run.map(|r| {
                        if r.trim().is_empty() {
                            bag_cmd::RunTarget::Sole
                        } else {
                            bag_cmd::RunTarget::Named(r)
                        }
                    }),
                    // Inside a workspace, the schemas/ store is what
                    // lets the recorder NAME the hashes it taps and stamp their
                    // text into the bag. Outside one, the bag still records
                    // exactly as before.
                    schemas_dir: discover_workspace().ok().map(|w| w.schemas_dir),
                },
                running,
                &mut out,
            )?;
            print!("{}", bag_cmd::render_record_summary(&summary));
            out.flush()?;
            Ok(())
        }
        // Rewrite a bag whose embedded `graph.yaml` carries keys the
        // graph format no longer defines, into a NEW bag. The engine owns the
        // whole consent ladder (dry-run / --yes / no-TTY refusal / interactive
        // confirm); the binary supplies the REAL TTY probe, the y/N prompt, and
        // the wall clock the migration record is stamped with — which is the
        // one value a migration is not a pure function of, and so is passed IN
        // rather than read inside the rewrite (see `MigrationStamp`).
        BagAction::Migrate {
            bag,
            out: out_path,
            dry_run,
            yes,
        } => {
            use cerulion_cli_engine::bag_migrate;
            use std::io::IsTerminal as _;

            let opts = bag_migrate::MigrateOptions {
                out: out_path,
                dry_run,
                assume_yes: yes,
            };
            let is_tty = std::io::stdin().is_terminal();
            let mut confirm =
                |preview: &str| prompt_yes_no(preview, "Write the migrated bag to disk?");
            let report = bag_migrate::bag_migrate(
                &bag,
                &opts,
                bag_migrate::MigrationStamp::now(),
                is_tty,
                &mut confirm,
            )?;
            if !report.preview_shown {
                print!("{}", report.preview);
            }
            match report.outcome {
                bag_migrate::MigrateOutcome::DryRun => {
                    println!("\ndry run — nothing written.");
                }
                bag_migrate::MigrateOutcome::Written => {
                    println!("\nMigrated bag written to {}", report.output.display());
                    println!("The original '{}' is unchanged.", report.input.display());
                }
                bag_migrate::MigrateOutcome::Declined => {
                    println!("\naborted — nothing written.");
                }
            }
            std::io::stdout().flush()?;
            Ok(())
        }
    }
}

/// Non-Unix stub: the `cerulion bag` family is unavailable (the bag reader /
/// writer is `#![cfg(unix)]`), so the verb refuses by name instead of failing
/// to compile.
#[cfg(not(unix))]
fn run_bag(_action: BagAction) -> CliResult<()> {
    Err(cerulion_cli_engine::error::CliError::Validation(
        "`cerulion bag` is only supported on Unix platforms (the bag reader/writer depends on \
         Unix-only POSIX trace-ring types)"
            .to_string(),
    ))
}

/// Non-Unix stub: `cerulion bag play --resim` is unavailable (the bag reader is
/// `#![cfg(unix)]`). Mirrors the `cerulion bagd` platform stub.
#[cfg(not(unix))]
fn resim_exit_code(_command: Commands) -> ExitCode {
    eprintln!(
        "Error: `cerulion bag play --resim` is only supported on Unix platforms (the bag reader \
         depends on Unix-only POSIX trace-ring types)"
    );
    ExitCode::FAILURE
}

/// Resolve + spawn `cerulion-connectd`, streaming its stdio and
/// FORWARDING its exit code. The `cerulion` CLI stays iroh-free — the desk iroh
/// half lives entirely in the spawned binary (the license + build-graph boundary
/// is a subprocess, mirroring the gateway / `vizd`).
///
/// A signal handler keeps `cerulion` alive through Ctrl-C so it REAPS the child:
/// the child receives the SAME terminal SIGINT and shuts down gracefully, and we
/// forward its exit code. `.status()` inherits stdio, so the child's catalog
/// (stdout) + logs (stderr) stream straight to the user.
fn connect_exit_code(command: Commands) -> ExitCode {
    let Commands::Connect {
        robot,
        eid,
        addrs,
        topics,
        all,
        key_file,
        schemas_dir,
        relay_url,
        relay_disabled,
        network,
    } = command
    else {
        unreachable!("connect_exit_code is only called for Commands::Connect");
    };
    let args = connect_cmd::ConnectArgs {
        robot,
        eid,
        addrs,
        topics,
        all,
        key_file,
        schemas_dir,
        relay_url,
        relay_disabled,
        network,
    };
    let plan = match connect_cmd::plan(&args) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Error: {e}");
            return ExitCode::FAILURE;
        }
    };
    // Install the signal handler + FORWARD a directed SIGINT/SIGTERM/SIGHUP to the
    // child (f4): `setup_ctrlc_handler` keeps `cerulion` alive on the signal and
    // flips `running`; `spawn_and_wait` polls that flag and, on a directed signal
    // the child did NOT receive, forwards SIGINT to it + reaps it — so `cerulion`
    // never hangs in `.status()` and the child is never orphaned. A failed handler
    // install degrades to the default disposition (both get the terminal signal).
    let running = match setup_ctrlc_handler() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("warning: could not install a signal handler: {e}");
            Arc::new(AtomicBool::new(true))
        }
    };
    tracing::info!(bin = %plan.bin.display(), "cerulion connect: spawning cerulion-connectd");
    match connect_cmd::spawn_and_wait(&plan, running) {
        Ok(code) => ExitCode::from(code as u8),
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Resolve + spawn `cerulion-connectd pair`, streaming its stdio,
/// FORWARDING its 0–4 pairing exit code, and — on a successful pairing — pinning
/// `name → eid` into `~/.cerulion/robots.toml`. The `cerulion` CLI stays iroh-free
/// (the CPace ceremony lives entirely in the spawned sibling), exactly as
/// `cerulion connect` does. A resolution / config error is exit 1 (usage).
fn pair_exit_code(command: Commands) -> ExitCode {
    let Commands::Pair {
        robot,
        eid,
        addrs,
        code,
        account,
        label,
        key_file,
        relay_url,
        relay_disabled,
    } = command
    else {
        unreachable!("pair_exit_code is only called for Commands::Pair");
    };
    let args = pair_cmd::PairArgs {
        robot,
        eid,
        addrs,
        code,
        account,
        label,
        key_file,
        relay_url,
        relay_disabled,
    };
    let plan = match pair_cmd::plan(&args) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Error: {e}");
            // A resolution / config failure is exit 1 (usage), matching the
            // `cerulion-connectd pair` contract.
            return ExitCode::from(1u8);
        }
    };
    // Same directed-signal forwarding as `connect` (see `connect_exit_code`).
    let running = match setup_ctrlc_handler() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("warning: could not install a signal handler: {e}");
            Arc::new(AtomicBool::new(true))
        }
    };
    tracing::info!(bin = %plan.bin.display(), "cerulion pair: spawning cerulion-connectd pair");
    match pair_cmd::spawn_and_pair(&plan, running) {
        Ok(code) => ExitCode::from(code as u8),
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// The raw-argv half of the `cerulion ros2` pass-through: recognize
/// `cerulion [-v|--verbose]... ros2 run|launch [anything...]` and dispatch it
/// WITHOUT clap, so every forwarded token — hyphenated or not — reaches the
/// native `ros2` verbatim. Returns `None` for anything else (bare
/// `cerulion ros2`, `--help`, `ros2 attach` / `ros2 migrate` — NATIVE clap
/// verbs, not pass-throughs — or an unknown action), which clap then owns.
/// Installs the normal logging first (the auto-inject breadcrumb rides
/// `tracing`); a runtime verb, so the `info` default applies.
#[cfg(unix)]
fn ros2_passthrough_intercept() -> Option<ExitCode> {
    let argv: Vec<String> = std::env::args().collect();
    let mut i = 1;
    let mut verbose = false;
    while i < argv.len() && (argv[i] == "-v" || argv[i] == "--verbose") {
        verbose = true;
        i += 1;
    }
    if argv.get(i).map(String::as_str) != Some("ros2") {
        return None;
    }
    let verb = match argv.get(i + 1).map(String::as_str) {
        Some("run") => cerulion_cli_engine::ros2_cmd::Ros2NativeVerb::Run,
        Some("launch") => cerulion_cli_engine::ros2_cmd::Ros2NativeVerb::Launch,
        _ => return None,
    };
    init_logging(verbose, false);
    Some(ros2_exit_code(verb, argv[i + 2..].to_vec()))
}

/// Dispatch `cerulion ros2 run|launch` — resolve the lib dir, stage the
/// minimal ament prefix, build the pass-through exec plan, then `exec()` the
/// real `ros2`. A successful `exec()` never returns (this process BECOMES
/// `ros2` and inherits its exit code); every returned path here is a
/// pre-exec failure mapped to the verb's typed exit contract via
/// `ros2_cmd::classify` — 69 required library missing, or `--adopt-take`
/// given at all (`EX_UNAVAILABLE`: the `ros2` CLI cannot be handed the heap
/// hook), 127 `ros2` not on PATH, 2 an ambient `CERULION_RMW_ADOPT_TAKE`
/// with no flag (this function really does return 2; clap raises its own,
/// upstream, for a bare `cerulion ros2`), 1 other.
#[cfg(unix)]
fn ros2_exit_code(
    verb: cerulion_cli_engine::ros2_cmd::Ros2NativeVerb,
    args: Vec<String>,
) -> ExitCode {
    use cerulion_cli_engine::ros2_cmd;
    // ONE engine call: the ordering (refuse BEFORE staging) is the engine's
    // rule, not this dispatch's — see `plan_ros2_passthrough`.
    let plan = match ros2_cmd::plan_ros2_passthrough(verb, &args) {
        Ok(plan) => plan,
        Err(e) => {
            eprintln!("Error: {e}");
            return ExitCode::from(ros2_cmd::classify(&e));
        }
    };
    // exec() returns ONLY on failure — on success this process IS ros2 now.
    let err = ros2_cmd::exec_ros2(&plan);
    let code = ros2_cmd::classify(&err);
    if code == ros2_cmd::EXIT_ROS2_NOT_FOUND {
        eprintln!(
            "Error: `ros2` not found on PATH — install ROS 2 (or source its setup script) so \
             the real `ros2` is resolvable"
        );
    } else {
        eprintln!("Error: {err}");
    }
    ExitCode::from(code)
}

/// Non-Unix stub: `cerulion ros2` is Unix-only — LD_PRELOAD + rmw_cerulion
/// require the Unix dynamic loader and `exec()` semantics. Mirrors the
/// `bagd` platform stub.
#[cfg(not(unix))]
fn ros2_exit_code(
    _verb: cerulion_cli_engine::ros2_cmd::Ros2NativeVerb,
    _args: Vec<String>,
) -> ExitCode {
    eprintln!(
        "Error: `cerulion ros2` is only supported on Unix platforms (LD_PRELOAD + rmw_cerulion \
         require the Unix dynamic loader and exec() semantics)"
    );
    ExitCode::FAILURE
}

/// Dispatch `cerulion ros2 migrate` — resolve the clang engine
/// binary (69 = not built, the run/launch missing-library code; the message
/// carries the container build instructions), then run the engine
/// orchestration with the real seams: a stdin y/N confirm, the production
/// colcon runner, stdout as the report sink. A colcon failure after a
/// successful apply exits 1 with the revert path on stderr — the commit
/// stays, and the message says so.
fn ros2_migrate_exit_code(workspace: PathBuf, write: bool, yes: bool) -> ExitCode {
    use cerulion_cli_engine::ros2_migrate as rm;
    use std::io::IsTerminal as _;
    use std::io::Write as _;

    let tool = match rm::resolve_engine_binary() {
        Ok(tool) => tool,
        Err(msg) => {
            eprintln!("Error: {msg}");
            return ExitCode::from(cerulion_cli_engine::ros2_cmd::EXIT_UNAVAILABLE);
        }
    };
    let mut engine = rm::ClangToolEngine { tool };
    let mut confirm = |prompt: &str| -> CliResult<bool> {
        print!("{prompt}");
        std::io::stdout()
            .flush()
            .map_err(cerulion_cli_engine::error::CliError::Io)?;
        let mut line = String::new();
        std::io::stdin()
            .read_line(&mut line)
            .map_err(cerulion_cli_engine::error::CliError::Io)?;
        Ok(matches!(line.trim(), "y" | "Y" | "yes" | "Yes" | "YES"))
    };
    let mut build_runner = |ws: &std::path::Path, pkgs: &[String]| rm::run_colcon_build(ws, pkgs);
    let mut stdout = std::io::stdout();
    // Interrupt safety: for --write, Ctrl-C must not terminate
    // the process between the patch/source writes — install the same
    // flag-flipping handler the long-running verbs use; the engine polls it
    // at its write safepoints and rolls back on a trip. A write that cannot
    // arm the handler REFUSES rather than running unprotected. Dry-run (and
    // ros2 run/launch, which never reach this fn) keep the no-handler
    // behavior.
    let running: Option<Arc<AtomicBool>> = if write {
        match setup_ctrlc_handler() {
            Ok(r) => Some(r),
            Err(e) => {
                eprintln!(
                    "Error: cannot protect the write window from Ctrl-C \
                     ({e}) — refusing to write. Re-run when a signal \
                     handler can be installed."
                );
                return ExitCode::FAILURE;
            }
        }
    } else {
        None
    };
    let interrupted = move || running.as_ref().is_some_and(|r| !r.load(Ordering::Relaxed));
    let outcome = rm::run_migrate(
        &rm::MigrateOptions {
            workspace,
            write,
            assume_yes: yes,
        },
        &mut rm::MigrateDeps {
            engine: &mut engine,
            is_tty: std::io::stdin().is_terminal(),
            confirm: &mut confirm,
            build_runner: &mut build_runner,
            out: &mut stdout,
            interrupted: &interrupted,
        },
    );
    match outcome {
        Ok(rm::MigrateOutcome::Applied {
            build_error: Some(msg),
            ..
        }) => {
            eprintln!("Error: {msg}");
            ExitCode::FAILURE
        }
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> CliResult<()> {
    match cli.command {
        Commands::Workspace { action } => match action {
            WorkspaceAction::Create { name } => {
                let cwd = std::env::current_dir()?;
                let ws = workspace::workspace_create(&cwd, &name)?;
                println!("Created workspace at {}", ws.root.display());
                if let Some(source) = &ws.dependency_source {
                    println!("  dependencies: {source}");
                }
                Ok(())
            }
            WorkspaceAction::Init { location } => {
                let path = PathBuf::from(&location)
                    .canonicalize()
                    .unwrap_or_else(|_| PathBuf::from(&location));
                let ws = workspace::workspace_init(&path)?;
                println!("Initialized workspace at {}", ws.root.display());
                if let Some(source) = &ws.dependency_source {
                    println!("  dependencies: {source}");
                }
                Ok(())
            }
        },
        Commands::Node { action } => {
            let ws = discover_workspace()?;
            match action {
                NodeAction::Create {
                    node_type,
                    output,
                    input,
                    trigger_input,
                    policy,
                    raw_ffi,
                } => {
                    // Reject multi-invocation BEFORE any
                    // `parse_port_args` call. With `num_args = 2`
                    // on each flag, a valid single invocation
                    // produces exactly 2 elements; >2 means the
                    // user passed the flag multiple times.
                    // `parse_port_args` has a debug-assert on
                    // `args.len() == 2`, so guarding here lets the
                    // user see a clean validation error instead of
                    // an assertion panic in debug builds.
                    if trigger_input.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-T` per node. For a sync node over several inputs, \
                             create it with `--policy sync_window_ms=N`, add each further input \
                             with `cerulion node modify <TYPE> -i SCHEMA NAME`, then mark the \
                             inputs to align as `#[input(trigger)]` in nodes/<TYPE>/src/lib.rs"
                                .to_string(),
                        ));
                    }
                    if input.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-i` per `node create` (use `node modify` to add \
                             more inputs after creation)"
                                .to_string(),
                        ));
                    }
                    if output.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-o` per `node create` (use `node modify` to add \
                             more outputs after creation)"
                                .to_string(),
                        ));
                    }
                    // Resolve each port schema right after
                    // parsing — bare built-in names qualify (with a
                    // stderr note), workspace shadows warn, and
                    // ambiguous/unknown names error before anything
                    // is created. The engine re-resolves as the
                    // enforcement backstop (idempotent, free).
                    let outputs: Vec<(String, String)> = if output.is_empty() {
                        vec![]
                    } else {
                        let (schema, name) = parse_port_args(&output);
                        vec![(resolve_and_report(&ws.schemas_dir, &schema)?, name)]
                    };
                    let regular_inputs: Vec<(String, String)> = if input.is_empty() {
                        vec![]
                    } else {
                        let (schema, name) = parse_port_args(&input);
                        vec![(resolve_and_report(&ws.schemas_dir, &schema)?, name)]
                    };
                    let trigger_inputs: Vec<(String, String)> = if trigger_input.is_empty() {
                        vec![]
                    } else {
                        let (schema, name) = parse_port_args(&trigger_input);
                        vec![(resolve_and_report(&ws.schemas_dir, &schema)?, name)]
                    };
                    let explicit_policy = match policy.as_deref() {
                        Some(spec) => Some(parse_policy_spec(spec)?),
                        None => None,
                    };
                    let resolved_policy = resolve_create_policy(
                        explicit_policy.as_ref(),
                        trigger_inputs.first(),
                        &regular_inputs,
                    )?;
                    check_dash_t_dash_i_name_collision(&trigger_inputs, &regular_inputs)?;
                    // Combined inputs: trigger inputs first (matches macro
                    // emission order convention).
                    let mut combined_inputs = trigger_inputs.clone();
                    combined_inputs.extend(regular_inputs.iter().cloned());
                    let trigger_name = trigger_inputs
                        .first()
                        .map(|(_, name)| name.clone())
                        .or_else(|| match resolved_policy.as_ref() {
                            Some(cerulion_core::MacroPolicy::DataTrigger { input_name }) => {
                                Some(input_name.clone())
                            }
                            _ => None,
                        });
                    // Computed before `resolved_policy` moves into the engine call.
                    let build_note = node_create_build_note(
                        &node_type,
                        resolved_policy.as_ref(),
                        trigger_name.is_some(),
                        raw_ffi,
                    );
                    let options = node_cmd::NodeCreateOptions {
                        outputs,
                        inputs: combined_inputs,
                        trigger: trigger_name,
                        raw_ffi,
                    };
                    node_cmd::node_create_with_options(
                        &ws.nodes_dir,
                        &ws.root.join("Cargo.toml"),
                        &node_type,
                        resolved_policy,
                        &options,
                    )?;
                    println!("Created node type '{}'", node_type);
                    if let Some(note) = build_note {
                        eprintln!("{note}");
                    }
                    Ok(())
                }
                NodeAction::Delete { node_type } => {
                    node_cmd::node_delete(&ws.nodes_dir, &ws.root.join("Cargo.toml"), &node_type)?;
                    println!("Deleted node type '{}'", node_type);
                    Ok(())
                }
                NodeAction::Modify {
                    node_type,
                    input,
                    trigger_input,
                    output,
                    policy,
                } => {
                    // Same raw-vec length check as Create. With
                    // `num_args = 2` on the args, each valid invocation
                    // contributes exactly 2 elements, so any vec longer
                    // than 2 means the user passed the flag multiple
                    // times.
                    if trigger_input.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-T` per call. For a sync node over several inputs, \
                             set `--policy sync_window_ms=N`, add one input per call with `-i \
                             SCHEMA NAME`, then mark the inputs to align as `#[input(trigger)]` \
                             in nodes/<TYPE>/src/lib.rs"
                                .to_string(),
                        ));
                    }
                    if input.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-i` per `node modify` call".to_string(),
                        ));
                    }
                    if output.len() > 2 {
                        return Err(cerulion_cli_engine::error::CliError::Validation(
                            "at most one `-o` per `node modify` call".to_string(),
                        ));
                    }
                    // Resolve each port schema right after
                    // parsing — bare built-in names qualify (with a
                    // stderr note), workspace shadows warn, and
                    // ambiguous/unknown names error with the node
                    // source untouched. The engine re-resolves as the
                    // enforcement backstop (idempotent, free).
                    let regular_inputs: Vec<(String, String)> = if input.is_empty() {
                        vec![]
                    } else {
                        let (schema, name) = parse_port_args(&input);
                        vec![(resolve_and_report(&ws.schemas_dir, &schema)?, name)]
                    };
                    let trigger_inputs: Vec<(String, String)> = if trigger_input.is_empty() {
                        vec![]
                    } else {
                        let (schema, name) = parse_port_args(&trigger_input);
                        vec![(resolve_and_report(&ws.schemas_dir, &schema)?, name)]
                    };
                    let explicit_policy = match policy.as_deref() {
                        Some(spec) => Some(parse_policy_spec(spec)?),
                        None => None,
                    };
                    // `-T` and `--policy data_trigger=NAME` must agree
                    // when both are supplied.
                    let effective_policy: Option<cerulion_core::MacroPolicy> = match (
                        explicit_policy,
                        trigger_inputs.first(),
                    ) {
                        (Some(p), None) => Some(p),
                        (None, Some((_, name))) => Some(cerulion_core::MacroPolicy::DataTrigger {
                            input_name: name.clone(),
                        }),
                        (
                            Some(cerulion_core::MacroPolicy::DataTrigger { input_name }),
                            Some((_, t_name)),
                        ) => {
                            if input_name != *t_name {
                                return Err(cerulion_cli_engine::error::CliError::Validation(
                                    format!(
                                        "`--policy data_trigger={input_name}` and \
                                             `-T <SCHEMA> {t_name}` disagree on the trigger input"
                                    ),
                                ));
                            }
                            Some(cerulion_core::MacroPolicy::DataTrigger {
                                input_name: input_name.clone(),
                            })
                        }
                        (Some(_), Some(_)) => {
                            return Err(cerulion_cli_engine::error::CliError::Validation(
                                    "`-T` is shorthand for `--policy data_trigger=<NAME>`; supply one or the other, not both"
                                        .to_string(),
                                ));
                        }
                        (None, None) => None,
                    };
                    // For `data_trigger=NAME` the named input must be
                    // declared somewhere reachable: this call (`-i` /
                    // `-T`) or a prior modify.
                    if let Some(cerulion_core::MacroPolicy::DataTrigger { input_name }) =
                        effective_policy.as_ref()
                    {
                        let added_in_this_call = trigger_inputs
                            .iter()
                            .chain(regular_inputs.iter())
                            .any(|(_, n)| n == input_name);
                        if !added_in_this_call {
                            let metadata = node_cmd::node_info(&ws.nodes_dir, &node_type)?;
                            let exists = metadata.inputs.iter().any(|p| p.name == *input_name);
                            if !exists {
                                return Err(cerulion_cli_engine::error::CliError::Validation(
                                    format!(
                                        "`--policy data_trigger={input_name}` requires the input \
                                         to exist; add it via `-i SCHEMA {input_name}` (or \
                                         `-T SCHEMA {input_name}`) in the same call or in a \
                                         prior `node modify`"
                                    ),
                                ));
                            }
                        }
                    }
                    if !output.is_empty() {
                        let (schema, name) = parse_port_args(&output);
                        // Same resolve-and-report as the input arms.
                        let schema = resolve_and_report(&ws.schemas_dir, &schema)?;
                        node_cmd::node_modify_add_port(
                            &ws.nodes_dir,
                            &node_type,
                            &name,
                            Some(&schema),
                            true,
                            false,
                        )?;
                        println!("Added output '{}' to '{}'", name, node_type);
                    }
                    let trigger_target_name = match effective_policy.as_ref() {
                        Some(cerulion_core::MacroPolicy::DataTrigger { input_name }) => {
                            Some(input_name.clone())
                        }
                        _ => None,
                    };
                    // Add `-T` first (so its trigger semantics are
                    // applied), then any `-i`.
                    for (schema, name) in trigger_inputs.iter().chain(regular_inputs.iter()) {
                        let mark_as_trigger = trigger_target_name
                            .as_deref()
                            .map(|t| t == name.as_str())
                            .unwrap_or(false);
                        node_cmd::node_modify_add_port(
                            &ws.nodes_dir,
                            &node_type,
                            name,
                            Some(schema),
                            false,
                            mark_as_trigger,
                        )?;
                        if mark_as_trigger {
                            println!(
                                "Added input '{}' to '{}' as the data trigger",
                                name, node_type
                            );
                        } else {
                            println!("Added input '{}' to '{}'", name, node_type);
                        }
                    }
                    // Non-DataTrigger policies require an explicit
                    // mutation to the macro args. DataTrigger is already
                    // applied by the field-level write above when the
                    // trigger input was added; if the trigger pointed at
                    // an existing input, we still need to flip the macro
                    // args (clear conflicting node-level args).
                    if let Some(spec) = effective_policy.as_ref() {
                        match spec {
                            cerulion_core::MacroPolicy::DataTrigger { input_name } => {
                                let added_in_this_call = trigger_inputs
                                    .iter()
                                    .chain(regular_inputs.iter())
                                    .any(|(_, n)| n == input_name);
                                if !added_in_this_call {
                                    // The input pre-existed; the field-
                                    // attr write didn't run, so we need
                                    // to flip it via add_port with
                                    // is_trigger=true on the existing
                                    // field. `node_modify_add_port`
                                    // refuses to add a duplicate, so use
                                    // a dedicated helper.
                                    node_cmd::node_modify_promote_input_to_trigger(
                                        &ws.nodes_dir,
                                        &node_type,
                                        input_name,
                                    )?;
                                    println!(
                                        "Promoted existing input '{}' to data trigger on '{}'",
                                        input_name, node_type
                                    );
                                }
                            }
                            cerulion_core::MacroPolicy::External => {
                                node_cmd::node_modify_ext_trigger(&ws.nodes_dir, &node_type, true)?;
                                println!("Set policy=external on '{}'", node_type);
                            }
                            cerulion_core::MacroPolicy::Period { period_ms } => {
                                node_cmd::node_modify_set_period(
                                    &ws.nodes_dir,
                                    &node_type,
                                    *period_ms,
                                )?;
                                println!("Set policy=period_ms={} on '{}'", period_ms, node_type);
                            }
                            cerulion_core::MacroPolicy::Sync { window_ms } => {
                                node_cmd::node_modify_set_sync(
                                    &ws.nodes_dir,
                                    &node_type,
                                    *window_ms,
                                )?;
                                println!(
                                    "Set policy=sync_window_ms={} on '{}'",
                                    window_ms, node_type
                                );
                            }
                            cerulion_core::MacroPolicy::UnboundedSync => {
                                node_cmd::node_modify_set_policy(
                                    &ws.nodes_dir,
                                    &node_type,
                                    &cerulion_core::MacroPolicy::UnboundedSync,
                                )?;
                                println!("Set policy=unbounded_sync on '{}'", node_type);
                            }
                        }
                    }
                    Ok(())
                }
                NodeAction::Build { node_type, release } => {
                    // Report what the build is doing about the node's
                    // optional SYSTEM dependencies BEFORE cargo runs. Printing
                    // it after the build would lose it on exactly the run that
                    // needs it — a build that FAILS because of the feature this
                    // CLI decided to enable, where cargo's stderr is otherwise
                    // the user's only clue and never mentions `--features`.
                    //
                    // Cargo's output is captured, so the progress line is the
                    // only thing on screen while it runs. It goes to stderr
                    // with the notice: stdout carries the result alone.
                    let _outcome = node_cmd::node_build_with_progress(
                        &ws.root,
                        &node_type,
                        release,
                        &mut |notice| eprint!("{notice}"),
                        &mut |line| eprintln!("{line}"),
                    )?;
                    println!("Built '{}'", node_type);
                    Ok(())
                }
                NodeAction::Stage {
                    node_type,
                    id,
                    graph,
                    input_binding,
                } => {
                    let graph_name = resolve_graph_name(&ws, graph)?;
                    let inputs: Vec<(String, String)> = parse_input_bindings(&input_binding);
                    // Outputs come from the node's DECLARED ports, through the
                    // same engine fn `cerulion-wsd` uses — never from flags.
                    graph_cmd::stage_declared_node(
                        &ws.nodes_dir,
                        &ws.graphs_dir,
                        &graph_name,
                        &node_type,
                        id.as_deref(),
                        &inputs,
                    )?;
                    println!("Staged '{}' into graph '{}'", node_type, graph_name);
                    Ok(())
                }
                NodeAction::Run {
                    node_type,
                    prefix,
                    id,
                    release,
                    no_cpu_dma_lock,
                    no_monitor_wait,
                    network,
                } => {
                    let running = setup_ctrlc_handler()?;
                    let prefix = prefix.unwrap_or_else(|| "standalone".to_string());
                    let metadata = node_cmd::node_info(&ws.nodes_dir, &node_type)?;

                    let outputs: Vec<(String, Option<String>)> = metadata
                        .outputs
                        .iter()
                        .map(|p| (p.name.clone(), p.schema.clone()))
                        .collect();

                    let node_def =
                        graph_cmd::build_node_def(&node_type, id.as_deref(), &outputs, &[]);

                    // Create temporary graph
                    let temp_graph = format!("__temp_{}", node_type);
                    graph_cmd::graph_create(&ws.graphs_dir, &temp_graph, Some(&prefix))?;
                    graph_cmd::node_stage(&ws.graphs_dir, &temp_graph, node_def)?;

                    let result = graph_cmd::graph_run(
                        &ws.root,
                        &ws.graphs_dir,
                        &temp_graph,
                        running,
                        graph_cmd::TimeSource::Real, // live RealClock default (matches `graph run`)
                        if no_cpu_dma_lock {
                            graph_cmd::CpuDmaLockMode::Disabled
                        } else {
                            graph_cmd::CpuDmaLockMode::Auto
                        },
                        if no_monitor_wait {
                            graph_cmd::MonitorWaitMode::Disabled
                        } else {
                            graph_cmd::MonitorWaitMode::Auto
                        },
                        true,    // skip validation for temporary single-node graphs
                        release, // honour `node run --release`, mirroring `graph run`
                        // `node run`'s temp graph is always a
                        // single-node monolith — no peer-loss flag, no forced
                        // single-process, default trace cap.
                        None,
                        false,
                        // `node run` rides the SAME permissive network
                        // default as `graph run` — a real-clock run spawns the
                        // gateway and announces the node's topics unless the
                        // kill-switch is passed (`--network off` here mirrors
                        // graph run; `off` is clap-enforced as the only value).
                        network.is_some(),
                        graph_cmd::PRODUCTION_TRACE_LIMIT,
                        None, // `node run` does not support recording (use `graph run --record`)
                        graph_cmd::RecordEnvMode::default(), // unused (record is None)
                        graph_cmd::RecordCpu::default(), // unused (record is None)
                        // `node run`'s temp single-node graph must
                        // never auto-partition — no consent seam.
                        None,
                        // `node run` is a monolith, which mints no
                        // trace ring on any path, so there is nothing to decline.
                        false,
                    );

                    // Clean up temporary graph
                    let _ =
                        std::fs::remove_file(ws.graphs_dir.join(format!("{}.yaml", temp_graph)));

                    result
                }
                NodeAction::List => {
                    let nodes = node_cmd::node_list(&ws.nodes_dir)?;
                    if nodes.is_empty() {
                        println!("No nodes found.");
                    } else {
                        println!("{:<20} {:<10} {:<10} POLICY", "TYPE", "INPUTS", "OUTPUTS");
                        for node in &nodes {
                            println!(
                                "{:<20} {:<10} {:<10} {}",
                                node.node_type,
                                node.inputs.len(),
                                node.outputs.len(),
                                format_policy_short(node.policy.as_ref())
                            );
                        }
                    }
                    Ok(())
                }
                NodeAction::Info { node_type } => {
                    let info = node_cmd::node_info(&ws.nodes_dir, &node_type)?;
                    println!("Node type: {}", info.node_type);
                    println!("Policy: {}", format_policy_short(info.policy.as_ref()));
                    if !info.inputs.is_empty() {
                        println!("Inputs:");
                        for port in &info.inputs {
                            println!(
                                "  {} {}",
                                port.name,
                                port.schema.as_deref().unwrap_or("(untyped)")
                            );
                        }
                    }
                    if !info.outputs.is_empty() {
                        println!("Outputs:");
                        for port in &info.outputs {
                            println!(
                                "  {} {}",
                                port.name,
                                port.schema.as_deref().unwrap_or("(untyped)")
                            );
                        }
                    }
                    Ok(())
                }
            }
        }
        Commands::Graph { action } => {
            let ws = discover_workspace()?;
            match action {
                GraphAction::Create { name, prefix } => {
                    graph_cmd::graph_create(&ws.graphs_dir, &name, prefix.as_deref())?;
                    println!("Created graph '{}'", name);
                    Ok(())
                }
                GraphAction::Run {
                    name,
                    time_source,
                    no_validate,
                    release,
                    no_cpu_dma_lock,
                    no_monitor_wait,
                    peer_loss,
                    single_process,
                    network,
                    trace_limit,
                    no_rings,
                    record,
                    record_env,
                    record_cpu,
                    auto_partition,
                    yes,
                } => {
                    use std::io::IsTerminal as _;
                    let running = setup_ctrlc_handler()?;
                    // The auto-partition consent seam — the real
                    // TTY probe + the shared y/N prompt (the engine only
                    // consults it on the interactive persist arm).
                    let mut confirm = stdin_yes_no_confirm;
                    let consent = partition_emit::PartitionConsent {
                        auto_partition,
                        assume_yes: yes,
                        is_tty: std::io::stdin().is_terminal(),
                        confirm: &mut confirm,
                    };
                    graph_cmd::graph_run(
                        &ws.root,
                        &ws.graphs_dir,
                        &name,
                        running,
                        time_source.into(),
                        if no_cpu_dma_lock {
                            graph_cmd::CpuDmaLockMode::Disabled
                        } else {
                            graph_cmd::CpuDmaLockMode::Auto
                        },
                        if no_monitor_wait {
                            graph_cmd::MonitorWaitMode::Disabled
                        } else {
                            graph_cmd::MonitorWaitMode::Auto
                        },
                        no_validate,
                        release,
                        // `Some` iff the user typed the
                        // flag (the flag wins over the hidden env seam).
                        peer_loss.map(Into::into),
                        single_process,
                        // `--network off` is the only accepted value
                        // (clap-enforced), so presence == the kill-switch.
                        network.is_some(),
                        // Clap enforces `1..` (0 rejected at parse).
                        trace_limit as usize,
                        record,
                        record_env.into(),
                        cli::record_cpu_mode(record_cpu),
                        Some(consent),
                        // `--no-rings` — decline this run's per-rank
                        // scheduler-trace rings and its window recorder.
                        no_rings,
                    )
                }
                GraphAction::Validate { name, release } => {
                    let report = graph_cmd::graph_validate(&ws.root, &name, release)?;
                    println!("{}", report);
                    if report.all_passed() {
                        Ok(())
                    } else {
                        Err(cerulion_cli_engine::error::CliError::Validation(
                            "graph validation failed".to_string(),
                        ))
                    }
                }
                GraphAction::List => {
                    let graphs = graph_cmd::graph_list(&ws.graphs_dir)?;
                    if graphs.is_empty() {
                        println!("No graphs found.");
                    } else {
                        for name in &graphs {
                            println!("{}", name);
                        }
                    }
                    Ok(())
                }
                GraphAction::Levels { name } => {
                    // Levelization inspection. The engine
                    // returns the fully-rendered report; print it verbatim,
                    // THEN exit nonzero if the declared process_groups
                    // partition failed the spawner-consumability check
                    // (inspect-then-fail — the report IS the diagnostic, and
                    // the exit code makes `graph levels` CI-gateable on
                    // partition validity, matching `graph run`'s enforcement).
                    let report = graph_cmd::graph_levels(&ws.root, &name)?;
                    print!("{}", report.rendered);
                    map_levels_partition_result(report.partition_error)
                }
                GraphAction::Profile {
                    name,
                    duration,
                    fires,
                    out,
                } => {
                    // Live profiling run. `running` is the
                    // SAME Ctrl+C flag `graph run` wires — the profiler's
                    // watcher thread and the SIGINT handler both flip it, so
                    // gate-met / cap-elapsed / Ctrl+C all stop the run the
                    // same way (an early Ctrl+C still harvests + writes the
                    // artifact over whatever window was observed).
                    let running = setup_ctrlc_handler()?;
                    // `--fires` is an OPTIONAL uniform-target
                    // override — omitted (None, the DEFAULT) the engine
                    // AUTO-DERIVES per-node fire targets from a warm-up
                    // observation (clamp(cap/10, 1s, 3s)). Nodes silent
                    // through the warm-up — including nodes whose FIRST fire
                    // lands after it (e.g. a period longer than 3 s) —
                    // derive no target, are excluded from the stop gate, and
                    // isolate loudly; `--fires N` is their escape hatch.
                    let report = graph_cmd::graph_profile(
                        &ws.root,
                        &name,
                        running,
                        std::time::Duration::from_secs(duration),
                        fires,
                        out.as_deref(),
                    )?;
                    println!(
                        "Profiled graph '{}': {} node(s) over {:.2}s",
                        name,
                        report.node_count,
                        report.window_ns as f64 / 1e9
                    );
                    println!("Cost snapshot written: {}", report.artifact_path.display());
                    if let Some(bak) = &report.backup_path {
                        println!("Previous artifact backed up to: {}", bak.display());
                    }
                    // The LOUD isolation stanza. Isolation is a valid outcome
                    // (the warn is the contract), NOT an error — exit 0.
                    if !report.isolated.is_empty() {
                        println!();
                        match report.fires_override {
                            Some(fires_target) => {
                                // Uniform --fires mode. The reachability
                                // arithmetic: to reach the fire
                                // target within the cap a node must sustain at
                                // least fires/duration Hz. Surface the IMPLIED
                                // rate from this run's ACTUAL knob values
                                // (never hardcoded) so the user sees WHY a
                                // slow node isolated.
                                let implied_hz =
                                    (fires_target as f64 / duration as f64).ceil() as u64;
                                println!(
                                    "WARNING: {} node(s) under-sampled: the fires target ({}) \
                                     requires >={} Hz sustained over the {} s cap; raise \
                                     --duration or lower --fires for slower nodes. ISOLATED \
                                     (no cost recorded):",
                                    report.isolated.len(),
                                    fires_target,
                                    implied_hz,
                                    duration
                                );
                            }
                            None => {
                                // Auto-derive mode: targets are
                                // per-node (each line below carries its own),
                                // so no uniform ">= X Hz" arithmetic applies.
                                println!(
                                    "WARNING: {} node(s) under-sampled against their \
                                     auto-derived fire targets. ISOLATED (no cost recorded):",
                                    report.isolated.len()
                                );
                            }
                        }
                        for iso in &report.isolated {
                            match iso.target {
                                Some(target) => println!(
                                    "  {} — {} fire(s) observed (target {})",
                                    iso.node, iso.fires, target
                                ),
                                None => println!(
                                    "  {} — silent through warm-up — no target derived",
                                    iso.node
                                ),
                            }
                            if !iso.starved_trigger_inputs.is_empty() {
                                println!(
                                    "      zero-rate trigger input(s): {}; an upstream \
                                     goal/driver-fed input may be silent - profile under \
                                     representative load (run the driver graph alongside)",
                                    iso.starved_trigger_inputs.join(", ")
                                );
                            }
                        }
                        match report.fires_override {
                            Some(_) => println!(
                                "Each isolated node stays in its own process group; re-run \
                                 with a longer --duration or a lower --fires to sample it."
                            ),
                            None => println!(
                                "Each isolated node stays in its own process group; raise \
                                 --duration to sample slow nodes (--fires N forces the \
                                 uniform fixed-target mode)."
                            ),
                        }
                    }
                    Ok(())
                }
                GraphAction::Partition {
                    name,
                    costs,
                    budget_ns,
                    dry_run,
                    yes,
                } => {
                    // Derive + surgically write the
                    // process_groups partition. The engine owns the whole
                    // consent ladder (dry-run / unchanged / --yes / no-TTY
                    // refusal / interactive confirm); the binary supplies the
                    // REAL TTY probe and the y/N prompt, and prints the
                    // preview on the paths where the confirm closure did not
                    // already display it.
                    // `--budget-ns` is an OPTIONAL override — omitted
                    // (None, the DEFAULT) the engine resolves the budget to
                    // the cost artifact's FROZEN core-count value
                    // (`derived_budget_ns`, computed at profile time); an
                    // earlier (v1) artifact or an absent frozen value
                    // falls back to unbounded fusion with a loud info
                    // suggesting a re-profile. An explicit value always
                    // overrides the frozen one.
                    use std::io::IsTerminal as _;
                    let opts = partition_emit::PartitionOptions {
                        costs_path: costs,
                        budget_ns,
                        dry_run,
                        assume_yes: yes,
                    };
                    let is_tty = std::io::stdin().is_terminal();
                    let mut confirm = stdin_yes_no_confirm;
                    let report = partition_emit::graph_partition(
                        &ws.root,
                        &name,
                        &opts,
                        is_tty,
                        &mut confirm,
                    )?;
                    if !report.preview_shown {
                        print!("{}", report.preview);
                    }
                    match &report.outcome {
                        partition_emit::PartitionOutcome::DryRun => {
                            println!("\ndry run — nothing written.");
                        }
                        partition_emit::PartitionOutcome::Unchanged => {
                            println!(
                                "\n{} already carries this partition — nothing to write.",
                                report.graph_path.display()
                            );
                        }
                        partition_emit::PartitionOutcome::Written { backup } => {
                            println!("\nPartition written to {}", report.graph_path.display());
                            if let Some(bak) = backup {
                                println!("Previous graph file backed up to: {}", bak.display());
                            }
                        }
                        partition_emit::PartitionOutcome::Declined => {
                            println!("\naborted — nothing written.");
                        }
                    }
                    Ok(())
                }
                GraphAction::RunWorker { plan } => {
                    // Hidden verb: run ONE worker of a multi-process
                    // deployment from its serialized plan. The supervisor
                    // execs this with cwd = the workspace root, so
                    // `ws.root` resolves the subgraph's node cdylibs the same
                    // way `graph run` does. `running` forwards Ctrl+C so the
                    // worker's live loop stops cleanly on SIGINT.
                    let running = setup_ctrlc_handler()?;
                    graph_cmd::graph_run_worker(&ws.root, &plan, running)
                }
                GraphAction::RunGateway { handoff } => {
                    // Hidden verb: the network GATEWAY process of a
                    // `graph run`. The parent (monolith arm / supervisor) execs
                    // this with a serialized `GatewayHandoff`; it owns the
                    // robot's whole network plane while graph/worker processes
                    // stay network-free. `running` forwards Ctrl+C for a clean
                    // stop (the parent's guard is the SIGKILL backstop).
                    let running = setup_ctrlc_handler()?;
                    graph_cmd::graph_run_gateway(&handoff, running)
                }
            }
        }
        Commands::Topic { action } => match action {
            TopicAction::List {
                all,
                no_network,
                connect,
                listen,
                scan,
            } => {
                // LOCAL topics print FIRST/instantly, so a slow or
                // failing remote query never delays or hides them.
                //
                // A LOCAL `*/data` service that is really a
                // MIRROR of a remote robot (its canonical name is in the
                // `/__cerulion/mirrors` provenance registry) must NOT present as a
                // second LOCAL topic (the "one data source = one topic"
                // decision). Gather provenance (best-effort; instant on a desk with
                // no live mirror) and PARTITION the local enumeration before
                // printing: genuine-local topics stay under LOCAL, mirrored ones
                // fold into the REMOTE section as `● streaming` rows attributed to
                // their origin robot.
                //
                // The framework's own channels (`/bagd/status` while a
                // recording runs, anything under `/__cerulion/`) are HIDDEN
                // from this LOCAL section by default and named by a count
                // line; `--all` shows them with an `internal` marker. REMOTE
                // rows (network and mirror alike) are never filtered. The renderer is the
                // engine's oracle-tested `render_local_topics_section`; the
                // binary prints its string verbatim.
                let topics = topic_cmd::topic_list()?;
                let mirrors = topic_cmd::gather_mirror_provenance();
                let (genuine_local, streaming) =
                    topic_cmd::partition_local_topics(topics, &mirrors);
                print!(
                    "{}",
                    topic_cmd::render_local_topics_section(&genuine_local, all)
                );
                // Automagic: remote discovery runs BY DEFAULT with
                // scouting ON — unpaired robots on the LAN show up with no
                // flags. `--no-network` skips it (scripts / CI / offline).
                // The options are built by the ENGINE's unit-pinned builder
                // (`remote_discovery_options` — the
                // scouting-ON default lives there, not in an untested literal
                // here). Because remote discovery is the DEFAULT, a
                // session/query failure must NOT fail the whole command (the
                // LOCAL list is already printed) — it is a LOUD note + exit 0.
                // Never a silent empty, never a fake-success.
                //
                // The mirror-streaming rows are LOCAL knowledge (read from
                // SHM, not the network), so they must ALWAYS render — even under
                // `--no-network` and even when the network query fails — or a
                // partitioned-out mirror would vanish entirely. `remote_rendered`
                // tracks whether an Ok remote render already folded them in.
                let mut remote_rendered = false;
                if let Some(opts) = topic_cmd::remote_discovery_options(no_network, connect, listen)
                {
                    // Discovery rung 4: `--scan` (opt-in) appends the unicast
                    // subnet-sweep rung to the discovery ladder. This flag is the
                    // SOLE producer of a `true` scan — the structural guarantee
                    // that the sweep never fires on a default run (a horizontal
                    // SYN sweep reads as port-scan recon to corporate IDS).
                    match topic_cmd::query_remote_topics_with_scan(&opts, scan) {
                        Ok(disc) => {
                            print!(
                                "{}",
                                topic_cmd::render_remote_topics_section_with_mirrors(
                                    &disc,
                                    opts.has_endpoints(),
                                    &streaming
                                )
                            );
                            remote_rendered = true;
                        }
                        Err(e) => {
                            // Loud note — the remote half is best-effort;
                            // the local topics already printed. `--no-network`
                            // skips this query entirely.
                            // One line, the shape of the empty-success
                            // `remote:` line (no header with no rows under it).
                            // The ENGINE renders it (exact oracle there, the
                            // docs quote that sentence); printed verbatim, the
                            // newline is part of the returned string.
                            eprint!("{}", topic_cmd::render_remote_discovery_unavailable(&e));
                        }
                    }
                }
                // Render the LOCAL mirror-streaming rows if an Ok remote
                // render did not already fold them in (--no-network, or a remote
                // query failure). A mirror is REMOTE regardless of the network
                // query, since its provenance is read from local SHM.
                if !remote_rendered && !streaming.is_empty() {
                    print!(
                        "{}",
                        topic_cmd::render_remote_topics_section_with_mirrors(
                            &topic_cmd::RemoteDiscovery::empty(),
                            false,
                            &streaming
                        )
                    );
                }
                Ok(())
            }
            TopicAction::Info { topic } => {
                // Pass the workspace `schemas/` dir (when in a
                // workspace) so `topic info` resolves + prints the schema NAME
                // via the same local ladder `topic echo` uses. Workspace-OPTIONAL
                // (built-ins-only local walker outside a workspace).
                let ws = discover_workspace().ok();
                let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
                let info = topic_cmd::topic_info(&topic, schemas_dir)?;
                println!("{}", info);
                Ok(())
            }
            TopicAction::Echo {
                topic,
                truncate_length,
            } => {
                let running = setup_ctrlc_handler()?;
                // Pass the workspace `schemas/` dir (when in a
                // workspace) so `topic echo` decodes WORKSPACE types LOCALLY —
                // the remote tier stays the fallback. Workspace-OPTIONAL: echo
                // also runs outside a workspace (built-ins-only local walker).
                let ws = discover_workspace().ok();
                let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
                // `--truncate-length` (clap-guaranteed >= 1) bounds every
                // rendered array; narrow the u64 flag to the engine's usize,
                // saturating (never wrapping) on a hypothetical 32-bit target
                // where a > usize::MAX bound would just render every element.
                topic_cmd::topic_echo(
                    &topic,
                    schemas_dir,
                    running,
                    &mut std::io::stdout(),
                    usize::try_from(truncate_length).unwrap_or(usize::MAX),
                )
            }
            TopicAction::Hz { topic } => {
                let running = setup_ctrlc_handler()?;
                // Pass the workspace `schemas/` dir (when in a
                // workspace) so a REMOTE `topic hz` resolves a catalog-named
                // built-in / workspace type from the desk's LOCAL corpus (no wire
                // schema fetch) — the SAME rung echo/info use. Workspace-OPTIONAL:
                // `hz` also runs outside a workspace (built-ins-only resolution).
                let ws = discover_workspace().ok();
                let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
                topic_cmd::topic_hz(&topic, schemas_dir, running, &mut std::io::stdout())
            }
        },
        Commands::Viz {
            topics,
            robot,
            connect,
            listen,
            detach,
        } => {
            // `cerulion viz` is a thin CLIENT of the long-lived
            // `cerulion-vizd` daemon — no per-topic codegen, no ephemeral graph, no
            // restart to add a topic. It ensures the daemon is running and sends one
            // `attach` per requested topic. The daemon owns the taps + the
            // schema-generic dispatch + the hosted proxy; the CLI never links rerun,
            // and never starts a viewer: Cerulion Studio connects to the same daemon.
            let running = setup_ctrlc_handler()?;
            run_viz(
                running,
                &topics,
                robot.as_deref(),
                &connect,
                &listen,
                detach,
            )
        }
        // The explicit login verb. Runs the RFC 8628 device-code
        // flow (re-auth / account switch); the first identity-needing command
        // auto-triggers the SAME flow via the gate. The prompt rides stderr.
        Commands::Login => {
            login_cmd::run_login(&mut std::io::stderr())?;
            Ok(())
        }
        // Account self-service device management (list / revoke). The
        // session is resolved (+ refreshed) here — the login gate already guaranteed a
        // logged-in-ever account for this identity-needing command.
        Commands::Account { action } => match action {
            AccountAction::Devices { action } => {
                let session = account_cmd::require_session()?;
                match action {
                    DevicesAction::List => {
                        let devices = account_cmd::list_my_devices(&session)?;
                        account_cmd::render_devices(&mut std::io::stdout(), &devices).map_err(|e| {
                            cerulion_cli_engine::error::CliError::Validation(format!(
                                "could not print devices: {e}"
                            ))
                        })
                    }
                    DevicesAction::Revoke { device_id } => {
                        let res = account_cmd::revoke_my_device(&session, &device_id)?;
                        println!("revoked device {} (revoked={})", res.device_id, res.revoked);
                        // Enforcement scope: the device is cut on
                        // the robots you OWN once they sync; robots you're only a guest
                        // on need their owner to revoke it, and enforcement lands only
                        // after each robot next syncs (the offline gap).
                        println!(
                            "  cut on {} robot(s) you own (enforced once each syncs).",
                            res.robots_updated
                        );
                        if !res.robots_failed.is_empty() {
                            println!(
                                "  WARNING: {} owned robot(s) could not be updated: {} — \
                                 the device IS revoked; re-run this command to retry them.",
                                res.robots_failed.len(),
                                res.robots_failed.join(", ")
                            );
                        }
                        println!(
                            "  note: a robot you don't OWN is not cut by this — its owner must \
                             revoke the device; and an offline robot enforces only after it syncs."
                        );
                        Ok(())
                    }
                }
            }
        },
        Commands::Tui => cerulion_cli_tui::run().map_err(|e| {
            cerulion_cli_engine::error::CliError::Validation(format!(
                "`cerulion tui` failed: {e}. It needs an interactive terminal: if you ran it \
                 from a pipe, a script or a non-interactive SSH command, run it in a terminal \
                 window instead"
            ))
        }),
        Commands::Trace { action } => match action {
            TraceAction::Inspect {
                dir,
                filter,
                limit,
                reverse,
            } => trace_inspect(&dir, filter.as_deref(), limit, reverse),
        },
        Commands::Completions { shell } => {
            // Straight to stdout: this output is meant to be piped into a file
            // or `source`d, so it must be the ONLY thing on the stream.
            let mut stdout = std::io::stdout().lock();
            completion::write_registration_script(shell, &mut stdout)
                .map_err(cerulion_cli_engine::error::CliError::Io)?;
            // The hint goes to STDERR so `cerulion completions zsh > file`
            // captures only the script. Skipped when stdout is redirected away
            // from a terminal — inside `source <(...)` the user is not reading
            // a tip, and inside the recommended rc one-liner it would print on
            // every single shell start.
            if std::io::IsTerminal::is_terminal(&std::io::stdout()) {
                eprintln!("\n# To enable completions permanently, run:");
                // Rendered line-by-line: the zsh hint is TWO lines (the
                // compinit guard, then the source line) — see `install_hint`.
                for line in shell.install_hint().lines() {
                    eprintln!("#   {line}");
                }
            }
            Ok(())
        }
        // Bag as a data source. Unix-only (the `cerulion_bag` reader
        // reuses `cerulion_core`'s Unix-only POSIX trace-ring types), so the
        // whole family degrades to a named refusal off Unix (see `run_bag`).
        Commands::Bag { action } => run_bag(action),
        // The manual Flashback trigger. Unix-only for the same
        // reason `bag` is — the recorder that holds the window is.
        #[cfg(unix)]
        Commands::Flashback { note, pin, no_wait } => {
            let running = setup_ctrlc_handler()?;
            let mut out = std::io::stdout();
            cerulion_cli_engine::flashback_cmd::flashback_capture(
                cerulion_cli_engine::flashback_cmd::FlashbackOptions { note, pin, no_wait },
                running,
                &mut out,
            )
        }
        #[cfg(not(unix))]
        Commands::Flashback => Err(CliError::Validation(
            "`cerulion flashback` is Unix-only: the recorder that holds the rolling window is \
             driven by SIGTERM lifecycle signals and does not build on this platform"
                .to_string(),
        )),
        Commands::Clean { report_only } => clean_iceoryx2_state(report_only),
        // `cerulion connect` never reaches `run()`: `main`
        // intercepts it to spawn `cerulion-connectd` + forward its exit code.
        Commands::Connect { .. } => {
            unreachable!("`cerulion connect` is dispatched in main() before run()")
        }
        // `cerulion pair` never reaches `run()`: `main` intercepts it to
        // spawn `cerulion-connectd pair` + forward its 0–4 pairing exit code.
        Commands::Pair { .. } => {
            unreachable!("`cerulion pair` is dispatched in main() before run()")
        }
        // `cerulion bagd` never reaches `run()`: `main`
        // dispatches it (unix) or errors (non-unix) before the logging setup.
        #[cfg(unix)]
        Commands::Bagd(_) => unreachable!("`cerulion bagd` is dispatched in main() before run()"),
        #[cfg(not(unix))]
        Commands::Bagd => unreachable!("`cerulion bagd` is rejected in main() before run()"),
        Commands::Schema { action } => {
            match action {
                // Create/Delete WRITE into the workspace `schemas/` dir, so a
                // workspace is required (loud error naming `workspace create`).
                SchemaAction::Create { name } => {
                    let ws = discover_workspace()?;
                    schema_cmd::schema_create(&ws.schemas_dir, &name)?;
                    println!("Created schema '{}'", name);
                    Ok(())
                }
                SchemaAction::Delete { name } => {
                    let ws = discover_workspace()?;
                    schema_cmd::schema_delete(&ws.schemas_dir, &name)?;
                    println!("Deleted schema '{}'", name);
                    Ok(())
                }
                SchemaAction::Info { name } => {
                    // `schema info` does NOT require a
                    // workspace — the intended use is a brand-new laptop
                    // resolving a robot's types with zero setup. A missing
                    // workspace ⇒ skip the workspace + `.msg`-store tiers and
                    // resolve builtin → remote exactly as designed; a workspace
                    // present ⇒ the full resolution ladder (workspace
                    // wins). `schema_info_unified_opt` keeps the provenance
                    // accurate either way.
                    let ws = discover_workspace().ok();
                    let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
                    // Unified lookup — workspace schemas first,
                    // then the embedded built-in ROS 2 registry (the exact
                    // .msg text the generated types compiled against), with
                    // ONE renderer for both sources. Replaces the sibling
                    // `../native_ros2_messages/msg` path probe, which only
                    // worked for workspaces created inside this repository.
                    let info = match schema_cmd::schema_info_unified_opt(schemas_dir, &name) {
                        Ok(info) => info,
                        // Unresolvable locally (neither workspace
                        // nor built-in). If it is a QUALIFIED `pkg/Type` and robots
                        // are discoverable, ask THEM — the automagic bar: a desk
                        // with zero local knowledge of a robot's custom types can
                        // still `schema info` them. Best-effort: a failed/empty
                        // fetch re-raises the original local error (exit code
                        // unchanged). A found remote schema prints with a loud
                        // provenance header, then returns Ok.
                        Err(
                            local_err @ cerulion_cli_engine::error::CliError::SchemaNotFound {
                                ..
                            },
                        ) => match schema_cmd::remote_fetch_target(&name) {
                            Some(requested) => {
                                let opts = topic_cmd::RemoteTopicsOptions {
                                    connect: vec![],
                                    listen: vec![],
                                    scouting: true,
                                };
                                // An empty remote answer means two different
                                // things. Only a SETTLED discovery licenses re-raising
                                // the terminal local "schema not found"; a cold
                                // `cerulion-netd` (or a transient gather that read
                                // nothing) searched NOTHING, so saying the type does
                                // not exist would be a claim with no evidence — the
                                // same false absence already removed from the topic
                                // path.
                                //
                                // The variant match lives in
                                // `topic_cmd::schema_info_remote_outcome`, NOT here —
                                // as a `match` in this file the mapping has no test,
                                // and a mutation reverting its cold-start arm would pass
                                // the whole suite. One `?` leaves no arm to mis-route.
                                let fetched = topic_cmd::fetch_remote_schema(&requested, &opts)?;
                                let reply = topic_cmd::schema_info_remote_outcome(
                                    &requested, fetched, local_err,
                                )?;
                                print!("{}", schema_cmd::render_remote_schema(&reply));
                                return Ok(());
                            }
                            None => return Err(local_err),
                        },
                        Err(e) => return Err(e),
                    };
                    // A workspace schema colliding with a
                    // built-in name is LOUD — one stderr warning per
                    // (workspace schema, shadowed built-in) pair, BEFORE
                    // the rendered output. `shadowed_builtin` is only
                    // `Some` on workspace results; the ", "-joined list
                    // splits back losslessly (names carry no commas).
                    if let Some(shadowed) = &info.shadowed_builtin {
                        for builtin in shadowed.split(", ") {
                            let bare = builtin.rsplit('/').next().unwrap_or(builtin);
                            for entry in &info.result.entries {
                                // The ONE canonical identity (the engine's
                                // `shadowed_builtin_names` decided the shadow
                                // that way): an entry declared `pkg::Type`
                                // must get the warning its `pkg/Type` twin
                                // gets, or the loud line goes missing on
                                // exactly the spelling the engine flagged.
                                let declared =
                                    cerulion_cli_engine::schema_cmd::normalize_schema(&entry.name);
                                if declared == builtin || declared == bare {
                                    eprintln!(
                                        "WARNING: workspace schema '{}' shadows built-in '{}' \
                                         — workspace wins for schema info",
                                        entry.name, builtin
                                    );
                                }
                            }
                        }
                    }
                    print!("{}", info);
                    Ok(())
                }
                SchemaAction::List => {
                    // Workspace-local schemas + the embedded
                    // built-in ROS 2 registry grouped by package, with
                    // shadowed built-ins marked inline. One renderer.
                    // Workspace-OPTIONAL — no workspace ⇒ the
                    // built-in registry alone (never a stray cwd `schemas/`).
                    let ws = discover_workspace().ok();
                    let schemas_dir = ws.as_ref().map(|w| w.schemas_dir.as_path());
                    let listing = schema_cmd::schema_list_opt(schemas_dir);
                    print!("{}", listing);
                    Ok(())
                }
            }
        }
        // The REMOVED `cerulion ros` family: `main` intercepts it above the
        // login gate and exits 2 with the migration message. Belt-and-braces
        // here in case a future reorder lets it through — the same message,
        // via the generic FAILURE mapping.
        Commands::Ros { .. } => Err(cerulion_cli_engine::error::CliError::Validation(
            ROS_VERB_MOVED_MSG.to_string(),
        )),
        Commands::Ros2 { action } => match action {
            // `ros2 run|launch|migrate` never reach run() — `main` intercepts
            // them before the generic dispatch to honor their typed exit
            // contracts (the {2, 69, 127, 1} run/launch contract, whose
            // successful exec() never returns at all; migrate's own
            // dry-run/write codes, 69 = engine not built included).
            Ros2Action::Run { .. } | Ros2Action::Launch { .. } | Ros2Action::Migrate { .. } => {
                unreachable!(
                    "`cerulion ros2 run|launch|migrate` are dispatched in main() before run()"
                )
            }
            Ros2Action::Attach {
                iface,
                domain,
                timeout,
                dry_run,
                yes,
                graph_name,
                topic_prefix,
                robot_name,
            } => {
                use std::io::IsTerminal as _;
                let ws = discover_workspace()?;
                let running = setup_ctrlc_handler()?;
                // Resolve --topic-prefix at the CLI
                // boundary (the resolve-and-report shape): a missing
                // leading '/' is prepended with a stderr `note:`; invalid
                // shapes are loud errors BEFORE discovery runs. The engine
                // re-validates as the enforcement backstop.
                let topic_prefix = match topic_prefix {
                    Some(raw) => {
                        let n = ros_cmd::normalize_topic_prefix(&raw)?;
                        if n.was_normalized {
                            eprintln!(
                                "note: --topic-prefix '{raw}' has no leading '/' — using \
                                 '{}' (pass the leading '/' to silence this note)",
                                n.prefix
                            );
                        }
                        Some(n.prefix)
                    }
                    None => None,
                };
                let opts = ros_cmd::RosAttachOptions {
                    iface,
                    domain_id: domain,
                    // clap's value_parser guarantees a
                    // FINITE window in (0, 3600] seconds (no silent 5.0
                    // coercion, and the upper bound kills the
                    // `from_secs_f64` overflow-panic class — it panics above
                    // ~5.8e11 s, so "finite positive" alone is NOT enough).
                    window: std::time::Duration::from_secs_f64(timeout),
                    dry_run,
                    assume_yes: yes,
                    graph_name,
                    topic_prefix,
                    // The robot-name label written as
                    // the graph `prefix:` — NOT the network announce identity
                    // (that resolves from the hostname / CERULION_ROBOT_IDENTITY
                    // at runtime). None ⇒ derived from the topics' shared
                    // namespace, else this machine's hostname.
                    robot_name,
                };
                // The live ros2-client/rustdds backend is wired HERE (the
                // composition root); the engine stays DDS-free over the seam.
                let discovery = cerulion_dds::LiveDiscovery;
                // Compose the production schema-acquisition
                // ladder — the wire-native `~/get_type_description` rung FIRST
                // (live DDS service calls on the same interface/domain), then
                // the local ament harvest. The rungs are OWNED here
                // (`ChainedAcquirer` borrows them); the engine stays DDS-free
                // over the acquirer seam (the wire rung rides a boxed
                // `dyn SchemaAcquirer`). The wire rung builds its own
                // participant lazily when it runs — AFTER discovery has dropped
                // its participant, so the one-per-process slot is free.
                let wire_params = cerulion_dds::DiscoveryParams {
                    only_networks: vec![opts.iface],
                    domain_id: opts.domain_id,
                    window: opts.window,
                };
                // The wire rung is
                // a REQUIRED argument to `production` (dropping the argument is
                // a compile error), and no wire-less named constructor
                // (`from_env`, `with_wire`) exists for a one-line edit to
                // reach — the only wire-less paths are the documented
                // hermetic TEST seams (`with_local_ament` / a struct literal),
                // which do not read as the production idiom.
                let acquirers = ros_cmd::AttachAcquirers::production(Box::new(
                    cerulion_dds::WireServiceAcquirer::new(wire_params),
                ));
                let chain = acquirers.chain();
                let is_tty = std::io::stdin().is_terminal();
                let mut confirm = ros_attach_confirm;
                let report = ros_cmd::ros_attach_with_acquirer(
                    &discovery,
                    &chain,
                    &ws.root,
                    &opts,
                    is_tty,
                    &mut confirm,
                )?;

                // The discovery report always reaches the user. On the
                // interactive arm the confirm closure already displayed the
                // full preview; on `--yes` print the preview (what was written);
                // otherwise print just the report.
                if report.preview_shown {
                    // Already displayed by the confirm provider.
                } else if matches!(report.outcome, ros_cmd::AttachOutcome::Written { .. }) {
                    print!("{}", report.preview);
                } else {
                    print!("{}", report.report);
                }

                // Extract the run target before borrowing `report.outcome`.
                let graph_to_run = report.graph_to_run.clone();
                match &report.outcome {
                    ros_cmd::AttachOutcome::DryRun | ros_cmd::AttachOutcome::NothingResolvable => {
                        Ok(())
                    }
                    ros_cmd::AttachOutcome::Declined => {
                        println!("ros2 attach: declined — nothing written.");
                        Ok(())
                    }
                    ros_cmd::AttachOutcome::Written {
                        graph_path,
                        config_path,
                        schema_writes,
                        ..
                    } => {
                        println!(
                            "ros2 attach: wrote {} and {}",
                            graph_path.display(),
                            config_path.display()
                        );
                        // The acquired `.msg` files are a THIRD
                        // write class. Name each on stderr (the `note:` advisory
                        // convention) — the stdout "wrote X and Y" line names only
                        // the graph + config. Surface a `.bak` backup when a prior
                        // store file was overwritten (explicit about clobbering a
                        // hand-dropped schema); silent when none were acquired
                        // (the shape from before schema acquisition existed).
                        for sw in schema_writes {
                            let note = match &sw.backup {
                                Some(bak) => format!(
                                    "note: acquired schema written to {} (backed up the prior \
                                     file to {})",
                                    sw.path.display(),
                                    bak.display()
                                ),
                                None => format!(
                                    "note: acquired schema written to {}",
                                    sw.path.display()
                                ),
                            };
                            eprintln!("{note}");
                        }
                        match graph_to_run {
                            Some(graph) => {
                                // The consented run MUST
                                // use the config the user just approved — a
                                // pre-existing DDS_BRIDGE_CONFIG would silently
                                // re-point the bridge at some OTHER mapping.
                                // Set it unconditionally for this run; a
                                // DIFFERENT pre-existing value gets a loud
                                // note naming both paths.
                                if let Some(note) = bridge_config_override_note(
                                    std::env::var_os("DDS_BRIDGE_CONFIG").as_deref(),
                                    config_path,
                                ) {
                                    eprintln!("{note}");
                                }
                                std::env::set_var("DDS_BRIDGE_CONFIG", config_path);
                                println!(
                                    "ros2 attach: running `cerulion graph run {graph} \
                                     --single-process` (Ctrl+C to stop)"
                                );
                                graph_cmd::graph_run(
                                    &ws.root,
                                    &ws.graphs_dir,
                                    &graph,
                                    running,
                                    graph_cmd::TimeSource::Real,
                                    graph_cmd::CpuDmaLockMode::Auto,
                                    graph_cmd::MonitorWaitMode::Auto,
                                    false, // skip_validate
                                    false, // prefer_release
                                    None,  // peer_loss
                                    true,  // single_process — the bridge is a monolith
                                    // `ros2 attach` is the FLAGSHIP
                                    // automagic surface — the attached robot's
                                    // topics ride the permissive network default
                                    // (gateway spawned, every produced topic
                                    // announced, zero config). Kill switch:
                                    // `CERULION_NETWORK=off` (env knob).
                                    false, // network_off
                                    graph_cmd::PRODUCTION_TRACE_LIMIT,
                                    None, // record
                                    graph_cmd::RecordEnvMode::None,
                                    cli::record_cpu_mode(None),
                                    None, // consent: internal caller, never auto-partitions
                                    // `ros2 attach` runs the bridge as a
                                    // MONOLITH, which mints no trace ring on any
                                    // path (the gating clock is wall-driven), so
                                    // there is nothing to decline.
                                    false, // no_rings
                                )
                            }
                            None => Ok(()),
                        }
                    }
                }
            }
        },
    }
}

/// The DDS_BRIDGE_CONFIG override decision — PURE so the
/// semantic is unit-testable. The consented run always uses the config
/// `ros2 attach` just wrote (the caller sets the var UNCONDITIONALLY); this
/// returns the loud note to print when a DIFFERENT pre-existing value is being
/// overridden for the run (unset or already-equal ⇒ `None`, silent).
fn bridge_config_override_note(
    existing: Option<&std::ffi::OsStr>,
    generated: &Path,
) -> Option<String> {
    match existing {
        Some(prev) if prev != generated.as_os_str() => Some(format!(
            "note: DDS_BRIDGE_CONFIG was already set to '{}' — overriding for this run with \
             the config you just approved: '{}'",
            Path::new(prev).display(),
            generated.display()
        )),
        _ => None,
    }
}

/// The `cerulion ros2 attach` interactive confirm — displays the
/// engine-built preview (discovery report + the two generated files + the run
/// line) and asks y/N on the real stdin. The engine threads it as the consent
/// seam's provider and only invokes it on the interactive arm.
fn ros_attach_confirm(preview: &str) -> CliResult<bool> {
    use std::io::Write as _;
    print!("{preview}");
    print!("\nWrite these files and run the graph? [y/N] ");
    std::io::stdout()
        .flush()
        .map_err(cerulion_cli_engine::error::CliError::Io)?;
    let mut line = String::new();
    std::io::stdin()
        .read_line(&mut line)
        .map_err(cerulion_cli_engine::error::CliError::Io)?;
    Ok(matches!(line.trim(), "y" | "Y" | "yes" | "Yes" | "YES"))
}

fn discover_workspace() -> CliResult<CerulionWorkspace> {
    let cwd = std::env::current_dir()?;
    CerulionWorkspace::discover(&cwd)
}

/// `cerulion viz [TOPIC…] [--robot NAME] [--detach]` — the
/// daemon CLIENT path. `cerulion viz` compiles no graph per run: it
/// ensures the long-lived `cerulion-vizd` daemon is running (spawning it detached
/// if absent) and sends one `attach` per requested topic over the daemon's NDJSON
/// control socket. The daemon owns the taps + the schema-generic dispatch + the
/// hosted proxy — so the CLI never links `rerun`; it speaks the socket protocol
/// (the `ros2 attach` decoupling). The viewer is Cerulion Studio, which connects
/// to that same daemon, so this verb starts no viewer.
///
/// - LOCAL (`--robot` absent): explicit `TOPIC` args are attached directly; ZERO
///   args ask the daemon to `discover` every attachable local topic and attach
///   each. A schema pin (`TOPIC=SCHEMA`) is accepted but not required (the daemon
///   resolves a local topic's schema from its frames).
/// - REMOTE (`--robot NAME`): at least one `TOPIC` is required (there is no remote
///   discover-all), but a schema pin is NOT: the daemon resolves each
///   topic's ROS type from the robot's served catalog and fetches any
///   custom type's schema over the network; a `TOPIC=SCHEMA` pin is still accepted
///   to skip the catalog lookup. The daemon declares ingress + taps the local
///   mirror; only a kill-switched (`CERULION_VIZD_NETWORK=off`) daemon surfaces the
///   not-network-configured residual.
///
/// Stays attached until Ctrl+C (then detaches the topics it added, leaving the
/// daemon running for the next run), or returns immediately with `--detach`
/// (leaving the taps + daemon up — the Studio handoff).
fn run_viz(
    running: Arc<AtomicBool>,
    topics: &[String],
    robot: Option<&str>,
    connect: &[String],
    listen: &[String],
    detach: bool,
) -> CliResult<()> {
    #[cfg(not(unix))]
    {
        let _ = (running, topics, robot, connect, listen, detach);
        Err(cerulion_cli_engine::error::CliError::Validation(
            "cerulion viz needs the cerulion-vizd daemon, which speaks over a Unix domain socket \
             and is not supported on this platform (Windows is a future target)"
                .to_string(),
        ))
    }
    #[cfg(unix)]
    {
        run_viz_unix(running, topics, robot, connect, listen, detach)
    }
}

/// One `TOPIC` / `TOPIC=SCHEMA` argument → `(absolute_topic, Option<schema>)`. A
/// bare topic is normalized to a leading `/` (the daemon taps absolute topics).
#[cfg(unix)]
fn parse_viz_topic_arg(arg: &str) -> (String, Option<String>) {
    let (raw, schema) = match arg.split_once('=') {
        Some((t, s)) => (t.trim(), Some(s.trim().to_string())),
        None => (arg.trim(), None),
    };
    let topic = if raw.starts_with('/') {
        raw.to_string()
    } else {
        format!("/{raw}")
    };
    (topic, schema)
}

#[cfg(unix)]
fn run_viz_unix(
    running: Arc<AtomicBool>,
    topics: &[String],
    robot: Option<&str>,
    connect: &[String],
    listen: &[String],
    detach: bool,
) -> CliResult<()> {
    use cerulion_cli_engine::error::CliError;
    use std::time::Duration;

    let val = |m: String| CliError::Validation(m);

    // 0. Validate the arg SHAPE + build the explicit attach list BEFORE spawning
    //    ANYTHING (a typo'd command must never leave an orphan daemon running).
    //    Every arg-shape error is decided here, up front;
    //    only the zero-arg LOCAL-discovery path genuinely needs the live daemon, so
    //    it alone is deferred (as `None`) until after the connect.
    let explicit_want: Option<Vec<(String, Option<String>)>> = if robot.is_some() {
        // REMOTE: a schema pin is OPTIONAL — the daemon resolves each topic's
        // ROS type from the robot's served catalog and fetches any custom
        // type's schema over the network. Topics are still REQUIRED (the daemon's
        // `discover` lists only LOCAL topics, so there is no remote discover-all).
        if topics.is_empty() {
            return Err(val(
                "cerulion viz --robot needs at least one TOPIC (a schema pin is optional — the \
                 daemon resolves the type from the robot's catalog): e.g. `/utlidar/cloud` or \
                 `/utlidar/cloud=sensor_msgs/PointCloud2`"
                    .to_string(),
            ));
        }
        Some(topics.iter().map(|a| parse_viz_topic_arg(a)).collect())
    } else if topics.is_empty() {
        None // zero-arg → ask the live daemon to discover (resolved after connect)
    } else {
        Some(topics.iter().map(|a| parse_viz_topic_arg(a)).collect())
    };

    // 1. Ensure the daemon is running (spawn it detached if absent), then connect.
    //    When THIS command starts the daemon, the --connect/--listen locators are
    //    threaded into its network config (env-at-spawn); a daemon already running
    //    keeps its own boot-time locators (a shared long-lived daemon's config is
    //    fixed at start — scouting covers the LAN regardless).
    let socket = viz_client::vizd_socket_path();
    let log_path = viz_client::vizd_log_path(&socket);
    let log_for_spawn = log_path.clone();
    let connect_for_spawn = connect.to_vec();
    let listen_for_spawn = listen.to_vec();
    let (mut conn, spawned) =
        viz_client::ensure_daemon(&socket, Duration::from_secs(10), move || {
            let bin = viz_client::find_vizd_binary().ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!(
                        "no `{}` daemon binary found beside `cerulion` or on $PATH. Cerulion \
                     Studio bundles its own copy; to use `cerulion viz` from a terminal, build \
                     the daemon from a checkout of the Cerulion source tree (`cargo build \
                     --release -p cerulion_vizd`) and put it beside `cerulion` or on $PATH",
                        viz_client::VIZD_BINARY
                    ),
                )
            })?;
            viz_client::spawn_vizd_detached(
                &bin,
                &log_for_spawn,
                &connect_for_spawn,
                &listen_for_spawn,
            )?;
            println!("cerulion viz: started the cerulion-vizd daemon");
            Ok(())
        })
        .map_err(|e| {
            // Surface the daemon log path: if the daemon spawned but died before it
            // could bind (or a foreign process wedged the socket), its stderr — the
            // only diagnostic — is in this file.
            val(format!(
                "cerulion viz: could not reach the cerulion-vizd daemon: {e}\n\
             (if the daemon spawned, its diagnostics are in {})",
                log_path.display()
            ))
        })?;

    // Note to the user: locators given, but the daemon was ALREADY running — this
    // command's locators do not reconfigure a live shared daemon (its network
    // config is fixed at start). Scouting (on by default) still covers a LAN
    // robot; restart the daemon to bake in new locators for a robot scouting
    // can't reach. Never a silent drop (the project's loud-inference rule).
    if !spawned && (!connect.is_empty() || !listen.is_empty()) {
        eprintln!(
            "cerulion viz: note — the cerulion-vizd daemon was already running, so \
             --connect/--listen apply only to a daemon THIS command starts; the running daemon \
             keeps its boot-time locators (scouting still covers LAN robots). Restart it \
             (stop the daemon, then re-run) to bake in these locators."
        );
    }

    // 2. Resolve the attach list (explicit shape validated above; the zero-arg
    //    path asks the now-live daemon to discover).
    let want: Vec<(String, Option<String>)> = match explicit_want {
        Some(w) => w,
        None => {
            println!("cerulion viz: discovering local topics from the daemon…");
            let discovered = conn
                .discover(1)
                .map_err(|e| val(format!("cerulion viz: daemon discover failed: {e}")))?;
            if discovered.is_empty() {
                return Err(val(
                    "cerulion viz: the daemon found no local topics to visualize — start some \
                     producers (or a `cerulion graph run`), or name a topic explicitly"
                        .to_string(),
                ));
            }
            discovered
        }
    };

    // 3. Attach each topic; print the resolved placement or the daemon error
    //    VERBATIM. All-fail is a nonzero exit (never a silent empty viewer).
    //
    //    The daemon's taps are GLOBAL shared state, so we track ONLY the taps THIS
    //    run created (`already_attached == false`) — a topic another controller
    //    (Studio / a `--detach` run) already tapped is left alone on exit, never
    //    torn down (tearing it down silently freezes the other
    //    controller's scene). `ok_count` (all successes, new OR already-tapped)
    //    drives the never-silent-empty-viewer gate; `created` drives the Ctrl+C
    //    detach.
    let mut created: Vec<String> = Vec::new();
    let mut ok_count = 0usize;
    let mut failures = 0usize;
    // The topic we were still discovering when the user interrupted us, if any.
    // A cancelled wait licenses NO claim about that topic — see the `Cancelled` arm.
    let mut cancelled_on: Option<String> = None;
    for (i, (topic, schema)) in want.iter().enumerate() {
        // KEEP ASKING while the daemon reports the topic is not discoverable
        // YET. Real-LAN convergence is ~3 to 13 s (measured), so a cold
        // desk's first attach lands before the robot is discoverable — and Studio's
        // layout restore, plus this loop over `want`, means one cold answer would
        // lose every topic. The wait is CLIENT-side because the control socket arms a
        // 5 s read deadline: a daemon that waited would not be slower, it would time
        // this verb out. `running` is the verb's own Ctrl-C flag, so the wait is
        // interruptible; the counter goes to stderr, never stdout.
        let mut progress = |waited: std::time::Duration| {
            eprintln!(
                "cerulion viz: waiting for '{topic}' to be discovered on the network… ({:.1}s)",
                waited.as_secs_f64()
            );
        };
        let attached = conn
            .attach_waiting_for_discovery(
                (i as u64) + 100,
                topic,
                None,
                robot,
                schema.as_deref(),
                viz_client::first_contact_attach_policy(),
                Some(running.as_ref()),
                &mut progress,
            )
            .map_err(|e| val(format!("cerulion viz: attach `{topic}` failed: {e}")))?;
        // An INTERRUPTED wait is not a verdict. The reply we are holding is
        // whatever the last round trip returned — typically empty + not-converged — so
        // rendering it would print the UNKNOWN ABSENCE paragraph and count a
        // failure for a question the user stopped us asking. Stop here instead, make no
        // claim, and skip the remaining topics: the user asked us to stop, not to keep
        // going without them. (The CLI precedent: Ctrl-C is an interruption, so
        // the run still exits 0 — the repo's `signal_matrix_e2e_test` rule.)
        if attached.outcome == viz_client::WaitOutcome::Cancelled {
            cancelled_on = Some(topic.clone());
            break;
        }
        let reply = attached.reply;
        if reply.ok {
            ok_count += 1;
            if !reply.already_attached {
                created.push(topic.clone());
            }
            let schema = reply.schema.as_deref().unwrap_or("(resolving…)");
            let archetype = reply.archetype.as_deref().unwrap_or("(pending)");
            let entity = reply.entity.as_deref().unwrap_or("");
            let note = if reply.already_attached {
                " (already tapped by another controller — left running on exit)"
            } else {
                ""
            };
            println!("cerulion viz: attached {topic} → {schema} [{archetype}] at {entity}{note}");
        } else {
            failures += 1;
            eprintln!(
                "cerulion viz: could not attach {topic}: {}",
                reply.error.as_deref().unwrap_or("(no error message)")
            );
        }
    }
    // The interruption is reported making NO claim about the topic. Pinned
    // NEGATIVELY against the absence/UNKNOWN vocabulary the UNKNOWN paragraph
    // uses (as was done for `topic echo`/`hz`) by
    // `crates/cerulion_cli/tests/viz_interrupt_e2e_test.rs`, which drives THIS verb over a fake
    // vizd socket and SIGINTs it mid-wait.
    if let Some(topic) = &cancelled_on {
        eprintln!(
            "cerulion viz: interrupted while discovering '{topic}' — nothing was concluded \
             about it, and the remaining topics were not attempted"
        );
    }
    // A run the USER stopped is not a run that failed to attach anything: skipping this
    // gate is what makes the interruption exit 0 rather than nonzero.
    if ok_count == 0 && cancelled_on.is_none() {
        return Err(val(format!(
            "cerulion viz: attached 0 of {} topic(s) — see the per-topic errors above",
            want.len()
        )));
    }
    if failures > 0 {
        eprintln!("cerulion viz: attached {ok_count} topic(s); {failures} failed (see above)");
    }
    // Nothing attached AND we were interrupted: there is no scene to hold open, so tear
    // down (there is nothing in `created`) and exit cleanly rather than printing a
    // "visualizing 0 topic(s)" banner.
    if ok_count == 0 && cancelled_on.is_some() {
        return Ok(());
    }

    // 4. Say where to SEE the topics. The verb starts no viewer: Cerulion Studio
    //    is the viewer, and it connects to this same daemon, so the scene is
    //    already waiting whether Studio is open now or opened later.
    println!(
        "cerulion viz: open Cerulion Studio to see {ok_count} attached topic(s) \
         (Studio connects to the same cerulion-vizd daemon)."
    );

    // 5. --detach: return, leaving the taps + daemon up (Studio handoff).
    if detach {
        println!(
            "cerulion viz: attached {ok_count} topic(s) and detached — the daemon keeps running."
        );
        return Ok(());
    }

    // Foreground: stay attached until Ctrl+C, then detach ONLY the taps WE created
    // (the shared daemon + any other controller's taps keep running).
    println!("cerulion viz: visualizing {ok_count} topic(s) via cerulion-vizd (Ctrl+C to stop)");
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(200));
    }
    for (i, topic) in created.iter().enumerate() {
        let _ = conn.detach((i as u64) + 1000, topic);
    }
    if created.is_empty() {
        println!(
            "cerulion viz: exiting — every tap was opened by another controller and keeps running."
        );
    } else {
        println!(
            "cerulion viz: detached {} topic(s) this run created; the daemon keeps running.",
            created.len()
        );
    }
    Ok(())
}

/// Map a `graph levels` report's partition verdict to the
/// process exit result — the inspect-then-fail contract (the `graph_validate`
/// precedent).
///
/// A valid or absent `process_groups:` partition (`partition_error == None`)
/// exits 0 (`Ok`); an INVALID one maps to a nonzero
/// [`CliError::Validation`](cerulion_cli_engine::error::CliError::Validation)
/// carrying the "not spawner-consumable" preface + the partition diagnostic
/// (which names the bridge node). Pure so the mapping is unit-testable: the
/// report PRINT is a side effect kept at the call site and always runs BEFORE
/// this decides the exit code (the report IS the diagnostic).
fn map_levels_partition_result(partition_error: Option<String>) -> CliResult<()> {
    match partition_error {
        None => Ok(()),
        Some(diagnostic) => Err(cerulion_cli_engine::error::CliError::Validation(format!(
            "process_groups partition is not spawner-consumable: {diagnostic}"
        ))),
    }
}

/// Remove iceoryx2's on-disk bookkeeping for **dead** nodes only.
///
/// iceoryx2 0.9 exposes `Node::try_cleanup_dead_nodes(config)`
/// (renamed from 0.8's `cleanup_dead_nodes`), which
/// walks the node registry and removes the stale resources of any
/// node whose owning process has died. Live nodes are left alone.
/// A graph that no longer matches a previous run's service shape
/// (e.g. a renamed topic or an output moved to a new node) will
/// see the dead node's services swept away as soon as the dead
/// node is detected, without disturbing any other Cerulion
/// process running concurrently on the same host.
///
/// Both `cerulion clean` and the implicit cleanup at the top of
/// `graph_run` route through
/// `cerulion_cli_engine::ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics`.
///
/// After the sweep, the verb reports the two POPULATIONS every
/// sweep has to walk — iceoryx2's node registry, and (on macOS/FreeBSD) the
/// `/tmp/*.shm_state` files `iceoryx2-pal-posix` leaves behind — and, unless
/// `--report-only`, reclaims the state files whose creating process is
/// PROVABLY gone. The diagnostic runs AFTER the sweep on purpose: the sweep
/// removes state files of its own through iceoryx2's `shm_unlink`, so a
/// population reported before it would over-count, and a reclamation running
/// first would unlink objects the sweep was about to inspect.
fn clean_iceoryx2_state(report_only: bool) -> CliResult<()> {
    let (result, converged) = clean_dead_nodes(report_only);
    // The reclamation is DOWNGRADED to a report when the sweep above
    // left dead nodes registered. A `.shm_state` file is the only mapping from
    // an iceoryx2 resource name to the object behind it, so removing one that a
    // still-registered dead node needs makes that node permanently
    // unreclaimable — the sweep can never read its details again. The
    // population is still reported; only the destructive half stands down.
    let reclaim = !report_only && converged;
    if !report_only && !converged {
        println!(
            "Skipping stale `.shm_state` reclamation: the dead-node sweep did not converge \
             (see above — a refused node is still registered, or the registry could not be \
             fully scanned). Removing a name mapping a still-registered dead node needs would \
             make it permanently unreclaimable. Clear the failures, then re-run `cerulion clean`."
        );
    }
    report_shm_state_population(!reclaim);
    result
}

/// The diagnostic block: the two populations, and the reclamation.
///
/// Never fails the verb. It is a diagnostic bolted onto a cleanup command, so
/// an unreadable directory or a refused unlink is reported in the block and
/// leaves `cerulion clean`'s own exit code to the sweep.
fn report_shm_state_population(report_only: bool) {
    use cerulion_cli_engine::shm_state;

    let nodes = shm_state::count_node_registry(
        &shm_state::iceoryx2_node_dir(),
        shm_state::SHM_STATE_REPORT_BUDGET,
    );

    // The state-file half exists only where `iceoryx2-pal-posix` keeps state
    // files, which is a strict subset of Unix — so the CODE is Unix-gated
    // (`LibcProbe` needs `kill`/`shm_unlink`) and the DECISION is the
    // platform predicate. A non-Unix build renders the not-applicable line.
    #[cfg(unix)]
    let shm = shm_state::platform_uses_shm_state_files().then(|| {
        // A REPORT must never make the operator wait, so it is bounded short
        // and may hand back a floor. A RECLAIM was explicitly asked for and
        // is the remedy, so it is bounded long enough to clear the measured
        // pathological directory in ONE run — bounded either way.
        let (mode, budget) = if report_only {
            (
                shm_state::ScanMode::ReportOnly,
                shm_state::SHM_STATE_REPORT_BUDGET,
            )
        } else {
            (
                shm_state::ScanMode::Reclaim,
                shm_state::SHM_STATE_RECLAIM_BUDGET,
            )
        };
        shm_state::scan(
            std::path::Path::new(shm_state::SHM_STATE_DIRECTORY),
            budget,
            shm_state::classify_budget(budget),
            mode,
            &shm_state::LibcProbe,
        )
    });
    #[cfg(not(unix))]
    let shm: Option<shm_state::ShmStateReport> = {
        let _ = report_only;
        None
    };

    for line in shm_state::render_lines(&nodes, shm.as_ref()) {
        println!("{line}");
    }
}

/// The original `clean` body: sweep iceoryx2's dead nodes and report the outcome.
///
/// Returns the verb's result and whether the registry CONVERGED — no dead node
/// left registered. The `.shm_state` reclamation is gated on that second
/// value; see [`clean_iceoryx2_state`].
///
/// ONE refusal shape is healed here rather than reported and
/// left standing. A dead node whose directory holds nothing but orphan port
/// tags — a publisher was destroyed while one of its loaned samples had been
/// leaked, so the port was deregistered but its tag outlived it — fails the
/// sweep at the final `rmdir` on EVERY sweep, forever, and one such node blocks
/// the `.shm_state` reclamation for good. After the first sweep, the nodes
/// whose refusal is EXACTLY that chain
/// (`orphan_port_tags::orphan_port_tag_candidates`) have their tags removed
/// (`orphan_port_tags::reclaim_orphan_port_tags`: the process must be provably
/// gone, and the directory — re-listed at that instant — must hold nothing
/// else; anything else is refused and named), then ONE more sweep runs and its
/// summary is printed; the convergence handed back is the SECOND sweep's, so
/// the reclamation gate sees the healed registry. That sweep runs whenever
/// candidates EXISTED — not only when a tag came off: a candidate whose
/// directory was already empty (`ReclaimVerdict::AlreadyEmpty` — another
/// session, or an interrupted earlier run, removed its tags) has nothing to
/// reclaim and converges on exactly that sweep. `--report-only` prints what
/// WOULD be removed, removes nothing, and skips the second sweep — nothing
/// changed, so it would re-find the same refusals. The source fix (the rmw
/// destroy path) is what stops the shape being minted; this only heals a
/// robot already carrying it.
fn clean_dead_nodes(report_only: bool) -> (CliResult<()>, bool) {
    use cerulion_cli_engine::ipc_cleanup;
    use cerulion_cli_engine::orphan_port_tags;
    use cerulion_cli_engine::shm_state::creator_verdict;

    let report = ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics();
    report_sweep(&report);
    // A failure of the registry WALK itself (printed first by `report_sweep`)
    // blocks convergence exactly as a refused node does: a dead node the walk
    // never reached is still registered and counted nowhere, so the
    // `.shm_state` reclamation must stand down for it.
    let converged = report.failed_cleanups == 0 && report.registry_errors.is_empty();
    if report.failed_cleanups == 0 {
        return (Ok(()), converged);
    }

    let config = orphan_port_tags::Iceoryx2Config::global_config();
    let candidates = orphan_port_tags::orphan_port_tag_candidates(&report.failures, config);
    if candidates.is_empty() {
        return (Ok(()), false);
    }
    // The liveness evidence is `shm_state`'s ONE predicate, reached through its
    // exported verdict — never re-spelt here.
    let reclaims = orphan_port_tags::reclaim_orphan_port_tags(
        &candidates,
        config,
        report_only,
        &creator_verdict,
    );
    for line in render_orphan_tag_reclaims(&reclaims, report_only) {
        println!("{line}");
    }
    if report_only {
        // Nothing changed: what WOULD be removed is listed above, and a
        // second sweep would only re-find the same refusals.
        return (Ok(()), false);
    }

    // ONE more sweep, whenever candidates EXISTED: a reclaimed node's
    // directory now holds nothing, and an `AlreadyEmpty` one never did, so
    // `remove_node` can finish what every earlier sweep could not. Its
    // counters are the proof the shape is healed (`cleanups` counts the
    // converged nodes), and its convergence is what the `.shm_state` gate
    // must see. A refused candidate costs one re-sweep that re-finds it —
    // cheap, and the report it prints is the truth of the registry NOW.
    println!("Second sweep after the orphan port-tag reclaim:");
    let second = ipc_cleanup::cleanup_dead_iceoryx2_nodes_with_diagnostics();
    report_sweep(&second);
    (
        Ok(()),
        second.failed_cleanups == 0 && second.registry_errors.is_empty(),
    )
}

/// Render the orphan port-tag reclaim for `cerulion clean`, one line per
/// candidate, exactly as the outcome was:
///
/// * `node <id> (pid <pid>, process gone): directory not empty — removed <k>
///   orphan port tag(s) [<port ids>]` — or `would remove` under
///   `--report-only`, which also closes with a line saying nothing was
///   removed;
/// * `node <id> (pid <pid>, process gone): directory already empty — nothing
///   to reclaim; the next sweep removes it` for an `AlreadyEmpty` verdict —
///   converged pending sweep, never rendered as a refusal;
/// * `node <id> (pid <pid>): not reclaimed — <reason>` for a refusal, with
///   any tags removed before a mid-way failure stated rather than hidden.
///
/// PURE (a `Vec` of lines) so `clean_diagnostic_tests` pins it against hand
/// oracles. Empty input renders NOTHING.
fn render_orphan_tag_reclaims(
    reclaims: &[cerulion_cli_engine::orphan_port_tags::OrphanTagReclaim],
    report_only: bool,
) -> Vec<String> {
    if reclaims.is_empty() {
        return Vec::new();
    }
    let ids = |removed: &[u128]| {
        removed
            .iter()
            .map(u128::to_string)
            .collect::<Vec<_>>()
            .join(", ")
    };
    let mut lines = vec![if report_only {
        "Orphan port tags `cerulion clean` would reclaim (report only — nothing removed):"
            .to_string()
    } else {
        "Reclaiming orphan port tags (a leaked loan kept each tag alive past its port's \
         deregistration; the process is gone and iceoryx2 already reclaimed the ports' resources):"
            .to_string()
    }];
    for reclaim in reclaims {
        use cerulion_cli_engine::orphan_port_tags::ReclaimVerdict;
        match &reclaim.verdict {
            ReclaimVerdict::AlreadyEmpty => lines.push(format!(
                "node {} (pid {}, process gone): directory already empty — nothing to reclaim; \
                 the next sweep removes it",
                reclaim.node_id, reclaim.pid
            )),
            ReclaimVerdict::Reclaimed => lines.push(format!(
                "node {} (pid {}, process gone): directory not empty — {} {} orphan port tag(s) [{}]",
                reclaim.node_id,
                reclaim.pid,
                if report_only { "would remove" } else { "removed" },
                reclaim.removed.len(),
                ids(&reclaim.removed)
            )),
            ReclaimVerdict::Refused(reason) if reclaim.removed.is_empty() => lines.push(format!(
                "node {} (pid {}): not reclaimed — {reason}",
                reclaim.node_id, reclaim.pid
            )),
            ReclaimVerdict::Refused(reason) => lines.push(format!(
                "node {} (pid {}): not fully reclaimed — {reason}; {} tag(s) removed before the \
                 failure [{}]",
                reclaim.node_id,
                reclaim.pid,
                reclaim.removed.len(),
                ids(&reclaim.removed)
            )),
        }
    }
    if report_only {
        lines.push("(run `cerulion clean` without `--report-only` to reclaim them)".to_string());
    }
    lines
}

/// Print one sweep's outcome: the registry-wide block first (if any), the
/// summary line (or the "nothing was reached" line when the registry could
/// not be scanned), then — only when something was refused — the per-cause
/// breakdown, the per-node refusal listing and the unclassified arm. Every
/// pre-existing line is byte-identical to what `cerulion clean` printed
/// before the orphan-tag reclaim existed; called once per sweep.
fn report_sweep(report: &cerulion_cli_engine::ipc_cleanup::CleanupReport) {
    // A failure of the registry WALK itself is printed first and
    // under its own heading, so the operator reads a global failure as
    // global — never as the cause of whichever node happens to be listed
    // next — and before the early "nothing to clean" return below, whose 0/0
    // counters would otherwise hide that nothing was reached.
    for line in render_registry_errors(&report.registry_errors) {
        println!("{line}");
    }
    if report.cleanups == 0 && report.failed_cleanups == 0 {
        if report.registry_errors.is_empty() {
            println!("No dead iceoryx2 nodes found — nothing to clean.");
        } else {
            // 0/0 with a scan failure is NOT "nothing to clean" — it is
            // "nothing was reached", and claiming the former would be an
            // affirmatively false absence.
            println!(
                "No dead iceoryx2 node was reached — the registry could not be fully scanned \
                 (see above), so this is not evidence that none is registered."
            );
        }
        return;
    }
    println!(
        "Cleaned {} dead iceoryx2 node(s); {} cleanup(s) failed.",
        report.cleanups, report.failed_cleanups
    );
    if report.failed_cleanups == 0 {
        return;
    }
    // Print per-cause breakdown when we have classified failures.
    if !report.failures_by_cause.is_empty() {
        println!("Failure breakdown:");
        for (cause, count) in &report.failures_by_cause {
            let remediation = match cause.as_str() {
                "permission denied" => {
                    "the dead node's resources belong to a different user; \
                     re-run as that user, or `chmod` the resources under `/tmp/iceoryx2/`"
                }
                "version mismatch" => {
                    "the dead node's on-disk state was produced by a different \
                     iceoryx2 version; live nodes' state is unaffected, but the \
                     dead-node remnants need a manual `rm -rf /tmp/iceoryx2/` to clear"
                }
                "lock contention" => {
                    "another process is concurrently cleaning the same dead node; \
                     usually self-resolves on retry"
                }
                "monitoring resource still in use" => {
                    "iceoryx2's monitoring layer holds a reference; should resolve \
                     after the holding process exits"
                }
                "iceoryx2 internal error" => {
                    "iceoryx2 refused to remove the node's registry entry; \
                     the node ids of the refused nodes listed below name the \
                     culprit and carry the sub-causes iceoryx2 logged. A directory \
                     holding only orphan port tags (a publisher destroyed while a \
                     loaned sample was leaked — the tag outlives the port) is \
                     reclaimed by this verb right after this listing; any other \
                     stranded entry never converges on its own"
                }
                _ => "see iceoryx2 logs for details",
            };
            println!("  {} × {} — {}", count, cause, remediation);
        }
    }
    // WHICH node, and WHY — the breakdown above is counts only.
    for line in render_refused_nodes(&report.failures) {
        println!("{line}");
    }
    if !report.unclassified.is_empty() {
        println!(
            "Unclassified failures ({}): {RAW_IOX2_LINES_HINT}",
            report.unclassified.len()
        );
    }
}

/// How many refused nodes `cerulion clean` lists in full before folding the
/// rest into an `… and N more` line. A pathological desk (for example
/// session-accumulated dead nodes) can hold dozens; the listing exists to
/// attribute ONE stranded node, so ten is plenty and keeps the verb's output
/// readable.
const REFUSED_NODES_SHOWN: usize = 10;

/// The hint kept for the arms the listing cannot serve — a refusal iceoryx2
/// gave NO captured explanation for, and the unclassified variants.
///
/// `RUST_LOG`, not `IOX2_LOG_LEVEL`: the diagnostic sweep
/// (`cleanup_dead_iceoryx2_nodes_with_diagnostics`) already pins iceoryx2's
/// own level at `Trace` for its duration and the bridge forwards every line
/// to `tracing` under `target: "iceoryx2"`, so the only gate left between the
/// raw lines and the terminal is this process's `RUST_LOG` filter. The
/// earlier internal-error remediation named `IOX2_LOG_LEVEL=trace`,
/// which is INERT for exactly this verb.
const RAW_IOX2_LINES_HINT: &str =
    "run with `RUST_LOG=iceoryx2=trace cerulion clean` for the raw iceoryx2 lines";

/// Render the registry-wide block for `cerulion clean`: the lines
/// iceoryx2 logged about the registry WALK itself (the full-scan failure,
/// `Node::list`'s and `list_all_nodes`' own lines), verbatim and indented
/// under their own heading, closed by ONE line saying what the block means.
/// Printed BEFORE anything about individual nodes, so a global failure is
/// read as global. Empty input renders NOTHING.
///
/// PURE for the same reason as [`render_refused_nodes`]: the unit tests in
/// `clean_diagnostic_tests` pin it against hand oracles.
fn render_registry_errors(errors: &[String]) -> Vec<String> {
    if errors.is_empty() {
        return Vec::new();
    }
    let mut lines = vec!["registry-wide sweep errors:".to_string()];
    lines.extend(errors.iter().map(|error| format!("  {error}")));
    lines.push(
        "  (the registry itself could not be fully scanned — not any node's fault; a dead node \
         the sweep never reached may still be registered and is counted nowhere above)"
            .to_string(),
    );
    lines
}

/// Render the per-node refusal listing for `cerulion clean`:
/// `  node <id>: <variant>` followed by the sub-cause lines iceoryx2 logged
/// about that node, indented beneath it, capped at [`REFUSED_NODES_SHOWN`]
/// nodes with an `… and N more` line. Empty input renders NOTHING — a
/// converged sweep must not print a header for a listing that has no rows.
///
/// PURE (a `Vec` of lines, no printing) so the unit tests in
/// `clean_diagnostic_tests` can pin it against hand oracles without a dead
/// node in the global iceoryx2 namespace. Explicit about absences: a refusal
/// line with no parenthesised variant renders `(variant not reported)`, and a
/// node with no captured sub-cause renders the trace hint instead of nothing,
/// never a fabricated cause.
fn render_refused_nodes(
    failures: &[cerulion_cli_engine::ipc_cleanup::FailedNodeCleanup],
) -> Vec<String> {
    if failures.is_empty() {
        return Vec::new();
    }
    let mut lines = vec!["Refused nodes:".to_string()];
    for failure in failures.iter().take(REFUSED_NODES_SHOWN) {
        let variant = if failure.variant.is_empty() {
            "(variant not reported)"
        } else {
            failure.variant.as_str()
        };
        lines.push(format!("  node {}: {}", failure.node, variant));
        if failure.causes.is_empty() {
            lines.push(format!(
                "    (no sub-cause captured — {RAW_IOX2_LINES_HINT})"
            ));
        }
        for cause in &failure.causes {
            lines.push(format!("    {cause}"));
        }
    }
    if failures.len() > REFUSED_NODES_SHOWN {
        lines.push(format!(
            "  … and {} more",
            failures.len() - REFUSED_NODES_SHOWN
        ));
    }
    lines
}

/// Implement `cerulion trace inspect <dir>` — read JSON-Lines bag
/// files written by `cerulion_core::trace::BagWriter` and print a
/// human-readable timeline. Files are read in lexicographic order
/// (matches the `trace_XXXX.jsonl` sequential naming).
fn trace_inspect(
    dir: &str,
    filter: Option<&str>,
    limit: Option<usize>,
    reverse: bool,
) -> CliResult<()> {
    let dir_path = std::path::Path::new(dir);
    if !dir_path.is_dir() {
        return Err(cerulion_cli_engine::error::CliError::Validation(format!(
            "trace inspect: `{}` is not a directory",
            dir
        )));
    }
    // Collect bag files in lexicographic order.
    let mut bag_files: Vec<std::path::PathBuf> = std::fs::read_dir(dir_path)
        .map_err(|e| {
            cerulion_cli_engine::error::CliError::Validation(format!(
                "trace inspect: failed to read directory `{}`: {}",
                dir, e
            ))
        })?
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("trace_") && n.ends_with(".jsonl"))
        })
        .collect();
    bag_files.sort();
    if bag_files.is_empty() {
        println!("No trace_*.jsonl files found in `{}`", dir);
        return Ok(());
    }

    // Collect all lines into memory — Regime A entries are tiny
    // (~80 B each) so even an hour of trace at 1000 Hz fits in
    // < 300 MB.
    let mut lines: Vec<String> = Vec::new();
    for file in &bag_files {
        let contents = std::fs::read_to_string(file).map_err(|e| {
            cerulion_cli_engine::error::CliError::Validation(format!(
                "trace inspect: failed to read `{}`: {}",
                file.display(),
                e
            ))
        })?;
        for line in contents.lines() {
            if let Some(topic_filter) = filter {
                // Cheap substring check; the JSON shape always
                // contains `"topic":"<value>"`.
                let needle = format!("\"topic\":\"{}\"", topic_filter);
                if !line.contains(&needle) {
                    continue;
                }
            }
            lines.push(line.to_string());
        }
    }

    if reverse {
        lines.reverse();
    }
    if let Some(n) = limit {
        lines.truncate(n);
    }

    // Format each line as a compact human-readable timeline entry.
    for line in &lines {
        match parse_jsonl_record(line) {
            Some((topic, seq, ts, schema)) => {
                println!("{} seq={} t={}ns schema=0x{:016X}", topic, seq, ts, schema);
            }
            None => {
                // Pass through the raw line — better than dropping
                // unrecognized records silently.
                println!("(malformed) {}", line);
            }
        }
    }

    println!();
    println!(
        "[{} record(s) across {} file(s){}]",
        lines.len(),
        bag_files.len(),
        if let Some(t) = filter {
            format!(" filtered by topic={:?}", t)
        } else {
            String::new()
        }
    );
    Ok(())
}

/// Minimal JSON-Lines parser for the BagWriter format
/// (`{"topic":"X","seq":N,"ts_ns":N,"schema_hash":"0xH"}`). Returns
/// `(topic, sequence, publish_time_ns, schema_hash)` on success.
fn parse_jsonl_record(line: &str) -> Option<(String, u64, u64, u64)> {
    // The previous inclusive `rest[..=end]` slice (with `end` the
    // delimiter's own index for the number branch) captured the trailing
    // `,`/`}` INTO the extracted numeric substring — `"seq":5,...` sliced to
    // `"5,"`, which `.parse::<u64>()` always rejects. That made this
    // function return `None` for every conforming line (seq/ts_ns are never
    // last in the object), so `trace_inspect` printed EVERY well-formed
    // record through the "(malformed)" fallback and the pretty
    // `<topic> seq=.. t=..ns schema=..` line was dead code. Slice up to
    // (not including) the delimiter for both branches instead.
    let extract = |key: &str| -> Option<&str> {
        let pat = format!("\"{}\":", key);
        let start = line.find(&pat)? + pat.len();
        let rest = &line[start..];
        if let Some(stripped) = rest.strip_prefix('"') {
            let end = stripped.find('"')?;
            Some(&stripped[..end])
        } else {
            let end = rest.find([',', '}'])?;
            Some(&rest[..end])
        }
    };
    let topic = extract("topic")?.to_string();
    let seq: u64 = extract("seq")?.parse().ok()?;
    let ts: u64 = extract("ts_ns")?.parse().ok()?;
    let schema_raw = extract("schema_hash")?;
    let schema = u64::from_str_radix(schema_raw.trim_start_matches("0x"), 16).ok()?;
    Some((topic, seq, ts, schema))
}

/// Render a [`cerulion_core::MacroPolicy`] as a short, human-readable label
/// for the `cerulion node list` and `cerulion node info` outputs.
/// `None` falls back to `"-"` so columns stay aligned across rows.
fn format_policy_short(spec: Option<&cerulion_core::MacroPolicy>) -> String {
    use cerulion_core::MacroPolicy;
    match spec {
        Some(MacroPolicy::Period { period_ms }) => format!("period {}ms", period_ms),
        Some(MacroPolicy::Sync { window_ms }) => format!("sync {}ms", window_ms),
        Some(MacroPolicy::UnboundedSync) => "sync ∞".to_string(),
        Some(MacroPolicy::External) => "external".to_string(),
        Some(MacroPolicy::DataTrigger { input_name }) => format!("trigger:{}", input_name),
        None => "-".to_string(),
    }
}

/// Reject duplicate port names across `-T` and `-i` on `node
/// create`. Passing `-T sensor_msgs/Image foo -i sensor_msgs/Imu
/// foo` would otherwise produce a `lib.rs` declaring `foo` twice
/// — invalid Rust caught only at `cargo build` with an opaque
/// "duplicate field" error.
///
/// `node_modify_add_port` already checks for this via
/// `metadata.has_input(port_name)`, and the engine's
/// `node_create_with_options` mirrors the gate too (per the
/// CLI/engine contract alignment rule) — this CLI helper is the
/// user-facing first line: a clear "drop either the `-T` or the
/// `-i`" message before the request ever reaches the engine.
fn check_dash_t_dash_i_name_collision(
    trigger_inputs: &[(String, String)],
    regular_inputs: &[(String, String)],
) -> CliResult<()> {
    use cerulion_cli_engine::error::CliError;
    if let Some((_, t_name)) = trigger_inputs.first() {
        if regular_inputs.iter().any(|(_, n)| n == t_name) {
            return Err(CliError::Validation(format!(
                "port name '{t_name}' appears in both `-T` (trigger) and \
                 `-i` (regular input). Each port name must be unique. \
                 Drop either the `-T` or the `-i` for this name."
            )));
        }
    }
    Ok(())
}

/// Resolve the trigger policy for `cerulion node create` from the
/// `--policy` flag, the `-T` flag, and the regular `-i` inputs.
///
/// Defaulting rules (when `--policy` is absent):
/// - `-T` set → `DataTrigger { input_name: <T's name> }`
///   (an explicit `-T` declares the trigger, so the policy is
///   threaded through as `data_trigger=NAME`).
/// - 0 inputs (no `-i` and no `-T`) → error: a source-only node
///   must declare a non-data policy explicitly
///   (`--policy period_ms=N` or `--policy external`).
/// - 1+ inputs via `-i` only → `None` (the source emits no
///   node-level policy attribute; the runtime fires on any input
///   arrival and emits a warning at graph-build time so the
///   user notices). The warning is intentional: a user might want
///   a different policy (Sync, data_trigger on a
///   specific input), and silence would let unintended firing
///   behavior ship.
///
/// When `--policy` is present:
/// - `--policy data_trigger=NAME` and `-T NAME'` must agree on the
///   trigger input.
/// - `--policy data_trigger=NAME` requires NAME to match exactly
///   one of the `-i` or `-T` inputs.
/// - Non-data policies (Period/Sync/External) ignore the
///   `-T` / `-i` set and use the explicit policy as-is. `-T` is
///   incompatible with non-data policies — the caller errors.
fn resolve_create_policy(
    explicit: Option<&cerulion_core::MacroPolicy>,
    trigger_input: Option<&(String, String)>,
    regular_inputs: &[(String, String)],
) -> CliResult<Option<cerulion_core::MacroPolicy>> {
    use cerulion_cli_engine::error::CliError;
    use cerulion_core::MacroPolicy;
    match (explicit, trigger_input) {
        (Some(MacroPolicy::DataTrigger { input_name }), Some((_, t_name))) => {
            if input_name != t_name {
                return Err(CliError::Validation(format!(
                    "`--policy data_trigger={input_name}` and `-T <SCHEMA> {t_name}` disagree \
                     on the trigger input"
                )));
            }
            Ok(Some(MacroPolicy::DataTrigger {
                input_name: input_name.clone(),
            }))
        }
        (Some(_non_data), Some(_)) => Err(CliError::Validation(
            "`-T` declares a data-trigger input, which conflicts with a non-data `--policy`. \
             Drop `-T` or change the policy."
                .to_string(),
        )),
        (Some(MacroPolicy::DataTrigger { input_name }), None) => {
            let matches_input = regular_inputs.iter().any(|(_, n)| n == input_name);
            if !matches_input {
                return Err(CliError::Validation(format!(
                    "`--policy data_trigger={input_name}` requires `-i SCHEMA {input_name}` \
                     (or `-T SCHEMA {input_name}`) to declare the trigger input"
                )));
            }
            Ok(Some(MacroPolicy::DataTrigger {
                input_name: input_name.clone(),
            }))
        }
        (Some(p), None) => Ok(Some(p.clone())),
        (None, Some((_, name))) => Ok(Some(MacroPolicy::DataTrigger {
            input_name: name.clone(),
        })),
        (None, None) => {
            if regular_inputs.is_empty() {
                Err(CliError::Validation(
                    "source-only nodes (no `-i` or `-T`) must declare a non-data trigger policy. \
                     Pass `--policy period_ms=N` or `--policy external`."
                        .to_string(),
                ))
            } else {
                // 1+ inputs with no explicit `--policy` or `-T`:
                // emit no node-level policy attribute. The runtime
                // fires on any input arrival and emits a warning
                // at graph-build time so the user notices and can
                // pick a more specific policy if desired.
                Ok(None)
            }
        }
    }
}

/// The stderr note `node create` prints for the two macro-node shapes it
/// accepts but `#[cerulion_node]` refuses at build time: inputs with no trigger
/// policy at all, and a sync policy (which needs two or more trigger inputs,
/// more than one `node create` call can declare). Text only: what is created is
/// unchanged. `None` for every shape that builds as written, and for a
/// raw-FFI node, which the macro never sees.
fn node_create_build_note(
    node_type: &str,
    policy: Option<&cerulion_core::MacroPolicy>,
    has_trigger_input: bool,
    raw_ffi: bool,
) -> Option<String> {
    use cerulion_core::MacroPolicy;
    if raw_ffi {
        return None;
    }
    match policy {
        None if !has_trigger_input => Some(format!(
            "note: '{node_type}' has no trigger policy yet, so `cerulion node build {node_type}` \
             fails until it has one. Set it with `cerulion node modify {node_type} --policy \
             data_trigger=<INPUT>` (or `--policy period_ms=<N>`)."
        )),
        Some(MacroPolicy::Sync { .. }) | Some(MacroPolicy::UnboundedSync) => Some(format!(
            "note: a sync node aligns two or more trigger inputs, so `cerulion node build \
             {node_type}` fails until '{node_type}' has them. Add inputs with `cerulion node \
             modify {node_type} -i SCHEMA NAME`, then mark each input to align as \
             `#[input(trigger)]` in nodes/{node_type}/src/lib.rs."
        )),
        _ => None,
    }
}

/// Parse a `--policy <SPEC>` argument into a [`cerulion_core::MacroPolicy`].
///
/// Accepted forms:
/// - `period_ms=N` → `Period { period_ms: N }`
/// - `sync_window_ms=N` → `Sync { window_ms: N }`
/// - `external` → `External`
/// - `data_trigger=NAME` or `trigger=NAME` → `DataTrigger { input_name: NAME }`
///
/// Bare `period_ms` / `sync_window_ms` (no `=N`) are
/// rejected with a "requires a value" error. Zero values for either of
/// the two time-based forms (`period_ms`, `sync_window_ms`) are rejected
/// with a "must be > 0" error since they would spin-loop the scheduler.
/// (Was "three" before the `deadline_ms` trigger was removed.)
fn parse_policy_spec(spec: &str) -> CliResult<cerulion_core::MacroPolicy> {
    use cerulion_cli_engine::error::CliError;
    use cerulion_core::MacroPolicy;
    let trimmed = spec.trim();
    if trimmed.eq_ignore_ascii_case("external") {
        return Ok(MacroPolicy::External);
    }
    let (key, value) = match trimmed.split_once('=') {
        Some((k, v)) => (k.trim(), Some(v.trim())),
        None => (trimmed, None),
    };
    let parse_u64_positive = |label: &str, v: Option<&str>| -> CliResult<u64> {
        let v = v.ok_or_else(|| {
            CliError::Validation(format!(
                "`--policy {label}` requires a value, e.g. `--policy {label}=100`"
            ))
        })?;
        let parsed = v.parse::<u64>().map_err(|_| {
            CliError::Validation(format!(
                "`--policy {label}={v}` — value must be a positive integer (milliseconds)"
            ))
        })?;
        // Zero-duration trigger policies would spin-loop the
        // scheduler. The macro's `validate_policy_bounds` rejects
        // them at compile time as a defence-in-depth — but the CLI
        // shouldn't synthesize the zero-valued attribute in the
        // first place, so users get an immediate clear error
        // instead of a downstream `cargo build` failure.
        if parsed == 0 {
            return Err(CliError::Validation(format!(
                "`--policy {label}=0` — value must be > 0 (zero-duration {label} would spin-loop the scheduler)"
            )));
        }
        Ok(parsed)
    };
    let parse_name = |label: &str, v: Option<&str>| -> CliResult<String> {
        let v = v.ok_or_else(|| {
            CliError::Validation(format!(
                "`--policy {label}` requires a value, e.g. `--policy {label}=count`"
            ))
        })?;
        if v.is_empty() {
            return Err(CliError::Validation(format!(
                "`--policy {label}=` — value (input name) must not be empty"
            )));
        }
        Ok(v.to_string())
    };
    match key {
        "period_ms" => Ok(MacroPolicy::Period {
            period_ms: parse_u64_positive("period_ms", value)?,
        }),
        "sync_window_ms" => Ok(MacroPolicy::Sync {
            window_ms: parse_u64_positive("sync_window_ms", value)?,
        }),
        "data_trigger" | "trigger" => Ok(MacroPolicy::DataTrigger {
            input_name: parse_name(key, value)?,
        }),
        other => Err(CliError::Validation(format!(
            "unknown policy spec `{other}`. Accepted: \
             `period_ms=N`, `sync_window_ms=N`, `external`, \
             `data_trigger=NAME` (or `trigger=NAME`)"
        ))),
    }
}

/// The shared interactive partition confirm: displays the
/// engine-built preview, asks y/N on the real stdin, and returns the answer.
/// Used by BOTH `graph partition` and `graph run`'s auto-partition pre-flight
/// (the engine threads it as the consent seam's `confirm` provider and only
/// invokes it on the interactive arm).
fn stdin_yes_no_confirm(preview: &str) -> CliResult<bool> {
    prompt_yes_no(preview, "Apply this partition to the graph file?")
}

/// Print `preview`, then ask `question` as a y/N prompt on stdin.
///
/// This was factored out of [`stdin_yes_no_confirm`] rather than adding a
/// second copy: the ANSWER RULE — only an explicit yes counts, and anything
/// else (including a bare newline and an EOF) declines — is the part that must
/// not diverge between two write-gating prompts. Only the question differs.
fn prompt_yes_no(preview: &str, question: &str) -> CliResult<bool> {
    use std::io::Write as _;
    print!("{preview}");
    print!("\n{question} [y/N] ");
    std::io::stdout()
        .flush()
        .map_err(cerulion_cli_engine::error::CliError::Io)?;
    let mut line = String::new();
    std::io::stdin()
        .read_line(&mut line)
        .map_err(cerulion_cli_engine::error::CliError::Io)?;
    Ok(matches!(line.trim(), "y" | "Y" | "yes" | "Yes" | "YES"))
}

/// Whether a command must run under a logged-in-ever identity (the
/// runtime login gate — "every command requires a logged-in-ever
/// identity"). Everything a user invokes gates, `account` and its sub-verbs
/// included: those talk to the account service and need a session anyway, so
/// exempting them would only move the failure later and word it worse.
///
/// Two more things answer above this gate and therefore need no exemption here:
/// clap's own `--help` and `--version`, which exit inside `Cli::parse()` before
/// `main` has a body to run, and the usage refusals `main` performs before the
/// gate call (a moved verb, a malformed resim invocation). You do not have to
/// prove who you are to be told a command line is wrong.
///
/// The exemptions are:
///
/// - **`login`** itself — it IS the login (gating it would auto-trigger a device
///   flow BEFORE the real login runs — a doubled/confusing login).
/// - **`graph run-worker` / `graph run-gateway`** — the internal subprocess
///   verbs the multi-process supervisor / monolith parent spawns. **Why exempt
///   is safe:** the PARENT already passed this gate, and it spawns these children
///   with the parent's env (the cached auth is visible); gating them would make
///   each spawned worker re-hit the gate and — on a child env without a visible
///   auth cache — auto-trigger an interactive device-code login MID-SPAWN,
///   wedging the whole run. (`bagd` is also internal but is dispatched away in
///   `main` BEFORE this hook, so it never reaches here.)
///
/// The enforcement boundary: `run-worker` and `run-gateway` are hidden from
/// help but still reachable as standalone `cerulion graph run-worker`
/// invocations, so a machine that has never signed in can run them ungated.
/// The local gate is a convenience check on a user's own machine; access to
/// an account and its robots is enforced by the account service.
fn command_needs_identity(command: &Commands) -> bool {
    !matches!(
        command,
        Commands::Login
            // Emitting a completion script is a pure local text
            // render — gating it behind the login flow would make `cerulion
            // completions zsh` in a shell rc file block startup on a device
            // -code prompt nobody is watching.
            | Commands::Completions { .. }
            | Commands::Graph {
                action: GraphAction::RunWorker { .. } | GraphAction::RunGateway { .. },
            }
    )
}

/// Install the process-wide shutdown-signal handler and return the shared
/// `running` flag every long-running verb (`graph run`, `topic echo`/`hz`, …)
/// polls to exit cleanly.
///
/// On Unix this handler fires for SIGINT **and** SIGTERM **and**
/// SIGHUP — the `ctrlc` crate's `termination` feature (declared in
/// `crates/cerulion_cli/Cargo.toml`) routes all three to this ONE closure. Making the
/// feature explicit closes a feature-unification fragility: coverage that
/// comes only from the `cfg(unix)` `cerulion_bagd` dependency pulling
/// `termination` in transitively is silently stripped by un-linking bagd,
/// TERM/HUP handling with it. A DIRECTED `kill <pid>` / systemd stop (SIGTERM)
/// flips `running` the same as an interactive Ctrl-C.
///
/// The closure is a pure `store(false)` — async-signal-safe (a single relaxed
/// atomic write; no allocation, locking, or reentrant I/O), so it is safe to
/// run in signal context. Making the feature explicit left it BYTE-IDENTICAL.
fn setup_ctrlc_handler() -> CliResult<Arc<AtomicBool>> {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::Relaxed);
    })
    .map_err(|e| {
        cerulion_cli_engine::error::CliError::Validation(format!(
            "failed to set Ctrl+C handler: {}",
            e
        ))
    })?;
    Ok(running)
}

fn init_logging(verbose: bool, quiet_default: bool) {
    cerulion_core::init_logging(verbose, quiet_default);
}

/// Parse port args: \[SCHEMA, NAME\] or \[SCHEMA\] (derive name from schema).
///
/// The CLI accepts both `sensor_msgs/Image` (slash form, the
/// canonical form `docs/user-api.md` documents) and `sensor_msgs::Image`
/// (Rust-style colon form). We canonicalize to the slash form
/// right here at the input boundary so every downstream consumer
/// (templates codegen, the source code, graph YAML staging) sees
/// one shape.
fn parse_port_args(args: &[String]) -> (String, String) {
    // Clap enforces `num_args = 2` on the flags, so a single
    // invocation gives exactly 2 elements. The caller must check
    // for multi-invocation (4+ elements) BEFORE calling this
    // function. We defensively assert here so a future caller
    // that forgets the guard doesn't silently truncate.
    debug_assert_eq!(
        args.len(),
        2,
        "parse_port_args expects exactly 2 args; caller must reject multi-invocation \
         (`args.len() > 2`) before calling"
    );
    let schema = cerulion_cli_engine::schema_cmd::normalize_schema(&args[0]);
    let name = args.get(1).cloned().unwrap_or_else(|| {
        // Defensive fallback for release builds where the
        // `debug_assert_eq!` is compiled out and clap somehow
        // delivered only 1 arg: derive name from schema.
        schema.rsplit('/').next().unwrap_or(&schema).to_lowercase()
    });
    (schema, name)
}

/// Resolve one port-schema argument for the `node create` /
/// `node modify` arms and report the resolution on stderr.
///
/// Bare names resolve via [`schema_cmd::resolve_port_schema`]
/// (workspace YAML schemas win, then the workspace `.msg` store, then
/// built-ins; a unique short name qualifies; ambiguous/unknown names are
/// loud errors that propagate to the standard `Error:` handler). A name
/// resolving to the `.msg` store — bare or qualified — is REFUSED via
/// [`schema_cmd::refuse_store_port_scaffold`]: a store type has no
/// generated Rust type, so the scaffolded import could never compile,
/// and the refusal (naming the store type + the remedy) is the store
/// hit's loud surface here. The engine re-refuses as the enforcement
/// backstop. The presentation contract (the stderr-note option) for what
/// proceeds:
/// - a bare name resolved to a built-in prints a `note:` on EVERY
///   invocation (qualifying the name silences it);
/// - a workspace YAML schema shadowing built-in(s) prints the
///   standard shadow `WARNING:` (workspace wins);
/// - already-qualified names and non-shadowing workspace names are
///   silent.
///
/// Returns the RESOLVED schema string to pass down to the engine,
/// which re-resolves as the enforcement backstop (idempotent, so the
/// double resolution is free and byte-identical).
fn resolve_and_report(schemas_dir: &Path, raw: &str) -> CliResult<String> {
    use cerulion_cli_engine::schema_cmd::PortSchemaProvenance;
    let resolved = schema_cmd::resolve_port_schema(schemas_dir, raw)?;
    schema_cmd::refuse_store_port_scaffold(&resolved, raw)?;
    match &resolved.provenance {
        PortSchemaProvenance::ResolvedBuiltin { qualified } => {
            eprintln!(
                "note: resolved bare schema '{raw}' to built-in '{qualified}' \
                 (qualify it to silence this note)"
            );
        }
        PortSchemaProvenance::Workspace {
            shadowed: Some(shadowed),
        } => {
            eprintln!(
                "WARNING: workspace schema '{raw}' shadows built-in '{shadowed}' \
                 — workspace wins for node ports"
            );
        }
        // The store provenances are refused above and cannot reach this
        // match; they are listed (not wildcarded) so a NEW provenance
        // variant still fails compilation here instead of going silent.
        PortSchemaProvenance::Qualified
        | PortSchemaProvenance::QualifiedStore
        | PortSchemaProvenance::ResolvedStore { .. }
        | PortSchemaProvenance::Workspace { shadowed: None } => {}
    }
    Ok(resolved.schema)
}

/// Parse input binding pairs from flat arg list.
fn parse_input_bindings(args: &[String]) -> Vec<(String, String)> {
    args.chunks(2)
        .filter_map(|chunk| {
            if chunk.len() == 2 {
                Some((chunk[0].clone(), chunk[1].clone()))
            } else {
                None
            }
        })
        .collect()
}

/// Resolve graph name: explicit, auto-select if one exists, or error.
fn resolve_graph_name(ws: &CerulionWorkspace, explicit: Option<String>) -> CliResult<String> {
    if let Some(name) = explicit {
        return Ok(name);
    }

    // Auto-select if exactly one graph exists
    let graphs: Vec<_> = std::fs::read_dir(&ws.graphs_dir)?
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.path()
                .extension()
                .map(|ext| ext == "yaml")
                .unwrap_or(false)
        })
        .collect();

    match graphs.len() {
        0 => Err(cerulion_cli_engine::error::CliError::Validation(
            "no graphs found — create one with `cerulion graph create <name>`".to_string(),
        )),
        1 => {
            let name = graphs[0]
                .path()
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .to_string();
            Ok(name)
        }
        _ => Err(cerulion_cli_engine::error::CliError::Validation(
            "multiple graphs found — specify with -g <name>".to_string(),
        )),
    }
}

#[cfg(test)]
mod login_gate_exemption_tests {
    use super::*;

    // Pin the load-bearing exemption surface so a
    // future edit that drops/reorders the match — un-exempting `login` (a doubled
    // login) or the worker/gateway subprocess verbs (a mid-spawn wedge) — fails
    // CI instead of shipping green.

    #[test]
    fn login_verb_is_exempt() {
        assert!(!command_needs_identity(&Commands::Login));
    }

    #[test]
    fn internal_subprocess_verbs_are_exempt() {
        assert!(!command_needs_identity(&Commands::Graph {
            action: GraphAction::RunWorker {
                plan: PathBuf::from("plan.json"),
            },
        }));
        assert!(!command_needs_identity(&Commands::Graph {
            action: GraphAction::RunGateway {
                handoff: PathBuf::from("handoff.json"),
            },
        }));
    }

    #[test]
    fn ordinary_verbs_are_gated() {
        // A representative spread of user-facing verbs must gate.
        assert!(command_needs_identity(&Commands::Clean {
            report_only: false
        }));
        assert!(command_needs_identity(&Commands::Graph {
            action: GraphAction::List,
        }));
        assert!(command_needs_identity(&Commands::Node {
            action: NodeAction::List,
        }));
    }
}

#[cfg(test)]
mod parse_port_args_tests {
    use super::*;

    #[test]
    fn parse_port_args_canonicalizes_colon_form() {
        // Colon-form schema gets canonicalised to slash form.
        let args = vec!["sensor_msgs::Image".to_string(), "image".to_string()];
        assert_eq!(
            parse_port_args(&args),
            ("sensor_msgs/Image".to_string(), "image".to_string()),
            "colon-form input must be canonicalised to slash form"
        );
    }

    #[test]
    fn parse_port_args_passes_slash_form() {
        let args = vec!["sensor_msgs/Image".to_string(), "image".to_string()];
        assert_eq!(
            parse_port_args(&args),
            ("sensor_msgs/Image".to_string(), "image".to_string()),
            "slash-form input must round-trip unchanged"
        );
    }

    #[test]
    fn parse_port_args_preserves_explicit_name() {
        let args = vec!["sensor_msgs::Image".to_string(), "frame".to_string()];
        assert_eq!(
            parse_port_args(&args),
            ("sensor_msgs/Image".to_string(), "frame".to_string()),
            "explicit NAME arg must be preserved verbatim"
        );
    }

    #[test]
    fn parse_port_args_unqualified_passes_through() {
        // Unqualified schemas (in-workspace types with no package
        // prefix) round-trip unchanged.
        let args = vec!["MyType".to_string(), "mytype".to_string()];
        assert_eq!(
            parse_port_args(&args),
            ("MyType".to_string(), "mytype".to_string())
        );
    }
}

#[cfg(test)]
mod resolve_and_report_tests {
    use super::*;

    // The helper's RETURN-VALUE branches. The literal stderr
    // strings (the `note:` and the shadow `WARNING:`) are pinned by
    // the subprocess e2e — no stderr capture machinery here.

    #[test]
    fn unique_bare_builtin_returns_qualified() {
        // Vector3 exists only in geometry_msgs — the bare name
        // resolves to the one qualified built-in.
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(
            resolve_and_report(tmp.path(), "Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
    }

    #[test]
    fn qualified_name_passes_through_unchanged() {
        // The silent Qualified branch: already-qualified names (the
        // shape `parse_port_args` canonicalizes to) round-trip.
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(
            resolve_and_report(tmp.path(), "geometry_msgs/Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
    }

    #[test]
    fn ambiguous_bare_name_propagates_validation_err() {
        // Pose2D lives in geometry_msgs AND vision_msgs — the
        // resolver's Validation error propagates untouched (the
        // standard `Error:` handler renders it).
        let tmp = tempfile::tempdir().unwrap();
        let err = resolve_and_report(tmp.path(), "Pose2D").unwrap_err();
        assert!(
            matches!(err, cerulion_cli_engine::error::CliError::Validation(_)),
            "expected Validation error, got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("geometry_msgs/Pose2D") && msg.contains("vision_msgs/Pose2D"),
            "error must name both candidate packages: {msg}"
        );
    }

    #[test]
    fn unknown_bare_name_propagates_schema_not_found() {
        let tmp = tempfile::tempdir().unwrap();
        let err = resolve_and_report(tmp.path(), "Vectorr3").unwrap_err();
        assert!(
            matches!(
                err,
                cerulion_cli_engine::error::CliError::SchemaNotFound { .. }
            ),
            "expected SchemaNotFound error, got {err:?}"
        );
    }
}

#[cfg(test)]
mod map_levels_partition_result_tests {
    use super::*;

    #[test]
    fn none_partition_error_maps_to_ok() {
        // A valid or absent `process_groups:` partition
        // (`partition_error == None`) exits 0 — inspect-then-fail's success arm.
        assert!(
            map_levels_partition_result(None).is_ok(),
            "a valid/absent partition must map to Ok (exit 0)"
        );
    }

    #[test]
    fn some_partition_error_maps_to_validation_carrying_diagnostic() {
        // An INVALID partition maps to a nonzero `CliError::Validation` carrying
        // the "not spawner-consumable" preface + the partition diagnostic verbatim
        // (the report was already printed at the call site — this is the exit
        // code only). Oracle string is HAND-WRITTEN, not derived from the fn.
        let diagnostic =
            "process group 'X': node 'n2' ... foreign bridge node 'n1' breaks its chain"
                .to_string();
        let err = map_levels_partition_result(Some(diagnostic))
            .expect_err("an invalid partition must map to a nonzero error");
        match err {
            cerulion_cli_engine::error::CliError::Validation(msg) => {
                assert!(
                    msg.starts_with("process_groups partition is not spawner-consumable: "),
                    "the spawner-consumability preface must be present; got: {msg}"
                );
                assert!(
                    msg.contains("foreign bridge node 'n1'"),
                    "the partition diagnostic (naming the bridge) must pass through verbatim; \
                     got: {msg}"
                );
            }
            other => panic!("expected CliError::Validation, got {other:?}"),
        }
    }
}

#[cfg(test)]
mod parse_policy_spec_tests {
    use super::*;
    use cerulion_core::MacroPolicy;

    #[test]
    fn period_ms_bare_form_errors() {
        // Bare `--policy period_ms` (no value) is rejected: a
        // value-less time-based policy has no sensible default,
        // and silently picking 100ms would mask user mistakes
        // (e.g. forgetting the `=N` suffix). The user must pass
        // a concrete value, e.g. `--policy period_ms=100`.
        let err = parse_policy_spec("period_ms").unwrap_err();
        assert!(err.to_string().contains("requires a value"), "got: {err}");
    }

    #[test]
    fn period_ms_with_value_parses_normally() {
        let result = parse_policy_spec("period_ms=33").unwrap();
        assert_eq!(result, MacroPolicy::Period { period_ms: 33 });
    }

    #[test]
    fn deadline_ms_policy_removed() {
        // There is no node-level `deadline_ms` trigger
        // (it decomposes into a `Data` trigger + the per-input
        // `expect_within_ms` QoS knob). The CLI does not accept it as
        // a `--policy` spec — both the `=N` and bare forms fall through
        // to the unknown-policy error.
        for spec in ["deadline_ms=100", "deadline_ms"] {
            let err = parse_policy_spec(spec).unwrap_err();
            assert!(
                err.to_string()
                    .contains("unknown policy spec `deadline_ms`"),
                "got: {err}"
            );
        }
    }

    #[test]
    fn sync_window_ms_bare_form_errors() {
        let err = parse_policy_spec("sync_window_ms").unwrap_err();
        assert!(err.to_string().contains("requires a value"), "got: {err}");
    }

    #[test]
    fn period_ms_zero_errors() {
        // `--policy period_ms=0` would spin-loop the scheduler.
        // Reject at the CLI surface so users get an immediate
        // clear error rather than a downstream `cargo build`
        // failure from `validate_policy_bounds`.
        let err = parse_policy_spec("period_ms=0").unwrap_err();
        assert!(err.to_string().contains("must be > 0"), "got: {err}");
    }

    #[test]
    fn sync_window_ms_zero_errors() {
        let err = parse_policy_spec("sync_window_ms=0").unwrap_err();
        assert!(err.to_string().contains("must be > 0"), "got: {err}");
    }

    #[test]
    fn default_keyword_is_rejected() {
        // `--policy default` is NOT an alias for
        // `period_ms=100`: that would be misleading — the real
        // "default" behavior depends on input count (see
        // `resolve_create_policy`). There is no such keyword;
        // users wanting a 100ms period pass
        // `--policy period_ms=100`.
        let err = parse_policy_spec("default").unwrap_err();
        assert!(
            err.to_string().contains("unknown policy spec"),
            "got: {err}"
        );
    }

    #[test]
    fn external_keyword_round_trips() {
        let result = parse_policy_spec("external").unwrap();
        assert_eq!(result, MacroPolicy::External);
    }
}

#[cfg(test)]
mod resolve_create_policy_tests {
    use super::*;
    use cerulion_core::MacroPolicy;

    fn input(name: &str) -> (String, String) {
        ("test_msgs/Foo".to_string(), name.to_string())
    }

    #[test]
    fn zero_inputs_no_policy_errors() {
        // Source-only node with no explicit policy must error —
        // there is no defensible default for a node with no
        // inputs and no time-based trigger.
        let err = resolve_create_policy(None, None, &[]).unwrap_err();
        assert!(err.to_string().contains("source-only nodes"), "got: {err}");
    }

    #[test]
    fn one_input_no_policy_returns_none() {
        // Per the unified defaulting rule: 1+ inputs with no
        // `--policy` returns None (no node-level policy attr is
        // written; the runtime warns + fires on any input).
        // No more silent auto-promotion to `data_trigger=<input>`.
        let inputs = vec![input("data")];
        let result = resolve_create_policy(None, None, &inputs).unwrap();
        assert_eq!(result, None);
    }

    #[test]
    fn two_inputs_no_policy_returns_none() {
        // 2+ inputs without `--policy` already returned None
        // before the unification — the test pins that down.
        let inputs = vec![input("a"), input("b")];
        let result = resolve_create_policy(None, None, &inputs).unwrap();
        assert_eq!(result, None);
    }

    #[test]
    fn explicit_dash_t_still_writes_data_trigger() {
        // An explicit `-T` is an explicit policy declaration, so
        // the policy is threaded through even when `--policy` is
        // omitted. Only the IMPLICIT 1-input auto-promotion went
        // away — `-T` retains its meaning.
        let trigger = input("scan");
        let result = resolve_create_policy(None, Some(&trigger), &[]).unwrap();
        assert_eq!(
            result,
            Some(MacroPolicy::DataTrigger {
                input_name: "scan".to_string(),
            })
        );
    }

    #[test]
    fn explicit_period_ms_unaffected_by_inputs() {
        // A `--policy period_ms=N` survives any number of inputs.
        let explicit = MacroPolicy::Period { period_ms: 50 };
        let inputs = vec![input("a")];
        let result = resolve_create_policy(Some(&explicit), None, &inputs).unwrap();
        assert_eq!(result, Some(MacroPolicy::Period { period_ms: 50 }));
    }
}

#[cfg(test)]
mod dash_t_dash_i_collision_tests {
    use super::*;

    fn port(schema: &str, name: &str) -> (String, String) {
        (schema.to_string(), name.to_string())
    }

    #[test]
    fn no_trigger_no_collision() {
        // No `-T` declared → guard is a no-op regardless of `-i`s.
        let result = check_dash_t_dash_i_name_collision(&[], &[port("sensor_msgs/Image", "image")]);
        assert!(result.is_ok());
    }

    #[test]
    fn trigger_alone_no_collision() {
        // `-T` declared, no `-i` → no possible collision.
        let result = check_dash_t_dash_i_name_collision(&[port("sensor_msgs/Image", "image")], &[]);
        assert!(result.is_ok());
    }

    #[test]
    fn trigger_and_regular_with_different_names_ok() {
        // `-T scan` + `-i image` → different names, accepted.
        let result = check_dash_t_dash_i_name_collision(
            &[port("sensor_msgs/LaserScan", "scan")],
            &[port("sensor_msgs/Image", "image")],
        );
        assert!(result.is_ok());
    }

    #[test]
    fn trigger_and_regular_with_same_name_rejected() {
        // `-T foo` + `-i foo` → name collision, must error.
        let err = check_dash_t_dash_i_name_collision(
            &[port("sensor_msgs/Image", "foo")],
            &[port("sensor_msgs/Imu", "foo")],
        )
        .expect_err("must reject name collision");
        let msg = err.to_string();
        assert!(
            msg.contains("'foo'"),
            "error must name the colliding port; got: {msg}"
        );
        assert!(
            msg.contains("-T") && msg.contains("-i"),
            "error must mention both flags; got: {msg}"
        );
    }

    #[test]
    fn trigger_collides_with_one_of_many_regulars() {
        // `-T foo` + `-i bar -i foo -i baz` (the CLI does not allow
        // multiple `-i`, but the helper is shape-agnostic and
        // should still catch a collision in a multi-element slice).
        let err = check_dash_t_dash_i_name_collision(
            &[port("sensor_msgs/Image", "foo")],
            &[
                port("sensor_msgs/Image", "bar"),
                port("sensor_msgs/Imu", "foo"),
                port("std_msgs/String", "baz"),
            ],
        )
        .expect_err("must reject collision even when not first");
        assert!(err.to_string().contains("'foo'"));
    }
}

#[cfg(test)]
mod clap_parse_tests {
    //! End-to-end clap parsing tests for the current CLI surface.
    //!
    //! These tests exercise the actual `clap::Parser::try_parse_from`
    //! path — proving the `num_args = 2` invariants documented on
    //! `-T`, `-i`, `-o` hold against the binary's argv-style input.
    //!
    //! They do NOT actually create files, so they're fast and don't
    //! touch the filesystem. They only verify clap accepts/rejects
    //! the args and that the parsed shape (Vec lengths) is what the
    //! handler expects.
    use super::cli::{Cli, Commands, GraphAction, NodeAction};
    use clap::Parser;

    fn try_parse(argv: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(argv)
    }

    #[test]
    fn graph_validate_release_flag_parses() {
        let cli = try_parse(&["cerulion", "graph", "validate", "perception", "--release"])
            .expect("`graph validate --release` must parse");
        let Commands::Graph {
            action: GraphAction::Validate { name, release },
        } = cli.command
        else {
            panic!("expected GraphAction::Validate");
        };
        assert_eq!(name, "perception");
        assert!(release);
    }

    #[test]
    fn graph_validate_defaults_to_debug_preference() {
        let cli = try_parse(&["cerulion", "graph", "validate", "perception"])
            .expect("`graph validate` without `--release` must parse");
        let Commands::Graph {
            action: GraphAction::Validate { release, .. },
        } = cli.command
        else {
            panic!("expected GraphAction::Validate");
        };
        assert!(!release, "release preference must default to off");
    }

    #[test]
    fn node_run_release_flag_parses() {
        // `node run --release` must carry the release preference so it
        // can thread through to `graph_run`, mirroring
        // `graph run --release`.
        let cli = try_parse(&["cerulion", "node", "run", "camera", "--release"])
            .expect("`node run --release` must parse");
        let Commands::Node {
            action: NodeAction::Run {
                node_type, release, ..
            },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert_eq!(node_type, "camera");
        assert!(release, "`--release` must set the release preference");
    }

    #[test]
    fn node_run_defaults_to_debug_preference() {
        let cli = try_parse(&["cerulion", "node", "run", "camera"])
            .expect("`node run` without `--release` must parse");
        let Commands::Node {
            action: NodeAction::Run { release, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert!(!release, "release preference must default to off");
    }

    #[test]
    fn node_run_no_cpu_dma_lock_flag_parses() {
        // `node run --no-cpu-dma-lock` must carry the opt-out so
        // single-node runs can disable the default-on CPU DMA / C-state
        // lock, mirroring `graph run --no-cpu-dma-lock`.
        let cli = try_parse(&["cerulion", "node", "run", "camera", "--no-cpu-dma-lock"])
            .expect("`node run --no-cpu-dma-lock` must parse");
        let Commands::Node {
            action:
                NodeAction::Run {
                    node_type,
                    no_cpu_dma_lock,
                    ..
                },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert_eq!(node_type, "camera");
        assert!(
            no_cpu_dma_lock,
            "`--no-cpu-dma-lock` must set no_cpu_dma_lock = true"
        );
    }

    #[test]
    fn node_run_no_cpu_dma_lock_defaults_false() {
        // Bare `node run` leaves the lock default-on (flag false).
        let cli = try_parse(&["cerulion", "node", "run", "camera"])
            .expect("`node run` without `--no-cpu-dma-lock` must parse");
        let Commands::Node {
            action: NodeAction::Run {
                no_cpu_dma_lock, ..
            },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert!(
            !no_cpu_dma_lock,
            "no_cpu_dma_lock must default to off (lock on for the live path)"
        );
    }

    #[test]
    fn node_run_no_monitor_wait_flag_parses() {
        // `node run --no-monitor-wait` must carry the opt-out so
        // single-node runs can disable the default-on CPU monitor-wait park,
        // mirroring `graph run --no-monitor-wait`.
        let cli = try_parse(&["cerulion", "node", "run", "camera", "--no-monitor-wait"])
            .expect("`node run --no-monitor-wait` must parse");
        let Commands::Node {
            action:
                NodeAction::Run {
                    node_type,
                    no_monitor_wait,
                    ..
                },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert_eq!(node_type, "camera");
        assert!(
            no_monitor_wait,
            "`--no-monitor-wait` must set no_monitor_wait = true"
        );
    }

    #[test]
    fn node_run_no_monitor_wait_defaults_false() {
        // Bare `node run` leaves the park default-on (flag false).
        let cli = try_parse(&["cerulion", "node", "run", "camera"])
            .expect("`node run` without `--no-monitor-wait` must parse");
        let Commands::Node {
            action: NodeAction::Run {
                no_monitor_wait, ..
            },
        } = cli.command
        else {
            panic!("expected NodeAction::Run");
        };
        assert!(
            !no_monitor_wait,
            "no_monitor_wait must default to off (park on for the live path)"
        );
    }

    #[test]
    fn dash_t_with_schema_and_name_parses() {
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "detector",
            "-T",
            "sensor_msgs/Image",
            "image",
        ])
        .expect("`-T SCHEMA NAME` must parse");
        let Commands::Node { action } = cli.command else {
            panic!("expected Node command");
        };
        let NodeAction::Create { trigger_input, .. } = action else {
            panic!("expected Create action");
        };
        assert_eq!(trigger_input, vec!["sensor_msgs/Image", "image"]);
    }

    #[test]
    fn dash_t_with_only_schema_is_rejected_by_clap() {
        // num_args = 2 means clap rejects single-arg invocations
        // BEFORE the handler runs. The earlier `num_args = 1..=2`
        // ambiguity is gone.
        // `Cli` doesn't impl `Debug`, so `.expect_err()` doesn't
        // compile — match on the Result directly.
        let result = try_parse(&[
            "cerulion",
            "node",
            "create",
            "detector",
            "-T",
            "sensor_msgs/Image",
        ]);
        let err = match result {
            Ok(_) => panic!("`-T SCHEMA` alone must be rejected by clap"),
            Err(e) => e,
        };
        // clap's error message for missing arg counts mentions the value name.
        let msg = err.to_string();
        assert!(
            msg.contains("NAME") || msg.contains("required") || msg.contains("argument"),
            "expected clap to flag the missing `NAME` arg; got: {msg}"
        );
    }

    #[test]
    fn dash_i_with_schema_and_name_parses() {
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "camera",
            "-i",
            "sensor_msgs/LaserScan",
            "scan",
        ])
        .expect("`-i SCHEMA NAME` must parse");
        let Commands::Node {
            action: NodeAction::Create { input, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Create");
        };
        assert_eq!(input, vec!["sensor_msgs/LaserScan", "scan"]);
    }

    #[test]
    fn multiple_dash_t_flat_vec_caught_by_length_guard() {
        // `-T A nameA -T B nameB` flat-Vecs to len=4. The CLI
        // handler's length guard (`trigger_input.len() > 2`) catches
        // this; here we just verify clap parses it into the flat-Vec
        // shape we rely on.
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "fusion",
            "-T",
            "sensor_msgs/Image",
            "image",
            "-T",
            "sensor_msgs/Imu",
            "imu",
        ])
        .expect(
            "clap must accept the flat-Vec accumulation; length guard runs later in the handler",
        );
        let Commands::Node {
            action: NodeAction::Create { trigger_input, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Create");
        };
        assert_eq!(
            trigger_input.len(),
            4,
            "expected 4 args from two `-T` invocations"
        );
        assert_eq!(
            trigger_input,
            vec!["sensor_msgs/Image", "image", "sensor_msgs/Imu", "imu"]
        );
    }

    #[test]
    fn dash_o_with_schema_and_name_parses() {
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "camera",
            "-o",
            "sensor_msgs/Image",
            "image",
        ])
        .expect("`-o SCHEMA NAME` must parse");
        let Commands::Node {
            action: NodeAction::Create { output, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Create");
        };
        assert_eq!(output, vec!["sensor_msgs/Image", "image"]);
    }

    #[test]
    fn raw_ffi_flag_parses() {
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "camera",
            "--raw-ffi",
            "-o",
            "sensor_msgs/Image",
            "image",
        ])
        .expect("`--raw-ffi` must parse alongside `-o`");
        let Commands::Node {
            action: NodeAction::Create { raw_ffi, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Create");
        };
        assert!(raw_ffi);
    }

    #[test]
    fn policy_period_ms_parses() {
        let cli = try_parse(&[
            "cerulion",
            "node",
            "create",
            "camera",
            "--policy",
            "period_ms=33",
            "-o",
            "sensor_msgs/Image",
            "image",
        ])
        .expect("`--policy period_ms=33` must parse");
        let Commands::Node {
            action: NodeAction::Create { policy, .. },
        } = cli.command
        else {
            panic!("expected NodeAction::Create");
        };
        assert_eq!(policy.as_deref(), Some("period_ms=33"));
    }
}

#[cfg(test)]
mod bridge_config_override_tests {
    use super::*;

    /// The consented run uses the config `ros2 attach`
    /// just wrote — the override note fires ONLY when a DIFFERENT pre-existing
    /// DDS_BRIDGE_CONFIG is being overridden, naming BOTH paths. (An
    /// `is_none()` guard would let the pre-existing value silently WIN by
    /// skipping the set entirely; the call site sets the var unconditionally
    /// and this helper pins the note semantics.)
    #[test]
    fn override_note_fires_only_for_a_different_preexisting_value() {
        let generated = Path::new("/ws/graphs/attach.bridge.yaml");

        // Unset ⇒ silent (nothing is being overridden).
        assert_eq!(bridge_config_override_note(None, generated), None);

        // Already pointing at the generated file ⇒ silent (idempotent re-run).
        assert_eq!(
            bridge_config_override_note(
                Some(std::ffi::OsStr::new("/ws/graphs/attach.bridge.yaml")),
                generated
            ),
            None
        );

        // A DIFFERENT pre-existing value ⇒ loud note naming BOTH paths.
        let note = bridge_config_override_note(
            Some(std::ffi::OsStr::new("/other/go2.bridge.yaml")),
            generated,
        )
        .expect("a different pre-existing value must produce the note");
        assert!(note.contains("/other/go2.bridge.yaml"), "{note}");
        assert!(note.contains("/ws/graphs/attach.bridge.yaml"), "{note}");
        assert!(note.contains("overriding for this run"), "{note}");
    }
}

/// The clap-variant → `PlayFlags` mapping, pinned
/// field by field.
///
/// The chain from argv to the engine was pinned at every link EXCEPT this one:
/// clap → variant in `cli.rs`, `PlayFlags` → `ResimOptions` and `ResimOptions`
/// → `ReplayOptions` in `resim_cmd`, and `--strict-state`'s own semantics twice
/// in `replay_engine_test`. `play_flags_of` sat between the two chains with no
/// test on it.
///
/// That gap is not uniform across the flags. Most of them are observable from
/// the CLI e2e — drop `--verify` and the exit contract changes, drop
/// `--tolerance` and a tolerated bag fails. `--strict-state` is the exception:
/// it is documented INERT on a bag beginning at step 0, which is the only bag
/// shape `replay_cli_test` builds, so `strict_state: false` written here would
/// have passed the whole suite while discarding an operator's explicit refusal
/// request. A CLI-level behavioural arm would need a mid-run checkpoint bag;
/// this pins the same link for the cost of a struct literal.
#[cfg(test)]
mod resim_flag_mapping_tests {
    use super::*;
    use cli::BagAction;
    use std::path::PathBuf;

    fn play(strict_state: bool, verify: bool) -> BagAction {
        BagAction::Play {
            bag: PathBuf::from("/tmp/b.mcap"),
            resim: Some("all".into()),
            verify,
            rate: None,
            repeat: false,
            topics: vec![],
            duration: Some(11.0),
            start_offset: Some(4.5),
            report: Some(PathBuf::from("/tmp/r.json")),
            tolerance: Some(PathBuf::from("/tmp/t.yaml")),
            strict_state,
        }
    }

    #[test]
    fn every_flag_reaches_play_flags() {
        let f = play_flags_of(&play(true, true)).expect("a `bag play` maps");
        assert_eq!(f.resim.as_deref(), Some("all"));
        assert!(f.verify);
        assert_eq!(f.duration, Some(11.0));
        assert_eq!(f.start_offset, Some(4.5));
        assert_eq!(
            f.report.as_deref(),
            Some(std::path::Path::new("/tmp/r.json"))
        );
        assert_eq!(
            f.tolerance.as_deref(),
            Some(std::path::Path::new("/tmp/t.yaml"))
        );
        // THE one this exists for: inert on a step-0 bag, so no CLI e2e in this
        // repo can observe it being dropped.
        assert!(
            f.strict_state,
            "`--strict-state` must survive the variant -> PlayFlags mapping"
        );
    }

    #[test]
    fn a_flag_nobody_gave_is_not_invented() {
        // Anti-tautology: without this, a mapping that hardcoded `true` for
        // every bool would pass the arm above.
        let f = play_flags_of(&play(false, false)).expect("a `bag play` maps");
        assert!(!f.strict_state);
        assert!(!f.verify);
        assert!(!f.repeat);
        assert!(f.rate.is_none());
        assert!(f.topics.is_empty());
        // The two bag-time bounds are `Option`s for the same
        // reason `--rate` is — the refusal seam has to tell "asked for nothing"
        // from "asked for the default".
    }

    #[test]
    fn a_non_play_action_carries_no_resim_flags() {
        assert!(play_flags_of(&BagAction::Info {
            bag: PathBuf::from("/tmp/b.mcap"),
        })
        .is_none());
    }
}

/// WHICH `bag play` invocations belong to the resim
/// surface — the routing decision, pinned per flag.
///
/// It earns its own module because the predicate is a CLASSIFIER over six
/// flags and the e2e can only reach it through one invocation at a time. The
/// behavioural half lives in
/// `replay_cli_test::a_playback_start_offset_reaches_the_player_not_the_resim_surface`.
#[cfg(test)]
mod is_resim_family_tests {
    use super::*;
    use cli::BagAction;
    use std::path::PathBuf;

    /// The flags under test, defaulted to "nobody asked".
    ///
    /// A struct rather than a long argument list because `BagAction::Play` is an
    /// ENUM VARIANT, and functional-update (`..bare()`) is a struct-only form —
    /// so a per-case builder needs somewhere to default from.
    #[derive(Default)]
    struct Flags {
        resim: Option<String>,
        verify: bool,
        duration: Option<f64>,
        start_offset: Option<f64>,
        report: Option<PathBuf>,
        tolerance: Option<PathBuf>,
        strict_state: bool,
    }

    fn play(f: Flags) -> BagAction {
        BagAction::Play {
            bag: PathBuf::from("/tmp/b.mcap"),
            resim: f.resim,
            verify: f.verify,
            rate: None,
            repeat: false,
            topics: vec![],
            duration: f.duration,
            start_offset: f.start_offset,
            report: f.report,
            tolerance: f.tolerance,
            strict_state: f.strict_state,
        }
    }

    /// The DEFECT this test pins: `--start-offset` is PLAYBACK-only,
    /// so an invocation carrying it and no `--resim` is ordinary playback and
    /// must reach `run_bag`. Listing it in the family sends it to
    /// `run_play_resim`, which refuses a perfectly legal command with "no
    /// `--resim` given" and exit 2 — a seek the engine implements
    /// (`bag_cmd`'s `start_offset_ns` window) and the CLI then cannot reach.
    ///
    /// `--duration` is its BOTH-HALVES sibling and is asserted beside it, so
    /// the two bag-time bounds cannot drift apart.
    #[test]
    fn a_playback_only_bound_is_not_a_resim_invocation() {
        for (label, action) in [
            (
                "--start-offset",
                play(Flags {
                    start_offset: Some(1.5),
                    ..Flags::default()
                }),
            ),
            (
                "--duration",
                play(Flags {
                    duration: Some(2.5),
                    ..Flags::default()
                }),
            ),
            (
                "both bounds",
                play(Flags {
                    start_offset: Some(1.5),
                    duration: Some(2.5),
                    ..Flags::default()
                }),
            ),
        ] {
            assert!(
                !is_resim_family(&action),
                "`bag play <bag> {label}` is PLAYBACK and must reach `run_bag`"
            );
        }
    }

    /// The same flag WITH `--resim` still routes here — the misuse the removed
    /// disjunct was reaching for is covered by `--resim` itself, which is the
    /// whole reason the disjunct was redundant as well as harmful.
    #[test]
    fn a_playback_only_bound_under_resim_still_routes_to_the_resim_surface() {
        assert!(is_resim_family(&play(Flags {
            resim: Some("all".into()),
            start_offset: Some(1.5),
            ..Flags::default()
        })));
    }

    /// ANTI-TAUTOLOGY: the predicate is not simply `false`. Every RESIM-ONLY
    /// flag still claims the invocation on its own, so it is answered with
    /// `EXIT_USAGE` rather than the generic failure code 1.
    #[test]
    fn every_resim_only_flag_claims_the_invocation_on_its_own() {
        for (label, action) in [
            (
                "--resim",
                play(Flags {
                    resim: Some("all".into()),
                    ..Flags::default()
                }),
            ),
            (
                "--verify",
                play(Flags {
                    verify: true,
                    ..Flags::default()
                }),
            ),
            (
                "--report",
                play(Flags {
                    report: Some(PathBuf::from("/tmp/r.json")),
                    ..Flags::default()
                }),
            ),
            (
                "--tolerance",
                play(Flags {
                    tolerance: Some(PathBuf::from("/tmp/t.yaml")),
                    ..Flags::default()
                }),
            ),
            (
                "--strict-state",
                play(Flags {
                    strict_state: true,
                    ..Flags::default()
                }),
            ),
        ] {
            assert!(
                is_resim_family(&action),
                "`{label}` is resim-only and must answer with EXIT_USAGE"
            );
        }

        // …and a bare `bag play` is neither.
        assert!(!is_resim_family(&play(Flags::default())));
        // `bag info` / `bag record` carry no resim flags at all.
        assert!(!is_resim_family(&BagAction::Info {
            bag: PathBuf::from("/tmp/b.mcap"),
        }));
    }
}
