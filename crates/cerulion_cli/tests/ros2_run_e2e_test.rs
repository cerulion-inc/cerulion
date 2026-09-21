// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion ros2 run` / `ros2 launch` over the REAL binary: exec()
//! transparency, VERBATIM forwarding, the heap-hook preload matrix, and the
//! typed exit contract — with NO ROS 2 installed.
//!
//! A fixture `ros2` shell script is put FIRST on the child's `PATH` (both
//! verbs resolve `ros2` through `PATH`, so no product seam is needed to
//! substitute it), a fixture lib dir carries an empty `librmw_cerulion.so`
//! (the verbs only check presence), and `HOME` points at a tempdir so the
//! ament-prefix staging never touches the real `~/.cerulion`. Every env
//! override is per-CHILD (`Command::env`) — no process-global mutation, so
//! the file is parallel-safe and needs no `#[serial]`.
//!
//! Two headline oracles: exit-code INHERITANCE (the fixture exits 42 and
//! `cerulion` exits 42, which only an `exec()` gives for free) on EACH verb,
//! and the LEADING-hyphen forwarding pin — `cerulion ros2 run --prefix …`
//! reaches the fixture as `run --prefix …`, which only `main`'s raw-argv
//! intercept (never clap) can guarantee. A third pins the launcher
//! refusal: `--adopt-take` is refused by both verbs (exit 69,
//! nothing exec'd) because the `ros2` Python CLI spawns the node as a
//! grandchild with `close_fds=True`, so the descriptor-bound heap hook
//! cannot reach it.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::Command;

/// The sandbox: a fake `ros2` on its own PATH dir, a lib dir with the rmw
/// cdylib fixture, a HOME for staging, and the report file the fake writes.
struct Sandbox {
    root: tempfile::TempDir,
    bin_dir: PathBuf,
    lib_dir: PathBuf,
    home: PathBuf,
    report: PathBuf,
}

const FAKE_ROS2: &str = r#"#!/bin/sh
{
  echo "argv=$*"
  echo "rmw=$RMW_IMPLEMENTATION"
  echo "ament=$AMENT_PREFIX_PATH"
  echo "ldlp=$LD_LIBRARY_PATH"
  echo "preload=${LD_PRELOAD:-<unset>}"
} > "$CER_TEST_REPORT"
exit "${CER_TEST_EXIT:-0}"
"#;

fn write_executable(path: &Path, body: &str) {
    use std::os::unix::fs::PermissionsExt;
    std::fs::write(path, body).expect("write script");
    let mut perms = std::fs::metadata(path).expect("meta").permissions();
    perms.set_mode(0o755);
    std::fs::set_permissions(path, perms).expect("chmod");
}

impl Sandbox {
    fn new() -> Self {
        let root = tempfile::tempdir().expect("tempdir");
        let bin_dir = root.path().join("bin");
        let lib_dir = root.path().join("libs");
        let home = root.path().join("home");
        for d in [&bin_dir, &lib_dir, &home] {
            std::fs::create_dir_all(d).expect("mkdir");
        }
        write_executable(&bin_dir.join("ros2"), FAKE_ROS2);
        std::fs::write(lib_dir.join("librmw_cerulion.so"), b"fixture").expect("write rmw");
        let report = root.path().join("report.txt");
        Sandbox {
            root,
            bin_dir,
            lib_dir,
            home,
            report,
        }
    }

    /// `cerulion <argv...>` with the sandbox env staged onto the CHILD only.
    /// Ambient vars the oracles depend on are scrubbed so the assertions are
    /// exact regardless of the developer's shell.
    fn command(&self, argv: &[&str]) -> Command {
        let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
        cmd.args(argv);
        cmd.env("PATH", &self.bin_dir)
            .env("CERULION_LIB_DIR", &self.lib_dir)
            .env("HOME", &self.home)
            .env("CER_TEST_REPORT", &self.report)
            .env("CERULION_NETWORK", "off")
            .env_remove("CERULION_ROS2_PRELOAD")
            // An exported arming value turns any flagless run into the
            // exit-2 env-conflict refusal (`stage_base_child_env`'s
            // `AdoptTakeGate::NotRun` arm), which would flip this file's
            // control arms on a developer machine.
            .env_remove("CERULION_RMW_ADOPT_TAKE")
            // The login gate is on by default. This file is not about the gate,
            // so it pins the value this repository's own runs carry rather than
            // inheriting it: a test binary run directly, outside cargo, would
            // otherwise meet a device-code prompt instead of the exit codes
            // asserted below.
            .env("CERULION_LOGIN_GATE", "off")
            .env_remove("LD_PRELOAD")
            .env_remove("LD_LIBRARY_PATH")
            .env_remove("AMENT_PREFIX_PATH")
            .env_remove("RUST_LOG");
        cmd
    }

    fn report_lines(&self) -> Vec<String> {
        std::fs::read_to_string(&self.report)
            .expect("the fake ros2 must have written its report")
            .lines()
            .map(str::to_string)
            .collect()
    }
}

fn line<'a>(lines: &'a [String], key: &str) -> &'a str {
    let prefix = format!("{key}=");
    lines
        .iter()
        .find(|l| l.starts_with(&prefix))
        .map(|l| &l[prefix.len()..])
        .unwrap_or_else(|| panic!("report has no `{key}=` line: {lines:?}"))
}

/// HEADLINE (`ros2 run`): the fake ros2's exit code (42) comes back
/// UNCHANGED (exec() inheritance), it received `run <args...>` verbatim, and
/// it saw the staged transport env — `rmw_cerulion` selected,
/// `LD_LIBRARY_PATH` = the lib dir, `AMENT_PREFIX_PATH` = a staging prefix
/// under `$HOME/.cerulion/ros2/` whose `lib/librmw_cerulion.so` really
/// symlinks the fixture cdylib, and NO `LD_PRELOAD` (no hook shipped).
#[test]
fn run_verb_inherits_exit_code_and_stages_transport_env() {
    let sb = Sandbox::new();
    let status = sb
        .command(&["ros2", "run", "demo_nodes_cpp", "talker", "--ros-args"])
        .env("CER_TEST_EXIT", "42")
        .status()
        .expect("spawn cerulion");
    assert_eq!(
        status.code(),
        Some(42),
        "exit code must be inherited through exec()"
    );

    let lines = sb.report_lines();
    assert_eq!(line(&lines, "argv"), "run demo_nodes_cpp talker --ros-args");
    assert_eq!(line(&lines, "rmw"), "rmw_cerulion");
    assert_eq!(line(&lines, "ldlp"), sb.lib_dir.display().to_string());
    assert_eq!(line(&lines, "preload"), "<unset>");

    let ament = line(&lines, "ament");
    let staging_root = sb.home.join(".cerulion").join("ros2");
    assert!(
        Path::new(ament).starts_with(&staging_root),
        "ament prefix must be staged under $HOME/.cerulion/ros2 — got {ament}"
    );
    let link = Path::new(ament).join("lib").join("librmw_cerulion.so");
    assert_eq!(
        std::fs::read_link(&link).expect("staged prefix must symlink the cdylib"),
        sb.lib_dir.join("librmw_cerulion.so")
    );
    assert!(sb.root.path().exists());
}

/// HEADLINE (`ros2 launch`): same inheritance + staging, `launch` token
/// first, launch args verbatim.
#[test]
fn launch_verb_inherits_exit_code_and_forwards_args() {
    let sb = Sandbox::new();
    let status = sb
        .command(&["ros2", "launch", "demo.launch.py", "use_rviz:=false"])
        .env("CER_TEST_EXIT", "42")
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(42));
    let lines = sb.report_lines();
    assert_eq!(
        line(&lines, "argv"),
        "launch demo.launch.py use_rviz:=false"
    );
    assert_eq!(line(&lines, "rmw"), "rmw_cerulion");
}

/// THE forwarding pin: a LEADING hyphenated token after the verb reaches
/// `ros2` untouched on BOTH verbs — `--prefix` and `-s` are ros2's flags,
/// and only the raw-argv intercept (clap never sees these argv) can forward
/// them by construction.
#[test]
fn leading_hyphenated_tokens_are_forwarded_verbatim() {
    let sb = Sandbox::new();
    let status = sb
        .command(&[
            "ros2",
            "run",
            "--prefix",
            "xterm -e",
            "demo_nodes_cpp",
            "talker",
        ])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(
        line(&sb.report_lines(), "argv"),
        "run --prefix xterm -e demo_nodes_cpp talker"
    );

    let status = sb
        .command(&["ros2", "launch", "-s", "pkg", "demo.launch.py"])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(
        line(&sb.report_lines(), "argv"),
        "launch -s pkg demo.launch.py"
    );
}

/// The global `-v`/`--verbose` prefix is Cerulion's and is skipped by the
/// intercept — everything after the verb still forwards verbatim.
#[test]
fn verbose_prefix_still_intercepts_and_forwards() {
    let sb = Sandbox::new();
    let status = sb
        .command(&["-v", "ros2", "run", "pkg", "exe"])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(line(&sb.report_lines(), "argv"), "run pkg exe");
}

/// AUTO-INJECT: `libcerulion_heaphook.so` present beside the rmw lib is
/// prepended to the child's `LD_PRELOAD` with nothing asked for; the
/// `CERULION_ROS2_PRELOAD=off` kill switch suppresses it; an explicit value
/// STACKS `.bashrc`-style — user's `.so` first, the hook kept, the ambient
/// `LD_PRELOAD` kept (order pinned).
#[test]
fn heaphook_auto_injects_and_the_kill_switch_suppresses() {
    let sb = Sandbox::new();
    let hook = sb.lib_dir.join("libcerulion_heaphook.so");
    std::fs::write(&hook, b"fixture").expect("write hook");

    let status = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(
        line(&sb.report_lines(), "preload"),
        hook.display().to_string(),
        "the hook must be auto-injected when present"
    );

    let status = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .env("CERULION_ROS2_PRELOAD", "off")
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(
        line(&sb.report_lines(), "preload"),
        "<unset>",
        "the kill switch must suppress all preload injection"
    );

    let explicit = sb.lib_dir.join("libcerulion_custom.so");
    std::fs::write(&explicit, b"fixture").expect("write explicit");
    let status = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .env("CERULION_ROS2_PRELOAD", &explicit)
        .env("LD_PRELOAD", "/existing.so")
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert_eq!(
        line(&sb.report_lines(), "preload"),
        format!("{}:{}:/existing.so", explicit.display(), hook.display()),
        "an explicit value stacks: user's .so first, then the hook, then the ambient LD_PRELOAD"
    );
}

/// A missing `librmw_cerulion.so` is exit 69 with the remediation on stderr,
/// and the fake ros2 is NEVER exec'd (no report written) — the launcher
/// never falls through to the stock rmw. Same class on both verbs.
///
/// The remedy is the HOST's: the shipped binary must print exactly the
/// engine's message for this OS (a Linux user is sent to reinstall a
/// package, a macOS user is told the verbs need a Linux machine), and never
/// a cargo command, which an installed user cannot run.
#[test]
fn missing_rmw_lib_is_exit_69_and_never_execs() {
    let sb = Sandbox::new();
    let rmw_lib = sb.lib_dir.join("librmw_cerulion.so");
    std::fs::remove_file(&rmw_lib).expect("remove rmw");
    let expected =
        cerulion_cli_engine::ros2_cmd::missing_rmw_lib_message(std::env::consts::OS, &rmw_lib);
    for verb in ["run", "launch"] {
        let out = sb
            .command(&["ros2", verb, "anything"])
            .output()
            .expect("spawn cerulion");
        assert_eq!(out.status.code(), Some(69), "verb {verb}");
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(stderr.contains("librmw_cerulion.so"), "stderr: {stderr}");
        assert!(
            stderr.contains(&expected),
            "the shipped binary must print the host's remedy verbatim: {stderr}"
        );
        assert!(
            !stderr.contains("cargo build"),
            "an installed user is never sent to cargo: {stderr}"
        );
        assert!(
            !sb.report.exists(),
            "ros2 must not be exec'd without the rmw lib"
        );
    }
}

/// Pinned over the real binary: `--adopt-take` is
/// REFUSED by BOTH verbs — exit 69, the cause and the working alternative
/// both on stderr, and `ros2` NEVER exec'd (no report file). `ros2` is a
/// Python CLI that spawns the node as a further subprocess with
/// `close_fds=True`, so the heap hook this launcher hands over as an
/// inherited descriptor cannot reach the node; before that decision the
/// launcher reported success and every take was served by copy.
///
/// The hook IS staged in this sandbox (auto-injection puts it in the
/// child's `LD_PRELOAD` — the CONTROL below proves that on the same
/// fixture), so the refusal cannot be the "nothing staged" one; the control
/// is the identical launch WITHOUT the flag, which execs normally.
///
/// Which assertion is decisive here, stated exactly rather than assumed:
/// this fixture's hook is `b"fixture"`, not a real ELF, so DELETING the
/// refusal does not make these verbs exec — the retained gate refuses too
/// (non-ELF on a Linux/GNU runner, the platform gate on macOS), also with
/// exit 69 and no report. So `Some(69)` and `!report.exists()` both survive
/// deleting the refusal; what fails is the message — `close_fds=True`, `DIRECTLY`
/// and `CERULION_RMW_ADOPT_TAKE=1` belong to that refusal alone. The absence
/// assertions name the sentences the three OTHER refusals own — including
/// the NON-ELF one, which is what a Linux/GNU runner would actually hit
/// here (this fixture's hook is not an ELF).
///
/// A caveat about their pairing, because the house rule asks for a positive
/// over the SAME string and there cannot be one in this file: the decision
/// owns every `--adopt-take` refusal the real binary can now reach, so no
/// invocation of `cerulion` can emit any of those three sentences. They are
/// supporting evidence, not paired negatives. The paired-positive versions
/// live in the engine suite (`ros2_cmd_test.rs` drives the retained gate to
/// PRODUCE the staged-preload sentence in the same body that asserts its
/// absence, and its host-gate arm does the same). Residual: a production
/// reword that updates the engine's constants leaves these literals stale
/// and silently non-discriminating — the token loop above still fails
/// first, so the arm degrades to weaker rather than vacuous.
#[test]
fn adopt_take_is_refused_by_both_verbs_and_never_execs() {
    let sb = Sandbox::new();
    std::fs::write(sb.lib_dir.join("libcerulion_heaphook.so"), b"fixture").expect("write hook");

    for verb in ["run", "launch"] {
        let out = sb
            .command(&["ros2", verb, "--adopt-take", "pkg", "exe"])
            .output()
            .expect("spawn cerulion");
        assert_eq!(out.status.code(), Some(69), "verb {verb} must refuse");
        let stderr = String::from_utf8_lossy(&out.stderr);
        for token in [
            "--adopt-take",
            "close_fds=True",
            "LD_PRELOAD",
            "DIRECTLY",
            "CERULION_RMW_ADOPT_TAKE=1",
            "copy path",
            // The recipe must PREPEND each path var to the sourced value,
            // the way `prepend_path_var` stages it. A recipe that ASSIGNS
            // them is unrunnable in the sourced ROS 2 shell it is printed
            // in — it drops the distro's own ament index and libraries —
            // which is the remedy-that-looks-runnable class this whole
            // refusal exists to kill, so it gets its own pin rather than
            // riding the bare `LD_PRELOAD` token above.
            "${LD_PRELOAD:+:$LD_PRELOAD}",
            "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}",
            "${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}",
        ] {
            assert!(
                stderr.contains(token),
                "verb {verb} refusal must name `{token}`: {stderr}"
            );
        }
        for other in [
            // the nothing-staged refusal's sentence …
            "is not staged into the child's LD_PRELOAD",
            // … the NON-ELF one, which is the contender on a Linux/GNU
            // runner (this sandbox's hook is `b"fixture"`, not an ELF) …
            "no staged preload is the built hook",
            // … and the platform gate's, the contender on this desk.
            "requires a Linux/GNU host",
        ] {
            assert!(
                !stderr.contains(other),
                "verb {verb}: the `--adopt-take` refusal must answer, not `{other}`: {stderr}"
            );
        }
        assert!(
            !sb.report.exists(),
            "verb {verb}: ros2 must never be exec'd behind a flag it cannot honour"
        );
    }

    // CONTROL: the same launch WITHOUT the flag execs, and the hook this
    // fixture staged really is in the child's LD_PRELOAD — so the refusals
    // above come from the decision, not from a missing preload.
    let status = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0), "the copy path is untouched");
    let lines = sb.report_lines();
    assert_eq!(line(&lines, "argv"), "run pkg exe");
    assert_eq!(
        line(&lines, "preload"),
        sb.lib_dir
            .join("libcerulion_heaphook.so")
            .display()
            .to_string(),
        "the hook was staged on this very fixture"
    );
}

/// The ordering pin: the refusal is decided
/// BEFORE anything is staged, so it creates nothing and cannot be pre-empted
/// by a staging failure.
///
/// Calling `stage_ament_prefix` first — it resolves HOME and
/// creates `~/.cerulion/ros2/prefix-<hash>/lib/librmw_cerulion.so` — would leave
/// staging state behind on a refused `--adopt-take`, and on a machine where
/// that staging could not happen the user would get a staging error instead of the
/// decision's host-independent exit 69. The engine owns the order
/// (`plan_ros2_passthrough`).
///
/// The fault is injected at a seam THE ENVIRONMENT CANNOT BYPASS: `HOME`
/// points INSIDE a regular file, so `create_dir_all` fails with ENOTDIR for
/// every user, root included. (A read-only directory would have been the
/// obvious choice and is exactly the guard that goes vacuous where this
/// ships — the ros2-bench image runs as root.)
///
/// FOUR legs, in this order, because a panic ends the test and each
/// run can only prove the leg it reaches. (1) The everyday shape: a
/// perfectly WRITABLE home, where a stage-then-refuse order would create
/// `~/.cerulion/ros2/prefix-<hash>/lib/librmw_cerulion.so` before refusing —
/// "creates nothing" is a claim about a healthy machine too. (2) The
/// hostile shape: a home that CANNOT be staged, where a stage-then-refuse order
/// would return a staging error instead of exit 69. (3) The anti-tautology for
/// (2): with that same blocked home the FLAGLESS run must FAIL at staging,
/// so (2) is a claim about ORDER and not about a fixture that never stages.
/// (4) The unblocked control: staging really does happen on the copy path,
/// down to the rmw symlink, so none of this is "staging never runs".
///
/// A stage-then-refuse order was tested against BOTH orderings of legs (1)
/// and (2) and fails each: leg (1) at `a refused run must leave NO staging
/// state behind on a writable home`, and leg (2) at its own assertion that
/// the refusal answers even when staging cannot run.
#[test]
fn the_refusal_is_decided_before_anything_is_staged() {
    let sb = Sandbox::new();

    // The other half: with a perfectly writable home a
    // stage-then-refuse order would create `~/.cerulion/ros2/prefix-<hash>/lib/
    // librmw_cerulion.so` before refusing. "Creates nothing" is a claim
    // about a healthy machine too, not only about one that cannot stage.
    let staged_root = sb.home.join(".cerulion").join("ros2");
    assert!(
        !staged_root.exists(),
        "precondition: this sandbox has not staged yet"
    );
    let out = sb
        .command(&["ros2", "run", "--adopt-take", "pkg", "exe"])
        .output()
        .expect("spawn cerulion");
    assert_eq!(out.status.code(), Some(69));
    assert!(
        !staged_root.exists(),
        "a refused run must leave NO staging state behind on a writable home"
    );

    // A regular file, used as a DIRECTORY component of HOME.
    let blocker = sb.root.path().join("not-a-dir");
    std::fs::write(&blocker, b"regular file").expect("write blocker");
    let blocked_home = blocker.join("home");

    for verb in ["run", "launch"] {
        let out = sb
            .command(&["ros2", verb, "--adopt-take", "pkg", "exe"])
            .env("HOME", &blocked_home)
            .output()
            .expect("spawn cerulion");
        assert_eq!(
            out.status.code(),
            Some(69),
            "verb {verb}: the `--adopt-take` refusal answers even when staging cannot run"
        );
        let stderr = String::from_utf8_lossy(&out.stderr);
        assert!(
            stderr.contains("close_fds=True"),
            "verb {verb}: and it is the `--adopt-take` REFUSAL, not a staging error: {stderr}"
        );
        assert!(
            !stderr.contains("failed to stage the minimal ament prefix")
                && !stderr.contains("cannot resolve a home directory"),
            "verb {verb}: staging must not have been attempted: {stderr}"
        );
        assert!(
            !blocked_home.exists(),
            "verb {verb}: the refusal must create nothing"
        );
        assert!(
            !sb.report.exists(),
            "verb {verb}: and must not exec ros2 either"
        );
    }

    // ANTI-TAUTOLOGY: the same blocked HOME really does stop staging, so
    // "no prefix created" above is a claim about ORDER, not about a fixture
    // that never stages anyway.
    let out = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .env("HOME", &blocked_home)
        .output()
        .expect("spawn cerulion");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_ne!(
        out.status.code(),
        Some(0),
        "the flagless run must FAIL with this HOME, or the blocker is inert: {stderr}"
    );
    assert!(
        stderr.contains("failed to stage the minimal ament prefix")
            || stderr.contains("cannot resolve a home directory"),
        "and it must fail AT STAGING: {stderr}"
    );
    assert!(
        !sb.report.exists(),
        "a run that cannot stage never reaches ros2"
    );

    // CONTROL: with a usable HOME the copy path really does stage, so the
    // refusal is not simply "staging never happens".
    let status = sb
        .command(&["ros2", "run", "pkg", "exe"])
        .status()
        .expect("spawn cerulion");
    assert_eq!(status.code(), Some(0));
    assert!(
        staged_root.exists(),
        "the flagless copy path stages under $HOME/.cerulion/ros2"
    );
    let symlink = std::fs::read_dir(&staged_root)
        .expect("read staged root")
        .filter_map(Result::ok)
        .map(|e| e.path().join("lib").join("librmw_cerulion.so"))
        .find(|p| p.symlink_metadata().is_ok())
        .expect("the staged prefix carries the rmw symlink");
    assert!(
        symlink.symlink_metadata().expect("lstat").is_symlink(),
        "and it is the symlink the refusal must not have created: {}",
        symlink.display()
    );
}

/// Help is the FIRST place a user goes after the exit-69 refusal, so it must
/// not still describe `--adopt-take` as a flag that works. Nothing asserted
/// one byte of either verb's help before this arm: reverting `cli.rs`'s text
/// to its earlier wording ("sets CERULION_RMW_ADOPT_TAKE=1 for the child
/// … refuses to launch unless the heap hook is staged") left the entire
/// suite green while the CLI documented, in the present tense, a flag that
/// always exits 69.
///
/// It is `cerulion ros2 --help` that carries it, NOT `cerulion ros2 run
/// --help`: `run` and `launch` are raw-argv intercepted before clap, so a
/// `--help` after the verb is FORWARDED to the real `ros2` like every other
/// token (that is the pass-through guarantee this file's other arms pin).
/// Both verbs' text renders in the one subcommand listing, so one invocation
/// covers both.
///
/// The negative is paired with two positives in the same body — the flag is
/// still DOCUMENTED (removing it silently is also wrong: a user who typed it
/// needs to find out WHY it is refused) and the listing says it is refused.
#[test]
fn ros2_help_says_adopt_take_is_refused_not_that_it_arms_the_child() {
    let sb = Sandbox::new();
    let out = sb
        .command(&["ros2", "--help"])
        .output()
        .expect("spawn cerulion");
    assert_eq!(out.status.code(), Some(0));
    let help = String::from_utf8_lossy(&out.stdout);
    assert!(
        help.contains("--adopt-take"),
        "the flag stays documented: {help}"
    );
    // Each phrase is bound to ITS OWN verb's SECTION of the listing, never
    // counted over the whole output: a global count — even one
    // per phrase — is satisfied by moving `launch`'s wording onto `run`,
    // leaving one verb's help stale while both counts still read 1. The
    // rendered listing is `  run  …`, `  launch  …`, `  migrate  …` in that
    // order, so the two entry markers bracket each section.
    let run_at = help
        .find("\n  run ")
        .expect("the listing has a `run` entry");
    let launch_at = help
        .find("\n  launch ")
        .expect("the listing has a `launch` entry");
    let migrate_at = help
        .find("\n  migrate ")
        .expect("the listing has a `migrate` entry");
    assert!(
        run_at < launch_at && launch_at < migrate_at,
        "the entries must render in declaration order for this split to hold: {help}"
    );
    let run_section = &help[run_at..launch_at];
    let launch_section = &help[launch_at..migrate_at];

    assert!(
        run_section.contains("REFUSES (exit 69)"),
        "the `run` section must say it is refused: {run_section}"
    );
    assert!(
        launch_section.contains("refused here for the same reason"),
        "the `launch` section must say it is refused: {launch_section}"
    );
    // The cross-checks that make the split load-bearing: neither verb's
    // wording may be the OTHER's, which is what a move would produce.
    assert!(
        !run_section.contains("refused here for the same reason"),
        "the `run` section must carry its own wording: {run_section}"
    );
    assert!(
        !launch_section.contains("REFUSES (exit 69)"),
        "the `launch` section must carry its own wording: {launch_section}"
    );
    assert!(
        !help.contains("sets CERULION_RMW_ADOPT_TAKE=1 for the child"),
        "the superseded claim must be gone: {help}"
    );
}

/// `ros2` absent from PATH is exit 127 with a stderr line naming the remedy.
#[test]
fn ros2_not_on_path_is_exit_127() {
    let sb = Sandbox::new();
    let empty = sb.root.path().join("empty-path");
    std::fs::create_dir_all(&empty).expect("mkdir");
    let out = sb
        .command(&["ros2", "launch", "demo.launch.py"])
        .env("PATH", &empty)
        .output()
        .expect("spawn cerulion");
    assert_eq!(out.status.code(), Some(127));
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("not found on PATH"), "stderr: {stderr}");
}

/// Bare `cerulion ros2` is clap's usage error — exit 2 (the one usage class
/// the wrappers own; everything argument-shaped belongs to ros2 itself).
#[test]
fn bare_ros2_is_exit_2() {
    let sb = Sandbox::new();
    let out = sb.command(&["ros2"]).output().expect("spawn cerulion");
    assert_eq!(out.status.code(), Some(2));
}
