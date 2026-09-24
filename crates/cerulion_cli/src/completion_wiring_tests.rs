// SPDX-License-Identifier: AGPL-3.0-only
//! Pins for the SHELL-FACING wiring.
//!
//! The engine's `completions_test.rs` covers the value SOURCES; this file
//! covers what nothing else did — that those sources are actually attached to
//! the right arguments, that the path arguments carry a `value_hint`, and that
//! the three `create` arms carry no completer at all. A source can be perfect
//! and the feature still dead if the `add =` attribute is on the wrong arg or
//! missing.
//!
//! Everything here runs IN-PROCESS against the real `Cli::command()` and the
//! real `clap_complete::engine::complete()` — the same function the generated
//! shell script calls back into — so a regression in the attributes fails here
//! rather than only on a live TAB press.
//!
//! **Binary-crate unit tests, not `tests/`**: `cerulion_cli` has no library
//! target, so an integration test could only reach it by spawning the binary.
//! Run with `cargo test -p cerulion_cli --bin cerulion`.
//!
//! The workspace completers resolve their workspace from
//! `std::env::current_dir()` (exactly as the verbs themselves do), so these tests
//! mutate the PROCESS working directory and `HOME`. That is process-global
//! state, hence the file-local mutex and the RAII guards below.

use std::ffi::OsString;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, MutexGuard, OnceLock};

use clap::CommandFactory;
use clap_complete::engine::{ArgValueCandidates, ArgValueCompleter, CompletionCandidate};

use crate::cli::Cli;

/// Serializes the tests that mutate the process cwd / `HOME`.
///
/// libtest runs a binary's tests on parallel threads, and both of those are
/// per-PROCESS, so two tests scaffolding different workspaces would see each
/// other's. Poisoning is ignored — a panicking test leaves the guard poisoned
/// but the state is fully re-established by the next `enter_workspace`.
fn env_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Restores the process cwd and `HOME` when the test ends, panic or not.
struct EnvGuard {
    cwd: PathBuf,
    home: Option<OsString>,
}

impl Drop for EnvGuard {
    fn drop(&mut self) {
        let _ = std::env::set_current_dir(&self.cwd);
        match &self.home {
            Some(home) => std::env::set_var("HOME", home),
            None => std::env::remove_var("HOME"),
        }
    }
}

/// A scaffolded workspace + isolated `HOME`, with the process cwd moved into
/// it. Holds the lock and the restore guard for the test's lifetime.
///
/// FIELD ORDER IS LOAD-BEARING. Rust drops struct
/// fields in DECLARATION order, so the mutex must be declared LAST to be
/// dropped last: with the lock first it was released while `EnvGuard` had not
/// yet restored the cwd and `HOME`, leaving a window in which the next test
/// could take the lock and scaffold against this test's still-current
/// directory. The lock exists to cover the restore too, not just the body.
struct Fixture {
    _guard: EnvGuard,
    dir: tempfile::TempDir,
    _lock: MutexGuard<'static, ()>,
}

impl Fixture {
    fn root(&self) -> PathBuf {
        // Resolve symlinks: on macOS the process cwd reports `/private/var/…`
        // where the tempdir path is `/var/…`, and a mismatch would make a
        // path-completion prefix comparison fail for the wrong reason.
        std::fs::canonicalize(self.dir.path()).expect("canonicalize tempdir")
    }
}

fn write(path: &Path, contents: &str) {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).expect("create parent");
    }
    std::fs::write(path, contents).expect("write fixture");
}

/// Scaffold a workspace with two graphs, two node types, one workspace schema,
/// one `.msg` store entry, and a `~/.cerulion` holding one paired robot; then
/// move the process into it.
fn enter_workspace() -> Fixture {
    let lock = env_lock();
    let guard = EnvGuard {
        cwd: std::env::current_dir().expect("read cwd"),
        home: std::env::var_os("HOME"),
    };
    let dir = tempfile::tempdir().expect("tempdir");
    let root = dir.path();

    write(&root.join("Cargo.toml"), "[workspace]\nmembers = []\n");
    write(&root.join("graphs").join("perception.yaml"), "nodes: {}\n");
    write(
        &root.join("graphs").join("control_loop.yaml"),
        "nodes: {}\n",
    );
    write(
        &root
            .join("nodes")
            .join("lidar_filter")
            .join("src")
            .join("lib.rs"),
        "// node\n",
    );
    write(
        &root
            .join("nodes")
            .join("imu_fusion")
            .join("src")
            .join("lib.rs"),
        "// node\n",
    );
    write(
        &root.join("schemas").join("my_detection.yaml"),
        "schemas: {}\n",
    );
    write(
        &root
            .join("schemas")
            .join("go2_msgs")
            .join("msg")
            .join("LowState.msg"),
        "uint8 level\n",
    );
    // Two bags and a decoy, for the `.mcap` filter arm.
    write(&root.join("run1.mcap"), "");
    write(&root.join("run2.mcap"), "");
    write(&root.join("notes.txt"), "");
    // An isolated HOME so the developer's real `~/.cerulion` cannot leak in.
    write(
        &root.join("home").join(".cerulion").join("robots.toml"),
        "[robots]\nspot_lab = \"aabbcc\"\n",
    );

    std::env::set_var("HOME", root.join("home"));
    std::env::set_current_dir(root).expect("enter workspace");

    Fixture {
        dir,
        _guard: guard,
        _lock: lock,
    }
}

/// Drive the REAL completion engine over `words`, completing the LAST one.
///
/// This is the same entry point `Zsh::write_complete` / `Bash::write_complete`
/// call, so a wiring regression shows up here exactly as it would on a TAB.
fn complete(words: &[&str]) -> Vec<String> {
    let mut cmd = Cli::command();
    let args: Vec<OsString> = words.iter().map(OsString::from).collect();
    let index = args.len() - 1;
    let cwd = std::env::current_dir().ok();
    let candidates: Vec<CompletionCandidate> =
        clap_complete::engine::complete(&mut cmd, args, index, cwd.as_deref())
            .expect("completion engine");
    candidates
        .into_iter()
        .filter(|c| !c.is_hide_set())
        .map(|c| c.get_value().to_string_lossy().into_owned())
        .collect()
}

/// The candidates that are VALUES rather than flags.
///
/// clap always offers the in-scope `--flags` when the current word is empty,
/// and they are not what any of these tests is about.
fn values(words: &[&str]) -> Vec<String> {
    let mut out: Vec<String> = complete(words)
        .into_iter()
        .filter(|c| !c.starts_with('-'))
        .collect();
    out.sort();
    out
}

// ---------------------------------------------------------------------------
// Behavioural arms — the completer is attached to the arg the user types at
// ---------------------------------------------------------------------------

#[test]
fn graph_run_completes_the_scaffolded_graph_names() {
    let _fx = enter_workspace();
    assert_eq!(
        values(&["cerulion", "graph", "run", ""]),
        ["control_loop", "perception"]
    );
    // Prefix filtering is clap's job, but it is the behaviour a user sees.
    assert_eq!(values(&["cerulion", "graph", "run", "per"]), ["perception"]);
}

#[test]
fn node_build_completes_the_scaffolded_node_types() {
    let _fx = enter_workspace();
    assert_eq!(
        values(&["cerulion", "node", "build", ""]),
        ["imu_fusion", "lidar_filter"]
    );
}

#[test]
fn ros2_offers_attach_and_the_removed_ros_family_is_not_offered() {
    let _fx = enter_workspace();
    // The `ros2` family completes all three verbs — `attach` lives here, not in
    // the removed `ros` family (`cerulion ros attach` →
    // `cerulion ros2 attach`, hard break, no alias).
    let ros2_verbs = values(&["cerulion", "ros2", ""]);
    for verb in ["attach", "run", "launch"] {
        assert!(
            ros2_verbs.contains(&verb.to_string()),
            "`cerulion ros2` must offer `{verb}`, got {ros2_verbs:?}"
        );
    }
    // The attach flag surface rode along intact.
    let flags = complete(&["cerulion", "ros2", "attach", "--"]);
    assert!(
        flags.contains(&"--iface".to_string()),
        "`cerulion ros2 attach` must complete `--iface`, got {flags:?}"
    );
    // Top level: `ros2` is offered; the removed `ros` family is a HIDDEN
    // migration stub and must never be offered — a completion suggesting
    // `ros` would advertise a verb that only prints the migration error.
    let top = values(&["cerulion", ""]);
    assert!(
        top.contains(&"ros2".to_string()),
        "top level must offer `ros2`, got {top:?}"
    );
    assert!(
        !top.contains(&"ros".to_string()),
        "the removed `ros` family must not be offered at top level, got {top:?}"
    );
}

#[test]
fn schema_info_completes_workspace_msg_store_and_builtin_names() {
    let _fx = enter_workspace();
    let got = values(&["cerulion", "schema", "info", ""]);
    assert!(
        got.contains(&"my_detection".to_string()),
        "workspace schema missing"
    );
    assert!(
        got.contains(&"go2_msgs/LowState".to_string()),
        "msg store missing"
    );
    assert!(
        got.contains(&"sensor_msgs/Image".to_string()),
        "built-in missing"
    );
}

#[test]
fn viz_robot_completes_the_paired_robot_from_the_isolated_home() {
    let _fx = enter_workspace();
    assert_eq!(values(&["cerulion", "viz", "--robot", ""]), ["spot_lab"]);
}

// ---------------------------------------------------------------------------
// The create arms complete NOTHING
// ---------------------------------------------------------------------------

#[test]
fn create_verbs_offer_no_value_candidates_while_their_siblings_do() {
    let _fx = enter_workspace();

    // The pin. A create argument names something that does not exist yet, so
    // offering the existing set completes a guaranteed error.
    for verb in [
        ["cerulion", "node", "create", ""],
        ["cerulion", "graph", "create", ""],
        ["cerulion", "schema", "create", ""],
    ] {
        assert!(
            values(&verb).is_empty(),
            "`{}` must offer no value candidates, got {:?}",
            verb.join(" "),
            values(&verb)
        );
    }

    // The ANTI-TAUTOLOGY half, in the same body and the same workspace: the
    // sibling verbs that take an EXISTING name still complete. Without this,
    // an empty workspace — or a completely un-wired build — would pass the
    // loop above.
    assert_eq!(
        values(&["cerulion", "node", "delete", ""]),
        ["imu_fusion", "lidar_filter"]
    );
    assert_eq!(
        values(&["cerulion", "graph", "validate", ""]),
        ["control_loop", "perception"]
    );
    assert!(values(&["cerulion", "schema", "delete", ""]).contains(&"my_detection".to_string()));
}

// ---------------------------------------------------------------------------
// Path arguments — `ValueHint::Unknown` is the "do not complete" arm, so a
// dynamic completer without hints would SILENTLY delete file completion
// ---------------------------------------------------------------------------

#[test]
fn bag_play_completes_mcap_bags_and_directories_but_not_other_files() {
    let _fx = enter_workspace();
    // `bag play <bag>` carries the `.mcap` PathCompleter and is the
    // path `--resim` arrives through (there is no `replay <bag>` verb).
    let got = values(&["cerulion", "bag", "play", ""]);

    assert!(
        got.contains(&"run1.mcap".to_string()),
        "bag missing from {got:?}"
    );
    assert!(
        got.contains(&"run2.mcap".to_string()),
        "bag missing from {got:?}"
    );
    assert!(
        !got.contains(&"notes.txt".to_string()),
        "the .mcap filter is not applied: {got:?}"
    );
    // Directories stay offered — a bag in a subdirectory must remain
    // reachable, which is why the filter admits `p.is_dir()`.
    assert!(
        got.iter().any(|c| c == "graphs/"),
        "directories must stay traversable: {got:?}"
    );
}

#[test]
fn bag_migrate_completes_mcap_bags_on_both_the_input_and_the_output() {
    let _fx = enter_workspace();
    // BOTH args, because they are wired independently: `-o` names a
    // file that does not exist yet, so it would be easy to leave it un-wired
    // and never notice from the input arg alone.
    for argv in [
        vec!["cerulion", "bag", "migrate", ""],
        vec!["cerulion", "bag", "migrate", "run1.mcap", "-o", ""],
    ] {
        let got = values(&argv);
        assert!(
            got.contains(&"run1.mcap".to_string()) && got.contains(&"run2.mcap".to_string()),
            "bags missing from {argv:?}: {got:?}"
        );
        assert!(
            !got.contains(&"notes.txt".to_string()),
            "the .mcap filter is not applied on {argv:?}: {got:?}"
        );
        assert!(
            got.iter().any(|c| c == "graphs/"),
            "directories must stay traversable on {argv:?}: {got:?}"
        );
    }
}

#[test]
fn hinted_path_args_still_fall_back_to_file_completion() {
    let fx = enter_workspace();
    let root = fx.root();

    // A FilePath arg: files and directories both offered.
    let tolerance = values(&["cerulion", "bag", "play", "x.mcap", "--tolerance", ""]);
    assert!(
        tolerance.contains(&"notes.txt".to_string()),
        "an input-file arg must complete files: {tolerance:?}"
    );

    // A DirPath arg: directories only.
    let schemas_dir = values(&["cerulion", "connect", "--schemas-dir", ""]);
    assert!(
        schemas_dir.contains(&"schemas/".to_string()),
        "a directory arg must complete directories: {schemas_dir:?}"
    );
    assert!(
        !schemas_dir.contains(&"notes.txt".to_string()),
        "a directory arg must not complete plain files: {schemas_dir:?}"
    );

    // Sanity that the fixture really is what these completions are reading.
    assert!(root.join("notes.txt").is_file());
}

// ---------------------------------------------------------------------------
// Structural inventory — covers the arg (`topic hz`) whose source needs a live
// SHM topic, and makes any added/removed completer a DELIBERATE change
// ---------------------------------------------------------------------------

/// Walk the built command tree, collecting `"path::arg"` for every argument
/// carrying an `ArgValueCandidates` or `ArgValueCompleter` extension.
fn wired_args() -> Vec<String> {
    fn walk(cmd: &clap::Command, path: &str, out: &mut Vec<String>) {
        for arg in cmd.get_arguments() {
            let wired = arg.get::<ArgValueCandidates>().is_some()
                || arg.get::<ArgValueCompleter>().is_some();
            if wired {
                out.push(format!("{path}::{}", arg.get_id()));
            }
        }
        for sub in cmd.get_subcommands() {
            walk(sub, &format!("{path} {}", sub.get_name()), out);
        }
    }
    let mut cmd = Cli::command();
    cmd.build();
    let mut out = Vec::new();
    walk(&cmd, "cerulion", &mut out);
    out.sort();
    out
}

#[test]
fn the_wired_completer_inventory_is_exactly_the_declared_set() {
    // Hand-written oracle. Adding a completer without adding it here fails;
    // so does silently dropping one (the create-verb hazard is the
    // reverse — three completers that must never be attached).
    // NOTE the absentees: every path argument is covered by a `value_hint`
    // rather than an extension, so only `replay::bag` — which needs the
    // `.mcap` FILTER a hint cannot express — appears here. The hints are
    // enforced by `every_arg_that_completes_nothing_is_a_declared_free_form_value`
    // (an un-hinted path arg lands in its actual set and fails the walk).
    let expected = [
        // `bag play`/`bag info` take the same `.mcap` PathCompleter as
        // `replay <bag>` (a hint cannot express the extension filter), and
        // `bag record` completes the LIVE topic set — unlike `bag play
        // --topics`, whose set lives inside a bag nothing has opened yet.
        "cerulion bag info::bag",
        // `bag migrate` takes a `.mcap` bag IN and writes one OUT, so
        // both args carry the same PathCompleter filter — a `value_hint` cannot
        // express "only .mcap, but keep directories traversable".
        "cerulion bag migrate::bag",
        "cerulion bag migrate::out",
        "cerulion bag play::bag",
        // `--resim` stays a free-form `String` so a node subset
        // reaches the engine's refusal instead of clap's "invalid
        // value", and the completer offers the one selection that resolves.
        "cerulion bag play::resim",
        "cerulion bag record::topics",
        "cerulion connect::robot",
        "cerulion graph chains::name",
        "cerulion graph levels::name",
        "cerulion graph partition::name",
        "cerulion graph profile::name",
        "cerulion graph run::name",
        "cerulion graph validate::name",
        "cerulion node build::node_type",
        "cerulion node delete::node_type",
        "cerulion node info::node_type",
        "cerulion node modify::node_type",
        "cerulion node run::node_type",
        "cerulion node stage::graph",
        "cerulion node stage::node_type",
        "cerulion pair::robot",
        // The one arg whose source needs a LIVE iceoryx2 topic — behaviourally
        // unreachable in a hermetic test, so the structural pin is what covers
        // it. A live positive is in `topic hz`'s own e2e transcript.
        "cerulion topic echo::topic",
        "cerulion topic hz::topic",
        "cerulion topic info::topic",
        "cerulion schema delete::name",
        "cerulion schema info::name",
        "cerulion viz::robot",
        "cerulion viz::topics",
    ];
    let mut expected: Vec<String> = expected.iter().map(|s| s.to_string()).collect();
    expected.sort();

    assert_eq!(wired_args(), expected);

    // Spelled out because it is the create-verb regression guard, and because a
    // set equality can be satisfied by an oracle someone edited to match a
    // regression. These three must NEVER appear.
    for forbidden in [
        "cerulion node create::node_type",
        "cerulion graph create::name",
        "cerulion schema create::name",
    ] {
        assert!(
            !wired_args().iter().any(|w| w == forbidden),
            "`{forbidden}` completes names the verb REJECTS"
        );
    }
}

#[test]
fn the_help_text_quotes_the_same_install_line_the_hint_prints() {
    // The fish line is quoted on THREE surfaces —
    // `install_hint`, this `--help` table, and `docs/cli_completions.md`. Two
    // of them are code, so pin them against each other: whatever
    // `install_hint` says must appear VERBATIM in the long help. (The doc file
    // is prose and cannot be pinned here; these two cannot drift.)
    let mut cmd = crate::cli::Cli::command();
    cmd.build();
    let long_about = cmd
        .get_subcommands()
        .find(|c| c.get_name() == "completions")
        .and_then(|c| c.get_long_about())
        .expect("completions long help")
        .to_string();

    for shell in <crate::completion::CompletionShell as clap::ValueEnum>::value_variants() {
        for line in shell.install_hint().lines() {
            // The hint is the shell-ready `echo '…' >> rc` form; the help
            // table shows the same command. Compare the whole line.
            assert!(
                long_about.contains(line),
                "`cerulion completions --help` does not quote the {shell:?} \
                 install line it would print:\n  {line}\n\
                 The two surfaces have drifted — fix whichever is wrong."
            );
        }
    }
}

/// Every value-taking argument that completes NOTHING — no `value_hint`, no
/// completer, no `ValueEnum`/`value_parser` possible-values.
///
/// `ValueHint::Unknown` is literally clap_complete's "should not complete"
/// arm, so once a dynamic completer is registered for the binary these args
/// lose even the shell's own default file completion.
fn args_completing_nothing() -> Vec<String> {
    fn walk(cmd: &clap::Command, path: &str, out: &mut Vec<String>) {
        for arg in cmd.get_arguments() {
            // A flag takes no value, so there is nothing to complete.
            if matches!(
                arg.get_action(),
                clap::ArgAction::SetTrue | clap::ArgAction::SetFalse
            ) || arg.get_num_args().is_some_and(|r| r.max_values() == 0)
            {
                continue;
            }
            let covered = arg.get_value_hint() != clap::ValueHint::Unknown
                || arg.get::<ArgValueCompleter>().is_some()
                || arg.get::<ArgValueCandidates>().is_some()
                || !arg.get_possible_values().is_empty();
            if !covered {
                out.push(format!("{path}::{}", arg.get_id()));
            }
        }
        for sub in cmd.get_subcommands() {
            walk(sub, &format!("{path} {}", sub.get_name()), out);
        }
    }
    let mut cmd = Cli::command();
    cmd.build();
    let mut out = Vec::new();
    walk(&cmd, "cerulion", &mut out);
    out.sort();
    out.dedup();
    out
}

#[test]
fn every_arg_that_completes_nothing_is_a_declared_free_form_value() {
    // A walk keyed on the VALUE
    // NAME (`PATH` / `DIR` / `FILE`) is not total, and a
    // sibling path argument goes straight through it: `workspace init
    // <LOCATION>` is `String`-typed and named LOCATION, so it is invisible to
    // such a walk exactly as it is to a `PathBuf` sweep — the
    // same class missed twice.
    //
    // So the walk is INVERTED. It does not guess which args are paths; it
    // lists every arg that completes NOTHING and requires each to be DECLARED
    // here as a deliberate free-form value. A new argument of ANY name fails
    // until someone classifies it, and for a path the fix is a `value_hint`,
    // never an addition below.
    //
    // Note what is NOT here: `PathBuf`-typed args. clap auto-derives
    // `ValueHint::AnyPath` from the value parser's type id
    // (`Arg::get_value_hint`), so those complete paths with no attribute at
    // all — which is why an argument that completes
    // nothing is always `String`-typed. An explicit hint on a `PathBuf` arg is a
    // REFINEMENT (FilePath / DirPath narrow what is offered), not a fix.
    //
    // Everything listed is genuinely free-form: a name being invented, an
    // opaque identifier, a number, or a string with no enumerable local
    // source. The shell falls back to filenames for these — noise, but not
    // wrong; the alternative is inventing candidates nobody can know.
    let declared_free_form = [
        // ---- Names being CREATED. The existing set is what these verbs
        // REJECT, so completing anything is an error.
        "cerulion graph create::name",
        "cerulion node create::node_type",
        "cerulion ros2 attach::graph_name",
        "cerulion schema create::name",
        "cerulion workspace create::name",
        // ---- Identifiers, prefixes and labels the user INVENTS.
        // A free-text note recorded into the capture's own
        // manifest. There is nothing to complete it FROM — it is prose about an
        // incident that has just happened.
        "cerulion flashback::note",
        "cerulion graph create::prefix",
        "cerulion node run::id",
        "cerulion node run::prefix",
        "cerulion node stage::id",
        "cerulion pair::label",
        "cerulion ros2 attach::robot_name",
        "cerulion ros2 attach::topic_prefix",
        // ---- Port declarations: `SCHEMA NAME` pairs where NAME is invented.
        // The SCHEMA half IS a known set, but `num_args = 2` makes the pair a
        // single arg and `ArgValueCandidates` cannot address one value of it
        // (`ArgValueCompleter::complete_at` could; it is not wired).
        "cerulion node create::input",
        "cerulion node create::output",
        "cerulion node create::trigger_input",
        "cerulion node modify::input",
        "cerulion node modify::output",
        "cerulion node modify::trigger_input",
        "cerulion node stage::input_binding",
        // ---- Opaque identifiers: hex endpoint ids, account ids, device ids,
        // pairing codes. Nothing local enumerates them.
        "cerulion account devices revoke::device_id",
        "cerulion connect::eid",
        "cerulion pair::account",
        "cerulion pair::code",
        "cerulion pair::eid",
        // ---- Network locators, URLs and interface addresses.
        "cerulion connect::addrs",
        "cerulion connect::relay_url",
        "cerulion pair::addrs",
        "cerulion pair::relay_url",
        "cerulion ros2 attach::iface",
        "cerulion topic list::connect",
        "cerulion topic list::listen",
        "cerulion viz::connect",
        "cerulion viz::listen",
        // ---- Free-form policy / filter / mode strings behind a custom parser
        // rather than an enumerable `value_parser` list.
        "cerulion connect::network",
        "cerulion graph run::record_cpu",
        "cerulion node create::policy",
        "cerulion node modify::policy",
        "cerulion trace inspect::filter",
        // ---- Verbatim pass-through args. `cerulion ros2 run|launch` forward
        // everything after the verb token to the native `ros2` untouched —
        // the candidate set is ros2's own (packages, executables, launch
        // files, its flags), which a TAB press may not enumerate.
        "cerulion ros2 launch::args",
        "cerulion ros2 run::args",
        // ---- The REMOVED `cerulion ros` family's hidden stub. It swallows
        // the old argv (`attach --iface …`) so `main` can print the migration
        // error naming `cerulion ros2 attach`; completing anything here would
        // suggest the verb still exists. The command is `hide = true`, so the
        // shells never offer it — this entry only classifies the swallowing
        // arg for this walk, which visits hidden commands too.
        "cerulion ros::args",
        // ---- Topics whose set is NOT locally knowable. `connect --topic`
        // names a topic on a robot this desk has not mirrored yet — that is
        // the entire point of the verb — so the local SHM scan would offer
        // exactly the wrong set. (`topic echo/info/hz` and `viz` DO complete:
        // those act on what is already here.)
        "cerulion connect::topics",
        // ---- Topics whose set is NOT locally knowable, part 2.
        // `bag play --topics` names a topic INSIDE a bag. The local SHM scan
        // would offer exactly the wrong set (what is live HERE, which for a bag
        // being played back is by definition not the robot's), and reading the
        // bag would mean opening and walking a file on a TAB press — which the
        // completer's hard "no I/O beyond an instant local read" budget forbids.
        // `cerulion bag info <bag>` is the way to see a bag's topics.
        "cerulion bag play::topics",
        // ---- Free-form REGEX patterns (the `ros2 bag record -e/-x`
        // surface). A pattern is not a name: completing the topic set here
        // would offer literals where the user is typing an expression.
        "cerulion bag record::exclude",
        "cerulion bag record::regex",
        // ---- A RUN ID (or a graph name), which is not offline
        // knowable. The live-run set lives in a `/__cerulion/runs` iceoryx2
        // service, and reading it means a `RUN_GATHER_WINDOW` (600 ms) windowed
        // LISTEN over a transport — four times the whole completion budget, and
        // squarely inside the "a TAB press never opens a transport" rule the
        // engine's own source walk enforces. `cerulion bag record --run` with no
        // value is the common case anyway (it attaches to the one live run and
        // NAMES the alternatives when there are several), so the id is
        // copy-pasted from a refusal rather than typed.
        "cerulion bag record::run",
        // ---- Numbers.
        // BAG-TIME bounds in seconds, not paths. (They replace
        // `--max-ticks`, which is deleted outright — no alias, no shim.)
        "cerulion bag play::duration",
        "cerulion bag play::rate",
        "cerulion bag play::start_offset",
        "cerulion bag record::duration",
        "cerulion bag record::schema_wait_ms",
        "cerulion graph partition::budget_ns",
        "cerulion graph profile::duration",
        "cerulion graph profile::fires",
        "cerulion graph run::trace_limit",
        "cerulion ros2 attach::domain",
        "cerulion ros2 attach::timeout",
        "cerulion topic echo::truncate_length",
        "cerulion trace inspect::limit",
        // ---- `cerulion bagd` (the recorder daemon `graph run --record`
        // spawns). Its args live in `cerulion_bagd::BagdArgs`, so wiring a
        // completer onto them would make that crate depend on `clap_complete`
        // — deliberately not done for an internal verb. Its `PathBuf` args
        // (`--out`, `--topics-json`, `--ready-file`, the config files) are
        // ABSENT from this list because clap auto-hints `PathBuf`; these are
        // the ones with no path type to infer from.
        "cerulion bagd::attach", // `NAME:PATH` pairs, not a bare path
        "cerulion bagd::discovery_settle_ms",
        // The recorder's own label for its Flashback verdicts —
        // `graph run` supplies the graph name and nothing else knows it. Like
        // its `cerulion bagd` siblings it is not completable anyway: completing
        // them would make `cerulion_bagd` depend on `clap_complete`.
        "cerulion bagd::flashback_label",
        "cerulion bagd::flush_interval_ms",
        "cerulion bagd::rings",
        // The id of the run this recorder is BOUND to, so
        // it finalizes when that run ends. CALLER-ONLY PROVENANCE — only the
        // process that resolved the run knows it, and an operator typing one by
        // hand would be guessing. Enumerating live run ids would also mean
        // opening the run registry, which a completion may not do (the HARD RULE
        // in `completions.rs`). Free-form for the same reason as every other
        // `cerulion bagd` arg here: completing it would make `cerulion_bagd`
        // depend on `clap_complete`.
        "cerulion bagd::run_id",
        // A millisecond budget — an invented number, like its
        // `--discovery-settle-ms` and `--schema-wait-timeout-ms` siblings above
        // and below.
        "cerulion bagd::schema_demand_timeout_ms",
        "cerulion bagd::schema_wait_timeout_ms",
        "cerulion bagd::status_period_ms",
        // The checkpoint half.
        // `--state-ring` is a POSIX SHM object name (the same shape as `--ring`
        // above — not a filesystem path), and `--state-tag` is the mapped-SHM tag
        // of the plane a GRAPH armed — a name derived from a run id, which
        // nothing on this machine can enumerate without opening the run registry
        // (a completion may not: see the HARD RULE in `completions.rs`).
        //
        // `--state-arm`, `--state-cadence-steps` and `--state-first-anchor-step`
        // were here and are RETIRED with the recorder's ability to arm. Their
        // absence is pinned by the set equality below — re-adding a clap arg
        // without a completer fails it — and, for the flag itself, by
        // `a_retired_state_arm_flag_is_an_unknown_argument`.
        "cerulion bagd::state_rings",
        "cerulion bagd::state_tag",
        "cerulion bagd::topics",
    ];
    let mut expected: Vec<String> = declared_free_form.iter().map(|s| s.to_string()).collect();
    expected.sort();

    assert_eq!(
        args_completing_nothing(),
        expected,
        "\nAn argument's completion coverage changed.\n\
         If a NEW argument appears: is its value a PATH? Give it a \
         `value_hint` (FilePath / DirPath / AnyPath) — do NOT add it to the \
         declared list, or it will silently complete nothing. Add it only if \
         the value is genuinely free-form (an invented name, an opaque id, a \
         number, a locator).\n\
         If one DISAPPEARED: it grew a hint or a completer — drop it from the \
         list."
    );
}
