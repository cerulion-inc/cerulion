// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion rcp --sim` and `cerulion rcp bench`: find the harness and plan
//! the one program they run.
//!
//! The harness is a folder of Python, shell and pinned data: the measured
//! tools, the simulator install and the sim server. Its launcher
//! ([`LAUNCHER`]) is the one program both verbs run, under `bash`, with
//! inherited stdio ([`crate::connect_cmd::spawn_child_and_wait`]). Nothing of
//! the harness is linked into, or runs inside, the `cerulion` process.
//!
//! The folder is found in this order ([`find_harness_dir`]):
//!
//! 1. [`HARNESS_DIR_ENV`], when set and not empty. It is final: a value that
//!    names a missing folder, or a folder that holds no harness, is an error,
//!    never a fall through to the rungs below.
//! 2. `harness/` in the directory of the canonicalized running binary, where
//!    the release packages install it beside `cerulion`, `cerulion-netd` and
//!    the other programs. Canonicalized, so a link to `cerulion` in a `bin`
//!    directory still reaches the folder beside the real binary.
//! 3. `harness/` in the current directory or the nearest directory above it:
//!    a source checkout.
//!
//! A folder holds a harness when both [`LAUNCHER`] and [`INSTALL_SCRIPT`] are
//! files in it.
//!
//! The install writes inside the folder the launcher runs from. Nothing writes
//! into the folder beside the binary, a release package's, whichever rung
//! found it ([`HarnessSource::Packaged`]): [`plan`] passes [`PACKAGED_FLAG`]
//! for it, and the launcher copies it once to the copy home ([`copy_home`])
//! and installs and runs there. Any other folder runs in place, and the
//! launcher copies it the same way only when this user cannot write to it.

use std::ffi::OsStr;
use std::path::{Path, PathBuf};
use std::time::Duration;

use crate::connect_cmd::{ChildName, ConnectPlan};
use crate::error::{CliError, CliResult};

/// The environment variable that names the harness folder.
pub const HARNESS_DIR_ENV: &str = "CERULION_HARNESS_DIR";

/// The folder's name beside the binary and in a source checkout.
pub const HARNESS_FOLDER: &str = "harness";

/// The launcher both verbs run, relative to the folder.
pub const LAUNCHER: &str = "rcp.sh";

/// The simulator install the launcher runs on first use, relative to the
/// folder.
pub const INSTALL_SCRIPT: &str = "install.sh";

/// The shell the launcher is written for.
const SHELL: &str = "bash";

/// The launcher flag, after `--home DIR`, that says the folder is a release
/// package's: the launcher runs from the copy at `DIR` even when this user can
/// write to the folder.
pub const PACKAGED_FLAG: &str = "--packaged";

/// How long the sim server has to stop after the forwarded SIGINT: it scores
/// and stops the simulator of every live episode, waits for a simulator that is
/// still starting, and over HTTP waits up to its `stop_grace_s` setting (10 s by
/// default) for the open connections. A launcher killed at the backstop leaves its
/// simulators running until the next `cerulion rcp --sim` stops them.
pub const HARNESS_SHUTDOWN_GRACE: Duration = Duration::from_secs(60);

/// How the spawn names the launcher in its messages, and its shutdown grace.
pub const HARNESS_CHILD: ChildName = ChildName {
    name: "the Cerulion harness",
    not_found_hint: "The harness runs under `bash`; install it, or put it on PATH.",
    shutdown_grace: HARNESS_SHUTDOWN_GRACE,
};

/// What a harness invocation runs, with its parsed flags.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HarnessVerb {
    /// `cerulion rcp --sim`: serve the harness's tools with a simulated
    /// robot, over stdio, or over streamable HTTP at `http`.
    Sim {
        /// `--http ADDR`.
        http: Option<String>,
        /// `--task NAME`: the task the simulated robot is set up for.
        task: Option<String>,
    },
    /// `cerulion rcp bench`: run one benchmark cell with and without the
    /// harness against the user's model endpoint.
    Bench(BenchArgs),
    /// `cerulion rcp bench --verify`: check the harness code on disk against
    /// the measured code's manifests.
    Verify,
}

/// The flags of `cerulion rcp bench` without `--verify`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BenchArgs {
    /// `--model`: the model name the endpoint serves.
    pub model: String,
    /// `--base-url`: the endpoint's OpenAI-compatible API root.
    pub base_url: Option<String>,
    /// `--key-env`: the environment variable that holds the endpoint's key.
    pub key_env: Option<String>,
    /// `--cell`: the benchmark cell.
    pub cell: Option<String>,
    /// `--out`: the results folder.
    pub out: Option<PathBuf>,
}

/// Which kind of folder the search found, and so whether the launcher may
/// write into it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarnessSource {
    /// The folder [`HARNESS_DIR_ENV`] names (rung 1), the user's own: it runs
    /// in place.
    Named,
    /// The folder beside the canonicalized binary (rung 2, or rung 1 naming
    /// that same folder), a release package's: never written, it runs from
    /// its copy.
    Packaged,
    /// A source checkout's folder (rung 3): it runs in place.
    Checkout,
}

/// A harness folder and the kind of folder it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FoundHarness {
    /// The folder.
    pub dir: PathBuf,
    /// Which kind of folder it is.
    pub source: HarnessSource,
}

/// Whether `dir` holds a harness: both marker files are files there.
fn holds_harness(dir: &Path) -> bool {
    dir.join(LAUNCHER).is_file() && dir.join(INSTALL_SCRIPT).is_file()
}

/// Whether `a` and `b` resolve to the same folder; `false` when either cannot
/// be resolved.
fn same_folder(a: &Path, b: &Path) -> bool {
    matches!(
        (std::fs::canonicalize(a), std::fs::canonicalize(b)),
        (Ok(a), Ok(b)) if a == b
    )
}

/// The harness folder, searched in the module docs' order, and the kind of
/// folder it is.
///
/// `env_dir` is the value of [`HARNESS_DIR_ENV`] (`None` = unset), `exe_dir`
/// the directory of the canonicalized running binary (`None` when it cannot
/// be read) and `cwd` the current directory. Reads the filesystem; spawns
/// nothing.
///
/// # Errors
///
/// [`HARNESS_DIR_ENV`] set to a missing folder or to a folder that holds no
/// harness, or no harness on any rung. Each message names where it looked.
pub fn find_harness_dir(
    env_dir: Option<&OsStr>,
    exe_dir: Option<&Path>,
    cwd: &Path,
) -> CliResult<FoundHarness> {
    let beside = exe_dir.map(|dir| dir.join(HARNESS_FOLDER));
    if let Some(named) = env_dir.filter(|value| !value.is_empty()) {
        let dir = PathBuf::from(named);
        if !dir.is_dir() {
            return Err(CliError::Validation(format!(
                "{HARNESS_DIR_ENV} names '{}', which does not exist; point it at a harness \
                 folder, or unset it to use the one installed beside cerulion",
                dir.display()
            )));
        }
        if !holds_harness(&dir) {
            return Err(CliError::Validation(format!(
                "{HARNESS_DIR_ENV} names '{}', which holds no Cerulion harness (it needs \
                 {LAUNCHER} and {INSTALL_SCRIPT}); point it at a harness folder, or unset it \
                 to use the one installed beside cerulion",
                dir.display()
            )));
        }
        let source = match beside.as_deref() {
            Some(beside) if same_folder(&dir, beside) => HarnessSource::Packaged,
            _ => HarnessSource::Named,
        };
        return Ok(FoundHarness { dir, source });
    }
    if let Some(dir) = beside.as_deref().filter(|dir| holds_harness(dir)) {
        return Ok(FoundHarness {
            dir: dir.to_path_buf(),
            source: HarnessSource::Packaged,
        });
    }
    for above in cwd.ancestors() {
        let dir = above.join(HARNESS_FOLDER);
        if holds_harness(&dir) {
            return Ok(FoundHarness {
                dir,
                source: HarnessSource::Checkout,
            });
        }
    }
    let beside = match beside {
        Some(dir) => format!("'{}'", dir.display()),
        None => "beside the cerulion binary (its path could not be read)".to_string(),
    };
    Err(CliError::Validation(format!(
        "no Cerulion harness found. Looked at (1) {HARNESS_DIR_ENV}, which is unset, (2) \
         {beside}, where the release packages install it, and (3) a '{HARNESS_FOLDER}' folder \
         in '{}' or any directory above it, as in a source checkout. Reinstall Cerulion, run \
         this inside a checkout of https://github.com/cerulion-inc/cerulion, or set \
         {HARNESS_DIR_ENV} to a harness folder",
        cwd.display()
    )))
}

/// [`find_harness_dir`] for this process: its environment, its binary and its
/// current directory.
///
/// # Errors
///
/// As [`find_harness_dir`], or an unreadable current directory.
pub fn find_harness_dir_here() -> CliResult<FoundHarness> {
    let exe_dir = std::env::current_exe()
        .and_then(std::fs::canonicalize)
        .ok()
        .and_then(|exe| exe.parent().map(Path::to_path_buf));
    let cwd = std::env::current_dir()?;
    find_harness_dir(
        std::env::var_os(HARNESS_DIR_ENV).as_deref(),
        exe_dir.as_deref(),
        &cwd,
    )
}

/// Where the launcher keeps its own copy of a harness folder it must not
/// write to (a release package's, or one this user cannot write to):
/// `harness/<version>` under the Cerulion config directory `config_dir`, one
/// copy per release.
pub fn copy_home(config_dir: &Path) -> PathBuf {
    config_dir
        .join(HARNESS_FOLDER)
        .join(env!("CARGO_PKG_VERSION"))
}

/// `path` as the text a command line carries, or the refusal that names
/// `what` it is.
fn as_text<'a>(path: &'a Path, what: &str) -> CliResult<&'a str> {
    path.to_str().ok_or_else(|| {
        CliError::Validation(format!(
            "{what} '{}' is not valid UTF-8, so it cannot be named on a command line; use a \
             path that is",
            path.display()
        ))
    })
}

/// Push `--flag value` when `value` is set.
fn push_opt(argv: &mut Vec<String>, flag: &str, value: Option<&str>) {
    if let Some(value) = value {
        argv.push(flag.to_string());
        argv.push(value.to_string());
    }
}

/// The launcher command line for `verb`, with the harness folder `harness`
/// and the copy home `home` ([`copy_home`]). PURE.
///
/// `bash <dir>/rcp.sh --home <home> [--packaged] <sim|bench|verify> [flags]`:
/// [`PACKAGED_FLAG`] for a [`HarnessSource::Packaged`] folder only, and the
/// flags spelled as the launcher reads them, one word each.
///
/// # Errors
///
/// A path that is not UTF-8: the spawn plan carries its arguments as text.
pub fn plan(verb: &HarnessVerb, harness: &FoundHarness, home: &Path) -> CliResult<ConnectPlan> {
    let launcher = harness.dir.join(LAUNCHER);
    let mut argv = vec![
        as_text(&launcher, "the harness launcher")?.to_string(),
        "--home".to_string(),
        as_text(home, "the harness copy folder")?.to_string(),
    ];
    match harness.source {
        HarnessSource::Packaged => argv.push(PACKAGED_FLAG.to_string()),
        HarnessSource::Named | HarnessSource::Checkout => {}
    }
    match verb {
        HarnessVerb::Sim { http, task } => {
            argv.push("sim".to_string());
            push_opt(&mut argv, "--http", http.as_deref());
            push_opt(&mut argv, "--task", task.as_deref());
        }
        HarnessVerb::Bench(bench) => {
            argv.push("bench".to_string());
            argv.push("--model".to_string());
            argv.push(bench.model.clone());
            push_opt(&mut argv, "--base-url", bench.base_url.as_deref());
            push_opt(&mut argv, "--key-env", bench.key_env.as_deref());
            push_opt(&mut argv, "--cell", bench.cell.as_deref());
            let out = match &bench.out {
                Some(out) => Some(as_text(out, "the --out folder")?),
                None => None,
            };
            push_opt(&mut argv, "--out", out);
        }
        HarnessVerb::Verify => argv.push("verify".to_string()),
    }
    Ok(ConnectPlan {
        bin: PathBuf::from(SHELL),
        argv,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::OsString;

    /// Make `dir` hold a harness: both marker files, empty.
    fn harness_at(dir: &Path) {
        std::fs::create_dir_all(dir).expect("the harness folder");
        std::fs::write(dir.join(LAUNCHER), "").expect("rcp.sh");
        std::fs::write(dir.join(INSTALL_SCRIPT), "").expect("install.sh");
    }

    fn message(result: CliResult<FoundHarness>) -> String {
        match result {
            Err(CliError::Validation(message)) => message,
            other => panic!("wanted a refusal, got {other:?}"),
        }
    }

    /// Three places that each hold a harness: the variable's folder, the
    /// folder beside the binary and a checkout around the current directory.
    struct ThreeRungs {
        _root: tempfile::TempDir,
        named: PathBuf,
        exe_dir: PathBuf,
        checkout: PathBuf,
        cwd: PathBuf,
    }

    fn three_rungs() -> ThreeRungs {
        let root = tempfile::tempdir().expect("a scratch tree");
        let named = root.path().join("named-harness");
        harness_at(&named);
        let exe_dir = root.path().join("opt/cerulion/bin");
        harness_at(&exe_dir.join(HARNESS_FOLDER));
        let checkout = root.path().join("checkout");
        harness_at(&checkout.join(HARNESS_FOLDER));
        let cwd = checkout.join("docs/guides");
        std::fs::create_dir_all(&cwd).expect("a folder inside the checkout");
        ThreeRungs {
            _root: root,
            named,
            exe_dir,
            checkout,
            cwd,
        }
    }

    /// A plan's harness: `dir`, found as `source`.
    fn found(dir: &str, source: HarnessSource) -> FoundHarness {
        FoundHarness {
            dir: PathBuf::from(dir),
            source,
        }
    }

    #[test]
    fn the_variable_wins_then_the_folder_beside_the_binary_then_the_checkout() {
        let t = three_rungs();
        assert_eq!(
            find_harness_dir(Some(t.named.as_os_str()), Some(&t.exe_dir), &t.cwd).expect("found"),
            FoundHarness {
                dir: t.named.clone(),
                source: HarnessSource::Named
            },
            "rung 1: the variable, the user's own folder"
        );
        assert_eq!(
            find_harness_dir(None, Some(&t.exe_dir), &t.cwd).expect("found"),
            FoundHarness {
                dir: t.exe_dir.join("harness"),
                source: HarnessSource::Packaged
            },
            "rung 2: beside the binary, over a checkout around the current directory"
        );
        std::fs::remove_file(t.exe_dir.join(HARNESS_FOLDER).join(LAUNCHER)).expect("drop rung 2");
        assert_eq!(
            find_harness_dir(None, Some(&t.exe_dir), &t.cwd).expect("found"),
            FoundHarness {
                dir: t.checkout.join("harness"),
                source: HarnessSource::Checkout
            },
            "rung 3: the checkout above the current directory"
        );
    }

    #[test]
    fn a_variable_naming_the_folder_beside_the_binary_is_still_packaged() {
        let t = three_rungs();
        let beside = t.exe_dir.join(HARNESS_FOLDER);
        let spelled_otherwise = t.exe_dir.join("../bin/harness/.");
        for named in [&beside, &spelled_otherwise] {
            let got =
                find_harness_dir(Some(named.as_os_str()), Some(&t.exe_dir), &t.cwd).expect("found");
            assert_eq!(got.source, HarnessSource::Packaged, "{}", named.display());
            assert_eq!(
                &got.dir, named,
                "the folder is named as the variable spells it"
            );
        }
        #[cfg(unix)]
        {
            let link = t.cwd.join("my-harness");
            std::os::unix::fs::symlink(&beside, &link).expect("a link to the packaged folder");
            assert_eq!(
                find_harness_dir(Some(link.as_os_str()), Some(&t.exe_dir), &t.cwd)
                    .expect("found")
                    .source,
                HarnessSource::Packaged,
                "a link to the packaged folder"
            );
        }
        assert_eq!(
            find_harness_dir(Some(t.named.as_os_str()), None, &t.cwd)
                .expect("found")
                .source,
            HarnessSource::Named,
            "no binary path to compare with: the variable's folder is the user's own"
        );
    }

    #[test]
    fn finding_twice_gives_the_same_folder_and_kind() {
        let t = three_rungs();
        for env in [None, Some(t.named.as_os_str())] {
            let first = find_harness_dir(env, Some(&t.exe_dir), &t.cwd).expect("found");
            let second = find_harness_dir(env, Some(&t.exe_dir), &t.cwd).expect("found");
            assert_eq!(first, second);
        }
    }

    #[test]
    fn a_variable_naming_a_missing_folder_refuses_and_never_falls_through() {
        let t = three_rungs();
        let missing = t.cwd.join("no-such-folder");
        let said = message(find_harness_dir(
            Some(missing.as_os_str()),
            Some(&t.exe_dir),
            &t.cwd,
        ));
        assert_eq!(
            said,
            format!(
                "CERULION_HARNESS_DIR names '{}', which does not exist; point it at a harness \
                 folder, or unset it to use the one installed beside cerulion",
                missing.display()
            )
        );
    }

    #[test]
    fn a_variable_naming_a_folder_without_a_harness_refuses_and_never_falls_through() {
        let t = three_rungs();
        let empty = t.cwd.clone();
        let said = message(find_harness_dir(
            Some(empty.as_os_str()),
            Some(&t.exe_dir),
            &t.cwd,
        ));
        assert_eq!(
            said,
            format!(
                "CERULION_HARNESS_DIR names '{}', which holds no Cerulion harness (it needs \
                 rcp.sh and install.sh); point it at a harness folder, or unset it to use the \
                 one installed beside cerulion",
                empty.display()
            )
        );
    }

    #[test]
    fn an_empty_variable_is_unset() {
        let t = three_rungs();
        let empty = OsString::new();
        assert_eq!(
            find_harness_dir(Some(empty.as_os_str()), Some(&t.exe_dir), &t.cwd).expect("found"),
            FoundHarness {
                dir: t.exe_dir.join("harness"),
                source: HarnessSource::Packaged
            }
        );
    }

    #[test]
    fn nothing_found_names_all_three_places_it_looked() {
        let root = tempfile::tempdir().expect("an empty tree");
        let exe_dir = root.path().join("bin");
        let said = message(find_harness_dir(None, Some(&exe_dir), root.path()));
        assert_eq!(
            said,
            format!(
                "no Cerulion harness found. Looked at (1) CERULION_HARNESS_DIR, which is \
                 unset, (2) '{}', where the release packages install it, and (3) a 'harness' \
                 folder in '{}' or any directory above it, as in a source checkout. Reinstall \
                 Cerulion, run this inside a checkout of \
                 https://github.com/cerulion-inc/cerulion, or set CERULION_HARNESS_DIR to a \
                 harness folder",
                exe_dir.join("harness").display(),
                root.path().display()
            )
        );
        let unread = message(find_harness_dir(None, None, root.path()));
        assert!(
            unread.contains(
                "(2) beside the cerulion binary (its path could not be read), where the release"
            ),
            "{unread}"
        );
    }

    #[test]
    fn a_folder_with_one_marker_or_a_marker_that_is_a_folder_holds_no_harness() {
        for only in [LAUNCHER, INSTALL_SCRIPT] {
            let root = tempfile::tempdir().expect("a tree");
            let harness = root.path().join("harness");
            std::fs::create_dir_all(&harness).expect("harness/");
            std::fs::write(harness.join(only), "").expect("one marker");
            assert!(
                find_harness_dir(None, None, root.path()).is_err(),
                "{only} alone"
            );
        }
        let root = tempfile::tempdir().expect("a tree");
        let harness = root.path().join("harness");
        std::fs::create_dir_all(harness.join(LAUNCHER)).expect("rcp.sh/ as a folder");
        std::fs::create_dir_all(harness.join(INSTALL_SCRIPT)).expect("install.sh/ as a folder");
        assert!(find_harness_dir(None, None, root.path()).is_err());
    }

    #[test]
    fn the_copy_home_is_one_folder_per_release_under_the_config_directory() {
        assert_eq!(
            copy_home(Path::new("/cfg/.cerulion")),
            PathBuf::from(format!(
                "/cfg/.cerulion/harness/{}",
                env!("CARGO_PKG_VERSION")
            ))
        );
    }

    #[test]
    fn sim_runs_the_launcher_under_bash_on_stdio_or_http() {
        let harness = found("/src/cerulion/harness", HarnessSource::Checkout);
        let home = Path::new("/cfg/.cerulion/harness/2.1.0");
        let stdio = plan(
            &HarnessVerb::Sim {
                http: None,
                task: None,
            },
            &harness,
            home,
        )
        .expect("a plan");
        assert_eq!(stdio.bin, PathBuf::from("bash"));
        assert_eq!(
            stdio.argv,
            [
                "/src/cerulion/harness/rcp.sh",
                "--home",
                "/cfg/.cerulion/harness/2.1.0",
                "sim"
            ]
        );
        let http = plan(
            &HarnessVerb::Sim {
                http: Some("127.0.0.1:8765".to_string()),
                task: Some("open-drawer".to_string()),
            },
            &harness,
            home,
        )
        .expect("a plan");
        assert_eq!(
            http.argv[3..],
            ["sim", "--http", "127.0.0.1:8765", "--task", "open-drawer"]
        );
    }

    #[test]
    fn only_the_packaged_folder_is_told_to_run_from_its_copy() {
        let home = Path::new("/cfg/.cerulion/harness/2.1.0");
        let sim = HarnessVerb::Sim {
            http: None,
            task: None,
        };
        let bench = HarnessVerb::Bench(BenchArgs {
            model: "m".to_string(),
            ..BenchArgs::default()
        });
        let packaged = found("/opt/cerulion/bin/harness", HarnessSource::Packaged);
        for (verb, words) in [
            (&sim, &["sim"][..]),
            (&bench, &["bench", "--model", "m"][..]),
            (&HarnessVerb::Verify, &["verify"][..]),
        ] {
            let mut want = vec![
                "/opt/cerulion/bin/harness/rcp.sh",
                "--home",
                "/cfg/.cerulion/harness/2.1.0",
                "--packaged",
            ];
            want.extend_from_slice(words);
            assert_eq!(
                plan(verb, &packaged, home).expect("a plan").argv,
                want,
                "{verb:?}"
            );
        }
        for source in [HarnessSource::Named, HarnessSource::Checkout] {
            let in_place = found("/home/me/harness", source);
            for verb in [&sim, &bench, &HarnessVerb::Verify] {
                let argv = plan(verb, &in_place, home).expect("a plan").argv;
                assert_eq!(
                    argv[..3],
                    [
                        "/home/me/harness/rcp.sh",
                        "--home",
                        "/cfg/.cerulion/harness/2.1.0"
                    ],
                    "{source:?} {verb:?}"
                );
                assert!(
                    !argv.iter().any(|word| word == PACKAGED_FLAG),
                    "{source:?} {verb:?}: {argv:?}"
                );
            }
        }
    }

    #[test]
    fn bench_hands_each_flag_over_as_one_word_spaces_included() {
        let verb = HarnessVerb::Bench(BenchArgs {
            model: "my model".to_string(),
            base_url: Some("https://example.test/v1".to_string()),
            key_env: Some("MY_KEY".to_string()),
            cell: Some("pro_spatial_swap_t00_i00".to_string()),
            out: Some(PathBuf::from("/data/bench out")),
        });
        let in_place = found("harness", HarnessSource::Checkout);
        let plan = plan(&verb, &in_place, Path::new("/h")).expect("a plan");
        assert_eq!(
            plan.argv,
            [
                "harness/rcp.sh",
                "--home",
                "/h",
                "bench",
                "--model",
                "my model",
                "--base-url",
                "https://example.test/v1",
                "--key-env",
                "MY_KEY",
                "--cell",
                "pro_spatial_swap_t00_i00",
                "--out",
                "/data/bench out",
            ]
        );
        let bare = HarnessVerb::Bench(BenchArgs {
            model: "m".to_string(),
            ..BenchArgs::default()
        });
        assert_eq!(
            super::plan(&bare, &in_place, Path::new("/h"))
                .expect("a plan")
                .argv,
            ["harness/rcp.sh", "--home", "/h", "bench", "--model", "m"]
        );
    }

    #[test]
    fn verify_runs_the_launcher_with_no_flags() {
        let in_place = found("harness", HarnessSource::Named);
        let plan = plan(&HarnessVerb::Verify, &in_place, Path::new("/h")).expect("a plan");
        assert_eq!(plan.argv, ["harness/rcp.sh", "--home", "/h", "verify"]);
    }

    #[cfg(unix)]
    #[test]
    fn a_path_that_is_not_utf8_is_refused_and_named_for_what_it_is() {
        use std::os::unix::ffi::OsStrExt;
        let bad = Path::new(OsStr::from_bytes(b"/src/\xff/harness"));
        let good = Path::new("/src/harness");
        let bad_harness = FoundHarness {
            dir: bad.to_path_buf(),
            source: HarnessSource::Packaged,
        };
        let good_harness = found("/src/harness", HarnessSource::Packaged);
        let out = HarnessVerb::Bench(BenchArgs {
            model: "m".to_string(),
            out: Some(bad.to_path_buf()),
            ..BenchArgs::default()
        });
        let cases = [
            (
                plan(&HarnessVerb::Verify, &bad_harness, good),
                "the harness launcher '",
            ),
            (
                plan(&HarnessVerb::Verify, &good_harness, bad),
                "the harness copy folder '",
            ),
            (plan(&out, &good_harness, good), "the --out folder '"),
        ];
        for (refused, opens_with) in cases {
            match refused {
                Err(CliError::Validation(said)) => {
                    assert!(said.starts_with(opens_with), "{said}");
                    assert!(
                        said.ends_with(
                            "is not valid UTF-8, so it cannot be named on a command line; use a \
                             path that is"
                        ),
                        "{said}"
                    );
                }
                other => panic!("wanted a refusal, got {other:?}"),
            }
        }
    }

    #[test]
    fn planning_twice_gives_the_same_plan() {
        let verb = HarnessVerb::Sim {
            http: Some("127.0.0.1:8765".to_string()),
            task: None,
        };
        for source in [
            HarnessSource::Named,
            HarnessSource::Packaged,
            HarnessSource::Checkout,
        ] {
            let harness = found("/h/harness", source);
            let first = plan(&verb, &harness, Path::new("/c")).expect("a plan");
            let second = plan(&verb, &harness, Path::new("/c")).expect("a plan");
            assert_eq!(first.bin, second.bin);
            assert_eq!(first.argv, second.argv);
        }
    }
}
