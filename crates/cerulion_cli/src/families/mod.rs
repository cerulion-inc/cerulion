// SPDX-License-Identifier: AGPL-3.0-only
//! The verb-family registry: a top-level `cerulion <family>` verb that lives in
//! its own module instead of the central `Commands` enum.
//!
//! A family owns everything about its verb in ONE module: the clap definition
//! (its args carry their own completers, so completion comes with it), the log
//! class, whether the login gate runs first, and the dispatch. It joins the
//! binary through ONE line in [`FAMILIES`]. Nothing else changes: not the
//! `Commands` enum, not `main`'s dispatch, not the log-class match in `cli.rs`,
//! not the completion inventory code. [`cli_command`] is the one command
//! factory `main` parses with and the completion engine completes from, so a
//! registered family is in help, parsing and completion by construction.
//!
//! The registry is checked when the command is built ([`build`]): two families
//! with one name, a family that shadows a built-in verb, a family listed after
//! a verb that does not exist, and a family whose clap definition is not the
//! one verb it names are all refused with a [`RegistryError`], never resolved
//! by whichever registration came last.

use std::fmt;
use std::process::ExitCode;

use clap::{ArgMatches, Command, CommandFactory};

use crate::cli::{Cli, VerbLogClass};

/// One top-level verb family. See the module docs for the contract.
///
/// Every method that takes `matches` receives the family's OWN sub-matches
/// (`cerulion <family> ...`), never the root's.
pub(crate) trait VerbFamily: Sync {
    /// The top-level verb, as typed: `cerulion <name>`.
    fn name(&self) -> &'static str;

    /// The verb this family is listed directly after in `cerulion --help` and
    /// in completion. `None` lists it after every built-in verb.
    fn listed_after(&self) -> Option<&'static str>;

    /// Add exactly one subcommand, named [`VerbFamily::name`], to `root`.
    fn augment(&self, root: Command) -> Command;

    /// The default tracing filter class of the parsed invocation.
    fn log_class(&self, matches: &ArgMatches) -> VerbLogClass;

    /// Whether the login gate runs before [`VerbFamily::run`].
    fn needs_identity(&self, matches: &ArgMatches) -> bool;

    /// Run the parsed invocation and return the process exit code. Logging is
    /// initialised and the login gate has passed by the time this is called.
    fn run(&self, matches: &ArgMatches) -> ExitCode;
}

/// Every registered family. Adding a family is one line here.
pub(crate) static FAMILIES: &[&dyn VerbFamily] = &[];

/// Why a set of families cannot be registered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RegistryError {
    /// Two families declare the same top-level verb.
    DuplicateName(&'static str),
    /// A family declares the name of a verb `Commands` already defines.
    ShadowsBuiltin(&'static str),
    /// A family is listed after a verb that does not exist.
    UnknownAnchor {
        family: &'static str,
        anchor: &'static str,
    },
    /// A family's `augment` did not add exactly the one verb it names.
    AugmentMismatch(&'static str),
}

impl fmt::Display for RegistryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RegistryError::DuplicateName(name) => write!(
                f,
                "two verb families are registered as `cerulion {name}`; each top-level verb \
                 must be registered once"
            ),
            RegistryError::ShadowsBuiltin(name) => write!(
                f,
                "the verb family `{name}` has the name of a built-in verb; rename the family \
                 or remove the built-in verb"
            ),
            RegistryError::UnknownAnchor { family, anchor } => write!(
                f,
                "the verb family `{family}` is listed after `{anchor}`, which is not a verb"
            ),
            RegistryError::AugmentMismatch(name) => write!(
                f,
                "the verb family `{name}` must add exactly one subcommand named `{name}`"
            ),
        }
    }
}

impl std::error::Error for RegistryError {}

/// Add `families` to `base` and fix the listing order.
///
/// The order is `base`'s own subcommand order with each family inserted
/// directly after its [`VerbFamily::listed_after`] verb (in registration
/// order when several name the same anchor), and families with no anchor at
/// the end. Help and completion both sort by display order, so the explicit
/// order set here is what both show; clap's own `help` verb keeps its default
/// (last) place.
pub(crate) fn build(base: Command, families: &[&dyn VerbFamily]) -> Result<Command, RegistryError> {
    let builtins: Vec<String> = base
        .get_subcommands()
        .map(|c| c.get_name().to_string())
        .collect();
    let mut order = builtins.clone();
    let mut registered: Vec<&'static str> = Vec::with_capacity(families.len());
    let mut cmd = base;
    for family in families {
        let name = family.name();
        if registered.contains(&name) {
            return Err(RegistryError::DuplicateName(name));
        }
        if builtins.iter().any(|b| b == name) {
            return Err(RegistryError::ShadowsBuiltin(name));
        }
        let before = cmd.get_subcommands().count();
        cmd = family.augment(cmd);
        let added = cmd.get_subcommands().count() - before;
        if added != 1 || cmd.find_subcommand(name).is_none() {
            return Err(RegistryError::AugmentMismatch(name));
        }
        match family.listed_after() {
            Some(anchor) => {
                let Some(at) = order.iter().position(|n| n == anchor) else {
                    return Err(RegistryError::UnknownAnchor {
                        family: name,
                        anchor,
                    });
                };
                // After the anchor and after any family already placed behind it.
                let mut at = at + 1;
                while at < order.len() && registered.contains(&order[at].as_str()) {
                    at += 1;
                }
                order.insert(at, name.to_string());
            }
            None => order.push(name.to_string()),
        }
        registered.push(name);
    }
    for (position, name) in order.iter().enumerate() {
        cmd = cmd.mut_subcommand(name, |sub| sub.display_order(position));
    }
    Ok(cmd)
}

/// The `cerulion` command: the `Commands` verbs plus every registered family.
///
/// The one factory `main` parses with and the completion engine completes
/// from.
pub(crate) fn cli_command() -> Command {
    match build(Cli::command(), FAMILIES) {
        Ok(cmd) => cmd,
        // A registry mistake is a build-time bug in this binary, never a user
        // error; `the_registered_families_build` fails on it first.
        Err(e) => panic!("verb-family registry: {e}"),
    }
}

/// The registered family `matches` selected, with that family's sub-matches.
/// `None` when the verb is a built-in `Commands` verb.
pub(crate) fn find<'a>(
    families: &[&'a dyn VerbFamily],
    matches: &'a ArgMatches,
) -> Option<(&'a dyn VerbFamily, &'a ArgMatches)> {
    let (name, sub) = matches.subcommand()?;
    families
        .iter()
        .find(|f| f.name() == name)
        .map(|f| (*f, sub))
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::ffi::OsString;
    use std::sync::Mutex;

    use clap::{FromArgMatches, Subcommand};
    use clap_complete::engine::ArgValueCandidates;

    /// The values the dummy family's `echo` verb was run with.
    static DUMMY_RUNS: Mutex<Vec<String>> = Mutex::new(Vec::new());

    #[derive(Subcommand)]
    enum DummyCmd {
        /// A test-only family
        Dummy {
            #[command(subcommand)]
            action: DummyAction,
        },
    }

    #[derive(Subcommand)]
    enum DummyAction {
        /// Echo a word
        Echo {
            /// The word
            #[arg(add = ArgValueCandidates::new(|| vec!["alpha".into(), "beta".into()]))]
            word: String,
        },
        /// Wait for a while
        Wait,
    }

    /// A family that exists only in this test. Registering it touches nothing
    /// outside this module.
    struct Dummy {
        name: &'static str,
        after: Option<&'static str>,
    }

    const DUMMY: Dummy = Dummy {
        name: "dummy",
        after: None,
    };

    impl VerbFamily for Dummy {
        fn name(&self) -> &'static str {
            self.name
        }
        fn listed_after(&self) -> Option<&'static str> {
            self.after
        }
        fn augment(&self, root: Command) -> Command {
            let sub = DummyCmd::augment_subcommands(Command::new("x"))
                .find_subcommand("dummy")
                .cloned()
                .unwrap_or_else(|| Command::new("dummy"));
            root.subcommand(sub.name(self.name))
        }
        fn log_class(&self, matches: &ArgMatches) -> VerbLogClass {
            match matches.subcommand_name() {
                Some("wait") => VerbLogClass::LongRunning,
                _ => VerbLogClass::OneShot,
            }
        }
        fn needs_identity(&self, _matches: &ArgMatches) -> bool {
            false
        }
        fn run(&self, matches: &ArgMatches) -> ExitCode {
            match DummyAction::from_arg_matches(matches) {
                Ok(DummyAction::Echo { word }) => {
                    DUMMY_RUNS
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push(word);
                    ExitCode::from(42)
                }
                Ok(DummyAction::Wait) => ExitCode::from(43),
                Err(_) => ExitCode::FAILURE,
            }
        }
    }

    /// A family whose `augment` adds nothing.
    struct Silent;
    impl VerbFamily for Silent {
        fn name(&self) -> &'static str {
            "silent"
        }
        fn listed_after(&self) -> Option<&'static str> {
            None
        }
        fn augment(&self, root: Command) -> Command {
            root
        }
        fn log_class(&self, _: &ArgMatches) -> VerbLogClass {
            VerbLogClass::OneShot
        }
        fn needs_identity(&self, _: &ArgMatches) -> bool {
            true
        }
        fn run(&self, _: &ArgMatches) -> ExitCode {
            ExitCode::SUCCESS
        }
    }

    fn with(families: &[&dyn VerbFamily]) -> Command {
        build(Cli::command(), families).expect("families register")
    }

    fn listing(cmd: &mut Command) -> Vec<String> {
        // The `Commands:` block of the root help, verb names only.
        let help = cmd.render_help().to_string();
        help.lines()
            .skip_while(|l| *l != "Commands:")
            .skip(1)
            .take_while(|l| !l.is_empty())
            .filter_map(|l| l.split_whitespace().next().map(str::to_string))
            .collect()
    }

    fn complete(cmd: &mut Command, words: &[&str]) -> Vec<String> {
        let args: Vec<OsString> = words.iter().map(OsString::from).collect();
        let index = args.len() - 1;
        clap_complete::engine::complete(cmd, args, index, None)
            .expect("completion engine")
            .into_iter()
            .filter(|c| !c.is_hide_set())
            .map(|c| c.get_value().to_string_lossy().into_owned())
            .filter(|c| !c.starts_with('-'))
            .collect()
    }

    // ---- happy path: one registration point puts a family everywhere ----

    #[test]
    fn a_registered_family_is_listed_in_root_help() {
        let mut cmd = with(&[&DUMMY]);
        let help = cmd.render_help().to_string();
        assert!(
            help.contains("  dummy        A test-only family\n"),
            "{help}"
        );
        // No anchor: after every built-in verb, before clap's `help`.
        let verbs = listing(&mut cmd);
        assert_eq!(verbs[verbs.len() - 2..], ["dummy", "help"]);
    }

    #[test]
    fn a_registered_family_completes_its_name_its_verbs_and_its_values() {
        let mut cmd = with(&[&DUMMY]);
        assert_eq!(complete(&mut cmd, &["cerulion", "dum"]), ["dummy"]);
        assert_eq!(
            complete(&mut cmd, &["cerulion", "dummy", ""]),
            ["echo", "wait", "help"]
        );
        assert_eq!(
            complete(&mut cmd, &["cerulion", "dummy", "echo", ""]),
            ["alpha", "beta"]
        );
    }

    #[test]
    fn a_registered_family_is_dispatched_with_its_own_matches() {
        let families: [&dyn VerbFamily; 1] = [&DUMMY];
        let matches = with(&families)
            .try_get_matches_from(["cerulion", "dummy", "echo", "gamma"])
            .expect("parse");
        let (family, sub) = find(&families, &matches).expect("dummy is a family");
        assert_eq!(family.name(), "dummy");
        assert!(!family.needs_identity(sub));
        assert_eq!(family.log_class(sub), VerbLogClass::OneShot);
        assert_eq!(crate::telemetry::exit_code_number(family.run(sub)), 42);
        assert!(DUMMY_RUNS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains(&"gamma".to_string()));
    }

    #[test]
    fn a_family_decides_its_log_class_per_verb() {
        let families: [&dyn VerbFamily; 1] = [&DUMMY];
        let matches = with(&families)
            .try_get_matches_from(["cerulion", "dummy", "wait"])
            .expect("parse");
        let (family, sub) = find(&families, &matches).expect("family");
        assert_eq!(family.log_class(sub), VerbLogClass::LongRunning);
        assert_eq!(crate::telemetry::exit_code_number(family.run(sub)), 43);
    }

    #[test]
    fn a_family_takes_the_global_verbose_flag() {
        let families: [&dyn VerbFamily; 1] = [&DUMMY];
        let matches = with(&families)
            .try_get_matches_from(["cerulion", "dummy", "wait", "-v"])
            .expect("parse");
        assert!(matches.get_flag("verbose"));
    }

    // ---- edge: built-in verbs are untouched and placement is exact ----

    #[test]
    fn a_built_in_verb_is_not_a_family() {
        let families: [&dyn VerbFamily; 1] = [&DUMMY];
        let matches = with(&families)
            .try_get_matches_from(["cerulion", "graph", "list"])
            .expect("parse");
        assert!(find(&families, &matches).is_none());
        // The built-in parse still reads the same matches.
        assert!(Cli::from_arg_matches(&matches).is_ok());
    }

    #[test]
    fn an_empty_registry_leaves_the_listing_as_the_enum_declares_it() {
        let mut cmd = with(&[]);
        let mut plain = Cli::command();
        assert_eq!(listing(&mut cmd), listing(&mut plain));
    }

    #[test]
    fn a_family_listed_after_a_verb_sits_directly_behind_it() {
        let a = Dummy {
            name: "dummy",
            after: Some("viz"),
        };
        let b = Dummy {
            name: "dummy2",
            after: Some("viz"),
        };
        let mut cmd = with(&[&a, &b]);
        let verbs = listing(&mut cmd);
        let at = verbs.iter().position(|v| v == "viz").expect("viz");
        // Two families behind one anchor keep their registration order.
        assert_eq!(verbs[at..at + 4], ["viz", "dummy", "dummy2", "connect"]);
    }

    // ---- error arms: no silent shadowing ----

    #[test]
    fn two_families_with_one_name_are_refused() {
        let err = build(Cli::command(), &[&DUMMY, &DUMMY]).expect_err("duplicate");
        assert_eq!(err, RegistryError::DuplicateName("dummy"));
        assert_eq!(
            err.to_string(),
            "two verb families are registered as `cerulion dummy`; each top-level verb must \
             be registered once"
        );
    }

    #[test]
    fn a_family_with_a_built_in_name_is_refused() {
        let graph = Dummy {
            name: "graph",
            after: None,
        };
        let err = build(Cli::command(), &[&graph]).expect_err("shadow");
        assert_eq!(err, RegistryError::ShadowsBuiltin("graph"));
    }

    #[test]
    fn a_family_listed_after_a_missing_verb_is_refused() {
        let lost = Dummy {
            name: "dummy",
            after: Some("nope"),
        };
        let err = build(Cli::command(), &[&lost]).expect_err("anchor");
        assert_eq!(
            err,
            RegistryError::UnknownAnchor {
                family: "dummy",
                anchor: "nope"
            }
        );
    }

    #[test]
    fn a_family_that_adds_no_verb_is_refused() {
        let err = build(Cli::command(), &[&Silent]).expect_err("mismatch");
        assert_eq!(err, RegistryError::AugmentMismatch("silent"));
    }

    // ---- determinism and the production registry ----

    #[test]
    fn building_twice_gives_the_same_help() {
        let render = || with(&[&DUMMY]).render_long_help().to_string();
        assert_eq!(render(), render());
    }

    #[test]
    fn the_registered_families_build() {
        build(Cli::command(), FAMILIES).expect("the production registry builds");
    }
}
