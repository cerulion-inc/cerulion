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
//! with one name, a family whose name or alias is a word that already selects
//! a verb (a built-in verb, a built-in alias, clap's own `help`, or an earlier
//! family), a family listed after a verb that does not exist, and a family
//! whose clap definition does anything but add the one verb it names are all
//! refused with a [`RegistryError`], never resolved by whichever registration
//! came last.

use std::collections::BTreeSet;
use std::fmt;
use std::process::ExitCode;

use clap::{ArgMatches, Command, CommandFactory};

use cerulion_cli_engine::error::{render_user_error, CliResult};

use crate::cli::{Cli, VerbLogClass};

mod schema;

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
pub(crate) static FAMILIES: &[&dyn VerbFamily] = &[&schema::Schema];

/// Why a set of families cannot be registered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RegistryError {
    /// Two families declare the same top-level verb, or a family is named like
    /// an earlier family's alias.
    DuplicateName(&'static str),
    /// A family declares the name or an alias of a verb `Commands` already
    /// defines, or clap's own `help`.
    ShadowsBuiltin(&'static str),
    /// A family's own alias is a word that already selects another verb.
    AliasTaken { family: &'static str, alias: String },
    /// A family is listed after a verb that does not exist.
    UnknownAnchor {
        family: &'static str,
        anchor: &'static str,
    },
    /// A family's `augment` did not add exactly the one verb it names.
    AugmentMismatch(&'static str),
    /// A family's `augment` changed the root command's own help text.
    RetitlesRoot(&'static str),
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
                "the verb family `{name}` has the name or alias of a built-in verb (or clap's \
                 `help`); rename the family or remove the built-in verb"
            ),
            RegistryError::AliasTaken { family, alias } => write!(
                f,
                "the verb family `{family}` has the alias `{alias}`, which already selects \
                 another verb; drop or rename the alias"
            ),
            RegistryError::UnknownAnchor { family, anchor } => write!(
                f,
                "the verb family `{family}` is listed after `{anchor}`, which is not a verb"
            ),
            RegistryError::AugmentMismatch(name) => write!(
                f,
                "the verb family `{name}` must add exactly one subcommand named `{name}` and \
                 leave every other subcommand as it was"
            ),
            RegistryError::RetitlesRoot(name) => write!(
                f,
                "the verb family `{name}` changed the `cerulion` help text; a doc comment on \
                 a derived `Subcommand` enum becomes the root's about, so use a plain comment"
            ),
        }
    }
}

impl std::error::Error for RegistryError {}

/// The generic exit mapping every `Commands` verb gets: success is 0, an error
/// is printed as `Error: <message>` on stderr and is exit 1.
pub(crate) fn exit_code_of(result: CliResult<()>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {}", render_user_error(&e));
            ExitCode::FAILURE
        }
    }
}

/// Add `families` to `base` and fix the listing order.
///
/// The order is `base`'s own subcommand order with each family inserted
/// directly after its [`VerbFamily::listed_after`] verb (in registration
/// order when several name the same anchor), and families with no anchor at
/// the end. Help and completion both sort by display order, so the explicit
/// order set here is what both show; clap's own `help` verb keeps its default
/// (last) place.
pub(crate) fn build(base: Command, families: &[&dyn VerbFamily]) -> Result<Command, RegistryError> {
    let mut order: Vec<String> = base
        .get_subcommands()
        .map(|c| c.get_name().to_string())
        .collect();
    // Every word that already selects a verb: the built-in names, their aliases
    // and clap's implicit `help`, which clap adds only when the command is built
    // and so is not among `get_subcommands` yet.
    let builtin_words: BTreeSet<String> = base
        .get_subcommands()
        .flat_map(|c| std::iter::once(c.get_name()).chain(c.get_all_aliases()))
        .chain(std::iter::once("help"))
        .map(str::to_string)
        .collect();
    // The names and aliases the families registered so far took.
    let mut family_words: BTreeSet<String> = BTreeSet::new();
    let mut registered: Vec<&'static str> = Vec::with_capacity(families.len());
    let mut cmd = base;
    for family in families {
        let name = family.name();
        if family_words.contains(name) {
            return Err(RegistryError::DuplicateName(name));
        }
        if builtin_words.contains(name) {
            return Err(RegistryError::ShadowsBuiltin(name));
        }
        let mut expected = subcommand_names(&cmd);
        expected.insert(name.to_string());
        let root_text = (
            cmd.get_about().map(ToString::to_string),
            cmd.get_long_about().map(ToString::to_string),
        );
        cmd = family.augment(cmd);
        // Exactly the one verb added, and nothing dropped or renamed.
        if subcommand_names(&cmd) != expected {
            return Err(RegistryError::AugmentMismatch(name));
        }
        let aliases: Vec<String> = cmd
            .find_subcommand(name)
            .map(|c| c.get_all_aliases().map(str::to_string).collect())
            .unwrap_or_default();
        for alias in &aliases {
            if builtin_words.contains(alias) || family_words.contains(alias) || alias == name {
                return Err(RegistryError::AliasTaken {
                    family: name,
                    alias: alias.clone(),
                });
            }
        }
        let root_text_now = (
            cmd.get_about().map(ToString::to_string),
            cmd.get_long_about().map(ToString::to_string),
        );
        if root_text_now != root_text {
            return Err(RegistryError::RetitlesRoot(name));
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
        family_words.insert(name.to_string());
        family_words.extend(aliases);
    }
    for (position, name) in order.iter().enumerate() {
        cmd = cmd.mut_subcommand(name, |sub| sub.display_order(position));
    }
    Ok(cmd)
}

/// The names of `cmd`'s direct subcommands.
fn subcommand_names(cmd: &Command) -> BTreeSet<String> {
    cmd.get_subcommands()
        .map(|c| c.get_name().to_string())
        .collect()
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
        gated: bool,
        aliases: &'static [&'static str],
    }

    const DUMMY: Dummy = Dummy {
        name: "dummy",
        after: None,
        gated: false,
        aliases: &[],
    };

    /// The dummy family under another name, everything else as [`DUMMY`].
    const fn dummy(name: &'static str) -> Dummy {
        Dummy {
            name,
            after: None,
            gated: false,
            aliases: &[],
        }
    }

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
            root.subcommand(sub.name(self.name).aliases(self.aliases))
        }
        fn log_class(&self, matches: &ArgMatches) -> VerbLogClass {
            match matches.subcommand_name() {
                Some("wait") => VerbLogClass::LongRunning,
                _ => VerbLogClass::OneShot,
            }
        }
        fn needs_identity(&self, _matches: &ArgMatches) -> bool {
            self.gated
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

    /// A family whose `augment` adds its verb but also rewrites the root's about,
    /// which is what a doc comment on a derived `Subcommand` enum does.
    struct Retitle;
    impl VerbFamily for Retitle {
        fn name(&self) -> &'static str {
            "retitle"
        }
        fn listed_after(&self) -> Option<&'static str> {
            None
        }
        fn augment(&self, root: Command) -> Command {
            root.about("not cerulion")
                .subcommand(Command::new("retitle"))
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

    /// A family whose `augment` is any function of the root, to drive every
    /// way an `augment` can do something other than add its one verb.
    struct Augments {
        name: &'static str,
        augment: fn(Command) -> Command,
    }
    impl VerbFamily for Augments {
        fn name(&self) -> &'static str {
            self.name
        }
        fn listed_after(&self) -> Option<&'static str> {
            None
        }
        fn augment(&self, root: Command) -> Command {
            (self.augment)(root)
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
            ..dummy("")
        };
        let b = Dummy {
            name: "dummy2",
            after: Some("viz"),
            ..dummy("")
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
            ..dummy("")
        };
        let err = build(Cli::command(), &[&graph]).expect_err("shadow");
        assert_eq!(err, RegistryError::ShadowsBuiltin("graph"));
    }

    #[test]
    fn a_family_listed_after_a_missing_verb_is_refused() {
        let lost = Dummy {
            name: "dummy",
            after: Some("nope"),
            ..dummy("")
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

    #[test]
    fn a_family_that_adds_a_verb_with_another_name_is_refused() {
        let other = Augments {
            name: "mine",
            augment: |root| root.subcommand(Command::new("other")),
        };
        let err = build(Cli::command(), &[&other]).expect_err("mismatch");
        assert_eq!(err, RegistryError::AugmentMismatch("mine"));
        assert_eq!(
            err.to_string(),
            "the verb family `mine` must add exactly one subcommand named `mine` and leave \
             every other subcommand as it was"
        );
    }

    #[test]
    fn a_family_that_adds_two_verbs_is_refused() {
        let two = Augments {
            name: "mine",
            augment: |root| {
                root.subcommand(Command::new("mine"))
                    .subcommand(Command::new("extra"))
            },
        };
        let err = build(Cli::command(), &[&two]).expect_err("mismatch");
        assert_eq!(err, RegistryError::AugmentMismatch("mine"));
    }

    #[test]
    fn a_family_that_drops_built_in_verbs_is_refused() {
        // Drops two built-ins and adds its own verb: the count FALLS, which a
        // subtraction of counts underflowed on; comparing the names refuses it.
        let drops = Augments {
            name: "mine",
            augment: |root| {
                let kept: Vec<Command> = root
                    .get_subcommands()
                    .filter(|c| !matches!(c.get_name(), "graph" | "viz"))
                    .cloned()
                    .collect();
                let mut rebuilt = Command::new("cerulion").subcommands(kept);
                if let Some(about) = root.get_about() {
                    rebuilt = rebuilt.about(about.clone());
                }
                rebuilt.subcommand(Command::new("mine"))
            },
        };
        let err = build(Cli::command(), &[&drops]).expect_err("mismatch");
        assert_eq!(err, RegistryError::AugmentMismatch("mine"));
    }

    #[test]
    fn a_family_that_renames_a_built_in_verb_is_refused() {
        let renames = Augments {
            name: "mine",
            augment: |root| {
                root.mut_subcommand("graph", |c| c.name("graphs"))
                    .subcommand(Command::new("mine"))
            },
        };
        let err = build(Cli::command(), &[&renames]).expect_err("mismatch");
        assert_eq!(err, RegistryError::AugmentMismatch("mine"));
    }

    #[test]
    fn a_family_named_help_is_refused() {
        // clap adds `help` only when the command is built, so it is not among
        // the base's subcommands and must be reserved by name.
        let help = dummy("help");
        let err = build(Cli::command(), &[&help]).expect_err("shadow");
        assert_eq!(err, RegistryError::ShadowsBuiltin("help"));
        assert_eq!(
            err.to_string(),
            "the verb family `help` has the name or alias of a built-in verb (or clap's \
             `help`); rename the family or remove the built-in verb"
        );
    }

    #[test]
    fn a_family_named_like_a_built_in_alias_is_refused() {
        let base = Cli::command().subcommand(Command::new("aliased").alias("al"));
        let al = dummy("al");
        let err = build(base, &[&al]).expect_err("shadow");
        assert_eq!(err, RegistryError::ShadowsBuiltin("al"));
    }

    #[test]
    fn a_family_alias_that_selects_another_verb_is_refused() {
        let cases: [(&'static [&'static str], Option<&'static str>); 3] = [
            (&["graph"], None),
            (&["help"], None),
            (&["dummy2"], Some("dummy2")),
        ];
        for (aliases, earlier) in cases {
            let aliased = Dummy {
                aliases,
                ..dummy("dummy")
            };
            let first = earlier.map(dummy);
            let mut families: Vec<&dyn VerbFamily> = Vec::new();
            if let Some(first) = &first {
                families.push(first);
            }
            families.push(&aliased);
            let err = build(Cli::command(), &families).expect_err("alias");
            assert_eq!(
                err,
                RegistryError::AliasTaken {
                    family: "dummy",
                    alias: aliases[0].to_string()
                },
                "alias {aliases:?}"
            );
        }
        let err = RegistryError::AliasTaken {
            family: "dummy",
            alias: "graph".into(),
        };
        assert_eq!(
            err.to_string(),
            "the verb family `dummy` has the alias `graph`, which already selects another \
             verb; drop or rename the alias"
        );
    }

    #[test]
    fn a_family_named_like_an_earlier_family_alias_is_refused() {
        let aliased = Dummy {
            aliases: &["d2"],
            ..dummy("dummy")
        };
        let second = dummy("d2");
        let err = build(Cli::command(), &[&aliased, &second]).expect_err("duplicate");
        assert_eq!(err, RegistryError::DuplicateName("d2"));
    }

    #[test]
    fn a_family_alias_that_is_free_registers_and_parses() {
        let aliased = Dummy {
            aliases: &["dm"],
            ..dummy("dummy")
        };
        let families: [&dyn VerbFamily; 1] = [&aliased];
        let matches = with(&families)
            .try_get_matches_from(["cerulion", "dm", "wait"])
            .expect("parse");
        let (family, _) = find(&families, &matches).expect("the alias selects the family");
        assert_eq!(family.name(), "dummy");
    }

    #[test]
    fn a_family_that_rewrites_the_root_help_is_refused() {
        let err = build(Cli::command(), &[&Retitle]).expect_err("retitle");
        assert_eq!(err, RegistryError::RetitlesRoot("retitle"));
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
