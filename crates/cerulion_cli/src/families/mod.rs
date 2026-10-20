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
//! whose clap definition is named for another verb are all refused with a
//! [`RegistryError`], never resolved by whichever registration came last. A
//! family hands over its own verb ([`VerbFamily::command`]) and never sees the
//! root, so it cannot change a built-in verb, a root argument or the root's
//! help text.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::process::ExitCode;

use clap::{ArgMatches, Command, CommandFactory};

use cerulion_cli_engine::error::{render_user_error, CliError, CliResult, EXIT_NODE_FATAL};

use crate::cli::{Cli, VerbLogClass};

mod nested;
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

    /// The family's verb: one command named [`VerbFamily::name`], which
    /// [`build`] adds to the root. [`derived_verb`] makes it from a derived
    /// one-variant `Subcommand` enum.
    fn command(&self) -> Command;

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
    /// A family's [`VerbFamily::command`] is not named for the family.
    WrongVerb { family: &'static str, verb: String },
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
            RegistryError::WrongVerb { family, verb } => write!(
                f,
                "the verb family `{family}` defines the command `{verb}`; its command must be \
                 named `{family}`, with exactly one variant in its derived `Subcommand` enum"
            ),
        }
    }
}

impl std::error::Error for RegistryError {}

/// The generic exit mapping every verb gets, built-in or family: success is 0,
/// an error is printed as `Error: <message>` on stderr and exits with
/// [`cli_error_exit_code`].
pub(crate) fn exit_code_of(result: CliResult<()>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {}", render_user_error(&e));
            ExitCode::from(cli_error_exit_code(&e))
        }
    }
}

/// The process exit status for an error a verb returned: `EXIT_NODE_FATAL` (3)
/// for a run that stopped a node on a fatal error, in process or in a worker
/// group, and 1 for every other error.
fn cli_error_exit_code(err: &CliError) -> u8 {
    match err {
        CliError::NodeFatal { .. } | CliError::WorkerNodeFatal { .. } => EXIT_NODE_FATAL,
        _ => 1,
    }
}

/// The one verb a derived `Subcommand` enum `S` defines, for
/// [`VerbFamily::command`].
///
/// The derive adds its variants to a scratch root that is then dropped, so the
/// enum's doc comment (which the derive copies onto that root's about) never
/// reaches `cerulion`. An enum with no variant or with several gives a command
/// with an empty name, which [`build`] refuses as [`RegistryError::WrongVerb`].
pub(crate) fn derived_verb<S: clap::Subcommand>() -> Command {
    let scratch = S::augment_subcommands(Command::new("scratch"));
    let mut verbs = scratch.get_subcommands();
    match (verbs.next(), verbs.next()) {
        (Some(verb), None) => verb.clone(),
        _ => Command::new(""),
    }
}

/// Add `families` to `base` and fix the listing order.
///
/// The order is `base`'s own subcommand order with each family inserted
/// directly after its [`VerbFamily::listed_after`] verb, behind every family
/// placed there before it together with the families listed after those (in
/// registration order when several name the same anchor), and families with
/// no anchor at the end. Help and completion both sort by display order, so
/// the explicit order set here is what both show; clap's own `help` verb keeps
/// its default (last) place.
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
    // Each registered family's anchor, to place a family behind the whole
    // group already listed after the same verb.
    let mut anchors: BTreeMap<&'static str, Option<&'static str>> = BTreeMap::new();
    let mut cmd = base;
    for family in families {
        let name = family.name();
        if family_words.contains(name) {
            return Err(RegistryError::DuplicateName(name));
        }
        if builtin_words.contains(name) {
            return Err(RegistryError::ShadowsBuiltin(name));
        }
        let verb = family.command();
        if verb.get_name() != name {
            return Err(RegistryError::WrongVerb {
                family: name,
                verb: verb.get_name().to_string(),
            });
        }
        let aliases: Vec<String> = verb.get_all_aliases().map(str::to_string).collect();
        for alias in &aliases {
            if builtin_words.contains(alias) || family_words.contains(alias) || alias == name {
                return Err(RegistryError::AliasTaken {
                    family: name,
                    alias: alias.clone(),
                });
            }
        }
        cmd = cmd.subcommand(verb);
        match family.listed_after() {
            Some(anchor) => {
                let Some(at) = order.iter().position(|n| n == anchor) else {
                    return Err(RegistryError::UnknownAnchor {
                        family: name,
                        anchor,
                    });
                };
                // After the anchor and after every family already listed behind
                // it, directly or through a family listed after one of those.
                let mut at = at + 1;
                while at < order.len() && listed_behind(&anchors, &order[at], anchor) {
                    at += 1;
                }
                order.insert(at, name.to_string());
            }
            None => order.push(name.to_string()),
        }
        anchors.insert(name, family.listed_after());
        family_words.insert(name.to_string());
        family_words.extend(aliases);
    }
    for (position, name) in order.iter().enumerate() {
        cmd = cmd.mut_subcommand(name, |sub| sub.display_order(position));
    }
    Ok(cmd)
}

/// Whether the registered family `verb` is listed after `anchor`, directly or
/// through a chain of families each listed after the one before. A built-in
/// verb is listed behind nothing. The chain ends: each family names a verb
/// that was already listed when it registered.
fn listed_behind(
    anchors: &BTreeMap<&'static str, Option<&'static str>>,
    verb: &str,
    anchor: &str,
) -> bool {
    let mut at = anchors.get(verb).copied().flatten();
    while let Some(parent) = at {
        if parent == anchor {
            return true;
        }
        at = anchors.get(parent).copied().flatten();
    }
    false
}

/// The `cerulion` command: the `Commands` verbs plus every registered family.
///
/// The one factory `main` parses with and the completion engine completes
/// from.
pub(crate) fn cli_command() -> Command {
    let cmd = match build(Cli::command(), FAMILIES) {
        Ok(cmd) => cmd,
        // A registry mistake is a build-time bug in this binary, never a user
        // error; `the_registered_families_build` fails on it first.
        Err(e) => panic!("verb-family registry: {e}"),
    };
    match nested::attach(cmd, nested::NESTED) {
        Ok(cmd) => cmd,
        // The same class of bug; `the_registered_nested_verbs_attach` fails first.
        Err(e) => panic!("nested-verb registry: {e}"),
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

/// Run the registered family or nested verb `matches` selected, the way
/// `main` does: `setup` with its log class (logging and the iceoryx2 bridge),
/// then `gate` when it needs an identity, then its own handler.
///
/// `None` when the verb is a built-in `Commands` verb, with neither `setup` nor
/// `gate` called. A gate that refuses returns its exit code and the verb does
/// not run.
pub(crate) fn route(
    families: &[&dyn VerbFamily],
    matches: &ArgMatches,
    setup: impl FnOnce(VerbLogClass),
    gate: impl FnOnce() -> Option<ExitCode>,
) -> Option<ExitCode> {
    route_with(families, nested::NESTED, matches, setup, gate)
}

/// [`route`] over an explicit nested-verb registry.
///
/// A nested verb is looked up FIRST: its parent can be a registered family,
/// and that family's own handler does not know the verbs added under it.
fn route_with(
    families: &[&dyn VerbFamily],
    nested: &[&dyn nested::NestedVerbs],
    matches: &ArgMatches,
    setup: impl FnOnce(VerbLogClass),
    gate: impl FnOnce() -> Option<ExitCode>,
) -> Option<ExitCode> {
    let (class, needs_identity, run): (VerbLogClass, bool, Box<dyn FnOnce() -> ExitCode + '_>) =
        if let Some((verbs, sub)) = nested::find(nested, matches) {
            (
                verbs.log_class(sub),
                verbs.needs_identity(sub),
                Box::new(move || verbs.run(sub)),
            )
        } else {
            let (family, sub) = find(families, matches)?;
            (
                family.log_class(sub),
                family.needs_identity(sub),
                Box::new(move || family.run(sub)),
            )
        };
    setup(class);
    if needs_identity {
        if let Some(refused) = gate() {
            return Some(refused);
        }
    }
    Some(run())
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
        fn command(&self) -> Command {
            derived_verb::<DummyCmd>()
                .name(self.name)
                .aliases(self.aliases)
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

    /// A family whose command is whatever `command` makes: drives every way a
    /// family's verb can be named for something else.
    struct Commands {
        name: &'static str,
        command: fn() -> Command,
    }
    impl VerbFamily for Commands {
        fn name(&self) -> &'static str {
            self.name
        }
        fn listed_after(&self) -> Option<&'static str> {
            None
        }
        fn command(&self) -> Command {
            (self.command)()
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

    /// A derived enum with two top-level verbs: not one family.
    /// (This doc comment is what the derive copies onto a root's about.)
    #[derive(Subcommand)]
    enum TwoVerbs {
        /// One
        Mine,
        /// Two
        Extra,
    }

    /// A derived enum with no verb at all.
    #[derive(Subcommand)]
    enum NoVerb {}

    /// A doc-commented one-variant enum: the derive would retitle a root with
    /// this text if it were handed one.
    #[derive(Subcommand)]
    enum Titled {
        /// A titled family
        Titled,
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

    /// What [`route`] did: the log class `setup` got, how often the gate ran,
    /// and the exit code.
    fn routed(
        families: &[&dyn VerbFamily],
        argv: &[&str],
        refuse: Option<u8>,
    ) -> (Option<VerbLogClass>, usize, Option<i64>) {
        let matches = with(families).try_get_matches_from(argv).expect("parse");
        let mut class = None;
        let mut gate_runs = 0;
        let code = route(
            families,
            &matches,
            |c| class = Some(c),
            || {
                gate_runs += 1;
                refuse.map(ExitCode::from)
            },
        );
        (
            class,
            gate_runs,
            code.map(crate::telemetry::exit_code_number),
        )
    }

    #[test]
    fn main_routes_a_registered_family_to_its_handler() {
        let (class, gate_runs, code) =
            routed(&[&DUMMY], &["cerulion", "dummy", "echo", "routed"], Some(7));
        assert_eq!(class, Some(VerbLogClass::OneShot));
        // The dummy needs no identity, so even a refusing gate is never asked.
        assert_eq!(gate_runs, 0);
        assert_eq!(code, Some(42));
        assert!(DUMMY_RUNS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains(&"routed".to_string()));
        let (class, _, code) = routed(&[&DUMMY], &["cerulion", "dummy", "wait"], None);
        assert_eq!(class, Some(VerbLogClass::LongRunning));
        assert_eq!(code, Some(43));
    }

    /// A nested verb `probe` added under the dummy family: returns 44 and
    /// needs an identity, so a route through the family's handler (42, 43,
    /// or FAILURE for a verb it does not know, with no gate) is told apart.
    struct UnderDummy;
    impl nested::NestedVerbs for UnderDummy {
        fn parent(&self) -> &'static str {
            "dummy"
        }
        fn commands(&self) -> Vec<Command> {
            vec![Command::new("probe")]
        }
        fn log_class(&self, _: &ArgMatches) -> VerbLogClass {
            VerbLogClass::LongRunning
        }
        fn needs_identity(&self, _: &ArgMatches) -> bool {
            true
        }
        fn run(&self, matches: &ArgMatches) -> ExitCode {
            match matches.subcommand_name() {
                Some("probe") => ExitCode::from(44),
                _ => ExitCode::FAILURE,
            }
        }
    }

    #[test]
    fn a_nested_verb_under_a_registered_family_reaches_its_own_handler() {
        let families: [&dyn VerbFamily; 1] = [&DUMMY];
        let sets: [&dyn nested::NestedVerbs; 1] = [&UnderDummy];
        let cmd = nested::attach(with(&families), &sets).expect("nested verbs attach");
        let route = |argv: &[&str]| {
            let matches = cmd.clone().try_get_matches_from(argv).expect("parse");
            let mut class = None;
            let mut gate_runs = 0;
            let code = route_with(
                &families,
                &sets,
                &matches,
                |c| class = Some(c),
                || {
                    gate_runs += 1;
                    None
                },
            );
            (
                class,
                gate_runs,
                code.map(crate::telemetry::exit_code_number),
            )
        };
        assert_eq!(
            route(&["cerulion", "dummy", "probe"]),
            (Some(VerbLogClass::LongRunning), 1, Some(44))
        );
        // The family's own verbs still reach the family.
        assert_eq!(
            route(&["cerulion", "dummy", "wait"]),
            (Some(VerbLogClass::LongRunning), 0, Some(43))
        );
    }

    #[test]
    fn main_runs_the_gate_before_a_family_that_needs_an_identity() {
        let gated = Dummy {
            gated: true,
            ..dummy("dummy")
        };
        // The gate passes: the family runs.
        let (class, gate_runs, code) = routed(&[&gated], &["cerulion", "dummy", "wait"], None);
        assert_eq!(
            (class, gate_runs, code),
            (Some(VerbLogClass::LongRunning), 1, Some(43))
        );
        // The gate refuses: its code is the exit code and the family never runs.
        let (class, gate_runs, code) = routed(
            &[&gated],
            &["cerulion", "dummy", "echo", "refused-word"],
            Some(7),
        );
        assert_eq!(
            (class, gate_runs, code),
            (Some(VerbLogClass::OneShot), 1, Some(7))
        );
        assert!(!DUMMY_RUNS
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains(&"refused-word".to_string()));
    }

    #[test]
    fn main_leaves_a_built_in_verb_to_the_generic_dispatch() {
        let (class, gate_runs, code) = routed(&[&DUMMY], &["cerulion", "graph", "list"], Some(7));
        assert_eq!((class, gate_runs, code), (None, 0, None));
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

    /// The verbs from `from` on, in listing order, `len` of them.
    fn listed_from(families: &[&dyn VerbFamily], from: &str, len: usize) -> Vec<String> {
        let verbs = listing(&mut with(families));
        let at = verbs.iter().position(|v| v == from).expect("anchor listed");
        verbs[at..at + len].to_vec()
    }

    #[test]
    fn a_family_follows_its_own_anchor_not_every_earlier_family() {
        let after = |name, anchor| Dummy {
            after: Some(anchor),
            ..dummy(name)
        };
        let (a, b, c) = (after("a", "topic"), after("b", "topic"), after("c", "a"));
        // `c` sits directly behind its anchor `a`, ahead of `b`, which shares
        // only the `topic` anchor with `a`.
        assert_eq!(
            listed_from(&[&a, &b, &c], "topic", 5),
            ["topic", "a", "c", "b", "ros2"]
        );
        // Registered the other way round: `b` goes behind `a` and the family
        // listed after `a`, the whole group already behind `topic`.
        assert_eq!(
            listed_from(&[&a, &c, &b], "topic", 5),
            ["topic", "a", "c", "b", "ros2"]
        );
        // A family behind a family behind an anchor: the chain is followed.
        let d = after("d", "c");
        assert_eq!(
            listed_from(&[&a, &c, &d, &b], "topic", 6),
            ["topic", "a", "c", "d", "b", "ros2"]
        );
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
    fn a_family_whose_enum_has_no_verb_is_refused() {
        let none = Commands {
            name: "mine",
            command: derived_verb::<NoVerb>,
        };
        let err = build(Cli::command(), &[&none]).expect_err("no verb");
        assert_eq!(
            err,
            RegistryError::WrongVerb {
                family: "mine",
                verb: String::new()
            }
        );
    }

    #[test]
    fn a_family_whose_verb_has_another_name_is_refused() {
        let other = Commands {
            name: "mine",
            command: || Command::new("other"),
        };
        let err = build(Cli::command(), &[&other]).expect_err("wrong verb");
        assert_eq!(
            err,
            RegistryError::WrongVerb {
                family: "mine",
                verb: "other".into()
            }
        );
        assert_eq!(
            err.to_string(),
            "the verb family `mine` defines the command `other`; its command must be named \
             `mine`, with exactly one variant in its derived `Subcommand` enum"
        );
    }

    #[test]
    fn a_family_whose_enum_has_two_verbs_is_refused() {
        // Neither verb is taken: the family is refused, not half registered.
        let two = Commands {
            name: "mine",
            command: derived_verb::<TwoVerbs>,
        };
        let err = build(Cli::command(), &[&two]).expect_err("two verbs");
        assert_eq!(
            err,
            RegistryError::WrongVerb {
                family: "mine",
                verb: String::new()
            }
        );
    }

    /// Everything a reader of `cerulion` sees of a command other than the
    /// listing order: the help of the root and of every built-in verb, each
    /// verb's aliases and visibility, and the root's own arguments.
    fn built_ins(cmd: &Command) -> Vec<String> {
        let mut cmd = cmd.clone().disable_help_subcommand(true);
        cmd.build();
        let mut out = vec![
            format!("{:?}", cmd.get_about()),
            format!("{:?}", cmd.get_long_about()),
        ];
        out.extend(cmd.get_arguments().map(|a| format!("{a:?}")));
        let base: BTreeSet<String> = Cli::command()
            .get_subcommands()
            .map(|c| c.get_name().to_string())
            .collect();
        for sub in cmd.get_subcommands_mut() {
            if !base.contains(sub.get_name()) {
                continue;
            }
            let head = format!(
                "{} {:?} hidden={}",
                sub.get_name(),
                sub.get_all_aliases().collect::<Vec<_>>(),
                sub.is_hide_set()
            );
            out.push(format!("{head} {}", sub.render_long_help()));
        }
        out
    }

    #[test]
    fn registering_families_leaves_every_built_in_verb_and_root_arg_as_it_was() {
        // A family hands over its verb and never sees the root, so it has no
        // way to hide, rename or strip a built-in verb or a root argument.
        let titled = Commands {
            name: "titled",
            command: derived_verb::<Titled>,
        };
        let aliased = Dummy {
            aliases: &["dm"],
            after: Some("topic"),
            ..dummy("dummy")
        };
        let cmd = with(&[&aliased, &titled]);
        assert_eq!(built_ins(&cmd), built_ins(&Cli::command()));
    }

    #[test]
    fn a_doc_commented_family_enum_leaves_the_root_help_alone() {
        let titled = Commands {
            name: "titled",
            command: derived_verb::<Titled>,
        };
        let mut cmd = with(&[&titled]);
        let mut plain = Cli::command();
        assert_eq!(cmd.get_about(), plain.get_about());
        let help = cmd.render_help().to_string();
        assert!(help.contains("  titled       A titled family\n"), "{help}");
        assert!(help.starts_with(
            plain
                .render_help()
                .to_string()
                .lines()
                .next()
                .unwrap_or_default()
        ));
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

    /// A run that stopped a node on a fatal error exits 3, in process or in
    /// a worker group; every other error exits 1.
    #[test]
    fn a_fatal_node_exits_3_and_every_other_error_exits_1() {
        let in_process = CliError::NodeFatal {
            nodes: vec![("lidar".to_string(), "Fatal error: lost".to_string())],
        };
        let worker = CliError::WorkerNodeFatal {
            groups: vec!["g0".to_string()],
        };
        assert_eq!(cli_error_exit_code(&in_process), 3);
        assert_eq!(cli_error_exit_code(&worker), 3);
        assert_eq!(
            cli_error_exit_code(&CliError::Validation("x".to_string())),
            1
        );
    }
}
