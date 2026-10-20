// SPDX-License-Identifier: AGPL-3.0-only
//! Nested verbs: new `cerulion <parent> <verb>` verbs added under a verb that
//! already exists (a built-in `Commands` verb or a registered family), each
//! set in its own module instead of the parent's action enum.
//!
//! A set owns its clap definition, log class, login-gate rule and dispatch,
//! exactly like a [`VerbFamily`](super::VerbFamily) does for a top-level
//! verb, and joins the binary through ONE line in [`NESTED`]. Its module
//! declares `const PARENT: &str = "<parent>";` so the docs gate files its
//! verbs under the parent.
//!
//! [`attach`] checks the registry when the command is built: a parent that
//! is not a verb, a verb name that already selects a verb of the parent (a
//! built-in name or alias, clap's `help`, an earlier set's verb), and an
//! `augment` that does anything but add exactly its named verbs (or that
//! changes the parent's help text) are refused with a [`NestedError`].

use std::collections::BTreeSet;
use std::fmt;
use std::process::ExitCode;

use clap::{ArgMatches, Command};

use crate::cli::VerbLogClass;

mod topic_introspect;

/// A set of verbs added under one existing verb. See the module docs.
///
/// Every method that takes `matches` receives the PARENT's sub-matches
/// (`cerulion <parent> ...`), so `matches.subcommand()` names the verb.
pub(crate) trait NestedVerbs: Sync {
    /// The verb these verbs are added under, as typed.
    fn parent(&self) -> &'static str;

    /// The verbs this set adds, as typed.
    fn names(&self) -> &'static [&'static str];

    /// Add exactly [`NestedVerbs::names`] to the parent's command.
    fn augment(&self, parent: Command) -> Command;

    /// The default tracing filter class of the parsed invocation.
    fn log_class(&self, matches: &ArgMatches) -> VerbLogClass;

    /// Whether the login gate runs before [`NestedVerbs::run`].
    fn needs_identity(&self, matches: &ArgMatches) -> bool;

    /// Run the parsed invocation and return the process exit code.
    fn run(&self, matches: &ArgMatches) -> ExitCode;
}

/// Every registered nested-verb set. Adding a set is one line here.
pub(crate) static NESTED: &[&dyn NestedVerbs] = &[&topic_introspect::TopicIntrospect];

/// Why a set of nested verbs cannot be registered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum NestedError {
    /// The parent is not a verb of the command.
    UnknownParent(&'static str),
    /// A verb's name already selects a verb of the parent.
    NameTaken {
        parent: &'static str,
        verb: &'static str,
    },
    /// A set's `augment` did not add exactly its named verbs.
    AugmentMismatch(&'static str),
}

impl fmt::Display for NestedError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NestedError::UnknownParent(parent) => {
                write!(
                    f,
                    "nested verbs are added under `{parent}`, which is not a verb"
                )
            }
            NestedError::NameTaken { parent, verb } => write!(
                f,
                "the nested verb `{parent} {verb}` has the name or alias of a verb `{parent}` \
                 already has (or clap's `help`); rename it"
            ),
            NestedError::AugmentMismatch(parent) => write!(
                f,
                "a nested verb set under `{parent}` must add exactly the verbs it names and \
                 leave every other subcommand as it was"
            ),
        }
    }
}

impl std::error::Error for NestedError {}

/// The words that select a subcommand of `cmd`: names, aliases and `help`.
fn taken_words(cmd: &Command) -> BTreeSet<String> {
    cmd.get_subcommands()
        .flat_map(|c| std::iter::once(c.get_name()).chain(c.get_all_aliases()))
        .chain(std::iter::once("help"))
        .map(str::to_string)
        .collect()
}

/// Add every set in `sets` under its parent in `root`.
pub(crate) fn attach(root: Command, sets: &[&dyn NestedVerbs]) -> Result<Command, NestedError> {
    let mut root = root;
    for set in sets {
        let parent = set.parent();
        let Some(before) = root.find_subcommand(parent) else {
            return Err(NestedError::UnknownParent(parent));
        };
        let taken = taken_words(before);
        if let Some(verb) = set.names().iter().find(|v| taken.contains(**v)) {
            return Err(NestedError::NameTaken { parent, verb });
        }
        let mut expected: BTreeSet<String> = before
            .get_subcommands()
            .map(|c| c.get_name().to_string())
            .collect();
        expected.extend(set.names().iter().map(|v| v.to_string()));
        let text = |c: &Command| {
            (
                c.get_about().map(ToString::to_string),
                c.get_long_about().map(ToString::to_string),
            )
        };
        let text_before = text(before);
        let mut mismatch = false;
        root = root.mut_subcommand(parent, |cmd| {
            let cmd = set.augment(cmd);
            let added = cmd
                .get_subcommands()
                .map(|c| c.get_name().to_string())
                .collect::<BTreeSet<_>>();
            // A doc comment on a derived `Subcommand` enum rewrites the
            // parent's help text, so that counts as a mismatch too.
            mismatch = added != expected || text(&cmd) != text_before;
            cmd
        });
        if mismatch {
            return Err(NestedError::AugmentMismatch(parent));
        }
    }
    Ok(root)
}

/// The registered set that owns the nested verb `matches` selected, with the
/// parent's sub-matches. `None` for any other verb.
pub(crate) fn find<'a>(
    sets: &[&'a dyn NestedVerbs],
    matches: &'a ArgMatches,
) -> Option<(&'a dyn NestedVerbs, &'a ArgMatches)> {
    let (parent, sub) = matches.subcommand()?;
    let verb = sub.subcommand_name()?;
    sets.iter()
        .find(|s| s.parent() == parent && s.names().contains(&verb))
        .map(|s| (*s, sub))
}

#[cfg(test)]
mod tests {
    use super::*;

    use clap::CommandFactory;

    use crate::cli::Cli;

    /// A test-only set: `cerulion topic probe`, which records nothing and
    /// returns a fixed code, and `cerulion topic watch`, a long-running verb.
    struct Probe {
        parent: &'static str,
        names: &'static [&'static str],
        gated: bool,
        augment: fn(Command) -> Command,
    }

    fn add_probe_and_watch(parent: Command) -> Command {
        parent
            .subcommand(Command::new("probe").arg(clap::Arg::new("word")))
            .subcommand(Command::new("watch"))
    }

    const PROBE: Probe = Probe {
        parent: "topic",
        names: &["probe", "watch"],
        gated: false,
        augment: add_probe_and_watch,
    };

    impl NestedVerbs for Probe {
        fn parent(&self) -> &'static str {
            self.parent
        }
        fn names(&self) -> &'static [&'static str] {
            self.names
        }
        fn augment(&self, parent: Command) -> Command {
            (self.augment)(parent)
        }
        fn log_class(&self, matches: &ArgMatches) -> VerbLogClass {
            match matches.subcommand_name() {
                Some("watch") => VerbLogClass::LongRunning,
                _ => VerbLogClass::OneShot,
            }
        }
        fn needs_identity(&self, _matches: &ArgMatches) -> bool {
            self.gated
        }
        fn run(&self, matches: &ArgMatches) -> ExitCode {
            match matches.subcommand() {
                Some(("probe", sub)) if sub.get_one::<String>("word").is_some() => {
                    ExitCode::from(42)
                }
                Some(("probe", _)) => ExitCode::from(41),
                _ => ExitCode::from(43),
            }
        }
    }

    fn with(sets: &[&dyn NestedVerbs]) -> Command {
        attach(Cli::command(), sets).expect("nested verbs attach")
    }

    fn routed(sets: &[&dyn NestedVerbs], argv: &[&str], refuse: Option<u8>) -> RouteRecord {
        let matches = with(sets).try_get_matches_from(argv).expect("parse");
        let mut class = None;
        let mut gate_runs = 0;
        let code = super::super::route_with(
            &[],
            sets,
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

    type RouteRecord = (Option<VerbLogClass>, usize, Option<i64>);

    #[test]
    fn the_registered_nested_verbs_attach() {
        let root = super::super::build(Cli::command(), super::super::FAMILIES).expect("families");
        attach(root, NESTED).expect("the production nested-verb registry attaches");
    }

    #[test]
    fn a_nested_verb_parses_under_its_parent_beside_the_built_in_verbs() {
        let mut cmd = with(&[&PROBE]);
        let help = cmd
            .find_subcommand_mut("topic")
            .expect("topic")
            .render_help()
            .to_string();
        for verb in ["list", "info", "echo", "hz", "probe", "watch"] {
            assert!(
                help.lines()
                    .any(|l| l.split_whitespace().next() == Some(verb)),
                "{verb} missing from:\n{help}"
            );
        }
    }

    #[test]
    fn main_routes_a_nested_verb_to_its_set_with_the_parent_matches() {
        assert_eq!(
            routed(&[&PROBE], &["cerulion", "topic", "probe", "w"], Some(7)),
            (Some(VerbLogClass::OneShot), 0, Some(42))
        );
        assert_eq!(
            routed(&[&PROBE], &["cerulion", "topic", "watch"], None),
            (Some(VerbLogClass::LongRunning), 0, Some(43))
        );
    }

    #[test]
    fn main_runs_the_gate_before_a_nested_verb_that_needs_an_identity() {
        let gated = Probe {
            gated: true,
            ..PROBE
        };
        assert_eq!(
            routed(&[&gated], &["cerulion", "topic", "probe"], None),
            (Some(VerbLogClass::OneShot), 1, Some(41))
        );
        assert_eq!(
            routed(&[&gated], &["cerulion", "topic", "probe", "w"], Some(7)),
            (Some(VerbLogClass::OneShot), 1, Some(7))
        );
    }

    #[test]
    fn a_built_in_verb_of_the_parent_stays_with_the_generic_dispatch() {
        assert_eq!(
            routed(&[&PROBE], &["cerulion", "topic", "list"], Some(7)),
            (None, 0, None)
        );
        assert_eq!(
            routed(&[&PROBE], &["cerulion", "graph", "list"], Some(7)),
            (None, 0, None)
        );
    }

    #[test]
    fn a_set_under_a_missing_parent_is_refused() {
        let orphan = Probe {
            parent: "nope",
            ..PROBE
        };
        assert_eq!(
            attach(Cli::command(), &[&orphan]).unwrap_err(),
            NestedError::UnknownParent("nope")
        );
    }

    #[test]
    fn a_verb_named_like_a_parent_verb_or_help_is_refused() {
        let clashes: [&'static [&'static str]; 2] = [&["echo"], &["help"]];
        for names in clashes {
            let verb = names[0];
            let clash = Probe { names, ..PROBE };
            assert_eq!(
                attach(Cli::command(), &[&clash]).unwrap_err(),
                NestedError::NameTaken {
                    parent: "topic",
                    verb
                }
            );
        }
        // An earlier set's verb is taken too.
        assert_eq!(
            attach(Cli::command(), &[&PROBE, &PROBE]).unwrap_err(),
            NestedError::NameTaken {
                parent: "topic",
                verb: "probe"
            }
        );
    }

    #[test]
    fn an_augment_that_adds_other_verbs_than_it_names_is_refused() {
        fn adds_only_probe(parent: Command) -> Command {
            parent.subcommand(Command::new("probe"))
        }
        fn drops_a_built_in(parent: Command) -> Command {
            let kept: Vec<Command> = parent
                .get_subcommands()
                .filter(|c| c.get_name() != "hz")
                .cloned()
                .collect();
            add_probe_and_watch(Command::new("topic").subcommands(kept))
        }
        fn retitles_the_parent(parent: Command) -> Command {
            add_probe_and_watch(parent.about("not topic"))
        }
        for augment in [
            adds_only_probe as fn(Command) -> Command,
            drops_a_built_in,
            retitles_the_parent,
        ] {
            let wrong = Probe { augment, ..PROBE };
            assert_eq!(
                attach(Cli::command(), &[&wrong]).unwrap_err(),
                NestedError::AugmentMismatch("topic")
            );
        }
    }
}
