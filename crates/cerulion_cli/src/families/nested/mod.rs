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
//! is not a verb, a verb with no name, and a verb name that already selects a
//! verb of the parent (a built-in name or alias, clap's `help`, an earlier
//! set's verb) are refused with a [`NestedError`]. A set hands over its verbs
//! ([`NestedVerbs::commands`]) and never sees the parent, so it cannot change
//! a built-in verb or the parent's help text.

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

    /// The verbs, which [`attach`] adds under the parent. [`derived_verbs`]
    /// makes them from a derived `Subcommand` enum.
    fn commands(&self) -> Vec<Command>;

    /// The default tracing filter class of the parsed invocation.
    fn log_class(&self, matches: &ArgMatches) -> VerbLogClass;

    /// Whether the login gate runs before [`NestedVerbs::run`].
    fn needs_identity(&self, matches: &ArgMatches) -> bool;

    /// Run the parsed invocation and return the process exit code.
    fn run(&self, matches: &ArgMatches) -> ExitCode;
}

/// Every registered nested-verb set. Adding a set is one line here.
pub(crate) static NESTED: &[&dyn NestedVerbs] = &[&topic_introspect::TopicIntrospect];

/// Every verb a derived `Subcommand` enum `S` defines, for
/// [`NestedVerbs::commands`]. The derive adds them to a scratch command that
/// is then dropped, so the enum's doc comment never reaches the parent.
pub(crate) fn derived_verbs<S: clap::Subcommand>() -> Vec<Command> {
    S::augment_subcommands(Command::new("scratch"))
        .get_subcommands()
        .cloned()
        .collect()
}

/// Why a set of nested verbs cannot be registered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum NestedError {
    /// The parent is not a verb of the command.
    UnknownParent(&'static str),
    /// A set hands over a verb with an empty name.
    Unnamed(&'static str),
    /// A verb's name or alias already selects a verb of the parent.
    NameTaken { parent: &'static str, verb: String },
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
            NestedError::Unnamed(parent) => {
                write!(
                    f,
                    "a nested verb set under `{parent}` hands over a verb with no name"
                )
            }
            NestedError::NameTaken { parent, verb } => write!(
                f,
                "the nested verb `{parent} {verb}` has the name or alias of a verb `{parent}` \
                 already has (or clap's `help`); rename it"
            ),
        }
    }
}

impl std::error::Error for NestedError {}

/// Add every set in `sets` under its parent in `root`.
pub(crate) fn attach(root: Command, sets: &[&dyn NestedVerbs]) -> Result<Command, NestedError> {
    let mut root = root;
    for set in sets {
        let parent = set.parent();
        let Some(existing) = root.find_subcommand(parent) else {
            return Err(NestedError::UnknownParent(parent));
        };
        // Every word that already selects a verb of the parent: names,
        // aliases and clap's implicit `help`.
        let mut taken: Vec<String> = existing
            .get_subcommands()
            .flat_map(|c| std::iter::once(c.get_name()).chain(c.get_all_aliases()))
            .chain(std::iter::once("help"))
            .map(str::to_string)
            .collect();
        let verbs = set.commands();
        for verb in &verbs {
            if verb.get_name().is_empty() {
                return Err(NestedError::Unnamed(parent));
            }
            for word in std::iter::once(verb.get_name()).chain(verb.get_all_aliases()) {
                if taken.iter().any(|t| t == word) {
                    return Err(NestedError::NameTaken {
                        parent,
                        verb: word.to_string(),
                    });
                }
                taken.push(word.to_string());
            }
        }
        root = root.mut_subcommand(parent, |cmd| cmd.subcommands(verbs));
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
        .find(|s| s.parent() == parent && s.commands().iter().any(|c| c.get_name() == verb))
        .map(|s| (*s, sub))
}

#[cfg(test)]
mod tests {
    use super::*;

    use clap::{CommandFactory, Subcommand};

    use crate::cli::Cli;

    /// A test-only set under `topic`: `probe [word]`, which returns 42 with a
    /// word and 41 without, and `watch`, a long-running verb returning 43.
    struct Probe {
        parent: &'static str,
        gated: bool,
        verbs: fn() -> Vec<Command>,
    }

    fn probe_and_watch() -> Vec<Command> {
        vec![
            Command::new("probe").arg(clap::Arg::new("word")),
            Command::new("watch"),
        ]
    }

    const PROBE: Probe = Probe {
        parent: "topic",
        gated: false,
        verbs: probe_and_watch,
    };

    impl NestedVerbs for Probe {
        fn parent(&self) -> &'static str {
            self.parent
        }
        fn commands(&self) -> Vec<Command> {
            (self.verbs)()
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

    type RouteRecord = (Option<VerbLogClass>, usize, Option<i64>);

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

    fn refusal(set: &Probe) -> NestedError {
        attach(Cli::command(), &[set]).unwrap_err()
    }

    #[test]
    fn the_registered_nested_verbs_attach() {
        let root = super::super::build(Cli::command(), super::super::FAMILIES).expect("families");
        attach(root, NESTED).expect("the production nested-verb registry attaches");
    }

    #[test]
    fn a_nested_verb_is_listed_under_its_parent_beside_the_built_in_verbs() {
        let mut cmd = with(&[&PROBE]);
        let help = cmd
            .find_subcommand_mut("topic")
            .expect("topic")
            .render_help()
            .to_string();
        let listed: Vec<&str> = help
            .lines()
            .skip_while(|l| *l != "Commands:")
            .skip(1)
            .take_while(|l| !l.is_empty())
            .filter_map(|l| l.split_whitespace().next())
            .collect();
        assert_eq!(
            listed,
            ["list", "info", "echo", "hz", "probe", "watch", "help"]
        );
    }

    #[test]
    fn a_derived_enum_hands_over_its_verbs_without_its_doc_comment() {
        /// This doc comment would become the parent's about if it reached it.
        #[derive(Subcommand)]
        enum Derived {
            /// Probe a topic
            Probe,
            /// Watch a topic
            Watch,
        }
        struct FromDerive;
        impl NestedVerbs for FromDerive {
            fn parent(&self) -> &'static str {
                "topic"
            }
            fn commands(&self) -> Vec<Command> {
                derived_verbs::<Derived>()
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
        let names: Vec<String> = derived_verbs::<Derived>()
            .iter()
            .map(|c| c.get_name().to_string())
            .collect();
        assert_eq!(names, ["probe", "watch"]);
        let cmd = with(&[&FromDerive]);
        let topic = cmd.find_subcommand("topic").expect("topic");
        assert_eq!(
            topic.get_about().map(ToString::to_string).as_deref(),
            Some("Topic introspection")
        );
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
        // The gate refuses: its code is the exit code and the verb never runs.
        assert_eq!(
            routed(&[&gated], &["cerulion", "topic", "probe", "w"], Some(7)),
            (Some(VerbLogClass::OneShot), 1, Some(7))
        );
    }

    #[test]
    fn a_built_in_verb_stays_with_the_generic_dispatch() {
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
        assert_eq!(refusal(&orphan), NestedError::UnknownParent("nope"));
    }

    #[test]
    fn a_verb_with_no_name_is_refused() {
        let unnamed = Probe {
            verbs: || vec![Command::new("")],
            ..PROBE
        };
        assert_eq!(refusal(&unnamed), NestedError::Unnamed("topic"));
    }

    #[test]
    fn a_verb_named_like_a_parent_verb_or_help_is_refused() {
        let clashes: [fn() -> Vec<Command>; 4] = [
            || vec![Command::new("echo")],
            || vec![Command::new("help")],
            || vec![Command::new("probe").alias("hz")],
            || vec![Command::new("probe"), Command::new("probe")],
        ];
        for (verbs, word) in clashes.into_iter().zip(["echo", "help", "hz", "probe"]) {
            let clash = Probe { verbs, ..PROBE };
            assert_eq!(
                refusal(&clash),
                NestedError::NameTaken {
                    parent: "topic",
                    verb: word.to_string()
                }
            );
        }
        // An earlier set's verb is taken too.
        assert_eq!(
            attach(Cli::command(), &[&PROBE, &PROBE]).unwrap_err(),
            NestedError::NameTaken {
                parent: "topic",
                verb: "probe".to_string()
            }
        );
    }
}
