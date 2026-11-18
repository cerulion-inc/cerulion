// SPDX-License-Identifier: AGPL-3.0-only
//! Which packages ONE workflow command line runs tests for.
//!
//! `ci_test_coverage_test` (is every package with tests run by some step?) and
//! `ci_doc_pin_walk_test` (is every doc-reading binary's package run beside its
//! marker?) ask the same question of the same workflow lines, and each used to
//! carry its own copy of the answer. The reader lives here once, so a runner
//! form one gate learns is a form the other reads too.

/// Does ONE workflow line run the tests of `pkg`?
///
/// The line is split into shell commands at whole-token `&&`, `||`, `|` and
/// `;`, and each command is read on its own. Three forms credit a package:
///
///   * `cargo test … -p a -p b …`: every `-p` / `--package` occurrence counts,
///     in every spelling cargo accepts (`-p x`, `-px`, `--package x`,
///     `--package=x`), so the `b` of `cargo test -p a -p b` is covered too;
///   * `cargo nextest run …`, with the same package flags. Only `run` executes
///     tests: `cargo nextest list` and `cargo nextest archive` do not;
///   * `ci_test_shard.sh <pkg> …`, the sharded form, which takes the package
///     name as its first argument so the name stays literally in the workflow.
///
/// A cargo command whose own flags carry `--doc` or `--no-run` credits nothing.
/// `--doc` runs only the doctests, and `--no-run` runs no test at all, so a step
/// of either kind left standing after the package's real test step was deleted
/// would keep a coverage gate green with none of that package's unit or
/// integration tests run. Flags are read up to a bare `--`: what follows it goes
/// to the test binaries, not to cargo.
///
/// Matching is by WHOLE WHITESPACE TOKEN, which is what keeps `cerulion_viz`
/// and `cerulion_vizd` apart: a plain `contains` would report `cerulion_viz`
/// covered by the `cerulion_vizd` step and vice versa.
pub fn line_runs_tests_of(line: &str, pkg: &str) -> bool {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    tokens
        .split(|t| matches!(*t, "&&" | "||" | "|" | ";"))
        .any(|command| command_runs_tests_of(command, pkg))
}

/// [`line_runs_tests_of`] for one shell command, already split into tokens.
fn command_runs_tests_of(command: &[&str], pkg: &str) -> bool {
    if let Some(at) = command.iter().position(|t| t.ends_with("ci_test_shard.sh")) {
        if command.get(at + 1) == Some(&pkg) {
            return true;
        }
    }
    let Some(flags) = cargo_test_flags(command) else {
        return false;
    };
    if flags.iter().any(|t| matches!(*t, "--doc" | "--no-run")) {
        return false;
    }
    let mut it = flags.iter();
    while let Some(tok) = it.next() {
        let named = match *tok {
            "-p" | "--package" => it.next() == Some(&pkg),
            _ => tok
                .strip_prefix("--package=")
                .or_else(|| tok.strip_prefix("-p"))
                .is_some_and(|rest| rest == pkg),
        };
        if named {
            return true;
        }
    }
    false
}

/// The cargo flags of a `cargo test` or `cargo nextest run` command: the tokens
/// after the subcommand, up to a bare `--`. `None` when the command runs neither.
fn cargo_test_flags<'a>(command: &'a [&'a str]) -> Option<&'a [&'a str]> {
    let flags = (0..command.len()).find_map(|at| match &command[at..] {
        ["cargo", "test", flags @ ..] => Some(flags),
        ["cargo", "nextest", "run", flags @ ..] => Some(flags),
        _ => None,
    })?;
    let end = flags.iter().position(|t| *t == "--").unwrap_or(flags.len());
    Some(&flags[..end])
}
