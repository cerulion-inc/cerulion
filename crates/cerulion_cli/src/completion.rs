// SPDX-License-Identifier: AGPL-3.0-only
//! The clap wiring for shell tab-completion.
//!
//! Two halves meet here.
//!
//! **The clap tree completes itself.** `clap_complete`'s dynamic engine walks
//! the very [`Cli`](crate::cli::Cli) command the binary parses with, so every
//! subcommand, flag, `ValueEnum` value and `value_parser` possible-value is
//! completable with no per-arg work. That is why there is no generated static
//! script anywhere in this crate: a static script is a frozen snapshot of the
//! tree that also cannot complete a single live name, and sourcing both
//! mechanisms would leave whichever was loaded last in charge. One mechanism,
//! and it is the one that can answer `cerulion topic hz <TAB>`.
//!
//! **Live values come from the engine.** [`cerulion_cli_engine::completions`]
//! owns every source and the wall-clock budget around it; this module is the
//! thin adapter that turns its clap-free `Candidate` into a
//! [`CompletionCandidate`]. The engine crate does not depend on clap, and
//! keeping it that way is what makes the sources oracle-testable against plain
//! `Vec<Candidate>` values.
//!
//! # Registration
//!
//! `cerulion completions <shell>` emits the shell code for the
//! `COMPLETE=<shell> cerulion` protocol. The generated snippet calls THIS
//! binary back on each TAB with the words typed so far.

use std::io::Write;

use clap_complete::engine::CompletionCandidate;

use cerulion_cli_engine::completions;

/// Map the engine's clap-free candidates onto clap's.
fn map(candidates: Vec<completions::Candidate>) -> Vec<CompletionCandidate> {
    candidates
        .into_iter()
        .map(|c| {
            let candidate = CompletionCandidate::new(c.value);
            match c.help {
                Some(help) => candidate.help(Some(help.into())),
                None => candidate,
            }
        })
        .collect()
}

/// Topics visible on this machine — local producers and `cerulion-netd`
/// mirrors of remote robots alike (they are indistinguishable from the local
/// service directory, and telling them apart is unaffordable here — see the
/// engine module's budget section).
pub fn topics() -> Vec<CompletionCandidate> {
    map(completions::complete_topic_names())
}

/// Node type names in the workspace containing the cwd.
pub fn node_types() -> Vec<CompletionCandidate> {
    map(completions::complete_node_types())
}

/// The accepted `bag play --resim` selections.
///
/// A constant, not a source — `all` is the only value that resolves today, and
/// the argument stays a free-form `String` so a node subset reaches the engine's
/// refusal (which states the restriction and the spelling that works) rather than clap's
/// "invalid value". Reads nothing: no bag is open at TAB time, and the bag path
/// may not even be typed yet.
pub fn resim_selections() -> Vec<CompletionCandidate> {
    vec![CompletionCandidate::new("all")
        .help(Some("re-execute every node in the bag's graph".into()))]
}

/// Graph names in the workspace containing the cwd.
pub fn graph_names() -> Vec<CompletionCandidate> {
    map(completions::complete_graph_names())
}

/// Workspace schemas plus every built-in ROS 2 type.
pub fn schema_names() -> Vec<CompletionCandidate> {
    map(completions::complete_schema_names())
}

/// Robots this desk has met: paired (`~/.cerulion/robots.toml`) or seen on the
/// LAN (`~/.cerulion/peers.json`, within its 7-day TTL).
pub fn robot_names() -> Vec<CompletionCandidate> {
    map(completions::complete_robot_names())
}

/// The shells `cerulion completions` can emit registration code for.
///
/// A `ValueEnum` rather than a free `String` so `cerulion completions <TAB>`
/// completes the shell name itself and a typo is a clap error naming the valid
/// set — instead of a runtime "unknown shell" after the fact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, clap::ValueEnum)]
pub enum CompletionShell {
    /// Bash (4.4+).
    Bash,
    /// Elvish.
    Elvish,
    /// Fish.
    Fish,
    /// PowerShell.
    Powershell,
    /// Zsh.
    Zsh,
}

impl CompletionShell {
    /// The name `clap_complete`'s shell registry knows this shell by. It is
    /// also the value the generated script sets `COMPLETE` to, so the two
    /// halves of the protocol agree by construction.
    fn as_str(self) -> &'static str {
        match self {
            Self::Bash => "bash",
            Self::Elvish => "elvish",
            Self::Fish => "fish",
            Self::Powershell => "powershell",
            Self::Zsh => "zsh",
        }
    }

    /// The line(s) that wire completions into this shell permanently.
    ///
    /// May be MULTI-LINE (zsh is), so callers must render each line rather
    /// than assuming one.
    ///
    /// **zsh needs `compinit` first.** The generated
    /// zsh script ends in `compdef _clap_dynamic_completer_cerulion cerulion`,
    /// and `compdef` is a FUNCTION defined by `compinit` — not a builtin. On a
    /// bare zsh it does not exist: macOS ships a `/etc/zshrc` that never calls
    /// `compinit`, so a user with a stock `~/.zshrc` got `command not found:
    /// compdef` on EVERY shell start and completions silently never worked.
    /// Frameworks like oh-my-zsh do
    /// call it, which is exactly why the bug survives casual testing.
    ///
    /// The guard is `(( $+functions[compdef] ))` — initialise the completion
    /// system only when something else has not already — so the line is
    /// correct on a bare zsh AND does not force a second `compinit` (and its
    /// dump rebuild) on a framework shell.
    pub fn install_hint(self) -> String {
        match self {
            Self::Bash => "echo 'source <(COMPLETE=bash cerulion)' >> ~/.bashrc".to_string(),
            Self::Elvish => {
                "echo 'eval (E:COMPLETE=elvish cerulion | slurp)' >> ~/.elvish/rc.elv".to_string()
            }
            // A file under `completions/` is AUTOLOADED by fish the first time
            // `cerulion` is completed, so the snippet regenerates itself (and
            // costs nothing at shell start). One canonical form — this string
            // is the single source the docs and `--help` both quote.
            Self::Fish => concat!(
                "echo 'COMPLETE=fish cerulion | source' > ",
                "~/.config/fish/completions/cerulion.fish"
            )
            .to_string(),
            Self::Powershell => concat!(
                r#"echo '$env:COMPLETE = "powershell"; cerulion | Out-String | "#,
                r#"Invoke-Expression; Remove-Item Env:\COMPLETE' >> $PROFILE"#
            )
            .to_string(),
            Self::Zsh => concat!(
                "echo 'autoload -Uz compinit && (( $+functions[compdef] )) || compinit' ",
                ">> ~/.zshrc\n",
                "echo 'source <(COMPLETE=zsh cerulion)' >> ~/.zshrc"
            )
            .to_string(),
        }
    }
}

/// Write the registration script for `shell` to `out`.
///
/// The emitted snippet is the DYNAMIC registration: it calls a `cerulion`
/// binary back on every TAB rather than baking a snapshot of the command tree
/// into the file. That is what lets a TAB complete a topic that did not exist
/// when the script was written.
///
/// The callback target is this binary's own absolute path when it can be
/// resolved, falling back to the bare name `cerulion` (found on `PATH`). The
/// absolute path is preferred because it always works for the binary the user
/// just ran — including a `./target/debug/cerulion` that is not installed
/// anywhere. Its cost is that the script goes stale if the binary MOVES, which
/// is exactly why the help text leads with the self-correcting one-liner:
/// `source <(COMPLETE=zsh cerulion)` regenerates on every shell start.
pub fn write_registration_script(
    shell: CompletionShell,
    out: &mut dyn Write,
) -> std::io::Result<()> {
    let name = shell.as_str();
    let shells = clap_complete::env::Shells::builtins();
    let completer = shells
        .completer(name)
        // Unreachable: `CompletionShell`'s variants are exactly the builtin
        // set, and clap rejects anything else before dispatch. Handled rather
        // than unwrapped so a future variant added without a registry entry
        // fails loudly at runtime instead of panicking.
        .ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                format!("no completion generator is registered for shell '{name}'"),
            )
        })?;

    let self_path = std::env::current_exe()
        .ok()
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_else(|| "cerulion".to_string());

    completer.write_registration("COMPLETE", "cerulion", "cerulion", &self_path, out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::ValueEnum;

    #[test]
    fn every_declared_shell_has_a_generator_and_emits_a_registration_script() {
        // Anti-tautology first: the enum must actually carry the shells the
        // docs promise, so a silently-emptied enum cannot pass the loop below.
        let all = CompletionShell::value_variants();
        assert_eq!(all.len(), 5, "expected 5 shells, got {all:?}");

        for shell in all {
            let mut buf = Vec::new();
            write_registration_script(*shell, &mut buf)
                .unwrap_or_else(|e| panic!("{shell:?} must have a generator: {e}"));
            let script = String::from_utf8(buf).expect("script must be UTF-8");

            assert!(
                !script.is_empty(),
                "{shell:?} emitted an empty registration script"
            );
            // The two halves of the protocol: the script must set COMPLETE to
            // the SAME token `CompleteEnv` dispatches on, and must name the
            // command being completed. A mismatch here is the failure mode
            // where TAB silently does nothing.
            assert!(
                script.contains("COMPLETE"),
                "{shell:?} script must set the COMPLETE variable:\n{script}"
            );
            assert!(
                script.contains(shell.as_str()),
                "{shell:?} script must carry its own shell token:\n{script}"
            );
            assert!(
                script.contains("cerulion"),
                "{shell:?} script must name the `cerulion` command:\n{script}"
            );
        }
    }

    #[test]
    fn each_install_hint_names_its_own_shell_token_and_the_binary() {
        for shell in CompletionShell::value_variants() {
            let hint = shell.install_hint();
            assert!(
                hint.contains(shell.as_str()),
                "{shell:?} hint must set COMPLETE to its own token: {hint}"
            );
            assert!(
                hint.contains("cerulion"),
                "{shell:?} hint must invoke the binary: {hint}"
            );
        }
    }

    #[test]
    fn the_zsh_hint_guards_compinit_because_compdef_is_not_a_builtin() {
        // Reproduced against a clean `ZDOTDIR`: the
        // generated zsh script ends in `compdef …`, which is a FUNCTION
        // `compinit` defines. macOS's `/etc/zshrc` never calls `compinit`, so
        // a stock `~/.zshrc` printed `command not found: compdef` on every
        // shell start and completions silently never worked.
        //
        // Byte-pinned rather than merely "contains compinit": these exact two
        // lines were executed against a clean zsh (bug reproduced without the
        // guard, fixed with it) AND against a shell whose framework had
        // already run `compinit` (guard skips the second one — its dump file
        // mtime is untouched). A reworded guard is a DIFFERENT claim and must
        // be re-verified on a real shell, not waved through by a substring.
        let hint = CompletionShell::Zsh.install_hint();
        let lines: Vec<&str> = hint.lines().collect();
        assert_eq!(
            lines,
            vec![
                "echo 'autoload -Uz compinit && (( $+functions[compdef] )) || compinit' >> ~/.zshrc",
                "echo 'source <(COMPLETE=zsh cerulion)' >> ~/.zshrc",
            ],
            "the zsh hint changed — re-verify it on a compinit-less zsh before \
             re-blessing this oracle"
        );
    }

    #[test]
    fn the_fish_hint_is_the_single_canonical_form() {
        // The fish line is quoted on three
        // surfaces — this hint, `cli.rs`'s long_about table, and
        // `docs/cli_completions.md`, which must not drift apart. One
        // canonical form, and it is the LAZY one: a file under
        // `completions/` is autoloaded by fish the first time `cerulion` is
        // completed, so it self-regenerates and costs nothing at shell start.
        assert_eq!(
            CompletionShell::Fish.install_hint(),
            "echo 'COMPLETE=fish cerulion | source' > ~/.config/fish/completions/cerulion.fish"
        );
    }
}
