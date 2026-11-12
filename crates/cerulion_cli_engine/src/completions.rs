// SPDX-License-Identifier: AGPL-3.0-only
//! The value sources behind shell tab-completion.
//!
//! The clap tree (commands, subcommands, flags, `ValueEnum` values) completes
//! itself — `clap_complete`'s dynamic engine walks the same `clap::Command`
//! the binary parses with. This module supplies the half clap cannot know:
//! the LIVE names a roboticist actually types — topics, node types, graph
//! names, schemas, robots.
//!
//! # The hard rule: a TAB press never hangs and never touches the network
//!
//! Completion runs on the keystroke. Every source here is therefore an
//! INSTANT, LOCAL read — a directory listing, a JSON/TOML file in
//! `~/.cerulion`, a compile-time static, or the local iceoryx2 service
//! directory. Specifically NOT used, and structurally absent from this module:
//!
//! - **No process is ever started.** In particular `cerulion-netd` is never
//!   spawned. Its client's only public constructors (`connect_or_spawn*`) fork
//!   a daemon and then block up to ~10 s polling for readiness — that is a
//!   twenty-second stare at a TAB press in the worst case.
//! - **No network.** No zenoh session, no mDNS browse, no TCP probe, no DNS
//!   resolution. The whole discovery ladder is off-limits here.
//! - **No parsing pass that scales with the workspace.** `node_cmd::node_list`
//!   runs `syn::parse_file` over every node's `lib.rs`; `schema_cmd::
//!   schema_list` re-parses all 254 built-in `.msg` texts to build a
//!   `LayoutResolver`. Both are correct for their verbs and far too expensive
//!   here, so this module does its own name-only scans.
//!
//! That is enforced two ways: by construction (nothing in this file can reach
//! those paths) and by `tests/completions_test.rs`, which reads this file's
//! source with comments stripped and fails if a spawn/network token appears.
//!
//! # Why netd's demand table is not consulted
//!
//! netd's control socket answers a `status` verb listing every demanded
//! `(robot, topic)`. Consulting it would add nothing: by a demanded
//! remote topic IS a local `{topic}/data` mirror service, so it already shows
//! up in the local iceoryx2 scan [`topic_names_from`] consumes. A second,
//! hand-rolled NDJSON round-trip would duplicate netd's client protocol for
//! zero extra names.
//!
//! The consequence: a remote topic completes only while something on
//! this desk is HOLDING a mirror of it. Demands are process-scoped — netd's
//! `DemandGuard` releases on connection close and the last release RETIRES the
//! mirror — so:
//!
//! - `cerulion viz --robot NAME` leaves a mirror standing, because the
//!   long-lived `cerulion-vizd` daemon holds the demand.
//! - `topic echo` / `info` / `hz` hold one only for as long as that command
//!   runs; when it exits, the mirror is torn down and the topic stops
//!   completing.
//! - `topic list` demands NOTHING at all — it is a service-directory scan plus
//!   a liveliness gather, and it establishes no mirror.
//!
//! So with no vizd attach, a remote robot's topics do not complete. Learning
//! their names requires a network round-trip, which the hard rule forbids.
//!
//! # Budget, and what it cost
//!
//! Even a local read can wedge (a stale NFS mount, a sick `/dev/shm`), so every
//! source runs through [`run_bounded`], which hands the work to a thread and
//! gives up at the deadline. Giving up yields NO candidates, which the shell
//! renders as "nothing to complete" — never a hang, never an error on screen.
//!
//! The ceiling is not theoretical. Two SHM reads were MEASURED on an idle desk
//! with zero live topics: `topic_cmd::topic_list` (a static-config directory
//! scan plus one small mmap per topic) at **4 ms**, and
//! `topic_cmd::gather_mirror_provenance` — which opens an iceoryx2 node — at
//! **620 ms**, four times the whole budget. The first is a source here; the
//! second is not, at the cost of the origin-robot annotation on a mirrored
//! topic and the "streaming now" robot row. Both were cosmetic; a completer
//! that burns its allowance on a label and then times out is strictly worse
//! than one that never looked.

use std::collections::BTreeSet;
use std::path::Path;
use std::time::Duration;

use crate::peer_cache::CachedPeer;

/// The whole-invocation wall-clock ceiling for producing completions.
///
/// Sized against the measured floor: process start to `--version` is under
/// 20 ms on a warm desk, and the dominant source (the local iceoryx2 service
/// scan) is single-digit ms for tens of topics. 150 ms leaves an order of
/// magnitude of headroom while staying under the ~200 ms threshold where a
/// keystroke stops feeling instant.
pub const COMPLETION_BUDGET: Duration = Duration::from_millis(150);

/// One completion candidate: the literal text inserted, plus optional help the
/// shell shows beside it.
///
/// Deliberately clap-free — this crate does not depend on `clap`, so the
/// binary maps these onto `clap_complete::CompletionCandidate` at the wiring
/// seam. That also keeps every source in this module oracle-testable against a
/// plain `Vec<Candidate>` instead of an opaque clap type.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct Candidate {
    /// The text the shell inserts.
    pub value: String,
    /// A short provenance note rendered next to the value (`built-in`,
    /// `robot 'go2'`, …). `None` renders the bare value.
    pub help: Option<String>,
}

impl Candidate {
    /// A candidate with no help text.
    pub fn bare(value: impl Into<String>) -> Self {
        Self {
            value: value.into(),
            help: None,
        }
    }

    /// A candidate carrying a provenance note.
    pub fn with_help(value: impl Into<String>, help: impl Into<String>) -> Self {
        Self {
            value: value.into(),
            help: Some(help.into()),
        }
    }
}

/// Whether `s` is safe to emit on the completion wire.
///
/// Checked on every candidate VALUE, because not all of them are ours: robot
/// names arrive from `~/.cerulion/peers.json`, and a netd mirror's local
/// service name is minted from what a remote robot announced. A value that
/// fails is dropped rather than mangled — silently, because this runs on a
/// keystroke.
///
/// The two shells fail in different places, and the rejected set is the UNION —
/// the same candidate stream feeds both, so a value only bash mangles is still
/// unsafe. Both protocols were read (`clap_complete-4.6.7/src/env/shells.rs`)
/// AND driven on a real shell:
///
/// - **zsh** emits `value:help` on `\n`-separated lines. `Zsh::escape_value`
///   escapes `\` and `:`, and `_describe` honours that, so zsh inserts a
///   colon-bearing value correctly. A raw NEWLINE is fatal — the line
///   separator is `\n`, so one value would be read as two candidates.
/// - **bash** emits values only, `\013`-separated, into
///   `COMPREPLY=( $( … ) )`. Two distinct hazards:
///   1. That command substitution is UNQUOTED, so its output undergoes word
///      splitting AND **pathname expansion**: a value holding `*`, `?` or `[`
///      is re-globbed against the cwd and inserts something never offered, or
///      vanishes when nothing matches. (`$` and backticks are safe — a
///      substitution's *result* is never re-scanned for substitution.)
///   2. `:` is in bash's default `COMP_WORDBREAKS`, so readline treats the
///      text after the last colon as the word being completed and replaces
///      only that segment — while the script returns the FULL candidate and
///      calls no `__ltrim_colon_completions` to compensate. The prefix is
///      RE-INSERTED.
///
/// Hazard 2 is why `:` is rejected, and it is MEASURED, not reasoned: on real
/// `/bin/bash` with the real generated registration, typing `robot:` and
/// pressing TAB against a `robot:1` candidate inserts **`robot:robot:1`**,
/// while the colon-free control `robot-` correctly completes to `robot-1`
/// (harness anchored by `cerulion gr` → `cerulion graph`).
///
/// An earlier revision allowed `:` on the grounds that zsh escapes it. That
/// was true and irrelevant: the verdict was right for a reason that only
/// covered one of the two shells, and the same revision was already rejecting
/// `*?[` for a bash-only hazard — so allowing `:` was internally inconsistent
/// as well as wrong.
fn is_wire_safe(s: &str) -> bool {
    !s.is_empty()
        && !s.contains(':')
        && !s.chars().any(|c| c.is_control() || c.is_whitespace())
        && !s.contains(['*', '?', '['])
}

/// Strip the characters that would corrupt the line protocol out of a HELP
/// string.
///
/// Help is cosmetic, so unlike a value it is repaired rather than dropped:
/// control and whitespace runs collapse to a single space, and a string that
/// repairs to nothing yields `None`.
///
/// Only zsh renders help at all — bash's `write_complete` emits values and
/// nothing else — and only a NEWLINE can hurt it (the line separator). That
/// asymmetry is why help may keep characters a VALUE may not: colons are left
/// ALONE here (`_describe` splits on the first UNESCAPED colon, which is the
/// separator zsh itself writes after the escaped value, so everything after it
/// is help no matter how many colons it holds — a locator reads
/// `tcp/10.0.0.9:7683` rather than a mangled `tcp/10.0.0.9 7683`), and glob
/// characters are fine because bash, the shell that re-expands them, never
/// receives help at all.
///
/// A value gets the stricter rule because it is INSERTED into the user's
/// command line by both shells; help is only ever displayed, by one.
fn sanitize_help(s: &str) -> Option<String> {
    let cleaned: String = s
        .chars()
        .map(|c| {
            if c.is_control() || c.is_whitespace() {
                ' '
            } else {
                c
            }
        })
        .collect();
    let trimmed = cleaned.split_whitespace().collect::<Vec<_>>().join(" ");
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed)
    }
}

/// Drop wire-unsafe values, repair help, de-duplicate by value (FIRST wins, so
/// a caller lists its most authoritative source first), and sort.
///
/// Sorting is what makes every source in this module deterministic regardless
/// of directory-iteration order.
pub fn finish(candidates: Vec<Candidate>) -> Vec<Candidate> {
    let mut seen = BTreeSet::new();
    let mut out: Vec<Candidate> = candidates
        .into_iter()
        .filter(|c| is_wire_safe(&c.value))
        .filter(|c| seen.insert(c.value.clone()))
        .map(|c| Candidate {
            help: c.help.as_deref().and_then(sanitize_help),
            value: c.value,
        })
        .collect();
    out.sort();
    out
}

/// Run `f` on a helper thread, giving up after `limit`.
///
/// Returns `Some(value)` if `f` finished in time, `None` otherwise. A `limit`
/// of zero returns `None` WITHOUT running `f` at all — an exhausted budget must
/// not buy the next source a fresh start.
///
/// A timed-out thread is abandoned, not cancelled: a blocking `read_dir` or SHM
/// `mmap` has no interruption point. That is sound HERE and nowhere else,
/// because a completion process exists only to print candidates and exits
/// microseconds later — the abandoned thread dies with it. Do not lift this
/// helper into a long-running path.
///
/// A panic inside `f` presents as a dropped sender, i.e. the same `None` as a
/// timeout. On a keystroke, "no candidates" is the only sane response to
/// either.
pub fn run_bounded<T, F>(limit: Duration, f: F) -> Option<T>
where
    T: Send + 'static,
    F: FnOnce() -> T + Send + 'static,
{
    if limit.is_zero() {
        return None;
    }
    let (tx, rx) = std::sync::mpsc::channel();
    // The receiver may hang up first (timeout); `send` failing is that case and
    // is not an error worth reporting from a completion.
    std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    rx.recv_timeout(limit).ok()
}

// ---------------------------------------------------------------------------
// Sources — pure over their inputs, so every one is oracle-testable.
// ---------------------------------------------------------------------------

/// Node type names: the immediate subdirectories of `nodes/` that carry a
/// `src/lib.rs`.
///
/// The `src/lib.rs` probe is one `stat` per directory and buys parity with
/// `cerulion node list`, which skips a directory whose `lib.rs` is missing or
/// unparseable. This cannot detect the *unparseable* half without a `syn` pass,
/// so a scaffold in progress may be offered — the accepted trade for staying
/// inside the budget. A missing or unreadable `nodes/` yields no candidates.
pub fn node_types_in(nodes_dir: &Path) -> Vec<Candidate> {
    let Ok(entries) = std::fs::read_dir(nodes_dir) else {
        return Vec::new();
    };
    let found = entries
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .filter(|e| e.path().join("src").join("lib.rs").is_file())
        .filter_map(|e| e.file_name().into_string().ok())
        .map(Candidate::bare)
        .collect();
    finish(found)
}

/// Graph names: the `.yaml` file stems in `graphs/`.
///
/// Delegates to [`crate::graph_cmd::graph_list`] rather than re-implementing
/// the scan, so the offered set is exactly what `cerulion graph list` shows and
/// cannot drift from it. That inherits the `.yaml`-only rule — `graph_read`
/// joins `{name}.yaml`, so a `.yml` file is not runnable and is not offered.
///
/// It is a SUPERSET of what `graph run` accepts, not an exact match: the
/// auto-partitioner's cost snapshot sits beside the graph as
/// `<graph>.costs.yaml`, whose stem is a `.yaml` stem like any other, so
/// `<graph>.costs` is offered and would fail to load. Filtering it here would
/// put a second, silently-diverging notion of "what is a graph" beside
/// `graph_list`; the shared miss is pinned by
/// `graph_names_are_yaml_stems_and_exclude_what_graph_run_cannot_resolve` so a
/// future filter is a deliberate change to both.
///
/// `graph_list`'s `Err` (an unreadable dirent) degrades to no candidates.
pub fn graph_names_in(graphs_dir: &Path) -> Vec<Candidate> {
    let names = crate::graph_cmd::graph_list(graphs_dir).unwrap_or_default();
    finish(names.into_iter().map(Candidate::bare).collect())
}

/// Schema names: workspace YAML stems, the `.msg` store, and every built-in
/// ROS 2 type.
///
/// Ordering encodes precedence, and `finish`'s first-wins de-duplication makes
/// that real: a workspace schema SHADOWS a built-in of the same name (the rule
/// `cerulion schema info` applies), so it is listed first and its help wins.
///
/// Workspace YAML files contribute their FILE STEM, not the declared `schemas:`
/// key. `schema info` accepts either, and reading the stem is a directory entry
/// while reading the key costs a `read_to_string` + serde parse per file. Where
/// the two differ — an unusual workspace — the declared name still resolves
/// when typed in full, it just is not offered.
///
/// `schemas_dir` is `None` outside a workspace, which reduces this to the
/// built-in registry — the same degradation `schema_cmd::schema_list_opt`
/// applies, and the reason `cerulion schema info sensor_msgs/Image <TAB>`
/// works anywhere.
pub fn schema_names_in(schemas_dir: Option<&Path>) -> Vec<Candidate> {
    let mut out = Vec::new();

    if let Some(dir) = schemas_dir {
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.filter_map(|e| e.ok()) {
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) == Some("yaml") {
                    if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                        out.push(Candidate::with_help(stem, "workspace"));
                    }
                }
            }
        }
        out.extend(msg_store_names_in(dir));
    }

    for (pkg, msg, _text) in native_ros2_messages::BUILTIN_MSGS {
        out.push(Candidate::with_help(format!("{pkg}/{msg}"), "built-in"));
    }

    finish(out)
}

/// The `schemas/<pkg>/msg/<Type>.msg` store, as `pkg/Type` names.
///
/// A two-level directory walk with no parse — `SchemaStore::load` reads and
/// `parse_rosmsg`s every file, which is the right thing for resolution and the
/// wrong thing for a keystroke. Names are all completion needs.
fn msg_store_names_in(schemas_dir: &Path) -> Vec<Candidate> {
    let Ok(packages) = std::fs::read_dir(schemas_dir) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for pkg_entry in packages.filter_map(|e| e.ok()) {
        let msg_dir = pkg_entry.path().join("msg");
        if !msg_dir.is_dir() {
            continue;
        }
        let Some(pkg) = pkg_entry.file_name().into_string().ok() else {
            continue;
        };
        let Ok(msgs) = std::fs::read_dir(&msg_dir) else {
            continue;
        };
        for msg_entry in msgs.filter_map(|e| e.ok()) {
            let path = msg_entry.path();
            if path.extension().and_then(|e| e.to_str()) != Some("msg") {
                continue;
            }
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                out.push(Candidate::with_help(
                    format!("{pkg}/{stem}"),
                    "msg store".to_string(),
                ));
            }
        }
    }
    out
}

/// Robot names, from the three instant local records of a robot this desk has
/// actually met.
///
/// - `peers_path` — `~/.cerulion/peers.json`, the warm-peer cache.
///   Read through [`crate::peer_cache::load_peers`], so the 7-day TTL applies
///   and a robot that has been gone a week stops being offered.
/// - `robots_toml_path` — `~/.cerulion/robots.toml`, the `name → eid` pins
///   `cerulion pair` writes. A paired robot completes even if it has never
///   been seen on THIS LAN, which is the point of pairing.
///
/// A fourth source was measured and REJECTED: the origin robots of the mirrors
/// live in local SHM. Reading them means
/// `mirror_registry::gather_current_provenance`, which opens an iceoryx2 node —
/// MEASURED at 620 ms on an idle desk with zero topics, four times the whole
/// budget. It would also have added little: a robot only ends up mirrored here
/// after this desk has already met it (a `pair`, or a `topic list` gather that
/// wrote it back to the peer cache), so a robot streaming right now is
/// overwhelmingly a robot already in one of the two files above.
///
/// `now_unix_secs` is a parameter rather than a `SystemTime::now()` call so the
/// TTL boundary is testable without waiting a week.
pub fn robot_names_at(
    peers_path: Option<&Path>,
    robots_toml_path: Option<&Path>,
    now_unix_secs: u64,
) -> Vec<Candidate> {
    let mut out = Vec::new();

    if let Some(path) = robots_toml_path {
        if let Ok(text) = std::fs::read_to_string(path) {
            for name in crate::connect_cmd::parse_robots_toml(&text).into_keys() {
                out.push(Candidate::with_help(name, "paired"));
            }
        }
    }

    if let Some(path) = peers_path {
        for CachedPeer { robot, locator, .. } in crate::peer_cache::load_peers(path, now_unix_secs)
        {
            out.push(Candidate::with_help(robot, locator));
        }
    }

    finish(out)
}

/// Topic names, from the local iceoryx2 service directory.
///
/// `local` is the already-gathered topic list, so this half stays pure; the SHM
/// read happens in [`complete_topic_names`].
///
/// Mirrors are not filtered out — the "one data source = one topic"
/// decision makes a mirror the local handle for a remote topic, and `topic echo` /
/// `viz` take it by exactly this name.
///
/// Candidates carry NO help, deliberately. The obvious annotation would be the
/// mirror's origin robot, but reading the C0 provenance registry costs
/// 620 ms (see [`robot_names_at`]) — and the fallback of labelling everything
/// `local` would print an affirmatively WRONG claim on exactly the rows an
/// operator most needs to distinguish. A missing note is harmless; a wrong one is
/// not. `cerulion topic list` shows the attribution.
pub fn topic_names_from(local: &[String]) -> Vec<Candidate> {
    finish(local.iter().cloned().map(Candidate::bare).collect())
}

// ---------------------------------------------------------------------------
// Live entry points — resolve the environment, then run the sources above
// under the budget. These are what the binary's clap attributes call.
//
// Each completer is ONE `run_bounded` call. That is not an accident of the
// current source set: a source measured to cost 620 ms (the mirror-provenance
// registry, see `robot_names_at`) was dropped rather than run second on a
// drained budget, because a completer that spends its whole allowance on a
// cosmetic annotation and then returns nothing is worse than one that never
// looked. If a second source is ever added here, it must be cheap enough that
// BOTH fit — not merely ordered so the important one runs first.
// ---------------------------------------------------------------------------

/// The workspace directories for this cwd, or `None` outside a workspace.
///
/// A completion NEVER errors: outside a workspace the node/graph completers
/// simply have nothing to say, and the shell falls back to its own default.
fn workspace_dirs() -> Option<crate::workspace::CerulionWorkspace> {
    let cwd = std::env::current_dir().ok()?;
    crate::workspace::CerulionWorkspace::discover(&cwd).ok()
}

/// Node types in the workspace containing the cwd.
pub fn complete_node_types() -> Vec<Candidate> {
    run_bounded(COMPLETION_BUDGET, || {
        workspace_dirs()
            .map(|ws| node_types_in(&ws.nodes_dir))
            .unwrap_or_default()
    })
    .unwrap_or_default()
}

/// Graph names in the workspace containing the cwd.
pub fn complete_graph_names() -> Vec<Candidate> {
    run_bounded(COMPLETION_BUDGET, || {
        workspace_dirs()
            .map(|ws| graph_names_in(&ws.graphs_dir))
            .unwrap_or_default()
    })
    .unwrap_or_default()
}

/// Workspace schemas plus every built-in ROS 2 type.
pub fn complete_schema_names() -> Vec<Candidate> {
    run_bounded(COMPLETION_BUDGET, || {
        let ws = workspace_dirs();
        schema_names_in(ws.as_ref().map(|w| w.schemas_dir.as_path()))
    })
    .unwrap_or_default()
}

/// Robots this desk has met: paired or seen on the LAN.
pub fn complete_robot_names() -> Vec<Candidate> {
    run_bounded(COMPLETION_BUDGET, || {
        let peers = crate::peer_cache::default_cache_path();
        let robots = crate::connect_cmd::robots_toml_path();
        robot_names_at(peers.as_deref(), robots.as_deref(), now_unix_secs())
    })
    .unwrap_or_default()
}

/// Topics visible on this machine — genuine local producers and netd mirrors
/// of remote robots alike.
pub fn complete_topic_names() -> Vec<Candidate> {
    // BEFORE the helper thread, deliberately. `quiet_iceoryx2` calls
    // `set_var`, which is process-global and not thread-safe against a
    // concurrent reader; running it here keeps the mutation on the only thread
    // alive at that moment. The process is single-purpose so the race was
    // harmless in practice, but it costs nothing to remove and a future caller
    // that keeps working after this returns would inherit it.
    quiet_iceoryx2();
    run_bounded(COMPLETION_BUDGET, || {
        let local: Vec<String> = crate::topic_cmd::topic_list()
            .unwrap_or_default()
            .into_iter()
            .map(|t| t.name)
            .collect();
        topic_names_from(&local)
    })
    .unwrap_or_default()
}

/// Wall-clock seconds since the Unix epoch (0 if the clock is before it).
fn now_unix_secs() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Force iceoryx2's own logger to `error` for the duration of this process.
///
/// A completion must print NOTHING but candidates — anything else lands in the
/// user's shell as garbage. Two paths could otherwise write to stderr: an
/// unparseable `IOX2_LOG_LEVEL`, which makes
/// `init_iceoryx_log_level_from_env` emit one `eprintln!` naming the offender,
/// and iceoryx2's built-in console logger on a malformed service if the user
/// raised the level themselves. Pinning the variable to a value that always
/// parses closes both.
///
/// Overriding the user's choice is right here and only here: this process
/// produces completions and exits. It cannot affect a real run, which resolves
/// the variable in its own process.
fn quiet_iceoryx2() {
    std::env::set_var("IOX2_LOG_LEVEL", "error");
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn wire_safety_rejects_exactly_the_characters_that_corrupt_the_protocol() {
        // Ordinary Cerulion / ROS names survive.
        assert!(is_wire_safe("/utlidar/cloud"));
        assert!(is_wire_safe("sensor_msgs/Image"));
        assert!(is_wire_safe("go2"));
        assert!(is_wire_safe("my-robot_2"));
        // A colon is REJECTED — for bash, not zsh. `:` is in bash's default
        // COMP_WORDBREAKS, so readline replaces only the segment after the
        // colon while the generated script returns the whole candidate and
        // never calls `__ltrim_colon_completions`. MEASURED on real /bin/bash:
        // typing `robot:` + TAB against a `robot:1` candidate inserts
        // `robot:robot:1`, while the colon-free control `robot-` completes to
        // `robot-1` correctly. zsh escapes it and is fine; the rejected set is
        // the UNION over both shells, because one candidate stream feeds both.
        assert!(!is_wire_safe("robot:1"));
        assert!(!is_wire_safe("tcp/10.0.0.9:7683"));
        // A newline forges a second candidate on zsh's `\n`-separated wire.
        assert!(!is_wire_safe("two\nlines"));
        // Other control characters: `\013` is bash's separator, ESC carries a
        // terminal sequence into the completion display.
        assert!(!is_wire_safe("bell\u{0b}sep"));
        assert!(!is_wire_safe("esc\x1b[31m"));
        // Whitespace: a bash COMPREPLY entry is inserted unquoted.
        assert!(!is_wire_safe("has space"));
        assert!(!is_wire_safe("tab\there"));
        // Glob metacharacters: bash's `COMPREPLY=( $( … ) )` is an UNQUOTED
        // command substitution, so these are re-expanded against the cwd.
        assert!(!is_wire_safe("/topic/*"));
        assert!(!is_wire_safe("robot?"));
        assert!(!is_wire_safe("name[0]"));
        // Empty is not a candidate.
        assert!(!is_wire_safe(""));
    }

    #[test]
    fn help_is_repaired_rather_than_dropped() {
        assert_eq!(sanitize_help("robot go2"), Some("robot go2".to_string()));
        // A colon SURVIVES: zsh's `_describe` splits on the first UNESCAPED
        // colon — the separator zsh writes itself — so everything after it is
        // help regardless of how many colons it holds. A locator must read
        // like a locator.
        assert_eq!(
            sanitize_help("tcp/1.2.3.4:7683"),
            Some("tcp/1.2.3.4:7683".to_string())
        );
        // Control and whitespace runs collapse to one space (a newline would
        // forge a candidate line).
        assert_eq!(sanitize_help("a\n\n\tb"), Some("a b".to_string()));
        // Nothing left after repair means no help at all.
        assert_eq!(sanitize_help("   "), None);
        assert_eq!(sanitize_help("\n\t"), None);
    }

    #[test]
    fn finish_drops_unsafe_values_dedups_first_wins_and_sorts() {
        let out = finish(vec![
            Candidate::with_help("beta", "second"),
            Candidate::with_help("alpha", "workspace"),
            // Duplicate of `alpha` — the FIRST occurrence's help must win, so
            // a caller can encode precedence by ordering.
            Candidate::with_help("alpha", "built-in"),
            // Dropped: bash would re-glob this against the cwd.
            Candidate::bare("bad*value"),
            // Dropped: a newline would be read as two candidates.
            Candidate::bare("bad\nvalue"),
        ]);
        assert_eq!(
            out,
            vec![
                Candidate::with_help("alpha", "workspace"),
                Candidate::with_help("beta", "second"),
            ]
        );
    }

    #[test]
    fn a_zero_budget_refuses_to_start_the_work_at_all() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        // The anti-tautology half: a live budget really does run the closure.
        let ran = Arc::new(AtomicUsize::new(0));
        let probe = Arc::clone(&ran);
        let got = run_bounded(Duration::from_secs(5), move || {
            probe.fetch_add(1, Ordering::SeqCst);
            7usize
        });
        assert_eq!(got, Some(7));
        assert_eq!(ran.load(Ordering::SeqCst), 1);

        // An exhausted budget must not merely discard the result — it must
        // never START the source. A source that ran and was thrown away has
        // already spent the time the budget existed to protect.
        let probe = Arc::clone(&ran);
        let got = run_bounded(Duration::ZERO, move || {
            probe.fetch_add(1, Ordering::SeqCst);
            7usize
        });
        assert_eq!(got, None);
        assert_eq!(
            ran.load(Ordering::SeqCst),
            1,
            "the source must not have run"
        );
    }

    #[test]
    fn a_slow_source_is_abandoned_at_the_deadline() {
        let started = Instant::now();
        let got = run_bounded(Duration::from_millis(30), || {
            std::thread::sleep(Duration::from_secs(30));
            "never"
        });
        let elapsed = started.elapsed();
        assert_eq!(got, None, "a source past its limit must yield nothing");
        // Generous ceiling: this only needs to prove we did not wait 30s.
        assert!(
            elapsed < Duration::from_secs(5),
            "gave up after {elapsed:?}, expected ~30ms"
        );
    }

    #[test]
    fn a_panicking_source_yields_no_candidates_instead_of_unwinding() {
        // The sender is dropped by the panic, which the receiver sees as a
        // disconnect — the same `None` a timeout produces.
        let got: Option<usize> = run_bounded(Duration::from_secs(5), || panic!("boom"));
        assert_eq!(got, None);
    }

    #[test]
    fn the_shipped_budget_stays_inside_the_keystroke_feel_window() {
        // A drift guard on the one constant the whole design rests on. The
        // floor is the MEASURED cost of the dominant source (`topic_list`,
        // 4 ms on an idle desk, growing with topic count) plus process start
        // (~15 ms), so a budget much under 50 ms would start truncating real
        // completions on a busy desk. The ceiling is where a TAB stops feeling
        // instant. The 620 ms provenance read sits far outside this window,
        // which is why it is not a source.
        assert!(
            COMPLETION_BUDGET >= Duration::from_millis(50),
            "too tight to cover the measured source cost plus process start"
        );
        assert!(
            COMPLETION_BUDGET <= Duration::from_millis(200),
            "past the threshold where a keystroke stops feeling instant"
        );
    }
}
