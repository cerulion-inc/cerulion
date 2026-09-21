#!/usr/bin/env bash
# check_public_surface.sh: the public-surface gate.
#
# Eight classes of mistake in a shipped tree are shapes a script can see, so
# this gate makes them fail in CI before the page ships. It runs
# over the TRACKED tree (`git ls-files`), needs no cargo and no network, and
# finishes in seconds (the work-state scan, the one costly pass, is split over
# a process pool and falls back to one process where a pool cannot start).
#
# THE CLASSES (each finding names its class; each failing class prints one
# `remedy:` line naming the rule it enforces)
#
#   examples-shape        Every `examples/<name>/` is a workspace: `Cargo.toml`,
#                         `graphs/*.yaml` and `nodes/<type>/src/lib.rs`, each
#                         lib.rs declaring exactly ONE `#[cerulion_node]` type.
#                         No file under `examples/` or `crates/*/examples/`
#                         builds a graph in code (`GraphRuntime::build`,
#                         `TransportManager::init`/`get_or_init`,
#                         `Box<dyn NodeEntry>` factories, `parse_graph(`)
#                         outside a `tests/` directory or a `#[cfg(test)]`
#                         module. No `cargo run|test ... --example` in any
#                         README, docs page or AGENTS.md.
#   docs-refs             In README.md, docs/**/*.md, examples/**/*.md, every
#                         AGENTS.md and every README.md: every relative link
#                         resolves to a file or directory in the tree (an
#                         ALL_CAPS placeholder such as a download URL that the
#                         cut replaces is reported as a note, never a finding);
#                         every `cerulion <verb> [<sub> ...]` in a fenced block
#                         or backticked span exists in the CLI, with the verb
#                         tree derived from the clap definitions in
#                         `crates/cerulion_cli/src/cli.rs` (hidden verbs count
#                         as existing; `help` stops the walk); and no
#                         pre-move path spelling appears (`<crate>/...` for any
#                         directory under `crates/`, `test_fixtures/...`,
#                         `scripts/...`, `USER_API.md`).
#   unreferenced-media    Every file under `docs/media/` (except README.md and
#                         the `charts/` generator directory) is named by
#                         README.md or a docs page. The media index itself is
#                         NOT a use: a row in an index is how an orphan hides.
#   bench-citation        Every directory under `docs/benchmarks/results/` is
#                         named by README.md, docs/PERFORMANCE.md or
#                         docs/benchmarks/README.md; every top-level directory
#                         under `benches/` is named by README.md or
#                         benches/README.md.
#   shipped-text          Over every tracked file outside `tests/` directories,
#                         `tests.rs` files and `crates/test_fixtures/`:
#                         (a) no typographic dash (U+2013, U+2014) in shipped
#                             text: Rust STRING LITERALS (comments exempt), any
#                             `.md` page in full, a `Cargo.toml` `description`,
#                             a workflow `name:`. The tree carried thousands
#                             when this gate landed, so this rule is a LEDGER
#                             (`public_surface_dash_ledger.txt`, `<count>
#                             <path>`): a file above its count fails (a new
#                             dash), a file below its count fails until the
#                             line is lowered (a stale line pre-authorises the
#                             next dash), a file not listed allows none.
#                         (b) no tracker id of the shape `<letters>-<digits>`
#                             used by the issue tracker, anywhere but
#                             CHANGELOG.md (comments included: the id means
#                             nothing to a reader outside the tracker).
#                         (c) no to-do or fix-me marker that names a person in
#                             its parentheses; a hyphenated reason there passes
#                             THIS rule (class work-state refuses the marker
#                             itself, whatever its parentheses hold).
#                         (d) no phrase from `public_surface_phrases.txt`, a
#                             data file of claims the README contradicts (it
#                             grows the day the README changes a fact).
#   work-state            Shipped text describes the product to its user: what
#                         works, what is experimental, what is not supported,
#                         what to do. It never reports how the project was
#                         built: who decided, which plan step or review round
#                         a line came from, which machine a test ran on, or
#                         what is still on a to-do list.
#                         `public_surface_workstate.txt` is the single source
#                         of truth: `<key> | <python regex> | <what to write
#                         instead>` per line, matched line by line; a finding
#                         names its key and the class prints one `note:` per
#                         key that fired, saying what to write instead. A
#                         pattern line of another shape, a regex that does not
#                         compile or matches the empty string, a repeated key
#                         or a missing file is exit 3: a skipped line would be
#                         a key that silently stopped matching.
#                         SCOPE: every tracked text file, `tests/` and
#                         `crates/test_fixtures/` INCLUDED (a test comment
#                         ships too), except raw benchmark evidence
#                         (`docs/benchmarks/results/`), the vendored message
#                         and binding trees, lockfiles, `docs/legal/`, the CLA
#                         texts, LICENSE, and this gate's own scripts and data
#                         files and `leak_scan.py`, which spell the wording in
#                         order to refuse it.
#                         LEDGER `public_surface_workstate_ledger.txt`,
#                         `<count> <key> <path>`: the tree carried this wording
#                         in hundreds of files when the class landed, so the
#                         rule burns down like the dash ledger. A file above
#                         its count fails and every matching line is printed;
#                         a file below its count fails until the line is
#                         lowered; a file not listed allows none.
#                         USER-FACING pages are never listed: README.md,
#                         CHANGELOG.md, `docs/` outside `docs/internals/`,
#                         `examples/**/*.md`, `crates/**/README.md` and
#                         `.github/**/*.md` read zero, and a ledger line for
#                         one is itself a finding.
#                         A LEGITIMATE line (a chunk number of the MCAP
#                         format, a request a protocol itself calls by the
#                         word) is named in the allow list; the entry excuses
#                         the line BEFORE the count, so it is never ledger
#                         debt, and `*` is refused for this class: an entry
#                         names the one line it excuses.
#   numbers-without-data  A measured figure in README.md or docs/PERFORMANCE.md
#                         (a number followed by µs, ns, ms, Hz, MiB/s,
#                         replies/s, p50 or p99; a range `a to b <unit>`; every
#                         cell after the first in a table whose header names
#                         p50/p99 or a unit) must appear, spelled the same, in a
#                         shipped package under `docs/benchmarks/results/`: a
#                         CSV, a HEADLINE.md or the package's own README.md.
#                         RULE: formatting is tolerated only as far as the
#                         package convention goes. A CSV nanosecond cell also
#                         matches as microseconds at two decimals or as a
#                         whole number and as milliseconds at one decimal (how
#                         HEADLINE.md renders it). 10.45 versus 10.4 is a MISS:
#                         a page prints the package's precision, never a
#                         rounding of it. Whole numbers with one significant
#                         digit (100 Hz, 10 ms, 2 ms) are configuration, not
#                         measurements, and are skipped. A round figure can
#                         match another package by coincidence; the class is
#                         built for the measured decimals.
#   string-literal-rewrite This one cannot be a tree check. A bulk edit that
#                         strips a token turns "/cer561pos/producer/out" into
#                         "/producer/out" inside a test's string literal and
#                         breaks every test that reads it: a bulk edit never
#                         rewrites the inside of a string literal; each literal
#                         is fixed by hand and the tests that read it are run.
#                         The rule lives in AGENTS.md; `--self-test` carries
#                         the shape as a named fixture (the damaged and the
#                         careful rewrite) so the class is spelled out where an
#                         agent looks.
#
# ALLOW LIST `public_surface_allow.txt`: `<path> | <class> | <match> | <reason>`
# per line; `<match>` is a substring of the finding message or `*`. An entry
# that excuses nothing FAILS the run (a stale waiver pre-authorises the next
# finding), so the list can only describe what is really there.
#
# `--self-test` builds a fixture tree with a positive arm (caught) and a
# negative control (not caught) for every class, checks the verb-tree parser
# against a hand-written oracle, the Rust lexer, the fail-closed path (no CLI
# definition = exit 3) and the ledger regenerator. For work-state it reads the
# REAL pattern file: one caught line per key, a legitimate neighbour per key
# that must stay silent, a ledgered file at, above and below its count, a
# user-facing page someone tried to ledger, a malformed pattern line and a
# missing pattern file (both exit 3), and the pool against one process. It
# runs first in CI as its own step, because the real tree is clean by
# construction and a rule that stopped matching looks exactly like success.
#
# USAGE
#   check_public_surface.sh                          # the tree at the repo root
#   check_public_surface.sh --root <dir>             # another checkout
#   check_public_surface.sh --self-test              # the fixture arms
#   check_public_surface.sh --regenerate-dash-ledger # rewrite the ledger from the tree
#   check_public_surface.sh --regenerate-workstate-ledger
#                                                    # rewrite the work-state ledger from the
#                                                    # tree (user-facing pages are never written)
#   CHECK_PUBLIC_SURFACE_JOBS=<n>                    # the work-state pool size (1 = one process)
#
# OUTPUT: one `<file>:<line>: <class>: <message>` per finding, `note:` lines
# that never fail, one `remedy:` line per failing class, and ONE summary line
# last. Exit 0 = clean, 1 = findings, 2 = usage, 3 = could not run (python3 or
# git missing, an input unreadable, the CLI definition absent): a gate must
# not fail open.
#
# Portable: bash 3.2+ (macOS), BSD and GNU userland. The logic is
# `check_public_surface.py` (python3, standard library only); this wrapper
# owns the argument contract and the exit codes.

set -u

cd "$(dirname "$0")/../.." || exit 3
HELPER="tools/scripts/check_public_surface.py"

args=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --self-test|--regenerate-dash-ledger|--regenerate-workstate-ledger)
            args+=("$1")
            shift
            ;;
        --root)
            if [ "$#" -lt 2 ]; then
                printf 'check_public_surface: --root needs a directory\n' >&2
                exit 2
            fi
            args+=("$1" "$2")
            shift 2
            ;;
        -h|--help)
            # The header ends where the comments end, computed rather than
            # counted, so `--help` never truncates as the header grows.
            awk 'NR == 1 { next } /^#/ { print; next } { exit }' "$0"
            exit 0
            ;;
        *)
            # An unknown flag is an error, never a silent no-op that prints OK.
            printf 'check_public_surface: unknown argument %s\n' "$1" >&2
            exit 2
            ;;
    esac
done

if ! command -v python3 >/dev/null 2>&1; then
    printf 'check_public_surface: CANNOT RUN: python3 is not on PATH\n'
    exit 3
fi
if [ ! -r "$HELPER" ]; then
    printf 'check_public_surface: CANNOT RUN: %s is missing\n' "$HELPER"
    exit 3
fi

# `[@]+` expands an empty array without tripping `set -u` on bash 3.2.
python3 -B "$HELPER" ${args[@]+"${args[@]}"}
exit $?
