#!/usr/bin/env bash
# ci_test_shard.sh — run ONE shard of a package's integration-test suite.
#
# WHY THIS EXISTS. `cerulion_core` carries 250+ integration-test binaries and
# the jobs that run them (`Test (Linux)` / `Test (macOS)` in
# .github/workflows/ci.yml) had grown to ~60 min against an 80-min ceiling,
# with self-cancels in every ceiling era. A shard is one VM running a SUBSET of
# the test binaries, because each runner VM has its own /dev/shm.
#
# THE SHARD DOES NOT RUN SERIALLY. Ending in
# `exec cargo test "$@" -- --test-threads=1` would rest
# on the belief that the whole package cannot be
# parallelised because iceoryx2's shared memory is a per-machine singleton.
# That belief was measured and is false: the large majority of these binaries
# mint per-test SHM roots (`init_for_test` / `generate_isolated_config`
# / `build_for_test`) and share no namespace with anything. The shard runs
# under `cargo nextest`, which gives each test its OWN PROCESS and serialises
# only the binaries that genuinely cannot run beside a sibling — the fence in
# `.config/nextest.toml`. See that file for the two fences and why they use two
# different mechanisms.
#
# So `--test-threads=1` is ABSENT from this script, and its absence is the point.
# Serialisation is a property of the FENCE, which is gated by
# `cerulion_core/tests/serial_discipline_test.rs` (membership must equal a
# declared inventory), rather than of a flag applied to everything.
#
# THE LIST IS GENERATED, NEVER HAND-WRITTEN. A hand-maintained shard list is
# a known failure mode (a package
# added to the workspace and named in no CI step). So the shard membership is
# computed here from what is on disk: every `<dir>/tests/*.rs` at depth 1,
# sorted, assigned round-robin by `index mod count`. Add a test file and it
# lands in a shard automatically; nothing needs editing.
#
# WITH EXACTLY ONE NAMED EXCEPTION, and the rule above is why it is one name
# and not a list: `macro_compile_fail_test` is PINNED to a fixed shard and held
# out of the rotation, so every other file keeps the shard the package map was
# balanced against. Everything else still round-robins with no edit here. Why
# the pin exists and the bar for adding a second name are at `PINNED_TEST`
# below; `--check` proves the pin landed where it says it did.
#
# `--lib` (the package's own unit tests) rides shard 0. It is a single target,
# it is cheap, and it has to go somewhere exactly once.
#
# OTHER PACKAGES' TESTS RIDE THE SAME INVOCATION with `--with <package>`. A
# package step that runs after this one adds its whole time to the leg, while
# a test in this invocation runs beside the shard's own tests and is built by
# the same cargo invocation. The `#[cerulion_node]` diagnostics suite and the
# node context helper suite ride shard 2 of both lanes this way: each holds a
# test that runs trybuild over its fixtures, for minutes in the diagnostics
# suite, and `.config/nextest.toml` starts them right after the timing tests
# that claim every slot. An extra package adds `-p <package> --test <name>` for each of its
# `tests/*.rs` at depth 1. Cargo applies `--lib` and every `--test` to all the
# packages one command selects, so the script refuses `--with` on shard 0,
# which carries `--lib`, and refuses an extra that holds a test this
# invocation could not run (see `check_extra`), or a `tests/*.rs` named like
# one of this package's (cargo would run that file of this package too). Each
# package is read against the selection on its own.
#
# THE EXTRAS' FEATURES ARE MEASURED, NOT GATED. Building an extra in this
# invocation unions its dev-dependency features into this package's test
# build. For `node_macro_ui` that union adds nothing: its dev-dependencies ask
# for `test-helpers` on `cerulion_core` and `native_ros2_messages`, which
# `cerulion_core`'s own test build already turns on. To check it again after
# an extra's manifest changes, compare two unit graphs of the shard-2 build,
# `cargo test --no-run $(ci_test_shard.sh --list cerulion_core 2 4) --unit-graph
# -Z unstable-options` with and without `--with <extra>` inside the `$( )`
# (on stable cargo, with `__CARGO_TEST_CHANNEL_OVERRIDE_DO_NOT_USE_THIS=nightly`
# set): every unit both graphs hold must carry the same features, and the
# second may add only the extra's own units.
#
# USAGE
#   ci_test_shard.sh <package> <shard_index> <shard_count> [package_dir] [--with <extra>]...
#   ci_test_shard.sh --list  <package> <shard_index> <shard_count> [package_dir] [--with <extra>]...
#   ci_test_shard.sh --check [package] [shard_count] [package_dir]
#   ci_test_shard.sh --selected <package>
#
#   run     — invoke `cargo nextest run --profile <p> -p <package> --lib?
#             --test a --test b ... [-p <extra> --test c ...]`
#   --list: print the argument list the run mode would hand nextest, after the
#             same selection, and exit (no cargo, no build); a `selection:` line
#             for each package the selection leaves out goes to stderr.
#   --with: add <extra>'s integration tests to this shard (see above). Each of
#             <package> and the extras rides when CI_SELECTED_PACKAGES names it;
#             the shard skips only when none rides.
#   --selected: answer whether <package> rides CI_SELECTED_PACKAGES, the same
#             reading the run mode gates on, as an exit code (0 rides, 1 does
#             not) plus one `selection:` line. For a workflow step that spends
#             something before this script's run step is reached.
#   --check: prove the partition is TOTAL and DISJOINT: the union of all
#             shards equals the full file list exactly once. Defaults to the
#             package + shard count CI actually uses, so a bare `--check`
#             partitions the shipped package. It does not read the `--with`
#             packages; `--list` with them checks those, and the coverage suite
#             in cerulion_cli_engine runs it on the workflow's own lines.
#
# Extra `cargo nextest run` arguments may be passed through the
# CI_TEST_SHARD_EXTRA environment variable; they are appended to the command
# line. NOTE the position: they do NOT land after a `--` separator,
# i.e. they are not libtest arguments. nextest does not take libtest arguments, so
# they are nextest's own (`--no-capture`, `--run-ignored all`, …).
#
# ONE EXIT-CODE DIFFERENCE, worth knowing before it surprises someone: nextest
# FAILS a run that matches NO tests ("error: no tests to run"), where
# `cargo test` exited 0. That default is kept deliberately — a filter that
# silently selects nothing is precisely the "green run that actually ran
# nothing" this script's `--check` exists to prevent — but it means a narrowing
# `CI_TEST_SHARD_EXTRA` filter (`-E …`) that matches nothing in THIS shard exits
# nonzero. An entirely empty shard is a different case and is still handled
# below, exiting 0 with a printed notice. Pass `--no-tests=pass` in
# CI_TEST_SHARD_EXTRA if you deliberately want an empty selection to succeed.
#
# CI_TEST_SHARD_PROFILE selects the nextest profile (default `ci`).
#
# Exit 0 = shard passed / partition proven. Nonzero = a violation, printed
# with the offender and the fix. Portable: bash 3.2+ (macOS), BSD and GNU
# userland, no GNU-only flags.

set -euo pipefail

cd "$(dirname "$0")/../.."

# The shipped defaults: the package and the Linux shard count. A bare `--check`
# (the Lint step) partitions at these; the macOS lane passes its own count, 3,
# on its `run:` line, and the coverage test in cerulion_cli_engine checks each
# lane's matrix against the count that lane passes.
DEFAULT_PACKAGE=cerulion_core
DEFAULT_SHARD_COUNT=4

# --------------------------------------------------------------------------
# The one routing exception (a deliberate decision).
#
# Read the header rule first — "THE LIST IS GENERATED, NEVER HAND-WRITTEN" —
# because this is a deliberate, single-name carve-out of it and nothing else
# may follow it in without the same evidence.
#
# WHY. `macro_compile_fail_test` held the serial trybuild tail: one test of a
# few minutes, essentially the whole of whichever shard held it. That suite is
# now its own package, `node_macro_ui`, whose tests ride this script's shard 2
# invocation on both lanes (`--with node_macro_ui`), and this binary keeps only the
# `#[ignore]`d rustc-rendering group, so by default it runs nothing. The pin
# still holds the file out of the rotation, which keeps every other file on the
# shard the package map in `.github/workflows/ci.yml` was balanced against:
# without it the file rejoins the rotation at its sorted position and every file
# after it moves to the next shard.
#
# WHY IT WAS PINNED rather than placed by the round-robin. The round-robin is
# over the SORTED file list, so a binary's shard is a function of how many
# `cerulion_core/tests/*.rs` sort before it: adding ANY test file had a
# ~1-in-`count` chance of moving the trybuild tail to another runner. The tail
# once sat on shard 1 and moved to shard 2 on ONE added file. That
# makes the fleet's critical path a lottery: no package-to-shard map can be
# balanced against a load that relocates on an unrelated PR, and the observed
# walls swung 24 -> 45 min on shards whose own contents never changed.
# Pinning converted the tail from a variable into a constant, which is what let
# the workflow's package map be packed around it; that trybuild work now runs
# as `node_macro_ui`'s tests in the same shard 2 invocation, and the package steps are placed
# largest-first around every shard's fixed term. See the map and its
# measurement in `.github/workflows/ci.yml` above the package steps.
#
# WHAT IS NOT CHANGED. New test files still round-robin, automatically, with no
# edit here — the generated-list property the header defends is intact for
# every other binary. This is one name, not a list, and `--check` PROVES the
# partition is still total and disjoint AND that the pin landed where declared
# (`do_check` below), so a future arithmetic slip is a red, not a silent drop.
#
# ADDING A SECOND NAME HERE NEEDS EVIDENCE: a measured wall showing one binary
# dominates a shard, and a map that is balanced around it. Absent that, the
# answer is the round-robin. The one name here no longer dominates its shard;
# it stays so the files after it keep their shards.
#
# PINNED_SHARD is taken `% count`, so the pin is always a valid index for any
# shard count: `--check cerulion_core 2` still partitions. TWO counts ship, 4 on
# Linux and 3 on macOS (each lane's `shard:` matrix and the count it passes to
# this script in `.github/workflows/ci.yml`), and `2 % 4` and `2 % 3` both land
# on shard 2. It is inert for any package that does not contain the file, so
# `ci_test_shard.sh cerulion_bag …` is unaffected.
PINNED_TEST=macro_compile_fail_test
PINNED_SHARD=2
# The package that OWNS the pin. `--check` on this package REQUIRES the pinned
# file to exist (see the fail-closed arm in `do_check`); on any other package
# the pin is inert. Separate from DEFAULT_PACKAGE, which happens to hold the
# same value but answers a different question ("what does CI shard?").
PINNED_PACKAGE=cerulion_core

die() {
    printf 'ci_test_shard: %s\n' "$1" >&2
    exit 2
}

# Validate one shard index / count argument: a DECIMAL non-negative integer,
# with no leading zero.
#
# The leading-zero refusal is not pedantry. `$(( ))` reads `08` as OCTAL and
# dies "value too great for base", and this script does shard arithmetic in
# `$(( ))`; on the `--check` path that is a loud exit, but on `--list`/run it
# printed one stderr line and then handed cargo an argument vector with NO
# `--test` flags AT ALL, exiting 0 — the "green step that ran nothing" this
# file's own `--check` exists to prevent. `10#$x` would fix the arithmetic, but
# it fixes it by ACCEPTING `08` as a second spelling of shard 8, and two
# spellings of one shard count is how a matrix and a script come to disagree
# while both look right. Refusing is the strictest reading and the only one
# that cannot be misread. It also closes the same ambiguity on the pre-existing
# decimal/octal seam, where `08` silently meant 8 to `awk` and something else
# to the shell.
require_index() {
    _what=$1; _val=$2
    case "$_val" in
        ''|*[!0-9]*) die "$_what must be a non-negative decimal integer (got '$_val')" ;;
        0) ;;
        0*) _bare="${_val#"${_val%%[!0]*}"}"
            die "$_what must not have a leading zero (got '$_val') — write it as ${_bare:-0}" ;;
    esac
}

# Print the whole leading comment block: line 2 down to the first line that is
# not a comment.
#
# DERIVED, NOT A LINE NUMBER. This was `sed -n '2,45p'`, and a magic number is
# the wrong tool for a range whose end moves whenever anyone edits the header
# above it. It had ALREADY drifted before this line was touched: at the merge
# base `2,45p` stopped in the middle of the `--list` description, so `--check`
# — one of the three modes — was missing from `--help` entirely, silently, and
# nothing could notice. Inserting eight lines above it then pushed the cut up
# to the literal `# USAGE` heading, taking the synopsis with it.
#
# A second magic number would have the same lifetime as the first. A sentinel
# comment would work but is one more thing to keep in the file; the block's own
# end — the first non-`#` line — is already unambiguous, needs no maintenance,
# and cannot be edited out of existence, so that is the bound. `--help` is now
# a superset of what it printed before, which is the point: the truncation was
# the bug, not the length.
usage() {
    awk 'NR == 1 { next } !/^#/ { exit } { print }' "$0" >&2
    exit 2
}

# Enumerate a package's integration-test files, one per line, sorted.
#
# DEPTH 1 ONLY (`tests/*.rs`), which is what makes fixture directories —
# `tests/ui/**` (trybuild sources, compiled by the harness, never their own
# target) and `tests/common/mod.rs` (shared helper modules) — fall out by
# construction rather than by an exclusion list that could go stale.
enumerate() {
    _dir=$1
    [ -d "$_dir/tests" ] || die "$_dir/tests does not exist: pass the package DIRECTORY after the shard count if it differs from the package name"
    # `ls` rather than a glob so an empty directory yields nothing instead of
    # the literal pattern; `basename`-with-suffix strips `.rs` portably.
    find "$_dir/tests" -maxdepth 1 -type f -name '*.rs' -print \
        | sed -e 's|.*/||' -e 's|\.rs$||' \
        | LC_ALL=C sort
}

# Print the members of one shard: round-robin by position, EXCEPT `$PINNED_TEST`,
# which is held out of the rotation and appended to shard `$PINNED_SHARD % count`
# (see the block above the constants for why that one name is pinned).
#
# ONE awk pass, not a `grep -v` pipeline, on purpose: under `set -o pipefail` a
# `grep` that matches nothing exits 1 and would take the whole command
# substitution down with it — for a package whose only test file happened to be
# the pinned one, and for every package that does not contain it.
shard_members() {
    _dir=$1; _index=$2; _count=$3
    enumerate "$_dir" | awk \
        -v idx="$_index" -v cnt="$_count" \
        -v pin="$PINNED_TEST" -v pinshard="$((PINNED_SHARD % _count))" '
        # Hold the pinned binary out of the rotation entirely, so the other
        # files close ranks and their positions stay contiguous.
        $0 == pin { pinned = $0; next }
        (pos++ % cnt) == idx { print }
        END { if (pinned != "" && idx == pinshard) print pinned }
    '
}

# Build the `cargo test` argument vector for one shard.
shard_args() {
    _pkg=$1; _dir=$2; _index=$3; _count=$4
    printf -- '-p\n%s\n' "$_pkg"
    # `--lib` rides shard 0 (see header).
    if [ "$_index" -eq 0 ]; then
        printf -- '--lib\n'
    fi
    shard_members "$_dir" "$_index" "$_count" | while IFS= read -r t; do
        [ -n "$t" ] || continue
        printf -- '--test\n%s\n' "$t"
    done
}

# The same selection, expressed as a nextest FILTERSET rather than as cargo
# target flags.
#
# WHY A SECOND SPELLING EXISTS. `-p` / `--lib` / `--test` are CARGO selection
# flags: they tell cargo what to BUILD. Running from an archive builds nothing,
# so nextest rejects them there, and the shard's membership has to be expressed
# over the binaries the archive already contains. The MEMBERSHIP itself is
# unchanged — both spellings walk the same `shard_members`, so a file moves
# shard in exactly one place (the map above) and both modes follow it.
#
# Emitted one TERM per line for the same reason `shard_args` is: the caller
# joins them, because a `while read` in a pipeline runs in a subshell and cannot
# hand a variable back.
shard_filterset_terms() {
    _pkg=$1; _dir=$2; _index=$3; _count=$4
    # The package's own unit tests ride shard 0, mirroring `--lib` above.
    if [ "$_index" -eq 0 ]; then
        printf -- 'kind(lib)\n'
    fi
    shard_members "$_dir" "$_index" "$_count" | while IFS= read -r t; do
        [ -n "$t" ] || continue
        printf -- 'binary(=%s)\n' "$t"
    done
}

# --------------------------------------------------------------------------
# THE SELECTION.
#
# `CI_SELECTED_PACKAGES` carries the `pkgs` output of ci.yml's `changes` job:
# the JSON array of the packages a pull request's change can reach, the reverse
# cargo closure unioned with the observation edges in
# `tools/ci/observation_edges.tsv`. This script runs ONE package, so the
# intersection of its member list with that array is either that package or
# nothing, and nothing means this shard has no work to do.
#
# FAIL CLOSED, twice. An unset or empty variable RUNS: a shard invoked by hand
# or by a job that publishes no selection must behave as it always did. A value
# that is not a JSON array RUNS too: a selection this script cannot read is not
# a selection it may act on, and the word `all` arrives here exactly that way.
#
# WHOLE TOKEN, quotes included, which is what keeps `cerulion_viz` and
# `cerulion_vizd` apart: a bare substring test would report `cerulion_viz`
# selected by an array holding only `cerulion_vizd`.
#
# READABLE MEANS A JSON ARRAY OF QUOTED NAMES, not merely a bracketed string.
# `[cerulion_bag]` reads as a selection under a brackets-only test and holds no
# quoted name, so the shard SKIPPED on a value no producer emits and no reader
# can parse, which is the one direction this function is documented never to
# take. Every shape below that is not `[]` or `["a"]` or `["a","b"]` is
# unreadable and RUNS.
selection_holds() {
    # $1 is the package this shard runs, $2 the selection.
    case "$2" in
        "") return 0 ;;
        \[*\]) ;;
        *) return 0 ;;
    esac
    _sel_inner=${2#\[}
    _sel_inner=${_sel_inner%\]}
    _sel_found=1
    _sel_rest=$_sel_inner
    while [ -n "$_sel_rest" ]; do
        _sel_tok=${_sel_rest%%,*}
        case "$_sel_rest" in
            *,*) _sel_rest=${_sel_rest#*,} ;;
            *)   _sel_rest= ;;
        esac
        # JSON puts optional whitespace around a value, so the token is trimmed
        # before it is judged: a producer that starts writing `["a", "b"]` must
        # not silently stop being readable.
        while :; do
            case "$_sel_tok" in
                " "*)  _sel_tok=${_sel_tok# } ;;
                *" ")  _sel_tok=${_sel_tok% } ;;
                *)     break ;;
            esac
        done
        # A QUOTED NAME, and nothing else: opening and closing quote, at least
        # one character between them, and no quote inside.
        case "$_sel_tok" in
            '"'*'"') ;;
            *) return 0 ;;
        esac
        _sel_name=${_sel_tok#\"}
        _sel_name=${_sel_name%\"}
        case "$_sel_name" in
            ''|*'"'*) return 0 ;;
        esac
        [ "$_sel_name" != "$1" ] || _sel_found=0
    done
    return "$_sel_found"
}

# The packages of one invocation that ride the selection, space separated, in
# command-line order: $1 is the shard's own package, $2 the `--with` packages
# (space separated), $3 the selection. Each package is read by
# `selection_holds` on its own, so a selection that names an extra and not the
# shard's package runs that extra and none of the shard's own files, and one
# that names only the shard's package runs no extra. In CI every selection that
# names a package names `cerulion_core`, which observes the whole tree, so the
# step sees `[]` or `cerulion_core` with or without each extra.
shard_riders() {
    _riders=
    for _rider in $1 $2; do
        if selection_holds "$_rider" "$3"; then
            _riders="${_riders:+$_riders }$_rider"
        fi
    done
    printf '%s\n' "$_riders"
}

# The directory of the workspace package $1: the one manifest under `crates/`
# whose `[package]` table carries that name. Anything but exactly one match is
# fatal, because a `--with` that found no package would run nothing for it.
package_dir_of() {
    _hits=$(find crates -maxdepth 4 -name Cargo.toml -not -path '*/target/*' -print | LC_ALL=C sort | while IFS= read -r _manifest; do
        if awk -v want="$1" '
            /^\[/ { inpkg = ($0 == "[package]"); next }
            inpkg && /^name[ \t]*=/ {
                v = $0; sub(/^name[ \t]*=[ \t]*"/, "", v); sub(/".*$/, "", v)
                found = (v == want); exit
            }
            END { exit found ? 0 : 1 }' "$_manifest"; then
            dirname "$_manifest"
        fi
    done)
    _nhits=$(printf '%s\n' "$_hits" | grep -c . || true)
    [ "$_nhits" -eq 1 ] || die "--with $1: $_nhits manifests under crates/ name that package, not exactly one${_hits:+ ($_hits)}"
    printf '%s\n' "$_hits"
}

# Refuse an extra package (name $1, directory $2) whose tests this invocation
# could not run in full:
#   * a test attribute under `src/` (`#[test]`, `#[<path>::test]`,
#     `#[test_case`, `#[rstest`): cargo builds an extra's library and binary
#     tests only under `--lib` or `--bins`, which would select the shard
#     package's own;
#   * a doctest's possible source under `src/`: a fenced block (three
#     backticks or tildes) or a line indented as code (five spaces or a tab
#     past the marker) in a `//!` or `///` comment, any `#[doc`/`#![doc` attribute (a `#[doc = "..."]`
#     string or an `include_str!` can hold a block too) or a `/**`/`/*!` block
#     comment, because nextest runs no doctest;
#   * a test target the depth-1 list does not name, or test code outside
#     `src/`: a `[[test]]` table, a `tests/<dir>/main.rs`, a `[[bin]]` without
#     `test = false`, an `[[example]]` or `[[bench]]` with `test = true`, or a
#     `[lib]` whose `path` is outside `src/`;
#   * an integration test named like one of the shard package's, because
#     `--test` selects that name in every package of the command.
# The test attribute forms include the ones the coverage walk in
# `cerulion_cli_engine/tests/ci_test_coverage_test.rs` reads as tests. Called
# in this shell, so `die` ends the script.
check_extra() {
    _unit=$(grep -rlE -e '#\[([A-Za-z_][A-Za-z0-9_]*::)*test[](]' -e '#\[(test_case|rstest)' -- "$2/src" 2>/dev/null | LC_ALL=C sort | head -n 1 || true)
    [ -z "$_unit" ] || die "--with $1: $_unit holds a unit test, which this invocation cannot run (move it to $2/tests/)"
    _tab=$(printf '\t')
    _doc=$(grep -rlE -e '^[[:space:]]*//[/!].*(```|~~~)' -e "^[[:space:]]*//[/!]( {5}| *$_tab)" -e '#!?\[doc' -e '/\*[*!]' -- "$2/src" 2>/dev/null | LC_ALL=C sort | head -n 1 || true)
    [ -z "$_doc" ] || die "--with $1: $_doc holds a doc comment code block or a doc attribute, and nextest runs no doctest"
    grep -q '^\[\[test\]\]' "$2/Cargo.toml" \
        && die "--with $1: $2/Cargo.toml declares a [[test]] target, which the tests/*.rs list does not name"
    _nested=$(find "$2/tests" -mindepth 2 -maxdepth 2 -name main.rs -print 2>/dev/null | LC_ALL=C sort | head -n 1 || true)
    [ -z "$_nested" ] || die "--with $1: $_nested is a test target the tests/*.rs list does not name"
    # One pass over the manifest's target tables; the first offence decides.
    _target=$(awk '
        function close_table() { if (table == "[[bin]]" && !off && !why) why = "a [[bin]] lacks `test = false`, so its tests would run only under --bins" }
        /^\[/ { close_table(); table = $0; off = 0; next }
        table == "[[bin]]" && /^test[ \t]*=[ \t]*false/ { off = 1 }
        (table == "[[example]]" || table == "[[bench]]") && /^test[ \t]*=[ \t]*true/ && !why { why = "a " table " has `test = true`, which the tests/*.rs list does not name" }
        table == "[lib]" && /^path[ \t]*=/ && !/^path[ \t]*=[ \t]*"src\// && !why { why = "its [lib] path is outside src/, where the unit test scan does not look" }
        END { close_table(); print why }' "$2/Cargo.toml")
    [ -z "$_target" ] || die "--with $1: in $2/Cargo.toml, $_target"
    [ -d "$2/tests" ] || die "--with $1: $2/tests does not exist, so it would add nothing to the shard"
    _xtests=$(enumerate "$2")
    [ -n "$_xtests" ] || die "--with $1: $2/tests holds no *.rs files, so it would add nothing to the shard"
    while IFS= read -r _xt; do
        if grep -qxF -- "$_xt" <<<"$ALL_TESTS"; then
            die "--with $1: tests/$_xt.rs shares its name with $PKG_DIR/tests/$_xt.rs, and --test $_xt would run both"
        fi
    done <<<"$_xtests"
}

# The hand table `--check` holds `selection_holds` to.
#
# `<package>|<selection>|<runs>`. Written out rather than generated: the rows
# say what the answer IS, so a reader that changed its mind fails here instead
# of agreeing with itself.
#
# THE MALFORMED ROWS PAIR WITH A WELL-FORMED SKIP, and the pair differs only in
# the quoting: `["cerulion_bag"]` is readable and skips `cerulion_core`, while
# `[cerulion_bag]` is not a JSON array of quoted names, so it is unreadable and
# RUNS. `[cerulion_core]` is the same shape naming the package itself, so the
# answer cannot come from a match. The spaced rows say the trimming is real in
# both directions.
SELECTION_CASES='
cerulion_core|["cerulion_core"]|yes
cerulion_core|["cerulion_bag","cerulion_core"]|yes
cerulion_core|["cerulion_bag"]|no
cerulion_core|[]|no
cerulion_core||yes
cerulion_core|all|yes
cerulion_core|not json|yes
cerulion_core|[cerulion_bag]|yes
cerulion_core|[cerulion_core]|yes
cerulion_core|["cerulion_bag]|yes
cerulion_core|[cerulion_bag,"cerulion_core"]|yes
cerulion_core|[""]|yes
cerulion_core|["cerulion_bag", "cerulion_viz"]|no
cerulion_core|["cerulion_bag", "cerulion_core"]|yes
cerulion_viz|["cerulion_viz"]|yes
cerulion_viz|["cerulion_vizd"]|no
cerulion_vizd|["cerulion_viz"]|no
cerulion-wire|["cerulion-wire"]|yes
'

# The hand table `--check` holds `shard_riders` to: `<package and extras>|
# <selection>|<riders>`, `-` for none. The second row is a selection of the
# diagnostics suite alone: it runs that suite and none of the shard's own
# files. An unreadable selection runs every package, as `selection_holds` reads
# it. The order row says the riders follow the command line, not the selection.
SHARD_SELECTION_CASES='
cerulion_core node_macro_ui|["cerulion_core"]|cerulion_core
cerulion_core node_macro_ui|["node_macro_ui"]|node_macro_ui
cerulion_core node_macro_ui|["cerulion_core","node_macro_ui"]|cerulion_core node_macro_ui
cerulion_core node_macro_ui|["cerulion_bag"]|-
cerulion_core node_macro_ui|[]|-
cerulion_core node_macro_ui||cerulion_core node_macro_ui
cerulion_core node_macro_ui|all|cerulion_core node_macro_ui
cerulion_core node_macro_ui|[node_macro_ui]|cerulion_core node_macro_ui
cerulion_core|["node_macro_ui"]|-
cerulion_core cerulion_viz|["cerulion_vizd"]|-
cerulion_core node_macro_ui cerulion_viz|["cerulion_viz","node_macro_ui"]|node_macro_ui cerulion_viz
cerulion_core node_macro_ui test_node_macro_cdylib|["test_node_macro_cdylib"]|test_node_macro_cdylib
cerulion_core node_macro_ui test_node_macro_cdylib|["test_node_macro_cdylib","node_macro_ui"]|node_macro_ui test_node_macro_cdylib
cerulion_core node_macro_ui test_node_macro_cdylib|["cerulion_core","test_node_macro_cdylib"]|cerulion_core test_node_macro_cdylib
cerulion_core node_macro_ui test_node_macro_cdylib|[]|-
'

check_selection() {
    _fails=0
    while IFS= read -r _row; do
        [ -n "$_row" ] || continue
        _pkg=${_row%%|*}
        _rest=${_row#*|}
        _sel=${_rest%|*}
        _want=${_rest##*|}
        if selection_holds "$_pkg" "$_sel"; then _got=yes; else _got=no; fi
        if [ "$_got" != "$_want" ]; then
            printf 'VIOLATION: selection_holds %s "%s" -> %s, wanted %s\n' \
                "$_pkg" "$_sel" "$_got" "$_want" >&2
            _fails=$((_fails + 1))
        fi
    done <<< "$SELECTION_CASES"
    while IFS= read -r _row; do
        [ -n "$_row" ] || continue
        _pkgs=${_row%%|*}
        _rest=${_row#*|}
        _sel=${_rest%|*}
        _want=${_rest##*|}
        _got=$(shard_riders "${_pkgs%% *}" "$(printf '%s' "$_pkgs" | sed 's/^[^ ]*//')" "$_sel")
        if [ "${_got:--}" != "$_want" ]; then
            printf 'VIOLATION: shard_riders "%s" "%s" -> %s, wanted %s\n' \
                "$_pkgs" "$_sel" "${_got:--}" "$_want" >&2
            _fails=$((_fails + 1))
        fi
    done <<< "$SHARD_SELECTION_CASES"
    [ "$_fails" -eq 0 ] || exit 1
    printf 'ci_test_shard --check: OK, the selection reader agrees with its hand tables.\n'
}

# --------------------------------------------------------------------------
# --check: the partition is TOTAL and DISJOINT.
#
# A shard runner that silently DROPPED a test file would be worse than the
# hand list it replaces — the whole point is that nothing goes unrun — so the
# property is asserted rather than argued from the `mod` arithmetic.
# --------------------------------------------------------------------------
do_check() {
    _pkg=${1:-$DEFAULT_PACKAGE}
    _count=${2:-$DEFAULT_SHARD_COUNT}
    _dir=${3:-crates/$_pkg}

    require_index "shard count" "$_count"
    [ "$_count" -ge 1 ] || die "shard count must be >= 1 (got '$_count')"

    _all=$(enumerate "$_dir")
    _total=$(printf '%s\n' "$_all" | grep -c . || true)
    [ "$_total" -gt 0 ] || die "$_dir/tests holds no *.rs files — nothing to shard"

    _union=""
    _sum=0
    _i=0
    while [ "$_i" -lt "$_count" ]; do
        _members=$(shard_members "$_dir" "$_i" "$_count")
        _n=$(printf '%s\n' "$_members" | grep -c . || true)
        _sum=$((_sum + _n))
        printf 'shard %d/%d: %d test file(s)\n' "$_i" "$_count" "$_n"
        _union="$_union
$_members"
        _i=$((_i + 1))
    done

    # DISJOINT: the shards' sizes must sum to the total. A file assigned to
    # two shards inflates the sum; one assigned to none deflates it.
    if [ "$_sum" -ne "$_total" ]; then
        printf 'VIOLATION: shard sizes sum to %d but %s/tests holds %d file(s).\n' "$_sum" "$_dir" "$_total" >&2
        printf 'The partition is not disjoint-and-total. Fix shard_members() in %s.\n' "$0" >&2
        exit 1
    fi

    # TOTAL: the union, deduplicated, must be the full list byte for byte.
    _got=$(printf '%s\n' "$_union" | grep . | LC_ALL=C sort -u)
    if [ "$_got" != "$_all" ]; then
        printf 'VIOLATION: the union of the shards is not %s/tests.\n' "$_dir" >&2
        printf 'Missing from the shards (these would run in NO job):\n' >&2
        printf '%s\n' "$_all" | LC_ALL=C comm -23 - <(printf '%s\n' "$_got") >&2 || true
        printf 'Fix shard_members() in %s.\n' "$0" >&2
        exit 1
    fi

    # PINNED: the one routing exception must land on the shard it DECLARES —
    # and, on the package that owns the pin, must still EXIST.
    #
    # Totality and disjointness above already prove the pinned binary runs
    # exactly once — they do NOT prove it runs where the workflow's package map
    # was balanced to expect. That map counts the serial tail as a FIXED term
    # of the pinned shard's wall and packs the package steps around it; a pin
    # that silently drifted would leave two shards mis-sized with every gate
    # green.
    #
    # FAIL CLOSED on absence. Guarding this whole block on
    # "the pinned file is present" would make a rename or a delete report
    # SUCCESS with no `pinned:` line at all: the tail would rejoin the
    # round-robin (landing on shard 0, the heaviest) and the only signal was a
    # line of output nobody was asserting on. An exemption that stops
    # describing anything must fail rather than evaporate, so on
    # $PINNED_PACKAGE an absent pin target is a VIOLATION naming both real
    # remedies. Other packages stay inert — they never had the file.
    # A here-string, not a pipe: `grep -q` exits on the match, and a `printf`
    # still writing the rest of the list would take SIGPIPE, which `pipefail`
    # reads as "no such file".
    if grep -qx -- "$PINNED_TEST" <<<"$_all"; then
        _pin_shard=$((PINNED_SHARD % _count))
        _pin_found=$(shard_members "$_dir" "$_pin_shard" "$_count" | grep -cx "$PINNED_TEST" || true)
        if [ "$_pin_found" -ne 1 ]; then
            printf 'VIOLATION: %s is pinned to shard %d/%d but is not in it.\n' \
                "$PINNED_TEST" "$_pin_shard" "$_count" >&2
            printf 'The workflow package map in .github/workflows/ci.yml is balanced around\n' >&2
            printf 'the other files staying where the pin leaves them. Fix shard_members() in %s.\n' "$0" >&2
            exit 1
        fi
        printf 'pinned: %s -> shard %d/%d (the one routing exception)\n' \
            "$PINNED_TEST" "$_pin_shard" "$_count"
    elif [ "$_pkg" = "$PINNED_PACKAGE" ]; then
        printf 'VIOLATION: %s pins %s, but %s/tests holds no such file.\n' \
            "$PINNED_PACKAGE" "$PINNED_TEST" "$_dir" >&2
        printf 'The pin is now describing nothing, and the file positions the workflow\n' >&2
        printf 'package map is balanced against no longer hold.\n' >&2
        printf 'Fix ONE of:\n' >&2
        printf '  * restore %s/tests/%s.rs (it was renamed or deleted), or\n' "$_dir" "$PINNED_TEST" >&2
        printf '  * update PINNED_TEST in %s to the new name AND re-check the package\n' "$0" >&2
        printf '    map in .github/workflows/ci.yml still matches shard %d.\n' "$PINNED_SHARD" >&2
        exit 1
    fi

    printf 'ci_test_shard --check: OK — %d test file(s) partitioned across %d shard(s) of %s, total and disjoint.\n' \
        "$_total" "$_count" "$_pkg"

    check_selection
}

# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
[ $# -ge 1 ] || usage

MODE=run
case "${1:-}" in
    --check) shift; do_check "${1:-}" "${2:-}" "${3:-}"; exit 0 ;;
    --list)  shift; MODE=list ;;
    --selected)
        # The SAME `selection_holds` the run mode below gates on, asked as a
        # question by a workflow step that builds something BEFORE this script's
        # own run step is reached. A second reading of the selection written in
        # YAML would be free to drift from this one; a mode cannot, and
        # `--check`'s table already pins the function's answers. rc 0 means the
        # package rides this selection, rc 1 means it does not, and the line
        # says which either way, because a command with no output proves nothing
        # to whoever reads the log.
        shift
        [ $# -ge 1 ] || usage
        if selection_holds "$1" "${CI_SELECTED_PACKAGES:-}"; then
            printf 'selection: %s rides the selection (selected: %s)\n' \
                "$1" "${CI_SELECTED_PACKAGES:-}"
            exit 0
        fi
        printf 'selection: %s is not in the selection (selected: %s)\n' \
            "$1" "${CI_SELECTED_PACKAGES:-}"
        exit 1
        ;;
    -h|--help) usage ;;
    -*) die "unknown option '$1' (expected --check, --list, --selected or a package name)" ;;
esac

[ $# -ge 3 ] || usage
PACKAGE=$1
INDEX=$2
COUNT=$3
shift 3
# After the three shard arguments: the package directory, at most once, and any
# number of `--with <package>`, in any order.
PKG_DIR=
EXTRAS=
while [ $# -gt 0 ]; do
    case "$1" in
        --with)
            [ $# -ge 2 ] || die "--with needs a package name"
            case "$2" in
                ''|-*|*[!A-Za-z0-9_-]*) die "--with needs a package name (got '$2')" ;;
            esac
            [ "$2" != "$PACKAGE" ] || die "--with $2 names the shard's own package"
            case " $EXTRAS " in
                *" $2 "*) die "--with $2 is given twice" ;;
            esac
            EXTRAS="${EXTRAS:+$EXTRAS }$2"
            shift 2
            ;;
        -*) die "unknown option '$1' after the shard arguments (expected --with <package>)" ;;
        *)
            [ -z "$PKG_DIR" ] || die "a second package directory '$1' (the first is '$PKG_DIR')"
            PKG_DIR=$1
            shift
            ;;
    esac
done
PKG_DIR=${PKG_DIR:-crates/$PACKAGE}

require_index "shard index" "$INDEX"
require_index "shard count" "$COUNT"
[ "$COUNT" -ge 1 ] || die "shard count must be >= 1 (got '$COUNT')"
[ "$INDEX" -lt "$COUNT" ] || die "shard index $INDEX is out of range for $COUNT shard(s)"
[ "$INDEX" -ne 0 ] || [ -z "$EXTRAS" ] \
    || die "--with on shard 0: that shard carries --lib, and cargo would hand it to every --with package too"

# THE SELECTION, before anything is enumerated or built, in the run mode and
# in `--list` alike. One line with the fixed `selection:` prefix for each
# package the selection leaves out (on stderr under `--list`, whose stdout is
# the argument list), then exit 0 without invoking cargo when none rides. A
# `--with` package rides on its own reading, so a selection that names it and
# not this package runs it and none of this package's files.
#
# The workflow step that runs this shard is deliberately NOT gated on the selection:
# `cerulion_core` observes the whole repository tree (its whole-tree walks,
# `serial_discipline_test` among them, read every `.rs` file in the source tree), and
# `tools/ci/observation_edges.tsv` records it as observing `all`; a package
# observing `all` rides every selection that names a package. A pull request that
# selects no package at all (one touching only a root markdown file that no
# doc-pin marker names) reaches this step with the selection `[]`; the step then
# prints its `selection:` line and exits without running the suite. Every step
# gated on the selection naming its package skips for that change too, and each
# package's companion step prints its own `selection:` line; the steps of the packages
# observing `all` carry no selection condition and still run, the `cerulion_core`
# doctests step among them (gated on the shard index alone), and among those only
# this sharded suite skips, by the runner's own reading of the selection. The
# intersection below is the mechanism for any package whose shard step is added
# later and whose observation edges the walk can attribute.
RIDERS=$(shard_riders "$PACKAGE" "$EXTRAS" "${CI_SELECTED_PACKAGES:-}")
for _left in $PACKAGE $EXTRAS; do
    case " $RIDERS " in
        *" $_left "*) continue ;;
    esac
    if [ "$_left" = "$PACKAGE" ]; then
        _line="selection: skipped $PACKAGE shard $INDEX (selected: ${CI_SELECTED_PACKAGES:-})"
    else
        _line="selection: skipped $_left beside $PACKAGE shard $INDEX (selected: ${CI_SELECTED_PACKAGES:-})"
    fi
    if [ "$MODE" = list ]; then
        printf '%s\n' "$_line" >&2
    else
        printf '%s\n' "$_line"
    fi
done
[ -n "$RIDERS" ] || exit 0
rides() {
    case " $RIDERS " in
        *" $1 "*) return 0 ;;
    esac
    return 1
}

# ENUMERATE FIRST, IN THIS SHELL. `enumerate` is otherwise only ever reached
# from inside a `$( … )`, where its `die` exits the SUBSHELL: a missing or
# renamed `tests/` directory printed one line to stderr and the run carried on
# with an EMPTY argument vector, i.e. `cargo test -- --test-threads=1` with no
# `-p` at all — a green step that ran something else entirely. A standalone
# assignment propagates the failure under `set -e`, so the same `die` is fatal
# here.
#
# TOTAL == 0 is likewise a failure, not an empty shard: a legitimately empty
# INDIVIDUAL shard (more shards than files) is reported below and exits 0, but
# a package with no test files at all means the directory moved.
ALL_TESTS=$(enumerate "$PKG_DIR")
TOTAL_TESTS=$(printf '%s\n' "$ALL_TESTS" | grep -c . || true)
[ "$TOTAL_TESTS" -gt 0 ] \
    || die "$PKG_DIR/tests holds no *.rs files, so there is nothing to shard (pass the package DIRECTORY after the shard count if it differs from the package name)"

# Every `--with` package, checked whether or not it rides, so a wrong one fails
# on every change that runs any package of this invocation, not only on the
# changes that select it. A change that runs none exited above.
# `<package>:<directory>` pairs, space separated.
EXTRA_PAIRS=
for _extra in $EXTRAS; do
    _extra_dir=$(package_dir_of "$_extra")
    check_extra "$_extra" "$_extra_dir"
    EXTRA_PAIRS="${EXTRA_PAIRS:+$EXTRA_PAIRS }$_extra:$_extra_dir"
done

# ARCHIVE MODE (CI's build-once path). When `CI_TEST_SHARD_ARCHIVE` names an
# existing nextest archive, this shard RUNS the binaries that job already built
# instead of building its own copy — which is the whole point: four shards used
# to link the same ~180 test binaries.
#
# Two assertions before a single test runs, because a stale or foreign archive
# must fail LOUDLY rather than quietly run the wrong code:
#
#   * the RUSTC the archive was built with must equal this shard's. Different
#     compilers mean different binaries, and an archive whose provenance we have
#     not checked is exactly the thing a shared directory makes possible.
#   * `CARGO_PROFILE_DEV_DEBUG` must match too. It is not cosmetic here: the
#     archive job and the shards must agree on it or the artifacts differ, and a
#     silent mismatch would show up only as a confusing debuginfo or timing
#     difference much later.
#
# Both are compared against values the ARCHIVE JOB recorded, not against
# anything read out of the shared directory itself.
ARCHIVE=${CI_TEST_SHARD_ARCHIVE:-}
if [ -n "$ARCHIVE" ]; then
    [ -f "$ARCHIVE" ] || die "CI_TEST_SHARD_ARCHIVE points at '$ARCHIVE', which does not exist.
The archive-resolution step must run before this one (it either copies the
pool's local archive or downloads the artifact), and it must fail rather than
leave this variable set."
    _want_rustc=${CI_TEST_SHARD_ARCHIVE_RUSTC:-}
    if [ -n "$_want_rustc" ]; then
        _have_rustc=$(rustc -V 2>/dev/null || echo "rustc: not found")
        [ "$_want_rustc" = "$_have_rustc" ] || die "toolchain DRIFT between the archive job and this shard:
  archive built with: $_want_rustc
  this shard runs:    $_have_rustc
Running one compiler's test binaries under another is not a supported
configuration, and silently rebuilding would defeat the build-once job. Pin both
jobs to the same toolchain."
    fi
    _want_debug=${CI_TEST_SHARD_ARCHIVE_PROFILE_DEBUG:-}
    _have_debug=${CARGO_PROFILE_DEV_DEBUG:-}
    [ "$_want_debug" = "$_have_debug" ] || die "CARGO_PROFILE_DEV_DEBUG DRIFT between the archive job and this shard:
  archive: '$_want_debug'
  shard:   '$_have_debug'
Both jobs must carry the SAME value, or they are not talking about the same
artifacts."
fi

# Read the argument vector into a positional list (bash 3.2 has no readarray).
# RUNNING collects the packages that bring at least one target to it.
RUNNING=
set -- # clear
OLDIFS=$IFS
IFS='
'
if [ -n "$ARCHIVE" ]; then
    # `--archive-file` runs what was built elsewhere; `--workspace-remap` tells
    # nextest where THIS checkout is, since the archive records the path of the
    # tree it was built from and the two jobs need not share one.
    set -- --archive-file "$ARCHIVE" --workspace-remap "$PWD"
    # WHERE THE ARCHIVE IS UNPACKED, and it is not a detail.
    #
    # nextest extracts into `$TMPDIR` by default. In a job container that is
    # `/tmp`, which is far smaller than the uncompressed test tree: extracting
    # there dies on all four shards with
    # `error writing file target/debug/deps/… No space left on device (os error
    # 28)` after `Extracting 263 binaries to /tmp/nextest-archive-…`. The archive
    # transport is fine — the step has already logged that it verified the
    # pool's local copy by sha256 — so the failure is the DESTINATION, not the
    # source.
    #
    # `$CI_TEST_SHARD_EXTRACT_DIR` puts it on the work volume instead, and when
    # the caller does not set one nextest keeps its own default, so a developer
    # running this locally is unaffected. `--extract-overwrite` because a re-run
    # of the step lands in a directory that already exists.
    if [ -n "${CI_TEST_SHARD_EXTRACT_DIR:-}" ]; then
        mkdir -p "$CI_TEST_SHARD_EXTRACT_DIR"
        set -- "$@" --extract-to "$CI_TEST_SHARD_EXTRACT_DIR" --extract-overwrite
    fi
    # The "this shard selected nothing" vector, captured rather than counted by
    # hand: it is whatever the base arguments above came to, BEFORE any
    # filterset is appended. A literal count (4, say) is silently falsified
    # by any argument added above — and the cost of that is not a
    # red run, it is an EMPTY shard reporting green.
    _empty_argc=$#
    # One clause per riding package: this package's shard, then each extra's
    # binaries. One clause is passed as it stands; several are joined by `or`.
    _clauses=""
    _nclauses=0
    _filter=""
    if rides "$PACKAGE"; then
        for term in $(shard_filterset_terms "$PACKAGE" "$PKG_DIR" "$INDEX" "$COUNT"); do
            if [ -z "$_filter" ]; then
                _filter="$term"
            else
                _filter="$_filter + $term"
            fi
        done
    fi
    # An empty shard keeps the SAME meaning it has in build mode (reported
    # below, exit 0) rather than degenerating into "run everything".
    if [ -n "$_filter" ]; then
        _clauses="package($PACKAGE) and ($_filter)"
        _nclauses=1
        RUNNING=$PACKAGE
    fi
    IFS=$OLDIFS
    for _pair in $EXTRA_PAIRS; do
        rides "${_pair%%:*}" || continue
        RUNNING="${RUNNING:+$RUNNING }${_pair%%:*}"
        _filter=""
        for t in $(enumerate "${_pair#*:}"); do
            _filter="${_filter:+$_filter + }binary(=$t)"
        done
        if [ "$_nclauses" -eq 0 ]; then
            _clauses="package(${_pair%%:*}) and ($_filter)"
        elif [ "$_nclauses" -eq 1 ]; then
            _clauses="($_clauses) or (package(${_pair%%:*}) and ($_filter))"
        else
            _clauses="$_clauses or (package(${_pair%%:*}) and ($_filter))"
        fi
        _nclauses=$((_nclauses + 1))
    done
    IFS='
'
    if [ -n "$_clauses" ]; then
        set -- "$@" -E "$_clauses"
    fi
else
    # This package's `-p` only with at least one target after it: `-p` alone
    # selects every test target the package has.
    _empty_argc=0
    if rides "$PACKAGE"; then
        _own=$(shard_args "$PACKAGE" "$PKG_DIR" "$INDEX" "$COUNT")
        if [ "$(printf '%s\n' "$_own" | grep -c .)" -gt 2 ]; then
            for a in $_own; do
                set -- "$@" "$a"
            done
            RUNNING=$PACKAGE
        fi
    fi
    IFS=$OLDIFS
    for _pair in $EXTRA_PAIRS; do
        rides "${_pair%%:*}" || continue
        RUNNING="${RUNNING:+$RUNNING }${_pair%%:*}"
        set -- "$@" -p "${_pair%%:*}"
        for t in $(enumerate "${_pair#*:}"); do
            set -- "$@" --test "$t"
        done
    done
    IFS='
'
fi
IFS=$OLDIFS

if [ "$MODE" = list ]; then
    printf '%s\n' "$@"
    exit 0
fi

# An empty shard is legitimate (more shards than files) but must be visible in
# the log rather than reported as a silent green.
#
# The "nothing selected" argument vector differs by mode: in build mode it is
# empty, because this package's `-p` is added only with a target after it and
# an extra always brings at least one; in archive mode the archive branch
# RECORDED its own base count above, because that base is conditional (the
# extract directory adds two words when the caller names one) and a
# hand-counted constant would go stale the next time a flag is added there.
# Either way the filterset is omitted entirely rather than emitted empty,
# because an empty `-E` selects EVERYTHING.
if [ $# -eq "$_empty_argc" ]; then
    printf 'ci_test_shard: shard %s/%s of %s is EMPTY (fewer test files than shards) — nothing to run.\n' \
        "$INDEX" "$COUNT" "$PACKAGE"
    exit 0
fi

if [ -n "$EXTRAS" ]; then
    case " $RUNNING " in
        *" $PACKAGE "*) ;;
        *)
            rides "$PACKAGE" && printf 'ci_test_shard: shard %s/%s of %s holds none of its files (fewer test files than shards)\n' \
                "$INDEX" "$COUNT" "$PACKAGE"
            ;;
    esac
    printf 'ci_test_shard: shard %s/%s of %s, running %s\n' "$INDEX" "$COUNT" "$PACKAGE" "$RUNNING"
else
    printf 'ci_test_shard: shard %s/%s of %s\n' "$INDEX" "$COUNT" "$PACKAGE"
fi

# AGENTS.md's pre-commit list and the PR template both tell contributors to run
# this script, so the missing-tool case is a LOCAL one and deserves the fix
# rather than cargo's bare "no such command: nextest".
command -v cargo-nextest >/dev/null 2>&1 || die \
    "cargo-nextest is not installed — this shard runs under nextest now.
Install the pinned version with:  ./tools/scripts/install_nextest.sh
(see .config/nextest.toml for the serial fence it applies)"
# NO `--test-threads=1`. nextest runs each test in its own PROCESS, so the
# process-global hazards that flag was defending (the cdylib `NODES` registry,
# `set_var`, the `#[traced_test]` global subscriber, a counting allocator)
# cannot reach across tests at all. What genuinely must not run beside a
# sibling — the DEFAULT iceoryx2 namespace, and the wall-clock latency gates —
# is fenced by name in `.config/nextest.toml`, and that membership is gated by
# `cerulion_core/tests/serial_discipline_test.rs`.
set -x
# CI_TEST_SHARD_EXTRA is a caller-supplied ARGUMENT LIST, so the word
# splitting shellcheck warns about is the whole mechanism; quoting it would
# pass "--no-capture --run-ignored all" as ONE argument.
# shellcheck disable=SC2086
exec cargo nextest run --profile "${CI_TEST_SHARD_PROFILE:-ci}" "$@" ${CI_TEST_SHARD_EXTRA:-}
