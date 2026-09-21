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
# and not a list: `macro_compile_fail_test` is PINNED to a fixed shard, because
# it is a multi-minute serial trybuild tail whose round-robin position — and
# therefore which runner pays it — changes every time an unrelated test file is
# added. Everything else still round-robins with no edit here. The evidence,
# the arithmetic and the bar for adding a second name are at `PINNED_TEST`
# below; `--check` proves the pin landed where it says it did.
#
# `--lib` (the package's own unit tests) rides shard 0. It is a single target,
# it is cheap, and it has to go somewhere exactly once.
#
# USAGE
#   ci_test_shard.sh <package> <shard_index> <shard_count> [package_dir]
#   ci_test_shard.sh --list  <package> <shard_index> <shard_count> [package_dir]
#   ci_test_shard.sh --check [package] [shard_count] [package_dir]
#
#   run     — invoke `cargo nextest run --profile <p> -p <package> --lib?
#             --test a --test b ...`
#   --list  — print the cargo argument list and exit (no cargo, no build);
#             what the CI log shows and what the tests assert against.
#   --check — prove the partition is TOTAL and DISJOINT: the union of all
#             shards equals the full file list exactly once. Defaults to the
#             package + shard count CI actually uses, so a bare `--check` is
#             the shipped configuration.
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

# The shipped configuration — the ONE place the defaults live, so `--check`
# and the CI matrix cannot describe different partitions.
DEFAULT_PACKAGE=cerulion_core
DEFAULT_SHARD_COUNT=4

# --------------------------------------------------------------------------
# The one routing exception (a deliberate decision).
#
# Read the header rule first — "THE LIST IS GENERATED, NEVER HAND-WRITTEN" —
# because this is a deliberate, single-name carve-out of it and nothing else
# may follow it in without the same evidence.
#
# WHY. `macro_compile_fail_test::compile_fail_tests` is a trybuild harness: it
# invokes rustc once per `tests/ui/*.rs` case, inside ONE test, serially.
# THE SIZE IS A PER-RUN MEASUREMENT, and this block is where it is RECORDED — the
# reading and the earlier samples. Everything else refers to
# "the serial tail" without a figure, with ONE deliberate exception: the shard-map
# block in `.github/workflows/ci.yml` restates it at MINUTE scale (~4.4 vs the
# ~9.2 its sums were built on) because its worked arithmetic is in minutes and an
# argument you cannot follow is worse than a second copy. That is the whole of the
# duplication, and it is named in both directions. Latest: 266.771 s
# (~4.4 min), 99.3 % of its shard's nextest run wall, on `Test (Linux) shard 2`
# (2026-09-12). On the self-hosted Linux
# runners, six green runs on 2026-09-14 put shard 2's
# whole nextest run wall at 228.7 / 247.6 / 296.3 / 265.7 / 250.7 / 428.3 s,
# against 12-22 s for every other shard's — and in the one of
# those runs whose log names every test this test alone was
# 227.0 s of that 228.7. Earlier: 545.2 s and 559.3 s
# on two green main runs, ~97 % of that wall. The
# RATIO is what has held across every sample — this one test is essentially the
# whole of whichever quarter holds it, against 20-38 s for every other shard's.
# That cost is not addressable here: splitting it by expectation WITHIN the
# same job costs +158 s of trybuild work and +2.4 min of shard-step wall,
# so it stays whole.
#
# WHY PIN IT rather than let the round-robin place it. The round-robin is over
# the SORTED file list, so the tail's shard is a function of how many
# `cerulion_core/tests/*.rs` sort before it — i.e. adding ANY test file has a
# ~1-in-4 chance of moving that whole block onto a different runner. It sat on
# shard 1 for both runs above and moved to shard 2 on ONE added file. That
# makes the fleet's critical path a lottery: no package-to-shard map can be
# balanced against a load that relocates on an unrelated PR, and the observed
# walls swung 24 -> 45 min on shards whose own contents never changed.
# Pinning converts the tail from a variable into a constant, which is what lets
# the workflow's package map be packed around it: the tail is one FIXED term of
# shard 2's measured wall, and the package steps are placed largest-first
# around all four shards' fixed terms. See the map and its measurement in
# `.github/workflows/ci.yml` above the package steps.
#
# WHAT IS NOT CHANGED. New test files still round-robin, automatically, with no
# edit here — the generated-list property the header defends is intact for
# every other binary. This is one name, not a list, and `--check` PROVES the
# partition is still total and disjoint AND that the pin landed where declared
# (`do_check` below), so a future arithmetic slip is a red, not a silent drop.
#
# ADDING A SECOND NAME HERE NEEDS THE SAME EVIDENCE: a measured wall showing
# one binary dominates a shard, and a map that is balanced around it. Absent
# that, the answer is the round-robin.
#
# PINNED_SHARD is taken `% count`, so the pin is always a valid index for any
# shard count (the shipped count is 4; `--check cerulion_core 2` still
# partitions). It is inert for any package that does not contain the file, so
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
    [ -d "$_dir/tests" ] || die "$_dir/tests does not exist — pass the package DIRECTORY as the 4th argument if it differs from the package name"
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
    if printf '%s\n' "$_all" | grep -qx "$PINNED_TEST"; then
        _pin_shard=$((PINNED_SHARD % _count))
        _pin_found=$(shard_members "$_dir" "$_pin_shard" "$_count" | grep -cx "$PINNED_TEST" || true)
        if [ "$_pin_found" -ne 1 ]; then
            printf 'VIOLATION: %s is pinned to shard %d/%d but is not in it.\n' \
                "$PINNED_TEST" "$_pin_shard" "$_count" >&2
            printf 'The workflow package map in .github/workflows/ci.yml is balanced around\n' >&2
            printf 'that shard carrying this serial tail. Fix shard_members() in %s.\n' "$0" >&2
            exit 1
        fi
        printf 'pinned: %s -> shard %d/%d (the one routing exception)\n' \
            "$PINNED_TEST" "$_pin_shard" "$_count"
    elif [ "$_pkg" = "$PINNED_PACKAGE" ]; then
        printf 'VIOLATION: %s pins %s, but %s/tests holds no such file.\n' \
            "$PINNED_PACKAGE" "$PINNED_TEST" "$_dir" >&2
        printf 'The pin is now describing nothing, and the serial trybuild tail it\n' >&2
        printf 'names has silently rejoined the round-robin — which is what the pin\n' >&2
        printf 'exists to prevent, and what the workflow package map is balanced against.\n' >&2
        printf 'Fix ONE of:\n' >&2
        printf '  * restore %s/tests/%s.rs (it was renamed or deleted), or\n' "$_dir" "$PINNED_TEST" >&2
        printf '  * update PINNED_TEST in %s to the new name AND re-check the package\n' "$0" >&2
        printf '    map in .github/workflows/ci.yml still counts shard %d as carrying the tail.\n' "$PINNED_SHARD" >&2
        exit 1
    fi

    printf 'ci_test_shard --check: OK — %d test file(s) partitioned across %d shard(s) of %s, total and disjoint.\n' \
        "$_total" "$_count" "$_pkg"
}

# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
[ $# -ge 1 ] || usage

MODE=run
case "${1:-}" in
    --check) shift; do_check "${1:-}" "${2:-}" "${3:-}"; exit 0 ;;
    --list)  shift; MODE=list ;;
    -h|--help) usage ;;
    -*) die "unknown option '$1' (expected --check, --list or a package name)" ;;
esac

[ $# -ge 3 ] || usage
PACKAGE=$1
INDEX=$2
COUNT=$3
PKG_DIR=${4:-crates/$PACKAGE}

require_index "shard index" "$INDEX"
require_index "shard count" "$COUNT"
[ "$COUNT" -ge 1 ] || die "shard count must be >= 1 (got '$COUNT')"
[ "$INDEX" -lt "$COUNT" ] || die "shard index $INDEX is out of range for $COUNT shard(s)"

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
    || die "$PKG_DIR/tests holds no *.rs files — nothing to shard (pass the package DIRECTORY as the 4th argument if it differs from the package name)"

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
    _filter=""
    for term in $(shard_filterset_terms "$PACKAGE" "$PKG_DIR" "$INDEX" "$COUNT"); do
        if [ -z "$_filter" ]; then
            _filter="$term"
        else
            _filter="$_filter + $term"
        fi
    done
    # An empty shard keeps the SAME meaning it has in build mode (reported
    # below, exit 0) rather than degenerating into "run everything".
    if [ -n "$_filter" ]; then
        set -- "$@" -E "package($PACKAGE) and ($_filter)"
    fi
else
    for a in $(shard_args "$PACKAGE" "$PKG_DIR" "$INDEX" "$COUNT"); do
        set -- "$@" "$a"
    done
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
# just `-p <pkg>` (2 words); in archive mode the archive branch RECORDED its own
# base count above, because that base is now conditional (the extract directory
# adds two words when the caller names one) and a hand-counted constant would go
# stale the next time a flag is added there. Either way the filterset is omitted
# entirely rather than emitted empty, because an empty `-E` selects EVERYTHING.
[ -n "$ARCHIVE" ] || _empty_argc=2
if [ "$INDEX" -gt 0 ] && [ $# -eq "$_empty_argc" ]; then
    printf 'ci_test_shard: shard %s/%s of %s is EMPTY (fewer test files than shards) — nothing to run.\n' \
        "$INDEX" "$COUNT" "$PACKAGE"
    exit 0
fi

printf 'ci_test_shard: shard %s/%s of %s\n' "$INDEX" "$COUNT" "$PACKAGE"

# AGENTS.md's pre-commit list and the PR template both tell contributors to run
# this script, so the missing-tool case is a LOCAL one and deserves the fix
# rather than cargo's bare "no such command: nextest".
command -v cargo-nextest >/dev/null 2>&1 || die \
    "cargo-nextest is not installed — this shard runs under nextest now.
Install the pinned version with:  ./scripts/install_nextest.sh
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
