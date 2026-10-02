#!/usr/bin/env bash
# test_advisory_issue.sh: the oracle table for tools/scripts/advisory_issue.sh.
#
# `gh` is a PATH shim. It appends every invocation to one file, answers
# `auth status`, `label list` and `issue list` from fixtures, and copies the
# `--body-file` content of every write into a second file, so a case asserts
# both which calls were made and what text they carried. One case per
# behaviour; the first failing assertion prints one `FAIL:` line and exits 1.
# The case count is asserted against CASES_EXPECTED at the end: a case deleted
# or never reached fails the run rather than passing quietly.
#
# Every log fixture is cargo-deny 0.20.2 `--format json` output: one JSON object
# per line, keys in the order serde_json writes them. The yanked diagnostic is
# the line a real run printed against the committed lockfile; the rest are built
# to the same serializer's shape, and each says what in it is not from a run.
# Portable: bash 3.2+, GNU and BSD userland. Needs `jq`. No network.
#
# The fixtures below, and the strings asserted against them, quote cargo-deny
# output that carries backticks. None of it is a shell expansion.
# shellcheck disable=SC2016

set -euo pipefail

CASES_EXPECTED=13

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-advisory-issue-test.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

cases=0
case_name=
case_dir=
output=
status=0

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    if [ -n "${output:-}" ]; then
        printf '%s\n' '--- output ---' "$output" >&2
    fi
    if [ -n "${case_dir:-}" ] && [ -f "$case_dir/calls" ]; then
        printf '%s\n' '--- gh calls ---' >&2
        cat "$case_dir/calls" >&2
    fi
    exit 1
}

command -v jq >/dev/null 2>&1 || fail 'jq is not on PATH: advisory_issue.sh reads the JSON log with it'

shim_bin="$workdir/bin"
mkdir -p "$shim_bin"

cat > "$shim_bin/gh" <<'SHIM'
#!/bin/sh
printf '%s\n' "$*" >> "$FAKE_GH_CALLS"
case "$1 $2" in
    'auth status')
        exit "${FAKE_GH_AUTH:-0}"
        ;;
    'label list')
        if [ "${FAKE_GH_LABEL_REFUSED:-0}" != 0 ]; then
            printf 'gh: HTTP 403: Resource not accessible\n' >&2
            exit 1
        fi
        cat "$FAKE_GH_LABELS"
        ;;
    'issue list')
        cat "$FAKE_GH_ISSUES"
        ;;
    'issue create' | 'issue edit' | 'issue comment')
        prev=
        for arg in "$@"; do
            if [ "$prev" = --body-file ]; then
                cat "$arg" >> "$FAKE_GH_BODY"
            fi
            prev=$arg
        done
        printf 'https://github.invalid/owner/repo/issues/42\n'
        ;;
    'issue close')
        ;;
    *)
        printf 'fake gh: unexpected invocation: %s\n' "$*" >&2
        exit 99
        ;;
esac
exit 0
SHIM
chmod 0755 "$shim_bin/gh"

printf '%s\n' bug documentation security pinned > "$workdir/labels_with_security"
printf '%s\n' bug documentation pinned > "$workdir/labels_without_security"

TITLE='Dependency advisory audit reports a finding on the committed lockfile'
: > "$workdir/issues_none"
# The matching row is NOT last: the title loop must select it wherever it sits.
{
    printf '%s\t%s\n' 42 "$TITLE"
    printf '%s\t%s\n' 7 'Pairing fails on a cold start'
} > "$workdir/issues_one"
{
    printf '%s\t%s\n' 42 "$TITLE"
    printf '%s\t%s\n' 91 "$TITLE"
} > "$workdir/issues_two"

# A clean run: the trailing summary, no diagnostic, exit 0.
printf '%s\n' '{"fields":{"advisories":{"errors":0,"helps":0,"notes":0,"warnings":0}},"type":"summary"}' > "$workdir/log_clean"

# A yanked crate. The diagnostic line is what a run of
# `cargo deny --format json --config tools/release/deny.toml check advisories`
# printed against the committed lockfile, with the `graphs[0].parents` chain
# dropped so the line fits here; a crate reached directly carries this shape.
# `yanked = "warn"` is why it is `warning` and why the run still exited 0.
{
    printf '%s\n' '{"fields":{"code":"yanked","graphs":[{"Krate":{"name":"chacha20","version":"0.10.1"}}],"labels":[{"column":1,"line":128,"message":"yanked version","span":"chacha20 0.10.1 registry+https://github.com/rust-lang/crates.io-index"}],"message":"detected yanked crate (try `cargo update -p chacha20`)","severity":"warning"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"advisories":{"errors":0,"helps":0,"notes":6,"warnings":1}},"type":"summary"}'
} > "$workdir/log_yanked"

# Two advisories that failed the check: `error[<class>]` under `version = 2`,
# and a nonzero exit. The ids here are not real RustSec ids, and the `advisory`
# object a run also attaches to such a diagnostic is left out: the rule reads
# severity, class, crate and id, and `notes[0]` carries the id on every one.
{
    printf '%s\n' '{"fields":{"code":"vulnerability","graphs":[{"Krate":{"name":"mio","version":"0.6.23"}}],"labels":[{"column":1,"line":512,"message":"security vulnerability detected","span":"mio 0.6.23 registry+https://github.com/rust-lang/crates.io-index"}],"message":"a fixture vulnerability title","notes":["ID: RUSTSEC-2026-0101","Advisory: https://rustsec.org/advisories/RUSTSEC-2026-0101","A fixture description.","Solution: No safe upgrade is available!"],"severity":"error"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"code":"unmaintained","graphs":[{"Krate":{"name":"mio-extras","version":"2.0.6"}}],"labels":[{"column":1,"line":517,"message":"unmaintained advisory detected","span":"mio-extras 2.0.6 registry+https://github.com/rust-lang/crates.io-index"}],"message":"a fixture unmaintained title","notes":["ID: RUSTSEC-2026-0007","Advisory: https://rustsec.org/advisories/RUSTSEC-2026-0007","A fixture description.","Solution: No safe upgrade is available!"],"severity":"error"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"advisories":{"errors":2,"helps":0,"notes":0,"warnings":0}},"type":"summary"}'
} > "$workdir/log_errors"

# An advisory accepted in deny.toml's `ignore` table. Its own diagnostic drops
# to `note` severity and a second `note` records the ignore, which is why the
# default `--log-level warn` prints neither; both are here so the severity bound
# is exercised rather than assumed. Synthetic id, exit 0.
{
    printf '%s\n' '{"fields":{"code":"advisory-ignored","graphs":[{"Krate":{"name":"paste","version":"1.0.15"}}],"labels":[{"column":7,"line":36,"message":"advisory ignored here","span":"RUSTSEC-2026-0042"},{"column":48,"line":36,"message":"ignore reason","span":"a fixture reason"}],"message":"advisory ignored","severity":"note"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"code":"unmaintained","graphs":[{"Krate":{"name":"paste","version":"1.0.15"}}],"labels":[{"column":1,"line":1104,"message":"unmaintained advisory detected","span":"paste 1.0.15 registry+https://github.com/rust-lang/crates.io-index"}],"message":"a fixture unmaintained title","notes":["ID: RUSTSEC-2026-0042","Advisory: https://rustsec.org/advisories/RUSTSEC-2026-0042","A fixture description.","Solution: No safe upgrade is available!"],"severity":"note"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"advisories":{"errors":0,"helps":0,"notes":2,"warnings":0}},"type":"summary"}'
} > "$workdir/log_ignored"

# cargo-deny could not run at all. Under `--format json` its own log goes out as
# JSON too, with an upper-case level, and a run that died before the check
# prints no diagnostic and no summary.
printf '%s\n' '{"fields":{"level":"ERROR","message":"failed to fetch advisory database: Could not resolve host: github.com","timestamp":"2026-10-02T04:37:11.481293Z"},"type":"log"}' > "$workdir/log_tool_failure"

# A registry index query that failed. cargo-deny emits one of these per crate it
# could not query, at WARNING severity, while `yanked` is not "allow", and the
# run exits 0: the message, the class, the two labels and the `notes[0]` error
# string are the shape it builds, with the `graphs[0].parents` chain dropped as
# above. The crate here is the one the committed lockfile yanks, so the fixture
# stands for the run whose yanked half went unrun. Exit 0 and a summary object
# are both present, which is exactly why neither the exit status nor the severity
# bound can route this.
{
    printf '%s\n' '{"fields":{"code":"index-failure","graphs":[{"Krate":{"name":"chacha20","version":"0.10.1"}}],"labels":[{"column":1,"line":128,"message":"crate whose registry we failed to query","span":"chacha20 0.10.1 registry+https://github.com/rust-lang/crates.io-index"},{"column":10,"line":59,"message":"lint level defined here","span":"\"warn\""}],"message":"unable to check for yanked crates","notes":["failed to query crate chacha20: error sending request"],"severity":"warning"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"advisories":{"errors":0,"helps":0,"notes":6,"warnings":1}},"type":"summary"}'
} > "$workdir/log_index_query_failed"

# A nonzero exit whose only error is cargo-deny's own: it counts against the
# advisories check, so the status is nonzero, and its class is not an advisory.
{
    printf '%s\n' '{"fields":{"code":"index-cache-load-failure","graphs":[],"message":"failed to load index cache","notes":["No such file or directory (os error 2)"],"severity":"error"},"type":"diagnostic"}'
    printf '%s\n' '{"fields":{"advisories":{"errors":1,"helps":0,"notes":0,"warnings":0}},"type":"summary"}'
} > "$workdir/log_index_failure"

# run_case <name> <status> <log>; FAKE_GH_* may be set on the call to override a
# fixture. Leaves $output, $status and the recorded calls and bodies in $case_dir.
run_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    mkdir -p "$case_dir"
    : > "$case_dir/calls"
    : > "$case_dir/body"
    shift
    set +e
    output=$(
        PATH="$shim_bin:$PATH" \
            GH_TOKEN=fake-token \
            GITHUB_SERVER_URL=https://github.invalid \
            GITHUB_REPOSITORY=owner/repo \
            GITHUB_RUN_ID=5150 \
            GITHUB_SHA=0123456789abcdef0123456789abcdef01234567 \
            FAKE_GH_CALLS="$case_dir/calls" \
            FAKE_GH_BODY="$case_dir/body" \
            FAKE_GH_LABELS="${FAKE_GH_LABELS:-$workdir/labels_with_security}" \
            FAKE_GH_ISSUES="${FAKE_GH_ISSUES:-$workdir/issues_none}" \
            FAKE_GH_AUTH="${FAKE_GH_AUTH:-0}" \
            FAKE_GH_LABEL_REFUSED="${FAKE_GH_LABEL_REFUSED:-0}" \
            "$script_dir/advisory_issue.sh" "$@" 2>&1
    )
    status=$?
    set -e
    cases=$((cases + 1))
}

expect_status() {
    [ "$status" -eq "$1" ] || fail "$case_name: expected exit $1, got $status"
}

expect_output() {
    printf '%s\n' "$output" | grep -Fq "$1" ||
        fail "$case_name: the output does not carry '$1'"
}

expect_call() {
    grep -Fq "$1" "$case_dir/calls" ||
        fail "$case_name: no gh call carries '$1'"
}

expect_no_call() {
    grep -Fq "$1" "$case_dir/calls" &&
        fail "$case_name: a gh call carries '$1' and must not"
    return 0
}

expect_body() {
    grep -Fq "$1" "$case_dir/body" ||
        fail "$case_name: the issue body does not carry '$1'"
}

expect_no_writes() {
    grep -Eq '^issue (create|edit|comment|close)' "$case_dir/calls" &&
        fail "$case_name: a write call was made and must not be"
    return 0
}

expect_no_gh_at_all() {
    [ -s "$case_dir/calls" ] &&
        fail "$case_name: gh was called and must not be"
    return 0
}

# --- advisories that failed the check, with no issue open, open one ---------
run_case create 1 "$workdir/log_errors"
expect_status 0
expect_output 'opened the issue carrying the advisory result'
expect_call "issue create --title $TITLE --label security --body-file"
expect_no_call 'issue edit'
expect_body "reports a finding against the lockfile committed at \`0123456789abcdef0123456789abcdef01234567\`, and exited 1."
expect_body '2 advisory diagnostic(s) failed the check (`error[...]`).'
expect_body '* `error[vulnerability]` mio 0.6.23, RUSTSEC-2026-0101: a fixture vulnerability title'
expect_body '* `error[unmaintained]` mio-extras 2.0.6, RUSTSEC-2026-0007: a fixture unmaintained title'
expect_body 'Run: https://github.invalid/owner/repo/actions/runs/5150'
expect_body 'A vulnerability or notice advisory on any crate in the resolved graph fails the check'
expect_body 'An unsound advisory is checked at scope `workspace`'
expect_body "A vulnerability in Cerulion itself goes to the address in \`.github/SECURITY.md\`"
printf '%s\n' 'advisories that failed the check open the issue: passed'

# --- the same finding with the issue already open rewrites its body --------
FAKE_GH_ISSUES="$workdir/issues_one" run_case update 1 "$workdir/log_errors"
expect_status 0
expect_output 'rewrote the body of issue 42'
expect_call 'issue edit 42 --body-file'
expect_no_call 'issue create'
expect_body '* `error[vulnerability]` mio 0.6.23, RUSTSEC-2026-0101'
printf '%s\n' 'a finding with the issue open rewrites its body: passed'

# --- a clean run with the issue open comments the commit and closes it -----
FAKE_GH_ISSUES="$workdir/issues_one" run_case close 0 "$workdir/log_clean"
expect_status 0
expect_output 'closed issue 42'
expect_call 'issue comment 42 --body-file'
expect_call 'issue close 42'
expect_body "exited 0 on the lockfile committed at \`0123456789abcdef0123456789abcdef01234567\`"
expect_body 'carries no `error[...]` advisory diagnostic, no `warning[yanked]`, and no `warning[index-failure]`'
expect_body 'Run: https://github.invalid/owner/repo/actions/runs/5150'
printf '%s\n' 'a clean run closes the open issue: passed'

# --- a clean run with no issue open writes nothing -------------------------
run_case nothing_to_do 0 "$workdir/log_clean"
expect_status 0
expect_output 'nothing to write'
expect_no_writes
expect_call 'issue list --state open --label security'
printf '%s\n' 'a clean run with no issue open writes nothing: passed'

# --- a yanked crate on exit 0 is still a finding, and is named -------------
# This is the whole reason the exit status alone is not the rule: yanked warns.
run_case yanked 0 "$workdir/log_yanked"
expect_status 0
expect_output 'opened the issue carrying the advisory result'
expect_call "issue create --title $TITLE --label security --body-file"
expect_body "reports a finding against the lockfile committed at \`0123456789abcdef0123456789abcdef01234567\`, and exited 0."
expect_body '1 yanked crate(s) warned (`warning[yanked]`, which does not fail the check).'
expect_body '* `warning[yanked]` chacha20 0.10.1: detected yanked crate'
printf '%s\n' 'a yanked crate on exit 0 opens the issue and is named: passed'

# --- an advisory accepted in the ignore table is not a finding -------------
# Both of its diagnostics sit below warn level, so neither can carry a verdict.
run_case accepted 0 "$workdir/log_ignored"
expect_status 0
expect_output 'nothing to write'
expect_no_writes
printf '%s\n' 'an accepted advisory is not a finding: passed'

# --- a nonzero exit with no advisory diagnostic writes nothing -------------
# The open issue stays exactly as it is, and the message names the run.
FAKE_GH_ISSUES="$workdir/issues_one" run_case tool_failure 1 "$workdir/log_tool_failure"
expect_status 3
expect_output 'cargo deny exited 1 and its log carries no advisory diagnostic'
expect_output 'no issue was written'
expect_output 'Run: https://github.invalid/owner/repo/actions/runs/5150'
expect_no_gh_at_all
printf '%s\n' 'a nonzero exit with no advisory diagnostic writes nothing: passed'

# --- a nonzero exit carrying only cargo-deny's own error does the same -----
FAKE_GH_ISSUES="$workdir/issues_one" run_case index_failure 1 "$workdir/log_index_failure"
expect_status 3
expect_output 'the check did not complete'
expect_no_gh_at_all
printf '%s\n' "cargo-deny's own error is not an advisory finding: passed"

# --- a failed registry index query routes to exit 3 on exit 0 --------------
# `warning[index-failure]` arrives at warn level and exit 0, so the clean path
# would otherwise comment and close the issue over a yanked check that did not
# run for the crate it names.
FAKE_GH_ISSUES="$workdir/issues_one" run_case index_query_failed 0 "$workdir/log_index_query_failed"
expect_status 3
expect_output 'carrying 1 `warning[index-failure]` diagnostic(s), on chacha20 0.10.1'
expect_output 'the registry query failed, so the yanked half of the check did not run'
expect_output 'Run: https://github.invalid/owner/repo/actions/runs/5150'
expect_no_gh_at_all
printf '%s\n' 'a failed registry index query routes to exit 3 on exit 0: passed'

# --- a repository without the label refuses before any issue call ----------
FAKE_GH_LABELS="$workdir/labels_without_security" run_case missing_label 1 "$workdir/log_errors"
expect_status 2
expect_output "the repository carries no \`security\` label"
expect_no_call 'issue list'
expect_no_writes
printf '%s\n' 'a missing label refuses before any issue call: passed'

# --- an unauthenticated gh refuses before the label listing ----------------
FAKE_GH_AUTH=1 run_case unauthenticated 1 "$workdir/log_errors"
expect_status 2
expect_output 'gh is not authenticated'
expect_no_call 'label list'
expect_no_writes
printf '%s\n' 'an unauthenticated gh refuses before the label listing: passed'

# --- a refused label listing is NOT reported as a missing label ------------
# Under `pipefail` a listing piped into `grep` would report both the same way.
FAKE_GH_LABEL_REFUSED=1 run_case label_list_refused 1 "$workdir/log_errors"
expect_status 2
expect_output 'gh refused the label listing'
expect_output 'HTTP 403'
expect_no_call 'issue list'
printf '%s\n' 'a refused label listing is told apart from a missing label: passed'

# --- two issues carrying the title refuse rather than pick one -------------
FAKE_GH_ISSUES="$workdir/issues_two" run_case duplicate_titles 1 "$workdir/log_errors"
expect_status 2
expect_output 'issues 42 91 all carry the title'
expect_no_writes
printf '%s\n' 'two issues carrying the title refuse: passed'

[ "$cases" -eq "$CASES_EXPECTED" ] ||
    fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_advisory_issue: all %s cases passed\n' "$cases"
