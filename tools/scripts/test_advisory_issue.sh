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
# Portable: bash 3.2+, GNU and BSD userland. No network.

set -euo pipefail

CASES_EXPECTED=9

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

TITLE='Dependency advisory audit fails on the committed lockfile'
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

# A clean advisories run: the trailing summary and no diagnostic.
printf '%s\n' 'advisories ok' > "$workdir/log_clean"
# A denied vulnerability: nonzero exit, two ids, one of them twice.
{
    printf '%s\n' 'error[vulnerability]: a crate in the lockfile is vulnerable'
    printf '%s\n' '    = ID: RUSTSEC-2026-0101'
    printf '%s\n' '    = ID: RUSTSEC-2026-0101'
    printf '%s\n' '    = ID: RUSTSEC-2026-0007'
    printf '%s\n' 'advisories FAILED'
} > "$workdir/log_vulnerability"
# A warn-only class: exit 0, and no id on the line cargo-deny prints for it.
{
    printf '%s\n' 'warning[yanked]: detected yanked crate (try cargo update -p chacha20)'
    printf '%s\n' 'advisories ok'
} > "$workdir/log_warning_only"

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

# --- a denied vulnerability with no issue open opens one -------------------
run_case create 1 "$workdir/log_vulnerability"
expect_status 0
expect_output 'opened the issue carrying the advisory result'
expect_call "issue create --title $TITLE --label security --body-file"
expect_no_call 'issue edit'
expect_body "reports a finding against the lockfile committed at \`0123456789abcdef0123456789abcdef01234567\`"
expect_body 'Advisory ids in the run log: RUSTSEC-2026-0007, RUSTSEC-2026-0101'
expect_body 'Run: https://github.invalid/owner/repo/actions/runs/5150'
expect_body "A vulnerability in Cerulion itself goes to the address in \`.github/SECURITY.md\`"
printf '%s\n' 'a finding with no issue open creates one: passed'

# --- the same finding with the issue already open rewrites its body --------
FAKE_GH_ISSUES="$workdir/issues_one" run_case update 1 "$workdir/log_vulnerability"
expect_status 0
expect_output 'rewrote the body of issue 42'
expect_call 'issue edit 42 --body-file'
expect_no_call 'issue create'
expect_body 'Advisory ids in the run log: RUSTSEC-2026-0007, RUSTSEC-2026-0101'
printf '%s\n' 'a finding with the issue open rewrites its body: passed'

# --- a clean run with the issue open comments the commit and closes it -----
FAKE_GH_ISSUES="$workdir/issues_one" run_case close 0 "$workdir/log_clean"
expect_status 0
expect_output 'closed issue 42'
expect_call 'issue comment 42 --body-file'
expect_call 'issue close 42'
expect_body "passes on the lockfile committed at \`0123456789abcdef0123456789abcdef01234567\`"
expect_body 'Run: https://github.invalid/owner/repo/actions/runs/5150'
printf '%s\n' 'a clean run closes the open issue: passed'

# --- a clean run with no issue open writes nothing -------------------------
run_case nothing_to_do 0 "$workdir/log_clean"
expect_status 0
expect_output 'nothing to write'
expect_no_writes
expect_call 'issue list --state open --label security'
printf '%s\n' 'a clean run with no issue open writes nothing: passed'

# --- a warn-only class on exit 0 is still a finding ------------------------
# This is the whole reason the status alone is not the rule: `yanked` warns.
run_case warning_only 0 "$workdir/log_warning_only"
expect_status 0
expect_output 'opened the issue carrying the advisory result'
expect_call "issue create --title $TITLE --label security --body-file"
expect_body 'The run log names no advisory id'
printf '%s\n' 'a warning-only log on exit 0 opens the issue: passed'

# --- a repository without the label refuses before any issue call ----------
FAKE_GH_LABELS="$workdir/labels_without_security" run_case missing_label 1 "$workdir/log_vulnerability"
expect_status 2
expect_output "the repository carries no \`security\` label"
expect_no_call 'issue list'
expect_no_writes
printf '%s\n' 'a missing label refuses before any issue call: passed'

# --- an unauthenticated gh refuses before the label listing ----------------
FAKE_GH_AUTH=1 run_case unauthenticated 1 "$workdir/log_vulnerability"
expect_status 2
expect_output 'gh is not authenticated'
expect_no_call 'label list'
expect_no_writes
printf '%s\n' 'an unauthenticated gh refuses before the label listing: passed'

# --- a refused label listing is NOT reported as a missing label ------------
# Under `pipefail` a listing piped into `grep` would report both the same way.
FAKE_GH_LABEL_REFUSED=1 run_case label_list_refused 1 "$workdir/log_vulnerability"
expect_status 2
expect_output 'gh refused the label listing'
expect_output 'HTTP 403'
expect_no_call 'issue list'
printf '%s\n' 'a refused label listing is told apart from a missing label: passed'

# --- two issues carrying the title refuse rather than pick one -------------
FAKE_GH_ISSUES="$workdir/issues_two" run_case duplicate_titles 1 "$workdir/log_vulnerability"
expect_status 2
expect_output 'issues 42 91 all carry the title'
expect_no_writes
printf '%s\n' 'two issues carrying the title refuse: passed'

[ "$cases" -eq "$CASES_EXPECTED" ] ||
    fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_advisory_issue: all %s cases passed\n' "$cases"
