#!/usr/bin/env bash
# advisory_issue.sh: record one `cargo deny check advisories` result on one issue.
#
#   tools/scripts/advisory_issue.sh <exit-status> <log-path>
#
# WHAT COUNTS AS A FINDING. `tools/release/deny.toml`'s `[advisories]` section
# sets `version = 2` and `yanked = "warn"`: a vulnerability denies and the run
# exits nonzero, while the unmaintained, unsound, notice and yanked classes emit
# a `warning[<class>]` diagnostic and the run exits 0. THE RULE IMPLEMENTED HERE
# READS BOTH: a finding is a nonzero exit status OR a log line matching
# `warning[(unmaintained|unsound|notice|yanked)]`. An advisory listed in the
# `ignore` table of that file emits no `warning[...]` line and is not a finding.
#
# WHAT IT WRITES. One issue carries the result. It is found among the open
# issues labelled `security` by the exact title in TITLE below.
#
#   finding, no such issue    create it, labelled `security`
#   finding, one such issue   rewrite its body
#   clean, one such issue     comment the commit the check passed on, then close it
#   clean, no such issue      no write at all
#
# REFUSALS, exit 2 with one message each: `gh` is unauthenticated; the
# repository carries no `security` label; `gh` refused a listing; two open
# issues carry the title. The unauthenticated check runs FIRST, so the
# label-missing message never stands in for a token that cannot read.
#
# ENVIRONMENT. `GH_TOKEN` is read by `gh` itself. `GITHUB_SERVER_URL`,
# `GITHUB_REPOSITORY`, `GITHUB_RUN_ID` and `GITHUB_SHA` build the run URL and
# name the lockfile commit; `GH_REPO` stands in for `GITHUB_REPOSITORY`.
#
# Oracle: tools/scripts/test_advisory_issue.sh.

set -euo pipefail

TITLE='Dependency advisory audit fails on the committed lockfile'
LABEL=security
WARN_CLASSES='warning\[(unmaintained|unsound|notice|yanked)\]'
POLICY='tools/release/deny.toml'

die() {
    printf 'advisory_issue: %s\n' "$1" >&2
    exit "$2"
}

[ "$#" -eq 2 ] || die 'usage: advisory_issue.sh <exit-status> <log-path>' 2
status=$1
log=$2
case $status in
    '' | *[!0-9]*) die "the exit status is not a number: ${status}" 2 ;;
esac
[ -r "$log" ] || die "the log cannot be read: ${log}" 2

server=${GITHUB_SERVER_URL:-https://github.com}
repository=${GITHUB_REPOSITORY:-${GH_REPO:-}}
run_id=${GITHUB_RUN_ID:-}
commit=${GITHUB_SHA:-}
[ -n "$repository" ] || die 'GITHUB_REPOSITORY and GH_REPO are both empty: the run names no repository' 2
[ -n "$run_id" ] || die 'GITHUB_RUN_ID is empty: the body would carry no run URL' 2
[ -n "$commit" ] || die 'GITHUB_SHA is empty: the body would name no lockfile commit' 2
run_url="${server}/${repository}/actions/runs/${run_id}"

# An unauthenticated `gh` fails every call below with the same shape as a
# missing label, so the token is checked on its own first.
gh auth status >/dev/null 2>&1 ||
    die 'gh is not authenticated: GH_TOKEN carries no token that can read this repository' 2

# Captured into a variable rather than piped: under `pipefail` a refused listing
# piped into `grep` is indistinguishable from a listing that lacks the label.
if ! labels=$(gh label list --limit 200 --json name --jq '.[].name' 2>&1); then
    die "gh refused the label listing: ${labels}" 2
fi
printf '%s\n' "$labels" | grep -Fxq "$LABEL" ||
    die "the repository carries no \`${LABEL}\` label: .github/labels.yml declares it and the label sync creates it" 2

if ! listing=$(gh issue list --state open --label "$LABEL" --limit 200 \
    --json number,title --jq '.[] | [.number, .title] | @tsv' 2>&1); then
    die "gh refused the issue listing: ${listing}" 2
fi

matches=
count=0
while IFS=$'\t' read -r number title; do
    [ -n "${number:-}" ] || continue
    [ "${title:-}" = "$TITLE" ] || continue
    matches="${matches}${matches:+ }${number}"
    count=$((count + 1))
done <<< "$listing"
[ "$count" -le 1 ] ||
    die "issues ${matches} all carry the title \"${TITLE}\": one issue carries this result" 2

finding=0
[ "$status" -eq 0 ] || finding=1

# `grep` exits 1 on no match and 2 or more on a read error. Collapsing the two
# would report a log it never scanned as carrying no warning, so the status is
# read: 1 leaves the verdict alone, anything above it refuses.
warn_status=0
grep -Eq "$WARN_CLASSES" "$log" || warn_status=$?
[ "$warn_status" -le 1 ] ||
    die "grep exited ${warn_status} on ${log}: the warn-level classes went unscanned" 2
[ "$warn_status" -ne 0 ] || finding=1

id_status=0
ids_found=$(grep -oE 'RUSTSEC-[0-9]{4}-[0-9]{4}' "$log") || id_status=$?
[ "$id_status" -le 1 ] ||
    die "grep exited ${id_status} on ${log}: the advisory ids went unread" 2
ids=$(printf '%s\n' "$ids_found" | LC_ALL=C sort -u | paste -sd, - | sed 's/,/, /g')

body_file=$(mktemp "${TMPDIR:-/tmp}/advisory-issue.XXXXXX")
trap 'rm -f "$body_file"' EXIT

if [ "$finding" -eq 1 ]; then
    {
        printf '%s\n\n' "\`cargo deny --config ${POLICY} check advisories\` reports a finding against the lockfile committed at \`${commit}\`."
        if [ -n "$ids" ]; then
            printf 'Advisory ids in the run log: %s\n\n' "$ids"
        else
            printf '%s\n\n' 'The run log names no advisory id. Read the run for the diagnostics it printed.'
        fi
        printf 'Run: %s\n\n' "$run_url"
        printf '%s\n\n' "\`${POLICY}\` is the policy this check and the per-change dependency audit both read: a vulnerability fails the check, and the unmaintained, unsound, notice and yanked classes are warnings that this issue reports as well. An advisory accepted in the \`ignore\` table of that file is not reported here."
        printf '%s\n' "Every id above is already public in the RustSec database and names a third-party crate. A vulnerability in Cerulion itself goes to the address in \`.github/SECURITY.md\`, not here."
    } > "$body_file"
    if [ "$count" -eq 0 ]; then
        gh issue create --title "$TITLE" --label "$LABEL" --body-file "$body_file"
        printf 'advisory_issue: opened the issue carrying the advisory result\n'
    else
        gh issue edit "$matches" --body-file "$body_file"
        printf 'advisory_issue: rewrote the body of issue %s\n' "$matches"
    fi
    exit 0
fi

if [ "$count" -eq 0 ]; then
    printf 'advisory_issue: the check is clean and no issue is open; nothing to write\n'
    exit 0
fi

{
    printf '%s\n\n' "\`cargo deny --config ${POLICY} check advisories\` passes on the lockfile committed at \`${commit}\`, with no advisory warning in the run log."
    printf 'Run: %s\n' "$run_url"
} > "$body_file"
gh issue comment "$matches" --body-file "$body_file"
gh issue close "$matches"
printf 'advisory_issue: the check is clean; closed issue %s\n' "$matches"
