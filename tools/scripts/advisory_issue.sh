#!/usr/bin/env bash
# advisory_issue.sh: record one `cargo deny check advisories` result on one issue.
#
#   tools/scripts/advisory_issue.sh <exit-status> <log-path>
#
# THE LOG IS cargo-deny's `--format json` output with both streams merged: one
# JSON object per line. A diagnostic is `{"fields":{"code":...,"graphs":[...],
# "labels":[...],"message":...,"severity":...},"type":"diagnostic"}`, and the
# crate it is about is each `graphs[].Krate`. A check that ran to the end also
# prints `{"fields":{"advisories":{...}},"type":"summary"}`; a tool error
# arrives as `{"fields":{"level":"ERROR",...},"type":"log"}`. `jq` reads the
# lines it can parse and skips the rest, so a plain-text line in the log never
# ends the scan.
#
# WHAT THIS REPORTS, and what it leaves silent. Under `tools/release/deny.toml`,
# cargo-deny 0.20.2 checks each class over a different set of crates:
#
#   vulnerability   every crate in the resolved graph. `error[vulnerability]`,
#                   and the check exits nonzero.
#   notice          every crate in the resolved graph. `error[notice]`, nonzero.
#   unmaintained    the `unmaintained` scope key, which that file leaves at its
#                   default `all`: every crate. `error[unmaintained]`, nonzero.
#   unsound         the `unsound` scope key, which that file leaves at its
#                   default `workspace`: only a crate at least one of whose
#                   DIRECT dependents is a workspace member. `error[unsound]`,
#                   nonzero. An unsound advisory on a crate reached only through
#                   third-party edges emits no diagnostic at all and the check
#                   exits 0, so nothing below sees it and this script reports
#                   nothing about it.
#   yanked          `yanked = "warn"` in that file: `warning[yanked]`, exit 0.
#   an `ignore` id  a `note`, which the default `--log-level warn` keeps out of
#                   the log altogether.
#
# WHAT THIS SCRIPT ACTS ON IS A DIAGNOSTIC AT ERROR OR WARNING SEVERITY whose
# class is one of the five in CLASSES below. Reading the log rather than the exit
# status alone is what catches a yanked crate, which exits 0; the severity bound
# is what keeps an advisory accepted in the `ignore` table out.
#
# TWO SHAPES ROUTE TO EXIT 3 INSTEAD, each writing nothing, each leaving an open
# issue exactly as it stands, each naming the run URL so the run goes red:
#
#   * `warning[index-failure]` in the log, at any exit status. cargo-deny emits
#     it per crate whose registry index query failed, while `yanked` is not
#     `allow`, at WARNING severity and exit 0. The registry query failed, so the
#     yanked half of the check did not run for the crates it names, and a clean
#     verdict on that log would close the issue over an unrun check.
#   * a nonzero exit carrying no advisory diagnostic: `cargo metadata` failed,
#     the advisory database would not fetch, the index cache would not load
#     (`error[index-cache-load-failure]`).
#
# WHAT IT WRITES. One issue carries the result. It is found among the open
# issues labelled `security` by the exact title in TITLE below.
#
#   finding, no such issue    create it, labelled `security`
#   finding, one such issue   rewrite its body
#   clean, one such issue     comment the commit the check passed on, then close it
#   clean, no such issue      no write at all
#
# The body names the crate and version behind every diagnostic it reports,
# beside any `RUSTSEC-` id on that diagnostic: a yanked crate carries no
# advisory id, so its name is the only handle on it.
#
# REFUSALS, exit 2 with one message each: `jq` is absent; `jq` could not read
# the log; `gh` is unauthenticated; the repository carries no `security` label;
# `gh` refused a listing; two open issues carry the title. The unauthenticated
# check runs FIRST of the three `gh` ones, so the label-missing message never
# stands in for a token that cannot read.
#
# ENVIRONMENT. `GH_TOKEN` is read by `gh` itself. `GITHUB_SERVER_URL`,
# `GITHUB_REPOSITORY`, `GITHUB_RUN_ID` and `GITHUB_SHA` build the run URL and
# name the lockfile commit; `GH_REPO` stands in for `GITHUB_REPOSITORY`. The
# scheduled run that calls this runs on the public default branch, so the
# `GITHUB_SHA` the body quotes is a public commit.
#
# Oracle: tools/scripts/test_advisory_issue.sh.

set -euo pipefail

TITLE='Dependency advisory audit reports a finding on the committed lockfile'
LABEL=security
POLICY='tools/release/deny.toml'
CHECK="cargo deny --format json --config ${POLICY} check advisories"
# The five advisory classes this check reports. `yanked` is the one deny.toml
# holds at warn level; the other four render at error level.
CLASSES='["vulnerability","unmaintained","unsound","notice","yanked"]'
# cargo-deny's own class for a registry index query that failed. The same scan
# picks it up, and it routes to exit 3 rather than into an issue body.
TOOL_CLASS=index-failure
# One body row per reported diagnostic, bounded: a pathological log must not
# push the body past what `gh issue create` accepts.
MAX_ROWS=50

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

command -v jq >/dev/null 2>&1 ||
    die 'jq is not on PATH: the cargo-deny JSON log cannot be read without it' 2

rows_file=$(mktemp "${TMPDIR:-/tmp}/advisory-rows.XXXXXX")
jq_err_file=$(mktemp "${TMPDIR:-/tmp}/advisory-jq-err.XXXXXX")
body_file=$(mktemp "${TMPDIR:-/tmp}/advisory-issue.XXXXXX")
trap 'rm -f "$rows_file" "$jq_err_file" "$body_file"' EXIT

# One tab-separated row per reported diagnostic: severity, class, crates, ids,
# message. `fromjson?` drops a line that is not JSON instead of ending the scan.
# The id scan reads the whole diagnostic, which is where cargo-deny puts the id:
# `notes[0]` is `ID: RUSTSEC-...` on every advisory diagnostic, and the
# `advisory` object `--format json` attaches repeats it. TOOL_CLASS rows are
# admitted here and partitioned out below, before any row reaches a body.
# shellcheck disable=SC2016  # a jq program, not a shell expansion
jq_rows='
fromjson?
| select(.type == "diagnostic")
| .fields as $f
| select($f.severity == "error" or $f.severity == "warning")
| select($f.code != null and (($classes | index($f.code)) != null or $f.code == $toolclass))
| [ $f.severity,
    $f.code,
    ([$f.graphs[]? | .Krate | "\(.name) \(.version)"] | unique | join(", ")),
    ([tojson | scan("RUSTSEC-[0-9]{4}-[0-9]{4}")] | unique | join(", ")),
    (($f.message // "") | gsub("[[:space:]]+"; " "))
  ]
| @tsv
'
if ! jq -R -r --argjson classes "$CLASSES" --arg toolclass "$TOOL_CLASS" \
    "$jq_rows" "$log" > "$rows_file" 2> "$jq_err_file"; then
    die "jq could not read ${log}: $(tr '\n' ' ' < "$jq_err_file")" 2
fi

# A failed registry index query comes at WARNING severity and exit 0, so neither
# the exit status nor the severity bound tells it apart from a clean run: without
# this the clean path would comment and close the issue over a yanked check that
# did not run for the crates named here. Checked FIRST of the two exit-3 shapes,
# because it says which half of the check went unrun, and before any `gh` call.
index_count=$(awk -F'\t' -v c="$TOOL_CLASS" '$2 == c { n++ } END { print n + 0 }' "$rows_file")
if [ "$index_count" -gt 0 ]; then
    # Named under the same bound the body rows use: a registry outage reports
    # every crate in the lockfile, and the message has to stay readable.
    index_crates=$(awk -F'\t' -v c="$TOOL_CLASS" -v max="$MAX_ROWS" '
        $2 == c { n++; if (n <= max) { s = s (s == "" ? "" : "; ") $3 } }
        END { if (n > max) { s = s "; and " (n - max) " more" } print s }
    ' "$rows_file")
    die "cargo deny exited ${status} carrying ${index_count} \`warning[${TOOL_CLASS}]\` diagnostic(s), on ${index_crates}: the registry query failed, so the yanked half of the check did not run for those crates and no issue was written. Run: ${run_url}" 3
fi

row_count=$(awk 'END { print NR + 0 }' "$rows_file")

# A verdict needs at least one advisory diagnostic to rest on. Checked before
# any `gh` call, so this path writes nothing at all.
if [ "$row_count" -eq 0 ] && [ "$status" -ne 0 ]; then
    die "cargo deny exited ${status} and its log carries no advisory diagnostic: the check did not complete, so no issue was written. Run: ${run_url}" 3
fi

# The reported diagnostics, printed where the step log shows them beside the
# cargo-deny JSON they came from.
if [ "$row_count" -gt 0 ]; then
    printf 'advisory_issue: %s reported diagnostic(s)\n' "$row_count"
    awk -F'\t' '{ printf "advisory_issue:   %s[%s] %s\n", $1, $2, $3 }' "$rows_file"
fi

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

if [ "$row_count" -gt 0 ]; then
    error_rows=$(awk -F'\t' '$1 == "error" { n++ } END { print n + 0 }' "$rows_file")
    warn_rows=$(awk -F'\t' '$1 == "warning" { n++ } END { print n + 0 }' "$rows_file")
    classes_fired=
    if [ "$error_rows" -gt 0 ]; then
        classes_fired="${error_rows} advisory diagnostic(s) failed the check (\`error[...]\`)"
    fi
    if [ "$warn_rows" -gt 0 ]; then
        classes_fired="${classes_fired}${classes_fired:+; }${warn_rows} yanked crate(s) warned (\`warning[yanked]\`, which does not fail the check)"
    fi
    {
        printf '%s\n\n' "\`${CHECK}\` reports a finding against the lockfile committed at \`${commit}\`, and exited ${status}."
        printf '%s.\n\n' "$classes_fired"
        awk -F'\t' -v max="$MAX_ROWS" '
            NR <= max {
                crates = ($3 == "" ? "(the log names no crate)" : $3)
                row = "* `" $1 "[" $2 "]` " crates
                if ($4 != "") { row = row ", " $4 }
                print row ": " $5
            }
            END { if (NR > max) { printf "* and %d more, in the run log.\n", NR - max } }
        ' "$rows_file"
        printf '\n'
        printf 'Run: %s\n\n' "$run_url"
        printf '%s\n\n' "\`${POLICY}\` is the policy this check and the per-change dependency audit both read. A vulnerability or notice advisory on any crate in the resolved graph fails the check, and so does an unmaintained advisory, which that file leaves at scope \`all\`. An unsound advisory is checked at scope \`workspace\`: it is reported only when at least one direct dependent of the crate is a workspace member, so an unsound advisory on a crate reached only through third-party edges is neither reported here nor failing the check. A yanked crate warns and is reported here as well. An advisory accepted in the \`ignore\` table of that file renders below warn level and is not reported here."
        printf '%s\n' "Every crate named above is a third-party dependency of this workspace, and every \`RUSTSEC-\` id above is already public in the RustSec database. A vulnerability in Cerulion itself goes to the address in \`.github/SECURITY.md\`, not here."
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
    printf '%s\n\n' "\`${CHECK}\` exited 0 on the lockfile committed at \`${commit}\`, and its log carries no \`error[...]\` advisory diagnostic, no \`warning[yanked]\`, and no \`warning[index-failure]\`: every crate's registry index query answered."
    printf 'Run: %s\n' "$run_url"
} > "$body_file"
gh issue comment "$matches" --body-file "$body_file"
gh issue close "$matches"
printf 'advisory_issue: the check is clean; closed issue %s\n' "$matches"
