#!/usr/bin/env bash
# test_nightly_full_run.sh: the oracle table for tools/scripts/nightly_full_run.sh.
#
# `gh`, `date` and `sleep` are PATH shims. `gh` appends every invocation to one
# file and answers `gh api <path>` from a per-case answer directory: the answer
# to the Nth call of a method and path is the file `<key>.<N>` when it exists,
# else `<key>`, and an answer starting `FAIL:` is an error on stderr with exit
# 1. `gh issue list` answers from the case's `issues` file, one answer for every
# call, in the TSV shape the script's `--jq` asks for or starting `FAIL:` as an
# error; it keeps the rows whose title holds the quoted phrase of `--search`,
# then the first `--limit` of them (30 when not given), and the body of every
# issue write is appended to one file under a `== <verb>` line, so a case
# asserts both which calls were made and the exact text they carried. `date`
# prints FAKE_NOW plus
# FAKE_DATE_STEP seconds per earlier call; `sleep` returns at once, counts, and
# ends the script past 500 waits. One case per behaviour; the first failing
# assertion prints one `FAIL:` line and exits 1. The case count is asserted
# against CASES_EXPECTED at the end.
# Needs `jq`. No network.
#
# The answers and the bodies asserted against them are markdown and JSON that
# carry backticks and dollar signs; none of it is a shell expansion.
# shellcheck disable=SC2016

set -euo pipefail

CASES_EXPECTED=35

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-nightly-test.XXXXXX")
trap 'rm -rf "$workdir"' EXIT

cases=0
case_name=
case_dir=
answers=
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

command -v jq >/dev/null 2>&1 || fail 'jq is not on PATH: nightly_full_run.sh reads the API answers with it'

shim_bin="$workdir/bin"
mkdir -p "$shim_bin"

cat > "$shim_bin/gh" <<'SHIM'
#!/bin/sh
printf '%s\n' "$*" >> "$FAKE_GH_CALLS"
case "$1" in
    api)
        shift
        method=GET
        path=
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -X) method=$2; shift 2 ;;
                -f) shift 2 ;;
                *) path=$1; shift ;;
            esac
        done
        key=$(printf '%s %s' "$method" "$path" | tr -c 'A-Za-z0-9._-' '_')
        count_file="$FAKE_GH_DIR/count.$key"
        n=$(( $(cat "$count_file" 2>/dev/null || echo 0) + 1 ))
        echo "$n" > "$count_file"
        answer="$FAKE_GH_DIR/$key.$n"
        [ -f "$answer" ] || answer="$FAKE_GH_DIR/$key"
        if [ ! -f "$answer" ]; then
            printf 'fake gh: no answer for %s %s\n' "$method" "$path" >&2
            exit 99
        fi
        if [ "$(head -c 5 "$answer")" = FAIL: ]; then
            cat "$answer" >&2
            exit 1
        fi
        cat "$answer"
        ;;
    issue)
        case "$2" in
            list)
                if [ "$(head -c 5 "$FAKE_GH_DIR/issues")" = FAIL: ]; then
                    cat "$FAKE_GH_DIR/issues" >&2
                    exit 1
                fi
                search=
                limit=30
                prev=
                for arg in "$@"; do
                    case "$prev" in
                        --search) search=$arg ;;
                        --limit) limit=$arg ;;
                    esac
                    prev=$arg
                done
                phrase=${search#*\"}
                phrase=${phrase%\"}
                awk -F '\t' -v search="$search" -v phrase="$phrase" \
                    'search == "" || index($2, phrase) > 0' "$FAKE_GH_DIR/issues" | head -n "$limit"
                ;;
            create | comment)
                prev=
                for arg in "$@"; do
                    if [ "$prev" = --body-file ]; then
                        printf '== %s\n' "$2" >> "$FAKE_GH_BODY"
                        cat "$arg" >> "$FAKE_GH_BODY"
                    fi
                    prev=$arg
                done
                ;;
            close) ;;
            *) printf 'fake gh: unexpected invocation: %s\n' "$*" >&2; exit 99 ;;
        esac
        ;;
    *)
        printf 'fake gh: unexpected invocation: %s\n' "$*" >&2
        exit 99
        ;;
esac
exit 0
SHIM

cat > "$shim_bin/date" <<'SHIM'
#!/bin/sh
n=$(( $(cat "$FAKE_DATE_COUNT" 2>/dev/null || echo 0) + 1 ))
echo "$n" > "$FAKE_DATE_COUNT"
echo $(( FAKE_NOW + (n - 1) * FAKE_DATE_STEP ))
SHIM

# A script that never stops waiting is a failure, not a hang: past 500 waits
# the shim ends the script that called it.
cat > "$shim_bin/sleep" <<'SHIM'
#!/bin/sh
printf '%s\n' "$*" >> "$FAKE_SLEEPS"
if [ "$(grep -c . "$FAKE_SLEEPS")" -gt 500 ]; then
    printf 'fake sleep: more than 500 waits\n' >&2
    kill -TERM "$PPID"
fi
SHIM
chmod 0755 "$shim_bin/gh" "$shim_bin/date" "$shim_bin/sleep"

TITLE='Nightly full run of main failed or did not run'
R=repos/owner/repo
U=https://github.invalid/owner/repo/actions/runs
BOT='github-actions[bot]'
# 2026-11-17T12:00:00Z
NOW=1794916800

key_of() {
    printf '%s %s' "$1" "$2" | tr -c 'A-Za-z0-9._-' '_'
}

# answer <METHOD> <path> [<call number>], the answer text on stdin.
answer() {
    local f
    f="$answers/$(key_of "$1" "$2")"
    [ -z "${3:-}" ] || f="$f.$3"
    cat > "$f"
}

list_path() {
    printf '%s/actions/workflows/%s/runs?event=workflow_dispatch&branch=main&per_page=20' "$R" "$1"
}

jobs_path() {
    printf '%s/actions/runs/%s/attempts/%s/jobs?per_page=100' "$R" "$1" "$2"
}

# run_json <id> <status> <conclusion JSON> <attempt>
run_json() {
    printf '{"id":%s,"status":"%s","conclusion":%s,"run_attempt":%s,"html_url":"%s/%s","head_sha":"0123abc"}\n' \
        "$1" "$2" "$3" "$4" "$U" "$1"
}

# runs_json <id>:<actor>[:<triggering actor>] ...; the triggering actor is the
# actor unless named.
runs_json() {
    local sep='' e id actor trigger
    printf '{"workflow_runs":['
    for e in "$@"; do
        id=${e%%:*}
        actor=${e#*:}
        trigger=$actor
        case $actor in
            *:*)
                trigger=${actor#*:}
                actor=${actor%%:*}
                ;;
        esac
        printf '%s{"id":%s,"actor":{"login":"%s"},"triggering_actor":{"login":"%s"}}' "$sep" "$id" "$actor" "$trigger"
        sep=,
    done
    printf ']}\n'
}

# new_case <name>: a fresh answer directory with no issue open.
new_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    answers="$case_dir/answers"
    mkdir -p "$answers"
    : > "$case_dir/calls"
    : > "$case_dir/body"
    : > "$case_dir/sleeps"
    : > "$answers/issues"
}

# The dispatch both workflows answer by default: ci.yml's new bot run is 200,
# rmw-distros.yml's is 400. A person's earlier dispatch of ci.yml (90) is listed
# throughout and is never the nightly's.
standard_dispatch() {
    runs_json "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)" 1
    runs_json "200:$BOT" "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
    runs_json "300:$BOT" | answer GET "$(list_path rmw-distros.yml)" 1
    runs_json "400:$BOT" "300:$BOT" | answer GET "$(list_path rmw-distros.yml)"
    : | answer POST "$R/actions/workflows/ci.yml/dispatches"
    : | answer POST "$R/actions/workflows/rmw-distros.yml/dispatches"
}

issue_open() {
    printf '%s\t%s\t%s\n' 7 'Pairing fails on a cold start' '2026-11-10T00:00:00Z' > "$answers/issues"
    printf '%s\t%s\t%s\n' 42 "$TITLE" "$1" >> "$answers/issues"
}

# run_case <mode>; FAKE_DATE_STEP may be set on the call. Leaves $output and
# $status, and the calls, bodies and sleeps in $case_dir.
run_case() {
    set +e
    output=$(
        PATH="$shim_bin:${FAKE_PATH-$PATH}" \
            GH_TOKEN=fake-token \
            GITHUB_SERVER_URL=https://github.invalid \
            GITHUB_REPOSITORY="${FAKE_REPOSITORY-owner/repo}" \
            GH_REPO='' \
            GITHUB_RUN_ID=777 \
            FAKE_GH_CALLS="$case_dir/calls" \
            FAKE_GH_BODY="$case_dir/body" \
            FAKE_GH_DIR="$answers" \
            FAKE_SLEEPS="$case_dir/sleeps" \
            FAKE_DATE_COUNT="$case_dir/date_count" \
            FAKE_NOW="$NOW" \
            FAKE_DATE_STEP="${FAKE_DATE_STEP:-0}" \
            "$script_dir/nightly_full_run.sh" "$@" 2>&1
    )
    status=$?
    set -e
    cases=$((cases + 1))
}

expect_status() {
    [ "$status" -eq "$1" ] || fail "$case_name: expected exit $1, got $status"
}

expect_output() {
    grep -Fq -- "$1" <<< "$output" || fail "$case_name: the output does not carry '$1'"
}

expect_call() {
    grep -Fq -- "$1" "$case_dir/calls" || fail "$case_name: no gh call carries '$1'"
}

expect_no_call() {
    if grep -Fq -- "$1" "$case_dir/calls"; then
        fail "$case_name: a gh call carries '$1' and must not"
    fi
}

expect_no_writes() {
    if grep -Eq '^issue (create|comment|close)' "$case_dir/calls"; then
        fail "$case_name: an issue write was made and must not be"
    fi
}

# expect_body: the exact text of every issue write, from stdin.
expect_body() {
    local want
    want=$(cat)
    [ "$want" = "$(cat "$case_dir/body")" ] ||
        fail "$case_name: the issue text differs:
--- want ---
$want
--- got ---
$(cat "$case_dir/body")"
}

expect_sleeps() {
    local got
    got=$(grep -c . "$case_dir/sleeps" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 sleep(s), got $got"
}

ok() {
    printf '%s: passed\n' "$1"
}

# --- RUN --------------------------------------------------------------------

# Both runs pass on the first attempt and no issue is open: nothing is written.
# The first read of run 200 fails once and is retried, and 200 is in progress
# on the read after that, so the wait polls before it ends.
new_case green_no_issue
standard_dispatch
printf 'FAIL: HTTP 502\n' | answer GET "$R/actions/runs/200/attempts/1" 1
run_json 200 in_progress null 1 | answer GET "$R/actions/runs/200/attempts/1" 2
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
expect_output 'dispatched rmw-distros.yml on main: run 400'
expect_output 'every run passed on its first attempt and no issue is open; nothing to write'
expect_call "api -X POST $R/actions/workflows/ci.yml/dispatches -f ref=main"
expect_call "api -X POST $R/actions/workflows/rmw-distros.yml/dispatches -f ref=main"
expect_no_call 'rerun-failed-jobs'
expect_no_call '/jobs'
expect_no_writes
# One retry of the failed read, one poll while 200 was in progress.
expect_sleeps 2
ok 'two green runs and no open issue write nothing'

# Both green with the issue open: comment the runs, then close it.
new_case green_closes_issue
standard_dispatch
issue_open '2026-11-16T05:00:00Z'
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'every run passed; closed issue 42'
expect_call 'issue close 42'
expect_body <<EOF
== comment
The nightly full run of \`main\` passed on its first attempt.

- \`ci.yml\` at \`0123abc\`: $U/200
- \`rmw-distros.yml\` at \`0123abc\`: $U/400

Nightly: $U/777
EOF
ok 'two green runs close the open issue'

# 200 newer open `bug` issues come before the nightly's in a plain listing, and
# one of them holds the title inside a longer one: the listing asks for the
# title, and the issue past the first 200 is still the one found and closed.
new_case issue_past_the_listing_limit
standard_dispatch
for ((n = 500; n > 301; n--)); do
    printf '%s\t%s\t%s\n' "$n" 'Pairing fails on a cold start' '2026-11-16T06:00:00Z'
done > "$answers/issues"
printf '%s\t%s\t%s\n' 301 "$TITLE on a release branch" '2026-11-16T06:00:00Z' >> "$answers/issues"
printf '%s\t%s\t%s\n' 42 "$TITLE" '2026-11-16T05:00:00Z' >> "$answers/issues"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'every run passed; closed issue 42'
expect_call 'issue close 42'
expect_no_call 'issue close 301'
expect_call "--search in:title \"$TITLE\""
ok 'the issue past the first 200 open bug issues is still found by its title'

# ci.yml fails, its failed jobs are rerun once and pass: the issue opens and
# names the job that failed. Right after the rerun request attempt 2 does not
# exist yet and answers 404 (three tries), which reads as not finished rather
# than as a refusal; then it is in progress, then it passes.
make_red_then_green() {
    standard_dispatch
    run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
    # The newest attempt, read before the rerun request: 1.
    run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200"
    for n in 1 2 3; do
        printf 'FAIL: HTTP 404: Not Found\n' | answer GET "$R/actions/runs/200/attempts/2" "$n"
    done
    run_json 200 in_progress null 2 | answer GET "$R/actions/runs/200/attempts/2" 4
    run_json 200 completed '"success"' 2 | answer GET "$R/actions/runs/200/attempts/2"
    run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
    printf '%s\n' '{"total_count":3,"jobs":[{"name":"Lint","conclusion":"success"},{"name":"Test (macOS) shard 1","conclusion":"failure"},{"name":"Fuzz (Linux)","conclusion":"skipped"}]}' |
        answer GET "$(jobs_path 200 1)"
    : | answer POST "$R/actions/runs/200/rerun-failed-jobs"
}
red_then_green_body() {
    cat <<EOF
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Test (macOS) shard 1\`: failure
  - rerun of the failed jobs: success
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: success

A failed step skips every later step of its job, so the failed jobs ran once more. A job that passed on that rerun failed on the first attempt only.

Nightly: $U/777
EOF
}
new_case red_then_green
make_red_then_green
run_case run
expect_status 0
expect_output 'opened the issue'
expect_call "api -X POST $R/actions/runs/200/rerun-failed-jobs"
expect_no_call "$R/actions/runs/400/rerun-failed-jobs"
expect_no_call 'attempts/2/jobs'
expect_call "issue create --title $TITLE --label bug --body-file "
red_then_green_body | expect_body
expect_sleeps 4
ok 'a red run rerun to green opens the issue and names the failed job'

# The same fixtures twice give the same calls and the same text.
# The body file is a fresh temporary name on every run, so it is left out.
sed 's/--body-file .*/--body-file/' "$case_dir/calls" > "$workdir/first_calls"
cp "$case_dir/body" "$workdir/first_body"
new_case red_then_green_again
make_red_then_green
run_case run
expect_status 0
sed 's/--body-file .*/--body-file/' "$case_dir/calls" | cmp -s - "$workdir/first_calls" ||
    fail "$case_name: the calls differ from the first run of the same fixtures"
cmp -s "$case_dir/body" "$workdir/first_body" || fail "$case_name: the text differs from the first run of the same fixtures"
ok 'the same fixtures give the same calls and text'

# ci.yml fails on both attempts with the issue open: comment on it, listing the
# job that failed again. rmw-distros.yml times out on the first attempt and
# passes on the rerun.
new_case red_twice_issue_open
standard_dispatch
issue_open '2026-11-15T05:00:00Z'
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200"
run_json 200 completed '"failure"' 2 | answer GET "$R/actions/runs/200/attempts/2"
run_json 400 completed '"timed_out"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_json 400 completed '"timed_out"' 1 | answer GET "$R/actions/runs/400"
run_json 400 completed '"success"' 2 | answer GET "$R/actions/runs/400/attempts/2"
printf '%s\n' '{"total_count":2,"jobs":[{"name":"Lint","conclusion":"failure"},{"name":"Documentation","conclusion":"cancelled"}]}' |
    answer GET "$(jobs_path 200 1)"
printf '%s\n' '{"total_count":2,"jobs":[{"name":"Lint","conclusion":"failure"},{"name":"Documentation","conclusion":"success"}]}' |
    answer GET "$(jobs_path 200 2)"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"rmw distro lane (jazzy)","conclusion":"timed_out"}]}' |
    answer GET "$(jobs_path 400 1)"
: | answer POST "$R/actions/runs/200/rerun-failed-jobs"
: | answer POST "$R/actions/runs/400/rerun-failed-jobs"
run_case run
expect_status 0
expect_output 'commented on issue 42'
expect_no_call 'issue create'
expect_call "$(jobs_path 200 2)"
expect_no_call "$(jobs_path 400 2)"
expect_body <<EOF
== comment
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Lint\`: failure
    - \`Documentation\`: cancelled
  - rerun of the failed jobs: failure
    - \`Lint\`: failure
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: timed_out
    - \`rmw distro lane (jazzy)\`: timed_out
  - rerun of the failed jobs: success

A failed step skips every later step of its job, so the failed jobs ran once more. A job that passed on that rerun failed on the first attempt only.

Nightly: $U/777
EOF
ok 'a run red on both attempts comments on the open issue'

# The rerun request is refused: the issue says so, and no attempt 2 is read.
new_case rerun_refused
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
# The run as a whole, read after the refusal: still on attempt 1.
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
printf 'FAIL: HTTP 403: This workflow run cannot be retried\n' | answer POST "$R/actions/runs/200/rerun-failed-jobs"
run_case run
expect_status 0
expect_output 'opened the issue'
expect_no_call 'attempts/2'
expect_body <<EOF2
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Lint\`: failure
  - rerun of the failed jobs: refused (FAIL: HTTP 403: This workflow run cannot be retried)
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: success

Nightly: $U/777
EOF2
ok 'a refused rerun is reported and the result still recorded'

# The rerun request errors three times, yet the server took it: the run as a
# whole reads attempt 2, so the rerun is waited for and reported as it ended.
new_case rerun_taken_despite_an_error
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200" 1
run_json 200 in_progress null 2 | answer GET "$R/actions/runs/200"
run_json 200 completed '"success"' 2 | answer GET "$R/actions/runs/200/attempts/2"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
printf 'FAIL: HTTP 502\n' | answer POST "$R/actions/runs/200/rerun-failed-jobs"
run_case run
expect_status 0
expect_body <<EOF2
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Lint\`: failure
  - rerun of the failed jobs: success
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: success

A failed step skips every later step of its job, so the failed jobs ran once more. A job that passed on that rerun failed on the first attempt only.

Nightly: $U/777
EOF2
ok 'a rerun the server took despite an error is waited for'

# rmw-distros.yml never finishes: every `date` read is two hours later, so the
# deadline passes on the third poll round. That run is reported unfinished and
# is not rerun; the green one is reported as it is.
new_case deadline
standard_dispatch
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 in_progress null 1 | answer GET "$R/actions/runs/400/attempts/1"
FAKE_DATE_STEP=7200 run_case run
expect_status 0
expect_output 'opened the issue'
expect_no_call 'rerun-failed-jobs'
expect_no_call '/jobs'
expect_body <<EOF2
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: success
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: not finished after 330 min

Nightly: $U/777
EOF2
ok 'a run unfinished at the deadline is reported and not rerun'

# The dispatched run is listed only on the third read after the dispatch, and a
# person's dispatch (201) that lands at the same moment is not taken for it.
new_case late_run_and_a_person
standard_dispatch
runs_json "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)" 2
runs_json "201:someone" "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)" 3
runs_json "201:someone" "200:$BOT" "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
expect_no_call "$R/actions/runs/201"
expect_sleeps 2
ok "a late run is waited for and a person's dispatch is never the nightly's"

# A person started the nightly by hand: the run the job's token dispatched has
# the bot as its actor and that person as its triggering actor, and is found.
new_case person_started_the_nightly
standard_dispatch
runs_json "200:$BOT:someone" "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
ok 'a nightly a person started finds the run its token dispatched'

# The dispatched run never appears: refuse after FIND_TRIES reads, write nothing.
new_case run_never_appears
standard_dispatch
runs_json "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
run_case run
expect_status 2
expect_output 'the dispatched run of ci.yml did not appear within 300 s'
expect_no_call 'rmw-distros.yml/dispatches'
expect_no_call 'issue'
expect_sleeps 19
ok 'a dispatched run that never appears is refused'

# Two new bot runs after one dispatch: refuse rather than pick one.
new_case two_new_runs
standard_dispatch
runs_json "201:$BOT" "200:$BOT" "100:$BOT" | answer GET "$(list_path ci.yml)"
run_case run
expect_status 2
expect_output '2 bot-dispatched runs of ci.yml appeared after one dispatch (201 200)'
expect_no_call 'issue'
ok 'two new bot runs after one dispatch are refused'

# A read that fails three times refuses, naming the call and the error.
new_case api_fails_three_times
standard_dispatch
printf 'FAIL: HTTP 502\n' | answer GET "$R/actions/runs/200/attempts/1"
run_case run
expect_status 2
expect_output "gh api $R/actions/runs/200/attempts/1 failed three times: FAIL: HTTP 502"
expect_no_call 'issue'
ok 'a read that fails three times is refused'

# The dispatch answers with the run id: no listing is read after it.
new_case dispatch_answers_the_id
runs_json "100:$BOT" | answer GET "$(list_path ci.yml)"
runs_json "300:$BOT" | answer GET "$(list_path rmw-distros.yml)"
printf '{"workflow_run_id":200,"run_url":"https://api.github.invalid/x","html_url":"%s/200"}\n' "$U" |
    answer POST "$R/actions/workflows/ci.yml/dispatches"
printf '{"workflow_run_id":400,"run_url":"https://api.github.invalid/y","html_url":"%s/400"}\n' "$U" |
    answer POST "$R/actions/workflows/rmw-distros.yml/dispatches"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
expect_output 'dispatched rmw-distros.yml on main: run 400'
[ "$(grep -c 'event=workflow_dispatch' "$case_dir/calls")" -eq 2 ] ||
    fail "$case_name: the run listing was read again after the dispatch named its run"
ok 'a dispatch that names its run is taken at its word'

# The bot is the triggering actor only. Which field the token's account lands
# in is not documented, so either one is accepted.
new_case bot_triggering_actor_only
standard_dispatch
runs_json "200:someone:$BOT" "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
ok 'a run whose triggering actor is the bot is found'

# The deadline passes while a rerun is still running: the rerun is reported
# unfinished and its attempt's jobs are not read. A neutral job is not a failure, and a
# run with more jobs than one page says so.
new_case rerun_unfinished_at_deadline
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200"
run_json 200 in_progress null 2 | answer GET "$R/actions/runs/200/attempts/2"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":150,"jobs":[{"name":"Lint","conclusion":"failure"},{"name":"Gate","conclusion":"neutral"}]}' |
    answer GET "$(jobs_path 200 1)"
: | answer POST "$R/actions/runs/200/rerun-failed-jobs"
FAKE_DATE_STEP=3600 run_case run
expect_status 0
expect_no_call "$(jobs_path 200 2)"
expect_body <<EOF2
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Lint\`: failure
    - the run has 150 jobs; this lists the first 100
  - rerun of the failed jobs: not finished after 330 min
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: success

A failed step skips every later step of its job, so the failed jobs ran once more. A job that passed on that rerun failed on the first attempt only.

Nightly: $U/777
EOF2
ok 'a rerun unfinished at the deadline is reported unfinished'

# A run that ends red after the deadline has passed is not rerun.
new_case red_past_deadline
standard_dispatch
run_json 200 in_progress null 1 | answer GET "$R/actions/runs/200/attempts/1" 1
run_json 200 in_progress null 1 | answer GET "$R/actions/runs/200/attempts/1" 2
run_json 200 in_progress null 1 | answer GET "$R/actions/runs/200/attempts/1" 3
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
FAKE_DATE_STEP=5000 run_case run
expect_status 0
expect_no_call 'rerun-failed-jobs'
expect_body <<EOF2
== create
The nightly full run of \`main\` did not pass.

- \`ci.yml\` at \`0123abc\`: $U/200
  - first attempt: failure
    - \`Lint\`: failure
  - rerun of the failed jobs: not requested: past the 330-minute deadline
- \`rmw-distros.yml\` at \`0123abc\`: $U/400
  - first attempt: success

Nightly: $U/777
EOF2
ok 'a run red after the deadline is not rerun'

# A refused issue listing is refused, not read as no issue open.
new_case issue_list_refused
standard_dispatch
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf 'FAIL: HTTP 403\n' > "$answers/issues"
run_case run
expect_status 2
expect_output 'gh refused the issue listing: FAIL: HTTP 403'
expect_no_writes
ok 'a refused issue listing is refused'

# Someone reran the failed run by hand before the first read: the latest attempt
# passed, but attempt 1 is what is read, so the failure is still reported.
new_case manual_rerun_before_the_first_read
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 200 completed '"success"' 2 | answer GET "$R/actions/runs/200/attempts/2"
run_json 200 completed '"success"' 2 | answer GET "$R/actions/runs/200"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
# Its newest attempt passed, so there is nothing left to rerun.
printf 'FAIL: HTTP 422: no failed jobs to rerun\n' | answer POST "$R/actions/runs/200/rerun-failed-jobs"
run_case run
expect_status 0
expect_output 'opened the issue'
expect_no_call 'issue close'
grep -Fq -- '  - first attempt: failure' "$case_dir/body" || fail "$case_name: attempt 1's failure is not reported"
ok "a rerun someone started by hand never stands in for attempt 1"

# Someone reran the failed jobs by hand and that attempt failed too, before the
# nightly's rerun: the nightly's rerun is attempt 3, and attempt 3 is the one
# waited on and reported, never the hand-run attempt 2.
new_case rerun_after_a_rerun_by_hand
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 200 completed '"failure"' 2 | answer GET "$R/actions/runs/200"
run_json 200 completed '"failure"' 2 | answer GET "$R/actions/runs/200/attempts/2"
run_json 200 in_progress null 3 | answer GET "$R/actions/runs/200/attempts/3" 1
run_json 200 completed '"success"' 3 | answer GET "$R/actions/runs/200/attempts/3"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
: | answer POST "$R/actions/runs/200/rerun-failed-jobs"
run_case run
expect_status 0
expect_no_call "$R/actions/runs/200/attempts/2"
grep -Fq -- '  - rerun of the failed jobs: success' "$case_dir/body" || fail "$case_name: attempt 3's result is not the one reported"
ok "the nightly's rerun is read at the attempt it started"

# The dispatch errors but the server started the run: it is found on the
# first listing after the error, and no second request is sent.
new_case dispatch_errors_but_starts_a_run
standard_dispatch
printf 'FAIL: HTTP 502\n' | answer POST "$R/actions/workflows/ci.yml/dispatches"
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
[ "$(grep -Fxc -- "api -X POST $R/actions/workflows/ci.yml/dispatches -f ref=main" "$case_dir/calls")" -eq 1 ] ||
    fail "$case_name: a second dispatch was sent for a run that had started"
ok 'a dispatch that errors after starting a run is not sent again'

# The dispatch errors and no run appears in four reads: it is sent once more,
# which starts the run.
new_case dispatch_errors_then_starts
standard_dispatch
printf 'FAIL: HTTP 502\n' | answer POST "$R/actions/workflows/ci.yml/dispatches" 1
for n in 2 3 4 5; do
    runs_json "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)" "$n"
done
run_json 200 completed '"success"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
run_case run
expect_status 0
expect_output 'dispatched ci.yml on main: run 200'
[ "$(grep -Fxc -- "api -X POST $R/actions/workflows/ci.yml/dispatches -f ref=main" "$case_dir/calls")" -eq 2 ] ||
    fail "$case_name: expected two dispatch requests"
ok 'a dispatch that errors and starts nothing is sent again'

# Three dispatches error and none starts a run: refuse, naming the error.
new_case dispatch_fails_three_times
standard_dispatch
printf 'FAIL: HTTP 502\n' | answer POST "$R/actions/workflows/ci.yml/dispatches"
runs_json "100:$BOT" "90:someone" | answer GET "$(list_path ci.yml)"
run_case run
expect_status 2
expect_output 'the dispatch of ci.yml failed three times and started no run: FAIL: HTTP 502'
expect_no_call 'issue'
ok 'three failed dispatches that start nothing are refused'

# Two open issues carrying the title: refuse rather than write to either.
new_case two_open_issues
standard_dispatch
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200/attempts/1"
run_json 400 completed '"success"' 1 | answer GET "$R/actions/runs/400/attempts/1"
printf '%s\n' '{"total_count":1,"jobs":[{"name":"Lint","conclusion":"failure"}]}' | answer GET "$(jobs_path 200 1)"
printf 'FAIL: HTTP 403\n' | answer POST "$R/actions/runs/200/rerun-failed-jobs"
run_json 200 completed '"failure"' 1 | answer GET "$R/actions/runs/200"
printf '%s\t%s\t%s\n' 42 "$TITLE" '2026-11-15T05:00:00Z' 91 "$TITLE" '2026-11-16T05:00:00Z' > "$answers/issues"
run_case run
expect_status 2
expect_output 'issues 42 91 all carry the title'
expect_no_writes
ok 'two open issues carrying the title are refused'

# --- ALARM ------------------------------------------------------------------

NEWEST="$R/actions/workflows/nightly.yml/runs?branch=main&per_page=1"
SUCCESS="$R/actions/workflows/nightly.yml/runs?branch=main&status=success&per_page=1"

workflow_json() {
    printf '{"state":"%s","created_at":"%s"}\n' "$1" "$2"
}
# newest_run <id> <status> <conclusion JSON>
newest_run() {
    printf '{"workflow_runs":[{"html_url":"%s/%s","status":"%s","conclusion":%s}]}\n' "$U" "$1" "$2" "$3" |
        answer GET "$NEWEST"
}
# newest_success <id> <updated_at>
newest_success() {
    printf '{"workflow_runs":[{"html_url":"%s/%s","updated_at":"%s"}]}\n' "$U" "$1" "$2" |
        answer GET "$SUCCESS"
}
no_runs() {
    printf '{"workflow_runs":[]}\n' | answer GET "$1"
}
active_since_november() {
    workflow_json active '2026-11-01T00:00:00Z' | answer GET "$R/actions/workflows/nightly.yml"
}

# Tonight's run recorded its result two hours ago: nothing to write.
new_case alarm_on_schedule
active_since_november
newest_run 600 completed '"success"'
newest_success 600 '2026-11-17T10:00:00Z'
issue_open '2026-11-10T00:00:00Z'
run_case alarm
expect_status 0
expect_output 'the nightly full run is on schedule; nothing to write'
expect_no_call 'issue'
ok 'a nightly that reached a result raises no alarm'

# A run is in progress and the last result is 27 hours old: the run in flight
# is the next result, so nothing is written yet.
new_case alarm_run_in_progress
active_since_november
newest_run 600 in_progress null
newest_success 500 '2026-11-16T09:00:00Z'
run_case alarm
expect_status 0
expect_output "a nightly run is in_progress: $U/600; nothing to write"
expect_no_call 'issue'
ok 'a nightly in progress raises no alarm'

# No result in 27 hours and the newest run is the result itself: no run
# started since. Open the issue.
new_case alarm_stale
active_since_november
newest_run 500 completed '"success"'
newest_success 500 '2026-11-16T09:00:00Z'
run_case alarm
expect_status 0
expect_output 'opened the issue'
expect_call "issue create --title $TITLE --label bug --body-file "
expect_body <<EOF2
== create
No nightly full run on \`main\` reached a result in the last 26 hours. The newest result: $U/500

Alarm: $U/777
EOF2
ok 'no result in 26 hours opens the issue'

# The newest run failed before it reached a result, the last result is 27
# hours old, and the open issue was last updated 30 hours ago: comment on it,
# naming the run that failed.
new_case alarm_newest_failed
active_since_november
newest_run 600 completed '"failure"'
newest_success 500 '2026-11-16T09:00:00Z'
issue_open '2026-11-16T06:00:00Z'
run_case alarm
expect_status 0
expect_output 'commented on issue 42'
expect_body <<EOF2
== comment
No nightly full run on \`main\` reached a result in the last 26 hours. The newest result: $U/500 The newest run ended \`failure\`: $U/600

Alarm: $U/777
EOF2
ok 'a nightly that ended without a result comments on the open issue'

# The same finding with the issue updated an hour ago writes nothing.
new_case alarm_repeat_suppressed
active_since_november
newest_run 600 completed '"cancelled"'
newest_success 500 '2026-11-16T09:00:00Z'
issue_open '2026-11-17T11:00:00Z'
run_case alarm
expect_status 0
expect_output 'issue 42 was updated in the last 24 hours; nothing to write'
expect_no_writes
ok 'an alarm repeats at most once a day'

# The first night: a run that failed, no result ever, and the workflow added
# two hours ago. Not yet a finding.
new_case alarm_first_night
workflow_json active '2026-11-17T10:00:00Z' | answer GET "$R/actions/workflows/nightly.yml"
newest_run 600 completed '"failure"'
no_runs "$SUCCESS"
run_case alarm
expect_status 0
expect_output 'the nightly full run is on schedule; nothing to write'
expect_no_call 'issue'
ok 'a workflow added under 26 hours ago raises no alarm'

# Never ran, and the workflow was added 30 hours ago: open the issue.
new_case alarm_never_ran
workflow_json active '2026-11-16T06:00:00Z' | answer GET "$R/actions/workflows/nightly.yml"
no_runs "$NEWEST"
no_runs "$SUCCESS"
run_case alarm
expect_status 0
expect_body <<EOF2
== create
\`nightly.yml\` was added more than 26 hours ago and no run of it on \`main\` has reached a result.

Alarm: $U/777
EOF2
ok 'a workflow with no result in 26 hours opens the issue'

# A disabled workflow is a finding at once, whatever its last run did.
new_case alarm_disabled
workflow_json disabled_manually '2026-11-01T00:00:00Z' | answer GET "$R/actions/workflows/nightly.yml"
newest_run 600 completed '"success"'
newest_success 600 '2026-11-17T10:00:00Z'
run_case alarm
expect_status 0
expect_body <<EOF2
== create
\`nightly.yml\` is \`disabled_manually\`, so no nightly full run starts.

Alarm: $U/777
EOF2
ok 'a disabled nightly workflow opens the issue'

# A timestamp that does not parse is refused, not read as zero.
new_case alarm_bad_timestamp
active_since_november
newest_run 500 completed '"success"'
newest_success 500 'yesterday'
run_case alarm
expect_status 2
expect_output "the timestamp does not parse: 'yesterday'"
expect_no_call 'issue'
ok 'a timestamp that does not parse is refused'

# --- USAGE ------------------------------------------------------------------

new_case usage
run_case
expect_status 2
expect_output 'usage: nightly_full_run.sh run|alarm'
run_case nightly
expect_status 2
expect_output 'unknown mode: nightly'
FAKE_REPOSITORY='' run_case run
expect_status 2
expect_output 'GITHUB_REPOSITORY and GH_REPO are both empty'
# A PATH holding the shims and bash alone has no jq.
mkdir -p "$workdir/nojq"
ln -sf "$(command -v bash)" "$workdir/nojq/bash"
FAKE_PATH="$workdir/nojq" run_case run
expect_status 2
expect_output 'jq is not on PATH'
expect_no_call 'api'
ok 'a missing or unknown mode, a missing repository and a missing jq are refused'

# `usage` ran four invocations; the table counts behaviours.
cases=$((cases - 3))
[ "$cases" -eq "$CASES_EXPECTED" ] || fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_nightly_full_run: all %s cases passed\n' "$cases"
