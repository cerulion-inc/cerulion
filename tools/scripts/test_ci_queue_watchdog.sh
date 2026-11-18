#!/usr/bin/env bash
# test_ci_queue_watchdog.sh: the oracle table for tools/scripts/ci_queue_watchdog.sh.
#
# `gh` and `sleep` are PATH shims. `gh` appends every invocation to one file
# and answers `gh api <path>` from a per-case answer directory: the answer to
# the Nth call of a method and path is the file `<key>.<N>` when it exists, else
# `<key>`, and an answer starting `FAIL:` is an error on stderr with exit 1.
# `sleep` returns at once, records its argument, and ends the script past 500
# waits. One case per behaviour; the first failing assertion prints one `FAIL:`
# line and exits 1. The case count is asserted against CASES_EXPECTED at the
# end. Needs `jq`. No network.

set -euo pipefail

CASES_EXPECTED=22

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-watchdog-test.XXXXXX")
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

command -v jq >/dev/null 2>&1 || fail 'jq is not on PATH: ci_queue_watchdog.sh reads the job list with it'

shim_bin="$workdir/bin"
mkdir -p "$shim_bin"

cat > "$shim_bin/gh" <<'SHIM'
#!/bin/sh
printf '%s\n' "$*" >> "$FAKE_GH_CALLS"
[ "$1" = api ] || { printf 'fake gh: unexpected invocation: %s\n' "$*" >&2; exit 99; }
shift
method=GET
path=
while [ "$#" -gt 0 ]; do
    case "$1" in
        -X) method=$2; shift 2 ;;
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
chmod 0755 "$shim_bin/gh" "$shim_bin/sleep"

R=repos/owner/repo
SHA=0123456789abcdef0123456789abcdef01234567
LIST="$R/actions/workflows/ci.yml/runs?event=merge_group&head_sha=$SHA&per_page=10"
RUN="$R/actions/runs/555"
JOBS="$R/actions/runs/555/jobs?filter=latest&per_page=100"
CANCEL="$R/actions/runs/555/cancel"

# The required list: a comment, a blank line and three names, two of them
# matrix legs as the run renders them.
{
    printf '%s\n' '# The status contexts main requires.' ''
    printf '%s\n' 'Lint' 'Test (macOS) shard 1' 'Viz tests (viz, ubuntu-latest)'
} > "$workdir/required"
printf '%s\n' '# nothing but a comment' '' > "$workdir/required_empty"

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

# jobs <name>|<status>|<conclusion> ...: a jobs page; conclusion `null` is JSON null.
jobs() {
    local sep='' e name rest st co
    printf '{"total_count":%s,"jobs":[' "$#"
    for e in "$@"; do
        name=${e%%|*}
        rest=${e#*|}
        st=${rest%%|*}
        co=${rest#*|}
        [ "$co" = null ] || co="\"$co\""
        printf '%s{"name":"%s","status":"%s","conclusion":%s}' "$sep" "$name" "$st" "$co"
        sep=,
    done
    printf ']}\n'
}

# runs <id> ...: a run listing whose runs all carry SHA.
runs() {
    local sep='' id
    printf '{"workflow_runs":['
    for id in "$@"; do
        printf '%s{"id":%s,"head_sha":"%s"}' "$sep" "$id" "$SHA"
        sep=,
    done
    printf ']}\n'
}

# run_json <status> <conclusion JSON>
run_json() {
    printf '{"id":555,"status":"%s","conclusion":%s}\n' "$1" "$2"
}

# new_case <name>: a fresh answer directory in which run 555 is listed at once
# and reads as in progress until a case says otherwise.
new_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    answers="$case_dir/answers"
    mkdir -p "$answers"
    : > "$case_dir/calls"
    : > "$case_dir/sleeps"
    runs 555 | answer GET "$LIST"
    run_json in_progress null | answer GET "$RUN"
}

# run_case [<args>...]; the default argument is the three-name required list.
run_case() {
    [ "$#" -gt 0 ] || set -- "$workdir/required"
    set +e
    output=$(
        PATH="$shim_bin:${FAKE_PATH-$PATH}" \
            GH_TOKEN=fake-token \
            GITHUB_REPOSITORY="${FAKE_REPOSITORY-owner/repo}" \
            GH_REPO='' \
            WATCH_SHA="${FAKE_SHA-$SHA}" \
            FAKE_GH_CALLS="$case_dir/calls" \
            FAKE_GH_DIR="$answers" \
            FAKE_SLEEPS="$case_dir/sleeps" \
            "$script_dir/ci_queue_watchdog.sh" "$@" 2>&1
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

expect_no_call() {
    if grep -Fq -- "$1" "$case_dir/calls"; then
        fail "$case_name: a gh call carries '$1' and must not"
    fi
}

expect_cancels() {
    local got
    got=$(grep -Fxc -- "api -X POST $CANCEL" "$case_dir/calls" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 cancel request(s), got $got"
}

expect_reads() {
    local got
    got=$(grep -Fxc -- "api $JOBS" "$case_dir/calls" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 job list read(s), got $got"
}

# expect_sleeps <seconds> ...: the exact waits, in order.
expect_sleeps() {
    local want got
    want=$(printf '%s\n' "$@")
    got=$(cat "$case_dir/sleeps")
    [ "$want" = "$got" ] || fail "$case_name: expected waits [$(printf '%s' "$want" | tr '\n' ' ')], got [$(printf '%s' "$got" | tr '\n' ' ')]"
}

expect_no_gh_at_all() {
    [ ! -s "$case_dir/calls" ] || fail "$case_name: gh was called and must not be"
}

expect_run_reads() {
    local got
    got=$(grep -Fxc -- "api $RUN" "$case_dir/calls" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 run read(s), got $got"
}

ok() {
    printf '%s: passed\n' "$1"
}

# A required job fails on the second read: cancel once, naming it. A
# non-required job that failed on the first read changed nothing, and while
# jobs run the run itself is not read.
new_case required_failure
jobs 'Lint|in_progress|null' 'Python client (macos-latest)|completed|failure' | answer GET "$JOBS" 1
jobs 'Lint|completed|failure' 'Test (macOS) shard 1|in_progress|null' 'Python client (macos-latest)|completed|failure' |
    answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output "watching run 555 of ci.yml for commit $SHA"
expect_output 'required job "Lint" ended failure; cancelling run 555'
expect_reads 2
expect_run_reads 0
expect_cancels 1
expect_sleeps 60
ok 'a failed required job cancels the run once'

# Two failed required jobs: the first one listed is named.
new_case two_required_failures
jobs 'Lint|completed|failure' 'Test (macOS) shard 1|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'required job "Lint" ended failure'
expect_cancels 1
ok 'of two failed required jobs the first listed is named'

# A required matrix leg that timed out cancels the run under its rendered name.
new_case required_leg_timed_out
jobs 'Lint|completed|success' 'Test (macOS) shard 1|completed|timed_out' 'Test (macOS) shard 2|in_progress|null' |
    answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'required job "Test (macOS) shard 1" ended timed_out'
expect_cancels 1
ok 'a timed-out required matrix leg cancels the run'

# A required job cancelled by hand is a failed context too.
new_case required_cancelled
jobs 'Viz tests (viz, ubuntu-latest)|completed|cancelled' 'Lint|in_progress|null' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'required job "Viz tests (viz, ubuntu-latest)" ended cancelled'
expect_cancels 1
ok 'a cancelled required job cancels the run'

# The run completes with no required job failed: nothing is cancelled. The
# run is read only once no job is left running, and read again until it says
# completed. A failed job that is not required is not a reason.
new_case run_completes_green
jobs 'Lint|completed|success' 'Test (macOS) shard 1|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|completed|success' 'Test (macOS) shard 1|completed|success' 'Fuzz (Linux)|completed|failure' |
    answer GET "$JOBS"
run_json in_progress null | answer GET "$RUN" 1
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'run 555 completed (success); nothing to cancel'
expect_reads 3
expect_run_reads 2
expect_cancels 0
expect_sleeps 60 60
ok 'a run with no required failure is left alone'

# No job is listed yet, then a required job is queued, then it fails: the empty
# list sends the watch to the run, which is not completed, so it reads again.
new_case jobs_listed_late
printf '{"total_count":0,"jobs":[]}\n' | answer GET "$JOBS" 1
jobs 'Viz tests (viz, ubuntu-latest)|queued|null' | answer GET "$JOBS" 2
jobs 'Viz tests (viz, ubuntu-latest)|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'required job "Viz tests (viz, ubuntu-latest)" ended failure'
expect_reads 3
expect_run_reads 1
expect_cancels 1
ok 'jobs listed late are still watched'

# A run that completed with no job at all (cancelled while it waited in its
# group) ends the watch on the first read.
new_case run_completed_without_jobs
printf '{"total_count":0,"jobs":[]}\n' | answer GET "$JOBS"
run_json completed '"cancelled"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'run 555 completed (cancelled); nothing to cancel'
expect_reads 1
expect_cancels 0
ok 'a run that completed with no jobs ends the watch'

# A skipped required job is not a failure: GitHub reads it as satisfied.
new_case required_skipped
jobs 'Lint|completed|skipped' 'Test (macOS) shard 1|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_cancels 0
ok 'a skipped required job does not cancel the run'

# One failed read is retried; three refuse with the error and cancel nothing.
new_case jobs_read_fails_three_times
printf 'FAIL: HTTP 502\n' | answer GET "$JOBS"
run_case
expect_status 2
expect_output "gh api $JOBS failed three times: FAIL: HTTP 502"
expect_reads 3
expect_cancels 0
expect_sleeps 10 10
ok 'a jobs read that fails three times refuses and cancels nothing'

# The run read refuses the same way.
new_case run_read_fails_three_times
jobs 'Lint|completed|success' | answer GET "$JOBS"
printf 'FAIL: HTTP 502\n' | answer GET "$RUN"
run_case
expect_status 2
expect_output "gh api $RUN failed three times: FAIL: HTTP 502"
expect_run_reads 3
expect_cancels 0
ok 'a run read that fails three times refuses and cancels nothing'

# A failed listing of the runs refuses before anything is watched.
new_case list_read_fails_three_times
printf 'FAIL: HTTP 502\n' | answer GET "$LIST"
run_case
expect_status 2
expect_output "failed three times: FAIL: HTTP 502"
expect_reads 0
ok 'a run listing that fails three times refuses'

# A cancel refused once is retried.
new_case cancel_retried
jobs 'Lint|completed|failure' | answer GET "$JOBS"
printf 'FAIL: HTTP 502\n' | answer POST "$CANCEL" 1
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_cancels 2
expect_sleeps 10
ok 'a cancel refused once is retried'

# A cancel refused three times while the run still runs exits 1 with the error.
new_case cancel_refused
jobs 'Lint|completed|failure' | answer GET "$JOBS"
printf 'FAIL: HTTP 403: Resource not accessible by integration\n' | answer POST "$CANCEL"
run_case
expect_status 1
expect_output 'the cancel request was refused: FAIL: HTTP 403'
expect_cancels 3
ok 'a cancel refused while the run runs exits 1'

# The failed required job was the last to finish, so the run completed before
# the cancel reached it: that refusal is no error.
new_case cancel_after_the_run_ended
jobs 'Lint|completed|failure' | answer GET "$JOBS"
run_json completed '"failure"' | answer GET "$RUN"
printf 'FAIL: HTTP 409: Cannot cancel a workflow run that is completed.\n' | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'the run completed (failure) before the cancel reached it; nothing to cancel'
ok 'a cancel refused because the run had completed exits 0'

# A file naming no context refuses before any call: the watch would be blind.
new_case empty_required
run_case "$workdir/required_empty"
expect_status 2
expect_output 'the required contexts file names no context'
expect_no_gh_at_all
ok 'a required list naming no context is refused'

# The committed list is read as the watch reads it: a failed `Lint` cancels.
new_case committed_required_list
jobs 'Lint|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case "$script_dir/../ci/required_contexts.txt"
expect_status 0
expect_output 'required job "Lint" ended failure'
expect_cancels 1
ok 'the committed required list names the jobs the watch reads'

# More jobs than one page: said, and the first page still decides.
new_case more_jobs_than_a_page
printf '{"total_count":150,"jobs":[{"name":"Lint","status":"completed","conclusion":"failure"}]}\n' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'the run has 150 jobs; only the first 100 are read'
expect_cancels 1
ok 'a run with more jobs than one page says so'

# The argument and environment refusals, each before any call.
new_case usage
run_case "$workdir/required" extra
expect_status 2
expect_output 'usage: ci_queue_watchdog.sh <required-contexts-file>'
run_case "$workdir/no_such_file"
expect_status 2
expect_output 'the required contexts file cannot be read'
FAKE_REPOSITORY='' run_case
expect_status 2
expect_output 'GITHUB_REPOSITORY and GH_REPO are both empty'
FAKE_SHA='' run_case
expect_status 2
expect_output 'WATCH_SHA is empty'
# A PATH holding the shims, bash and grep alone has no jq.
mkdir -p "$workdir/nojq"
ln -sf "$(command -v bash)" "$workdir/nojq/bash"
ln -sf "$(command -v grep)" "$workdir/nojq/grep"
FAKE_PATH="$workdir/nojq" run_case
expect_status 2
expect_output 'jq is not on PATH'
expect_no_gh_at_all
ok 'a wrong argument, an unreadable file, a missing commit and a missing jq are refused'

# The run is listed only on the third read; the first listing holds only a
# run of another commit, which is not it.
new_case run_listed_late
printf '{"workflow_runs":[{"id":444,"head_sha":"ffff"}]}\n' | answer GET "$LIST" 1
runs | answer GET "$LIST" 2
jobs 'Lint|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'watching run 555 of ci.yml'
expect_cancels 1
expect_no_call 'runs/444'
expect_sleeps 15 15
ok 'a run listed late is waited for and a run of another commit is not it'

# Two runs carry the commit: refuse rather than pick one.
new_case two_runs
runs 556 555 | answer GET "$LIST"
run_case
expect_status 2
expect_output '2 runs of ci.yml carry commit'
expect_reads 0
expect_cancels 0
ok 'two runs for the commit are refused'

# No run ever appears: refuse after FIND_TRIES reads, 15 s apart.
new_case no_run
runs | answer GET "$LIST"
run_case
expect_status 2
expect_output "no merge-queue run of ci.yml for commit $SHA appeared within 300 s"
expect_reads 0
[ "$(grep -Fxc -- "api $LIST" "$case_dir/calls")" -eq 20 ] || fail "$case_name: expected 20 run listings"
expect_sleeps 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15
ok 'a run that never appears is refused'

# The same answers twice give the same calls, waits and exit.
same_answers() {
    jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
    jobs 'Lint|completed|failure' | answer GET "$JOBS"
    : | answer POST "$CANCEL"
}
new_case same_twice_first
same_answers
run_case
expect_status 0
expect_cancels 1
cp "$case_dir/calls" "$workdir/first_calls"
cp "$case_dir/sleeps" "$workdir/first_sleeps"
new_case same_twice_second
same_answers
run_case
expect_status 0
expect_cancels 1
cmp -s "$case_dir/calls" "$workdir/first_calls" || fail "$case_name: the calls differ from the first run"
cmp -s "$case_dir/sleeps" "$workdir/first_sleeps" || fail "$case_name: the waits differ from the first run"
ok 'the same answers give the same calls'

# `usage` ran five invocations and the last case two; the table counts behaviours.
cases=$((cases - 5))
[ "$cases" -eq "$CASES_EXPECTED" ] || fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_ci_queue_watchdog: all %s cases passed\n' "$cases"
