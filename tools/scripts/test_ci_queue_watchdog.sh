#!/usr/bin/env bash
# test_ci_queue_watchdog.sh: the oracle table for tools/scripts/ci_queue_watchdog.sh.
#
# `gh` and `sleep` are PATH shims. `gh` appends every invocation to one file
# and answers `gh api <path>` from a per-case answer directory: the answer to
# the Nth call of a method, path and fields is the file `<key>.<N>` when it
# exists, else `<key>`, and an answer starting `FAIL:` is an error on stderr
# with exit 1; one starting `BODYFAIL:` prints the rest on stdout, as `gh`
# prints a GraphQL error body, and exits 1. Past 2000 calls every call fails,
# so a read loop that never ends fails its case instead of hanging it. A `-f`
# or `-F` field is part of the key, except `query=`, which
# the shim records as `query=<query>` and writes to the case's `query` file; a
# call with a field and no `-X` is a POST, as `gh api` sends it.
# `sleep` returns at once, records its argument, and ends the script past 500
# waits. One case per behaviour; the first failing assertion prints one `FAIL:`
# line and exits 1. The case count is asserted against CASES_EXPECTED at the
# end. Needs `jq`. No network.

set -euo pipefail

CASES_EXPECTED=42

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
line=
for a in "$@"; do
    case $a in
        query=*)
            printf '%s\n' "${a#query=}" > "$FAKE_GH_QUERY"
            a='query=<query>'
            ;;
    esac
    line="${line:+$line }$a"
done
printf '%s\n' "$line" >> "$FAKE_GH_CALLS"
if [ "$(grep -c . "$FAKE_GH_CALLS")" -gt 2000 ]; then
    printf 'fake gh: more than 2000 calls\n' >&2
    exit 98
fi
[ "$1" = api ] || { printf 'fake gh: unexpected invocation: %s\n' "$*" >&2; exit 99; }
shift
method=
path=
fields=
while [ "$#" -gt 0 ]; do
    case "$1" in
        -X) method=$2; shift 2 ;;
        -f | -F)
            case "$2" in
                query=*) ;;
                *) fields="$fields $2" ;;
            esac
            shift 2
            ;;
        *) path=$1; shift ;;
    esac
done
if [ -z "$method" ]; then
    if [ -n "$fields" ]; then method=POST; else method=GET; fi
fi
key=$(printf '%s %s%s' "$method" "$path" "$fields" | tr -c 'A-Za-z0-9._-' '_')
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
if [ "$(head -c 9 "$answer")" = BODYFAIL: ]; then
    tail -c +10 "$answer"
    printf 'gh: the answer carries errors\n' >&2
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
COMPARE="$R/compare/main...$SHA?per_page=1"
# The first page of the main merge queue, as its answer key and as its call.
QUEUE='graphql owner=owner name=repo branch=main'
QUEUE_CALL='api graphql -f query=<query> -f owner=owner -f name=repo -f branch=main'
# The head commit of another queue entry.
OTHER=fedcba9876543210fedcba9876543210fedcba98

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

# queue_page <hasNextPage> <endCursor> <head commit>|null ...: one page of the
# merge queue; `null` is an entry not built yet.
queue_page() {
    local more=$1 cursor=$2 sep='' oid
    shift 2
    printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{"hasNextPage":%s,"endCursor":"%s"},"nodes":[' "$more" "$cursor"
    for oid in "$@"; do
        if [ "$oid" = null ]; then
            printf '%s{"headCommit":null}' "$sep"
        else
            printf '%s{"headCommit":{"oid":"%s"}}' "$sep" "$oid"
        fi
        sep=,
    done
    printf ']}}}}}\n'
}

# compare_json <status>: the compare of main with SHA.
compare_json() {
    printf '{"status":"%s","ahead_by":1,"behind_by":0,"commits":[]}\n' "$1"
}

# new_case <name>: a fresh answer directory in which run 555 is listed at once
# and reads as in progress, and SHA is the head commit of the second of three
# queue entries, until a case says otherwise.
new_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    answers="$case_dir/answers"
    mkdir -p "$answers"
    : > "$case_dir/calls"
    : > "$case_dir/sleeps"
    runs 555 | answer GET "$LIST"
    run_json in_progress null | answer GET "$RUN"
    queue_page false MQ "$OTHER" "$SHA" null | answer POST "$QUEUE"
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
            WATCH_BASE_REF="${FAKE_BASE_REF-refs/heads/main}" \
            FAKE_GH_CALLS="$case_dir/calls" \
            FAKE_GH_QUERY="$case_dir/query" \
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

# expect_queue_reads <n>: merge queue page reads, of every page.
expect_queue_reads() {
    local got
    got=$(grep -c '^api graphql ' "$case_dir/calls" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 merge queue read(s), got $got"
}

expect_compares() {
    local got
    got=$(grep -Fxc -- "api $COMPARE" "$case_dir/calls" || true)
    [ "$got" -eq "$1" ] || fail "$case_name: expected $1 compare read(s), got $got"
}

# expect_calls <call> ...: the exact gh calls, in order.
expect_calls() {
    local want got
    want=$(printf '%s\n' "$@")
    got=$(cat "$case_dir/calls")
    [ "$want" = "$got" ] || fail "$case_name: the gh calls are not [$(printf '%s' "$want" | tr '\n' '|')]"
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
FAKE_BASE_REF='' run_case
expect_status 2
expect_output 'WATCH_BASE_REF names no branch'
FAKE_BASE_REF='refs/heads/' run_case
expect_status 2
expect_output 'WATCH_BASE_REF names no branch'
# A PATH holding the shims, bash and grep alone has no jq.
mkdir -p "$workdir/nojq"
ln -sf "$(command -v bash)" "$workdir/nojq/bash"
ln -sf "$(command -v grep)" "$workdir/nojq/grep"
FAKE_PATH="$workdir/nojq" run_case
expect_status 2
expect_output 'jq is not on PATH'
expect_no_gh_at_all
ok 'a wrong argument, an unreadable file, a missing commit or base branch and a missing jq are refused'

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

# A live queue entry keeps the watch going: the queue is read once per poll
# that reads again, the compare never, and the run completing ends the watch
# as before.
new_case live_entry_keeps_watching
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'run 555 completed (success); nothing to cancel'
expect_output "commit $SHA is a live entry of the main merge queue; it is read every poll"
[ "$(grep -Fc 'is a live entry of the main merge queue' <<< "$output")" -eq 1 ] ||
    fail "$case_name: the live entry is said more than once"
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $JOBS" "$QUEUE_CALL" "api $JOBS" "api $RUN"
expect_cancels 0
expect_sleeps 60 60
# shellcheck disable=SC2016  # GraphQL variables, not shell expansions
grep -Fq 'mergeQueue(branch: $branch)' "$case_dir/query" || fail "$case_name: the query does not read the queue of \$branch"
# shellcheck disable=SC2016  # GraphQL variables, not shell expansions
grep -Fq 'entries(first: 100, after: $after)' "$case_dir/query" || fail "$case_name: the query does not page the entries"
grep -Fq 'pageInfo { hasNextPage endCursor }' "$case_dir/query" || fail "$case_name: the query does not read the page flag and cursor"
grep -Fq 'nodes { headCommit { oid } }' "$case_dir/query" || fail "$case_name: the query does not read the head commits"
ok 'a live queue entry keeps the watch going to the end of the run'

# Dead on one poll, live on the next: nothing is cancelled, and both turns are
# said.
new_case dead_once_then_live
queue_page false MQ "$OTHER" null | answer POST "$QUEUE" 1
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "commit $SHA is no live entry of the main merge queue and not in main; a second such poll cancels run 555"
expect_output "commit $SHA is a live entry of the main merge queue again; nothing to cancel"
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "$QUEUE_CALL" "api $JOBS" "api $RUN"
expect_cancels 0
expect_sleeps 60 60
ok 'a batch dead on one poll and live on the next is not cancelled'

# Dead, live, dead: the live poll starts the count again, so the second dead
# poll is the first of a new count and nothing is cancelled.
new_case dead_live_dead
queue_page false MQ "$OTHER" | answer POST "$QUEUE" 1
queue_page false MQ "$OTHER" | answer POST "$QUEUE" 3
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 3
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "$QUEUE_CALL" \
    "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "api $RUN"
expect_cancels 0
expect_sleeps 60 60 60
ok 'dead, live, dead cancels nothing'

# Dead on two polls in a row, the second after main moved on: cancel once,
# naming the commit, and exit 0.
new_case dead_twice_cancels
queue_page false MQ "$OTHER" null | answer POST "$QUEUE"
compare_json ahead | answer GET "$COMPARE" 1
compare_json diverged | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output "commit $SHA is no live entry of the main merge queue and not in main, on two polls in a row; cancelling run 555"
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api -X POST $CANCEL"
expect_sleeps 60
ok 'a batch dead on two polls in a row is cancelled once'

# The batch merges between the two reads of a poll: off the queue but not yet
# in main once, then in main. Nothing is cancelled, the queue is not read
# again, and the failed-job watch goes on.
new_case dead_once_then_merged
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
compare_json ahead | answer GET "$COMPARE" 1
compare_json behind | answer GET "$COMPARE"
jobs 'Lint|completed|success' 'Fuzz (Linux)|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|completed|success' 'Fuzz (Linux)|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' 'Fuzz (Linux)|in_progress|null' | answer GET "$JOBS" 3
jobs 'Lint|completed|success' 'Fuzz (Linux)|completed|failure' | answer GET "$JOBS"
run_json completed '"failure"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "commit $SHA is in main: its batch merged, so the queue is not read again"
expect_output 'run 555 completed (failure); nothing to cancel'
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" \
    "api $JOBS" "api $JOBS" "api $RUN"
expect_cancels 0
expect_sleeps 60 60 60
ok 'a batch off the queue once and then in main is never cancelled'

# A batch already in main on the first poll, its run still busy with a job
# that is not required: never cancelled.
new_case merged_never_cancels
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
compare_json identical | answer GET "$COMPARE"
jobs 'Fuzz (Linux)|in_progress|null' | answer GET "$JOBS" 1
jobs 'Fuzz (Linux)|in_progress|null' | answer GET "$JOBS" 2
jobs 'Fuzz (Linux)|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "commit $SHA is in main: its batch merged, so the queue is not read again"
expect_queue_reads 1
expect_compares 1
expect_cancels 0
expect_sleeps 60 60
ok 'a batch in main is never cancelled'

# The queue read fails on every poll: retried, said, and nothing is cancelled
# however many polls it lasts.
new_case queue_read_fails
printf 'FAIL: gh: Something went wrong while executing your query. This may be the result of a timeout\n' | answer POST "$QUEUE"
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 3
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'the main merge queue read failed three times; this poll decides nothing: FAIL: gh: Something went wrong'
expect_queue_reads 9
expect_compares 0
expect_cancels 0
expect_sleeps 10 10 60 10 10 60 10 10 60
ok 'a queue read that keeps failing cancels nothing'

# A failed queue read between two dead polls starts the count again: dead,
# unread, dead is no cancel.
new_case failed_read_between_dead_polls
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
printf 'FAIL: HTTP 502\n' | answer POST "$QUEUE" 2
printf 'FAIL: HTTP 502\n' | answer POST "$QUEUE" 3
printf 'FAIL: HTTP 502\n' | answer POST "$QUEUE" 4
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 3
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_queue_reads 5
expect_compares 2
expect_cancels 0
expect_sleeps 60 10 10 60 60
ok 'a failed read between two dead polls cancels nothing'

# The compare fails on every poll: nothing is cancelled.
new_case compare_read_fails
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
printf 'FAIL: HTTP 404: Not Found\n' | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "the compare of main with $SHA failed three times; this poll decides nothing: FAIL: HTTP 404: Not Found"
expect_compares 6
expect_cancels 0
expect_sleeps 10 10 60 10 10 60
ok 'a compare that keeps failing cancels nothing'

# The job token is refused the queue read: not retried, said once, the queue
# not read again, and a failed required job still cancels the run.
new_case queue_read_refused
printf 'FAIL: gh: Resource not accessible by integration\n' | answer POST "$QUEUE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'the job token may not read the main merge queue: FAIL: gh: Resource not accessible by integration'
expect_output 'the queue is not read again in this run and nothing is cancelled for it; a failed required job still cancels run 555'
expect_output 'required job "Lint" ended failure; cancelling run 555'
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $JOBS" "api $JOBS" "api -X POST $CANCEL"
expect_sleeps 60 60
ok 'a queue read refused to the job token turns the dead check off'

# A GraphQL error body of type FORBIDDEN is a refusal too; one of another
# type is retried as a failed read.
new_case queue_read_forbidden_body
printf 'BODYFAIL:{"data":{"repository":{"mergeQueue":null}},"errors":[{"type": "FORBIDDEN","message":"no"}]}\n' |
    answer POST "$QUEUE" 2
printf 'BODYFAIL:{"errors":[{"type":"RATE_LIMITED","message":"API rate limit exceeded"}]}\n' | answer POST "$QUEUE" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'the job token may not read the main merge queue: gh: the answer carries errors'
expect_queue_reads 2
expect_cancels 0
expect_sleeps 10 60 60
ok 'a FORBIDDEN error body is a refusal and a rate limit is not'

# The job token is refused the compare: not retried, and the check turns off.
new_case compare_refused
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
printf 'FAIL: gh: Resource not accessible by integration (HTTP 403)\n' | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "the job token may not compare main with $SHA: FAIL: gh: Resource not accessible by integration (HTTP 403)"
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "api $COMPARE" "api $JOBS" "api $JOBS" "api $RUN"
expect_cancels 0
expect_sleeps 60 60
ok 'a compare refused to the job token turns the dead check off'

# The entry is on the second page: the first page's cursor reads it, and the
# compare is never asked.
new_case entry_on_page_two
queue_page true CURSOR1 "$OTHER" null | answer POST "$QUEUE"
queue_page false CURSOR2 null "$SHA" | answer POST "$QUEUE after=CURSOR1"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|failure' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'required job "Lint" ended failure; cancelling run 555'
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "$QUEUE_CALL -f after=CURSOR1" \
    "api $JOBS" "$QUEUE_CALL" "$QUEUE_CALL -f after=CURSOR1" "api $JOBS" "api -X POST $CANCEL"
ok 'an entry on the second page of the queue is found'

# Two pages without the commit: dead, after every page is read.
new_case absent_from_every_page
queue_page true CURSOR1 "$OTHER" | answer POST "$QUEUE"
queue_page false CURSOR2 null | answer POST "$QUEUE after=CURSOR1"
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS"
: | answer POST "$CANCEL"
run_case
expect_status 0
expect_calls "api $LIST" "api $JOBS" "$QUEUE_CALL" "$QUEUE_CALL -f after=CURSOR1" "api $COMPARE" \
    "api $JOBS" "$QUEUE_CALL" "$QUEUE_CALL -f after=CURSOR1" "api $COMPARE" "api -X POST $CANCEL"
ok 'a commit on no page of the queue reads as dead'

# Answers missing a field the decision reads decide nothing: no merge queue,
# an entry with no head commit field, a page with no flag, a next page with
# no cursor, a head commit with no oid, an oid that is not a string, a body
# that is not JSON.
new_case queue_answer_malformed
printf '{"data":{"repository":{"mergeQueue":null}}}\n' | answer POST "$QUEUE" 1
printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{"hasNextPage":false},"nodes":[{}]}}}}}\n' |
    answer POST "$QUEUE" 2
printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{},"nodes":[]}}}}}\n' | answer POST "$QUEUE" 3
printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{"hasNextPage":true,"endCursor":null},"nodes":[]}}}}}\n' |
    answer POST "$QUEUE" 4
printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{"hasNextPage":false},"nodes":[{"headCommit":{}}]}}}}}\n' |
    answer POST "$QUEUE" 5
printf '{"data":{"repository":{"mergeQueue":{"entries":{"pageInfo":{"hasNextPage":false},"nodes":[{"headCommit":{"oid":1}}]}}}}}\n' |
    answer POST "$QUEUE" 6
printf 'not json\n' | answer POST "$QUEUE"
compare_json ahead | answer GET "$COMPARE"
for n in 1 2 3 4 5 6 7; do
    jobs 'Lint|in_progress|null' | answer GET "$JOBS" "$n"
done
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'the main merge queue answer carries no entry list, head commit, page flag or cursor; this poll decides nothing'
[ "$(grep -Fc 'this poll decides nothing' <<< "$output")" -eq 7 ] || fail "$case_name: expected seven polls that decide nothing"
expect_queue_reads 7
expect_compares 0
expect_cancels 0
ok 'a queue answer missing a field decides nothing'

# A compare with a status the watch does not know, or a body that is not
# JSON, decides nothing.
new_case compare_status_unknown
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
printf '<html>unicorn</html>\n' | answer GET "$COMPARE" 1
printf '{"message":"Not Found","status":"404"}\n' | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output "the compare of main with $SHA carries no known status; this poll decides nothing"
expect_compares 2
expect_cancels 0
[ "$(grep -Fc 'carries no known status; this poll decides nothing' <<< "$output")" -eq 2 ] ||
    fail "$case_name: expected two polls that decide nothing"
ok 'a compare with an unknown status or a body that is not JSON decides nothing'

# A queue whose pages never end is read ten pages deep, then decides nothing.
new_case endless_pages
queue_page true CURSOR1 "$OTHER" | answer POST "$QUEUE"
queue_page true CURSOR1 "$OTHER" | answer POST "$QUEUE after=CURSOR1"
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 1
jobs 'Lint|in_progress|null' | answer GET "$JOBS" 2
jobs 'Lint|completed|success' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
run_case
expect_status 0
expect_output 'the main merge queue lists more than 1000 entries; this poll decides nothing'
expect_queue_reads 20
expect_compares 0
expect_cancels 0
ok 'a queue whose pages never end decides nothing'

# The base branch is the one the event names: its queue is read and it is
# the compare's base.
new_case base_branch_from_the_event
queue_page false MQ "$OTHER" | answer POST 'graphql owner=owner name=repo branch=release/2.x'
compare_json ahead | answer GET "$R/compare/release/2.x...$SHA?per_page=1"
jobs 'Lint|in_progress|null' | answer GET "$JOBS"
: | answer POST "$CANCEL"
FAKE_BASE_REF=refs/heads/release/2.x run_case
expect_status 0
expect_output "commit $SHA is no live entry of the release/2.x merge queue and not in release/2.x, on two polls in a row; cancelling run 555"
expect_calls "api $LIST" "api $JOBS" 'api graphql -f query=<query> -f owner=owner -f name=repo -f branch=release/2.x' \
    "api $R/compare/release/2.x...$SHA?per_page=1" "api $JOBS" \
    'api graphql -f query=<query> -f owner=owner -f name=repo -f branch=release/2.x' \
    "api $R/compare/release/2.x...$SHA?per_page=1" "api -X POST $CANCEL"
ok 'the base branch comes from the event'

# The dead batch's run completed before the cancel reached it: no error.
new_case dead_cancel_after_the_run_ended
queue_page false MQ "$OTHER" | answer POST "$QUEUE"
compare_json ahead | answer GET "$COMPARE"
jobs 'Lint|in_progress|null' | answer GET "$JOBS"
run_json completed '"success"' | answer GET "$RUN"
printf 'FAIL: HTTP 409: Cannot cancel a workflow run that is completed.\n' | answer POST "$CANCEL"
run_case
expect_status 0
expect_output 'the run completed (success) before the cancel reached it; nothing to cancel'
expect_cancels 3
ok 'a dead batch cancel refused because the run had completed exits 0'

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

# The same dead answers twice give the same calls, waits and output.
dead_answers() {
    queue_page false MQ "$OTHER" null | answer POST "$QUEUE"
    compare_json ahead | answer GET "$COMPARE"
    jobs 'Lint|in_progress|null' | answer GET "$JOBS"
    : | answer POST "$CANCEL"
}
new_case dead_twice_first
dead_answers
run_case
expect_status 0
expect_cancels 1
cp "$case_dir/calls" "$workdir/first_dead_calls"
cp "$case_dir/sleeps" "$workdir/first_dead_sleeps"
first_dead_output=$output
new_case dead_twice_second
dead_answers
run_case
expect_status 0
expect_cancels 1
cmp -s "$case_dir/calls" "$workdir/first_dead_calls" || fail "$case_name: the calls differ from the first run"
cmp -s "$case_dir/sleeps" "$workdir/first_dead_sleeps" || fail "$case_name: the waits differ from the first run"
[ "$output" = "$first_dead_output" ] || fail "$case_name: the output differs from the first run"
ok 'the same dead answers give the same calls and output'

# `usage` ran seven invocations and each of the last two cases two; the table
# counts behaviours.
cases=$((cases - 8))
[ "$cases" -eq "$CASES_EXPECTED" ] || fail "ran $cases case(s), CASES_EXPECTED is $CASES_EXPECTED"
printf 'test_ci_queue_watchdog: all %s cases passed\n' "$cases"
