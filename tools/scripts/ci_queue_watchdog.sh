#!/usr/bin/env bash
# ci_queue_watchdog.sh: cancel a merge-queue run of ci.yml at its first failed
# required job.
#
#   tools/scripts/ci_queue_watchdog.sh <required-contexts-file>
#
# Runs in `queue-watchdog.yml`, on the same merge-queue event as the `ci.yml`
# run it watches. It finds that run as the one run of WORKFLOW with event
# `merge_group` and head commit WATCH_SHA, then every POLL_SECONDS reads the
# newest attempt's jobs of that run:
#
#   a job whose name is a required context ended failure, timed_out or
#   cancelled                               cancel the run, name the job, exit 0
#   a job is still queued or running        read again
#   no job is listed, or every one has completed
#                                           read the run itself: completed,
#                                           exit 0 with nothing cancelled; not
#                                           yet, read again
#
# One read a minute while jobs run, two once none is left running: the job's
# token has an hourly request budget that every job of this repository
# shares.
#
# A failed required context fails the queue batch on its own. What the rest of
# the run then adds is its runner minutes, and its hold on its concurrency
# group: a pull request queued again on the same `main` gets the same queue
# branch, so its new run waits in that group until the failed one ends.
#
# The required names are the rows of <required-contexts-file>
# (tools/ci/required_contexts.txt), `#` lines and blank lines skipped. Three of
# them are reported by `leak-guard.yml`, never by a `ci.yml` job, so a failure
# there cancels nothing here.
#
# REFUSALS, exit 2 with one message each: a wrong argument count; an unreadable
# file or one naming no context; `jq` absent; no repository or commit named; a
# run that does not appear, or more than one run for the commit; a `gh api`
# read that fails three times. A cancel request refused three times while the
# run still reads as not completed exits 1; refused because the run had
# completed meanwhile, it exits 0. Nothing is cancelled on any refusal, and the
# workflow reports no required context, so none of them can block the batch.
#
# ENVIRONMENT. `GH_TOKEN` is read by `gh` itself; `GITHUB_REPOSITORY` (or
# `GH_REPO`) names the repository and WATCH_SHA the queue commit.
#
# Oracle: tools/scripts/test_ci_queue_watchdog.sh.

set -euo pipefail

WORKFLOW=ci.yml
POLL_SECONDS=60
FIND_TRIES=20
FIND_SECONDS=15
# One page of jobs; ci.yml expands to fewer jobs than this.
MAX_JOBS=100

die() {
    printf 'ci_queue_watchdog: %s\n' "$1" >&2
    exit "$2"
}

say() {
    printf 'ci_queue_watchdog: %s\n' "$1"
}

[ "$#" -eq 1 ] || die 'usage: ci_queue_watchdog.sh <required-contexts-file>' 2
[ -r "$1" ] || die "the required contexts file cannot be read: $1" 2
required=$(grep -v '^#' "$1" | grep . || true)
[ -n "$required" ] || die "the required contexts file names no context: $1" 2
repository=${GITHUB_REPOSITORY:-${GH_REPO:-}}
[ -n "$repository" ] || die 'GITHUB_REPOSITORY and GH_REPO are both empty: the run names no repository' 2
sha=${WATCH_SHA:-}
[ -n "$sha" ] || die 'WATCH_SHA is empty: there is no queue commit to watch' 2
command -v jq >/dev/null 2>&1 || die 'jq is not on PATH: the API answers cannot be read without it' 2

err_file=$(mktemp "${TMPDIR:-/tmp}/watchdog-err.XXXXXX")
trap 'rm -f "$err_file"' EXIT

# `gh api` with three attempts, 10 s apart: a run can last two hours, and one
# server error must not end the watch. Prints the answer on stdout; after the third
# failure it returns 1 with the error in err_file.
api_try() {
    local out attempt=1
    while :; do
        if out=$(gh api "$@" 2> "$err_file"); then
            printf '%s\n' "$out"
            return 0
        fi
        [ "$attempt" -lt 3 ] || return 1
        attempt=$((attempt + 1))
        sleep 10
    done
}

# api_try, refusing after the third failure.
api() {
    api_try "$@" || die "gh api $* failed three times: $(tr '\n' ' ' < "$err_file")" 2
}

# The run to watch: created by the same event as this one, so it may be listed
# a moment later.
try=1
while :; do
    page=$(api "repos/${repository}/actions/workflows/${WORKFLOW}/runs?event=merge_group&head_sha=${sha}&per_page=10")
    ids=$(jq -r --arg sha "$sha" '.workflow_runs[] | select(.head_sha == $sha) | .id' <<< "$page")
    count=$(printf '%s' "$ids" | grep -c . || true)
    [ "$count" -le 1 ] ||
        die "${count} runs of ${WORKFLOW} carry commit ${sha} ($(printf '%s' "$ids" | tr '\n' ' ')): which one to watch cannot be told" 2
    [ "$count" -eq 0 ] || break
    [ "$try" -lt "$FIND_TRIES" ] ||
        die "no merge-queue run of ${WORKFLOW} for commit ${sha} appeared within $((FIND_TRIES * FIND_SECONDS)) s" 2
    try=$((try + 1))
    sleep "$FIND_SECONDS"
done
run_id=$ids
say "watching run ${run_id} of ${WORKFLOW} for commit ${sha}"

# The run's status and conclusion, as `<status> <conclusion>`.
run_state() {
    local json
    json=$(api "repos/${repository}/actions/runs/${run_id}") || exit 2
    jq -r '"\(.status) \(.conclusion // "none")"' <<< "$json"
}

# One line: `cancel<TAB><job><TAB><conclusion>`, `running` or `settled`. A job
# that has not been queued yet is not listed; it is read once it is.
# shellcheck disable=SC2016  # a jq program, not a shell expansion
decide='
($req | split("\n") | map(select(length > 0))) as $r
| [.jobs[]
   | select((.name | IN($r[])) and (.conclusion | IN("failure", "timed_out", "cancelled")))
  ] as $bad
| if ($bad | length) > 0 then "cancel\t\($bad[0].name)\t\($bad[0].conclusion)"
  elif any(.jobs[]; .status != "completed") then "running"
  else "settled"
  end'

while :; do
    page=$(api "repos/${repository}/actions/runs/${run_id}/jobs?filter=latest&per_page=${MAX_JOBS}")
    total=$(jq -r '.total_count // 0' <<< "$page")
    if [ "$total" -gt "$MAX_JOBS" ]; then
        say "the run has ${total} jobs; only the first ${MAX_JOBS} are read"
    fi
    decision=$(jq -r --arg req "$required" "$decide" <<< "$page")
    case $decision in
        cancel*)
            IFS=$'\t' read -r _ job conclusion <<< "$decision"
            say "required job \"${job}\" ended ${conclusion}; cancelling run ${run_id}"
            if api_try -X POST "repos/${repository}/actions/runs/${run_id}/cancel" > /dev/null; then
                exit 0
            fi
            refusal=$(tr '\n' ' ' < "$err_file")
            state=$(run_state)
            if [ "${state%% *}" = completed ]; then
                say "the run completed (${state#* }) before the cancel reached it; nothing to cancel"
                exit 0
            fi
            die "the cancel request was refused: ${refusal}" 1
            ;;
        settled)
            state=$(run_state)
            if [ "${state%% *}" = completed ]; then
                say "run ${run_id} completed (${state#* }); nothing to cancel"
                exit 0
            fi
            ;;
    esac
    sleep "$POLL_SECONDS"
done
