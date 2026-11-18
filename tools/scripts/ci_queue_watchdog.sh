#!/usr/bin/env bash
# ci_queue_watchdog.sh: cancel a merge-queue run of ci.yml at its first failed
# required job, or once its batch has left the queue without merging.
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
# Before each wait it also reads whether the batch is still live:
#
#   WATCH_SHA is the head commit of an entry of the merge queue of the base
#   branch, every page of entries read      live
#   it is no entry's head commit, and the compare of the base branch with it
#   reads `identical` or `behind`           merged: the queue is not read again
#   it is no entry's head commit, and the compare reads `ahead` or `diverged`
#                                           dead
#
# GitHub does not cancel a merge-queue run when its batch leaves the queue (the
# pull request is removed or queued again, or a batch ahead of it fails): the
# run fails nothing and holds its runners to its end. Dead on two polls in a
# row cancels the run, with the cancel handling of a failed job, and exits 0.
# One dead poll alone cancels nothing: the queue pages and the compare are
# separate reads, so a batch that moves or merges between them can read as dead
# once. A read that fails three times, and an answer missing a field the
# decision reads, decide nothing for that poll and start the count of dead
# polls again, so no run of failed reads becomes a cancel.
#
# Each poll is one jobs read and one queue read, plus the run read once no job
# is left running and the compare while the commit is no entry's head commit.
# The REST reads come out of the job token's hourly request budget that every
# job of this repository shares; the queue read is GraphQL, counted against a
# budget of its own.
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
# file or one naming no context; `jq` absent; no repository, commit or base
# branch named; a run that does not appear, or more than one run for the
# commit; a jobs, run or run listing read that fails three times. A cancel
# request refused three times while the run still reads as not completed exits
# 1; refused because the run had completed meanwhile, it exits 0. Nothing is
# cancelled on any refusal, and the workflow reports no required context, so
# none of them can block the batch.
#
# ENVIRONMENT. `GH_TOKEN` is read by `gh` itself; `GITHUB_REPOSITORY` (or
# `GH_REPO`) names the repository, WATCH_SHA the queue commit and
# WATCH_BASE_REF the branch the queue merges into, as the event names it
# (`refs/heads/main`).
#
# Oracle: tools/scripts/test_ci_queue_watchdog.sh.

set -euo pipefail

WORKFLOW=ci.yml
POLL_SECONDS=60
FIND_TRIES=20
FIND_SECONDS=15
# One page of jobs; ci.yml expands to fewer jobs than this.
MAX_JOBS=100
# Pages of 100 queue entries read in one poll before it decides nothing.
MAX_QUEUE_PAGES=10

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
base=${WATCH_BASE_REF:-}
base=${base#refs/heads/}
[ -n "$base" ] || die 'WATCH_BASE_REF names no branch: there is no merge queue to read' 2
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

# Cancels the run and exits: 0 once the cancel is taken, or when it was refused
# because the run had completed; 1 when it is refused while the run runs.
cancel_run() {
    local refusal state
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

# One page of the base branch's merge queue. An entry that is not built yet
# has a null head commit.
# shellcheck disable=SC2016  # GraphQL variables, not shell expansions
queue_query='query($owner: String!, $name: String!, $branch: String!, $after: String) {
  repository(owner: $owner, name: $name) {
    mergeQueue(branch: $branch) {
      entries(first: 100, after: $after) {
        pageInfo { hasNextPage endCursor }
        nodes { headCommit { oid } }
      }
    }
  }
}'

# One line per queue page: `found` when an entry's head commit is $sha, else
# `more<TAB><cursor>` or `end`; `malformed` when the page has no entry list, an
# entry without a null or `{oid}` head commit, no page flag, or no cursor for
# the next page.
# shellcheck disable=SC2016  # a jq program, not a shell expansion
queue_verdict='
def entry_ok: type == "object" and has("headCommit")
  and (.headCommit == null
       or ((.headCommit | type) == "object" and (.headCommit.oid | type) == "string"));
.data.repository.mergeQueue.entries as $e
| if ($e | type) != "object" or ($e.nodes | type) != "array"
     or ($e.nodes | all(entry_ok) | not)
     or ($e.pageInfo | type) != "object" or ($e.pageInfo.hasNextPage | type) != "boolean"
    then "malformed"
  elif any($e.nodes[]; .headCommit != null and .headCommit.oid == $sha) then "found"
  elif ($e.pageInfo.hasNextPage | not) then "end"
  elif ($e.pageInfo.endCursor | type) == "string" and ($e.pageInfo.endCursor | length) > 0
    then "more\t\($e.pageInfo.endCursor)"
  else "malformed"
  end'

# `merged` when $sha is the base branch's tip or behind it, `dead` when it is
# ahead of it or diverged from it, `malformed` for any other status.
compare_verdict='
if .status == "identical" or .status == "behind" then "merged"
elif .status == "ahead" or .status == "diverged" then "dead"
else "malformed"
end'

owner=${repository%%/*}
name=${repository#*/}

# Sets `batch` to `live`, `merged` or `dead`, or to `unread` after saying which
# read failed or carried no answer.
read_batch() {
    local cursor='' page verdict pages=0
    local -a args
    batch=unread
    while :; do
        if [ "$pages" -ge "$MAX_QUEUE_PAGES" ]; then
            say "the ${base} merge queue lists more than $((MAX_QUEUE_PAGES * 100)) entries; this poll decides nothing"
            return 0
        fi
        pages=$((pages + 1))
        args=(graphql -f query="$queue_query" -f owner="$owner" -f name="$name" -f branch="$base")
        [ -z "$cursor" ] || args+=(-f after="$cursor")
        if ! page=$(api_try "${args[@]}"); then
            say "the ${base} merge queue read failed three times; this poll decides nothing: $(tr '\n' ' ' < "$err_file")"
            return 0
        fi
        verdict=$(jq -r --arg sha "$sha" "$queue_verdict" <<< "$page") || verdict=malformed
        case $verdict in
            found)
                batch=live
                return 0
                ;;
            end)
                break
                ;;
            more$'\t'*)
                cursor=${verdict#more$'\t'}
                ;;
            *)
                say "the ${base} merge queue answer carries no entry list, head commit, page flag or cursor; this poll decides nothing"
                return 0
                ;;
        esac
    done
    if ! page=$(api_try "repos/${repository}/compare/${base}...${sha}?per_page=1"); then
        say "the compare of ${base} with ${sha} failed three times; this poll decides nothing: $(tr '\n' ' ' < "$err_file")"
        return 0
    fi
    verdict=$(jq -r "$compare_verdict" <<< "$page") || verdict=malformed
    case $verdict in
        merged | dead)
            batch=$verdict
            ;;
        *)
            say "the compare of ${base} with ${sha} carries no known status; this poll decides nothing"
            ;;
    esac
}

dead_polls=0
queue_read=yes
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
            cancel_run
            ;;
        settled)
            state=$(run_state)
            if [ "${state%% *}" = completed ]; then
                say "run ${run_id} completed (${state#* }); nothing to cancel"
                exit 0
            fi
            ;;
    esac
    if [ "$queue_read" = yes ]; then
        read_batch
        case $batch in
            live)
                if [ "$dead_polls" -gt 0 ]; then
                    say "commit ${sha} is a live entry of the ${base} merge queue again; nothing to cancel"
                fi
                dead_polls=0
                ;;
            merged)
                say "commit ${sha} is in ${base}: its batch merged, so the queue is not read again"
                queue_read=no
                dead_polls=0
                ;;
            dead)
                dead_polls=$((dead_polls + 1))
                if [ "$dead_polls" -ge 2 ]; then
                    say "commit ${sha} is no live entry of the ${base} merge queue and not in ${base}, on two polls in a row; cancelling run ${run_id}"
                    cancel_run
                fi
                say "commit ${sha} is no live entry of the ${base} merge queue and not in ${base}; a second such poll cancels run ${run_id}"
                ;;
            *)
                dead_polls=0
                ;;
        esac
    fi
    sleep "$POLL_SECONDS"
done
