#!/usr/bin/env bash
# ci_queue_watchdog.sh: cancel a merge-queue run of ci.yml at its first failed
# required job, or once its batch has left the queue without merging.
#
#   tools/scripts/ci_queue_watchdog.sh <required-contexts-file> <required-parts-file>
#
# Runs in `queue-watchdog.yml`, on the same merge-queue event as the `ci.yml`
# run it watches. It finds that run as the one run of WORKFLOW with event
# `merge_group` and head commit WATCH_SHA, then every POLL_SECONDS reads the
# newest attempt's jobs of that run:
#
#   a job whose name is a required context, or a part of one, ended
#   failure, timed_out or cancelled         cancel the run, name the job, exit 0
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
# polls again, so no run of failed reads becomes a cancel. A queue or compare
# answer refusing the job token (`Resource not accessible`, or a GraphQL error
# of type FORBIDDEN or INSUFFICIENT_SCOPES) is not retried: it is said once,
# the queue is not read again in that run, and nothing is cancelled for it,
# while a failed required job still cancels the run. The first queue read
# that finds the commit says so once, so the log shows the read works.
#
# Each poll is one jobs read, plus the run read once no job is left running;
# until the batch has merged or a read is refused, one queue read per page of
# 100 entries, and the compare while the commit is no entry's head commit. The REST reads come out
# of the job token's hourly REST budget, the queue reads out of its hourly
# GraphQL budget; each is per repository and shared by every job of it.
#
# A failed required context fails the queue batch on its own. What the rest of
# the run then adds is its runner minutes, and its hold on its concurrency
# group: a pull request queued again on the same `main` gets the same queue
# branch, so its new run waits in that group until the failed one ends.
#
# The required names are the rows of <required-contexts-file>
# (tools/ci/required_contexts.txt), `#` lines and blank lines skipped. Three of
# them are reported by `leak-guard.yml`, never by a `ci.yml` job, so a failure
# there cancels nothing through the jobs read; it takes the batch out of the
# queue, and the dead batch check cancels the run two polls later.
#
# THE PARTS. A required context reported by an aggregate job (`Lint`) ends only
# once every job it needs has ended, and it fails unless every one of them
# succeeded. The rows of <required-parts-file> (tools/ci/required_parts.txt)
# are `<part job name>|<required context>`, `#` lines and empty lines skipped,
# and a part that ended failure, timed_out or cancelled cancels the run as its
# context would, up to minutes before the aggregate reports. A cancelled part is read
# the same way. A cancel that is taken ends the watch, so the cancelled jobs it
# makes are never read as a reason to ask again, and a run that finished
# cancelling meanwhile is read after the first refused request and asked no
# more.
#
# REFUSALS, exit 2 with one message each: a wrong argument count; an unreadable
# file, a contexts file naming no context or a parts file naming no part; a
# part row that is not `<name>|<context>`, a part listed twice, a part that is
# itself a required context, or a part of a context the contexts file does not
# name; `jq` absent; no repository, commit or base branch named; a run that
# does not appear, or more than one run for the commit; a jobs read, a run
# read once no job runs, or a run listing read that fails three times. A
# refused cancel request is followed by a read of the run: completed
# meanwhile, the watch exits 0 with no second request; after the third
# refusal with the run not read as completed, or not read at all, it exits 1
# naming the refusal. Nothing is cancelled on any refusal, and the workflow
# reports no required context, so none of them can block the batch.
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

[ "$#" -eq 2 ] || die 'usage: ci_queue_watchdog.sh <required-contexts-file> <required-parts-file>' 2
[ -r "$1" ] || die "the required contexts file cannot be read: $1" 2
required=$(grep -v '^#' "$1" | grep . || true)
[ -n "$required" ] || die "the required contexts file names no context: $1" 2
[ -r "$2" ] || die "the required parts file cannot be read: $2" 2
parts=$(grep -v '^#' "$2" | grep . || true)
[ -n "$parts" ] || die "the required parts file names no part: $2" 2
# Whether one parts row is `<part job name>|<required context>`: exactly one
# `|`, a name on each side, and no blank at either end of a name, since a name
# with one is a name no job carries.
part_row_ok() {
    local part context
    case $1 in
        *'|'*) ;;
        *) return 1 ;;
    esac
    part=${1%%|*}
    context=${1#*|}
    case $context in
        *'|'*) return 1 ;;
    esac
    [ -n "$part" ] && [ -n "$context" ] || return 1
    case $part in
        [[:space:]]* | *[[:space:]]) return 1 ;;
    esac
    case $context in
        [[:space:]]* | *[[:space:]]) return 1 ;;
    esac
}
seen_parts=
while IFS= read -r row; do
    part_row_ok "$row" || die "the required parts file row '${row}' is not <part job name>|<required context>: $2" 2
    part=${row%%|*}
    context=${row#*|}
    ! grep -Fxq -- "$part" <<< "$seen_parts" || die "the required parts file lists the part '${part}' twice: $2" 2
    ! grep -Fxq -- "$part" <<< "$required" ||
        die "the part '${part}' is itself a required context, so it cannot be a part of '${context}': $2" 2
    grep -Fxq -- "$context" <<< "$required" ||
        die "the part '${part}' belongs to '${context}', which the required contexts file does not name: $2" 2
    seen_parts="${seen_parts}${part}"$'\n'
done <<< "$parts"
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
# failure it returns 1 with the error in err_file. With `--refusal-ends` first,
# an answer refusing the job token is not retried: it returns 3 at once.
api_try() {
    local out attempt=1 refusal_ends=no
    local refusal='Resource not accessible|"type": *"(FORBIDDEN|INSUFFICIENT_SCOPES)"'
    if [ "${1:-}" = --refusal-ends ]; then
        refusal_ends=yes
        shift
    fi
    while :; do
        if out=$(gh api "$@" 2> "$err_file"); then
            printf '%s\n' "$out"
            return 0
        fi
        # Two reads with no pipe: under pipefail, `grep -q` ending a pipe early
        # can fail the writer ahead of it, and the refusal would read as none.
        if [ "$refusal_ends" = yes ] &&
            { grep -Eq "$refusal" <<< "$out" || grep -Eq "$refusal" "$err_file"; }; then
            return 3
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
# because the run had completed; 1 when it is refused three times while the run
# does not read as completed. Each refusal is followed by a read of the run
# (three tries, as every read), so a run that completed meanwhile is asked no
# more; a run that cannot be read is said, and the cancel is asked again.
cancel_run() {
    local refusal json state attempt=1
    while :; do
        if gh api -X POST "repos/${repository}/actions/runs/${run_id}/cancel" > /dev/null 2> "$err_file"; then
            exit 0
        fi
        refusal=$(tr '\n' ' ' < "$err_file")
        if json=$(api_try "repos/${repository}/actions/runs/${run_id}"); then
            state=$(jq -r '"\(.status) \(.conclusion // "none")"' <<< "$json" 2> /dev/null) || {
                say "the run read after a refused cancel is not JSON; the cancel is asked again"
                state='unread none'
            }
            if [ "${state%% *}" = completed ]; then
                say "the run completed (${state#* }) before the cancel reached it; nothing to cancel"
                exit 0
            fi
        else
            say "the run read after a refused cancel failed three times: $(tr '\n' ' ' < "$err_file")"
        fi
        [ "$attempt" -lt 3 ] || die "the cancel request was refused: ${refusal}" 1
        attempt=$((attempt + 1))
        sleep 10
    done
}

# One line: `cancel<TAB><job><TAB><conclusion>[<TAB><context>]`, `running` or
# `settled`; the context is there when the job is a part of one. A job that has
# not been queued yet is not listed; it is read once it is.
# shellcheck disable=SC2016  # a jq program, not a shell expansion
decide='
($req | split("\n") | map(select(length > 0))) as $r
| ($parts | split("\n") | map(select(length > 0) | split("|") | {key: .[0], value: .[1]})
   | from_entries) as $p
| [.jobs[]
   | select(((.name | IN($r[])) or ($p[.name // ""] != null))
            and (.conclusion | IN("failure", "timed_out", "cancelled")))
  ] as $bad
| if ($bad | length) > 0
    then "cancel\t\($bad[0].name)\t\($bad[0].conclusion)" + (if $p[$bad[0].name // ""] != null then "\t\($p[$bad[0].name])" else "" end)
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

# Sets `batch` to `live`, `merged` or `dead`; to `refused` after saying which
# read refused the job token; or to `unread` after saying which read failed or
# carried no answer.
read_batch() {
    local cursor='' page verdict pages=0 rc
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
        if page=$(api_try --refusal-ends "${args[@]}"); then rc=0; else rc=$?; fi
        if [ "$rc" -eq 3 ]; then
            say "the job token may not read the ${base} merge queue: $(tr '\n' ' ' < "$err_file")"
            batch=refused
            return 0
        fi
        if [ "$rc" -ne 0 ]; then
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
    if page=$(api_try --refusal-ends "repos/${repository}/compare/${base}...${sha}?per_page=1"); then rc=0; else rc=$?; fi
    if [ "$rc" -eq 3 ]; then
        say "the job token may not compare ${base} with ${sha}: $(tr '\n' ' ' < "$err_file")"
        batch=refused
        return 0
    fi
    if [ "$rc" -ne 0 ]; then
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
seen_live=no
while :; do
    page=$(api "repos/${repository}/actions/runs/${run_id}/jobs?filter=latest&per_page=${MAX_JOBS}")
    total=$(jq -r '.total_count // 0' <<< "$page")
    if [ "$total" -gt "$MAX_JOBS" ]; then
        say "the run has ${total} jobs; only the first ${MAX_JOBS} are read"
    fi
    decision=$(jq -r --arg req "$required" --arg parts "$parts" "$decide" <<< "$page")
    case $decision in
        cancel*)
            IFS=$'\t' read -r _ job conclusion context <<< "$decision"
            if [ -n "$context" ]; then
                say "job \"${job}\", a part of required \"${context}\", ended ${conclusion}; cancelling run ${run_id}"
            else
                say "required job \"${job}\" ended ${conclusion}; cancelling run ${run_id}"
            fi
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
                elif [ "$seen_live" = no ]; then
                    say "commit ${sha} is a live entry of the ${base} merge queue; it is read every poll"
                fi
                seen_live=yes
                dead_polls=0
                ;;
            merged)
                say "commit ${sha} is in ${base}: its batch merged, so the queue is not read again"
                queue_read=no
                ;;
            refused)
                say "the queue is not read again in this run and nothing is cancelled for it; a failed required job still cancels run ${run_id}"
                queue_read=no
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
