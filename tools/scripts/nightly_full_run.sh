#!/usr/bin/env bash
# nightly_full_run.sh: the nightly full run of `main`, and the alarm for when it stops.
#
#   tools/scripts/nightly_full_run.sh run
#   tools/scripts/nightly_full_run.sh alarm
#
# RUN dispatches every workflow in WORKFLOWS on `main` with the job's own token,
# waits for each run to finish, reruns the failed jobs of each red run once,
# waits again, and records the result on ONE issue. A failed step skips every
# later step of its job: the rerun shows whether the failure repeats, and when
# it does not, the steps it skipped run too. A job that passes on the rerun
# failed on the first attempt and is reported as such.
#
#   every run green on its first attempt, no such issue open   no write
#   every run green on its first attempt, one such issue open  comment, close it
#   any run red, or not finished by the deadline, no issue     open it
#   any run red, or not finished by the deadline, one issue    comment on it
#
# When the dispatch answers with the id of the run it started, that id is the
# run. When the answer carries none (an empty 204), the run is the one run of
# that workflow, event
# `workflow_dispatch` and branch `main` whose actor or triggering actor is BOT
# (the account the job's token acts as) and which was not listed before the
# dispatch; a person's dispatch carries that person in both, and `nightly.yml`
# holds a concurrency group, so two nightly runs never dispatch at once. No
# rerun is requested once the deadline has passed.
#
# RUN exits 0 once the result is recorded, red or green, or once every run
# passed with no issue open to close, so a successful run of `nightly.yml` means
# a result was reached. A refusal exits 2, and the alarm reports that run.
#
# ALARM reads the runs of NIGHTLY_WORKFLOW on `main` and writes to the same
# issue when the workflow is not active, or when no run of it is in progress
# and none reached a result (succeeded) in the last STALE_SECONDS, counted
# from the newest success's end, or from the workflow's creation when it has
# none. It writes at most once per REPEAT_SECONDS: an open issue updated more
# recently than that is left alone.
#
# REFUSALS, exit 2 with one message each: an unknown mode; no repository named;
# `jq` absent; a `gh api` call that fails three times; a dispatched run that does
# not appear; more than one bot-dispatched run appearing after one dispatch; a
# timestamp that does not parse; a refused issue listing; two open issues
# carrying TITLE, which holds both modes until one is closed.
#
# ENVIRONMENT. `GH_TOKEN` is read by `gh` itself. `GITHUB_REPOSITORY` (or
# `GH_REPO`), `GITHUB_SERVER_URL` and `GITHUB_RUN_ID` name the repository and
# this run's URL.
#
# Oracle: tools/scripts/test_nightly_full_run.sh.
#
# The issue text is markdown and carries backticks; none of it is a shell
# expansion.
# shellcheck disable=SC2016

set -euo pipefail

WORKFLOWS='ci.yml rmw-distros.yml'
REF=main
BOT='github-actions[bot]'
NIGHTLY_WORKFLOW=nightly.yml
TITLE='Nightly full run of main failed or did not run'
LABEL=bug
POLL_SECONDS=180
# Under the 345-minute limit of the nightly job, so the result is written before
# the runner is taken away.
DEADLINE_SECONDS=$((330 * 60))
FIND_TRIES=20
FIND_SECONDS=15
STALE_SECONDS=$((26 * 3600))
REPEAT_SECONDS=$((24 * 3600))
# One page of jobs per run attempt; ci.yml expands to fewer jobs than this.
MAX_JOBS=100

die() {
    printf 'nightly_full_run: %s\n' "$1" >&2
    exit "$2"
}

say() {
    printf 'nightly_full_run: %s\n' "$1"
}

[ "$#" -eq 1 ] || die 'usage: nightly_full_run.sh run|alarm' 2
mode=$1
case $mode in
    run | alarm) ;;
    *) die "unknown mode: ${mode} (usage: nightly_full_run.sh run|alarm)" 2 ;;
esac

repository=${GITHUB_REPOSITORY:-${GH_REPO:-}}
[ -n "$repository" ] || die 'GITHUB_REPOSITORY and GH_REPO are both empty: the run names no repository' 2
server=${GITHUB_SERVER_URL:-https://github.com}
self_url="${server}/${repository}/actions/runs/${GITHUB_RUN_ID:-unknown}"
command -v jq >/dev/null 2>&1 || die 'jq is not on PATH: the API answers cannot be read without it' 2

body_file=$(mktemp "${TMPDIR:-/tmp}/nightly-issue.XXXXXX")
err_file=$(mktemp "${TMPDIR:-/tmp}/nightly-err.XXXXXX")
trap 'rm -f "$body_file" "$err_file"' EXIT

# Every caller below captures a function's output with `$(...)`, where bash
# clears errexit, so each capture tests its own status: a refusal printed in
# the subshell then ends the script instead of reading as an empty answer.

now() {
    date -u +%s
}

# Seconds since the epoch for an ISO 8601 UTC timestamp.
epoch_of() {
    local e
    e=$(jq -rn --arg t "$1" '$t | fromdateiso8601' 2>/dev/null) || die "the timestamp does not parse: '$1'" 2
    printf '%s\n' "$e"
}

# `gh api` with three attempts, 10 s apart: a poll that lasts hours must not end
# on one server error. Prints the answer on stdout; after the third failure it
# returns 1 with the error in err_file.
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

# The ids of the bot-dispatched runs of workflow $1 on REF, newest first.
bot_runs() {
    local page
    page=$(api "repos/${repository}/actions/workflows/$1/runs?event=workflow_dispatch&branch=${REF}&per_page=20") || exit 2
    jq -r --arg bot "$BOT" '.workflow_runs[]
        | select(.actor.login == $bot or .triggering_actor.login == $bot) | .id' <<< "$page"
}

# The one bot-dispatched run of workflow $1 whose id is not in the list $2.
find_new_run() {
    local wf=$1 before=$2 try=1 listed fresh count
    while :; do
        listed=$(bot_runs "$wf") || exit 2
        # Through the environment: awk refuses a `-v` value that holds a newline.
        fresh=$(BEFORE=$before awk '
            BEGIN { n = split(ENVIRON["BEFORE"], b, "\n"); for (i = 1; i <= n; i++) seen[b[i]] = 1 }
            $0 != "" && !($0 in seen)' <<< "$listed")
        count=$(printf '%s' "$fresh" | grep -c . || true)
        if [ "$count" -eq 1 ]; then
            printf '%s\n' "$fresh"
            return 0
        fi
        [ "$count" -eq 0 ] ||
            die "${count} bot-dispatched runs of ${wf} appeared after one dispatch ($(printf '%s' "$fresh" | tr '\n' ' ')): which one is the nightly's cannot be told" 2
        [ "$try" -lt "$FIND_TRIES" ] ||
            die "the dispatched run of ${wf} did not appear within $((FIND_TRIES * FIND_SECONDS)) s" 2
        try=$((try + 1))
        sleep "$FIND_SECONDS"
    done
}

# The jobs of run $1, attempt $2, that did not pass, one `name: conclusion` per
# line. A skipped or neutral job did not fail.
failed_jobs() {
    local page
    page=$(api "repos/${repository}/actions/runs/$1/attempts/$2/jobs?per_page=${MAX_JOBS}") || exit 2
    jq -r --argjson max "$MAX_JOBS" '
        (.jobs[] | select(.conclusion != null and .conclusion != "success"
            and .conclusion != "skipped" and .conclusion != "neutral")
         | "`\(.name)`: \(.conclusion)"),
        (if .total_count > $max then "the run has \(.total_count) jobs; this lists the first \($max)" else empty end)' <<< "$page"
}

# The one open issue carrying TITLE, as `number<TAB>updatedAt`, or nothing.
open_issue() {
    local listing matches='' count=0 found='' number title updated
    if ! listing=$(gh issue list --state open --label "$LABEL" --limit 200 \
        --json number,title,updatedAt --jq '.[] | [.number, .title, .updatedAt] | @tsv' 2> "$err_file"); then
        die "gh refused the issue listing: $(tr '\n' ' ' < "$err_file")" 2
    fi
    while IFS=$'\t' read -r number title updated; do
        [ -n "${number:-}" ] || continue
        [ "${title:-}" = "$TITLE" ] || continue
        matches="${matches}${matches:+ }${number}"
        count=$((count + 1))
        found="${number}"$'\t'"${updated:-}"
    done <<< "$listing"
    [ "$count" -le 1 ] || die "issues ${matches} all carry the title \"${TITLE}\": one issue carries this result" 2
    [ "$count" -eq 0 ] || printf '%s\n' "$found"
}

# Write body_file to the open issue $1, or open the issue when $1 is empty.
write_issue() {
    if [ -z "$1" ]; then
        gh issue create --title "$TITLE" --label "$LABEL" --body-file "$body_file" > /dev/null
        say 'opened the issue'
    else
        gh issue comment "$1" --body-file "$body_file" > /dev/null
        say "commented on issue $1"
    fi
}

# Per dispatched run, by index: workflow, run id, the attempt waited for,
# whether it finished, its conclusion, URL and commit, and the report fields.
wfs=()
ids=()
want=()
finished=()
concl=()
url=()
sha=()
first=()
failed1=()
rerun=()
failed2=()
started=0

# Whether run $1 reads as rerun: a rerun request that the server took but
# answered with an error still starts attempt 2. The refusal stays in err_file.
rerun_started() {
    local json attempt saved
    saved=$(cat "$err_file")
    json=$(api_try "repos/${repository}/actions/runs/$1") || {
        printf '%s\n' "$saved" > "$err_file"
        return 1
    }
    printf '%s\n' "$saved" > "$err_file"
    attempt=$(jq -r '.run_attempt // 1' <<< "$json")
    [ "$attempt" -ge 2 ]
}

past_deadline() {
    [ $(($(now) - started)) -ge "$DEADLINE_SECONDS" ]
}

# Wait until every run reports `completed` on the attempt it is waiting for, or
# the deadline passes. A rerun reads `completed` on the old attempt for a
# moment after it is requested, which is why the attempt number is read too.
wait_all() {
    local i json status attempt all_done
    while :; do
        all_done=1
        for i in "${!ids[@]}"; do
            [ "${finished[i]}" -eq 0 ] || continue
            json=$(api "repos/${repository}/actions/runs/${ids[i]}")
            status=$(jq -r '.status' <<< "$json")
            attempt=$(jq -r '.run_attempt' <<< "$json")
            url[i]=$(jq -r '.html_url' <<< "$json")
            sha[i]=$(jq -r '.head_sha' <<< "$json")
            if [ "$status" = completed ] && [ "$attempt" -ge "${want[i]}" ]; then
                finished[i]=1
                concl[i]=$(jq -r '.conclusion // "none"' <<< "$json")
            else
                all_done=0
            fi
        done
        [ "$all_done" -eq 0 ] || return 0
        if past_deadline; then
            for i in "${!ids[@]}"; do
                [ "${finished[i]}" -eq 1 ] || concl[i]="not finished after $((DEADLINE_SECONDS / 60)) min"
            done
            return 0
        fi
        sleep "$POLL_SECONDS"
    done
}

cmd_run() {
    local wf before answer id i red=0 reran=0 issue number
    started=$(now)
    for wf in $WORKFLOWS; do
        before=$(bot_runs "$wf")
        answer=$(api -X POST "repos/${repository}/actions/workflows/${wf}/dispatches" -f "ref=${REF}")
        # An empty answer (204) or one carrying no run id falls back to the listing.
        id=$(jq -r '.workflow_run_id // empty' <<< "$answer" 2> /dev/null || true)
        [ -n "$id" ] || id=$(find_new_run "$wf" "$before")
        say "dispatched ${wf} on ${REF}: run ${id}"
        wfs+=("$wf")
        ids+=("$id")
        want+=(1)
        finished+=(0)
        concl+=('')
        url+=('')
        sha+=('')
        first+=('')
        failed1+=('')
        rerun+=('')
        failed2+=('')
    done

    wait_all
    for i in "${!ids[@]}"; do
        first[i]=${concl[i]}
        [ "${first[i]}" != success ] || continue
        red=1
        # A run still going at the deadline has no failed jobs to rerun.
        [ "${finished[i]}" -eq 1 ] || continue
        failed1[i]=$(failed_jobs "${ids[i]}" 1)
        if past_deadline; then
            rerun[i]="not requested: past the $((DEADLINE_SECONDS / 60))-minute deadline"
        elif api_try -X POST "repos/${repository}/actions/runs/${ids[i]}/rerun-failed-jobs" > /dev/null \
            || rerun_started "${ids[i]}"; then
            rerun[i]=pending
            want[i]=2
            finished[i]=0
            reran=1
        else
            rerun[i]="refused ($(tr '\n' ' ' < "$err_file" | sed 's/ *$//'))"
        fi
    done
    wait_all
    for i in "${!ids[@]}"; do
        [ "${rerun[i]}" = pending ] || continue
        rerun[i]=${concl[i]}
        [ "${finished[i]}" -eq 1 ] || continue
        [ "${rerun[i]}" = success ] || failed2[i]=$(failed_jobs "${ids[i]}" 2)
    done

    issue=$(open_issue)
    number=${issue%%$'\t'*}

    if [ "$red" -eq 0 ]; then
        if [ -z "$number" ]; then
            say 'every run passed on its first attempt and no issue is open; nothing to write'
            return 0
        fi
        {
            printf 'The nightly full run of `%s` passed on its first attempt.\n\n' "$REF"
            for i in "${!ids[@]}"; do
                printf -- '- `%s` at `%s`: %s\n' "${wfs[i]}" "${sha[i]}" "${url[i]}"
            done
            printf '\nNightly: %s\n' "$self_url"
        } > "$body_file"
        gh issue comment "$number" --body-file "$body_file" > /dev/null
        gh issue close "$number" > /dev/null
        say "every run passed; closed issue ${number}"
        return 0
    fi

    {
        printf 'The nightly full run of `%s` did not pass.\n\n' "$REF"
        for i in "${!ids[@]}"; do
            printf -- '- `%s` at `%s`: %s\n' "${wfs[i]}" "${sha[i]}" "${url[i]}"
            printf '  - first attempt: %s\n' "${first[i]}"
            if [ -n "${failed1[i]}" ]; then
                printf '%s\n' "${failed1[i]}" | sed 's/^/    - /'
            fi
            if [ -n "${rerun[i]}" ]; then
                printf '  - rerun of the failed jobs: %s\n' "${rerun[i]}"
            fi
            if [ -n "${failed2[i]}" ]; then
                printf '%s\n' "${failed2[i]}" | sed 's/^/    - /'
            fi
        done
        if [ "$reran" -eq 1 ]; then
            printf '\nA failed step skips every later step of its job, so the failed jobs ran once more. '
            printf 'A job that passed on that rerun failed on the first attempt only.\n'
        fi
        printf '\nNightly: %s\n' "$self_url"
    } > "$body_file"
    write_issue "$number"
}

cmd_alarm() {
    local wf state newest success newest_status newest_url success_url t since issue number updated finding=''
    wf=$(api "repos/${repository}/actions/workflows/${NIGHTLY_WORKFLOW}")
    newest=$(api "repos/${repository}/actions/workflows/${NIGHTLY_WORKFLOW}/runs?branch=${REF}&per_page=1")
    success=$(api "repos/${repository}/actions/workflows/${NIGHTLY_WORKFLOW}/runs?branch=${REF}&status=success&per_page=1")
    t=$(now)

    state=$(jq -r '.state' <<< "$wf")
    newest_status=$(jq -r '.workflow_runs[0].status // empty' <<< "$newest")
    newest_url=$(jq -r '.workflow_runs[0].html_url // empty' <<< "$newest")
    success_url=$(jq -r '.workflow_runs[0].html_url // empty' <<< "$success")

    if [ "$state" != active ]; then
        finding="\`${NIGHTLY_WORKFLOW}\` is \`${state}\`, so no nightly full run starts."
    elif [ -n "$newest_status" ] && [ "$newest_status" != completed ]; then
        say "a nightly run is ${newest_status}: ${newest_url}; nothing to write"
        return 0
    elif [ -z "$success_url" ]; then
        since=$(epoch_of "$(jq -r '.created_at' <<< "$wf")")
        if [ $((t - since)) -gt "$STALE_SECONDS" ]; then
            finding="\`${NIGHTLY_WORKFLOW}\` was added more than $((STALE_SECONDS / 3600)) hours ago and no run of it on \`${REF}\` has reached a result."
        fi
    else
        since=$(epoch_of "$(jq -r '.workflow_runs[0].updated_at' <<< "$success")")
        if [ $((t - since)) -gt "$STALE_SECONDS" ]; then
            finding="No nightly full run on \`${REF}\` reached a result in the last $((STALE_SECONDS / 3600)) hours. The newest result: ${success_url}"
        fi
    fi
    if [ -n "$finding" ] && [ -n "$newest_url" ] && [ "$newest_url" != "$success_url" ]; then
        finding="${finding} The newest run ended \`$(jq -r '.workflow_runs[0].conclusion // "none"' <<< "$newest")\`: ${newest_url}"
    fi

    if [ -z "$finding" ]; then
        say 'the nightly full run is on schedule; nothing to write'
        return 0
    fi
    say "$finding"

    issue=$(open_issue)
    number=${issue%%$'\t'*}
    if [ -n "$number" ]; then
        updated=$(epoch_of "${issue#*$'\t'}")
        if [ $((t - updated)) -lt "$REPEAT_SECONDS" ]; then
            say "issue ${number} was updated in the last $((REPEAT_SECONDS / 3600)) hours; nothing to write"
            return 0
        fi
    fi
    {
        printf '%s\n\n' "$finding"
        printf 'Alarm: %s\n' "$self_url"
    } > "$body_file"
    write_issue "$number"
}

case $mode in
    run) cmd_run ;;
    alarm) cmd_alarm ;;
esac
