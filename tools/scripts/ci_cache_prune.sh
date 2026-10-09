#!/usr/bin/env bash
# ci_cache_prune.sh — keep exactly one cache entry per namespace, delete the rest.
#
#   bash tools/scripts/ci_cache_prune.sh <keep-key> [namespace-prefix]
#   bash tools/scripts/ci_cache_prune.sh --self-test
#
# WHY THIS EXISTS. The repository's GitHub Actions cache store has a 10 GB free
# allowance, and above it saves are refused while the account carries a failed
# payment. A cargo cache for one job is a few GB, so a handful of namespaces
# each holding a `main` entry, a stale `main` entry from the previous lockfile,
# and one `pr` entry per open branch reaches the allowance in a day. The
# workflows run this script in the step DIRECTLY BEFORE every
# `actions/cache/save` of a lockfile-keyed archive, so the namespace holds at
# most the entry the job is about to write.
#
# THE PRUNE RUNS WIDER THAN THE SAVE, deliberately. A save is gated on its
# namespace being named in `CACHE_SAVE_NAMESPACES`; the prune is not. Every
# default-branch run of a job that has a prune step prunes, and keeps exactly
# that job's current key even when no entry under that key exists — so a
# namespace that is restore-only under the policy is EMPTIED rather than left
# holding archives nothing will ever replace. Pull-request restores refresh an
# entry's last-access time, so GitHub's seven-day idle eviction never fires on
# a stale archive a pull request keeps reading; this script is the only thing
# that reclaims it.
#
# WHAT IT DELETES, and why the shape test is narrow. `gh cache list --key` is a
# PREFIX filter, so a listing for `cargo-crates-Linux-` also returns
# `cargo-crates-Linux-foo-main-<hash>`, which belongs to a different job. An
# entry is a candidate only when the text after the prefix is a bare 64-hex
# lockfile hash, optionally preceded by `main-` or `pr-`. That covers the three
# things a namespace accumulates — stale `main` generations, every `pr` scoped
# entry, and unqualified keys written before the scope was introduced — and
# nothing else. Anything under the prefix that does not have that shape is
# counted and left alone: a prune step must never be the reason another job's
# cache disappears.
#
# THE TIP CHECK (default-branch runs only). Two runs on `main` overlap all the
# time: run A for an older commit is still building when run B for a newer one
# prunes and saves, and A's prune would then delete the generation B just
# wrote — with A skipping its own save as an exact hit, so nothing replaces it.
# Neither "created after my run started" nor "created after my prune began"
# fixes that on its own, because a RERUN of an older push run starts after the
# newer generation was saved. So on `refs/heads/main` the script reads the
# branch tip once and deletes nothing unless the tip is this run's own commit.
# Only the run for the newest commit ever deletes; the others still save, and
# the next tip run prunes what they left.
#
# THE CLOCK GUARD (every run). The prune records the UTC second it began and
# deletes only entries created STRICTLY EARLIER than that. An entry that
# appeared while the prune was running belongs to a run that is ahead of this
# one, so it is kept and named in the log. An entry whose `createdAt` cannot be
# read as a timestamp stops the run: pruning blind is worse than not pruning.
#
# THE INSTANT IS TAKEN FIRST, before the tip check and before the listing. The
# tip check is a network round trip, and a newer run that saves DURING it would
# otherwise carry a `createdAt` earlier than the recorded instant and be deleted
# as stale -- the exact race the tip check exists to close, reopened one call
# later. Everything the guard can misjudge is on the safe side of that ordering:
# it keeps too much, never too little.
#
# WHAT THE STEP TELLS THE SAVE. On the paths that exit 0 the script writes
# `proceed=true` or `proceed=false` to `$GITHUB_OUTPUT`, and every save step
# tests `steps.cache-prune.outputs.proceed == 'true'`. `false` is written on
# exactly one path: a default-branch run that is behind the tip, which deletes
# nothing. Without that test such a run would skip the prune and still UPLOAD,
# putting a second generation of its namespace in the store beside the tip's --
# 14.90 GB for the two shard namespaces, over the allowance, and the next save
# refused. A run behind the tip now deletes nothing and saves nothing. A failing
# exit writes no output at all, and the job is red on the step itself.
#
# THE SCOPE RULE. A keep key scoped `-pr-` may delete only `-pr-` scoped
# entries of its namespace. A pull-request run (which only prunes at all when
# `CACHE_SAVE_ON_PULL_REQUEST` is set) must never delete the `-main-` archive
# or the legacy unqualified entries: those are what every other run restores,
# and it is about to write a key that no other ref can read. A keep key scoped
# `-main-`, or unscoped (the push-only namespaces), deletes every shape-
# matching entry of the namespace, which is the whole point of the prune.
#
# REF IS PART OF THE IDENTITY. GitHub stores one entry per (key, ref), so with
# `CACHE_SAVE_ON_PULL_REQUEST` set every open pull request holds its own entry
# under the same `-pr-` key string. The entry this run is about to write is the
# one whose key AND ref match; a same-key entry on another ref is a candidate
# like any other (within the scope rule above). A merge-queue ref never reaches
# this script — the prune and save gates both exclude `merge_group` — so no
# queue-branch entry can be the one this run would keep.
#
# THE EXPLICIT PREFIX. `cargo-rmw-distros-<distro>-<header hash>-<scope>-<lock
# hash>` carries a second hash, so the prefix derived from the keep key pins one
# header generation and the previous generation is never reclaimed. Passing
# `cargo-rmw-distros-<distro>-` as the second argument widens the sweep across
# generations. It must be a leading substring of the keep key, so the widest a
# caller can reach is its own key's namespace.
#
# JUDGMENT CALL, recorded because the specification did not settle it: with an
# explicit prefix the candidate shape additionally allows ONE leading hex
# generation segment (`<prefix><hex>-[main-|pr-]<64 hex>`), because otherwise the
# keep key's own generation segment would put every rmw entry outside the
# candidate shape and the explicit prefix would prune nothing. The segment must
# be hex, so `cargo-rmw-distros-jazzy-extra-main-<hash>` is still ignored. With a
# derived prefix the strict shape applies unchanged.
#
# CONCURRENCY. Two jobs in the same namespace can prune at the same moment, so a
# delete losing the race is expected, not an error. A failed delete is retried
# as a question: re-list once, and if the id is gone the other runner took it.
# A delete that fails while the entry is still listed is a real failure and
# stops the run, because continuing would delete more under an error we do not
# understand. A re-list that itself fails, or whose body cannot be read, is
# also a real failure: "I could not ask" is not "it vanished".
#
# THE LISTING IS BOUNDED. `gh cache list` is asked for at most 1000 entries and
# the script refuses to continue if it gets exactly that many, because a
# truncated listing would silently under-prune a namespace that has run away.
#
# A KEEP KEY MAY BE UNSCOPED. The push-only namespaces (`cargo-fuzz-Linux-<lock
# hash>`, miri, cross-aarch64, release) carry no `(main|pr)` segment
# because those jobs never run on a pull request. The scoped shape is tried
# first, so a namespace whose own name contains `-main-` still derives
# correctly; then `<prefix->(64 hex)`. The candidate shape pins the namespace
# either way (`<prefix>foo-<hash>` is a sibling and is ignored). A key with no
# 64-hex hash at all (the machete binary cache) is refused (exit 2): it has no
# lockfile generation to prune by, and its save step carries no prune step.
#
# Environment: GH_TOKEN (the workflow step sets it from `github.token`), and
# GITHUB_REPOSITORY, GITHUB_REF, GITHUB_SHA and GITHUB_OUTPUT (Actions sets all
# four). A missing one is exit 2, never a prune that guesses or a `proceed` the
# save step will read as empty and fail closed on in silence. Uses `gh` and
# `jq`, both preinstalled on hosted runner images; a job running in a
# `container:` installs them in the step before the prune.
#
# Exit 0 pruned (or skipped because this run is not the tip), 1 an API failure,
# 2 a usage or key-shape failure.
#
# `--self-test` drives the whole script against a fake `gh` on PATH and asserts
# the exact delete set, the exact `gh` invocations (including the listing's
# `--key` prefix and `-L` limit), the `$GITHUB_OUTPUT` line and the exit code of
# nineteen lettered cases, (a) to (s). Every case that proves a rule is stated twice: the failing side
# and a passing near-miss one step away from it, so a rule that stopped holding
# cannot pass as a rule that never fired. The case count is printed. Entries
# staged for the clock guard carry far-past (2020) and far-future (2999)
# `createdAt` values, which is how the "created while I was running" ordering is
# staged without a sleep.
set -euo pipefail

SELF=$(cd "$(dirname "$0")" >/dev/null 2>&1 && pwd)/$(basename "$0")

# A keep key is `<prefix->(main|pr)-<64 hex>` or, for a push-only namespace,
# `<prefix->(64 hex)`. The scoped shape is tried first; its leading `.+-` is
# greedy, so group 1 is the longest prefix and a namespace that itself contains
# `-main-` still derives correctly.
KEEP_KEY_RE='^(.+-)(main|pr)-[0-9a-f]{64}$'
KEEP_KEY_UNSCOPED_RE='^(.+-)[0-9a-f]{64}$'

# One listing, bounded. 1000 is far above any namespace this repository can
# hold (one entry per lockfile generation per ref); reaching it means the
# listing was truncated, and a truncated listing under-prunes silently.
LIST_LIMIT=1000

err() { printf '::error::%s\n' "$*" >&2; }
note() { printf '::notice::%s\n' "$*"; }

# Bytes to one decimal place of MB, rounded, without bc or awk.
mb() {
    local tenths
    tenths=$(( ($1 * 10 + 524288) / 1048576 ))
    printf '%d.%d' "$((tenths / 10))" "$((tenths % 10))"
}

sha8() { printf '%.8s' "$1"; }

# The one thing this step tells the save step after it. Called on the two paths
# that exit 0 and on neither failing path.
emit_proceed() {
    printf 'proceed=%s\n' "$1" >> "$GITHUB_OUTPUT"
}

# Escape every character that is not alphanumeric, `_` or `-` so the prefix is
# matched literally inside an extended regular expression.
ere_quote() {
    printf '%s' "$1" | sed 's/[^A-Za-z0-9_-]/\\&/g'
}

# An RFC 3339 instant as the 14 digits YYYYMMDDHHMMSS, or the empty string if
# the text does not carry that many. Comparing two of these as integers orders
# them; comparing them as text would not, once a fractional second appears.
ts_digits() {
    local digits
    digits=$(printf '%s' "$1" | tr -cd '0-9')
    [ "${#digits}" -ge 14 ] || { printf ''; return 0; }
    printf '%s' "${digits:0:14}"
}

gh_cache_list() {
    gh cache list -R "$GITHUB_REPOSITORY" --key "$1" -L "$LIST_LIMIT" \
        --json id,key,ref,sizeInBytes,createdAt
}

prune() {
    local keep_key="$1" explicit_prefix="${2:-}"
    local prefix scope esc shape_re cand_re json tsv tip started

    if [[ $keep_key =~ $KEEP_KEY_RE ]]; then
        prefix="${BASH_REMATCH[1]}"
        scope="${BASH_REMATCH[2]}"
    elif [[ $keep_key =~ $KEEP_KEY_UNSCOPED_RE ]]; then
        prefix="${BASH_REMATCH[1]}"
        scope=""
    else
        err "cache prune: keep key carries no 64-hex lockfile hash: $keep_key"
        return 2
    fi

    if [ -n "$explicit_prefix" ]; then
        case "$keep_key" in
            "$explicit_prefix"*) ;;
            *)
                err "cache prune: prefix $explicit_prefix is not a leading substring of $keep_key"
                return 2
                ;;
        esac
        prefix="$explicit_prefix"
    fi

    if [ -z "${GITHUB_REPOSITORY:-}" ]; then
        err "cache prune: GITHUB_REPOSITORY is unset"
        return 2
    fi
    if [ -z "${GITHUB_REF:-}" ]; then
        err "cache prune: GITHUB_REF is unset; the prune needs the ref to know which entry is its own"
        return 2
    fi
    if [ -z "${GITHUB_SHA:-}" ]; then
        err "cache prune: GITHUB_SHA is unset; the prune needs the commit to compare against the tip"
        return 2
    fi
    if [ -z "${GITHUB_OUTPUT:-}" ]; then
        err "cache prune: GITHUB_OUTPUT is unset; the save step reads proceed from it and an unwritten value would skip every save in silence"
        return 2
    fi

    # BEFORE the tip check, not after: see THE INSTANT IS TAKEN FIRST above.
    started=$(date -u +%Y%m%d%H%M%S)

    esc=$(ere_quote "$prefix")
    if [ -n "$explicit_prefix" ]; then
        shape_re="^${esc}([0-9a-f]+-)?((main|pr)-)?[0-9a-f]{64}$"
        cand_re="^${esc}([0-9a-f]+-)?pr-[0-9a-f]{64}$"
    else
        shape_re="^${esc}((main|pr)-)?[0-9a-f]{64}$"
        cand_re="^${esc}pr-[0-9a-f]{64}$"
    fi
    # The scope rule: only a `-pr-` keep narrows the candidate set.
    if [ "$scope" != "pr" ]; then
        cand_re="$shape_re"
    fi

    # The tip check. A pull-request run never asks: it is not competing with
    # another ref for the `main` generation, and the scope rule already keeps it
    # inside its own `-pr-` entries.
    if [ "$GITHUB_REF" = "refs/heads/main" ]; then
        if ! tip=$(gh api "repos/$GITHUB_REPOSITORY/branches/main" --jq .commit.sha 2>&1); then
            err "cache prune: reading the default-branch tip failed: $tip"
            return 1
        fi
        tip=$(printf '%s' "$tip" | tr -d '[:space:]')
        if [ "$tip" != "$GITHUB_SHA" ]; then
            printf "cache prune: skipped, this run's commit %s is not the current default-branch tip %s\n" \
                "$(sha8 "$GITHUB_SHA")" "$(sha8 "$tip")"
            # The save step is gated on this: a run behind the tip deletes
            # nothing, so it must not upload a second generation either.
            emit_proceed false
            return 0
        fi
    fi

    if ! json=$(gh_cache_list "$prefix" 2>&1); then
        err "cache prune: listing $prefix failed: $json"
        return 1
    fi
    if ! tsv=$(printf '%s' "$json" | jq -r '.[] | [(.id|tostring), .key, .ref, (.sizeInBytes|tostring), .createdAt] | @tsv' 2>&1); then
        err "cache prune: could not read the cache listing for $prefix: $tsv"
        return 1
    fi

    local cand_ids=() cand_keys=() cand_refs=() cand_sizes=()
    local n_cand=0 ignored=0 out_of_scope=0 newer=0 n_rows=0
    local id key ref size created created_digits
    while IFS=$'\t' read -r id key ref size created; do
        [ -n "$id" ] || continue
        n_rows=$((n_rows + 1))
        case "$key" in
            "$prefix"*) ;;
            *) continue ;;
        esac
        if ! [[ $key =~ $shape_re ]]; then
            ignored=$((ignored + 1))
            continue
        fi
        if [ "$key" = "$keep_key" ] && [ "$ref" = "$GITHUB_REF" ]; then
            continue
        fi
        if ! [[ $key =~ $cand_re ]]; then
            printf 'cache prune: %s (%s) kept by the pull-request scope rule\n' "$key" "$ref"
            out_of_scope=$((out_of_scope + 1))
            continue
        fi
        created_digits=$(ts_digits "$created")
        if [ -z "$created_digits" ]; then
            err "cache prune: entry $key ($id) has an unreadable createdAt: ${created:-<empty>}"
            return 1
        fi
        if [ "$((10#$created_digits))" -ge "$((10#$started))" ]; then
            printf 'cache prune: %s (%s) kept (created after this prune began)\n' "$key" "$ref"
            newer=$((newer + 1))
            continue
        fi
        cand_ids[n_cand]="$id"
        cand_keys[n_cand]="$key"
        cand_refs[n_cand]="$ref"
        cand_sizes[n_cand]="${size:-0}"
        n_cand=$((n_cand + 1))
    done <<< "$tsv"

    if [ "$n_rows" -ge "$LIST_LIMIT" ]; then
        err "cache prune: the listing for $prefix returned $n_rows entries, the full limit of $LIST_LIMIT; it is truncated, so this namespace cannot be pruned safely"
        return 1
    fi

    local i=0 deleted=0 deleted_bytes=0 vanished=0 out relist still
    while [ "$i" -lt "$n_cand" ]; do
        id="${cand_ids[$i]}"
        key="${cand_keys[$i]}"
        ref="${cand_refs[$i]}"
        size="${cand_sizes[$i]}"
        i=$((i + 1))
        if out=$(gh cache delete "$id" -R "$GITHUB_REPOSITORY" 2>&1); then
            printf 'cache prune: deleted %s (%s, %s MB)\n' "$key" "$ref" "$(mb "$size")"
            deleted=$((deleted + 1))
            deleted_bytes=$((deleted_bytes + size))
            continue
        fi
        # Ask once whether the entry is simply gone: a peer job in the same
        # namespace pruning at the same moment is expected, not a failure.
        if ! relist=$(gh_cache_list "$prefix" 2>&1); then
            err "cache prune: deleting $key ($id) failed and the re-list failed too: $out"
            return 1
        fi
        if ! still=$(printf '%s' "$relist" | jq -r --arg id "$id" '[.[] | select((.id|tostring) == $id)] | length' 2>&1); then
            err "cache prune: deleting $key ($id) failed and the re-list could not be read ($still); refusing to treat that as vanished: $out"
            return 1
        fi
        if [ "$still" = "0" ]; then
            note "cache prune: $key vanished before delete (concurrent run)"
            vanished=$((vanished + 1))
            continue
        fi
        err "cache prune: deleting $key ($id) failed and it is still listed: $out"
        return 1
    done

    local summary
    summary=$(printf 'cache prune: kept %s; deleted %d entries, %s MB; ignored %d other keys under %s' \
        "$keep_key" "$deleted" "$(mb "$deleted_bytes")" "$ignored" "$prefix")
    if [ "$vanished" -gt 0 ]; then
        summary="$summary; $vanished vanished concurrently"
    fi
    if [ "$newer" -gt 0 ]; then
        summary="$summary; $newer kept (created after this prune began)"
    fi
    if [ "$out_of_scope" -gt 0 ]; then
        summary="$summary; $out_of_scope kept by the pull-request scope rule"
    fi
    printf '%s\n' "$summary"
    emit_proceed true
    return 0
}

# ----------------------------------------------------------------------------
# self-test
# ----------------------------------------------------------------------------

ST_DIR=""
ST_FAILS=0
ST_CASES=0

st_cleanup() { [ -z "$ST_DIR" ] || rm -rf "$ST_DIR"; }

st_fail() {
    printf 'ci_cache_prune: self-test FAILED: case %s: %s\n' "$1" "$2" >&2
    ST_FAILS=$((ST_FAILS + 1))
}

# 64 hex characters from an 8-character seed.
h64() { printf '%s' "$1$1$1$1$1$1$1$1"; }

st_write_fake_gh() {
    cat > "$ST_DIR/bin/gh" <<'FAKE'
#!/usr/bin/env bash
# Fake `gh` for ci_cache_prune.sh --self-test. Logs every invocation so a case
# can assert the exact API calls and delete set, HONOURS the listing's `--key`
# prefix filter and `-L` limit (a mutant that widens either is then visible in
# the log and in what comes back), and can be told to fail one delete.
set -eu
log="$FAKE_GH_LOG"
state="$FAKE_GH_STATE"

if [ "${1:-}" = "api" ]; then
    printf 'API:%s\n' "${2:-}" >> "$log"
    : > "$state/api"
    if [ -z "${FAKE_GH_TIP:-}" ]; then
        printf 'fake gh: no branch tip staged for this case\n' >&2
        exit 1
    fi
    printf '%s\n' "$FAKE_GH_TIP"
    exit 0
fi

sub=""
[ "${1:-}" = "cache" ] && sub="${2:-}"
case "$sub" in
    list)
        shift 2
        key=""
        limit=""
        while [ $# -gt 0 ]; do
            case "$1" in
                --key) key="${2:-}"; shift 2 ;;
                -L) limit="${2:-}"; shift 2 ;;
                -R | --json) shift 2 ;;
                *) shift ;;
            esac
        done
        printf 'LIST key=%s limit=%s\n' "$key" "$limit" >> "$log"
        if [ -f "$state/gone" ] && [ -n "${FAKE_GH_RELIST_RAW:-}" ]; then
            cat "$FAKE_GH_RELIST_RAW"
            exit 0
        fi
        src="$FAKE_GH_LIST_JSON"
        if [ -f "$state/gone" ] && [ -n "${FAKE_GH_RELIST_JSON:-}" ]; then
            src="$FAKE_GH_RELIST_JSON"
        fi
        jq --arg k "$key" --argjson l "$limit" \
           '[.[] | select(.key | startswith($k))] | .[0:$l]' "$src"
        ;;
    delete)
        id="${3:-}"
        if [ -n "${FAKE_GH_FAIL_ID:-}" ] && [ "$id" = "$FAKE_GH_FAIL_ID" ]; then
            printf 'FAIL:%s\n' "$id" >> "$log"
            if [ "${FAKE_GH_FAIL_MODE:-404}" = "404" ]; then
                : > "$state/gone"
                printf 'HTTP 404: Not Found (cache %s)\n' "$id" >&2
            else
                printf 'HTTP 500: Internal Server Error (cache %s)\n' "$id" >&2
            fi
            exit 1
        fi
        printf 'OK:%s\n' "$id" >> "$log"
        printf 'Deleted 1 cache entry\n'
        ;;
    *)
        printf 'fake gh: unsupported invocation: %s\n' "$*" >&2
        exit 64
        ;;
esac
FAKE
    chmod +x "$ST_DIR/bin/gh"
}

st_write_fake_date() {
    cat > "$ST_DIR/bin/date" <<'FAKEDATE'
#!/usr/bin/env bash
# Fake `date` for the ordering case. It answers with the EARLY instant while the
# tip check has not run and with the LATE one afterwards, so a case can stage an
# entry created BETWEEN the two and assert which side of it the prune's recorded
# instant fell on -- the ordering, with no sleep and no clock injected into the
# script. Unstaged (FAKE_DATE_EARLY empty) it is the real `date`.
set -eu
if [ -z "${FAKE_DATE_EARLY:-}" ]; then
    exec /bin/date "$@"
fi
if [ -f "$FAKE_GH_STATE/api" ]; then
    printf '%s\n' "$FAKE_DATE_LATE"
else
    printf '%s\n' "$FAKE_DATE_EARLY"
fi
FAKEDATE
    chmod +x "$ST_DIR/bin/date"
}

# The run environment of one case. `st_reset` puts every knob back to the
# ordinary shape — a default-branch run that IS the tip — so a case states only
# what it changes and no setting can leak from the case above it.
st_reset() {
    ST_LIST=""
    ST_RELIST="-"
    ST_RELIST_RAW="-"
    ST_FAIL_ID="-"
    ST_FAIL_MODE="-"
    ST_REF="refs/heads/main"
    ST_SHA="$ST_SHA_TIP"
    ST_TIP="$ST_SHA_TIP"
    ST_DATE_EARLY="-"
    ST_DATE_LATE="-"
    ST_GHOUT="$ST_DIR/gh_output"
}

# st_run <case> <args...>. Reads the ST_* knobs above; sets ST_RC, ST_OUT and
# ST_LOG for the assertions that follow. An empty ST_REF or ST_SHA is passed
# through as an empty value, which is what the script's own guard tests.
st_run() {
    local name="$1"
    shift
    rm -rf "$ST_DIR/state"
    mkdir -p "$ST_DIR/state"
    : > "$ST_DIR/gh.log"
    : > "$ST_DIR/gh_output"
    ST_CASES=$((ST_CASES + 1))
    ST_RC=0
    ST_OUT=$(
        PATH="$ST_DIR/bin:$PATH" \
        GITHUB_REPOSITORY="owner/repo" \
        GITHUB_REF="$ST_REF" \
        GITHUB_SHA="$ST_SHA" \
        GH_TOKEN="fake-token" \
        FAKE_GH_LOG="$ST_DIR/gh.log" \
        FAKE_GH_STATE="$ST_DIR/state" \
        GITHUB_OUTPUT="$ST_GHOUT" \
        FAKE_GH_TIP="$ST_TIP" \
        FAKE_DATE_EARLY="$([ "$ST_DATE_EARLY" = "-" ] || printf '%s' "$ST_DATE_EARLY")" \
        FAKE_DATE_LATE="$([ "$ST_DATE_LATE" = "-" ] || printf '%s' "$ST_DATE_LATE")" \
        FAKE_GH_LIST_JSON="$ST_LIST" \
        FAKE_GH_RELIST_JSON="$([ "$ST_RELIST" = "-" ] || printf '%s' "$ST_RELIST")" \
        FAKE_GH_RELIST_RAW="$([ "$ST_RELIST_RAW" = "-" ] || printf '%s' "$ST_RELIST_RAW")" \
        FAKE_GH_FAIL_ID="$([ "$ST_FAIL_ID" = "-" ] || printf '%s' "$ST_FAIL_ID")" \
        FAKE_GH_FAIL_MODE="$([ "$ST_FAIL_MODE" = "-" ] || printf '%s' "$ST_FAIL_MODE")" \
        bash "$SELF" "$@" 2>&1
    ) || ST_RC=$?
    ST_LOG=$(cat "$ST_DIR/gh.log")
    ST_GHOUT_TEXT=$(cat "$ST_DIR/gh_output")
    ST_NAME="$name"
}

st_expect_rc() {
    [ "$ST_RC" = "$1" ] || st_fail "$ST_NAME" "exit $ST_RC, wanted $1; output: $ST_OUT"
}

st_expect_log() {
    [ "$ST_LOG" = "$1" ] || st_fail "$ST_NAME" "gh log was [$ST_LOG], wanted [$1]"
}

# The whole of what the step handed the save step after it, never a substring.
st_expect_output() {
    [ "$ST_GHOUT_TEXT" = "$1" ] \
        || st_fail "$ST_NAME" "GITHUB_OUTPUT was [$ST_GHOUT_TEXT], wanted [$1]"
}

st_expect_contains() {
    case "$ST_OUT" in
        *"$1"*) ;;
        *) st_fail "$ST_NAME" "output does not contain [$1]; output: $ST_OUT" ;;
    esac
}

st_expect_absent() {
    case "$ST_OUT" in
        *"$1"*) st_fail "$ST_NAME" "output unexpectedly contains [$1]; output: $ST_OUT" ;;
        *) ;;
    esac
}

# A listing of `count` entries that the shape test IGNORES, used to drive the
# listing-limit rule without staging a thousand deletes.
st_gen_ignored_listing() {
    local out="$1" count="$2" prefix="$3" hash="$4" created="$5" i=0
    {
        printf '['
        while [ "$i" -lt "$count" ]; do
            [ "$i" -eq 0 ] || printf ','
            printf '{"id": %d, "key": "%sfoo-main-%s", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "%s"}' \
                "$((i + 1))" "$prefix" "$hash" "$created"
            i=$((i + 1))
        done
        printf ']'
    } > "$out"
}

self_test() {
    ST_DIR=$(mktemp -d "${TMPDIR:-/tmp}/ci_cache_prune_selftest.XXXXXX")
    trap st_cleanup EXIT
    mkdir -p "$ST_DIR/bin"
    st_write_fake_gh
    st_write_fake_date

    local H1 H2 H3 H4 H5 H6 H63
    H1=$(h64 11111111); H2=$(h64 22222222); H3=$(h64 33333333)
    H4=$(h64 44444444); H5=$(h64 55555555); H6=$(h64 66666666)
    H63="${H5:0:63}"

    # Staged instants. The clock guard compares against the second the prune
    # began, so a far-past and a far-future value stage "already there" and
    # "appeared while I was running" with no sleep and no clock injection.
    local PAST="2020-01-01T00:00:00Z"
    local FUTURE="2999-01-01T00:00:00Z"
    local UNREADABLE="whenever"

    ST_SHA_TIP="aaaaaaaabbbbbbbbccccccccdddddddd11111111"
    local OLD_SHA="99999999888888887777777766666666555555ff"
    local API="API:repos/owner/repo/branches/main"

    local P="cargo-crates-Linux-"
    local KEEP="${P}main-${H1}"
    local LIST_P="LIST key=${P} limit=1000"

    # ---- (a) the full mixture: three stale, two ignored, one duplicate keep --
    cat > "$ST_DIR/a.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}pr-${H3}", "ref": "refs/pull/9/merge", "sizeInBytes": 3145728, "createdAt": "${PAST}"},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"},
  {"id": 5, "key": "${P}foo-main-${H5}", "ref": "refs/heads/main", "sizeInBytes": 5242880, "createdAt": "${PAST}"},
  {"id": 6, "key": "${P}main-${H63}", "ref": "refs/heads/main", "sizeInBytes": 6291456, "createdAt": "${PAST}"},
  {"id": 7, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run a "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
OK:3
OK:4"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 3 entries, 9.0 MB; ignored 2 other keys under ${P}"
    st_expect_contains "cache prune: deleted ${P}pr-${H3} (refs/pull/9/merge, 3.0 MB)"
    st_expect_output 'proceed=true'
    st_expect_absent "${P}foo-main-${H5}"
    # Near-miss beside it: the sibling IS a well-formed keep key, and pruning
    # for it derives the narrower `${P}foo-` prefix, so it touches nothing in
    # the namespace above. A prune step can only ever reach its own namespace.
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run a-nearmiss "${P}foo-main-${H5}"
    st_expect_rc 0
    st_expect_log "${API}
LIST key=${P}foo- limit=1000"
    st_expect_contains "deleted 0 entries, 0.0 MB; ignored 0 other keys under ${P}foo-"

    # ---- (b) keep key without a 64-hex hash: exit 2, gh never invoked --------
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run b "cargo-crates-Linux-deadbeef"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: keep key carries no 64-hex lockfile hash: cargo-crates-Linux-deadbeef"
    # Passing near-miss: 64 hex with a scope is accepted.
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run b-nearmiss "$KEEP"
    st_expect_rc 0

    # ---- (c) explicit prefix that is not a leading substring ----------------
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run c "$KEEP" "cargo-other-"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: prefix cargo-other- is not a leading substring of ${KEEP}"
    # Passing near-miss: a genuine leading substring is accepted and widens, and
    # every key in the listing is then outside the candidate shape.
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run c-nearmiss "$KEEP" "cargo-crates-"
    st_expect_rc 0
    st_expect_log "${API}
LIST key=cargo-crates- limit=1000"
    st_expect_contains "deleted 0 entries, 0.0 MB; ignored 7 other keys under cargo-crates-"

    # ---- (d) empty listing ---------------------------------------------------
    printf '[]\n' > "$ST_DIR/d.json"
    st_reset; ST_LIST="$ST_DIR/d.json"
    st_run d "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 0 entries, 0.0 MB; ignored 0 other keys under ${P}"

    # ---- (e) a delete loses a race: 404-like, gone on the re-list -----------
    cat > "$ST_DIR/e.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}pr-${H3}", "ref": "refs/pull/9/merge", "sizeInBytes": 3145728, "createdAt": "${PAST}"},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"}
]
JSON
    cat > "$ST_DIR/e-relist.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/e.json"; ST_RELIST="$ST_DIR/e-relist.json"
    ST_FAIL_ID=3; ST_FAIL_MODE=404
    st_run e "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
FAIL:3
${LIST_P}
OK:4"
    st_expect_contains "::notice::cache prune: ${P}pr-${H3} vanished before delete (concurrent run)"
    st_expect_contains "deleted 2 entries, 6.0 MB; ignored 0 other keys under ${P}; 1 vanished concurrently"

    # ---- (f) a delete fails for real: 500-like, still listed ----------------
    st_reset; ST_LIST="$ST_DIR/e.json"; ST_RELIST="$ST_DIR/e-relist.json"
    ST_FAIL_ID=3; ST_FAIL_MODE=500
    st_run f "$KEEP"
    st_expect_rc 1
    st_expect_log "${API}
${LIST_P}
OK:2
FAIL:3
${LIST_P}"
    st_expect_contains "::error::cache prune: deleting ${P}pr-${H3} (3) failed and it is still listed"
    st_expect_absent "OK:4"
    # A failing exit writes nothing: the job is red on this step, and a stale
    # `proceed` would otherwise decide the save.
    st_expect_output ''

    # ---- (g) explicit rmw prefix prunes across header generations -----------
    local RP="cargo-rmw-distros-jazzy-"
    local RKEEP="${RP}aaaaaaaaaaaaaaaa-main-${H1}"
    cat > "$ST_DIR/g.json" <<JSON
[
  {"id": 1, "key": "${RKEEP}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${RP}bbbbbbbbbbbbbbbb-main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${RP}bbbbbbbbbbbbbbbb-pr-${H3}", "ref": "refs/pull/4/merge", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 4, "key": "${RP}extra-main-${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"},
  {"id": 5, "key": "cargo-rmw-distros-humble-cccccccccccccccc-main-${H5}", "ref": "refs/heads/main", "sizeInBytes": 5242880, "createdAt": "${PAST}"},
  {"id": 6, "key": "${RP}aaaaaaaaaaaaaaaa-${H6}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/g.json"
    st_run g "$RKEEP" "$RP"
    st_expect_rc 0
    st_expect_log "${API}
LIST key=${RP} limit=1000
OK:2
OK:3
OK:6"
    st_expect_contains "cache prune: kept ${RKEEP}; deleted 3 entries, 6.0 MB; ignored 1 other keys under ${RP}"
    st_expect_absent "cargo-rmw-distros-humble-"
    # Failing mutant beside it: WITHOUT the explicit prefix the derived prefix
    # pins the header generation, so the other generation is never reclaimed.
    st_reset; ST_LIST="$ST_DIR/g.json"
    st_run g-derived "$RKEEP"
    st_expect_rc 0
    st_expect_log "${API}
LIST key=${RP}aaaaaaaaaaaaaaaa- limit=1000
OK:6"
    st_expect_contains "deleted 1 entries, 3.0 MB"

    # ---- (h) an unscoped keep key: the push-only namespaces -----------------
    # `cargo-fuzz-Linux-<hash>` carries no scope by design. The prefix derives
    # from the hash boundary, a scoped entry under the same prefix is still a
    # generation to prune, and a sibling namespace is still ignored.
    local UP="cargo-fuzz-Linux-"
    local UKEEP="${UP}${H1}"
    cat > "$ST_DIR/h.json" <<JSON
[
  {"id": 1, "key": "${UP}${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${UP}${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${UP}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${PAST}"},
  {"id": 4, "key": "${UP}foo-${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"},
  {"id": 5, "key": "${UP}pr-${H63}", "ref": "refs/pull/9/merge", "sizeInBytes": 5242880, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/h.json"
    st_run h "$UKEEP"
    st_expect_rc 0
    st_expect_log "${API}
LIST key=${UP} limit=1000
OK:2
OK:3"
    st_expect_contains "cache prune: kept ${UKEEP}; deleted 2 entries, 5.0 MB; ignored 2 other keys under ${UP}"
    st_expect_absent "${UP}foo-${H4}"
    # Failing partner: no 64-hex hash at all (the machete binary key shape).
    st_reset; ST_LIST="$ST_DIR/h.json"
    st_run h-fail "cargo-machete-bin-Linux-v0.9.2"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: keep key carries no 64-hex lockfile hash: cargo-machete-bin-Linux-v0.9.2"

    # ---- (i) TIP CHECK: an older main run deletes nothing --------------------
    # The ordering the bot found: run A (older commit) is still going when run B
    # (newer commit) has pruned and saved. A is not the tip, so A deletes
    # nothing and B's generation survives.
    st_reset; ST_LIST="$ST_DIR/a.json"; ST_SHA="$OLD_SHA"
    st_run i "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}"
    st_expect_contains "cache prune: skipped, this run's commit 99999999 is not the current default-branch tip aaaaaaaa"
    st_expect_absent "deleted"
    st_expect_output 'proceed=false'
    # Passing near-miss: the same run one commit later, now the tip, prunes.
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run i-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
OK:3
OK:4"

    # ---- (j) CLOCK GUARD: an entry created after the prune began is kept -----
    cat > "$ST_DIR/j.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${FUTURE}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/j.json"
    st_run j "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2"
    st_expect_contains "cache prune: ${P}main-${H3} (refs/heads/main) kept (created after this prune began)"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 1 entries, 2.0 MB; ignored 0 other keys under ${P}; 1 kept (created after this prune began)"
    # Failing partner: the same entry with an older createdAt IS deleted, so the
    # guard is what spared it and not the shape test or the scope rule.
    cat > "$ST_DIR/j-old.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/j-old.json"
    st_run j-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
OK:3"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 2 entries, 5.0 MB; ignored 0 other keys under ${P}"

    # ---- (k) the tip run with everything stale deletes all of it -------------
    st_reset; ST_LIST="$ST_DIR/j-old.json"
    st_run k "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
OK:3"
    # Failing partner: every entry newer than the prune, so nothing goes.
    cat > "$ST_DIR/k-future.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${FUTURE}"},
  {"id": 3, "key": "${P}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${FUTURE}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/k-future.json"
    st_run k-future "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 0 entries, 0.0 MB; ignored 0 other keys under ${P}; 2 kept (created after this prune began)"

    # ---- (l) an unreadable createdAt stops the run --------------------------
    cat > "$ST_DIR/l.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${UNREADABLE}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/l.json"
    st_run l "$KEEP"
    st_expect_rc 1
    st_expect_log "${API}
${LIST_P}"
    st_expect_contains "::error::cache prune: entry ${P}main-${H2} (2) has an unreadable createdAt: ${UNREADABLE}"
    # Passing near-miss: the same entry with a readable instant is deleted.
    cat > "$ST_DIR/l-ok.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/l-ok.json"
    st_run l-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2"

    # ---- (m) THE SCOPE RULE: a pull-request prune touches only `pr` entries --
    cat > "$ST_DIR/m.json" <<JSON
[
  {"id": 1, "key": "${P}pr-${H1}", "ref": "refs/pull/9/merge", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304, "createdAt": "${PAST}"},
  {"id": 4, "key": "${P}pr-${H3}", "ref": "refs/pull/7/merge", "sizeInBytes": 3145728, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/m.json"; ST_REF="refs/pull/9/merge"
    st_run m "${P}pr-${H1}"
    st_expect_rc 0
    st_expect_log "${LIST_P}
OK:4"
    st_expect_contains "cache prune: ${P}main-${H2} (refs/heads/main) kept by the pull-request scope rule"
    st_expect_contains "cache prune: ${P}${H4} (refs/heads/main) kept by the pull-request scope rule"
    st_expect_contains "cache prune: kept ${P}pr-${H1}; deleted 1 entries, 3.0 MB; ignored 0 other keys under ${P}; 2 kept by the pull-request scope rule"
    # A pull-request run has no tip check, so it always proceeds.
    st_expect_output 'proceed=true'
    # Failing partner: the same listing under a `main` keep deletes all three,
    # so the scope rule is what spared them.
    st_reset; ST_LIST="$ST_DIR/m.json"
    st_run m-main "${P}main-${H2}"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:1
OK:3
OK:4"
    st_expect_contains "cache prune: kept ${P}main-${H2}; deleted 3 entries, 8.0 MB; ignored 0 other keys under ${P}"

    # ---- (n) the same key on another ref is a different entry ---------------
    cat > "$ST_DIR/n.json" <<JSON
[
  {"id": 1, "key": "${P}pr-${H1}", "ref": "refs/pull/9/merge", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}pr-${H1}", "ref": "refs/pull/8/merge", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${PAST}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/n.json"; ST_REF="refs/pull/9/merge"
    st_run n "${P}pr-${H1}"
    st_expect_rc 0
    st_expect_log "${LIST_P}
OK:2"
    st_expect_contains "cache prune: kept ${P}pr-${H1}; deleted 1 entries, 2.0 MB; ignored 0 other keys under ${P}; 1 kept by the pull-request scope rule"
    # Partner: a main run deletes every `pr` entry whatever its ref.
    st_reset; ST_LIST="$ST_DIR/n.json"
    st_run n-main "${P}main-${H1}"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:1
OK:2"
    st_expect_contains "cache prune: kept ${P}main-${H1}; deleted 2 entries, 3.0 MB; ignored 0 other keys under ${P}"

    # ---- (o) the run context must be present --------------------------------
    st_reset; ST_LIST="$ST_DIR/a.json"; ST_SHA=""
    st_run o "$KEEP"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: GITHUB_SHA is unset; the prune needs the commit to compare against the tip"
    st_reset; ST_LIST="$ST_DIR/a.json"; ST_REF=""
    st_run o-ref "$KEEP"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: GITHUB_REF is unset; the prune needs the ref to know which entry is its own"
    st_reset; ST_LIST="$ST_DIR/a.json"; ST_GHOUT=""
    st_run o-output "$KEEP"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: GITHUB_OUTPUT is unset; the save step reads proceed from it and an unwritten value would skip every save in silence"
    # Passing near-miss: all three present is the ordinary run.
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run o-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_output 'proceed=true'

    # ---- (p) a listing that comes back at the limit is refused --------------
    st_gen_ignored_listing "$ST_DIR/p.json" 1000 "$P" "$H1" "$PAST"
    st_reset; ST_LIST="$ST_DIR/p.json"
    st_run p "$KEEP"
    st_expect_rc 1
    st_expect_log "${API}
${LIST_P}"
    st_expect_contains "::error::cache prune: the listing for ${P} returned 1000 entries, the full limit of 1000; it is truncated, so this namespace cannot be pruned safely"
    # Passing near-miss: one entry fewer is a complete listing.
    st_gen_ignored_listing "$ST_DIR/p-999.json" 999 "$P" "$H1" "$PAST"
    st_reset; ST_LIST="$ST_DIR/p-999.json"
    st_run p-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}"
    st_expect_contains "cache prune: kept ${KEEP}; deleted 0 entries, 0.0 MB; ignored 999 other keys under ${P}"

    # ---- (q) a re-list that cannot be read is a failure, not a vanishing ----
    printf 'gateway timeout, not json\n' > "$ST_DIR/q-relist.txt"
    st_reset; ST_LIST="$ST_DIR/e.json"; ST_RELIST_RAW="$ST_DIR/q-relist.txt"
    ST_FAIL_ID=3; ST_FAIL_MODE=404
    st_run q "$KEEP"
    st_expect_rc 1
    st_expect_log "${API}
${LIST_P}
OK:2
FAIL:3
${LIST_P}"
    st_expect_contains "::error::cache prune: deleting ${P}pr-${H3} (3) failed and the re-list could not be read"
    st_expect_absent "vanished before delete"
    # Passing near-miss: a readable re-list that no longer holds the id is the
    # concurrent-run case and is not fatal.
    st_reset; ST_LIST="$ST_DIR/e.json"; ST_RELIST="$ST_DIR/e-relist.json"
    ST_FAIL_ID=3; ST_FAIL_MODE=404
    st_run q-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_contains "::notice::cache prune: ${P}pr-${H3} vanished before delete (concurrent run)"

    # ---- (r) the instant is recorded BEFORE the tip check -------------------
    # The tip check is a network round trip. A newer run that saves DURING it
    # carries a createdAt later than the moment this prune began but earlier
    # than the moment the tip answer came back, and reading the clock after the
    # check would classify it as stale and delete it -- the race the tip check
    # exists to close, reopened one call later. The fake `date` answers EARLY
    # until the tip check has run and LATE afterwards, so which of the two the
    # prune recorded is visible in what survives.
    local T_EARLY="20260101000000" T_LATE="20260101000200"
    local T_BETWEEN="2026-01-01T00:01:00Z" T_BEFORE="2025-12-31T23:59:00Z"
    cat > "$ST_DIR/r.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${T_BETWEEN}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/r.json"
    ST_DATE_EARLY="$T_EARLY"; ST_DATE_LATE="$T_LATE"
    st_run r "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2"
    st_expect_contains "cache prune: ${P}main-${H3} (refs/heads/main) kept (created after this prune began)"
    st_expect_output 'proceed=true'
    # Passing near-miss one second the other side of the EARLY instant: an entry
    # created before this prune began IS deleted, so it is the recorded instant
    # that spared the one above and not the shape test.
    cat > "$ST_DIR/r-before.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576, "createdAt": "${PAST}"},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152, "createdAt": "${PAST}"},
  {"id": 3, "key": "${P}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728, "createdAt": "${T_BEFORE}"}
]
JSON
    st_reset; ST_LIST="$ST_DIR/r-before.json"
    ST_DATE_EARLY="$T_EARLY"; ST_DATE_LATE="$T_LATE"
    st_run r-nearmiss "$KEEP"
    st_expect_rc 0
    st_expect_log "${API}
${LIST_P}
OK:2
OK:3"

    # ---- (s) what the step hands the save after it --------------------------
    st_reset; ST_LIST="$ST_DIR/a.json"
    st_run s "$KEEP"
    st_expect_rc 0
    st_expect_output 'proceed=true'
    # The failing side of the same rule: a run behind the tip prunes nothing, so
    # it must not upload a second generation of its namespace either.
    st_reset; ST_LIST="$ST_DIR/a.json"; ST_SHA="$OLD_SHA"
    st_run s-behind-tip "$KEEP"
    st_expect_rc 0
    st_expect_output 'proceed=false'

    if [ "$ST_FAILS" -ne 0 ]; then
        printf 'ci_cache_prune: %d self-test case(s) failed\n' "$ST_FAILS" >&2
        exit 1
    fi
    printf 'ci_cache_prune: self-test OK (%d cases)\n' "$ST_CASES"
}

usage() {
    printf 'usage: bash tools/scripts/ci_cache_prune.sh <keep-key> [namespace-prefix]\n' >&2
    printf '       bash tools/scripts/ci_cache_prune.sh --self-test\n' >&2
}

main() {
    case "${1:-}" in
        --self-test)
            [ $# -eq 1 ] || { usage; exit 2; }
            self_test
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        "")
            usage
            exit 2
            ;;
        *)
            [ $# -le 2 ] || { usage; exit 2; }
            local rc=0
            prune "$@" || rc=$?
            exit "$rc"
            ;;
    esac
}

main "$@"
