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
# `actions/cache/save`, so the namespace holds at most the entry the job is
# about to write.
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
# understand.
#
# A KEEP KEY MAY BE UNSCOPED. The push-only namespaces (`cargo-fuzz-Linux-<lock
# hash>`, miri, msrv, cross-aarch64, release) carry no `(main|pr)` segment
# because those jobs never run on a pull request. The scoped shape is tried
# first, so a namespace whose own name contains `-main-` still derives
# correctly; then `<prefix->(64 hex)`. The candidate shape pins the namespace
# either way (`<prefix>foo-<hash>` is a sibling and is ignored). A key with no
# 64-hex hash at all (the machete binary cache) is refused (exit 2): it has no
# lockfile generation to prune by, and its save step carries no prune step.
#
# Environment: GH_TOKEN (the workflow step sets it from `github.token`) and
# GITHUB_REPOSITORY (Actions sets it). Uses `gh` and `jq`, both preinstalled on
# hosted runners.
#
# Exit 0 pruned, 1 an API failure, 2 a usage or key-shape failure.
#
# `--self-test` drives the whole script against a fake `gh` on PATH and asserts
# the exact delete set and exit code of seven cases, each with a passing
# near-miss beside the failing one.
set -euo pipefail

SELF=$(cd "$(dirname "$0")" >/dev/null 2>&1 && pwd)/$(basename "$0")

# A keep key is `<prefix->(main|pr)-<64 hex>` or, for a push-only namespace,
# `<prefix->(64 hex)`. The scoped shape is tried first; its leading `.+-` is
# greedy, so group 1 is the longest prefix and a namespace that itself contains
# `-main-` still derives correctly.
KEEP_KEY_RE='^(.+-)(main|pr)-[0-9a-f]{64}$'
KEEP_KEY_UNSCOPED_RE='^(.+-)[0-9a-f]{64}$'

err() { printf '::error::%s\n' "$*" >&2; }
note() { printf '::notice::%s\n' "$*"; }

# Bytes to one decimal place of MB, rounded, without bc or awk.
mb() {
    local tenths
    tenths=$(( ($1 * 10 + 524288) / 1048576 ))
    printf '%d.%d' "$((tenths / 10))" "$((tenths % 10))"
}

# Escape every character that is not alphanumeric, `_` or `-` so the prefix is
# matched literally inside an extended regular expression.
ere_quote() {
    printf '%s' "$1" | sed 's/[^A-Za-z0-9_-]/\\&/g'
}

gh_cache_list() {
    gh cache list -R "$GITHUB_REPOSITORY" --key "$1" -L 100 --json id,key,ref,sizeInBytes
}

prune() {
    local keep_key="$1" explicit_prefix="${2:-}"
    local prefix esc cand_re json tsv

    if [[ $keep_key =~ $KEEP_KEY_RE ]]; then
        prefix="${BASH_REMATCH[1]}"
    elif [[ $keep_key =~ $KEEP_KEY_UNSCOPED_RE ]]; then
        prefix="${BASH_REMATCH[1]}"
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

    esc=$(ere_quote "$prefix")
    if [ -n "$explicit_prefix" ]; then
        cand_re="^${esc}([0-9a-f]+-)?((main|pr)-)?[0-9a-f]{64}$"
    else
        cand_re="^${esc}((main|pr)-)?[0-9a-f]{64}$"
    fi

    if ! json=$(gh_cache_list "$prefix" 2>&1); then
        err "cache prune: listing $prefix failed: $json"
        return 1
    fi
    if ! tsv=$(printf '%s' "$json" | jq -r '.[] | [(.id|tostring), .key, .ref, (.sizeInBytes|tostring)] | @tsv' 2>&1); then
        err "cache prune: could not read the cache listing for $prefix: $tsv"
        return 1
    fi

    local cand_ids=() cand_keys=() cand_refs=() cand_sizes=()
    local n_cand=0 ignored=0
    local id key ref size
    while IFS=$'\t' read -r id key ref size; do
        [ -n "$id" ] || continue
        case "$key" in
            "$prefix"*) ;;
            *) continue ;;
        esac
        if ! [[ $key =~ $cand_re ]]; then
            ignored=$((ignored + 1))
            continue
        fi
        [ "$key" != "$keep_key" ] || continue
        cand_ids[n_cand]="$id"
        cand_keys[n_cand]="$key"
        cand_refs[n_cand]="$ref"
        cand_sizes[n_cand]="${size:-0}"
        n_cand=$((n_cand + 1))
    done <<< "$tsv"

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
        still=$(printf '%s' "$relist" | jq -r --arg id "$id" '.[] | select((.id|tostring) == $id) | .id' 2>/dev/null || true)
        if [ -z "$still" ]; then
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
    printf '%s\n' "$summary"
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
# can assert the exact delete set, and can be told to fail one delete.
set -eu
log="$FAKE_GH_LOG"
state="$FAKE_GH_STATE"
sub=""
[ "${1:-}" = "cache" ] && sub="${2:-}"
case "$sub" in
    list)
        printf 'LIST\n' >> "$log"
        if [ -f "$state/gone" ] && [ -n "${FAKE_GH_RELIST_JSON:-}" ]; then
            cat "$FAKE_GH_RELIST_JSON"
        else
            cat "$FAKE_GH_LIST_JSON"
        fi
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

# st_run <case> <list-json-file> <relist-json-file|-> <fail-id|-> <fail-mode|-> <args...>
# Sets ST_RC, ST_OUT and ST_LOG for the assertions that follow.
st_run() {
    local name="$1" list_json="$2" relist_json="$3" fail_id="$4" fail_mode="$5"
    shift 5
    rm -rf "$ST_DIR/state"
    mkdir -p "$ST_DIR/state"
    : > "$ST_DIR/gh.log"
    ST_CASES=$((ST_CASES + 1))
    ST_RC=0
    ST_OUT=$(
        PATH="$ST_DIR/bin:$PATH" \
        GITHUB_REPOSITORY="owner/repo" \
        GH_TOKEN="fake-token" \
        FAKE_GH_LOG="$ST_DIR/gh.log" \
        FAKE_GH_STATE="$ST_DIR/state" \
        FAKE_GH_LIST_JSON="$list_json" \
        FAKE_GH_RELIST_JSON="$([ "$relist_json" = "-" ] || printf '%s' "$relist_json")" \
        FAKE_GH_FAIL_ID="$([ "$fail_id" = "-" ] || printf '%s' "$fail_id")" \
        FAKE_GH_FAIL_MODE="$([ "$fail_mode" = "-" ] || printf '%s' "$fail_mode")" \
        bash "$SELF" "$@" 2>&1
    ) || ST_RC=$?
    ST_LOG=$(cat "$ST_DIR/gh.log")
    ST_NAME="$name"
}

st_expect_rc() {
    [ "$ST_RC" = "$1" ] || st_fail "$ST_NAME" "exit $ST_RC, wanted $1; output: $ST_OUT"
}

st_expect_log() {
    [ "$ST_LOG" = "$1" ] || st_fail "$ST_NAME" "gh log was [$ST_LOG], wanted [$1]"
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

self_test() {
    ST_DIR=$(mktemp -d "${TMPDIR:-/tmp}/ci_cache_prune_selftest.XXXXXX")
    trap st_cleanup EXIT
    mkdir -p "$ST_DIR/bin"
    st_write_fake_gh

    local H1 H2 H3 H4 H5 H6 H63
    H1=$(h64 11111111); H2=$(h64 22222222); H3=$(h64 33333333)
    H4=$(h64 44444444); H5=$(h64 55555555); H6=$(h64 66666666)
    H63="${H5:0:63}"

    local P="cargo-crates-Linux-"
    local KEEP="${P}main-${H1}"

    # ---- (a) the full mixture: three stale, two ignored, one duplicate keep --
    cat > "$ST_DIR/a.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152},
  {"id": 3, "key": "${P}pr-${H3}", "ref": "refs/pull/9/merge", "sizeInBytes": 3145728},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304},
  {"id": 5, "key": "${P}foo-main-${H5}", "ref": "refs/heads/main", "sizeInBytes": 5242880},
  {"id": 6, "key": "${P}main-${H63}", "ref": "refs/heads/main", "sizeInBytes": 6291456},
  {"id": 7, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576}
]
JSON
    st_run a "$ST_DIR/a.json" - - - "$KEEP"
    st_expect_rc 0
    st_expect_log 'LIST
OK:2
OK:3
OK:4'
    st_expect_contains "cache prune: kept ${KEEP}; deleted 3 entries, 9.0 MB; ignored 2 other keys under ${P}"
    st_expect_contains "cache prune: deleted ${P}pr-${H3} (refs/pull/9/merge, 3.0 MB)"
    st_expect_absent "${P}foo-main-${H5}"
    # Near-miss beside it: the sibling IS a well-formed keep key, and pruning
    # for it derives the narrower `${P}foo-` prefix, so it touches nothing in
    # the namespace above. A prune step can only ever reach its own namespace.
    st_run a-nearmiss "$ST_DIR/a.json" - - - "${P}foo-main-${H5}"
    st_expect_rc 0
    st_expect_log 'LIST'
    st_expect_contains "deleted 0 entries, 0.0 MB; ignored 0 other keys under ${P}foo-"

    # ---- (b) keep key without a 64-hex hash: exit 2, gh never invoked --------
    st_run b "$ST_DIR/a.json" - - - "cargo-crates-Linux-deadbeef"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: keep key carries no 64-hex lockfile hash: cargo-crates-Linux-deadbeef"
    # Passing near-miss: 64 hex with a scope is accepted.
    st_run b-nearmiss "$ST_DIR/a.json" - - - "$KEEP"
    st_expect_rc 0

    # ---- (c) explicit prefix that is not a leading substring ----------------
    st_run c "$ST_DIR/a.json" - - - "$KEEP" "cargo-other-"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: prefix cargo-other- is not a leading substring of ${KEEP}"
    # Passing near-miss: a genuine leading substring is accepted and widens.
    st_run c-nearmiss "$ST_DIR/a.json" - - - "$KEEP" "cargo-crates-"
    st_expect_rc 0

    # ---- (d) empty listing ---------------------------------------------------
    printf '[]\n' > "$ST_DIR/d.json"
    st_run d "$ST_DIR/d.json" - - - "$KEEP"
    st_expect_rc 0
    st_expect_log 'LIST'
    st_expect_contains "cache prune: kept ${KEEP}; deleted 0 entries, 0.0 MB; ignored 0 other keys under ${P}"

    # ---- (e) a delete loses a race: 404-like, gone on the re-list -----------
    cat > "$ST_DIR/e.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576},
  {"id": 2, "key": "${P}main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152},
  {"id": 3, "key": "${P}pr-${H3}", "ref": "refs/pull/9/merge", "sizeInBytes": 3145728},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304}
]
JSON
    cat > "$ST_DIR/e-relist.json" <<JSON
[
  {"id": 1, "key": "${P}main-${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576},
  {"id": 4, "key": "${P}${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304}
]
JSON
    st_run e "$ST_DIR/e.json" "$ST_DIR/e-relist.json" 3 404 "$KEEP"
    st_expect_rc 0
    st_expect_log 'LIST
OK:2
FAIL:3
LIST
OK:4'
    st_expect_contains "::notice::cache prune: ${P}pr-${H3} vanished before delete (concurrent run)"
    st_expect_contains "deleted 2 entries, 6.0 MB; ignored 0 other keys under ${P}; 1 vanished concurrently"

    # ---- (f) a delete fails for real: 500-like, still listed ----------------
    st_run f "$ST_DIR/e.json" "$ST_DIR/e-relist.json" 3 500 "$KEEP"
    st_expect_rc 1
    st_expect_log 'LIST
OK:2
FAIL:3
LIST'
    st_expect_contains "::error::cache prune: deleting ${P}pr-${H3} (3) failed and it is still listed"
    st_expect_absent "OK:4"

    # ---- (g) explicit rmw prefix prunes across header generations -----------
    local RP="cargo-rmw-distros-jazzy-"
    local RKEEP="${RP}aaaaaaaaaaaaaaaa-main-${H1}"
    cat > "$ST_DIR/g.json" <<JSON
[
  {"id": 1, "key": "${RKEEP}", "ref": "refs/heads/main", "sizeInBytes": 1048576},
  {"id": 2, "key": "${RP}bbbbbbbbbbbbbbbb-main-${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152},
  {"id": 3, "key": "${RP}bbbbbbbbbbbbbbbb-pr-${H3}", "ref": "refs/pull/4/merge", "sizeInBytes": 1048576},
  {"id": 4, "key": "${RP}extra-main-${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304},
  {"id": 5, "key": "cargo-rmw-distros-humble-cccccccccccccccc-main-${H5}", "ref": "refs/heads/main", "sizeInBytes": 5242880},
  {"id": 6, "key": "${RP}aaaaaaaaaaaaaaaa-${H6}", "ref": "refs/heads/main", "sizeInBytes": 3145728}
]
JSON
    st_run g "$ST_DIR/g.json" - - - "$RKEEP" "$RP"
    st_expect_rc 0
    st_expect_log 'LIST
OK:2
OK:3
OK:6'
    st_expect_contains "cache prune: kept ${RKEEP}; deleted 3 entries, 6.0 MB; ignored 1 other keys under ${RP}"
    st_expect_absent "cargo-rmw-distros-humble-"
    # Failing mutant beside it: WITHOUT the explicit prefix the derived prefix
    # pins the header generation, so the other generation is never reclaimed.
    st_run g-derived "$ST_DIR/g.json" - - - "$RKEEP"
    st_expect_rc 0
    st_expect_log 'LIST
OK:6'
    st_expect_contains "deleted 1 entries, 3.0 MB"

    # ---- (h) an unscoped keep key: the push-only namespaces -----------------
    # `cargo-fuzz-Linux-<hash>` carries no scope by design. The prefix derives
    # from the hash boundary, a scoped entry under the same prefix is still a
    # generation to prune, and a sibling namespace is still ignored.
    local UP="cargo-fuzz-Linux-"
    local UKEEP="${UP}${H1}"
    cat > "$ST_DIR/h.json" <<JSON
[
  {"id": 1, "key": "${UP}${H1}", "ref": "refs/heads/main", "sizeInBytes": 1048576},
  {"id": 2, "key": "${UP}${H2}", "ref": "refs/heads/main", "sizeInBytes": 2097152},
  {"id": 3, "key": "${UP}main-${H3}", "ref": "refs/heads/main", "sizeInBytes": 3145728},
  {"id": 4, "key": "${UP}foo-${H4}", "ref": "refs/heads/main", "sizeInBytes": 4194304},
  {"id": 5, "key": "${UP}pr-${H63}", "ref": "refs/pull/9/merge", "sizeInBytes": 5242880}
]
JSON
    st_run h "$ST_DIR/h.json" - - - "$UKEEP"
    st_expect_rc 0
    st_expect_log 'LIST
OK:2
OK:3'
    st_expect_contains "cache prune: kept ${UKEEP}; deleted 2 entries, 5.0 MB; ignored 2 other keys under ${UP}"
    st_expect_absent "${UP}foo-${H4}"
    # Failing partner: no 64-hex hash at all (the machete binary key shape).
    st_run h-fail "$ST_DIR/h.json" - - - "cargo-machete-bin-Linux-v0.9.2"
    st_expect_rc 2
    st_expect_log ''
    st_expect_contains "::error::cache prune: keep key carries no 64-hex lockfile hash: cargo-machete-bin-Linux-v0.9.2"

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
