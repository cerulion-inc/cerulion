#!/usr/bin/env bash
# publish_crates.sh — publish the workspace to crates.io through its
# new-crate rate limit.
#
# WHY. crates.io lets one account publish a burst of NEW crate names, then
# one every ten minutes; past that it answers HTTP 429 naming the instant the
# next slot opens ("Please try again after <RFC 1123 date>"). A first
# release of this 17-crate workspace hits that after 6 crates, and a plain
# `cargo publish --workspace` dies there with 11 crates unpublished.
#
# `cargo publish --workspace` cannot simply be re-run: cargo 1.97 refuses the
# WHOLE invocation as soon as one selected member already exists at that
# version ("crate X@V already exists on crates.io index" — `verify_unpublished`
# in src/cargo/ops/registry/publish.rs bails; only `--dry-run` downgrades it
# to a warning). Verified against a two-crate workspace with one member
# already on crates.io. And the "the following crates have not been published
# yet" note cargo prints on failure iterates a HashMap, so it is not an order
# to publish by hand either.
#
# WHAT IT DOES, per attempt:
#   1. lists the publishable workspace members (`cargo metadata`; members
#      with `publish = false` are ignored, as `--workspace` ignores them);
#   2. asks the sparse index — the source cargo's own duplicate check reads —
#      which already exist at their version, and hands those to cargo as
#      `--exclude`, so cargo still decides the publish ORDER and waits for
#      each upload to land on the index before publishing its dependents;
#   3. runs `cargo publish --workspace [--exclude …] <extra args>`.
# On a crates.io rate limit it sleeps until the instant crates.io named plus
# SETTLE_SECONDS (FALLBACK_WAIT_SECONDS when that instant does not parse) and
# starts the next attempt from step 1, so the retry covers only what is still
# missing. Any other failure exits at once with cargo's own status; so does
# an exhausted attempt or wall-clock bound.
#
# A run whose every member is already published exits 0 without calling
# cargo, so re-running the release job after a partial publish is safe. The
# one residual: an upload crates.io accepted seconds before the 429 may not
# be on the index yet when the retry probes (the wait is minutes in
# practice); then cargo refuses with "already exists", this script exits with
# that status, and re-running the job later resumes.
#
# USAGE
#   publish_crates.sh [<extra cargo publish args>...]
#
# Portable: bash 3.2+ (macOS), GNU and BSD `date`; needs cargo, curl and jq.

set -euo pipefail

MAX_ATTEMPTS=8

# DEADLINE_SECONDS is this script's OWN retry budget: the wall-clock span over
# which it keeps waiting out crates.io's new-crate rate limit before giving up
# cleanly (re-running the job then resumes with only the crates still missing —
# see header). It is deliberately a BOUNDED, named value and must stay <= the
# `publish` job's `timeout-minutes` in .github/workflows/release.yml (120 min),
# so the script exits with cargo's status and a clear message instead of being
# SIGKILLed mid-attempt. The bound is checked at the START of every attempt
# (not only before a sleep), so no fresh, unbounded `cargo publish` is launched
# once the budget is spent.
#
# CROSS-WORKFLOW. release-artifacts.yml runs on the same tag and, in its smoke
# job, waits on crates.io for only 2700s (45 min) with a 45-min job timeout —
# SHORTER than this 90-min budget, so a heavily rate-limited publish can finish
# after that smoke has already failed. That is tolerable and loses no release:
# the GitHub Release BINARIES are uploaded by release-artifacts.yml's own
# `release` job independently of crates.io, and the APT publish
# gate keys on that `release` job — so a slow crates.io publish delays only the
# crates.io-dependent smoke step, never the binaries. The complementary knob
# (raising the smoke's crates.io wait and its job timeout to cover this budget)
# lives in release-artifacts.yml and is NOT edited here.
DEADLINE_SECONDS=$((90 * 60))
SETTLE_SECONDS=10         # margin past the instant crates.io names
FALLBACK_WAIT_SECONDS=300 # when that instant cannot be parsed
INDEX_URL='https://index.crates.io'

say() { printf 'publish_crates: %s\n' "$*"; }
die() {
    printf 'publish_crates: error: %s\n' "$*" >&2
    exit 1
}

for tool in cargo curl jq; do
    command -v "$tool" >/dev/null 2>&1 || die "$tool is required"
done

workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-publish-crates.XXXXXX")
trap 'rm -rf "$workdir"' EXIT

# index_path <lowercase name> — the sparse-index layout (the same one
# wait_for_crates_io.sh walks): 1/a, 2/ab, 3/a/abc, ab/cd/abcdef.
index_path() {
    case ${#1} in
        1) printf '1/%s' "$1" ;;
        2) printf '2/%s' "$1" ;;
        3) printf '3/%s/%s' "${1:0:1}" "$1" ;;
        *) printf '%s/%s/%s' "${1:0:2}" "${1:2:2}" "$1" ;;
    esac
}

# is_published <name> <version> — 0 when the index lists that exact version
# (yanked or not: neither can be uploaded again), 1 when the crate or the
# version is absent. Anything but 200/404 is fatal: a guess either way ends
# in a wrong `--exclude` list, which cargo then refuses.
is_published() {
    local name=$1 version=$2 body="$workdir/index" lower code
    lower=$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]')
    code=$(curl -sSL -o "$body" -w '%{http_code}' --max-time 30 \
        "$INDEX_URL/$(index_path "$lower")" </dev/null) ||
        die "cannot reach the crates.io index for $name"
    case $code in
        200) grep -Fq "\"vers\":\"$version\"" "$body" ;;
        404) return 1 ;;
        *) die "crates.io index answered HTTP $code for $name" ;;
    esac
}

# rfc1123_to_epoch <"Wed, 09 Sep 2026 06:26:24 GMT"> — seconds since the
# epoch, on GNU date (the release runner) and BSD date (macOS) alike. GNU is
# told apart by `--version`, which BSD date does not have, rather than by
# letting one form fail into the other. LC_ALL=C pins the English day and
# month names crates.io writes.
rfc1123_to_epoch() {
    local epoch
    if date --version >/dev/null 2>&1; then
        epoch=$(LC_ALL=C date -u -d "$1" +%s 2>/dev/null) || return 1
    else
        epoch=$(LC_ALL=C date -j -u -f '%a, %d %b %Y %H:%M:%S %Z' "$1" +%s 2>/dev/null) ||
            return 1
    fi
    case $epoch in
        '' | *[!0-9]*) return 1 ;;
    esac
    printf '%s' "$epoch"
}

is_rate_limited() {
    grep -qi -e 'status 429' -e 'too many new crates' "$1"
}

# retry_after_instant <cargo log> — the RFC 1123 date after "try again
# after", or nothing.
retry_after_instant() {
    sed -n 's/.*[Tt]ry again after \([A-Z][a-z][a-z], [0-9][0-9]* [A-Z][a-z][a-z] [0-9][0-9][0-9][0-9] [0-9][0-9]:[0-9][0-9]:[0-9][0-9] GMT\).*/\1/p' \
        "$1" | head -n 1
}

# `--no-deps` lists exactly the workspace members; `publish` is `[]` for
# `publish = false`, null or a registry list otherwise.
members=$(cargo metadata --no-deps --format-version 1 |
    jq -r '.packages[] | select(.publish != []) | "\(.name) \(.version)"')
[ -n "$members" ] || die "cargo metadata listed no publishable workspace member"
total=$(printf '%s\n' "$members" | grep -c .)

# Nothing publishes until every package listing is clean (source, manifest,
# README, license and notice files only): a stray log or note in a crate
# directory would otherwise ship to the registry, where nothing is ever deleted.
member_names=()
while IFS= read -r line; do member_names+=("${line%% *}"); done <<<"$members"
"$(dirname "$0")/publish_preflight.sh" "${member_names[@]}" || die "publish preflight refused; see the lines above"

start=$(date +%s)
attempt=0
status=1
while :; do
    attempt=$((attempt + 1))

    # Bound the wall clock at the START of every attempt, before launching
    # cargo: never begin a fresh, unbounded `cargo publish` once the retry
    # budget is spent. The pre-sleep check below refuses only a sleep that would
    # END past the deadline and, being `-gt`, still accepts one ending EXACTLY
    # on it; the publish that would then follow is itself unbounded, so without
    # this a sleep landing on the deadline would be chased by another full
    # publish that runs past the budget. When no budget remains we stop here
    # with cargo's last exit code (the job timeout is the hard outer bound).
    now=$(date +%s)
    if [ $((now - start)) -ge "$DEADLINE_SECONDS" ]; then
        say "the $((DEADLINE_SECONDS / 60))-minute publish deadline is spent; not starting attempt $attempt; giving up"
        exit "$status"
    fi

    excludes=()
    pending=0
    while read -r name version; do
        if is_published "$name" "$version"; then
            excludes+=(--exclude "$name")
        else
            pending=$((pending + 1))
        fi
    done <<<"$members"
    if [ "$pending" -eq 0 ]; then
        say "all $total publishable crates are already on crates.io at their version; nothing to publish"
        exit 0
    fi
    say "attempt $attempt of $MAX_ATTEMPTS: publishing $pending of $total crates ($((total - pending)) already on crates.io)"

    # The is_published probes above each allow up to `curl --max-time 30`, so a
    # sweep across many members can itself consume minutes. Re-check the budget
    # here, immediately before launching cargo, so a slow probe sweep cannot let
    # a fresh unbounded publish start after the deadline the top-of-attempt
    # guard already honoured.
    now=$(date +%s)
    if [ $((now - start)) -ge "$DEADLINE_SECONDS" ]; then
        say "the $((DEADLINE_SECONDS / 60))-minute publish deadline elapsed during pre-publish probes; not starting attempt $attempt; giving up"
        exit "$status"
    fi

    log="$workdir/attempt-$attempt.log"
    set +e
    cargo publish --workspace ${excludes[@]+"${excludes[@]}"} "$@" 2>&1 | tee "$log" >&2
    status=${PIPESTATUS[0]}
    set -e
    if [ "$status" -eq 0 ]; then
        say "attempt $attempt succeeded"
        exit 0
    fi
    if ! is_rate_limited "$log"; then
        say "attempt $attempt failed (exit $status) for a reason other than a crates.io rate limit; giving up"
        exit "$status"
    fi
    if [ "$attempt" -ge "$MAX_ATTEMPTS" ]; then
        say "attempt $attempt failed on a crates.io rate limit; attempt limit $MAX_ATTEMPTS reached; giving up"
        exit "$status"
    fi

    now=$(date +%s)
    instant=$(retry_after_instant "$log")
    if [ -n "$instant" ] && target=$(rfc1123_to_epoch "$instant"); then
        wait_seconds=$((target + SETTLE_SECONDS - now))
        [ "$wait_seconds" -ge "$SETTLE_SECONDS" ] || wait_seconds=$SETTLE_SECONDS
        reason="crates.io says try again after $instant"
    else
        wait_seconds=$FALLBACK_WAIT_SECONDS
        reason="crates.io named no parseable retry instant"
    fi
    if [ $((now + wait_seconds - start)) -gt "$DEADLINE_SECONDS" ]; then
        say "attempt $attempt failed on a crates.io rate limit; waiting ${wait_seconds}s would pass the $((DEADLINE_SECONDS / 60))-minute deadline; giving up"
        exit "$status"
    fi
    say "attempt $attempt failed on a crates.io rate limit ($reason); retrying in ${wait_seconds}s"
    sleep "$wait_seconds"
done
