#!/usr/bin/env bash
# test_publish_crates.sh — the oracle table for scripts/publish_crates.sh.
#
# `cargo`, `curl` and `sleep` are PATH shims: the fake cargo answers
# `metadata` with a fixed workspace and consumes one scripted outcome per
# `publish` call (recording its arguments); the fake curl answers the
# sparse-index probe from a fixture of published "<name> <version>" pairs;
# the fake sleep records its argument instead of sleeping. jq and date are
# real. One case per behaviour; the first failing assertion prints one
# `FAIL:` line and exits 1. Portable: bash 3.2+, GNU and BSD userland.

set -euo pipefail

script_dir=$(CDPATH='' cd "$(dirname "$0")" && pwd)
workdir=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-publish-crates-test.XXXXXX")
cleanup() {
    rm -rf "$workdir"
}
trap cleanup EXIT

fail() {
    printf 'FAIL: %s\n' "$1" >&2
    if [ -n "${output:-}" ]; then
        printf '%s\n' "--- output ---" "$output" >&2
    fi
    exit 1
}

shim_bin="$workdir/bin"
mkdir -p "$shim_bin"

cat > "$workdir/metadata.json" <<'JSON'
{"packages":[
 {"name":"cerulion_macros","version":"0.1.0","publish":null},
 {"name":"cerulion_core","version":"0.1.0","publish":null},
 {"name":"native_ros2_messages","version":"0.1.0","publish":null},
 {"name":"cerulion_bag","version":"0.1.0","publish":["crates-io"]},
 {"name":"cerulion_connectd","version":"0.1.0","publish":[]}
]}
JSON

# A workspace with no publishable member, for the empty-metadata die arm.
# Every fake package points at one manifest that carries an include list, which
# the publish preflight requires before anything may be published.
printf '[package]\nname = "fake"\ninclude = ["src/**"]\n' > "$workdir/Cargo.toml"
sed -i.bak "s#\"publish\"#\"manifest_path\":\"$workdir/Cargo.toml\",\"publish\"#" "$workdir/metadata.json" && rm -f "$workdir/metadata.json.bak"

cat > "$workdir/metadata_empty.json" <<'JSON'
{"packages":[]}
JSON

# Plan lines: `ok||`, `fail||`, or `429|<names published by this call>|<text
# after "short period of time. ">` — the 429 body mirrors what cargo 1.97
# printed on release run 34316282377, with only the trailing sentence varied.
# `429s` and `429t` carry the two rate-limit SIGNALS in isolation: `429s` a
# body with 'status 429' but not 'too many new crates', `429t` the reverse,
# so a case can prove is_rate_limited()'s two -e patterns independently.
cat > "$shim_bin/cargo" <<'SHIM'
#!/bin/sh
case $1 in
    metadata) cat "$FAKE_METADATA"; exit 0 ;;
    # The preflight lists the package; every fake crate is clean.
    # Listed as real cargo lists a package from a git checkout: its own three
    # additions first, so the preflight is exercised on the files cargo adds.
    package) printf '%s\n' .cargo_vcs_info.json Cargo.lock Cargo.toml Cargo.toml.orig README.md src/lib.rs; exit 0 ;;
    publish) ;;
    *) printf 'fake cargo: unexpected subcommand: %s\n' "$*" >&2; exit 99 ;;
esac
printf '%s\n' "$*" >> "$FAKE_CALLS"
outcome=$(head -n 1 "$FAKE_PLAN")
[ -n "$outcome" ] || { printf 'fake cargo: plan exhausted\n' >&2; exit 99; }
tail -n +2 "$FAKE_PLAN" > "$FAKE_PLAN.next" && mv "$FAKE_PLAN.next" "$FAKE_PLAN"
kind=${outcome%%|*}
rest=${outcome#*|}
for name in ${rest%%|*}; do
    printf '%s 0.1.0\n' "$name" >> "$FAKE_PUBLISHED"
done
case $kind in
    ok)
        printf '   Uploading fake v0.1.0\n' >&2
        exit 0
        ;;
    429)
        printf '%s\n' \
            'error: failed to publish to registry at https://crates.io' \
            '' \
            'Caused by:' \
            "  the remote server responded with an error (status 429 Too Many Requests): You have published too many new crates in a short period of time. ${rest#*|} or email help@crates.io to have your limit increased." \
            '' \
            'note: the following crates have not been published yet:' \
            '  cerulion_bag v0.1.0' >&2
        exit 101
        ;;
    429s)
        # 'status 429' present, 'too many new crates' ABSENT.
        printf '%s\n' \
            'error: failed to publish to registry at https://crates.io' \
            '' \
            'Caused by:' \
            "  the remote server responded with an error (status 429 Too Many Requests): rate limited. ${rest#*|}" >&2
        exit 101
        ;;
    429t)
        # 'too many new crates' present, no literal 'status 429'.
        printf '%s\n' \
            'error: failed to publish to registry at https://crates.io' \
            '' \
            'Caused by:' \
            "  the remote server rejected the upload: you have published too many new crates in a short period of time. ${rest#*|}" >&2
        exit 101
        ;;
    fail)
        printf 'error: failed to verify package tarball\n' >&2
        exit 101
        ;;
    *)
        printf 'fake cargo: bad plan line: %s\n' "$outcome" >&2
        exit 99
        ;;
esac
SHIM

# The sparse index, keyed by the last URL segment: one index line per
# fixture pair whose name matches, HTTP 404 when none does, or the HTTP
# status in FAKE_INDEX_HTTP to fake an outage.
cat > "$shim_bin/curl" <<'SHIM'
#!/bin/sh
if [ -n "${FAKE_CURL_FAIL:-}" ]; then
    printf 'fake curl: forced transport failure\n' >&2
    exit 7
fi
out=
url=
while [ "$#" -gt 0 ]; do
    case $1 in
        -o) out=$2; shift 2 ;;
        -w | --max-time) shift 2 ;;
        http://* | https://*) url=$1; shift ;;
        *) shift ;;
    esac
done
if [ -z "$out" ] || [ -z "$url" ]; then printf 'fake curl: bad invocation\n' >&2; exit 99; fi
printf '%s\n' "$url" >> "$FAKE_URLS"
if [ -n "${FAKE_INDEX_HTTP:-}" ]; then
    : > "$out"
    printf '%s' "$FAKE_INDEX_HTTP"
    exit 0
fi
name=${url##*/}
: > "$out"
while read -r fixture_name fixture_version; do
    [ "$fixture_name" = "$name" ] || continue
    printf '{"name":"%s","vers":"%s","deps":[],"cksum":"0","features":{},"yanked":false}\n' \
        "$name" "$fixture_version" >> "$out"
done < "$FAKE_PUBLISHED"
if [ -s "$out" ]; then
    printf 200
else
    printf 'Not Found' > "$out"
    printf 404
fi
SHIM

cat > "$shim_bin/sleep" <<'SHIM'
#!/bin/sh
printf '%s\n' "$1" >> "$FAKE_SLEEPS"
SHIM
chmod 0755 "$shim_bin/cargo" "$shim_bin/curl" "$shim_bin/sleep"

past='Please try again after Thu, 01 Jan 2026 00:00:00 GMT'

# run_case <name> <plan lines> <published fixture> [extra script args...]
# Leaves $output, $status, and the recorded calls/urls/sleeps in $case_dir.
run_case() {
    case_name=$1
    case_dir="$workdir/$case_name"
    mkdir -p "$case_dir"
    printf '%s\n' "$2" > "$case_dir/plan"
    printf '%s\n' "$3" > "$case_dir/published"
    : > "$case_dir/calls"
    : > "$case_dir/urls"
    : > "$case_dir/sleeps"
    shift 3
    set +e
    output=$(
        PATH="${EXTRA_SHIM_DIR:+$EXTRA_SHIM_DIR:}$shim_bin:$PATH" \
        FAKE_METADATA="${FAKE_METADATA:-$workdir/metadata.json}" \
        FAKE_PLAN="$case_dir/plan" \
        FAKE_PUBLISHED="$case_dir/published" \
        FAKE_CALLS="$case_dir/calls" \
        FAKE_URLS="$case_dir/urls" \
        FAKE_SLEEPS="$case_dir/sleeps" \
        FAKE_INDEX_HTTP="${FAKE_INDEX_HTTP:-}" \
        FAKE_CURL_FAIL="${FAKE_CURL_FAIL:-}" \
        FAKE_CLOCK="${FAKE_CLOCK:-}" \
            "$script_dir/publish_crates.sh" "$@" 2>&1
    )
    status=$?
    set -e
}

expect_status() {
    [ "$status" -eq "$1" ] || fail "$case_name: exit $status, expected $1"
}

expect_output() {
    printf '%s\n' "$output" | grep -Fq -- "$1" ||
        fail "$case_name: output lacks '$1'"
}

# expect_file <what> <file> <expected content, one line per entry>
expect_file() {
    actual=$(cat "$2")
    [ "$actual" = "$3" ] || {
        printf '%s\n' "--- $1 ---" "$actual" >&2
        fail "$case_name: recorded $1 differ from the expected list"
    }
}

# --- a rate limit is waited out, and the retry excludes what landed -------
# Two 429s, each after some crates uploaded; the third call succeeds. The
# published fixture starts with cerulion_core at an OLDER version, which
# must not count as published at 0.1.0. `--locked` proves pass-through.
run_case rate_limited_then_published "429|cerulion_macros cerulion_core|$past
429|native_ros2_messages|$past
ok||" 'cerulion_core 0.0.1-alpha' --locked
expect_status 0
expect_output 'attempt 1 of 8: publishing 4 of 4 crates (0 already on crates.io)'
expect_output "attempt 1 failed on a crates.io rate limit (crates.io says try again after Thu, 01 Jan 2026 00:00:00 GMT); retrying in 10s"
expect_output 'attempt 2 of 8: publishing 2 of 4 crates (2 already on crates.io)'
expect_output 'attempt 3 of 8: publishing 1 of 4 crates (3 already on crates.io)'
expect_output 'attempt 3 succeeded'
expect_file sleeps "$case_dir/sleeps" '10
10'
expect_file 'cargo calls' "$case_dir/calls" 'publish --workspace --locked
publish --workspace --exclude cerulion_macros --exclude cerulion_core --locked
publish --workspace --exclude cerulion_macros --exclude cerulion_core --exclude native_ros2_messages --locked'
# Every attempt probes exactly the four publishable members, at the sparse
# index paths cargo itself reads; `publish = false` is never probed.
expect_file 'index urls' "$case_dir/urls" 'https://index.crates.io/ce/ru/cerulion_macros
https://index.crates.io/ce/ru/cerulion_core
https://index.crates.io/na/ti/native_ros2_messages
https://index.crates.io/ce/ru/cerulion_bag
https://index.crates.io/ce/ru/cerulion_macros
https://index.crates.io/ce/ru/cerulion_core
https://index.crates.io/na/ti/native_ros2_messages
https://index.crates.io/ce/ru/cerulion_bag
https://index.crates.io/ce/ru/cerulion_macros
https://index.crates.io/ce/ru/cerulion_core
https://index.crates.io/na/ti/native_ros2_messages
https://index.crates.io/ce/ru/cerulion_bag'
printf '%s\n' 'rate limit waited out, retry excludes published crates: passed'

# --- the wait runs to the instant crates.io named, plus the margin ---------
future_epoch=$(($(date +%s) + 600))
if date --version >/dev/null 2>&1; then
    future=$(LC_ALL=C date -u -d "@$future_epoch" '+%a, %d %b %Y %H:%M:%S GMT')
else
    future=$(LC_ALL=C date -u -r "$future_epoch" '+%a, %d %b %Y %H:%M:%S GMT')
fi
run_case future_instant "429||Please try again after $future
ok||" ''
expect_status 0
expect_output "crates.io says try again after $future"
slept=$(cat "$case_dir/sleeps")
case $slept in
    '' | *[!0-9]*) fail "$case_name: no single integer sleep recorded: '$slept'" ;;
esac
# 600 s ahead plus the 10 s margin, minus however long the run itself took.
if [ "$slept" -lt 590 ] || [ "$slept" -gt 610 ]; then
    fail "$case_name: slept ${slept}s, expected 590..610"
fi
printf '%s\n' "future retry instant honoured (slept ${slept}s): passed"

# --- an instant that does not parse falls back to a fixed wait -------------
run_case unparseable_instant '429||Please try again after soon
ok||' ''
expect_status 0
expect_output 'crates.io named no parseable retry instant); retrying in 300s'
expect_file sleeps "$case_dir/sleeps" '300'
printf '%s\n' 'unparseable retry instant falls back to 300 s: passed'

# --- any other failure exits at once, with cargo's status, no sleep --------
run_case other_failure 'fail||' ''
expect_status 101
expect_output 'attempt 1 failed (exit 101) for a reason other than a crates.io rate limit; giving up'
expect_file sleeps "$case_dir/sleeps" ''
expect_file 'cargo calls' "$case_dir/calls" 'publish --workspace'
printf '%s\n' 'non-rate-limit failure exits at once: passed'

# --- the attempt bound: eight invocations, seven waits, then give up -------
plan=
for _ in 1 2 3 4 5 6 7 8; do
    plan="$plan${plan:+
}429||$past"
done
run_case attempt_bound "$plan" ''
expect_status 101
expect_output 'attempt 8 failed on a crates.io rate limit; attempt limit 8 reached; giving up'
expect_file sleeps "$case_dir/sleeps" '10
10
10
10
10
10
10'
[ "$(grep -c . "$case_dir/calls")" -eq 8 ] ||
    fail "$case_name: expected 8 cargo publish calls"
printf '%s\n' 'attempt bound honoured: passed'

# --- the wall-clock bound: a wait past 90 minutes is refused, not slept ----
run_case deadline_bound '429||Please try again after Fri, 01 Jan 2100 00:00:00 GMT' ''
expect_status 101
expect_output 'would pass the 90-minute deadline; giving up'
expect_file sleeps "$case_dir/sleeps" ''
printf '%s\n' 'wall-clock deadline honoured: passed'

# --- the deadline is enforced at the START of an attempt, not only pre-sleep -
# Reproduces the exact escape the fix closes: a wait that ends EXACTLY on the
# 90-minute deadline is accepted by the pre-sleep guard (it uses `-gt`), so the
# script sleeps to the deadline; the NEXT attempt must then refuse to launch
# another (unbounded) `cargo publish` because the budget is spent. A mock clock
# makes that measurable in a fast test: the case's own `date` shim reads the
# elapsed time from a file that the case's `sleep` shim advances by the seconds
# it "sleeps", so one recorded sleep moves the clock the full wait. The retry
# instant is placed so wait == DEADLINE_SECONDS to the second.
# NOTE: 5400 and 10 below MUST equal DEADLINE_SECONDS and SETTLE_SECONDS in
# publish_crates.sh — the test asserts the boundary they define.
real_date=$(command -v date)
clock_bin="$workdir/bin_clock"
mkdir -p "$clock_bin"
cat >"$clock_bin/date" <<SHIM
#!/bin/sh
# Mock clock: \`date +%s\` reads FAKE_CLOCK; every other form is the real date
# (so RFC 1123 parsing in publish_crates.sh still works).
if [ "\$1" = "+%s" ]; then cat "\$FAKE_CLOCK"; exit 0; fi
exec "$real_date" "\$@"
SHIM
cat >"$clock_bin/sleep" <<'SHIM'
#!/bin/sh
# Record the sleep AND advance the mock clock by that many seconds.
printf '%s\n' "$1" >> "$FAKE_SLEEPS"
clock=$(cat "$FAKE_CLOCK")
printf '%s\n' "$((clock + $1))" > "$FAKE_CLOCK"
SHIM
chmod 0755 "$clock_bin/date" "$clock_bin/sleep"

clock_base=$(date -u +%s)
printf '%s\n' "$clock_base" >"$workdir/clock"
# wait = (target + SETTLE) - now = DEADLINE, with now == clock_base at attempt 1.
deadline_target=$((clock_base + 5400 - 10))
if date --version >/dev/null 2>&1; then
    deadline_instant=$(LC_ALL=C date -u -d "@$deadline_target" '+%a, %d %b %Y %H:%M:%S GMT')
else
    deadline_instant=$(LC_ALL=C date -u -r "$deadline_target" '+%a, %d %b %Y %H:%M:%S GMT')
fi
EXTRA_SHIM_DIR="$clock_bin" FAKE_CLOCK="$workdir/clock" \
    run_case deadline_start "429||Please try again after $deadline_instant
ok||" ''
expect_status 101
expect_output "attempt 1 failed on a crates.io rate limit (crates.io says try again after $deadline_instant); retrying in 5400s"
expect_output 'the 90-minute publish deadline is spent; not starting attempt 2; giving up'
# The sleep that ends exactly on the deadline is taken, but the second attempt's
# cargo publish is NOT launched: only attempt 1's call is recorded.
expect_file sleeps "$case_dir/sleeps" '5400'
expect_file 'cargo calls' "$case_dir/calls" 'publish --workspace'
printf '%s\n' 'deadline enforced at the start of an attempt: passed'

# --- everything already published is a no-op ------------------------------
run_case nothing_to_publish 'ok||' 'cerulion_macros 0.1.0
cerulion_core 0.1.0
native_ros2_messages 0.1.0
cerulion_bag 0.1.0'
expect_status 0
expect_output 'all 4 publishable crates are already on crates.io at their version; nothing to publish'
expect_file 'cargo calls' "$case_dir/calls" ''
printf '%s\n' 'fully published workspace is a no-op: passed'

# --- an index that cannot answer stops the run before any upload ----------
FAKE_INDEX_HTTP=503 run_case index_outage 'ok||' ''
expect_status 1
expect_output 'crates.io index answered HTTP 503 for cerulion_macros'
expect_file 'cargo calls' "$case_dir/calls" ''
printf '%s\n' 'index outage aborts before publishing: passed'

# --- a curl transport failure aborts before any upload --------------------
# The curl-nonzero-exit arm of is_published() (its "cannot reach the index"
# die) had no case; only the non-200/404 HTTP arm (index_outage) did. A
# regression that swallowed curl's error would build a wrong --exclude list.
FAKE_CURL_FAIL=1 run_case index_unreachable 'ok||' ''
expect_status 1
expect_output 'cannot reach the crates.io index for cerulion_macros'
expect_file 'cargo calls' "$case_dir/calls" ''
printf '%s\n' 'curl transport failure aborts before publishing: passed'

# --- an empty workspace (no publishable member) aborts loudly -------------
FAKE_METADATA="$workdir/metadata_empty.json" run_case empty_metadata 'ok||' ''
expect_status 1
expect_output 'cargo metadata listed no publishable workspace member'
expect_file 'cargo calls' "$case_dir/calls" ''
printf '%s\n' 'empty metadata aborts before publishing: passed'

# --- the 'status 429' signal ALONE is treated as a rate limit -------------
# The 429 body carries 'status 429' but NOT 'too many new crates'. If the
# 'status 429' -e pattern is dropped from is_rate_limited() this 429 falls
# through to the "other failure" arm and exits 101, so exit 0 would fail.
run_case status_429_signal_only "429s||$past
ok||" ''
expect_status 0
expect_output 'attempt 1 failed on a crates.io rate limit'
expect_output 'attempt 2 succeeded'
expect_file sleeps "$case_dir/sleeps" '10'
printf '%s\n' "'status 429' alone is a rate limit: passed"

# --- the 'too many new crates' signal ALONE is treated as a rate limit ----
# The 429 body carries 'too many new crates' but no literal 'status 429', so
# dropping the 'too many new crates' -e pattern makes this exit 101 instead.
run_case too_many_new_crates_signal_only "429t||$past
ok||" ''
expect_status 0
expect_output 'attempt 1 failed on a crates.io rate limit'
expect_output 'attempt 2 succeeded'
expect_file sleeps "$case_dir/sleeps" '10'
printf '%s\n' "'too many new crates' alone is a rate limit: passed"

# --- a non-rate-limit failure AFTER a wait exits with cargo's status ------
# The header's documented residual: attempt 1 is rate-limited and waited
# out, then a LATER attempt fails for another reason (e.g. an accepted
# upload not yet on the index -> cargo's "already exists"). The loop must
# exit non-zero after the wait, not loop.
run_case fail_after_wait "429||$past
fail||" ''
expect_status 101
expect_output "attempt 1 failed on a crates.io rate limit (crates.io says try again after Thu, 01 Jan 2026 00:00:00 GMT); retrying in 10s"
expect_output 'attempt 2 failed (exit 101) for a reason other than a crates.io rate limit; giving up'
expect_file sleeps "$case_dir/sleeps" '10'
[ "$(grep -c . "$case_dir/calls")" -eq 2 ] ||
    fail "$case_name: expected 2 cargo publish calls after one wait"
printf '%s\n' 'non-rate-limit failure after a wait gives up: passed'

printf '%s\n' 'test_publish_crates: all cases passed'
