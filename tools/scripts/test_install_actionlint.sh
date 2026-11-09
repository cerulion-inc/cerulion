#!/usr/bin/env bash
# test_install_actionlint.sh: offline self-test for install_actionlint.sh.
#
#   tools/scripts/test_install_actionlint.sh
#
# `curl` is a PATH shim. `curl --version` reports the version a case sets, and
# a download is answered the way a curl of that version answers it: one older
# than 7.71 rejects `--retry-all-errors` as an unknown option, as a real one
# does, and every download must carry `--retry`, `--connect-timeout` and
# `--max-time`. A download that passes is served a fixture tarball whose
# `actionlint` prints the pinned version.
#
# The installer under test is a copy whose four pinned checksums are replaced
# by the fixture's checksum; the real checksums name the real release, which
# this test never downloads. The copy runs under a PATH of the shim and the
# system directories only, so no actionlint already on the machine is seen.
#
# Exit 0 = every case held. Nonzero = the first failed case, named on stderr.
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
installer=$script_dir/install_actionlint.sh
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-actionlint-test.XXXXXX")
trap 'rm -rf "$work"' EXIT

fail() {
    printf 'test_install_actionlint: FAIL: %s\n' "$1" >&2
    if [ -n "${2:-}" ] && [ -f "$2" ]; then
        printf -- '--- output of the failed case ---\n' >&2
        cat "$2" >&2
    fi
    exit 1
}

if command -v shasum >/dev/null 2>&1; then
    sha256_of() { shasum -a 256 "$1" | cut -d' ' -f1; }
else
    sha256_of() { sha256sum "$1" | cut -d' ' -f1; }
fi

version=$(sed -n 's/^ACTIONLINT_PINNED_VERSION=//p' "$installer")
[ -n "$version" ] || fail "could not read ACTIONLINT_PINNED_VERSION from $installer"

case "$(uname -s)" in
    Linux) os=linux ;;
    Darwin) os=darwin ;;
    *) fail "no fixture platform for $(uname -s)" ;;
esac
case "$(uname -m)" in
    x86_64|amd64)  arch=amd64 ;;
    aarch64|arm64) arch=arm64 ;;
    *) fail "no fixture platform for $(uname -m)" ;;
esac
expected_url=https://github.com/rhysd/actionlint/releases/download/v$version/actionlint_${version}_${os}_${arch}.tar.gz

fixture_dir=$(mktemp -d "$work/tarball.XXXXXX")
printf '#!/bin/sh\nprintf '"'"'%s\\n'"'"'\n' "$version" > "$fixture_dir/actionlint"
chmod +x "$fixture_dir/actionlint"
COPYFILE_DISABLE=1 tar -czf "$work/fixture.tar.gz" -C "$fixture_dir" actionlint
fixture_sha=$(sha256_of "$work/fixture.tar.gz")

# The copy differs from the installer in the four checksum lines only.
sed -e "s/^\(ACTIONLINT_SHA256_[a-z0-9_]*\)=.*/\1=$fixture_sha/" "$installer" > "$work/install_actionlint.sh"
chmod +x "$work/install_actionlint.sh"
changed=$(diff "$installer" "$work/install_actionlint.sh" | grep -c '^>' || true)
[ "$changed" -eq 4 ] || fail "the fixture copy changed $changed lines, not the 4 checksum lines"

shim_bin=$work/bin
mkdir -p "$shim_bin"
cat > "$shim_bin/curl" <<'SHIM'
#!/bin/sh
set -eu
if [ "$*" = --version ]; then
    printf 'curl %s (fixture) libcurl/%s\nProtocols: https\n' "$FIXTURE_CURL_VERSION" "$FIXTURE_CURL_VERSION"
    exit 0
fi
printf '%s\n' "$*" >> "$FIXTURE_LOG"
case " $* " in
    *" --retry-all-errors "*)
        if [ "$FIXTURE_CURL_OLD" = 1 ]; then
            printf 'curl: option --retry-all-errors: is unknown\n' >&2
            exit 2
        fi
        ;;
esac
for flag in --retry --connect-timeout --max-time; do
    case " $* " in
        *" $flag "*) ;;
        *) printf 'fixture curl: missing %s in: %s\n' "$flag" "$*" >&2; exit 98 ;;
    esac
done
url='' out=''
while [ "$#" -gt 0 ]; do
    case "$1" in
        -o) out=$2; shift 2 ;;
        https://*) url=$1; shift ;;
        *) shift ;;
    esac
done
[ "$url" = "$FIXTURE_URL" ] || { printf 'fixture curl: unexpected url %s\n' "$url" >&2; exit 97; }
cp "$FIXTURE_TARBALL" "$out"
SHIM
chmod +x "$shim_bin/curl"
export FIXTURE_URL=$expected_url FIXTURE_TARBALL=$work/fixture.tar.gz

# run_case NAME CURL_VERSION OLD: run the copy with a curl reporting
# CURL_VERSION; OLD=1 makes that curl reject --retry-all-errors. Leaves the exit
# status in $status, the output in $out, the logged download arguments in $log
# and the install directory in $dir.
run_case() {
    local name=$1
    dir=$work/$name/bin
    out=$work/$name/output
    log=$work/$name/args
    mkdir -p "$work/$name"
    : > "$log"
    status=0
    env PATH="$shim_bin:/usr/bin:/bin" HOME="$work/$name" ACTIONLINT_INSTALL_DIR="$dir" \
        FIXTURE_CURL_VERSION="$2" FIXTURE_CURL_OLD="$3" FIXTURE_LOG="$log" \
        "$work/install_actionlint.sh" > "$out" 2>&1 || status=$?
}

# A curl at or past 7.71 gets --retry-all-errors.
for curl_version in 8.7.1 7.71.0 7.88.1 10.0.0; do
    run_case "new-$curl_version" "$curl_version" 0
    [ "$status" -eq 0 ] || fail "curl $curl_version: exit $status" "$out"
    grep -q -- '--retry-all-errors' "$log" || fail "curl $curl_version: --retry-all-errors was not passed" "$out"
    [ "$("$dir/actionlint" -version)" = "$version" ] || fail "curl $curl_version: actionlint was not installed" "$out"
    grep -q 'transient errors are retried' "$out" && fail "curl $curl_version: reported as older than 7.71" "$out"
done
printf 'curl 7.71 and newer retries every error: passed\n'

# An older curl, or one whose version does not parse, gets plain --retry and
# still installs.
for curl_version in 7.70.0 7.68.0 7.7 6.99.0 unknown ""; do
    run_case "old-${curl_version:-empty}" "$curl_version" 1
    [ "$status" -eq 0 ] || fail "curl '$curl_version': exit $status" "$out"
    grep -q -- '--retry-all-errors' "$log" && fail "curl '$curl_version': --retry-all-errors was passed" "$out"
    grep -q -- '--retry 5' "$log" || fail "curl '$curl_version': plain --retry was not passed" "$out"
    grep -q 'does not report 7.71 or newer, so only its transient errors are retried' "$out" \
        || fail "curl '$curl_version': the fallback was not reported" "$out"
    [ "$("$dir/actionlint" -version)" = "$version" ] || fail "curl '$curl_version': actionlint was not installed" "$out"
done
printf 'curl older than 7.71 falls back to plain --retry: passed\n'

# Every attempt is bounded, on both paths.
for case_log in "$work"/new-8.7.1/args "$work"/old-7.68.0/args; do
    for flag in '--connect-timeout 30' '--max-time 180' '--retry-max-time 120'; do
        grep -q -- "$flag" "$case_log" || fail "$flag missing from $(cat "$case_log")"
    done
done
printf 'each download attempt is bounded: passed\n'

printf 'test_install_actionlint: all cases passed\n'
