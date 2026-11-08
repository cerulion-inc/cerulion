#!/usr/bin/env bash
# test_install_nextest.sh: offline self-test for install_nextest.sh.
#
#   tools/scripts/test_install_nextest.sh
#
# `curl` is a PATH shim that serves fixture tarballs by URL. The primary route
# (get.nexte.st) and the fallback route (the GitHub release asset) are each set
# per case to serve the right bytes, wrong bytes, or an HTTP error, and every
# URL the shim is asked for is logged. The shim answers only the two exact URL
# shapes of the pinned version, so a changed route URL fails here too.
#
# The installer under test is a copy whose three pinned checksums are replaced
# by the fixtures' checksums; the real checksums name the real release, which
# this test never downloads. The copy runs under a PATH of the shim and the
# system directories only, so no cargo-nextest already on the machine is seen.
#
# Exit 0 = every case held. Nonzero = the first failed case, named on stderr.
set -euo pipefail

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
installer=$script_dir/install_nextest.sh
work=$(mktemp -d "${TMPDIR:-/tmp}/cerulion-nextest-test.XXXXXX")
trap 'rm -rf "$work"' EXIT

fail() {
    printf 'test_install_nextest: FAIL: %s\n' "$1" >&2
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

version=$(sed -n 's/^NEXTEST_PINNED_VERSION=//p' "$installer")
[ -n "$version" ] || fail "could not read NEXTEST_PINNED_VERSION from $installer"

case "$(uname -s)" in
    Linux)
        case "$(uname -m)" in
            x86_64|amd64)  host_platform=linux ;;
            aarch64|arm64) host_platform=linux-arm ;;
            *) fail "no fixture platform for Linux $(uname -m)" ;;
        esac
        ;;
    Darwin) host_platform=mac ;;
    *) fail "no fixture platform for $(uname -s)" ;;
esac

# --- fixtures --------------------------------------------------------------
# One good tarball per platform, each with a cargo-nextest that names its
# platform, and one wrong tarball whose binary says so.
fixtures=$work/fixtures
mkdir -p "$fixtures"
make_tarball() {
    local name=$1 label=$2 dir
    dir=$(mktemp -d "$work/tarball.XXXXXX")
    printf '#!/bin/sh\nprintf '"'"'cargo-nextest %s (fixture %s)\\n'"'"'\n' "$version" "$label" \
        > "$dir/cargo-nextest"
    chmod +x "$dir/cargo-nextest"
    COPYFILE_DISABLE=1 tar -czf "$fixtures/$name.tar.gz" -C "$dir" cargo-nextest
    rm -rf "$dir"
}
for platform in linux linux-arm mac; do
    make_tarball "good-$platform" "$platform"
done
make_tarball wrong "wrong bytes"

sha_linux=$(sha256_of "$fixtures/good-linux.tar.gz")
sha_linux_arm=$(sha256_of "$fixtures/good-linux-arm.tar.gz")
sha_mac=$(sha256_of "$fixtures/good-mac.tar.gz")

# The copy differs from the installer in the three checksum lines only.
sed -e "s/^NEXTEST_SHA256_linux=.*/NEXTEST_SHA256_linux=$sha_linux/" \
    -e "s/^NEXTEST_SHA256_linux_arm=.*/NEXTEST_SHA256_linux_arm=$sha_linux_arm/" \
    -e "s/^NEXTEST_SHA256_mac=.*/NEXTEST_SHA256_mac=$sha_mac/" \
    "$installer" > "$work/install_nextest.sh"
chmod +x "$work/install_nextest.sh"
changed=$(diff "$installer" "$work/install_nextest.sh" | grep -c '^>' || true)
[ "$changed" -eq 3 ] || fail "the fixture copy changed $changed lines, not the 3 checksum lines"

# --- the curl shim ---------------------------------------------------------
# FIXTURE_PRIMARY and FIXTURE_FALLBACK are `good`, `wrong` or `fail` for every
# platform; FIXTURE_ODD=`route:platform:mode` overrides one of them.
shim_bin=$work/bin
mkdir -p "$shim_bin"
cat > "$shim_bin/curl" <<'SHIM'
#!/bin/sh
set -eu
printf '%s\n' "$*" >> "$FIXTURE_LOG"
url='' out=''
for flag in --retry --connect-timeout --max-time --fail; do
    case " $* " in
        *" $flag "*) ;;
        *) printf 'fixture curl: missing %s in: %s\n' "$flag" "$*" >&2; exit 98 ;;
    esac
done
while [ "$#" -gt 0 ]; do
    case "$1" in
        --url) url=$2; shift 2 ;;
        -o) out=$2; shift 2 ;;
        *) shift ;;
    esac
done
[ -n "$url" ] && [ -n "$out" ] || { printf 'fixture curl: no --url or -o\n' >&2; exit 99; }
release=https://github.com/nextest-rs/nextest/releases/download/cargo-nextest-$FIXTURE_VERSION/cargo-nextest-$FIXTURE_VERSION
route='' platform=''
for candidate in linux:x86_64-unknown-linux-gnu linux-arm:aarch64-unknown-linux-gnu mac:universal-apple-darwin; do
    name=${candidate%%:*}
    target=${candidate#*:}
    case "$url" in
        "https://get.nexte.st/$FIXTURE_VERSION/$name"|"https://get.nexte.st/$FIXTURE_VERSION/$name?"*)
            route=primary platform=$name ;;
        "$release-$target.tar.gz")
            route=fallback platform=$name ;;
    esac
done
[ -n "$route" ] || { printf 'fixture curl: no fixture for %s\n' "$url" >&2; exit 97; }
if [ "$route" = primary ]; then mode=$FIXTURE_PRIMARY; else mode=$FIXTURE_FALLBACK; fi
case "${FIXTURE_ODD:-}" in
    "$route:$platform:"*) mode=${FIXTURE_ODD##*:} ;;
esac
case "$mode" in
    good)  cp "$FIXTURE_DIR/good-$platform.tar.gz" "$out" ;;
    wrong) cp "$FIXTURE_DIR/wrong.tar.gz" "$out" ;;
    fail)  printf 'curl: (22) The requested URL returned error: 503\n' >&2; exit 22 ;;
    *)     printf 'fixture curl: unknown mode %s\n' "$mode" >&2; exit 96 ;;
esac
SHIM
chmod +x "$shim_bin/curl"

export FIXTURE_DIR=$fixtures FIXTURE_VERSION=$version

primary_url=https://get.nexte.st/$version/$host_platform

# run_case NAME PRIMARY FALLBACK [ARGS...]: run the copy in a fresh CARGO_HOME;
# leaves the exit status in $status, stdout in $stdout, stdout and stderr
# together in $out, the URL log in $log and the CARGO_HOME in $home.
run_case() {
    local name=$1 stderr
    export FIXTURE_PRIMARY=$2 FIXTURE_FALLBACK=$3
    shift 3
    home=$work/$name/cargo
    stdout=$work/$name/stdout
    stderr=$work/$name/stderr
    out=$work/$name/output
    log=$work/$name/urls
    mkdir -p "$work/$name"
    : > "$log"
    status=0
    env PATH="$shim_bin:$home/bin:/usr/bin:/bin" CARGO_HOME="$home" HOME="$work/$name" \
        FIXTURE_LOG="$log" "$work/install_nextest.sh" "$@" > "$stdout" 2> "$stderr" || status=$?
    cat "$stdout" "$stderr" > "$out"
    unset FIXTURE_ODD
}

urls_requested() { awk '{for (i = 1; i < NF; i++) if ($i == "--url") print $(i + 1)}' "$log"; }

installed_label() { "$home/bin/cargo-nextest" --version 2>/dev/null | sed -n 's/.*(fixture \(.*\))$/\1/p'; }

# --- install cases ---------------------------------------------------------

# The primary serves the pinned bytes: no annotation, no fallback request.
run_case primary-good good fail
[ "$status" -eq 0 ] || fail "primary good: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "primary good: the installed binary is not the $host_platform fixture" "$out"
grep -q '^::' "$out" && fail "primary good: printed an annotation" "$out"
[ "$(urls_requested)" = "$primary_url" ] || fail "primary good: requested $(urls_requested | tr '\n' ' ')" "$out"
printf 'primary route serves the pinned bytes: passed\n'

# The primary fails to download: a warning, the fallback installs.
run_case primary-fails fail good
[ "$status" -eq 0 ] || fail "primary fails: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "primary fails: the fallback did not install the $host_platform fixture" "$out"
grep -q "^::warning::install_nextest: $primary_url did not deliver" "$out" || fail "primary fails: no warning annotation" "$out"
grep -q '^::error::' "$out" && fail "primary fails: a download failure was annotated as an error" "$out"
[ "$(urls_requested | wc -l | tr -d ' ')" -eq 2 ] || fail "primary fails: expected two requests" "$out"
printf 'failed primary download warns and falls back: passed\n'

# The primary serves wrong bytes: an ERROR annotation, the fallback installs,
# the exit status stays 0.
run_case primary-wrong wrong good
[ "$status" -eq 0 ] || fail "primary wrong bytes: exit $status" "$out"
[ "$(installed_label)" = "$host_platform" ] || fail "primary wrong bytes: the installed binary is not the fallback's" "$out"
grep -q "^::error::install_nextest: $primary_url served bytes that do not match the pinned sha256 of cargo-nextest $version ($host_platform)" "$out" \
    || fail "primary wrong bytes: no error annotation naming the primary URL" "$out"
grep -q '^::warning::' "$out" && fail "primary wrong bytes: annotated as a warning" "$out"
printf 'wrong bytes from the primary are an error annotation and still fall back: passed\n'

# Both routes serve wrong bytes: one error per route, nonzero, nothing installed.
run_case both-wrong wrong wrong
[ "$status" -ne 0 ] || fail "both wrong: exit 0" "$out"
[ "$(grep -c '^::error::' "$out")" -eq 2 ] || fail "both wrong: expected two error annotations" "$out"
[ ! -e "$home/bin/cargo-nextest" ] || fail "both wrong: a binary was installed" "$out"
grep -q 'no route delivered the pinned cargo-nextest' "$out" || fail "both wrong: no final diagnosis" "$out"
printf 'wrong bytes on both routes fail with two errors: passed\n'

# Both routes fail to download: a warning, no error annotation, nonzero.
run_case both-fail fail fail
[ "$status" -ne 0 ] || fail "both fail: exit 0" "$out"
grep -q '^::error::' "$out" && fail "both fail: a download failure was annotated as an error" "$out"
[ "$(grep -c '^::warning::' "$out")" -eq 1 ] || fail "both fail: expected one warning annotation" "$out"
[ ! -e "$home/bin/cargo-nextest" ] || fail "both fail: a binary was installed" "$out"
printf 'no route delivering fails without an error annotation: passed\n'

# The annotation text is escaped: `%` in an overridden primary URL reaches the
# workflow command as `%25`.
export NEXTEST_PRIMARY_URL="$primary_url?probe=100%"
run_case escaped wrong good
unset NEXTEST_PRIMARY_URL
[ "$status" -eq 0 ] || fail "escaped annotation: exit $status" "$out"
grep -q "^::error::install_nextest: $primary_url?probe=100%25 served bytes" "$out" \
    || fail "escaped annotation: the % in the URL was not escaped" "$out"
printf 'annotation text is escaped: passed\n'

# Without shasum or sha256sum nothing is fetched, in either mode; a pinned
# cargo-nextest already on PATH is still accepted, because that check needs no
# checksum tool.
nosha_bin=$work/nosha-bin
mkdir -p "$nosha_bin"
for tool in bash head awk uname mkdir; do
    ln -s "$(command -v "$tool")" "$nosha_bin/$tool"
done
run_without_sha() {
    local name=$1
    shift
    home=$work/$name/cargo
    out=$work/$name/output
    log=$work/$name/urls
    mkdir -p "$work/$name"
    : > "$log"
    status=0
    env PATH="$shim_bin:$home/bin:$nosha_bin" CARGO_HOME="$home" HOME="$work/$name" \
        FIXTURE_LOG="$log" FIXTURE_PRIMARY=good FIXTURE_FALLBACK=good \
        "$work/install_nextest.sh" "$@" > "$out" 2>&1 || status=$?
}
for mode in install pins; do
    if [ "$mode" = pins ]; then
        run_without_sha "nosha-$mode" --print-pins "$version"
    else
        run_without_sha "nosha-$mode"
    fi
    [ "$status" -ne 0 ] || fail "no checksum tool ($mode): exit 0" "$out"
    grep -q 'neither shasum nor sha256sum is available' "$out" || fail "no checksum tool ($mode): not named" "$out"
    [ ! -s "$log" ] || fail "no checksum tool ($mode): fetched before refusing" "$out"
done
mkdir -p "$work/nosha-present/cargo/bin"
cp "$work/primary-good/cargo/bin/cargo-nextest" "$work/nosha-present/cargo/bin/cargo-nextest"
run_without_sha nosha-present
[ "$status" -eq 0 ] || fail "no checksum tool, pinned binary present: exit $status" "$out"
grep -q "cargo-nextest $version already on PATH" "$out" || fail "no checksum tool, pinned binary present: not accepted" "$out"
printf 'no checksum tool refuses before any download: passed\n'

# --- the pin recipe --------------------------------------------------------

expected_pins="NEXTEST_PINNED_VERSION=$version
NEXTEST_SHA256_linux=$sha_linux
NEXTEST_SHA256_linux_arm=$sha_linux_arm
NEXTEST_SHA256_mac=$sha_mac"

# Both routes agree on every platform: the pin block, six requests, nothing
# installed, and every fetch line on stderr so stdout is only the block.
run_case pins-agree good good --print-pins "$version"
[ "$status" -eq 0 ] || fail "pins agree: exit $status" "$out"
[ "$(cat "$stdout")" = "$expected_pins" ] || fail "pins agree: stdout is not the pin block" "$out"
[ "$(urls_requested | wc -l | tr -d ' ')" -eq 6 ] || fail "pins agree: expected six requests" "$out"
[ ! -e "$home/bin" ] || fail "pins agree: the recipe installed something" "$out"
printf 'pin recipe prints the block when both routes agree: passed\n'

# The block it prints is the block the installer reads: pasting it into the
# fixture copy changes nothing.
python3 -I - "$work/install_nextest.sh" "$expected_pins" <<'PY' || fail "pins round trip: the printed block is not the installer's pin block"
import sys
text = open(sys.argv[1]).read()
sys.exit(0 if sys.argv[2] + "\n" in text else 1)
PY
printf 'pin recipe output is the installer pin block: passed\n'

# The fallback serves different bytes for one platform: nonzero, the platform
# named, no partial block on stdout.
export FIXTURE_ODD=fallback:linux-arm:wrong
run_case pins-differ good good --print-pins "$version"
[ "$status" -ne 0 ] || fail "pins differ: exit 0" "$out"
grep -q '^install_nextest: linux-arm: the two routes serve different bytes' "$out" || fail "pins differ: the platform was not named" "$out"
[ ! -s "$stdout" ] || fail "pins differ: printed part of a pin block" "$out"
printf 'pin recipe refuses routes that serve different bytes: passed\n'

# The fallback asset is missing for one platform: nonzero, the URL named.
export FIXTURE_ODD=fallback:mac:fail
run_case pins-missing good good --print-pins "$version"
[ "$status" -ne 0 ] || fail "pins missing: exit 0" "$out"
grep -q '^install_nextest: mac: the GitHub release did not serve https://github.com/nextest-rs/nextest/releases/download/' "$out" \
    || fail "pins missing: the fallback URL was not named" "$out"
[ ! -s "$stdout" ] || fail "pins missing: printed part of a pin block" "$out"
printf 'pin recipe refuses a missing fallback asset: passed\n'

# Usage: a version is required and may not carry a path.
for bad in "" "0.9.137/../x" "--print-pins"; do
    run_case pins-usage good good --print-pins "$bad"
    [ "$status" -ne 0 ] || fail "pins usage: '$bad' was accepted" "$out"
    [ ! -s "$log" ] || fail "pins usage: '$bad' fetched something" "$out"
done
run_case pins-noarg good good --print-pins
[ "$status" -ne 0 ] || fail "pins usage: a missing version was accepted" "$out"
run_case unknown-arg good good --install
[ "$status" -ne 0 ] || fail "usage: an unknown argument was accepted" "$out"
[ ! -s "$log" ] || fail "usage: an unknown argument fetched something" "$out"
printf 'pin recipe and installer refuse bad arguments: passed\n'

printf 'test_install_nextest: all cases passed\n'
